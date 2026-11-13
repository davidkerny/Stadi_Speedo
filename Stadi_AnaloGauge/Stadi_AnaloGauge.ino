#include <Arduino.h>
#include <Wire.h>  // MiniCore Wire s timeoutem (kontrola níže)
#include "U8g2lib.h"
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <util/atomic.h>
#include <util/crc16.h>
#include <stddef.h>
#include "stadionlogo.h"

// Bez timeoutu ve Wire se zaseklé I2C nikdy nevzpamatuje a skončí to resetem.
#ifndef WIRE_TIMEOUT
#error "Wire je bez timeoutu! Odkomentuj #define WIRE_TIMEOUT v Arduino15/packages/MiniCore/hardware/avr/<verze>/libraries/Wire/src/Wire_timeout.h"
#endif

// ============================================================

const char FW_VERZE[] = "AnaloGauge X27 V3.0";

//  Speedo čte kolo na D2 (INT0) = KM/H, ukazuje ručičkou X27 168
//  a na malém OLEDu (128x32): km/h, ODO (EEPROM), TRIP a TOP SPEED.
//
//  V3.0:
//  - Ručička jede z přerušení Timeru1 se zrychlením/brzděním.
//    Kreslení displeje ani zaseklé I2C ji nezastaví.
//  - X27 řízení 6 stavů (oficiální "partial step", 1/3°) místo 4 stavů.
//    Cívky X27 jsou posunuté o 60°, takže 4 stavy dávaly nerovnoměrné kroky.
//  - Rychlost se mezi pulzy plynule odhaduje (trend zrychlení + strop
//    "kolo se ještě neotočilo"), takže ručka při zastavení plynule klesá
//    k nule a nečeká 2 s.
//  - Ochrana proti rušení od zapalování: ověření úrovně pinu v ISR,
//    fronta pulzů, fyzikální kontrola zrychlení. Jiskra mezi pulzy
//    nerozbije měření dalšího pulzu. Vynechané pulzy se dopočítají.
//  - Stav přežívá reset (watchdog, brownout, I2C) ve dvou kopiích s CRC.
//    Rozbitá RAM = čistý studený start, nikdy nesmyslný TRIP.
//  - ODO v EEPROM: každý zápis s kontrolní (invertovanou) kopií.
//  - Periodická obnova nastavení periferií a OLEDu (rušení je umí přepsat).
//
// ============================================================

// 1)  NASTAVENÍ
// 1B) NASTAVENÍ X27 168 - RUČIČKA SPEEDA
// 2)  STAV PROGRAMU
// 3)  ZÁCHRANA STAVU PŘES RESET (.noinit + CRC)
// 4)  MĚŘENÍ NAPÁJENÍ (Vcc) a BOD
// 5)  EEPROM - ukládání ODO
// 6)  PŘERUŠENÍ - SPEEDO
// 7)  ZPRACOVÁNÍ PULZŮ SPEEDA
// 8)  ODHAD RYCHLOSTI MEZI PULZY
// 9)  X27 168 - RUČIČKA SPEEDA (Timer1)
// 10) HEARTBEAT LED, LOGO, I2C FIX
// 10B) TACHO (vypnuté)
// 10C) NULOVÁNÍ STATISTIKY SNÍMAČEM KOLA
// 11) SETUP
// 12) LOOP
// 13) DISPLEJ

// ============================================================
// 1) NASTAVENÍ
// ============================================================

// --- TESTOVACÍ NÁSOBIČ (běžný provoz = 1) ---
#define TEST_MULTIPLIER 1

// --- TEST STUPNICE: zobrazit pevnou hodnotu pro test ---
//#define TEST_RUCICKA_KMH 60

// --- DEBUG: zobrazit čítače závad a Vcc na OLEDu ---
#define DEBUG_MODE

// --- DEBUG: výpis každého pulzu na sériovou linku (TX = D1, 115200 Bd) ---
//#define DEBUG_SERIAL

// --- BUDÍK: VYBER TYP STUPNICE ---
#define STADION_BUDIK
//#define FICHTL_BUDIK

// Obvod kola v milimetrech = vzdálenost na jeden pulz
//const uint16_t OBVOD_KOLA_MM = 1740;  // 16" kolo, guma terénní 16x2.75 FICHTL / SIMSON
const uint16_t OBVOD_KOLA_MM = 1860;    // 19" kolo, guma 19x2.25 STADION
const float OBVOD_KOLA_M = OBVOD_KOLA_MM / 1000.0;

// --- TACHO funkce Zapnout / Vypnout (snímač na D3 / INT1) ---
//#define TACHO

// --- SPEEDO snímač kola (INT0) ---
const byte SENSOR_PIN = 2;

// --- SPEEDO: odolnost proti rušení ---
// Jiskra ze zapalování trvá jednotky us, magnet drží snímač sepnutý
// stovky us až ms. ISR proto chce vidět LOW na pinu po celou dobu ověření.
const uint8_t OVERENI_VZORKU = 8;              // 8 x 5 us = cca 40 us pin musí držet LOW
const unsigned long ZAKMITY_US = 10000UL;       // hrany do 10 ms po přijaté = zákmity kontaktu
const float MAX_ROZUMNA_RYCHLOST_KMH = 130.0;   // nad tím je to vždycky blbost
const float MAX_ZRYCHLENI_MS2 = 2.5;            // víc moped neumí (ani z kopce)
const float MAX_BRZDENI_MS2 = 6.0;              // tvrdé brzdění bubnovkou
const float FYZIKA_TOLERANCE = 1.05;            // rezerva +-5 % (čas pulzu je přesný na us, víc netřeba)
const float FYZIKA_REZERVA_MS = 0.15;           // a ještě pevná rezerva v m/s (hlavně pro malé rychlosti)
const float INTERVAL_POMER_MIN = 0.6;           // další interval proti předchozímu: zrychlování ...
const float INTERVAL_POMER_MAX = 1.35;          // ... a zpomalování při rozjezdu (pravidelnost - rušení je nepravidelné)
const float INTERVAL_POMER_MAX_JIZDA = 1.6;     // za jízdy smí interval narůst víc (brzdění do zastavení)
const uint8_t ROZJEZD_SHOD_KLID = 1;            // klidná linka: 2 sedící intervaly (3 pulzy) = jedeme
const uint8_t ROZJEZD_SHOD_RUSENI = 3;          // rušená linka: 4 sedící intervaly (5 pulzů) = jedeme
const uint8_t HLUCNOST_PRAH = 2;                // od kolika je linka "rušená" (+2 za zahozený pulz, -1 za 2 s)
const float SHODA_POTVRZENI = 0.25;             // po resetu: interval do 25 % od držené rychlosti = jedeme
const float NEJISTOTA_SKORE = 0.4;              // skok rychlosti o víc než 40 % proti čekané = počkat na potvrzení
const float STARSI_PREDCHUDCE_PENALE = 0.25;    // přeskočit pulz v historii (= prohlásit ho za rušení) musí sedět o hodně líp
const float VYNECHANY_PENALE = 0.15;            // vysvětlení přes vynechaný pulz musí sedět o kus líp než 1 otáčka
const float VYNECHANY_OD_MS = 4.0;              // vynechané pulzy hledat až od 14 km/h (pod tím je nejde poznat od brzdění)
const float NOVA_STOPA_POMER = 1.33;            // 3 nesedící pulzy se stejnou roztečí (+-33 %) = nová stopa

// --- SPEEDO: plynulost ---
const unsigned long CAS_DO_ZASTAVENI_MS = 3000UL; // bez pulzu tak dlouho = stojíme (1.86 m / 3 s = 2.2 km/h)
const float NULA_POD_KMH = 3.0;                  // odhad pod touhle rychlostí = stojíme
const float ZOBRAZENI_TAU_S = 0.20;              // časová konstanta, jak rychle ručka dojíždí na odhad

// --- Welcome logo ---
const unsigned long LOGO_DOBA_ZOBRAZENI_MS = 2000UL;

// --- EEPROM / ODO ---
// Nový formát V3: sloty po 8 B (metry + invertovaná kopie) od adresy 256.
// Starý formát (0..252, 4 B bez kontroly) se jen jednou přečte při přechodu na V3.
const int ODO_STARY_ADR_START = 0;
const int ODO_STARY_ADR_KONEC = 252;
const int ODO_ADR_START = 256;
// Statistika resetů podle příčiny: posledních 16 B EEPROM (5 x uint16).
// Přežije i úplné vypnutí, ať je vidět, co se dělo.
const int RESET_STAT_ADR = (int)E2END + 1 - 16;
const int ODO_POCET_SLOTU = (RESET_STAT_ADR - ODO_ADR_START) / 8;
const unsigned long KROK_ULOZENI_M = 100UL;
const unsigned long ODO_FYZICKY_STROP_M = 100000000UL;
const unsigned long ODO_MAX_SKOK_MEZI_BUNKAMI_M = 1000000UL;
#define KONTROLA_NAPETI_ZAPNUTA 1
const long MIN_VCC_PRO_ZAPIS_MV = 4200L;   // napájení z magneta kolísá: ODO zapisovat jen při zdravém napětí (nominál 4.9-5 V)

// --- DISPLEJ ---
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // OLED Init: Placatej zmrd 0.91"
const unsigned long DISPLEJ_INTERVAL_MS = 100;
const unsigned long UDRZBA_INTERVAL_MS = 1000;   // obnova periferií a nastavení OLEDu
const unsigned long I2C_TIMEOUT_US = 10000UL;    // max čekání na jednu I2C operaci (10 ms)

// ============================================================
// 1B) NASTAVENÍ X27 168 - RUČIČKA SPEEDA
// ============================================================

// Cívka 1 = A0 / A1, cívka 2 = D4 / D5 (plyne z původní funkční 4-stavové sekvence)
const byte X27_A = A0;  // cívka 1 +
const byte X27_B = 4;   // cívka 2 +
const byte X27_C = A1;  // cívka 1 -
const byte X27_D = 5;   // cívka 2 -

// Kroků na CELÝ rozsah krokouše. 6 stavů = 1/3° na krok.
// Původní kalibrace 159*4 (4 stavy, 1/2°) = stejný úhel jako 159*6.
const int X27_KROKU_CELKEM = 159*6;

#if defined(STADION_BUDIK)             // STADION
  const float X27_MIN_KMH = 4.0;       //   km/h, kdy ručička opustí doraz
  const float X27_MAX_KMH = 83.0;      //   Konec rozsahu ručičky
  const float LIMIT_STUPNICE = 60.0;   //   MAX rozsah stupnice Stadion
  const uint16_t PRAH_RYCHLOSTI = 30;  //   Od kolika zobrazujem na displeji velké km/h

#elif defined(FICHTL_BUDIK)            // FICHTL
  const float X27_MIN_KMH = 4.0;       //   km/h, kdy ručička opustí doraz
  const float X27_MAX_KMH = 103.0;     //   Konec rozsahu ručičky
  const float LIMIT_STUPNICE = 75.0;   //   MAX rozsah stupnice Fichtl 80
  const uint16_t PRAH_RYCHLOSTI = 30;  //   Od kolika zobrazujem na displeji velké km/h
#else
  #error "Vyber budík: odkomentuj STADION_BUDIK nebo FICHTL_BUDIK"
#endif

// Rozjezd ručičky: doba kroku v us podle rychlosti. Poslední = max rychlost.
// 1333 us na 1/3° = stejná max rychlost jako původních 2000 us na 1/2° (250°/s).
const uint16_t X27_ZRYCHLENI_US[] = { 3000, 2400, 2000, 1750, 1550, 1420, 1333 };
const uint8_t X27_KROKU_NA_STUPEN = 3;   // kolik kroků se jede, než se zrychlí o stupeň tabulky
const uint8_t X27_VEL_MAX = (sizeof(X27_ZRYCHLENI_US) / sizeof(X27_ZRYCHLENI_US[0]) - 1) * X27_KROKU_NA_STUPEN;
const unsigned int X27_HOMING_KROK_US = 1000;  // = původních 1500 us na 1/2° krok (těžká ručka s ferity)
const unsigned int X27_KLID_TIK_US = 1000;     // jak často se v klidu kontroluje nový cíl

const bool X27_OBRACENY_SMER = 1;

const bool KALIBRACE_PO_RESETU = false; // Kalibrovat ručičku jen po zapnutí nebo i po resetu za jízdy?

// ============================================================
// 2) STAV PROGRAMU
// ============================================================

// -- Ujetá vzdálenost --
unsigned long odoMetry = 0;
unsigned long tripMetry = 0;
uint16_t zbytekMm = 0;            // zlomek metru, ať se neztrácí po pulzech
unsigned int topRychlostKmh = 0;

// -- Zobrazená rychlost --
float vZobrazenaKmh = 0;          // plynulá hodnota pro ručku
unsigned int zobrazenaRychlostKmh = 0; // celé km/h pro OLED (s hysterezí)
unsigned long posledniVyhlazeniUs = 0;

// -- Sledování pulzů (viz sekce 7) --
struct PulzZaznam
{ unsigned long t;                // čas pulzu (us)
  uint16_t otacka;                // pořadové číslo otáčky kola (pro ODO)
  float interval;                 // interval na jednu otáčku, kterým se sem došlo (s)
  float v;                        // rychlost tohoto intervalu (m/s)
  bool nejisty;                   // nečekaný skok: použije se, až ho potvrdí další pulz
};
const uint8_t HISTORIE_DELKA = 4;
PulzZaznam historie[HISTORIE_DELKA];  // [0] nejstarší ... [historieN-1] nejnovější
uint8_t historieN = 0;
uint16_t otackaZapocitana = 0;    // poslední otáčka už započtená do ODO
unsigned long nevysvetlene[2];    // pulzy, které nikam nesedí (rušení, nebo ztracená stopa)
uint8_t nevysvetleneN = 0;

unsigned long referenceUs = 0;    // poslední použitý pulz, nebo začátek držení po resetu
bool jedeme = false;              // rychlost potvrzená, ručka ukazuje
float vPosledniMs = 0;            // poslední změřená rychlost (m/s)
float intervalPosledniS = 1;      // poslední interval na jednu otáčku
float zrychleniMs2 = 0;           // odhad trendu (m/s2)
bool drzime = false;              // po resetu za jízdy držíme poslední rychlost, než přijdou pulzy
float drzenaMs = 0;
bool posledniVynechany = false;   // minulý pulz se dopočítal přes vynechaný pulz
uint8_t rozjezdShody = 0;         // kolik intervalů po sobě při rozjezdu sedí
uint8_t hlucnost = 0;             // jak moc je linka rušená (+2 za zahozený pulz, -1 za 2 s)

// -- EEPROM housekeeping --
int odoSlot = -1;                 // poslední zapsaný slot nového formátu
unsigned long odoNaposledyUlozeno = 0;
long posledniZmereneVccMv = 0;    // v mV, 0 = ještě se neměřilo
bool nizkeNapeti = false;         // hnědý poplach: zápis ODO odmítnut kvůli nízkému Vcc
bool bodOk = true;                // pojistky: brownout zapnutý (2.7 nebo 4.3 V)?

// -- Časování --
unsigned long displejNaposledyCas = 0;
unsigned long zachranaNaposledyCas = 0;
unsigned long udrzbaNaposledyCas = 0;

// -- Čítače závad (přežívají reset v záchraně) --
uint16_t resetPocet = 0;          // resety za provozu (RAM přežila)
uint16_t i2cTimeoutPocet = 0;     // I2C timeouty zotavené bez resetu
uint16_t eepromNapetiPocet = 0;   // zápisy ODO odmítnuté kvůli nízkému Vcc
uint16_t zamitnutoPocet = 0;      // pulzy zahozené fyzikální kontrolou (rušení)
uint16_t vynechanoPocet = 0;      // dopočítané vynechané pulzy

// -- Příčina posledního resetu (MCUSR), zachycená hned po startu --
uint8_t resetPricina __attribute__((section(".noinit")));

// ============================================================
// 3) ZÁCHRANA STAVU PŘES RESET (.noinit + CRC)
// ============================================================

// Dvě kopie, zapisuje se střídavě. Když reset přijde uprostřed zápisu,
// rozepsaná kopie neprojde kontrolou a vezme se ta druhá (o 20 ms starší).
// Rozbitá / náhodná RAM neprojde magic + CRC + kontrolou rozumnosti
// a bere se jako studený start.

const uint32_t ZACHRANA_MAGIC = 0x5AD1A030UL;

struct Zachrana
{ uint32_t magic;
  uint32_t seq;
  uint32_t odoMetry;
  uint32_t tripMetry;
  uint16_t zbytekMm;
  uint16_t topKmh;
  float    rychlostKmh;         // zobrazená rychlost - po resetu za jízdy ručka nespadne
  uint16_t resetPocet;
  uint16_t i2cTimeoutPocet;
  uint16_t eepromNapetiPocet;
  uint16_t zamitnutoPocet;
  uint16_t vynechanoPocet;
  uint16_t crc;                 // musí být poslední
};

Zachrana zachrana[2] __attribute__((section(".noinit")));
uint32_t zachranaSeq = 0;

// Poloha ručičky + kontrolní kopie (~poloha). Zapisuje jen ISR Timeru1.
volatile int16_t x27Pozice __attribute__((section(".noinit")));
volatile uint16_t x27PoziceKontrola __attribute__((section(".noinit")));

uint16_t spocitejCrc(const Zachrana *z)
{ const uint8_t *p = (const uint8_t *)z;
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < offsetof(Zachrana, crc); i++)
  { crc = _crc16_update(crc, p[i]); }
  return crc; }

void ulozZachranu()
{ zachranaSeq++;
  Zachrana *z = &zachrana[zachranaSeq & 1];
  z->magic = 0;                     // nejdřív zneplatnit, rozepsaná kopie nikdy neprojde
  z->seq = zachranaSeq;
  z->odoMetry = odoMetry;
  z->tripMetry = tripMetry;
  z->zbytekMm = zbytekMm;
  z->topKmh = topRychlostKmh;
  z->rychlostKmh = vZobrazenaKmh;
  z->resetPocet = resetPocet;
  z->i2cTimeoutPocet = i2cTimeoutPocet;
  z->eepromNapetiPocet = eepromNapetiPocet;
  z->zamitnutoPocet = zamitnutoPocet;
  z->vynechanoPocet = vynechanoPocet;
  z->crc = spocitejCrc(z);
  z->magic = ZACHRANA_MAGIC; }

// Vrátí platnou kopii s vyšším pořadovým číslem, nebo nullptr.
const Zachrana *najdiPlatnouZachranu()
{ const Zachrana *nejlepsi = nullptr;
  for (uint8_t i = 0; i < 2; i++)
  { const Zachrana *z = &zachrana[i];
    if (z->magic != ZACHRANA_MAGIC) continue;
    if (spocitejCrc(z) != z->crc) continue;
    if (nejlepsi == nullptr || (int32_t)(z->seq - nejlepsi->seq) > 0)
    { nejlepsi = z; } }
  return nejlepsi; }

// Ještě kontrola, jestli data dávají smysl (CRC chytí rozbitou RAM,
// tohle chytí blbosti zapsané programem, co jel při podpětí).
bool jeZachranaRozumna(const Zachrana *z, unsigned long odoZEeprom)
{ if (z->odoMetry < odoZEeprom) return false;               // ODO v RAM nemůže být za EEPROM
  if (z->odoMetry > odoZEeprom + 50000UL) return false;     // ani o 50 km napřed
  if (z->tripMetry > z->odoMetry) return false;
  if (z->zbytekMm >= 1000) return false;
  if (z->topKmh > (unsigned int)MAX_ROZUMNA_RYCHLOST_KMH) return false;
  if (!(z->rychlostKmh >= 0.0f && z->rychlostKmh <= MAX_ROZUMNA_RYCHLOST_KMH)) return false; // chytí i NaN
  return true; }

bool jePoziceRucickyPlatna()
{ int16_t p = x27Pozice;
  int16_t kontrola = (int16_t)~x27PoziceKontrola;
  int16_t rozdil = p - kontrola;   // reset mezi zápisem polohy a kontroly = rozdíl 1 krok
  return p >= 0 && p <= X27_KROKU_CELKEM && rozdil >= -1 && rozdil <= 1; }

// MCUSR přečíst a smazat úplně na začátku (před konstruktory), ať po
// resetu watchdogem neběží watchdog s krátkým časem.
void zachytResetPricinu() __attribute__((naked, used, section(".init3")));
void zachytResetPricinu()
{ resetPricina = MCUSR;
  MCUSR = 0;
  wdt_disable(); }

// ============================================================
// 4) MĚŘENÍ NAPÁJENÍ (Vcc) a BOD
// ============================================================

#if KONTROLA_NAPETI_ZAPNUTA
long nactiNapetiVcc()
{  // Na 328PB čteme interní bandgap vůči Vcc.
  ADCSRB &= ~(1 << 5);
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delayMicroseconds(500);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  uint16_t vysledekAdc = ADC;
  if (vysledekAdc == 0) return 0;
  const long VCC_KALIBRACNI_KONSTANTA = 1125300L;
  return VCC_KALIBRACNI_KONSTANTA / vysledekAdc;}
#endif

// Brownout detektor nastavený pojistkami (při "Vypálit zavaděč").
// BOD 4.3 V je na napájení z magneta moc blízko (práh je 4.1-4.5 V)
// a při poklesu by čip zůstal viset v resetu. Proto BOD 2.7 V a zápis
// do EEPROM hlídá program (MIN_VCC_PRO_ZAPIS_MV). Varovat jen když je
// BOD vypnutý nebo na 1.8 V.
bool jeBodNastaveny()
{ uint8_t bod = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS) & 0x07;
  return bod == 0x05 || bod == 0x04; }

// Příčina posledního resetu z MCUSR (zachyceno v .init3).
// P napájení, B podpětí (brownout), W watchdog, X reset pin / programátor,
// S žádný příznak = program skočil na začátek sám (pád programu).
const char RESET_TYPY[] = "PBWXS";
uint16_t resetStat[5];             // počty resetů podle RESET_TYPY (z EEPROM)
bool resetStatCeka = true;         // tenhle reset ještě není zapsaný v EEPROM

char pricinaResetu()
{ if (resetPricina & _BV(WDRF)) return 'W';
  if (resetPricina & _BV(PORF)) return 'P';   // při zapnutí bývá i BORF, napájení má přednost
  if (resetPricina & _BV(BORF)) return 'B';
  if (resetPricina & _BV(EXTRF)) return 'X';
  return 'S'; }

void nactiResetStat()
{ for (uint8_t i = 0; i < 5; i++)
  { EEPROM.get(RESET_STAT_ADR + 2 * i, resetStat[i]);
    if (resetStat[i] == 0xFFFF) resetStat[i] = 0; } }   // prázdná EEPROM

// Zapsat až při zdravém napětí (hned po výpadku může být napájení ještě slabé).
void zapisResetStatPokudJdeTo()
{ if (!resetStatCeka || posledniZmereneVccMv < MIN_VCC_PRO_ZAPIS_MV) return;
  uint8_t i = strchr(RESET_TYPY, pricinaResetu()) - RESET_TYPY;
  if (resetStat[i] < 0xFFFE) resetStat[i]++;
  EEPROM.put(RESET_STAT_ADR + 2 * i, resetStat[i]);
  resetStatCeka = false; }

// ============================================================
// 5) EEPROM - ukládání ODO
// ============================================================

// Starý formát (do V2.2): čte se jen při přechodu na V3.
unsigned long nactiOdoStaryFormat()
{ unsigned long nejvyssiPlatnaHodnota = 0;
  for (int adr = ODO_STARY_ADR_START; adr <= ODO_STARY_ADR_KONEC; adr += 4)
  { unsigned long hodnota = 0;
    EEPROM.get(adr, hodnota);
    if (hodnota == 0xFFFFFFFFUL) continue;
    if (hodnota > ODO_FYZICKY_STROP_M) continue;
    if (nejvyssiPlatnaHodnota > 0 &&
        hodnota > nejvyssiPlatnaHodnota + ODO_MAX_SKOK_MEZI_BUNKAMI_M) continue;
    if (hodnota >= nejvyssiPlatnaHodnota) nejvyssiPlatnaHodnota = hodnota; }
  return nejvyssiPlatnaHodnota; }

struct OdoZaznam
{ uint32_t metry;
  uint32_t kontrola;   // = ~metry. Rozepsaný zápis (výpadek) se pozná a přeskočí.
};

unsigned long nactiOdoZEEPROM()
{ unsigned long nejvyssi = 0;
  int nejvyssiSlot = -1;
  for (int i = 0; i < ODO_POCET_SLOTU; i++)
  { OdoZaznam z;
    EEPROM.get(ODO_ADR_START + i * 8, z);
    if (z.metry != ~z.kontrola) continue;
    if (z.metry > ODO_FYZICKY_STROP_M) continue;
    if (nejvyssiSlot >= 0 && z.metry > nejvyssi + ODO_MAX_SKOK_MEZI_BUNKAMI_M) continue;
    if (nejvyssiSlot < 0 || z.metry >= nejvyssi)
    { nejvyssi = z.metry;
      nejvyssiSlot = i; } }

  if (nejvyssiSlot < 0)
  { // V novém formátu nic není: převzít ze starého. Zapíše se při prvním uložení.
    odoSlot = -1;
    return nactiOdoStaryFormat(); }

  odoSlot = nejvyssiSlot;
  return nejvyssi; }

bool ulozOdoDoEEPROM(unsigned long hodnotaMetru)
{
#if KONTROLA_NAPETI_ZAPNUTA
  posledniZmereneVccMv = nactiNapetiVcc();
  if (posledniZmereneVccMv < MIN_VCC_PRO_ZAPIS_MV)
  { nizkeNapeti = true;                      // hnědý poplach -> tečka na displeji
    if (eepromNapetiPocet < 0xFFFF) eepromNapetiPocet++;
    return false; }
#endif
  odoSlot++;
  if (odoSlot >= ODO_POCET_SLOTU) odoSlot = 0;
  OdoZaznam z;
  z.metry = hodnotaMetru;
  z.kontrola = ~hodnotaMetru;
  EEPROM.put(ODO_ADR_START + odoSlot * 8, z);
  return true;}

void ulozOdoPokudJeTreba()
{ if (odoMetry - odoNaposledyUlozeno >= KROK_ULOZENI_M)
  { if (ulozOdoDoEEPROM(odoMetry))
    { odoNaposledyUlozeno = odoMetry; } } }

// ============================================================
// 6) PŘERUŠENÍ - SPEEDO
// ============================================================

// ISR jen ověří, že to není jiskra, a uloží čas do fronty.
// Všechno rozhodování se dělá v loop() - tam je vidět celá historie.

const uint8_t FRONTA_MASKA = 7;    // fronta na 8 pulzů
volatile unsigned long frontaCasu[FRONTA_MASKA + 1];
volatile uint8_t frontaHlava = 0;
volatile uint8_t frontaOcas = 0;
volatile unsigned long isrPosledniPrijataUs = 0;
volatile bool isrMamePrijatou = false;
volatile uint16_t isrJiskryPocet = 0;   // hrany zahozené ověřením pinu (rušení)

void snimacSpeedoPreruseni()
{ unsigned long ted = micros();

  // Jiskra: pin se hned vrátí do HIGH. Skutečný magnet drží LOW dlouho.
  for (uint8_t i = 0; i < OVERENI_VZORKU; i++)
  { if (digitalRead(SENSOR_PIN) == HIGH)
    { if (isrJiskryPocet < 0xFFFF) isrJiskryPocet++;
      return; }
    delayMicroseconds(5); }

  // Zákmity kontaktu hned po přijatém pulzu.
  if (isrMamePrijatou && (ted - isrPosledniPrijataUs) < ZAKMITY_US)
  { return; }
  isrPosledniPrijataUs = ted;
  isrMamePrijatou = true;

  uint8_t dalsi = (frontaHlava + 1) & FRONTA_MASKA;
  if (dalsi == frontaOcas) return;  // plná fronta (loop stál) - nejstarší zůstanou
  frontaCasu[frontaHlava] = ted;
  frontaHlava = dalsi;
}

bool vezmiPulzZFronty(unsigned long &cas)
{ bool mame = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  { if (frontaOcas != frontaHlava)
    { cas = frontaCasu[frontaOcas];
      frontaOcas = (frontaOcas + 1) & FRONTA_MASKA;
      mame = true; } }
  return mame; }

// ============================================================
// 7) ZPRACOVÁNÍ PULZŮ SPEEDA
// ============================================================

// Historie = posledních pár pulzů, které tvoří řetěz skutečných otáček kola.
// Nový pulz se zkusí navázat na každý z nich (i přes 1-2 vynechané pulzy)
// a vybere se navázání, které je fyzikálně možné a nejlíp sedí na
// očekávanou rychlost. Pulzy mezi vybraným předchůdcem a novým pulzem
// byly rušení: z historie se vyhodí a měření se tím nerozbije.
// Pulz, který nikam nesedí, se ignoruje. Tři takové za sebou se stejnou
// roztečí = ztratili jsme stopu, chytí se znovu na ně.
// Nečekaný skok rychlosti se použije, až ho potvrdí další pulz.
// ODO počítá pořadová čísla otáček, takže zahozená jiskra nic nepřičte.

void pridejOtacky(uint16_t pocet)
{ uint32_t mm = (uint32_t)zbytekMm + (uint32_t)pocet * OBVOD_KOLA_MM;
  unsigned long metry = mm / 1000UL;
  zbytekMm = mm % 1000UL;
  odoMetry += metry;
  tripMetry += metry; }

void zapocitejOtacku(uint16_t otacka)
{ int16_t nove = (int16_t)(otacka - otackaZapocitana);
  if (nove > 0)
  { pridejOtacky(nove);
    otackaZapocitana = otacka; } }

void pricti(uint16_t &citac, uint8_t kolik)
{ citac = (citac > 0xFFFF - kolik) ? 0xFFFF : citac + kolik; }

void zapocitejRuseniBezCitace()
{ hlucnost = (hlucnost > 28) ? 30 : hlucnost + 2; }

void zapocitejRuseni(uint8_t kolik)
{ pricti(zamitnutoPocet, kolik);
  for (uint8_t i = 0; i < kolik; i++) zapocitejRuseniBezCitace(); }

#ifdef DEBUG_SERIAL
void vypisPulz(char verdikt, float interval, float v, uint8_t n)
{ Serial.print(verdikt); Serial.print(' ');
  Serial.print(interval * 1000.0, 1); Serial.print(F("ms "));
  Serial.print(v * 3.6, 1); Serial.print(F("kmh n"));
  Serial.print(n); Serial.print(F(" a"));
  Serial.println(zrychleniMs2, 2); }
#else
#define vypisPulz(verdikt, interval, v, n)
#endif

void vymazSledovani()
{ historieN = 0;
  rozjezdShody = 0;
  nevysvetleneN = 0;
  posledniVynechany = false;
  jedeme = false; }

void pridejDoHistorie(const PulzZaznam &z)
{ if (historieN == HISTORIE_DELKA)
  { for (uint8_t i = 1; i < HISTORIE_DELKA; i++) historie[i - 1] = historie[i];
    historieN--; }
  historie[historieN++] = z; }

// Rychlost je průměr jedné otáčky. Mezi dvěma po sobě jdoucími průměry
// kolo ujede právě jeden obvod, takže v1^2 = v0^2 +- 2*a*obvod (s rezervou).
float rychlostMax(float v0)
{ return sqrtf(v0 * v0 + 2.0f * MAX_ZRYCHLENI_MS2 * OBVOD_KOLA_M) * FYZIKA_TOLERANCE + FYZIKA_REZERVA_MS; }

float rychlostMin(float v0)
{ float v2 = v0 * v0 - 2.0f * MAX_BRZDENI_MS2 * OBVOD_KOLA_M;
  return (v2 > 0) ? sqrtf(v2) / FYZIKA_TOLERANCE - FYZIKA_REZERVA_MS : 0.0f; }

// Použít změřený interval: rychlost, trend, ODO, TOP.
void pouzijPulz(const PulzZaznam &z)
{ float casMeziStredy = 0.5f * (intervalPosledniS + z.interval);  // rychlosti platí ve středech intervalů
  float a = (z.v - vPosledniMs) / casMeziStredy;
  if (a > MAX_ZRYCHLENI_MS2) a = MAX_ZRYCHLENI_MS2;
  if (a < -MAX_BRZDENI_MS2) a = -MAX_BRZDENI_MS2;
  zrychleniMs2 = 0.5f * (zrychleniMs2 + a);
  vPosledniMs = z.v;
  intervalPosledniS = z.interval;
  referenceUs = z.t;
  zapocitejOtacku(z.otacka);
  unsigned int kmh = (unsigned int)(z.v * 3.6f + 0.5f);
  if (kmh > topRychlostKmh) topRychlostKmh = kmh; }

void zacniJizdu(float vPredchozi, const PulzZaznam &z)
{ jedeme = true;
  drzime = false;
  nevysvetleneN = 0;
  posledniVynechany = false;
  vPosledniMs = vPredchozi;
  intervalPosledniS = z.interval;
  zrychleniMs2 = 0;
  pouzijPulz(z); }

// Rozjezd z klidu (nebo po resetu): ručka se pohne, až když dva intervaly
// po sobě k sobě fyzikálně sedí. Rušení na stojící motorce ručkou nehne.
void zpracujRozjezd(unsigned long t)
{ PulzZaznam z;
  z.t = t;
  z.nejisty = false;

  if (historieN == 0)
  { z.otacka = otackaZapocitana + 1;
    z.interval = 0;
    z.v = 0;
    pridejDoHistorie(z);
    vypisPulz('K', 0.0, 0.0, 0);
    return; }

  PulzZaznam predchozi = historie[historieN - 1];
  z.interval = (t - predchozi.t) * 1e-6f;
  z.v = OBVOD_KOLA_M / z.interval;
  if (z.v * 3.6f > MAX_ROZUMNA_RYCHLOST_KMH)
  { zapocitejRuseni(1);                        // jiskra hned po pulzu
    vypisPulz('X', z.interval, z.v, 0);
    return; }
  z.otacka = predchozi.otacka + 1;
  pridejDoHistorie(z);

  // Po resetu za jízdy: první interval sedí na drženou rychlost.
  if (drzime && fabsf(z.v - drzenaMs) <= SHODA_POTVRZENI * drzenaMs)
  { zacniJizdu(drzenaMs, z);
    vypisPulz('H', z.interval, z.v, 1);
    return; }

  // Intervaly po sobě podobné a fyzikálně možné = jedeme.
  // Na rušené lince jich musí sedět víc za sebou.
  bool sedi = false;
  if (predchozi.interval > 0)
  { float pomer = z.interval / predchozi.interval;
    sedi = pomer >= INTERVAL_POMER_MIN && pomer <= INTERVAL_POMER_MAX &&
           z.v <= rychlostMax(predchozi.v) && z.v >= rychlostMin(predchozi.v); }
  if (sedi)
  { rozjezdShody++; }
  else
  { rozjezdShody = 0;
    if (predchozi.interval > 0) zapocitejRuseniBezCitace(); }   // nepravidelné pulzy na stojící motorce = rušení

  uint8_t potreba = (hlucnost >= HLUCNOST_PRAH) ? ROZJEZD_SHOD_RUSENI : ROZJEZD_SHOD_KLID;
  if (rozjezdShody >= potreba)
  { zacniJizdu(predchozi.v, z);
    vypisPulz('A', z.interval, z.v, 1);
    return; }

  vypisPulz('C', z.interval, z.v, 0);
}

// Pulz, který nikam nesedí. Tři za sebou se stejnou roztečí = stopa ztracená
// (třeba po resetu), chytit se znovu.
void zpracujNevysvetleny(unsigned long t)
{ zapocitejRuseni(1);
  vypisPulz('G', 0.0, 0.0, 0);

  if (nevysvetleneN == 2)
  { float i1 = (nevysvetlene[1] - nevysvetlene[0]) * 1e-6f;
    float i2 = (t - nevysvetlene[1]) * 1e-6f;
    float pomer = i2 / i1;
    if (pomer > 1.0f / NOVA_STOPA_POMER && pomer < NOVA_STOPA_POMER &&
        OBVOD_KOLA_M / i2 * 3.6f <= MAX_ROZUMNA_RYCHLOST_KMH)
    { PulzZaznam z;
      z.nejisty = false;
      historieN = 0;
      z.t = nevysvetlene[0]; z.otacka = otackaZapocitana; z.interval = 0;  z.v = 0;
      pridejDoHistorie(z);
      z.t = nevysvetlene[1]; z.otacka++;  z.interval = i1; z.v = OBVOD_KOLA_M / i1;
      pridejDoHistorie(z);
      float v1 = z.v;
      z.t = t;               z.otacka++;  z.interval = i2; z.v = OBVOD_KOLA_M / i2;
      pridejDoHistorie(z);
      zacniJizdu(v1, z);
      vypisPulz('R', i2, z.v, 1);
      return; }
    nevysvetlene[0] = nevysvetlene[1];
    nevysvetleneN = 1; }

  nevysvetlene[nevysvetleneN++] = t;
}

void zpracujPulz(unsigned long t)
{
  // Dlouho nic: stáli jsme, začíná nový rozjezd.
  if (historieN > 0 && (t - historie[historieN - 1].t) > CAS_DO_ZASTAVENI_MS * 1000UL)
  { vymazSledovani(); }

  if (!jedeme)
  { zpracujRozjezd(t);
    return; }

  // Čekaná rychlost: trend jen napůl, ať změna trendu nevykolejí výběr.
  float trend = 0.5f * zrychleniMs2 * intervalPosledniS;

  int8_t nejI = -1;
  uint8_t nejN = 1;
  float nejSkore = 1e9f, nejV = 0, nejInterval = 0;

  for (int8_t i = historieN - 1; i >= 0; i--)
  { float dt = (t - historie[i].t) * 1e-6f;
    if (dt > CAS_DO_ZASTAVENI_MS * 1e-3f) break;     // starší předchůdci jsou ještě starší
    // Vynechaný pulz dvakrát po sobě = spíš falešná stopa na dvojnásobku,
    // tak to nepovolit (stopa se pak chytí znovu na skutečných pulzech).
    // Meze podle rychlosti, se kterou se do předchůdce přišlo
    // (u nejistého předchůdce tak potvrzení musí sedět na jeho rychlost).
    float v0 = (historie[i].v > 0) ? historie[i].v : vPosledniMs;
    uint8_t nMax = (posledniVynechany || v0 < VYNECHANY_OD_MS) ? 1 : 3;
    float vMax = rychlostMax(v0);
    float vMin = rychlostMin(v0);
    float vPred = v0 + trend;
    if (vPred < 0.5f) vPred = 0.5f;
    for (uint8_t n = 1; n <= nMax; n++)
    { float interval = dt / n;
      float v = OBVOD_KOLA_M / interval;
      if (v > vMax) break;            // moc brzo (víc otáček = ještě rychlejší)
      if (v < vMin) continue;         // moc pozdě: nevynechal se pulz?
      if (historie[i].interval > 0)   // pravidelnost proti intervalu předchůdce
      { float pomerIntervalu = interval / historie[i].interval;
        if (pomerIntervalu < INTERVAL_POMER_MIN) break;
        if (pomerIntervalu > INTERVAL_POMER_MAX_JIZDA) continue; }
      float pomer = v / vPred;
      float skore = (pomer > 1.0f ? pomer - 1.0f : 1.0f / pomer - 1.0f)
                    + STARSI_PREDCHUDCE_PENALE * (historieN - 1 - i)   // starší předchůdce jen když novější fakt nesedí
                    + VYNECHANY_PENALE * (n - 1);
      if (skore < nejSkore)
      { nejSkore = skore;
        nejI = i;
        nejN = n;
        nejV = v;
        nejInterval = interval; } } }

  // Přeskočit pulz v historii (= prohlásit ho za rušení) jen když to sedí jistě.
  if (nejI < 0 || (nejI < historieN - 1 && nejSkore > NEJISTOTA_SKORE))
  { zpracujNevysvetleny(t);
    return; }

  nevysvetleneN = 0;
  uint8_t zahozeno = historieN - 1 - nejI;   // pulzy za předchůdcem = rušení
  if (zahozeno) zapocitejRuseni(zahozeno);
  historieN = nejI + 1;

  if (historie[nejI].nejisty)                // řetěz přes něj vede = byl skutečný
  { historie[nejI].nejisty = false;
    pouzijPulz(historie[nejI]); }

  if (nejN > 1) pricti(vynechanoPocet, nejN - 1);
  posledniVynechany = (nejN > 1);

  PulzZaznam z;
  z.t = t;
  z.otacka = historie[nejI].otacka + nejN;
  z.interval = nejInterval;
  z.v = nejV;
  z.nejisty = nejSkore > NEJISTOTA_SKORE;
  pridejDoHistorie(z);
  if (!z.nejisty) pouzijPulz(z);

  vypisPulz(z.nejisty ? '?' : (nejN > 1 ? 'M' : 'P'), nejInterval, nejV, nejN);
}

// ============================================================
// 8) ODHAD RYCHLOSTI MEZI PULZY
// ============================================================

// Mezi pulzy se rychlost dopočítává podle trendu. A hlavně:
// když od posledního pulzu uběhl čas e, kolo se ještě neotočilo,
// takže rychlost je nanejvýš obvod / e. Při zastavení tak ručka
// plynule klesá hned, jak má pulz zpoždění, a nečeká 2 s.

float odhadRychlostiMs(unsigned long ted)
{ if (!jedeme && !drzime) return 0;

  float e = (ted - referenceUs) * 1e-6f;
  if (e > CAS_DO_ZASTAVENI_MS * 1e-3f)
  { vymazSledovani();
    drzime = false;
    return 0; }

  float v;
  if (jedeme)
  { float h = (e < intervalPosledniS) ? e : intervalPosledniS;  // trend nejvýš o jednu otáčku dopředu
    v = vPosledniMs + zrychleniMs2 * (0.5f * intervalPosledniS + h);
    float strop = vPosledniMs * 1.5f + 1.0f;
    if (v > strop) v = strop; }
  else
  { v = drzenaMs; }

  if (e > 0.001f)
  { float mez = OBVOD_KOLA_M / e;
    if (v > mez) v = mez; }

  if (v < 0) v = 0;
  return v; }

void aktualizujZobrazenouRychlost(unsigned long ted)
{ float dt = (ted - posledniVyhlazeniUs) * 1e-6f;
  posledniVyhlazeniUs = ted;
  if (dt > 0.5f) dt = 0.5f;

  float cil = odhadRychlostiMs(ted) * 3.6f;
  if (cil < NULA_POD_KMH) cil = 0;

  vZobrazenaKmh += (cil - vZobrazenaKmh) * (dt / (ZOBRAZENI_TAU_S + dt));
  if (cil == 0 && vZobrazenaKmh < 1.0f) vZobrazenaKmh = 0;

  // Celé km/h pro OLED s hysterezí, ať číslo neblikne mezi dvěma hodnotami.
  if (vZobrazenaKmh < 1.0f)
  { zobrazenaRychlostKmh = 0; }
  else if (fabsf(vZobrazenaKmh - (float)zobrazenaRychlostKmh) > 0.7f)
  { zobrazenaRychlostKmh = (unsigned int)(vZobrazenaKmh + 0.5f); }
}

// ============================================================
// 9) X27 168 - RUČIČKA SPEEDA (Timer1)
// ============================================================

// 6 stavů na elektrickou otáčku (2°), 1/3° na krok. Bity: A B C D
// Cívky X27 svírají 60°, proto tahle sekvence dává rovnoměrné kroky.
const uint8_t X27_SEKVENCE[6] = {
  0b1000,  //   0°  cívka1 +
  0b0100,  //  60°  cívka2 +
  0b0110,  // 120°  cívka1 -, cívka2 +
  0b0010,  // 180°  cívka1 -
  0b0001,  // 240°  cívka2 -
  0b1001,  // 300°  cívka1 +, cívka2 -
};

volatile int16_t x27Cil = 0;   // kam má ručka dojet (zapisuje loop)
int16_t x27CilLoop = 0;        // kopie pro loop, ať se cíl neposílá zbytečně
uint8_t x27Vel = 0;            // stupeň rozjezdu (jen ISR)
int8_t x27Smer = 0;            // směr pohybu (jen ISR)

void nastavCivkyX27(int16_t krok)
{ int8_t stav = krok % 6;
  if (stav < 0) stav += 6;
  if (X27_OBRACENY_SMER) stav = 5 - stav;
  uint8_t b = X27_SEKVENCE[stav];
  digitalWrite(X27_A, (b >> 3) & 1);
  digitalWrite(X27_B, (b >> 2) & 1);
  digitalWrite(X27_C, (b >> 1) & 1);
  digitalWrite(X27_D, b & 1); }

inline void ulozPoziciX27(int16_t p)
{ x27Pozice = p;
  x27PoziceKontrola = ~(uint16_t)p; }

// Homing ručičky: zajede plný rozsah a zpátky na nulu (na doraz).
void homingRucicky()
{ int16_t p = 0;
  for (int i = 0; i < X27_KROKU_CELKEM; i++)
  { p++;
    nastavCivkyX27(p);
    delayMicroseconds(X27_HOMING_KROK_US); }

  delay(100);

  for (int i = 0; i < X27_KROKU_CELKEM; i++)
  { p--;
    nastavCivkyX27(p);
    delayMicroseconds(X27_HOMING_KROK_US); }

  ulozPoziciX27(0); }

// Timer1 CTC, 0.5 us na tik. OCR1A = doba do dalšího kroku.
void nastavTimerRucicky()
{ TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11);       // CTC, prescaler 8
  TIMSK1 |= _BV(OCIE1A); }

ISR(TIMER1_COMPA_vect)
{ int16_t pozice = x27Pozice;
  int16_t cil = x27Cil;

  if (x27Vel == 0)
  { if (pozice == cil)
    { nastavCivkyX27(pozice);            // v klidu znovu pustit proud (kdyby rušení přepsalo piny)
      OCR1A = X27_KLID_TIK_US * 2 - 1;
      return; }
    x27Smer = (cil > pozice) ? 1 : -1; }

  int16_t nova = pozice + x27Smer;
  if (nova < 0 || nova > X27_KROKU_CELKEM)
  { x27Vel = 0;
    OCR1A = X27_KLID_TIK_US * 2 - 1;
    return; }

  ulozPoziciX27(nova);
  nastavCivkyX27(nova);

  // Zbývá kladně = cíl je ještě před námi. Brzdit, když je blíž než
  // brzdná dráha, nebo když se cíl otočil za nás (zastavit, pak couvnout).
  int16_t zbyva = (cil - nova) * x27Smer;
  if (zbyva <= (int16_t)x27Vel)
  { if (x27Vel > 0) x27Vel--; }
  else if (x27Vel < X27_VEL_MAX)
  { x27Vel++; }

  OCR1A = X27_ZRYCHLENI_US[x27Vel / X27_KROKU_NA_STUPEN] * 2 - 1;
}

int16_t kmhNaKroky(float hodnotaKmh)
{ if (hodnotaKmh > LIMIT_STUPNICE) hodnotaKmh = LIMIT_STUPNICE;  // konec stupnice na ciferníku
  if (hodnotaKmh > X27_MAX_KMH) hodnotaKmh = X27_MAX_KMH;
  if (hodnotaKmh <= X27_MIN_KMH) return 0;                       // pod minimem = na dorazu
  float podil = (hodnotaKmh - X27_MIN_KMH) / (X27_MAX_KMH - X27_MIN_KMH);
  return (int16_t)(podil * X27_KROKU_CELKEM + 0.5f); }

// Plynulý cíl pro ručku. Změny o 1 krok (1/3°) se ignorují, ať se ručka v klidu neklepe.
void nastavRucickuKmH(float hodnotaKmh)
{ int16_t cil = kmhNaKroky(hodnotaKmh);
  int16_t rozdil = cil - x27CilLoop;
  if (cil != 0 && rozdil >= -1 && rozdil <= 1) return;
  if (cil == x27CilLoop) return;
  x27CilLoop = cil;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { x27Cil = cil; } }

// ============================================================
// 10) HEARTBEAT LED, LOGO, I2C FIX
// ============================================================

// Timer2 v CTC módu, prescaler 1024 -> jeden "tik" cca 16 ms.
// V ISR čekáme 16 tiků (~256 ms), než LED přepneme (~2 Hz).
volatile uint8_t heartbeatPocitadloPreruseni = 0;

void nastavHeartbeatTimer()
{ TCCR2A = (1 << WGM21);                              // CTC mód
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);   // prescaler 1024
  OCR2A = 249;                                         // perioda ~16 ms
  TIMSK2 = (1 << OCIE2A); }

ISR(TIMER2_COMPA_vect)
{ heartbeatPocitadloPreruseni++;
  if (heartbeatPocitadloPreruseni >= 16)
  { heartbeatPocitadloPreruseni = 0;
    digitalWrite(13, !digitalRead(13)); } }

void zobrazUvodniLogo()
{ int x = (128 - STADION_WIDTH) / 2 -4;  //posunuty o -4px, aby to vycházelo do výřezu
  u8g2.clearBuffer();
  u8g2.drawXBMP(x, 0, STADION_WIDTH, STADION_HEIGHT, logo_stadion);
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setCursor(15, 28);
  u8g2.print(FW_VERZE);
#ifdef DEBUG_MODE
  u8g2.print(F(" D"));   // debug build
  u8g2.setCursor(0, 28);
  u8g2.print(pricinaResetu());  // příčina resetu (P/B/W/X/S)
#endif
  u8g2.sendBuffer(); }

// Pokud displej drží SDA dole (zaseklý uprostřed bajtu), vyklepe se
// ručně 9 hodinovými pulzy a STOP podmínkou. Funguje jen když TWI
// hardware nedrží piny (před Wire.begin() nebo po Wire.end()).
void i2cBusClearPokudJeZaseknuto()
{ pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(10);

  bool byloZaseknute = (digitalRead(SDA) == LOW);
  if (byloZaseknute) {
    for (int i = 0; i < 9; i++) {
      if (digitalRead(SDA) == HIGH) break; // Displej už pustil SDA
      pinMode(SCL, OUTPUT);
      digitalWrite(SCL, LOW);
      delayMicroseconds(5);
      pinMode(SCL, INPUT_PULLUP);
      delayMicroseconds(5);
    }
    // STOP podmínka
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    pinMode(SDA, OUTPUT);
    digitalWrite(SDA, LOW);
    delayMicroseconds(5);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(5);
    pinMode(SDA, INPUT_PULLUP);
    delayMicroseconds(5);
  }
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP); }

// Start I2C od nuly: uvolnit piny, vyklepat sběrnici, nahodit Wire s timeoutem a displej.
void i2cStart()
{ Wire.end();
  i2cBusClearPokudJeZaseknuto();
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);   // při timeoutu resetni TWI hardware
  u8g2.begin();                                // Wire.begin() + init displeje
  Wire.clearWireTimeoutFlag(); }

void zkontrolujI2C()
{ if (Wire.getWireTimeoutFlag())
  { if (i2cTimeoutPocet < 0xFFFF) i2cTimeoutPocet++;
    i2cStart(); } }

// Nastavení OLEDu jako při initu, ale BEZ vypnutí displeje (0xAE),
// takže neblikne. Když rušení displej resetne nebo přepíše registry,
// do vteřiny je zase v pořádku. Hodnoty = U8g2 init pro 128x32 univision.
const uint8_t OLED_OBNOVA_SEQ[] = {
  U8X8_START_TRANSFER(),
  U8X8_CA(0x0d5, 0x080),   // clock divide / oscilátor
  U8X8_CA(0x0a8, 0x01f),   // multiplex
  U8X8_CA(0x0d3, 0x000),   // offset
  U8X8_C(0x040),           // start line
  U8X8_CA(0x08d, 0x014),   // charge pump zapnout
  U8X8_CA(0x020, 0x000),   // horizontální adresování
  U8X8_C(0x0a1),           // segment remap
  U8X8_C(0x0c8),           // scan směr
  U8X8_CA(0x0da, 0x002),   // COM piny
  U8X8_CA(0x081, 0x08f),   // kontrast
  U8X8_CA(0x0d9, 0x0f1),   // pre-charge
  U8X8_CA(0x0db, 0x040),   // vcomh
  U8X8_C(0x02e),           // scroll vypnout
  U8X8_C(0x0a4),           // zobrazit RAM
  U8X8_C(0x0a6),           // neinvertovaně
  U8X8_C(0x0af),           // displej zapnout
  U8X8_END_TRANSFER(),
  U8X8_END()
};

void obnovNastaveniDispleje()
{ u8x8_cad_SendSequence(u8g2.getU8x8(), OLED_OBNOVA_SEQ);
  zkontrolujI2C(); }

// Rušení / brownout může přepsat registry periferií. Jednou za vteřinu
// se znovu nastaví všechno, na čem závisí ručka a snímač.
void obnovHardware()
{ pinMode(X27_A, OUTPUT);
  pinMode(X27_B, OUTPUT);
  pinMode(X27_C, OUTPUT);
  pinMode(X27_D, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  { EICRA = (EICRA & ~(_BV(ISC01) | _BV(ISC00))) | _BV(ISC01);   // INT0 na sestupnou hranu
    EIMSK |= _BV(INT0);
    nastavTimerRucicky();
    nastavHeartbeatTimer(); } }

// ============================================================
// 10B) TACHO (vypnuté, zapnout #define TACHO v sekci 1)
// ============================================================

#ifdef TACHO
const byte TACHO_PIN = 3;
const unsigned long MIN_MEZERA_TACHO_US = 1333UL;
const float PULZY_NA_OTACKU = 3.0;  // Tacho pulzy per otáčka DUCATI ENERGIA
const float MAX_ROZUMNE_RPM = 15000.0;
const float MIN_ROZUMNE_RPM = 500.0;
const float MAX_SKOK_RPM = 5000.0;
const float MAX_ROZDIL_SOUSEDNICH_PULZU_RPM = 1500.0;
const unsigned long CAS_DO_ZASTAVENI_TACHO_MS = 200UL;
const float TACHO_KALMAN_Q = 2.0;
const float TACHO_KALMAN_R = 8.0;

unsigned int zobrazeneRpm = 0;
float tachoKalman_x = 0;
float tachoKalman_p = 1;
float posledniSuroveRpm = 0.0;
bool tachoMamePredchoziPulz = false;
bool motorBezi = false;
volatile unsigned long tachoIsrCasPoslednihoPulzuUs = 0;
volatile unsigned long tachoIsrDelkaPoslednihoIntervaluUs = 0;
volatile bool tachoIsrMameNovyPulz = false;

void snimacTachoPreruseni()
{ unsigned long ted = micros();
  if (tachoIsrCasPoslednihoPulzuUs == 0)
  { tachoIsrCasPoslednihoPulzuUs = ted; return;}
  unsigned long interval = ted - tachoIsrCasPoslednihoPulzuUs;
  if (interval < MIN_MEZERA_TACHO_US) return;
  tachoIsrDelkaPoslednihoIntervaluUs = interval;
  tachoIsrCasPoslednihoPulzuUs = ted;
  tachoIsrMameNovyPulz = true; }

float vyhladRpm(float noveMereniRpm)
{ tachoKalman_p = tachoKalman_p + TACHO_KALMAN_Q;
  float zisk = tachoKalman_p / (tachoKalman_p + TACHO_KALMAN_R);
  tachoKalman_x = tachoKalman_x + zisk * (noveMereniRpm - tachoKalman_x);
  tachoKalman_p = (1.0 - zisk) * tachoKalman_p;
  return tachoKalman_x; }

bool jePulzTachaDuveryhodny(float suroveRpm)
{ if (suroveRpm > MAX_ROZUMNE_RPM || suroveRpm < MIN_ROZUMNE_RPM) return false;
  if (motorBezi)
  { float rozdil = fabs(suroveRpm - tachoKalman_x);
    if (rozdil > MAX_SKOK_RPM)
    { bool souhlasiSPredchozim = tachoMamePredchoziPulz &&
        fabs(suroveRpm - posledniSuroveRpm) < MAX_ROZDIL_SOUSEDNICH_PULZU_RPM;
      if (souhlasiSPredchozim)
      { tachoKalman_x = suroveRpm;
        tachoKalman_p = 1.0; }
      else return false; } }
  return true; }

void zpracujNovyTachoPulzPokudExistuje()
{ if (!tachoIsrMameNovyPulz) return;
  unsigned long intervalUs;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  { intervalUs = tachoIsrDelkaPoslednihoIntervaluUs;
    tachoIsrMameNovyPulz = false; }
  float suroveRpm = (60000000.0 / intervalUs) / PULZY_NA_OTACKU;
  if (!jePulzTachaDuveryhodny(suroveRpm))
  { tachoMamePredchoziPulz = false;
    return; }
  tachoMamePredchoziPulz = true;
  posledniSuroveRpm = suroveRpm;
  motorBezi = true;
  zobrazeneRpm = (unsigned int)(vyhladRpm(suroveRpm) + 0.5); }

void zkontrolujJestliStojimeTacho()
{ unsigned long casPoslednihoPulzu;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { casPoslednihoPulzu = tachoIsrCasPoslednihoPulzuUs; }
  if (casPoslednihoPulzu != 0 &&
      micros() - casPoslednihoPulzu > CAS_DO_ZASTAVENI_TACHO_MS * 1000UL)
  { tachoKalman_x = 0;
    tachoKalman_p = 1;
    zobrazeneRpm = 0;
    tachoMamePredchoziPulz = false;
    motorBezi = false; } }
#endif

// ============================================================
// 10C) NULOVÁNÍ STATISTIKY SNÍMAČEM KOLA
// ============================================================

// Po zapnutí držet snímač kola sepnutý 3 s, pak ho do 5 s pustit.
// Pustit = pojistka: motorka zaparkovaná s magnetem u snímače nic nesmaže.
// Nuluje statistiku resetů v EEPROM a čítače R I E J G M. ODO a TRIP ne.
const unsigned long NULOVANI_DRZET_MS = 3000UL;
const unsigned long NULOVANI_PUSTIT_MS = 5000UL;

// Texty hlášek jsou ve flash (F("...")), RAM je málo.
void zobrazHlasku(const __FlashStringHelper *radek1, const __FlashStringHelper *radek2)
{ char buf[24];
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  strncpy_P(buf, (const char *)radek1, sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
  u8g2.setCursor((128 - u8g2.getStrWidth(buf)) / 2, 13);
  u8g2.print(buf);
  strncpy_P(buf, (const char *)radek2, sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
  u8g2.setCursor((128 - u8g2.getStrWidth(buf)) / 2, 28);
  u8g2.print(buf);
  u8g2.sendBuffer();
  zkontrolujI2C(); }

void pockej(unsigned long ms)
{ unsigned long start = millis();
  while (millis() - start < ms) { wdt_reset(); } }

bool snimacSepnuty()
{ return digitalRead(SENSOR_PIN) == LOW; }

void nulovaniStatistikyPokudChce()
{ if (!snimacSepnuty()) return;

  zobrazHlasku(F("Nulovat statistiku?"), F("drz snimac 3 s"));
  unsigned long start = millis();
  while (millis() - start < NULOVANI_DRZET_MS)
  { wdt_reset();
    if (!snimacSepnuty())
    { pockej(30);                        // zákmit? pustil doopravdy?
      if (!snimacSepnuty()) return; } }  // pustil moc brzo = normální start

  zobrazHlasku(F("Nulovat statistiku?"), F("pust snimac"));
  start = millis();
  while (millis() - start < NULOVANI_PUSTIT_MS)
  { wdt_reset();
    if (!snimacSepnuty())
    { pockej(50);
      if (snimacSepnuty()) continue;     // zákmit

#if KONTROLA_NAPETI_ZAPNUTA
      posledniZmereneVccMv = nactiNapetiVcc();
      if (posledniZmereneVccMv < MIN_VCC_PRO_ZAPIS_MV)
      { zobrazHlasku(F("Nizke napeti"), F("nevynulovano"));
        pockej(2000);
        return; }
#endif
      for (uint8_t i = 0; i < 5; i++)
      { resetStat[i] = 0;
        EEPROM.put(RESET_STAT_ADR + 2 * i, resetStat[i]); }
      resetStatCeka = false;             // tenhle start se už nepočítá
      resetPocet = 0;
      i2cTimeoutPocet = 0;
      eepromNapetiPocet = 0;
      zamitnutoPocet = 0;
      vynechanoPocet = 0;
      isrJiskryPocet = 0;
      ulozZachranu();
      zobrazHlasku(F("Statistika"), F("vynulovana"));
      pockej(1500);
      return; } }

  zobrazHlasku(F("Nulovani"), F("zruseno"));     // nepustil = zaparkováno s magnetem u snímače
  pockej(1500);
}

// ============================================================
// 11) SETUP
// ============================================================

void setup()
{
  // Benevolentní watchdog pro setup (8 s je strop AVR). MCUSR už je
  // přečtený a smazaný v zachytResetPricinu().
  wdt_enable(WDTO_8S);

  // Ručička: piny hned na začátku, ať při resetu za jízdy co nejdřív drží.
  pinMode(X27_A, OUTPUT);
  pinMode(X27_B, OUTPUT);
  pinMode(X27_C, OUTPUT);
  pinMode(X27_D, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(13, OUTPUT);

#ifdef DEBUG_SERIAL
  Serial.begin(115200);
#endif

  // ODO z EEPROM a záchrana z RAM: teplý start jen když RAM prošla
  // magic + CRC + kontrolou rozumnosti. Jinak čistý studený start.
  unsigned long odoZEeprom = nactiOdoZEEPROM();
  nactiResetStat();
  const Zachrana *z = najdiPlatnouZachranu();
  bool teplyStart = (z != nullptr) && jeZachranaRozumna(z, odoZEeprom);

  if (teplyStart)
  { zachranaSeq = z->seq;
    odoMetry = z->odoMetry;
    tripMetry = z->tripMetry;
    zbytekMm = z->zbytekMm;
    topRychlostKmh = z->topKmh;
    resetPocet = z->resetPocet;
    i2cTimeoutPocet = z->i2cTimeoutPocet;
    eepromNapetiPocet = z->eepromNapetiPocet;
    zamitnutoPocet = z->zamitnutoPocet;
    vynechanoPocet = z->vynechanoPocet;
    if (resetPocet < 0xFFFF) resetPocet++;
    if (z->rychlostKmh >= NULA_POD_KMH)
    { drzime = true;                       // držet rychlost, než přijdou pulzy
      drzenaMs = z->rychlostKmh / 3.6f;
      vZobrazenaKmh = z->rychlostKmh;
      zobrazenaRychlostKmh = (unsigned int)(z->rychlostKmh + 0.5f); } }
  else
  { zachranaSeq = 0;
    odoMetry = odoZEeprom;
    tripMetry = 0;
    zbytekMm = 0;
    topRychlostKmh = 0; }
  odoNaposledyUlozeno = odoZEeprom;

  // Poloha ručičky: věřit jen při teplém startu a platné kontrole.
  bool poziceOk = teplyStart && !KALIBRACE_PO_RESETU && jePoziceRucickyPlatna();
  if (poziceOk)
  { nastavCivkyX27(x27Pozice); }          // jen znovu pustit proud, ručka se nehne

  ulozZachranu();                          // hned zapsat platný stav (druhou kopii přepíše loop za 20 ms)

  // I2C: bus clear + Wire s timeoutem + displej
  i2cStart();

  // Po zapnutí se sepnutým snímačem kola: nulování statistiky
  nulovaniStatistikyPokudChce();

  if (!teplyStart)
  { zobrazUvodniLogo(); }
  else
  { vykresliDashboard(); }               // reset za jízdy: displej hned, žádné logo
  zkontrolujI2C();

  unsigned long startLoga = millis();
  if (!poziceOk)
  { ulozPoziciX27(0);
    homingRucicky(); }                     // při studeném startu běží během loga
  if (!teplyStart)
  { while (millis() - startLoga < LOGO_DOBA_ZOBRAZENI_MS) { } }

  bodOk = jeBodNastaveny();

  // Ručička z Timeru1: cíl = kde je, ať se nic nehne.
  x27CilLoop = x27Pozice;
  x27Cil = x27Pozice;
  if (drzime) nastavRucickuKmH(vZobrazenaKmh * TEST_MULTIPLIER);
  OCR1A = X27_KLID_TIK_US * 2 - 1;
  TCNT1 = 0;
  nastavTimerRucicky();
  nastavHeartbeatTimer();

  // Přerušení pro kolo. Stará čekající hrana pryč.
  EIFR = _BV(INTF0);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), snimacSpeedoPreruseni, FALLING);

#ifdef TACHO
  pinMode(TACHO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACHO_PIN), snimacTachoPreruseni, FALLING);
#endif

  referenceUs = micros();                  // od teď běží držení po resetu
  posledniVyhlazeniUs = referenceUs;
  vykresliDashboard();
  zkontrolujI2C();

  // Přísný 2s watchdog pro běžný provoz.
  wdt_enable(WDTO_2S);
}

// ============================================================
// 12) LOOP
// ============================================================

void loop()
{
  wdt_reset();

  unsigned long casPulzu;
  while (vezmiPulzZFronty(casPulzu))
  { zpracujPulz(casPulzu); }

  aktualizujZobrazenouRychlost(micros());

  float rucickaKmh = vZobrazenaKmh * TEST_MULTIPLIER;
#ifdef TEST_RUCICKA_KMH
  rucickaKmh = TEST_RUCICKA_KMH;   // kalibrace stupnice: ručka i displej drží pevnou hodnotu
#endif
  nastavRucickuKmH(rucickaKmh);

  ulozOdoPokudJeTreba();

#ifdef TACHO
  zpracujNovyTachoPulzPokudExistuje();
  zkontrolujJestliStojimeTacho();
#endif

  unsigned long ted = millis();

  if (ted - zachranaNaposledyCas >= 20)
  { zachranaNaposledyCas = ted;
    ulozZachranu(); }

  if (ted - displejNaposledyCas >= DISPLEJ_INTERVAL_MS)
  { displejNaposledyCas = ted;
    vykresliDashboard();
    zkontrolujI2C(); }

  if (ted - udrzbaNaposledyCas >= UDRZBA_INTERVAL_MS)
  { udrzbaNaposledyCas = ted;
    if ((ted / UDRZBA_INTERVAL_MS) % 2 == 0 && hlucnost > 0) hlucnost--;
#if KONTROLA_NAPETI_ZAPNUTA
    posledniZmereneVccMv = nactiNapetiVcc();   // pro debug řádek a zápisy do EEPROM
#endif
    zapisResetStatPokudJdeTo();
    obnovHardware();
    obnovNastaveniDispleje(); }
}

// ============================================================
// 13) DISPLEJ
// ============================================================

// Ikonka top speed 5x7: (šipka nahoru)
const uint8_t IKONA_TOP[] PROGMEM = {
0xff,0xe4,0xee,0xff,0xe4,0xe4,0xe4
};

void vykresliDashboard()
{
  unsigned int testRychlost = zobrazenaRychlostKmh * TEST_MULTIPLIER;
#ifdef TEST_RUCICKA_KMH
  testRychlost = TEST_RUCICKA_KMH;
#endif

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_crox4h_tn);  //výška 21, álá helvetica (setcursor 0,14)

  // -- Řádek 1: ODO --
  if (testRychlost < PRAH_RYCHLOSTI){
  char radekOdo[16];  snprintf(radekOdo, sizeof(radekOdo), "%lu", odoMetry / 1000UL);
  u8g2.setCursor(0, 17);
  u8g2.print(radekOdo);}

  // -- TRIP (zarovnaný vpravo) --
  char cisloTripu[12];
  snprintf(cisloTripu, sizeof(cisloTripu), "%lu.%lu", tripMetry / 1000UL, (tripMetry % 1000UL) / 100UL);
  int sirka = u8g2.getStrWidth(cisloTripu);
  u8g2.setCursor(u8g2.getDisplayWidth() - sirka, 17);
  u8g2.print(cisloTripu);

  // -- VELKA RYCHLOST nad xx km/h --
  u8g2.setFont(u8g2_font_logisoso32_tn);
  if (testRychlost >= PRAH_RYCHLOSTI){
  char radekRychlost[16];
  snprintf(radekRychlost, sizeof(radekRychlost), "%u", testRychlost);
  if (testRychlost > 99) {u8g2.setCursor(30, 32);}
    else {u8g2.setCursor(43, 32);}
  u8g2.print(radekRychlost);}

  u8g2.setFont(u8g2_font_6x10_tr);

  // -- VELKA RYCHLOST: maly popisek km/h --
  if (testRychlost != 0 && testRychlost >= PRAH_RYCHLOSTI){
  u8g2.setCursor(7, 18);
  u8g2.print("km/h");}

  // -- Řádek 2: Pomalá Rychlost + km/h --
  if (testRychlost != 0 && testRychlost < PRAH_RYCHLOSTI){
  char radekRychlost[16];
  snprintf(radekRychlost, sizeof(radekRychlost), "%u km/h", testRychlost);
  u8g2.setCursor(43, 32);
  u8g2.print(radekRychlost);}

  // -- Řádek 2: TOP speed (v debug verzi je tam při stání statistika resetů) --
#ifdef DEBUG_MODE
  if (false){
#else
  if (testRychlost == 0 && topRychlostKmh !=0){
#endif
    u8g2.drawXBMP(50, 25, 5, 7, IKONA_TOP);
    u8g2.setCursor(60,32);
    u8g2.print(topRychlostKmh * TEST_MULTIPLIER);}

  // -- Hnědý poplach: tečka v rohu po odmítnutém zápisu ODO (nízké Vcc) --
  if (nizkeNapeti)
  { u8g2.drawBox(0, 0, 1, 1); }

  // -- DEBUG řádky (uprostřed, kvůli výřezu ciferníku) --
  // Pod ODO (při stání a pomalé jízdě):
  //   R resety se zachovanou RAM, I I2C timeouty, E odmítnuté zápisy ODO,
  //   J jiskry v ISR, G zahozené pulzy, M vynechané pulzy, Vcc, !BOD
  // Dole (jen při stání, místo TOP):
  //   poslední příčina resetu + počty resetů z EEPROM (přežijí i vypnutí):
  //   P napájení, B podpětí, W watchdog, X reset pin, S pád programu
#ifdef DEBUG_MODE
  if (testRychlost < PRAH_RYCHLOSTI)
  { char radek[44];
    uint16_t jiskry;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { jiskry = isrJiskryPocet; }
    snprintf(radek, sizeof(radek), "R%u I%u E%u J%u G%u M%u",
             resetPocet, i2cTimeoutPocet, eepromNapetiPocet, jiskry, zamitnutoPocet, vynechanoPocet);
    size_t n = strlen(radek);
    if (posledniZmereneVccMv > 0)
    { snprintf(radek + n, sizeof(radek) - n, " %u.%uV",
               (unsigned int)(posledniZmereneVccMv / 1000), (unsigned int)((posledniZmereneVccMv % 1000) / 100));
      n = strlen(radek); }
    if (!bodOk) snprintf(radek + n, sizeof(radek) - n, " !BOD");
    u8g2.setFont(u8g2_font_5x7_tr);
    int x = (128 - (int)u8g2.getStrWidth(radek)) / 2;
    u8g2.setCursor(x < 0 ? 0 : x, 24);
    u8g2.print(radek);

    if (testRychlost == 0)
    { snprintf(radek, sizeof(radek), "%c P%u B%u W%u X%u S%u", pricinaResetu(),
               resetStat[0], resetStat[1], resetStat[2], resetStat[3], resetStat[4]);
      x = (128 - (int)u8g2.getStrWidth(radek)) / 2;
      u8g2.setCursor(x < 0 ? 0 : x, 32);
      u8g2.print(radek); }
  }
#endif

  u8g2.sendBuffer();
}
