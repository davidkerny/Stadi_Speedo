#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "stadionlogo.h"

// ============================================================

// Texty v PROGMEM = leží ve flash, ne v RAM.
const char FW_VERZE[] PROGMEM = "  Stadi GPSGauge V1.0";

//  RYCHLOST se bere z GPS (u-blox M10, MicroAir MG-A01) po UARTu
//  D0 = RX (z GPS TX), D1 = TX (do GPS RX). Věty NMEA: RMC + GGA.
//  Tacho čte otáčky motoru na D3 = RPM (beze změny).
//  ODO/TRIP se počítá integrací GPS rychlosti (žádný snímač kola).
//  ODOmetr je v EEPROM. TRIP, TopRychlost a TopRPM v RAM.
//
//  Vpravo nahoře: ikonka satelitu + počet satelitů ve fixu.
//    plný satelit  = máme fix
//    blikající     = modul žije, ale hledá fix
//    křížek (X)    = od modulu nechodí žádná data (kabel/baud)
//
// ============================================================

// 1) NASTAVENÍ
// 2) STAV PROGRAMU
// 3) MĚŘENÍ NAPÁJENÍ (Vcc)
// 4) EEPROM - ukládání ODO
// 5) GPS - KONFIGURACE MODULU (UBX)
// 6) PŘERUŠENÍ - TACHO
// 7) GPS - PŘÍJEM A PARSER NMEA
// 8) FILTR TACHA
// 9A) HEARTBEAT LED (diagnostika, D13)
// 9B) ÚVODNÍ LOGO
// 10) HLAVNÍ SMYČKA - SETUP
// 11) ZPRACOVÁNÍ GPS RYCHLOSTI A ODO
// 12) ZPRACOVÁNÍ TACHA
// 13) KONTROLA ZTRÁTY GPS
// 14) KONTROLA ZASTAVENÍ TACHA
// 15) LOOP
// 16) DISPLEJ
//  16.1) VÝPOČTY A LOGIKA REŽIMŮ (PŘED KRESLENÍM)
//  16.2) KRESLENÍ
//  16.3) LAYOUT A: TACHO aka RevMode
//  16.4) LAYOUT B: SPÍDO

// ============================================================
// 1) NASTAVENÍ
// ============================================================

// --- TESTOVACÍ NÁSOBIČ (pro finální provoz změň na 1) ---
#define TEST_MULTIPLIER 1

// --- OLED init ---
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);//Klasika 0.96"
//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); //Laskakit 1.3"

// --- GPS modul (HW UART, D0 = RX, D1 = TX) ---
const unsigned long GPS_BAUD = 9600UL;     // tovární rychlost M10 je 9600; když modul mlčí, zkus 38400
#define GPS_KONFIGURACE 1                  // 1 = po první větě pošli modulu UBX konfiguraci (viz sekce 5)
const unsigned int GPS_RATE_MS = 200;      // perioda měření: 200 = 5 Hz (při 9600 Bd víc nedávat)
const byte GPS_GGA_DELITEL = 1000 / GPS_RATE_MS;  // GGA (počet satelitů) stačí 1x za sekundu
const byte GPS_RADEK_MAX = 100;            // max. délka věty NMEA (norma 82)

// --- GPS rychlost ---
const float GPS_MIN_KMH = 5.0;             // pod tím je to šum stojícího GPS -> rychlost 0
const float GPS_VYHLAZENI = 0.5;           // 1.0 = bez vyhlazení, nižší = plynulejší, ale pomalejší
const float MAX_ROZUMNA_RYCHLOST_KMH = 120.0;  // nad tím vzorek ignorujem
const unsigned long GPS_TIMEOUT_MS = 2000UL;   // tohle dlouho bez věty = ztráta GPS
const unsigned long GPS_MAX_MEZERA_MS = 1500UL; // delší mezera mezi RMC se do ODO nepočítá

// --- GPS indikátor na displeji (vpravo nahoře) ---
const int GPS_IKONA_Y = 1;                 // horní řádek ikonky (5x7)
const int GPS_TEXT_Y = 8;                  // baseline čísla satelitů
const byte GPS_STAV_BEZ_KOMUNIKACE = 0;
const byte GPS_STAV_HLEDA = 1;
const byte GPS_STAV_FIX = 2;

// --- TACHO snímač motor rpm (INT1) ---
const byte TACHO_PIN = 3; // ducati energia simson

// --- TACHO tuning ---
// const unsigned long MIN_MEZERA_TACHO_US = 2800UL; v případě 1pulz=1ot.
const unsigned long MIN_MEZERA_TACHO_US = 1333UL; // (15Krpm 3pulz/ot)
const float PULZY_NA_OTACKU = 3.0;  // Tacho pulzy per otáčka DUCATI ENERGIA
const float MAX_ROZUMNE_RPM = 15000.0;
const float MIN_ROZUMNE_RPM = 500.0;
const float MAX_SKOK_RPM = 7000.0; // benevolentnější měření
const float MAX_ROZDIL_SOUSEDNICH_PULZU_RPM = 2500.0; // benevolentnější měření
const unsigned long CAS_DO_ZASTAVENI_TACHO_MS = 200UL; // Když 200 ms nic nepřijde, motor chcípl.
const float TACHO_KALMAN_Q = 2.0; // Vyšší Q = rychlejší reakce na vrknutí plynem.
const float TACHO_KALMAN_R = 8.0;

// --- Welcome logo ---
const unsigned long LOGO_DOBA_ZOBRAZENI_MS = 1500UL;
const byte LOGO_POCET_BLIKNUTI = 8;

// --- EEPROM / ODO ---
const int ODO_ADR_START = 0;
const int ODO_ADR_KONEC = 252;
const unsigned long KROK_ULOZENI_M = 100UL;
const unsigned long ODO_FYZICKY_STROP_M = 100000000UL;
const unsigned long ODO_MAX_SKOK_MEZI_BUNKAMI_M = 1000000UL;
#define KONTROLA_NAPETI_ZAPNUTA 1
const long MIN_VCC_PRO_ZAPIS_MV = 3400L;

// --- Displej ---
const unsigned long DISPLEJ_INTERVAL_MS = 100;

// ============================================================
// 2) STAV PROGRAMU
// ============================================================

// -- Ujetá vzdálenost --
unsigned long odoMetry = 0;

// TRIP, TopRychlost a TopRPM + magic number leží v .noinit -> startup kód je při resetu
// nenuluje. Při studeném startu v nich je náhodný obsah.
unsigned long tripMetry __attribute__((section(".noinit")));
unsigned long tripMagic __attribute__((section(".noinit")));
unsigned int topRychlostKmh __attribute__((section(".noinit")));
unsigned int topRpm __attribute__((section(".noinit")));

// -- Aktuální rychlost (z GPS) --
unsigned int zobrazenaRychlostKmh = 0;

// -- Aktuální otáčky Tacho --
unsigned int zobrazeneRpm = 0;
bool rezimOtackomeru = false; // Režim otáčkoměru na začátku vypnutý

// -- GPS --
byte gpsPocetSatelitu = 0;            // satelity použité ve fixu (z GGA)
bool gpsMaFix = false;                // RMC status 'A'
bool gpsKomunikuje = false;           // chodí platné věty NMEA
bool gpsKonfigurovano = false;        // UBX konfigurace už byla poslána
unsigned long gpsCasPosledniVetyMs = 0;   // millis() poslední platné věty (0 = nikdy)
unsigned long gpsCasPoslednihoRmcMs = 0;  // millis() posledního RMC (0 = nikdy)
float gpsRychlostVyhlazena = 0.0;
char gpsRadek[GPS_RADEK_MAX];         // právě skládaná věta (bez '$')
byte gpsDelka = 0;
bool gpsVRadku = false;               // jsme mezi '$' a koncem řádku

// -- Kalman tacho --
float tachoKalman_x = 0;
float tachoKalman_p = 1;

// -- Pomoc tacha s rušením --
float posledniSuroveRpm = 0.0;
bool tachoMamePredchoziPulz = false;
bool motorBezi = false;

// -- EEPROM housekeeping --
int odoAdresaAktualni = ODO_ADR_START;
unsigned long odoNaposledyUlozeno = 0;

// -- Časování displeje --
unsigned long displejNaposledyCas = 0;

// -- Sdílené s tacho ISR --
volatile unsigned long tachoIsrCasPoslednihoPulzuUs = 0;
volatile unsigned long tachoIsrDelkaPoslednihoIntervaluUs = 0;
volatile bool tachoIsrMameNovyPulz = false;

// -- Heartbeat LED (diagnostika, D13) --
volatile uint8_t heartbeatPocitadloPreruseni = 0;

// ============================================================
// 3) MĚŘENÍ NAPÁJENÍ (Vcc)
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
  const long VCC_KALIBRACNI_KONSTANTA = 1125300L;
  return VCC_KALIBRACNI_KONSTANTA / vysledekAdc;}
#endif

// ============================================================
// 4) EEPROM a RAM - ukládání ODO, TRIP, topRpm
// ============================================================

void nactiOdoZEEPROM()
{  unsigned long nejvyssiPlatnaHodnota = 0;
  int adresaSNejvyssiHodnotou = ODO_ADR_START;
  for (int adr = ODO_ADR_START; adr <= ODO_ADR_KONEC; adr += 4)
  { unsigned long hodnota = 0;
    EEPROM.get(adr, hodnota);
    if (hodnota == 0xFFFFFFFFUL) continue;
    if (hodnota > ODO_FYZICKY_STROP_M) continue;
    bool jePodezreleVysoka =
      nejvyssiPlatnaHodnota > 0 &&
      hodnota > nejvyssiPlatnaHodnota + ODO_MAX_SKOK_MEZI_BUNKAMI_M;
    if (jePodezreleVysoka) continue;
    if (hodnota >= nejvyssiPlatnaHodnota)
    { nejvyssiPlatnaHodnota = hodnota;
      adresaSNejvyssiHodnotou = adr;}
  }
  odoMetry = nejvyssiPlatnaHodnota;
  odoAdresaAktualni = adresaSNejvyssiHodnotou;}

bool ulozOdoDoEEPROM(unsigned long hodnotaMetru)
{
#if KONTROLA_NAPETI_ZAPNUTA
  if (nactiNapetiVcc() < MIN_VCC_PRO_ZAPIS_MV)
  { return false; }
#endif
  odoAdresaAktualni += 4;
  if (odoAdresaAktualni > ODO_ADR_KONEC)
  { odoAdresaAktualni = ODO_ADR_START;}
  EEPROM.put(odoAdresaAktualni, hodnotaMetru);
  return true;}

// --- TRIP a TOP hodnoty přežívajou reset (detekce sraček v RAM) ---
const unsigned long TRIP_MAGIC = 0xA5C3F00DUL;  // platná RAM = pokračuj, jinak nuluj


// ============================================================
// 5) GPS - KONFIGURACE MODULU (UBX)
// ============================================================
// Modul po zapnutí jede 1 Hz a posílá všechny věty. Tady mu (jen do RAM,
// po každém startu znovu) řekneme: 5 Hz, jen RMC + GGA.
// Klíče jsou z u-blox M10 interface description (CFG-VALSET).
// Když je modul ignoruje, nic se neděje - jede dál s továrním nastavením.

const uint32_t UBX_KLIC_RATE_MEAS = 0x30210001UL;  // CFG-RATE-MEAS (U2, ms)
const uint32_t UBX_KLIC_NMEA_RMC  = 0x209100acUL;  // CFG-MSGOUT-NMEA_ID_RMC_UART1 (U1)
const uint32_t UBX_KLIC_NMEA_GGA  = 0x209100bbUL;  // ... GGA
const uint32_t UBX_KLIC_NMEA_GLL  = 0x209100caUL;  // ... GLL
const uint32_t UBX_KLIC_NMEA_GSA  = 0x209100c0UL;  // ... GSA
const uint32_t UBX_KLIC_NMEA_GSV  = 0x209100c5UL;  // ... GSV
const uint32_t UBX_KLIC_NMEA_VTG  = 0x209100b1UL;  // ... VTG

// Pošle UBX-CFG-VALSET (vrstva RAM) s jedním klíčem a hodnotou o velikosti 1 nebo 2 bajty.
void gpsNastavKlic(uint32_t klic, uint16_t hodnota, byte velikost)
{
  wdt_reset();

  uint8_t zprava[18];
  byte n = 0;

  zprava[n++] = 0xB5;                    // synchronizace
  zprava[n++] = 0x62;
  zprava[n++] = 0x06;                    // třída CFG
  zprava[n++] = 0x8A;                    // ID VALSET
  zprava[n++] = 8 + velikost;            // délka dat (LSB)
  zprava[n++] = 0;                       // délka dat (MSB)
  zprava[n++] = 0;                       // verze
  zprava[n++] = 0x01;                    // vrstva: RAM
  zprava[n++] = 0;                       // rezervováno
  zprava[n++] = 0;
  for (byte i = 0; i < 4; i++) { zprava[n++] = (klic >> (8 * i)) & 0xFF; }
  for (byte i = 0; i < velikost; i++) { zprava[n++] = (hodnota >> (8 * i)) & 0xFF; }

  // Kontrolní součet přes třídu, ID, délku a data.
  uint8_t ckA = 0, ckB = 0;
  for (byte i = 2; i < n; i++) { ckA += zprava[i]; ckB += ckA; }
  zprava[n++] = ckA;
  zprava[n++] = ckB;

  Serial.write(zprava, n);
  Serial.flush();
  delay(10);
}

void gpsKonfiguruj()
{
  // Věty, které nepotřebujem, pryč (šetří přenos při 9600 Bd).
  gpsNastavKlic(UBX_KLIC_NMEA_GLL, 0, 1);
  gpsNastavKlic(UBX_KLIC_NMEA_GSA, 0, 1);
  gpsNastavKlic(UBX_KLIC_NMEA_VTG, 0, 1);
  gpsNastavKlic(UBX_KLIC_NMEA_GSV, 0, 1);

  // RMC (rychlost, fix) každé měření, GGA (satelity) každé N-té.
  gpsNastavKlic(UBX_KLIC_NMEA_RMC, 1, 1);
  gpsNastavKlic(UBX_KLIC_NMEA_GGA, GPS_GGA_DELITEL, 1);

  // Nakonec rychlost měření.
  gpsNastavKlic(UBX_KLIC_RATE_MEAS, GPS_RATE_MS, 2);
}

// ============================================================
// 6) PŘERUŠENÍ - TACHO
// ============================================================

void snimacTachoPreruseni()
{ unsigned long ted = micros();
  if (tachoIsrCasPoslednihoPulzuUs == 0)
  { tachoIsrCasPoslednihoPulzuUs = ted;
    return;}
  unsigned long interval = ted - tachoIsrCasPoslednihoPulzuUs;

    // Bordel od zapalování - pokud je pulz moc brzo, ignorujem ho.
  if (interval < MIN_MEZERA_TACHO_US)
  { return;}

  tachoIsrDelkaPoslednihoIntervaluUs = interval;
  tachoIsrCasPoslednihoPulzuUs = ted;
  tachoIsrMameNovyPulz = true;}

// ============================================================
// 7) GPS - PŘÍJEM A PARSER NMEA
// ============================================================

// Vrátí začátek aktuálního pole a posune kurzor za čárku. Prázdná pole zachová.
char *dalsiPole(char **kurzor)
{
  static char prazdne[] = "";
  char *zacatek = *kurzor;
  if (zacatek == NULL) { return prazdne; }

  char *carka = strchr(zacatek, ',');
  if (carka != NULL)
  { *carka = '\0';
    *kurzor = carka + 1; }
  else
  { *kurzor = NULL; }
  return zacatek;
}

// Zpracuje jednu kompletní větu (bez '$', bez CR/LF).
void zpracujNmeaRadek()
{
  // Kontrola checksumu: XOR všeho mezi '$' a '*'.
  char *hvezdicka = strchr(gpsRadek, '*');
  if (hvezdicka == NULL || strlen(hvezdicka) < 3) { return; }

  byte soucet = 0;
  for (char *p = gpsRadek; p < hvezdicka; p++) { soucet ^= (byte)*p; }

  char hex[3] = { hvezdicka[1], hvezdicka[2], '\0' };
  if (soucet != (byte)strtol(hex, NULL, 16)) { return; }   // rozbitá věta = zahodit

  *hvezdicka = '\0';
  gpsCasPosledniVetyMs = millis();   // modul žije

#if GPS_KONFIGURACE
  // První platná věta = modul je vzhůru, teď má smysl ho nakonfigurovat.
  if (!gpsKonfigurovano)
  { gpsKonfigurovano = true;
    gpsKonfiguruj();
    return; }
#endif

  if (strlen(gpsRadek) < 6) { return; }

  // Věty se liší jen třemi písmeny za talkerem (GN/GP/...): GNRMC, GPRMC, GNGGA...
  if (strncmp(gpsRadek + 2, "RMC", 3) == 0)
  {
    char *kurzor = gpsRadek;
    for (byte i = 0; i < 2; i++) { dalsiPole(&kurzor); }   // 0 = název věty, 1 = čas
    char *status = dalsiPole(&kurzor);                     // 2 = A (fix) / V (bez fixu)
    for (byte i = 0; i < 4; i++) { dalsiPole(&kurzor); }   // 3-6 = poloha
    char *uzly = dalsiPole(&kurzor);                       // 7 = rychlost v uzlech

    zpracujGpsRychlost(status[0] == 'A', atof(uzly) * 1.852);   // 1 uzel = 1,852 km/h
  }
  else if (strncmp(gpsRadek + 2, "GGA", 3) == 0)
  {
    char *kurzor = gpsRadek;
    for (byte i = 0; i < 7; i++) { dalsiPole(&kurzor); }   // 0-6 = název, čas, poloha, kvalita
    int pocet = atoi(dalsiPole(&kurzor));                  // 7 = počet satelitů ve fixu
    if (pocet < 0) { pocet = 0; }
    if (pocet > 99) { pocet = 99; }
    gpsPocetSatelitu = (byte)pocet;
  }
}

// Čte bajty z UARTu a skládá věty. Volat často (loop + mezi stránkami displeje),
// jinak přeteče 64 B RX buffer.
void nactiGpsBajty()
{
  while (Serial.available())
  {
    char c = (char)Serial.read();

    if (c == '$')                       // začátek věty (i uprostřed rozbité = nová synchronizace)
    { gpsDelka = 0;
      gpsVRadku = true;
      continue; }

    if (!gpsVRadku) { continue; }

    if (c == '\r' || c == '\n')         // konec věty
    { gpsVRadku = false;
      gpsRadek[gpsDelka] = '\0';
      zpracujNmeaRadek();
      continue; }

    if (gpsDelka < GPS_RADEK_MAX - 1)
    { gpsRadek[gpsDelka++] = c; }
    else
    { gpsVRadku = false; }              // moc dlouhé = smetí
  }
}

// ============================================================
// 8) FILTR TACHA
// ============================================================

float vyhladRpm(float noveMereniRpm)
{ tachoKalman_p = tachoKalman_p + TACHO_KALMAN_Q;
  float zisk = tachoKalman_p / (tachoKalman_p + TACHO_KALMAN_R);
  tachoKalman_x = tachoKalman_x + zisk * (noveMereniRpm - tachoKalman_x);
  tachoKalman_p = (1.0 - zisk) * tachoKalman_p;
  return tachoKalman_x;
}

bool jePulzTachaDuveryhodny(float suroveRpm)
{ if (suroveRpm > MAX_ROZUMNE_RPM ||
      suroveRpm < MIN_ROZUMNE_RPM)
  { return false;}

  if (motorBezi)
  { float rozdil = fabs(suroveRpm - tachoKalman_x);

    if (rozdil > MAX_SKOK_RPM)
    { bool souhlasiSPredchozim =
        tachoMamePredchoziPulz &&
        fabs(suroveRpm - posledniSuroveRpm)
          < MAX_ROZDIL_SOUSEDNICH_PULZU_RPM;

      if (souhlasiSPredchozim) // Dva pulzy po sobě sedí -> motor fakt změnil otáčky.
      { tachoKalman_x = suroveRpm;
        tachoKalman_p = 1.0;
      } else { return false;}
    }}
  return true;}


// ============================================================
// 9A) HEARTBEAT LED (diagnostické blikání LED D13)
// ============================================================

// Timer2 -> jeden "tik" cca 16 ms.
// V ISR čeká 16 tiků (~256 ms), než LED přepne. Běží nezávisle na loop().

void nastavHeartbeatTimer()
{ pinMode(13, OUTPUT);
  TCCR2A = (1 << WGM21);                              // CTC mód
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);   // prescaler 1024
  OCR2A = 249;                                         // perioda ~16 ms
  TIMSK2 = (1 << OCIE2A);                              // povolit přerušení
}

ISR(TIMER2_COMPA_vect)
{ heartbeatPocitadloPreruseni++;
  if (heartbeatPocitadloPreruseni >= 16)   // ~16 x 16 ms ≈ 256 ms
  { heartbeatPocitadloPreruseni = 0;
    digitalWrite(13, !digitalRead(13));  }
}

// ============================================================
// 9B) ÚVODNÍ LOGO
// ============================================================

void zobrazUvodniLogo()
{ pinMode(13, OUTPUT);

  const int VYSKA_OBLASTI_LOGA = 50;
  int x = (128 - STADION_WIDTH) / 2;
  int y = (VYSKA_OBLASTI_LOGA - STADION_HEIGHT) / 2;

  // Page buffer: kreslení se opakuje pro každou část displeje.
  u8g2.firstPage();
  do
  { u8g2.drawXBMP(x, y, STADION_WIDTH, STADION_HEIGHT, logo_stadion);
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setCursor(0, 64);
    u8g2.print((__FlashStringHelper*)FW_VERZE); }
  while (u8g2.nextPage());

  const unsigned long krokMs =
    LOGO_DOBA_ZOBRAZENI_MS / (LOGO_POCET_BLIKNUTI * 2);

  for (byte i = 0; i < LOGO_POCET_BLIKNUTI; i++)
  { digitalWrite(13, HIGH);
    delay(krokMs);
    digitalWrite(13, LOW);
    delay(krokMs); }
}

// ============================================================
// 10) HLAVNÍ SMYČKA - SETUP
// ============================================================

void setup()

{ // Po resetu watchdogem by mohl zůstat běžet s krátkým limitem.
  MCUSR = 0;
  wdt_disable();
  wdt_enable(WDTO_8S);

  // === I2C BUS CLEAR ===
  // zvon na hajzl pro I2C. aka defibrilátor = kurva důležité.

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(10);

  bool byloZaseknute = (digitalRead(SDA) == LOW);
  if (byloZaseknute) {
    // 9 hodinových pulzů
    for (int i = 0; i < 9; i++) {
      if (digitalRead(SDA) == HIGH) break; // Displej už pustil SDA, můžeme se na to vyjebat
      pinMode(SCL, OUTPUT);
      digitalWrite(SCL, LOW);
      delayMicroseconds(5);
      pinMode(SCL, INPUT_PULLUP);
      delayMicroseconds(5);
    }

    // STOP podmínka - VŽDY, když jsme dělali clearing
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, LOW);       // SCL = LOW
    delayMicroseconds(5);
    pinMode(SDA, OUTPUT);
    digitalWrite(SDA, LOW);       // SDA = LOW
    delayMicroseconds(5);
    pinMode(SCL, INPUT_PULLUP);   // SCL jde HIGH
    delayMicroseconds(5);
    pinMode(SDA, INPUT_PULLUP);   // SDA LOW->HIGH při SCL=HIGH = STOP
    delayMicroseconds(5);
  }

  // Obnovení běžného I2C režimu
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);


  u8g2.begin();

  // GPS: HW UART (D0/D1).
  Serial.begin(GPS_BAUD);

  // Tacho: D3 / INT1.
  pinMode(TACHO_PIN, INPUT_PULLUP);

  // Magic number nesedí = studený start (v RAM je bordel).
  // Sedí = reset za jízdy, TRIP a TOP zůstávají.
  bool studenyStart = (tripMagic != TRIP_MAGIC);
  if (studenyStart)
  { tripMetry = 0;
    topRychlostKmh = 0;
    topRpm = 0;
    tripMagic = TRIP_MAGIC;

    zobrazUvodniLogo();     // logo jen při studeném startu
  }

  // Během loga se RX buffer zaplnil starými větami - zahodit.
  while (Serial.available()) { Serial.read(); }

  // Heartbeat běží od teď dál, nezávisle na zbytku programu.
  nastavHeartbeatTimer();

  nactiOdoZEEPROM();
  odoNaposledyUlozeno = odoMetry;

  // Přerušení pro otáčky motoru.
  attachInterrupt(digitalPinToInterrupt(TACHO_PIN), snimacTachoPreruseni, FALLING);

  vykresliDashboard();

  // Přezbrojení na přísný 2s watchdog PRO BĚŽNÝ PROVOZ
  wdt_enable(WDTO_2S);
}

// ============================================================
// 11) ZPRACOVÁNÍ GPS RYCHLOSTI A ODO
// ============================================================

// Volá parser při každé větě RMC.
void zpracujGpsRychlost(bool maFix, float rychlostKmh)
{
  unsigned long ted = millis();

  // Čas od minulého RMC (pro integraci vzdálenosti), pak teprve přepsat.
  unsigned long dtMs = ted - gpsCasPoslednihoRmcMs;
  bool mameReferenci = (gpsCasPoslednihoRmcMs != 0) && (dtMs <= GPS_MAX_MEZERA_MS);
  gpsCasPoslednihoRmcMs = ted;

  gpsMaFix = maFix;

  // Bez fixu rychlost neexistuje.
  if (!maFix)
  { zobrazenaRychlostKmh = 0;
    gpsRychlostVyhlazena = 0;
    return; }

  // Nesmysl = vzorek ignorujem.
  if (rychlostKmh > MAX_ROZUMNA_RYCHLOST_KMH)
  { return; }

  // Stojíme (GPS šum): rychlost 0, do ODO nic.
  if (rychlostKmh < GPS_MIN_KMH)
  { zobrazenaRychlostKmh = 0;
    gpsRychlostVyhlazena = 0;
    return; }

  // Lehké vyhlazení (první hodnota po stání se bere rovnou).
  if (gpsRychlostVyhlazena == 0)
  { gpsRychlostVyhlazena = rychlostKmh; }
  else
  { gpsRychlostVyhlazena += GPS_VYHLAZENI * (rychlostKmh - gpsRychlostVyhlazena); }

  zobrazenaRychlostKmh = (unsigned int)(gpsRychlostVyhlazena + 0.5);

  //TOP SPEED
  if (zobrazenaRychlostKmh > topRychlostKmh)
  { topRychlostKmh = zobrazenaRychlostKmh; }

  if (!mameReferenci) { return; }

  // Akumulace zlomkových metrů: v [m/s] * dt [s].
  static float zbytekMetruAkumulator = 0.0;
  zbytekMetruAkumulator += (rychlostKmh / 3.6) * (dtMs / 1000.0);
  if (zbytekMetruAkumulator >= 1.0)
  {    unsigned long celeMetry =
      (unsigned long)zbytekMetruAkumulator;
    odoMetry += celeMetry;
    tripMetry += celeMetry;
    zbytekMetruAkumulator -= celeMetry;
  }

  // EEPROM zápis po 100 m.
  if (odoMetry - odoNaposledyUlozeno >= KROK_ULOZENI_M)
  { if (ulozOdoDoEEPROM(odoMetry))
    { odoNaposledyUlozeno = odoMetry; }}
}

// ============================================================
// 12) ZPRACOVÁNÍ TACHA
// ============================================================

void zpracujNovyTachoPulzPokudExistuje()
{ if (!tachoIsrMameNovyPulz)
  { return; }

  unsigned long intervalUs;

  noInterrupts();
  intervalUs = tachoIsrDelkaPoslednihoIntervaluUs;
  tachoIsrMameNovyPulz = false;
  interrupts();

  // 1 pulz = 1 otáčka / PULZY_NA_OTACKU.
  // 60 000 000 us za minutu / perioda / pulzy_na_otacku = RPM.
  float suroveRpm =
    (60000000.0 / intervalUs) / PULZY_NA_OTACKU;

  if (!jePulzTachaDuveryhodny(suroveRpm))
  {tachoMamePredchoziPulz = false; return;}

  tachoMamePredchoziPulz = true;
  posledniSuroveRpm = suroveRpm;
  motorBezi = true;

  float vyhlazeneRpm = vyhladRpm(suroveRpm);
  zobrazeneRpm = (unsigned int)(vyhlazeneRpm + 0.5);

  if (zobrazeneRpm > topRpm)
  { topRpm = zobrazeneRpm; }
}

// ============================================================
// 13) KONTROLA ZTRÁTY GPS
// ============================================================

void zkontrolujZtratuGps()
{
  unsigned long ted = millis();

  // Dlouho bez RMC: rychlost neznáme -> 0, fix neplatí.
  if (gpsCasPoslednihoRmcMs != 0 && ted - gpsCasPoslednihoRmcMs > GPS_TIMEOUT_MS)
  { zobrazenaRychlostKmh = 0;
    gpsRychlostVyhlazena = 0;
    gpsMaFix = false;
  }

  // Dlouho bez jakékoliv platné věty: modul nekomunikuje.
  gpsKomunikuje = (gpsCasPosledniVetyMs != 0) &&
                  (ted - gpsCasPosledniVetyMs <= GPS_TIMEOUT_MS);
}

// ============================================================
// 14) KONTROLA ZASTAVENÍ TACHA
// ============================================================

void zkontrolujJestliStojimeTacho()
{
  unsigned long casPoslednihoPulzu;

  noInterrupts();
  casPoslednihoPulzu =
    tachoIsrCasPoslednihoPulzuUs;
  interrupts();

  bool mameAsponJedenPulz =
    (casPoslednihoPulzu != 0);

  unsigned long dobaBezPulzuUs =
    micros() - casPoslednihoPulzu;

  if (mameAsponJedenPulz &&
      dobaBezPulzuUs >
        CAS_DO_ZASTAVENI_TACHO_MS * 1000UL)
  {
    tachoKalman_x = 0;
    tachoKalman_p = 1;
    zobrazeneRpm = 0;
    tachoMamePredchoziPulz = false;
    motorBezi = false;
  }
}

// ============================================================
// 15) LOOP
// ============================================================

void loop()
{
  // Watchdog krmíme pravidelně, jinak MCU udělá reset.
  wdt_reset();

  nactiGpsBajty();
  zpracujNovyTachoPulzPokudExistuje();

  zkontrolujZtratuGps();
  zkontrolujJestliStojimeTacho();

  unsigned long ted = millis();

  if (ted - displejNaposledyCas > DISPLEJ_INTERVAL_MS)
  {
    vykresliDashboard();
    displejNaposledyCas = ted;
  }
}

// ============================================================
// 16) DISPLEJ
// ============================================================

// Ikonka TOP 5x7: (šipka nahoru)
const uint8_t IKONA_TOP[] PROGMEM = {
  0xff, 0xe4, 0xee, 0xff, 0xe4, 0xe4, 0xe4
};

// Ikonky GPS 5x7 (XBM: 1 bajt = 1 řádek, nejnižší bit = levý pixel; ikonky jsou symetrické).
// Fix: plný satelit s paprsky
const uint8_t IKONA_GPS_FIX[] PROGMEM = {
  0x04,   // ..#..
  0x15,   // #.#.#
  0x1F,   // #####
  0x15,   // #.#.#
  0x04,   // ..#..
  0x0A,   // .#.#.
  0x11    // #...#
};
// Hledá: prázdný satelit bez paprsků (v displeji bliká)
const uint8_t IKONA_GPS_HLEDA[] PROGMEM = {
  0x04,   // ..#..
  0x15,   // #.#.#
  0x1B,   // ##.##
  0x15,   // #.#.#
  0x04,   // ..#..
  0x00,   // .....
  0x00    // .....
};
// Bez komunikace s modulem: křížek
const uint8_t IKONA_GPS_NIC[] PROGMEM = {
  0x00,   // .....
  0x11,   // #...#
  0x0A,   // .#.#.
  0x04,   // ..#..
  0x0A,   // .#.#.
  0x11,   // #...#
  0x00    // .....
};

void vykresliDashboard()
{
  // --------------------------------------------------------
  // 16.1) VÝPOČTY A LOGIKA REŽIMŮ (PŘED KRESLENÍM)
  // --------------------------------------------------------
  // Kreslení v page bufferu se opakuje (u "_2_" 4x), takže uvnitř smyčky
  // má být jen kreslení, nic jiného (kromě čtení GPS, viz níž).
  // --------------------------------------------------------

  // Aplikace násobiče pro testovací účely
  unsigned int testRpm = zobrazeneRpm * TEST_MULTIPLIER;
  unsigned int testRychlost = zobrazenaRychlostKmh * TEST_MULTIPLIER;

  // Hystereze a logika přepínání do Režimu Otáčkoměru (Rev Mode)
  if (odoMetry < 100 && testRpm > 0)     // kdyz ma budik najeto 0km, ukazujem tacho.
  {rezimOtackomeru = true;}
  else if (topRychlostKmh == 0 && testRpm > 0)   // túrujem na místě ale ještě jsme nevyjeli: budík je tacho
  {rezimOtackomeru = true;}
  else if (testRychlost == 0 && testRpm >= 5000)  // tůrujem po jízdě, ukazujem tacho.
  {rezimOtackomeru = true;}
  else if (testRychlost > 0 || testRpm < 2400)  // dotůrovali jsme.
  {rezimOtackomeru = false;}

  // RPM ve stovkách (8000 RPM = 80)
  unsigned int rpmStovky = testRpm / 100;

  // TRIP KM celé+desetiny (2.3km)- příprava proměnných pro celý scope
  char tripText[16] = "";
  int sirkaTripu = 0;
  if (tripMetry > 0){
  unsigned long tripDesetiny = tripMetry / 100UL;
  snprintf(tripText, sizeof(tripText), "t %lu.%lu", tripDesetiny / 10, tripDesetiny % 10);}

  // GPS indikátor: stav a text s počtem satelitů
  byte gpsStav = GPS_STAV_BEZ_KOMUNIKACE;
  if (gpsKomunikuje) { gpsStav = gpsMaFix ? GPS_STAV_FIX : GPS_STAV_HLEDA; }

  char gpsText[4] = "";
  if (gpsStav != GPS_STAV_BEZ_KOMUNIKACE)
  { snprintf(gpsText, sizeof(gpsText), "%u", (unsigned int)gpsPocetSatelitu); }

  bool gpsBlikFaze = ((millis() / 500UL) % 2UL) == 0;   // hledání: ikonka bliká 1 Hz

  // Šířky pro zarovnání doprava (font musí být nastavený před getStrWidth)
  u8g2.setFont(u8g2_font_6x10_tf);
  sirkaTripu = u8g2.getStrWidth(tripText);
  int sirkaGps = u8g2.getStrWidth(gpsText);
  int gpsTextX = 128 - sirkaGps;
  int gpsIkonaX = 128 - sirkaGps - (sirkaGps > 0 ? 3 : 0) - 5;

  // --------------------------------------------------------
  // 16.2) KRESLENÍ
  // --------------------------------------------------------

  u8g2.firstPage(); do {

    // Mezi stránkami displeje (každá cca 25 ms I2C) dočteme GPS, ať nepřeteče RX buffer.
    nactiGpsBajty();

    // ----------------------------------------------------
    // GPS indikátor vpravo nahoře (společný pro oba layouty)
    // ----------------------------------------------------
    if (gpsStav == GPS_STAV_FIX)
    { u8g2.drawXBMP(gpsIkonaX, GPS_IKONA_Y, 5, 7, IKONA_GPS_FIX); }
    else if (gpsStav == GPS_STAV_HLEDA)
    { if (gpsBlikFaze) { u8g2.drawXBMP(gpsIkonaX, GPS_IKONA_Y, 5, 7, IKONA_GPS_HLEDA); } }
    else
    { u8g2.drawXBMP(gpsIkonaX, GPS_IKONA_Y, 5, 7, IKONA_GPS_NIC); }

    if (sirkaGps > 0)
    { u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(gpsTextX, GPS_TEXT_Y);
      u8g2.print(gpsText); }

    if (rezimOtackomeru)
    {
      // ====================================================
      // 16.3) TACHO aka RevMode
      // ====================================================

      u8g2.setFont(u8g2_font_logisoso42_tn);
      unsigned int tisiceRpm = testRpm / 1000;
      unsigned int stovkyZbytekRpm = testRpm % 1000;

      if (testRpm < 1000)
      { if (testRpm < 10)       { u8g2.setCursor(50, 45); }
        else if (testRpm < 100) { u8g2.setCursor(33, 45); }
        else                    { u8g2.setCursor(20, 45); }
        u8g2.print(testRpm);}
      else {
        int RpmStartX = (tisiceRpm >= 10) ? -14 : 0;
        u8g2.setCursor(RpmStartX, 45);
        u8g2.print(tisiceRpm);
        // Mezera 10px mezi tisíci a stovkami
        int xKurzor = u8g2.getCursorX() + 10;
        u8g2.setCursor(xKurzor, 45);
        if (stovkyZbytekRpm < 10)       { u8g2.print(F("00")); }
        else if (stovkyZbytekRpm < 100) { u8g2.print('0'); }
        u8g2.print(stovkyZbytekRpm);
      }

      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 64);
      u8g2.print(F("RPM"));

      if (topRpm != 0)   // TOP RPM
      { u8g2.drawXBMP(26, 57, 5, 7, IKONA_TOP);
        u8g2.setCursor(35, 64);
        u8g2.print(topRpm * TEST_MULTIPLIER);}

      if (tripMetry > 100) // TRIP v otáčkoměru
      { u8g2.setCursor(128 - sirkaTripu, 64);
        u8g2.print(tripText);
      }
    }else{


      // ====================================================
      // 16.4) LAYOUT B: SPÍDO
      // ====================================================


    // ----------------------------------------------------
    // RPM vpravo nahoře (ve stovkách RPM). Zobrazí se pokud nějaké otáčky čteme
    // ----------------------------------------------------

    if (testRpm != 0){
      u8g2.setFont(u8g2_font_logisoso24_tn);
      if (rpmStovky < 100) { u8g2.setCursor(99, 31); }  // dvoumístné
      else                 { u8g2.setCursor(85, 31); }  // třímístné
      if (rpmStovky < 10)  { u8g2.print('0'); }         // vodicí nula
      u8g2.print(rpmStovky);
    }

    // ----------------------------------------------------
    // Velké číslo rychlosti.
    // ----------------------------------------------------

    u8g2.setFont(u8g2_font_logisoso58_tn);

    if (testRychlost < 10)       { u8g2.setCursor(37, 58); }  // (x,y)
    else if (testRychlost < 100) { u8g2.setCursor(6, 58); }
    else                         { u8g2.setCursor(-20, 58); }
    u8g2.print(testRychlost);


    // ----------------------------------------------------
    // TOP SPEED (jen když stojíme a něco už se zaznamenalo).
    // ----------------------------------------------------

    if (testRychlost == 0 && topRychlostKmh != 0)
    { if (testRpm == 0) { u8g2.drawXBMP(98, 17, 5, 7, IKONA_TOP); }  // (šipka nahoru; x,y,w,h)
      else              { u8g2.drawXBMP(0, 10, 5, 7, IKONA_TOP); }

      u8g2.setFont(u8g2_font_6x10_tf);
      if (testRpm == 0) { u8g2.setCursor(106, 24); }
      else              { u8g2.setCursor(7, 17); }
      u8g2.print(topRychlostKmh * TEST_MULTIPLIER);
    }

    // ----------------------------------------------------
    // Drobné nápisy: TRIP, km/h, rpm, ODO
    // ----------------------------------------------------
    u8g2.setFont(u8g2_font_6x10_tf);

    // TRIP, zarovnaný doprava
    //_______________________________________
    if (tripMetry > 90) {
    u8g2.setCursor(128 - sirkaTripu, 64);
    u8g2.print(tripText);}

    // Popisek km/h, jen když nečteme RPM
    //_______________________________________
    if (testRpm == 0)
    { u8g2.setCursor(98, 36);
      u8g2.print(F("km/h"));
    }

    // Při pomalé jízdě: popisek rpm a ODO km
    //_______________________________________
    if (testRychlost <= 19)
    { if (testRpm != 0)
      { u8g2.setCursor(103, 41);
        u8g2.print(F("rpm"));
      }
      if (odoMetry >= 900) // pokud už jsme nějaké zaznamenali
      { u8g2.setCursor(0, 64);
        u8g2.print(odoMetry / 1000);}
    }
    }
  }while (u8g2.nextPage());
}
