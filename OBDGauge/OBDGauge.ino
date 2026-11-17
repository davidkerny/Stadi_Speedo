#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "stadionlogo.h"

// Bez timeoutu ve Wire se zaseklé I2C nikdy nevzpamatuje (displej zamrzne, watchdog resetuje dokola).
#ifndef WIRE_TIMEOUT
#error "Wire je bez timeoutu! Odkomentuj #define WIRE_TIMEOUT v Arduino15/packages/MiniCore/hardware/avr/<verze>/libraries/Wire/src/Wire_timeout.h"
#endif

// ============================================================

const char FW_VERZE[] PROGMEM = "  OBDGauge V0.22";

//  Čtyři hodnoty z OBD-II přes CAN (500 kbps, 11bit, ISO 15765-4), dělané na
//  Subaru XV 2.0D (EE20, 2017) - ale chodit to bude na čemkoli s CANem:
//    BOOST   = MAP (PID 0x87 nebo 0x0B) - barometr (PID 0x33)     [bar]
//    COOLANT = PID 0x05                                             [°C]
//    OIL     = PID 0x5C, když ho ECU umí; jinak volitelně mode 22   [°C]
//    BATT    = PID 0x42 (napětí řídicí jednotky = palubní síť)      [V]
//  Torque dělá přesně tohle samé, jen přes ELM327.
//
//  Po startu se ECU zeptá, které PIDy umí (0x00, 0x20, ...), a co neumí, neptá se.
//  Ztlumení displeje: drát od osvětlení (D3) nebo bit z CAN rámce.
//  REZIM_SNIFF 1 = místo budíku CAN sniffer na USB serial (hledání bitu světel).
//
//  MCP2515 se řeší bit-bangem na libovolných pinech, žádná knihovna.
//  CAN_CS=A0, CAN_SCK=A1, CAN_MISO=4, CAN_MOSI=5  (AUX konektor)
//
// ============================================================

// 1) NASTAVENÍ
// 2) STAV PROGRAMU
// 3) MCP2515 (SPI, init, posílání, příjem)
// 4) OBD: PODPOROVANÉ PIDY, DOTAZY, ODPOVĚDI
// 5) ZTLUMENÍ DISPLEJE
// 6) HEARTBEAT LED (D13)
// 7) ÚVODNÍ LOGO
// 8) SETUP
// 9) LOOP
// 10) DISPLEJ
// 11) SNIFFER

// ============================================================
// 1) NASTAVENÍ
// ============================================================

// --- OLED init (page buffer "_2_": kreslit jen v firstPage()/nextPage()) ---
//U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);//Klasika 0.96"
U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); //Laskakit 1.3"

// --- MCP2515 piny (software SPI, přehodit se dají jak chceš) ---
const byte CAN_CS = A0, CAN_SCK = A1, CAN_MISO = 4, CAN_MOSI = 5;
#define MCP_KRYSTAL_MHZ 8      // 8 nebo 16 - co je napsané na krystalu destičky (čínský bývá 8)

// Přes ICSP (MISO 12, MOSI 11, SCK 13): SCK je D13 = heartbeat LED, tak ji tam vypni!
#define HEARTBEAT_LED 1

// --- Režim ---
//  0 = budík
//  1 = sniffer: MCP jen poslouchá (listen-only, nic nevysílá), na USB serial 115200
//      vypisuje změny bitů v rámcích. Prvních SNIFF_UCENI_MS na nic nesahej - naučí se,
//      které bity se mění samy (čítače, otáčky...). Pak zapínej/vypínej světla a dívej se,
//      který bit se překlápí s nimi. Ten pak patří do DIM_CAN_xxx níž.
#define REZIM_SNIFF 0
const unsigned long SNIFF_UCENI_MS = 10000UL;

// --- Adresy ECU motoru ---
const uint16_t OBD_FUNKCNI_ID = 0x7DF;   // dotaz "všem" (mode 01, jako Torque)
const uint16_t ECU_DOTAZ_ID   = 0x7E0;   // fyzická adresa motoru (mode 22)
const uint16_t ECU_ODPOVED_ID = 0x7E8;   // odpovědi motoru; ostatní ECU ignorujem

// --- Olej ---
//  EE20 PID 0x5C (standardní olej) nejspíš neumí - firmware to po startu sám zjistí.
//  Pak zbývá výrobcův mode 22 (to, co v Torque zadáváš jako "custom PID", třeba "221xxx").
//  Na EE20 není veřejně ověřený; jak ho najít je v OBDGauge/README_OBD.md.
//  Až ho budeš mít ověřený v Torque, zadej ho sem 1:1 (hlavička 7E0, vzorec A+offset).
#define OLEJ_MODE22 0
const uint16_t OLEJ_M22_PID = 0x0000;    // např. 0x1234 pro Torque "221234"
const int      OLEJ_M22_OFFSET = -40;    // Torque vzorec "A-40" -> -40

// --- Boost ---
const int  BARO_VYCHOZI_KPA = 101;       // když ECU nedá PID 0x33
#define BOOST_DESETINY 2                 // 2 = "1.25 bar", 1 = "1.2 bar"

// --- Ztlumení displeje (když svítí světla) ---
//  0 = vypnuto
//  1 = drát: D3 (vstup ENG RPM na desce, snese 12 V) na osvětlení / obrysovky
//  2 = CAN: bit z vysílaného rámce (najdeš sniffem, viz REZIM_SNIFF)
#define DIM_ZDROJ 1
const byte DIM_PIN = 3;
const byte DIM_AKTIVNI_UROVEN = LOW;     // D3 je za optočlenem Q1 (kolektor na D3): napětí na vstupu = LOW
const uint16_t DIM_CAN_ID = 0x000;       // z sniffu
const byte DIM_CAN_BAJT = 0;             // 0..7
const byte DIM_CAN_MASKA = 0x00;         // bit(y) v bajtu; světla = (bajt & maska) != 0
const unsigned long DIM_ODSKOK_MS = 300; // stav musí vydržet tak dlouho, než se přepne

const byte JAS_DEN = 0xCF;               // kontrast 0..255 (u8g2 výchozí 0xCF)
const byte JAS_NOC = 0x00;               // Laskakit SH1106 při 0x00 pořád svítí
#define HLUBOKY_DIM 0                    // 1 = i precharge/VCOMH. Na Laskakit SH1106 to dimuje sotva znatelně a zasekává I2C (i samotné 0xD9) -> nechat 0

// --- Časování ---
const unsigned long ODPOVED_TIMEOUT_MS = 150;     // na jeden dotaz
const unsigned long MEZERA_DOTAZU_MS = 20;        // nezahltit ECU
const unsigned long HODNOTA_PLATNA_MS = 3000;     // pak "--"
const unsigned long CAS_BEZ_ODPOVEDI_MS = 3000;   // celé ticho -> NO DATA + nový start MCP
const unsigned long DISPLEJ_INTERVAL_MS = 100;
const unsigned long OBNOVA_DISPLEJE_MS = 1000;   // znovu poslat registry displeje (proti rušení)
const unsigned long I2C_TIMEOUT_US = 10000UL;     // max čekání na jednu I2C operaci (10 ms)
const byte CHYB_DO_VYRAZENI = 5;                  // tolik timeoutů po sobě = PID neumí

// --- Welcome logo ---
const unsigned long LOGO_DOBA_ZOBRAZENI_MS = 1500UL;
const byte LOGO_POCET_BLIKNUTI = 8;

// ============================================================
// 2) STAV PROGRAMU
// ============================================================

// Magic number v .noinit: logo jen při studeném startu, ne po resetu watchdogem.
const unsigned long START_MAGIC = 0xA5C3F00DUL;
unsigned long startMagic __attribute__((section(".noinit")));

// Sloty = co se ptáme. Každý slot má jednu zobrazovanou (nebo pomocnou) hodnotu.
enum { S_MAP, S_VODA, S_OLEJ, S_NAPETI, S_BARO, S_POCET };

// Pořadí dotazů: boost často (rychle se mění), zbytek jednou za kolo.
const byte SEKVENCE[] = { S_MAP, S_VODA, S_MAP, S_NAPETI, S_MAP, S_OLEJ, S_MAP, S_BARO };
const byte SEKVENCE_DELKA = sizeof(SEKVENCE);

int  hodnota[S_POCET];            // VODA/OLEJ °C, NAPETI mV, MAP a BARO v 0.1 kPa
unsigned long hodnotaCas[S_POCET];
bool hodnotaJe[S_POCET];          // aspoň jednou přišla
byte chybSlotu[S_POCET];          // timeouty po sobě
bool mapSaturace = false;         // PID 0x0B ukazuje 255 kPa = strop, skutečnost může být víc

// Podporované PIDy mode 01: 0x01..0xA0 v 5 bitmapách (dotazy 0x00, 0x20, ... 0x80)
uint32_t podpora[5];
bool podporaZnama = false;        // false = nevíme, zkoušíme všechno
byte podporaKrok = 0;             // který dotaz 0x00/0x20/... je na řadě (0..4)
byte podporaPrislo = 0;           // bit i = bitmapa i přišla
byte podporaPokusu = 0;
bool zjistujiPodporu = true;

// Rozpracovaný dotaz
bool cekam = false;
byte cekMode = 0;
uint16_t cekPid = 0;
byte cekSlot = 0xFF;              // 0xFF = dotaz na podporu, ne slot
unsigned long cekCas = 0;
byte sekvenceIndex = 0;

bool mcpOk = false;
unsigned long odpovedCas = 0, initCas = 0, displejCas = 0;

unsigned long obnovaCas = 0;
bool svetla = false;              // ztlumeno
bool svetlaSurove = false;
unsigned long svetlaZmenaCas = 0;
unsigned long dimCanCas = 0;      // kdy naposled přišel rámec světel
bool dimCanBit = false;

volatile uint8_t heartbeatPocitadloPreruseni = 0;

void vykresliDashboard();

// ============================================================
// 3) MCP2515
// ============================================================

byte spiByte(byte b)   // SPI mód 0, MSB first
{ byte r = 0;
  for (byte i = 0; i < 8; i++)
  { digitalWrite(CAN_MOSI, b & 0x80);
    b <<= 1;
    digitalWrite(CAN_SCK, HIGH);
    r <<= 1;
    if (digitalRead(CAN_MISO)) r |= 1;
    digitalWrite(CAN_SCK, LOW); }
  return r;
}

byte mcpCti(byte reg)
{ digitalWrite(CAN_CS, LOW);
  spiByte(0x03); spiByte(reg);
  byte v = spiByte(0);
  digitalWrite(CAN_CS, HIGH);
  return v;
}

void mcpPis(byte reg, byte a)
{ digitalWrite(CAN_CS, LOW);
  spiByte(0x02); spiByte(reg); spiByte(a);
  digitalWrite(CAN_CS, HIGH);
}

// Filtr/maska: dva po sobě jdoucí registry (SIDH, SIDL) pro 11bit ID
void mcpPisId(byte reg, uint16_t id)
{ digitalWrite(CAN_CS, LOW);
  spiByte(0x02); spiByte(reg); spiByte(id >> 3); spiByte((id & 7) << 5);
  digitalWrite(CAN_CS, HIGH);
}

bool mcpInit()
{ digitalWrite(CAN_CS, LOW); spiByte(0xC0); digitalWrite(CAN_CS, HIGH);   // RESET
  delay(10);
  if ((mcpCti(0x0E) & 0xE0) != 0x80) return false;   // po resetu musí být v config módu

  // Bitrate 500 kbps (CNF1, CNF2, CNF3)
#if MCP_KRYSTAL_MHZ == 16
  mcpPis(0x2A, 0x00); mcpPis(0x29, 0xF0); mcpPis(0x28, 0x86);
#else
  mcpPis(0x2A, 0x00); mcpPis(0x29, 0x90); mcpPis(0x28, 0x82);
#endif

  // Hardwarové filtry, ať Subaru se spoustou rámců nepřeteče buffer, když kreslíme displej.
  //  RXB0 (RXM0, RXF0-1): 0x7E8..0x7EF = odpovědi OBD
  //  RXB1 (RXM1, RXF2-5): rámec světel (DIM_ZDROJ 2), jinak taky OBD
  //  Sniffer: masky 0 = projde všechno.
  const byte filtry[] = { 0x00, 0x04, 0x08, 0x10, 0x14, 0x18 };   // RXF0..RXF5
#if REZIM_SNIFF
  for (byte i = 0; i < 6; i++) mcpPisId(filtry[i], 0);
  mcpPisId(0x20, 0);
  mcpPisId(0x24, 0);
#else
  mcpPisId(filtry[0], 0x7E8); mcpPisId(filtry[1], 0x7E8);
  mcpPisId(0x20, 0x7F8);                                            // RXM0
  #if DIM_ZDROJ == 2
  for (byte i = 2; i < 6; i++) mcpPisId(filtry[i], DIM_CAN_ID);
  mcpPisId(0x24, 0x7FF);                                            // RXM1: přesná shoda
  #else
  for (byte i = 2; i < 6; i++) mcpPisId(filtry[i], 0x7E8);
  mcpPisId(0x24, 0x7F8);
  #endif
#endif
  mcpPis(0x60, 0x04);    // RXB0CTRL: filtry zapnuté, při plném přetéct do RXB1
  mcpPis(0x70, 0x00);    // RXB1CTRL: filtry zapnuté
  mcpPis(0x2B, 0x00);    // CANINTE: bez přerušení, pollujem

#if REZIM_SNIFF
  const byte mod = 0x60;   // listen-only: nic nevysílá, ani ACK -> do auta nezasahuje
#else
  const byte mod = 0x00;   // normální
#endif
  mcpPis(0x0F, mod | 0x08);                          // CANCTRL + one-shot (bez nekonečného opakování)
  for (byte i = 0; i < 50; i++)                      // přepnutí čeká na klid na sběrnici
  { if ((mcpCti(0x0E) & 0xE0) == mod) return true;
    delay(1); }
  return false;
}

void canPosli(uint16_t id, const byte* data)   // vždy DLC 8
{ digitalWrite(CAN_CS, LOW);
  spiByte(0x40);                                   // LOAD TX BUFFER 0, od SIDH
  spiByte(id >> 3); spiByte((id & 7) << 5);
  spiByte(0); spiByte(0);                          // EID (nepoužito)
  spiByte(8);                                      // DLC
  for (byte i = 0; i < 8; i++) spiByte(data[i]);
  digitalWrite(CAN_CS, HIGH);
  digitalWrite(CAN_CS, LOW); spiByte(0x81); digitalWrite(CAN_CS, HIGH);   // RTS TXB0
}

// Vytáhne jeden přijatý rámec. d = SIDH SIDL EID8 EID0 DLC D0..D7. Vrací false, když nic není.
bool canPrijmi(byte* d)
{ byte flagy = mcpCti(0x2C);                       // CANINTF
  byte cmd;
  if (flagy & 0x01)      cmd = 0x90;               // RXB0
  else if (flagy & 0x02) cmd = 0x94;               // RXB1
  else return false;
  digitalWrite(CAN_CS, LOW);
  spiByte(cmd);
  for (byte i = 0; i < 13; i++) d[i] = spiByte(0);
  digitalWrite(CAN_CS, HIGH);                      // tímhle se příznak sám smaže
  return true;
}

inline uint16_t canId(const byte* d) { return ((uint16_t)d[0] << 3) | (d[1] >> 5); }
inline bool canExt(const byte* d)    { return d[1] & 0x08; }

// ============================================================
// 4) OBD
// ============================================================

bool pidUmi(byte pid)
{ if (!podporaZnama) return true;              // nevíme -> zkusit
  if (pid == 0 || pid > 0xA0) return false;
  byte i = (pid - 1) / 32;
  byte bit = 31 - ((pid - 1) % 32);
  return (podpora[i] >> bit) & 1;
}

// Co se pro daný slot posílá. false = slot přeskočit (ECU to neumí).
bool slotDotaz(byte slot, byte& mode, uint16_t& pid)
{ if (chybSlotu[slot] >= CHYB_DO_VYRAZENI) return false;
  mode = 0x01;
  switch (slot)
  { case S_MAP:
      if (podporaZnama && pidUmi(0x87)) pid = 0x87;   // 2 bajty, bez stropu 255 kPa
      else pid = 0x0B;
      return pidUmi(pid);
    case S_VODA:   pid = 0x05; return pidUmi(pid);
    case S_NAPETI: pid = 0x42; return pidUmi(pid);
    case S_BARO:   pid = 0x33; return pidUmi(pid);
    case S_OLEJ:
      if (podporaZnama && pidUmi(0x5C)) { pid = 0x5C; return true; }
#if OLEJ_MODE22
      mode = 0x22; pid = OLEJ_M22_PID; return true;
#else
      if (!podporaZnama) { pid = 0x5C; return true; }
      return false;
#endif
  }
  return false;
}

// Je slot trvale bez dat? (pro "N/A" na displeji)
bool slotNA(byte slot)
{ byte m; uint16_t p;
  if (zjistujiPodporu) return false;
  return !slotDotaz(slot, m, p);
}

void posliDotaz(byte mode, uint16_t pid, byte slot)
{ byte d[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  if (mode == 0x22)
  { d[0] = 0x03; d[1] = 0x22; d[2] = pid >> 8; d[3] = pid & 0xFF;
    canPosli(ECU_DOTAZ_ID, d); }
  else
  { d[0] = 0x02; d[1] = mode; d[2] = pid;
    canPosli(OBD_FUNKCNI_ID, d); }
  cekam = true; cekMode = mode; cekPid = pid; cekSlot = slot;
  cekCas = millis();
}

void ulozHodnotu(byte slot, int v)
{ hodnota[slot] = v;
  hodnotaCas[slot] = millis();
  hodnotaJe[slot] = true;
  chybSlotu[slot] = 0;
}

// Rozebere odpověď ECU (single frame). a = první datový bajt (A), n = počet datových bajtů.
void zpracujData(byte mode, uint16_t pid, const byte* a, byte n)
{ if (mode == 0x01)
  { if ((pid & 0x1F) == 0 && pid <= 0x80 && n >= 4)        // bitmapa podporovaných PIDů
    { podpora[pid / 32] = ((uint32_t)a[0] << 24) | ((uint32_t)a[1] << 16) | ((uint16_t)a[2] << 8) | a[3];
      podporaPrislo |= 1 << (pid / 32);
      return; }
    switch (pid)
    { case 0x05: if (n >= 1) ulozHodnotu(S_VODA, (int)a[0] - 40); break;
      case 0x5C: if (n >= 1) ulozHodnotu(S_OLEJ, (int)a[0] - 40); break;
      case 0x42: if (n >= 2) ulozHodnotu(S_NAPETI, ((uint16_t)a[0] << 8) | a[1]); break;   // mV
      case 0x33: if (n >= 1) ulozHodnotu(S_BARO, a[0] * 10); break;
      case 0x0B: if (n >= 1) { ulozHodnotu(S_MAP, a[0] * 10); mapSaturace = (a[0] == 255); } break;
      case 0x87:                                    // A = které senzory, B-C senzor A, D-E senzor B (1/32 kPa)
        if (n >= 5)
        { uint16_t raw;
          if (a[0] & 0x01)      raw = ((uint16_t)a[1] << 8) | a[2];
          else if (a[0] & 0x02) raw = ((uint16_t)a[3] << 8) | a[4];
          else break;
          ulozHodnotu(S_MAP, (int)(((uint32_t)raw * 10 + 16) / 32));
          mapSaturace = false; }
        break;
    }
  }
#if OLEJ_MODE22
  else if (mode == 0x22 && pid == OLEJ_M22_PID && n >= 1)
    ulozHodnotu(S_OLEJ, (int)a[0] + OLEJ_M22_OFFSET);
#endif
}

void obdZpracujRamec(const byte* d)
{ if (canExt(d) || canId(d) != ECU_ODPOVED_ID) return;
  byte pci = d[5];
  if ((pci & 0xF0) != 0) return;                   // jen single frame
  byte len = pci & 0x0F;
  if (len < 2 || len > 7) return;
  odpovedCas = millis();

  byte sid = d[6];
  if (sid == 0x7F)                                 // negativní odpověď: [7F] [služba] [kód]
  { if (cekam && d[7] == cekMode)
    { if (cekSlot < S_POCET) chybSlotu[cekSlot] = CHYB_DO_VYRAZENI;   // neumí -> rovnou vyřadit
      cekam = false; }
    return; }

  byte mode = sid - 0x40;
  uint16_t pid;
  const byte* a;
  byte n;
  if (mode == 0x22) { if (len < 4) return; pid = ((uint16_t)d[7] << 8) | d[8]; a = &d[9]; n = len - 3; }
  else if (mode == 0x01) { pid = d[7]; a = &d[8]; n = len - 2; }
  else return;

  zpracujData(mode, pid, a, n);
  if (cekam && mode == cekMode && pid == cekPid) cekam = false;
}

void zacniZjistovatPodporu()
{ zjistujiPodporu = true;
  podporaZnama = false;
  podporaKrok = 0;
  podporaPokusu = 0;
  podporaPrislo = 0;
  for (byte i = 0; i < 5; i++) podpora[i] = 0;
  for (byte i = 0; i < S_POCET; i++) chybSlotu[i] = 0;
  cekam = false;
}

// Plánovač: vždycky max jeden dotaz "ve vzduchu".
void obdKrok(unsigned long ted)
{ if (cekam)
  { if (ted - cekCas < ODPOVED_TIMEOUT_MS) return;
    cekam = false;                                         // timeout
    if (cekSlot < S_POCET)
    { if (chybSlotu[cekSlot] < CHYB_DO_VYRAZENI && odpovedCas != 0 && ted - odpovedCas < CAS_BEZ_ODPOVEDI_MS)
        chybSlotu[cekSlot]++; }                            // počítat jen když ECU jinak odpovídá
    else if (++podporaPokusu >= 3)                         // bitmapa nepřišla ani na třetí pokus
    { podporaZnama = (podporaKrok > 0);                    // aspoň první máme -> zbytek "neumí";
      zjistujiPodporu = false; }                           // nic -> nevíme, zkoušet všechno
    cekCas = ted;
  }
  if (ted - cekCas < MEZERA_DOTAZU_MS) return;

  // Fáze 1: které PIDy ECU umí (bitmapy 0x00, 0x20, ... dokud má bit "další rozsah")
  if (zjistujiPodporu)
  { if (podporaPrislo & (1 << podporaKrok))
    { bool dalsi = podpora[podporaKrok] & 1;
      podporaKrok++;
      podporaPokusu = 0;
      if (!dalsi || podporaKrok >= 5)
      { podporaZnama = true;
        zjistujiPodporu = false;
        return; }
    }
    posliDotaz(0x01, podporaKrok * 0x20, 0xFF);
    return;
  }

  // Fáze 2: kolečko hodnot
  for (byte pokus = 0; pokus < SEKVENCE_DELKA; pokus++)
  { byte slot = SEKVENCE[sekvenceIndex];
    sekvenceIndex = (sekvenceIndex + 1) % SEKVENCE_DELKA;
    byte mode; uint16_t pid;
    if (slotDotaz(slot, mode, pid)) { posliDotaz(mode, pid, slot); return; }
  }
}

bool hodnotaPlatna(byte slot, unsigned long ted)
{ return hodnotaJe[slot] && (ted - hodnotaCas[slot] < HODNOTA_PLATNA_MS); }

// ============================================================
// 5) ZTLUMENÍ DISPLEJE
// ============================================================

void nastavJas(bool noc)
{ u8g2.setContrast(noc ? JAS_NOC : JAS_DEN);
#if HLUBOKY_DIM
  // SSD1306/SH1106: pre-charge (0xD9) a VCOMH (0xDB). Denní hodnoty = u8g2 init.
  u8g2.sendF("ca", 0xD9, noc ? 0x22 : 0xF1);
  u8g2.sendF("ca", 0xDB, noc ? 0x00 : 0x40);
#endif
}

void dimRamec(const byte* d)
{
#if DIM_ZDROJ == 2
  if (canExt(d) || canId(d) != DIM_CAN_ID) return;
  if ((d[4] & 0x0F) <= DIM_CAN_BAJT) return;
  dimCanBit = (d[5 + DIM_CAN_BAJT] & DIM_CAN_MASKA) != 0;
  dimCanCas = millis();
#else
  (void)d;
#endif
}

void dimKrok(unsigned long ted)
{
#if DIM_ZDROJ == 0
  (void)ted;
  return;
#else
  #if DIM_ZDROJ == 1
  bool s = (digitalRead(DIM_PIN) == DIM_AKTIVNI_UROVEN);
  #else
  bool s = dimCanBit && (ted - dimCanCas < CAS_BEZ_ODPOVEDI_MS);   // rámec zmizel -> den
  #endif
  if (s != svetlaSurove) { svetlaSurove = s; svetlaZmenaCas = ted; }
  if (svetlaSurove != svetla && ted - svetlaZmenaCas >= DIM_ODSKOK_MS)
  { svetla = svetlaSurove;
    nastavJas(svetla); }
#endif
}

// ============================================================
// 6) HEARTBEAT LED (D13, Timer2, ~256 ms)
// ============================================================

#if HEARTBEAT_LED
void nastavHeartbeatTimer()
{ pinMode(13, OUTPUT);
  TCCR2A = (1 << WGM21);                              // CTC
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);   // prescaler 1024
  OCR2A = 249;                                         // ~16 ms
  TIMSK2 = (1 << OCIE2A);
}

ISR(TIMER2_COMPA_vect)
{ if (++heartbeatPocitadloPreruseni >= 16)
  { heartbeatPocitadloPreruseni = 0;
    digitalWrite(13, !digitalRead(13)); }
}
#endif

// ============================================================
// 7) ÚVODNÍ LOGO
// ============================================================

void zobrazUvodniLogo()
{ const int VYSKA_OBLASTI_LOGA = 50;
  int x = (128 - STADION_WIDTH) / 2;
  int y = (VYSKA_OBLASTI_LOGA - STADION_HEIGHT) / 2;

  u8g2.firstPage();
  do
  { u8g2.drawXBMP(x, y, STADION_WIDTH, STADION_HEIGHT, logo_stadion);
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.setCursor(0, 64);
    u8g2.print((__FlashStringHelper*)FW_VERZE); }
  while (u8g2.nextPage());

  const unsigned long krokMs = LOGO_DOBA_ZOBRAZENI_MS / (LOGO_POCET_BLIKNUTI * 2);
#if HEARTBEAT_LED
  pinMode(13, OUTPUT);   // heartbeat timer ještě neběží, pin nastavit tady
  for (byte i = 0; i < LOGO_POCET_BLIKNUTI; i++)
  { digitalWrite(13, HIGH); delay(krokMs);
    digitalWrite(13, LOW);  delay(krokMs); }
#else
  delay(LOGO_DOBA_ZOBRAZENI_MS);
#endif
}

// ============================================================
// 7B) I2C / DISPLEJ: START A OBNOVA PO RUŠENÍ
// ============================================================

// Start I2C od nuly: uvolnit piny, vyklepat zaseklou sběrnici (9 pulzů + STOP),
// nahodit Wire s timeoutem a displej.
void i2cStart()
{ Wire.end();
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(10);
  if (digitalRead(SDA) == LOW)
  { for (byte i = 0; i < 9 && digitalRead(SDA) == LOW; i++)   // max 9 hodinových pulzů
    { pinMode(SCL, OUTPUT); digitalWrite(SCL, LOW); delayMicroseconds(5);
      pinMode(SCL, INPUT_PULLUP);                   delayMicroseconds(5); }
    // STOP podmínka
    pinMode(SCL, OUTPUT); digitalWrite(SCL, LOW); delayMicroseconds(5);
    pinMode(SDA, OUTPUT); digitalWrite(SDA, LOW); delayMicroseconds(5);
    pinMode(SCL, INPUT_PULLUP);                   delayMicroseconds(5);
    pinMode(SDA, INPUT_PULLUP);                   delayMicroseconds(5);
  }

  Wire.setWireTimeout(I2C_TIMEOUT_US, true);   // při timeoutu resetni TWI hardware
  u8g2.begin();                                // Wire.begin() + init displeje (denní jas = init u8g2)
  Wire.clearWireTimeoutFlag();
}

void zkontrolujI2C()
{ if (Wire.getWireTimeoutFlag())                     // zaseklé I2C -> displej znovu nahodit
  { i2cStart();
    nastavJas(svetla); } }

// Rušení umí displeji přepsat registry (start line, offset, adresování...),
// pak ukazuje tečky / staré obrázky ze zbytku RAM, i když kreslíme správně.
// Jednou za vteřinu se znovu pošle nastavení jako při initu u8g2 pro SH1106,
// ale BEZ 0xAE (neblikne) a BEZ 0xD9/0xDB (na Laskakit SH1106 zasekávaly I2C).
// Kontrast řeší nastavJas().
const uint8_t OLED_OBNOVA_SEQ[] = {
  U8X8_START_TRANSFER(),
  U8X8_CA(0x0d5, 0x080),   // clock divide / oscilátor
  U8X8_CA(0x0a8, 0x03f),   // multiplex 1/64
  U8X8_CA(0x0d3, 0x000),   // offset
  U8X8_C(0x040),           // start line 0
  U8X8_CA(0x08d, 0x014),   // charge pump (SSD1306; SH1106 ignoruje)
  U8X8_CA(0x020, 0x000),   // adresování (SSD1306; SH1106 ignoruje)
  U8X8_C(0x0a1),           // segment remap
  U8X8_C(0x0c8),           // scan směr
  U8X8_CA(0x0da, 0x012),   // COM piny
  U8X8_C(0x02e),           // scroll vypnout
  U8X8_C(0x0a4),           // zobrazit RAM
  U8X8_C(0x0a6),           // neinvertovaně
  U8X8_C(0x0af),           // displej zapnout
  U8X8_END_TRANSFER(),
  U8X8_END()
};

void obnovDisplej()
{ u8x8_cad_SendSequence(u8g2.getU8x8(), OLED_OBNOVA_SEQ);
  nastavJas(svetla);
  zkontrolujI2C(); }

// ============================================================
// 8) SETUP
// ============================================================

void setup()
{ MCUSR = 0;
  wdt_disable();
  wdt_enable(WDTO_8S);

  i2cStart();

  // MCP2515 piny
  pinMode(CAN_CS, OUTPUT);   digitalWrite(CAN_CS, HIGH);
  pinMode(CAN_SCK, OUTPUT);  digitalWrite(CAN_SCK, LOW);
  pinMode(CAN_MOSI, OUTPUT);
  pinMode(CAN_MISO, INPUT);

#if DIM_ZDROJ == 1
  pinMode(DIM_PIN, INPUT_PULLUP);     // optočlen nemá vnější pull-up (stejně jako tacho v MultiGauge)
#endif

#if REZIM_SNIFF
  Serial.begin(115200);
#endif

  if (startMagic != START_MAGIC)     // studený start -> logo
  { startMagic = START_MAGIC;
    zobrazUvodniLogo(); }

#if HEARTBEAT_LED
  nastavHeartbeatTimer();
#endif

  mcpOk = mcpInit();
  initCas = millis();
  zacniZjistovatPodporu();
  vykresliDashboard();

  wdt_enable(WDTO_2S);   // běžný provoz: přísný watchdog
}

// ============================================================
// 9) LOOP
// ============================================================

void snifferKrok(unsigned long ted);

void loop()
{ wdt_reset();
  unsigned long ted = millis();

#if REZIM_SNIFF
  snifferKrok(ted);
  return;
#endif

  if (mcpOk)
  { byte d[13];
    for (byte n = 0; n < 6 && canPrijmi(d); n++)    // max 6 rámců na průchod, ať se to nezasekne
    { obdZpracujRamec(d);
      dimRamec(d); }
    obdKrok(ted);
  }

  if (ted - odpovedCas > CAS_BEZ_ODPOVEDI_MS)         // ticho -> zkusit MCP nahodit znovu
  { if (ted - initCas > CAS_BEZ_ODPOVEDI_MS)
    { mcpOk = mcpInit();
      initCas = ted;
      zacniZjistovatPodporu(); }                     // po zapalování znovu (mohla se probudit jiná ECU)
  }

  dimKrok(ted);

  if (ted - displejCas >= DISPLEJ_INTERVAL_MS)
  { vykresliDashboard();
    displejCas = ted; }

  if (ted - obnovaCas >= OBNOVA_DISPLEJE_MS)          // registry displeje znovu (rušení)
  { obnovDisplej();
    obnovaCas = ted; }

  zkontrolujI2C();
}

// ============================================================
// 10) DISPLEJ
// ============================================================
//
//   +---------------+---------------+
//   | BOOST         | COOLANT       |
//   |   +1.25  bar  |    88  °C     |
//   +---------------+---------------+
//   | OIL           | BATT          |
//   |    92  °C     |  14.2  V      |
//   +---------------+---------------+

// Text hodnoty do buf. Vrací false = zobrazit "N/A" (ECU to neumí).
bool textHodnoty(byte slot, char* buf, byte vel, unsigned long ted)
{ strcpy(buf, "--");
  if (slotNA(slot)) return false;
  if (!hodnotaPlatna(slot, ted)) return true;
  int v = hodnota[slot];
  switch (slot)
  { case S_VODA:
    case S_OLEJ:
      snprintf(buf, vel, "%d", v);
      break;
    case S_NAPETI:
    { int d = (v + 50) / 100;                          // mV -> desetiny V
      snprintf(buf, vel, "%d.%d", d / 10, d % 10);
      break; }
    case S_MAP:
    { int baro = hodnotaPlatna(S_BARO, ted) ? hodnota[S_BARO] : BARO_VYCHOZI_KPA * 10;
      long rozdil = (long)v - baro;                    // 0.1 kPa = 0.0001 bar
#if BOOST_DESETINY == 2
      const int dele = 10, nas = 100;                  // setiny baru
#else
      const int dele = 100, nas = 10;                  // desetiny baru
#endif
      long c = (rozdil + (rozdil >= 0 ? dele / 2 : -dele / 2)) / dele;
      char znak = c < 0 ? '-' : (c > 0 ? '+' : ' ');
      if (c < 0) c = -c;
#if BOOST_DESETINY == 2
      snprintf(buf, vel, "%c%d.%02d", znak, (int)(c / nas), (int)(c % nas));
#else
      snprintf(buf, vel, "%c%d.%d", znak, (int)(c / nas), (int)(c % nas));
#endif
      break; }
  }
  return true;
}

void vykresliDashboard()
{
#if REZIM_SNIFF
  return;   // sniffer si kreslí sám
#endif
  // 10.1) PŘÍPRAVA (mimo kreslící smyčku, ta se opakuje 4x)
  unsigned long ted = millis();
  const byte poradi[4] = { S_MAP, S_VODA, S_OLEJ, S_NAPETI };
  static const char j0[] PROGMEM = "BOOST";
  static const char j1[] PROGMEM = "COOLANT";
  static const char j2[] PROGMEM = "OIL";
  static const char j3[] PROGMEM = "BATT";
  static const char* const jmena[4] PROGMEM = { j0, j1, j2, j3 };
  static const char* const jednotky[4] = { "bar", "\260C", "\260C", "V" };   // "\260" = ° (octal)

  char txt[4][8];
  bool ok[4];
  int  xCisla[4];
  u8g2.setFont(u8g2_font_logisoso16_tn);
  for (byte i = 0; i < 4; i++)
  { ok[i] = textHodnoty(poradi[i], txt[i], sizeof(txt[i]), ted);
    xCisla[i] = 46 - u8g2.getStrWidth(txt[i]);      // zarovnáno doprava, ať neskáče
    if (xCisla[i] < 0) xCisla[i] = 0; }

  bool bezDat = !mcpOk || (ted - odpovedCas > CAS_BEZ_ODPOVEDI_MS) || odpovedCas == 0;

  // 10.2) KRESLENÍ
  u8g2.firstPage(); do {

    u8g2.drawHLine(0, 31, 128);
    u8g2.drawVLine(63, 0, 64);

    for (byte i = 0; i < 4; i++)
    { int x = (i & 1) ? 66 : 2;
      int y = (i & 2) ? 33 : 0;

      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.setCursor(x, y + 7);
      u8g2.print((__FlashStringHelper*)pgm_read_ptr(&jmena[i]));
      if (i == 0 && mapSaturace && hodnotaPlatna(S_MAP, ted)) u8g2.print(F(" MAX"));

      if (ok[i])
      { u8g2.setFont(u8g2_font_logisoso16_tn);
        u8g2.drawStr(x + xCisla[i], y + 28, txt[i]); }
      else
      { u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(x + 22, y + 26, "N/A"); }

      u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.drawStr(x + 47, y + 28, jednotky[i]);
    }

    if (svetla) u8g2.drawBox(125, 0, 3, 3);         // kontrolka "ztlumeno" (ověření polarity drátu)

    if (bezDat)                                       // diagnostika přes střed
    { u8g2.setDrawColor(0); u8g2.drawBox(26, 23, 76, 17);
      u8g2.setDrawColor(1); u8g2.drawFrame(26, 23, 76, 17);
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(37, 35);
      if (!mcpOk) u8g2.print(F("NO CANBUS"));
      else        u8g2.print(F(" NO DATA")); }
    else if (zjistujiPodporu)
    { u8g2.setFont(u8g2_font_5x7_tf);
      u8g2.drawStr(110, 7, "..."); }

  } while (u8g2.nextPage());
}

// ============================================================
// 11) SNIFFER (REZIM_SNIFF 1)
// ============================================================
//  Výpis na serial (115200), jeden řádek = změna, která se neděje "sama":
//    12345 3D1 [8] b2 0x00->0x04 (bit 2)
//  Bit-bang SPI stihne cca 500 rámců/s, Subaru jich má víc -> něco se ztratí,
//  ale periodické rámce chodí pořád dokola, takže změnu světel to stejně chytí.

#if REZIM_SNIFF
const byte SNIFF_MAX_ID = 40;
uint16_t snId[SNIFF_MAX_ID];
byte snData[SNIFF_MAX_ID][8];
byte snSum[SNIFF_MAX_ID][8];       // bity, které se měnily během učení = šum
byte snPocet = 0;
bool snPlno = false;
unsigned long snRamcu = 0, snZmen = 0, snKresleni = 0;

void snifferVypisHex(byte b)
{ if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void snifferRamec(const byte* d, unsigned long ted)
{ if (canExt(d)) return;
  uint16_t id = canId(d);
  byte dlc = d[4] & 0x0F;
  if (dlc > 8) dlc = 8;
  snRamcu++;

  byte i = 0;
  while (i < snPocet && snId[i] != id) i++;
  if (i == snPocet)
  { if (snPocet >= SNIFF_MAX_ID) { snPlno = true; return; }
    snId[i] = id;
    for (byte k = 0; k < 8; k++) { snData[i][k] = (k < dlc) ? d[5 + k] : 0; snSum[i][k] = 0; }
    snPocet++;
    Serial.print(F("NEW ")); Serial.print(id, HEX);
    Serial.print(F(" [")); Serial.print(dlc); Serial.print(F("]"));
    for (byte k = 0; k < dlc; k++) { Serial.print(' '); snifferVypisHex(d[5 + k]); }
    Serial.println();
    return; }

  bool uceni = ted < SNIFF_UCENI_MS;
  for (byte k = 0; k < dlc; k++)
  { byte nove = d[5 + k];
    byte zmena = snData[i][k] ^ nove;
    if (!zmena) continue;
    if (uceni) snSum[i][k] |= zmena;
    else if (zmena & ~snSum[i][k])
    { snZmen++;
      Serial.print(ted); Serial.print(' ');
      Serial.print(id, HEX);
      Serial.print(F(" b")); Serial.print(k); Serial.print(F(" 0x"));
      snifferVypisHex(snData[i][k]); Serial.print(F("->0x")); snifferVypisHex(nove);
      Serial.print(F(" (bit"));
      byte z = zmena & ~snSum[i][k];
      for (byte b = 0; b < 8; b++) if (z & (1 << b)) { Serial.print(' '); Serial.print(b); }
      Serial.println(')'); }
    snData[i][k] = nove;
  }
}

void snifferKrok(unsigned long ted)
{ if (!mcpOk)
  { if (ted - initCas > 1000) { mcpOk = mcpInit(); initCas = ted; } }
  else
  { byte d[13];
    for (byte n = 0; n < 8 && canPrijmi(d); n++) snifferRamec(d, ted); }

  if (ted - snKresleni >= 1000)
  { snKresleni = ted;
    u8g2.firstPage(); do {
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 10); u8g2.print(F("CAN SNIFF (serial)"));
      u8g2.setCursor(0, 24);
      if (!mcpOk) u8g2.print(F("NO CANBUS"));
      else if (ted < SNIFF_UCENI_MS) u8g2.print(F("UCENI - nesahat!"));
      else u8g2.print(F("prepinej svetla"));
      u8g2.setCursor(0, 38); u8g2.print(F("ID: ")); u8g2.print(snPocet); if (snPlno) u8g2.print(F(" PLNO"));
      u8g2.setCursor(0, 50); u8g2.print(F("ramcu: ")); u8g2.print(snRamcu);
      u8g2.setCursor(0, 62); u8g2.print(F("zmen: ")); u8g2.print(snZmen);
    } while (u8g2.nextPage()); }
}
#else
void snifferKrok(unsigned long) {}
#endif
