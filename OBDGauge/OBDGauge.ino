#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "stadionlogo.h"

// ============================================================

const char FW_VERZE[] PROGMEM = "  OBDGauge V0.1";

//  Jeden parametr z OBD-II přes CAN (500 kbps, 11bit, ISO 15765-4).
//  Pošle dotaz 0x7DF "mode 01, PID xx", ECU odpoví na 0x7E8..0x7EF.
//  MCP2515 se řeší bit-bangem na libovolných pinech, žádná knihovna.
//  CAN_CS=A0, CAN_SCK=A1, CAN_MISO=4, CAN_MOSI=5
//  
//  Pro auta s CANBusem. E46 by potřebovala K-line tranceiver, třeba L9637D.
//
// ============================================================

// 1) NASTAVENÍ
// 2) STAV PROGRAMU
// 3) MCP2515 (SPI, init)
// 4) OBD DOTAZ A ODPOVĚĎ
// 5) HEARTBEAT LED (D13)
// 6) ÚVODNÍ LOGO
// 7) SETUP
// 8) LOOP
// 9) DISPLEJ

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

// --- Co čteme (OBD mode 01, jednobajtové PIDy: hodnota = A + OFFSET) ---
//  0x05 chladící kapalina  A-40      0x0F teplota sání   A-40
//  0x5C olej (ne všude)    A-40      0x0D rychlost km/h  A+0
const byte OBD_PID = 0x05;
const int  OBD_OFFSET = -40;
const char OBD_JMENO[] PROGMEM = "COOLANT";

// --- Časování ---
const unsigned long OBD_DOTAZ_INTERVAL_MS = 500;
const unsigned long CAS_BEZ_ODPOVEDI_MS = 3000;   // pak hodnota "---" a pokus o nový start MCP
const unsigned long DISPLEJ_INTERVAL_MS = 100;

// --- Welcome logo ---
const unsigned long LOGO_DOBA_ZOBRAZENI_MS = 1500UL;
const byte LOGO_POCET_BLIKNUTI = 8;

// ============================================================
// 2) STAV PROGRAMU
// ============================================================

// Magic number v .noinit: logo jen při studeném startu, ne po resetu watchdogem.
const unsigned long START_MAGIC = 0xA5C3F00DUL;
unsigned long startMagic __attribute__((section(".noinit")));

int  hodnota = 0;
bool mamData = false;      // přišla platná odpověď v posledních CAS_BEZ_ODPOVEDI_MS
bool mcpOk = false;        // MCP2515 odpovídá a je v normálním módu

unsigned long dotazCas = 0, odpovedCas = 0, initCas = 0, displejCas = 0;
volatile uint8_t heartbeatPocitadloPreruseni = 0;

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

void mcpPis2(byte reg, byte a, byte b)   // dva po sobě jdoucí registry (SIDH, SIDL)
{ digitalWrite(CAN_CS, LOW);
  spiByte(0x02); spiByte(reg); spiByte(a); spiByte(b);
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

  // Hardwarový filtr: projdou jen ID 0x7E8..0x7EF (maska 0x7F8). Zbytek provozu na sběrnici
  // MCP zahodí sám, takže ani Subaru se spoustou ramců nepřeteče buffer, když kreslíme displej.
  const byte filtry[] = { 0x00, 0x04, 0x08, 0x10, 0x14, 0x18 };   // RXF0..RXF5
  for (byte i = 0; i < 6; i++) mcpPis2(filtry[i], 0xFD, 0x00);     // 0x7E8
  mcpPis2(0x20, 0xFF, 0x00);                                        // RXM0
  mcpPis2(0x24, 0xFF, 0x00);                                        // RXM1
  mcpPis(0x60, 0x04);    // RXB0CTRL: filtry zapnuté, při plném přetéct do RXB1
  mcpPis(0x70, 0x00);    // RXB1CTRL: filtry zapnuté
  mcpPis(0x2B, 0x00);    // CANINTE: bez přerušení, pollujem

  mcpPis(0x0F, 0x08);    // CANCTRL: normální mód + one-shot (bez nekonečného opakování)
  for (byte i = 0; i < 50; i++)                      // přepnutí čeká na klid na sběrnici
  { if ((mcpCti(0x0E) & 0xE0) == 0x00) return true;
    delay(1); }
  return false;
}

// ============================================================
// 4) OBD DOTAZ A ODPOVĚĎ
// ============================================================

void obdDotaz()
{ digitalWrite(CAN_CS, LOW);
  spiByte(0x40);                                   // LOAD TX BUFFER 0, od SIDH
  spiByte(0x7DF >> 3); spiByte((0x7DF & 7) << 5);  // ID 0x7DF (funkční dotaz na všechny ECU)
  spiByte(0); spiByte(0);                          // EID (nepoužito)
  spiByte(8);                                      // DLC
  spiByte(0x02); spiByte(0x01); spiByte(OBD_PID);  // 2 bajty: mode 01, PID
  for (byte i = 0; i < 5; i++) spiByte(0);         // padding
  digitalWrite(CAN_CS, HIGH);
  digitalWrite(CAN_CS, LOW); spiByte(0x81); digitalWrite(CAN_CS, HIGH);   // RTS TXB0
}

void obdZpracujOdpoved()
{ for (byte n = 0; n < 4; n++)                     // max 4 ramce na průchod, ať se to nezasekne
  { byte flagy = mcpCti(0x2C);                     // CANINTF
    byte cmd;
    if (flagy & 0x01)      cmd = 0x90;             // RXB0
    else if (flagy & 0x02) cmd = 0x94;             // RXB1
    else return;

    byte d[13];                                    // SIDH SIDL EID8 EID0 DLC D0..D7
    digitalWrite(CAN_CS, LOW);
    spiByte(cmd);
    for (byte i = 0; i < 13; i++) d[i] = spiByte(0);
    digitalWrite(CAN_CS, HIGH);                    // tímhle se příznak sám smaže

    // Odpověď: [PCI 0x03] [0x41] [PID] [A] ...
    if ((d[5] & 0xF0) == 0 && d[6] == 0x41 && d[7] == OBD_PID)
    { hodnota = (int)d[8] + OBD_OFFSET;
      mamData = true;
      odpovedCas = millis(); }
  }
}

// ============================================================
// 5) HEARTBEAT LED (D13, Timer2, ~256 ms)
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
// 6) ÚVODNÍ LOGO
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
// 7) SETUP
// ============================================================

void setup()
{ MCUSR = 0;
  wdt_disable();
  wdt_enable(WDTO_8S);

  // === I2C BUS CLEAR (zvon na hajzl pro zaseknutej displej) ===
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

  u8g2.begin();

  // MCP2515 piny
  pinMode(CAN_CS, OUTPUT);   digitalWrite(CAN_CS, HIGH);
  pinMode(CAN_SCK, OUTPUT);  digitalWrite(CAN_SCK, LOW);
  pinMode(CAN_MOSI, OUTPUT);
  pinMode(CAN_MISO, INPUT);

  if (startMagic != START_MAGIC)     // studený start -> logo
  { startMagic = START_MAGIC;
    zobrazUvodniLogo(); }

#if HEARTBEAT_LED
  nastavHeartbeatTimer();
#endif

  mcpOk = mcpInit();
  initCas = millis();
  vykresliDashboard();

  wdt_enable(WDTO_2S);   // běžný provoz: přísný watchdog
}

// ============================================================
// 8) LOOP
// ============================================================

void loop()
{ wdt_reset();
  unsigned long ted = millis();

  if (mcpOk)
  { obdZpracujOdpoved();
    if (ted - dotazCas >= OBD_DOTAZ_INTERVAL_MS)
    { obdDotaz();
      dotazCas = ted; }
  }

  if (ted - odpovedCas > CAS_BEZ_ODPOVEDI_MS)         // ticho -> data neplatná
  { mamData = false;
    if (ted - initCas > CAS_BEZ_ODPOVEDI_MS)          // a zkusit MCP nahodit znovu
    { mcpOk = mcpInit();
      initCas = ted; }
  }

  if (ted - displejCas >= DISPLEJ_INTERVAL_MS)
  { vykresliDashboard();
    displejCas = ted; }
}

// ============================================================
// 9) DISPLEJ
// ============================================================

void vykresliDashboard()
{
  // 9.1) PŘÍPRAVA (mimo kreslící smyčku, ta se opakuje 4x)
  char t[6] = "---";
  if (mamData) snprintf(t, sizeof(t), "%d", hodnota);

  u8g2.setFont(u8g2_font_logisoso42_tn);
  int x = 100 - u8g2.getStrWidth(t);     // zarovnáno doprava, ať neskáče
  if (x < 0) x = 0;

  // 9.2) KRESLENÍ
  u8g2.firstPage(); do {

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 9);
    u8g2.print((__FlashStringHelper*)OBD_JMENO);

    u8g2.setFont(u8g2_font_logisoso42_tn);
    u8g2.setCursor(x, 54);
    u8g2.print(t);

    u8g2.setFont(u8g2_font_10x20_tf);      // "\260C" = °C (octal; "\xb0C" by sežralo 'C')
    u8g2.setCursor(103, 54);
    u8g2.print(F("\260C"));

    // Diagnostika dole: rozlišíme "MCP2515 nezapojenej" od "auto neodpovídá"
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 64);
    if (!mcpOk)        u8g2.print(F("NO CANBUS"));
    else if (!mamData) u8g2.print(F("NO DATA"));

  } while (u8g2.nextPage());
}
