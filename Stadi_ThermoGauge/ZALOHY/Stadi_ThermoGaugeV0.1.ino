#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "stadionlogo.h"
#include "teplomer.h"

// ============================================================

const char FW_VERZE[] PROGMEM = "  Stadi ThermoGauge V0.1";

//  Žádný kola, žádný otáčky. Jen heat jak cyp.
//  TEMP1 = velké číslo + MAX v RAM (přežije reset za jízdy)
//  TEMP2, TEMP3 = malá čísla vpravo
//  Která teplota je která: viz TEMP1_ZDROJ.. v nastavení.
//
// ============================================================

// 1) NASTAVENÍ
// 2) STAV PROGRAMU
// 3) ČTENÍ TEPLOT (MAX31855, NTC, CPU)
// 4) HEARTBEAT LED (D13)
// 5) ÚVODNÍ LOGO
// 6) SETUP
// 7) LOOP
// 8) DISPLEJ

// ============================================================
// 1) NASTAVENÍ
// ============================================================

// --- OLED init (page buffer "_2_": kreslit jen v firstPage()/nextPage()) ---
//U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);//Klasika 0.96"
U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); //Laskakit 1.3"
// (layout je psaný na 128x64; placatej 128x32 by chtěl vlastní)

// --- Zdroje teplot ---
enum { SRC_TC, SRC_NTC, SRC_CPU };
const byte TEMP1_ZDROJ = SRC_TC;   // velké číslo + MAX
const byte TEMP2_ZDROJ = SRC_NTC;  // malé nahoře
const byte TEMP3_ZDROJ = SRC_CPU;  // malé dole
const char* const JMENA[] = { "TC", "NTC", "CPU" };  // popisky na displeji

// --- MAX31855 (software SPI) ---
const byte TC_SCK = 7, TC_SO = 8, TC_CS = 6;

// --- NTC 100k (Průša MK3 = ATC Semitec 104GT-2, v Einsy dělič 4k7 na Vcc) ---
const byte  NTC_PIN = A0;
const float NTC_R_PULLUP = 4700.0;
const float NTC_R0 = 100000.0;
const float NTC_T0_K = 298.15;     // 25 °C
const float NTC_BETA = 4267.0;     // doladit podle reality (3950 je typická "čínská")

// --- Teplota čipu (ATmega328PB, interní senzor) ---
const float CPU_OFFSET_ADC = 324.31;  // typicky ADC pro 0 °C
const float CPU_GAIN = 1.22;          // ADC kroků na 1 °C
const float CPU_KOREKCE_C = 0.0;      // dolaď podle teploměru

// --- Filtr (1.0 = žádný, menší = klidnější číslo) ---
const float FILTR_ALFA = 0.3;

// --- Časování ---
const unsigned long TEPLOTY_INTERVAL_MS = 250;
const unsigned long DISPLEJ_INTERVAL_MS = 100;

// --- Welcome logo ---
const unsigned long LOGO_DOBA_ZOBRAZENI_MS = 1500UL;
const byte LOGO_POCET_BLIKNUTI = 8;

// ============================================================
// 2) STAV PROGRAMU
// ============================================================

// MAX teplota + magic number leží v .noinit -> přežijí reset za jízdy.
const unsigned long MAX_MAGIC = 0xA5C3F00DUL;
const int MAX_NIC = -999;
int maxTeplota __attribute__((section(".noinit")));
unsigned long maxMagic __attribute__((section(".noinit")));

float tepl[3] = { NAN, NAN, NAN };   // indexováno SRC_xxx

unsigned long teplotyNaposledyCas = 0;
unsigned long displejNaposledyCas = 0;
volatile uint8_t heartbeatPocitadloPreruseni = 0;

// ============================================================
// 3) ČTENÍ TEPLOT
// ============================================================

// MAX31855: 32 bitů, bity 0-2 = chyby (OC/SCG/SCV), bit 16 = fault, 31..18 = teplota (0.25 °C)
float cteniThermocouple()
{ uint32_t v = 0;
  digitalWrite(TC_CS, LOW);
  delayMicroseconds(2);
  for (int8_t i = 31; i >= 0; i--)
  { digitalWrite(TC_SCK, LOW);
    delayMicroseconds(2);
    v <<= 1;
    if (digitalRead(TC_SO)) v |= 1;
    digitalWrite(TC_SCK, HIGH);
    delayMicroseconds(2);
  }
  digitalWrite(TC_CS, HIGH);
  digitalWrite(TC_SCK, LOW);

  if (v == 0 || (v & 0x10007UL)) return NAN;   // nic nepřipojeno / zkrat / utržený drát
  return ((int32_t)v >> 18) * 0.25;
}

float cteniNtc()
{ uint16_t soucet = 0;
  for (byte i = 0; i < 8; i++) soucet += analogRead(NTC_PIN);
  float adc = soucet / 8.0;
  if (adc < 1 || adc > 1022) return NAN;
  float r = NTC_R_PULLUP * adc / (1023.0 - adc);
  return 1.0 / (1.0 / NTC_T0_K + log(r / NTC_R0) / NTC_BETA) - 273.15;
}

uint16_t adcCti()
{ ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  return ADC;
}

float cteniCpu()
{ ADCSRB &= ~(1 << 5);
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);   // kanál 8 = teplotní senzor, ref 1.1 V
  delay(2);
  adcCti();                                       // první měření zahodit
  uint16_t soucet = 0;
  for (byte i = 0; i < 4; i++) soucet += adcCti();
  return (soucet / 4.0 - CPU_OFFSET_ADC) / CPU_GAIN + CPU_KOREKCE_C;
}

// Exponenciální filtr; NAN projde rovnou (chyba se má ukázat hned).
void ulozTeplotu(byte zdroj, float nove)
{ float &s = tepl[zdroj];
  if (isnan(nove))      s = NAN;
  else if (isnan(s))    s = nove;
  else                  s += FILTR_ALFA * (nove - s);
}

void cteniVsechTeplot()
{ ulozTeplotu(SRC_TC,  cteniThermocouple());
  ulozTeplotu(SRC_NTC, cteniNtc());
  ulozTeplotu(SRC_CPU, cteniCpu());

  float t1 = tepl[TEMP1_ZDROJ];
  if (!isnan(t1) && (int)floor(t1 + 0.5) > maxTeplota)
  { maxTeplota = (int)floor(t1 + 0.5); }
}

// ============================================================
// 4) HEARTBEAT LED (D13, Timer2, ~256 ms)
// ============================================================

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

// ============================================================
// 5) ÚVODNÍ LOGO
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
  for (byte i = 0; i < LOGO_POCET_BLIKNUTI; i++)
  { digitalWrite(13, HIGH); delay(krokMs);
    digitalWrite(13, LOW);  delay(krokMs); }
}

// ============================================================
// 6) SETUP
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

  // MAX31855
  pinMode(TC_CS, OUTPUT);  digitalWrite(TC_CS, HIGH);
  pinMode(TC_SCK, OUTPUT); digitalWrite(TC_SCK, LOW);
  pinMode(TC_SO, INPUT);

  // Magic number nesedí = studený start. Sedí = reset za jízdy, MAX zůstává.
  if (maxMagic != MAX_MAGIC)
  { maxTeplota = MAX_NIC;
    maxMagic = MAX_MAGIC;
    zobrazUvodniLogo();
  }

  nastavHeartbeatTimer();
  cteniVsechTeplot();
  vykresliDashboard();

  wdt_enable(WDTO_2S);   // běžný provoz: přísný watchdog
}

// ============================================================
// 7) LOOP
// ============================================================

void loop()
{ wdt_reset();
  unsigned long ted = millis();

  if (ted - teplotyNaposledyCas >= TEPLOTY_INTERVAL_MS)
  { cteniVsechTeplot();
    teplotyNaposledyCas = ted; }

  if (ted - displejNaposledyCas >= DISPLEJ_INTERVAL_MS)
  { vykresliDashboard();
    displejNaposledyCas = ted; }
}

// ============================================================
// 8) DISPLEJ
// ============================================================

// Ikonka MAX 5x7 (šipka nahoru)
const uint8_t IKONA_TOP[] PROGMEM = { 0xff, 0xe4, 0xee, 0xff, 0xe4, 0xe4, 0xe4 };

// Teplota -> text, nebo "nahradni" při chybě čidla.
void formatujTeplotu(char *buf, float t, const char *nahradni)
{ if (isnan(t)) strcpy(buf, nahradni);
  else snprintf(buf, 6, "%d", (int)floor(t + 0.5)); }

void vykresliDashboard()
{
  // --------------------------------------------------------
  // 8.1) PŘÍPRAVA (mimo kreslící smyčku, ta se opakuje 4x)
  // --------------------------------------------------------
  char t1[6], t2[6], t3[6];
  formatujTeplotu(t1, tepl[TEMP1_ZDROJ], "---");
  formatujTeplotu(t2, tepl[TEMP2_ZDROJ], "--");
  formatujTeplotu(t3, tepl[TEMP3_ZDROJ], "--");

  // Velké číslo zarovnané doprava (konec na x=94), ať neskáče.
  u8g2.setFont(u8g2_font_logisoso42_tn);
  int xVelke = 94 - u8g2.getStrWidth(t1);
  if (xVelke < 15) xVelke = 15;

  // --------------------------------------------------------
  // 8.2) KRESLENÍ
  // --------------------------------------------------------
  u8g2.firstPage(); do {

    // Teploměr vlevo
    u8g2.drawXBMP(0, 2, TEPLOMER_WIDTH, TEPLOMER_HEIGHT, ikona_teplomer);

    // TEMP1 - velké
    u8g2.setFont(u8g2_font_logisoso42_tn);
    u8g2.setCursor(xVelke, 45);
    u8g2.print(t1);

    // TEMP2 a TEMP3 - malé, vpravo
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(98, 9);  u8g2.print(JMENA[TEMP2_ZDROJ]);
    u8g2.setCursor(98, 33); u8g2.print(JMENA[TEMP3_ZDROJ]);
    u8g2.setFont(u8g2_font_7x13B_tn);
    u8g2.setCursor(98, 23); u8g2.print(t2);
    u8g2.setCursor(98, 47); u8g2.print(t3);

    // MAX dole (šipka + číslo) + jméno zdroje TEMP1 vpravo
    u8g2.setFont(u8g2_font_6x10_tf);
    if (maxTeplota > MAX_NIC)
    { u8g2.drawXBMP(0, 57, 5, 7, IKONA_TOP);
      u8g2.setCursor(9, 64);
      u8g2.print(maxTeplota); }
    u8g2.setCursor(128 - 6 * strlen(JMENA[TEMP1_ZDROJ]), 64);
    u8g2.print(JMENA[TEMP1_ZDROJ]);

  } while (u8g2.nextPage());
}
