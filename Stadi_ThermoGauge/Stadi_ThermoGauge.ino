#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "stadionlogo.h"
#include "teplomer.h"

// ============================================================

const char FW_VERZE[] PROGMEM = " Stadi ThermoGauge V0.32";

//  Žádný kola, žádný otáčky. Jen horko.
//  TEMP1 = velké číslo + MAX v RAM (přežije reset za jízdy)
//  TEMP2, TEMP3 = malá čísla vpravo
//  Která teplota je která: TEMP1_ZDROJ.. v nastavení.
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
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);//Klasika 0.96"
//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); //Laskakit 1.3"
// (layout je psaný na 128x64; placatej 128x32 by chtěl vlastní)

// --- Zdroje teplot ---
enum { SRC_TC, SRC_NTC, SRC_CPU, SRC_COUNT };
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
const float CPU_KOREKCE_C = -.0;      // dolaď podle teploměru

// --- Filtr (1.0 = žádný, menší = klidnější číslo) a tolerance chyb čidla ---
const float FILTR_ALFA = 0.25;
const byte  CHYB_TOLERANCE = 3;       // tolik vadných čtení po sobě = ERR

// --- Časování ---
const unsigned long TEPLOTY_INTERVAL_MS = 200;
const unsigned long DISPLEJ_INTERVAL_MS = 100;
const unsigned long BURN_IN_SHIFT_MS = 30000UL;  // pixel shift OLED o 1 px

// --- Layout (ladit podle displeje) ---
const int VELKE_PRAVY_OKRAJ = 92;     // kde končí velké číslo
const int SLOUPEC_X = 95;             // začátek pravého sloupce

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

float tepl[SRC_COUNT] = { NAN, NAN, NAN };   // indexováno SRC_xxx
byte chybPocet[SRC_COUNT] = { 0, 0, 0 };

byte posun = 0;                              // burn-in shift 0/1 px
unsigned long teplotyNaposledyCas = 0;
unsigned long displejNaposledyCas = 0;
unsigned long posunNaposledyCas = 0;
volatile uint8_t heartbeatPocitadloPreruseni = 0;

// Prototyp pro kompilátor
void vykresliDashboard();

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

  if (v == 0 || (v & 0x10007UL)) return NAN;    // nic nepřipojeno / zkrat / utržený drát
  return ((int32_t)v >> 18) * 0.25;            // aritmetický shift = záporné °C fungujou
}

float cteniNtc()
{
  // Zahodit první čtení - přepíná VREF z 1.1V zpět na VCC a ustálí MUX pro A0
  analogRead(NTC_PIN);
  delayMicroseconds(200);

  uint16_t soucet = 0;
  for (byte i = 0; i < 8; i++) soucet += analogRead(NTC_PIN);
  float adc = soucet / 8.0;
  if (adc < 2 || adc > 1021) return NAN;
  float r = NTC_R_PULLUP * adc / (1023.0 - adc);
  return 1.0 / (1.0 / NTC_T0_K + log(r / NTC_R0) / NTC_BETA) - 273.15;
}
uint16_t adcCti()
{ ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  return ADC;}

float cteniCpu()
{
  // 1. Kompletně vypnout ADC, aby se vyprázdnily interní registry
  ADCSRA &= ~_BV(ADEN);

  // 2. Nastavit MUX5 (ADC8) a interní 1.1V referenci
  ADCSRB = (ADCSRB & ~(1 << 5)) | (1 << 5);
  ADMUX = _BV(REFS1) | _BV(REFS0);

  // 3. Znovu zapnout ADC
  ADCSRA |= _BV(ADEN);

  // 4. KLÍČOVÝ KROK: Dát vnitřnímu Sample-and-Hold kondenzátoru (14pF) 
  // čas se vybít přes vysokoimplikační teplotní diodu (2 ms)
  delay(2);

  // 5. Udělat 8 "prázdných" převodů jen proto, aby se náboj vybije do převodníku
  for (byte i = 0; i < 8; i++) {
    adcCti();
  }

  // 6. Teprve teď změřit reálnou hodnotu (průměr z 16 čtení)
  uint32_t soucet = 0;
  for (byte i = 0; i < 16; i++) {
    soucet += adcCti();
  }

  // 7. Vypnout ADC a vrátit MUX5 do nuly před odchodem
  ADCSRA &= ~_BV(ADEN);
  ADCSRB &= ~(1 << 5);
  ADCSRA |= _BV(ADEN);

  float rawAdc = soucet / 16.0;

  // Pokud je rawAdc mimo fyzikální meze senzoru, vrátíme NAN
  if (rawAdc < 250.0 || rawAdc > 450.0) return NAN;

  return (rawAdc - CPU_OFFSET_ADC) / CPU_GAIN + CPU_KOREKCE_C;
}





// Exponenciální filtr. Jedno dvě vadná čtení po sobě přežijem, pak ERR.
void ulozTeplotu(byte zdroj, float nove)
{ float &s = tepl[zdroj];
  if (isnan(nove))
  { if (chybPocet[zdroj] < CHYB_TOLERANCE) chybPocet[zdroj]++;
    if (chybPocet[zdroj] >= CHYB_TOLERANCE) s = NAN;
    return; }
  chybPocet[zdroj] = 0;
  s = isnan(s) ? nove : s + FILTR_ALFA * (nove - s);
}

void cteniVsechTeplot()
{ 
  ulozTeplotu(SRC_TC,  cteniThermocouple());
  ulozTeplotu(SRC_NTC, cteniNtc());
  
  // CPU měříme PŘÍMO bez exponenciálního filtru, který driftoval:
  float cpuRaw = cteniCpu();
  if (!isnan(cpuRaw)) {
    tepl[SRC_CPU] = cpuRaw;
  } else {
    tepl[SRC_CPU] = NAN;
  }

  float t1 = tepl[TEMP1_ZDROJ];
  if (!isnan(t1))
  { int t = (int)floor(t1 + 0.5);
    if (t > maxTeplota) maxTeplota = t; }
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
{ pinMode(13, OUTPUT);   // heartbeat timer ještě neběží, pin je třeba nastavit tady

  const int VYSKA_OBLASTI_LOGA = 50;
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

  if (ted - posunNaposledyCas >= BURN_IN_SHIFT_MS)
  { posun = !posun;
    posunNaposledyCas = ted; }

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
void formatujTeplotu(char *buf, size_t size, float t, const char *nahradni)
{ if (isnan(t)) strncpy(buf, nahradni, size);
  else snprintf(buf, size, "%d", (int)floor(t + 0.5)); }

// Malá teplota v pravém sloupci: popisek (5x7) + hodnota (6x10) + °C.
// POZOR: "\260C" (octal), ne "\xb0C" - 'C' je hex číslice a sežralo by se to do escape.
void kresliMalou(int y, byte zdroj)
{ char t[6];
  formatujTeplotu(t, sizeof(t), tepl[zdroj], "--");
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(SLOUPEC_X + posun, y + posun);
  u8g2.print(JMENA[zdroj]);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(SLOUPEC_X + posun, y + 11 + posun);
  u8g2.print(t);
  if (!isnan(tepl[zdroj])) u8g2.print(F("\260C"));
}

void vykresliDashboard()
{
  // --------------------------------------------------------
  // 8.1) PŘÍPRAVA (mimo kreslící smyčku, ta se opakuje 4x)
  // --------------------------------------------------------
  bool chyba = isnan(tepl[TEMP1_ZDROJ]);
  char t1[6];
  formatujTeplotu(t1, sizeof(t1), tepl[TEMP1_ZDROJ], "ERR");

  // Velké číslo zarovnané doprava, ať neskáče.
  int xVelke = 16 + posun;
  if (!chyba)
  { u8g2.setFont(u8g2_font_logisoso42_tn);
    xVelke = VELKE_PRAVY_OKRAJ - u8g2.getStrWidth(t1) + posun;
    if (xVelke < 16) xVelke = 16; }

  // --------------------------------------------------------
  // 8.2) KRESLENÍ
  // --------------------------------------------------------
  u8g2.firstPage(); do {

    // Teploměr ikona vlevo
    u8g2.drawXBMP(posun +3, posun +10, TEPLOMER_WIDTH, TEPLOMER_HEIGHT, ikona_teplomer);

    // TEMP1 - velké (ERR je menší a s písmeny)
    if (chyba)
    { u8g2.setFont(u8g2_font_logisoso32_tr);
      u8g2.setCursor(22 + posun, 42 + posun); }
    else
    { u8g2.setFont(u8g2_font_logisoso42_tn);
      u8g2.setCursor(xVelke, 44 + posun); }
    u8g2.print(t1);

    // TEMP2 a TEMP3 - malé, vpravo
    kresliMalou(8,  TEMP2_ZDROJ);
    kresliMalou(31, TEMP3_ZDROJ);

    // MAX dole (šipka + číslo) + jméno zdroje TEMP1 vpravo
    u8g2.setFont(u8g2_font_6x10_tf);
    if (maxTeplota > MAX_NIC)
    { u8g2.drawXBMP(posun, 55 + posun, 5, 7, IKONA_TOP);
      u8g2.setCursor(9 + posun, 62 + posun);
      u8g2.print(maxTeplota);
      u8g2.print(F("\260C")); }
    u8g2.setCursor(127 - u8g2.getStrWidth(JMENA[TEMP1_ZDROJ]) + posun, 62 + posun);
    u8g2.print(JMENA[TEMP1_ZDROJ]);

  } while (u8g2.nextPage());
}
