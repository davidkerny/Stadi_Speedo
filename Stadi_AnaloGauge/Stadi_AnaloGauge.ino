
#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "stadionlogo.h"

// ============================================================

const char FW_VERZE[] = "Stadi AnaloGauge X27 V1.4";

//  Speedo furt čte kolo na D2. = KM/H
//  Tacho navíc čte otáčky motoru na D3 = RPM (zpracovává se
//  na pozadí, i když ho teď nikde nezobrazujeme - ať je hotový
//  pro budoucí druhou ručičku).
//
//  RYCHLOST se teď ukazuje analogově přes krokový motorek
//  X27 168, co hejbe ručičkou na stupnici.
//
//  Malý OLED (128x32) ukazuje jen: km/h textem (diagnostický/
//  kontrolní údaj), ODO (v EEPROM) a TRIP (v RAM).
// ============================================================

// 1)  NASTAVENÍ
// 1B) NASTAVENÍ X27 168 - RUČIČKA SPEEDA
// 2)  STAV PROGRAMU
// 3)  MĚŘENÍ NAPÁJENÍ (Vcc)
// 4)  EEPROM - ukládání ODO
// 5)  PŘERUŠENÍ - SPEEDO
// 6)  PŘERUŠENÍ - TACHO
// 7)  FILTR SPEEDA
// 8)  FILTR TACHA
// 8B) X27 168 - RUČIČKA SPEEDA
// 8C) HEARTBEAT LED (diagnostika, D13)
// 9)  ÚVODNÍ LOGO
// 10) HLAVNÍ SMYČKA - SETUP
// 11) ZPRACOVÁNÍ SPEEDA
// 12) ZPRACOVÁNÍ TACHA
// 13) KONTROLA ZASTAVENÍ SPEEDA
// 14) KONTROLA ZASTAVENÍ TACHA
// 15) LOOP
// 16) DISPLEJ

// ============================================================
// 1) NASTAVENÍ
// ============================================================

// --- OLED init ---
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);//Klasika 0.96"
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); //Laskakit 1.3"
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); //Placatej zmrd 0.91"

// --- TESTOVACÍ NÁSOBIČ (pro finální provoz změň na 1) ---
#define TEST_MULTIPLIER 1

// --- Zapnout / Vypnout TACHO funkci ---
//#define TACHO

// --- SPEEDO snímač kola (INT0) ---
const byte SENSOR_PIN = 2;

// Obvod kola v metrech. = vzdálenost per pulz
//const float OBVOD_KOLA_M = 1.74; //16" kolo, guma terénní 16x2.75 simson
const float OBVOD_KOLA_M = 1.86; //19" kolo, guma 19x2.25 stadion

// --- SPEEDO tuning ---
const unsigned long MIN_MEZERA_PULZU_US = 45000UL; // do 120 km/h. Rychlejší pulz ignorujem.
const float MAX_ROZUMNA_RYCHLOST_KMH = 120.0;
const float MAX_SKOK_KMH = 25.0;
const float MAX_ROZDIL_SOUSEDNICH_PULZU_KMH = 8.0;
const unsigned long CAS_DO_ZASTAVENI_MS = 2000UL;
const float KALMAN_Q = 0.5;
const float KALMAN_R = 8.0;

// --- TACHO snímač motor rpm (INT1) ---
const byte TACHO_PIN = 3;

// --- TACHO tuning ---
const unsigned long MIN_MEZERA_TACHO_US = 1333UL;
const float PULZY_NA_OTACKU = 3.0;  // Tacho pulzy per otáčka DUCATI ENERGIA
const float MAX_ROZUMNE_RPM = 15000.0;
const float MIN_ROZUMNE_RPM = 500.0;
const float MAX_SKOK_RPM = 5000.0;
const float MAX_ROZDIL_SOUSEDNICH_PULZU_RPM = 1500.0;
const unsigned long CAS_DO_ZASTAVENI_TACHO_MS = 200UL; // Když 200 ms nic nepřijde, motor chcípl.
const float TACHO_KALMAN_Q = 2.0; // Vyšší Q = rychlejší reakce na vrknutí plynem.
const float TACHO_KALMAN_R = 8.0;


// --- Welcome logo ---
const unsigned long LOGO_DOBA_ZOBRAZENI_MS = 2000UL;
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
// 1B) NASTAVENÍ X27 168 - RUČIČKA SPEEDA
// ============================================================

const byte X27_A = A0;
const byte X27_B = 4;
const byte X27_C = A1;
const byte X27_D = 5;

// Kolik kroků odpovídá CELÉMU rozsahu ručičky (0 až max)
const int X27_KROKU_CELKEM = 159*4;  // *4 bo jeden krok je sekvence 4 kliků

const float X27_MAX_KMH = 103.0;     // Rozsah stupnice ciferníku.
const float X27_MIN_KMH = 4.0;       //  km/h, kdy ručička opustí doraz

// Minimální doba mezi kroky  aka rychlost pohybu
const unsigned long X27_MIN_KROK_US = 2000UL;
const unsigned long X27_HOMING_KROK_US = 1300UL;

const bool X27_OBRACENY_SMER = 1;

// ============================================================
// 2) STAV PROGRAMU
// ============================================================

// -- Ujetá vzdálenost --
unsigned long odoMetry = 0;
unsigned long tripMetry = 0;

// -- Aktuální rychlost --
unsigned int zobrazenaRychlostKmh = 0;

// -- Aktuální otáčky (počítá se dál na pozadí, i když se teď
//    nikde nezobrazuje - připraveno pro budoucí druhou ručičku) --
unsigned int zobrazeneRpm = 0;

// -- Kalman speedo --
float kalman_x = 0;
float kalman_p = 1;

// -- Pomoc speeda s rušením --
float posledniSurovaRychlostKmh = 0.0;
bool mamePredchoziPulz = false;

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

// -- Časování displeja --
unsigned long displejNaposledyCas = 0;

// -- Sdílené se speedo ISR --
volatile unsigned long speedoIsrCasPoslednihoPulzuUs = 0;
volatile unsigned long speedoIsrDelkaPoslednihoIntervaluUs = 0;
volatile bool speedoIsrMameNovyPulz = false;

// -- Sdílené s tacho ISR --
volatile unsigned long tachoIsrCasPoslednihoPulzuUs = 0;
volatile unsigned long tachoIsrDelkaPoslednihoIntervaluUs = 0;
volatile bool tachoIsrMameNovyPulz = false;

// -- X27 ručička speeda --
long x27AktualniKrok = 0;   // kde ručička skutečně je (v krocích od nuly)
long x27CilovyKrok = 0;     // kam se má dojet
unsigned long x27PoslednKrokUs = 0;

// -- Heartbeat LED (diagnostika, D13) --
volatile uint8_t heartbeatPocitadloPreruseni = 0;

// ============================================================
// 3) MĚŘENÍ NAPÁJENÍ (Vcc)
// ============================================================

#if KONTROLA_NAPETI_ZAPNUTA

long nactiNapetiVcc()
{
  // Na 328PB čteme interní bandgap vůči Vcc.
  ADCSRB &= ~(1 << 5);
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  delayMicroseconds(500);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));

  uint16_t vysledekAdc = ADC;

  const long VCC_KALIBRACNI_KONSTANTA = 1125300L;

  return VCC_KALIBRACNI_KONSTANTA / vysledekAdc;
}

#endif

// ============================================================
// 4) EEPROM - ukládání ODO
// ============================================================

void nactiOdoZEEPROM()
{
  unsigned long nejvyssiPlatnaHodnota = 0;
  int adresaSNejvyssiHodnotou = ODO_ADR_START;

  for (int adr = ODO_ADR_START; adr <= ODO_ADR_KONEC; adr += 4)
  {
    unsigned long hodnota = 0;
    EEPROM.get(adr, hodnota);

    if (hodnota == 0xFFFFFFFFUL) continue;

    if (hodnota > ODO_FYZICKY_STROP_M) continue;

    bool jePodezreleVysoka =
      nejvyssiPlatnaHodnota > 0 &&
      hodnota > nejvyssiPlatnaHodnota + ODO_MAX_SKOK_MEZI_BUNKAMI_M;

    if (jePodezreleVysoka) continue;

    if (hodnota >= nejvyssiPlatnaHodnota)
    {
      nejvyssiPlatnaHodnota = hodnota;
      adresaSNejvyssiHodnotou = adr;
    }
  }

  odoMetry = nejvyssiPlatnaHodnota;
  odoAdresaAktualni = adresaSNejvyssiHodnotou;
}

bool ulozOdoDoEEPROM(unsigned long hodnotaMetru)
{
#if KONTROLA_NAPETI_ZAPNUTA
  if (nactiNapetiVcc() < MIN_VCC_PRO_ZAPIS_MV)
  {
    return false;
  }
#endif

  odoAdresaAktualni += 4;

  if (odoAdresaAktualni > ODO_ADR_KONEC)
  {
    odoAdresaAktualni = ODO_ADR_START;
  }

  EEPROM.put(odoAdresaAktualni, hodnotaMetru);
  return true;
}

// ============================================================
// 5) PŘERUŠENÍ - SPEEDO
// ============================================================

void snimacSpeedoPreruseni()
{
  unsigned long ted = micros();

  if (speedoIsrCasPoslednihoPulzuUs == 0)
  {
    speedoIsrCasPoslednihoPulzuUs = ted;
    return;
  }

  unsigned long interval = ted - speedoIsrCasPoslednihoPulzuUs;

  // Kratší pulzy zahazujem rovnou v ISR jako rušení.
  if (interval < MIN_MEZERA_PULZU_US)
  {
    return;
  }

  speedoIsrDelkaPoslednihoIntervaluUs = interval;
  speedoIsrCasPoslednihoPulzuUs = ted;
  speedoIsrMameNovyPulz = true;
}

// ============================================================
// 6) PŘERUŠENÍ - TACHO
// ============================================================

void snimacTachoPreruseni()
{
  unsigned long ted = micros();

  if (tachoIsrCasPoslednihoPulzuUs == 0)
  {
    tachoIsrCasPoslednihoPulzuUs = ted;
    return;
  }

  unsigned long interval = ted - tachoIsrCasPoslednihoPulzuUs;

  // Bordel od zapalování - pokud je pulz moc brzo, ignorujem ho.
  if (interval < MIN_MEZERA_TACHO_US)
  {
    return;
  }

  tachoIsrDelkaPoslednihoIntervaluUs = interval;
  tachoIsrCasPoslednihoPulzuUs = ted;
  tachoIsrMameNovyPulz = true;
}

// ============================================================
// 7) FILTR SPEEDA
// ============================================================

float vyhladRychlost(float noveMereniKmh)
{
  kalman_p = kalman_p + KALMAN_Q;

  float zisk = kalman_p / (kalman_p + KALMAN_R);

  kalman_x = kalman_x + zisk * (noveMereniKmh - kalman_x);
  kalman_p = (1.0 - zisk) * kalman_p;

  return kalman_x;
}

bool jePulzSpeedaDuveryhodny(float surovaRychlostKmh)
{
  bool vysledek = true;

  if (surovaRychlostKmh > MAX_ROZUMNA_RYCHLOST_KMH)
  {
    vysledek = false;
  }

  if (vysledek && jsmeVPohybu)
  {
    float rozdilOdVyhlazene = fabs(surovaRychlostKmh - kalman_x);

    if (rozdilOdVyhlazene > MAX_SKOK_KMH)
    {
      bool souhlasiSPredchozim =
        mamePredchoziPulz &&
        fabs(surovaRychlostKmh - posledniSurovaRychlostKmh)
          < MAX_ROZDIL_SOUSEDNICH_PULZU_KMH;

      if (souhlasiSPredchozim)
      {
        // Dva pulzy za sebou si sedí -> reálné zrychlení. 
        // NEBO rychlá resynchronizace po resetu za jízdy
        kalman_x = surovaRychlostKmh;
        kalman_p = 1.0;
      }
      else
      {
        vysledek = false;
      }
    }
  }

  return vysledek;
}

// ============================================================
// 8) FILTR TACHA
// ============================================================

float vyhladRpm(float noveMereniRpm)
{
  tachoKalman_p = tachoKalman_p + TACHO_KALMAN_Q;

  float zisk =
    tachoKalman_p / (tachoKalman_p + TACHO_KALMAN_R);

  tachoKalman_x =
    tachoKalman_x +
    zisk * (noveMereniRpm - tachoKalman_x);

  tachoKalman_p =
    (1.0 - zisk) * tachoKalman_p;

  return tachoKalman_x;
}

bool jePulzTachaDuveryhodny(float suroveRpm)
{
  if (suroveRpm > MAX_ROZUMNE_RPM ||
      suroveRpm < MIN_ROZUMNE_RPM)
  {
    return false;
  }

  if (motorBezi)
  {
    float rozdil = fabs(suroveRpm - tachoKalman_x);

    if (rozdil > MAX_SKOK_RPM)
    {
      bool souhlasiSPredchozim =
        tachoMamePredchoziPulz &&
        fabs(suroveRpm - posledniSuroveRpm)
          < MAX_ROZDIL_SOUSEDNICH_PULZU_RPM;

      if (souhlasiSPredchozim)
      {
        // Dva pulzy po sobě sedí -> motor fakt změnil otáčky.
        tachoKalman_x = suroveRpm;
        tachoKalman_p = 1.0;
      }
      else
      {
        return false;
      }
    }
  }

  return true;
}

// ============================================================
// 8B) X27 168 - RUČIČKA SPEEDA
// ============================================================


// Postupným průchodem stavy 0->1->2->3 se motorek
// otáčí jedním směrem, opačným průchodem druhým směrem.
const uint8_t X27_SEKVENCE[4][4] = {
  { 1, 0, 0, 0 },
  { 0, 1, 0, 0 },
  { 0, 0, 1, 0 },
  { 0, 0, 0, 1 },
};

// Pošle na cívky napětí odpovídající danému kroku. "krok" může
// být jakékoliv celé číslo (i záporné) - vezme se jen zbytek
// po dělení 4, aby vždy vyšel platný index do tabulky.
void provedKrokX27(long krok)
{
  int stav = (int)(((krok % 4) + 4) % 4);

  if (X27_OBRACENY_SMER)
  {
    stav = 3 - stav;
  }

  digitalWrite(X27_A, X27_SEKVENCE[stav][0]);
  digitalWrite(X27_B, X27_SEKVENCE[stav][1]);
  digitalWrite(X27_C, X27_SEKVENCE[stav][2]);
  digitalWrite(X27_D, X27_SEKVENCE[stav][3]);
}


// Homing ručičky: zajede plný rozsah a zpátky na nulu.
void HomingRucicky()
{

  // Dojet na plný rozsah.
  for (long i = 0; i < X27_KROKU_CELKEM; i++)
  {
    x27AktualniKrok++;
    provedKrokX27(x27AktualniKrok);
    delayMicroseconds(X27_HOMING_KROK_US);
  }

  //chvíli počkáme, ať to neni tak uspěchaný
  delay(100);

  // A zpátky na nulu.
  for (long i = 0; i < X27_KROKU_CELKEM; i++)
  {
    x27AktualniKrok--;
    provedKrokX27(x27AktualniKrok);
    delayMicroseconds(X27_HOMING_KROK_US);
  }

  x27CilovyKrok = 0;  // ať se aktualizujRucicku() nesnaží znovu nikam jet
}

// Veřejné API - zavolej kdykoliv s požadovanou rychlostí,
// ručička se k ní sama, plynule a neblokujícím způsobem
// dopočítá (viz aktualizujRucicku(), volaná z loop()).
//
// Použití přesně podle zadání:
//   nastavRucickuKmH(0);
//   nastavRucickuKmH(50);
//   nastavRucickuKmH(100);
void nastavRucickuKmH(float hodnotaKmh)
{
  if (hodnotaKmh < 0) hodnotaKmh = 0;
  if (hodnotaKmh > X27_MAX_KMH) hodnotaKmh = X27_MAX_KMH;

  // Pod minimem = ručička na dorazu
  if (hodnotaKmh <= X27_MIN_KMH)
  {
    x27CilovyKrok = 0;
    return;
  }

  float podil =
    (hodnotaKmh - X27_MIN_KMH) / (X27_MAX_KMH - X27_MIN_KMH);

  x27CilovyKrok =
    (long)(podil * X27_KROKU_CELKEM + 0.5);
}

// Volej z loop() při KAŽDÉM průchodu - sama si pohlídá časování
// (X27_MIN_KROK_US) a udělá nanejvýš jeden krok za volání.
// Díky tomu se ručička hýbe plynule na pozadí, aniž by cokoliv
// v programu blokovala.
void aktualizujRucicku()
{
  if (x27AktualniKrok == x27CilovyKrok)
  {
    return;
  }

  unsigned long ted = micros();

  if (ted - x27PoslednKrokUs < X27_MIN_KROK_US)
  {
    return;
  }

  x27PoslednKrokUs = ted;

  if (x27AktualniKrok < x27CilovyKrok)
  {
    x27AktualniKrok++;
  }
  else
  {
    x27AktualniKrok--;
  }

  provedKrokX27(x27AktualniKrok);
}

// ============================================================
// 8C) HEARTBEAT LED (diagnostika, D13)
// ============================================================

// Timer2 v CTC módu, prescaler 1024 -> jeden "tik" cca 16 ms.
// V ISR čekáme 16 tiků (~256 ms), než LED přepneme - vznikne
// viditelné, pomalé blikání (~2 Hz). Běží nezávisle na loop().
//
// Nekoliduje s X27 ani se speedo/tacho piny (D13 je jinak
// volný) a nekoliduje ani s Timerem0 (millis/delay) ani
// s hardwarovým I2C.

void nastavHeartbeatTimer()
{
  pinMode(13, OUTPUT);

  TCCR2A = (1 << WGM21);                              // CTC mód
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);   // prescaler 1024
  OCR2A = 249;                                         // perioda ~16 ms
  TIMSK2 = (1 << OCIE2A);                              // povolit přerušení
}

ISR(TIMER2_COMPA_vect)
{
  heartbeatPocitadloPreruseni++;

  if (heartbeatPocitadloPreruseni >= 16)   // ~16 x 16 ms ≈ 256 ms
  {
    heartbeatPocitadloPreruseni = 0;
    digitalWrite(13, !digitalRead(13));
  }
}

// ============================================================
// 9) ÚVODNÍ LOGO
// ============================================================

void zobrazUvodniLogo()
{
  pinMode(LED_BUILTIN, OUTPUT);

  // Displej je teď jen 32 px vysoký - logo (16 px) dáme nahoru,
  // verzi firmwaru na spodní řádek.
  int x = (128 - STADION_WIDTH) / 2 -4;  //posunuty o -4px, aby to vycházelo do výřezu
  int y = 0;

  u8g2.clearBuffer();
  u8g2.drawXBMP(x, y, STADION_WIDTH, STADION_HEIGHT, logo_stadion);

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(0, 31);
  u8g2.print(FW_VERZE);

  u8g2.sendBuffer();

  pinMode(13, OUTPUT);

  const unsigned long krokMs =
    LOGO_DOBA_ZOBRAZENI_MS / (LOGO_POCET_BLIKNUTI * 2);

  for (byte i = 0; i < LOGO_POCET_BLIKNUTI; i++)
  {
    digitalWrite(13, HIGH);
    delay(krokMs);
    digitalWrite(13, LOW);
    delay(krokMs);
  }
}

// ============================================================
// 10) HLAVNÍ SMYČKA - SETUP
// ============================================================

void setup()
{
  // Některé bootloadery nechávaj watchdog po resetu běžet.
  wdt_disable();

  // Benevolentní watchdog už od úplného začátku setup() -
  // 8 s je hardwarový strop AVR 
  wdt_enable(WDTO_8S);

  // === I2C BUS CLEAR ===
  // Pokud vyhnilo I2C, preventivně vyčistíme sběrnici ručním bit-bang defibrilátorem.

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(10);

  bool byloZaseknute = (digitalRead(SDA) == LOW);

  if (byloZaseknute) {

    // 9 hodinových pulzů
    for (int i = 0; i < 9; i++) {
      if (digitalRead(SDA) == HIGH) break; // Displej už pustil SDA
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

  // Speedo: D2 / INT0.
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  // Tacho: D3 / INT1.
  pinMode(TACHO_PIN, INPUT_PULLUP);

  // X27 - všechny 4 piny jako výstup. A0/A1 fungujou jako
  // normální GPIO přesně stejně jako D4/D5 - nic zvláštního
  // se s nima dělat nemusí.
  pinMode(X27_A, OUTPUT);
  pinMode(X27_B, OUTPUT);
  pinMode(X27_C, OUTPUT);
  pinMode(X27_D, OUTPUT);

  zobrazUvodniLogo();

  nastavHeartbeatTimer();

  HomingRucicky();

  nactiOdoZEEPROM();
  tripMetry = 0;

  odoNaposledyUlozeno = odoMetry;

  // Přerušení pro kolo.
  attachInterrupt(
    digitalPinToInterrupt(SENSOR_PIN),
    snimacSpeedoPreruseni,
    FALLING
  );

#ifdef TACHO
  // Přerušení pro otáčky motoru.
  attachInterrupt(
    digitalPinToInterrupt(TACHO_PIN),
    snimacTachoPreruseni,
    FALLING
  );
#endif

  vykresliDashboard();

  // Přezbrojení na přísný 2s watchdog PRO BĚŽNÝ PROVOZ
  wdt_enable(WDTO_2S);
}

// ============================================================
// 11) ZPRACOVÁNÍ SPEEDA
// ============================================================

void zpracujNovySpeedoPulzPokudExistuje()
{
  if (!speedoIsrMameNovyPulz)
  {
    return;
  }

  unsigned long intervalUs;

  noInterrupts();
  intervalUs = speedoIsrDelkaPoslednihoIntervaluUs;
  speedoIsrMameNovyPulz = false;
  interrupts();

  const float MS_PER_HODINU = 3600000.0;

  float surovaRychlostKmh =
    (OBVOD_KOLA_M * MS_PER_HODINU) / intervalUs;

  bool duveryhodny = jePulzSpeedaDuveryhodny(surovaRychlostKmh);

  // Bez ohledu na výsledek si VŽDYCKY zapamatujeme surovou
  // hodnotu... i zamítnutý pulz je platný referenční bod
  // pro porovnání s tím příštím. Bez tohohle by jediné zamítnutí
  // při prudkém zrychlení nebo při resetu za jízdy mohlo přetrhnout řetězec
  // a ukazovalo by to furt stejnou rychlost až do zastavení.
  posledniSurovaRychlostKmh = surovaRychlostKmh;
  mamePredchoziPulz = true;

  if (!duveryhodny)
  {
    return;
  }

  jsmeVPohybu = true;

  float vyhlazenaKmh = vyhladRychlost(surovaRychlostKmh);
  zobrazenaRychlostKmh =
    (unsigned int)(vyhlazenaKmh + 0.5);

  // Akumulace zlomkových metrů.
  static float zbytekMetruAkumulator = 0.0;

  zbytekMetruAkumulator += OBVOD_KOLA_M;

  if (zbytekMetruAkumulator >= 1.0)
  {
    unsigned long celeMetry =
      (unsigned long)zbytekMetruAkumulator;

    odoMetry += celeMetry;
    tripMetry += celeMetry;

    zbytekMetruAkumulator -= celeMetry;
  }

  // EEPROM zápis po 100 m.
  if (odoMetry - odoNaposledyUlozeno >= KROK_ULOZENI_M)
  {
    if (ulozOdoDoEEPROM(odoMetry))
    {
      odoNaposledyUlozeno = odoMetry;
    }
  }
}

// ============================================================
// 12) ZPRACOVÁNÍ TACHA
// ============================================================

void zpracujNovyTachoPulzPokudExistuje()
{
  if (!tachoIsrMameNovyPulz)
  {
    return;
  }

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
  {
    tachoMamePredchoziPulz = false;
    return;
  }

  tachoMamePredchoziPulz = true;
  posledniSuroveRpm = suroveRpm;
  motorBezi = true;

  float vyhlazeneRpm = vyhladRpm(suroveRpm);

  zobrazeneRpm =
    (unsigned int)(vyhlazeneRpm + 0.5);
}

// ============================================================
// 13) KONTROLA ZASTAVENÍ SPEEDA
// ============================================================

void zkontrolujJestliStojimeSpeedo()
{
  unsigned long casPoslednihoPulzu;

  noInterrupts();
  casPoslednihoPulzu = speedoIsrCasPoslednihoPulzuUs;
  interrupts();

  bool jsmeUzNekdyJeli =
    (casPoslednihoPulzu != 0);

  unsigned long dobaBezPulzuUs =
    micros() - casPoslednihoPulzu;

  if (jsmeUzNekdyJeli &&
      dobaBezPulzuUs > CAS_DO_ZASTAVENI_MS * 1000UL)
  {
    kalman_x = 0;
    kalman_p = 1;
    zobrazenaRychlostKmh = 0;
    mamePredchoziPulz = false;
    jsmeVPohybu = false;
  }
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

  zpracujNovySpeedoPulzPokudExistuje();
  zpracujNovyTachoPulzPokudExistuje();

  zkontrolujJestliStojimeSpeedo();
  zkontrolujJestliStojimeTacho();

  // Ručička se posouvá nezávisle na displeji, nanejvýš jeden
  // krok za průchod smyčkou (viz X27_MIN_KROK_US uvnitř).
  aktualizujRucicku();

  unsigned long ted = millis();

  if (ted - displejNaposledyCas >
      DISPLEJ_INTERVAL_MS)
  {
    vykresliDashboard();
    displejNaposledyCas = ted;
  }
}

// ============================================================
// 16) DISPLEJ
// ============================================================

void vykresliDashboard()
{
  // Aplikace násobiče pro testovací účely - škáluje jak
  // ručičku, tak digitální readout na OLED, ať jde otestovat
  // celý řetězec i bez skutečné jízdy.
  unsigned int testRychlost = zobrazenaRychlostKmh * TEST_MULTIPLIER;

  // Ručička dostává cíl při každém překreslení displeje
  // (10x za vteřinu) - samotný pohyb je pak plynulý a
  // neblokující, řízený z loop() přes aktualizujRucicku().
  nastavRucickuKmH((float)testRychlost);

  u8g2.clearBuffer();
  //u8g2.setFont(u8g2_font_9x15B_tf);
  //u8g2.setFont(u8g2_font_10x20_tf);
  //u8g2.setFont(u8g2_font_VCR_OSD_tu);
  //u8g2.setFont(u8g2_font_courB14_tf);  //patkove
  //u8g2.setFont(u8g2_font_logisoso16_tn); //hezke ale moc vysoke
  //u8g2.setFont(u8g2_font_crox4hb_tn);  //docela tučné, výška 21, álá helvetica
  u8g2.setFont(u8g2_font_crox4h_tn);  //výška 21, álá helvetica

  
  // -- Řádek 1: ODO --
  char radekOdo[16];
  snprintf(radekOdo, sizeof(radekOdo), "%lu", odoMetry / 1000UL);
  u8g2.setCursor(0, 14);
  u8g2.print(radekOdo);

  // -- TRIP --
  float tripKm = tripMetry / 1000.0;
  char cisloTripu[8];
  dtostrf(tripKm, 4, 1, cisloTripu);
  char *zacatekCisla = cisloTripu;
  while (*zacatekCisla == ' ')
  {
    zacatekCisla++;
  }
  char radekTrip[16];
  snprintf(radekTrip, sizeof(radekTrip), "%s", zacatekCisla);
  u8g2.setCursor(85, 14);
  u8g2.print(radekTrip);

 u8g2.setFont(u8g2_font_6x10_tf);

  // -- Řádek 2: km/h (diag. údaj, běžně ukrytý pod stupnicí) --
  char radekRychlost[16];
  snprintf(radekRychlost, sizeof(radekRychlost), "%u km/h", testRychlost);
  u8g2.setCursor(0, 30);
  u8g2.print(radekRychlost);

  

  u8g2.sendBuffer();
}
