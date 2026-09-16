#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include <EEPROM.h>
#include <avr/wdt.h>
#include "stadionlogo.h"

// ============================================================

const char FW_VERZE[] = "Stadi AnaloGauge X27 V1.0 ";

//  Speedo furt čte kolo na D2. = KM/H
//  Tacho navíc čte otáčky motoru na D3 = RPM (zpracovává se
//  na pozadí, i když ho teď nikde nezobrazujeme - ať je hotový
//  pro budoucí druhou ručičku).
//
//  RYCHLOST se teď ukazuje analogově přes krokový motorek
//  X27 589 (jehla tachometru z auta/mopedu), ne digitálně.
//
//  Malý OLED (128x32) ukazuje jen: km/h textem (záložní/
//  kontrolní údaj), ODO (v EEPROM) a TRIP (v RAM).
// ============================================================

// 1)  NASTAVENÍ
// 2)  STAV PROGRAMU
// 3)  MĚŘENÍ NAPÁJENÍ (Vcc)
// 4)  EEPROM - ukládání ODO
// 5)  PŘERUŠENÍ - SPEEDO
// 6)  PŘERUŠENÍ - TACHO
// 7)  FILTR SPEEDA
// 8)  FILTR TACHA
// 9)  X27 589 - RUČIČKA SPEEDA
// 10) ÚVODNÍ LOGO
// 11) HLAVNÍ SMYČKA - SETUP
// 12) ZPRACOVÁNÍ SPEEDA
// 13) ZPRACOVÁNÍ TACHA
// 14) KONTROLA ZASTAVENÍ SPEEDA
// 15) KONTROLA ZASTAVENÍ TACHA
// 16) LOOP
// 17) DISPLEJ

// ============================================================
// 1) NASTAVENÍ
// ============================================================


//Init OLED klasika 0.96"
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
//Init OLED Laskakit 1.3"
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
//Init OLED placatej zmrd 0.91" - TENHLE JE TEĎ AKTIVNÍ
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);


// --- Speedo snímač kola (INT0) ---
const byte SENSOR_PIN = 2;

// Obvod kola v metrech. = vzdálenost per pulz
const float OBVOD_KOLA_M = 1.77;

// --- Ochrana speeda proti rušení ---

// Moped jede do 120 km/h. Rychlejší pulz ignorujem.
const unsigned long MIN_MEZERA_PULZU_US = 45000UL;

const float MAX_ROZUMNA_RYCHLOST_KMH = 120.0;
const float MAX_SKOK_KMH = 25.0;
const float MAX_ROZDIL_SOUSEDNICH_PULZU_KMH = 8.0;

const unsigned long CAS_DO_ZASTAVENI_MS = 2000UL;

// --- Tacho snímač motor rpm (INT1) ---
const byte TACHO_PIN = 3;

// Tady berem 1 přijatý pulz = 1 otáčka motoru.
// Pulzy kratší než 2.8 ms zahazujem jako bordel od zapalování.
const unsigned long MIN_MEZERA_TACHO_US = 2800UL;

const float MAX_ROZUMNE_RPM = 18000.0;
const float MIN_ROZUMNE_RPM = 500.0;
const float MAX_SKOK_RPM = 5000.0;
const float MAX_ROZDIL_SOUSEDNICH_PULZU_RPM = 1500.0;

// Když 200 ms nic nepřijde, motor chcípl.
const unsigned long CAS_DO_ZASTAVENI_TACHO_MS = 200UL;

// --- Welcome logo ---
const unsigned long LOGO_DOBA_ZOBRAZENI_MS = 2000UL;
const byte LOGO_POCET_BLIKNUTI = 4;

// --- Kalman speedo ---
const float KALMAN_Q = 0.5;
const float KALMAN_R = 8.0;

// --- Kalman tacho ---
// Vyšší Q = rychlejší reakce na vrknutí plynem.
const float TACHO_KALMAN_Q = 2.0;
const float TACHO_KALMAN_R = 8.0;

// --- EEPROM / ODO ---
const int ODO_ADR_START = 0;
const int ODO_ADR_KONEC = 252;
const unsigned long KROK_ULOZENI_M = 100UL;

const unsigned long ODO_FYZICKY_STROP_M = 100000000UL;
const unsigned long ODO_MAX_SKOK_MEZI_BUNKAMI_M = 1000000UL;

// --- Ochrana EEPROM při nízkým napětí ---
#define KONTROLA_NAPETI_ZAPNUTA 1
const long MIN_VCC_PRO_ZAPIS_MV = 3400L;

// --- Displej ---
const unsigned long DISPLEJ_INTERVAL_MS = 100;

// ============================================================
// X27 589 - RUČIČKA SPEEDA
// ============================================================
//
// X27 je 2fázový krokový motorek se 4 vývody (žádný společný
// vodič) - každý pin jede přímo na jeden konec jedné cívky.
// Piny A0/A1 = cívka 1, D4/D5 = cívka 2. Řídicí sekvence níž
// (X27_SEKVENCE) je standardní 8stavová "půlkroková" tabulka
// používaná i v knihovnách jako SwitecX25.
//
// DŮLEŽITÁ VLASTNOST X27: má vnitřní vratnou pružinu - bez
// napájení se ručička sama vrátí na nulu. Díky tomu je homing
// (najetí na nulu při startu) bezpečný i "na tvrdo": stačí
// zajet o víc kroků, než je celý rozsah, motorek narazí na
// mechanický doraz a jen tam neškodně prokluzuje.

#define X27_A  A0
#define X27_B  A1
#define X27_C  4
#define X27_D  5

// Kolik kroků odpovídá CELÉMU rozsahu ručičky (0 až
// X27_MAX_KMH). 315 je běžná hodnota pro X27.168/589, ale
// KAŽDÝ konkrétní kus si vždycky zkalibruj - zkus poslat pár
// desítek/stovek kroků a podle toho, kde skutečně skončí
// ručička na stupnici, tuhle konstantu doladit.
const int X27_KROKU_CELKEM = 315;

// Kolik kroků navíc "natvrdo" zajet do dorazu při homingu -
// rezerva, ať i při nepřesné kalibraci výše jistě dorazíme
// na skutečnou nulu.
const int X27_HOMING_REZERVA_KROKU = 20;

// Rozsah stupnice ciferníku.
const float X27_MAX_KMH = 100.0;

// Minimální doba mezi jednotlivými kroky za normálního běhu.
// Moc rychle = motorek přeskakuje kroky / bzučí / neposlouchá.
// Moc pomalu = ručička viditelně "leze" místo plynulého pohybu.
// Když ručička cuká nebo vibruje místo hladkého pohybu, ZVYŠ
// tohle číslo.
const unsigned long X27_MIN_KROK_US = 1200UL;

// Pokud po zapojení jede ručička obráceně (k 100 misto k 0),
// přepni na true - je to jednodušší než přepojovat kabely.
const bool X27_OBRACENY_SMER = false;

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
bool jsmeVPohybu = false;

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
// 9) X27 589 - RUČIČKA SPEEDA
// ============================================================

// Standardní 8stavová řídicí sekvence pro 2fázový krokový
// motorek typu X27 (pořadí sloupců: X27_A, X27_B, X27_C, X27_D).
// Postupným průchodem stavy 0->1->2->...->7->0 se motorek
// otáčí jedním směrem, opačným průchodem druhým směrem.
const uint8_t X27_SEKVENCE[8][4] = {
  { 1, 0, 1, 0 },
  { 0, 0, 1, 0 },
  { 0, 1, 1, 0 },
  { 0, 1, 0, 0 },
  { 0, 1, 0, 1 },
  { 0, 0, 0, 1 },
  { 1, 0, 0, 1 },
  { 1, 0, 0, 0 },
};

// Pošle na cívky napětí odpovídající danému kroku. "krok" může
// být jakékoliv celé číslo (i záporné) - vezme se jen zbytek
// po dělení 8, aby vždy vyšel platný index do tabulky.
void provedKrokX27(long krok)
{
  int stav = (int)(((krok % 8) + 8) % 8);

  if (X27_OBRACENY_SMER)
  {
    stav = 7 - stav;
  }

  digitalWrite(X27_A, X27_SEKVENCE[stav][0]);
  digitalWrite(X27_B, X27_SEKVENCE[stav][1]);
  digitalWrite(X27_C, X27_SEKVENCE[stav][2]);
  digitalWrite(X27_D, X27_SEKVENCE[stav][3]);
}

// Homing - najetí na skutečnou nulu při startu. Provádí se
// natvrdo (blokující delay), ale jen jednou v setup(), ještě
// před zapnutím watchdogu, takže to nevadí.
//
// X27 má vnitřní vratnou pružinu, takže "přejetí" do
// mechanického dorazu je bezpečné - motorek tam jen neškodně
// prokluzuje. Proto zajedeme o dost víc kroků, než je celý
// rozsah, a pak si prostě řekneme, že jsme na nule.
void hometniRucicku()
{
  const unsigned long HOMING_KROK_US = 3000UL;  // pomalu a jistě, je to jen jednou

  long krokuKHomingu = X27_KROKU_CELKEM + X27_HOMING_REZERVA_KROKU;

  for (long i = 0; i < krokuKHomingu; i++)
  {
    x27AktualniKrok--;
    provedKrokX27(x27AktualniKrok);
    delayMicroseconds(HOMING_KROK_US);
  }

  x27AktualniKrok = 0;
  x27CilovyKrok = 0;
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
  if (hodnotaKmh < 0)
  {
    hodnotaKmh = 0;
  }
  if (hodnotaKmh > X27_MAX_KMH)
  {
    hodnotaKmh = X27_MAX_KMH;
  }

  x27CilovyKrok =
    (long)((hodnotaKmh / X27_MAX_KMH) * X27_KROKU_CELKEM + 0.5);
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
// 10) ÚVODNÍ LOGO
// ============================================================

void zobrazUvodniLogo()
{
  pinMode(LED_BUILTIN, OUTPUT);

  // Displej je teď jen 32 px vysoký - logo (16 px) dáme nahoru,
  // verzi firmwaru na spodní řádek.
  int x = (128 - STADION_WIDTH) / 2;
  int y = 2;

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
// 11) HLAVNÍ SMYČKA - SETUP
// ============================================================

void setup()
{
  // Některé bootloadery nechávaj watchdog po resetu běžet.
  wdt_disable();

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

  // Homing ručičky - najede na skutečnou nulu, ještě před
  // zapnutím watchdogu (viz komentář u hometniRucicku()).
  hometniRucicku();

  nactiOdoZEEPROM();
  tripMetry = 0;

  odoNaposledyUlozeno = odoMetry;

  // Přerušení pro kolo.
  attachInterrupt(
    digitalPinToInterrupt(SENSOR_PIN),
    snimacSpeedoPreruseni,
    FALLING
  );

  // Přerušení pro otáčky motoru.
  attachInterrupt(
    digitalPinToInterrupt(TACHO_PIN),
    snimacTachoPreruseni,
    FALLING
  );

  vykresliDashboard();

  wdt_enable(WDTO_2S);
}

// ============================================================
// 12) ZPRACOVÁNÍ SPEEDA
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

  if (!jePulzSpeedaDuveryhodny(surovaRychlostKmh))
  {
    mamePredchoziPulz = false;
    return;
  }

  mamePredchoziPulz = true;
  posledniSurovaRychlostKmh = surovaRychlostKmh;
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
// 13) ZPRACOVÁNÍ TACHA
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

  // 1 pulz = 1 otáčka.
  // 60 000 000 us za minutu / perioda = RPM.
  float suroveRpm =
    60000000.0 / intervalUs;

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
// 14) KONTROLA ZASTAVENÍ SPEEDA
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
// 15) KONTROLA ZASTAVENÍ TACHA
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
// 16) LOOP
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
// 17) DISPLEJ
// ============================================================

// --- TESTOVACÍ NÁSOBIČ (pro finální provoz změň na 1) ---
#define TEST_MULTIPLIER 1

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
  u8g2.setFont(u8g2_font_6x10_tf);

  // -- Řádek 1: km/h (malý textový záložní údaj k ručičce) --
  char radekRychlost[16];
  snprintf(radekRychlost, sizeof(radekRychlost), "%u km/h", testRychlost);
  u8g2.setCursor(0, 10);
  u8g2.print(radekRychlost);

  // -- Řádek 2: ODO --
  char radekOdo[16];
  snprintf(radekOdo, sizeof(radekOdo), "ODO %lu km", odoMetry / 1000UL);
  u8g2.setCursor(0, 21);
  u8g2.print(radekOdo);

  // -- Řádek 3: TRIP --
  float tripKm = tripMetry / 1000.0;

  char cisloTripu[8];
  dtostrf(tripKm, 4, 1, cisloTripu);

  char *zacatekCisla = cisloTripu;
  while (*zacatekCisla == ' ')
  {
    zacatekCisla++;
  }

  char radekTrip[16];
  snprintf(radekTrip, sizeof(radekTrip), "TRIP %s km", zacatekCisla);
  u8g2.setCursor(0, 31);
  u8g2.print(radekTrip);

  u8g2.sendBuffer();
}
