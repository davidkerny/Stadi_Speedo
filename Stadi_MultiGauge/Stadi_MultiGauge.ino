#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "stadionlogo.h"

// ============================================================

const char FW_VERZE[] = "Stadi MultiGauge V1.52";

//  Speedo furt čte kolo na D2. = KM/H
//  Tacho navíc čte otáčky motoru na D3 = RPM
//  RPM ve stovkách (50 = 5000 RPM, 70 = 7000 RPM.
//  ODOmetr (v EEPROM) a TRIP (v RAM)
//  MAX RPM ani MTH tady nejsou, bo na to kašlem.
// ============================================================

// 1) NASTAVENÍ
// 2) STAV PROGRAMU
// 3) MĚŘENÍ NAPÁJENÍ (Vcc)
// 4) EEPROM - ukládání ODO
// 5) PŘERUŠENÍ - SPEEDO
// 6) PŘERUŠENÍ - TACHO
// 7) FILTR SPEEDA
// 8) FILTR TACHA
// 8b) HEARTBEAT LED (diagnostika, D13)
// 9) ÚVODNÍ LOGO
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
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); //Laskakit 1.3"
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); //Placatej zmrd 0.91"


// --- SPEEDO snímač kola (INT0) ---
const byte SENSOR_PIN = 2;

// Obvod kola v metrech. = vzdálenost per pulz
const float OBVOD_KOLA_M = 1.74; //16" kolo, guma terénní 16x2.75 simson
//const float OBVOD_KOLA_M = 1.86; //19" kolo, guma 19x2.25 stadion

// --- SPEEDO tuning ---
const unsigned long MIN_MEZERA_PULZU_US = 45000UL; // do 120 km/h. Rychlejší pulz ignorujem.
const float MAX_ROZUMNA_RYCHLOST_KMH = 120.0;
const float MAX_SKOK_KMH = 25.0; // mezi pulzy
const float MAX_ROZDIL_SOUSEDNICH_PULZU_KMH = 8.0;
const float MAX_ZMENA_RYCHLOSTI_KMH_ZA_S = 60.0;
const unsigned long CAS_DO_ZASTAVENI_MS = 2000UL;
const float KALMAN_Q = 0.5;
const float KALMAN_R = 8.0;

// --- TACHO snímač motor rpm (INT1) ---
const byte TACHO_PIN = 3; ducati energia simson

// --- TACHO tuning ---
// const unsigned long MIN_MEZERA_TACHO_US = 2800UL; v případě 1pulz=1ot.
const unsigned long MIN_MEZERA_TACHO_US = 1333UL; (15Krpm 3pulz/ot)
const float PULZY_NA_OTACKU = 3.0;  // Tacho pulzy per otáčka DUCATI ENERGIA
const float MAX_ROZUMNE_RPM = 15000.0;
const float MIN_ROZUMNE_RPM = 500.0;
const float MAX_SKOK_RPM = 7000.0; // bylo 5000, zkousim benevolentnejsi mereni
const float MAX_ROZDIL_SOUSEDNICH_PULZU_RPM = 2500.0; // bylo 1500, zkousim benevolentnejsi mereni
const unsigned long CAS_DO_ZASTAVENI_TACHO_MS = 200UL; // Když 200 ms nic nepřijde, motor chcípl.
const float TACHO_KALMAN_Q = 2.0; // Vyšší Q = rychlejší reakce na vrknutí plynem.
const float TACHO_KALMAN_R = 8.0;

// --- Welcome logo ---
const unsigned long LOGO_DOBA_ZOBRAZENI_MS = 1900UL;
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
unsigned long tripMetry = 0;

// -- Aktuální rychlost --
unsigned int zobrazenaRychlostKmh = 0;

// -- Aktuální otáčky --
unsigned int zobrazeneRpm = 0;

// -- Kalman speedo --
float kalman_x = 0;
float kalman_p = 1;

// -- Pomoc speeda s rušením --
float posledniSurovaRychlostKmh = 0.0;
bool mamePredchoziPulz = false;
bool jsmeVPohybu = false;

// Čas posledního zpracovaného pulzu (přijatého i zamítnutého) -
// potřeba pro výpočet implikovaného zrychlení/zpomalení mezi
// dvěma po sobě jdoucími pulzy.
unsigned long casPoslednihoZpracovanehoPulzuUs = 0;

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

// "ted" je čas (z micros()) tohodle pulzu - potřeba pro
// fyzikální kontrolu rychlosti změny.
bool jePulzSpeedaDuveryhodny(float surovaRychlostKmh, unsigned long ted)
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

      // Fyzikální kontrola: i kdyby dva pulzy souhlasily,
      // implikovaná rychlost změny nesmí přesáhnout to, co
      // moped fyzicky dokáže. Periodické rušení dokáže
      // "souhlasit samo se sebou", ale nedokáže obelhat čas.
      float uplynuleSekundy =
        (ted - casPoslednihoZpracovanehoPulzuUs) / 1000000.0;

      bool jeZmenaFyzikalneMozna =
        uplynuleSekundy > 0 &&
        (rozdilOdVyhlazene / uplynuleSekundy) <= MAX_ZMENA_RYCHLOSTI_KMH_ZA_S;

      if (souhlasiSPredchozim && jeZmenaFyzikalneMozna)
      {
        // Dva pulzy za sebou si sedí a je to fyzikálně možné
        // -> reálné zrychlení.
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
// 8b) HEARTBEAT LED (diagnostika, D13)
// ============================================================

// Timer2 v CTC módu, prescaler 1024 -> jeden "tik" cca 16 ms.
// V ISR čekáme 16 tiků (~256 ms), než LED přepneme - vznikne
// viditelné, pomalé blikání (~2 Hz). Běží nezávisle na loop().

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

  const int VYSKA_OBLASTI_LOGA = 50;
  int x = (128 - STADION_WIDTH) / 2;
  int y = (VYSKA_OBLASTI_LOGA - STADION_HEIGHT) / 2;

  u8g2.clearBuffer();
  u8g2.drawXBMP(x, y, STADION_WIDTH, STADION_HEIGHT, logo_stadion);

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(0, 64);
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
  wdt_enable(WDTO_8S);

  // === I2C BUS CLEAR ===
// něco jako zvon na hajzl pro I2C. aka defibrilátor.

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

  // STOP podmínka — VŽDY, když jsme dělali clearing
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

  zobrazUvodniLogo();

  // Heartbeat běží od teď dál, nezávisle na zbytku programu.
  nastavHeartbeatTimer();

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

  unsigned long ted = micros();

  bool duveryhodny = jePulzSpeedaDuveryhodny(surovaRychlostKmh, ted);

  // Bez ohledu na výsledek si VŽDYCKY zapamatujeme surovou
  // hodnotu a čas - i zamítnutý pulz je platný referenční bod
  // pro porovnání s tím příštím. Bez tohohle by jediné zamítnutí
  // při prudkém zrychlení zpřetrhalo řetězec navždy (dokud
  // nezastavíš) - přesně tohle způsobovalo zaseknutí na
  // konstantní špatné rychlosti po prudkém rozjezdu.
  posledniSurovaRychlostKmh = surovaRychlostKmh;
  mamePredchoziPulz = true;
  casPoslednihoZpracovanehoPulzuUs = ted;

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

// --- TESTOVACÍ NÁSOBIČ (pro finální provoz změň na 1) ---
#define TEST_MULTIPLIER 1

void vykresliDashboard()
{
  u8g2.clearBuffer();

  // Aplikace násobiče pro testovací účely
  unsigned int testRpm = zobrazeneRpm * TEST_MULTIPLIER;
  unsigned int testRychlost = zobrazenaRychlostKmh * TEST_MULTIPLIER;

  // ----------------------------------------------------------
  // RPM vpravo nahoře (ve stovkách RPM).
  // Např. 8000 RPM = 80, 11200 RPM = 112
  // ----------------------------------------------------------
  u8g2.setFont(u8g2_font_logisoso24_tn);

  // Převod na stovky RPM (dělení 100)
  int rpmStovky = testRpm / 100;

  // Formátování textu na 2 až 3 cifry (včetně vodicí nuly pro <10)
  char rpmText[8];
  if (rpmStovky < 10)
  {
    snprintf(rpmText, sizeof(rpmText), "0%d", rpmStovky);
  }
  else
  {
    snprintf(rpmText, sizeof(rpmText), "%d", rpmStovky);
  }

  // --- POLOHOVÁNÍ RPM (upravuj podle potřeby X, Y) ---
  if (rpmStovky < 100)
  {
    // Pro dvoumístné číslo (např. "00", "10", "80")
    u8g2.setCursor(99, 31); 
  }
  else
  {
    // Pro třímístné číslo (např. "112", "150")
    u8g2.setCursor(85, 31); 
  }
  u8g2.print(rpmText);

  // ----------------------------------------------------------
  // Velké číslo rychlosti.
  // ----------------------------------------------------------
  u8g2.setFont(u8g2_font_logisoso58_tn);

  if (testRychlost < 10)
  {
    u8g2.setCursor(37, 58);
  }
  else if (testRychlost < 100)
  {
    u8g2.setCursor(6, 58);
  }
  else
  {
    u8g2.setCursor(-20, 58);
  }
  u8g2.print(testRychlost);

  // ----------------------------------------------------------
  // ODO, TRIP a text rpm (zobrazují se pouze při rychlosti <= 19 km/h)
  // ----------------------------------------------------------
  if (testRychlost <= 19)
  {
    u8g2.setFont(u8g2_font_6x10_tf);

     // Popisek x100.
     //u8g2.setCursor(95, 51);
     //u8g2.print("x100");
     
    // Popisek rpm.
     u8g2.setCursor(103, 41);
     u8g2.print("rpm");

    // ODO
    u8g2.setCursor(0, 64);
    u8g2.print(odoMetry / 1000);

    // TRIP
    float tripKm = tripMetry / 1000.0;

    char cisloTripu[8];
    dtostrf(tripKm, 4, 1, cisloTripu);

    char *zacatekCisla = cisloTripu;

    while (*zacatekCisla == ' ')
    {
      zacatekCisla++;
    }

    char tripText[12];

    snprintf(
      tripText,
      sizeof(tripText),
      "t %s",
      zacatekCisla
    );

    int sirkaTextu = u8g2.getStrWidth(tripText);

    u8g2.setCursor(
      128 - sirkaTextu,
      64
    );

    u8g2.print(tripText);
  }

  u8g2.sendBuffer();
}
