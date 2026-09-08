#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include <EEPROM.h>
#include <avr/wdt.h>
#include "stadionlogo.h"

// ============================================================
//   STADI TACHO  V1
//   Čte otáčky motoru ze snímače/smyčky kolem kabelu svíčky na D3 (přerušení INT1).
//   Počítá aktuální RPM, MAX RPM (v RAM) a motohodiny MTH (v EEPROM).
// ============================================================

// ============================================================
// 1) NASTAVENÍ
// ============================================================

const char FW_VERZE[] = "Stadi Tacho V1";
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Pin 3 pro přerušení INT1
const byte TACHO_PIN = 3;

// --- Ochrana proti rušení zapalování ---
// Při 18 000 RPM je perioda 3 333 us. 
// Jakýkoliv pulz s mezerou kratší než 2.8 ms (odpovídá ~21 400 RPM)
// je zahozen jako zákmita zapalování.
const unsigned long MIN_MEZERA_PULZU_US = 2800UL;

const float MAX_ROZUMNE_RPM = 18000.0;
const float MIN_ROZUMNE_RPM = 500.0;
const float MAX_SKOK_RPM = 5000.0;
const float MAX_ROZDIL_SOUSEDNICH_PULZU_RPM = 1500.0;

// Při 600 RPM jde pulz každých 100 ms.
// Pokud nepřijde pulz po dobu 200 ms, motor zhasnul -> okamžitě na 0 RPM.
const unsigned long CAS_DO_ZASTAVENI_MS = 200UL;

// Motohodiny se do EEPROM ukládají každých 0.1 MTH = 360 sekund běhu motoru.
const unsigned long KROK_ULOZENI_MTH_SEK = 360UL;

#define WATCHDOG_TIMEOUT WDTO_2S
const unsigned long LOGO_DOBA_ZOBRAZENI_MS = 2000;

// --- Kalmanův filtr (vyhlazení otáček) ---
const float KALMAN_Q = 2.0;   // Vyšší Q = blesková reakce na vrknutí plynem
const float KALMAN_R = 8.0;   // Mírné vyhlazení šumu zapalování

// --- EEPROM adresy pro Motohodiny ---
const int MTH_ADR_START = 0;
const int MTH_ADR_KONEC = 252; // 64 pozic po 4 bajtech (Round-Robin)


// ============================================================
// 2) STAV PROGRAMU
// ============================================================

// -- Motohodiny & MAX RPM --
unsigned long mthSekundy = 0;          // Ukládá se do EEPROM (v sekundách)
unsigned int maxRpm = 0;               // Pouze v RAM, po restartu 0

// -- Aktuální otáčky --
unsigned int zobrazeneRychlostRpm = 0;

// -- Kalmanův filtr --
float kalman_x = 0;
float kalman_p = 1;

float posledniSuroveRychlostRpm = 0.0;
bool mamePredchoziPulz = false;
bool motorBezi = false;

int mthAdresaAktualni = MTH_ADR_START;
unsigned long mthNaposledyUlozeno = 0;
unsigned long posIcasMthCasMs = 0;

// -- Časování displeje --
unsigned long displejNaposledyCas = 0;
const unsigned long DISPLEJ_INTERVAL_MS = 100; // 10 fps obnova displeje

// -- Sdíleno s ISR --
volatile unsigned long isr_casPoslednihoPulzuUs = 0;
volatile unsigned long isr_delkaPoslednihoIntervaluUs = 0;
volatile bool isr_mameNovyPulz = false;


// ============================================================
// 3) EEPROM - Motohodiny (MTH)
// ============================================================

void nactiMthZEEPROM()
{
  unsigned long nejvyssiNalezenaHodnota = 0;
  int adresaSNejvyssiHodnotou = MTH_ADR_START;

  for (int adr = MTH_ADR_START; adr <= MTH_ADR_KONEC; adr += 4)
  {
    unsigned long hodnota = 0;
    EEPROM.get(adr, hodnota);

    if (hodnota != 0xFFFFFFFFUL && hodnota >= nejvyssiNalezenaHodnota)
    {
      nejvyssiNalezenaHodnota = hodnota;
      adresaSNejvyssiHodnotou = adr;
    }
  }

  mthSekundy = nejvyssiNalezenaHodnota;
  mthAdresaAktualni = adresaSNejvyssiHodnotou;
}

void ulozMthDoEEPROM(unsigned long hodnotaSekund)
{
  mthAdresaAktualni += 4;
  if (mthAdresaAktualni > MTH_ADR_KONEC)
  {
    mthAdresaAktualni = MTH_ADR_START;
  }

  EEPROM.put(mthAdresaAktualni, hodnotaSekund);
}


// ============================================================
// 4) PŘERUŠENÍ (ISR)
// ============================================================

void snimacTachoPreruseni()
{
  unsigned long ted = micros();

  if (isr_casPoslednihoPulzuUs == 0)
  {
    isr_casPoslednihoPulzuUs = ted;
    return;
  }

  unsigned long interval = ted - isr_casPoslednihoPulzuUs;

  // Debounce zákmity zapalování (kratší než 2.8 ms ignorujeme)
  if (interval < MIN_MEZERA_PULZU_US)
  {
    return;
  }

  isr_delkaPoslednihoIntervaluUs = interval;
  isr_casPoslednihoPulzuUs = ted;
  isr_mameNovyPulz = true;
}


// ============================================================
// 5) FILTR OTÁČEK
// ============================================================

float vyhladRpm(float noveMereniRpm)
{
  kalman_p = kalman_p + KALMAN_Q;
  float zisk = kalman_p / (kalman_p + KALMAN_R);

  kalman_x = kalman_x + zisk * (noveMereniRpm - kalman_x);
  kalman_p = (1.0 - zisk) * kalman_p;

  return kalman_x;
}

bool jePulzDuveryhodny(float suroveRpm)
{
  if (suroveRpm > MAX_ROZUMNE_RPM || suroveRpm < MIN_ROZUMNE_RPM)
  {
    return false;
  }

  if (motorBezi)
  {
    float rozdil = fabs(suroveRpm - kalman_x);

    if (rozdil > MAX_SKOK_RPM)
    {
      bool souhlasiSPredchozim =
        mamePredchoziPulz &&
        fabs(suroveRpm - posledniSuroveRychlostRpm) < MAX_ROZDIL_SOUSEDNICH_PULZU_RPM;

      if (souhlasiSPredchozim)
      {
        kalman_x = suroveRpm;
        kalman_p = 1.0;
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
// ÚVODNÍ LOGO
// ============================================================

void zobrazUvodniLogo()
{
  pinMode(LED_BUILTIN, OUTPUT);

  int x = (128 - STADION_WIDTH) / 2;
  int y = (64 - STADION_HEIGHT) / 2;

  u8g2.clearBuffer();
  u8g2.drawXBMP(x, y, STADION_WIDTH, STADION_HEIGHT, logo_stadion);

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(0, 64);
  u8g2.print(FW_VERZE);

  u8g2.sendBuffer();

  const byte POCET_BLIKNUTI = 4;
  const unsigned long krokMs = LOGO_DOBA_ZOBRAZENI_MS / (POCET_BLIKNUTI * 2);
  for (byte i = 0; i < POCET_BLIKNUTI; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(krokMs);
    digitalWrite(LED_BUILTIN, LOW);
    delay(krokMs);
  }
}


// ============================================================
// 6) HLAVNÍ SMYČKA
// ============================================================

void setup()
{
  wdt_disable();

  u8g2.begin();
  pinMode(TACHO_PIN, INPUT_PULLUP);

  zobrazUvodniLogo();

  nactiMthZEEPROM();
  mthNaposledyUlozeno = mthSekundy;

  // Připojení přerušení na Pin D3 (INT1)
  attachInterrupt(
    digitalPinToInterrupt(TACHO_PIN),
    snimacTachoPreruseni,
    FALLING
  );

  vykresliDashboard();
  wdt_enable(WATCHDOG_TIMEOUT);
}

void zpracujNovyPulzPokudExistuje()
{
  if (!isr_mameNovyPulz)
  {
    return;
  }

  unsigned long intervalUs;
  noInterrupts();
  intervalUs = isr_delkaPoslednihoIntervaluUs;
  isr_mameNovyPulz = false;
  interrupts();

  // 1 otáčka = 1 pulz. 60 000 000 us v minutě / interval v us = RPM
  float suroveRpm = 60000000.0 / intervalUs;

  if (!jePulzDuveryhodny(suroveRpm))
  {
    mamePredchoziPulz = false;
    return;
  }

  mamePredchoziPulz = true;
  posledniSuroveRychlostRpm = suroveRpm;
  motorBezi = true;

  float vyhlazeneRpm = vyhladRpm(suroveRpm);
  zobrazeneRychlostRpm = (unsigned int)(vyhlazeneRpm + 0.5);

  // Záznam MAX RPM
  if (zobrazeneRychlostRpm > maxRpm)
  {
    maxRpm = zobrazeneRychlostRpm;
  }
}

void zkontrolujJestliStojime()
{
  unsigned long casPoslednihoPulzu;
  noInterrupts();
  casPoslednihoPulzu = isr_casPoslednihoPulzuUs;
  interrupts();

  bool jsmeUzNekdyJeli = (casPoslednihoPulzu != 0);
  unsigned long dobaBezPulzuUs = micros() - casPoslednihoPulzu;

  // Pokud neprisel pulz do 200 ms, motor zhasnul
  if (jsmeUzNekdyJeli && dobaBezPulzuUs > CAS_DO_ZASTAVENI_MS * 1000UL)
  {
    kalman_x = 0;
    kalman_p = 1;
    zobrazeneRychlostRpm = 0;
    mamePredchoziPulz = false;
    motorBezi = false;
  }
}

void aktualizujMotohodiny()
{
  if (!motorBezi)
  {
    posIcasMthCasMs = millis();
    return;
  }

  unsigned long ted = millis();
  if (posIcasMthCasMs == 0)
  {
    posIcasMthCasMs = ted;
    return;
  }

  // Přičítání reálných sekund běhu motoru
  if (ted - posIcasMthCasMs >= 1000)
  {
    mthSekundy += (ted - posIcasMthCasMs) / 1000;
    posIcasMthCasMs = ted;
  }

  // Zápis do EEPROM každých 360 s (0.1 MTH)
  if (mthSekundy - mthNaposledyUlozeno >= KROK_ULOZENI_MTH_SEK)
  {
    ulozMthDoEEPROM(mthSekundy);
    mthNaposledyUlozeno = mthSekundy;
  }
}

void loop()
{
  wdt_reset();

  zpracujNovyPulzPokudExistuje();
  zkontrolujJestliStojime();
  aktualizujMotohodiny();

  unsigned long ted = millis();
  if (ted - displejNaposledyCas > DISPLEJ_INTERVAL_MS)
  {
    vykresliDashboard();
    displejNaposledyCas = ted;
  }
}


// ============================================================
// 7) DISPLEJ
// ============================================================

void vykresliDashboard()
{
  u8g2.clearBuffer();

  // --- Formátování velkých otáček s mezerou pro tisíce (např. 12 450) ---
  char strRpm[10];
  if (zobrazeneRychlostRpm >= 1000)
  {
    snprintf(strRpm, sizeof(strRpm), "%u %03u", zobrazeneRychlostRpm / 1000, zobrazeneRychlostRpm % 1000);
  }
  else
  {
    snprintf(strRpm, sizeof(strRpm), "%u", zobrazeneRychlostRpm);
  }

  // --- Velké číslo otáček ---
  u8g2.setFont(u8g2_font_logisoso58_tn);

  int sirkaRpm = u8g2.getStrWidth(strRpm);
  int xPozice = (128 - 25) - sirkaRpm; // 25 px rezerva zprava na "rpm"
  if (xPozice < 0) xPozice = 0;

  u8g2.setCursor(xPozice, 57);
  u8g2.print(strRpm);

  // --- Popisek "rpm" ---
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(108, 57);
  u8g2.print("rpm");

  // --- MAX RPM vlevo dole (v RAM) ---
  u8g2.setCursor(0, 64);
  u8g2.print("MAX ");
  u8g2.print(maxRpm);

  // --- Motohodiny MTH vpravo dole (1 desetinné místo, z EEPROM) ---
  float mthHodiny = mthSekundy / 3600.0;

  char cisloMth[8];
  dtostrf(mthHodiny, 4, 1, cisloMth);

  char *zacatekCisla = cisloMth;
  while (*zacatekCisla == ' ')
  {
    zacatekCisla++;
  }

  char mthText[12];
  snprintf(mthText, sizeof(mthText), "%s MTH", zacatekCisla);

  int sirkaTextu = u8g2.getStrWidth(mthText);
  u8g2.setCursor(128 - sirkaTextu, 64);
  u8g2.print(mthText);

  u8g2.sendBuffer();
}
