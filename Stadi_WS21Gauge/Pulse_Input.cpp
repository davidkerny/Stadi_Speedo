// ============================================================================
//  Pulse_Input.cpp  -  snímání pulzů (RYCHLOST + OTÁČKY) pro budík
//  Waveshare ESP32-S3-Touch-LCD-2.1  (Arduino-ESP32 3.x)
//
//  Pulse_Input průběžně plní tyto proměnné:
//        speed_kmh   rpm   odo_km   trip_km   max_kmh   max_rpm   mth_h
//  Stejné proměnné čte displej v LVGL_MainBudik.cpp
//
//  PINOUT (čidla PŘIZEMŇUJÍ pin, tj. spínač / otevřený kolektor proti GND)
//  ----------------------------------------------------------------------
//     RYCHLOST (speedo)  ->  GPIO44  (na desce popsáno RXD, UART konektor)
//     OTÁČKY   (tacho)   ->  GPIO43  (na desce popsáno TXD, UART konektor)
//     společná zem čidel ->  GND na stejném konektoru
//
//  !! Aby byly GPIO43/44 volné, musí být v Arduino IDE (Tools):
//        USB CDC On Boot: Enabled
//     (pak jde Serial přes USB-C a UART0 se nepoužívá). V kódu nikdy nevolej
//     Serial0.begin() ani Serial1.begin() na těchto pinech.
//  !! Piny snesou max 3,3 V. Čidlo smí pin jen spojit se zemí, nikdy na něj
//     nepřivádět 5 V / 12 V.
//
//  Doporučení k zapojení: do série s každým čidlem odpor 1 kOhm (ochrana pinu),
//  u delšího kabelu navíc 100 nF mezi pin a GND. Vnitřní pull-up je zapnutý,
//  při rušení přidej vnější 10 kOhm na 3V3.
// ============================================================================

#include <Arduino.h>
#include <Preferences.h>

// ------------------------------- NASTAVENÍ ----------------------------------
#define PIN_SPEEDO            44        // vstup rychlosti
#define PIN_TACHO             43        // vstup otáček
#define PULSE_EDGE            FALLING   // pulz = sestupná hrana (přizemnění)

#define PULSE_AUTOSTART       1         // 1 = spustí se samo, 0 = volej Pulse_Init() v setup()
#define PULSE_PERSIST         1         // 1 = ODO / TRIP / MTH se ukládají do paměti
#define PULSE_DEBUG           1         // 1 = jednou za sekundu výpis do Serial Monitoru (115200)

// Rychlost (hodnoty převzaté z původního kódu)
static const float OBVOD_KOLA_M            = 1.77f;     // obvod kola [m]
#define SPEEDO_PULZY_NA_OTACKU_KOLA        1            // kolik pulzů dá čidlo na 1 otáčku kola
#define SPEEDO_MIN_MEZERA_US               45000UL      // kratší mezera mezi pulzy = zákmit, ignorovat
static const float MAX_ROZUMNA_RYCHLOST_KMH = 120.0f;
#define CAS_DO_ZASTAVENI_MS                2000UL       // bez pulzu tak dlouho = stojíme

// Otáčky
#define TACHO_MIN_MEZERA_US                560UL        // kratší mezera = rušení
static const float PULZY_NA_OTACKU         = 5.0f;      // pulzů na 1 otáčku motoru
static const float MAX_ROZUMNE_RPM         = 18000.0f;
static const float MIN_ROZUMNE_RPM         = 500.0f;
#define TACHO_TIMEOUT_MS                   400UL        // bez pulzu tak dlouho = motor stojí

// Vyhlazení (Kalman, stejné konstanty jako v původním kódu)
static const float KALMAN_Q = 0.5f, KALMAN_R = 8.0f;           // rychlost
static const float TACHO_KALMAN_Q = 2.0f, TACHO_KALMAN_R = 8.0f;  // otáčky

// Ukládání
#define PULSE_ODO_START_KM     0.0f     // počáteční ODO při úplně prvním spuštění
#define SAVE_KAZDYCH_M         1000UL   // ukládat po ujetí tolika metrů
#define SAVE_KAZDYCH_MTH_S     360UL    // ... nebo po tolika sekundách chodu motoru (0,1 h)
#define SAVE_PO_ZASTAVENI_MS   3000UL   // po zastavení uložit po této době klidu
#define SAVE_MIN_ODSTUP_MS     5000UL   // nejkratší odstup mezi zápisy po ujeté vzdálenosti
#define SAVE_MIN_ODSTUP_MTH_MS 120000UL // nejkratší odstup, když se mění jen motohodiny (motor běží ve stání)

#define SAMPLE_MS              20       // jak často se pulzy vyhodnocují

// ------------------- PROMĚNNÉ PRO DISPLEJ (rozhraní) ------------------------
// "weak" = pokud je stejná proměnná definovaná v LVGL_Example.cpp, použije se
// tamější a tady nevznikne kolize; pokud ne, platí tyto.
__attribute__((weak)) float speed_kmh = 0;      // rychlost [km/h]
__attribute__((weak)) float rpm       = 0;      // otáčky [1/min]
__attribute__((weak)) float odo_km    = 0;      // celkem km
__attribute__((weak)) float trip_km   = 0;      // denní km
__attribute__((weak)) float max_kmh   = 0;      // max. rychlost
__attribute__((weak)) float max_rpm   = 0;      // max. otáčky
__attribute__((weak)) float mth_h     = 0;      // motohodiny [h]

// ------------------------------- PŘERUŠENÍ ----------------------------------
// ISR jen počítá pulzy a zapisuje čas posledního. Žádná matematika.
static portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t spCount = 0, spLastUs = 0, spRejected = 0;
static volatile uint32_t tcCount = 0, tcLastUs = 0, tcRejected = 0;

static void IRAM_ATTR isrSpeedo() {
  uint32_t now = micros();
  if (now == 0) now = 1;
  portENTER_CRITICAL_ISR(&pulseMux);
  if (spLastUs != 0 && (uint32_t)(now - spLastUs) < SPEEDO_MIN_MEZERA_US) {
    spRejected++;
  } else {
    spLastUs = now;
    spCount++;
  }
  portEXIT_CRITICAL_ISR(&pulseMux);
}

static void IRAM_ATTR isrTacho() {
  uint32_t now = micros();
  if (now == 0) now = 1;
  portENTER_CRITICAL_ISR(&pulseMux);
  if (tcLastUs != 0 && (uint32_t)(now - tcLastUs) < TACHO_MIN_MEZERA_US) {
    tcRejected++;
  } else {
    tcLastUs = now;
    tcCount++;
  }
  portEXIT_CRITICAL_ISR(&pulseMux);
}

// ----------------------------- POMOCNÉ TŘÍDY --------------------------------
struct Kalman {
  float x = 0, p = 1;
  bool  ready = false;
  void  reset() { x = 0; p = 1; ready = false; }
  float update(float z, float q, float r) {
    if (!ready) { x = z; p = 1; ready = true; return x; }   // první hodnota: bez rozjezdu od nuly
    p += q;
    float k = p / (p + r);
    x += k * (z - x);
    p = (1.0f - k) * p;
    return x;
  }
};

struct Median3 {
  float v[3] = {0, 0, 0};
  uint8_t n = 0;
  void reset() { n = 0; }
  float push(float z) {
    if (n < 3) v[n++] = z; else { v[0] = v[1]; v[1] = v[2]; v[2] = z; }
    if (n < 3) return z;
    float a = v[0], b = v[1], c = v[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
  }
};

// Měření frekvence z počtu pulzů a časů pulzů (ne z časů vzorkování):
// f = (počet nových pulzů) / (čas posledního pulzu - čas posledního pulzu v minulém kroku)
struct Chan {
  bool     primed = false;
  uint32_t prevCount = 0, prevLast = 0;
  uint32_t seenMs = 0;
  float    f = 0;
};

static void chanSample(Chan &c, uint32_t cnt, uint32_t last, uint32_t nowUs, uint32_t nowMs,
                       uint32_t timeoutMs, float &fHz, bool &fresh) {
  fresh = false;
  fHz = 0;
  if (cnt == 0) return;                                   // zatím žádný pulz
  if (!c.primed) {                                        // první pulz: jen si ho zapamatuj
    c.primed = true; c.prevCount = cnt; c.prevLast = last; c.seenMs = nowMs; c.f = 0;
    return;
  }
  uint32_t dN = cnt - c.prevCount;
  if (dN > 0) {
    if ((uint32_t)(nowMs - c.seenMs) > timeoutMs) {       // po klidu: první pulz nic neříká o rychlosti
      c.prevCount = cnt; c.prevLast = last; c.seenMs = nowMs; c.f = 0;
      return;
    }
    uint32_t dT = last - c.prevLast;                      // [us] mezi posledním minulým a posledním novým pulzem
    if (dT > 0) {
      c.f = (float)dN * 1.0e6f / (float)dT;
      fresh = true;
    }
    c.prevCount = cnt; c.prevLast = last; c.seenMs = nowMs;
    fHz = c.f;
    return;
  }
  // žádný nový pulz
  if ((uint32_t)(nowMs - c.seenMs) > timeoutMs) { c.f = 0; return; }   // vypršel čas -> 0
  if (c.f > 0) {                                          // další pulz je opožděný => skutečná frekvence je nižší
    float age = (float)(uint32_t)(nowUs - last);
    if (age > 0) {
      float bound = 1.0e6f / age;
      if (bound < c.f) c.f = bound;
    }
  }
  fHz = c.f;
}

// --------------------------------- STAV -------------------------------------
static Chan chSpeed, chTacho;
static Kalman kSpeed, kTacho;
static Median3 mSpeed, mTacho;
static uint32_t speedValidMs = 0, tachoValidMs = 0;
static uint32_t odoCountPrev = 0;

static double odoM = 0, tripM = 0, mthS = 0;
static uint32_t savedOdoM = 0, savedTripM = 0, savedMthS = 0;
static uint32_t lastSaveMs = 0, lastMoveMs = 0, lastStepMs = 0;
static volatile bool resetTripRequest = false;
static volatile bool resetMaxRequest = false;

// ------------------------------ UKLÁDÁNÍ ------------------------------------
static void stateLoad() {
  odoM = PULSE_ODO_START_KM * 1000.0;
  tripM = 0; mthS = 0;
#if PULSE_PERSIST
  Preferences prefs;
  if (prefs.begin("gauge", true)) {
    odoM  = prefs.getUInt("odo_m",  (uint32_t)(PULSE_ODO_START_KM * 1000.0f));
    tripM = prefs.getUInt("trip_m", 0);
    mthS  = prefs.getUInt("mth_s",  0);
    prefs.end();
  }
#endif
  savedOdoM = (uint32_t)odoM; savedTripM = (uint32_t)tripM; savedMthS = (uint32_t)mthS;
}

static void stateSave(uint32_t nowMs) {
#if PULSE_PERSIST
  Preferences prefs;
  if (prefs.begin("gauge", false)) {
    if ((uint32_t)odoM  != savedOdoM)  prefs.putUInt("odo_m",  (uint32_t)odoM);
    if ((uint32_t)tripM != savedTripM) prefs.putUInt("trip_m", (uint32_t)tripM);
    if ((uint32_t)mthS  != savedMthS)  prefs.putUInt("mth_s",  (uint32_t)mthS);
    prefs.end();
  }
#endif
  savedOdoM = (uint32_t)odoM; savedTripM = (uint32_t)tripM; savedMthS = (uint32_t)mthS;
  lastSaveMs = nowMs;
}

// --------------------------- JEDEN KROK VÝPOČTU ------------------------------
static void pulseStep() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  uint32_t sc, sl, tc, tl;
  portENTER_CRITICAL(&pulseMux);
  sc = spCount; sl = spLastUs; tc = tcCount; tl = tcLastUs;
  portEXIT_CRITICAL(&pulseMux);

  float dtS = (lastStepMs == 0) ? (SAMPLE_MS / 1000.0f) : (float)(uint32_t)(nowMs - lastStepMs) / 1000.0f;
  lastStepMs = nowMs;
  if (dtS > 1.0f) dtS = 1.0f;

  // ---- ujetá vzdálenost: každý přijatý pulz = kus obvodu kola
  uint32_t dPulses = sc - odoCountPrev;
  odoCountPrev = sc;
  if (dPulses) {
    double m = (double)dPulses * OBVOD_KOLA_M / SPEEDO_PULZY_NA_OTACKU_KOLA;
    odoM += m; tripM += m;
    lastMoveMs = nowMs;
  }

  // ---- rychlost
  float f; bool fresh;
  chanSample(chSpeed, sc, sl, nowUs, nowMs, CAS_DO_ZASTAVENI_MS, f, fresh);
  float v = speed_kmh;
  if (f <= 0) {
    kSpeed.reset(); mSpeed.reset(); v = 0;
  } else {
    float raw = f * OBVOD_KOLA_M * 3.6f / SPEEDO_PULZY_NA_OTACKU_KOLA;
    if (raw <= MAX_ROZUMNA_RYCHLOST_KMH) {
      float z = fresh ? mSpeed.push(raw) : raw;
      v = kSpeed.update(z, KALMAN_Q, KALMAN_R);
      speedValidMs = nowMs;
    } else if ((uint32_t)(nowMs - speedValidMs) > CAS_DO_ZASTAVENI_MS) {
      kSpeed.reset(); mSpeed.reset(); v = 0;
    }
  }
  if (v < 0.3f) v = 0;
  speed_kmh = v;
  if (v > max_kmh) max_kmh = v;

  // ---- otáčky
  chanSample(chTacho, tc, tl, nowUs, nowMs, TACHO_TIMEOUT_MS, f, fresh);
  float r = rpm;
  if (f <= 0) {
    kTacho.reset(); mTacho.reset(); r = 0;
  } else {
    float raw = f * 60.0f / PULZY_NA_OTACKU;
    if (raw >= MIN_ROZUMNE_RPM && raw <= MAX_ROZUMNE_RPM) {
      float z = fresh ? mTacho.push(raw) : raw;
      r = kTacho.update(z, TACHO_KALMAN_Q, TACHO_KALMAN_R);
      tachoValidMs = nowMs;
    } else if ((uint32_t)(nowMs - tachoValidMs) > TACHO_TIMEOUT_MS) {
      kTacho.reset(); mTacho.reset(); r = 0;
    }
  }
  if (r < 1.0f) r = 0;
  rpm = r;
  if (r > max_rpm) max_rpm = r;

  // ---- motohodiny: počítají se, dokud motor běží
  if (r > 0) mthS += dtS;

  // ---- požadavky z venku
  if (resetTripRequest) { resetTripRequest = false; tripM = 0; }
  if (resetMaxRequest)  { resetMaxRequest = false;  max_kmh = 0; max_rpm = 0; }

  // ---- výstup do proměnných pro displej
  odo_km  = (float)(odoM / 1000.0);
  trip_km = (float)(tripM / 1000.0);
  mth_h   = (float)(mthS / 3600.0);

  // ---- ukládání (v jízdě zřídka, ve stání po chvíli klidu)
#if PULSE_PERSIST
  uint32_t dOdo = (uint32_t)odoM - savedOdoM;
  uint32_t dMth = (uint32_t)mthS - savedMthS;
  bool distChanged = ((uint32_t)odoM != savedOdoM) || ((uint32_t)tripM != savedTripM);
  bool mthChanged  = ((uint32_t)mthS != savedMthS);
  bool stopped = (uint32_t)(nowMs - lastMoveMs) > SAVE_PO_ZASTAVENI_MS;
  uint32_t sinceSave = (uint32_t)(nowMs - lastSaveMs);
  if (distChanged || mthChanged) {
    if (dOdo >= SAVE_KAZDYCH_M || dMth >= SAVE_KAZDYCH_MTH_S ||
        (stopped && distChanged && sinceSave >= SAVE_MIN_ODSTUP_MS) ||
        (stopped && sinceSave >= SAVE_MIN_ODSTUP_MTH_MS)) {
      stateSave(nowMs);
    }
  }
#endif
}

// ---------------------------------- TASK -------------------------------------
static void pulseTask(void *) {
  vTaskDelay(pdMS_TO_TICKS(1000));          // počkat, až doběhne start systému a displeje
  stateLoad();
  lastSaveMs = millis();

  pinMode(PIN_SPEEDO, INPUT_PULLUP);
  pinMode(PIN_TACHO,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SPEEDO), isrSpeedo, PULSE_EDGE);
  attachInterrupt(digitalPinToInterrupt(PIN_TACHO),  isrTacho,  PULSE_EDGE);

  odo_km  = (float)(odoM / 1000.0);
  trip_km = (float)(tripM / 1000.0);
  mth_h   = (float)(mthS / 3600.0);

  TickType_t t = xTaskGetTickCount();
  uint32_t dbgMs = millis();
  for (;;) {
    vTaskDelayUntil(&t, pdMS_TO_TICKS(SAMPLE_MS));
    pulseStep();

#if PULSE_DEBUG && defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
    if ((uint32_t)(millis() - dbgMs) >= 1000 && Serial.availableForWrite() >= 128) {
      dbgMs = millis();
      Serial.printf("[pulse] speedo n=%lu rej=%lu | tacho n=%lu rej=%lu | %.1f km/h  %.0f rpm  ODO %.3f km  TRIP %.3f km  MTH %.3f h\n",
                    (unsigned long)spCount, (unsigned long)spRejected,
                    (unsigned long)tcCount, (unsigned long)tcRejected,
                    speed_kmh, rpm, odo_km, trip_km, mth_h);
    }
#else
    (void)dbgMs;
#endif
  }
}

// -------------------------------- VEŘEJNÉ API --------------------------------
void Pulse_Init() {
  static bool started = false;
  if (started) return;
  started = true;
  xTaskCreatePinnedToCore(pulseTask, "pulse", 6144, NULL, 2, NULL, 0);
}

void Pulse_ResetTrip() { resetTripRequest = true; }   // vynuluje TRIP
void Pulse_ResetMax()  { resetMaxRequest = true; }    // vynuluje MAX km/h a MAX rpm

#if PULSE_AUTOSTART
// Arduino-ESP32 volá initVariant() při startu, ještě před setup(). Díky tomu se
// modul spustí sám a do .ino nemusíš nic psát.
extern "C" void initVariant(void) {
  Pulse_Init();
}
#endif

#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#warning "Pulse_Input: piny GPIO43/44 jsou UART0. V Tools nastav 'USB CDC On Boot: Enabled' a nepouzivej Serial0/Serial1 na UART0."
#endif
