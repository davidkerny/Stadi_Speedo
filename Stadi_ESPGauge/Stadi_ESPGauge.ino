// ============================================================
// Stadi AnaloGauge pro ESP32-S3 2.1" kruhový displej
// LVGL 8.x + Arduino_GFX_Library
// ============================================================

#include <Arduino.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <Adafruit_CST8XX.h>
#include <EEPROM.h>

// ============================================================
// 1) PINOVÉ DEFINICE (Waveshare ESP32-S3-Touch-LCD-2.1)
// ============================================================
// LCD RGB panel piny
#define LCD_BL      6
#define LCD_RST     -1  // přes EXIO, zjednodušeno
#define PCLK        41
#define DE          40
#define VSYNC       39
#define HSYNC       38
#define B0          5
#define B1          45
#define B2          48
#define B3          47
#define B4          21
#define G0          14
#define G1          13
#define G2          12
#define G3          11
#define G4          10
#define G5          9
#define R0          46
#define R1          3
#define R2          8
#define R3          18
#define R4          17

// Dotyková vrstva (CST820)
#define TP_SDA      15
#define TP_SCL      7
#define TP_INT      16
#define TP_RST      -1

// Speedo a Tacho vstupy (přiveďte z motorku)
#define SENSOR_PIN  4
#define TACHO_PIN   5

// ============================================================
// 2) GLOBÁLNÍ OBJEKTY
// ============================================================

// RGB panel
Arduino_DataBus *bus = new Arduino_SWSPI(
    GFX_NOT_DEFINED, 1, 2, 1, GFX_NOT_DEFINED);  // CS, SCK, MOSI

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    DE, VSYNC, HSYNC, PCLK,
    B0, B1, B2, B3, B4,
    G0, G1, G2, G3, G4, G5,
    R0, R1, R2, R3, R4,
    1, 10, 8, 50,   // hsync polarity, front, pulse, back
    1, 10, 8, 20    // vsync polarity, front, pulse, back
);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    480, 480, rgbpanel, 0, true,
    bus, GFX_NOT_DEFINED,
    st7701_type5_init_operations,
    sizeof(st7701_type5_init_operations)
);

// Dotyková vrstva
Adafruit_CST8XX ts;

// LVGL buffery
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[480 * 40];

// LVGL objekty
static lv_obj_t *meter;
static lv_obj_t *speed_label;
static lv_obj_t *rpm_label;
static lv_obj_t *odo_label;
static lv_obj_t *trip_label;
static lv_obj_t *max_speed_label;
static lv_obj_t *max_rpm_label;
static lv_meter_indicator_t *needle;
static lv_meter_indicator_t *rpm_arc;

// ============================================================
// 3) STAV PROGRAMU (stejná logika jako původní kód)
// ============================================================

unsigned long odoMetry = 0;
unsigned long tripMetry = 0;
unsigned int zobrazenaRychlostKmh = 0;
unsigned int zobrazeneRpm = 0;
unsigned int maxRychlostKmh = 0;
unsigned int maxRpm = 0;

float kalman_x = 0, kalman_p = 1;
float tachoKalman_x = 0, tachoKalman_p = 1;

volatile unsigned long speedoIsrCasPoslednihoPulzuUs = 0;
volatile unsigned long speedoIsrDelkaPoslednihoIntervaluUs = 0;
volatile bool speedoIsrMameNovyPulz = false;

volatile unsigned long tachoIsrCasPoslednihoPulzuUs = 0;
volatile unsigned long tachoIsrDelkaPoslednihoIntervaluUs = 0;
volatile bool tachoIsrMameNovyPulz = false;

// Konstanty (stejné jako původní)
const float OBVOD_KOLA_M = 1.77;
const unsigned long MIN_MEZERA_PULZU_US = 45000UL;
const float MAX_ROZUMNA_RYCHLOST_KMH = 120.0;
const float MAX_SKOK_KMH = 25.0;
const float MAX_ROZDIL_SOUSEDNICH_PULZU_KMH = 8.0;
const unsigned long CAS_DO_ZASTAVENI_MS = 2000UL;

const unsigned long MIN_MEZERA_TACHO_US = 560UL;
const float PULZY_NA_OTACKU = 5.0;
const float MAX_ROZUMNE_RPM = 18000.0;
const float MIN_ROZUMNE_RPM = 500.0;

// Kalman konstanty
const float KALMAN_Q = 0.5;
const float KALMAN_R = 8.0;
const float TACHO_KALMAN_Q = 2.0;
const float TACHO_KALMAN_R = 8.0;

// ============================================================
// 4) PŘERUŠENÍ
// ============================================================

void IRAM_ATTR snimacSpeedoPreruseni() {
  unsigned long ted = micros();
  if (speedoIsrCasPoslednihoPulzuUs == 0) {
    speedoIsrCasPoslednihoPulzuUs = ted;
    return;
  }
  unsigned long interval = ted - speedoIsrCasPoslednihoPulzuUs;
  if (interval < MIN_MEZERA_PULZU_US) return;
  speedoIsrDelkaPoslednihoIntervaluUs = interval;
  speedoIsrCasPoslednihoPulzuUs = ted;
  speedoIsrMameNovyPulz = true;
}

void IRAM_ATTR snimacTachoPreruseni() {
  unsigned long ted = micros();
  if (tachoIsrCasPoslednihoPulzuUs == 0) {
    tachoIsrCasPoslednihoPulzuUs = ted;
    return;
  }
  unsigned long interval = ted - tachoIsrCasPoslednihoPulzuUs;
  if (interval < MIN_MEZERA_TACHO_US) return;
  tachoIsrDelkaPoslednihoIntervaluUs = interval;
  tachoIsrCasPoslednihoPulzuUs = ted;
  tachoIsrMameNovyPulz = true;
}

// ============================================================
// 5) KALMAN FILTRY
// ============================================================

float vyhladRychlost(float mereni) {
  kalman_p += KALMAN_Q;
  float zisk = kalman_p / (kalman_p + KALMAN_R);
  kalman_x += zisk * (mereni - kalman_x);
  kalman_p = (1.0 - zisk) * kalman_p;
  return kalman_x;
}

float vyhladRpm(float mereni) {
  tachoKalman_p += TACHO_KALMAN_Q;
  float zisk = tachoKalman_p / (tachoKalman_p + TACHO_KALMAN_R);
  tachoKalman_x += zisk * (mereni - tachoKalman_x);
  tachoKalman_p = (1.0 - zisk) * tachoKalman_p;
  return tachoKalman_x;
}

// ============================================================
// 6) ZPRACOVÁNÍ PULZŮ
// ============================================================

void zpracujSpeedo() {
  if (!speedoIsrMameNovyPulz) return;
  unsigned long intervalUs;
  noInterrupts();
  intervalUs = speedoIsrDelkaPoslednihoIntervaluUs;
  speedoIsrMameNovyPulz = false;
  interrupts();

  float surova = (OBVOD_KOLA_M * 3600000.0) / intervalUs;
  if (surova > MAX_ROZUMNA_RYCHLOST_KMH) return;

  float vyhlazena = vyhladRychlost(surova);
  zobrazenaRychlostKmh = (unsigned int)(vyhlazena + 0.5);

  if (zobrazenaRychlostKmh > maxRychlostKmh)
    maxRychlostKmh = zobrazenaRychlostKmh;

  // ODO akumulace
  static float zbytek = 0.0;
  zbytek += OBVOD_KOLA_M;
  if (zbytek >= 1.0) {
    unsigned long m = (unsigned long)zbytek;
    odoMetry += m;
    tripMetry += m;
    zbytek -= m;
  }
}

void zpracujTacho() {
  if (!tachoIsrMameNovyPulz) return;
  unsigned long intervalUs;
  noInterrupts();
  intervalUs = tachoIsrDelkaPoslednihoIntervaluUs;
  tachoIsrMameNovyPulz = false;
  interrupts();

  float surove = (60000000.0 / intervalUs) / PULZY_NA_OTACKU;
  if (surove > MAX_ROZUMNE_RPM || surove < MIN_ROZUMNE_RPM) return;

  float vyhlazene = vyhladRpm(surove);
  zobrazeneRpm = (unsigned int)(vyhlazene + 0.5);

  if (zobrazeneRpm > maxRpm)
    maxRpm = zobrazeneRpm;
}

// ============================================================
// 7) VYTVOŘENÍ UI (LVGL)
// ============================================================

void vytvorUI() {
  // Pozadí
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);

  // --- Kruhový tachometr ---
  meter = lv_meter_create(lv_scr_act());
  lv_obj_set_size(meter, 460, 460);
  lv_obj_center(meter);
  lv_obj_set_style_bg_color(meter, lv_color_black(), 0);
  lv_obj_set_style_border_width(meter, 0, 0);

  // Stupnice 0-110 km/h
  lv_meter_scale_t *scale = lv_meter_add_scale(meter);
  lv_meter_set_scale_ticks(meter, scale, 45, 2, 12, lv_palette_main(LV_PALETTE_GREY));
  lv_meter_set_scale_major_ticks(meter, scale, 5, 4, 20, lv_color_white(), 15);
  lv_meter_set_scale_range(meter, scale, 0, 110, 270, 135);

  // Jehla
  needle = lv_meter_add_needle_line(meter, scale, 4,
                                     lv_palette_main(LV_PALETTE_RED), -20);

  // RPM oblouk (vnitřní)
  lv_meter_scale_t *rpmScale = lv_meter_add_scale(meter);
  lv_meter_set_scale_ticks(meter, rpmScale, 0, 0, 0, lv_color_black());
  lv_meter_set_scale_range(meter, rpmScale, 0, 12000, 270, 135);
  rpm_arc = lv_meter_add_arc(meter, rpmScale, 8,
                              lv_palette_main(LV_PALETTE_BLUE), -35);

  // --- Textové labely ---
  // Rychlost (velké číslo)
  speed_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(speed_label, lv_color_white(), 0);
  lv_label_set_text(speed_label, "0");
  lv_obj_align(speed_label, LV_ALIGN_CENTER, 0, -40);

  // "km/h"
  lv_obj_t *unit = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(unit, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(unit, lv_color_white(), 0);
  lv_label_set_text(unit, "km/h");
  lv_obj_align(unit, LV_ALIGN_CENTER, 0, 10);

  // RPM
  rpm_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(rpm_label, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(rpm_label, lv_palette_main(LV_PALETTE_BLUE), 0);
  lv_label_set_text(rpm_label, "0 RPM");
  lv_obj_align(rpm_label, LV_ALIGN_CENTER, 0, 40);

  // ODO (spodní část)
  odo_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(odo_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(odo_label, lv_color_white(), 0);
  lv_label_set_text(odo_label, "ODO 0 km");
  lv_obj_align(odo_label, LV_ALIGN_BOTTOM_LEFT, 60, -40);

  // TRIP
  trip_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(trip_label, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(trip_label, lv_color_white(), 0);
  lv_label_set_text(trip_label, "TRIP 0.0 km");
  lv_obj_align(trip_label, LV_ALIGN_BOTTOM_RIGHT, -60, -40);

  // MAX rychlost
  max_speed_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(max_speed_label, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(max_speed_label, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_label_set_text(max_speed_label, "MAX 0");
  lv_obj_align(max_speed_label, LV_ALIGN_TOP_LEFT, 60, 40);

  // MAX RPM
  max_rpm_label = lv_label_create(lv_scr_act());
  lv_obj_set_style_text_font(max_rpm_label, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(max_rpm_label, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_label_set_text(max_rpm_label, "MAX 0 RPM");
  lv_obj_align(max_rpm_label, LV_ALIGN_TOP_RIGHT, -60, 40);
}

// ============================================================
// 8) AKTUALIZACE UI
// ============================================================

void aktualizujUI() {
  // Rychlost
  char buf[16];
  snprintf(buf, sizeof(buf), "%u", zobrazenaRychlostKmh);
  lv_label_set_text(speed_label, buf);
  lv_meter_set_indicator_value(meter, needle, zobrazenaRychlostKmh);

  // RPM
  snprintf(buf, sizeof(buf), "%u RPM", zobrazeneRpm);
  lv_label_set_text(rpm_label, buf);
  lv_meter_set_indicator_start_value(meter, rpm_arc, 0);
  lv_meter_set_indicator_end_value(meter, rpm_arc,
      (zobrazeneRpm > 12000) ? 12000 : zobrazeneRpm);

  // ODO
  snprintf(buf, sizeof(buf), "ODO %lu km", odoMetry / 1000);
  lv_label_set_text(odo_label, buf);

  // TRIP
  float tripKm = tripMetry / 1000.0;
  snprintf(buf, sizeof(buf), "TRIP %.1f km", tripKm);
  lv_label_set_text(trip_label, buf);

  // MAX
  snprintf(buf, sizeof(buf), "MAX %u", maxRychlostKmh);
  lv_label_set_text(max_speed_label, buf);
  snprintf(buf, sizeof(buf), "MAX %u RPM", maxRpm);
  lv_label_set_text(max_rpm_label, buf);
}

// ============================================================
// 9) LVGL TICK
// ============================================================

void lvglTickTask(void *pvParameters) {
  while (1) {
    lv_tick_inc(5);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ============================================================
// 10) SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // I2C pro dotyk
  Wire.begin(TP_SDA, TP_SCL);

  // LCD
  if (!gfx->begin()) {
    Serial.println("LCD init failed!");
  }
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  // LVGL
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, 480 * 40);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 480;
  disp_drv.ver_res = 480;
  disp_drv.flush_cb = [](lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t*)color_p, w, h);
    lv_disp_flush_ready(drv);
  };
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Tick task
  xTaskCreate(lvglTickTask, "lvgl_tick", 2048, NULL, 1, NULL);

  // Dotyk
  if (ts.begin()) {
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = [](lv_indev_drv_t *drv, lv_indev_data_t *data) {
      if (ts.touched()) {
        TS_Point p = ts.getPoint();
        data->point.x = p.x;
        data->point.y = p.y;
        data->state = LV_INDEV_STATE_PR;
      } else {
        data->state = LV_INDEV_STATE_REL;
      }
    };
    lv_indev_drv_register(&indev_drv);
  }

  // UI
  vytvorUI();

  // Přerušení pro speedo/tacho
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(TACHO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), snimacSpeedoPreruseni, FALLING);
  attachInterrupt(digitalPinToInterrupt(TACHO_PIN), snimacTachoPreruseni, FALLING);

  Serial.println("Stadi AnaloGauge ESP32-S3 ready");
}

// ============================================================
// 11) LOOP
// ============================================================

void loop() {
  zpracujSpeedo();
  zpracujTacho();

  // Zastavení
  static unsigned long posledniPulzSpeedo = 0;
  if (speedoIsrMameNovyPulz) posledniPulzSpeedo = millis();
  if (millis() - posledniPulzSpeedo > CAS_DO_ZASTAVENI_MS) {
    zobrazenaRychlostKmh = 0;
    kalman_x = 0;
  }

  // UI update (10 Hz)
  static unsigned long posledniUI = 0;
  if (millis() - posledniUI > 100) {
    aktualizujUI();
    posledniUI = millis();
  }

  lv_timer_handler();
  delay(5);
}