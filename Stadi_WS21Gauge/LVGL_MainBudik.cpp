#include "LVGL_Example.h"
#include "Wireless.h"
#include <Arduino.h>
#include <math.h>
#include <stdio.h>

/* DEMO data se zapínají přepínačem v Diagnostice. Výchozí stav: VYPNUTO. */
static bool demo_data = false;   // false = vlastní data, true = simulovaná data

/*****************************************************************************
 *  MAIN BUDIK:   
 *  GLOBÁLNÍ PROMĚNNÉ PRO DATA A FONTY
 *****************************************************************************/
float speed_kmh = 0;
float rpm       = 0;
float odo_km    = 12345.6f;
float trip_km   = 0;
float max_kmh   = 0;
float max_rpm   = 0;
float mth_h     = 123.4f;

#define PIN_SPEEDO            44        // vstup rychlosti
#define PIN_TACHO             43        // vstup otáček

/* Fonty */
#if LV_FONT_MONTSERRAT_48
  #define FONT_HUGE   (&lv_font_montserrat_48)
#else
  #define FONT_HUGE   LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_28
  #define FONT_MEDIUM (&lv_font_montserrat_28)
#else
  #define FONT_MEDIUM LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_20
  #define FONT_SMALL  (&lv_font_montserrat_20)
#else
  #define FONT_SMALL  LV_FONT_DEFAULT
#endif

/*****************************************************************************
 *  STYLY A OBJEKTY DIAGNOSTIKY (Musí být nahoře před všemi funkcemi!)
 *****************************************************************************/
lv_style_t style_text_muted;
lv_style_t style_title;

static const lv_font_t * font_large;
static const lv_font_t * font_normal;

static lv_timer_t * auto_step_timer = NULL;
static lv_timer_t * dash_timer = NULL;
static lv_timer_t * pin_timer = NULL;

/* Dashboard objekty */
static lv_obj_t * meter_speed;
static lv_meter_indicator_t * needle_speed;
static lv_obj_t * lbl_s1_speed;
static lv_obj_t * lbl_s1_rpm;
static lv_obj_t * lbl_s1_odo;
static lv_obj_t * lbl_s1_trip;
static lv_obj_t * lbl_s1_maxkmh;

static lv_obj_t * lbl_s2_rpm;
static lv_obj_t * lbl_s2_speed;
static lv_obj_t * lbl_s2_maxrpm;
static lv_obj_t * lbl_s2_mth;

/* Indikátory vstupních pinů */
static lv_obj_t * led_pin1 = NULL;
static lv_obj_t * led_pin2 = NULL;
static lv_obj_t * lbl_pin1 = NULL;
static lv_obj_t * lbl_pin2 = NULL;

/* Sekce Advanced (rozbalovací) */
static lv_obj_t * adv_panel    = NULL;
static lv_obj_t * buzzer_panel = NULL;
static lv_obj_t * adv_btn_lbl  = NULL;

/* WiFi/BLE scan - přepínač v Advanced */
static lv_obj_t * wifi_sw = NULL;
static bool wifi_scan_started = false;

/* Diagnostické objekty */
lv_obj_t * SD_Size          = NULL;
lv_obj_t * FlashSize        = NULL;
lv_obj_t * BAT_Volts        = NULL;
lv_obj_t * Board_angle      = NULL;
lv_obj_t * RTC_Time         = NULL;
lv_obj_t * Wireless_Scan    = NULL;
lv_obj_t * Backlight_slider = NULL;

/*****************************************************************************
 *  PROTOTYPY FUNKCÍ (Aby je kompilátor znal v jakémkoliv pořadí)
 *****************************************************************************/
static void Onboard_create(lv_obj_t * parent);
static void ta_event_cb(lv_event_t * e);
static void dash_update_cb(lv_timer_t * t);
static void pin_update_cb(lv_timer_t * t);
static void demo_switch_event_cb(lv_event_t * e);
static void adv_toggle_event_cb(lv_event_t * e);
static void wifi_switch_event_cb(lv_event_t * e);
void example1_increase_lvgl_tick(lv_timer_t * t);
void Backlight_adjustment_event_cb(lv_event_t * e);
void LVGL_Backlight_adjustment(uint8_t Backlight);

/*****************************************************************************
 *  HELPERZZ
 *****************************************************************************/
static lv_obj_t * make_label(lv_obj_t * parent, const lv_font_t * font, const char * txt, lv_color_t color)
{
  lv_obj_t * l = lv_label_create(parent);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, color, 0);
  lv_label_set_text(l, txt);
  return l;
}

static lv_obj_t * make_section(lv_obj_t * page)
{
  lv_obj_t * s = lv_obj_create(page);
  lv_obj_remove_style_all(s);
  lv_obj_set_size(s, LV_HOR_RES, LV_VER_RES);
  lv_obj_set_style_bg_opa(s, LV_OPA_TRANSP, 0);
  lv_obj_set_flex_flow(s, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(s, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(s, 4, 0);
  lv_obj_clear_flag(s, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(s, LV_OBJ_FLAG_SNAPPABLE);
  return s;
}


/*****************************************************************************
 *  1. FONTZZZ
 *****************************************************************************/
#if LV_FONT_MONTSERRAT_48
  #define FONT_HUGE   (&lv_font_montserrat_48)
#else
  #define FONT_HUGE   LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_28
  #define FONT_MEDIUM (&lv_font_montserrat_28)
#else
  #define FONT_MEDIUM LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_20
  #define FONT_SMALL  (&lv_font_montserrat_20)
#else
  #define FONT_SMALL  LV_FONT_DEFAULT
#endif






/*****************************************************************************
 *  2. HLAVNÍ EXAMPL
 *****************************************************************************/


void Lvgl_Example1(void){

  lv_obj_t * scr = lv_scr_act();
  lv_obj_clean(scr);

  /* Vstupní piny - aktivní při sepnutí na zem */
 // pinMode(PIN_TACHO, INPUT_PULLUP);
 // pinMode(PIN_SPEEDO, INPUT_PULLUP);

  font_normal = LV_FONT_DEFAULT;
  font_large  = FONT_SMALL;

  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  lv_style_init(&style_text_muted);
  lv_style_set_text_opa(&style_text_muted, LV_OPA_60);

  lv_style_init(&style_title);
  lv_style_set_text_font(&style_title, font_large);

  /* Hlavní stránka se svislým posunem */
  lv_obj_t * page = lv_obj_create(scr);
  lv_obj_remove_style_all(page);
  lv_obj_set_size(page, LV_HOR_RES, LV_VER_RES);
  lv_obj_set_flex_flow(page, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(page, 0, 0);
  lv_obj_set_scroll_dir(page, LV_DIR_VER);
  lv_obj_set_scroll_snap_y(page, LV_SCROLL_SNAP_START);
  lv_obj_set_scrollbar_mode(page, LV_SCROLLBAR_MODE_OFF);

  /* --- Obrazovka 1: Rychlost + Ručička (480x480) --- */
  lv_obj_t * s1 = make_section(page);
  lv_obj_set_style_pad_all(s1, 0, 0);

  meter_speed = lv_meter_create(s1);
  lv_obj_remove_style_all(meter_speed);
  lv_obj_set_size(meter_speed, 480, 480);
  lv_obj_align(meter_speed, LV_ALIGN_CENTER, 0, 0);

  /* Stupnice 0-120 km/h */
  lv_meter_scale_t * scale = lv_meter_add_scale(meter_speed);
  lv_meter_set_scale_ticks(meter_speed, scale, 25, 2, 10, lv_palette_main(LV_PALETTE_GREY));
  lv_meter_set_scale_major_ticks(meter_speed, scale, 4, 4, 18, lv_color_white(), 15);
  lv_meter_set_scale_range(meter_speed, scale, 0, 120, 220, 160);


/* Ručička - číslo posune začátek ručičky dál od středu směrem k okraji */
/* 1. Vytvoření ručičky (poslední parametr určuje konec u stupnice) */
needle_speed = lv_meter_add_needle_line(meter_speed, scale, 10, lv_palette_main(LV_PALETTE_RED), -10);

/* Černý kruh uprostřed, který zakryje střed ručičky */
lv_obj_t * center_cover = lv_obj_create(meter_speed);
lv_obj_remove_style_all(center_cover);
lv_obj_set_size(center_cover, 220, 220); // Velikost kruhu podle potřeby
lv_obj_align(center_cover, LV_ALIGN_CENTER, 0, 0);
lv_obj_set_style_bg_color(center_cover, lv_color_black(), 0);
lv_obj_set_style_bg_opa(center_cover, LV_OPA_COVER, 0);
lv_obj_set_style_radius(center_cover, LV_RADIUS_CIRCLE, 0); // Udělá z boxu kruh




  /* Kontejner pro texty uprostřed budíku */
  lv_obj_t * content_box1 = lv_obj_create(meter_speed);
  lv_obj_remove_style_all(content_box1);
  lv_obj_set_size(content_box1, 320, 220);
  lv_obj_align(content_box1, LV_ALIGN_CENTER, 0, 10);
  lv_obj_set_flex_flow(content_box1, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(content_box1, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(content_box1, 6, 0);

  /* 1. ŘÁDEK: Číslo Rychlosti + "km/h" VEDLE SEBE (Opraveno na LV_FLEX_ALIGN_END) */
  lv_obj_t * row_speed = lv_obj_create(content_box1);
  lv_obj_remove_style_all(row_speed);
  lv_obj_set_flex_flow(row_speed, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_speed, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row_speed, 6, 0);
  lv_obj_set_size(row_speed, LV_SIZE_CONTENT, LV_SIZE_CONTENT);

  lbl_s1_speed = make_label(row_speed, FONT_HUGE, "0", lv_color_white());
  make_label(row_speed, FONT_SMALL, "km/h", lv_palette_main(LV_PALETTE_GREY));

  /* 2. ŘÁDEK: Otáčky (RPM) */
  lv_obj_t * row_rpm = lv_obj_create(content_box1);
  lv_obj_remove_style_all(row_rpm);
  lv_obj_set_flex_flow(row_rpm, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_rpm, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row_rpm, 4, 0);
  lv_obj_set_size(row_rpm, LV_SIZE_CONTENT, LV_SIZE_CONTENT);

  lbl_s1_rpm = make_label(row_rpm, FONT_MEDIUM, "0", lv_color_white());
  make_label(row_rpm, FONT_SMALL, "RPM", lv_palette_main(LV_PALETTE_GREY));

  /* 3. ŘÁDEK: MAX (ODO a TRIP jsou níž, dole u okraje) */
  lv_obj_t * row_bottom1 = lv_obj_create(content_box1);
  lv_obj_remove_style_all(row_bottom1);
  lv_obj_set_flex_flow(row_bottom1, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_bottom1, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row_bottom1, 12, 0);
  lv_obj_set_size(row_bottom1, LV_SIZE_CONTENT, LV_SIZE_CONTENT);

  lbl_s1_maxkmh = make_label(row_bottom1, FONT_SMALL, "MAX 0", lv_palette_main(LV_PALETTE_GREY));

  /* ODO + TRIP dole v prázdné části stupnice (stupnice končí vlevo/vpravo dole,
     spodek je volný). Displej je kulatý: dolní okraj textu je cca 190 px pod středem,
     tam je vodorovně k dispozici ~290 px, text je široký ~230 px, takže se vejde. */
  lv_obj_t * row_odo_trip = lv_obj_create(meter_speed);
  lv_obj_remove_style_all(row_odo_trip);
  lv_obj_set_size(row_odo_trip, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_align(row_odo_trip, LV_ALIGN_BOTTOM_MID, 0, -50);
  lv_obj_set_flex_flow(row_odo_trip, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_odo_trip, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row_odo_trip, 20, 0);

  lbl_s1_odo  = make_label(row_odo_trip, FONT_SMALL, "ODO 0.0", lv_color_white());
  lbl_s1_trip = make_label(row_odo_trip, FONT_SMALL, "TRIP 0.0", lv_color_white());





  /* --- Obrazovka 2: RPM --- */
  lv_obj_t * s2 = make_section(page);

  lbl_s2_rpm = make_label(s2, FONT_HUGE, "0", lv_color_white());
  make_label(s2, FONT_SMALL, "RPM", lv_palette_main(LV_PALETTE_GREY));

  lv_obj_t * row_spd = lv_obj_create(s2);
  lv_obj_remove_style_all(row_spd);
  lv_obj_set_flex_flow(row_spd, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_spd, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_size(row_spd, LV_SIZE_CONTENT, LV_SIZE_CONTENT);

  lbl_s2_speed = make_label(row_spd, FONT_MEDIUM, "0", lv_color_white());
  make_label(row_spd, FONT_SMALL, " km/h", lv_palette_main(LV_PALETTE_GREY));

  lv_obj_t * row_bottom2 = lv_obj_create(s2);
  lv_obj_remove_style_all(row_bottom2);
  lv_obj_set_flex_flow(row_bottom2, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row_bottom2, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(row_bottom2, 8, 0);
  lv_obj_set_size(row_bottom2, LV_SIZE_CONTENT, LV_SIZE_CONTENT);

  lbl_s2_mth    = make_label(row_bottom2, FONT_SMALL, "MTH 0.0", lv_color_white());
  lbl_s2_maxrpm = make_label(row_bottom2, FONT_SMALL, "MAX 0", lv_palette_main(LV_PALETTE_GREY));

  /* --- Obrazovka 3: Diagnostika --- */
  lv_obj_t * s3 = lv_obj_create(page);
  lv_obj_remove_style_all(s3);
  lv_obj_set_size(s3, LV_HOR_RES, LV_VER_RES);
  lv_obj_set_style_pad_all(s3, 30, 0);
  lv_obj_set_scroll_dir(s3, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(s3, LV_SCROLLBAR_MODE_OFF);
  lv_obj_add_flag(s3, LV_OBJ_FLAG_SNAPPABLE);
  Onboard_create(s3);

  dash_timer = lv_timer_create(dash_update_cb, 100, NULL);
  dash_update_cb(NULL);
}







static void dash_update_cb(lv_timer_t * t)
{
  (void)t;
  if(demo_data) {
    static uint32_t tick = 0;
    tick++;
    speed_kmh = 60.0f + 55.0f * sinf(tick * 0.05f);
    rpm       = 4000.0f + 3500.0f * sinf(tick * 0.07f);
    trip_km  += speed_kmh / 36000.0f;
    odo_km   += speed_kmh / 36000.0f;
    mth_h    += 0.1f / 3600.0f;
  }

  if(speed_kmh > max_kmh) max_kmh = speed_kmh;
  if(rpm > max_rpm)       max_rpm = rpm;

  if(meter_speed && needle_speed) {
    lv_meter_set_indicator_value(meter_speed, needle_speed, (int32_t)speed_kmh);
  }

  char buf[48];

  snprintf(buf, sizeof(buf), "%d", (int)(speed_kmh + 0.5f));
  lv_label_set_text(lbl_s1_speed, buf);

  snprintf(buf, sizeof(buf), "%d", (int)(rpm + 0.5f));
  lv_label_set_text(lbl_s1_rpm, buf);

  snprintf(buf, sizeof(buf), "ODO %.1f", odo_km);
  lv_label_set_text(lbl_s1_odo, buf);

  snprintf(buf, sizeof(buf), "TRIP %.1f", trip_km);
  lv_label_set_text(lbl_s1_trip, buf);

  snprintf(buf, sizeof(buf), "MAX %d", (int)(max_kmh + 0.5f));
  lv_label_set_text(lbl_s1_maxkmh, buf);

  snprintf(buf, sizeof(buf), "%d", (int)(rpm + 0.5f));
  lv_label_set_text(lbl_s2_rpm, buf);

  snprintf(buf, sizeof(buf), "%d", (int)(speed_kmh + 0.5f));
  lv_label_set_text(lbl_s2_speed, buf);

  snprintf(buf, sizeof(buf), "MTH %.1f", mth_h);
  lv_label_set_text(lbl_s2_mth, buf);

  snprintf(buf, sizeof(buf), "MAX %d", (int)(max_rpm + 0.5f));
  lv_label_set_text(lbl_s2_maxrpm, buf);
}

void Lvgl_Example1_close(void)
{
  lv_anim_del(NULL, NULL);

  if(dash_timer)      { lv_timer_del(dash_timer);      dash_timer = NULL; }
  if(auto_step_timer) { lv_timer_del(auto_step_timer); auto_step_timer = NULL; }
  if(pin_timer)       { lv_timer_del(pin_timer);       pin_timer = NULL; }

  lv_obj_clean(lv_scr_act());

  led_pin1 = NULL;
  led_pin2 = NULL;
  lbl_pin1 = NULL;
  lbl_pin2 = NULL;
  adv_panel    = NULL;
  buzzer_panel = NULL;
  adv_btn_lbl  = NULL;
  wifi_sw      = NULL;

  lv_style_reset(&style_text_muted);
  lv_style_reset(&style_title);
}




/*****************************************************************************
 *  DIAGNOSTIKA
 *****************************************************************************/
static void led_event_cb(lv_event_t *e) {
    lv_obj_t *led = (lv_obj_t *)lv_event_get_user_data(e);
    lv_obj_t *sw = lv_event_get_target(e);

    if (lv_obj_get_state(sw) & LV_STATE_CHECKED) {
      lv_led_on(led);
      Set_EXIO(EXIO_PIN8, High);
    }
    else {
      lv_led_off(led);
      Set_EXIO(EXIO_PIN8, Low);
    }
}

/* Přepínač DEMO dat */
static void demo_switch_event_cb(lv_event_t * e)
{
  lv_obj_t * sw = lv_event_get_target(e);
  demo_data = (lv_obj_get_state(sw) & LV_STATE_CHECKED) ? true : false;

  if(!demo_data) {          /* po vypnutí vynulovat ukazatele */
    speed_kmh = 0;
    rpm       = 0;
  }
}

/* Rychlé čtení vstupních pinů (20 ms), aby se nepropásly krátké kliky */
static void pin_update_cb(lv_timer_t * t)
{
  (void)t;
  static int8_t last1 = -1;
  static int8_t last2 = -1;

  int8_t now1 = (digitalRead(PIN_TACHO) == LOW) ? 1 : 0;
  int8_t now2 = (digitalRead(PIN_SPEEDO) == LOW) ? 1 : 0;

  if(now1 != last1) {
    last1 = now1;
    if(led_pin1) {
      if(now1) lv_led_on(led_pin1); else lv_led_off(led_pin1);
    }
    if(lbl_pin1) lv_label_set_text(lbl_pin1, now1 ? "GPIO33/RPM: GND" : "GPIO33/RPM: ---");
  }

  if(now2 != last2) {
    last2 = now2;
    if(led_pin2) {
      if(now2) lv_led_on(led_pin2); else lv_led_off(led_pin2);
    }
    if(lbl_pin2) lv_label_set_text(lbl_pin2, now2 ? "GPIO34/KMH: GND" : "GPIO34/KMH: ---");
  }
}

/* Přepínač WiFi/BLE scanu: zapnutí spustí jeden scan (task sám na konci
   vypne rádio). Po dokončení se přepínač sám vrátí do polohy vypnuto. */
static void wifi_switch_event_cb(lv_event_t * e)
{
  lv_obj_t * sw = lv_event_get_target(e);

  if(!(lv_obj_get_state(sw) & LV_STATE_CHECKED)) return;
  if(wifi_scan_started) return;

  wifi_scan_started = true;
  Scan_finish = 0;
  WIFI_NUM = 0;
  BLE_NUM = 0;
  lv_obj_add_state(sw, LV_STATE_DISABLED);   /* během scanu nejde přepnout */
  Wireless_Test2();
}

/* Rozbalení / sbalení sekce Advanced */
static void adv_toggle_event_cb(lv_event_t * e)
{
  lv_obj_t * btn = lv_event_get_target(e);
  bool open = (lv_obj_get_state(btn) & LV_STATE_CHECKED) ? true : false;

  if(adv_panel) {
    if(open) lv_obj_clear_flag(adv_panel, LV_OBJ_FLAG_HIDDEN);
    else     lv_obj_add_flag(adv_panel, LV_OBJ_FLAG_HIDDEN);
  }
  if(buzzer_panel) {
    if(open) lv_obj_clear_flag(buzzer_panel, LV_OBJ_FLAG_HIDDEN);
    else     lv_obj_add_flag(buzzer_panel, LV_OBJ_FLAG_HIDDEN);
  }
  if(adv_btn_lbl) {
    lv_label_set_text(adv_btn_lbl, open ? "Advanced" LV_SYMBOL_UP : "Advanced " LV_SYMBOL_DOWN);
  }
}

static void Onboard_create(lv_obj_t * parent)
{
  lv_obj_t * panel1 = lv_obj_create(parent);
  lv_obj_set_height(panel1, LV_SIZE_CONTENT);

  lv_obj_t * panel1_title = lv_label_create(panel1);
  lv_label_set_text(panel1_title, "Settings");
  lv_obj_add_style(panel1_title, &style_title, 0);

  /* 1. BACKLIGHT SLIDER */
  lv_obj_t * Backlight_label = lv_label_create(panel1);
  lv_label_set_text(Backlight_label, "Backlight");
  lv_obj_add_style(Backlight_label, &style_text_muted, 0);

  Backlight_slider = lv_slider_create(panel1);
  lv_obj_add_flag(Backlight_slider, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_size(Backlight_slider, 200, 35);
  lv_obj_set_style_radius(Backlight_slider, 3, LV_PART_KNOB);
  lv_obj_set_style_bg_opa(Backlight_slider, LV_OPA_TRANSP, LV_PART_KNOB);
  lv_obj_set_style_bg_color(Backlight_slider, lv_color_hex(0xAAAAAA), LV_PART_KNOB);
  lv_obj_set_style_bg_color(Backlight_slider, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
  lv_obj_set_style_outline_width(Backlight_slider, 2, LV_PART_INDICATOR);
  lv_obj_set_style_outline_color(Backlight_slider, lv_color_hex(0xD3D3D3), LV_PART_INDICATOR);
  lv_slider_set_range(Backlight_slider, 5, Backlight_MAX);
  lv_slider_set_value(Backlight_slider, LCD_Backlight, LV_ANIM_ON);
  lv_obj_add_event_cb(Backlight_slider, Backlight_adjustment_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  /* 2. SD CARD */
  lv_obj_t * SD_label = lv_label_create(panel1);
  lv_label_set_text(SD_label, "SD Card");
  lv_obj_add_style(SD_label, &style_text_muted, 0);

  SD_Size = lv_textarea_create(panel1);
  lv_textarea_set_one_line(SD_Size, true);
  lv_textarea_set_placeholder_text(SD_Size, "SD Size");
  lv_obj_add_event_cb(SD_Size, ta_event_cb, LV_EVENT_ALL, NULL);

  /* 3. FLASH SIZE */
  lv_obj_t * Flash_label = lv_label_create(panel1);
  lv_label_set_text(Flash_label, "Flash Size");
  lv_obj_add_style(Flash_label, &style_text_muted, 0);

  FlashSize = lv_textarea_create(panel1);
  lv_textarea_set_one_line(FlashSize, true);
  lv_textarea_set_placeholder_text(FlashSize, "Flash Size");
  lv_obj_add_event_cb(FlashSize, ta_event_cb, LV_EVENT_ALL, NULL);

  /* 4. BATTERY VOLTAGE */
  lv_obj_t * BAT_label = lv_label_create(panel1);
  lv_label_set_text(BAT_label, "Battery Voltage");
  lv_obj_add_style(BAT_label, &style_text_muted, 0);

  BAT_Volts = lv_textarea_create(panel1);
  lv_textarea_set_one_line(BAT_Volts, true);
  lv_textarea_set_placeholder_text(BAT_Volts, "BAT Volts");
  lv_obj_add_event_cb(BAT_Volts, ta_event_cb, LV_EVENT_ALL, NULL);

  /* 5. ANGULAR DEFLECTION */
  lv_obj_t * angle_label = lv_label_create(panel1);
  lv_label_set_text(angle_label, "Angular deflection");
  lv_obj_add_style(angle_label, &style_text_muted, 0);

  Board_angle = lv_textarea_create(panel1);
  lv_textarea_set_one_line(Board_angle, true);
  lv_textarea_set_placeholder_text(Board_angle, "Board angle");
  lv_obj_add_event_cb(Board_angle, ta_event_cb, LV_EVENT_ALL, NULL);

  /* 6. RTC TIME */
  lv_obj_t * Time_label = lv_label_create(panel1);
  lv_label_set_text(Time_label, "RTC Time");
  lv_obj_add_style(Time_label, &style_text_muted, 0);

  RTC_Time = lv_textarea_create(panel1);
  lv_textarea_set_one_line(RTC_Time, true);
  lv_textarea_set_placeholder_text(RTC_Time, "Display time");
  lv_obj_add_event_cb(RTC_Time, ta_event_cb, LV_EVENT_ALL, NULL);

  /* 7. WIRELESS SCAN */
  lv_obj_t * Wireless_label = lv_label_create(panel1);
  lv_label_set_text(Wireless_label, "Wireless scan");
  lv_obj_add_style(Wireless_label, &style_text_muted, 0);

  Wireless_Scan = lv_textarea_create(panel1);
  lv_textarea_set_one_line(Wireless_Scan, true);
  lv_textarea_set_placeholder_text(Wireless_Scan, "Wireless number");
  lv_obj_add_event_cb(Wireless_Scan, ta_event_cb, LV_EVENT_ALL, NULL);

  /* PANEL 2 - BUZZER TEST */
  lv_obj_t * panel2 = lv_obj_create(parent);
  lv_obj_set_height(panel2, LV_SIZE_CONTENT);

  lv_obj_t * panel2_title = lv_label_create(panel2);
  lv_label_set_text(panel2_title, "Buzzerant test");
  lv_obj_add_style(panel2_title, &style_title, 0);

  lv_obj_t *led = lv_led_create(panel2);
  lv_obj_set_size(led, 50, 50);
  lv_obj_align(led, LV_ALIGN_CENTER, -60, 0);
  lv_led_off(led);

  lv_obj_t *sw = lv_switch_create(panel2);
  lv_obj_set_size(sw, 65, 40);
  lv_obj_align(sw, LV_ALIGN_CENTER, 60, 0);
  lv_obj_add_event_cb(sw, led_event_cb, LV_EVENT_VALUE_CHANGED, led);

  /* PANEL 3 - VSTUPNI PINY (GPIO33 / GPIO34) */
  lv_obj_t * panel3 = lv_obj_create(parent);
  lv_obj_set_height(panel3, LV_SIZE_CONTENT);

  lv_obj_t * panel3_title = lv_label_create(panel3);
  lv_label_set_text(panel3_title, "Vstupy");
  lv_obj_add_style(panel3_title, &style_title, 0);

  led_pin1 = lv_led_create(panel3);
  lv_obj_set_size(led_pin1, 40, 40);
  lv_led_set_color(led_pin1, lv_palette_main(LV_PALETTE_GREEN));
  lv_led_off(led_pin1);

  led_pin2 = lv_led_create(panel3);
  lv_obj_set_size(led_pin2, 40, 40);
  lv_led_set_color(led_pin2, lv_palette_main(LV_PALETTE_GREEN));
  lv_led_off(led_pin2);

  lbl_pin1 = lv_label_create(panel3);
  lv_label_set_text(lbl_pin1, "GPIO33: ---");

  lbl_pin2 = lv_label_create(panel3);
  lv_label_set_text(lbl_pin2, "GPIO34: ---");

  /* PANEL 4 - DEMO DATA (výchozí stav: vypnuto) */
  lv_obj_t * panel4 = lv_obj_create(parent);
  lv_obj_set_height(panel4, LV_SIZE_CONTENT);

  lv_obj_t * panel4_title = lv_label_create(panel4);
  lv_label_set_text(panel4_title, "Demo data");
  lv_obj_add_style(panel4_title, &style_title, 0);

  lv_obj_t * demo_label = lv_label_create(panel4);
  lv_label_set_text(demo_label, " Km/h & RPM");

  lv_obj_t * demo_sw = lv_switch_create(panel4);
  lv_obj_set_size(demo_sw, 65, 40);
  if(demo_data) lv_obj_add_state(demo_sw, LV_STATE_CHECKED);
  lv_obj_add_event_cb(demo_sw, demo_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  /* SEKCE ADVANCED - původní diagnostika (SD ... Wireless) přesunutá do vlastního
     skrytého panelu; Buzzerant test se skrývá spolu s ní. */
  lv_obj_t * adv = lv_obj_create(parent);
  lv_obj_set_height(adv, LV_SIZE_CONTENT);

  /* WiFi/BLE scan - výchozí stav: vypnuto */
  lv_obj_t * wifi_label = lv_label_create(adv);
  lv_label_set_text(wifi_label, "WiFi/BLE scan");
  lv_obj_add_style(wifi_label, &style_text_muted, 0);

  wifi_sw = lv_switch_create(adv);
  lv_obj_set_size(wifi_sw, 65, 40);
  lv_obj_add_event_cb(wifi_sw, wifi_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_set_parent(SD_label, adv);
  lv_obj_set_parent(SD_Size, adv);
  lv_obj_set_parent(Flash_label, adv);
  lv_obj_set_parent(FlashSize, adv);
  lv_obj_set_parent(BAT_label, adv);
  lv_obj_set_parent(BAT_Volts, adv);
  lv_obj_set_parent(angle_label, adv);
  lv_obj_set_parent(Board_angle, adv);
  lv_obj_set_parent(Time_label, adv);
  lv_obj_set_parent(RTC_Time, adv);
  lv_obj_set_parent(Wireless_label, adv);
  lv_obj_set_parent(Wireless_Scan, adv);

  adv_panel    = adv;
  buzzer_panel = panel2;
  lv_obj_add_flag(adv_panel, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(buzzer_panel, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t * adv_btn = lv_btn_create(parent);
  lv_obj_add_flag(adv_btn, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(adv_btn, 220, 45);
  adv_btn_lbl = lv_label_create(adv_btn);
  lv_label_set_text(adv_btn_lbl, "Advanced " LV_SYMBOL_DOWN);
  lv_obj_center(adv_btn_lbl);
  lv_obj_add_event_cb(adv_btn, adv_toggle_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

  /* GRID MŘÍŽKA - Přeuspořádané řádky pro Grid */
  static lv_coord_t grid_main_col_dsc[] = {LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
  /* Řádky: 0 Diagnostika+Backlight, 1 Vstupy, 2 Demo data, 3 tlačítko Advanced, 4 Advanced panel, 5 Buzzerant test */
  static lv_coord_t grid_main_row_dsc[] = {LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};

  static lv_coord_t grid_1_col_dsc[] = {LV_GRID_FR(4), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(4), LV_GRID_TEMPLATE_LAST};
  static lv_coord_t grid_1_row_dsc[] = {
    LV_GRID_CONTENT,
    LV_GRID_CONTENT,
    LV_GRID_CONTENT,
    LV_GRID_CONTENT,
    LV_GRID_TEMPLATE_LAST
  };

  static lv_coord_t grid_2_col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(5), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
  static lv_coord_t grid_2_row_dsc[] = {
    LV_GRID_CONTENT,  /* Title */
    5,                /* Separator */
    LV_GRID_CONTENT,  /* Backlight title */
    40,               /* Backlight slider */
    LV_GRID_TEMPLATE_LAST
  };

  /* Grid sekce Advanced (stejné řádky jako dřív, jen začínají od 0) */
  static lv_coord_t grid_adv_row_dsc[] = {
    LV_GRID_CONTENT,  /* SD title */
    40,               /* SD Box */
    LV_GRID_CONTENT,  /* Flash title */
    40,               /* Flash Box */
    LV_GRID_CONTENT,  /* BAT title */
    40,               /* BAT Box */
    LV_GRID_CONTENT,  /* Angle title */
    40,               /* Angle Box */
    LV_GRID_CONTENT,  /* Time title */
    40,               /* Time Box */
    LV_GRID_CONTENT,  /* Wireless title */
    40,               /* Wireless Box */
    LV_GRID_CONTENT,  /* WiFi/BLE scan title */
    40,               /* WiFi/BLE scan switch */
    LV_GRID_TEMPLATE_LAST
  };

  /* Grid pro panel Vstupy: 2 sloupce (LED + popisek pod ní) */
  static lv_coord_t grid_3_col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
  static lv_coord_t grid_3_row_dsc[] = {LV_GRID_CONTENT, 50, LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};

  /* Grid pro panel Demo data: popisek vlevo, přepínač vpravo */
  static lv_coord_t grid_4_col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
  static lv_coord_t grid_4_row_dsc[] = {LV_GRID_CONTENT, 50, LV_GRID_TEMPLATE_LAST};

  lv_obj_set_grid_dsc_array(parent, grid_main_col_dsc, grid_main_row_dsc);

  lv_obj_set_grid_cell(panel1, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_START, 0, 1);
  lv_obj_set_grid_dsc_array(panel1, grid_2_col_dsc, grid_2_row_dsc);

  lv_obj_set_grid_cell(panel1_title, LV_GRID_ALIGN_CENTER, 1, 1, LV_GRID_ALIGN_CENTER, 0, 1);

  /* Backlight jako 1. položka v Gridu (řádky 2 a 3) */
  lv_obj_set_grid_cell(Backlight_label, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_START, 2, 1);
  lv_obj_set_grid_cell(Backlight_slider, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_CENTER, 3, 1);

  /* Panel Advanced (4. řádek hlavního gridu) - vnitřní řádky 0 až 11 */
  lv_obj_set_grid_cell(adv, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_START, 4, 1);
  lv_obj_set_grid_dsc_array(adv, grid_2_col_dsc, grid_adv_row_dsc);

  /* SD Card (řádky 0 a 1) */
  lv_obj_set_grid_cell(SD_label, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_START, 0, 1);
  lv_obj_set_grid_cell(SD_Size, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_CENTER, 1, 1);

  /* Flash (řádky 2 a 3) */
  lv_obj_set_grid_cell(Flash_label, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_START, 2, 1);
  lv_obj_set_grid_cell(FlashSize, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_CENTER, 3, 1);

  /* BAT (řádky 4 a 5) */
  lv_obj_set_grid_cell(BAT_label, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_START, 4, 1);
  lv_obj_set_grid_cell(BAT_Volts, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_CENTER, 5, 1);

  /* Angle (řádky 6 a 7) */
  lv_obj_set_grid_cell(angle_label, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_START, 6, 1);
  lv_obj_set_grid_cell(Board_angle, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_CENTER, 7, 1);

  /* RTC (řádky 8 a 9) */
  lv_obj_set_grid_cell(Time_label, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_START, 8, 1);
  lv_obj_set_grid_cell(RTC_Time, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_CENTER, 9, 1);

  /* Wireless (řádky 10 a 11) */
  lv_obj_set_grid_cell(Wireless_label, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_START, 10, 1);
  lv_obj_set_grid_cell(Wireless_Scan, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_CENTER, 11, 1);

  /* WiFi/BLE scan přepínač (řádky 12 a 13) */
  lv_obj_set_grid_cell(wifi_label, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_START, 12, 1);
  lv_obj_set_grid_cell(wifi_sw, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_CENTER, 13, 1);

  /* Tlačítko Advanced (3. řádek hlavního gridu) */
  lv_obj_set_grid_cell(adv_btn, LV_GRID_ALIGN_CENTER, 0, 1, LV_GRID_ALIGN_CENTER, 3, 1);

  /* Buzzerant test (posledni radek hlavniho gridu, skryty v Advanced) */
  lv_obj_set_grid_cell(panel2, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 5, 1);
  lv_obj_set_grid_dsc_array(panel2, grid_1_col_dsc, grid_1_row_dsc);
  lv_obj_set_grid_cell(panel2_title, LV_GRID_ALIGN_CENTER, 2, 1, LV_GRID_ALIGN_CENTER, 0, 1);
  lv_obj_set_grid_cell(led, LV_GRID_ALIGN_CENTER, 1, 1, LV_GRID_ALIGN_CENTER, 2, 1);
  lv_obj_set_grid_cell(sw, LV_GRID_ALIGN_CENTER, 3, 1, LV_GRID_ALIGN_CENTER, 2, 1);

  /* Panel Vstupy (3. řádek hlavního gridu) */
  lv_obj_set_grid_cell(panel3, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
  lv_obj_set_grid_dsc_array(panel3, grid_3_col_dsc, grid_3_row_dsc);
  lv_obj_set_grid_cell(panel3_title, LV_GRID_ALIGN_CENTER, 0, 2, LV_GRID_ALIGN_CENTER, 0, 1);
  lv_obj_set_grid_cell(led_pin1, LV_GRID_ALIGN_CENTER, 0, 1, LV_GRID_ALIGN_CENTER, 1, 1);
  lv_obj_set_grid_cell(led_pin2, LV_GRID_ALIGN_CENTER, 1, 1, LV_GRID_ALIGN_CENTER, 1, 1);
  lv_obj_set_grid_cell(lbl_pin1, LV_GRID_ALIGN_CENTER, 0, 1, LV_GRID_ALIGN_CENTER, 2, 1);
  lv_obj_set_grid_cell(lbl_pin2, LV_GRID_ALIGN_CENTER, 1, 1, LV_GRID_ALIGN_CENTER, 2, 1);

  /* Panel Demo data (4. řádek hlavního gridu) */
  lv_obj_set_grid_cell(panel4, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 2, 1);
  lv_obj_set_grid_dsc_array(panel4, grid_4_col_dsc, grid_4_row_dsc);
  lv_obj_set_grid_cell(panel4_title, LV_GRID_ALIGN_CENTER, 0, 2, LV_GRID_ALIGN_CENTER, 0, 1);
  lv_obj_set_grid_cell(demo_label, LV_GRID_ALIGN_CENTER, 0, 1, LV_GRID_ALIGN_CENTER, 1, 1);
  lv_obj_set_grid_cell(demo_sw, LV_GRID_ALIGN_CENTER, 1, 1, LV_GRID_ALIGN_CENTER, 1, 1);

  auto_step_timer = lv_timer_create(example1_increase_lvgl_tick, 100, NULL);
  pin_timer = lv_timer_create(pin_update_cb, 20, NULL);
}

void example1_increase_lvgl_tick(lv_timer_t * t)
{
  char buf[100];

  if(SD_Size) {
    snprintf(buf, sizeof(buf), "%d MB\r\n", SDCard_Size);
    lv_textarea_set_placeholder_text(SD_Size, buf);
  }
  if(FlashSize) {
    snprintf(buf, sizeof(buf), "%d MB\r\n", Flash_Size);
    lv_textarea_set_placeholder_text(FlashSize, buf);
  }
  if(BAT_Volts) {
    snprintf(buf, sizeof(buf), "%.2f V\r\n", BAT_analogVolts);
    lv_textarea_set_placeholder_text(BAT_Volts, buf);
  }
  if(Board_angle) {
    snprintf(buf, sizeof(buf), "X:%.2f  Y:%.2f  Z:%.2f\r\n", Accel.x, Accel.y, Accel.z);
    lv_textarea_set_placeholder_text(Board_angle, buf);
  }
  if(RTC_Time) {
    snprintf(buf, sizeof(buf), "%d.%d.%d   %d:%d:%d\r\n", datetime.year, datetime.month, datetime.day, datetime.hour, datetime.minute, datetime.second);
    lv_textarea_set_placeholder_text(RTC_Time, buf);
  }
  if(Wireless_Scan) {
    if(Scan_finish)
      snprintf(buf, sizeof(buf), "WIFI: %d    BLE: %d    ..Scan Finish.\r\n", WIFI_NUM, BLE_NUM);
    else
      snprintf(buf, sizeof(buf), "WIFI: %d    BLE: %d\r\n", WIFI_NUM, BLE_NUM);
    lv_textarea_set_placeholder_text(Wireless_Scan, buf);
  }
  if(Backlight_slider) {
    lv_slider_set_value(Backlight_slider, LCD_Backlight, LV_ANIM_ON);
  }
  /* Scan doběhl -> přepínač zpět do polohy vypnuto */
  if(wifi_sw && wifi_scan_started && Scan_finish) {
    wifi_scan_started = false;
    lv_obj_clear_state(wifi_sw, LV_STATE_CHECKED | LV_STATE_DISABLED);
  }

  LVGL_Backlight_adjustment(LCD_Backlight);
}

static void ta_event_cb(lv_event_t * e)
{
}

void Backlight_adjustment_event_cb(lv_event_t * e) {
  uint8_t Backlight = lv_slider_get_value((lv_obj_t *)lv_event_get_target(e));
  if (Backlight >= 0 && Backlight <= Backlight_MAX)  {
    if(Backlight_slider) {
      lv_slider_set_value(Backlight_slider, Backlight, LV_ANIM_ON);
    }
    LCD_Backlight = Backlight;
    LVGL_Backlight_adjustment(Backlight);
  }
  else
    printf("Backlight out of range: %d\n", Backlight);
}

void LVGL_Backlight_adjustment(uint8_t Backlight) {
  Set_Backlight(Backlight);
}