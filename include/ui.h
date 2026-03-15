#pragma once
// ============================================================================
//  ui.h — LVGL UI: helpers, screens, event handlers, update functions
// ============================================================================

#include "config.h"
#include "theme.h"

// --- Forward declarations for functions defined in other modules ---
bool startRunBetweenEndpoints();
void requestGracefulStop();
bool calibrateEndpointsSensorless();
void recomputeEffectiveEndpoints();
void safeCreepHome();
void clearWiFiCredentials();
void webLog(const char *fmt, ...);

// ==========================================================================
//  LVGL UI HELPERS
// ==========================================================================
void go(lv_obj_t *scr) { lv_scr_load(scr); }

void style_screen(lv_obj_t *scr) {
  lv_obj_set_style_bg_color(scr, lv_color_hex(Theme::BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
}

lv_obj_t* make_content(lv_obj_t *scr) {
  lv_obj_t *c = lv_obj_create(scr);
  lv_obj_set_size(c, SCR_W, CONTENT_H);
  lv_obj_align(c, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_opa(c, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(c, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(c, 10, LV_PART_MAIN);
  lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(c, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(c, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_row(c, 10, LV_PART_MAIN);
  return c;
}

lv_obj_t* make_content_free(lv_obj_t *scr) {
  lv_obj_t *c = lv_obj_create(scr);
  lv_obj_set_size(c, SCR_W, CONTENT_H);
  lv_obj_align(c, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_opa(c, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(c, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(c, 0, LV_PART_MAIN);
  lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
  return c;
}

lv_obj_t* make_nav(lv_obj_t *scr) {
  lv_obj_t *n = lv_obj_create(scr);
  lv_obj_set_size(n, SCR_W, NAV_H);
  lv_obj_align(n, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(n, lv_color_hex(Theme::NAV_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(n, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(n, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(n, 10, LV_PART_MAIN);
  lv_obj_clear_flag(n, LV_OBJ_FLAG_SCROLLABLE);
  return n;
}

lv_obj_t* make_title(lv_obj_t *p, const char *txt) {
  lv_obj_t *t = lv_label_create(p);
  lv_label_set_text(t, txt);
  lv_obj_set_style_text_font(t, &lv_font_montserrat_26, LV_PART_MAIN);
  lv_obj_set_style_text_color(t, lv_color_hex(Theme::TEXT), LV_PART_MAIN);
  return t;
}

lv_obj_t* make_btn(lv_obj_t *p, const char *txt, int w, int h, uint32_t bg, const lv_font_t *f) {
  lv_obj_t *b = lv_btn_create(p);
  lv_obj_set_size(b, w, h);
  lv_obj_set_style_bg_color(b, lv_color_hex(bg), LV_PART_MAIN);
  lv_obj_t *l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_color(l, lv_color_hex(Theme::TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(l, f, LV_PART_MAIN);
  lv_obj_center(l);
  return b;
}

lv_obj_t* make_btn_multiline(lv_obj_t *p, const char *txt, int w, int h, uint32_t bg, const lv_font_t *f) {
  lv_obj_t *b = lv_btn_create(p);
  lv_obj_set_size(b, w, h);
  lv_obj_set_style_bg_color(b, lv_color_hex(bg), LV_PART_MAIN);
  lv_obj_t *l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(l, w - 8);
  lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(l, lv_color_hex(Theme::TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(l, f, LV_PART_MAIN);
  lv_obj_center(l);
  return b;
}

lv_obj_t* make_card(lv_obj_t *p, int w, int h) {
  lv_obj_t *c = lv_obj_create(p);
  lv_obj_set_size(c, w, h);
  lv_obj_set_style_bg_color(c, lv_color_hex(Theme::CARD), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(c, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(c, 10, LV_PART_MAIN);
  lv_obj_set_style_border_width(c, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(c, 10, LV_PART_MAIN);
  lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
  return c;
}

// ==========================================================================
//  JAM SCREEN FUNCTIONS
// ==========================================================================
void showJamScreen() {
  if (jam_status_lbl) lv_label_set_text(jam_status_lbl, "Press to return home");
  go(jam_scr);
}

void onJamReturnHome(lv_event_t *e) {
  LV_UNUSED(e);
  if (runState != STALLED) return;
  safeCreepHome();
}

// ==========================================================================
//  UI UPDATE FUNCTIONS
// ==========================================================================
void ui_update_main_warning() {
  if (!main_warn) return;
  if (endpointsCalibrated) lv_obj_add_flag(main_warn, LV_OBJ_FLAG_HIDDEN);
  else lv_obj_clear_flag(main_warn, LV_OBJ_FLAG_HIDDEN);
}

void ui_create_main_warning(lv_obj_t *parent) {
  main_warn = lv_obj_create(parent);
  lv_obj_set_size(main_warn, 132, 24);
  lv_obj_set_style_radius(main_warn, 12, LV_PART_MAIN);
  lv_obj_set_style_border_width(main_warn, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(main_warn, lv_color_hex(Theme::WARN_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(main_warn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_pad_all(main_warn, 0, LV_PART_MAIN);
  lv_obj_clear_flag(main_warn, LV_OBJ_FLAG_SCROLLABLE);
  main_warn_lbl = lv_label_create(main_warn);
  lv_label_set_text(main_warn_lbl, "NOT CALIBRATED");
  lv_obj_set_style_text_font(main_warn_lbl, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_color(main_warn_lbl, lv_color_hex(Theme::WARN_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_align(main_warn_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(main_warn_lbl);
  ui_update_main_warning();
}

void ui_update_speed_val() {
  if (lbl_speed_val) lv_label_set_text_fmt(lbl_speed_val, "%s  %lukHz",
      profiles[activeProfile].name, (unsigned long)(ui_speed_hz / 1000));
}

void ui_update_profile_screen();  // forward decl (defined below)
void ui_update_sg_val();          // forward decl (defined below)

void setActiveProfile(uint8_t idx) {
  if (idx >= NUM_PROFILES) return;
  activeProfile = idx;
  if (stepper) {
    stepper->setSpeedInHz(ui_speed_hz);
  }
  ui_update_speed_val();
  ui_update_sg_val();
  ui_update_profile_screen();
  webLog("Profile: %s spd=%lu sg=%u", profiles[idx].name,
         (unsigned long)ui_speed_hz, RUN_SG_TRIP);
}

void ui_update_tuning_numbers() {
  if (!endpointsCalibrated) {
    if (lbl_ep_up)  lv_label_set_text(lbl_ep_up, "UP: -");
    if (lbl_ep_dn)  lv_label_set_text(lbl_ep_dn, "DOWN: -");
    if (lbl_travel) lv_label_set_text(lbl_travel, "TRAVEL: -");
    if (lbl_up_eff) lv_label_set_text(lbl_up_eff, "Eff UP: -");
    if (lbl_dn_eff) lv_label_set_text(lbl_dn_eff, "Eff DN: -");
    return;
  }
  if (lbl_ep_up)  lv_label_set_text_fmt(lbl_ep_up, "UP: %ld", rawUp);
  if (lbl_ep_dn)  lv_label_set_text_fmt(lbl_ep_dn, "DOWN: %ld", rawDown);
  if (lbl_travel) lv_label_set_text_fmt(lbl_travel, "TRAVEL: %ld", rawDown - rawUp);
  if (lbl_up_eff) lv_label_set_text_fmt(lbl_up_eff, "UP: %ld (%+ld)", endpointUp, (long)upOffsetSteps);
  if (lbl_dn_eff) lv_label_set_text_fmt(lbl_dn_eff, "DN: %ld (%+ld)", endpointDown, (long)downOffsetSteps);
}

void ui_update_endpoint_edit_values() {
  if (lbl_ep_up_val) lv_label_set_text_fmt(lbl_ep_up_val, "%ld", endpointUp);
  if (lbl_ep_dn_val) lv_label_set_text_fmt(lbl_ep_dn_val, "%ld", endpointDown);
}

void ui_update_sg_val() {
  if (lbl_sg_val) lv_label_set_text_fmt(lbl_sg_val, "%u", RUN_SG_TRIP);
}

void ui_update_profile_screen() {
  for (uint8_t i = 0; i < NUM_PROFILES; i++) {
    if (!profile_btns[i]) continue;
    if (i == activeProfile) {
      lv_obj_set_style_bg_color(profile_btns[i], lv_color_hex(Theme::ACCENT), LV_PART_MAIN);
      lv_obj_set_style_border_width(profile_btns[i], 2, LV_PART_MAIN);
      lv_obj_set_style_border_color(profile_btns[i], lv_color_hex(Theme::GREEN), LV_PART_MAIN);
    } else {
      lv_obj_set_style_bg_color(profile_btns[i], lv_color_hex(Theme::BTN_DARK), LV_PART_MAIN);
      lv_obj_set_style_border_width(profile_btns[i], 0, LV_PART_MAIN);
    }
  }
  if (lbl_profile_info) {
    lv_label_set_text_fmt(lbl_profile_info, "%luHz  SG=%u",
        (unsigned long)ui_speed_hz, RUN_SG_TRIP);
  }
}

void ui_update_batch_val() {
  if (lbl_batch_val) {
    if (batchTarget <= 0)
      lv_label_set_text(lbl_batch_val, "OFF");
    else
      lv_label_set_text_fmt(lbl_batch_val, "%ld", batchTarget);
  }
}

void ui_update_batch_remain() {
  if (!lbl_batch_remain) return;
  if (batchActive && batchTarget > 0) {
    int32_t remain = batchTarget - batchCount;
    if (remain < 0) remain = 0;
    lv_label_set_text_fmt(lbl_batch_remain, "Batch: %ld left", remain);
    lv_obj_clear_flag(lbl_batch_remain, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(lbl_batch_remain, LV_OBJ_FLAG_HIDDEN);
  }
}

void ui_update_wifi_label() {
  if (!lbl_wifi_status) return;
  if (wifiConnected)
    lv_label_set_text_fmt(lbl_wifi_status, "%s\nIP: %s", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  else if (wifiAPMode)
    lv_label_set_text_fmt(lbl_wifi_status, "AP: %s\n192.168.4.1\n(open, no password)", DEFAULT_AP_SSID);
  else
    lv_label_set_text(lbl_wifi_status, "Disconnected");
}

void setRunButtonState(bool running) {
  if (!btn_run_global) return;
  lv_obj_t *l = lv_obj_get_child(btn_run_global, 0);
  if (running) {
    lv_obj_set_style_bg_color(btn_run_global, lv_color_hex(Theme::RED), LV_PART_MAIN);
    lv_label_set_text(l, "STOP");
    lv_obj_set_style_text_color(l, lv_color_hex(Theme::TEXT), LV_PART_MAIN);
  } else {
    lv_obj_set_style_bg_color(btn_run_global, lv_color_hex(Theme::GREEN), LV_PART_MAIN);
    lv_label_set_text(l, "RUN");
    lv_obj_set_style_text_color(l, lv_color_hex(Theme::BG), LV_PART_MAIN);
  }
}

// ==========================================================================
//  UI EVENT HANDLERS
// ==========================================================================
void on_ep_up_delta(lv_event_t *e) {
  if (!endpointsCalibrated) return;
  int32_t d = (int32_t)(intptr_t)lv_event_get_user_data(e);
  upOffsetSteps = clamp_i32(upOffsetSteps + d, OFFSET_MIN, OFFSET_MAX);
  recomputeEffectiveEndpoints();
  ui_update_endpoint_edit_values();
  ui_update_tuning_numbers();
}

void on_ep_dn_delta(lv_event_t *e) {
  if (!endpointsCalibrated) return;
  int32_t d = (int32_t)(intptr_t)lv_event_get_user_data(e);
  downOffsetSteps = clamp_i32(downOffsetSteps + d, OFFSET_MIN, OFFSET_MAX);
  recomputeEffectiveEndpoints();
  ui_update_endpoint_edit_values();
  ui_update_tuning_numbers();
}

void build_endpoint_screen(lv_obj_t *scr, const char *titleTxt, bool isUp) {
  style_screen(scr);
  lv_obj_t *content = make_content(scr);
  lv_obj_t *nav = make_nav(scr);
  lv_obj_t *t = make_title(content, titleTxt);
  lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 2);

  lv_obj_t *card = make_card(content, 150, 92);
  lv_obj_t *ln = lv_label_create(card);
  lv_label_set_text(ln, "Steps");
  lv_obj_set_style_text_color(ln, lv_color_hex(Theme::MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(ln, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(ln, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *lv = lv_label_create(card);
  lv_obj_set_style_text_color(lv, lv_color_hex(Theme::GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(lv, &lv_font_montserrat_26, LV_PART_MAIN);
  lv_obj_center(lv);
  if (isUp) lbl_ep_up_val = lv; else lbl_ep_dn_val = lv;

  lv_obj_t *grid = lv_obj_create(content);
  lv_obj_set_size(grid, 150, 110);
  lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(grid, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(grid, 0, LV_PART_MAIN);
  lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);

  const int bw=48, bh=44, gap=3;
  const int x0=0, x1=bw+gap, x2=2*(bw+gap), y0=0, y1=bh+gap;
  lv_obj_t *bN100=make_btn(grid,"-100",bw,bh,Theme::BTN_DARK,&lv_font_montserrat_16);
  lv_obj_t *bN10 =make_btn(grid,"-10", bw,bh,Theme::BTN_DARK,&lv_font_montserrat_16);
  lv_obj_t *bN1  =make_btn(grid,"-1",  bw,bh,Theme::BTN_DARK,&lv_font_montserrat_16);
  lv_obj_t *bP100=make_btn(grid,"+100",bw,bh,Theme::ACCENT,&lv_font_montserrat_16);
  lv_obj_t *bP10 =make_btn(grid,"+10", bw,bh,Theme::ACCENT,&lv_font_montserrat_16);
  lv_obj_t *bP1  =make_btn(grid,"+1",  bw,bh,Theme::ACCENT,&lv_font_montserrat_16);
  lv_obj_set_pos(bN100,x0,y0); lv_obj_set_pos(bP100,x0,y1);
  lv_obj_set_pos(bN10, x1,y0); lv_obj_set_pos(bP10, x1,y1);
  lv_obj_set_pos(bN1,  x2,y0); lv_obj_set_pos(bP1,  x2,y1);

  auto cb = isUp ? on_ep_up_delta : on_ep_dn_delta;
  lv_obj_add_event_cb(bN100,cb,LV_EVENT_CLICKED,(void*)(intptr_t)-100);
  lv_obj_add_event_cb(bN10, cb,LV_EVENT_CLICKED,(void*)(intptr_t)-10);
  lv_obj_add_event_cb(bN1,  cb,LV_EVENT_CLICKED,(void*)(intptr_t)-1);
  lv_obj_add_event_cb(bP100,cb,LV_EVENT_CLICKED,(void*)(intptr_t)+100);
  lv_obj_add_event_cb(bP10, cb,LV_EVENT_CLICKED,(void*)(intptr_t)+10);
  lv_obj_add_event_cb(bP1,  cb,LV_EVENT_CLICKED,(void*)(intptr_t)+1);

  lv_obj_t *back = make_btn(nav, "Back", 140, 44, Theme::BTN_MID, &lv_font_montserrat_20);
  lv_obj_align(back, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(back, [](lv_event_t *e){ LV_UNUSED(e); go(tuning_scr); }, LV_EVENT_CLICKED, nullptr);
}

void on_go_main(lv_event_t *e) { LV_UNUSED(e); go(main_scr); }
void on_go_profile(lv_event_t *e) {
  LV_UNUSED(e);
  ui_update_speed_val();
  ui_update_profile_screen();
  go(profile_scr);
}
void on_go_tuning(lv_event_t *e) {
  LV_UNUSED(e); recomputeEffectiveEndpoints(); ui_update_tuning_numbers(); go(tuning_scr);
}
void on_go_ep_up(lv_event_t *e) {
  LV_UNUSED(e); recomputeEffectiveEndpoints(); ui_update_endpoint_edit_values(); go(ep_up_scr);
}
void on_go_ep_dn(lv_event_t *e) {
  LV_UNUSED(e); recomputeEffectiveEndpoints(); ui_update_endpoint_edit_values(); go(ep_dn_scr);
}
void on_calibrate(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  lv_obj_t *btn = lv_event_get_target(e);
  lv_obj_t *lbl = lv_obj_get_child(btn, 0);
  lv_obj_add_state(btn, LV_STATE_DISABLED);
  if (lbl) lv_label_set_text(lbl, "Calibrating...");
  bool ok = calibrateEndpointsSensorless();
  if (lbl) lv_label_set_text(lbl, ok ? "Calibrate" : "Retry");
  lv_obj_clear_state(btn, LV_STATE_DISABLED);
  ui_update_main_warning();
  recomputeEffectiveEndpoints();
  ui_update_tuning_numbers();
  ui_update_endpoint_edit_values();
}

void counter_timer_cb(lv_timer_t *t) {
  LV_UNUSED(t);
  if (counter_label) lv_label_set_text_fmt(counter_label, "%ld", min(counter, 9999L));
  if (main_scr && lv_scr_act() == main_scr) {
    ui_update_main_warning();
    ui_update_batch_remain();
  }
}

// ==========================================================================
//  Build all UI screens (extracted from setup())
// ==========================================================================
void ui_build_all_screens() {
  main_scr = lv_scr_act();
  style_screen(main_scr);
  lv_obj_t *mc = make_content_free(main_scr);
  lv_obj_t *mn = make_nav(main_scr);

  lv_obj_t *title = make_title(mc, "AutoLee");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
  lv_obj_t *sub = lv_label_create(mc);
  lv_label_set_text(sub, "by K.L Design");
  lv_obj_set_style_text_font(sub, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_color(sub, lv_color_hex(Theme::DIM), LV_PART_MAIN);
  lv_obj_align_to(sub, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

  // Speed profile indicator on main screen
  lbl_speed_val = lv_label_create(mc);
  lv_label_set_text(lbl_speed_val, "");
  lv_obj_set_style_text_font(lbl_speed_val, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_speed_val, lv_color_hex(Theme::SPEED_TXT), LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_speed_val, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_width(lbl_speed_val, SCR_W - 20);
  lv_obj_align_to(lbl_speed_val, sub, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

  ui_create_main_warning(mc);
  lv_obj_align_to(main_warn, lbl_speed_val, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

  counter_label = lv_label_create(mc);
  lv_label_set_text_fmt(counter_label, "%ld", min(counter, 9999L));
  lv_obj_set_style_text_font(counter_label, &lv_font_montserrat_48, LV_PART_MAIN);
  lv_obj_set_style_text_color(counter_label, lv_color_hex(Theme::GREEN), LV_PART_MAIN);
  lv_obj_align(counter_label, LV_ALIGN_CENTER, 0, -10);
  // Long-press counter to reset (make label clickable first)
  lv_obj_add_flag(counter_label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(counter_label, [](lv_event_t *e) {
    LV_UNUSED(e);
    counter = 0;
    lv_label_set_text(counter_label, "0");
  }, LV_EVENT_LONG_PRESSED, nullptr);

  // Batch remaining label (below counter, hidden when no batch)
  lbl_batch_remain = lv_label_create(mc);
  lv_label_set_text(lbl_batch_remain, "");
  lv_obj_set_style_text_font(lbl_batch_remain, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_batch_remain, lv_color_hex(Theme::WARN_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_batch_remain, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(lbl_batch_remain, LV_ALIGN_CENTER, 0, 26);
  lv_obj_add_flag(lbl_batch_remain, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *btn_batch = make_btn(mc, "Batch Run", 140, 36, Theme::ACCENT, &lv_font_montserrat_16);
  lv_obj_align(btn_batch, LV_ALIGN_BOTTOM_MID, 0, -56);

  lv_obj_t *btn_settings = make_btn(mc, "Settings", 140, 36, Theme::BTN_DARK, &lv_font_montserrat_16);
  lv_obj_align(btn_settings, LV_ALIGN_BOTTOM_MID, 0, -8);

  btn_run_global = make_btn(mn, "RUN", 140, 44, Theme::GREEN, &lv_font_montserrat_22);
  lv_obj_align(btn_run_global, LV_ALIGN_CENTER, 0, 0);

  // Settings screen
  settings_scr = lv_obj_create(NULL); style_screen(settings_scr);
  lv_obj_t *sc = make_content(settings_scr);
  lv_obj_set_style_pad_row(sc, 6, LV_PART_MAIN);
  lv_obj_add_flag(sc, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *sn = make_nav(settings_scr);
  lv_obj_t *st2 = make_title(sc, "Settings"); lv_obj_align(st2, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_t *b_cal    = make_btn(sc, "Calibrate",     140, 44, Theme::CAL_BTN, &lv_font_montserrat_20);
  lv_obj_t *b_config = make_btn(sc, "Config",        140, 44, Theme::ACCENT, &lv_font_montserrat_20);
  lv_obj_t *b_reset  = make_btn(sc, "Reset Count",   140, 44, 0xB42318, &lv_font_montserrat_20);
  lv_obj_t *b_back_s = make_btn(sn, "Back", 140, 44, Theme::BTN_MID, &lv_font_montserrat_20);
  lv_obj_align(b_back_s, LV_ALIGN_CENTER, 0, 0);

  // Configuration screen (sub-menu of Settings)
  config_scr = lv_obj_create(NULL); style_screen(config_scr);
  lv_obj_t *cfgc = make_content(config_scr);
  lv_obj_set_style_pad_row(cfgc, 6, LV_PART_MAIN);
  lv_obj_add_flag(cfgc, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *cfgn = make_nav(config_scr);
  lv_obj_t *cfgt = make_title(cfgc, "Config"); lv_obj_align(cfgt, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_t *b_speed  = make_btn(cfgc, "Speed",      140, 44, Theme::ACCENT, &lv_font_montserrat_20);
  lv_obj_t *b_tuning = make_btn(cfgc, "Endpoints",  140, 44, Theme::ACCENT, &lv_font_montserrat_20);
  lv_obj_t *b_stall  = make_btn(cfgc, "Stall Guard",140, 44, Theme::ACCENT, &lv_font_montserrat_20);
  lv_obj_t *b_wifi   = make_btn(cfgc, "WiFi Info",  140, 44, Theme::ACCENT, &lv_font_montserrat_20);
  lv_obj_t *b_back_cfg = make_btn(cfgn, "Back", 140, 44, Theme::BTN_MID, &lv_font_montserrat_20);
  lv_obj_align(b_back_cfg, LV_ALIGN_CENTER, 0, 0);

  // Profile screen (replaces old Speed screen)
  profile_scr = lv_obj_create(NULL); style_screen(profile_scr);
  lv_obj_t *pc = make_content(profile_scr);
  lv_obj_set_style_pad_row(pc, 6, LV_PART_MAIN);
  lv_obj_t *pn = make_nav(profile_scr);
  lv_obj_t *pt = make_title(pc, "Speed"); lv_obj_align(pt, LV_ALIGN_TOP_MID, 0, 2);

  // Info card showing current Hz + SG
  lv_obj_t *pcard = make_card(pc, 150, 40);
  lbl_profile_info = lv_label_create(pcard);
  lv_obj_set_style_text_color(lbl_profile_info, lv_color_hex(Theme::GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_profile_info, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_center(lbl_profile_info);

  // Three profile buttons
  for (uint8_t i = 0; i < NUM_PROFILES; i++) {
    char label[32];
    snprintf(label, sizeof(label), "%s  %lukHz", profiles[i].name, (unsigned long)(profiles[i].speed_hz / 1000));
    profile_btns[i] = make_btn(pc, label, 140, 40, Theme::BTN_DARK, &lv_font_montserrat_16);
    lv_obj_add_event_cb(profile_btns[i], [](lv_event_t *e) {
      uint8_t idx = (uint8_t)(intptr_t)lv_event_get_user_data(e);
      setActiveProfile(idx);
    }, LV_EVENT_CLICKED, (void*)(intptr_t)i);
  }

  lv_obj_t *b_back_p = make_btn(pn, "Back", 140, 44, Theme::BTN_MID, &lv_font_montserrat_20);
  lv_obj_align(b_back_p, LV_ALIGN_CENTER, 0, 0);

  // Tuning screen
  tuning_scr = lv_obj_create(NULL); style_screen(tuning_scr);
  lv_obj_t *tc = make_content_free(tuning_scr);
  lv_obj_t *tn = make_nav(tuning_scr);
  lv_obj_t *tt = make_title(tc, "Tuning"); lv_obj_align(tt, LV_ALIGN_TOP_MID, 0, 8);

  lv_obj_t *raw_card = make_card(tc, 156, 80); lv_obj_set_pos(raw_card, 8, 44);
  lbl_ep_up = lv_label_create(raw_card);
  lv_obj_set_style_text_color(lbl_ep_up, lv_color_hex(Theme::MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_ep_up, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(lbl_ep_up, LV_ALIGN_TOP_LEFT, 0, 0);
  lbl_ep_dn = lv_label_create(raw_card);
  lv_obj_set_style_text_color(lbl_ep_dn, lv_color_hex(Theme::MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_ep_dn, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(lbl_ep_dn, LV_ALIGN_TOP_LEFT, 0, 18);
  lbl_travel = lv_label_create(raw_card);
  lv_obj_set_style_text_color(lbl_travel, lv_color_hex(Theme::GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_travel, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(lbl_travel, LV_ALIGN_TOP_LEFT, 0, 36);

  lv_obj_t *eff_card = make_card(tc, 156, 64); lv_obj_set_pos(eff_card, 8, 130);
  lbl_up_eff = lv_label_create(eff_card);
  lv_obj_set_style_text_color(lbl_up_eff, lv_color_hex(Theme::TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_up_eff, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(lbl_up_eff, LV_ALIGN_TOP_LEFT, 0, 0);
  lbl_dn_eff = lv_label_create(eff_card);
  lv_obj_set_style_text_color(lbl_dn_eff, lv_color_hex(Theme::TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_dn_eff, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(lbl_dn_eff, LV_ALIGN_TOP_LEFT, 0, 22);

  lv_obj_t *btn_eu = make_btn_multiline(tc, "EP UP",   76, 52, Theme::ACCENT, &lv_font_montserrat_14);
  lv_obj_t *btn_ed = make_btn_multiline(tc, "EP DOWN", 76, 52, Theme::ACCENT, &lv_font_montserrat_14);
  lv_obj_set_pos(btn_eu, 8, 204); lv_obj_set_pos(btn_ed, 88, 204);
  lv_obj_t *b_back_t = make_btn(tn, "Back", 140, 44, Theme::BTN_MID, &lv_font_montserrat_20);
  lv_obj_align(b_back_t, LV_ALIGN_CENTER, 0, 0);

  // Endpoint screens
  ep_up_scr = lv_obj_create(NULL); build_endpoint_screen(ep_up_scr, "EP UP", true);
  ep_dn_scr = lv_obj_create(NULL); build_endpoint_screen(ep_dn_scr, "EP DOWN", false);

  // WiFi screen
  wifi_scr = lv_obj_create(NULL); style_screen(wifi_scr);
  lv_obj_t *wc = make_content(wifi_scr);
  lv_obj_t *wn = make_nav(wifi_scr);
  lv_obj_t *wt = make_title(wc, "WiFi"); lv_obj_align(wt, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_t *wcard = make_card(wc, 150, 100);
  lbl_wifi_status = lv_label_create(wcard);
  lv_obj_set_style_text_color(lbl_wifi_status, lv_color_hex(Theme::GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_wifi_status, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_label_set_long_mode(lbl_wifi_status, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_wifi_status, 130);
  lv_obj_set_style_text_align(lbl_wifi_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(lbl_wifi_status);
  lv_obj_t *b_wifi_reset = make_btn(wc, "Reset WiFi", 140, 44, 0xB42318, &lv_font_montserrat_20);
  lv_obj_t *b_back_w = make_btn(wn, "Back", 140, 44, Theme::BTN_MID, &lv_font_montserrat_20);
  lv_obj_align(b_back_w, LV_ALIGN_CENTER, 0, 0);

  // --- Jam detected screen ---
  jam_scr = lv_obj_create(NULL);
  style_screen(jam_scr);
  lv_obj_t *jc = make_content_free(jam_scr);
  lv_obj_t *jn = make_nav(jam_scr);

  // Warning icon / title
  lv_obj_t *jam_title = lv_label_create(jc);
  lv_label_set_text(jam_title, LV_SYMBOL_WARNING " JAM");
  lv_obj_set_style_text_font(jam_title, &lv_font_montserrat_26, LV_PART_MAIN);
  lv_obj_set_style_text_color(jam_title, lv_color_hex(Theme::RED), LV_PART_MAIN);
  lv_obj_align(jam_title, LV_ALIGN_TOP_MID, 0, 20);

  lv_obj_t *jam_msg = lv_label_create(jc);
  lv_label_set_text(jam_msg, "Stall detected!\nMotor stopped and\nbacked off safely.");
  lv_obj_set_style_text_font(jam_msg, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_color(jam_msg, lv_color_hex(Theme::JAM_MSG), LV_PART_MAIN);
  lv_obj_set_style_text_align(jam_msg, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_label_set_long_mode(jam_msg, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(jam_msg, SCR_W - 20);
  lv_obj_align(jam_msg, LV_ALIGN_TOP_MID, 0, 60);

  jam_status_lbl = lv_label_create(jc);
  lv_label_set_text(jam_status_lbl, "Press to return home");
  lv_obj_set_style_text_font(jam_status_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_color(jam_status_lbl, lv_color_hex(Theme::WARN_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_align(jam_status_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(jam_status_lbl, LV_ALIGN_CENTER, 0, 20);

  lv_obj_t *btn_home = make_btn(jn, "Return Home", 140, 44, Theme::ACCENT, &lv_font_montserrat_18);
  lv_obj_align(btn_home, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(btn_home, onJamReturnHome, LV_EVENT_CLICKED, nullptr);

  // --- Stall sensitivity screen ---
  stall_scr = lv_obj_create(NULL);
  style_screen(stall_scr);
  lv_obj_t *ssc = make_content(stall_scr);
  lv_obj_t *ssn = make_nav(stall_scr);

  lv_obj_t *sst = make_title(ssc, "Stall Sens.");
  lv_obj_align(sst, LV_ALIGN_TOP_MID, 0, 2);

  lv_obj_t *sg_card = make_card(ssc, 150, 92);

  lv_obj_t *sg_name = lv_label_create(sg_card);
  lv_label_set_text(sg_name, "SG Trip");
  lv_obj_set_style_text_color(sg_name, lv_color_hex(Theme::MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(sg_name, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(sg_name, LV_ALIGN_TOP_LEFT, 0, 0);

  lbl_sg_val = lv_label_create(sg_card);
  lv_obj_set_style_text_color(lbl_sg_val, lv_color_hex(Theme::GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_sg_val, &lv_font_montserrat_26, LV_PART_MAIN);
  lv_obj_center(lbl_sg_val);

  lv_obj_t *sg_hint = lv_label_create(sg_card);
  lv_label_set_text(sg_hint, "0=off  lower=sensitive");
  lv_obj_set_style_text_color(sg_hint, lv_color_hex(Theme::HINT), LV_PART_MAIN);
  lv_obj_set_style_text_font(sg_hint, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(sg_hint, LV_ALIGN_BOTTOM_MID, 0, 0);

  lv_obj_t *sg_grid = lv_obj_create(ssc);
  lv_obj_set_size(sg_grid, 150, 96);
  lv_obj_set_style_bg_opa(sg_grid, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(sg_grid, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(sg_grid, 0, LV_PART_MAIN);
  lv_obj_clear_flag(sg_grid, LV_OBJ_FLAG_SCROLLABLE);

  const int sgbw = 70, sgbh = 44, sggap = 4;
  lv_obj_t *sgM5 = make_btn(sg_grid, "-5", sgbw, sgbh, Theme::BTN_DARK, &lv_font_montserrat_18);
  lv_obj_t *sgM1 = make_btn(sg_grid, "-1", sgbw, sgbh, Theme::BTN_DARK, &lv_font_montserrat_18);
  lv_obj_t *sgP1 = make_btn(sg_grid, "+1", sgbw, sgbh, Theme::ACCENT, &lv_font_montserrat_18);
  lv_obj_t *sgP5 = make_btn(sg_grid, "+5", sgbw, sgbh, Theme::ACCENT, &lv_font_montserrat_18);
  lv_obj_set_pos(sgM5, 0, 0);
  lv_obj_set_pos(sgP5, sgbw + sggap, 0);
  lv_obj_set_pos(sgM1, 0, sgbh + sggap);
  lv_obj_set_pos(sgP1, sgbw + sggap, sgbh + sggap);

  auto sg_cb = [](lv_event_t *e) {
    int32_t d = (int32_t)(intptr_t)lv_event_get_user_data(e);
    int32_t v = (int32_t)RUN_SG_TRIP + d;
    RUN_SG_TRIP = (uint16_t)constrain(v, (int32_t)RUN_SG_TRIP_MIN, (int32_t)RUN_SG_TRIP_MAX);
    ui_update_sg_val();
  };
  lv_obj_add_event_cb(sgM5, sg_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-5);
  lv_obj_add_event_cb(sgM1, sg_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
  lv_obj_add_event_cb(sgP1, sg_cb, LV_EVENT_CLICKED, (void*)(intptr_t)+1);
  lv_obj_add_event_cb(sgP5, sg_cb, LV_EVENT_CLICKED, (void*)(intptr_t)+5);

  lv_obj_t *b_back_ss = make_btn(ssn, "Back", 140, 44, Theme::BTN_MID, &lv_font_montserrat_20);
  lv_obj_align(b_back_ss, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(b_back_ss, [](lv_event_t *e){ LV_UNUSED(e); go(config_scr); }, LV_EVENT_CLICKED, nullptr);

  // --- Batch run screen ---
  batch_scr = lv_obj_create(NULL);
  style_screen(batch_scr);
  lv_obj_t *brc = make_content(batch_scr);
  lv_obj_set_style_pad_row(brc, 4, LV_PART_MAIN);  // tight spacing
  lv_obj_t *brn = make_nav(batch_scr);

  lv_obj_t *brt = make_title(brc, "Batch Run");
  lv_obj_align(brt, LV_ALIGN_TOP_MID, 0, 2);

  lv_obj_t *br_card = make_card(brc, 150, 60);

  lv_obj_t *br_name = lv_label_create(br_card);
  lv_label_set_text(br_name, "Count");
  lv_obj_set_style_text_color(br_name, lv_color_hex(Theme::MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(br_name, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(br_name, LV_ALIGN_TOP_LEFT, 0, 0);

  lbl_batch_val = lv_label_create(br_card);
  lv_obj_set_style_text_color(lbl_batch_val, lv_color_hex(Theme::GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_batch_val, &lv_font_montserrat_26, LV_PART_MAIN);
  lv_obj_align(lbl_batch_val, LV_ALIGN_TOP_RIGHT, 0, 0);

  lv_obj_t *br_hint = lv_label_create(br_card);
  lv_label_set_text(br_hint, "0 = off (unlimited)");
  lv_obj_set_style_text_color(br_hint, lv_color_hex(Theme::HINT), LV_PART_MAIN);
  lv_obj_set_style_text_font(br_hint, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(br_hint, LV_ALIGN_BOTTOM_MID, 0, 0);

  // +/- buttons 2x3 grid
  lv_obj_t *br_grid = lv_obj_create(brc);
  lv_obj_set_size(br_grid, 150, 86);
  lv_obj_set_style_bg_opa(br_grid, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(br_grid, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(br_grid, 0, LV_PART_MAIN);
  lv_obj_clear_flag(br_grid, LV_OBJ_FLAG_SCROLLABLE);

  const int bbw = 48, bbh = 40, bgap = 3;
  lv_obj_t *brM100 = make_btn(br_grid, "-100", bbw, bbh, Theme::BTN_DARK, &lv_font_montserrat_14);
  lv_obj_t *brM10  = make_btn(br_grid, "-10",  bbw, bbh, Theme::BTN_DARK, &lv_font_montserrat_16);
  lv_obj_t *brM1   = make_btn(br_grid, "-1",   bbw, bbh, Theme::BTN_DARK, &lv_font_montserrat_16);
  lv_obj_t *brP1   = make_btn(br_grid, "+1",   bbw, bbh, Theme::ACCENT, &lv_font_montserrat_16);
  lv_obj_t *brP10  = make_btn(br_grid, "+10",  bbw, bbh, Theme::ACCENT, &lv_font_montserrat_16);
  lv_obj_t *brP100 = make_btn(br_grid, "+100", bbw, bbh, Theme::ACCENT, &lv_font_montserrat_14);
  lv_obj_set_pos(brM100, 0, 0);
  lv_obj_set_pos(brM10,  bbw+bgap, 0);
  lv_obj_set_pos(brM1,   2*(bbw+bgap), 0);
  lv_obj_set_pos(brP1,   0, bbh+bgap);
  lv_obj_set_pos(brP10,  bbw+bgap, bbh+bgap);
  lv_obj_set_pos(brP100, 2*(bbw+bgap), bbh+bgap);

  auto br_cb = [](lv_event_t *e) {
    int32_t d = (int32_t)(intptr_t)lv_event_get_user_data(e);
    int32_t v = batchTarget + d;
    batchTarget = constrain(v, (int32_t)0, (int32_t)9999);
    ui_update_batch_val();
  };
  lv_obj_add_event_cb(brM100, br_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-100);
  lv_obj_add_event_cb(brM10,  br_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-10);
  lv_obj_add_event_cb(brM1,   br_cb, LV_EVENT_CLICKED, (void*)(intptr_t)-1);
  lv_obj_add_event_cb(brP1,   br_cb, LV_EVENT_CLICKED, (void*)(intptr_t)+1);
  lv_obj_add_event_cb(brP10,  br_cb, LV_EVENT_CLICKED, (void*)(intptr_t)+10);
  lv_obj_add_event_cb(brP100, br_cb, LV_EVENT_CLICKED, (void*)(intptr_t)+100);

  // Start batch button
  lv_obj_t *btn_start_batch = make_btn(brc, "Start Batch", 140, 38, Theme::GREEN, &lv_font_montserrat_18);
  lv_obj_add_event_cb(btn_start_batch, [](lv_event_t *e) {
    LV_UNUSED(e);
    if (batchTarget <= 0 || runState != IDLE || !endpointsCalibrated) return;
    batchCount = 0;
    batchActive = true;
    startRunBetweenEndpoints();
    setRunButtonState(true);
    go(main_scr);
  }, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *b_back_br = make_btn(brn, "Back", 140, 44, Theme::BTN_MID, &lv_font_montserrat_20);
  lv_obj_align(b_back_br, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(b_back_br, [](lv_event_t *e){ LV_UNUSED(e); go(main_scr); }, LV_EVENT_CLICKED, nullptr);

  // ---- EVENTS ----
  lv_obj_add_event_cb(btn_batch, [](lv_event_t *e){ LV_UNUSED(e); ui_update_batch_val(); go(batch_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(btn_settings, [](lv_event_t *e){ LV_UNUSED(e); go(settings_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(btn_run_global, [](lv_event_t *e){
    LV_UNUSED(e);
    if (runState == IDLE) { startRunBetweenEndpoints(); setRunButtonState(runState == RUNNING); }
    else if (runState == RUNNING) { requestGracefulStop(); setRunButtonState(false); batchActive = false; }
  }, LV_EVENT_CLICKED, nullptr);

  lv_obj_add_event_cb(b_speed,  on_go_profile, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_tuning, on_go_tuning, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_cal,    on_calibrate, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_config, [](lv_event_t *e){ LV_UNUSED(e); go(config_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_reset, [](lv_event_t *e){ LV_UNUSED(e); counter = 0; if (counter_label) lv_label_set_text(counter_label, "0"); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_back_s, on_go_main, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_stall, [](lv_event_t *e){ LV_UNUSED(e); ui_update_sg_val(); go(stall_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_wifi, [](lv_event_t *e){ LV_UNUSED(e); ui_update_wifi_label(); go(wifi_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_wifi_reset, [](lv_event_t *e){
    LV_UNUSED(e);
    clearWiFiCredentials();
    webLog("WiFi credentials cleared, rebooting...");
    rebootRequested = true;
    rebootRequestMs = millis();
  }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_back_cfg, [](lv_event_t *e){ LV_UNUSED(e); go(settings_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_back_p, [](lv_event_t *e){ LV_UNUSED(e); go(config_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(btn_eu, on_go_ep_up, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(btn_ed, on_go_ep_dn, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_back_t, [](lv_event_t *e){ LV_UNUSED(e); go(config_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_back_w, [](lv_event_t *e){ LV_UNUSED(e); go(config_scr); }, LV_EVENT_CLICKED, nullptr);

  lv_timer_create(counter_timer_cb, 100, nullptr);

  ui_update_speed_val();
  ui_update_profile_screen();
  recomputeEffectiveEndpoints();
  ui_update_tuning_numbers();
  ui_update_endpoint_edit_values();
  ui_update_main_warning();
  ui_update_sg_val();
  ui_update_batch_val();
}
