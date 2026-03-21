// ============================================================================
//  AutoLee – globals.h
//  Shared state variables, hardware objects, forward declarations
// ============================================================================
#pragma once

#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <lvgl.h>

#include "config.h"

// ==========================================================================
//  HARDWARE OBJECTS
// ==========================================================================
extern TMC5160Stepper driver;
extern FastAccelStepperEngine engine;
extern FastAccelStepper *stepper;

extern Arduino_DataBus *bus;
extern Arduino_GFX *gfx;

// ==========================================================================
//  MOTOR / CALIBRATION STATE
// ==========================================================================
extern long rawUp, rawDown;
extern long endpointUp, endpointDown;
extern bool endpointsCalibrated;
extern long counter;

enum RunState : uint8_t { IDLE, RUNNING, STOPPING, CALIBRATING, STALLED, HOMING };
extern volatile RunState runState;
extern long     currentTarget;
extern uint32_t stopEntryMs;

// Runtime stall detection
extern uint32_t lastDirectionChangeMs;
extern uint8_t  runSGHighCount;
extern uint8_t  runSGLowCount;
static constexpr uint8_t RUN_SG_HIGH_NEEDED = 2;  // need this many high readings to trigger jam

// ==========================================================================
//  WIFI STATE
// ==========================================================================
extern bool wifiConnected;
extern bool wifiAPMode;
extern String wifiSSID, wifiPass;
extern String scannedOptionsHTML;
extern DNSServer dnsServer;
extern bool captivePortalRunning;

// ==========================================================================
//  WEB / PREFERENCES
// ==========================================================================
extern Preferences prefs;
extern AsyncWebServer webServer;
extern AsyncEventSource events;
extern uint32_t lastSSEMs;

// ==========================================================================
//  WEB REQUEST FLAGS (set from ISR/async context, handled in loop)
// ==========================================================================
extern volatile bool webCalRequested;
extern volatile bool webHomeRequested;
extern volatile bool rebootRequested;
extern uint32_t     rebootRequestMs;

// ==========================================================================
//  BATCH RUN
// ==========================================================================
extern int32_t  batchTarget;
extern int32_t  batchCount;
extern bool     batchActive;

// ==========================================================================
//  LOG RING BUFFER
// ==========================================================================
extern char logBuf[LOG_LINES][LOG_LINE_LEN];
extern uint16_t logHead;
extern uint32_t logSerial;
extern uint32_t logSentSerial;

// ==========================================================================
//  LVGL DISPLAY
// ==========================================================================
extern uint32_t bufSize;
extern lv_disp_draw_buf_t draw_buf;
extern lv_color_t *disp_draw_buf;
extern lv_disp_drv_t disp_drv;

// ==========================================================================
//  LVGL UI OBJECTS (shared between ui_touch.h and web_server.h)
// ==========================================================================
extern lv_obj_t *main_scr, *settings_scr, *config_scr, *profile_scr;
extern lv_obj_t *tuning_scr, *ep_up_scr, *ep_dn_scr;
extern lv_obj_t *wifi_scr;
extern lv_obj_t *counter_label, *main_warn, *main_warn_lbl;
extern lv_obj_t *lbl_speed_val;
extern lv_obj_t *profile_btns[NUM_PROFILES];
extern lv_obj_t *lbl_profile_info;
extern lv_obj_t *lbl_ep_up, *lbl_ep_dn, *lbl_travel;
extern lv_obj_t *lbl_up_eff, *lbl_dn_eff;
extern lv_obj_t *lbl_ep_up_val, *lbl_ep_dn_val;
extern lv_obj_t *lbl_wifi_status;
extern lv_obj_t *btn_run_global;

// Jam screen
extern lv_obj_t *jam_scr;
extern lv_obj_t *jam_status_lbl;
extern lv_obj_t *stall_scr;
extern lv_obj_t *lbl_sg_val;

// Batch screen
extern lv_obj_t *batch_scr;
extern lv_obj_t *lbl_batch_val;
extern lv_obj_t *lbl_batch_remain;

// ==========================================================================
//  UTILITY — webLog (used everywhere)
// ==========================================================================
void webLog(const char *fmt, ...);

// ==========================================================================
//  FORWARD DECLARATIONS — motion.h
// ==========================================================================
void startRunBetweenEndpoints();
void requestGracefulStop();
bool calibrateEndpointsSensorless();
void safeCreepHome();
void recomputeEffectiveEndpoints();
void handleMotion();
bool move_until_stall(int dir, long &hit_pos);
bool return_home_up_safe();
uint16_t read_sg_raw();
uint16_t read_sg();
void fas_wait_for_stop();

// ==========================================================================
//  FORWARD DECLARATIONS — ui_touch.h
// ==========================================================================
void ui_update_main_warning();
void ui_update_speed_val();
void ui_update_profile_screen();
void ui_update_tuning_numbers();
void ui_update_endpoint_edit_values();
void ui_update_sg_val();
void ui_update_batch_val();
void ui_update_batch_remain();
void ui_update_wifi_label();
void setRunButtonState(bool running);
void setActiveProfile(uint8_t idx);
void buildUI();

// ==========================================================================
//  FORWARD DECLARATIONS — wifi_ota.h
// ==========================================================================
void startWiFi();
void setupArduinoOTA();
void loadWiFiCredentials();
void saveWiFiCredentials(const String &ssid, const String &pass);
void clearWiFiCredentials();

// ==========================================================================
//  FORWARD DECLARATIONS — web_server.h
// ==========================================================================
void setupWebServer();
void broadcastState();
void handleWebCalibration();
void handleWebHome();
