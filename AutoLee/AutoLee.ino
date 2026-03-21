// ============================================================================
//  AutoLee v1.7
// ============================================================================

#include <lvgl.h>
#include "esp_lcd_touch_axs5106l.h"
#include <Arduino_GFX_Library.h>
#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Update.h>
#include <DNSServer.h>

#include "config.h"

// ==========================================================================
//  GLOBAL DEFINITIONS (storage for extern declarations in globals.h)
// ==========================================================================
TMC5160Stepper driver(TMC_CS, R_SENSE);
FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

static long rawUp = 0, rawDown = 0;
static long endpointUp = 0, endpointDown = 0;
static bool endpointsCalibrated = false;
static long counter = 0;

enum RunState : uint8_t { IDLE, RUNNING, STOPPING, CALIBRATING, STALLED, HOMING };
static volatile RunState runState = IDLE;
static long     currentTarget = 0;
static uint32_t stopEntryMs   = 0;

static uint32_t lastDirectionChangeMs = 0;
static uint8_t  runSGHighCount = 0;
static uint8_t  runSGLowCount = 0;

static bool wifiConnected = false;
static bool wifiAPMode = false;
static String wifiSSID = "", wifiPass = "";
static String scannedOptionsHTML = "";
DNSServer dnsServer;
static bool captivePortalRunning = false;

Preferences prefs;
AsyncWebServer webServer(80);
AsyncEventSource events("/events");
static uint32_t lastSSEMs = 0;

static volatile bool webCalRequested = false;
static volatile bool webHomeRequested = false;
static volatile bool rebootRequested = false;
static uint32_t     rebootRequestMs = 0;

static int32_t  batchTarget  = 0;
static int32_t  batchCount   = 0;
static bool     batchActive  = false;

static char logBuf[LOG_LINES][LOG_LINE_LEN];
static uint16_t logHead = 0;
static uint32_t logSerial = 0;
static uint32_t logSentSerial = 0;

Arduino_DataBus *bus = new Arduino_HWSPI(15, 14, 1, 2);
Arduino_GFX *gfx = new Arduino_ST7789(bus, 22, 0, false, 172, 320, 34, 0, 34, 0);

// LVGL
static uint32_t bufSize = 0;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf = nullptr;
static lv_disp_drv_t disp_drv;

static lv_obj_t *main_scr = nullptr, *settings_scr = nullptr, *config_scr = nullptr, *profile_scr = nullptr;
static lv_obj_t *tuning_scr = nullptr, *ep_up_scr = nullptr, *ep_dn_scr = nullptr;
static lv_obj_t *wifi_scr = nullptr;
static lv_obj_t *counter_label = nullptr, *main_warn = nullptr, *main_warn_lbl = nullptr;
static lv_obj_t *lbl_speed_val = nullptr;
static lv_obj_t *profile_btns[NUM_PROFILES] = {};
static lv_obj_t *lbl_profile_info = nullptr;
static lv_obj_t *lbl_ep_up = nullptr, *lbl_ep_dn = nullptr, *lbl_travel = nullptr;
static lv_obj_t *lbl_up_eff = nullptr, *lbl_dn_eff = nullptr;
static lv_obj_t *lbl_ep_up_val = nullptr, *lbl_ep_dn_val = nullptr;
static lv_obj_t *lbl_wifi_status = nullptr;
static lv_obj_t *btn_run_global = nullptr;

static lv_obj_t *jam_scr = nullptr;
static lv_obj_t *jam_status_lbl = nullptr;
static lv_obj_t *stall_scr = nullptr;
static lv_obj_t *lbl_sg_val = nullptr;

static lv_obj_t *batch_scr = nullptr;
static lv_obj_t *lbl_batch_val = nullptr;
static lv_obj_t *lbl_batch_remain = nullptr;

// ==========================================================================
//  webLog — used by all modules
// ==========================================================================
void webLog(const char *fmt, ...) {
  char line[LOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  Serial.println(line);
  strncpy(logBuf[logHead], line, LOG_LINE_LEN - 1);
  logBuf[logHead][LOG_LINE_LEN - 1] = '\0';
  logHead = (logHead + 1) % LOG_LINES;
  logSerial++;
}

// ==========================================================================
//  INCLUDE MODULES (order matters: motion before ui_touch before web_server)
//  Forward declarations resolve circular dependencies between motion and UI.
// ==========================================================================

// UI functions called by motion.h (defined in ui_touch.h)
static void go(lv_obj_t *scr);
void setRunButtonState(bool running);
void ui_update_main_warning();
void ui_update_tuning_numbers();
void ui_update_endpoint_edit_values();
void ui_update_sg_val();

#include "motion.h"

// WiFi functions called by ui_touch.h (defined in wifi_ota.h)
void clearWiFiCredentials();

#include "ui_touch.h"
#include "wifi_ota.h"
#include "web_server.h"

// ==========================================================================
//  SETUP
// ==========================================================================
void setup() {
  Serial.begin(115200);
  Serial.printf("=== AutoLee v%s ===\n", FW_VERSION);

  SPI.begin(1, 3, 2, TMC_CS);
  pinMode(14, OUTPUT); digitalWrite(14, HIGH);
  pinMode(TMC_CS, OUTPUT); digitalWrite(TMC_CS, HIGH);

  if (!gfx->begin()) Serial.println("gfx->begin() failed!");
  lcd_reg_init();
  gfx->setRotation(ROTATION);
  gfx->fillScreen(RGB565_BLACK);
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);

  Wire.begin(Touch_I2C_SDA, Touch_I2C_SCL);
  bsp_touch_init(&Wire, Touch_RST, Touch_INT, gfx->getRotation(), gfx->width(), gfx->height());

  lv_init();
#if LV_USE_BIDI
  lv_bidi_set_base_dir(LV_BASE_DIR_LTR);
#endif

  bufSize = (uint32_t)gfx->width() * 40;
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_8BIT);
  if (!disp_draw_buf) { Serial.println("LVGL buf alloc failed!"); for(;;) delay(1000); }

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, nullptr, bufSize);
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = gfx->width();
  disp_drv.ver_res = gfx->height();
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read_cb;
  lv_indev_drv_register(&indev_drv);

  // TMC5160
  driver.begin();
  driver.setSPISpeed(4000000);
  driver.toff(5);
  driver.rms_current(RUN_CURRENT_MA);
  driver.microsteps(16);
  driver.en_pwm_mode(false);
  driver.TPWMTHRS(0);
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(0); driver.semax(0); driver.seup(0); driver.sedn(0);
  int8_t sgt = constrain((int8_t)CAL_SGT, (int8_t)-64, (int8_t)63);
  driver.sgt(sgt);
  driver.diag1_stall(true);
  driver.diag1_index(false);
  driver.diag1_onstate(false);
  driver.diag1_steps_skipped(false);
  driver.diag1_pushpull(true);
  pinMode(TMC_DIAG_PIN, INPUT);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) { Serial.println("Stepper fail!"); while(true) delay(1000); }
  stepper->setDirectionPin(DIR_PIN, false);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);
  stepper->setSpeedInHz(ui_speed_hz);
  stepper->setAcceleration(RUN_DECEL);

  // Build the LVGL touch UI
  buildUI();

  // WiFi + Web + OTA
  startWiFi();
  setupWebServer();
  setupArduinoOTA();

  Serial.println("Setup complete!");
}

// ==========================================================================
//  LOOP
// ==========================================================================
void loop() {
  lv_timer_handler();
  handleMotion();
  handleWebCalibration();
  handleWebHome();
  ArduinoOTA.handle();
  broadcastState();

  // Process captive portal DNS requests
  if (captivePortalRunning) {
    dnsServer.processNextRequest();
  }

  // Deferred reboot (safe from main loop context)
  if (rebootRequested && (millis() - rebootRequestMs) > 500) {
    if (stepper && stepper->isRunning()) stepper->forceStop();
    delay(100);
    ESP.restart();
  }

  delay(1);
}
