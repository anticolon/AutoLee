// ============================================================================
//  AutoLee v1.7
// ============================================================================

#define FW_VERSION "1.7"

// --- Library includes ---
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
//  RunState enum (must be before module includes)
// ==========================================================================
enum RunState : uint8_t { IDLE, RUNNING, STOPPING, CALIBRATING, STALLED, HOMING };

// ==========================================================================
//  CONFIGURATION (mutable state & profile data)
// ==========================================================================

static SpeedProfile profiles[NUM_PROFILES] = {
  { "Slow",   15000, 350, 350 },
  { "Normal", 35000, 15, 15 },
  { "Fast",   45000, 1, 1 },
};
static uint8_t activeProfile = 1;  // default to Normal

// Convenience macros — resolve to active profile's speed/SG values (re-evaluated on each use)
#define ui_speed_hz  (profiles[activeProfile].speed_hz)
#define RUN_SG_TRIP  (profiles[activeProfile].sg_trip)

static int32_t upOffsetSteps   = 0;
static int32_t downOffsetSteps = 0;

// ==========================================================================
//  GLOBALS
// ==========================================================================
TMC5160Stepper driver(TMC_CS, R_SENSE);
FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

static long rawUp = 0, rawDown = 0;
static long endpointUp = 0, endpointDown = 0;
static bool endpointsCalibrated = false;
static long counter = 0;

static volatile RunState runState = IDLE;
static long     currentTarget = 0;
static uint32_t stopEntryMs   = 0;

// --- Runtime stall detection (during RUNNING) ---
static int32_t SG_WORK_ZONE_STEPS = 5500;  // skip SG this many steps before endpointDown

static uint32_t lastDirectionChangeMs = 0;
static uint32_t lastSGPrintMs = 0;         // reset per run; used by handleMotion()

// Jam screen
static lv_obj_t *jam_scr = nullptr;
static lv_obj_t *jam_status_lbl = nullptr;
static lv_obj_t *stall_scr = nullptr;
static lv_obj_t *lbl_sg_val = nullptr;

static bool wifiConnected = false;
static bool wifiAPMode = false;
static String wifiSSID = "", wifiPass = "";
static String scannedOptionsHTML = "";
DNSServer dnsServer;
static bool captivePortalRunning = false;

Preferences prefs;
AsyncWebServer webServer(80);
AsyncEventSource events("/events");

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
static lv_obj_t *profile_btns[NUM_PROFILES] = {}; // buttons on profile screen
static lv_obj_t *lbl_profile_info = nullptr;       // shows Hz + SG on profile screen
static lv_obj_t *lbl_ep_up = nullptr, *lbl_ep_dn = nullptr, *lbl_travel = nullptr;
static lv_obj_t *lbl_up_eff = nullptr, *lbl_dn_eff = nullptr;
static lv_obj_t *lbl_ep_up_val = nullptr, *lbl_ep_dn_val = nullptr;
static lv_obj_t *lbl_wifi_status = nullptr;
static lv_obj_t *btn_run_global = nullptr;

static volatile bool webCalRequested = false;
static volatile bool webHomeRequested = false;
static volatile bool rebootRequested = false;
static volatile bool webRunToggleRequested = false;
static volatile bool webProfileChangeRequested = false;
static volatile uint8_t webProfileIdx = 0;
// --- Deferred web mutations (applied from main loop) ---
static volatile bool webEndpointUpDeltaPending = false;
static volatile int32_t webEndpointUpDelta = 0;
static volatile bool webEndpointDnDeltaPending = false;
static volatile int32_t webEndpointDnDelta = 0;
static volatile bool webSgTripDeltaPending = false;
static volatile uint8_t webSgTripProfile = 0;
static volatile int32_t webSgTripDelta = 0;
static volatile bool webWorkZoneDeltaPending = false;
static volatile int32_t webWorkZoneDelta = 0;
static volatile bool webBatchDeltaPending = false;
static volatile int32_t webBatchDelta = 0;
static volatile bool webBatchStartRequested = false;
static volatile bool webBatchClearRequested = false;
static volatile bool webCounterResetRequested = false;
static volatile bool webLogClearRequested = false;
static volatile bool webCurrentMaPending = false;
static volatile uint16_t webCurrentMaValue = 0;
static volatile bool webSgTripAbsolute = false;  // when true, webSgTripDelta holds absolute value
static uint32_t     rebootRequestMs = 0;

// --- Batch run ---
static int32_t  batchTarget  = 0;     // 0 = unlimited
static int32_t  batchCount   = 0;     // completed in current batch
static bool     batchActive  = false;
static lv_obj_t *batch_scr       = nullptr;
static lv_obj_t *lbl_batch_val   = nullptr;
static lv_obj_t *lbl_batch_remain = nullptr;  // on main screen
static uint32_t lastSSEMs = 0;

// --- Log ring buffer for web ---
static char logBuf[LOG_LINES][LOG_LINE_LEN];
static uint16_t logHead = 0;       // next write index in ring buffer (wraps at LOG_LINES)
static uint32_t logSerial = 0;     // monotonic total lines written (never wraps in practice)
static uint32_t logSentSerial = 0; // serial number of last line sent to client

// ==========================================================================
// Pure endpoint math from endpoint_math.h (clamp_i32, nearPos, flipTarget).
// The 2-arg flipTarget below wraps the 3-arg version with runtime globals.
// ==========================================================================
#include "endpoint_math.h"

static inline long flipTarget(long t) { return flipTarget(t, endpointUp, endpointDown); }

// ==========================================================================
//  MODULE INCLUDES (order: motor, ui, web)
//  All globals above are visible to the included headers.
//  Forward declarations in each header break circular dependencies.
// ==========================================================================
#include "motor.h"
#include "ui.h"
#include "web.h"

// ==========================================================================
//  ST7789 INIT
// ==========================================================================
static const uint8_t PROGMEM init_operations[] = {
  BEGIN_WRITE, WRITE_COMMAND_8, 0x11, END_WRITE, DELAY, 120,
  BEGIN_WRITE,
  WRITE_C8_D16, 0xDF, 0x98, 0x53,
  WRITE_C8_D8,  0xB2, 0x23,
  WRITE_COMMAND_8, 0xB7, WRITE_BYTES, 4, 0x00, 0x47, 0x00, 0x6F,
  WRITE_COMMAND_8, 0xBB, WRITE_BYTES, 6, 0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0,
  WRITE_C8_D16, 0xC0, 0x44, 0xA4,
  WRITE_C8_D8,  0xC1, 0x16,
  WRITE_COMMAND_8, 0xC3, WRITE_BYTES, 8, 0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77,
  WRITE_COMMAND_8, 0xC4, WRITE_BYTES, 12, 0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82,
  WRITE_COMMAND_8, 0xC8, WRITE_BYTES, 32,
    0x3F,0x32,0x29,0x29,0x27,0x2B,0x27,0x28,0x28,0x26,0x25,0x17,0x12,0x0D,0x04,0x00,
    0x3F,0x32,0x29,0x29,0x27,0x2B,0x27,0x28,0x28,0x26,0x25,0x17,0x12,0x0D,0x04,0x00,
  WRITE_COMMAND_8, 0xD0, WRITE_BYTES, 5, 0x04, 0x06, 0x6B, 0x0F, 0x00,
  WRITE_C8_D16, 0xD7, 0x00, 0x30,
  WRITE_C8_D8,  0xE6, 0x14,
  WRITE_C8_D8,  0xDE, 0x01,
  WRITE_COMMAND_8, 0xB7, WRITE_BYTES, 5, 0x03, 0x13, 0xEF, 0x35, 0x35,
  WRITE_COMMAND_8, 0xC1, WRITE_BYTES, 3, 0x14, 0x15, 0xC0,
  WRITE_C8_D16, 0xC2, 0x06, 0x3A,
  WRITE_C8_D16, 0xC4, 0x72, 0x12,
  WRITE_C8_D8,  0xBE, 0x00,
  WRITE_C8_D8,  0xDE, 0x02,
  WRITE_COMMAND_8, 0xE5, WRITE_BYTES, 3, 0x00, 0x02, 0x00,
  WRITE_COMMAND_8, 0xE5, WRITE_BYTES, 3, 0x01, 0x02, 0x00,
  WRITE_C8_D8,  0xDE, 0x00,
  WRITE_C8_D8,  0x35, 0x00,
  WRITE_C8_D8,  0x3A, 0x05,
  WRITE_COMMAND_8, 0x2A, WRITE_BYTES, 4, 0x00, 0x22, 0x00, 0xCD,
  WRITE_COMMAND_8, 0x2B, WRITE_BYTES, 4, 0x00, 0x00, 0x01, 0x3F,
  WRITE_C8_D8,  0xDE, 0x02,
  WRITE_COMMAND_8, 0xE5, WRITE_BYTES, 3, 0x00, 0x02, 0x00,
  WRITE_C8_D8,  0xDE, 0x00,
  WRITE_C8_D8,  0x36, 0x00,
  WRITE_COMMAND_8, 0x21,
  END_WRITE, DELAY, 10,
  BEGIN_WRITE, WRITE_COMMAND_8, 0x29, END_WRITE
};
static void lcd_reg_init() { bus->batchOperation(init_operations, sizeof(init_operations)); }

// ==========================================================================
//  LVGL CALLBACKS
// ==========================================================================
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1, h = area->y2 - area->y1 + 1;
#if LV_COLOR_16_SWAP
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif
  lv_disp_flush_ready(disp);
}

static void touchpad_read_cb(lv_indev_drv_t *, lv_indev_data_t *data) {
  touch_data_t t;
  bsp_touch_read();
  if (bsp_touch_get_coordinates(&t)) {
    data->point.x = t.coords[0].x; data->point.y = t.coords[0].y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else data->state = LV_INDEV_STATE_RELEASED;
}

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
  stepper->setAcceleration(RUN_ACCEL);

  // Build all LVGL screens
  ui_build_all_screens();

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
  handleWebUIUpdates();
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
