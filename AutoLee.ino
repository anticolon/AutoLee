// ============================================================================
//  AutoLee v1.7
// ============================================================================

#define FW_VERSION "1.7"

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

// ==========================================================================
//  CONFIGURATION
// ==========================================================================

static const char *DEFAULT_AP_SSID = "AutoLee-Setup";

#define ENABLE_PIN   4
#define STEP_PIN     5
#define DIR_PIN      6
#define TMC_DIAG_PIN 7
#define TMC_CS       8
#define R_SENSE      0.022f

#define ROTATION     0
#define GFX_BL       23
#define Touch_I2C_SDA 18
#define Touch_I2C_SCL 19
#define Touch_RST     20
#define Touch_INT     21

// --- Speed profiles: each bundles speed + matching SG threshold ---
struct SpeedProfile {
  const char *name;
  uint32_t speed_hz;
  uint16_t sg_trip_default;  // factory default SG for this speed
  uint16_t sg_trip;          // current (user-tweakable) SG trip
};

static constexpr uint8_t NUM_PROFILES = 3;
static SpeedProfile profiles[NUM_PROFILES] = {
  { "Slow",   15000, 350, 350 },
  { "Normal", 35000, 15, 15 },
  { "Fast",   45000, 1, 1 },
};
static uint8_t activeProfile = 1;  // default to Normal

static constexpr uint32_t RUN_DECEL    = 800000;  // accel/decel rate for run moves (fast ramps, max SG coverage)

// Accessors — use these everywhere instead of raw globals
#define ui_speed_hz  (profiles[activeProfile].speed_hz)
#define RUN_SG_TRIP  (profiles[activeProfile].sg_trip)

static int32_t upOffsetSteps   = 0;
static int32_t downOffsetSteps = 0;
static constexpr int32_t DOWN_OFFSET_DEFAULT = -500;
static constexpr int32_t OFFSET_MIN     = -8000;
static constexpr int32_t OFFSET_MAX     = +8000;
static constexpr int32_t ENDPOINT_GUARD = 50;

static constexpr int32_t CAL_PREMOVE_DOWN_STEPS = 5500;

// Calibration
static constexpr int8_t   CAL_SGT          = -1;
static uint16_t           RUN_CURRENT_MA   = 3500;
static constexpr uint16_t RUN_CURRENT_MIN  = 1000;
static constexpr uint16_t RUN_CURRENT_MAX  = 4500;
static constexpr uint16_t CAL_CURRENT_MA   = 3200;
static constexpr uint32_t CAL_SPEED_HZ     = 8000;
static constexpr uint32_t CAL_ACCEL        = 25000;
static constexpr int32_t  CAL_SEARCH_STEPS = 120000;
static constexpr uint16_t CAL_ABS_MIN      = 12;
static constexpr uint8_t  CAL_REL_DROP_Q8  = 235;
static constexpr uint8_t  CAL_HIT_CONFIRM  = 2;

static constexpr uint32_t EARLY_WINDOW_MS      = 300;
static constexpr int32_t  EARLY_WINDOW_DST_MAX = 1200;
static constexpr uint32_t EARLY_MIN_TIME_MS    = 50;
static constexpr int32_t  EARLY_MIN_MOVE_STEPS = 200;
static constexpr uint16_t EARLY_TRIP           = CAL_ABS_MIN;

// Return-home: USE SAME SPEED as calibration to avoid decel overshoot
static constexpr uint32_t HOME_SPEED_HZ      = CAL_SPEED_HZ;
static constexpr uint32_t HOME_ACCEL          = CAL_ACCEL;
static constexpr uint16_t HOME_SG_TRIP        = 15;
static constexpr uint8_t  HOME_CONFIRM        = 3;
static constexpr uint32_t HOME_MIN_MS         = 200;
static constexpr int32_t  HOME_MIN_MOVE       = 600;
static constexpr int32_t  HOME_RELEASE_STEPS  = 1200;
static constexpr uint8_t  HOME_MAX_RETRIES    = 2;
static constexpr uint32_t HOME_TIMEOUT_MS     = 15000;

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

enum RunState : uint8_t { IDLE, RUNNING, STOPPING, CALIBRATING, STALLED, HOMING };
static volatile RunState runState = IDLE;
static long     currentTarget = 0;
static uint32_t stopEntryMs   = 0;
static constexpr uint32_t STOP_TIMEOUT_MS = 8000;

// --- Runtime stall detection (during RUNNING) ---
// SG trip is now per-profile — use active_sg() to read/write
static constexpr uint16_t RUN_SG_TRIP_MIN    = 0;
static constexpr uint16_t RUN_SG_TRIP_MAX    = 500;
static constexpr int32_t  RUN_BACKOFF_STEPS  = 1000; // steps to back off after jam

// Work zone: skip SG monitoring near the DOWN endpoint where the tool
// does useful work (e.g. pushing primers). The resistance here is normal
// and would false-trigger stall detection at low trip thresholds.
// SG is still active for the rest of the travel and near the UP endpoint.
static int32_t SG_WORK_ZONE_STEPS = 5500;  // skip SG this many steps before endpointDown
static constexpr int32_t SG_WORK_ZONE_MIN = 0;
static constexpr int32_t SG_WORK_ZONE_MAX = 20000;
static constexpr uint32_t CREEP_HOME_SPEED   = CAL_SPEED_HZ;
static constexpr uint32_t CREEP_HOME_ACCEL   = CAL_ACCEL;
static constexpr uint16_t CREEP_SG_TRIP      = 15;
static constexpr uint8_t  CREEP_SG_CONFIRM   = 3;
static constexpr uint32_t CREEP_IGNORE_MS    = 300;
static constexpr uint32_t CREEP_TIMEOUT_MS   = 20000;

static uint32_t lastDirectionChangeMs = 0;
static uint8_t  runSGHighCount = 0;        // sliding counter of above-trip readings
static uint8_t  runSGLowCount = 0;         // consecutive below-threshold readings (for slow decrement)
static constexpr uint8_t RUN_SG_HIGH_NEEDED = 2;  // need this many high readings to trigger jam (no sustain timer)

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

static constexpr int SCR_W = 172, SCR_H = 320, NAV_H = 60, CONTENT_H = SCR_H - NAV_H;

static volatile bool webCalRequested = false;
static volatile bool webHomeRequested = false;
static volatile bool rebootRequested = false;
static uint32_t     rebootRequestMs = 0;

// --- Batch run ---
static int32_t  batchTarget  = 0;     // 0 = unlimited
static int32_t  batchCount   = 0;     // completed in current batch
static bool     batchActive  = false;
static lv_obj_t *batch_scr       = nullptr;
static lv_obj_t *lbl_batch_val   = nullptr;
static lv_obj_t *lbl_batch_remain = nullptr;  // on main screen
static uint32_t lastSSEMs = 0;
static constexpr uint32_t SSE_INTERVAL_MS = 250;

// --- Log ring buffer for web ---
static constexpr uint16_t LOG_LINES = 500;
static constexpr uint16_t LOG_LINE_LEN = 140;
static char logBuf[LOG_LINES][LOG_LINE_LEN];
static uint16_t logHead = 0;       // next write index in ring buffer (wraps at LOG_LINES)
static uint32_t logSerial = 0;     // monotonic total lines written (never wraps in practice)
static uint32_t logSentSerial = 0; // serial number of last line sent to client

static void webLog(const char *fmt, ...) {
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
//  UTILITY
// ==========================================================================
static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline bool nearPos(long a, long b, long tol = 2) { return labs(a - b) <= tol; }
static inline long flipTarget(long t) { return (t == endpointUp) ? endpointDown : endpointUp; }
static inline uint16_t read_sg_raw() {
  // Ensure display CS is not active (shared SPI bus)
  digitalWrite(14, HIGH);
  delayMicroseconds(10);  // let bus settle after display CS release
  uint32_t drv = driver.DRV_STATUS();
  return (uint16_t)(drv & 0x03FF);
}

// Median-of-5 filter to reject SPI glitch spikes (more robust than median-of-3)
static uint16_t read_sg() {
  uint16_t s[5];
  for (int i = 0; i < 5; i++) s[i] = read_sg_raw();
  // Simple insertion sort for 5 elements
  for (int i = 1; i < 5; i++) {
    uint16_t key = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > key) { s[j + 1] = s[j]; j--; }
    s[j + 1] = key;
  }
  return s[2];  // median
}
static inline void fas_wait_for_stop() {
  while (stepper && stepper->isRunning()) { lv_timer_handler(); delay(1); }
}

// ==========================================================================
//  ENDPOINT MATH
// ==========================================================================
static void recomputeEffectiveEndpoints() {
  if (!endpointsCalibrated) { endpointUp = 0; endpointDown = 0; return; }
  upOffsetSteps   = clamp_i32(upOffsetSteps, OFFSET_MIN, OFFSET_MAX);
  downOffsetSteps = clamp_i32(downOffsetSteps, OFFSET_MIN, OFFSET_MAX);
  long upEff = rawUp + upOffsetSteps;
  long dnEff = rawDown + downOffsetSteps;
  if (dnEff <= (upEff + ENDPOINT_GUARD)) {
    dnEff = upEff + ENDPOINT_GUARD;
    downOffsetSteps = (int32_t)(dnEff - rawDown);
  }
  endpointUp = upEff;
  endpointDown = dnEff;
}

// ==========================================================================
//  MOTION
// ==========================================================================
// Decel distance at RUN_DECEL: v²/(2*a). At 40000Hz/800000 = 1000 steps, ~50ms.
// Both accel and decel use this rate — fast ramps, maximum cruise time for SG monitoring.

static void startRunBetweenEndpoints() {
  if (!endpointsCalibrated || !stepper) return;
  runState = RUNNING;
  runSGHighCount = 0; runSGLowCount = 0;
  lastDirectionChangeMs = millis();

  driver.rms_current(RUN_CURRENT_MA);
  driver.en_pwm_mode(false);
  driver.TPWMTHRS(0);
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(0); driver.semax(0);
  driver.sgt(constrain((int8_t)CAL_SGT, (int8_t)-64, (int8_t)63));

  long pos = stepper->getCurrentPosition();
  if (nearPos(pos, endpointUp))        currentTarget = endpointDown;
  else if (nearPos(pos, endpointDown)) currentTarget = endpointUp;
  else {
    currentTarget = (labs(pos - endpointUp) < labs(pos - endpointDown)) ? endpointUp : endpointDown;
  }

  stepper->setSpeedInHz(ui_speed_hz);
  stepper->setAcceleration(RUN_DECEL);
  stepper->moveTo(currentTarget);
}

static void requestGracefulStop() {
  if (!stepper) return;
  runState = STOPPING;
  stopEntryMs = millis();
  currentTarget = endpointUp;
  stepper->setAcceleration(RUN_DECEL);  // use fast decel for stop too
  stepper->moveTo(endpointUp);
}

// Forward declarations for jam handling
static void showJamScreen();
static void onJamReturnHome(lv_event_t *e);
static void safeCreepHome();

static void handleMotion() {
  if (!stepper || runState == CALIBRATING || runState == STALLED || runState == HOMING) return;
  switch (runState) {
    case RUNNING: {
      long pos = stepper->getCurrentPosition();

      if (!stepper->isRunning()) {
        if (currentTarget == endpointDown && counter < 9999) {
          counter++;
          if (batchActive) {
            batchCount++;
            if (batchCount >= batchTarget) {
              webLog("Batch complete: %ld/%ld", batchCount, batchTarget);
              batchActive = false;
              requestGracefulStop();
              setRunButtonState(false);
              break;
            }
          }
        }
        currentTarget = flipTarget(currentTarget);
        lastDirectionChangeMs = millis();
        runSGHighCount = 0; runSGLowCount = 0;
        stepper->setSpeedInHz(ui_speed_hz);
        stepper->setAcceleration(RUN_DECEL);
        stepper->moveTo(currentTarget);
        break;
      }

      // --- Runtime stall detection ---
      if (RUN_SG_TRIP == 0) break;

      uint32_t sinceChange = millis() - lastDirectionChangeMs;

      // Accel blank: v/a at RUN_DECEL + margin
      uint32_t accelWindowMs = (uint32_t)((uint64_t)ui_speed_hz * 1000ULL / (uint64_t)RUN_DECEL) + 80;
      if (sinceChange < accelWindowMs) break;

      // Work zone: skip SG near the DOWN endpoint where the tool does work
      // (primer push etc.) — normal resistance here would false-trigger.
      // Only applies when heading toward DOWN, not toward UP.
      if (currentTarget == endpointDown) {
        int32_t distToDown = labs(pos - endpointDown);
        if (distToDown < SG_WORK_ZONE_STEPS) {
          runSGHighCount = 0; runSGLowCount = 0;
          break;
        }
      }

      // Decel blank: position-based, using actual decel distance + margin.
      // Skip UNLESS we already have jam evidence (carry-through).
      {
        int32_t distToTarget = labs(pos - currentTarget);
        int32_t decelDist = (int32_t)((uint64_t)ui_speed_hz * ui_speed_hz / (2ULL * (uint64_t)RUN_DECEL));
        int32_t decelBlank = decelDist + 500;  // margin for planner timing
        if (distToTarget < decelBlank && runSGHighCount < RUN_SG_HIGH_NEEDED) {
          runSGHighCount = 0; runSGLowCount = 0;
          break;
        }
      }

      uint16_t sg = read_sg();
      if (sg <= 1) break;

      // Debug: print SG every 500ms
      static uint32_t lastSGPrintMs = 0;
      if ((millis() - lastSGPrintMs) > 500) {
        int32_t distToTarget = labs(pos - currentTarget);
        webLog("RUN SG=%u trip=%u pos=%ld dist=%ld t=%lu hi=%u/%u",
               sg, RUN_SG_TRIP, pos, distToTarget, sinceChange,
               runSGHighCount, RUN_SG_HIGH_NEEDED);
        lastSGPrintMs = millis();
      }

      if (sg > RUN_SG_TRIP) {
        if (runSGHighCount < RUN_SG_HIGH_NEEDED + 4) runSGHighCount++;
        runSGLowCount = 0;

        webLog("SG HIGH=%u trip=%u cnt=%u pos=%ld t=%lu",
               sg, RUN_SG_TRIP, runSGHighCount, pos, sinceChange);

        if (runSGHighCount >= RUN_SG_HIGH_NEEDED) {
          // JAM — trigger immediately, no sustain timer
          webLog("JAM! SG=%u trip=%u pos=%ld tgt=%ld cnt=%u",
                        sg, RUN_SG_TRIP, pos, currentTarget, runSGHighCount);

          stepper->forceStop();
          fas_wait_for_stop();

          stepper->setSpeedInHz(CREEP_HOME_SPEED);
          stepper->setAcceleration(CREEP_HOME_ACCEL);

          int32_t backoff = (currentTarget == endpointDown) ? -RUN_BACKOFF_STEPS : +RUN_BACKOFF_STEPS;
          stepper->move(backoff);
          fas_wait_for_stop();

          runState = STALLED;
          runSGHighCount = 0; runSGLowCount = 0;
          showJamScreen();
        }
      } else {
        runSGLowCount++;
        if (runSGLowCount >= 3) {
          runSGLowCount = 0;
          if (runSGHighCount > 0) runSGHighCount--;
        }
      }
      break;
    }

    case STOPPING: {
      long pos = stepper->getCurrentPosition();
      if (!stepper->isRunning() || nearPos(pos, endpointUp, 10)) {
        if (stepper->isRunning()) stepper->forceStop();
        runState = IDLE;
        break;
      }
      if ((millis() - stopEntryMs) > STOP_TIMEOUT_MS) {
        stepper->forceStop();
        runState = IDLE;
      }
      break;
    }

    default: break;
  }
}

// Safe creep home: slow sensorless move toward UP until mechanical stop,
// then back off and re-establish position. Called from jam screen button.
// Uses the same move_until_stall() as calibration — proven to work.
static void safeCreepHome() {
  if (!stepper) return;
  runState = HOMING;

  if (jam_status_lbl) lv_label_set_text(jam_status_lbl, "Returning home...");
  lv_timer_handler();

  // Use calibration current and speed — same as what works in calibration
  driver.rms_current(CAL_CURRENT_MA);
  stepper->setSpeedInHz(CAL_SPEED_HZ);
  stepper->setAcceleration(CAL_ACCEL);

  webLog("Creep home: start, I=%umA spd=%u", CAL_CURRENT_MA, CAL_SPEED_HZ);

  // Use the exact same stall detection that calibration uses
  long hit_pos = 0;
  bool found = move_until_stall(-1, hit_pos);  // -1 = toward UP

  if (found) {
    webLog("Creep home: found stop at %ld", hit_pos);

    // Back off from the mechanical stop (same as calibration)
    stepper->move(+300);
    fas_wait_for_stop();

    // Re-zero position at the mechanical UP stop
    stepper->setCurrentPosition(0);
    rawUp = 0;

    // Recompute effective endpoints
    recomputeEffectiveEndpoints();

    // Move to the effective UP endpoint
    stepper->moveTo(endpointUp);
    fas_wait_for_stop();
  } else {
    webLog("Creep home: FAILED to find stop!");
  }

  // Restore run current and speed
  driver.rms_current(RUN_CURRENT_MA);
  stepper->setSpeedInHz(ui_speed_hz);
  stepper->setAcceleration(RUN_DECEL);

  runState = IDLE;

  webLog("Creep home: done pos=%ld", stepper->getCurrentPosition());

  if (jam_status_lbl) lv_label_set_text(jam_status_lbl, found ? "Home OK!" : "FAILED");
  lv_timer_handler();

  setRunButtonState(false);
  ui_update_main_warning();
  ui_update_tuning_numbers();
  ui_update_endpoint_edit_values();

  delay(800);
  go(main_scr);
}

// ==========================================================================
//  SENSORLESS CALIBRATION (no static locals — all stack)
// ==========================================================================
static bool move_until_stall(int dir, long &hit_pos) {
  const int32_t target = (dir > 0) ? +CAL_SEARCH_STEPS : -CAL_SEARCH_STEPS;
  const int32_t start_pos = stepper->getCurrentPosition();
  const uint32_t accel_ms = (uint32_t)((uint64_t)CAL_SPEED_HZ * 1000ULL / (uint64_t)CAL_ACCEL);
  const int32_t  accel_dist = (int32_t)((uint64_t)CAL_SPEED_HZ * (uint64_t)CAL_SPEED_HZ / (2ULL * (uint64_t)CAL_ACCEL));
  const uint32_t ignore_ms  = accel_ms + 100;
  const int32_t  ignore_dst = (accel_dist * 8) / 10;

  driver.en_pwm_mode(false);
  driver.TPWMTHRS(0);
  driver.TCOOLTHRS(0xFFFFF);
  int8_t sgt = constrain((int8_t)CAL_SGT, (int8_t)-64, (int8_t)63);
  driver.sgt(sgt);

  webLog("MUS: dir=%d pos=%ld ign_ms=%lu ign_dst=%ld sgt=%d",
         dir, start_pos, ignore_ms, ignore_dst, sgt);

  stepper->move(target);
  const uint32_t start_ms = millis();
  delay(5);

  bool baseline_started = false;
  uint32_t base_start_ms = 0, base_sum = 0;
  uint16_t base_cnt = 0;
  bool dyn_ready = false;
  uint16_t dyn_trip = CAL_ABS_MIN;
  uint8_t confirm_dyn = 0, confirm_early = 0;

  while (stepper->isRunning()) {
    const uint32_t now = millis();
    const uint32_t elapsed_ms = now - start_ms;
    const int32_t dist = labs(stepper->getCurrentPosition() - start_pos);
    const uint16_t sg = read_sg();

    // Periodic debug during search
    static uint32_t lastMUSPrint = 0;
    if ((now - lastMUSPrint) > 400) {
      webLog("MUS: sg=%u dist=%ld el=%lu bl=%d dr=%d dtrip=%u",
             sg, dist, elapsed_ms, baseline_started, dyn_ready, dyn_trip);
      lastMUSPrint = now;
    }

    // Early trip
    if (elapsed_ms <= EARLY_WINDOW_MS && dist <= EARLY_WINDOW_DST_MAX) {
      if (elapsed_ms >= EARLY_MIN_TIME_MS && dist >= EARLY_MIN_MOVE_STEPS) {
        if (sg <= EARLY_TRIP) {
          if (++confirm_early >= CAL_HIT_CONFIRM) {
            webLog("MUS: EARLY HIT sg=%u pos=%ld", sg, stepper->getCurrentPosition());
            stepper->forceStop(); fas_wait_for_stop();
            hit_pos = stepper->getCurrentPosition();
            return true;
          }
        } else confirm_early = 0;
      }
    }

    // Baseline
    if (!baseline_started && elapsed_ms > ignore_ms && dist > ignore_dst) {
      baseline_started = true;
      base_start_ms = now;
      base_sum = 0; base_cnt = 0; confirm_dyn = 0;
    }
    if (baseline_started && !dyn_ready) {
      base_sum += sg;
      if (base_cnt < 1000) base_cnt++;
      if ((now - base_start_ms) >= 200 && base_cnt > 0) {
        uint16_t baseline = min((uint16_t)(base_sum / base_cnt), (uint16_t)1023);
        uint16_t rel_trip = (uint16_t)((baseline * (uint32_t)CAL_REL_DROP_Q8) >> 8);
        dyn_trip = max(rel_trip, CAL_ABS_MIN);
        dyn_ready = true;
        webLog("MUS: baseline=%u dyn_trip=%u", baseline, dyn_trip);
      }
    }

    // Dynamic stall
    if (dyn_ready) {
      if (sg <= dyn_trip) {
        if (++confirm_dyn >= CAL_HIT_CONFIRM) {
          webLog("MUS: DYN HIT sg=%u trip=%u pos=%ld", sg, dyn_trip, stepper->getCurrentPosition());
          stepper->forceStop(); fas_wait_for_stop();
          hit_pos = stepper->getCurrentPosition();
          return true;
        }
      } else confirm_dyn = 0;
    }

    lv_timer_handler();
    delay(1);
  }
  hit_pos = stepper->getCurrentPosition();
  webLog("MUS: NO STALL DETECTED, ended at pos=%ld", hit_pos);
  return false;
}

// Return home: uses cal speed to avoid decel overshoot, NO static locals
static bool return_home_up_safe() {
  if (!stepper || !endpointsCalibrated) return false;

  stepper->setSpeedInHz(HOME_SPEED_HZ);
  stepper->setAcceleration(HOME_ACCEL);

  driver.en_pwm_mode(false);
  driver.TPWMTHRS(0);
  driver.TCOOLTHRS(0xFFFFF);
  driver.sgt(constrain((int8_t)CAL_SGT, (int8_t)-64, (int8_t)63));

  const uint32_t start_ms = millis();
  uint8_t retries = 0;
  long move_origin = stepper->getCurrentPosition();
  uint32_t move_start_ms = millis();
  uint8_t confirm_count = 0;

  stepper->moveTo(endpointUp);

  while (stepper->isRunning()) {
    const uint32_t now = millis();
    const long pos = stepper->getCurrentPosition();

    if ((now - start_ms) > HOME_TIMEOUT_MS) {
      webLog("Home: TIMEOUT");
      stepper->forceStop(); fas_wait_for_stop();
      return false;
    }

    if (nearPos(pos, endpointUp, 20)) break;

    const int32_t moved = labs(pos - move_origin);
    const uint32_t time_moving = now - move_start_ms;

    if (time_moving >= HOME_MIN_MS && moved >= HOME_MIN_MOVE) {
      const uint16_t sg = read_sg();
      if (sg <= HOME_SG_TRIP) {
        if (++confirm_count >= HOME_CONFIRM) {
          webLog("Home: stall @%ld retry %d", pos, retries);
          stepper->forceStop(); fas_wait_for_stop();
          stepper->move(+HOME_RELEASE_STEPS);
          fas_wait_for_stop();

          if (retries >= HOME_MAX_RETRIES) {
            webLog("Home: max retries");
            return false;
          }
          retries++;
          confirm_count = 0;
          move_origin = stepper->getCurrentPosition();
          move_start_ms = millis();
          stepper->moveTo(endpointUp);
        }
      } else confirm_count = 0;
    }

    lv_timer_handler();
    delay(1);
  }

  fas_wait_for_stop();
  long finalPos = stepper->getCurrentPosition();
  webLog("Home: pos=%ld tgt=%ld diff=%ld", finalPos, endpointUp, finalPos - endpointUp);
  return nearPos(finalPos, endpointUp, 50);
}

static bool calibrateEndpointsSensorless() {
  if (!stepper) return false;
  runState = CALIBRATING;
  endpointsCalibrated = false;
  if (stepper->isRunning()) { stepper->forceStop(); fas_wait_for_stop(); }

  const uint32_t saved_speed = ui_speed_hz;
  stepper->setSpeedInHz(CAL_SPEED_HZ);
  stepper->setAcceleration(CAL_ACCEL);
  driver.rms_current(CAL_CURRENT_MA);

  // Pre-move DOWN
  stepper->move(+CAL_PREMOVE_DOWN_STEPS);
  fas_wait_for_stop();

  // Find UP
  long hit_up = 0;
  if (!move_until_stall(-1, hit_up)) {
    driver.rms_current(RUN_CURRENT_MA);
    stepper->setSpeedInHz(saved_speed);
    stepper->setAcceleration(RUN_DECEL);
    runState = IDLE;
    return false;
  }

  stepper->move(+300); fas_wait_for_stop();
  stepper->setCurrentPosition(0);
  rawUp = 0;

  // Find DOWN
  long hit_down = 0;
  if (!move_until_stall(+1, hit_down)) {
    driver.rms_current(RUN_CURRENT_MA);
    stepper->setSpeedInHz(saved_speed);
    stepper->setAcceleration(RUN_DECEL);
    runState = IDLE;
    return false;
  }

  stepper->move(-300); fas_wait_for_stop();
  rawDown = stepper->getCurrentPosition();

  webLog("CAL: up=%ld dn=%ld travel=%ld", rawUp, rawDown, rawDown - rawUp);
  endpointsCalibrated = true;
  upOffsetSteps   = 0;
  downOffsetSteps = DOWN_OFFSET_DEFAULT;
  recomputeEffectiveEndpoints();
  driver.rms_current(RUN_CURRENT_MA);

  // Position is trusted after calibration — just move directly to endpointUp
  stepper->setSpeedInHz(saved_speed);
  stepper->setAcceleration(RUN_DECEL);
  stepper->moveTo(endpointUp);
  fas_wait_for_stop();

  runState = IDLE;
  return true;
}

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
//  LVGL UI HELPERS
// ==========================================================================
static void go(lv_obj_t *scr) { lv_scr_load(scr); }
static void style_screen(lv_obj_t *scr) {
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
}

static lv_obj_t* make_content(lv_obj_t *scr) {
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

static lv_obj_t* make_content_free(lv_obj_t *scr) {
  lv_obj_t *c = lv_obj_create(scr);
  lv_obj_set_size(c, SCR_W, CONTENT_H);
  lv_obj_align(c, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_opa(c, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(c, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(c, 0, LV_PART_MAIN);
  lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
  return c;
}

static lv_obj_t* make_nav(lv_obj_t *scr) {
  lv_obj_t *n = lv_obj_create(scr);
  lv_obj_set_size(n, SCR_W, NAV_H);
  lv_obj_align(n, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(n, lv_color_hex(0x0C0C0C), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(n, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(n, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(n, 10, LV_PART_MAIN);
  lv_obj_clear_flag(n, LV_OBJ_FLAG_SCROLLABLE);
  return n;
}

static lv_obj_t* make_title(lv_obj_t *p, const char *txt) {
  lv_obj_t *t = lv_label_create(p);
  lv_label_set_text(t, txt);
  lv_obj_set_style_text_font(t, &lv_font_montserrat_26, LV_PART_MAIN);
  lv_obj_set_style_text_color(t, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  return t;
}

static lv_obj_t* make_btn(lv_obj_t *p, const char *txt, int w, int h, uint32_t bg, const lv_font_t *f) {
  lv_obj_t *b = lv_btn_create(p);
  lv_obj_set_size(b, w, h);
  lv_obj_set_style_bg_color(b, lv_color_hex(bg), LV_PART_MAIN);
  lv_obj_t *l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(l, f, LV_PART_MAIN);
  lv_obj_center(l);
  return b;
}

static lv_obj_t* make_btn_multiline(lv_obj_t *p, const char *txt, int w, int h, uint32_t bg, const lv_font_t *f) {
  lv_obj_t *b = lv_btn_create(p);
  lv_obj_set_size(b, w, h);
  lv_obj_set_style_bg_color(b, lv_color_hex(bg), LV_PART_MAIN);
  lv_obj_t *l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(l, w - 8);
  lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(l, f, LV_PART_MAIN);
  lv_obj_center(l);
  return b;
}

static lv_obj_t* make_card(lv_obj_t *p, int w, int h) {
  lv_obj_t *c = lv_obj_create(p);
  lv_obj_set_size(c, w, h);
  lv_obj_set_style_bg_color(c, lv_color_hex(0x242424), LV_PART_MAIN);
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
static void showJamScreen() {
  if (jam_status_lbl) lv_label_set_text(jam_status_lbl, "Press to return home");
  go(jam_scr);
}

static void onJamReturnHome(lv_event_t *e) {
  LV_UNUSED(e);
  if (runState != STALLED) return;
  safeCreepHome();
}

// ==========================================================================
//  UI UPDATE FUNCTIONS
// ==========================================================================
static void ui_update_main_warning() {
  if (!main_warn) return;
  if (endpointsCalibrated) lv_obj_add_flag(main_warn, LV_OBJ_FLAG_HIDDEN);
  else lv_obj_clear_flag(main_warn, LV_OBJ_FLAG_HIDDEN);
}

static void ui_create_main_warning(lv_obj_t *parent) {
  main_warn = lv_obj_create(parent);
  lv_obj_set_size(main_warn, 132, 24);
  lv_obj_set_style_radius(main_warn, 12, LV_PART_MAIN);
  lv_obj_set_style_border_width(main_warn, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(main_warn, lv_color_hex(0x3A2B12), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(main_warn, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_pad_all(main_warn, 0, LV_PART_MAIN);
  lv_obj_clear_flag(main_warn, LV_OBJ_FLAG_SCROLLABLE);
  main_warn_lbl = lv_label_create(main_warn);
  lv_label_set_text(main_warn_lbl, "NOT CALIBRATED");
  lv_obj_set_style_text_font(main_warn_lbl, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_color(main_warn_lbl, lv_color_hex(0xFFD37C), LV_PART_MAIN);
  lv_obj_set_style_text_align(main_warn_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(main_warn_lbl);
  ui_update_main_warning();
}

static void ui_update_speed_val() {
  if (lbl_speed_val) lv_label_set_text_fmt(lbl_speed_val, "%s  %lukHz",
      profiles[activeProfile].name, (unsigned long)(ui_speed_hz / 1000));
}

static void ui_update_profile_screen();  // forward decl

static void setActiveProfile(uint8_t idx) {
  if (idx >= NUM_PROFILES) return;
  activeProfile = idx;
  if (stepper) {
    stepper->setSpeedInHz(ui_speed_hz);
    if (runState == RUNNING) {
      // Speed change takes effect immediately
    }
  }
  ui_update_speed_val();
  ui_update_sg_val();
  ui_update_profile_screen();
  webLog("Profile: %s spd=%lu sg=%u", profiles[idx].name,
         (unsigned long)ui_speed_hz, RUN_SG_TRIP);
}

static void ui_update_tuning_numbers() {
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

static void ui_update_endpoint_edit_values() {
  if (lbl_ep_up_val) lv_label_set_text_fmt(lbl_ep_up_val, "%ld", endpointUp);
  if (lbl_ep_dn_val) lv_label_set_text_fmt(lbl_ep_dn_val, "%ld", endpointDown);
}

static void ui_update_sg_val() {
  if (lbl_sg_val) lv_label_set_text_fmt(lbl_sg_val, "%u", RUN_SG_TRIP);
}

static void ui_update_profile_screen() {
  for (uint8_t i = 0; i < NUM_PROFILES; i++) {
    if (!profile_btns[i]) continue;
    if (i == activeProfile) {
      lv_obj_set_style_bg_color(profile_btns[i], lv_color_hex(0x1F6FEB), LV_PART_MAIN);
      lv_obj_set_style_border_width(profile_btns[i], 2, LV_PART_MAIN);
      lv_obj_set_style_border_color(profile_btns[i], lv_color_hex(0x00FF00), LV_PART_MAIN);
    } else {
      lv_obj_set_style_bg_color(profile_btns[i], lv_color_hex(0x3A3A3A), LV_PART_MAIN);
      lv_obj_set_style_border_width(profile_btns[i], 0, LV_PART_MAIN);
    }
  }
  if (lbl_profile_info) {
    lv_label_set_text_fmt(lbl_profile_info, "%luHz  SG=%u",
        (unsigned long)ui_speed_hz, RUN_SG_TRIP);
  }
}

static void ui_update_batch_val() {
  if (lbl_batch_val) {
    if (batchTarget <= 0)
      lv_label_set_text(lbl_batch_val, "OFF");
    else
      lv_label_set_text_fmt(lbl_batch_val, "%ld", batchTarget);
  }
}

static void ui_update_batch_remain() {
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

static void ui_update_wifi_label() {
  if (!lbl_wifi_status) return;
  if (wifiConnected)
    lv_label_set_text_fmt(lbl_wifi_status, "%s\nIP: %s", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  else if (wifiAPMode)
    lv_label_set_text_fmt(lbl_wifi_status, "AP: %s\n192.168.4.1\n(open, no password)", DEFAULT_AP_SSID);
  else
    lv_label_set_text(lbl_wifi_status, "Disconnected");
}

static void setRunButtonState(bool running) {
  if (!btn_run_global) return;
  lv_obj_t *l = lv_obj_get_child(btn_run_global, 0);
  if (running) {
    lv_obj_set_style_bg_color(btn_run_global, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_label_set_text(l, "STOP");
    lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  } else {
    lv_obj_set_style_bg_color(btn_run_global, lv_color_hex(0x00FF00), LV_PART_MAIN);
    lv_label_set_text(l, "RUN");
    lv_obj_set_style_text_color(l, lv_color_hex(0x000000), LV_PART_MAIN);
  }
}

// ==========================================================================
//  UI EVENT HANDLERS
// ==========================================================================
static void on_ep_up_delta(lv_event_t *e) {
  if (!endpointsCalibrated) return;
  int32_t d = (int32_t)(intptr_t)lv_event_get_user_data(e);
  upOffsetSteps = clamp_i32(upOffsetSteps + d, OFFSET_MIN, OFFSET_MAX);
  recomputeEffectiveEndpoints();
  ui_update_endpoint_edit_values();
  ui_update_tuning_numbers();
}

static void on_ep_dn_delta(lv_event_t *e) {
  if (!endpointsCalibrated) return;
  int32_t d = (int32_t)(intptr_t)lv_event_get_user_data(e);
  downOffsetSteps = clamp_i32(downOffsetSteps + d, OFFSET_MIN, OFFSET_MAX);
  recomputeEffectiveEndpoints();
  ui_update_endpoint_edit_values();
  ui_update_tuning_numbers();
}

static void build_endpoint_screen(lv_obj_t *scr, const char *titleTxt, bool isUp) {
  style_screen(scr);
  lv_obj_t *content = make_content(scr);
  lv_obj_t *nav = make_nav(scr);
  lv_obj_t *t = make_title(content, titleTxt);
  lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 2);

  lv_obj_t *card = make_card(content, 150, 92);
  lv_obj_t *ln = lv_label_create(card);
  lv_label_set_text(ln, "Steps");
  lv_obj_set_style_text_color(ln, lv_color_hex(0xCFCFCF), LV_PART_MAIN);
  lv_obj_set_style_text_font(ln, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(ln, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t *lv = lv_label_create(card);
  lv_obj_set_style_text_color(lv, lv_color_hex(0x00FF00), LV_PART_MAIN);
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
  lv_obj_t *bN100=make_btn(grid,"-100",bw,bh,0x3A3A3A,&lv_font_montserrat_16);
  lv_obj_t *bN10 =make_btn(grid,"-10", bw,bh,0x3A3A3A,&lv_font_montserrat_16);
  lv_obj_t *bN1  =make_btn(grid,"-1",  bw,bh,0x3A3A3A,&lv_font_montserrat_16);
  lv_obj_t *bP100=make_btn(grid,"+100",bw,bh,0x1F6FEB,&lv_font_montserrat_16);
  lv_obj_t *bP10 =make_btn(grid,"+10", bw,bh,0x1F6FEB,&lv_font_montserrat_16);
  lv_obj_t *bP1  =make_btn(grid,"+1",  bw,bh,0x1F6FEB,&lv_font_montserrat_16);
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

  lv_obj_t *back = make_btn(nav, "Back", 140, 44, 0x2A2A2A, &lv_font_montserrat_20);
  lv_obj_align(back, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(back, [](lv_event_t *e){ LV_UNUSED(e); go(tuning_scr); }, LV_EVENT_CLICKED, nullptr);
}

static void on_go_main(lv_event_t *e) { LV_UNUSED(e); go(main_scr); }
static void on_go_profile(lv_event_t *e) {
  LV_UNUSED(e);
  ui_update_speed_val();
  ui_update_profile_screen();
  go(profile_scr);
}
static void on_go_tuning(lv_event_t *e) {
  LV_UNUSED(e); recomputeEffectiveEndpoints(); ui_update_tuning_numbers(); go(tuning_scr);
}
static void on_go_ep_up(lv_event_t *e) {
  LV_UNUSED(e); recomputeEffectiveEndpoints(); ui_update_endpoint_edit_values(); go(ep_up_scr);
}
static void on_go_ep_dn(lv_event_t *e) {
  LV_UNUSED(e); recomputeEffectiveEndpoints(); ui_update_endpoint_edit_values(); go(ep_dn_scr);
}
// on_speed_slider removed — replaced by profile selection
static void on_calibrate(lv_event_t *e) {
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

static void counter_timer_cb(lv_timer_t *t) {
  LV_UNUSED(t);
  if (counter_label) lv_label_set_text_fmt(counter_label, "%ld", min(counter, 9999L));
  if (main_scr && lv_scr_act() == main_scr) {
    ui_update_main_warning();
    ui_update_batch_remain();
  }
}

// ==========================================================================
//  WiFi
// ==========================================================================
static void loadWiFiCredentials() {
  prefs.begin("autolee", true);
  wifiSSID = prefs.getString("ssid", "");
  wifiPass = prefs.getString("pass", "");
  prefs.end();
}

static void saveWiFiCredentials(const String &ssid, const String &pass) {
  prefs.begin("autolee", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
  wifiSSID = ssid; wifiPass = pass;
}

static void clearWiFiCredentials() {
  prefs.begin("autolee", false);
  prefs.remove("ssid"); prefs.remove("pass");
  prefs.end();
  wifiSSID = ""; wifiPass = "";
}

static void scanNetworks() {
  scannedOptionsHTML = "<option value=''>-- Select WiFi --</option>";
  int n = WiFi.scanNetworks(false, true);
  if (n <= 0) {
    scannedOptionsHTML += "<option value=''>No networks found</option>";
    return;
  }
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    int rssi = WiFi.RSSI(i);
    String sec = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "OPEN" : "SEC";
    ssid.replace("\"", "&quot;"); ssid.replace("'", "&#39;");
    ssid.replace("<", "&lt;");    ssid.replace(">", "&gt;");
    scannedOptionsHTML += "<option value=\"" + ssid + "\">" + ssid +
      " (" + rssi + " dBm " + sec + ")</option>";
  }
  WiFi.scanDelete();
}

static bool connectToWiFi(const char *ssid, const char *pass, uint32_t timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.begin(ssid, pass);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    lv_timer_handler(); delay(10);
  }
  return (WiFi.status() == WL_CONNECTED);
}

static void startWiFi() {
  loadWiFiCredentials();
  if (wifiSSID.length() > 0) {
    Serial.printf("WiFi: connecting to '%s'...\n", wifiSSID.c_str());
    if (connectToWiFi(wifiSSID.c_str(), wifiPass.c_str(), 10000)) {
      wifiConnected = true; wifiAPMode = false;
      Serial.printf("WiFi: connected! IP=%s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("WiFi: STA failed, starting captive portal");
    }
  }
  if (!wifiConnected) {
    // Start OPEN captive portal AP (no password — easier to connect)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(DEFAULT_AP_SSID);  // open AP, no password
    delay(300);
    wifiAPMode = true;
    captivePortalRunning = true;
    scanNetworks();

    // Start DNS server for captive portal redirect
    dnsServer.start(53, "*", WiFi.softAPIP());

    Serial.printf("WiFi AP: %s @ %s (captive portal)\n", DEFAULT_AP_SSID, WiFi.softAPIP().toString().c_str());
  }
  ui_update_wifi_label();
}

// ==========================================================================
//  WEB SERVER
// ==========================================================================
static String buildStateJSON() {
  char buf[900];
  const char *wfStat = wifiConnected ? "Connected" : (wifiAPMode ? "AP Mode" : "Disconnected");
  String wfSSID = wifiConnected ? WiFi.SSID() : (wifiAPMode ? String(DEFAULT_AP_SSID) : String("—"));
  String wfIP = wifiConnected ? WiFi.localIP().toString() : (wifiAPMode ? WiFi.softAPIP().toString() : String("—"));
  snprintf(buf, sizeof(buf),
    "{\"version\":\"%s\",\"state\":\"%s\",\"counter\":%ld,\"speed\":%lu,\"calibrated\":%s,"
    "\"rawUp\":%ld,\"rawDown\":%ld,\"endpointUp\":%ld,\"endpointDown\":%ld,"
    "\"upOffset\":%ld,\"downOffset\":%ld,\"position\":%ld,\"sgTrip\":%u,"
    "\"workZone\":%ld,\"currentMa\":%u,"
    "\"profileIdx\":%u,\"profileName\":\"%s\","
    "\"profiles\":[{\"name\":\"Slow\",\"hz\":%lu,\"sg\":%u},"
    "{\"name\":\"Normal\",\"hz\":%lu,\"sg\":%u},"
    "{\"name\":\"Fast\",\"hz\":%lu,\"sg\":%u}],"
    "\"wifiStatus\":\"%s\",\"wifiSSID\":\"%s\",\"wifiIP\":\"%s\","
    "\"batchTarget\":%ld,\"batchCount\":%ld,\"batchActive\":%s}",
    FW_VERSION,
    runState==RUNNING?"RUNNING":runState==STOPPING?"STOPPING":runState==CALIBRATING?"CALIBRATING":runState==STALLED?"STALLED":runState==HOMING?"HOMING":"IDLE",
    counter, (unsigned long)ui_speed_hz, endpointsCalibrated?"true":"false",
    rawUp, rawDown, endpointUp, endpointDown,
    (long)upOffsetSteps, (long)downOffsetSteps,
    stepper ? stepper->getCurrentPosition() : 0L,
    RUN_SG_TRIP,
    (long)SG_WORK_ZONE_STEPS, RUN_CURRENT_MA,
    activeProfile, profiles[activeProfile].name,
    (unsigned long)profiles[0].speed_hz, profiles[0].sg_trip,
    (unsigned long)profiles[1].speed_hz, profiles[1].sg_trip,
    (unsigned long)profiles[2].speed_hz, profiles[2].sg_trip,
    wfStat, wfSSID.c_str(), wfIP.c_str(),
    batchTarget, batchCount,
    batchActive ? "true" : "false");
  return String(buf);
}

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AutoLee</title>
<style>
:root{--bg:#111;--box:#1b1b1b;--card:#222;--accent:#1F6FEB;--green:#00FF00;--red:#FF4444;--text:#eee;--muted:#888;--dim:#555;--border:#333}
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:var(--bg);color:var(--text);min-height:100vh;display:flex;justify-content:center;padding:16px 12px}
.wrap{width:100%;max-width:420px}
h1{font-size:1.5em;text-align:center;margin-bottom:2px}
.sub{color:var(--dim);font-size:.8em;text-align:center;margin-bottom:16px}
.sec{background:var(--box);border-radius:12px;padding:16px;margin-bottom:10px}
.sec h2{font-size:.85em;color:var(--muted);text-transform:uppercase;letter-spacing:.06em;margin-bottom:10px}
.badge{display:inline-block;padding:3px 10px;border-radius:10px;font-size:.75em;font-weight:600}
.badge.ok{background:#0d3320;color:var(--green)}.badge.warn{background:#3A2B12;color:#FFD37C}.badge.run{background:#331111;color:var(--red)}.badge.stall{background:#441111;color:#FF8844}
.counter{font-size:3.2em;font-weight:700;color:var(--green);text-align:center;margin:6px 0;font-variant-numeric:tabular-nums;line-height:1.1}
.btn{display:inline-block;padding:10px 20px;border:none;border-radius:8px;font-size:.9em;font-weight:600;cursor:pointer;color:#fff;text-align:center;transition:opacity .15s}
.btn:hover{opacity:.85}.btn:active{opacity:.7}.btn:disabled{opacity:.35;cursor:not-allowed}
.btn-run{background:var(--green);color:#000;font-size:1.1em;width:100%;padding:14px;border-radius:10px}.btn-run.active{background:var(--red);color:#fff}
.btn-blue{background:var(--accent)}.btn-dark{background:#333}.btn-red{background:#B42318}
.btn-sm{padding:6px 10px;font-size:.8em;border-radius:6px}
.row{display:flex;gap:6px;align-items:center;flex-wrap:wrap}.row.ctr{justify-content:center}
.sr{display:flex;justify-content:space-between;padding:3px 0;font-size:.85em}.sr .l{color:var(--muted)}.sr .v{font-weight:600;font-variant-numeric:tabular-nums}
.slider-row{display:flex;align-items:center;gap:10px;margin:6px 0}
.slider-row input[type=range]{flex:1;height:4px;-webkit-appearance:none;background:var(--border);border-radius:2px;outline:none}
.slider-row input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;background:var(--accent);cursor:pointer}
.slider-row span{min-width:50px;text-align:right;color:#fff;font-size:.85em;font-weight:600;font-variant-numeric:tabular-nums}
.ea{display:flex;gap:3px;align-items:center;justify-content:center;margin:6px 0}
.hint{font-size:.75em;color:var(--dim);margin:2px 0 6px}
hr{border:none;border-top:1px solid var(--border);margin:10px 0}
details{margin-top:10px}
details summary{color:var(--accent);font-size:.85em;cursor:pointer;padding:4px 0;user-select:none}
details summary:hover{opacity:.8}
details[open] summary{margin-bottom:8px}
.jam-alert{display:none;background:#2a1111;border:1px solid #442222;border-radius:10px;padding:12px;margin-top:10px;text-align:center}
input[type=text],input[type=password]{width:100%;padding:10px;margin-bottom:6px;background:var(--card);border:1px solid var(--border);border-radius:8px;color:#fff;font-size:.9em}
.upload{border:2px dashed #444;border-radius:8px;padding:16px;text-align:center;color:var(--muted);cursor:pointer;font-size:.85em}
.upload:hover{border-color:var(--accent)}.upload.on{border-color:var(--green);color:var(--green)}
.pbar{width:100%;height:5px;background:#333;border-radius:3px;margin-top:6px;overflow:hidden;display:none}
.pbar .fill{height:100%;background:var(--accent);width:0%;transition:width .3s;border-radius:3px}
#otaS{font-size:.8em;margin-top:4px;min-height:1em}
.log{background:#000;border-radius:8px;padding:8px;font-family:'Courier New',monospace;font-size:.7em;color:#0f0;height:400px;overflow-y:auto;white-space:pre-wrap;word-break:break-all}
.page{display:none}.page.active{display:block}
.nav-footer{margin-top:16px;padding:14px 0;text-align:center;border-top:1px solid var(--border)}
.nav-footer a{color:var(--accent);text-decoration:none;font-size:.85em;font-weight:600;margin:0 10px;cursor:pointer}
.nav-footer a:hover{opacity:.7}
.nav-footer a.active{color:var(--green)}
.back-link{display:inline-block;color:var(--accent);font-size:.85em;font-weight:600;cursor:pointer;margin-bottom:12px;text-decoration:none}
.back-link:hover{opacity:.7}
</style></head><body>
<div class="wrap">

<h1>AutoLee</h1>
<div class="sub">by K.L Design · <span id="ver"></span></div>

<!-- ==================== MAIN PAGE ==================== -->
<div id="pageMain" class="page active">

<!-- STATUS + COUNTER + RUN -->
<div class="sec">
<div class="row ctr" style="gap:6px;margin-bottom:6px">
<span id="cb" class="badge warn">NOT CALIBRATED</span>
<span id="sb" class="badge ok">IDLE</span></div>
<div class="counter" id="ctr">0</div>
<button class="btn btn-run" id="br" onclick="toggleRun()">RUN</button>
<div class="jam-alert" id="jamAlert">
<div style="color:#FF4444;font-weight:700;margin-bottom:4px">&#9888; JAM DETECTED</div>
<div style="color:#aaa;font-size:.8em;margin-bottom:8px">Motor stalled and backed off.</div>
<button class="btn btn-blue" onclick="doAct('return_home')" id="bh">Return Home</button>
</div>
<div class="row ctr" style="margin-top:10px;gap:8px">
<button class="btn btn-dark" onclick="doAct('calibrate')" id="bc">Calibrate</button>
<button class="btn btn-red" onclick="doAct('reset_counter')">Reset Counter</button></div>
<hr>
<h2 style="margin-top:8px">Batch Run</h2>
<div class="sr"><span class="l">Target</span><span class="v" id="btv">OFF</span></div>
<div id="btStatus" class="hint"></div>
<div class="ea">
<button class="btn btn-dark btn-sm" onclick="setBatch(-100)">-100</button>
<button class="btn btn-dark btn-sm" onclick="setBatch(-10)">-10</button>
<button class="btn btn-dark btn-sm" onclick="setBatch(-1)">-1</button>
<button class="btn btn-blue btn-sm" onclick="setBatch(1)">+1</button>
<button class="btn btn-blue btn-sm" onclick="setBatch(10)">+10</button>
<button class="btn btn-blue btn-sm" onclick="setBatch(100)">+100</button></div>
<div class="row ctr" style="margin-top:8px;gap:6px">
<button class="btn btn-blue btn-sm" onclick="doBatch('start')" id="bbStart">Start Batch</button>
<button class="btn btn-dark btn-sm" onclick="doBatch('clear')">Clear</button></div>
</div>

<!-- SPEED PROFILE -->
<div class="sec">
<h2>Speed Profile</h2>
<div class="row ctr" id="profileRow" style="gap:6px;margin-bottom:8px"></div>
<div class="sr"><span class="l">Active</span><span class="v" id="sv">-</span></div>
</div>

<!-- NAV FOOTER -->
<div class="nav-footer">
<a onclick="showPage('pageConfig')">Configuration</a>
<a onclick="showPage('pageLog')">Log</a>
<a onclick="showPage('pageFW')">Firmware</a>
<a onclick="showPage('pageWifi')">WiFi</a>
</div>

</div><!-- /pageMain -->

<!-- ==================== CONFIGURATION PAGE ==================== -->
<div id="pageConfig" class="page">
<a class="back-link" onclick="showPage('pageMain')">&#8592; Back</a>

<!-- MOTOR CURRENT -->
<div class="sec">
<h2>Motor Current</h2>
<div class="slider-row">
<input type="range" id="mcSlider" min="1000" max="4500" step="100" value="2500" oninput="setCurrent(this.value)">
<span id="mcv">2500</span>
</div>
<div class="hint">Run current in mA (1000–4500). Higher = more torque, more heat.</div>
<div id="mcWarn" style="display:none;background:#3A2B12;border-radius:6px;padding:6px 10px;margin-top:6px;font-size:.8em;color:#FFD37C">&#9888; Above 4000 mA exceeds motor rating. Ensure adequate cooling.</div>
</div>

<!-- ENDPOINT TUNING -->
<div class="sec">
<h2>Endpoint Tuning</h2>
<div class="sr"><span class="l">Position</span><span class="v" id="cp">-</span></div>
<div class="sr"><span class="l">Effective UP</span><span class="v" id="eu">-</span></div>
<div class="sr"><span class="l">Effective DOWN</span><span class="v" id="ed">-</span></div>
<details><summary>Endpoint offsets</summary>
<div class="sr"><span class="l">RAW UP</span><span class="v" id="ru">-</span></div>
<div class="sr"><span class="l">RAW DOWN</span><span class="v" id="rd">-</span></div>
<div class="sr"><span class="l">RAW TRAVEL</span><span class="v" id="rt">-</span></div>
<hr>
<div class="hint">UP offset</div>
<div class="ea">
<button class="btn btn-dark btn-sm" onclick="adj('up',-100)">-100</button>
<button class="btn btn-dark btn-sm" onclick="adj('up',-10)">-10</button>
<button class="btn btn-dark btn-sm" onclick="adj('up',-1)">-1</button>
<button class="btn btn-blue btn-sm" onclick="adj('up',1)">+1</button>
<button class="btn btn-blue btn-sm" onclick="adj('up',10)">+10</button>
<button class="btn btn-blue btn-sm" onclick="adj('up',100)">+100</button></div>
<div class="hint" style="margin-top:8px">DOWN offset</div>
<div class="ea">
<button class="btn btn-dark btn-sm" onclick="adj('down',-100)">-100</button>
<button class="btn btn-dark btn-sm" onclick="adj('down',-10)">-10</button>
<button class="btn btn-dark btn-sm" onclick="adj('down',-1)">-1</button>
<button class="btn btn-blue btn-sm" onclick="adj('down',1)">+1</button>
<button class="btn btn-blue btn-sm" onclick="adj('down',10)">+10</button>
<button class="btn btn-blue btn-sm" onclick="adj('down',100)">+100</button></div>
</details>
</div>

<!-- STALL GUARD + WORK ZONE -->
<div class="sec">
<h2>Stall Guard (per profile)</h2>
<div id="sgProfiles"></div>
<hr>
<div class="sr"><span class="l">Work Zone (steps)</span><span class="v" id="wzv">5500</span></div>
<div class="hint">Skip SG near DOWN endpoint (primer push area)</div>
<div class="ea">
<button class="btn btn-dark btn-sm" onclick="setWz(-500)">-500</button>
<button class="btn btn-dark btn-sm" onclick="setWz(-100)">-100</button>
<button class="btn btn-blue btn-sm" onclick="setWz(100)">+100</button>
<button class="btn btn-blue btn-sm" onclick="setWz(500)">+500</button></div>
</div>

<!-- NAV FOOTER -->
<div class="nav-footer">
<a onclick="showPage('pageMain')">Main</a>
<a onclick="showPage('pageLog')">Log</a>
<a onclick="showPage('pageFW')">Firmware</a>
<a onclick="showPage('pageWifi')">WiFi</a>
</div>

</div><!-- /pageConfig -->

<!-- ==================== LOG PAGE ==================== -->
<div id="pageLog" class="page">
<a class="back-link" onclick="showPage('pageMain')">&#8592; Back</a>

<div class="sec">
<h2>Log</h2>
<div class="log" id="logBox"></div>
<button class="btn btn-dark btn-sm" onclick="document.getElementById('logBox').textContent='';fetch('/api/log_clear',{method:'POST'})" style="margin-top:8px;width:100%">Clear Log</button>
</div>

<!-- NAV FOOTER -->
<div class="nav-footer">
<a onclick="showPage('pageMain')">Main</a>
<a onclick="showPage('pageConfig')">Configuration</a>
<a onclick="showPage('pageFW')">Firmware</a>
<a onclick="showPage('pageWifi')">WiFi</a>
</div>

</div><!-- /pageLog -->

<!-- ==================== FIRMWARE PAGE ==================== -->
<div id="pageFW" class="page">
<a class="back-link" onclick="showPage('pageMain')">&#8592; Back</a>

<div class="sec">
<h2>Firmware Update (OTA)</h2>
<div class="upload" id="ua" onclick="document.getElementById('fw').click()">
Tap to select .bin<br><span style="font-size:.8em">or drag &amp; drop</span></div>
<input type="file" id="fw" accept=".bin" style="display:none" onchange="upFW(this.files[0])">
<div class="pbar" id="pb"><div class="fill" id="pf"></div></div>
<div id="otaS"></div>
</div>

<!-- NAV FOOTER -->
<div class="nav-footer">
<a onclick="showPage('pageMain')">Main</a>
<a onclick="showPage('pageConfig')">Configuration</a>
<a onclick="showPage('pageLog')">Log</a>
<a onclick="showPage('pageWifi')">WiFi</a>
</div>

</div><!-- /pageFW -->

<!-- ==================== WIFI PAGE ==================== -->
<div id="pageWifi" class="page">
<a class="back-link" onclick="showPage('pageMain')">&#8592; Back</a>

<div class="sec">
<h2>Connection</h2>
<div class="sr"><span class="l">Status</span><span class="v" id="wfStatus">—</span></div>
<div class="sr"><span class="l">SSID</span><span class="v" id="wfSSID">—</span></div>
<div class="sr"><span class="l">IP Address</span><span class="v" id="wfIP">—</span></div>
</div>

<div class="sec">
<h2>WiFi Settings</h2>
<div class="hint" style="margin-bottom:8px">Change WiFi credentials (reboot required)</div>
<input type="text" id="ns" placeholder="SSID">
<input type="password" id="np" placeholder="Password">
<div class="row" style="gap:6px">
<button class="btn btn-blue btn-sm" onclick="saveWifi()" style="flex:1">Save &amp; Reboot</button>
<button class="btn btn-red btn-sm" onclick="resetWifi()" style="flex:1">Reset WiFi</button></div>
</div>

<!-- NAV FOOTER -->
<div class="nav-footer">
<a onclick="showPage('pageMain')">Main</a>
<a onclick="showPage('pageConfig')">Configuration</a>
<a onclick="showPage('pageLog')">Log</a>
<a onclick="showPage('pageFW')">Firmware</a>
</div>

</div><!-- /pageWifi -->

</div>

<script>
let es;
function sse(){es=new EventSource('/events');es.onmessage=e=>{try{upd(JSON.parse(e.data))}catch(x){}};
es.addEventListener('log',e=>{try{const d=JSON.parse(e.data);const lb=document.getElementById('logBox');lb.textContent+=d.log.join('\n')+'\n';lb.scrollTop=lb.scrollHeight}catch(x){}});
es.onerror=()=>{es.close();setTimeout(sse,3000)}}
sse();

function showPage(id){
  document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  window.scrollTo(0,0);
}


let profBuilt=false;
function buildProfileBtns(profiles,activeIdx){
  const row=document.getElementById('profileRow');
  if(!row)return;
  if(profBuilt){
    // Just update active highlight
    profiles.forEach((p,i)=>{
      const b=document.getElementById('profBtn'+i);
      if(b) b.className='btn '+(i===activeIdx?'btn-blue':'btn-dark')+' btn-sm';
    });
    return;
  }
  row.innerHTML='';
  profiles.forEach((p,i)=>{
    const b=document.createElement('button');
    b.id='profBtn'+i;
    b.className='btn '+(i===activeIdx?'btn-blue':'btn-dark')+' btn-sm';
    b.style.cssText='flex:1;padding:10px 4px';
    b.innerHTML=p.name+'<br><span style="font-size:.75em;opacity:.7">'+Math.round(p.hz/1000)+'kHz</span>';
    b.onclick=()=>setProfile(i);
    row.appendChild(b);
  });
  profBuilt=true;
}

let sgBuilt=false;
function buildSgControls(profiles,activeIdx){
  const c=document.getElementById('sgProfiles');
  if(!c)return;
  // Skip full rebuild if already built and an input is focused
  if(sgBuilt){
    // Just update the display values and highlights
    profiles.forEach((p,i)=>{
      const isActive=i===activeIdx;
      const lbl=document.getElementById('sgLbl'+i);
      const inp=document.getElementById('sgIn'+i);
      const row=document.getElementById('sgRow'+i);
      if(lbl) lbl.innerHTML=p.name+' ('+Math.round(p.hz/1000)+'kHz) <span style="color:'+(isActive?'var(--green)':'var(--muted)')+'">SG='+p.sg+'</span>';
      if(row) row.style.background=isActive?'#1a2a3a':'#161616';
      // Only update input if it's not focused (user might be typing)
      if(inp && document.activeElement!==inp) inp.value=p.sg;
    });
    return;
  }
  // First build
  c.innerHTML='';
  profiles.forEach((p,i)=>{
    const isActive=i===activeIdx;
    const div=document.createElement('div');
    div.id='sgRow'+i;
    div.style.cssText='margin-bottom:8px;padding:6px 8px;border-radius:8px;background:'+(isActive?'#1a2a3a':'#161616');
    div.innerHTML='<div class="sr" id="sgLbl'+i+'" style="margin-bottom:4px">'+p.name+' ('+Math.round(p.hz/1000)+'kHz) <span style="color:'+(isActive?'var(--green)':'var(--muted)')+'">SG='+p.sg+'</span></div>'
      +'<div style="display:flex;align-items:center;gap:8px">'
      +'<input type="text" inputmode="numeric" pattern="[0-9]*" id="sgIn'+i+'" value="'+p.sg+'" style="width:80px;padding:6px 8px;background:#222;border:1px solid #444;border-radius:6px;color:#fff;font-size:.9em;text-align:center" placeholder="0-500">'
      +'<button class="btn btn-blue btn-sm" id="sgBtn'+i+'">Set</button>'
      +'<span style="color:var(--dim);font-size:.7em">0 – 500</span></div>';
    c.appendChild(div);
    // Attach event listeners properly (not via inline onclick)
    document.getElementById('sgBtn'+i).addEventListener('click',function(){setSg(i)});
    document.getElementById('sgIn'+i).addEventListener('keydown',function(e){if(e.key==='Enter'){e.preventDefault();setSg(i)}});
    document.getElementById('sgIn'+i).addEventListener('blur',function(){setSg(i)});
  });
  sgBuilt=true;
}

function upd(d){
  if(d.version)document.getElementById('ver').textContent='v'+d.version;
  document.getElementById('ctr').textContent=d.counter;
  document.getElementById('sv').textContent=d.profileName+' \u2014 '+d.speed+'Hz (SG='+d.sgTrip+')';
    document.getElementById('cp').textContent=d.position;
    document.getElementById('wzv').textContent=d.workZone;
  if(d.wifiStatus){document.getElementById('wfStatus').textContent=d.wifiStatus;document.getElementById('wfSSID').textContent=d.wifiSSID;document.getElementById('wfIP').textContent=d.wifiIP}
  if(d.currentMa){document.getElementById('mcv').textContent=d.currentMa;document.getElementById('mcSlider').value=d.currentMa;document.getElementById('mcWarn').style.display=d.currentMa>4000?'block':'none'}
  if(d.profiles){buildProfileBtns(d.profiles,d.profileIdx);buildSgControls(d.profiles,d.profileIdx)}
  document.getElementById('btv').textContent=d.batchTarget>0?d.batchTarget:'OFF';
  const bts=document.getElementById('btStatus');
  const bbs=document.getElementById('bbStart');
  if(d.batchActive){bts.textContent='Running: '+d.batchCount+'/'+d.batchTarget;bts.style.color='var(--green)';bbs.disabled=true;bbs.textContent='Running...'}
  else if(d.batchTarget>0&&d.batchCount>=d.batchTarget){bts.textContent='Batch complete!';bts.style.color='var(--green)';bbs.disabled=false;bbs.textContent='Start Batch'}
  else{bts.textContent=d.batchTarget>0?'Ready':'Set a target count';bts.style.color='var(--muted)';bbs.disabled=d.batchTarget<=0;bbs.textContent='Start Batch'}
  const cb=document.getElementById('cb');
  cb.textContent=d.calibrated?'CALIBRATED':'NOT CALIBRATED';
  cb.className='badge '+(d.calibrated?'ok':'warn');
  const sb=document.getElementById('sb');
  sb.textContent=d.state;
  sb.className='badge '+(d.state==='RUNNING'?'run':d.state==='CALIBRATING'?'warn':d.state==='STALLED'||d.state==='HOMING'?'stall':'ok');
  const ja=document.getElementById('jamAlert');
  if(d.state==='STALLED'){ja.style.display='block';document.getElementById('bh').disabled=false;document.getElementById('bh').textContent='Return Home'}
  else if(d.state==='HOMING'){ja.style.display='block';document.getElementById('bh').disabled=true;document.getElementById('bh').textContent='Returning...'}
  else{ja.style.display='none'}
  const br=document.getElementById('br');
  if(d.state==='RUNNING'){br.textContent='STOP';br.classList.add('active')}
  else{br.textContent='RUN';br.classList.remove('active')}
  const bc=document.getElementById('bc');
  bc.disabled=d.state==='CALIBRATING';
  bc.textContent=d.state==='CALIBRATING'?'Calibrating...':'Calibrate';
  if(d.calibrated){
    document.getElementById('ru').textContent=d.rawUp;
    document.getElementById('rd').textContent=d.rawDown;
    document.getElementById('rt').textContent=d.rawDown-d.rawUp;
    document.getElementById('eu').textContent=d.endpointUp+' (off '+(d.upOffset>=0?'+':'')+d.upOffset+')';
    document.getElementById('ed').textContent=d.endpointDown+' (off '+(d.downOffset>=0?'+':'')+d.downOffset+')';
  }else{['ru','rd','rt','eu','ed'].forEach(i=>document.getElementById(i).textContent='-')}
}

function toggleRun(){fetch('/api/toggle_run',{method:'POST'})}
function setSg(p){const v=parseInt(document.getElementById('sgIn'+p).value)||0;fetch('/api/sg_trip?profile='+p+'&value='+v,{method:'POST'})}
function setWz(d){fetch('/api/work_zone?delta='+d,{method:'POST'})}
function setBatch(d){fetch('/api/batch?delta='+d,{method:'POST'})}
function doBatch(a){fetch('/api/batch?action='+a,{method:'POST'})}
function setProfile(i){fetch('/api/profile?idx='+i,{method:'POST'})}
function setCurrent(v){document.getElementById('mcv').textContent=v;document.getElementById('mcWarn').style.display=v>4000?'block':'none';fetch('/api/current?ma='+v,{method:'POST'})}
function adj(w,d){fetch('/api/endpoint?which='+w+'&delta='+d,{method:'POST'})}
function doAct(a){fetch('/api/action?do='+a,{method:'POST'})}
function saveWifi(){
  const s=document.getElementById('ns').value,p=document.getElementById('np').value;
  if(!s){alert('SSID required');return}
  fetch('/api/wifi?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p),{method:'POST'})
  .then(()=>{alert('Saved! Rebooting...');setTimeout(()=>location.reload(),5000)})}
function resetWifi(){
  if(!confirm('Clear saved WiFi credentials and reboot into setup mode?'))return;
  fetch('/api/wifi_reset',{method:'POST'})
  .then(()=>{alert('WiFi cleared! Rebooting into setup mode...');setTimeout(()=>location.reload(),5000)})}

function upFW(f){
  if(!f)return;
  const ua=document.getElementById('ua'),pb=document.getElementById('pb'),pf=document.getElementById('pf'),st=document.getElementById('otaS');
  ua.classList.add('on');ua.textContent=f.name;pb.style.display='block';pf.style.width='0%';
  st.textContent='Uploading...';st.style.color='#888';
  const x=new XMLHttpRequest();x.open('POST','/api/ota');
  x.upload.onprogress=e=>{if(e.lengthComputable){const p=Math.round(e.loaded/e.total*100);pf.style.width=p+'%';st.textContent='Uploading... '+p+'%'}};
  x.onload=()=>{if(x.status===200){pf.style.width='100%';st.textContent='Success! Rebooting...';st.style.color='#00FF00';setTimeout(()=>location.reload(),5000)}
  else{st.textContent='Error: '+x.responseText;st.style.color='#FF4444'}};
  x.onerror=()=>{st.textContent='Upload failed';st.style.color='#FF4444'};
  const fd=new FormData();fd.append('firmware',f);x.send(fd)}

const uae=document.getElementById('ua');
uae.addEventListener('dragover',e=>{e.preventDefault();uae.classList.add('on')});
uae.addEventListener('dragleave',()=>uae.classList.remove('on'));
uae.addEventListener('drop',e=>{e.preventDefault();upFW(e.dataTransfer.files[0])});
fetch('/api/state').then(r=>r.json()).then(upd).catch(()=>{});
</script></body></html>
)rawliteral";


// ==========================================================================
//  WiFi Setup Page (captive portal)
// ==========================================================================
static const char WIFI_CSS[] PROGMEM =
  "body{font-family:-apple-system,sans-serif;background:#111;color:#eee;padding:20px;}"
  "h2{color:#7cf;}"
  "input,select{width:100%;padding:12px;margin:6px 0;border-radius:8px;"
  "border:1px solid #444;background:#222;color:#fff;box-sizing:border-box;}"
  "button{width:100%;padding:12px;margin-top:10px;border:none;border-radius:8px;"
  "color:#fff;font-size:16px;cursor:pointer;}"
  ".btnSave{background:#28a745;}.btnClear{background:#c0392b;}"
  ".box{max-width:420px;margin:auto;background:#1b1b1b;padding:20px;border-radius:12px;}"
  "label{display:block;margin-top:10px;font-size:14px;color:#aaa;}";

static String wifiConfigPage() {
  String html;
  html.reserve(3000);
  html += "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>AutoLee WiFi Setup</title><style>";
  html += FPSTR(WIFI_CSS);
  html += "</style></head><body><div class='box'>";
  html += "<h2>AutoLee WiFi Setup</h2>";
  html += "<p style='color:#aaa;font-size:13px;'>by K.L Design</p>";
  html += "<form method='POST' action='/save'>";
  html += "<label>Select Network</label><select name='ssid_select'>" + scannedOptionsHTML + "</select>";
  html += "<label>Or type SSID manually</label><input name='ssid_manual' placeholder='SSID (optional)'>";
  html += "<label>Password</label><input name='pass' type='password' placeholder='WiFi password'>";
  html += "<button class='btnSave' type='submit'>Save &amp; Reboot</button></form>";
  html += "<form method='POST' action='/clear'><button class='btnClear' type='submit'>Clear Saved WiFi</button></form>";
  html += "</div></body></html>";
  return html;
}

static void setupCaptiveProbeEndpoints() {
  const char* probes[] = {
    "/generate_204", "/gen_204", "/hotspot-detect.html",
    "/library/test/success.html", "/ncsi.txt", "/connecttest.txt", "/fwlink"
  };
  for (auto &p : probes)
    webServer.on(p, HTTP_GET, [](AsyncWebServerRequest *r){ r->redirect("/"); });
}

static void setupWebServer() {
  // Root page: WiFi config in AP mode, control panel in STA mode
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (wifiAPMode && !wifiConnected) {
      req->send(200, "text/html", wifiConfigPage());
    } else {
      req->send_P(200, "text/html", INDEX_HTML);
    }
  });

  // WiFi setup save (captive portal)
  webServer.on("/save", HTTP_POST, [](AsyncWebServerRequest *req) {
    String ss, sm, pw;
    if (req->hasParam("ssid_select", true)) ss = req->getParam("ssid_select", true)->value();
    if (req->hasParam("ssid_manual", true)) sm = req->getParam("ssid_manual", true)->value();
    if (req->hasParam("pass", true))        pw = req->getParam("pass", true)->value();
    sm.trim(); ss.trim();
    String finalSSID = sm.length() ? sm : ss;
    if (finalSSID.length() == 0) {
      req->send(400, "text/html",
        "<html><body style='font-family:sans-serif;text-align:center;padding:40px;background:#111;color:#eee;'>"
        "<h2>Missing SSID</h2><p><a href='/' style='color:#7cf;'>Go back</a></p></body></html>");
      return;
    }
    saveWiFiCredentials(finalSSID, pw);
    req->send(200, "text/html",
      "<html><body style='font-family:sans-serif;text-align:center;padding:40px;background:#111;color:#eee;'>"
      "<h2 style='color:#28a745;'>Saved!</h2><p>Rebooting...</p></body></html>");
    rebootRequested = true;
    rebootRequestMs = millis();
  });

  // WiFi clear
  webServer.on("/clear", HTTP_POST, [](AsyncWebServerRequest *req) {
    clearWiFiCredentials();
    req->send(200, "text/html",
      "<html><body style='font-family:sans-serif;text-align:center;padding:40px;background:#111;color:#eee;'>"
      "<h2>WiFi Cleared</h2><p>Rebooting...</p></body></html>");
    rebootRequested = true;
    rebootRequestMs = millis();
  });

  events.onConnect([](AsyncEventSourceClient *client) {
    client->send(buildStateJSON().c_str(), NULL, millis(), 500);
  });
  webServer.addHandler(&events);

  webServer.on("/api/state", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", buildStateJSON());
  });

  webServer.on("/api/toggle_run", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (runState == IDLE) {
      startRunBetweenEndpoints();
      setRunButtonState(runState == RUNNING);
    } else if (runState == RUNNING) {
      requestGracefulStop();
      setRunButtonState(false);
    }
    req->send(200, "text/plain", "ok");
  });

  webServer.on("/api/profile", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (req->hasParam("idx")) {
      uint8_t idx = (uint8_t)req->getParam("idx")->value().toInt();
      if (idx < NUM_PROFILES) {
        setActiveProfile(idx);
      }
    }
    req->send(200, "text/plain", "ok");
  });

  webServer.on("/api/current", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (req->hasParam("ma")) {
      uint16_t ma = (uint16_t)req->getParam("ma")->value().toInt();
      RUN_CURRENT_MA = constrain(ma, RUN_CURRENT_MIN, RUN_CURRENT_MAX);
      driver.rms_current(RUN_CURRENT_MA);
      webLog("Current set to %u mA", RUN_CURRENT_MA);
    }
    req->send(200, "text/plain", "ok");
  });

  webServer.on("/api/endpoint", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (req->hasParam("which") && req->hasParam("delta") && endpointsCalibrated) {
      String w = req->getParam("which")->value();
      int32_t d = req->getParam("delta")->value().toInt();
      if (w == "up") upOffsetSteps = clamp_i32(upOffsetSteps + d, OFFSET_MIN, OFFSET_MAX);
      else downOffsetSteps = clamp_i32(downOffsetSteps + d, OFFSET_MIN, OFFSET_MAX);
      recomputeEffectiveEndpoints();
      ui_update_endpoint_edit_values();
      ui_update_tuning_numbers();
    }
    req->send(200, "text/plain", "ok");
  });

  webServer.on("/api/sg_trip", HTTP_POST, [](AsyncWebServerRequest *req) {
    uint8_t tgt = activeProfile;
    if (req->hasParam("profile")) {
      uint8_t p = (uint8_t)req->getParam("profile")->value().toInt();
      if (p < NUM_PROFILES) tgt = p;
    }
    if (req->hasParam("value")) {
      // Absolute value (from web text input)
      int32_t v = req->getParam("value")->value().toInt();
      profiles[tgt].sg_trip = (uint16_t)constrain(v, (int32_t)RUN_SG_TRIP_MIN, (int32_t)RUN_SG_TRIP_MAX);
      ui_update_sg_val();
      ui_update_profile_screen();
    } else if (req->hasParam("delta")) {
      // Relative delta (from touch UI)
      int32_t d = req->getParam("delta")->value().toInt();
      int32_t v = (int32_t)profiles[tgt].sg_trip + d;
      profiles[tgt].sg_trip = (uint16_t)constrain(v, (int32_t)RUN_SG_TRIP_MIN, (int32_t)RUN_SG_TRIP_MAX);
      ui_update_sg_val();
      ui_update_profile_screen();
    }
    req->send(200, "text/plain", "ok");
  });

  webServer.on("/api/work_zone", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (req->hasParam("delta")) {
      int32_t d = req->getParam("delta")->value().toInt();
      int32_t v = SG_WORK_ZONE_STEPS + d;
      SG_WORK_ZONE_STEPS = constrain(v, SG_WORK_ZONE_MIN, SG_WORK_ZONE_MAX);
    }
    req->send(200, "text/plain", "ok");
  });

  webServer.on("/api/batch", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (req->hasParam("delta")) {
      int32_t d = req->getParam("delta")->value().toInt();
      int32_t v = batchTarget + d;
      batchTarget = (v < 0) ? 0 : v;
      if (batchTarget > 9999) batchTarget = 9999;
      ui_update_batch_val();
    }
    if (req->hasParam("action")) {
      String a = req->getParam("action")->value();
      if (a == "start" && batchTarget > 0 && runState == IDLE && endpointsCalibrated) {
        batchCount = 0;
        batchActive = true;
        startRunBetweenEndpoints();
        setRunButtonState(true);
      } else if (a == "clear") {
        batchTarget = 0;
        batchCount = 0;
        batchActive = false;
        ui_update_batch_val();
      }
    }
    req->send(200, "text/plain", "ok");
  });

  webServer.on("/api/action", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (req->hasParam("do")) {
      String action = req->getParam("do")->value();
      if (action == "calibrate" && runState == IDLE) {
        webCalRequested = true;
      } else if (action == "return_home" && runState == STALLED) {
        webHomeRequested = true;
      } else if (action == "reset_counter") {
        counter = 0;
        if (counter_label) lv_label_set_text(counter_label, "0");
      }
    }
    req->send(200, "text/plain", "ok");
  });

  webServer.on("/api/wifi", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (req->hasParam("ssid")) {
      String ssid = req->getParam("ssid")->value();
      String pass = req->hasParam("pass") ? req->getParam("pass")->value() : "";
      saveWiFiCredentials(ssid, pass);
      req->send(200, "text/plain", "saved");
      rebootRequested = true;
      rebootRequestMs = millis();
    } else {
      req->send(400, "text/plain", "ssid required");
    }
  });

  webServer.on("/api/wifi_reset", HTTP_POST, [](AsyncWebServerRequest *req) {
    clearWiFiCredentials();
    req->send(200, "text/plain", "cleared");
    rebootRequested = true;
    rebootRequestMs = millis();
  });

  webServer.on("/api/log_clear", HTTP_POST, [](AsyncWebServerRequest *req) {
    logHead = 0;
    logSerial = 0;
    logSentSerial = 0;
    memset(logBuf, 0, sizeof(logBuf));
    req->send(200, "text/plain", "ok");
  });

  // Captive portal probe endpoints (redirect to root)
  if (captivePortalRunning) {
    setupCaptiveProbeEndpoints();
    webServer.onNotFound([](AsyncWebServerRequest *r) { r->redirect("/"); });
  }

  // OTA firmware upload
  webServer.on("/api/ota", HTTP_POST,
    [](AsyncWebServerRequest *req) {
      bool ok = !Update.hasError();
      if (ok) {
        req->send(200, "text/plain", "OK");
        rebootRequested = true;
        rebootRequestMs = millis();
      } else {
        req->send(500, "text/plain", String("Update failed: ") + Update.errorString());
      }
    },
    [](AsyncWebServerRequest *req, const String &filename, size_t index,
       uint8_t *data, size_t len, bool final) {
      if (index == 0) {
        Serial.printf("OTA: upload '%s'\n", filename.c_str());
        // Stop motor if running
        if (runState == RUNNING) requestGracefulStop();
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
          return;
        }
      }
      if (Update.isRunning()) {
        if (Update.write(data, len) != len) {
          Update.printError(Serial);
          return;
        }
      }
      if (final) {
        if (Update.end(true))
          Serial.printf("OTA: success, %u bytes\n", index + len);
        else
          Update.printError(Serial);
      }
    }
  );

  webServer.begin();
  Serial.println("Web server started on port 80");
}

// ==========================================================================
//  ArduinoOTA (for PlatformIO/Arduino IDE OTA)
// ==========================================================================
static void setupArduinoOTA() {
  ArduinoOTA.setHostname("autolee");
  ArduinoOTA.setPassword("autolee");
  ArduinoOTA.onStart([]() {
    if (runState == RUNNING) requestGracefulStop();
    Serial.println("OTA: start");
  });
  ArduinoOTA.onEnd([]() { Serial.println("OTA: done"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("OTA: %u%%", p / (t / 100));
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA err: %u", e); });
  ArduinoOTA.begin();
}

static void handleWebCalibration() {
  if (!webCalRequested) return;
  webCalRequested = false;
  if (runState != IDLE) return;
  calibrateEndpointsSensorless();
  ui_update_main_warning();
  recomputeEffectiveEndpoints();
  ui_update_tuning_numbers();
  ui_update_endpoint_edit_values();
}

static void handleWebHome() {
  if (!webHomeRequested) return;
  webHomeRequested = false;
  if (runState != STALLED) return;
  safeCreepHome();
}

static void broadcastState() {
  uint32_t now = millis();
  if ((now - lastSSEMs) < SSE_INTERVAL_MS) return;
  lastSSEMs = now;
  if (events.count() == 0) return;
  events.send(buildStateJSON().c_str(), NULL, millis());

  // Send only NEW log lines since last broadcast
  if (logSerial > logSentSerial) {
    uint32_t pending = logSerial - logSentSerial;
    // Can't send more than what's in the ring buffer
    if (pending > LOG_LINES) {
      logSentSerial = logSerial - LOG_LINES;
      pending = LOG_LINES;
    }
    // Cap per broadcast to avoid huge payloads
    if (pending > 20) {
      logSentSerial = logSerial - 20;
      pending = 20;
    }
    String logJson = "{\"log\":[";
    uint16_t startIdx = (logHead + LOG_LINES - (uint16_t)pending) % LOG_LINES;
    for (uint16_t i = 0; i < (uint16_t)pending; i++) {
      uint16_t idx = (startIdx + i) % LOG_LINES;
      if (i > 0) logJson += ',';
      logJson += '"';
      for (const char *p = logBuf[idx]; *p; p++) {
        if (*p == '"') logJson += "\\\"";
        else if (*p == '\\') logJson += "\\\\";
        else logJson += *p;
      }
      logJson += '"';
    }
    logJson += "]}";
    events.send(logJson.c_str(), "log", millis());
    logSentSerial = logSerial;
  }
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
  stepper->setAcceleration(RUN_DECEL);

  // ---- BUILD UI ----
  main_scr = lv_scr_act();
  style_screen(main_scr);
  lv_obj_t *mc = make_content_free(main_scr);
  lv_obj_t *mn = make_nav(main_scr);

  lv_obj_t *title = make_title(mc, "AutoLee");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
  lv_obj_t *sub = lv_label_create(mc);
  lv_label_set_text(sub, "by K.L Design");
  lv_obj_set_style_text_font(sub, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_color(sub, lv_color_hex(0xAAAAAA), LV_PART_MAIN);
  lv_obj_align_to(sub, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

  // Speed profile indicator on main screen — centered
  lbl_speed_val = lv_label_create(mc);
  lv_label_set_text(lbl_speed_val, "");
  lv_obj_set_style_text_font(lbl_speed_val, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_speed_val, lv_color_hex(0x6FA8FF), LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_speed_val, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_width(lbl_speed_val, SCR_W - 20);
  lv_obj_align_to(lbl_speed_val, sub, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

  // "NOT CALIBRATED" warning — above the counter, centered
  ui_create_main_warning(mc);
  lv_obj_align_to(main_warn, lbl_speed_val, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

  // Counter — centered in the space between speed/warning and bottom buttons
  counter_label = lv_label_create(mc);
  lv_label_set_text_fmt(counter_label, "%ld", min(counter, 9999L));
  lv_obj_set_style_text_font(counter_label, &lv_font_montserrat_48, LV_PART_MAIN);
  lv_obj_set_style_text_color(counter_label, lv_color_hex(0x00FF00), LV_PART_MAIN);
  lv_obj_align(counter_label, LV_ALIGN_CENTER, 0, -10);
  // Long-press counter to reset (make label clickable first)
  lv_obj_add_flag(counter_label, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(counter_label, [](lv_event_t *e) {
    LV_UNUSED(e);
    counter = 0;
    lv_label_set_text(counter_label, "0");
    // Brief red flash to confirm reset
    lv_obj_set_style_text_color(counter_label, lv_color_hex(0xFF4444), LV_PART_MAIN);
    lv_timer_handler();
    delay(200);
    lv_obj_set_style_text_color(counter_label, lv_color_hex(0x00FF00), LV_PART_MAIN);
  }, LV_EVENT_LONG_PRESSED, nullptr);

  // Batch remaining label (below counter, hidden when no batch)
  lbl_batch_remain = lv_label_create(mc);
  lv_label_set_text(lbl_batch_remain, "");
  lv_obj_set_style_text_font(lbl_batch_remain, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_batch_remain, lv_color_hex(0xFFD37C), LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_batch_remain, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(lbl_batch_remain, LV_ALIGN_CENTER, 0, 26);
  lv_obj_add_flag(lbl_batch_remain, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *btn_batch = make_btn(mc, "Batch Run", 140, 36, 0x1F6FEB, &lv_font_montserrat_16);
  lv_obj_align(btn_batch, LV_ALIGN_BOTTOM_MID, 0, -56);

  lv_obj_t *btn_settings = make_btn(mc, "Settings", 140, 36, 0x3A3A3A, &lv_font_montserrat_16);
  lv_obj_align(btn_settings, LV_ALIGN_BOTTOM_MID, 0, -8);

  btn_run_global = make_btn(mn, "RUN", 140, 44, 0x00FF00, &lv_font_montserrat_22);
  lv_obj_align(btn_run_global, LV_ALIGN_CENTER, 0, 0);

  // Settings screen
  settings_scr = lv_obj_create(NULL); style_screen(settings_scr);
  lv_obj_t *sc = make_content(settings_scr);
  lv_obj_set_style_pad_row(sc, 6, LV_PART_MAIN);
  lv_obj_add_flag(sc, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *sn = make_nav(settings_scr);
  lv_obj_t *st2 = make_title(sc, "Settings"); lv_obj_align(st2, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_t *b_cal    = make_btn(sc, "Calibrate",     140, 44, 0x444444, &lv_font_montserrat_20);
  lv_obj_t *b_config = make_btn(sc, "Config",        140, 44, 0x1F6FEB, &lv_font_montserrat_20);
  lv_obj_t *b_reset  = make_btn(sc, "Reset Count",   140, 44, 0xB42318, &lv_font_montserrat_20);
  lv_obj_t *b_back_s = make_btn(sn, "Back", 140, 44, 0x2A2A2A, &lv_font_montserrat_20);
  lv_obj_align(b_back_s, LV_ALIGN_CENTER, 0, 0);

  // Configuration screen (sub-menu of Settings)
  config_scr = lv_obj_create(NULL); style_screen(config_scr);
  lv_obj_t *cfgc = make_content(config_scr);
  lv_obj_set_style_pad_row(cfgc, 6, LV_PART_MAIN);
  lv_obj_add_flag(cfgc, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *cfgn = make_nav(config_scr);
  lv_obj_t *cfgt = make_title(cfgc, "Config"); lv_obj_align(cfgt, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_t *b_speed  = make_btn(cfgc, "Speed",      140, 44, 0x1F6FEB, &lv_font_montserrat_20);
  lv_obj_t *b_tuning = make_btn(cfgc, "Endpoints",  140, 44, 0x1F6FEB, &lv_font_montserrat_20);
  lv_obj_t *b_stall  = make_btn(cfgc, "Stall Guard",140, 44, 0x1F6FEB, &lv_font_montserrat_20);
  lv_obj_t *b_wifi   = make_btn(cfgc, "WiFi Info",  140, 44, 0x1F6FEB, &lv_font_montserrat_20);
  lv_obj_t *b_back_cfg = make_btn(cfgn, "Back", 140, 44, 0x2A2A2A, &lv_font_montserrat_20);
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
  lv_obj_set_style_text_color(lbl_profile_info, lv_color_hex(0x00FF00), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_profile_info, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_center(lbl_profile_info);

  // Three profile buttons
  for (uint8_t i = 0; i < NUM_PROFILES; i++) {
    char label[32];
    snprintf(label, sizeof(label), "%s  %lukHz", profiles[i].name, (unsigned long)(profiles[i].speed_hz / 1000));
    profile_btns[i] = make_btn(pc, label, 140, 40, 0x3A3A3A, &lv_font_montserrat_16);
    lv_obj_add_event_cb(profile_btns[i], [](lv_event_t *e) {
      uint8_t idx = (uint8_t)(intptr_t)lv_event_get_user_data(e);
      setActiveProfile(idx);
    }, LV_EVENT_CLICKED, (void*)(intptr_t)i);
  }

  lv_obj_t *b_back_p = make_btn(pn, "Back", 140, 44, 0x2A2A2A, &lv_font_montserrat_20);
  lv_obj_align(b_back_p, LV_ALIGN_CENTER, 0, 0);

  // Tuning screen
  tuning_scr = lv_obj_create(NULL); style_screen(tuning_scr);
  lv_obj_t *tc = make_content_free(tuning_scr);
  lv_obj_t *tn = make_nav(tuning_scr);
  lv_obj_t *tt = make_title(tc, "Tuning"); lv_obj_align(tt, LV_ALIGN_TOP_MID, 0, 8);

  lv_obj_t *raw_card = make_card(tc, 156, 80); lv_obj_set_pos(raw_card, 8, 44);
  lbl_ep_up = lv_label_create(raw_card);
  lv_obj_set_style_text_color(lbl_ep_up, lv_color_hex(0xCFCFCF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_ep_up, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(lbl_ep_up, LV_ALIGN_TOP_LEFT, 0, 0);
  lbl_ep_dn = lv_label_create(raw_card);
  lv_obj_set_style_text_color(lbl_ep_dn, lv_color_hex(0xCFCFCF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_ep_dn, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(lbl_ep_dn, LV_ALIGN_TOP_LEFT, 0, 18);
  lbl_travel = lv_label_create(raw_card);
  lv_obj_set_style_text_color(lbl_travel, lv_color_hex(0x00FF00), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_travel, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(lbl_travel, LV_ALIGN_TOP_LEFT, 0, 36);

  lv_obj_t *eff_card = make_card(tc, 156, 64); lv_obj_set_pos(eff_card, 8, 130);
  lbl_up_eff = lv_label_create(eff_card);
  lv_obj_set_style_text_color(lbl_up_eff, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_up_eff, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(lbl_up_eff, LV_ALIGN_TOP_LEFT, 0, 0);
  lbl_dn_eff = lv_label_create(eff_card);
  lv_obj_set_style_text_color(lbl_dn_eff, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_dn_eff, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(lbl_dn_eff, LV_ALIGN_TOP_LEFT, 0, 22);

  lv_obj_t *btn_eu = make_btn_multiline(tc, "EP UP",   76, 52, 0x1F6FEB, &lv_font_montserrat_14);
  lv_obj_t *btn_ed = make_btn_multiline(tc, "EP DOWN", 76, 52, 0x1F6FEB, &lv_font_montserrat_14);
  lv_obj_set_pos(btn_eu, 8, 204); lv_obj_set_pos(btn_ed, 88, 204);
  lv_obj_t *b_back_t = make_btn(tn, "Back", 140, 44, 0x2A2A2A, &lv_font_montserrat_20);
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
  lv_obj_set_style_text_color(lbl_wifi_status, lv_color_hex(0x00FF00), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_wifi_status, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_label_set_long_mode(lbl_wifi_status, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_wifi_status, 130);
  lv_obj_set_style_text_align(lbl_wifi_status, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(lbl_wifi_status);
  lv_obj_t *b_wifi_reset = make_btn(wc, "Reset WiFi", 140, 44, 0xB42318, &lv_font_montserrat_20);
  lv_obj_t *b_back_w = make_btn(wn, "Back", 140, 44, 0x2A2A2A, &lv_font_montserrat_20);
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
  lv_obj_set_style_text_color(jam_title, lv_color_hex(0xFF4444), LV_PART_MAIN);
  lv_obj_align(jam_title, LV_ALIGN_TOP_MID, 0, 20);

  lv_obj_t *jam_msg = lv_label_create(jc);
  lv_label_set_text(jam_msg, "Stall detected!\nMotor stopped and\nbacked off safely.");
  lv_obj_set_style_text_font(jam_msg, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_color(jam_msg, lv_color_hex(0xCCCCCC), LV_PART_MAIN);
  lv_obj_set_style_text_align(jam_msg, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_label_set_long_mode(jam_msg, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(jam_msg, SCR_W - 20);
  lv_obj_align(jam_msg, LV_ALIGN_TOP_MID, 0, 60);

  jam_status_lbl = lv_label_create(jc);
  lv_label_set_text(jam_status_lbl, "Press to return home");
  lv_obj_set_style_text_font(jam_status_lbl, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_color(jam_status_lbl, lv_color_hex(0xFFD37C), LV_PART_MAIN);
  lv_obj_set_style_text_align(jam_status_lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(jam_status_lbl, LV_ALIGN_CENTER, 0, 20);

  lv_obj_t *btn_home = make_btn(jn, "Return Home", 140, 44, 0x1F6FEB, &lv_font_montserrat_18);
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
  lv_obj_set_style_text_color(sg_name, lv_color_hex(0xCFCFCF), LV_PART_MAIN);
  lv_obj_set_style_text_font(sg_name, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(sg_name, LV_ALIGN_TOP_LEFT, 0, 0);

  lbl_sg_val = lv_label_create(sg_card);
  lv_obj_set_style_text_color(lbl_sg_val, lv_color_hex(0x00FF00), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_sg_val, &lv_font_montserrat_26, LV_PART_MAIN);
  lv_obj_center(lbl_sg_val);

  lv_obj_t *sg_hint = lv_label_create(sg_card);
  lv_label_set_text(sg_hint, "0=off  lower=sensitive");
  lv_obj_set_style_text_color(sg_hint, lv_color_hex(0x888888), LV_PART_MAIN);
  lv_obj_set_style_text_font(sg_hint, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(sg_hint, LV_ALIGN_BOTTOM_MID, 0, 0);

  lv_obj_t *sg_grid = lv_obj_create(ssc);
  lv_obj_set_size(sg_grid, 150, 96);
  lv_obj_set_style_bg_opa(sg_grid, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(sg_grid, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(sg_grid, 0, LV_PART_MAIN);
  lv_obj_clear_flag(sg_grid, LV_OBJ_FLAG_SCROLLABLE);

  const int sgbw = 70, sgbh = 44, sggap = 4;
  lv_obj_t *sgM5 = make_btn(sg_grid, "-5", sgbw, sgbh, 0x3A3A3A, &lv_font_montserrat_18);
  lv_obj_t *sgM1 = make_btn(sg_grid, "-1", sgbw, sgbh, 0x3A3A3A, &lv_font_montserrat_18);
  lv_obj_t *sgP1 = make_btn(sg_grid, "+1", sgbw, sgbh, 0x1F6FEB, &lv_font_montserrat_18);
  lv_obj_t *sgP5 = make_btn(sg_grid, "+5", sgbw, sgbh, 0x1F6FEB, &lv_font_montserrat_18);
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

  lv_obj_t *b_back_ss = make_btn(ssn, "Back", 140, 44, 0x2A2A2A, &lv_font_montserrat_20);
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
  lv_obj_set_style_text_color(br_name, lv_color_hex(0xCFCFCF), LV_PART_MAIN);
  lv_obj_set_style_text_font(br_name, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(br_name, LV_ALIGN_TOP_LEFT, 0, 0);

  lbl_batch_val = lv_label_create(br_card);
  lv_obj_set_style_text_color(lbl_batch_val, lv_color_hex(0x00FF00), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_batch_val, &lv_font_montserrat_26, LV_PART_MAIN);
  lv_obj_align(lbl_batch_val, LV_ALIGN_TOP_RIGHT, 0, 0);

  lv_obj_t *br_hint = lv_label_create(br_card);
  lv_label_set_text(br_hint, "0 = off (unlimited)");
  lv_obj_set_style_text_color(br_hint, lv_color_hex(0x888888), LV_PART_MAIN);
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
  lv_obj_t *brM100 = make_btn(br_grid, "-100", bbw, bbh, 0x3A3A3A, &lv_font_montserrat_14);
  lv_obj_t *brM10  = make_btn(br_grid, "-10",  bbw, bbh, 0x3A3A3A, &lv_font_montserrat_16);
  lv_obj_t *brM1   = make_btn(br_grid, "-1",   bbw, bbh, 0x3A3A3A, &lv_font_montserrat_16);
  lv_obj_t *brP1   = make_btn(br_grid, "+1",   bbw, bbh, 0x1F6FEB, &lv_font_montserrat_16);
  lv_obj_t *brP10  = make_btn(br_grid, "+10",  bbw, bbh, 0x1F6FEB, &lv_font_montserrat_16);
  lv_obj_t *brP100 = make_btn(br_grid, "+100", bbw, bbh, 0x1F6FEB, &lv_font_montserrat_14);
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
  lv_obj_t *btn_start_batch = make_btn(brc, "Start Batch", 140, 38, 0x00FF00, &lv_font_montserrat_18);
  lv_obj_add_event_cb(btn_start_batch, [](lv_event_t *e) {
    LV_UNUSED(e);
    if (batchTarget <= 0 || runState != IDLE || !endpointsCalibrated) return;
    batchCount = 0;
    batchActive = true;
    startRunBetweenEndpoints();
    setRunButtonState(true);
    go(main_scr);
  }, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *b_back_br = make_btn(brn, "Back", 140, 44, 0x2A2A2A, &lv_font_montserrat_20);
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
