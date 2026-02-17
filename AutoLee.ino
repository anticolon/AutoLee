// ============================================================================
//  AutoLee v1.2
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

// ==========================================================================
//  CONFIGURATION
// ==========================================================================

static const char *DEFAULT_AP_SSID = "AutoLee-Setup";
static const char *DEFAULT_AP_PASS = "autolee123";

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

static constexpr uint32_t UI_SPEED_MIN = 10000;
static constexpr uint32_t UI_SPEED_MAX = 55000;
static uint32_t           ui_speed_hz  = 30000;
static constexpr uint32_t RUN_ACCEL    = 80000;

static int32_t upOffsetSteps   = 0;
static int32_t downOffsetSteps = 0;
static constexpr int32_t DOWN_OFFSET_DEFAULT = -500;
static constexpr int32_t OFFSET_MIN     = -8000;
static constexpr int32_t OFFSET_MAX     = +8000;
static constexpr int32_t ENDPOINT_GUARD = 50;

static constexpr int32_t CAL_PREMOVE_DOWN_STEPS = 5500;

// Calibration
static constexpr int8_t   CAL_SGT          = -1;
static constexpr uint16_t RUN_CURRENT_MA   = 4000;
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
static uint16_t           RUN_SG_TRIP        = 80;   // SG above this = stall (normal ~26-29, jam ~130+)
static constexpr uint16_t RUN_SG_TRIP_MIN    = 0;
static constexpr uint16_t RUN_SG_TRIP_MAX    = 200;
static constexpr uint32_t RUN_SG_SUSTAIN_MS  = 300;  // SG must stay above trip for this long
static constexpr uint32_t RUN_SG_IGNORE_MS   = 400;  // ignore SG after each direction change
static constexpr int32_t  RUN_BACKOFF_STEPS  = 1000; // steps to back off after jam
static constexpr uint32_t CREEP_HOME_SPEED   = CAL_SPEED_HZ;
static constexpr uint32_t CREEP_HOME_ACCEL   = CAL_ACCEL;
static constexpr uint16_t CREEP_SG_TRIP      = 15;
static constexpr uint8_t  CREEP_SG_CONFIRM   = 3;
static constexpr uint32_t CREEP_IGNORE_MS    = 300;
static constexpr uint32_t CREEP_TIMEOUT_MS   = 20000;

static uint32_t runSGStallStartMs = 0;     // 0 = not currently above trip
static bool     runSGAboveTrip = false;
static uint32_t lastDirectionChangeMs = 0;

// Jam screen
static lv_obj_t *jam_scr = nullptr;
static lv_obj_t *jam_status_lbl = nullptr;
static lv_obj_t *stall_scr = nullptr;
static lv_obj_t *lbl_sg_val = nullptr;

static bool wifiConnected = false;
static bool wifiAPMode = false;
static String wifiSSID = "", wifiPass = "";

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

static lv_obj_t *main_scr = nullptr, *settings_scr = nullptr, *speed_scr = nullptr;
static lv_obj_t *tuning_scr = nullptr, *ep_up_scr = nullptr, *ep_dn_scr = nullptr;
static lv_obj_t *wifi_scr = nullptr;
static lv_obj_t *counter_label = nullptr, *main_warn = nullptr, *main_warn_lbl = nullptr;
static lv_obj_t *slider_speed = nullptr, *lbl_speed_val = nullptr;
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
static constexpr uint8_t  LOG_LINES = 40;
static constexpr uint16_t LOG_LINE_LEN = 120;
static char logBuf[LOG_LINES][LOG_LINE_LEN];
static uint8_t logHead = 0;   // next write index
static uint8_t logCount = 0;
static bool logDirty = false;

static void webLog(const char *fmt, ...) {
  char line[LOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  // Write to Serial
  Serial.println(line);
  // Write to ring buffer
  strncpy(logBuf[logHead], line, LOG_LINE_LEN - 1);
  logBuf[logHead][LOG_LINE_LEN - 1] = '\0';
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) logCount++;
  logDirty = true;
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
  // Disable step pulse interrupts during SPI to prevent corruption
  noInterrupts();
  uint32_t drv = driver.DRV_STATUS();
  interrupts();
  return (uint16_t)(drv & 0x03FF);
}

// Median-of-3 filter to reject SPI glitch spikes
static uint16_t read_sg() {
  uint16_t a = read_sg_raw();
  uint16_t b = read_sg_raw();
  uint16_t c = read_sg_raw();
  // Return median
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { b = c; }
  if (a > b) { b = a; }
  return b;
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
static void startRunBetweenEndpoints() {
  if (!endpointsCalibrated || !stepper) return;
  runState = RUNNING;
  runSGAboveTrip = false;
  runSGStallStartMs = 0;
  lastDirectionChangeMs = millis();

  // Ensure TMC5160 is in spreadCycle with SG2 active for runtime stall detection
  driver.rms_current(RUN_CURRENT_MA);
  driver.en_pwm_mode(false);    // spreadCycle (required for SG2)
  driver.TPWMTHRS(0);
  driver.TCOOLTHRS(0xFFFFF);    // SG2 active at all speeds
  driver.semin(0); driver.semax(0);  // CoolStep off

  long pos = stepper->getCurrentPosition();
  if (nearPos(pos, endpointUp))        currentTarget = endpointDown;
  else if (nearPos(pos, endpointDown)) currentTarget = endpointUp;
  else {
    currentTarget = (labs(pos - endpointUp) < labs(pos - endpointDown)) ? endpointUp : endpointDown;
  }
  stepper->setSpeedInHz(ui_speed_hz);
  stepper->setAcceleration(RUN_ACCEL);
  stepper->moveTo(currentTarget);
}

static void requestGracefulStop() {
  if (!stepper) return;
  runState = STOPPING;
  stopEntryMs = millis();
  currentTarget = endpointUp;
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
      if (!stepper->isRunning()) {
        // Completed a move — count if arrived at DOWN, then flip
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
        runSGAboveTrip = false;
        runSGStallStartMs = 0;
        stepper->moveTo(currentTarget);
        break;
      }

      // --- Runtime stall detection ---
      // Disabled when RUN_SG_TRIP == 0
      if (RUN_SG_TRIP == 0) break;

      // Skip SG reading during acceleration after direction change
      uint32_t sinceChange = millis() - lastDirectionChangeMs;
      if (sinceChange < RUN_SG_IGNORE_MS) break;

      uint16_t sg = read_sg();

      // Debug: print SG every 500ms so user can see normal values
      static uint32_t lastSGPrintMs = 0;
      if ((millis() - lastSGPrintMs) > 500) {
        webLog("RUN SG=%u trip=%u pos=%ld", sg, RUN_SG_TRIP, stepper->getCurrentPosition());
        lastSGPrintMs = millis();
      }

      // Jam = SG stays ABOVE threshold for sustained period
      // Normal running ~26-29, jam ~130-150
      // Brief spikes from resonance/tight spots are ignored
      if (sg > RUN_SG_TRIP) {
        if (!runSGAboveTrip) {
          runSGAboveTrip = true;
          runSGStallStartMs = millis();
        } else if ((millis() - runSGStallStartMs) >= RUN_SG_SUSTAIN_MS) {
          // JAM DETECTED — sustained high SG
          webLog("JAM! SG=%u trip=%u pos=%ld tgt=%ld sustained=%lums",
                        sg, RUN_SG_TRIP, stepper->getCurrentPosition(), currentTarget,
                        millis() - runSGStallStartMs);

          stepper->forceStop();
          fas_wait_for_stop();

          // Back off to release pressure
          stepper->setSpeedInHz(CREEP_HOME_SPEED);
          stepper->setAcceleration(CREEP_HOME_ACCEL);

          // Back off in the opposite direction of travel
          int32_t backoff = (currentTarget == endpointDown) ? -RUN_BACKOFF_STEPS : +RUN_BACKOFF_STEPS;
          stepper->move(backoff);
          fas_wait_for_stop();

          runState = STALLED;
          runSGAboveTrip = false;
          showJamScreen();
        }
      } else {
        runSGAboveTrip = false;
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
  stepper->setAcceleration(RUN_ACCEL);

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
    stepper->setAcceleration(RUN_ACCEL);
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
    stepper->setAcceleration(RUN_ACCEL);
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
  stepper->setAcceleration(RUN_ACCEL);
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
  if (lbl_speed_val) lv_label_set_text_fmt(lbl_speed_val, "%lu", (unsigned long)ui_speed_hz);
}

static void ui_update_tuning_numbers() {
  if (!endpointsCalibrated) {
    if (lbl_ep_up)  lv_label_set_text(lbl_ep_up, "RAW UP: -");
    if (lbl_ep_dn)  lv_label_set_text(lbl_ep_dn, "RAW DOWN: -");
    if (lbl_travel) lv_label_set_text(lbl_travel, "RAW TRAVEL: -");
    if (lbl_up_eff) lv_label_set_text(lbl_up_eff, "UP: -");
    if (lbl_dn_eff) lv_label_set_text(lbl_dn_eff, "DOWN: -");
    return;
  }
  if (lbl_ep_up)  lv_label_set_text_fmt(lbl_ep_up, "RAW UP: %ld", rawUp);
  if (lbl_ep_dn)  lv_label_set_text_fmt(lbl_ep_dn, "RAW DOWN: %ld", rawDown);
  if (lbl_travel) lv_label_set_text_fmt(lbl_travel, "RAW TRAVEL: %ld", rawDown - rawUp);
  if (lbl_up_eff) lv_label_set_text_fmt(lbl_up_eff, "UP: %ld (off %+ld)", endpointUp, (long)upOffsetSteps);
  if (lbl_dn_eff) lv_label_set_text_fmt(lbl_dn_eff, "DOWN: %ld (off %+ld)", endpointDown, (long)downOffsetSteps);
}

static void ui_update_endpoint_edit_values() {
  if (lbl_ep_up_val) lv_label_set_text_fmt(lbl_ep_up_val, "%ld", endpointUp);
  if (lbl_ep_dn_val) lv_label_set_text_fmt(lbl_ep_dn_val, "%ld", endpointDown);
}

static void ui_update_sg_val() {
  if (lbl_sg_val) lv_label_set_text_fmt(lbl_sg_val, "%u", RUN_SG_TRIP);
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
    lv_label_set_text_fmt(lbl_wifi_status, "IP: %s", WiFi.localIP().toString().c_str());
  else if (wifiAPMode)
    lv_label_set_text_fmt(lbl_wifi_status, "AP: %s\n192.168.4.1", DEFAULT_AP_SSID);
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
static void on_go_speed(lv_event_t *e) {
  LV_UNUSED(e);
  if (slider_speed) lv_slider_set_value(slider_speed, ui_speed_hz, LV_ANIM_OFF);
  ui_update_speed_val(); go(speed_scr);
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
static void on_reset_counter(lv_event_t *e) {
  LV_UNUSED(e); counter = 0;
  if (counter_label) lv_label_set_text(counter_label, "0");
}
static void on_speed_slider(lv_event_t *e) {
  ui_speed_hz = lv_slider_get_value(lv_event_get_target(e));
  if (stepper && runState == RUNNING) stepper->setSpeedInHz(ui_speed_hz);
  ui_update_speed_val();
}
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

static void startWiFi() {
  loadWiFiCredentials();
  if (wifiSSID.length() > 0) {
    Serial.printf("WiFi: connecting to '%s'...\n", wifiSSID.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID.c_str(), wifiPass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 10000) {
      delay(100); lv_timer_handler();
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true; wifiAPMode = false;
      Serial.printf("WiFi: connected! IP=%s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("WiFi: STA failed, falling back to AP");
      WiFi.disconnect();
    }
  }
  if (!wifiConnected) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(DEFAULT_AP_SSID, DEFAULT_AP_PASS);
    wifiAPMode = true;
    Serial.printf("WiFi AP: %s @ %s\n", DEFAULT_AP_SSID, WiFi.softAPIP().toString().c_str());
  }
  ui_update_wifi_label();
}

// ==========================================================================
//  WEB SERVER
// ==========================================================================
static String buildStateJSON() {
  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"state\":\"%s\",\"counter\":%ld,\"speed\":%lu,\"calibrated\":%s,"
    "\"rawUp\":%ld,\"rawDown\":%ld,\"endpointUp\":%ld,\"endpointDown\":%ld,"
    "\"upOffset\":%ld,\"downOffset\":%ld,\"position\":%ld,\"sgTrip\":%u,"
    "\"batchTarget\":%ld,\"batchCount\":%ld,\"batchActive\":%s}",
    runState==RUNNING?"RUNNING":runState==STOPPING?"STOPPING":runState==CALIBRATING?"CALIBRATING":runState==STALLED?"STALLED":runState==HOMING?"HOMING":"IDLE",
    counter, (unsigned long)ui_speed_hz, endpointsCalibrated?"true":"false",
    rawUp, rawDown, endpointUp, endpointDown,
    (long)upOffsetSteps, (long)downOffsetSteps,
    stepper ? stepper->getCurrentPosition() : 0L,
    RUN_SG_TRIP,
    batchTarget, batchCount,
    batchActive ? "true" : "false");
  return String(buf);
}

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AutoLee Control</title>
<style>
:root{--bg:#0a0a0a;--card:#1a1a1a;--accent:#1F6FEB;--green:#00FF00;--red:#FF4444;--text:#e0e0e0;--muted:#888}
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:var(--bg);color:var(--text);min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:16px}
h1{font-size:1.6em;margin-bottom:4px}.sub{color:var(--muted);font-size:.85em;margin-bottom:16px}
.card{background:var(--card);border-radius:12px;padding:16px;margin-bottom:12px;width:100%;max-width:400px}
.card h2{font-size:1.1em;margin-bottom:10px;color:var(--muted)}
.badge{display:inline-block;padding:4px 12px;border-radius:12px;font-size:.8em;font-weight:600}
.badge.ok{background:#0d3320;color:var(--green)}.badge.warn{background:#3A2B12;color:#FFD37C}.badge.run{background:#331111;color:var(--red)}.badge.stall{background:#441111;color:#FF8844}
.counter{font-size:3.5em;font-weight:700;color:var(--green);text-align:center;margin:8px 0;font-variant-numeric:tabular-nums}
.btn{display:inline-block;padding:12px 24px;border:none;border-radius:8px;font-size:1em;font-weight:600;cursor:pointer;color:#fff;text-align:center;min-width:100px;transition:opacity .2s}
.btn:hover{opacity:.85}.btn:active{opacity:.7}.btn:disabled{opacity:.4;cursor:not-allowed}
.btn-run{background:var(--green);color:#000;font-size:1.2em;width:100%}.btn-run.active{background:var(--red);color:#fff}
.btn-blue{background:var(--accent)}.btn-dark{background:#3a3a3a}.btn-danger{background:#B42318}
.btn-sm{padding:8px 12px;font-size:.85em;min-width:50px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}.row.ctr{justify-content:center}
.sr{display:flex;justify-content:space-between;padding:4px 0;font-size:.9em}.sr .l{color:var(--muted)}.sr .v{font-weight:600;font-variant-numeric:tabular-nums}
input[type=range]{width:100%;accent-color:var(--accent)}
.sl{display:flex;justify-content:space-between;font-size:.85em;color:var(--muted);margin-bottom:4px}
.st{font-size:.9em;color:var(--accent);font-weight:600;margin:12px 0 6px;text-transform:uppercase;letter-spacing:.05em}
.ea{display:flex;gap:4px;align-items:center;justify-content:center;margin-top:6px}
hr{border:none;border-top:1px solid #333;margin:8px 0}
.upload{border:2px dashed #444;border-radius:8px;padding:20px;text-align:center;color:var(--muted);margin-top:8px;cursor:pointer}
.upload:hover{border-color:var(--accent)}.upload.on{border-color:var(--green);color:var(--green)}
.pbar{width:100%;height:6px;background:#333;border-radius:3px;margin-top:8px;overflow:hidden;display:none}
.pbar .fill{height:100%;background:var(--accent);width:0%;transition:width .3s;border-radius:3px}
#otaS{font-size:.85em;margin-top:6px;min-height:1.2em}
input[type=text],input[type=password]{width:100%;padding:8px;margin-bottom:6px;background:#222;border:1px solid #444;border-radius:6px;color:#fff;font-size:.9em}
</style></head><body>
<h1>AutoLee</h1><div class="sub">by K.L Design</div>

<div class="card">
<div class="row ctr" style="gap:8px;margin-bottom:8px">
<span id="cb" class="badge warn">NOT CALIBRATED</span>
<span id="sb" class="badge ok">IDLE</span></div>
<div class="counter" id="ctr">0</div>
<button class="btn btn-run" id="br" onclick="toggleRun()">RUN</button>
<div id="jamAlert" style="display:none;background:#441111;border-radius:8px;padding:12px;margin-top:8px;text-align:center">
<div style="color:#FF4444;font-weight:700;font-size:1.1em;margin-bottom:6px">&#9888; JAM DETECTED</div>
<div style="color:#ccc;font-size:.85em;margin-bottom:8px">Motor stalled and backed off safely.</div>
<button class="btn btn-blue" onclick="doAct('return_home')" id="bh">Return Home</button>
</div></div>

<div class="card"><h2>Speed</h2>
<div class="sl"><span>Hz</span><span id="sv">30000</span></div>
<input type="range" id="ss" min="10000" max="55000" value="30000" oninput="setSpd(this.value)"></div>

<div class="card"><h2>Tuning</h2>
<div class="sr"><span class="l">RAW UP</span><span class="v" id="ru">-</span></div>
<div class="sr"><span class="l">RAW DOWN</span><span class="v" id="rd">-</span></div>
<div class="sr"><span class="l">RAW TRAVEL</span><span class="v" id="rt">-</span></div><hr>
<div class="sr"><span class="l">Effective UP</span><span class="v" id="eu">-</span></div>
<div class="sr"><span class="l">Effective DOWN</span><span class="v" id="ed">-</span></div>
<div class="sr"><span class="l">Position</span><span class="v" id="cp">-</span></div>
<div class="st">Endpoint UP offset</div>
<div class="ea">
<button class="btn btn-dark btn-sm" onclick="adj('up',-100)">-100</button>
<button class="btn btn-dark btn-sm" onclick="adj('up',-10)">-10</button>
<button class="btn btn-dark btn-sm" onclick="adj('up',-1)">-1</button>
<button class="btn btn-blue btn-sm" onclick="adj('up',1)">+1</button>
<button class="btn btn-blue btn-sm" onclick="adj('up',10)">+10</button>
<button class="btn btn-blue btn-sm" onclick="adj('up',100)">+100</button></div>
<div class="st">Endpoint DOWN offset</div>
<div class="ea">
<button class="btn btn-dark btn-sm" onclick="adj('down',-100)">-100</button>
<button class="btn btn-dark btn-sm" onclick="adj('down',-10)">-10</button>
<button class="btn btn-dark btn-sm" onclick="adj('down',-1)">-1</button>
<button class="btn btn-blue btn-sm" onclick="adj('down',1)">+1</button>
<button class="btn btn-blue btn-sm" onclick="adj('down',10)">+10</button>
<button class="btn btn-blue btn-sm" onclick="adj('down',100)">+100</button></div></div>

<div class="card"><h2>Stall Sensitivity</h2>
<div class="sr"><span class="l">SG Trip threshold</span><span class="v" id="sgv">8</span></div>
<div style="font-size:.8em;color:var(--muted);margin:4px 0">0 = off, lower = more sensitive (normal ~27, jam ~130+)</div>
<div class="ea">
<button class="btn btn-dark btn-sm" onclick="setSg(-5)">-5</button>
<button class="btn btn-dark btn-sm" onclick="setSg(-1)">-1</button>
<button class="btn btn-blue btn-sm" onclick="setSg(1)">+1</button>
<button class="btn btn-blue btn-sm" onclick="setSg(5)">+5</button></div></div>

<div class="card"><h2>Batch Run</h2>
<div class="sr"><span class="l">Target count</span><span class="v" id="btv">OFF</span></div>
<div id="btStatus" style="font-size:.85em;color:var(--muted);margin:4px 0"></div>
<div class="ea">
<button class="btn btn-dark btn-sm" onclick="setBatch(-100)">-100</button>
<button class="btn btn-dark btn-sm" onclick="setBatch(-10)">-10</button>
<button class="btn btn-dark btn-sm" onclick="setBatch(-1)">-1</button>
<button class="btn btn-blue btn-sm" onclick="setBatch(1)">+1</button>
<button class="btn btn-blue btn-sm" onclick="setBatch(10)">+10</button>
<button class="btn btn-blue btn-sm" onclick="setBatch(100)">+100</button></div>
<div class="row ctr" style="margin-top:8px">
<button class="btn btn-blue btn-sm" onclick="doBatch('start')" id="bbStart">Start Batch</button>
<button class="btn btn-dark btn-sm" onclick="doBatch('clear')">Clear</button></div></div>

<div class="card"><h2>Actions</h2>
<div class="row ctr">
<button class="btn btn-dark" onclick="doAct('calibrate')" id="bc">Calibrate</button>
<button class="btn btn-danger" onclick="doAct('reset_counter')">Reset Counter</button></div></div>

<div class="card"><h2>WiFi</h2>
<div style="font-size:.85em;color:var(--muted);margin-bottom:6px">Change credentials (reboot required):</div>
<input type="text" id="ns" placeholder="SSID">
<input type="password" id="np" placeholder="Password">
<button class="btn btn-blue" onclick="saveWifi()" style="width:100%">Save &amp; Reboot</button></div>

<div class="card"><h2>Log</h2>
<div id="logBox" style="background:#000;border-radius:6px;padding:8px;font-family:'Courier New',monospace;font-size:.75em;color:#0f0;height:200px;overflow-y:auto;white-space:pre-wrap;word-break:break-all"></div>
<button class="btn btn-dark btn-sm" onclick="document.getElementById('logBox').textContent=''" style="margin-top:6px">Clear</button></div>

<div class="card"><h2>Firmware Update (OTA)</h2>
<div class="upload" id="ua" onclick="document.getElementById('fw').click()">
Tap to select .bin file<br><span style="font-size:.8em">or drag &amp; drop</span></div>
<input type="file" id="fw" accept=".bin" style="display:none" onchange="upFW(this.files[0])">
<div class="pbar" id="pb"><div class="fill" id="pf"></div></div>
<div id="otaS"></div></div>

<script>
let es;
function sse(){es=new EventSource('/events');es.onmessage=e=>{try{upd(JSON.parse(e.data))}catch(x){}};
es.addEventListener('log',e=>{try{const d=JSON.parse(e.data);const lb=document.getElementById('logBox');lb.textContent=d.log.join('\n');lb.scrollTop=lb.scrollHeight}catch(x){}});
es.onerror=()=>{es.close();setTimeout(sse,3000)}}
sse();

function upd(d){
  document.getElementById('ctr').textContent=d.counter;
  document.getElementById('sv').textContent=d.speed;
  document.getElementById('ss').value=d.speed;
  document.getElementById('cp').textContent=d.position;
  document.getElementById('sgv').textContent=d.sgTrip;
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
function setSg(d){fetch('/api/sg_trip?delta='+d,{method:'POST'})}
function setBatch(d){fetch('/api/batch?delta='+d,{method:'POST'})}
function doBatch(a){fetch('/api/batch?action='+a,{method:'POST'})}
function setSpd(v){document.getElementById('sv').textContent=v;fetch('/api/speed?hz='+v,{method:'POST'})}
function adj(w,d){fetch('/api/endpoint?which='+w+'&delta='+d,{method:'POST'})}
function doAct(a){fetch('/api/action?do='+a,{method:'POST'})}
function saveWifi(){
  const s=document.getElementById('ns').value,p=document.getElementById('np').value;
  if(!s){alert('SSID required');return}
  fetch('/api/wifi?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p),{method:'POST'})
  .then(()=>{alert('Saved! Rebooting...');setTimeout(()=>location.reload(),5000)})}

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


static void setupWebServer() {
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", INDEX_HTML);
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

  webServer.on("/api/speed", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (req->hasParam("hz")) {
      uint32_t hz = req->getParam("hz")->value().toInt();
      ui_speed_hz = constrain(hz, UI_SPEED_MIN, UI_SPEED_MAX);
      if (stepper && runState == RUNNING) stepper->setSpeedInHz(ui_speed_hz);
      if (slider_speed) lv_slider_set_value(slider_speed, ui_speed_hz, LV_ANIM_OFF);
      ui_update_speed_val();
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
    if (req->hasParam("delta")) {
      int32_t d = req->getParam("delta")->value().toInt();
      int32_t v = (int32_t)RUN_SG_TRIP + d;
      RUN_SG_TRIP = (uint16_t)constrain(v, (int32_t)RUN_SG_TRIP_MIN, (int32_t)RUN_SG_TRIP_MAX);
      ui_update_sg_val();
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

  // OTA firmware upload
  webServer.on("/api/ota", HTTP_POST,
    [](AsyncWebServerRequest *req) {
      if (Update.hasError())
        req->send(500, "text/plain", String("Update failed: ") + Update.errorString());
      else {
        req->send(200, "text/plain", "OK");
        delay(500);
        ESP.restart();
      }
    },
    [](AsyncWebServerRequest *req, const String &filename, size_t index,
       uint8_t *data, size_t len, bool final) {
      if (index == 0) {
        Serial.printf("OTA: upload '%s'", filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
      }
      if (Update.isRunning()) {
        if (Update.write(data, len) != len) Update.printError(Serial);
      }
      if (final) {
        if (Update.end(true)) Serial.printf("OTA: success, %u bytes", index + len);
        else Update.printError(Serial);
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

  // Send new log lines if any
  if (logDirty) {
    logDirty = false;
    String logJson = "{\"log\":[";
    uint8_t start = (logCount < LOG_LINES) ? 0 : logHead;
    for (uint8_t i = 0; i < logCount; i++) {
      uint8_t idx = (start + i) % LOG_LINES;
      if (i > 0) logJson += ',';
      logJson += '"';
      // Escape quotes in log lines
      for (const char *p = logBuf[idx]; *p; p++) {
        if (*p == '"') logJson += "\\\"";
        else if (*p == '\\') logJson += "\\\\";
        else logJson += *p;
      }
      logJson += '"';
    }
    logJson += "]}";
    events.send(logJson.c_str(), "log", millis());
  }
}

// ==========================================================================
//  SETUP
// ==========================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== AutoLee v2.0 ===");

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

  ui_create_main_warning(mc);
  lv_obj_align_to(main_warn, sub, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

  counter_label = lv_label_create(mc);
  lv_label_set_text_fmt(counter_label, "%ld", min(counter, 9999L));
  lv_obj_set_style_text_font(counter_label, &lv_font_montserrat_48, LV_PART_MAIN);
  lv_obj_set_style_text_color(counter_label, lv_color_hex(0x00FF00), LV_PART_MAIN);
  lv_obj_align(counter_label, LV_ALIGN_CENTER, 0, -30);

  // Batch remaining label (below counter, hidden when no batch)
  lbl_batch_remain = lv_label_create(mc);
  lv_label_set_text(lbl_batch_remain, "");
  lv_obj_set_style_text_font(lbl_batch_remain, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_batch_remain, lv_color_hex(0xFFD37C), LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_batch_remain, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(lbl_batch_remain, LV_ALIGN_CENTER, 0, 10);
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
  lv_obj_set_style_pad_row(sc, 6, LV_PART_MAIN);    // tighter spacing
  lv_obj_add_flag(sc, LV_OBJ_FLAG_SCROLLABLE);       // enable scroll
  lv_obj_t *sn = make_nav(settings_scr);
  lv_obj_t *st2 = make_title(sc, "Settings"); lv_obj_align(st2, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_t *b_speed  = make_btn(sc, "Speed",    140, 44, 0x1F6FEB, &lv_font_montserrat_20);
  lv_obj_t *b_tuning = make_btn(sc, "Tuning",   140, 44, 0x1F6FEB, &lv_font_montserrat_20);
  lv_obj_t *b_cal    = make_btn(sc, "Calibrate",140, 44, 0x444444, &lv_font_montserrat_20);
  lv_obj_t *b_stall  = make_btn(sc, "Stall Sens.",140, 44, 0x1F6FEB, &lv_font_montserrat_20);
  lv_obj_t *b_reset  = make_btn(sc, "Reset counter", 140, 44, 0xB42318, &lv_font_montserrat_18);
  lv_obj_t *b_wifi   = make_btn(sc, "WiFi Info",140, 44, 0x1F6FEB, &lv_font_montserrat_20);
  lv_obj_t *b_back_s = make_btn(sn, "Back", 140, 44, 0x2A2A2A, &lv_font_montserrat_20);
  lv_obj_align(b_back_s, LV_ALIGN_CENTER, 0, 0);

  // Speed screen
  speed_scr = lv_obj_create(NULL); style_screen(speed_scr);
  lv_obj_t *pc = make_content(speed_scr);
  lv_obj_t *pn = make_nav(speed_scr);
  lv_obj_t *pt = make_title(pc, "Speed"); lv_obj_align(pt, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_t *card = make_card(pc, 150, 110);
  lv_obj_t *lblHz = lv_label_create(card);
  lv_label_set_text(lblHz, "Hz");
  lv_obj_set_style_text_color(lblHz, lv_color_hex(0xCFCFCF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lblHz, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(lblHz, LV_ALIGN_TOP_LEFT, 0, 0);
  lbl_speed_val = lv_label_create(card);
  lv_obj_set_style_text_color(lbl_speed_val, lv_color_hex(0x00FF00), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_speed_val, &lv_font_montserrat_22, LV_PART_MAIN);
  lv_obj_align(lbl_speed_val, LV_ALIGN_TOP_RIGHT, 0, 0);
  slider_speed = lv_slider_create(card);
  lv_slider_set_range(slider_speed, UI_SPEED_MIN, UI_SPEED_MAX);
  lv_slider_set_value(slider_speed, ui_speed_hz, LV_ANIM_OFF);
  lv_obj_set_size(slider_speed, 130, 16);
  lv_obj_align(slider_speed, LV_ALIGN_BOTTOM_MID, 0, -6);
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
  lv_obj_set_style_text_font(lbl_ep_up, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(lbl_ep_up, LV_ALIGN_TOP_LEFT, 0, 0);
  lbl_ep_dn = lv_label_create(raw_card);
  lv_obj_set_style_text_color(lbl_ep_dn, lv_color_hex(0xCFCFCF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_ep_dn, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(lbl_ep_dn, LV_ALIGN_TOP_LEFT, 0, 18);
  lbl_travel = lv_label_create(raw_card);
  lv_obj_set_style_text_color(lbl_travel, lv_color_hex(0x00FF00), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_travel, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(lbl_travel, LV_ALIGN_TOP_LEFT, 0, 36);

  lv_obj_t *eff_card = make_card(tc, 156, 64); lv_obj_set_pos(eff_card, 8, 130);
  lbl_up_eff = lv_label_create(eff_card);
  lv_obj_set_style_text_color(lbl_up_eff, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_up_eff, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(lbl_up_eff, LV_ALIGN_TOP_LEFT, 0, 0);
  lbl_dn_eff = lv_label_create(eff_card);
  lv_obj_set_style_text_color(lbl_dn_eff, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_dn_eff, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(lbl_dn_eff, LV_ALIGN_TOP_LEFT, 0, 22);

  lv_obj_t *btn_eu = make_btn_multiline(tc, "EndpointUP",   76, 52, 0x1F6FEB, &lv_font_montserrat_14);
  lv_obj_t *btn_ed = make_btn_multiline(tc, "EndpointDOWN", 76, 52, 0x1F6FEB, &lv_font_montserrat_14);
  lv_obj_set_pos(btn_eu, 8, 204); lv_obj_set_pos(btn_ed, 88, 204);
  lv_obj_t *b_back_t = make_btn(tn, "Back", 140, 44, 0x2A2A2A, &lv_font_montserrat_20);
  lv_obj_align(b_back_t, LV_ALIGN_CENTER, 0, 0);

  // Endpoint screens
  ep_up_scr = lv_obj_create(NULL); build_endpoint_screen(ep_up_scr, "Endpoint UP", true);
  ep_dn_scr = lv_obj_create(NULL); build_endpoint_screen(ep_dn_scr, "Endpoint DOWN", false);

  // WiFi screen
  wifi_scr = lv_obj_create(NULL); style_screen(wifi_scr);
  lv_obj_t *wc = make_content(wifi_scr);
  lv_obj_t *wn = make_nav(wifi_scr);
  lv_obj_t *wt = make_title(wc, "WiFi"); lv_obj_align(wt, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_t *wcard = make_card(wc, 150, 100);
  lbl_wifi_status = lv_label_create(wcard);
  lv_obj_set_style_text_color(lbl_wifi_status, lv_color_hex(0x00FF00), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl_wifi_status, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_label_set_long_mode(lbl_wifi_status, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_wifi_status, 130);
  lv_obj_align(lbl_wifi_status, LV_ALIGN_TOP_LEFT, 0, 0);
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
  lv_obj_add_event_cb(b_back_ss, [](lv_event_t *e){ LV_UNUSED(e); go(settings_scr); }, LV_EVENT_CLICKED, nullptr);

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

  lv_obj_add_event_cb(b_speed,  on_go_speed,  LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_tuning, on_go_tuning, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_cal,    on_calibrate, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_reset,  on_reset_counter, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_back_s, on_go_main, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_stall, [](lv_event_t *e){ LV_UNUSED(e); ui_update_sg_val(); go(stall_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_wifi, [](lv_event_t *e){ LV_UNUSED(e); ui_update_wifi_label(); go(wifi_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(slider_speed, on_speed_slider, LV_EVENT_VALUE_CHANGED, nullptr);
  lv_obj_add_event_cb(b_back_p, [](lv_event_t *e){ LV_UNUSED(e); go(settings_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(btn_eu, on_go_ep_up, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(btn_ed, on_go_ep_dn, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_back_t, [](lv_event_t *e){ LV_UNUSED(e); go(settings_scr); }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b_back_w, [](lv_event_t *e){ LV_UNUSED(e); go(settings_scr); }, LV_EVENT_CLICKED, nullptr);

  lv_timer_create(counter_timer_cb, 100, nullptr);

  ui_update_speed_val();
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

  // Deferred reboot (safe from main loop context)
  if (rebootRequested && (millis() - rebootRequestMs) > 500) {
    if (stepper && stepper->isRunning()) stepper->forceStop();
    delay(100);
    ESP.restart();
  }

  delay(1);
}
