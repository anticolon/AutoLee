#pragma once
// ============================================================================
//  Motor control: stall detection, calibration, homing, motion handler
// ============================================================================

#include "config.h"
#include "stall_monitor.h"
#include "calibration.h"

static StallMonitor stallMon;

static const CalibrationConfig calCfg = {
  EARLY_WINDOW_MS, EARLY_WINDOW_DST_MAX,
  EARLY_MIN_TIME_MS, EARLY_MIN_MOVE_STEPS,
  EARLY_TRIP, CAL_HIT_CONFIRM,
  CAL_ABS_MIN, CAL_REL_DROP_Q8, 200
};

// --- Forward declarations for functions defined in other modules ---
void webLog(const char *fmt, ...);
void showJamScreen();
void setRunButtonState(bool running);
void ui_update_main_warning();
void ui_update_tuning_numbers();
void ui_update_endpoint_edit_values();
void go(lv_obj_t *scr);

// ==========================================================================
//  StallGuard reading
// ==========================================================================
inline uint16_t read_sg_raw() {
  digitalWrite(14, HIGH);
  delayMicroseconds(10);
  uint32_t drv = driver.DRV_STATUS();
  // Reject SPI failures: all-ones or all-zeros indicate bus error
  if (drv == 0xFFFFFFFF || drv == 0x00000000) {
    return 0;  // sg <= 1 guard in handleMotion() will skip this reading
  }
  return (uint16_t)(drv & 0x03FF);
}

// Median-of-5 filter to reject SPI glitch spikes (more robust than median-of-3)
uint16_t read_sg() {
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

inline void fas_wait_for_stop() {
  while (stepper && stepper->isRunning()) { lv_timer_handler(); delay(1); }
}

// ==========================================================================
//  Endpoint math
// ==========================================================================
void recomputeEffectiveEndpoints() {
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
//  Run start / stop
// ==========================================================================
bool startRunBetweenEndpoints() {
  if (!stepper) { webLog("RUN: no stepper"); return false; }
  if (!endpointsCalibrated) { webLog("RUN: not calibrated"); return false; }
  runState = RUNNING;
  stallMon.reset();
  lastSGPrintMs = 0;
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
  stepper->setAcceleration(RUN_ACCEL);
  stepper->moveTo(currentTarget);
  return true;
}

void requestGracefulStop() {
  if (!stepper) return;
  runState = STOPPING;
  stopEntryMs = millis();
  currentTarget = endpointUp;
  stepper->setAcceleration(RUN_ACCEL);
  stepper->moveTo(endpointUp);
}

// ==========================================================================
//  Motion handler (called from loop)
// ==========================================================================
void handleMotion() {
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
        stallMon.reset();
        stepper->setSpeedInHz(ui_speed_hz);
        stepper->setAcceleration(RUN_ACCEL);
        stepper->moveTo(currentTarget);
        break;
      }

      // --- Runtime stall detection (via StallMonitor) ---
      if (RUN_SG_TRIP == 0) break;

      uint32_t sinceChange = millis() - lastDirectionChangeMs;

      if (stallMon.checkBlanking(pos, currentTarget, endpointDown,
                                 sinceChange, ui_speed_hz, RUN_ACCEL,
                                 SG_WORK_ZONE_STEPS, RUN_SG_HIGH_NEEDED))
        break;

      uint16_t sg = read_sg();

      // Debug: print SG every 500ms
      if (sg > 1 && (millis() - lastSGPrintMs) > 500) {
        int32_t distToTarget = labs(pos - currentTarget);
        webLog("RUN SG=%u trip=%u pos=%ld dist=%ld t=%lu hi=%u/%u",
               sg, RUN_SG_TRIP, pos, distToTarget, sinceChange,
               stallMon.highCount, RUN_SG_HIGH_NEEDED);
        lastSGPrintMs = millis();
      }

      StallAction action = stallMon.processSG(sg, RUN_SG_TRIP, RUN_SG_HIGH_NEEDED);

      if (action == StallAction::JAM) {
        webLog("JAM! SG=%u trip=%u pos=%ld tgt=%ld cnt=%u",
                      sg, RUN_SG_TRIP, pos, currentTarget, stallMon.highCount);

        stepper->forceStop();
        fas_wait_for_stop();

        stepper->setSpeedInHz(CREEP_HOME_SPEED);
        stepper->setAcceleration(CREEP_HOME_ACCEL);

        int32_t backoff = (currentTarget == endpointDown) ? -RUN_BACKOFF_STEPS : +RUN_BACKOFF_STEPS;
        stepper->move(backoff);
        fas_wait_for_stop();

        runState = STALLED;
        stallMon.reset();
        showJamScreen();
      } else if (sg > RUN_SG_TRIP) {
        webLog("SG HIGH=%u trip=%u cnt=%u pos=%ld t=%lu",
               sg, RUN_SG_TRIP, stallMon.highCount, pos, sinceChange);
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

// ==========================================================================
//  Sensorless calibration
// ==========================================================================
bool move_until_stall(int dir, long &hit_pos) {
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

  CalibrationDetector calDet(calCfg.absMin);
  uint32_t lastMUSPrint = start_ms;
  bool prevDynReady = false;

  while (stepper->isRunning()) {
    const uint32_t now = millis();
    const uint32_t elapsed_ms = now - start_ms;
    const int32_t dist = labs(stepper->getCurrentPosition() - start_pos);
    const uint16_t sg = read_sg();

    // Periodic debug during search
    if ((now - lastMUSPrint) > 400) {
      webLog("MUS: sg=%u dist=%ld el=%lu bl=%d dr=%d dtrip=%u",
             sg, dist, elapsed_ms, calDet.baselineStarted, calDet.dynReady, calDet.dynTrip);
      lastMUSPrint = now;
    }

    CalibrationResult result = calDet.process(sg, now, elapsed_ms, dist,
                                              ignore_ms, ignore_dst, calCfg);

    if (result == CalibrationResult::EARLY_HIT) {
      webLog("MUS: EARLY HIT sg=%u pos=%ld", sg, stepper->getCurrentPosition());
      stepper->forceStop(); fas_wait_for_stop();
      hit_pos = stepper->getCurrentPosition();
      return true;
    }
    if (result == CalibrationResult::DYN_HIT) {
      webLog("MUS: DYN HIT sg=%u trip=%u pos=%ld", sg, calDet.dynTrip, stepper->getCurrentPosition());
      stepper->forceStop(); fas_wait_for_stop();
      hit_pos = stepper->getCurrentPosition();
      return true;
    }
    if (calDet.dynReady && !prevDynReady) {
      webLog("MUS: baseline dyn_trip=%u", calDet.dynTrip);
      prevDynReady = true;
    }

    lv_timer_handler();
    delay(1);
  }
  hit_pos = stepper->getCurrentPosition();
  webLog("MUS: NO STALL DETECTED, ended at pos=%ld", hit_pos);
  return false;
}

// Return home: move toward UP endpoint with stall detection and retry logic
bool return_home_up_safe() {
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

bool calibrateEndpointsSensorless() {
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

// Safe creep home: slow sensorless move toward UP until mechanical stop,
// then back off and re-establish position. Called from jam screen button.
void safeCreepHome() {
  if (!stepper) return;
  runState = HOMING;

  if (jam_status_lbl) lv_label_set_text(jam_status_lbl, "Returning home...");
  lv_timer_handler();

  driver.rms_current(CAL_CURRENT_MA);
  stepper->setSpeedInHz(CAL_SPEED_HZ);
  stepper->setAcceleration(CAL_ACCEL);

  webLog("Creep home: start, I=%umA spd=%u", CAL_CURRENT_MA, CAL_SPEED_HZ);

  long hit_pos = 0;
  bool found = move_until_stall(-1, hit_pos);

  if (found) {
    webLog("Creep home: found stop at %ld", hit_pos);
    stepper->move(+300);
    fas_wait_for_stop();
    stepper->setCurrentPosition(0);
    rawUp = 0;
    recomputeEffectiveEndpoints();
    stepper->moveTo(endpointUp);
    fas_wait_for_stop();
  } else {
    webLog("Creep home: FAILED to find stop!");
    endpointsCalibrated = false;
    recomputeEffectiveEndpoints();
  }

  // Restore run current and speed
  driver.rms_current(RUN_CURRENT_MA);
  stepper->setSpeedInHz(ui_speed_hz);
  stepper->setAcceleration(RUN_ACCEL);

  runState = IDLE;
  webLog("Creep home: done pos=%ld", stepper->getCurrentPosition());
  setRunButtonState(false);
  ui_update_main_warning();

  if (found) {
    if (jam_status_lbl) lv_label_set_text(jam_status_lbl, "Home OK!");
    lv_timer_handler();
    ui_update_tuning_numbers();
    ui_update_endpoint_edit_values();
    delay(800);
    go(main_scr);
  } else {
    if (jam_status_lbl) lv_label_set_text(jam_status_lbl, "FAILED\nPlease recalibrate");
    lv_timer_handler();
    // Stay on jam screen — do NOT navigate to main_scr
  }
}
