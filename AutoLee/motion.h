// ============================================================================
//  AutoLee – motion.h
//  Motion control, stall detection, calibration, creep home
// ============================================================================
#pragma once

// All globals and forward declarations are provided by AutoLee.ino
// (single translation unit — Arduino IDE model)

// Forward declarations (defined later in this file)
bool move_until_stall(int dir, long &hit_pos);

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
void fas_wait_for_stop() {
  while (stepper && stepper->isRunning()) { lv_timer_handler(); delay(1); }
}

// ==========================================================================
//  ENDPOINT MATH
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
//  MOTION
// ==========================================================================
// Decel distance at RUN_DECEL: v²/(2*a). At 40000Hz/800000 = 1000 steps, ~50ms.
// Both accel and decel use this rate — fast ramps, maximum cruise time for SG monitoring.

void startRunBetweenEndpoints() {
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

void requestGracefulStop() {
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
void safeCreepHome() {
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
