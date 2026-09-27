// ============================================================================
//  AutoLee – motion.h
//  Motion control, stall detection, calibration, creep home
// ============================================================================
#pragma once

// All globals and forward declarations are provided by AutoLee.ino
// (single translation unit — Arduino IDE model)

// Forward declarations (defined later in this file)
bool move_until_stall(int dir, long &hit_pos, int32_t max_steps = CAL_SEARCH_STEPS);

// ==========================================================================
//  UTILITY
// ==========================================================================
static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline bool nearPos(long a, long b, long tol = 2) { return labs(a - b) <= tol; }

// Which end is this target? Uses proximity rather than equality so that a
// target captured before an endpoint offset was edited is still classified
// correctly (an exact == test silently mis-classifies a stale target).
static inline bool targetIsDown(long t) {
  return labs(t - endpointDown) <= labs(t - endpointUp);
}
static inline long flipTarget(long t) { return targetIsDown(t) ? endpointUp : endpointDown; }

static inline uint16_t read_sg_raw() {
  // Ensure display CS is not active (shared SPI bus)
  digitalWrite(LCD_CS, HIGH);
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
// Clears the per-stroke statistics. Called at every direction change.
static inline void resetStrokeStats() {
  runStrokeMinSG = 0xFFFF;
  runStrokeMaxSG = 0;
  runStrokeFloorCnt = 0;
}

// Reports the lowest SG of the stroke that just ended. Called at every
// direction change AND on any stop or jam — a run halted mid-stroke used to
// discard this silently, which is the one stroke worth seeing after a jam.
static void logStrokeStats(const char *why) {
  if (runStrokeMinSG == 0xFFFF) {
    if (runStrokeFloorCnt == 0) return;           // nothing sampled at all
    webLog("SG stroke %s: all %u readings at floor (<=1) trip=%u spd=%lu",
           why, runStrokeFloorCnt, RUN_SG_TRIP, (unsigned long)ui_speed_hz);
    return;
  }
  // max is what the trip is set against — a jam is an upward spike.
  webLog("SG stroke %s: max=%u min=%u floor=%u trip=%u spd=%lu",
         why, runStrokeMaxSG, runStrokeMinSG, runStrokeFloorCnt, RUN_SG_TRIP,
         (unsigned long)ui_speed_hz);
}

// SGT + filter. One place writes the StallGuard configuration.
static inline void applySGConfig() {
  driver.sgt(SGT_VALUE);
  driver.sfilt(SG_FILTER);
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
    int32_t clamped = (int32_t)(dnEff - rawDown);
    if (clamped != downOffsetSteps) {
      // Make the silent side effect visible: editing the UP offset can push
      // the DOWN offset to keep the minimum travel guard.
      webLog("EP: DOWN offset clamped %ld -> %ld (guard %ld)",
             (long)downOffsetSteps, (long)clamped, (long)ENDPOINT_GUARD);
      downOffsetSteps = clamped;
    }
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
  // Auto SG is exempt: it is the routine that clears this gate, and it
  // deliberately runs with detection off (autoSGActive is set before it
  // calls here). Without the exemption Auto SG could never start on a
  // machine that needs it.
  if (!sgCalibrated && !autoSGActive) {
    // Shipped trips are all 0, which means no jam detection whatsoever.
    // Refuse to run until Auto SG has measured them (or all three were set
    // by hand), rather than quietly running an unprotected press.
    webLog("Run blocked: jam detection not set up — run Auto SG first");
    return;
  }
  applyPendingProfile();      // never start a run with a switch still queued
  if (!autoSGActive && profileSgAtFloor(activeProfile)) {
    webLog("WARNING: %s runs at the SG floor (trip %u) — jam detection is LIMITED and may not stop a jam",
           profiles[activeProfile].name, RUN_SG_TRIP);
  }
  if (RUN_SG_TRIP == 0 && !autoSGActive) {
    // Allowed on purpose (0 = off is how raw SG is measured by hand), but
    // never silently.
    webLog("WARNING: %s trip is 0 — jam detection is OFF for this run",
           profiles[activeProfile].name);
  }
  runState = RUNNING;
  runSGHighCount = 0; runSGLowCount = 0;
  resetStrokeStats();
  lastDirectionChangeMs = millis();

  driver.rms_current(RUN_CURRENT_MA);
  driver.en_pwm_mode(false);
  driver.TPWMTHRS(0);
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(0); driver.semax(0);
  applySGConfig();

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
  if (runState == RUNNING) logStrokeStats("STOPPED");
  batchActive = false;
  if (!endpointsCalibrated) {
    // No trustworthy target to park at — just stop where we are.
    if (stepper->isRunning()) stepper->forceStop();
    runState = IDLE;
    return;
  }
  runState = STOPPING;
  stopEntryMs = millis();
  currentTarget = endpointUp;
  stepper->setSpeedInHz(ui_speed_hz);
  stepper->setAcceleration(RUN_DECEL);  // use fast decel for stop too
  stepper->moveTo(endpointUp);
}

// Forward declaration for jam handling (defined in ui_touch.h)
static void showJamScreen();

void handleMotion() {
  if (!stepper || runState == CALIBRATING || runState == STALLED || runState == HOMING) return;
  switch (runState) {
    case RUNNING: {
      long pos = stepper->getCurrentPosition();
      const bool headingDown = targetIsDown(currentTarget);

      if (!stepper->isRunning()) {
        // Per-stroke summary. min is the number that matters when choosing
        // thresholds: it is how close this stroke actually came to tripping.
        logStrokeStats(headingDown ? "DOWN" : "UP");

        if (autoSGActive) {
          autoSGStrokes++;
          // The first stroke starts from wherever the ram happened to be and
          // is usually a partial travel — discard it.
          if (autoSGStrokes > 1) {
            if (runStrokeMaxSG > autoSGMax) autoSGMax = runStrokeMaxSG;
            autoSGFloor += runStrokeFloorCnt;
          }
        }
        // Auto SG measurement strokes are not work cycles — keep them out of
        // the lifetime counter (and batches, which are never active then).
        if (headingDown && !autoSGActive) {
          // Display counter caps at 9999 (cosmetic only) — batch counting
          // must NOT be gated by it, or batches stall after 9999 lifetime cycles.
          if (counter < 9999) counter++;
          markSettingsDirty();
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
        // A queued profile switch lands HERE, between two strokes: the new
        // speed and its matching sg_trip come into force together, and the
        // blanking window + trip counters below are re-armed for it.
        applyPendingProfile();

        currentTarget = flipTarget(currentTarget);
        lastDirectionChangeMs = millis();
        runSGHighCount = 0; runSGLowCount = 0;
        resetStrokeStats();
        stepper->setSpeedInHz(ui_speed_hz);
        stepper->setAcceleration(RUN_DECEL);
        stepper->moveTo(currentTarget);
        break;
      }

      // --- Runtime SG monitoring / stall detection ---
      // POLARITY: SG_RESULT RISES under load on this machine, so a jam is
      // sg > trip. See the note in config.h — this is the v1.8 behaviour,
      // restored in v1.20 after the datasheet-derived sg < trip proved to
      // work only on the Slow profile.
      //
      // NOTE: there is deliberately NO early-out here when the trip is 0.
      // SG is always read and logged during monitored cruise, so raw values
      // can be measured with detection switched off — which is exactly the
      // state you tune from. The trip comparison below is what is gated.
      uint32_t sinceChange = millis() - lastDirectionChangeMs;

      // Accel blank: v/a at RUN_DECEL + margin
      uint32_t accelWindowMs = (uint32_t)((uint64_t)ui_speed_hz * 1000ULL / (uint64_t)RUN_DECEL) + 80;
      if (sinceChange < accelWindowMs) break;

      // Work zone: skip SG near the DOWN endpoint where the tool does work
      // (primer push etc.) — normal resistance here would false-trigger.
      // Only applies when heading toward DOWN, not toward UP.
      if (headingDown) {
        int32_t distToDown = labs(pos - endpointDown);
        if (distToDown < SG_WORK_ZONE_STEPS) {
          runSGHighCount = 0; runSGLowCount = 0;
          break;
        }
      }

      // Decel blank: position-based, using actual decel distance + margin.
      // Skip UNLESS we already have jam evidence (carry-through).
      // Note: runSGHighCount can never reach RUN_SG_TRIP_NEEDED here (the jam
      // would already have fired), so carry-through must key on ANY prior
      // loaded reading (count > 0), not on count >= needed.
      {
        int32_t distToTarget = labs(pos - currentTarget);
        int32_t decelDist = (int32_t)((uint64_t)ui_speed_hz * ui_speed_hz / (2ULL * (uint64_t)RUN_DECEL));
        int32_t decelBlank = decelDist + 500;  // margin for planner timing
        if (distToTarget < decelBlank && runSGHighCount == 0) {
          runSGLowCount = 0;
          break;
        }
      }

      uint16_t sg = read_sg();
      // 0/1 is the bottom of the SG_RESULT range. The median-of-5 makes a
      // single SPI glitch unlikely to land here; at speed on a low supply
      // voltage (24 V on Normal/Fast) it is the genuine clean-running value.
      // Kept out of min/max and detection as before, but counted.
      if (sg <= 1) {
        if (runStrokeFloorCnt < 0xFFFF) runStrokeFloorCnt++;
        break;
      }

      if (sg < runStrokeMinSG) runStrokeMinSG = sg;
      if (sg > runStrokeMaxSG) {
        runStrokeMaxSG = sg;
        // Log new maxima as they occur. The 500 ms sampler below steps right
        // over a ~2 ms load spike, which is the only moment that matters.
        // Throttled so the normal settling at stroke start is a couple of
        // lines, not a flood.
        static uint32_t lastMaxLogMs = 0;
        if ((millis() - lastMaxLogMs) > 150) {
          webLog("SG new max=%u trip=%u pos=%ld t=%lu", sg, RUN_SG_TRIP, pos, sinceChange);
          lastMaxLogMs = millis();
        }
      }

      // Debug: print SG every 500ms. This runs whether or not the trip is
      // armed, so raw values can be measured with detection switched off.
      static uint32_t lastSGPrintMs = 0;
      if ((millis() - lastSGPrintMs) > 500) {
        int32_t distToTarget = labs(pos - currentTarget);
        webLog("RUN SG=%u max=%u min=%u trip=%u%s pos=%ld dist=%ld t=%lu hi=%u/%u",
               sg, runStrokeMaxSG, runStrokeMinSG, RUN_SG_TRIP,
               (RUN_SG_TRIP == 0) ? " [OFF]" : "",
               pos, distToTarget, sinceChange, runSGHighCount, RUN_SG_TRIP_NEEDED);
        lastSGPrintMs = millis();
      }

      // A jam is SG_RESULT rising above the profile's trip. trip == 0
      // disables detection for this profile (readings are still logged).
      if (RUN_SG_TRIP > 0 && sg > RUN_SG_TRIP) {
        if (runSGHighCount < RUN_SG_TRIP_NEEDED + 4) runSGHighCount++;
        runSGLowCount = 0;

        webLog("SG HIGH=%u trip=%u cnt=%u pos=%ld t=%lu",
               sg, RUN_SG_TRIP, runSGHighCount, pos, sinceChange);

        if (runSGHighCount >= RUN_SG_TRIP_NEEDED) {
          // JAM — trigger immediately, no sustain timer
          webLog("JAM! SG=%u max=%u trip=%u pos=%ld tgt=%ld cnt=%u",
                        sg, runStrokeMaxSG, RUN_SG_TRIP, pos, currentTarget, runSGHighCount);

          logStrokeStats("JAMMED");
          stepper->forceStop();
          fas_wait_for_stop();

          stepper->setSpeedInHz(CREEP_HOME_SPEED);
          stepper->setAcceleration(CREEP_HOME_ACCEL);

          int32_t backoff = headingDown ? -RUN_BACKOFF_STEPS : +RUN_BACKOFF_STEPS;
          stepper->move(backoff);
          fas_wait_for_stop();

          runState = STALLED;
          runSGHighCount = 0; runSGLowCount = 0;
          resetStrokeStats();
          batchActive = false;   // a jam ends the batch; Return Home re-homes
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
        webLog("Stop: TIMEOUT at pos=%ld", pos);
        stepper->forceStop();
        runState = IDLE;
      }
      break;
    }

    default: break;
  }
}

// ==========================================================================
//  SAFE CREEP HOME
//  Slow sensorless move toward UP until the mechanical stop, then back off
//  and re-establish position. Triggered from the jam screen or the web UI,
//  but ALWAYS executed from loop() (see handleHomeRequest) so the nested
//  lv_timer_handler() calls inside move_until_stall() actually redraw.
// ==========================================================================
void safeCreepHome() {
  if (!stepper) return;
  runState = HOMING;
  batchActive = false;

  if (jam_status_lbl) lv_label_set_text(jam_status_lbl, "Returning home...");
  lv_refr_now(NULL);   // force a redraw now; lv_timer_handler() may be nested

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
    // We do NOT know where the ram is any more: the jam forceStop() lost an
    // unknown number of steps and homing failed to re-reference. Leaving
    // endpointsCalibrated set here would let the next RUN drive to a stale
    // target at full speed. Invalidate and force a recalibration.
    webLog("Creep home: FAILED to find stop — calibration invalidated");
    endpointsCalibrated = false;
    recomputeEffectiveEndpoints();
  }

  // Restore run current and speed
  driver.rms_current(RUN_CURRENT_MA);
  stepper->setSpeedInHz(ui_speed_hz);
  stepper->setAcceleration(RUN_DECEL);

  runState = IDLE;

  webLog("Creep home: done pos=%ld", stepper->getCurrentPosition());

  if (jam_status_lbl) lv_label_set_text(jam_status_lbl, found ? "Home OK!" : "FAILED - recalibrate");
  lv_refr_now(NULL);

  setRunButtonState(false);
  ui_update_main_warning();
  ui_update_tuning_numbers();
  ui_update_endpoint_edit_values();

  delay(800);
  go(main_scr);
}

// ==========================================================================
//  SENSORLESS STALL SEARCH (no static state that affects behaviour)
//
//  dir:       -1 = toward UP, +1 = toward DOWN
//  hit_pos:   position where the stop was detected
//  max_steps: travel limit for this search
//
//  Detection is two-stage and the stages OVERLAP:
//    1. absolute trip (sg <= EARLY_TRIP) from EARLY_MIN_* until the
//       baseline is ready — covers the whole accel ramp;
//    2. dynamic trip (sg <= 92% of the measured no-load baseline) after.
// ==========================================================================
bool move_until_stall(int dir, long &hit_pos, int32_t max_steps) {
  if (max_steps < 1) max_steps = 1;
  const int32_t target = (dir > 0) ? +max_steps : -max_steps;
  const int32_t start_pos = stepper->getCurrentPosition();
  const uint32_t accel_ms = (uint32_t)((uint64_t)CAL_SPEED_HZ * 1000ULL / (uint64_t)CAL_ACCEL);
  const int32_t  accel_dist = (int32_t)((uint64_t)CAL_SPEED_HZ * (uint64_t)CAL_SPEED_HZ / (2ULL * (uint64_t)CAL_ACCEL));
  const uint32_t ignore_ms  = accel_ms + 100;
  const int32_t  ignore_dst = (accel_dist * 8) / 10;

  driver.en_pwm_mode(false);
  driver.TPWMTHRS(0);
  driver.TCOOLTHRS(0xFFFFF);
  applySGConfig();

  webLog("MUS: dir=%d pos=%ld max=%ld ign_ms=%lu ign_dst=%ld sgt=%d",
         dir, (long)start_pos, (long)max_steps, ignore_ms, (long)ignore_dst, (int)SGT_VALUE);

  stepper->move(target);
  const uint32_t start_ms = millis();
  uint32_t last_print_ms = start_ms;
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
    if ((now - last_print_ms) > 400) {
      webLog("MUS: sg=%u dist=%ld el=%lu bl=%d dr=%d dtrip=%u",
             sg, (long)dist, elapsed_ms, baseline_started, dyn_ready, dyn_trip);
      last_print_ms = now;
    }

    // Stage 1 — absolute trip. Stays armed until the dynamic detector takes
    // over, so there is no blind window between the two.
    if (!dyn_ready && elapsed_ms >= EARLY_MIN_TIME_MS && dist >= EARLY_MIN_MOVE_STEPS) {
      if (sg <= EARLY_TRIP) {
        if (++confirm_early >= CAL_HIT_CONFIRM) {
          webLog("MUS: EARLY HIT sg=%u pos=%ld", sg, stepper->getCurrentPosition());
          stepper->forceStop(); fas_wait_for_stop();
          hit_pos = stepper->getCurrentPosition();
          return true;
        }
      } else confirm_early = 0;
    }

    // Baseline accumulation
    if (!baseline_started && elapsed_ms > ignore_ms && dist > ignore_dst) {
      baseline_started = true;
      base_start_ms = now;
      base_sum = 0; base_cnt = 0; confirm_dyn = 0;
    }
    if (baseline_started && !dyn_ready) {
      // Sum and count must saturate together or the average is skewed.
      if (base_cnt < 1000) { base_sum += sg; base_cnt++; }
      if ((now - base_start_ms) >= 200 && base_cnt > 0) {
        uint16_t baseline = min((uint16_t)(base_sum / base_cnt), (uint16_t)1023);
        uint16_t rel_trip = (uint16_t)((baseline * (uint32_t)CAL_REL_DROP_Q8) >> 8);
        dyn_trip = max(rel_trip, CAL_ABS_MIN);
        dyn_ready = true;
        confirm_early = 0;
        webLog("MUS: baseline=%u dyn_trip=%u", baseline, dyn_trip);
      }
    }

    // Stage 2 — dynamic trip
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
  webLog("MUS: no stall within %ld steps, ended at pos=%ld", (long)max_steps, hit_pos);
  return false;
}

// ==========================================================================
//  AUTO SG CALIBRATION
//  Runs every speed profile with its trip disabled, records the highest
//  SG_RESULT each one reaches, and sets the trip to max + SG_AUTO_MARGIN.
//
//  Blocking, and always called from loop() (see handleAutoSGRequest), so the
//  lv_timer_handler() calls below really do drive the display. runState stays
//  RUNNING while measuring — handleMotion() has to keep working — and
//  autoSGActive is what tells the rest of the firmware a measurement is in
//  progress. autoSGActive stays set for the WHOLE routine, including the
//  brief IDLE between two profiles, so no touch or web handler can start a
//  run, a calibration or a second Auto SG in that gap.
//
//  broadcastState() is called from the loops below so the web panel keeps
//  updating (and its STOP button keeps working) for the minute this takes.
//
//  DETECTION IS OFF WHILE THIS RUNS. The press must be empty.
// ==========================================================================
bool autoCalibrateSG() {
  if (!stepper) return false;
  if (!endpointsCalibrated) {
    webLog("AutoSG: endpoints not calibrated — run Calibrate first");
    return false;
  }
  if (runState != IDLE) return false;

  pendingProfile = -1;
  const uint8_t savedProfile = activeProfile;
  uint16_t savedTrip[NUM_PROFILES];
  for (uint8_t i = 0; i < NUM_PROFILES; i++) savedTrip[i] = profiles[i].sg_trip;

  uint16_t measured[NUM_PROFILES] = { 0 };
  bool ok = true;

  webLog("AutoSG: start — %u strokes per profile, detection OFF, press must be EMPTY",
         SG_AUTO_STROKES);
  autoSGActive = true;                  // cleared at the single exit below

  for (uint8_t p = 0; p < NUM_PROFILES && ok; p++) {
    activeProfile = p;
    profiles[p].sg_trip = 0;            // measure, don't detect
    stepper->setSpeedInHz(ui_speed_hz);
    ui_update_speed_val();
    ui_update_profile_screen();
    ui_update_sg_val();
    webLog("AutoSG: measuring %s (%lu Hz)...", profiles[p].name, (unsigned long)ui_speed_hz);

    autoSGMax = 0;
    autoSGStrokes = 0;
    autoSGFloor = 0;
    startRunBetweenEndpoints();
    setRunButtonState(true);

    const uint32_t t0 = millis();
    while (autoSGStrokes < (uint16_t)SG_AUTO_STROKES + 1 && runState == RUNNING) {
      handleMotion();
      lv_timer_handler();
      broadcastState();
      delay(1);
      // Abort paths: STOP from the web, or the touch RUN button (which puts
      // the machine into STOPPING and drops us out of the loop condition).
      if (webToggleRunRequested || webStopRequested) {
        webToggleRunRequested = false;
        webStopRequested = false;
        webLog("AutoSG: stopped by user");
        ok = false;
        break;
      }
      if ((millis() - t0) > SG_AUTO_TIMEOUT_MS) {
        webLog("AutoSG: TIMEOUT measuring %s", profiles[p].name);
        ok = false;
        break;
      }
    }
    if (ok && runState != RUNNING) {
      webLog("AutoSG: run ended early — aborting");
      ok = false;
    }

    requestGracefulStop();
    const uint32_t t1 = millis();
    while (runState == STOPPING && (millis() - t1) < (STOP_TIMEOUT_MS + 2000)) {
      handleMotion();
      lv_timer_handler();
      broadcastState();
      delay(1);
    }
    setRunButtonState(false);

    if (!ok) break;
    if (autoSGMax == 0 && autoSGFloor == 0) {
      webLog("AutoSG: no SG samples at all for %s — is the work zone covering the whole stroke?",
             profiles[p].name);
      ok = false;
      break;
    }
    if (autoSGMax == 0) {
      // Every sample was 0/1: SG is at its floor at this speed. That is a
      // valid clean measurement — a jam shows as a spike off the floor.
      measured[p] = 1;
      webLog("AutoSG: %s read only SG floor (%lu samples) — clean max=1",
             profiles[p].name, (unsigned long)autoSGFloor);
      webLog("AutoSG: %s detection relies on stall spikes off the floor — BLOCK-TEST IT",
             profiles[p].name);
    } else {
      measured[p] = autoSGMax;
      webLog("AutoSG: %s max=%u over %u strokes (floor samples=%lu)",
             profiles[p].name, measured[p], SG_AUTO_STROKES, (unsigned long)autoSGFloor);
    }
  }

  activeProfile = savedProfile;
  if (stepper) stepper->setSpeedInHz(ui_speed_hz);
  runState = IDLE;
  autoSGActive = false;

  if (!ok) {
    for (uint8_t i = 0; i < NUM_PROFILES; i++) profiles[i].sg_trip = savedTrip[i];
    webLog("AutoSG: ABORTED — previous trips restored");
  } else {
    for (uint8_t i = 0; i < NUM_PROFILES; i++) {
      int32_t trip = (int32_t)measured[i] + (int32_t)SG_AUTO_MARGIN;
      profiles[i].sg_trip = (uint16_t)clamp_i32(trip, RUN_SG_TRIP_MIN, RUN_SG_TRIP_MAX);
      webLog("AutoSG: %s  max=%u  +%u  -> trip=%u",
             profiles[i].name, measured[i], SG_AUTO_MARGIN, profiles[i].sg_trip);
    }
    sgCalibrated = allSgTripsSet();
    sgCalCurrentMa = RUN_CURRENT_MA;    // trips are only valid at this current
    markSettingsDirty();
    webLog("AutoSG: measured at %u mA", sgCalCurrentMa);
    webLog("AutoSG: done — verify with a deliberate block on each profile");
  }

  ui_update_speed_val();
  ui_update_profile_screen();
  ui_update_sg_val();
  ui_update_main_warning();
  return ok;
}

// ==========================================================================
//  SENSORLESS CALIBRATION
// ==========================================================================
bool calibrateEndpointsSensorless() {
  if (!stepper) return false;
  runState = CALIBRATING;
  endpointsCalibrated = false;
  batchActive = false;
  if (stepper->isRunning()) { stepper->forceStop(); fas_wait_for_stop(); }

  const uint32_t saved_speed = ui_speed_hz;
  stepper->setSpeedInHz(CAL_SPEED_HZ);
  stepper->setAcceleration(CAL_ACCEL);
  driver.rms_current(CAL_CURRENT_MA);

  // Pre-move DOWN to give the UP search room to accelerate.
  // Stall-guarded: if calibration is started with the ram already near the
  // bottom, a blind move would ram the mechanical stop at full cal current.
  long premove_hit = 0;
  if (move_until_stall(+1, premove_hit, CAL_PREMOVE_DOWN_STEPS)) {
    webLog("CAL: premove hit a stop at %ld — backing off", premove_hit);
    stepper->move(-300);
    fas_wait_for_stop();
  }

  // Find UP
  long hit_up = 0;
  if (!move_until_stall(-1, hit_up)) {
    webLog("CAL: UP search failed");
    driver.rms_current(RUN_CURRENT_MA);
    stepper->setSpeedInHz(saved_speed);
    stepper->setAcceleration(RUN_DECEL);
    recomputeEffectiveEndpoints();
    runState = IDLE;
    return false;
  }

  stepper->move(+300); fas_wait_for_stop();
  stepper->setCurrentPosition(0);
  rawUp = 0;

  // Find DOWN
  long hit_down = 0;
  if (!move_until_stall(+1, hit_down)) {
    webLog("CAL: DOWN search failed");
    driver.rms_current(RUN_CURRENT_MA);
    stepper->setSpeedInHz(saved_speed);
    stepper->setAcceleration(RUN_DECEL);
    recomputeEffectiveEndpoints();
    runState = IDLE;
    return false;
  }

  stepper->move(-300); fas_wait_for_stop();
  rawDown = stepper->getCurrentPosition();

  if ((rawDown - rawUp) < (ENDPOINT_GUARD * 2)) {
    webLog("CAL: travel too short (%ld steps) — rejecting", rawDown - rawUp);
    driver.rms_current(RUN_CURRENT_MA);
    stepper->setSpeedInHz(saved_speed);
    stepper->setAcceleration(RUN_DECEL);
    recomputeEffectiveEndpoints();
    runState = IDLE;
    return false;
  }

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
