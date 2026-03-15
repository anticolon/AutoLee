#pragma once
// ============================================================================
//  calibration.h — Calibration hit detection: early trip + baseline + dynamic
// ============================================================================

#include <cstdint>

struct CalibrationConfig {
    uint32_t earlyWindowMs;       // Time window for early trip detection (300)
    int32_t  earlyWindowDstMax;   // Max distance for early trip (1200)
    uint32_t earlyMinTimeMs;      // Min elapsed before early trip valid (50)
    int32_t  earlyMinMoveSteps;   // Min distance before early trip valid (200)
    uint16_t earlyTrip;           // SG threshold for early trip (12)
    uint8_t  hitConfirm;          // Consecutive confirmations needed (2)
    uint16_t absMin;              // Absolute minimum dynamic trip (12)
    uint8_t  relDropQ8;           // Q8 fixed-point drop fraction (235 = ~92%)
    uint32_t baselineSettleMs;    // Time after baseline starts before computing trip (200)
};

enum class CalibrationResult : uint8_t {
    CONTINUE,    // No hit — keep searching
    EARLY_HIT,   // Hit detected in early window
    DYN_HIT      // Hit detected via dynamic threshold
};

struct CalibrationDetector {
    // Early window
    uint8_t earlyConfirm = 0;

    // Baseline
    bool baselineStarted = false;
    uint32_t baseStartMs = 0;
    uint32_t baseSum = 0;
    uint16_t baseCnt = 0;

    // Dynamic trip
    bool dynReady = false;
    uint16_t dynTrip;
    uint8_t dynConfirm = 0;

    explicit CalibrationDetector(uint16_t absMin) : dynTrip(absMin) {}

    // Process one SG sample. Called once per loop iteration during move_until_stall.
    //
    // Parameters:
    //   sg         — current (filtered) StallGuard reading
    //   nowMs      — current millis()
    //   elapsedMs  — time since move started
    //   dist       — absolute distance traveled from start position
    //   ignoreMs   — accel ignore window (ms) — don't start baseline before this
    //   ignoreDist — accel ignore distance — don't start baseline before this
    //   cfg        — calibration configuration constants
    //
    CalibrationResult process(uint16_t sg, uint32_t nowMs, uint32_t elapsedMs,
                              int32_t dist, uint32_t ignoreMs, int32_t ignoreDist,
                              const CalibrationConfig& cfg) {
        // 1. Early trip detection
        if (elapsedMs <= cfg.earlyWindowMs && dist <= cfg.earlyWindowDstMax) {
            if (elapsedMs >= cfg.earlyMinTimeMs && dist >= cfg.earlyMinMoveSteps) {
                if (sg <= cfg.earlyTrip) {
                    if (++earlyConfirm >= cfg.hitConfirm) return CalibrationResult::EARLY_HIT;
                } else {
                    earlyConfirm = 0;
                }
            }
        }

        // 2. Baseline accumulation
        if (!baselineStarted && elapsedMs > ignoreMs && dist > ignoreDist) {
            baselineStarted = true;
            baseStartMs = nowMs;
            baseSum = 0;
            baseCnt = 0;
            dynConfirm = 0;
        }
        if (baselineStarted && !dynReady) {
            baseSum += sg;
            if (baseCnt < 1000) baseCnt++;
            if ((nowMs - baseStartMs) >= cfg.baselineSettleMs && baseCnt > 0) {
                uint16_t baseline = (uint16_t)(baseSum / baseCnt);
                if (baseline > 1023) baseline = 1023;
                uint16_t relTrip = (uint16_t)((baseline * (uint32_t)cfg.relDropQ8) >> 8);
                dynTrip = (relTrip > cfg.absMin) ? relTrip : cfg.absMin;
                dynReady = true;
            }
        }

        // 3. Dynamic stall detection
        if (dynReady) {
            if (sg <= dynTrip) {
                if (++dynConfirm >= cfg.hitConfirm) return CalibrationResult::DYN_HIT;
            } else {
                dynConfirm = 0;
            }
        }

        return CalibrationResult::CONTINUE;
    }
};
