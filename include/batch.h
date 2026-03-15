#pragma once
// ============================================================================
//  batch.h — Batch run state machine (testable)
// ============================================================================

#include <cstdint>

struct BatchState {
    int32_t target = 0;    // 0 = unlimited (no batch)
    int32_t count = 0;     // completed strokes in current batch
    bool active = false;

    void reset() {
        target = 0;
        count = 0;
        active = false;
    }

    // Adjust target by delta, clamped to [0, max]
    void adjustTarget(int32_t delta, int32_t max = 9999) {
        int32_t v = target + delta;
        if (v < 0) v = 0;
        if (v > max) v = max;
        target = v;
    }

    // Start a batch run. Resets count, sets active.
    // Returns false if target is 0 (can't start unlimited batch).
    bool start() {
        if (target <= 0) return false;
        count = 0;
        active = true;
        return true;
    }

    // Record one completed stroke. Returns true if batch is now complete.
    // Only counts if batch is active.
    bool recordStroke() {
        if (!active) return false;
        count++;
        if (count >= target) {
            active = false;
            return true;  // batch complete
        }
        return false;
    }

    // How many strokes remain
    int32_t remaining() const {
        if (!active || target <= 0) return 0;
        int32_t r = target - count;
        return (r < 0) ? 0 : r;
    }
};
