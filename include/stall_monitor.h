#pragma once
// ============================================================================
//  stall_monitor.h — Runtime stall detection state machine (testable)
// ============================================================================

#include <cstdint>
#include <cstdlib>
#include "sg_filter.h"

enum class StallAction : uint8_t {
    BLANKED,    // In a blanking window — do not read SG
    OK,         // SG was processed, no jam
    JAM         // Stall threshold reached — trigger jam response
};

struct StallMonitor {
    uint8_t highCount = 0;
    uint8_t lowCount = 0;

    void reset() { highCount = 0; lowCount = 0; }

    // Check blanking windows. If blanked, resets counters where appropriate.
    // Returns true if SG monitoring should be skipped this cycle.
    bool checkBlanking(int32_t pos, int32_t currentTarget, int32_t endpointDown,
                       uint32_t msSinceChange, uint32_t speed_hz, uint32_t accel,
                       int32_t workZoneSteps, uint8_t highNeeded) {
        // Accel blank (no counter reset — motor is still accelerating)
        uint32_t accelMs = compute_accel_window_ms(speed_hz, accel);
        if (msSinceChange < accelMs) return true;

        // Work zone blank (resets counters — normal resistance here)
        if (sg_in_work_zone(pos, endpointDown, workZoneSteps, currentTarget)) {
            reset();
            return true;
        }

        // Decel blank (resets counters, but only if no jam evidence yet)
        int32_t distToTarget = labs(pos - currentTarget);
        int32_t decelDist = compute_decel_distance(speed_hz, accel);
        int32_t decelBlank = decelDist + 500;  // margin for planner timing
        if (distToTarget < decelBlank && highCount < highNeeded) {
            reset();
            return true;
        }

        return false;
    }

    // Process a single SG reading. Call only when not blanked and trip > 0.
    // sg values of 0 or 1 are treated as SPI errors and ignored.
    StallAction processSG(uint16_t sg, uint16_t trip, uint8_t highNeeded) {
        if (sg <= 1) return StallAction::OK;  // SPI error

        if (sg > trip) {
            if (highCount < highNeeded + 4) highCount++;
            lowCount = 0;
            if (highCount >= highNeeded) return StallAction::JAM;
        } else {
            lowCount++;
            if (lowCount >= 3) {
                lowCount = 0;
                if (highCount > 0) highCount--;
            }
        }
        return StallAction::OK;
    }
};
