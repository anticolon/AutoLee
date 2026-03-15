#pragma once
// ============================================================================
//  sg_filter.h — StallGuard signal filtering and blanking helpers (testable)
// ============================================================================

#include <cstdint>
#include <cstdlib>

// Median-of-5: sort 5 samples and return the middle value.
// Rejects SPI glitch spikes more robustly than median-of-3.
static inline uint16_t median_of_5(uint16_t s[5]) {
    // Insertion sort for 5 elements
    for (int i = 1; i < 5; i++) {
        uint16_t key = s[i];
        int j = i - 1;
        while (j >= 0 && s[j] > key) {
            s[j + 1] = s[j];
            j--;
        }
        s[j + 1] = key;
    }
    return s[2];
}

// Check if current position is in the work zone (near DOWN endpoint).
// SG monitoring is skipped here because normal tool resistance would false-trigger.
// Only applies when heading toward DOWN endpoint.
static inline bool sg_in_work_zone(long pos, long endpointDown, int32_t workZoneSteps, long currentTarget) {
    if (currentTarget != endpointDown) return false;
    int32_t distToDown = labs(pos - endpointDown);
    return distToDown < workZoneSteps;
}

// Compute acceleration blanking window in milliseconds.
// During acceleration, SG readings are unreliable. Skip monitoring for v/a + margin.
static inline uint32_t compute_accel_window_ms(uint32_t speed_hz, uint32_t accel, uint32_t margin_ms = 80) {
    return (uint32_t)((uint64_t)speed_hz * 1000ULL / (uint64_t)accel) + margin_ms;
}

// Compute deceleration distance in steps: v^2 / (2*a)
// Used to blank SG monitoring during the final approach to target.
static inline int32_t compute_decel_distance(uint32_t speed_hz, uint32_t accel) {
    return (int32_t)((uint64_t)speed_hz * speed_hz / (2ULL * (uint64_t)accel));
}
