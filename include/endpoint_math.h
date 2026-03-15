#pragma once
// ============================================================================
//  endpoint_math.h — Pure endpoint math and utility functions (testable)
// ============================================================================

#include <cstdint>
#include <cstdlib>  // for labs()
static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline bool nearPos(long a, long b, long tol = 2) {
    return labs(a - b) <= tol;
}

static inline long flipTarget(long pos, long endpointUp, long endpointDown) {
    return (pos == endpointUp) ? endpointDown : endpointUp;
}

// --- Endpoint state and computation ---
struct EndpointState {
    long rawUp = 0;
    long rawDown = 0;
    int32_t upOffset = 0;
    int32_t downOffset = 0;
    long effectiveUp = 0;
    long effectiveDown = 0;
    bool calibrated = false;
};

// Recompute effective endpoints from raw + offsets, clamping offsets and enforcing guard gap.
// Modifies state in-place: clamps offsets, computes effectiveUp/Down.
// If not calibrated, zeros the effective endpoints.
static inline void recomputeEndpoints(EndpointState &s, int32_t offsetMin, int32_t offsetMax, int32_t guard) {
    if (!s.calibrated) {
        s.effectiveUp = 0;
        s.effectiveDown = 0;
        return;
    }
    s.upOffset = clamp_i32(s.upOffset, offsetMin, offsetMax);
    s.downOffset = clamp_i32(s.downOffset, offsetMin, offsetMax);
    long upEff = s.rawUp + s.upOffset;
    long dnEff = s.rawDown + s.downOffset;
    if (dnEff <= (upEff + guard)) {
        dnEff = upEff + guard;
        s.downOffset = (int32_t)(dnEff - s.rawDown);
    }
    s.effectiveUp = upEff;
    s.effectiveDown = dnEff;
}
