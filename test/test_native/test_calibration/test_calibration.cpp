#include <unity.h>
#include "calibration.h"

static CalibrationConfig defaultCfg() {
    return { 300, 1200, 50, 200, 12, 2, 12, 235, 200 };
}

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// 1. Initial state
// ---------------------------------------------------------------------------
void test_initial_state(void) {
    CalibrationDetector d(12);
    TEST_ASSERT_FALSE(d.dynReady);
    TEST_ASSERT_FALSE(d.baselineStarted);
    TEST_ASSERT_EQUAL_UINT8(0, d.earlyConfirm);
    TEST_ASSERT_EQUAL_UINT8(0, d.dynConfirm);
    TEST_ASSERT_EQUAL_UINT16(12, d.dynTrip);
    TEST_ASSERT_EQUAL_UINT16(0, d.baseCnt);
    TEST_ASSERT_EQUAL_UINT32(0, d.baseSum);
}

// ---------------------------------------------------------------------------
// 2. Early hit: two consecutive low SG in window → EARLY_HIT
// ---------------------------------------------------------------------------
void test_early_hit_two_consecutive(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // First low SG sample in early window
    auto r1 = d.process(5, 100, 100, 300, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r1);
    TEST_ASSERT_EQUAL_UINT8(1, d.earlyConfirm);
    // Second consecutive low SG → hit
    auto r2 = d.process(5, 110, 110, 310, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::EARLY_HIT, r2);
}

// ---------------------------------------------------------------------------
// 3. Early hit: one low then one high resets confirm → CONTINUE
// ---------------------------------------------------------------------------
void test_early_hit_reset_on_high(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // First low
    d.process(5, 100, 100, 300, 500, 500, cfg);
    TEST_ASSERT_EQUAL_UINT8(1, d.earlyConfirm);
    // High SG resets
    d.process(50, 110, 110, 310, 500, 500, cfg);
    TEST_ASSERT_EQUAL_UINT8(0, d.earlyConfirm);
    // Low again — only 1, not 2
    auto r = d.process(5, 120, 120, 320, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r);
    TEST_ASSERT_EQUAL_UINT8(1, d.earlyConfirm);
}

// ---------------------------------------------------------------------------
// 4. Early: too soon (elapsed < earlyMinTimeMs) → CONTINUE even with low SG
// ---------------------------------------------------------------------------
void test_early_too_soon(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // elapsed=30 < earlyMinTimeMs=50, dist=300 >= earlyMinMoveSteps=200
    auto r = d.process(1, 30, 30, 300, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r);
    TEST_ASSERT_EQUAL_UINT8(0, d.earlyConfirm);
}

// ---------------------------------------------------------------------------
// 5. Early: too little movement (dist < earlyMinMoveSteps) → CONTINUE
// ---------------------------------------------------------------------------
void test_early_too_little_movement(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // elapsed=100 >= 50, dist=100 < earlyMinMoveSteps=200
    auto r = d.process(1, 100, 100, 100, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r);
    TEST_ASSERT_EQUAL_UINT8(0, d.earlyConfirm);
}

// ---------------------------------------------------------------------------
// 6. Early: outside time window → CONTINUE (early detection stops)
// ---------------------------------------------------------------------------
void test_early_outside_time_window(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // elapsed=400 > earlyWindowMs=300
    auto r = d.process(1, 400, 400, 300, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r);
    TEST_ASSERT_EQUAL_UINT8(0, d.earlyConfirm);
}

// ---------------------------------------------------------------------------
// 7. Early: outside distance window → CONTINUE
// ---------------------------------------------------------------------------
void test_early_outside_distance_window(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // elapsed=100 <= 300, dist=1300 > earlyWindowDstMax=1200
    auto r = d.process(1, 100, 100, 1300, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r);
    TEST_ASSERT_EQUAL_UINT8(0, d.earlyConfirm);
}

// ---------------------------------------------------------------------------
// 8. Baseline: doesn't start during ignore period
// ---------------------------------------------------------------------------
void test_baseline_not_during_ignore(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // ignoreMs=500, ignoreDist=500; elapsed=400, dist=400 — still in ignore
    d.process(100, 400, 400, 400, 500, 500, cfg);
    TEST_ASSERT_FALSE(d.baselineStarted);
    // elapsed=500, dist=600 — elapsed NOT > ignoreMs (equal)
    d.process(100, 500, 500, 600, 500, 500, cfg);
    TEST_ASSERT_FALSE(d.baselineStarted);
    // elapsed=600, dist=400 — dist NOT > ignoreDist
    d.process(100, 600, 600, 400, 500, 500, cfg);
    TEST_ASSERT_FALSE(d.baselineStarted);
}

// ---------------------------------------------------------------------------
// 9. Baseline: starts after ignore period
// ---------------------------------------------------------------------------
void test_baseline_starts_after_ignore(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // elapsed=600 > 500, dist=600 > 500
    d.process(100, 600, 600, 600, 500, 500, cfg);
    TEST_ASSERT_TRUE(d.baselineStarted);
    TEST_ASSERT_EQUAL_UINT32(600, d.baseStartMs);
    TEST_ASSERT_EQUAL_UINT16(1, d.baseCnt);
    TEST_ASSERT_EQUAL_UINT32(100, d.baseSum);
}

// ---------------------------------------------------------------------------
// 10. Baseline: accumulates samples and computes trip after settleMs
// ---------------------------------------------------------------------------
void test_baseline_accumulates_and_computes(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // Start baseline at t=600
    d.process(100, 600, 600, 600, 500, 500, cfg);
    TEST_ASSERT_TRUE(d.baselineStarted);
    TEST_ASSERT_FALSE(d.dynReady);
    // Feed more samples, but still within settleMs (< 200ms from base start)
    d.process(100, 700, 700, 700, 500, 500, cfg);
    TEST_ASSERT_FALSE(d.dynReady);
    TEST_ASSERT_EQUAL_UINT16(2, d.baseCnt);
    TEST_ASSERT_EQUAL_UINT32(200, d.baseSum);
    // t=800 (200ms from base start = 600) — trip computed
    d.process(100, 800, 800, 800, 500, 500, cfg);
    TEST_ASSERT_TRUE(d.dynReady);
    // baseline = 300/3 = 100; relTrip = 100*235/256 = 91; max(91, 12) = 91
    TEST_ASSERT_EQUAL_UINT16(91, d.dynTrip);
}

// ---------------------------------------------------------------------------
// 11. Baseline: trip floor at absMin — very high SG still gives absMin trip
// ---------------------------------------------------------------------------
void test_baseline_trip_floor_at_absmin(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // Low SG baseline → relTrip below absMin
    // If baseline=10, relTrip = 10*235/256 = 9 → floored to 12
    d.process(10, 600, 600, 600, 500, 500, cfg);
    d.process(10, 800, 800, 800, 500, 500, cfg);
    TEST_ASSERT_TRUE(d.dynReady);
    // baseline=20/2=10, relTrip=10*235>>8=9, max(9,12)=12
    TEST_ASSERT_EQUAL_UINT16(12, d.dynTrip);
}

// ---------------------------------------------------------------------------
// 12. Baseline: baseCnt caps at 1000
// ---------------------------------------------------------------------------
void test_baseline_cnt_caps_at_1000(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // Use a settle time that won't be reached, so we can pump samples
    CalibrationConfig longSettle = cfg;
    longSettle.baselineSettleMs = 100000;  // Very long settle
    // Start baseline
    d.process(100, 600, 600, 600, 500, 500, longSettle);
    TEST_ASSERT_TRUE(d.baselineStarted);
    // Feed 1100 more samples (total 1101 process calls but cnt caps at 1000)
    for (int i = 0; i < 1100; i++) {
        d.process(100, 700 + i, 700 + i, 700 + i, 500, 500, longSettle);
    }
    TEST_ASSERT_EQUAL_UINT16(1000, d.baseCnt);
}

// ---------------------------------------------------------------------------
// 13. Dynamic hit: two consecutive below dynTrip → DYN_HIT
// ---------------------------------------------------------------------------
void test_dyn_hit_two_consecutive(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // Build baseline: SG=100 at t=600, t=800 → dynReady
    d.process(100, 600, 600, 600, 500, 500, cfg);
    d.process(100, 800, 800, 800, 500, 500, cfg);
    TEST_ASSERT_TRUE(d.dynReady);
    // dynTrip = 91 (from test_baseline_accumulates_and_computes)
    // First low SG
    auto r1 = d.process(5, 900, 900, 900, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r1);
    TEST_ASSERT_EQUAL_UINT8(1, d.dynConfirm);
    // Second low SG → DYN_HIT
    auto r2 = d.process(5, 910, 910, 910, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::DYN_HIT, r2);
}

// ---------------------------------------------------------------------------
// 14. Dynamic hit: one below then one above resets → CONTINUE
// ---------------------------------------------------------------------------
void test_dyn_hit_reset_on_high(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // Build baseline
    d.process(100, 600, 600, 600, 500, 500, cfg);
    d.process(100, 800, 800, 800, 500, 500, cfg);
    TEST_ASSERT_TRUE(d.dynReady);
    // Low
    d.process(5, 900, 900, 900, 500, 500, cfg);
    TEST_ASSERT_EQUAL_UINT8(1, d.dynConfirm);
    // High resets
    d.process(200, 910, 910, 910, 500, 500, cfg);
    TEST_ASSERT_EQUAL_UINT8(0, d.dynConfirm);
    // Low again — only 1
    auto r = d.process(5, 920, 920, 920, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r);
    TEST_ASSERT_EQUAL_UINT8(1, d.dynConfirm);
}

// ---------------------------------------------------------------------------
// 15. Dynamic: not checked before dynReady → CONTINUE
// ---------------------------------------------------------------------------
void test_dyn_not_checked_before_ready(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // Very low SG but baseline not started yet — no dynamic detection
    auto r1 = d.process(0, 600, 600, 600, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r1);
    // Baseline started but not ready yet (settle not met)
    auto r2 = d.process(0, 700, 700, 700, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r2);
    TEST_ASSERT_FALSE(d.dynReady);
    TEST_ASSERT_EQUAL_UINT8(0, d.dynConfirm);
}

// ---------------------------------------------------------------------------
// 16. Full sequence: early hit — simulate immediate wall contact
// ---------------------------------------------------------------------------
void test_full_sequence_early_hit(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // Motor starts, quickly hits wall in early window
    auto r1 = d.process(80, 20, 20, 50, 500, 500, cfg);   // too soon, too little dist
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r1);
    auto r2 = d.process(80, 40, 40, 150, 500, 500, cfg);   // still too little dist
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r2);
    auto r3 = d.process(5, 60, 60, 250, 500, 500, cfg);    // first low in valid early zone
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r3);
    auto r4 = d.process(3, 70, 70, 280, 500, 500, cfg);    // second consecutive → EARLY_HIT
    TEST_ASSERT_EQUAL(CalibrationResult::EARLY_HIT, r4);
    // Baseline should not have started (ignore period not passed)
    TEST_ASSERT_FALSE(d.baselineStarted);
}

// ---------------------------------------------------------------------------
// 17. Full sequence: normal calibration — ignore window, baseline, then stall
// ---------------------------------------------------------------------------
void test_full_sequence_normal_calibration(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    uint32_t t = 0;
    // Phase 1: early window — normal SG, no hit
    for (int i = 0; i < 5; i++) {
        t += 50;
        auto r = d.process(100, t, t, t * 2, 500, 500, cfg);
        TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r);
    }
    // Phase 2: ignore window (elapsed <= 500, dist <= 500) — still no baseline
    TEST_ASSERT_FALSE(d.baselineStarted);

    // Phase 3: past ignore window — baseline starts
    t = 600;
    d.process(80, t, t, 600, 500, 500, cfg);
    TEST_ASSERT_TRUE(d.baselineStarted);
    TEST_ASSERT_FALSE(d.dynReady);

    // Phase 4: accumulate baseline (settle = 200ms)
    for (int i = 1; i <= 10; i++) {
        d.process(80, 600 + i * 20, 600 + i * 20, 600 + i * 20, 500, 500, cfg);
    }
    // At t=800 (200ms from base start 600) — settled
    TEST_ASSERT_TRUE(d.dynReady);
    // baseline = 80 (all samples 80), relTrip = 80*235/256 = 73, max(73,12) = 73
    TEST_ASSERT_EQUAL_UINT16(73, d.dynTrip);

    // Phase 5: normal readings above trip — CONTINUE
    auto r1 = d.process(80, 900, 900, 900, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r1);

    // Phase 6: stall detected — two consecutive low readings
    auto r2 = d.process(10, 1000, 1000, 1000, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r2);
    auto r3 = d.process(8, 1010, 1010, 1010, 500, 500, cfg);
    TEST_ASSERT_EQUAL(CalibrationResult::DYN_HIT, r3);
}

// ---------------------------------------------------------------------------
// 18. Full sequence: no hit — all readings stay high, always CONTINUE
// ---------------------------------------------------------------------------
void test_full_sequence_no_hit(void) {
    CalibrationDetector d(12);
    auto cfg = defaultCfg();
    // Run through early window with high SG
    for (uint32_t t = 50; t <= 300; t += 50) {
        auto r = d.process(100, t, t, t, 500, 500, cfg);
        TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r);
    }
    // Past ignore, baseline builds
    for (uint32_t t = 600; t <= 2000; t += 50) {
        auto r = d.process(100, t, t, t, 500, 500, cfg);
        TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r);
    }
    // dynReady should be true by now
    TEST_ASSERT_TRUE(d.dynReady);
    // But SG stays high (100 > dynTrip), so always CONTINUE
    for (uint32_t t = 2050; t <= 3000; t += 50) {
        auto r = d.process(100, t, t, t, 500, 500, cfg);
        TEST_ASSERT_EQUAL(CalibrationResult::CONTINUE, r);
    }
    TEST_ASSERT_EQUAL_UINT8(0, d.dynConfirm);
}

// ---------------------------------------------------------------------------
// 19. Baseline calculation accuracy — verify dynTrip = max(baseline * 235/256, 12)
// ---------------------------------------------------------------------------
void test_baseline_calculation_accuracy(void) {
    // Test several baseline values and verify the math
    struct TestCase {
        uint16_t sg;
        uint16_t expectedTrip;
    };
    // baseline=sg (with 2 samples), relTrip = sg*235/256
    // sg=100 → 100*235/256 = 91.79 → 91, max(91,12) = 91
    // sg=50  → 50*235/256  = 45.89 → 45, max(45,12) = 45
    // sg=14  → 14*235/256  = 12.85 → 12, max(12,12) = 12
    // sg=13  → 13*235/256  = 11.93 → 11, max(11,12) = 12
    // sg=10  → 10*235/256  = 9.17  → 9,  max(9,12)  = 12
    // sg=200 → 200*235/256 = 183.59→ 183, max(183,12)= 183
    TestCase cases[] = {
        {100, 91},
        {50,  45},
        {14,  12},
        {13,  12},
        {10,  12},
        {200, 183},
    };

    for (auto& tc : cases) {
        CalibrationDetector d(12);
        auto cfg = defaultCfg();
        // Two samples of sg at t=600, t=800 → baseline = sg
        d.process(tc.sg, 600, 600, 600, 500, 500, cfg);
        d.process(tc.sg, 800, 800, 800, 500, 500, cfg);
        TEST_ASSERT_TRUE(d.dynReady);
        TEST_ASSERT_EQUAL_UINT16(tc.expectedTrip, d.dynTrip);
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
    UNITY_BEGIN();
    // Initial state
    RUN_TEST(test_initial_state);
    // Early hit detection
    RUN_TEST(test_early_hit_two_consecutive);
    RUN_TEST(test_early_hit_reset_on_high);
    RUN_TEST(test_early_too_soon);
    RUN_TEST(test_early_too_little_movement);
    RUN_TEST(test_early_outside_time_window);
    RUN_TEST(test_early_outside_distance_window);
    // Baseline accumulation
    RUN_TEST(test_baseline_not_during_ignore);
    RUN_TEST(test_baseline_starts_after_ignore);
    RUN_TEST(test_baseline_accumulates_and_computes);
    RUN_TEST(test_baseline_trip_floor_at_absmin);
    RUN_TEST(test_baseline_cnt_caps_at_1000);
    // Dynamic stall detection
    RUN_TEST(test_dyn_hit_two_consecutive);
    RUN_TEST(test_dyn_hit_reset_on_high);
    RUN_TEST(test_dyn_not_checked_before_ready);
    // Full sequences
    RUN_TEST(test_full_sequence_early_hit);
    RUN_TEST(test_full_sequence_normal_calibration);
    RUN_TEST(test_full_sequence_no_hit);
    // Calculation accuracy
    RUN_TEST(test_baseline_calculation_accuracy);
    return UNITY_END();
}
