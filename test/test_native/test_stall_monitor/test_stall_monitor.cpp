#include <unity.h>
#include "stall_monitor.h"

static StallMonitor sm;

// Typical motion parameters for tests
static const uint32_t SPEED   = 40000;   // 40 kHz
static const uint32_t ACCEL   = 800000;  // 800k steps/s^2
static const int32_t  EP_DOWN = 50000;   // DOWN endpoint position
static const int32_t  EP_UP   = 0;       // UP endpoint position
static const int32_t  WZ      = 5500;    // work zone steps
static const uint8_t  NEEDED  = 5;       // highNeeded threshold
static const uint16_t TRIP    = 80;      // SG trip threshold

void setUp(void) {
    sm = StallMonitor{};  // reset to initial state
}

void tearDown(void) {}

// ---- 1. Initial state ----
void test_initial_state(void) {
    TEST_ASSERT_EQUAL_UINT8(0, sm.highCount);
    TEST_ASSERT_EQUAL_UINT8(0, sm.lowCount);
}

// ---- 2. processSG: SPI error (sg=0, sg=1) ----
void test_processSG_spi_error_zero(void) {
    StallAction a = sm.processSG(0, TRIP, NEEDED);
    TEST_ASSERT_EQUAL(StallAction::OK, a);
    TEST_ASSERT_EQUAL_UINT8(0, sm.highCount);
    TEST_ASSERT_EQUAL_UINT8(0, sm.lowCount);
}

void test_processSG_spi_error_one(void) {
    sm.highCount = 3;
    sm.lowCount = 2;
    StallAction a = sm.processSG(1, TRIP, NEEDED);
    TEST_ASSERT_EQUAL(StallAction::OK, a);
    // Counters must not change
    TEST_ASSERT_EQUAL_UINT8(3, sm.highCount);
    TEST_ASSERT_EQUAL_UINT8(2, sm.lowCount);
}

// ---- 3. processSG: below trip ----
void test_processSG_below_trip_increments_lowCount(void) {
    sm.processSG(50, TRIP, NEEDED);  // 50 <= 80 → below trip
    TEST_ASSERT_EQUAL_UINT8(1, sm.lowCount);
    sm.processSG(50, TRIP, NEEDED);
    TEST_ASSERT_EQUAL_UINT8(2, sm.lowCount);
}

void test_processSG_below_trip_three_lows_decrements_high(void) {
    sm.highCount = 2;
    sm.processSG(50, TRIP, NEEDED);  // lowCount=1
    sm.processSG(50, TRIP, NEEDED);  // lowCount=2
    sm.processSG(50, TRIP, NEEDED);  // lowCount=3 → reset to 0, highCount 2→1
    TEST_ASSERT_EQUAL_UINT8(0, sm.lowCount);
    TEST_ASSERT_EQUAL_UINT8(1, sm.highCount);
}

// ---- 4. processSG: above trip ----
void test_processSG_above_trip_increments_high_resets_low(void) {
    sm.lowCount = 2;
    StallAction a = sm.processSG(TRIP + 1, TRIP, NEEDED);
    TEST_ASSERT_EQUAL(StallAction::OK, a);
    TEST_ASSERT_EQUAL_UINT8(1, sm.highCount);
    TEST_ASSERT_EQUAL_UINT8(0, sm.lowCount);
}

// ---- 5. processSG: jam trigger ----
void test_processSG_jam_trigger(void) {
    // Feed NEEDED high readings to trigger JAM
    StallAction a = StallAction::OK;
    for (uint8_t i = 0; i < NEEDED; i++) {
        a = sm.processSG(TRIP + 10, TRIP, NEEDED);
    }
    TEST_ASSERT_EQUAL(StallAction::JAM, a);
    TEST_ASSERT_EQUAL_UINT8(NEEDED, sm.highCount);
}

// ---- 6. processSG: highCount caps at highNeeded+4 ----
void test_processSG_highCount_caps(void) {
    // Feed many more than needed
    for (uint8_t i = 0; i < NEEDED + 10; i++) {
        sm.processSG(TRIP + 10, TRIP, NEEDED);
    }
    TEST_ASSERT_EQUAL_UINT8(NEEDED + 4, sm.highCount);
}

// ---- 7. processSG: slow decay ----
void test_processSG_slow_decay(void) {
    sm.highCount = 1;
    // 3 lows → highCount decrements from 1 to 0
    sm.processSG(50, TRIP, NEEDED);
    sm.processSG(50, TRIP, NEEDED);
    sm.processSG(50, TRIP, NEEDED);
    TEST_ASSERT_EQUAL_UINT8(0, sm.highCount);
    TEST_ASSERT_EQUAL_UINT8(0, sm.lowCount);

    // 3 more lows — highCount already 0, stays at 0
    sm.processSG(50, TRIP, NEEDED);
    sm.processSG(50, TRIP, NEEDED);
    sm.processSG(50, TRIP, NEEDED);
    TEST_ASSERT_EQUAL_UINT8(0, sm.highCount);
}

// ---- 8. processSG: fast accumulation with alternating highs ----
void test_processSG_alternating_highs_accumulate(void) {
    // high, low, high, low, high — highs accumulate, lows don't reach 3
    sm.processSG(TRIP + 10, TRIP, NEEDED);  // high=1, low=0
    sm.processSG(50, TRIP, NEEDED);          // high=1, low=1
    sm.processSG(TRIP + 10, TRIP, NEEDED);  // high=2, low=0
    sm.processSG(50, TRIP, NEEDED);          // high=2, low=1
    sm.processSG(TRIP + 10, TRIP, NEEDED);  // high=3, low=0
    TEST_ASSERT_EQUAL_UINT8(3, sm.highCount);
    TEST_ASSERT_EQUAL_UINT8(0, sm.lowCount);
}

// ---- 9. checkBlanking: accel window ----
void test_checkBlanking_accel_window(void) {
    // accel window = 40000*1000/800000 + 80 = 130 ms
    // During accel (t=100ms < 130ms): blanked
    int32_t midPos = 25000;  // mid-stroke
    TEST_ASSERT_TRUE(sm.checkBlanking(midPos, EP_UP, EP_DOWN, 100, SPEED, ACCEL, WZ, NEEDED));
    // After accel (t=200ms > 130ms): not blanked (mid-stroke, not in any zone)
    TEST_ASSERT_FALSE(sm.checkBlanking(midPos, EP_UP, EP_DOWN, 200, SPEED, ACCEL, WZ, NEEDED));
}

// ---- 10. checkBlanking: work zone heading DOWN ----
void test_checkBlanking_work_zone_heading_down(void) {
    // Position 4000 steps from DOWN, heading to DOWN → in work zone (4000 < 5500)
    int32_t pos = EP_DOWN - 4000;
    sm.highCount = 3;
    sm.lowCount = 2;
    TEST_ASSERT_TRUE(sm.checkBlanking(pos, EP_DOWN, EP_DOWN, 500, SPEED, ACCEL, WZ, NEEDED));
    // Counters must be reset
    TEST_ASSERT_EQUAL_UINT8(0, sm.highCount);
    TEST_ASSERT_EQUAL_UINT8(0, sm.lowCount);
}

// ---- 11. checkBlanking: work zone heading UP ----
void test_checkBlanking_work_zone_heading_up(void) {
    // Near DOWN but heading UP → work zone blanking does NOT apply
    int32_t pos = EP_DOWN - 4000;
    sm.highCount = 3;
    // Past accel window (500 > 130), heading UP, mid-stroke
    // Need to also be outside decel blank: distToTarget = |46000 - 0| = 46000 >> decel
    TEST_ASSERT_FALSE(sm.checkBlanking(pos, EP_UP, EP_DOWN, 500, SPEED, ACCEL, WZ, NEEDED));
    // Counters must NOT be reset (no blanking applied)
    TEST_ASSERT_EQUAL_UINT8(3, sm.highCount);
}

// ---- 12. checkBlanking: decel blank with no evidence ----
void test_checkBlanking_decel_blank_no_evidence(void) {
    // decel distance = 40000^2 / (2*800000) = 1000 steps
    // decel blank = 1000 + 500 = 1500 steps from target
    // Position 1000 steps from target UP (0) → 1000 < 1500 → blanked
    int32_t pos = 1000;
    sm.highCount = 2;
    sm.lowCount = 1;
    TEST_ASSERT_TRUE(sm.checkBlanking(pos, EP_UP, EP_DOWN, 500, SPEED, ACCEL, WZ, NEEDED));
    // Counters must be reset
    TEST_ASSERT_EQUAL_UINT8(0, sm.highCount);
    TEST_ASSERT_EQUAL_UINT8(0, sm.lowCount);
}

// ---- 13. checkBlanking: decel blank WITH evidence (highCount >= highNeeded) ----
void test_checkBlanking_decel_blank_with_evidence(void) {
    // Same position near target, but highCount >= highNeeded → NOT blanked
    int32_t pos = 1000;
    sm.highCount = NEEDED;  // jam evidence accumulated
    sm.lowCount = 1;
    TEST_ASSERT_FALSE(sm.checkBlanking(pos, EP_UP, EP_DOWN, 500, SPEED, ACCEL, WZ, NEEDED));
    // Counters must NOT be reset
    TEST_ASSERT_EQUAL_UINT8(NEEDED, sm.highCount);
    TEST_ASSERT_EQUAL_UINT8(1, sm.lowCount);
}

// ---- 14. checkBlanking: no blanking in middle of stroke ----
void test_checkBlanking_no_blanking_mid_stroke(void) {
    // Mid-stroke, past accel window, not near any endpoint
    int32_t pos = 25000;
    TEST_ASSERT_FALSE(sm.checkBlanking(pos, EP_UP, EP_DOWN, 500, SPEED, ACCEL, WZ, NEEDED));
}

// ---- 15. reset() ----
void test_reset(void) {
    sm.highCount = 7;
    sm.lowCount = 2;
    sm.reset();
    TEST_ASSERT_EQUAL_UINT8(0, sm.highCount);
    TEST_ASSERT_EQUAL_UINT8(0, sm.lowCount);
}

// ---- 16. Full sequence: normal run (many OK readings, no jam) ----
void test_full_sequence_normal_run(void) {
    // Simulate a full stroke with all-low SG values — no jam
    for (int i = 0; i < 50; i++) {
        StallAction a = sm.processSG(50, TRIP, NEEDED);
        TEST_ASSERT_EQUAL(StallAction::OK, a);
    }
    TEST_ASSERT_EQUAL_UINT8(0, sm.highCount);
}

// ---- 17. Full sequence: jam ----
void test_full_sequence_jam(void) {
    // Simulate sustained high SG → JAM
    StallAction last = StallAction::OK;
    for (uint8_t i = 0; i < NEEDED + 2; i++) {
        last = sm.processSG(TRIP + 20, TRIP, NEEDED);
        if (last == StallAction::JAM) break;
    }
    TEST_ASSERT_EQUAL(StallAction::JAM, last);
    TEST_ASSERT_TRUE(sm.highCount >= NEEDED);
}

// ---- 18. Full sequence: transient spike then recovery ----
void test_full_sequence_transient_spike(void) {
    // One high reading, then lots of lows → should recover, no jam
    sm.processSG(TRIP + 10, TRIP, NEEDED);  // high=1
    TEST_ASSERT_EQUAL_UINT8(1, sm.highCount);

    // 10 cycles of low SG — 3 lows per highCount decrement
    for (int i = 0; i < 10; i++) {
        StallAction a = sm.processSG(50, TRIP, NEEDED);
        TEST_ASSERT_EQUAL(StallAction::OK, a);
    }
    // After 10 lows: 3 lows → high=0, then remaining lows keep it at 0
    TEST_ASSERT_EQUAL_UINT8(0, sm.highCount);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    // Initial state
    RUN_TEST(test_initial_state);
    // processSG: SPI errors
    RUN_TEST(test_processSG_spi_error_zero);
    RUN_TEST(test_processSG_spi_error_one);
    // processSG: below trip
    RUN_TEST(test_processSG_below_trip_increments_lowCount);
    RUN_TEST(test_processSG_below_trip_three_lows_decrements_high);
    // processSG: above trip
    RUN_TEST(test_processSG_above_trip_increments_high_resets_low);
    // processSG: jam trigger
    RUN_TEST(test_processSG_jam_trigger);
    // processSG: highCount cap
    RUN_TEST(test_processSG_highCount_caps);
    // processSG: slow decay
    RUN_TEST(test_processSG_slow_decay);
    // processSG: alternating highs accumulate
    RUN_TEST(test_processSG_alternating_highs_accumulate);
    // checkBlanking: accel window
    RUN_TEST(test_checkBlanking_accel_window);
    // checkBlanking: work zone
    RUN_TEST(test_checkBlanking_work_zone_heading_down);
    RUN_TEST(test_checkBlanking_work_zone_heading_up);
    // checkBlanking: decel blank
    RUN_TEST(test_checkBlanking_decel_blank_no_evidence);
    RUN_TEST(test_checkBlanking_decel_blank_with_evidence);
    // checkBlanking: no blanking
    RUN_TEST(test_checkBlanking_no_blanking_mid_stroke);
    // reset
    RUN_TEST(test_reset);
    // Full sequences
    RUN_TEST(test_full_sequence_normal_run);
    RUN_TEST(test_full_sequence_jam);
    RUN_TEST(test_full_sequence_transient_spike);
    return UNITY_END();
}
