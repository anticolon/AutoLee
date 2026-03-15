#include <unity.h>
#include "endpoint_math.h"

void test_clamp_i32_within_range(void) {
    TEST_ASSERT_EQUAL_INT32(50, clamp_i32(50, 0, 100));
}

void test_clamp_i32_below_min(void) {
    TEST_ASSERT_EQUAL_INT32(0, clamp_i32(-10, 0, 100));
}

void test_clamp_i32_above_max(void) {
    TEST_ASSERT_EQUAL_INT32(100, clamp_i32(200, 0, 100));
}

void test_clamp_i32_at_boundaries(void) {
    TEST_ASSERT_EQUAL_INT32(0, clamp_i32(0, 0, 100));
    TEST_ASSERT_EQUAL_INT32(100, clamp_i32(100, 0, 100));
}

void test_nearPos_exact(void) {
    TEST_ASSERT_TRUE(nearPos(100, 100));
}

void test_nearPos_within_tolerance(void) {
    TEST_ASSERT_TRUE(nearPos(100, 102));
    TEST_ASSERT_TRUE(nearPos(100, 98));
}

void test_nearPos_outside_tolerance(void) {
    TEST_ASSERT_FALSE(nearPos(100, 103));
    TEST_ASSERT_FALSE(nearPos(100, 97));
}

void test_nearPos_custom_tolerance(void) {
    TEST_ASSERT_TRUE(nearPos(100, 110, 10));
    TEST_ASSERT_FALSE(nearPos(100, 111, 10));
}

void test_flipTarget_at_up(void) {
    TEST_ASSERT_EQUAL(5000, flipTarget(0, 0, 5000));
}

void test_flipTarget_at_down(void) {
    TEST_ASSERT_EQUAL(0, flipTarget(5000, 0, 5000));
}

void test_flipTarget_at_neither(void) {
    // When at neither endpoint, returns endpointUp (default case)
    TEST_ASSERT_EQUAL(0, flipTarget(2500, 0, 5000));
}

void test_recompute_basic(void) {
    EndpointState s;
    s.rawUp = 0; s.rawDown = 10000;
    s.upOffset = 100; s.downOffset = -500;
    s.calibrated = true;
    recomputeEndpoints(s, -8000, 8000, 50);
    TEST_ASSERT_EQUAL(100, s.effectiveUp);
    TEST_ASSERT_EQUAL(9500, s.effectiveDown);
}

void test_recompute_guard_enforced(void) {
    EndpointState s;
    s.rawUp = 0; s.rawDown = 100;
    s.upOffset = 0; s.downOffset = -100;  // would make effectiveDown = 0 = effectiveUp
    s.calibrated = true;
    recomputeEndpoints(s, -8000, 8000, 50);
    TEST_ASSERT_EQUAL(0, s.effectiveUp);
    TEST_ASSERT_EQUAL(50, s.effectiveDown);  // guard enforced
    TEST_ASSERT_EQUAL(-50, s.downOffset);    // offset adjusted
}

void test_recompute_offset_clamped(void) {
    EndpointState s;
    s.rawUp = 0; s.rawDown = 10000;
    s.upOffset = 99999; s.downOffset = -99999;
    s.calibrated = true;
    recomputeEndpoints(s, -8000, 8000, 50);
    TEST_ASSERT_EQUAL_INT32(8000, s.upOffset);
    // upOffset clamped to 8000 -> effectiveUp = 8000
    // downOffset clamped to -8000 -> effectiveDown = 2000
    // guard triggers: 2000 <= 8000+50, so effectiveDown = 8050, downOffset = 8050-10000 = -1950
    TEST_ASSERT_EQUAL_INT32(-1950, s.downOffset);
    TEST_ASSERT_EQUAL(8000, s.effectiveUp);
    TEST_ASSERT_EQUAL(8050, s.effectiveDown);
}

void test_recompute_not_calibrated(void) {
    EndpointState s;
    s.rawUp = 0; s.rawDown = 10000;
    s.upOffset = 100; s.downOffset = -500;
    s.calibrated = false;
    recomputeEndpoints(s, -8000, 8000, 50);
    TEST_ASSERT_EQUAL(0, s.effectiveUp);
    TEST_ASSERT_EQUAL(0, s.effectiveDown);
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_clamp_i32_within_range);
    RUN_TEST(test_clamp_i32_below_min);
    RUN_TEST(test_clamp_i32_above_max);
    RUN_TEST(test_clamp_i32_at_boundaries);
    RUN_TEST(test_nearPos_exact);
    RUN_TEST(test_nearPos_within_tolerance);
    RUN_TEST(test_nearPos_outside_tolerance);
    RUN_TEST(test_nearPos_custom_tolerance);
    RUN_TEST(test_flipTarget_at_up);
    RUN_TEST(test_flipTarget_at_down);
    RUN_TEST(test_flipTarget_at_neither);
    RUN_TEST(test_recompute_basic);
    RUN_TEST(test_recompute_guard_enforced);
    RUN_TEST(test_recompute_offset_clamped);
    RUN_TEST(test_recompute_not_calibrated);
    return UNITY_END();
}
