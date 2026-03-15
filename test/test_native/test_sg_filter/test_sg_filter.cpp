#include <unity.h>
#include "sg_filter.h"

void test_median_of_5_sorted(void) {
    uint16_t s[5] = {10, 20, 30, 40, 50};
    TEST_ASSERT_EQUAL_UINT16(30, median_of_5(s));
}

void test_median_of_5_reversed(void) {
    uint16_t s[5] = {50, 40, 30, 20, 10};
    TEST_ASSERT_EQUAL_UINT16(30, median_of_5(s));
}

void test_median_of_5_one_spike(void) {
    // One outlier spike — should be rejected
    uint16_t s[5] = {100, 102, 1023, 101, 99};
    TEST_ASSERT_EQUAL_UINT16(101, median_of_5(s));
}

void test_median_of_5_two_spikes(void) {
    // Two outlier spikes — median still correct
    uint16_t s[5] = {100, 1023, 102, 1023, 101};
    TEST_ASSERT_EQUAL_UINT16(102, median_of_5(s));
}

void test_median_of_5_all_same(void) {
    uint16_t s[5] = {42, 42, 42, 42, 42};
    TEST_ASSERT_EQUAL_UINT16(42, median_of_5(s));
}

void test_median_of_5_all_zero(void) {
    uint16_t s[5] = {0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL_UINT16(0, median_of_5(s));
}

void test_work_zone_inside_heading_down(void) {
    // Position is 4000 steps from endpoint DOWN (5000), heading to DOWN
    // Work zone is 5500 steps — so 4000 < 5500 → in work zone
    TEST_ASSERT_TRUE(sg_in_work_zone(46000, 50000, 5500, 50000));
}

void test_work_zone_outside_heading_down(void) {
    // Position is 10000 steps from endpoint DOWN — outside work zone
    TEST_ASSERT_FALSE(sg_in_work_zone(40000, 50000, 5500, 50000));
}

void test_work_zone_heading_up(void) {
    // Even if near DOWN endpoint, heading UP means no work zone blanking
    TEST_ASSERT_FALSE(sg_in_work_zone(46000, 50000, 5500, 0));
}

void test_work_zone_zero_size(void) {
    // Work zone of 0 means no blanking ever
    TEST_ASSERT_FALSE(sg_in_work_zone(49999, 50000, 0, 50000));
}

void test_accel_window_typical(void) {
    // 40kHz speed, 800k accel → 40000*1000/800000 = 50ms + 80ms margin = 130ms
    TEST_ASSERT_EQUAL_UINT32(130, compute_accel_window_ms(40000, 800000));
}

void test_accel_window_slow(void) {
    // 30kHz speed, 800k accel → 30000*1000/800000 = 37ms + 80ms = 117ms
    TEST_ASSERT_EQUAL_UINT32(117, compute_accel_window_ms(30000, 800000));
}

void test_accel_window_custom_margin(void) {
    TEST_ASSERT_EQUAL_UINT32(150, compute_accel_window_ms(40000, 800000, 100));
}

void test_decel_distance_typical(void) {
    // 40kHz speed, 800k accel → 40000^2 / (2*800000) = 1600000000/1600000 = 1000 steps
    TEST_ASSERT_EQUAL_INT32(1000, compute_decel_distance(40000, 800000));
}

void test_decel_distance_slow(void) {
    // 30kHz → 900000000/1600000 = 562 steps
    TEST_ASSERT_EQUAL_INT32(562, compute_decel_distance(30000, 800000));
}

void test_decel_distance_fast(void) {
    // 50kHz → 2500000000/1600000 = 1562 steps
    TEST_ASSERT_EQUAL_INT32(1562, compute_decel_distance(50000, 800000));
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_median_of_5_sorted);
    RUN_TEST(test_median_of_5_reversed);
    RUN_TEST(test_median_of_5_one_spike);
    RUN_TEST(test_median_of_5_two_spikes);
    RUN_TEST(test_median_of_5_all_same);
    RUN_TEST(test_median_of_5_all_zero);
    RUN_TEST(test_work_zone_inside_heading_down);
    RUN_TEST(test_work_zone_outside_heading_down);
    RUN_TEST(test_work_zone_heading_up);
    RUN_TEST(test_work_zone_zero_size);
    RUN_TEST(test_accel_window_typical);
    RUN_TEST(test_accel_window_slow);
    RUN_TEST(test_accel_window_custom_margin);
    RUN_TEST(test_decel_distance_typical);
    RUN_TEST(test_decel_distance_slow);
    RUN_TEST(test_decel_distance_fast);
    return UNITY_END();
}
