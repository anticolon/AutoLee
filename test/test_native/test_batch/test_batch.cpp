#include <unity.h>
#include "batch.h"

static BatchState batch;

void setUp(void) {
    batch.reset();
}

void test_batch_initial_state(void) {
    TEST_ASSERT_EQUAL_INT32(0, batch.target);
    TEST_ASSERT_EQUAL_INT32(0, batch.count);
    TEST_ASSERT_FALSE(batch.active);
}

void test_batch_adjust_target_positive(void) {
    batch.adjustTarget(10);
    TEST_ASSERT_EQUAL_INT32(10, batch.target);
    batch.adjustTarget(5);
    TEST_ASSERT_EQUAL_INT32(15, batch.target);
}

void test_batch_adjust_target_negative(void) {
    batch.target = 50;
    batch.adjustTarget(-20);
    TEST_ASSERT_EQUAL_INT32(30, batch.target);
}

void test_batch_adjust_target_clamp_zero(void) {
    batch.target = 5;
    batch.adjustTarget(-100);
    TEST_ASSERT_EQUAL_INT32(0, batch.target);
}

void test_batch_adjust_target_clamp_max(void) {
    batch.adjustTarget(99999);
    TEST_ASSERT_EQUAL_INT32(9999, batch.target);
}

void test_batch_adjust_target_custom_max(void) {
    batch.adjustTarget(500, 200);
    TEST_ASSERT_EQUAL_INT32(200, batch.target);
}

void test_batch_start_success(void) {
    batch.target = 100;
    TEST_ASSERT_TRUE(batch.start());
    TEST_ASSERT_TRUE(batch.active);
    TEST_ASSERT_EQUAL_INT32(0, batch.count);
}

void test_batch_start_fails_zero_target(void) {
    batch.target = 0;
    TEST_ASSERT_FALSE(batch.start());
    TEST_ASSERT_FALSE(batch.active);
}

void test_batch_record_stroke_incomplete(void) {
    batch.target = 5;
    batch.start();
    TEST_ASSERT_FALSE(batch.recordStroke());
    TEST_ASSERT_EQUAL_INT32(1, batch.count);
    TEST_ASSERT_TRUE(batch.active);
}

void test_batch_record_stroke_complete(void) {
    batch.target = 3;
    batch.start();
    TEST_ASSERT_FALSE(batch.recordStroke());  // 1/3
    TEST_ASSERT_FALSE(batch.recordStroke());  // 2/3
    TEST_ASSERT_TRUE(batch.recordStroke());   // 3/3 — complete!
    TEST_ASSERT_FALSE(batch.active);
    TEST_ASSERT_EQUAL_INT32(3, batch.count);
}

void test_batch_record_stroke_inactive(void) {
    // Not active — stroke doesn't count
    batch.target = 5;
    TEST_ASSERT_FALSE(batch.recordStroke());
    TEST_ASSERT_EQUAL_INT32(0, batch.count);
}

void test_batch_remaining(void) {
    batch.target = 10;
    batch.start();
    batch.recordStroke();
    batch.recordStroke();
    batch.recordStroke();
    TEST_ASSERT_EQUAL_INT32(7, batch.remaining());
}

void test_batch_remaining_not_active(void) {
    batch.target = 10;
    TEST_ASSERT_EQUAL_INT32(0, batch.remaining());
}

void test_batch_remaining_after_complete(void) {
    batch.target = 1;
    batch.start();
    batch.recordStroke();
    TEST_ASSERT_EQUAL_INT32(0, batch.remaining());
}

void test_batch_reset(void) {
    batch.target = 100;
    batch.start();
    batch.recordStroke();
    batch.reset();
    TEST_ASSERT_EQUAL_INT32(0, batch.target);
    TEST_ASSERT_EQUAL_INT32(0, batch.count);
    TEST_ASSERT_FALSE(batch.active);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_batch_initial_state);
    RUN_TEST(test_batch_adjust_target_positive);
    RUN_TEST(test_batch_adjust_target_negative);
    RUN_TEST(test_batch_adjust_target_clamp_zero);
    RUN_TEST(test_batch_adjust_target_clamp_max);
    RUN_TEST(test_batch_adjust_target_custom_max);
    RUN_TEST(test_batch_start_success);
    RUN_TEST(test_batch_start_fails_zero_target);
    RUN_TEST(test_batch_record_stroke_incomplete);
    RUN_TEST(test_batch_record_stroke_complete);
    RUN_TEST(test_batch_record_stroke_inactive);
    RUN_TEST(test_batch_remaining);
    RUN_TEST(test_batch_remaining_not_active);
    RUN_TEST(test_batch_remaining_after_complete);
    RUN_TEST(test_batch_reset);
    return UNITY_END();
}
