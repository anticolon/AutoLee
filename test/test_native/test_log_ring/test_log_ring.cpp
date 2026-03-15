#include <unity.h>
#include "log_ring.h"

// Use small buffer for testing (easier to reason about wrapping)
using TestLog = LogRing<4, 32>;

static TestLog log_ring;

void setUp(void) {
    log_ring.clear();
}

void test_log_empty(void) {
    TEST_ASSERT_EQUAL_UINT32(0, log_ring.pending());
    TEST_ASSERT_EQUAL_UINT32(0, log_ring.serial);
    TEST_ASSERT_EQUAL_UINT16(0, log_ring.head);
}

void test_log_append_one(void) {
    log_ring.append("hello");
    TEST_ASSERT_EQUAL_UINT32(1, log_ring.pending());
    TEST_ASSERT_EQUAL_UINT32(1, log_ring.serial);
    TEST_ASSERT_EQUAL_UINT16(1, log_ring.head);
    TEST_ASSERT_EQUAL_STRING("hello", log_ring.lineAt(0));
}

void test_log_append_multiple(void) {
    log_ring.append("line1");
    log_ring.append("line2");
    log_ring.append("line3");
    TEST_ASSERT_EQUAL_UINT32(3, log_ring.pending());
    TEST_ASSERT_EQUAL_STRING("line1", log_ring.lineAt(0));
    TEST_ASSERT_EQUAL_STRING("line2", log_ring.lineAt(1));
    TEST_ASSERT_EQUAL_STRING("line3", log_ring.lineAt(2));
}

void test_log_wrap_around(void) {
    // Buffer has 4 slots. Write 6 lines — first 2 should be overwritten.
    log_ring.append("a");
    log_ring.append("b");
    log_ring.append("c");
    log_ring.append("d");
    log_ring.append("e");  // overwrites slot 0 ("a")
    log_ring.append("f");  // overwrites slot 1 ("b")

    TEST_ASSERT_EQUAL_UINT32(6, log_ring.serial);
    TEST_ASSERT_EQUAL_UINT16(2, log_ring.head);  // wraps: 6 % 4 = 2

    // pending is capped at LINES (4), not 6
    TEST_ASSERT_EQUAL_UINT32(4, log_ring.pending());

    // Ring buffer now contains: [e, f, c, d] at indices [0,1,2,3]
    TEST_ASSERT_EQUAL_STRING("e", log_ring.lineAt(0));
    TEST_ASSERT_EQUAL_STRING("f", log_ring.lineAt(1));
    TEST_ASSERT_EQUAL_STRING("c", log_ring.lineAt(2));
    TEST_ASSERT_EQUAL_STRING("d", log_ring.lineAt(3));
}

void test_log_sent_tracking(void) {
    log_ring.append("x");
    log_ring.append("y");
    log_ring.append("z");
    TEST_ASSERT_EQUAL_UINT32(3, log_ring.pending());

    log_ring.markSent(2);
    TEST_ASSERT_EQUAL_UINT32(1, log_ring.pending());
    TEST_ASSERT_EQUAL_UINT32(2, log_ring.sentSerial);
}

void test_log_mark_sent_clamp(void) {
    log_ring.append("a");
    log_ring.markSent(999);  // can't exceed serial
    TEST_ASSERT_EQUAL_UINT32(1, log_ring.sentSerial);  // clamped to serial
    TEST_ASSERT_EQUAL_UINT32(0, log_ring.pending());
}

void test_log_clear(void) {
    log_ring.append("a");
    log_ring.append("b");
    log_ring.markSent(1);
    log_ring.clear();
    TEST_ASSERT_EQUAL_UINT32(0, log_ring.pending());
    TEST_ASSERT_EQUAL_UINT32(0, log_ring.serial);
    TEST_ASSERT_EQUAL_UINT32(0, log_ring.sentSerial);
    TEST_ASSERT_EQUAL_UINT16(0, log_ring.head);
    TEST_ASSERT_EQUAL_STRING("", log_ring.lineAt(0));
}

void test_log_truncation(void) {
    // LINE_LEN is 32, so a long string should be truncated
    log_ring.append("this is a very long line that exceeds the buffer size limit");
    const char *stored = log_ring.lineAt(0);
    TEST_ASSERT_EQUAL_UINT32(1, log_ring.pending());
    // Should be truncated to 31 chars + null
    TEST_ASSERT_TRUE(strlen(stored) <= 31);
    // First 31 chars should match
    TEST_ASSERT_EQUAL_INT(0, strncmp(stored, "this is a very long line that e", 31));
}

void test_log_start_index(void) {
    log_ring.append("a");
    log_ring.append("b");
    log_ring.append("c");
    // head is at 3, want last 2 lines -> start at index 1
    TEST_ASSERT_EQUAL_UINT16(1, log_ring.startIndex(2));
    TEST_ASSERT_EQUAL_STRING("b", log_ring.lineAt(log_ring.startIndex(2)));
}

void test_log_start_index_wrapped(void) {
    // Fill and wrap
    log_ring.append("a");
    log_ring.append("b");
    log_ring.append("c");
    log_ring.append("d");
    log_ring.append("e");  // head now at 1
    // Want last 3 lines: d, e — but 3 most recent are c, d, e
    // head=1, startIndex(3) = (1+4-3)%4 = 2
    TEST_ASSERT_EQUAL_UINT16(2, log_ring.startIndex(3));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_log_empty);
    RUN_TEST(test_log_append_one);
    RUN_TEST(test_log_append_multiple);
    RUN_TEST(test_log_wrap_around);
    RUN_TEST(test_log_sent_tracking);
    RUN_TEST(test_log_mark_sent_clamp);
    RUN_TEST(test_log_clear);
    RUN_TEST(test_log_truncation);
    RUN_TEST(test_log_start_index);
    RUN_TEST(test_log_start_index_wrapped);
    return UNITY_END();
}
