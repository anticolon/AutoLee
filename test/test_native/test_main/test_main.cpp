#include <unity.h>
#include "config.h"
#include "theme.h"

void test_config_constants(void) {
    TEST_ASSERT_EQUAL(172, SCR_W);
    TEST_ASSERT_EQUAL(320, SCR_H);
    TEST_ASSERT_EQUAL(3, NUM_PROFILES);
}

void test_theme_constants(void) {
    TEST_ASSERT_EQUAL_HEX32(0x000000, Theme::BG);
    TEST_ASSERT_EQUAL_HEX32(0x1F6FEB, Theme::ACCENT);
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFF, Theme::TEXT);
    TEST_ASSERT_EQUAL_HEX32(0x00FF00, Theme::GREEN);
    TEST_ASSERT_EQUAL_HEX32(0xFF4444, Theme::RED);
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_config_constants);
    RUN_TEST(test_theme_constants);
    return UNITY_END();
}
