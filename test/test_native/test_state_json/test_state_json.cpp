#include <unity.h>
#include <cstring>
#include "state_json.h"

static char buf[2048];

static StateSnapshot makeDefault() {
    StateSnapshot s = {};
    s.version = "1.6";
    s.state = "IDLE";
    s.profileName = "Normal";
    s.profiles[0] = {"Slow", 15000, 350};
    s.profiles[1] = {"Normal", 35000, 15};
    s.profiles[2] = {"Fast", 45000, 1};
    s.wifiStatus = "Connected";
    s.wifiSSID = "TestNet";
    s.wifiIP = "192.168.1.100";
    return s;
}

void setUp(void) {}
void tearDown(void) {}

// 1. Default state — all zeros/defaults produce valid JSON
void test_default_state_produces_valid_json(void) {
    StateSnapshot s = makeDefault();
    int n = buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_GREATER_THAN(0, n);
    TEST_ASSERT_LESS_THAN((int)sizeof(buf), n);
    // Starts with { and ends with }
    TEST_ASSERT_EQUAL_CHAR('{', buf[0]);
    TEST_ASSERT_EQUAL_CHAR('}', buf[n - 1]);
}

// 2. IDLE state — verify state field is "IDLE"
void test_idle_state(void) {
    StateSnapshot s = makeDefault();
    s.state = "IDLE";
    buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"state\":\"IDLE\""));
}

// 3. RUNNING state — verify state field
void test_running_state(void) {
    StateSnapshot s = makeDefault();
    s.state = "RUNNING";
    buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"state\":\"RUNNING\""));
}

// 4. All fields present — check every key exists
void test_all_fields_present(void) {
    StateSnapshot s = makeDefault();
    s.counter = 42;
    s.speed_hz = 35000;
    s.calibrated = true;
    s.rawUp = 100;
    s.rawDown = 200;
    s.endpointUp = 300;
    s.endpointDown = 400;
    s.upOffset = 10;
    s.downOffset = 20;
    s.position = 500;
    s.sgTrip = 15;
    s.workZone = 5500;
    s.currentMa = 800;
    s.profileIdx = 1;
    s.batchTarget = 50;
    s.batchCount = 10;
    s.batchActive = true;
    buildStateJSON(s, buf, sizeof(buf));

    TEST_ASSERT_NOT_NULL(strstr(buf, "\"version\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"state\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"counter\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"speed\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"calibrated\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"rawUp\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"rawDown\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"endpointUp\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"endpointDown\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"upOffset\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"downOffset\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"position\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"sgTrip\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"workZone\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"currentMa\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"profileIdx\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"profileName\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"profiles\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"wifiStatus\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"wifiSSID\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"wifiIP\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"batchTarget\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"batchCount\":"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"batchActive\":"));
}

// 5. Calibrated true/false — verify boolean serialization
void test_calibrated_true(void) {
    StateSnapshot s = makeDefault();
    s.calibrated = true;
    buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"calibrated\":true"));
}

void test_calibrated_false(void) {
    StateSnapshot s = makeDefault();
    s.calibrated = false;
    buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"calibrated\":false"));
}

// 6. Profile data — verify all three profiles appear with correct hz and sg
void test_profile_data(void) {
    StateSnapshot s = makeDefault();
    buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"name\":\"Slow\",\"hz\":15000,\"sg\":350"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"name\":\"Normal\",\"hz\":35000,\"sg\":15"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"name\":\"Fast\",\"hz\":45000,\"sg\":1"));
}

// 7. WiFi fields — verify wifiStatus, wifiSSID, wifiIP appear
void test_wifi_fields(void) {
    StateSnapshot s = makeDefault();
    s.wifiStatus = "AP Mode";
    s.wifiSSID = "AutoLee-AP";
    s.wifiIP = "192.168.4.1";
    buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"wifiStatus\":\"AP Mode\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"wifiSSID\":\"AutoLee-AP\""));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"wifiIP\":\"192.168.4.1\""));
}

// 8. Batch fields — batchTarget, batchCount, batchActive
void test_batch_fields(void) {
    StateSnapshot s = makeDefault();
    s.batchTarget = 100;
    s.batchCount = 37;
    s.batchActive = true;
    buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"batchTarget\":100"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"batchCount\":37"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"batchActive\":true"));
}

// 9. Negative offsets — verify negative numbers serialize correctly
void test_negative_offsets(void) {
    StateSnapshot s = makeDefault();
    s.upOffset = -500;
    s.downOffset = -1200;
    s.position = -300;
    buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"upOffset\":-500"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"downOffset\":-1200"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"position\":-300"));
}

// 10. Buffer too small — verify return value >= bufSize indicates truncation
void test_buffer_too_small(void) {
    StateSnapshot s = makeDefault();
    char tiny[32];
    int n = buildStateJSON(s, tiny, sizeof(tiny));
    TEST_ASSERT_GREATER_OR_EQUAL((int)sizeof(tiny), n);
}

// 11. Large values — max counter (9999), large positions, verify no overflow
void test_large_values(void) {
    StateSnapshot s = makeDefault();
    s.counter = 9999;
    s.position = 999999;
    s.rawUp = 100000;
    s.rawDown = 200000;
    s.endpointUp = 100000;
    s.endpointDown = 200000;
    s.speed_hz = 50000;
    s.workZone = 99999;
    int n = buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_LESS_THAN((int)sizeof(buf), n);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"counter\":9999"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"position\":999999"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"speed\":50000"));
}

// 12. batchActive true/false — verify boolean formatting
void test_batch_active_true(void) {
    StateSnapshot s = makeDefault();
    s.batchActive = true;
    buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"batchActive\":true"));
}

void test_batch_active_false(void) {
    StateSnapshot s = makeDefault();
    s.batchActive = false;
    buildStateJSON(s, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"batchActive\":false"));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_default_state_produces_valid_json);
    RUN_TEST(test_idle_state);
    RUN_TEST(test_running_state);
    RUN_TEST(test_all_fields_present);
    RUN_TEST(test_calibrated_true);
    RUN_TEST(test_calibrated_false);
    RUN_TEST(test_profile_data);
    RUN_TEST(test_wifi_fields);
    RUN_TEST(test_batch_fields);
    RUN_TEST(test_negative_offsets);
    RUN_TEST(test_buffer_too_small);
    RUN_TEST(test_large_values);
    RUN_TEST(test_batch_active_true);
    RUN_TEST(test_batch_active_false);
    return UNITY_END();
}
