#include <unity.h>
#include "motor_fsm.h"

static MotorFSM fsm;

void setUp(void) {
    fsm = MotorFSM{};  // reset to IDLE
}

void tearDown(void) {}

// --- Initial state ---
void test_initial_state(void) {
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
    TEST_ASSERT_EQUAL_UINT32(0, fsm.stateEnteredMs);
}

// --- Valid transitions ---
void test_idle_to_running(void) {
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::START, 100));
    TEST_ASSERT_EQUAL(MotorState::RUNNING, fsm.state);
    TEST_ASSERT_EQUAL_UINT32(100, fsm.stateEnteredMs);
}

void test_idle_to_calibrating(void) {
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::CALIBRATE, 200));
    TEST_ASSERT_EQUAL(MotorState::CALIBRATING, fsm.state);
}

void test_running_to_stopping(void) {
    fsm.transition(MotorEvent::START, 100);
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::STOP, 200));
    TEST_ASSERT_EQUAL(MotorState::STOPPING, fsm.state);
}

void test_running_to_stalled(void) {
    fsm.transition(MotorEvent::START, 100);
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::JAM, 300));
    TEST_ASSERT_EQUAL(MotorState::STALLED, fsm.state);
}

void test_stopping_to_idle_reached(void) {
    fsm.transition(MotorEvent::START, 100);
    fsm.transition(MotorEvent::STOP, 200);
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::REACHED_TARGET, 300));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

void test_stopping_to_idle_timeout(void) {
    fsm.transition(MotorEvent::START, 100);
    fsm.transition(MotorEvent::STOP, 200);
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::TIMEOUT, 9000));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

void test_calibrating_done(void) {
    fsm.transition(MotorEvent::CALIBRATE, 100);
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::DONE, 5000));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

void test_calibrating_fail(void) {
    fsm.transition(MotorEvent::CALIBRATE, 100);
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::FAIL, 3000));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

void test_stalled_to_homing(void) {
    fsm.transition(MotorEvent::START, 100);
    fsm.transition(MotorEvent::JAM, 200);
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::RETURN_HOME, 300));
    TEST_ASSERT_EQUAL(MotorState::HOMING, fsm.state);
}

void test_homing_done(void) {
    fsm.transition(MotorEvent::START, 100);
    fsm.transition(MotorEvent::JAM, 200);
    fsm.transition(MotorEvent::RETURN_HOME, 300);
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::DONE, 400));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

void test_homing_fail(void) {
    fsm.transition(MotorEvent::START, 100);
    fsm.transition(MotorEvent::JAM, 200);
    fsm.transition(MotorEvent::RETURN_HOME, 300);
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::FAIL, 400));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

// --- Invalid transitions (state unchanged) ---
void test_invalid_start_from_running(void) {
    fsm.transition(MotorEvent::START, 100);
    TEST_ASSERT_FALSE(fsm.transition(MotorEvent::START, 200));
    TEST_ASSERT_EQUAL(MotorState::RUNNING, fsm.state);
    TEST_ASSERT_EQUAL_UINT32(100, fsm.stateEnteredMs);  // unchanged
}

void test_invalid_start_from_calibrating(void) {
    fsm.transition(MotorEvent::CALIBRATE, 100);
    TEST_ASSERT_FALSE(fsm.transition(MotorEvent::START, 200));
    TEST_ASSERT_EQUAL(MotorState::CALIBRATING, fsm.state);
}

void test_invalid_calibrate_from_running(void) {
    fsm.transition(MotorEvent::START, 100);
    TEST_ASSERT_FALSE(fsm.transition(MotorEvent::CALIBRATE, 200));
    TEST_ASSERT_EQUAL(MotorState::RUNNING, fsm.state);
}

void test_invalid_stop_from_idle(void) {
    TEST_ASSERT_FALSE(fsm.transition(MotorEvent::STOP, 100));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

void test_invalid_jam_from_idle(void) {
    TEST_ASSERT_FALSE(fsm.transition(MotorEvent::JAM, 100));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

void test_invalid_return_home_from_idle(void) {
    TEST_ASSERT_FALSE(fsm.transition(MotorEvent::RETURN_HOME, 100));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

void test_invalid_return_home_from_running(void) {
    fsm.transition(MotorEvent::START, 100);
    TEST_ASSERT_FALSE(fsm.transition(MotorEvent::RETURN_HOME, 200));
    TEST_ASSERT_EQUAL(MotorState::RUNNING, fsm.state);
}

// --- Query methods ---
void test_can_run(void) {
    TEST_ASSERT_TRUE(fsm.canRun());
    fsm.transition(MotorEvent::START, 100);
    TEST_ASSERT_FALSE(fsm.canRun());
}

void test_can_calibrate(void) {
    TEST_ASSERT_TRUE(fsm.canCalibrate());
    fsm.transition(MotorEvent::START, 100);
    TEST_ASSERT_FALSE(fsm.canCalibrate());
}

void test_can_return_home(void) {
    TEST_ASSERT_FALSE(fsm.canReturnHome());
    fsm.transition(MotorEvent::START, 100);
    fsm.transition(MotorEvent::JAM, 200);
    TEST_ASSERT_TRUE(fsm.canReturnHome());
}

void test_is_moving(void) {
    TEST_ASSERT_FALSE(fsm.isMoving());  // IDLE

    fsm.transition(MotorEvent::START, 100);
    TEST_ASSERT_TRUE(fsm.isMoving());   // RUNNING

    fsm.transition(MotorEvent::STOP, 200);
    TEST_ASSERT_TRUE(fsm.isMoving());   // STOPPING

    fsm.transition(MotorEvent::REACHED_TARGET, 300);
    TEST_ASSERT_FALSE(fsm.isMoving());  // IDLE

    fsm.transition(MotorEvent::CALIBRATE, 400);
    TEST_ASSERT_TRUE(fsm.isMoving());   // CALIBRATING
}

void test_is_moving_stalled(void) {
    fsm.transition(MotorEvent::START, 100);
    fsm.transition(MotorEvent::JAM, 200);
    TEST_ASSERT_FALSE(fsm.isMoving());  // STALLED -- not moving

    fsm.transition(MotorEvent::RETURN_HOME, 300);
    TEST_ASSERT_TRUE(fsm.isMoving());   // HOMING -- moving
}

void test_elapsed(void) {
    fsm.transition(MotorEvent::START, 1000);
    TEST_ASSERT_EQUAL_UINT32(500, fsm.elapsed(1500));
    TEST_ASSERT_EQUAL_UINT32(2000, fsm.elapsed(3000));
}

// --- State names ---
void test_state_names(void) {
    TEST_ASSERT_EQUAL_STRING("IDLE", fsm.stateName());

    fsm.transition(MotorEvent::START, 0);
    TEST_ASSERT_EQUAL_STRING("RUNNING", fsm.stateName());

    fsm.transition(MotorEvent::STOP, 0);
    TEST_ASSERT_EQUAL_STRING("STOPPING", fsm.stateName());

    fsm = MotorFSM{};
    fsm.transition(MotorEvent::CALIBRATE, 0);
    TEST_ASSERT_EQUAL_STRING("CALIBRATING", fsm.stateName());

    fsm = MotorFSM{};
    fsm.transition(MotorEvent::START, 0);
    fsm.transition(MotorEvent::JAM, 0);
    TEST_ASSERT_EQUAL_STRING("STALLED", fsm.stateName());

    fsm.transition(MotorEvent::RETURN_HOME, 0);
    TEST_ASSERT_EQUAL_STRING("HOMING", fsm.stateName());
}

// --- Full lifecycle ---
void test_full_run_cycle(void) {
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::START, 0));
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::STOP, 1000));
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::REACHED_TARGET, 1500));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

void test_full_jam_recovery_cycle(void) {
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::START, 0));
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::JAM, 500));
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::RETURN_HOME, 1000));
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::DONE, 2000));
    TEST_ASSERT_EQUAL(MotorState::IDLE, fsm.state);
}

void test_full_calibration_cycle(void) {
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::CALIBRATE, 0));
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::DONE, 5000));
    TEST_ASSERT_TRUE(fsm.transition(MotorEvent::START, 5100));
    TEST_ASSERT_EQUAL(MotorState::RUNNING, fsm.state);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    // Initial state
    RUN_TEST(test_initial_state);
    // Valid transitions
    RUN_TEST(test_idle_to_running);
    RUN_TEST(test_idle_to_calibrating);
    RUN_TEST(test_running_to_stopping);
    RUN_TEST(test_running_to_stalled);
    RUN_TEST(test_stopping_to_idle_reached);
    RUN_TEST(test_stopping_to_idle_timeout);
    RUN_TEST(test_calibrating_done);
    RUN_TEST(test_calibrating_fail);
    RUN_TEST(test_stalled_to_homing);
    RUN_TEST(test_homing_done);
    RUN_TEST(test_homing_fail);
    // Invalid transitions
    RUN_TEST(test_invalid_start_from_running);
    RUN_TEST(test_invalid_start_from_calibrating);
    RUN_TEST(test_invalid_calibrate_from_running);
    RUN_TEST(test_invalid_stop_from_idle);
    RUN_TEST(test_invalid_jam_from_idle);
    RUN_TEST(test_invalid_return_home_from_idle);
    RUN_TEST(test_invalid_return_home_from_running);
    // Query methods
    RUN_TEST(test_can_run);
    RUN_TEST(test_can_calibrate);
    RUN_TEST(test_can_return_home);
    RUN_TEST(test_is_moving);
    RUN_TEST(test_is_moving_stalled);
    RUN_TEST(test_elapsed);
    // State names
    RUN_TEST(test_state_names);
    // Full lifecycle
    RUN_TEST(test_full_run_cycle);
    RUN_TEST(test_full_jam_recovery_cycle);
    RUN_TEST(test_full_calibration_cycle);
    return UNITY_END();
}
