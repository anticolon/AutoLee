#pragma once
// ============================================================================
//  motor_fsm.h — Motor state machine with typed transitions (testable)
// ============================================================================

#include <cstdint>

enum class MotorState : uint8_t {
    IDLE,
    RUNNING,
    STOPPING,
    CALIBRATING,
    STALLED,
    HOMING
};

enum class MotorEvent : uint8_t {
    START,          // User presses RUN
    STOP,           // User presses STOP or batch complete
    JAM,            // Runtime stall detected
    CALIBRATE,      // User requests calibration
    RETURN_HOME,    // User presses Return Home from jam screen
    REACHED_TARGET, // Motor reached its target position
    TIMEOUT,        // Stop or home timeout expired
    DONE,           // Calibration or homing completed successfully
    FAIL            // Calibration or homing failed
};

struct MotorFSM {
    MotorState state = MotorState::IDLE;
    uint32_t stateEnteredMs = 0;

    // Attempt a state transition. Returns true if valid, false if rejected.
    // On success, updates state and stateEnteredMs.
    bool transition(MotorEvent event, uint32_t nowMs = 0) {
        MotorState next = state;

        switch (state) {
            case MotorState::IDLE:
                if (event == MotorEvent::START)     next = MotorState::RUNNING;
                else if (event == MotorEvent::CALIBRATE) next = MotorState::CALIBRATING;
                break;

            case MotorState::RUNNING:
                if (event == MotorEvent::STOP)      next = MotorState::STOPPING;
                else if (event == MotorEvent::JAM)  next = MotorState::STALLED;
                break;

            case MotorState::STOPPING:
                if (event == MotorEvent::REACHED_TARGET) next = MotorState::IDLE;
                else if (event == MotorEvent::TIMEOUT)   next = MotorState::IDLE;
                break;

            case MotorState::CALIBRATING:
                if (event == MotorEvent::DONE) next = MotorState::IDLE;
                else if (event == MotorEvent::FAIL) next = MotorState::IDLE;
                break;

            case MotorState::STALLED:
                if (event == MotorEvent::RETURN_HOME) next = MotorState::HOMING;
                break;

            case MotorState::HOMING:
                if (event == MotorEvent::DONE) next = MotorState::IDLE;
                else if (event == MotorEvent::FAIL) next = MotorState::IDLE;
                break;
        }

        if (next == state) return false;  // no transition occurred

        state = next;
        stateEnteredMs = nowMs;
        return true;
    }

    bool canRun() const       { return state == MotorState::IDLE; }
    bool canCalibrate() const { return state == MotorState::IDLE; }
    bool canReturnHome() const { return state == MotorState::STALLED; }

    bool isMoving() const {
        return state == MotorState::RUNNING ||
               state == MotorState::STOPPING ||
               state == MotorState::CALIBRATING ||
               state == MotorState::HOMING;
    }

    // Time elapsed in current state
    uint32_t elapsed(uint32_t nowMs) const {
        return nowMs - stateEnteredMs;
    }

    const char* stateName() const {
        switch (state) {
            case MotorState::IDLE:        return "IDLE";
            case MotorState::RUNNING:     return "RUNNING";
            case MotorState::STOPPING:    return "STOPPING";
            case MotorState::CALIBRATING: return "CALIBRATING";
            case MotorState::STALLED:     return "STALLED";
            case MotorState::HOMING:      return "HOMING";
            default:                      return "UNKNOWN";
        }
    }
};
