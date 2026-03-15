#pragma once
// ============================================================================
//  state_json.h — Pure state JSON serialization (testable)
// ============================================================================

#include <cstdint>
#include <cstdio>

struct ProfileSnapshot {
    const char* name;
    uint32_t speed_hz;
    uint16_t sg_trip;
};

struct StateSnapshot {
    const char* version;
    const char* state;          // "IDLE", "RUNNING", etc.
    long counter;
    uint32_t speed_hz;
    bool calibrated;
    long rawUp, rawDown, endpointUp, endpointDown;
    long upOffset, downOffset;
    long position;
    uint16_t sgTrip;
    long workZone;
    uint16_t currentMa;
    uint8_t profileIdx;
    const char* profileName;
    ProfileSnapshot profiles[3];
    const char* wifiStatus;
    const char* wifiSSID;
    const char* wifiIP;
    long batchTarget, batchCount;
    bool batchActive;
};

// Serialize state to JSON. Returns number of chars written (excluding null terminator).
// If return value >= bufSize, output was truncated.
static inline int buildStateJSON(const StateSnapshot& s, char* buf, size_t bufSize) {
    return snprintf(buf, bufSize,
        "{\"version\":\"%s\",\"state\":\"%s\",\"counter\":%ld,\"speed\":%lu,\"calibrated\":%s,"
        "\"rawUp\":%ld,\"rawDown\":%ld,\"endpointUp\":%ld,\"endpointDown\":%ld,"
        "\"upOffset\":%ld,\"downOffset\":%ld,\"position\":%ld,\"sgTrip\":%u,"
        "\"workZone\":%ld,\"currentMa\":%u,"
        "\"profileIdx\":%u,\"profileName\":\"%s\","
        "\"profiles\":[{\"name\":\"%s\",\"hz\":%lu,\"sg\":%u},"
        "{\"name\":\"%s\",\"hz\":%lu,\"sg\":%u},"
        "{\"name\":\"%s\",\"hz\":%lu,\"sg\":%u}],"
        "\"wifiStatus\":\"%s\",\"wifiSSID\":\"%s\",\"wifiIP\":\"%s\","
        "\"batchTarget\":%ld,\"batchCount\":%ld,\"batchActive\":%s}",
        s.version,
        s.state,
        s.counter, (unsigned long)s.speed_hz, s.calibrated ? "true" : "false",
        s.rawUp, s.rawDown, s.endpointUp, s.endpointDown,
        s.upOffset, s.downOffset,
        s.position,
        s.sgTrip,
        s.workZone, s.currentMa,
        s.profileIdx, s.profileName,
        s.profiles[0].name, (unsigned long)s.profiles[0].speed_hz, s.profiles[0].sg_trip,
        s.profiles[1].name, (unsigned long)s.profiles[1].speed_hz, s.profiles[1].sg_trip,
        s.profiles[2].name, (unsigned long)s.profiles[2].speed_hz, s.profiles[2].sg_trip,
        s.wifiStatus, s.wifiSSID, s.wifiIP,
        s.batchTarget, s.batchCount,
        s.batchActive ? "true" : "false");
}
