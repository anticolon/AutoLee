#pragma once
// ============================================================================
//  log_ring.h — Fixed-size ring buffer for log lines (testable)
// ============================================================================

#include <cstdint>
#include <cstring>

// Ring buffer for log lines. Stores up to LINES entries, each up to LINE_LEN chars.
// Tracks write serial and sent serial for incremental streaming.
template<uint16_t LINES, uint16_t LINE_LEN>
struct LogRing {
    char buf[LINES][LINE_LEN];
    uint16_t head = 0;       // next write index (wraps at LINES)
    uint32_t serial = 0;     // monotonic total lines written
    uint32_t sentSerial = 0; // last serial sent to client

    void clear() {
        head = 0;
        serial = 0;
        sentSerial = 0;
        memset(buf, 0, sizeof(buf));
    }

    void append(const char *line) {
        strncpy(buf[head], line, LINE_LEN - 1);
        buf[head][LINE_LEN - 1] = '\0';
        head = (head + 1) % LINES;
        serial++;
    }

    // How many lines are pending (not yet sent to client)
    uint32_t pending() const {
        if (serial <= sentSerial) return 0;
        uint32_t p = serial - sentSerial;
        if (p > LINES) p = LINES;  // can't exceed ring buffer capacity
        return p;
    }

    // Compute the start index in the ring buffer for `count` most recent entries.
    // count must be <= LINES.
    uint16_t startIndex(uint32_t count) const {
        if (count > LINES) count = LINES;
        return (head + LINES - (uint16_t)count) % LINES;
    }

    // Get line at ring buffer index
    const char* lineAt(uint16_t idx) const {
        return buf[idx % LINES];
    }

    // Mark `count` lines as sent
    void markSent(uint32_t count) {
        sentSerial += count;
        if (sentSerial > serial) sentSerial = serial;
    }
};
