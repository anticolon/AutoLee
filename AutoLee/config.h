// ============================================================================
//  AutoLee – config.h
//  Pin definitions, speed profiles, tuning constants
// ============================================================================
#pragma once

#define FW_VERSION "1.8"

static const char *DEFAULT_AP_SSID = "AutoLee-Setup";

// ==========================================================================
//  PIN DEFINITIONS
// ==========================================================================
#define ENABLE_PIN   4
#define STEP_PIN     5
#define DIR_PIN      6
#define TMC_DIAG_PIN 7
#define TMC_CS       8
#define R_SENSE      0.022f

#define ROTATION     0
#define GFX_BL       23
#define Touch_I2C_SDA 18
#define Touch_I2C_SCL 19
#define Touch_RST     20
#define Touch_INT     21

// ==========================================================================
//  SPEED PROFILES
// ==========================================================================
struct SpeedProfile {
  const char *name;
  uint32_t speed_hz;
  uint16_t sg_trip_default;  // factory default SG for this speed
  uint16_t sg_trip;          // current (user-tweakable) SG trip
};

static constexpr uint8_t NUM_PROFILES = 3;
static SpeedProfile profiles[NUM_PROFILES] = {
  { "Slow",   15000, 350, 350 },
  { "Normal", 35000, 15, 15 },
  { "Fast",   45000, 1, 1 },
};
static uint8_t activeProfile = 1;  // default to Normal

static constexpr uint32_t RUN_DECEL    = 800000;  // accel/decel rate for run moves (fast ramps, max SG coverage)

// Accessors — use these everywhere instead of raw globals
#define ui_speed_hz  (profiles[activeProfile].speed_hz)
#define RUN_SG_TRIP  (profiles[activeProfile].sg_trip)

// ==========================================================================
//  ENDPOINT TUNING
// ==========================================================================
static int32_t upOffsetSteps   = 0;
static int32_t downOffsetSteps = 0;
static constexpr int32_t DOWN_OFFSET_DEFAULT = -500;
static constexpr int32_t OFFSET_MIN     = -8000;
static constexpr int32_t OFFSET_MAX     = +8000;
static constexpr int32_t ENDPOINT_GUARD = 50;

static constexpr int32_t CAL_PREMOVE_DOWN_STEPS = 5500;

// ==========================================================================
//  CALIBRATION CONSTANTS
// ==========================================================================
static constexpr int8_t   CAL_SGT          = -1;
static uint16_t           RUN_CURRENT_MA   = 3500;
static constexpr uint16_t RUN_CURRENT_MIN  = 1000;
static constexpr uint16_t RUN_CURRENT_MAX  = 4500;
static constexpr uint16_t CAL_CURRENT_MA   = 3200;
static constexpr uint32_t CAL_SPEED_HZ     = 8000;
static constexpr uint32_t CAL_ACCEL        = 25000;
static constexpr int32_t  CAL_SEARCH_STEPS = 120000;
static constexpr uint16_t CAL_ABS_MIN      = 12;
static constexpr uint8_t  CAL_REL_DROP_Q8  = 235;
static constexpr uint8_t  CAL_HIT_CONFIRM  = 2;

static constexpr uint32_t EARLY_WINDOW_MS      = 300;
static constexpr int32_t  EARLY_WINDOW_DST_MAX = 1200;
static constexpr uint32_t EARLY_MIN_TIME_MS    = 50;
static constexpr int32_t  EARLY_MIN_MOVE_STEPS = 200;
static constexpr uint16_t EARLY_TRIP           = CAL_ABS_MIN;

// Return-home: USE SAME SPEED as calibration to avoid decel overshoot
static constexpr uint32_t HOME_SPEED_HZ      = CAL_SPEED_HZ;
static constexpr uint32_t HOME_ACCEL          = CAL_ACCEL;
static constexpr uint16_t HOME_SG_TRIP        = 15;
static constexpr uint8_t  HOME_CONFIRM        = 3;
static constexpr uint32_t HOME_MIN_MS         = 200;
static constexpr int32_t  HOME_MIN_MOVE       = 600;
static constexpr int32_t  HOME_RELEASE_STEPS  = 1200;
static constexpr uint8_t  HOME_MAX_RETRIES    = 2;
static constexpr uint32_t HOME_TIMEOUT_MS     = 15000;

// ==========================================================================
//  RUNTIME STALL DETECTION
// ==========================================================================
static constexpr uint16_t RUN_SG_TRIP_MIN    = 0;
static constexpr uint16_t RUN_SG_TRIP_MAX    = 500;
static constexpr int32_t  RUN_BACKOFF_STEPS  = 1000; // steps to back off after jam
static constexpr uint8_t  RUN_SG_HIGH_NEEDED = 2;    // need this many high readings to trigger jam

// Work zone: skip SG monitoring near the DOWN endpoint where the tool
// does useful work (e.g. pushing primers). The resistance here is normal
// and would false-trigger stall detection at low trip thresholds.
// SG is still active for the rest of the travel and near the UP endpoint.
static int32_t SG_WORK_ZONE_STEPS = 5500;  // skip SG this many steps before endpointDown
static constexpr int32_t SG_WORK_ZONE_MIN = 0;
static constexpr int32_t SG_WORK_ZONE_MAX = 20000;
static constexpr uint32_t CREEP_HOME_SPEED   = CAL_SPEED_HZ;
static constexpr uint32_t CREEP_HOME_ACCEL   = CAL_ACCEL;
static constexpr uint16_t CREEP_SG_TRIP      = 15;
static constexpr uint8_t  CREEP_SG_CONFIRM   = 3;
static constexpr uint32_t CREEP_IGNORE_MS    = 300;
static constexpr uint32_t CREEP_TIMEOUT_MS   = 20000;

// ==========================================================================
//  DISPLAY / LAYOUT
// ==========================================================================
static constexpr int SCR_W = 172, SCR_H = 320, NAV_H = 60, CONTENT_H = SCR_H - NAV_H;

// ==========================================================================
//  LOG RING BUFFER
// ==========================================================================
static constexpr uint16_t LOG_LINES = 500;
static constexpr uint16_t LOG_LINE_LEN = 140;

// ==========================================================================
//  SSE / BROADCAST
// ==========================================================================
static constexpr uint32_t SSE_INTERVAL_MS = 250;

// ==========================================================================
//  STOP TIMEOUT
// ==========================================================================
static constexpr uint32_t STOP_TIMEOUT_MS = 8000;
