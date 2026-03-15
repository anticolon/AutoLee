#pragma once
// ============================================================================
//  config.h — Pin definitions, constants, and speed profile structure
// ============================================================================

#include <cstdint>

// ==========================================================================
//  Pin Definitions
// ==========================================================================

#define ENABLE_PIN   4
#define STEP_PIN     5
#define DIR_PIN      6
#define TMC_DIAG_PIN 7
#define TMC_CS       8
#define R_SENSE      0.022f

// ==========================================================================
//  Display Pins
// ==========================================================================

#define ROTATION     0
#define GFX_BL       23
#define Touch_I2C_SDA 18
#define Touch_I2C_SCL 19
#define Touch_RST     20
#define Touch_INT     21

// ==========================================================================
//  WiFi Defaults
// ==========================================================================

static constexpr const char *DEFAULT_AP_SSID = "AutoLee-Setup";

// ==========================================================================
//  Speed Profiles
// ==========================================================================

struct SpeedProfile {
  const char *name;
  uint32_t speed_hz;
  uint16_t sg_trip_default;  // factory default SG for this speed
  uint16_t sg_trip;          // current (user-tweakable) SG trip
};

static constexpr uint8_t NUM_PROFILES = 3;

// ==========================================================================
//  Motion Constants
// ==========================================================================

static constexpr uint32_t RUN_ACCEL    = 800000;  // acceleration & deceleration rate for all run/stop moves

static constexpr int32_t DOWN_OFFSET_DEFAULT = -500;
static constexpr int32_t OFFSET_MIN     = -8000;
static constexpr int32_t OFFSET_MAX     = +8000;
static constexpr int32_t ENDPOINT_GUARD = 50;

static constexpr int32_t CAL_PREMOVE_DOWN_STEPS = 5500;

// ==========================================================================
//  Calibration Constants
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
static constexpr uint8_t  CAL_REL_DROP_Q8  = 235;  // Q8 fixed-point: 235/256 ~ 92% of SG baseline
static constexpr uint8_t  CAL_HIT_CONFIRM  = 2;

// ==========================================================================
//  Early Window Constants
// ==========================================================================

static constexpr uint32_t EARLY_WINDOW_MS      = 300;
static constexpr int32_t  EARLY_WINDOW_DST_MAX = 1200;
static constexpr uint32_t EARLY_MIN_TIME_MS    = 50;
static constexpr int32_t  EARLY_MIN_MOVE_STEPS = 200;
static constexpr uint16_t EARLY_TRIP           = CAL_ABS_MIN;

// ==========================================================================
//  Home Constants
// ==========================================================================

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
//  Runtime Stall Detection
// ==========================================================================

static constexpr uint16_t RUN_SG_TRIP_MIN    = 0;
static constexpr uint16_t RUN_SG_TRIP_MAX    = 500;
static constexpr int32_t  RUN_BACKOFF_STEPS  = 1000;

static constexpr int32_t SG_WORK_ZONE_MIN = 0;
static constexpr int32_t SG_WORK_ZONE_MAX = 20000;

static constexpr uint32_t CREEP_HOME_SPEED   = CAL_SPEED_HZ;
static constexpr uint32_t CREEP_HOME_ACCEL   = CAL_ACCEL;
static constexpr uint16_t CREEP_SG_TRIP      = 15;
static constexpr uint8_t  CREEP_SG_CONFIRM   = 3;
static constexpr uint32_t CREEP_IGNORE_MS    = 300;
static constexpr uint32_t CREEP_TIMEOUT_MS   = 20000;

static constexpr uint8_t RUN_SG_HIGH_NEEDED = 2;

// ==========================================================================
//  Screen Dimensions
// ==========================================================================

static constexpr int SCR_W = 172, SCR_H = 320, NAV_H = 60, CONTENT_H = SCR_H - NAV_H;

// ==========================================================================
//  Log Buffer Dimensions
// ==========================================================================

static constexpr uint16_t LOG_LINES = 500;
static constexpr uint16_t LOG_LINE_LEN = 140;

// ==========================================================================
//  SSE & Timing
// ==========================================================================

static constexpr uint32_t SSE_INTERVAL_MS = 250;

// ==========================================================================
//  Stop Timeout
// ==========================================================================

static constexpr uint32_t STOP_TIMEOUT_MS = 8000;

