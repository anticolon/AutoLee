// ============================================================================
//  AutoLee – config.h
//  Pin definitions, speed profiles, tuning constants
// ============================================================================
#pragma once

#define FW_VERSION "1.32"

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

// Shared hardware SPI bus: TMC5160 + ST7789 share SCK/MOSI, each has its
// own chip select. MISO is only used by the TMC (the display is write-only).
#define SPI_SCK      1
#define SPI_MOSI     2
#define SPI_MISO     3

// ST7789 display
#define LCD_DC       15
#define LCD_CS       14
#define LCD_RST      22
#define ROTATION     0
#define GFX_BL       23

// Capacitive touch (AXS5106L)
#define Touch_I2C_SDA 18
#define Touch_I2C_SCL 19
#define Touch_RST     20
#define Touch_INT     21

// ==========================================================================
//  SPEED PROFILES
//
//  sg_trip is a StallGuard2 threshold in SG_RESULT units, one per speed
//  profile — SG_RESULT is strongly velocity dependent, so speed_hz and
//  sg_trip are only valid as a pair.
//
//      jam  <=>  sg > sg_trip      (LOWER trip = MORE sensitive, 0 = off)
//
//  See the POLARITY note below before changing anything here.
// ==========================================================================
struct SpeedProfile {
  const char *name;
  uint32_t speed_hz;
  uint16_t sg_trip;          // current (user-tweakable) SG trip
};

static constexpr uint8_t NUM_PROFILES = 3;
// Trip defaults are 0 = detection off. The correct values depend entirely on
// the individual machine — supply voltage, motor, driver current, mechanics —
// so there is no sensible number to ship. Run Auto SG once and it measures
// and writes all three; the press is locked out until that has been done.
//
// POLARITY: on this machine SG_RESULT RISES under load, so a jam is
//   sg > trip   ->  lower trip = MORE sensitive, 0 = off
//
// That is the opposite of the TMC5160 datasheet's "SG_RESULT falls with
// load". v1.11-v1.19 followed the datasheet and broke jam detection on the
// Normal and Fast profiles; logged evidence beats the datasheet here. During
// a stall the reading is erratic and swings BOTH ways — at 15 kHz it dips as
// well as spikes, but the upward spike is the only signature present at all
// three speeds, so it is the one detection keys on.
//
// SPEEDS: 15/30/40 kHz since v1.32 (v1.30-1.31: 20/30/40; before: 15/35/45). Measurements quoted in
// comments elsewhere in this file were taken at the old speeds. The speeds
// are saved to NVS next to the trips; if they differ at boot, the stored
// trips are cleared and the press stays locked until Auto SG is rerun.
static SpeedProfile profiles[NUM_PROFILES] = {
  { "Slow",   15000, 0 },
  { "Normal", 30000, 0 },
  { "Fast",   40000, 0 },
};
static uint8_t activeProfile = 1;  // default to Normal

static constexpr uint32_t RUN_DECEL    = 800000;  // accel/decel rate for run moves (fast ramps, max SG coverage)

// Accessors — use these everywhere instead of raw globals.
// NOTE: RUN_SG_TRIP is deliberately assignable (it expands to an lvalue) so
// the touch UI can nudge the active profile's trip point in place.
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
// StallGuard2 threshold (COOLCONF.SGT), shared by the run detector,
// calibration and homing. It offsets the whole SG_RESULT measurement window.
// Fixed again in v1.18: raising it does not recover usable resolution at high
// step rates — measured Fast (45 kHz) readings stay around 30 no matter what
// SGT is set to, because StallGuard2 itself runs out of velocity range there.
// Adjust here and reflash if you ever need to move it.
static constexpr int8_t   SGT_VALUE        = -1;

// StallGuard filter (COOLCONF.sfilt). Left OFF, as it was in v1.8.
// It averages SG_RESULT over four full steps — about 1.8 ms at 35 kHz — and
// the load spikes this machine detects on last only ~2 ms (70 steps at
// 35 kHz in the v1.8 log). Filtering would average the signal away.
static constexpr bool     SG_FILTER        = false;
static uint16_t           RUN_CURRENT_MA   = 3500;
static constexpr uint16_t RUN_CURRENT_MIN  = 1000;
static constexpr uint16_t RUN_CURRENT_MAX  = 4500;
static constexpr uint16_t CAL_CURRENT_MA   = 3200;
static constexpr uint32_t CAL_SPEED_HZ     = 8000;
static constexpr uint32_t CAL_ACCEL        = 25000;
static constexpr int32_t  CAL_SEARCH_STEPS = 120000;
static constexpr uint16_t CAL_ABS_MIN      = 12;
// Relative trip as Q8 fraction of the measured no-load baseline.
// 235/256 = 92%, i.e. an 8% drop in SG_RESULT is treated as "hit the stop".
static constexpr uint8_t  CAL_REL_DROP_Q8  = 235;
static constexpr uint8_t  CAL_HIT_CONFIRM  = 2;

// Absolute-threshold ("early") stall detector. It is armed once the move has
// travelled a little, and stays armed until the dynamic baseline detector
// takes over — this closes the blind window that existed in v1.10 between
// the end of the old fixed 300 ms window and the baseline arming at ~620 ms.
static constexpr uint32_t EARLY_MIN_TIME_MS    = 50;
static constexpr int32_t  EARLY_MIN_MOVE_STEPS = 200;
static constexpr uint16_t EARLY_TRIP           = CAL_ABS_MIN;

// ==========================================================================
//  RUNTIME STALL DETECTION
// ==========================================================================
static constexpr uint16_t RUN_SG_TRIP_MIN    = 0;
static constexpr uint16_t RUN_SG_TRIP_MAX    = 1023;  // full SG_RESULT range
static constexpr int32_t  RUN_BACKOFF_STEPS  = 1000; // steps to back off after jam
static constexpr uint8_t  RUN_SG_TRIP_NEEDED = 2;    // consecutive HIGH readings (sg > trip) needed to declare a jam

// ==========================================================================
//  AUTO SG CALIBRATION
//  Runs each speed profile with detection disabled, records the highest
//  SG_RESULT seen, and sets that profile's trip just above it.
//
//  MARGIN: exactly +1, and it has to be. The gap between the highest clean
//  reading and the reading during a real jam is only a few counts, and it
//  gets narrower as speed rises. A percentage margin, or even a flat +2, can
//  land the trip above the jam level itself on the faster profiles — setting
//  a threshold that can never fire. Measured max plus one is the only margin
//  that holds across the whole speed range.
//
//  Because the margin is one count, the measurement has to be thorough: miss
//  one high stroke and the trip lands inside the noise. That direction is
//  the safe one — it produces false jams, which are obvious and recoverable,
//  rather than missed jams. The measured max is logged either way so it can
//  be checked and nudged by hand afterwards.
//
//  The measurement is only valid for the run current it was taken at —
//  SG_RESULT scales with coil current. The current used is stored with the
//  trips, and changing it afterwards logs a reminder to re-run Auto SG.
// ==========================================================================
static constexpr uint8_t  SG_AUTO_STROKES    = 12;     // measured strokes per profile
static constexpr uint16_t SG_AUTO_MARGIN     = 1;      // trip = measured max + this
// A trip at or below this means Auto SG found SG_RESULT at its floor (0/1)
// for that profile (clean max recorded as 1). Detection then relies only on a
// stall spiking off the floor, so the UI warns that it is limited.
static constexpr uint16_t SG_FLOOR_TRIP_MAX  = 1 + SG_AUTO_MARGIN;
static constexpr uint32_t SG_AUTO_TIMEOUT_MS = 120000; // per profile

// Work zone: skip SG monitoring near the DOWN endpoint where the tool
// does useful work (e.g. pushing primers). The resistance here is normal
// and would false-trigger stall detection.
// SG is still active for the rest of the travel and near the UP endpoint.
static int32_t SG_WORK_ZONE_STEPS = 5500;  // skip SG this many steps before endpointDown
static constexpr int32_t SG_WORK_ZONE_MIN = 0;
static constexpr int32_t SG_WORK_ZONE_MAX = 20000;

// Speed/accel used for the post-jam back-off and the creep-home move.
static constexpr uint32_t CREEP_HOME_SPEED   = CAL_SPEED_HZ;
static constexpr uint32_t CREEP_HOME_ACCEL   = CAL_ACCEL;

// ==========================================================================
//  DISPLAY / LAYOUT
// ==========================================================================
static constexpr int SCR_W = 172, SCR_H = 320, NAV_H = 60, CONTENT_H = SCR_H - NAV_H;

// ==========================================================================
//  LOG RING BUFFER
// ==========================================================================
// 300 x 140 = 42 KB of static RAM (was 500 = 70 KB). The 28 KB freed here
// pays for the larger LVGL heap in lv_conf.h.
static constexpr uint16_t LOG_LINES = 300;
static constexpr uint16_t LOG_LINE_LEN = 140;

// ==========================================================================
//  SSE / BROADCAST
// ==========================================================================
static constexpr uint32_t SSE_INTERVAL_MS = 250;

// ==========================================================================
//  STOP TIMEOUT
// ==========================================================================
static constexpr uint32_t STOP_TIMEOUT_MS = 8000;

// ==========================================================================
//  SETTINGS PERSISTENCE
//  Settings are flushed to NVS only while IDLE and after a quiet period, so
//  a continuous run never writes flash. Calibration itself is deliberately
//  NOT persisted: the stepper position reference is lost across a reboot,
//  so stored endpoints would be meaningless (and dangerous) afterwards.
// ==========================================================================
static constexpr uint32_t SETTINGS_SAVE_DEBOUNCE_MS = 3000;

// ==========================================================================
//  WiFi SUPERVISION
// ==========================================================================
static constexpr uint32_t WIFI_CHECK_MS   = 2000;
static constexpr uint32_t WIFI_RETRY_MS   = 15000;
