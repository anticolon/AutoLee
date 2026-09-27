// ============================================================================
//  AutoLee v1.32
//
//  Changes vs v1.10:
//   - FIX: runtime StallGuard comparison was inverted vs calibration/homing.
//          A jam is now a LOW SG_RESULT (sg < trip), matching the TMC5160.
//   - FIX: a failed creep-home no longer leaves the machine "calibrated"
//          with a bogus position reference.
//   - FIX: calibration and creep-home can no longer be entered re-entrantly.
//   - FIX: long blocking work moved out of LVGL event callbacks into loop().
//   - FIX: batch state is cleared consistently on every stop path.
//   - FIX: web handlers no longer mutate motion state from the AsyncTCP task.
//   - NEW: settings persistence (NVS), WiFi supervision/reconnect.
//   - Dead code removed (return_home_up_safe, unused constants).
//
//  v1.12:
//   - FIX: switching speed profile mid-run caused an instant false jam.
//          speed_hz and sg_trip are tuned as a pair, but only sg_trip took
//          effect immediately (FastAccelStepper applies a new speed on the
//          next move command). A profile switch while RUNNING is now queued
//          and applied atomically at the next direction change.
//
//  v1.13:
//   - NEW: SGT (StallGuard threshold) is runtime-adjustable and persisted,
//          instead of being a compile-time constant.
//   - NEW: relative stall detection — each stroke measures its own no-load
//          baseline and trips at a percentage of it. The per-profile
//          absolute trip remains as a backstop.
//
//  v1.14:
//   - NEW: WiFi can be switched on/off from the WiFi screen (and the web
//          panel). Persisted; defaults to OFF on a first install.
//
//  v1.15:
//   - FIX: LVGL ran out of its 48 KB heap while building the UI, which
//          halts in LV_ASSERT_HANDLER (while(1);) with logging disabled —
//          a silent hang with a black screen and no WiFi. Pool raised to
//          80 KB, LVGL logging enabled, and boot now reports headroom.
//
//  v1.16:
//   - FIX: SG was not read or logged at all when both detectors were
//          disabled, which made it impossible to measure raw SG values
//          while tuning (the one time you most need them).
//   - NEW: per-stroke minimum SG is logged at each direction change — the
//          number that actually decides where the thresholds belong.
//
//  v1.18:
//   - REVERTED: runtime SGT, Cal SGT and the relative (baseline) detector
//          are gone, along with the SG Tuning screen and its web controls.
//          Raising SGT does not recover resolution at high step rates —
//          StallGuard2 is out of velocity range there — so the extra layers
//          added complexity without buying anything. Detection is back to a
//          single absolute trip per speed profile.
//   - KEPT: the per-stroke minimum SG log line and SG logging while
//          detection is disabled, which are what make those trips tunable.
//
//  v1.19:
//   - FIX: the per-stroke summary only printed at a direction change, so a
//          run stopped mid-stroke never reported its minimum — exactly the
//          stroke you care about after a jam. Now also printed on stop and
//          on a jam, and every new minimum is logged as it happens.
//   - NEW: StallGuard filter (sfilt) enabled to cut the reading noise.
//
//  v1.20:
//   - FIX: REVERTED the v1.11 stall polarity change. On this machine
//          SG_RESULT RISES under load at every speed, so a jam is
//          sg > trip, exactly as v1.8 had it. The datasheet-derived
//          sg < trip only ever worked on the Slow profile.
//   - sfilt back off (it averages away the ~2 ms load spike).
//   - Per-stroke MAX is now tracked and logged alongside min; the trip is
//          set just above max, the way v1.8's working values were.
//
//  v1.21:
//   - Default SG trips set to the values confirmed working on the machine:
//          Slow 373, Normal 80, Fast 20.
//
//  v1.22:
//   - FIX: turning WiFi back on left the web panel unreachable. The async
//          web server's listener does not reliably re-bind after the socket
//          has been closed and the interface taken down. Enabling WiFi now
//          reboots, which is the only guaranteed-clean path. Disabling is
//          unchanged and still immediate.
//
//  v1.23:
//   - NEW: Auto SG calibration. Runs all three speed profiles with
//          detection disabled, records the highest SG per profile and sets
//          each trip just above it. Margin scales with the signal — see the
//          note in config.h for why a flat offset breaks the Fast profile.
//
//  v1.24:
//   - Web: dropped the hard-coded "known-good" SG values (they are specific
//          to one supply voltage and motor), and the Auto SG empty-press
//          warning is now red.
//
//  v1.25:
//   - SG trips now default to 0 (detection off) and the press will not run
//          until Auto SG has been completed once, or the trips have been set
//          by hand. Persisted, so it is a one-time gate per machine.
//
//  v1.26:
//   - FIX: Auto SG could never run on a machine that needed it — it starts
//          its strokes through startRunBetweenEndpoints(), which the v1.25
//          lockout blocked. Auto SG is now exempt from the gate.
//   - FIX: upgrading from v1.24/v1.25 locked the press even with good
//          trips stored (no "sgcal" key, or sgcal=false saved by v1.25).
//          All three trips > 0 now always counts as set up.
//   - FIX: the web STOP button was disabled during a first Auto SG, and the
//          panel froze for the whole measurement. STOP stays live while
//          RUNNING and state is broadcast from inside the Auto SG loops.
//   - FIX: setting ONE trip by hand unlocked all three profiles. The gate
//          now needs all three trips > 0, and a run started on a profile
//          whose trip is 0 logs a warning.
//   - FIX: stale "SG_RESULT falls with load / sg < trip" comments removed
//          from config.h and handleMotion() — they contradicted the code.
//   - FIX: a tap in the brief IDLE between two Auto SG profiles could start
//          a run, a calibration or a second Auto SG. autoSGActive now spans
//          the whole routine and those handlers check it.
//   - FIX: Auto SG strokes no longer add to the lifetime counter.
//   - NEW: the run current Auto SG measured at is stored; changing current
//          (or the work zone) afterwards logs a reminder to re-run Auto SG.
//   - Web SG inputs now say 0-1023 (the real range) instead of 0-500.
//
//  v1.27:
//   - FIX: the jam-detection gate is now derived from the trips alone. A
//          stored sgcal=true (e.g. from v1.25 after ONE hand-set trip) no
//          longer unlocks the press, and setting a trip back to 0 re-locks.
//   - FIX: reboot requests set the timestamp before the flag, so loop()
//          can't restart immediately and cut off the OTA "OK" response.
//  v1.28:
//   - FIX: TBL set to 2, and TOFF to 5
//  v1.29:
//   - FIX: Auto SG aborted on machines where SG_RESULT sits at the floor
//          (0/1) at speed — e.g. 24 V on Normal/Fast. Floor readings are
//          now counted instead of silently dropped; a profile that read
//          only floor is measured as max=1 (trip = 1 + margin) and flagged
//          for a block test. Abort now only when nothing was sampled.
//   - NEW: per-stroke log line reports floor readings ("floor=N").
//  v1.30:
//   - CHANGE: profile speeds 20/30/40 kHz (were 15/35/45).
//   - NEW: each profile's speed is saved to NVS with its trip. At boot, a
//          stored trip whose speed differs from the compiled speed (or was
//          saved before v1.30, with no speed at all) is cleared, which locks
//          the press until Auto SG is rerun. Trips are only valid as a pair
//          with their speed.
//  v1.31:
//   - NEW: floor warning. A profile whose trip is at the SG floor level
//          (Auto SG read only 0/1, trip <= SG_FLOOR_TRIP_MAX) is flagged:
//          main-screen pill "JAM DETECT LIMITED" when it is active, a warning
//          symbol on its Speed button and info card, a banner and button
//          markers on the web page, and a log line when a run starts on it.
//   - NEW: Settings > Reset Count gives visible feedback (button turns
//          green and reads "Reset!" for ~1.2 s) and logs the reset.
//  v1.32:
//   - CHANGE: Slow back to 15 kHz (Normal 30, Fast 40 unchanged). The
//          speed/trip check clears a Slow trip measured at 20 kHz on boot.
// ============================================================================

#include <lvgl.h>
#include "esp_lcd_touch_axs5106l.h"
#include <Arduino_GFX_Library.h>
#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Update.h>
#include <DNSServer.h>

#include "config.h"

// ==========================================================================
//  GLOBAL DEFINITIONS (shared by all modules — single translation unit,
//  headers are included below after these definitions)
// ==========================================================================
TMC5160Stepper driver(TMC_CS, R_SENSE);
FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

static long rawUp = 0, rawDown = 0;
static long endpointUp = 0, endpointDown = 0;
static bool endpointsCalibrated = false;
static long counter = 0;

enum RunState : uint8_t { IDLE, RUNNING, STOPPING, CALIBRATING, STALLED, HOMING };
static volatile RunState runState = IDLE;
static long     currentTarget = 0;
static uint32_t stopEntryMs   = 0;

static uint32_t lastDirectionChangeMs = 0;
// Queued speed-profile switch. profiles[].speed_hz and profiles[].sg_trip are
// only valid as a pair — SG_RESULT is strongly velocity dependent — so a
// profile change requested mid-run is held here and applied at the next
// direction change, where the speed actually changes too. -1 = nothing queued.
static volatile int8_t pendingProfile = -1;

static uint8_t  runSGHighCount = 0;    // consecutive loaded (high SG) readings
static uint8_t  runSGLowCount  = 0;    // consecutive healthy readings

// Extremes of SG seen during the monitored part of the current stroke, both
// logged at each direction change. MAX is the one that sets a profile's trip
// (a jam is an upward spike); MIN is kept because a stall also dips the
// reading at low speed and it costs nothing to watch.
// 0xFFFF / 0 = nothing sampled yet this stroke.
static uint16_t runStrokeMinSG = 0xFFFF;
static uint16_t runStrokeMaxSG = 0;
// Readings at the SG floor (0/1) this stroke. These are excluded from
// min/max and from detection, but counted so a stroke that read ONLY floor
// is distinguishable from a stroke that was never sampled.
static uint16_t runStrokeFloorCnt = 0;

// Auto SG calibration state. autoSGActive is NOT a RunState: handleMotion()
// must keep running normally during the measurement, so the machine stays in
// RUNNING and this flag rides alongside it.
static volatile bool autoSGRequested = false;
static volatile bool autoSGActive    = false;
static bool     autoSGFailed  = false;
// True while every profile has a trip > 0 (via Auto SG or set by hand). The
// press is locked out otherwise, so no profile runs with jam detection off
// (the shipped defaults are all zero). Always derived from the trips with
// allSgTripsSet() — never set it directly.
static bool     sgCalibrated  = false;
static inline bool allSgTripsSet() {
  for (uint8_t i = 0; i < NUM_PROFILES; i++)
    if (profiles[i].sg_trip == 0) return false;
  return true;
}
// True when a profile's trip is at the SG-floor level (see SG_FLOOR_TRIP_MAX):
// jam detection on it depends only on stall spikes and may not catch a jam.
static inline bool profileSgAtFloor(uint8_t i) {
  return i < NUM_PROFILES && profiles[i].sg_trip > 0
         && profiles[i].sg_trip <= SG_FLOOR_TRIP_MAX;
}
// Run current the trips were measured at by Auto SG (0 = unknown / set by
// hand). SG_RESULT scales with coil current. Persisted.
static uint16_t sgCalCurrentMa = 0;
static uint16_t autoSGMax     = 0;
static uint16_t autoSGStrokes = 0;
static uint32_t autoSGFloor   = 0;   // floor readings over the counted strokes

// Master WiFi switch. OFF means: no STA, no AP, no DNS, no web server and
// no OTA — the machine runs entirely from the touch screen. Persisted;
// defaults to OFF on a first install (see loadSettings).
static bool wifiEnabled = false;
static bool wifiServicesRunning = false;

static bool wifiConnected = false;
static bool wifiAPMode = false;
static String wifiSSID = "", wifiPass = "";
static String scannedOptionsHTML = "";
DNSServer dnsServer;
static bool captivePortalRunning = false;

Preferences prefs;
AsyncWebServer webServer(80);
AsyncEventSource events("/events");
static uint32_t lastSSEMs = 0;

// ==========================================================================
//  DEFERRED REQUESTS
//  Async web handlers run in the AsyncTCP task; LVGL event callbacks run
//  nested inside lv_timer_handler(). Neither may touch SPI (TMC5160), the
//  stepper, LVGL or the log buffer directly, and neither may block. Both
//  therefore raise a flag here and loop() does the work — see
//  handleWebRequests().
// ==========================================================================
static volatile bool     calRequested            = false;  // touch UI + web
static volatile bool     homeRequested           = false;  // touch UI + web
static bool              calFailed               = false;  // for the Calibrate button label
static volatile bool     webToggleRunRequested   = false;
static volatile bool     webStopRequested        = false;  // stop-only (used by web OTA upload)
static volatile bool     webBatchStartRequested  = false;
static volatile bool     webBatchClearRequested  = false;
static volatile int32_t  webBatchDelta           = 0;
static volatile bool     webLogClearRequested    = false;
static volatile int8_t   webProfileRequested     = -1;     // -1 = none
static volatile int32_t  webCurrentMaRequested   = -1;     // -1 = none
static volatile int32_t  webUpOffsetDelta        = 0;
static volatile int32_t  webDownOffsetDelta      = 0;
static volatile int32_t  webWorkZoneDelta        = 0;
static volatile int8_t   wifiEnableRequested     = -1;    // -1 none, 0 off, 1 on

static volatile int8_t   webSgProfile            = -1;     // -1 = none pending
static volatile int32_t  webSgAbsolute           = -1;     // -1 = use webSgDelta
static volatile int32_t  webSgDelta              = 0;
static volatile bool     webWifiSaveRequested    = false;
static volatile bool     webWifiClearRequested   = false;
static String            pendingWifiSSID = "", pendingWifiPass = "";
static volatile bool     rebootRequested = false;
static volatile uint32_t rebootRequestMs = 0;

static int32_t  batchTarget  = 0;
static int32_t  batchCount   = 0;
static bool     batchActive  = false;

static char logBuf[LOG_LINES][LOG_LINE_LEN];
static uint16_t logHead = 0;
static uint32_t logSerial = 0;
static uint32_t logSentSerial = 0;

Arduino_DataBus *bus = new Arduino_HWSPI(LCD_DC, LCD_CS, SPI_SCK, SPI_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, false, 172, 320, 34, 0, 34, 0);

// LVGL
static uint32_t bufSize = 0;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf = nullptr;
static lv_disp_drv_t disp_drv;

static lv_obj_t *main_scr = nullptr, *settings_scr = nullptr, *config_scr = nullptr, *profile_scr = nullptr;
static lv_obj_t *tuning_scr = nullptr, *ep_up_scr = nullptr, *ep_dn_scr = nullptr;
static lv_obj_t *wifi_scr = nullptr;
static lv_obj_t *counter_label = nullptr, *main_warn = nullptr, *main_warn_lbl = nullptr;
static lv_obj_t *lbl_speed_val = nullptr;
static lv_obj_t *profile_btns[NUM_PROFILES] = {};
static lv_obj_t *lbl_profile_info = nullptr;
static lv_obj_t *lbl_ep_up = nullptr, *lbl_ep_dn = nullptr, *lbl_travel = nullptr;
static lv_obj_t *lbl_up_eff = nullptr, *lbl_dn_eff = nullptr;
static lv_obj_t *lbl_ep_up_val = nullptr, *lbl_ep_dn_val = nullptr;
static lv_obj_t *lbl_wifi_status = nullptr;
static lv_obj_t *btn_wifi_toggle = nullptr;
static lv_obj_t *btn_run_global = nullptr;
static lv_obj_t *btn_cal = nullptr;
static lv_obj_t *btn_autosg = nullptr;

static lv_obj_t *jam_scr = nullptr;
static lv_obj_t *jam_status_lbl = nullptr;
static lv_obj_t *stall_scr = nullptr;
static lv_obj_t *lbl_sg_val = nullptr;


static lv_obj_t *batch_scr = nullptr;
static lv_obj_t *lbl_batch_val = nullptr;
static lv_obj_t *lbl_batch_remain = nullptr;

// ==========================================================================
//  webLog — used by all modules.
//  MUST only be called from loop() context (it writes the shared ring
//  buffer that broadcastState() reads).
// ==========================================================================
void webLog(const char *fmt, ...) {
  char line[LOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  Serial.println(line);
  strncpy(logBuf[logHead], line, LOG_LINE_LEN - 1);
  logBuf[logHead][LOG_LINE_LEN - 1] = '\0';
  logHead = (logHead + 1) % LOG_LINES;
  logSerial++;
}

// ==========================================================================
//  SETTINGS PERSISTENCE (NVS)
//  Calibration results are intentionally NOT stored — see config.h.
// ==========================================================================
static bool     settingsDirty   = false;
static uint32_t settingsDirtyMs = 0;

void markSettingsDirty() {
  settingsDirty = true;
  settingsDirtyMs = millis();
}

void saveSettings() {
  prefs.begin("autolee", false);
  prefs.putUChar ("prof", activeProfile);
  prefs.putUShort("sg0",  profiles[0].sg_trip);
  prefs.putUShort("sg1",  profiles[1].sg_trip);
  prefs.putUShort("sg2",  profiles[2].sg_trip);
  // Speed each trip belongs to — checked against the compiled speeds at boot.
  prefs.putULong ("spd0", profiles[0].speed_hz);
  prefs.putULong ("spd1", profiles[1].speed_hz);
  prefs.putULong ("spd2", profiles[2].speed_hz);
  prefs.putUShort("cur",  RUN_CURRENT_MA);
  prefs.putInt   ("wz",   SG_WORK_ZONE_STEPS);
  prefs.putBool  ("wifien", wifiEnabled);
  prefs.putBool  ("sgcal",  sgCalibrated);
  prefs.putUShort("sgcalma", sgCalCurrentMa);

  prefs.putInt   ("upo",  upOffsetSteps);
  prefs.putInt   ("dno",  downOffsetSteps);
  prefs.putInt   ("btgt", batchTarget);
  prefs.putLong  ("cnt",  counter);
  prefs.end();
  settingsDirty = false;
}

void loadSettings() {
  prefs.begin("autolee", true);
  uint8_t p = prefs.getUChar("prof", activeProfile);
  activeProfile = (p < NUM_PROFILES) ? p : 1;
  // NOTE: Arduino's constrain() is a macro that evaluates its first argument
  // up to three times — always read into a temporary first.
  for (uint8_t i = 0; i < NUM_PROFILES; i++) {
    char key[8];
    snprintf(key, sizeof(key), "sg%u", (unsigned)i);
    int32_t v = (int32_t)prefs.getUShort(key, profiles[i].sg_trip);
    profiles[i].sg_trip = (uint16_t)constrain(v, (int32_t)RUN_SG_TRIP_MIN, (int32_t)RUN_SG_TRIP_MAX);
  }
  // A trip is only valid at the speed it was measured at. Clear any stored
  // trip whose saved speed doesn't match the compiled one — including trips
  // saved before v1.30, which have no speed key. A cleared trip re-locks the
  // press (allSgTripsSet() below) until Auto SG is rerun.
  bool tripsCleared = false;
  for (uint8_t i = 0; i < NUM_PROFILES; i++) {
    if (profiles[i].sg_trip == 0) continue;
    char key[8];
    snprintf(key, sizeof(key), "spd%u", (unsigned)i);
    const uint32_t stored = prefs.isKey(key) ? prefs.getULong(key, 0) : 0;
    if (stored != profiles[i].speed_hz) {
      webLog("NVS: %s trip %u was measured at %lu Hz, profile is now %lu Hz — trip cleared, rerun Auto SG",
             profiles[i].name, profiles[i].sg_trip,
             (unsigned long)stored, (unsigned long)profiles[i].speed_hz);
      profiles[i].sg_trip = 0;
      tripsCleared = true;
    }
  }
  int32_t cur = (int32_t)prefs.getUShort("cur", RUN_CURRENT_MA);
  RUN_CURRENT_MA = (uint16_t)constrain(cur, (int32_t)RUN_CURRENT_MIN, (int32_t)RUN_CURRENT_MAX);

  // Jam-detection gate, from the trips alone. The stored "sgcal" flag is
  // ignored: v1.24 had none, v1.25 could save false next to good trips or
  // true with only one trip set, and v1.26 never cleared it when a trip went
  // back to 0. Must come after the trips load.
  sgCalibrated = allSgTripsSet();
  sgCalCurrentMa = prefs.isKey("sgcalma") ? prefs.getUShort("sgcalma", 0) : 0;

  // WiFi master switch. A first install (no settings stored at all) comes up
  // with WiFi OFF. A device that already has settings keeps WiFi ON by
  // default, so updating firmware over OTA doesn't strand you without a web
  // panel — after that the stored value always wins.
  if (prefs.isKey("wifien"))     wifiEnabled = prefs.getBool("wifien", false);
  else if (prefs.isKey("prof"))  wifiEnabled = true;    // pre-existing install
  else                           wifiEnabled = false;   // first install

  int32_t wz  = prefs.getInt("wz",   SG_WORK_ZONE_STEPS);
  int32_t upo = prefs.getInt("upo",  0);
  int32_t dno = prefs.getInt("dno",  DOWN_OFFSET_DEFAULT);
  int32_t btg = prefs.getInt("btgt", 0);
  SG_WORK_ZONE_STEPS = constrain(wz,  SG_WORK_ZONE_MIN, SG_WORK_ZONE_MAX);
  upOffsetSteps      = constrain(upo, OFFSET_MIN, OFFSET_MAX);
  downOffsetSteps    = constrain(dno, OFFSET_MIN, OFFSET_MAX);
  batchTarget        = constrain(btg, (int32_t)0, (int32_t)9999);
  counter            = prefs.getLong("cnt", 0);
  if (counter < 0) counter = 0;
  prefs.end();
  // Persist the cleared trips + current speeds (flushed from loop() when idle).
  if (tripsCleared) markSettingsDirty();
}

// Flush only while idle and after a quiet period, so a run never writes flash.
void handleSettingsPersist() {
  if (!settingsDirty) return;
  if (runState != IDLE) return;
  if ((millis() - settingsDirtyMs) < SETTINGS_SAVE_DEBOUNCE_MS) return;
  saveSettings();
  webLog("Settings saved to NVS");
}

// ==========================================================================
//  INCLUDE MODULES (order matters: motion before ui_touch before web_server)
//  Forward declarations resolve circular dependencies between motion and UI.
// ==========================================================================

// UI functions called by motion.h (defined in ui_touch.h)
static void go(lv_obj_t *scr);
void setRunButtonState(bool running);
void ui_update_main_warning();
void ui_update_tuning_numbers();
void ui_update_endpoint_edit_values();
// Needed because setActiveProfile() (ui_touch.h) calls it before its definition
void ui_update_sg_val();
// Called by autoCalibrateSG() in motion.h, defined in ui_touch.h
void ui_update_speed_val();
void ui_update_profile_screen();
// Applies a queued profile switch (defined in ui_touch.h, called by motion.h)
void applyPendingProfile();
// Called from the Auto SG loops in motion.h (defined in web_server.h)
void broadcastState();

#include "motion.h"

// WiFi functions called by ui_touch.h (defined in wifi_ota.h)
void clearWiFiCredentials();
void setWifiEnabled(bool en);

#include "ui_touch.h"
#include "wifi_ota.h"
#include "web_server.h"

// ==========================================================================
//  SETUP
// ==========================================================================
void setup() {
  Serial.begin(115200);
  Serial.printf("=== AutoLee v%s ===\n", FW_VERSION);

  // Restore user settings before anything consumes them (driver current,
  // active profile, endpoint offsets, counter).
  loadSettings();

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, TMC_CS);
  pinMode(LCD_CS, OUTPUT); digitalWrite(LCD_CS, HIGH);
  pinMode(TMC_CS, OUTPUT); digitalWrite(TMC_CS, HIGH);

  if (!gfx->begin()) Serial.println("gfx->begin() failed!");
  lcd_reg_init();
  gfx->setRotation(ROTATION);
  gfx->fillScreen(RGB565_BLACK);
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);

  Wire.begin(Touch_I2C_SDA, Touch_I2C_SCL);
  bsp_touch_init(&Wire, Touch_RST, Touch_INT, gfx->getRotation(), gfx->width(), gfx->height());

  lv_init();
#if LV_USE_BIDI
  lv_bidi_set_base_dir(LV_BASE_DIR_LTR);
#endif

  bufSize = (uint32_t)gfx->width() * 40;
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!disp_draw_buf) disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_8BIT);
  if (!disp_draw_buf) { Serial.println("LVGL buf alloc failed!"); for(;;) delay(1000); }

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, nullptr, bufSize);
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = gfx->width();
  disp_drv.ver_res = gfx->height();
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read_cb;
  lv_indev_drv_register(&indev_drv);

  // TMC5160
  driver.begin();
  driver.setSPISpeed(4000000);
  driver.toff(5);           // chopper off-time 5 
  driver.tbl(2);            // TBL=2
                            // tbl() writes the raw CHOPCONF.TBL field (0..3).
                            // Do NOT swap this for blank_time(), which takes
                            // clock counts (16/24/36/54) instead.
  driver.rms_current(RUN_CURRENT_MA);
  driver.microsteps(16);
  driver.intpol(true);  // MicroPlyer: interpolate 16 usteps -> 256 internally (smoother, no torque cost)
  driver.en_pwm_mode(false);
  driver.TPWMTHRS(0);
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(0); driver.semax(0); driver.seup(0); driver.sedn(0);
  driver.sgt(SGT_VALUE);
  driver.sfilt(SG_FILTER);
  // DIAG1 is wired to GPIO7 and configured as a push-pull stall output, but
  // stall detection is currently polled over SPI (read_sg). The pin config is
  // kept so GPIO7 is actively driven (not floating) and reserved for a future
  // hardware-interrupt implementation.
  driver.diag1_stall(true);
  driver.diag1_index(false);
  driver.diag1_onstate(false);
  driver.diag1_steps_skipped(false);
  driver.diag1_pushpull(true);
  pinMode(TMC_DIAG_PIN, INPUT);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) { Serial.println("Stepper fail!"); while(true) delay(1000); }
  stepper->setDirectionPin(DIR_PIN, false);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);
  // Hold the coils for 1 s after a move finishes. Without this the driver is
  // de-energised the instant a stroke ends (and during the post-jam back-off),
  // so a spring-loaded ram can back-drive and the position reference is lost.
  stepper->setDelayToDisable(1000);
  stepper->setSpeedInHz(ui_speed_hz);
  stepper->setAcceleration(RUN_DECEL);

  // Build the LVGL touch UI
  buildUI();

  // Report LVGL heap headroom. If the pool is ever exhausted while building
  // screens, LV_ASSERT_MALLOC halts the CPU before this line is reached — so
  // seeing this print at all means the UI was built successfully.
  // "total" also confirms WHICH lv_conf.h the build actually used: it must
  // read 81920, not 49152.
  {
    lv_mem_monitor_t mon;
    lv_mem_monitor(&mon);
    Serial.printf("LVGL heap: %u/%u used (%u%%), largest free block %u, frag %u%%\n",
                  (unsigned)(mon.total_size - mon.free_size), (unsigned)mon.total_size,
                  (unsigned)mon.used_pct, (unsigned)mon.free_biggest_size,
                  (unsigned)mon.frag_pct);
    Serial.printf("Free internal RAM: %u bytes (min ever %u)\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
  }

  // WiFi + Web + OTA.
  // Handlers and OTA callbacks are registered unconditionally; the radio and
  // the listening sockets only come up when WiFi is switched on.
  setupWebServer();
  setupArduinoOTA();
  if (wifiEnabled) {
    startWiFi();
    wifiStartServices();
  } else {
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi: disabled (enable it from the WiFi screen)");
  }
  ui_update_wifi_label();

  Serial.println("Setup complete!");
}

// ==========================================================================
//  LOOP
// ==========================================================================
void loop() {
  lv_timer_handler();
  handleMotion();
  handleWebRequests();
  handleWiFi();
  handleSettingsPersist();
  if (wifiEnabled) {
    ArduinoOTA.handle();
    broadcastState();
    // Process captive portal DNS requests
    if (captivePortalRunning) {
      dnsServer.processNextRequest();
    }
  }

  // Deferred reboot (safe from main loop context)
  if (rebootRequested && (millis() - rebootRequestMs) > 500) {
    if (stepper && stepper->isRunning()) stepper->forceStop();
    if (settingsDirty) saveSettings();   // don't lose the counter on reboot
    delay(100);
    ESP.restart();
  }

  delay(1);
}
