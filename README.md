# AutoLee

**Automated Lee APP conversion — ESP32-C6 firmware with touchscreen UI and web control.**

AutoLee converts a manual Lee APP into a fully automated decapping machine using a stepper motor, sensorless homing, and stallguard jam detection. It runs on a tiny 1.47" touchscreen ESP32-C6 module and can also be controlled from any phone/computer via its built-in web interface.

> **By K.L Design**

---

## ⚠️ SAFETY WARNING — READ BEFORE BUILDING OR OPERATING

**This machine will crush fingers and hands without breaking a sweat.** It is a motorized press driven by a NEMA 23 stepper motor with significant torque. It does not know or care if something is in the way.

- **NEVER run this machine unattended.**
- **NEVER allow children near the machine, whether it is running or not.**
- **Keep your hands and fingers away from the machine at ALL times when it is powered on.**
- **Treat it like any industrial press — it can and will cause serious injury if misused.**

The stall detection and jam protection features are designed to detect brass getting stuck in the machine — nothing more. They will **not** detect or protect your fingers and hands. They were never designed for that. And even for brass jams, they can fail, be misconfigured, or react too slowly. **Do not rely on software to protect your body.**

**LIABILITY DISCLAIMER:** This project is provided as-is with absolutely no warranty of any kind. The author(s) accept no responsibility or liability for any injury, damage, or loss resulting from building, modifying, or operating this machine. You build and use it entirely at your own risk.

---

## Features

### Motion & Calibration
- **Sensorless calibration** — automatically finds the UP and DOWN mechanical stops using TMC5160 StallGuard, no limit switches needed
- **Adjustable endpoints** — fine-tune UP and DOWN positions in ±1/10/100 step increments after calibration
- **Fast ramp profile** — 800,000 steps/s² accel/decel for maximum cruise time and StallGuard coverage
- **Safe return-home** — after a jam or stall, creeps back to UP stop using calibration speed with stall detection and retries

### Speed Profiles (v1.5+)
- **Three preset profiles** — Slow (15 kHz), Normal (35 kHz), Fast (45 kHz)
- **Per-profile StallGuard threshold** — each speed has its own SG trip value, eliminating false jams when changing speed
- **One-tap switching** — change profile from the touchscreen or web UI; speed and SG update together instantly
- **Fine-tune SG per profile** — adjust ±1/±5 from the touch Stall Sensitivity screen or per-profile web controls

### Motor Current (v1.6)
- **Adjustable run current** — set motor current from 1000–4500 mA via touch UI or web interface
- **Independent from calibration current** — calibration always uses a fixed 3200 mA regardless of the run current setting

### Jam Detection & Protection
- **Runtime StallGuard monitoring** — reads SG2 via median-of-5 filtered SPI during operation
- **Sliding counter stall detection** — requires multiple consecutive high-SG readings to trigger (rejects transient spikes)
- **Work zone blanking** — skips SG monitoring near the DOWN endpoint where primer seating resistance is normal
- **Accel/decel blanking** — position-based and time-based SG ignore windows during speed transitions
- **Automatic backoff** — on jam detection, motor stops and backs off in the opposite direction before showing the jam screen
- **Jam recovery screen** — one-button return-home using the same proven sensorless homing as calibration

### Batch Run
- **Set a target count** (1–9999) and the machine stops automatically when done
- **Progress display** — remaining count shown on the main screen during batch operation
- **Works with all profiles** — batch runs at whatever speed profile is active

### Counter
- **Stroke counter** — counts each completed down-up cycle, displayed large on the main screen (max 9999)
- **Resettable** from touch UI or web interface

### Touch UI (172×320 LVGL, multi-page)
- **Main screen** — counter, active speed profile indicator, calibration warning, batch remaining
- **Speed Profile screen** — three large buttons with green highlight on active, info card showing Hz + SG
- **Motor Current screen** — adjust run current with ±100 mA buttons
- **Tuning screen** — raw and effective endpoint values, buttons to edit UP and DOWN endpoints
- **Stall Sensitivity screen** — adjust SG trip for the active profile with ±1/±5 buttons
- **Batch Run screen** — set target count with ±1/10/100 buttons, start batch
- **Settings screen** — access to all sub-screens, calibrate button, reset counter, WiFi info
- **Jam screen** — warning display with one-button return home
- **WiFi screen** — shows current IP or AP info

### Web Interface
- **Full control from any browser** — responsive dark-theme UI works on phone and desktop
- **Real-time updates** — Server-Sent Events (SSE) push state changes at 250 ms intervals
- **Speed profile selector** — three profile buttons with active highlight
- **Per-profile SG controls** — independent ±1/±5 adjustment for each profile's StallGuard threshold
- **Motor current controls** — adjust run current with ±100 mA buttons
- **Work zone adjustment** — ±100/±500 step controls for the SG blanking zone
- **Endpoint tuning** — collapsible section with ±1/10/100 offset controls for UP and DOWN
- **Batch run controls** — set target, start, clear
- **Live log viewer** — 500-line scrollable log with real-time streaming
- **OTA firmware update** — drag-and-drop .bin upload with progress bar
- **WiFi configuration** — scan and select networks, save credentials, reset to AP mode

### WiFi & Networking
- **Auto-connect** — attempts saved credentials on boot, falls back to AP if it fails
- **Captive portal** — open AP mode (`AutoLee-Setup`, no password) with DNS redirect so any device gets the setup page automatically
- **Network scanner** — scans available WiFi networks and presents them in a dropdown
- **ArduinoOTA support** — update firmware from PlatformIO over the network (hostname: `autolee`, password: `autolee`)

---

## Project Structure

The firmware was originally a single-file Arduino sketch. It has been refactored into a modular architecture with extracted, testable components:

```
AutoLee/
├── src/
│   └── AutoLee.ino          # Main sketch — setup(), loop(), glue code
├── include/
│   ├── config.h              # Pin defs, constants, speed profile struct
│   ├── motor_fsm.h           # Motor state machine (IDLE/RUNNING/STOPPING/…)
│   ├── motor.h               # Motor control, calibration, homing routines
│   ├── stall_monitor.h       # Runtime stall detection sliding counter + blanking
│   ├── calibration.h         # Calibration hit detection (early + baseline + dynamic)
│   ├── batch.h               # Batch run state machine
│   ├── sg_filter.h           # StallGuard median-of-5 filter + helpers
│   ├── endpoint_math.h       # Endpoint clamping and offset math
│   ├── log_ring.h            # Ring buffer for the 500-line log
│   ├── state_json.h          # Pure JSON state serialization
│   ├── theme.h               # UI theme constants (colors, sizes)
│   ├── ui.h                  # LVGL touchscreen UI (all screens)
│   └── web.h                 # Web server, SSE, OTA, WiFi config
├── test/
│   └── test_native/
│       ├── test_motor_fsm/   # 29 tests — state transitions, guards, edge cases
│       ├── test_stall_monitor/ # 20 tests — blanking windows, sliding counter, jam
│       ├── test_calibration/ # 19 tests — early trip, baseline, dynamic hit
│       ├── test_batch/       # 15 tests — batch counting, start/clear/done
│       ├── test_sg_filter/   # 16 tests — median filter, SG helpers
│       ├── test_endpoint_math/ # 15 tests — clamping, offset logic
│       ├── test_log_ring/    # 10 tests — ring buffer push/wrap/clear
│       ├── test_state_json/  # 14 tests — JSON serialization, truncation
│       └── test_main/        # Smoke test
├── tools/
│   └── mock_server.py        # Mock web UI server (no hardware needed)
├── docs/
│   └── plans/                # Architecture and refactor planning docs
├── platformio.ini            # PlatformIO project config (esp32c6 + native)
├── justfile                  # Build/test/flash task runner
└── lv_conf.h                 # LVGL configuration
```

### Architecture

The core logic is split into **pure, header-only modules** that can be compiled and tested on your host machine (no ESP32 required):

| Module | Responsibility |
|---|---|
| `motor_fsm.h` | Explicit state machine with typed `MotorState` / `MotorEvent` enums and a transition table. Guards like `canRun()` and `isMoving()` prevent invalid operations. |
| `stall_monitor.h` | Runtime stall detection — sliding high/low counter with accel, work-zone, and decel blanking windows. Returns `BLANKED`, `OK`, or `JAM`. |
| `calibration.h` | Calibration hit detection — three-phase detector: early trip (immediate wall), baseline tracking with dynamic threshold, and confirmation counting. |
| `batch.h` | Batch run counting — tracks target, completed count, and done/active state. |
| `sg_filter.h` | Median-of-5 filter for StallGuard SPI reads, plus blanking helper functions. |
| `endpoint_math.h` | Pure math for endpoint offset clamping and position calculations. |
| `log_ring.h` | Fixed-size ring buffer for log lines with wrap-around and clear. |
| `state_json.h` | Pure JSON serialization of device state — takes a `StateSnapshot` struct, produces JSON string. |

The heavier hardware-dependent modules (`motor.h`, `ui.h`, `web.h`) are separated for readability but compile only on ESP32.

---

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (recommended via `pipx`)
- [just](https://github.com/casey/just) command runner (optional but makes everything easier)

### Quick Start

```bash
# Clone
git clone https://github.com/your-org/AutoLee.git
cd AutoLee

# Install PlatformIO and fetch all libraries
just setup

# Run all unit tests (no hardware needed)
just test

# Build firmware
just build

# Flash to connected ESP32-C6
just flash
```

If you don't have `just` installed:

```bash
# macOS
brew install just

# Linux (most distros)
# See https://github.com/casey/just#installation

# Or use PlatformIO directly:
pio test -e native -v          # run tests
pio run -e esp32c6             # build firmware
pio run -e esp32c6 -t upload   # flash
```

### Available just Recipes

| Recipe | Description |
|---|---|
| `just setup` | Install PlatformIO and fetch all library dependencies |
| `just test` | Run all native unit tests |
| `just test-one <suite>` | Run a single test suite (e.g. `just test-one motor_fsm`) |
| `just build` | Build firmware for ESP32-C6 |
| `just flash` | Upload firmware to connected board |
| `just monitor` | Open serial monitor at 115200 baud |
| `just run` | Build + flash + monitor in one step |
| `just ota [host]` | OTA upload (default: `autolee.local`) |
| `just mock` | Run the mock web server for UI development |
| `just files` | Show source/test file structure and line counts |
| `just log` | Show recent git history |

### Mock Web Server

For working on the web UI without any hardware:

```bash
just mock
# Opens http://localhost:8080 with a simulated AutoLee backend
```

The mock server (`tools/mock_server.py`) serves the same web interface with fake state data and responds to all API endpoints, so you can develop and test the UI in a browser.

### OTA Updates

After first flash, firmware can be updated two ways:

- **Web UI** — open the AutoLee web interface, scroll to "Firmware Update", drag and drop a `.bin` file
- **ArduinoOTA** — `just ota` (or `just ota 192.168.1.x` for a specific IP)

---

## Bill of Materials

### Electronics

| # | Component | Specs | Link |
|---|-----------|-------|------|
| 1 | WaveShare 1.47" ESP32-C6 | Touchscreen controller & UI | [Amazon.se](https://www.amazon.se/dp/B0F8B845Y6) |
| 2 | TMC5160T Plus | Silent stepper driver with StallGuard2 | [Amazon.se](https://www.amazon.se/-/en/dp/B0D5HQWW1C) |
| 3 | Buck Converter | 24 V → 5 V, ⚠️ verify physical dimensions | [Amazon.se](https://www.amazon.se/dp/B07DJ5HZ7G) |

### Mechanical

| # | Component | Specs | Link |
|---|-----------|-------|------|
| 4 | NEMA 23 Stepper Motor | 2.4 Nm, 4.0 A, 57×57×82 mm, 8 mm shaft | [Amazon.se](https://www.amazon.se/-/en/dp/B091C37FJ2) |
| 5 | Shaft Coupling | Motor-to-leadscrew (8 mm to 8 mm) | [Amazon.se](https://www.amazon.se/dp/B07CLLW7Z3) |
| 6 | Ball Screw Kit SFU1605 250 mm | ⚠️ verify physical dimensions | [Amazon.de](https://www.amazon.de/dp/B08WRJRM22) |

### Power

| # | Component | Specs | Link |
|---|-----------|-------|------|
| 7 | Power Supply | 24 V, 5 A DC | [Amazon.se](https://www.amazon.se/dp/B0CNPMCP6F) |
| 8 | On/Off Switch | Panel mount | [Amazon.se](https://www.amazon.se/dp/B07GDCNXKP) |
| 9 | DC Power Jack | 2.5 mm socket | [Amazon.se](https://www.amazon.se/dp/B081CM1G4M) |

### Cooling

| # | Component | Specs | Link |
|---|-----------|-------|------|
| 10 | Fan | 24 V, 40×40×20 mm | [Amazon.se](https://www.amazon.se/dp/B00MNJD8BE) |

> **Note:** The buck converter and leadscrew dimensions have not been fully verified — double-check fitment before ordering.

---

## Wiring

### ESP32-C6 → TMC5160T Plus (SPI)

| ESP32-C6 Pin | TMC5160 Pin | Function |
|:---:|:---:|---|
| GPIO 1 | SCK | SPI Clock |
| GPIO 2 | SDI (MOSI) | SPI Data In |
| GPIO 3 | SDO (MISO) | SPI Data Out |
| GPIO 8 | CS | SPI Chip Select |
| GPIO 4 | EN | Enable (active low) |
| GPIO 5 | STEP | Step pulse |
| GPIO 6 | DIR | Direction |
| GPIO 7 | DIAG1 | StallGuard diagnostic output |

### Power

| Connection | Details |
|---|---|
| 24 V PSU → TMC5160 VM | Motor power (24 V) |
| 24 V PSU → Buck converter IN | Feeds the buck converter |
| Buck converter OUT (5 V) → ESP32-C6 | Logic power |
| 24 V PSU → Fan | Direct 24 V to cooling fan |
| GND | Common ground between all boards |

> **Important:** The display and TMC5160 share the SPI bus (GPIO 1, 2). The firmware manages chip-select lines (GPIO 8 for TMC, GPIO 14 for display) to avoid bus conflicts. The display CS is forced high before every StallGuard SPI read.

---

## Configuration

Key constants live in `include/config.h`:

| Constant | Default | Description |
|---|---|---|
| `profiles[0..2]` | 15k/35k/45k Hz | Speed profiles (Slow/Normal/Fast) with SG thresholds |
| `RUN_ACCEL` | 800,000 | Accel/decel rate (steps/s²) |
| `RUN_CURRENT_MA` | 2,500 | Motor run current (mA), adjustable 1000–4500 |
| `CAL_CURRENT_MA` | 3,200 | Calibration current (mA) |
| `CAL_SPEED_HZ` | 8,000 | Calibration speed (Hz) |
| `SG_WORK_ZONE_MIN/MAX` | 0 / 20,000 | SG blanking zone limits |
| `DOWN_OFFSET_DEFAULT` | −500 | Default DOWN endpoint offset after calibration |
| `CAL_SGT` | −1 | StallGuard sensitivity for calibration |

---

## API Reference

All endpoints accept `POST` requests.

| Endpoint | Parameters | Description |
|---|---|---|
| `GET /api/state` | — | Returns full JSON state |
| `/api/toggle_run` | — | Start or stop running |
| `/api/profile` | `idx=0\|1\|2` | Switch speed profile |
| `/api/sg_trip` | `delta=N` `&profile=N` (optional) | Adjust SG threshold; targets active profile by default |
| `/api/work_zone` | `delta=N` | Adjust work zone blanking steps |
| `/api/endpoint` | `which=up\|down` `&delta=N` | Adjust endpoint offset |
| `/api/batch` | `delta=N` or `action=start\|clear` | Adjust batch target or start/clear |
| `/api/action` | `do=calibrate\|return_home\|reset_counter` | Trigger actions |
| `/api/wifi` | `ssid=...` `&pass=...` | Save WiFi credentials and reboot |
| `/api/wifi_reset` | — | Clear saved WiFi, reboot to AP mode |
| `/api/ota` | multipart `.bin` upload | Firmware update |
| `/api/log_clear` | — | Clear the log buffer |

SSE stream available at `/events` — pushes JSON state every 250 ms and log lines as `log` events.

---

## Testing

The pure-logic modules are tested natively on your development machine using [Unity](http://www.throwtheswitch.org/unity) via PlatformIO's `native` environment — no ESP32 hardware needed.

```bash
# Run all tests
just test

# Run one suite
just test-one motor_fsm
just test-one stall_monitor
just test-one calibration
just test-one batch
just test-one sg_filter
just test-one endpoint_math
just test-one log_ring
just test-one state_json
```

The test suites cover state transitions, boundary conditions, overflow behavior, and edge cases across all extracted modules.

---

## Version History

| Version | Changes |
|---|---|
| **v1.6** | Modular architecture (motor FSM, stall monitor, calibration detector, batch, SG filter, endpoint math, log ring, state JSON extracted as testable headers); multi-page touch UI; adjustable motor current; updated speed profiles (15k/35k/45k); PlatformIO build system with `just` task runner; 140 native unit tests; mock web server for UI development |
| **v1.5** | Speed profiles (Slow/Normal/Fast) replace speed slider; per-profile SG thresholds; profile API |
| **v1.4** | Captive portal WiFi; work zone SG blanking; RUN_DECEL 800k; median-of-5 SPI filter; sliding counter stall detection; 500-line log; redesigned web UI |
| **v1.3** | Batch run; jam screen with return-home; runtime StallGuard monitoring; web log viewer |
| **v1.2** | Web UI with SSE; OTA updates; endpoint tuning; WiFi AP/STA |
| **v1.1** | Sensorless calibration; basic touch UI |
| **v1.0** | Initial release |

---

## License

This project is provided as-is for personal and hobby use. See the source code for details.
