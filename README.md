# AutoLee

**Automated Lee APP (Automatic Primer Press) conversion — ESP32-C6 firmware with touchscreen UI and web control.**

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
- **Fast ramp profile** — 800,000 steps/s² accel/decel (RUN_DECEL) for maximum cruise time and StallGuard coverage
- **Safe return-home** — after a jam or stall, creeps back to UP stop using calibration speed with stall detection and retries

### Speed Profiles (v1.5)
- **Three preset profiles** — Slow (30 kHz), Normal (40 kHz), Fast (50 kHz)
- **Per-profile StallGuard threshold** — each speed has its own SG trip value, eliminating false jams when changing speed
- **One-tap switching** — change profile from the touchscreen or web UI; speed and SG update together instantly
- **Fine-tune SG per profile** — adjust ±1/±5 from the touch Stall Sensitivity screen or per-profile web controls

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

### Touch UI (172×320 LVGL)
- **Main screen** — counter, active speed profile indicator, calibration warning, batch remaining
- **Speed Profile screen** — three large buttons with green highlight on active, info card showing Hz + SG
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
- **ArduinoOTA support** — update firmware from PlatformIO/Arduino IDE over the network (hostname: `autolee`, password: `autolee`)

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

### Display (shared SPI bus)

| ESP32-C6 Pin | Display Pin | Function |
|:---:|:---:|---|
| GPIO 1 | SCK | SPI Clock (shared with TMC) |
| GPIO 2 | MOSI | SPI Data (shared with TMC) |
| GPIO 14 | CS | Display Chip Select |
| GPIO 15 | DC | Data/Command |
| GPIO 22 | RST | Display Reset |
| GPIO 23 | BL | Backlight |

### Touch (I2C)

| ESP32-C6 Pin | Touch Pin | Function |
|:---:|:---:|---|
| GPIO 18 | SDA | I2C Data |
| GPIO 19 | SCL | I2C Clock |
| GPIO 20 | RST | Touch Reset |
| GPIO 21 | INT | Touch Interrupt |

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

## Software Setup

### Dependencies (Arduino / PlatformIO)

| Library | Purpose |
|---|---|
| `LVGL` (v8.4.x) | Touchscreen UI framework |
| `TMCStepper` | TMC5160 SPI communication |
| `FastAccelStepper` | Step pulse generation with acceleration |
| `Arduino_GFX_Library` | ST7789 display driver |
| `ESPAsyncWebServer` + `AsyncTCP` | Web server & SSE |
| `ArduinoOTA` | Over-the-air firmware updates |
| `DNSServer` | Captive portal redirect |

### Build & Flash

1. Clone this repo
2. Copy `lv_conf.h` next to your `lvgl` library folder
3. Open `AutoLee.ino` in Arduino IDE or PlatformIO
4. Select board: **ESP32-C6**
5. Set partition scheme: **Minimal SPIFFS (1.9 MB APP with OTA/190 KB SPIFFS)** — the firmware is too large for the default partition layout
6. Flash

### OTA Updates

After first flash, firmware can be updated two ways:

- **Web UI** — open the AutoLee web interface, scroll to "Firmware Update", drag and drop a `.bin` file
- **ArduinoOTA** — hostname `autolee`, password `autolee`

---

## Configuration

Key constants at the top of `AutoLee.ino`:

| Constant | Default | Description |
|---|---|---|
| `profiles[0..2]` | 30k/40k/50k Hz | Speed profiles (Slow/Normal/Fast) with SG thresholds |
| `RUN_DECEL` | 800,000 | Accel/decel rate (steps/s²) |
| `RUN_CURRENT_MA` | 2,500 | Motor run current (mA) |
| `CAL_CURRENT_MA` | 3,200 | Calibration current (mA) |
| `CAL_SPEED_HZ` | 8,000 | Calibration speed (Hz) |
| `SG_WORK_ZONE_STEPS` | 5,500 | SG blanking zone near DOWN endpoint |
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

## Version History

| Version | Changes |
|---|---|
| **v1.5** | Speed profiles (Slow/Normal/Fast) replace speed slider; per-profile SG thresholds; profile API |
| **v1.4** | Captive portal WiFi; work zone SG blanking; RUN_DECEL 800k; median-of-5 SPI filter; sliding counter stall detection; 500-line log; redesigned web UI |
| **v1.3** | Batch run; jam screen with return-home; runtime StallGuard monitoring; web log viewer |
| **v1.2** | Web UI with SSE; OTA updates; endpoint tuning; WiFi AP/STA |
| **v1.1** | Sensorless calibration; basic touch UI |
| **v1.0** | Initial release |

---

## License

This project is provided as-is for personal and hobby use. See the source code for details.
