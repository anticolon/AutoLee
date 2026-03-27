[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

# AutoLee

**Automated Lee APP conversion — ESP32-C6 firmware with touchscreen UI and web control.**

AutoLee converts a manual Lee APP into a fully automated decapping machine using a stepper motor, sensorless homing, and stallguard jam detection. It runs on a tiny 1.47" touchscreen ESP32-C6 module and can also be controlled from any phone/computer via its built-in web interface.

Find the 3D-printable parts here: https://makerworld.com/en/models/2529369-autolee-conversion-kit
Support my work: https://buymeacoffee.com/kl.design

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

### Speed Profiles
- **Three preset profiles** — Slow (15 kHz, SG=350), Normal (35 kHz, SG=15), Fast (45 kHz, SG=1)
- **Per-profile StallGuard threshold** — each speed has its own SG trip value, eliminating false jams when changing speed
- **One-tap switching** — change profile from the touchscreen or web UI; speed and SG update together instantly
- **Fine-tune SG per profile** — type a value directly into text inputs on the web Configuration page, or use ±1/±5 buttons on the touch screen

### Motor Current
- **Adjustable run current** — 1,000–4,500 mA via web slider (default 3,500 mA)
- **Overcurrent warning** — values above 4,000 mA show a warning (exceeds motor rating, ensure cooling)
- **Live adjustment** — takes effect immediately, no restart needed

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
- **Main screen** — Counter, active speed profile indicator, calibration warning, batch remaining, buttons for Batch Run and Settings
- **Settings screen** — Calibrate, Configuration, Reset Counter
- **Configuration screen** — Speed Profile, Endpoints, Stall Guard, WiFi Info
- **Speed Profile screen** — Three buttons with green highlight on active, info card showing Hz + SG
- **Endpoints (tuning) screen** — Raw and effective endpoint values, buttons to edit UP and DOWN
- **Stall Guard screen** — Adjust SG trip for the active profile with ±1/±5 buttons
- **Batch Run screen** — Set target count with ±1/10/100 buttons, start batch
- **Jam screen** — Warning display with one-button return home
- **WiFi screen** — Shows connected SSID and IP address, Reset WiFi button to clear credentials and reboot

### Web Interface (5-page layout)
- **Full control from any browser** — responsive dark-theme UI works on phone and desktop
- **Real-time updates** — Server-Sent Events (SSE) push state changes at 250 ms intervals
- **Main page** — status, counter, RUN/STOP, calibrate, reset counter, speed profile selector, batch run controls
- **Configuration page** — motor current slider (1,000–4,500 mA with overcurrent warning), endpoint tuning with collapsible offsets, per-profile StallGuard text inputs, work zone adjustment
- **Log page** — full-height 500-line scrollable log with real-time streaming and clear button
- **Firmware page** — drag-and-drop OTA .bin upload with progress bar
- **WiFi page** — shows connection status, SSID, and IP address; change credentials, reset to AP mode
- **Footer navigation** — blue text links on every page to jump between all five pages

### WiFi & Networking
- **Auto-connect** — attempts saved credentials on boot, falls back to AP if it fails
- **Captive portal** — open AP mode (`AutoLee-Setup`, no password) with DNS redirect so any device gets the setup page automatically
- **Network scanner** — scans available WiFi networks and presents them in a dropdown
- **ArduinoOTA support** — update firmware from PlatformIO/Arduino IDE over the network (hostname: `autolee`, password: `autolee`)

---

## File Structure

As of v1.8, the firmware is split into modular files for maintainability. All files must be in the same sketch folder.

| File | Purpose |
|---|---|
| `AutoLee.ino` | Main entry point — globals, `setup()`, `loop()`, include order |
| `config.h` | All tuning constants, pin definitions, speed profiles |
| `motion.h` | Motion control, stall detection, calibration, creep home |
| `ui_touch.h` | LVGL touch UI — screen builders, helpers, event handlers |
| `web_server.h` | Web server, API endpoints, SSE broadcast, HTML, OTA upload |
| `wifi_ota.h` | WiFi connection, captive portal, ArduinoOTA |
| `globals.h` | Reference document — lists all shared variables and forward declarations (not included in the build) |
| `lv_conf.h` | LVGL configuration — display size, enabled features, font selections |

The Arduino IDE compiles everything as a single translation unit. Include order in `AutoLee.ino` resolves all dependencies: `config.h` → globals → `motion.h` → `ui_touch.h` → `wifi_ota.h` → `web_server.h`.

---

## Bill of Materials

> **Support this project:** The product links below are affiliate links. If you purchase through them, I earn a small commission at no extra cost to you — it's a simple way to help fund continued development of AutoLee. Thank you!

### Electronics

| # | Component | Specs | Link |
|---|-----------|-------|------|
| 1 | WaveShare 1.47" ESP32-C6 | Touchscreen controller & UI | [Amazon.se](https://www.amazon.se/dp/B0F8B845Y6?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B0FC5SNKH4?tag=kldesign00-20) |
| 2 | TMC5160T Plus | Silent stepper driver with StallGuard2 | [Amazon.se](https://www.amazon.se/dp/B0D5HQWW1C?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B0CHFK7VBL?tag=kldesign00-20) |
| 3 | Buck Converter | 24 V → 5 V | [Amazon.se](https://www.amazon.se/dp/B07DJ5HZ7G?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B0DC3N7PMY?tag=kldesign00-20) |

### Mechanical

| # | Component | Specs | Link |
|---|-----------|-------|------|
| 4 | NEMA 23 Stepper Motor | 2.4 Nm, 4.0 A, 57×57×82 mm, 8 mm shaft | [Amazon.se](https://www.amazon.se/dp/B091C37FJ2?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B0GRWQSY5Q?tag=kldesign00-20) |
| 5 | Shaft Coupling | Motor-to-leadscrew (8 mm to 10 mm) | [Amazon.se](https://www.amazon.se/dp/B07CLLW7Z3?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B08QV1QN81?tag=kldesign00-20) |
| 6 | Ball Screw Kit SFU1605 250 mm | 250mm SFU1605 BK12/BF12 10 mm Shaft | [Amazon.de](https://www.amazon.de/dp/B08WRJRM22?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B09BQSWPM4?tag=kldesign00-20) |

### Power

| # | Component | Specs | Link |
|---|-----------|-------|------|
| 7 | Power Supply | 24 V, 5 A DC | [Amazon.se](https://www.amazon.se/dp/B0CNPMCP6F?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B0BY7P38Q5?tag=kldesign00-20) |
| 8 | On/Off Switch | Panel mount | [Amazon.se](https://www.amazon.se/dp/B07GDCNXKP?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B078KBC5VH?tag=kldesign00-20) |
| 9 | DC Power Jack | 2.5 mm socket | [Amazon.se](https://www.amazon.se/dp/B081CM1G4M?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B09W9SJ1B6?tag=kldesign00-20) |
| 10 | Emergency Stop | Button | [Amazon.se](https://www.amazon.se/dp/B0FFMTCFLK?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B0FFMTCFLK?tag=kldesign00-20) |

### Cooling

| # | Component | Specs | Link |
|---|-----------|-------|------|
| 11 | Fan | 24 V, 40×40×20 mm | [Amazon.se](https://www.amazon.se/dp/B00MNJD8BE?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B07B66DJYX?tag=kldesign00-20) |

### Wiring Supplies

| # | Component | Specs | Link |
|---|-----------|-------|------|
| 12 | Silicone Wire | 18 AWG, 24 V power wiring (PSU → driver) | [Amazon.com](https://www.amazon.com/Silicone-Electrical-Conductor-Parallel-Flexible/dp/B07FMRDP87?tag=kldesign00-20) |
| 13 | Silicone Wire | 24 AWG, flexible stranded, signal wiring | [Amazon.com](https://www.amazon.com/TUOFENG-Wire-Stranded-Flexible-Silicone-Different/dp/B07G2BWBX8?tag=kldesign00-20) |
| 14 | Dupont Connector Kit + Crimping Tool | 2.54 mm connectors, housings, and ratcheting crimper | [Amazon.com](https://www.amazon.com/Crimping-Connector-Assortment-Ratcheting-0-25-1-5mm%C2%B2/dp/B0FJ8LCZ9W?tag=kldesign00-20) |
| 15 | Ferrule Connector Kit + Crimping Tool | For power and motor wires to TMC5160 terminal block | [Amazon.com](https://www.amazon.com/Preciva-Hexagonal-Self-adjustable-Terminals-Connectors/dp/B0D3D65VZT?tag=kldesign00-20) |

### Hardware (Fasteners & Inserts)

#### Bolts / Screws

| Qty | Size | Used For |
|-----|------|----------|
| 15 pcs | M4 x 16mm | Motor, Motor mount, Backplane upper, Backplane lower |
| 11 pcs | M5 x 40mm | Ballscrew mounts, Sled clamp |
| 4 pcs | M5 x 25mm | Sled mount |
| 1 pcs | M4 x 20mm | Display mount |
| 4 pcs | M3 x 30mm | 24V Fan |
| 4 pcs | M3 x 5mm | TMC5160T |
| 1 pcs | M3 x 10mm | Driverhousing mounting to backplane|
| 2 pcs | M3 x 10mm | Driverhousinglid|
| 4 pcs | M2 x 5mm | Display |

#### Lock Nuts

| Qty | Size | Used For |
|-----|------|----------|
| 3 pcs | M5 Lock nut | Sled clamp |
| 4 pcs | M3 Lock nut | 24V Fan |

#### Heat Inserts

| Qty | Size | Used For | Link |
|-----|------|----------|------|
| 16 pcs | M4 Heat insert | Motor, Motor mount, Backplane upper, Backplane lower, Display mount | [Amazon.se](https://www.amazon.se/dp/B09MTTC7S9?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B0FCXXW62N?tag=kldesign00-20) ¹ |
| 8 pcs | M5 Heat insert | Ballscrew mount | [Amazon.se](https://www.amazon.se/dp/B07YSVXWS8?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B0FCXXW62N?tag=kldesign00-20) ¹ |
| 5 pcs | M3 Heat insert | TMC5160T mount, Driverhousing to backplane | [Amazon.se](https://www.amazon.se/dp/B08BCRZZS3?tag=kldesign-21) · [Amazon.com](https://www.amazon.com/dp/B0FCXXW62N?tag=kldesign00-20) ¹ |

> ¹ The US link is a bundle kit that includes M3, M4, and M5 inserts.

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

## Software Setup

### Dependencies (Arduino / PlatformIO)

| Library | Version | Install | Purpose |
|---|---|---|---|
| `LVGL` | v8.4.0 | Online | Touchscreen UI framework |
| `GFX_Library_for_Arduino` | v1.5.9 | Online | ST7789 display driver |
| `TMCStepper` | — | Online | TMC5160 SPI communication |
| `FastAccelStepper` | — | Online | Step pulse generation with acceleration |
| `ESPAsyncWebServer` + `AsyncTCP` | — | Online | Web server & SSE |
| `ArduinoOTA` | — | Online | Over-the-air firmware updates |
| `DNSServer` | — | Online | Captive portal redirect |
| `esp_lcd_touch_axs5106l` | — | **Offline** | AXS5106L touch controller driver |

> **Note:** The `esp_lcd_touch_axs5106l` library is **not available** in the Arduino Library Manager. You must install it manually from Waveshare's demo package — see step 3 below.

### Build & Flash

1. Clone this repo
2. Install all "Online" libraries above via the Arduino Library Manager
3. Install the touch driver **offline**:
   - Download the [Waveshare ESP32-C6-Touch-LCD-1.47 demo package](https://www.waveshare.com/wiki/ESP32-C6-Touch-LCD-1.47)
   - Find the `esp_lcd_touch_axs5106l` library folder inside the package
   - Copy it to your Arduino `libraries` directory
4. Set up LVGL:
   - Copy `lv_conf.h` from this repo to sit **next to** your `lvgl` library folder (not inside it)
   - Copy the `demos` folder from inside the LVGL library into its `src` folder
5. Open `AutoLee.ino` in Arduino IDE or PlatformIO — all `.h` files must be in the same folder as the `.ino`
6. Select board: **ESP32-C6**
7. Set partition scheme: **Minimal SPIFFS (1.9 MB APP with OTA/190 KB SPIFFS)** — the firmware is too large for the default partition layout
8. Compile and flash

## Flash Pre-Compiled Binary (No Arduino IDE Required)
 
If you don't want to set up the Arduino IDE and compile the firmware yourself, you can flash a pre-built binary directly to the ESP32-C6 using a web browser.
 
### What You Need
 
- A **Chrome** or **Edge** browser (Web Serial is not supported in Firefox or Safari)
- A **USB-C cable** connected to the Waveshare ESP32-C6 board
- The merged firmware `.bin` file from the [`/Firmware`](Firmware/) folder in this repo
 
### Steps
 
1. Download the latest `AutoLee_vX.X_merged.bin` from the [`/Firmware`](Firmware/) folder
2. Open the [**Espressif Web Flasher**](https://espressif.github.io/esptool-js/) in Chrome or Edge
3. Click **Connect** and select the port for your ESP32-C6
4. In the **Program** section, enter **`0x0`** in the Flash Address field
5. Click the file picker next to the address and select the downloaded `.bin` file
6. Set Flash Mode as **dio** and Flash Size as **4MB**
7. Click **Program**
8. Wait for flashing to complete — progress will show in the Console section at the bottom
9. After programming is complete click the **rst** button on the esp32 board to reboot
10. System should now be online

**Tip:** If the board doesn't show up as a COM port, hold the **BOOT** button on the Waveshare board while plugging in USB, then release after connecting. You may also need to install the [CH343 USB driver](https://www.wch-ic.com/downloads/CH343SER_ZIP.html) if your OS doesn't recognize the board.

### Updating Firmware Later
 
Once AutoLee is on your WiFi, go to the web UI → **Firmware** page and drag-and-drop the **app-only** `.bin` file (not the merged binary). The merged binary is only needed for the initial USB flash.

### OTA Updates

After first flash, firmware can be updated two ways:

- **Web UI** — open the AutoLee web interface, go to the Firmware page, drag and drop a `.bin` file
- **ArduinoOTA** — hostname `autolee`, password `autolee`
---

## Configuration

Key constants are in `config.h`:

| Constant | Default | Description |
|---|---|---|
| `profiles[0]` (Slow) | 15,000 Hz / SG 350 | Low speed, high SG threshold — max torque for tough primers |
| `profiles[1]` (Normal) | 35,000 Hz / SG 15 | Balanced speed and sensitivity |
| `profiles[2]` (Fast) | 45,000 Hz / SG 1 | High speed, very sensitive stall detection |
| `RUN_CURRENT_MA` | 3,500 mA | Motor run current (adjustable 1,000–4,500 via web UI) |
| `RUN_DECEL` | 800,000 | Accel/decel rate (steps/s²) |
| `CAL_CURRENT_MA` | 3,200 | Calibration current (mA, fixed) |
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
| `/api/sg_trip` | `value=N` or `delta=N`, `&profile=N` (optional) | Set or adjust SG threshold; targets active profile by default |
| `/api/current` | `ma=N` | Set motor run current (1,000–4,500 mA) |
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
| **v1.8** | Firmware split into modular files (`config.h`, `motion.h`, `ui_touch.h`, `web_server.h`, `wifi_ota.h`) for maintainability — no functional changes from v1.7 |
| **v1.7** | WiFi Info moved to Configuration sub-menu; Reset WiFi button on WiFi info screen; speed profile buttons resized to fit display; WiFi info centered in card |
| **v1.6** | Adjustable motor current (1,000–4,500 mA) via web; multi-page web UI (Main, Configuration, Log, Firmware, WiFi); touch UI restructured (Settings → Configuration sub-menu); WiFi page shows SSID + IP; SG text inputs with auto-submit on blur; profiles retuned (Slow 15kHz/350, Normal 35kHz/15, Fast 45kHz/1); all labels fitted to 172px display |
| **v1.5** | Speed profiles (Slow/Normal/Fast) replace speed slider; per-profile SG thresholds; profile API |
| **v1.4** | Captive portal WiFi; work zone SG blanking; RUN_DECEL 800k; median-of-5 SPI filter; sliding counter stall detection; 500-line log; redesigned web UI |
| **v1.3** | Batch run; jam screen with return-home; runtime StallGuard monitoring; web log viewer |
| **v1.2** | Web UI with SSE; OTA updates; endpoint tuning; WiFi AP/STA |
| **v1.1** | Sensorless calibration; basic touch UI |
| **v1.0** | Initial release |

---

## License

This project is licensed under the [Creative Commons Attribution-NonCommercial 4.0 International License](https://creativecommons.org/licenses/by-nc/4.0/).

You are free to use, modify, and share this work for personal, non-commercial purposes, provided you give appropriate credit. Commercial use is not permitted without prior written permission.

Commercial use — including selling devices, kits, or services based on this project — is prohibited without prior written permission from the author.

THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND. USE AT YOUR OWN RISK.

Copyright (c) 2025 K.L Design
