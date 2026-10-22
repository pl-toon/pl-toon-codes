# Tren CE — Mobile Firmware (Standalone)

ESP32-based train controller with a built-in WiFi access point and mobile web dashboard. Connect a phone to the ESP32's WiFi, open the browser, and run PID / Step Response / Deadband Calibration experiments. No router, no laptop, no MQTT broker required.

This folder is **self-contained**: firmware (`.ino`), partition table, web dashboard (`data/`), and these instructions. If you received only this folder, you have everything you need.

---

## Contents

```
tren_esp_mobile/
├── README.md               ← This file
├── tren_esp_mobile.ino     ← Main firmware
├── ota_github.ino          ← Over-the-air updates from GitHub Releases
├── partitions.csv          ← Custom partition table (2 × 2 MB app slots for OTA + 1 MB LittleFS)
└── data/
    ├── index.html          ← Mobile dashboard UI
    ├── uplot.min.js        ← Chart library (bundled, no CDN)
    └── uplot.min.css       ← Chart styles
```

No external assets — the dashboard works fully offline once flashed.

---

## Hardware

- ESP32-S3 Dev Module with **16 MB flash**
- VL53L0X Time-of-Flight distance sensor (I2C: SDA=GPIO 10, SCL=GPIO 9)
- Motor driver with these connections:
  - `STBY = GPIO 7`
  - `FWD  = GPIO 8`
  - `BACK = GPIO 18`
  - `PWM  = GPIO 17`
- USB cable for programming
- Phone or tablet (any modern browser)

---

## Software Prerequisites

- **Arduino IDE** 2.x (or 1.8.x)
- **ESP32 Board Package** v3.x — install `esp32` by Espressif in the Board Manager
- **esptool.py** + **mklittlefs** — bundled with the ESP32 core; paths shown below

### Required Arduino Libraries (Library Manager → Install)

| Library | Author | Version |
|---------|--------|---------|
| ESPAsyncWebServer | ESP32Async (mathieucarbou) | v3.0+ (tested with 3.12.1) |
| AsyncTCP | ESP32Async (mathieucarbou) | latest (tested with 3.5.0) |
| ArduinoJson | Benoit Blanchon | v7.x (tested with 7.4.3) |
| PID_v1_bc | — | latest (tested with 1.2.7) |
| VL53L0X | Pololu | latest (tested with 1.3.1) |

LittleFS is built into the ESP32 core — no separate install needed.

---

## Installation

### 1. Open the sketch

Open `tren_esp_mobile.ino` in Arduino IDE. The `.ino` file must remain in a folder named `tren_esp_mobile/` (Arduino IDE requirement). `ota_github.ino` must stay in the same folder.

### 2. Board settings

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| Partition Scheme | Custom (auto-detects `partitions.csv`) |
| Flash Size | **16MB** |
| PSRAM | **OPI PSRAM** (required: OTA buffers each image in PSRAM) |
| Upload Speed | 921600 |
| Port | Your ESP32's serial port |
| Erase All Flash Before Sketch Upload | Disabled (keeps the train ID) |

### 3. Upload the LittleFS filesystem (web dashboard files)

This is the step most people miss. The firmware needs `index.html`, `uplot.min.js`, and `uplot.min.css` flashed to a separate filesystem partition.

**Recommended: command line** (the Arduino IDE plugin often fails on ESP32-S3):

```bash
# Build the LittleFS image from the data/ folder
~/Library/Arduino15/packages/esp32/tools/mklittlefs/4.0.2-db0513a/mklittlefs \
  -c data -s 0x100000 -p 256 -b 4096 /tmp/littlefs.bin

# Flash the image (replace port with your actual serial port)
esptool.py --chip esp32s3 --port /dev/cu.usbmodem1101 --baud 460800 \
  write_flash 0x410000 /tmp/littlefs.bin
```

- macOS path shown; on Windows use `%LOCALAPPDATA%\Arduino15\packages\esp32\tools\mklittlefs\...`, on Linux `~/.arduino15/...`
- `-s 0x100000` = LittleFS partition size from `partitions.csv` (1 MB)
- `0x410000` = LittleFS partition offset from `partitions.csv`
- Use the **UART** port, not the native USB-CDC port, on ESP32-S3
- **Close the Arduino Serial Monitor first**, or the port will be busy

**Alternative — Arduino IDE 2.x plugin** ([arduino-littlefs-upload](https://github.com/earlephilhower/arduino-littlefs-upload)):
1. Install the plugin (see plugin README)
2. `Tools → Upload LittleFS to ...`

**Alternative — PlatformIO**: `pio run --target uploadfs`

> Upload LittleFS **before** the firmware, or the first boot will log "LittleFS mount failed."
>
> The LittleFS image must be generated with `-s 0x100000`. An image built for another partition size will not mount correctly.

### 4. Upload the firmware

Press the Upload button in Arduino IDE (or `Ctrl+U`).

### 5. First-time configuration

Open the Serial Monitor at **115200 baud, line ending: Newline**. You should see:

```
*** TRAIN NOT CONFIGURED ***
Waiting for configuration...
```

Type:

```
SET_TRAIN:trainA:5555
```

The ESP32 saves the config to its NVS flash area (`Preferences`), flashes the LED 3 times, and reboots. After reboot:

```
TRAIN CONFIGURATION LOADED
Train ID: trainA
Starting WiFi Access Point...
  SSID: TrenCE-trainA
  Password: train123
  IP: 192.168.4.1
Setup Complete! Ready for mobile control.
```

The port number is stored but unused in mobile mode (it exists for compatibility with the laptop firmware).

**Re-flashing keeps the train ID.** The ID lives in the `nvs` partition (0x9000), which neither the firmware upload nor the LittleFS upload touches. It is erased only by `RESET_TRAIN`, by `esptool.py erase_flash`, or by the Arduino IDE option *Tools → Erase All Flash Before Sketch Upload: Enabled*. Keep that option **Disabled** when updating trains that are already configured.

### Serial commands

| Command | Description |
|---------|-------------|
| `SET_TRAIN:id:port` | Configure train ID (e.g. `SET_TRAIN:trainB:5556`) |
| `GET_TRAIN` | Show current configuration |
| `RESET_TRAIN` | Clear config and reboot |
| `STATUS` | Same as GET_TRAIN |
| `SET_WIFI:ssid:password` | Store the Wi-Fi network used for OTA updates (see below) |
| `OTA_CHECK` | Check GitHub Releases for a newer firmware now |

### LED status

| Pattern | Meaning |
|---------|---------|
| Fast blink (5 Hz) | Not configured — waiting for `SET_TRAIN` |
| Slow blink (1 Hz) | AP active, waiting for phone |
| Solid ON | Phone connected via WebSocket |

---

## Over-the-air updates (GitHub Releases)

The firmware can update itself from the releases of this repository. The train keeps its own access point for the phone; only while checking for an update does it also join a Wi-Fi network that has Internet access, and it leaves that network as soon as the check ends.

### Setup (once per train)

1. Connect the train by USB and type over Serial:
   ```
   SET_WIFI:<network name>:<password>
   ```
   The ssid cannot contain `:`. The password can.
2. Reboot the train. The log shows `[STA] Connected: <ip>` and the check runs at boot.

Without `SET_WIFI`, the train works normally in AP-only mode and no update is attempted.

### How an update runs

- At boot, and on demand with `OTA_CHECK`, the train reads `version.json` from the latest release.
- If the version differs from `FW_VERSION`, the train downloads `firmware.bin` and `littlefs.bin` and checks each one against its SHA-256 **before** writing anything.
- Only then does it write the application and the filesystem, and it restarts.
- The update is skipped if an experiment is running or the motor is not stopped.
- If the download or verification fails, nothing is written and the current firmware keeps running.
- After the check, whether it succeeds or fails, the train disconnects from the Wi-Fi network and returns to AP-only mode, so it does not keep scanning for the router during experiments.

### Publishing a release (maintainers)

1. Set `FW_VERSION` in `tren_esp_mobile.ino` to the new version (for example `2.5.1-mobile`) and merge it into `main`.
2. Create and push a tag with the prefix `tren-v`:
   ```bash
   git tag -a tren-v2.5.1-mobile -m "Release tren_esp_mobile 2.5.1-mobile"
   git push origin tren-v2.5.1-mobile
   ```
3. GitHub Actions (`.github/workflows/release-tren.yml`) compiles the firmware, builds `littlefs.bin`, computes the hashes and publishes `firmware.bin`, `littlefs.bin` and `version.json` in a new release.

The tag must match `FW_VERSION` exactly, without the `tren-v` prefix. If they differ, the workflow stops before publishing.

### Notes

- **The first install of this partition table must be done by USB** (steps 3 and 4). Later updates can come from GitHub.
- The root certificates in `GITHUB_ROOT_CA` (`ota_github.ino`) must be valid for `github.com` and `objects.githubusercontent.com`. If GitHub changes its certificate chain, the update will stop until the certificate is replaced.
- The network must allow outbound HTTPS and DNS. The phone can stay connected to `TrenCE-xxx` while the update runs.

---

## Using the Dashboard

1. On your phone, open WiFi settings and connect to **`TrenCE-trainA`** (or whatever train ID you configured)
2. Password: **`train123`**
3. Open a browser and go to **`http://192.168.4.1`**
4. If the phone warns "No Internet" — that's expected. Tap "Stay connected" or "Use this network anyway."

The dashboard has three experiment modes selected by tabs at the top.

### PID Control
Closed-loop distance tracking. Sliders: Kp (0–100), Ki (0–50), Kd (0–50), Reference (1–100 cm), Deadband (0–200 PWM). Parameters can be adjusted live while running. Chart shows distance, reference, and error.

### Step Response
Open-loop motor step. Sliders: Amplitude (0–Vbatt V), Duration (0.5–20 s), Battery Voltage (3.0–12.0 V), Direction. Three-phase execution: 5-sample warm-up → 3-sample baseline → step applied. Auto-stops when duration expires.

### Deadband Calibration
Ramps PWM from 0 until motion is detected. Sliders: Motion Threshold (0.01–1.0 cm), Direction. Result appears as a large number when done — tap **Apply Result to PID** to use as PID deadband.

### Data Export
- **Save CSV** — downloads the current run's chart data as `<mode>_<timestamp>.csv`
- **Copy CSV** — same data, shown in a panel to copy (for browsers that block downloads)
- **Save Plot** — downloads chart as PNG
- **Export log** — downloads the session event log as `registro_<trainID>_<timestamp>.csv` (since 2.5.0, see below)

Run data lives only in the open page, and each new run clears the previous one: save the CSV after every run, before
starting the next. The event log is different: it is kept in the phone's browser storage and survives a reload or a
closed tab, until it is exported.

### Language / Theme / Presets
- **ES/EN** toggle (top-right) — full Spanish/English translation, saved in phone localStorage
- **Sun/moon icon** — dark/light theme, saved per phone
- **Presets** — save current parameters under a name, reload later. Stored on the phone, not the ESP32.

### Safety features
- **Auto-stop on phone disconnect** — if WiFi drops or browser closes, the ESP32 stops the motor
- **Deadband safety limit** — calibration aborts if PWM exceeds 800 without motion
- **PWM clamping** — motor PWM is constrained to 0–1024

---

## WebSocket Protocol (for developers)

Endpoint: `ws://192.168.4.1/ws`. All messages are JSON.

**Phone → ESP32 (commands):**
```json
{"t":"cmd","c":"kp","v":2.5}
{"t":"cmd","c":"start","m":"pid"}
{"t":"cmd","c":"stop"}
```

**ESP32 → Phone (data, ~11 Hz during experiment):**
```json
{"t":"d","m":"pid","ts":12345,"d":15.23,"r":10.0,"e":-5.23,
 "kp":2.5,"ki":0.8,"kd":1.2,"u":127.5,"pwm":177,"dir":1}
```

Full command/data field reference is in the firmware header comment (`tren_esp_mobile.ino`, lines 25–34).

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Serial: `FATAL: Cannot start without LittleFS!` | LittleFS data not uploaded — repeat Step 3 |
| Phone shows "No Internet Connection" | Expected. Tap "Stay connected." On iOS, dismiss popup and open Safari manually |
| Browser can't reach 192.168.4.1 | Confirm phone is on `TrenCE-xxx` WiFi (not your home WiFi). Use `http://` not `https://` |
| Connection indicator stays red | WebSocket never opened. Hard-refresh the page. Check Serial Monitor for `[WS] Client #1 connected` |
| Sliders seem unresponsive | Sliders send on finger **release**, not while dragging. Check "Confirmed ESP32 Values" — they only update after ESP32 ack |
| Motor doesn't move (PID) | If Kp=Ki=Kd=0, PID output is 0. Start with Kp≈5–10 |
| Motor doesn't move (Step) | Both Amplitude and Duration must be > 0 |
| esptool: "Port is busy" | Close Arduino Serial Monitor before flashing |
| esptool: "Failed to write to target RAM" | Use the UART port, not the native USB-CDC port |
| Chart not updating | Chart updates only during a running experiment. First 5 points are discarded (sensor warm-up) |
| Serial: `[OTA] Couldn't read the manifest` or `GET failed: -1` | Check the Wi-Fi network has Internet access, and that the root certificates in `GITHUB_ROOT_CA` are valid |
| Serial: `[OTA] SHA-256 mismatch` | The downloaded file does not match `version.json`. Check the release, then retry with `OTA_CHECK` |
| Serial: `[OTA] Experiment in progress or active engine` | Stop the experiment and the motor, then run `OTA_CHECK` again |

---

## Sensor Notes

The VL53L0X is configured for a balance of speed and accuracy:

- Timing budget: 33 ms (default)
- Continuous read mode with data-ready polling
- 5-sample moving average + EMA blend (80% new / 20% previous)
- Invalid readings (>= 8190 mm) rejected, previous value kept
- Effective loop rate: ~12 Hz (33ms read + compute + send + 50ms delay)

Reducing the timing budget to 20 ms increases noise by ~28% — not worth it for closed-loop control.

---

## License & Credits

Built for UAI's MIN215 control systems course. Firmware version: `2.5.0-mobile`.

## v2.4.2 — step-mode brake phase

The step-response experiment gains an optional stop phase, configured over the
WebSocket before `start` (backward compatible; default is the legacy stop):

| Command | Values | Meaning |
|---------|--------|---------|
| `brk_mode` | 0 / 1 / 2 | 0 = legacy stop (TB6612 short brake), 1 = coast (outputs high-Z), 2 = opposite-direction torque |
| `brk_pwm`  | 0–1023   | torque for `brk_mode` 2 (`brk_pwm` 0 = short brake with telemetry) |
| `brk_ms`   | seconds  | brake-phase duration (max 5 s) |

Telemetry keeps streaming through the brake phase (`dir` reports the live motor
direction), which makes matched-speed stopping-distance comparisons per direction
possible — the diagnostic that separates drivetrain, driver, and sensing effects.

## v2.5.0 — link-quality logging

- Every data frame (`"t":"d"`) carries `seq`, a per-boot counter. Gaps in `seq` seen by the phone are frames the ESP32
  dropped because the phone's WebSocket queue was full.
- The dashboard keeps an event log: page load/reload, WebSocket connect/disconnect, mode switches, experiment
  start/stop, parameter changes, exports, `seq` gaps and resets, and an automatic latency sample every 30 s.
- **Export log** writes it as CSV with columns
  `idx,wall_time_iso,session_ms,conn,event,key,old_value,new_value,frames_rx,frames_dropped,rtt_ms`.
- Both exports (run CSV and event log) start with four `#` header lines: `train_id`, `fw`, `dashboard_schema`,
  `exported_at`. Analysis scripts must skip lines starting with `#`.
- After an export, the next page load starts a fresh log instead of appending.

---

## Preparing the trains for a lab session

One person does this before the lab, not in the room. Use the same firmware version on every train for the whole term.

1. `git pull` this repository.
2. For each train: upload LittleFS (step 3), then the firmware (step 4), with *Erase All Flash* **Disabled**.
3. If the train is new, or its ID was lost, configure it (step 5): trains are `trainA` … `trainN`, one letter per
   train, matching the label on the chassis.
4. Run the acceptance test below. Mark the train as ready only if every item passes.
5. Charge the batteries.

### Acceptance test (about 3 minutes per train)

With the train on the track, switch ON, an object 18–28 cm in front of the sensor:

- [ ] The phone sees the WiFi network `TrenCE-trainX` with the right letter, and `http://192.168.4.1` loads.
- [ ] The badge at the top of the dashboard shows `fw 2.5.0-mobile`.
- [ ] Deadband tab: one sweep forward at the default threshold ends with a PWM value in the result box.
- [ ] Step tab: amplitude 1.2 V, duration 1 s, forward: the train moves and then brakes.
- [ ] **Save CSV** downloads a file. Its first lines are `# train_id: trainX` and `# fw: 2.5.0-mobile`.
- [ ] **Export log** downloads `registro_trainX_....csv` with the same `#` header lines.
- [ ] Switch OFF.

If the dashboard does not load but the WiFi network appears, the LittleFS upload is missing: repeat step 3.

### Bench test of a new firmware version (once, on two trains)

Run this once whenever the firmware version changes, before flashing the whole fleet.

- [ ] **Frames under load:** run PID for 2+ minutes while walking the phone away or shielding it. Export the log:
      `seq_gap` rows appear (at most one per second), `frames_dropped` is plausible, and `frames_rx` roughly matches the
      Packets counter.
- [ ] **Frame integrity:** with Kp/Ki/Kd at non-round values (e.g. 2.55), the browser console shows no
      `WS parse error`.
- [ ] **Reconnect:** turn the phone's WiFi off and on during a run. The log shows `ws_disconnect`, then `ws_connect`,
      `info` and `exp_idle` (the train stops for safety), and `frames_dropped` does not jump by the length of the outage.
- [ ] **Train reboot:** power-cycle the train with the page open. The log shows `seq_reset`, not a burst of drops.
- [ ] **Automatic latency:** leave the page idle 2 minutes. About four `rtt_sample` rows appear, with values close to a
      manual **Measure Latency** run.
- [ ] **Log survives the tab:** generate some events, close the tab from the app switcher, reopen
      `http://192.168.4.1`. The log is restored with a `page_reload` row, and the frame counters continue.
- [ ] **Two trains, one phone:** connect to train A, then join train B's network and reload. The `info` rows show the
      change of train.
- [ ] **Exports:** run CSV and event log both start with the four `#` lines; the download works and the copy panel
      works.
- [ ] **Parse check:** read both exported files with the analysis script (skip `#` lines, check `seq` increases within
      each connection, recount gaps against `frames_dropped`, join the two files by train ID) before the first lab.
- [ ] **OTA update:** with a newer release published, `OTA_CHECK` installs it, the badge shows the new version, and
      the train still starts an experiment normally.
