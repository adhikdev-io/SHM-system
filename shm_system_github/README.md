# SHM System — Structural Health Monitoring Node (ESP32 Revision)

ESP32-WROOM-32 + MPU-6050 + SSD1306 OLED + **internal Bluetooth Classic
SPP** (no HC-05), zero-point-calibrated, streaming CSV telemetry @ 10 Hz
to a CustomTkinter command-center dashboard with real-time Matplotlib
plots. The telemetry protocol is unchanged from the Uno/HC-05 revision —
the Python app carries over as-is.

```
shm_system/
├── firmware/SHM_Firmware/SHM_Firmware.ino         -> legacy Uno + HC-05 build
├── firmware/SHM_Firmware_ESP32/SHM_Firmware_ESP32.ino  -> ESP32 build (current)
├── command_center/shm_dashboard.py                -> Python dashboard (current)
├── command_center/shm-start                       -> Linux quick-start launcher
├── command_center/requirements.txt
└── README.md
```

---

## 1. Wiring Map (ESP32-WROOM-32 DevKit)

Shared I2C bus on the standard ESP32 pins: **SDA = GPIO 21, SCL = GPIO 22**.
MPU-6050 uses address `0x68` (AD0 low), OLED uses `0x3C`. Bus clock 400 kHz
is set in firmware: `Wire.begin(21, 22, 400000)`.

Everything runs at 3.3 V logic — the ESP32 GPIOs are **not 5 V tolerant**,
and the peripherals below are safe on 3.3 V.

### MPU-6050 (GY-521)

| MPU-6050 | ESP32 | Note                              |
|----------|-------|-----------------------------------|
| VCC      | 3V3   | Node logic supply                 |
| GND      | GND   |                                   |
| SCL      | GPIO 22 | Standard I2C clock              |
| SDA      | GPIO 21 | Standard I2C data               |
| XDA      | —     | Not connected                     |
| XCL      | —     | Not connected                     |
| AD0      | GND   | Fixes I2C address at 0x68         |
| INT      | —     | Not connected (polling loop)      |

### 0.96" SSD1306 OLED (I2C)

| OLED | ESP32 | Note                          |
|------|-------|-------------------------------|
| GND  | GND   |                               |
| VCC  | 3V3   |                               |
| SCL  | GPIO 22 | Shared bus with MPU-6050    |
| SDA  | GPIO 21 | Shared bus with MPU-6050    |

### Bluetooth

No external module. The radio is on-chip (`BluetoothSerial.h`); the node
advertises as **`SHM-NODE-01`**. En is controlled by the core; no wiring.

### Power

| Source                     | ESP32  |
|----------------------------|--------|
| USB 5 V (programming + run)| micro-USB |
| 7–12 V (e.g. 9 V battery)  | VIN / GND |
| Regulated 5 V (power bank, buck from 9 V) | V5 (5V) / GND |

For battery deployments prefer a 9 V → 5 V buck regulator into the 5V pin
(the onboard LDO on VIN at 9 V runs hot and costs runtime). A single PP3
cell otherwise works for bench bring-up.

---

## 2. Libraries (Arduino IDE)

Board: **ESP32 Dev Module** (or matching DevKit definition), ESP32 core
2.x+ with Bluetooth enabled.

| Library          | Source                                      |
|------------------|---------------------------------------------|
| MPU6050          | Library Manager — "MPU6050 by Electronic Cats" (i2cdevlib) |
| Adafruit SSD1306 | Library Manager — Adafruit                  |
| Adafruit GFX     | Library Manager — Adafruit                  |
| BluetoothSerial / Preferences / Wire | Bundled with the ESP32 core |

Upload `firmware/SHM_Firmware_ESP32/SHM_Firmware_ESP32.ino` at the usual
115200 upload rate. While USB is used for flashing, telemetry runs over
the internal Bluetooth — USB serial stays free for debugging.

### Dual-core task layout

| Core | Task      | Role                                              |
|------|-----------|---------------------------------------------------|
| 1 (APP) | SHM-SAMP | 200 Hz fixed-phase IMU sampling + filter math (priority 3) |
| 1 (APP) | SHM-TX   | 10 Hz CSV emission, event drain, alarm debounce, command service (priority 2) |
| 0 (PRO) | SHM-OLED | 10 Hz OLED render, isolated from the sample grid (priority 1) |

Shared I2C traffic is serialized by a bus mutex; state crosses tasks via
a mutex-protected snapshot, and every `SerialBT` write originates from
exactly one task (SHM-TX), so the OLED burst transfer can never corrupt
MPU-6050 reads or interleave telemetry. `vTaskDelayUntil` keeps the
sample grid jitter-free — no `delay()`.

---

## 3. Pairing the ESP32 (Bluetooth SPP)

The node boots into **Bluetooth Classic data mode**, device name
`SHM-NODE-01`. Pairing is standard Bluetooth (secure simple pairing; if a
PIN is requested use `1234`, fallback `0000`).

### Windows
1. Settings → Bluetooth & devices → Add device → select `SHM-NODE-01`
   (accept PIN if prompted).
2. Device Manager → Ports (COM & LPT) → *Standard Serial over Bluetooth
   link (COMx)*. If two ports appear, use the **outgoing** one.
3. Verify: `py -m serial.tools.list_ports` (pyserial installed).

### Linux
```bash
bluetoothctl
  scan on
  pair C8:F0:9E:XX:XX:XX      # MAC from scan output
  trust C8:F0:9E:XX:XX:XX
  scan off
sudo rfcomm bind rfcomm0 C8:F0:9E:XX:XX:XX    # -> /dev/rfcomm0
sudo usermod -aG dialout $USER                # serial permissions (re-login)
```
`/dev/rfcomm0` then appears as a serial port in the dashboard dropdown.

### macOS
Pair via System Settings (PIN if prompted); the device appears as
`/dev/tty.SHM-NODE-01-PS` (or similar `tty.*` entry).

Note: the COM port only exists **after** pairing — connect the port in
the dashboard, not before.

---

## 4. Python Dashboard

```bash
cd command_center
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python shm_dashboard.py
```

### Quick start (Linux, node pre-paired)

`command_center/shm-start` is a convenience launcher that binds the
Bluetooth SPP port (`rfcomm bind`), re-pairs the node if needed, and
starts the dashboard in one step:

```bash
chmod +x shm-start
./shm-start
```

Edit the `MAC=` line at the top with your node's address (find it via
`bluetoothctl devices` after powering the node).

Operation:
1. **REFRESH** → select the SPP COM port (device must be paired) →
   **CONNECT**. The baud field is ignored for Bluetooth SPP; keep 9600.
2. **APPLY PROFILE** sends `SET:TILT:` / `SET:VIB:` to the node and
   re-draws the threshold overlays.
3. **CALIBRATE ZERO-POINT** sends `CAL`; hold the structure at rest for
   1 s until the log reports `ZERO-POINT BASELINE WRITTEN`.
4. Health badge: emerald `STABLE` ⇒ crimson `THRESHOLD EXCEEDED`
   (device-side debounce: 250 ms entry / 1 s release).

Changes vs. the HC-05 revision are cosmetic only: ESP32/BT-SPP labels,
the baud-is-ignored note, and the connect log wording. Protocol,
thresholds, plotting, and threading logic are untouched.

---

## 5. Telemetry & Control Protocol

Frame, LF-terminated, 10 Hz (emitted only while an SPP client is connected):

```
<t_ms>,<tilt_x_deg>,<tilt_y_deg>,<vib_rms_g>,<state>
12451,-0.12,0.05,0.031,0        state 0 = STABLE, 1 = THRESHOLD EXCEEDED
```

Device lines: `#SHM-NODE,1.0,ESP32,BT-SPP` (boot), `EVT:*`, `OK:*`, `ERR:*`.
Commands: `CAL`, `PING`, `VER`, `SET:TILT:<deg>` (0.20–45.0),
`SET:VIB:<g>` (0.05–5.00).

---

## 6. Calibration Method (zero-point baseline)

Captured over 1 s (200 samples) at rest, auto-run at power-on if no
stored profile exists, and persisted to NVS partition `shm` (magic
`0xA55A` — survives reboots without EEPROM emulation).

**Tilt zero-point.** With the accelerometer at rest the reading ~ gravity:

```
θ0 = atan2( āx , √(āy² + āz²) )      φ0 = atan2( āy , √(āx² + āz²) )
```
where ā = mean acceleration (g). Live tilt is the low-pass-filtered
inclination minus the baseline:

```
θ = atan2(ax, √(ay²+az²)) − θ0       φ = atan2(ay, √(ax²+az²)) − φ0
TILT_EMA:  θf ← θf + 0.20 (θ − θf)   @ 200 Hz  (τ ≈ 22 ms)
```

**Vibration (dynamic load).** High-pass by DC removal, then windowed RMS:

```
ās ← ās + α (a − ās),   α = Δt/(τ+Δt),  Δt = 5 ms, τ = 0.5 s
a_dyn = a − ās
V_RMS = √( (1/N) Σ ‖a_dyn,i‖² ),  N = 40 samples (200 ms), units: g RMS
```

**Integrity gates at calibration** (capture is rejected otherwise):
- mean |ā| within 0.85–1.15 g (MEMS sanity / sensor mounted correctly),
- gyro bias < 60 dps (structure stayed still during capture).

Gyro bias is stored (`±250 dps` scale, 131 LSB/dps) for sensor-health
audits. Slow thermal drift of the zero-point is absorbed by the DC
tracker (τ = 0.5 s) for vibration and by per-deployment re-calibration
for static tilt.

---

## 7. OLED Dashboard (on-site)

```
┌────────────────────────────────┐
│ SHM NODE                 SPP   │     SPP = internal Bluetooth link
│ SYS STATUS: STABLE             │     CRITICAL = inverted block
│ TILT X:0.00 Y:0.00             │
│ VIB: 0.031 g                   │
│ TH V:0.30 T:2.5                │
│ UP 0:02:45 CAL OK              │     ERR / NO FEED on sensor fault
└────────────────────────────────┘
```
Static render at 10 Hz — no blinking, no animation, in line with the
"visual data only" constraint.
