// =====================================================================
//  SHM_Firmware_ESP32.ino  --  Structural Health Monitoring Node
//  Target : ESP32-WROOM-32 (DevKit V1, 38-pin)
//  Sensor : MPU-6050 (0x68)  |  Display : SSD1306 0.96" I2C (0x3C)
//  Link   : Internal Bluetooth Classic SPP ("BluetoothSerial.h"),
//           no external HC-05 module
//  Power  : 5 V USB / power bank, or 7-12 V -> VIN
// ---------------------------------------------------------------------
//  TELEMETRY (protocol identical to the Uno/HC-05 revision -- the Python
//  Command Center carries over without a protocol rewrite):
//      <t_ms>,<tilt_x_deg>,<tilt_y_deg>,<vib_rms_g>,<state>
//      state : 0 = STABLE, 1 = THRESHOLD EXCEEDED            @ 10 Hz
//
//  EVENTS / CONTROL (ASCII, LF terminated):
//      EVT:* | OK:* | ERR:* lines, commands: CAL | PING | VER
//                                 SET:TILT:<deg> | SET:VIB:<g>
//
//  DUAL-CORE DEPLOYMENT (FreeRTOS):
//      core 1 (APP)  SHM-SAMP : 200 Hz IMU sampling + filtering,
//                               vTaskDelayUntil fixed-phase grid
//                    SHM-TX   : 10 Hz CSV emission, event drain,
//                               alarm debounce, command service
//      core 0 (PRO)  SHM-OLED : 10 Hz static OLED render (isolated from
//                               the sample grid by a mutex + snapshot)
//      BT stack tasks remain scheduled by the framework on core 0.
//  All I2C traffic funnels through one bus mutex so the 1 kB OLED burst
//  can never corrupt an MPU-6050 read; shared state publishes through a
//  mutex-protected snapshot. No delay() anywhere.
// =====================================================================

#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE

#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <I2Cdev.h>
#include <MPU6050.h>             // "MPU6050" by Electronic Cats (i2cdevlib)
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Select an ESP32 board definition with Bluetooth support.
#endif

// ------------------------- hardware map -------------------------------
static const uint8_t PIN_SDA   = 21;   // standard ESP32 I2C
static const uint8_t PIN_SCL   = 22;
static const uint8_t OLED_ADDR = 0x3C;

static Adafruit_SSD1306  oled(128, 64, &Wire, -1);
static MPU6050           imu;          // AD0 -> GND = 0x68
static BluetoothSerial   SerialBT;

static const char* BT_DEVICE_NAME = "SHM-NODE-01";

// ------------------------- configuration ------------------------------
// (identical values to the Uno revision)
static const float    LSB_PER_G    = 16384.0f;   // +/-2 g full scale
static const float    LSB_PER_DPS  = 131.0f;     // +/-250 dps
static const float    RAD_TO_DEG_F = 57.29578f;

static const uint16_t PERIOD_SAMPLE_MS = 5;        // 200 Hz
static const uint16_t PERIOD_SEND_MS   = 100;      // 10 Hz telemetry
static const uint16_t PERIOD_OLED_MS   = 100;      // dashboard refresh
static const uint16_t VIB_WINDOW_MS    = 200;      // RMS window
static const uint16_t IMU_RETRY_MS     = 2000;

static const uint8_t  VIB_RING_CAP = 48;
static const uint8_t  VIB_WINDOW_N = VIB_WINDOW_MS / PERIOD_SAMPLE_MS; // 40

static const uint16_t ALARM_SET_MS     = 250;      // debounce into CRITICAL
static const uint16_t ALARM_RELEASE_MS = 1000;     // debounce into STABLE

static const float    TILT_EMA_A = 0.20f;                       // tilt low-pass
static const float    VIB_DC_A   = 0.005f / (0.500f + 0.005f);  // tau = 0.5 s

static const float    DEF_TILT_THR = 2.50f;      // degrees
static const float    DEF_VIB_THR  = 0.30f;      // g RMS
static const float    TILT_THR_MIN = 0.20f, TILT_THR_MAX = 45.0f;
static const float    VIB_THR_MIN  = 0.05f, VIB_THR_MAX  = 5.0f;

static const uint8_t  CAL_SAMPLES = 200;           // 1.0 s @ 200 Hz
static const uint16_t NVS_MAGIC   = 0xA55A;
static const char*    NVS_NS      = "shm";
static const char*    NVS_KEY     = "cal";

struct Calibration {
  uint16_t magic;
  float    tiltX0;      // zero-point tilt X [deg]
  float    tiltY0;      // zero-point tilt Y [deg]
  int16_t  gyroBias[3]; // zero-point gyro   [LSB]
};

// Published state (producer: SHM-SAMP; consumers: SHM-TX / SHM-OLED)
struct NodeState {
  bool    calValid;
  bool    imuOk;
  float   tiltX;      // baseline-relative, filtered [deg]
  float   tiltY;
  float   vibRms;     // 200 ms window [g]
  uint8_t alert;      // 0 STABLE / 1 CRITICAL
};

// ------------------------- runtime state ------------------------------
static Calibration cal;
static bool  calValid = false;
static volatile float tiltThr = DEF_TILT_THR;
static volatile float vibThr  = DEF_VIB_THR;

static bool  imuOk  = false;
static bool  oledOk = false;
static bool  dcInit = false;
static bool  autoCalDone = false;

static float tiltXF = 0.0f, tiltYF = 0.0f;     // SHM-SAMP only
static float aSlow[3];                          // SHM-SAMP only
static float vibRing[VIB_RING_CAP];             // SHM-SAMP only
static uint8_t vibHead = 0, vibFill = 0, vibCount = 0;
static float vibRms = 0.0f;                     // SHM-SAMP only

static uint32_t alarmT0 = 0, clearT0 = 0;       // SHM-TX only
static uint32_t tImuRetry = 0;                  // SHM-SAMP only
static bool     calActive = false;              // SHM-SAMP only
static uint8_t  calN = 0;
static float    calSx = 0, calSy = 0, calSm = 0;
static float    calGx = 0, calGy = 0, calGz = 0;
static char     cmdBuf[20];                     // SHM-TX only
static uint8_t  cmdIdx = 0;

static NodeState live;               // guarded by stateMux
static SemaphoreHandle_t stateMux;   // live <-> consumers
static SemaphoreHandle_t i2cMux;     // serializes Wire bus access

// Event pipe: all SerialBT output originates from SHM-TX only
enum DevEvent : uint8_t {
  EV_IMU_FAULT, EV_IMU_REST, EV_ANOMALY, EV_STABLE,
  EV_CAL_START, EV_BASELINE, EV_CAL_GRAVITY, EV_CAL_BIAS
};
static QueueHandle_t evtQ;

// ------------------------- small helpers ------------------------------
static float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// dtostrf(v, width=1, prec) -> unpadded "-1.23" / "0.031"
static void fmt(char* out, float v, uint8_t prec) {
  dtostrf(v, 1, prec, out);
}

static void emitEvent(DevEvent e) {
  xQueueSend(evtQ, &e, 0);
}

static inline void i2cLock()   { xSemaphoreTake(i2cMux, portMAX_DELAY); }
static inline void i2cUnlock() { xSemaphoreGive(i2cMux); }

static void publish(const NodeState& s) {
  xSemaphoreTake(stateMux, portMAX_DELAY);
  live = s;
  xSemaphoreGive(stateMux);
}

static NodeState snapshot() {
  xSemaphoreTake(stateMux, portMAX_DELAY);
  NodeState s = live;
  xSemaphoreGive(stateMux);
  return s;
}

static uint8_t getAlertUnlocked() {   // small read, mutex-guarded
  xSemaphoreTake(stateMux, portMAX_DELAY);
  uint8_t v = live.alert;
  xSemaphoreGive(stateMux);
  return v;
}

static void patchAlert(uint8_t alert) {  // SHM-TX only -> live.alert
  xSemaphoreTake(stateMux, portMAX_DELAY);
  live.alert = alert;
  xSemaphoreGive(stateMux);
}

// ======================================================================
//  IMU
// ======================================================================
static bool imuStart() {
  i2cLock();
  imu.initialize();
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  imu.setDLPFMode(MPU6050_DLPF_BW_98);
  i2cUnlock();
  return true;   // scanner already confirmed 0x68 is alive
}

// ======================================================================
//  CALIBRATION  (zero-point baseline, NVS persistence)
// ======================================================================
static void startCalibration() {      // caller: SHM-SAMP or SHM-TX
  if (calActive) return;
  calSx = calSy = calSm = calGx = calGy = calGz = 0;
  calN      = 0;
  calActive = true;
  emitEvent(EV_CAL_START);
}

static void finishCalibration(float lastRx, float lastRy) {  // SHM-SAMP
  calActive = false;

  float x0  = calSx / CAL_SAMPLES;               // baseline tilt X [deg]
  float y0  = calSy / CAL_SAMPLES;               // baseline tilt Y [deg]
  float mag = calSm / CAL_SAMPLES;               // mean |a| [g]

  float bx = calGx / CAL_SAMPLES / LSB_PER_DPS;  // gyro bias [dps]
  float by = calGy / CAL_SAMPLES / LSB_PER_DPS;
  float bz = calGz / CAL_SAMPLES / LSB_PER_DPS;

  // Integrity gate 1: structural mass must be at 1 g static (at rest).
  if (fabsf(mag - 1.0f) > 0.15f) { emitEvent(EV_CAL_GRAVITY); return; }
  // Integrity gate 2: node must not move during capture (< 60 dps).
  float bmax = fabsf(bx);
  if (fabsf(by) > bmax) bmax = fabsf(by);
  if (fabsf(bz) > bmax) bmax = fabsf(bz);
  if (bmax > 60.0f) { emitEvent(EV_CAL_BIAS); return; }

  cal.tiltX0 = x0;
  cal.tiltY0 = y0;
  cal.gyroBias[0] = (int16_t)(calGx / CAL_SAMPLES);
  cal.gyroBias[1] = (int16_t)(calGy / CAL_SAMPLES);
  cal.gyroBias[2] = (int16_t)(calGz / CAL_SAMPLES);
  cal.magic = NVS_MAGIC;

  Preferences prefs;
  prefs.begin(NVS_NS, false);
  prefs.putBytes(NVS_KEY, &cal, sizeof(cal));
  prefs.end();

  tiltXF = lastRx;   // restart filters at the new zero reference
  tiltYF = lastRy;
  dcInit = false;
  calValid = true;
  emitEvent(EV_BASELINE);
}

// ======================================================================
//  SAMPLING  (SHM-SAMP, core 1, 200 Hz fixed phase)
// ======================================================================
static void sampleStep() {
  // ---- IMU fault recovery path
  if (!imuOk) {
    uint32_t now = millis();
    if ((now - tImuRetry) >= IMU_RETRY_MS) {
      tImuRetry = now;
      if (imuStart()) {
        imuOk = true;
        dcInit = false;
        autoCalDone = false;
        emitEvent(EV_IMU_REST);
      }
    }
  }

  // ---- power-on auto zero-point (node must be at rest)
  if (imuOk && !calValid && !calActive && !autoCalDone && millis() > 500) {
    autoCalDone = true;
    startCalibration();
  }

  if (!imuOk) return;

  int16_t rax, ray, raz, rgx, rgy, rgz;
  i2cLock();
  imu.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);
  i2cUnlock();

  float fx = rax / LSB_PER_G;
  float fy = ray / LSB_PER_G;
  float fz = raz / LSB_PER_G;

  // ---- vibration: DC-removal high pass (tau = 0.5 s) -> ring buffer
  if (!dcInit) {
    aSlow[0] = fx; aSlow[1] = fy; aSlow[2] = fz;
    dcInit = true;
  } else {
    aSlow[0] += VIB_DC_A * (fx - aSlow[0]);
    aSlow[1] += VIB_DC_A * (fy - aSlow[1]);
    aSlow[2] += VIB_DC_A * (fz - aSlow[2]);
  }
  float dx = fx - aSlow[0];
  float dy = fy - aSlow[1];
  float dz = fz - aSlow[2];
  float joint = sqrtf(dx * dx + dy * dy + dz * dz);

  vibRing[vibHead] = joint;
  vibHead = (uint8_t)((vibHead + 1) % VIB_RING_CAP);
  if (vibFill < VIB_RING_CAP) vibFill++;

  // ---- tilt: gravity inclination (static), degrees
  float rx = RAD_TO_DEG_F * atan2f(fx, sqrtf(fy * fy + fz * fz));
  float ry = RAD_TO_DEG_F * atan2f(fy, sqrtf(fx * fx + fz * fz));

  if (calActive) {
    calSx += rx;  calSy += ry;
    calSm += sqrtf(fx * fx + fy * fy + fz * fz);
    calGx += rgx; calGy += rgy; calGz += rgz;
    if (++calN >= CAL_SAMPLES) finishCalibration(rx, ry);
    return;
  }

  if (calValid) {
    tiltXF += TILT_EMA_A * (rx - tiltXF);
    tiltYF += TILT_EMA_A * (ry - tiltYF);
  }

  // every 40 samples: refresh the 200 ms RMS window
  if (++vibCount >= VIB_WINDOW_N) {
    vibCount = 0;
    uint8_t n = vibFill;
    if (n > VIB_WINDOW_N) n = VIB_WINDOW_N;
    float acc = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
      uint8_t k = (uint8_t)((vibHead + VIB_RING_CAP - 1 - i) % VIB_RING_CAP);
      float v = vibRing[k];
      acc += v * v;
    }
    vibRms = (n > 0) ? sqrtf(acc / n) : 0.0f;
  }

  NodeState s;
  s.calValid = calValid;
  s.imuOk    = imuOk;
  s.tiltX    = calValid ? (tiltXF - cal.tiltX0) : 0.0f;
  s.tiltY    = calValid ? (tiltYF - cal.tiltY0) : 0.0f;
  s.vibRms   = vibRms;
  s.alert    = getAlertUnlocked();   // alert ownership: SHM-TX
  publish(s);
}

// ======================================================================
//  TELEMETRY / EVENT DRAIN / COMMAND SERVICE  (SHM-TX, core 1, 10 Hz)
// ======================================================================
static void ackSet(const char* axis, float v) {
  char b[10], line[24];
  fmt(b, v, 2);
  snprintf(line, sizeof(line), "OK:SET,%s,%s", axis, b);
  SerialBT.println(line);
}

static void handleCommand(char* c) {
  if (strcmp(c, "CAL") == 0) {
    if (!imuOk) { SerialBT.println(F("ERR:CAL:NO-IMU")); return; }
    startCalibration();
  } else if (strcmp(c, "PING") == 0) {
    SerialBT.println(F("OK:PING"));
  } else if (strcmp(c, "VER") == 0) {
    SerialBT.println(F("OK:SHM-NODE,1.0,ESP32,BT-SPP"));
  } else if (strncmp(c, "SET:TILT:", 9) == 0) {
    tiltThr = clampf(atof(c + 9), TILT_THR_MIN, TILT_THR_MAX);
    ackSet("TILT", tiltThr);
  } else if (strncmp(c, "SET:VIB:", 8) == 0) {
    vibThr = clampf(atof(c + 8), VIB_THR_MIN, VIB_THR_MAX);
    ackSet("VIB", vibThr);
  } else {
    SerialBT.println(F("ERR:SYNTAX"));
  }
}

static void serviceCommands() {       // non-blocking line reader
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (cmdIdx) {
        cmdBuf[cmdIdx] = '\0';
        handleCommand(cmdBuf);
        cmdIdx = 0;
      }
    } else if (cmdIdx < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdIdx++] = c;
    } else {
      cmdIdx = 0;   // overflow: drop line, protect stream integrity
    }
  }
}

static void serviceEvents() {         // drain event pipe (single writer)
  uint8_t e;
  while (xQueueReceive(evtQ, &e, 0) == pdTRUE) {
    switch ((DevEvent)e) {
      case EV_IMU_FAULT:   SerialBT.println(F("EVT:IMU:FAULT"));  break;
      case EV_IMU_REST:    SerialBT.println(F("EVT:IMU:REST"));   break;
      case EV_ANOMALY:     SerialBT.println(F("EVT:ANOMALY"));    break;
      case EV_STABLE:      SerialBT.println(F("EVT:STABLE"));     break;
      case EV_CAL_START:   SerialBT.println(F("EVT:CAL:START"));  break;
      case EV_BASELINE:    SerialBT.println(F("OK:CAL"));
                           SerialBT.println(F("EVT:BASELINE-SET")); break;
      case EV_CAL_GRAVITY: SerialBT.println(F("ERR:CAL:GRAVITY")); break;
      case EV_CAL_BIAS:    SerialBT.println(F("ERR:CAL:BIAS"));   break;
    }
  }
}

static void broadcastStep(uint32_t now) {
  serviceCommands();
  serviceEvents();

  NodeState s = snapshot();

  // ---- alarm logic (10 Hz evaluation, debounced)
  bool alarm = s.calValid &&
      (fabs(s.tiltX) >= tiltThr ||
       fabs(s.tiltY) >= tiltThr ||
       s.vibRms    >= vibThr);
  if (alarm) {
    clearT0 = 0;
    if (alarmT0 == 0) alarmT0 = now;
    if (getAlertUnlocked() == 0 && (now - alarmT0) >= ALARM_SET_MS) {
      patchAlert(1);
      emitEvent(EV_ANOMALY);
    }
  } else {
    alarmT0 = 0;
    if (getAlertUnlocked() == 1) {
      if (clearT0 == 0) clearT0 = now;
      else if ((now - clearT0) >= ALARM_RELEASE_MS) {
        patchAlert(0);
        clearT0 = 0;
        emitEvent(EV_STABLE);
      }
    }
  }

  // ---- CSV emission  (only with an active SPP subscriber)
  if (s.calValid && SerialBT.hasClient()) {
    char b0[12], b1[12], b2[12], line[44];
    fmt(b0, s.tiltX, 2);
    fmt(b1, s.tiltY, 2);
    fmt(b2, s.vibRms, 3);
    snprintf(line, sizeof(line), "%lu,%s,%s,%s,%u",
             (unsigned long)now, b0, b1, b2, (unsigned int)s.alert);
    SerialBT.print(line);
    SerialBT.print('\n');
  }
}

// ======================================================================
//  OLED DASHBOARD  (SHM-OLED, core 0, static render -- no animation)
// ======================================================================
static void drawSplash() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 20);
  oled.print(F("SHM NODE"));
  oled.setCursor(0, 32);
  oled.print(F("INITIALIZING"));
  oled.display();
}

static void drawDashboard(const NodeState& s) {
  if (!oledOk) return;

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // ---- header row
  oled.setCursor(0, 0);
  oled.print(F("SHM NODE"));
  oled.setCursor(96, 0);
  oled.print(s.imuOk ? F("SPP") : F("ERR"));

  // ---- status row  (SYS STATUS: STABLE / CRITICAL)
  oled.setCursor(0, 11);
  oled.print(F("SYS STATUS: "));
  if (s.alert == 0) {
    oled.print(F("STABLE"));
  } else {
    uint8_t x = oled.getCursorX();
    uint8_t y = oled.getCursorY();
    oled.fillRect(x - 1, y - 1, 50, 9, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.print(F("CRITICAL"));
    oled.setTextColor(SSD1306_WHITE);
  }

  // ---- tilt row   TILT X:0.00 Y:0.00
  char b0[12], b1[12], b2[12];
  oled.setCursor(0, 22);
  oled.print(F("TILT X:"));
  dtostrf(s.tiltX, 1, 2, b0);
  oled.print(b0);
  oled.print(F(" Y:"));
  dtostrf(s.tiltY, 1, 2, b1);
  oled.print(b1);

  // ---- vibration row   VIB: 0.031 g
  oled.setCursor(0, 33);
  oled.print(F("VIB: "));
  dtostrf(s.vibRms, 1, 3, b2);
  oled.print(b2);
  oled.print(F(" g"));

  // ---- threshold row   TH V:0.30 T:2.5
  oled.setCursor(0, 44);
  oled.print(F("TH V:"));
  dtostrf(vibThr, 1, 2, b0);
  oled.print(b0);
  oled.print(F(" T:"));
  dtostrf(tiltThr, 1, 1, b1);
  oled.print(b1);

  // ---- footer row
  oled.setCursor(0, 55);
  if (!s.calValid) {
    oled.print(F("AWAIT BASELINE"));
  } else {
    uint32_t ss_raw = millis() / 1000UL;
    uint8_t  h  = (uint8_t)(ss_raw / 3600UL);
    uint8_t  m  = (uint8_t)((ss_raw / 60UL) % 60UL);
    uint8_t  ss = (uint8_t)(ss_raw % 60UL);
    oled.print(F("UP "));
    oled.print(h);
    oled.print(':');
    if (m < 10) oled.print('0');
    oled.print(m);
    oled.print(':');
    if (ss < 10) oled.print('0');
    oled.print(ss);
    oled.print(F(" CAL OK"));
  }

  i2cLock();
  oled.display();          // 1 kB burst; exclusive bus access
  i2cUnlock();
}

// ======================================================================
//  TASKS
// ======================================================================
static void shmSampleTask(void*) {    // core 1: fixed-phase 200 Hz grid
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_SAMPLE_MS));
    sampleStep();
  }
}

static void shmTxTask(void*) {        // core 1: 10 Hz telemetry
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_SEND_MS));
    broadcastStep(millis());
  }
}

static void shmOledTask(void*) {      // core 0: OLED isolated here
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_OLED_MS));
    drawDashboard(snapshot());
  }
}

// ======================================================================
//  SETUP / LOOP
// ======================================================================
void setup() {
  stateMux = xSemaphoreCreateMutex();
  i2cMux   = xSemaphoreCreateMutex();
  evtQ     = xQueueCreate(8, sizeof(uint8_t));

  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) drawSplash();

  SerialBT.begin(BT_DEVICE_NAME);
  SerialBT.println(F("#SHM-NODE,1.0,ESP32,BT-SPP"));

  Preferences prefs;
  prefs.begin(NVS_NS, true);          // read-only session
  size_t got  = prefs.getBytes(NVS_KEY, &cal, sizeof(cal));
  calValid = (got == sizeof(cal)) && (cal.magic == NVS_MAGIC);
  prefs.end();

   imuOk = imuStart();
  Serial.begin(115200);
  Serial.print(F("IMU init: "));
  Serial.println(imuOk ? F("OK") : F("FAIL"));
  if (!imuOk) emitEvent(EV_IMU_FAULT);
  tImuRetry = millis();

  // ---- dual-core deployment
  xTaskCreatePinnedToCore(shmSampleTask, "SHM-SAMP", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(shmTxTask,     "SHM-TX",   4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(shmOledTask,   "SHM-OLED", 4096, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));   // service tasks own all scheduling
}