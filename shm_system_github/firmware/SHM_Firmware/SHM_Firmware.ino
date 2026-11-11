// =====================================================================
//  SHM_Firmware.ino  --  Structural Health Monitoring Node
//  Target : Arduino Uno R3
//  Sensor : MPU-6050 (0x68)  |  Display : SSD1306 0.96" I2C (0x3C)
//  Link   : HC-05 Bluetooth SPP via SoftwareSerial (9600 8N1)
//  Power  : 9 V battery -> VIN / barrel jack
// ---------------------------------------------------------------------
//  TELEMETRY FRAME (10 Hz CSV over SPP, LF terminated):
//      <t_ms>,<tilt_x_deg>,<tilt_y_deg>,<vib_rms_g>,<state>
//      state : 0 = STABLE, 1 = THRESHOLD EXCEEDED
//
//  EVENTS / CONTROL (ASCII, LF terminated):
//      #SHM-NODE,1.0,CSV,UNO        boot banner
//      EVT:ANOMALY  EVT:STABLE      threshold state transitions
//      EVT:CAL:START  OK:CAL        baseline capture lifecycle
//      EVT:IMU:FAULT  EVT:IMU:REST  sensor availability
//      OK:PING  OK:SHM-NODE,...     handshakes
//      ERR:...                      syntax / calibration rejection
//
//      Commands :  CAL  |  PING  |  VER
//                  SET:TILT:<deg>   (0.2 .. 45.0)
//                  SET:VIB:<g>      (0.05 .. 5.00)
//
//  NOTE: no delay() anywhere in the steady state; all timing via
//  millis(). Every string literal is stored in flash (F() macro).
// =====================================================================

#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>          // "MPU6050" by Electronic Cats (i2cdevlib)

// ------------------------- hardware map -------------------------------
static const uint8_t PIN_BT_RX = 10;   // UNO D10  <-  HC-05 TXD
static const uint8_t PIN_BT_TX = 11;   // UNO D11  ->  HC-05 RXD (divider)
static const uint8_t OLED_ADDR = 0x3C;

static SoftwareSerial     bt(PIN_BT_RX, PIN_BT_TX);
static MPU6050            imu;                    // AD0 -> GND = 0x68
static Adafruit_SSD1306   oled(128, 64, &Wire, -1);

// ------------------------- configuration ------------------------------
static const uint32_t BT_BAUD         = 9600;
static const float    LSB_PER_G       = 16384.0f;    // +/-2 g full scale
static const float    LSB_PER_DPS     = 131.0f;      // +/-250 dps
static const float    RAD_TO_DEG_F    = 57.29578f;

static const uint16_t PERIOD_SAMPLE_MS = 5;          // 200 Hz sampling
static const uint16_t PERIOD_SEND_MS   = 100;        // 10 Hz telemetry
static const uint16_t PERIOD_OLED_MS   = 100;        // dashboard refresh
static const uint16_t VIB_WINDOW_MS    = 200;        // RMS window
static const uint16_t IMU_RETRY_MS     = 2000;

static const uint8_t  VIB_RING_CAP     = 48;         // ring capacity
static const uint8_t  VIB_WINDOW_N     = VIB_WINDOW_MS / PERIOD_SAMPLE_MS;

static const uint16_t ALARM_SET_MS     = 250;        // debounce into CRITICAL
static const uint16_t ALARM_RELEASE_MS = 1000;       // debounce into STABLE

static const float    TILT_EMA_A       = 0.20f;                     // tilt low-pass
static const float    VIB_DC_A         = 0.005f / (0.500f + 0.005f); // tau = 0.5 s

static const float    DEF_TILT_THR     = 2.50f;      // degrees
static const float    DEF_VIB_THR      = 0.30f;      // g (RMS)
static const float    TILT_THR_MIN     = 0.20f, TILT_THR_MAX = 45.0f;
static const float    VIB_THR_MIN      = 0.05f, VIB_THR_MAX  = 5.0f;

static const uint8_t  CAL_SAMPLES      = 200;        // 1.0 s @ 200 Hz
static const uint16_t EEPROM_MAGIC     = 0xA55A;

struct Calibration {
  uint16_t magic;
  float    tiltX0;      // zero-point pitch  [deg]
  float    tiltY0;      // zero-point roll   [deg]
  int16_t  gyroBias[3]; // zero-point gyro   [LSB]
};

// ------------------------- runtime state ------------------------------
static Calibration cal;
static bool   calValid  = false;
static float  tiltThr   = DEF_TILT_THR;
static float  vibThr    = DEF_VIB_THR;

static bool   imuOk  = false;
static bool   oledOk = false;
static bool   dcInit = false;
static bool   oledDirty = true;

static uint32_t tSample = 0, tSend = 0, tOled = 0, tImu = 0;

static float tiltXF = 0.0f, tiltYF = 0.0f;     // filtered |angle| (deg)
static float aSlow[3];                          // DC tracker (gravity ref)
static float vibRms = 0.0f;

static float   vibRing[VIB_RING_CAP];
static uint8_t vibHead = 0, vibFill = 0;

static uint8_t  state = 0;                      // 0 STABLE / 1 CRITICAL
static uint32_t alarmT0 = 0, clearT0 = 0;

static bool   calActive = false;
static uint8_t calN = 0;
static float  calSx = 0, calSy = 0, calSm = 0;
static float  calGx = 0, calGy = 0, calGz = 0;

static char    cmdBuf[20];
static uint8_t cmdIdx = 0;

// ------------------------- forward decls ------------------------------
static bool imuStart();
static void sampleStep();
static void serviceTelemetry(uint32_t now);
static void drawDashboard();
static void serviceCommands();
static void startCalibration();

// ------------------------- small helpers ------------------------------
static float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// dtostrf(v, width=1, prec) -> unpadded "-1.23" / "0.031"
static void fmt(char* out, float v, uint8_t prec) {
  dtostrf(v, 1, prec, out);
}

// ======================================================================
//  IMU
// ======================================================================
static bool imuStart() {
  imu.initialize();
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  imu.setDLPFMode(MPU6050_DLPF_BW_98);           // alias control @ 200 Hz
  return imu.testConnection();
}

// ======================================================================
//  CALIBRATION  (zero-point baseline)
// ======================================================================
static void startCalibration() {
  if (calActive) return;
  if (!imuOk) {
    bt.println(F("ERR:CAL:NO-IMU"));
    return;
  }
  calSx = calSy = calSm = calGx = calGy = calGz = 0;
  calN       = 0;
  calActive  = true;
  bt.println(F("EVT:CAL:START"));
}

static void finishCalibration(float lastRx, float lastRy) {
  calActive = false;

  float x0  = calSx / CAL_SAMPLES;               // baseline pitch  [deg]
  float y0  = calSy / CAL_SAMPLES;               // baseline roll   [deg]
  float mag = calSm / CAL_SAMPLES;               // mean |a|  [g]

  float bx = calGx / CAL_SAMPLES / LSB_PER_DPS;  // gyro bias [dps]
  float by = calGy / CAL_SAMPLES / LSB_PER_DPS;
  float bz = calGz / CAL_SAMPLES / LSB_PER_DPS;

  // Integrity gate 1: structural mass must be at 1 g static (at rest).
  if (fabsf(mag - 1.0f) > 0.15f) {
    bt.println(F("ERR:CAL:GRAVITY"));
    return;
  }
  // Integrity gate 2: node must not move during capture (< 60 dps).
  float bmax = fabsf(bx);
  if (fabsf(by) > bmax) bmax = fabsf(by);
  if (fabsf(bz) > bmax) bmax = fabsf(bz);
  if (bmax > 60.0f) {
    bt.println(F("ERR:CAL:BIAS"));
    return;
  }

  cal.tiltX0 = x0;
  cal.tiltY0 = y0;
  cal.gyroBias[0] = (int16_t)(calGx / CAL_SAMPLES);
  cal.gyroBias[1] = (int16_t)(calGy / CAL_SAMPLES);
  cal.gyroBias[2] = (int16_t)(calGz / CAL_SAMPLES);
  cal.magic = EEPROM_MAGIC;
  EEPROM.put(0, cal);

  tiltXF = lastRx;   // restart filters at the new zero reference
  tiltYF = lastRy;
  dcInit = false;
  calValid  = true;
  oledDirty = true;
  bt.println(F("OK:CAL"));
  bt.println(F("EVT:BASELINE-SET"));
}

// ======================================================================
//  SAMPLING  (200 Hz)
// ======================================================================
static void sampleStep() {
  int16_t rax, ray, raz, rgx, rgy, rgz;
  imu.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);

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
}

static float windowRms() {
  uint8_t n = vibFill;
  if (n > VIB_WINDOW_N) n = VIB_WINDOW_N;
  if (n == 0) return 0.0f;
  float acc = 0.0f;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t k = (uint8_t)((vibHead + VIB_RING_CAP - 1 - i) % VIB_RING_CAP);
    float v = vibRing[k];
    acc += v * v;
  }
  return sqrtf(acc / n);
}

// ======================================================================
//  TELEMETRY + ALARM LOGIC  (10 Hz)
// ======================================================================
static void serviceTelemetry(uint32_t now) {
  if (!calValid) return;

  vibRms = windowRms();

  bool alarm =
      fabsf(tiltXF - cal.tiltX0) >= tiltThr ||
      fabsf(tiltYF - cal.tiltY0) >= tiltThr ||
      vibRms >= vibThr;

  if (alarm) {
    clearT0 = 0;
    if (alarmT0 == 0) alarmT0 = now;
    if (state == 0 && (now - alarmT0) >= ALARM_SET_MS) {
      state = 1;
      oledDirty = true;
      bt.println(F("EVT:ANOMALY"));
    }
  } else {
    alarmT0 = 0;
    if (state == 1) {
      if (clearT0 == 0) clearT0 = now;
      else if ((now - clearT0) >= ALARM_RELEASE_MS) {
        state = 0;
        clearT0 = 0;
        oledDirty = true;
        bt.println(F("EVT:STABLE"));
      }
    }
  }

  // ---- CSV frame ------------------------------------------------------
  char b0[12], b1[12], b2[12], line[44];
  fmt(b0, tiltXF - cal.tiltX0, 2);
  fmt(b1, tiltYF - cal.tiltY0, 2);
  fmt(b2, vibRms, 3);
  snprintf(line, sizeof(line), "%lu,%s,%s,%s,%u",
           (unsigned long)now, b0, b1, b2, (unsigned int)state);
  bt.print(line);
  bt.print('\n');
}

// ======================================================================
//  COMMAND SERVICE  (non-blocking line reader)
// ======================================================================
static void ackSet(const char* axis, float v) {
  char b[10], line[24];
  fmt(b, v, 2);
  snprintf(line, sizeof(line), "OK:SET,%s,%s", axis, b);
  bt.println(line);
}

static void handleCommand(char* c) {
  if (strcmp(c, "CAL") == 0) {
    startCalibration();
  } else if (strcmp(c, "PING") == 0) {
    bt.println(F("OK:PING"));
  } else if (strcmp(c, "VER") == 0) {
    bt.println(F("OK:SHM-NODE,1.0,UNO,CSV"));
  } else if (strncmp(c, "SET:TILT:", 9) == 0) {
    tiltThr = clampf(atof(c + 9), TILT_THR_MIN, TILT_THR_MAX);
    ackSet("TILT", tiltThr);
  } else if (strncmp(c, "SET:VIB:", 8) == 0) {
    vibThr = clampf(atof(c + 8), VIB_THR_MIN, VIB_THR_MAX);
    ackSet("VIB", vibThr);
  } else {
    bt.println(F("ERR:SYNTAX"));
  }
}

static void serviceCommands() {
  while (bt.available()) {
    char c = (char)bt.read();
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

// ======================================================================
//  OLED DASHBOARD  (static render, no animation)
// ======================================================================
static void drawDashboard() {
  if (!oledOk) return;

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // ---- header row
  oled.setCursor(0, 0);
  oled.print(F("SHM NODE"));
  oled.setCursor(96, 0);
  oled.print(imuOk ? F("SPP") : F("ERR"));

  // ---- status row  (SYS STATUS: STABLE / CRITICAL)
  oled.setCursor(0, 11);
  oled.print(F("SYS STATUS: "));
  if (state == 0) {
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
  fmt(b0, calValid ? (tiltXF - cal.tiltX0) : 0.0f, 2);
  oled.print(b0);
  oled.print(F(" Y:"));
  fmt(b1, calValid ? (tiltYF - cal.tiltY0) : 0.0f, 2);
  oled.print(b1);

  // ---- vibration row   VIB: 0.031 g
  oled.setCursor(0, 33);
  oled.print(F("VIB: "));
  fmt(b2, vibRms, 3);
  oled.print(b2);
  oled.print(F(" g"));

  // ---- threshold row   TH V:0.30 T:2.5
  oled.setCursor(0, 44);
  oled.print(F("TH V:"));
  fmt(b0, vibThr, 2);
  oled.print(b0);
  oled.print(F(" T:"));
  fmt(b1, tiltThr, 1);
  oled.print(b1);

  // ---- footer row
  oled.setCursor(0, 55);
  if (!calValid) {
    oled.print(F("AWAIT BASELINE"));
  } else {
    uint32_t s  = millis() / 1000UL;
    uint8_t  h  = (uint8_t)(s / 3600UL);
    uint8_t  m  = (uint8_t)((s / 60UL) % 60UL);
    uint8_t  ss = (uint8_t)(s % 60UL);
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

  oled.display();
}

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

// ======================================================================
//  SETUP / LOOP
// ======================================================================
void setup() {
  Wire.begin();
  Wire.setClock(400000);                       // shared bus: MPU + OLED

  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) drawSplash();

  bt.begin(BT_BAUD);
  bt.println(F("#SHM-NODE,1.0,CSV,UNO"));

  EEPROM.get(0, cal);
  calValid = (cal.magic == EEPROM_MAGIC);

  imuOk = imuStart();
  if (imuOk) {
    dcInit = false;
  } else {
    bt.println(F("EVT:IMU:FAULT"));
  }
  tImu = millis();

  if (!calValid) startCalibration();   // power-on auto zero-point
                                       // (node must be at rest)
}

void loop() {
  uint32_t now = millis();

  if (!imuOk && (now - tImu) >= IMU_RETRY_MS) {          // sensor recovery
    tImu = now;
    if (imuStart()) {
      imuOk = true;
      dcInit = false;
      oledDirty = true;
      bt.println(F("EVT:IMU:REST"));
      if (!calValid && !calActive) startCalibration();
    }
  }

  if (imuOk && (now - tSample) >= PERIOD_SAMPLE_MS) {    // 200 Hz
    tSample = now;
    sampleStep();
  }

  if ((now - tSend) >= PERIOD_SEND_MS) {                 // 10 Hz
    tSend = now;
    serviceTelemetry(now);
  }

  if ((now - tOled) >= PERIOD_OLED_MS || oledDirty) {    // 10 Hz
    tOled = now;
    oledDirty = false;
    drawDashboard();
  }

  serviceCommands();
}
