/*
 * GPS_INS - full navigation: TriSense + Quectel L76K GNSS + INSLIB.
 *
 * Position (local NED and WGS84), velocity, acceleration, attitude, the
 * estimated sensor biases and how accurate all of it is, once a second.
 *
 * Hardware: Raspberry Pi Pico 2 (RP2350, arduino-pico core) + TriSense + L76K
 *   TriSense : ICM-42688-P on SPI (CS = GP17), magnetometer + BMP580 on I2C
 *   L76K     : module RX <- GP0 (Pico TX), module TX -> GP1 (Pico RX)
 * Libraries: Voltino_TriSense, TriSenseINSLIB, TinyGPSPlus (Library Manager).
 *
 * Serial commands: c = CSV output on/off, n = raw NMEA on/off, h = help.
 *
 * Everything runs on one core. That keeps the whole 8 KB core-0 stack for the
 * filter (it needs ~7 KB). If you move printing or the GNSS parser to core 1
 * with setup1()/loop1(), also add `bool core1_separate_stack = true;` or both
 * cores share those 8 KB and the filter silently overwrites core 1's stack.
 * See the README.
 *
 * Licence: AGPL-3.0, like the library (INSLIB is AGPL-3.0).
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <TriSense.h>
#include <TriSenseINS.h>
#include <TinyGPSPlus.h>
#include <stdlib.h>

#if defined(ARDUINO_ARCH_ESP32)
SET_LOOP_TASK_STACK_SIZE(16 * 1024);
#endif

// ---- Wiring ----------------------------------------------------------------
#define IMU_CS_PIN     17
#define IMU_SPI_HZ     10000000UL
#define GPS_PIN_TX     0      // MCU TX -> L76K RX
#define GPS_PIN_RX     1      // MCU RX <- L76K TX
#define GPS_FAST_BAUD  115200UL
#define GPS_BOOT_BAUD  9600UL

// ---- Magnetometer calibration (MotionCal example of Voltino_TriSense) -------
static const float MAG_HARD_IRON[3] = { 0.0f, 0.0f, 0.0f };
static const float MAG_SOFT_IRON[3][3] = {
  { 1.0f, 0.0f, 0.0f },
  { 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 1.0f }
};

TriSense    sensor;
TriSenseINS ins(sensor);
TinyGPSPlus gps;
// GGA field 11 = geoid separation (MSL -> ellipsoid). Both talker IDs, since
// a multi-constellation L76K sends GNGGA.
TinyGPSCustom geoidGP(gps, "GPGGA", 11);
TinyGPSCustom geoidGN(gps, "GNGGA", 11);

static bool csvOutput = false;
static bool rawNmea = false;

// ---------------------------------------------------------------------------
// L76K setup (PCAS commands: 115200 baud, 5 Hz, GGA + RMC only)
// ---------------------------------------------------------------------------
static void gpsOpen(uint32_t baud) {
  Serial1.end();
#if defined(ARDUINO_ARCH_RP2040)
  Serial1.setTX(GPS_PIN_TX);
  Serial1.setRX(GPS_PIN_RX);
  Serial1.begin(baud);
#elif defined(ARDUINO_ARCH_ESP32)
  Serial1.begin(baud, SERIAL_8N1, GPS_PIN_RX, GPS_PIN_TX);
#else
  Serial1.begin(baud);
#endif
  delay(50);
}

static void gpsSend(const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; ++p) cs ^= (uint8_t)*p;
  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X\r\n", cs);
  Serial1.print('$');
  Serial1.print(body);
  Serial1.print(tail);
  Serial1.flush();
}

static bool gpsListening(uint32_t ms) {
  const uint32_t base = gps.passedChecksum();
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (Serial1.available()) {
      gps.encode((char)Serial1.read());
      if (gps.passedChecksum() - base >= 2) return true;
    }
  }
  return false;
}

static void gpsConfigure() {
  gpsOpen(GPS_FAST_BAUD);   // the module keeps its baud rate across an MCU reset
  if (!gpsListening(1500)) {
    gpsOpen(GPS_BOOT_BAUD);
    if (!gpsListening(1500)) {
      Serial.println(F("[GPS] no NMEA at 9600 or 115200 - check wiring. Running without GNSS."));
      return;
    }
    gpsSend("PCAS01,5");    // 115200 baud
    delay(200);
    gpsOpen(GPS_FAST_BAUD);
  }
  gpsSend("PCAS02,200");    // 5 Hz
  delay(50);
  gpsSend("PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0,,,,0");  // GGA + RMC only
  Serial.println(F("[GPS] L76K at 115200 baud, 5 Hz"));
}

// Hand a fix to the filter when a GGA (which carries altitude and HDOP) ends.
static void gpsPoll() {
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    if (rawNmea) Serial.write(c);
    if (!gps.encode(c)) continue;
    if (!gps.altitude.isUpdated() || !gps.location.isValid() || !gps.hdop.isValid()) continue;

    const char *g = geoidGN.isValid() ? geoidGN.value() : (geoidGP.isValid() ? geoidGP.value() : NULL);
    const float geoid = (g != NULL && g[0] != '\0') ? (float)atof(g) : NAN;

    ins.feedGnss(gps.location.lat(), gps.location.lng(), (float)gps.altitude.meters(),
                 (float)gps.hdop.hdop(),
                 gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0,
                 gps.speed.isValid() ? (float)gps.speed.mps() : NAN,
                 gps.course.isValid() ? (float)gps.course.deg() : NAN,
                 gps.location.age(), geoid);
  }
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
static void printCsvHeader() {
  Serial.println(F("mode,ready,lat,lon,h_ell,n,e,d,vn,ve,vd,an,ae,ad,roll,pitch,yaw,sig_n,sig_e,sig_d,sig_yaw,sats,hdop"));
}

static void printTelemetry() {
  float roll = NAN, pitch = NAN, yaw = NAN, sr = NAN, sp = NAN, sy = NAN;
  float n = NAN, e = NAN, d = NAN, sn = NAN, se = NAN, sd = NAN;
  float vn = NAN, ve = NAN, vd = NAN, an = NAN, ae = NAN, ad = NAN, hEll = NAN;
  double lat = NAN, lon = NAN;
  ins.getRPY(roll, pitch, yaw);
  ins.getRPYStdDev(sr, sp, sy);
  ins.getPositionNED(n, e, d);
  ins.getPositionStdDev(sn, se, sd);
  ins.getVelocityNED(vn, ve, vd);
  ins.getAccelerationNED(an, ae, ad);
  const bool haveAbs = ins.getLatLonH(lat, lon, hEll);
  const uint8_t sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  const float hdop = gps.hdop.isValid() ? (float)gps.hdop.hdop() : 99.0f;

  if (csvOutput) {
    Serial.print((int)ins.getMode()); Serial.print(',');
    Serial.print(ins.isReady() ? 1 : 0); Serial.print(',');
    Serial.print(lat, 8); Serial.print(','); Serial.print(lon, 8); Serial.print(',');
    const float cols[] = { hEll, n, e, d, vn, ve, vd, an, ae, ad, roll, pitch, yaw, sn, se, sd, sy };
    for (float c : cols) { Serial.print(c, 3); Serial.print(','); }
    Serial.print(sats); Serial.print(','); Serial.println(hdop, 2);
    return;
  }

  Serial.print('['); Serial.print(TriSenseINS::modeName(ins.getMode()));
  if (!ins.isReady()) Serial.print(F(" warmup"));
  Serial.print(F("] RPY "));
  Serial.print(roll, 1); Serial.print('/'); Serial.print(pitch, 1); Serial.print('/'); Serial.print(yaw, 1);
  Serial.print(F(" deg (+-")); Serial.print(sy, 1); Serial.print(F(" yaw)"));

  Serial.print(F(" | NED ")); Serial.print(n, 1); Serial.print(' '); Serial.print(e, 1); Serial.print(' '); Serial.print(d, 1);
  Serial.print(F(" m (+-")); Serial.print(sn, 1); Serial.print(F(")"));
  Serial.print(F(" | v ")); Serial.print(vn, 2); Serial.print(' '); Serial.print(ve, 2); Serial.print(' '); Serial.print(vd, 2);
  Serial.print(F(" | a ")); Serial.print(an, 2); Serial.print(' '); Serial.print(ae, 2); Serial.print(' '); Serial.print(ad, 2);
  if (haveAbs) {
    Serial.print(F(" | ")); Serial.print(lat, 7); Serial.print(' '); Serial.print(lon, 7);
    Serial.print(F(" h ")); Serial.print(hEll, 1);
  }
  Serial.print(F(" | sats ")); Serial.print(sats); Serial.print(F(" hdop ")); Serial.print(hdop, 1);
  if (ins.isZuptActive()) Serial.print(F(" ZUPT"));
  if (ins.getDeadReckoningMs() > 0) { Serial.print(F(" DR ")); Serial.print(ins.getDeadReckoningMs() / 1000); Serial.print('s'); }
  Serial.println();

  float bax, bay, baz, bgx, bgy, bgz;
  Serial.print(F("        bias a "));
  if (ins.getAccelBias(bax, bay, baz)) { Serial.print(bax, 3); Serial.print(' '); Serial.print(bay, 3); Serial.print(' '); Serial.print(baz, 3); }
  Serial.print(F(" m/s2, g "));
  if (ins.getGyroBias(bgx, bgy, bgz)) { Serial.print(bgx, 3); Serial.print(' '); Serial.print(bgy, 3); Serial.print(' '); Serial.print(bgz, 3); }
  Serial.print(F(" dps | ")); Serial.print(ins.getEpochRateHz()); Serial.print(F(" Hz | filter "));
  Serial.print(ins.getLastUpdateUs()); Serial.print('/'); Serial.print(ins.getMaxUpdateUs()); Serial.print(F(" us"));
  if (ins.getFreeStack() >= 0) { Serial.print(F(" | stack free ")); Serial.print(ins.getFreeStack()); }
  if (!ins.stackOk()) Serial.print(F(" !! STACK TOO SMALL, see README"));
  if (ins.getImuLostPackets()) { Serial.print(F(" | IMU lost ")); Serial.print(ins.getImuLostPackets()); }
  Serial.println();
}

static void handleCommands() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'c': case 'C': csvOutput = !csvOutput; if (csvOutput) printCsvHeader(); break;
      case 'n': case 'N': rawNmea = !rawNmea; break;
      case 'h': case 'H': Serial.println(F("c = CSV on/off, n = raw NMEA on/off, h = help")); break;
      default: break;
    }
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { delay(10); }
  Serial.println(F("Voltino TriSense + L76K + INSLIB (15-state ESKF). AGPL-3.0."));

  if (!sensor.beginAll(MODE_HYBRID, IMU_CS_PIN, IMU_SPI_HZ)) {
    Serial.println(F("TriSense not found - check wiring and IMU_CS_PIN."));
    while (true) { delay(1000); }
  }
  ins.setMagCalibration(MAG_HARD_IRON, MAG_SOFT_IRON);

  gpsConfigure();

  Serial.println(F("Keep still: calibrating gyro..."));
  if (!ins.begin()) {
    Serial.println(F("INSLIB refused the configuration."));
    while (true) { delay(1000); }
  }
  Serial.println(F("Filter running. Waiting for a GNSS fix good enough to start (open sky)."));
}

void loop() {
  ins.update();
  gpsPoll();
  handleCommands();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    printTelemetry();
  }
}
