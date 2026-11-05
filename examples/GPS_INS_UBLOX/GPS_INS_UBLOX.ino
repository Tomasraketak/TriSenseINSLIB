/*
 * GPS_INS_UBLOX - full navigation with a u-blox M10 (MAX-M10S, NEO-M10, ...)
 * through its binary UBX protocol: TriSense + u-blox + INSLIB.
 *
 * Why UBX instead of NMEA (the GPS_INS example):
 *   - UBX-NAV-PVT carries the receiver's OWN accuracy estimates (hAcc, vAcc,
 *     sAcc), so the filter weights every fix by how good it really is instead
 *     of guessing from HDOP;
 *   - it carries a full 3-axis NED velocity (incl. vertical), which INSLIB
 *     fuses directly - much better than NMEA's scalar ground speed;
 *   - ellipsoid height directly (no geoid-separation detour);
 *   - 10 Hz instead of 1-5 Hz, and one compact binary message per epoch.
 *
 * Hardware: Raspberry Pi Pico 2 (RP2350, arduino-pico core) + TriSense + M10
 *   TriSense : ICM-42688-P on SPI (CS = GP17), magnetometer + BMP580 on I2C
 *   u-blox   : UART  module RX <- GP0 (Pico TX), module TX -> GP1 (Pico RX)
 *              or I2C/Qwiic on the same bus as the TriSense (GNSS_USE_I2C 1)
 * Libraries: Voltino_TriSense, TriSenseINSLIB,
 *            "SparkFun u-blox GNSS v3" (Library Manager).
 *
 * The receiver is configured in RAM on every boot (UART 115200, UBX only,
 * 10 Hz, NAV-PVT every epoch, dynamic model below), so a factory-fresh module
 * works and nothing is written to its flash.
 *
 * Serial commands: c = CSV output on/off, h = help.
 *
 * Licence: AGPL-3.0, like the library (INSLIB is AGPL-3.0).
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <TriSense.h>
#include <TriSenseINS.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#if defined(ARDUINO_ARCH_ESP32)
SET_LOOP_TASK_STACK_SIZE(16 * 1024);
#endif

// ---- Wiring ----------------------------------------------------------------
#define IMU_CS_PIN     17
#define IMU_SPI_HZ     10000000UL
#define GNSS_USE_I2C   0           // 0 = UART (Serial1), 1 = I2C / Qwiic (Wire, address 0x42)
#define GPS_PIN_TX     0           // MCU TX -> u-blox RX   (UART only)
#define GPS_PIN_RX     1           // MCU RX <- u-blox TX   (UART only)
#define GPS_FAST_BAUD  115200UL
#define GPS_BOOT_BAUD  9600UL      // u-blox factory default

// ---- Receiver settings -----------------------------------------------------
#define GNSS_RATE_HZ   10          // M10 with its default constellations: up to 10 Hz
// Dynamic platform model. It bounds what the receiver's own filter believes
// is physically possible; the wrong one smooths away real motion.
//   DYN_MODEL_AIRBORNE4g  rockets, drones, aircraft (highest dynamics u-blox offers)
//   DYN_MODEL_AUTOMOTIVE  cars        DYN_MODEL_PEDESTRIAN  walking
//   DYN_MODEL_PORTABLE    u-blox default, general purpose
#define GNSS_DYN_MODEL DYN_MODEL_AIRBORNE4g
// Time from the measurement to the end of the NAV-PVT message [ms]. Receiver
// processing plus ~10 ms of UART at 115200. Measure it if you need precision
// under hard acceleration (INSLIB tools/inslib_clock_error.py).
#define GNSS_LATENCY_MS 50

// ---- Magnetometer calibration (MotionCal example of Voltino_TriSense) -------
static const float MAG_HARD_IRON[3] = { 0.0f, 0.0f, 0.0f };
static const float MAG_SOFT_IRON[3][3] = {
  { 1.0f, 0.0f, 0.0f },
  { 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 1.0f }
};

TriSense    sensor;
TriSenseINS ins(sensor);
#if GNSS_USE_I2C
SFE_UBLOX_GNSS gnss;
#else
SFE_UBLOX_GNSS_SERIAL gnss;
#endif

static bool     gnssOk = false;
static bool     csvOutput = false;
static uint32_t pvtCount = 0, pvtHz = 0, pvtWindowMs = 0;

// Last fix, for the telemetry line.
static uint8_t  lastFixType = 0, lastSats = 0;
static float    lastHAcc = NAN, lastVAcc = NAN, lastSAcc = NAN, lastPdop = NAN;

// ---------------------------------------------------------------------------
// Receiver setup
// ---------------------------------------------------------------------------
#if !GNSS_USE_I2C
static void uartOpen(uint32_t baud) {
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
#endif

static bool gnssConfigure() {
#if GNSS_USE_I2C
  if (!gnss.begin(sensor.getWire(), 0x42)) return false;
  gnss.setI2COutput(COM_TYPE_UBX);                      // no NMEA on the bus
#else
  // The module keeps its baud rate only until power-off (RAM config), so try
  // the fast rate first, then the factory default.
  uartOpen(GPS_FAST_BAUD);
  if (!gnss.begin(Serial1)) {
    uartOpen(GPS_BOOT_BAUD);
    if (!gnss.begin(Serial1)) return false;
    gnss.setVal32(UBLOX_CFG_UART1_BAUDRATE, GPS_FAST_BAUD);   // module switches right away
    delay(100);
    uartOpen(GPS_FAST_BAUD);
    if (!gnss.begin(Serial1)) return false;
  }
  gnss.setUART1Output(COM_TYPE_UBX);                    // NMEA off: it only costs bandwidth
#endif

  bool ok = true;
  ok &= gnss.setMeasurementRate(1000 / GNSS_RATE_HZ);
  ok &= gnss.setDynamicModel(GNSS_DYN_MODEL);
  ok &= gnss.setAutoPVT(true);                          // NAV-PVT every epoch, non-blocking reads
  if (!ok) Serial.println(F("[GNSS] some settings were not acknowledged - check the module type."));
  return true;
}

// Hand every new NAV-PVT to the filter. Polled every few ms, not every loop
// pass, so the GNSS interface never takes time from the IMU FIFO.
static void gnssPoll() {
  static uint32_t lastPollUs = 0;
  if (!gnssOk || micros() - lastPollUs < 3000) return;
  lastPollUs = micros();
  if (!gnss.getPVT(0)) return;          // nothing new

  pvtCount++;
  lastFixType = gnss.getFixType(0);
  lastSats    = gnss.getSIV(0);
  lastHAcc    = gnss.getHorizontalAccEst(0) * 1e-3f;   // mm -> m
  lastVAcc    = gnss.getVerticalAccEst(0) * 1e-3f;
  lastSAcc    = gnss.getSpeedAccEst(0) * 1e-3f;        // mm/s -> m/s
  lastPdop    = gnss.getPDOP(0) * 0.01f;

  // Only a real 3D fix the receiver itself marks as valid (gnssFixOK).
  if (!gnss.getGnssFixOk(0) || (lastFixType != 3 && lastFixType != 4)) return;

  TriSenseGnssFix f;
  f.latDeg  = gnss.getLatitude(0) * 1e-7;
  f.lonDeg  = gnss.getLongitude(0) * 1e-7;
  const float hMsl = gnss.getAltitudeMSL(0) * 1e-3f;
  const float hEll = gnss.getAltitude(0) * 1e-3f;
  f.altMslM = hMsl;
  f.geoidSeparationM = hEll - hMsl;     // exact, from the receiver's geoid model
  f.sats = lastSats;
  f.horizSigmaM = lastHAcc;             // u-blox hAcc/vAcc are 1-sigma estimates
  f.vertSigmaM  = lastVAcc;
  f.velN = gnss.getNedNorthVel(0) * 1e-3f;
  f.velE = gnss.getNedEastVel(0) * 1e-3f;
  f.velD = gnss.getNedDownVel(0) * 1e-3f;
  f.velSigmaMps = lastSAcc;
  f.speedMps  = gnss.getGroundSpeed(0) * 1e-3f;
  f.courseDeg = gnss.getHeading(0) * 1e-5f;
  f.ageMs = 0;                          // parsed just now; latency is in the config
  ins.feedGnss(f);
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
static void printCsvHeader() {
  Serial.println(F("mode,ready,lat,lon,h_ell,n,e,d,vn,ve,vd,an,ae,ad,roll,pitch,yaw,sig_n,sig_e,sig_d,sig_yaw,sats,hacc,vacc,sacc"));
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

  if (csvOutput) {
    Serial.print((int)ins.getMode()); Serial.print(',');
    Serial.print(ins.isReady() ? 1 : 0); Serial.print(',');
    Serial.print(lat, 8); Serial.print(','); Serial.print(lon, 8); Serial.print(',');
    const float cols[] = { hEll, n, e, d, vn, ve, vd, an, ae, ad, roll, pitch, yaw, sn, se, sd, sy };
    for (float c : cols) { Serial.print(c, 3); Serial.print(','); }
    Serial.print(lastSats); Serial.print(',');
    Serial.print(lastHAcc, 2); Serial.print(','); Serial.print(lastVAcc, 2); Serial.print(',');
    Serial.println(lastSAcc, 2);
    return;
  }

  Serial.print('['); Serial.print(TriSenseINS::modeName(ins.getMode()));
  if (!ins.isReady()) Serial.print(F(" warmup"));
  Serial.print(F("] RPY "));
  Serial.print(roll, 1); Serial.print('/'); Serial.print(pitch, 1); Serial.print('/'); Serial.print(yaw, 1);
  Serial.print(F(" deg (+-")); Serial.print(sy, 1); Serial.print(F(" yaw)"));
  Serial.print(F(" | NED ")); Serial.print(n, 2); Serial.print(' '); Serial.print(e, 2); Serial.print(' '); Serial.print(d, 2);
  Serial.print(F(" m (+-")); Serial.print(sn, 2); Serial.print(F(")"));
  Serial.print(F(" | v ")); Serial.print(vn, 2); Serial.print(' '); Serial.print(ve, 2); Serial.print(' '); Serial.print(vd, 2);
  Serial.print(F(" | a ")); Serial.print(an, 2); Serial.print(' '); Serial.print(ae, 2); Serial.print(' '); Serial.print(ad, 2);
  if (haveAbs) {
    Serial.print(F(" | ")); Serial.print(lat, 7); Serial.print(' '); Serial.print(lon, 7);
    Serial.print(F(" h ")); Serial.print(hEll, 2);
  }
  if (ins.isZuptActive()) Serial.print(F(" ZUPT"));
  if (ins.getDeadReckoningMs() > 0) { Serial.print(F(" DR ")); Serial.print(ins.getDeadReckoningMs() / 1000); Serial.print('s'); }
  Serial.println();

  Serial.print(F("        GNSS fix ")); Serial.print(lastFixType);
  Serial.print(F(" sats ")); Serial.print(lastSats);
  Serial.print(F(" hAcc ")); Serial.print(lastHAcc, 2);
  Serial.print(F(" vAcc ")); Serial.print(lastVAcc, 2);
  Serial.print(F(" sAcc ")); Serial.print(lastSAcc, 2);
  Serial.print(F(" pDOP ")); Serial.print(lastPdop, 1);
  Serial.print(F(" | PVT ")); Serial.print(pvtHz); Serial.print(F(" Hz"));
  const ins_diag_t &dg = ins.diagnostics();
  Serial.print(F(" | fused ")); Serial.print(dg.n_gnss_used); Serial.print('/'); Serial.print(dg.n_gnss_seen);
  Serial.print(F(" | ")); Serial.print(ins.getEpochRateHz()); Serial.print(F(" Hz, filter "));
  Serial.print(ins.getLastUpdateUs()); Serial.print('/'); Serial.print(ins.getMaxUpdateUs()); Serial.print(F(" us"));
  if (ins.getFreeStack() >= 0) { Serial.print(F(" | stack free ")); Serial.print(ins.getFreeStack()); }
  if (!ins.stackOk()) Serial.print(F(" !! STACK TOO SMALL, see README"));
  Serial.println();
}

static void handleCommands() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'c': case 'C': csvOutput = !csvOutput; if (csvOutput) printCsvHeader(); break;
      case 'h': case 'H': Serial.println(F("c = CSV on/off, h = help")); break;
      default: break;
    }
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { delay(10); }
  Serial.println(F("Voltino TriSense + u-blox M10 (UBX) + INSLIB. AGPL-3.0."));

  if (!sensor.beginAll(MODE_HYBRID, IMU_CS_PIN, IMU_SPI_HZ)) {
    Serial.println(F("TriSense not found - check wiring and IMU_CS_PIN."));
    while (true) { delay(1000); }
  }
  ins.setMagCalibration(MAG_HARD_IRON, MAG_SOFT_IRON);

  gnssOk = gnssConfigure();
  Serial.println(gnssOk ? F("[GNSS] u-blox configured: UBX, NAV-PVT")
                        : F("[GNSS] no u-blox answer - check wiring / GNSS_USE_I2C. Running without GNSS."));

  TriSenseINSConfig cfg;
  cfg.gnssLatencyMs = GNSS_LATENCY_MS;
  // UBX gives per-fix accuracies, so the gates can be tighter than for NMEA:
  // enter 3D once hAcc <= 5 m / vAcc <= 10 m, leave it above 15 m / 30 m.
  cfg.gnssStartMaxHorzM = 5.0f;
  cfg.gnssStartMaxVertM = 10.0f;

  Serial.println(F("Keep still: calibrating gyro..."));
  if (!ins.begin(cfg)) {
    Serial.println(F("INSLIB refused the configuration."));
    while (true) { delay(1000); }
  }
  Serial.println(F("Filter running. Waiting for a 3D fix (open sky)."));
  pvtWindowMs = millis();
}

void loop() {
  ins.update();
  gnssPoll();
  handleCommands();

  const uint32_t now = millis();
  if (now - pvtWindowMs >= 1000) {
    pvtHz = pvtCount;
    pvtCount = 0;
    pvtWindowMs = now;
    printTelemetry();
  }
}
