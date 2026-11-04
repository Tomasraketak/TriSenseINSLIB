/*
 * FlightLogger - record exactly what the filter sees, for post-processing.
 *
 * Every filter epoch prints the measurements TriSenseINS handed to INSLIB
 * (already in the FRD body frame and SI units) as tagged CSV lines, plus a
 * 10 Hz line with the live solution:
 *
 *   IMU,t_us,gyr_x,gyr_y,gyr_z,acc_x,acc_y,acc_z      rad/s, m/s^2   (every epoch)
 *   MAG,t_us,mag_x,mag_y,mag_z                        uT, calibrated (new samples)
 *   BARO,t_us,pressure_pa                             Pa             (new samples)
 *   GNSS,t_us,lat,lon,h_ell,cov nn,ne,nd,ee,ed,dd,delay_ms           (new fixes)
 *   SPEED,t_us,speed_mps                                             (new samples)
 *   SOL,t_us,mode,roll,pitch,yaw,h,vz,n,e,d,vn,ve,vd                 (10 Hz)
 *
 * Capture the serial output to a file (e.g. `cat /dev/ttyACM0 > flight.log`,
 * or PuTTY logging), then split it into INSLIB's dataset format:
 *
 *   python3 tools/split_flight_log.py flight.log flight/
 *
 * which writes imu.csv, mag.csv, baro.csv, gnss.csv, speed.csv and a
 * config.yaml that INSLIB's tools/inspostgui.py and tools/replay.py open
 * directly. That lets you re-run the flight through INSLIB on a PC with
 * different tuning, plots, a KML track and an RTS smoother.
 *
 * GNSS is optional: with USE_GNSS 1 an L76K on GP0/GP1 is used if present (the
 * same wiring as the GPS_INS example).
 *
 * For a real flight, log to an SD card instead of USB: replace the Serial
 * prints by writes to a File, and keep the buffering in mind - at 200 Hz this
 * is about 20 KB/s.
 *
 * Licence: AGPL-3.0, like the library. SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <TriSense.h>
#include <TriSenseINS.h>

#define USE_GNSS 1
#if USE_GNSS
#include <TinyGPSPlus.h>
#endif

#if defined(ARDUINO_ARCH_ESP32)
SET_LOOP_TASK_STACK_SIZE(16 * 1024);
#endif

#define IMU_CS_PIN   17
#define IMU_SPI_HZ   10000000UL
#define GPS_PIN_TX   0
#define GPS_PIN_RX   1
#define GPS_BAUD     115200UL   // set the L76K to 115200 once with the GPS_INS example

static const float MAG_HARD_IRON[3] = { 0.0f, 0.0f, 0.0f };
static const float MAG_SOFT_IRON[3][3] = {
  { 1.0f, 0.0f, 0.0f },
  { 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 1.0f }
};

TriSense    sensor;
TriSenseINS ins(sensor);
#if USE_GNSS
TinyGPSPlus gps;
TinyGPSCustom geoidGN(gps, "GNGGA", 11);
TinyGPSCustom geoidGP(gps, "GPGGA", 11);
#endif

// Prints "TAG,t" then the values with `digits` decimals, comma separated.
static void line(const char *tag, uint64_t t, const float *v, int n, int digits) {
  Serial.print(tag);
  Serial.print(',');
  Serial.print((unsigned long)(t / 1000000ULL));   // split so no 64-bit print is needed
  char frac[8];
  snprintf(frac, sizeof(frac), "%06lu", (unsigned long)(t % 1000000ULL));
  Serial.print(frac);
  for (int i = 0; i < n; i++) {
    Serial.print(',');
    Serial.print(v[i], digits);
  }
  Serial.println();
}

static void logEpoch() {
  const ins_measurements_t &m = ins.lastMeasurements();
  const uint64_t t = (uint64_t)m.timestamp;

  const float imu[6] = { m.gyr.data[0], m.gyr.data[1], m.gyr.data[2],
                         m.acc.data[0], m.acc.data[1], m.acc.data[2] };
  line("IMU", t, imu, 6, 6);

  if (m.mag.is_valid) line("MAG", t, m.mag.data, 3, 3);
  if (m.baro.is_valid) line("BARO", t, &m.baro.pressure_pa, 1, 2);
  if (m.speed.is_valid) line("SPEED", t, &m.speed.speed_mps, 1, 3);

  if (m.gnss_pos.is_valid) {
    // lat/lon need more digits than a float holds, so they are printed apart.
    const float *Q = m.gnss_pos.Qll_ned;  // column-major 3x3
    Serial.print(F("GNSS,"));
    Serial.print((unsigned long)(t / 1000000ULL));
    char frac[8];
    snprintf(frac, sizeof(frac), "%06lu", (unsigned long)(t % 1000000ULL));
    Serial.print(frac);
    Serial.print(','); Serial.print(m.gnss_pos.llh[0] * 57.29577951308232, 9);
    Serial.print(','); Serial.print(m.gnss_pos.llh[1] * 57.29577951308232, 9);
    Serial.print(','); Serial.print(m.gnss_pos.llh[2], 3);
    const float cov[6] = { Q[0], Q[3], Q[6], Q[4], Q[7], Q[8] };  // nn,ne,nd,ee,ed,dd
    for (float c : cov) { Serial.print(','); Serial.print(c, 4); }
    Serial.print(','); Serial.println(m.gnss_delay_ms + m.gnss_delay_us / 1000);
  }

  static uint64_t lastSol = 0;
  if (t - lastSol >= 100000ULL) {
    lastSol = t;
    float s[13];
    for (float &x : s) x = NAN;
    s[0] = (float)(int)ins.getMode();
    ins.getRPY(s[1], s[2], s[3]);
    ins.getHeight(s[4]);
    ins.getVerticalSpeed(s[5]);
    ins.getPositionNED(s[6], s[7], s[8]);
    ins.getVelocityNED(s[9], s[10], s[11]);
    line("SOL", t, s, 12, 3);
  }
}

#if USE_GNSS
static void gpsPoll() {
  while (Serial1.available()) {
    if (!gps.encode((char)Serial1.read())) continue;
    if (!gps.altitude.isUpdated() || !gps.location.isValid() || !gps.hdop.isValid()) continue;
    const char *g = geoidGN.isValid() ? geoidGN.value() : (geoidGP.isValid() ? geoidGP.value() : NULL);
    ins.feedGnss(gps.location.lat(), gps.location.lng(), (float)gps.altitude.meters(),
                 (float)gps.hdop.hdop(), gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0,
                 gps.speed.isValid() ? (float)gps.speed.mps() : NAN, NAN, gps.location.age(),
                 (g != NULL && g[0] != '\0') ? (float)atof(g) : NAN);
  }
}
#endif

void setup() {
  Serial.begin(921600);   // USB CDC ignores the number; a UART needs it fast
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { delay(10); }

  if (!sensor.beginAll(MODE_HYBRID, IMU_CS_PIN, IMU_SPI_HZ)) {
    Serial.println(F("# TriSense not found"));
    while (true) { delay(1000); }
  }
  ins.setMagCalibration(MAG_HARD_IRON, MAG_SOFT_IRON);

#if USE_GNSS
#if defined(ARDUINO_ARCH_RP2040)
  Serial1.setTX(GPS_PIN_TX);
  Serial1.setRX(GPS_PIN_RX);
  Serial1.begin(GPS_BAUD);
#elif defined(ARDUINO_ARCH_ESP32)
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_PIN_RX, GPS_PIN_TX);
#else
  Serial1.begin(GPS_BAUD);
#endif
#endif

  if (!ins.begin()) {
    Serial.println(F("# INSLIB refused the configuration"));
    while (true) { delay(1000); }
  }
  // Lines starting with '#' are comments for split_flight_log.py.
  Serial.println(F("# TriSenseINSLIB FlightLogger " TRISENSE_INSLIB_VERSION ", INSLIB " TRISENSE_INSLIB_INSLIB_VERSION));
  Serial.println(F("# frames: body FRD, nav NED; units SI"));
}

void loop() {
  if (ins.update()) logEpoch();
#if USE_GNSS
  gpsPoll();
#endif
}
