/*
 * TriSenseINS.cpp - Voltino TriSense + INSLIB navigation filter.
 *
 * Part of TriSenseINSLIB. SPDX-License-Identifier: AGPL-3.0-or-later
 * Copyright (c) 2026 VoltinoLabs (Tomas Michal). See TriSenseINS.h and LICENSE.
 *
 * Most of the glue here comes from the GPS_INS_INSLIB example of
 * Voltino_TriSense (draft PR #7), whose README explains each decision in more
 * detail: unit/axis mapping, IMU decimation, the "new sample only" rule for
 * the magnetometer and barometer, the GNSS covariance model and entry gate.
 */
#include "TriSenseINS.h"

#include <math.h>
#include <string.h>

namespace {

const float DEG2RADF = 0.017453292519943295f;
const float RAD2DEGF = 57.29577951308232f;
const double DEG2RAD = 0.017453292519943295;
const double RAD2DEG = 57.29577951308232;
// The g the ICM-42688-P scale factors imply (TriSense reports accel in g).
const float G_MPS2 = 9.80665f;

float wrap360(float deg) {
  deg = fmodf(deg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  return deg;
}

void matMul3(const float A[3][3], const float B[3][3], float C[3][3]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
}

// ZYX Euler (yaw about z, then pitch about y, then roll about x).
void eulerToMat(float r, float p, float y, float R[3][3]) {
  const float cr = cosf(r), sr = sinf(r);
  const float cp = cosf(p), sp = sinf(p);
  const float cy = cosf(y), sy = sinf(y);
  R[0][0] = cy * cp; R[0][1] = cy * sp * sr - sy * cr; R[0][2] = cy * sp * cr + sy * sr;
  R[1][0] = sy * cp; R[1][1] = sy * sp * sr + cy * cr; R[1][2] = sy * sp * cr - cy * sr;
  R[2][0] = -sp;     R[2][1] = cp * sr;                R[2][2] = cp * cr;
}

void quatToMat(const float q[4], float R[3][3]) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  R[0][0] = 1 - 2 * (y * y + z * z); R[0][1] = 2 * (x * y - w * z);     R[0][2] = 2 * (x * z + w * y);
  R[1][0] = 2 * (x * y + w * z);     R[1][1] = 1 - 2 * (x * x + z * z); R[1][2] = 2 * (y * z - w * x);
  R[2][0] = 2 * (x * z - w * y);     R[2][1] = 2 * (y * z + w * x);     R[2][2] = 1 - 2 * (x * x + y * y);
}

void diagCov(float Q[9], float varN, float varE, float varD) {
  memset(Q, 0, 9 * sizeof(float));
  Q[0] = varN;
  Q[4] = varE;
  Q[8] = varD;
}

}  // namespace

// ===========================================================================
// Construction and configuration
// ===========================================================================

TriSenseINS::TriSenseINS(TriSense &sensor) : _sensor(sensor) {
  memset(&_suite, 0, sizeof(_suite));
  memset(&_meas, 0, sizeof(_meas));
  memset(&_pending, 0, sizeof(_pending));
  memset(_accSum, 0, sizeof(_accSum));
  memset(_gyrSum, 0, sizeof(_gyrSum));
  memset(_magHard, 0, sizeof(_magHard));
  memset(_magSoft, 0, sizeof(_magSoft));
  _magSoft[0][0] = _magSoft[1][1] = _magSoft[2][2] = 1.0f;
  _mountRPY[0] = _mountRPY[1] = _mountRPY[2] = 0.0f;
  setMounting(TriSenseINSMount::ComponentsUp);
}

void TriSenseINS::setMounting(TriSenseINSMount mount) {
  // A TriSense lying flat, components up, reads +1 g on its Z axis at rest:
  // its frame is right-handed Z-up. FRD is forward = +X, right = -Y, down = -Z.
  // Upside down (rotated 180 deg about X): forward = +X, right = +Y, down = +Z.
  memset(_mountBase, 0, sizeof(_mountBase));
  _mountBase[0][0] = 1.0f;
  if (mount == TriSenseINSMount::ComponentsDown) {
    _mountBase[1][1] = 1.0f;
    _mountBase[2][2] = 1.0f;
  } else {
    _mountBase[1][1] = -1.0f;
    _mountBase[2][2] = -1.0f;
  }
  rebuildMount();
}

void TriSenseINS::setMountingRPY(float rollDeg, float pitchDeg, float yawDeg) {
  _mountRPY[0] = rollDeg * DEG2RADF;
  _mountRPY[1] = pitchDeg * DEG2RADF;
  _mountRPY[2] = yawDeg * DEG2RADF;
  rebuildMount();
}

void TriSenseINS::setMountingRotation(const float R[3][3]) {
  memcpy(_mountBase, R, sizeof(_mountBase));
  _mountRPY[0] = _mountRPY[1] = _mountRPY[2] = 0.0f;
  rebuildMount();
}

void TriSenseINS::rebuildMount() {
  float E[3][3];
  eulerToMat(_mountRPY[0], _mountRPY[1], _mountRPY[2], E);
  matMul3(E, _mountBase, _mount);
}

void TriSenseINS::toFRD(float x, float y, float z, float out[3]) const {
  out[0] = _mount[0][0] * x + _mount[0][1] * y + _mount[0][2] * z;
  out[1] = _mount[1][0] * x + _mount[1][1] * y + _mount[1][2] * z;
  out[2] = _mount[2][0] * x + _mount[2][1] * y + _mount[2][2] * z;
}

void TriSenseINS::setMagCalibration(const float hardIron[3], const float softIron[3][3]) {
  memcpy(_magHard, hardIron, sizeof(_magHard));
  memcpy(_magSoft, softIron, sizeof(_magSoft));
}

void TriSenseINS::setMagHardIron(float x, float y, float z) {
  _magHard[0] = x; _magHard[1] = y; _magHard[2] = z;
}

void TriSenseINS::setDeclination(float declinationDeg) { _declinationDeg = declinationDeg; }

void TriSenseINS::setMagneticModelPosition(double latDeg, double lonDeg) {
  setMagneticModelPosition(latDeg, lonDeg, _cfg.wmmYear);
}

void TriSenseINS::setMagneticModelPosition(double latDeg, double lonDeg, float year) {
  if (!isfinite(latDeg) || !isfinite(lonDeg)) return;
  _magModelLat = latDeg;
  _magModelLon = lonDeg;
  _magModelYear = year;
  _magModelSet = true;
  // nav_suite hands the position to the INS now and to the magnetometer AHRS
  // on every (re)start, so both reference true north.
  if (_begun) nav_suite_set_magnetic_model_position(&_suite, latDeg * DEG2RAD, lonDeg * DEG2RAD, year);
}

void TriSenseINS::setInitialHeading(float headingDeg, float sigmaDeg) {
  _haveInitHeading = sigmaDeg > 0.0f;
  float y = wrap360(headingDeg);
  if (y > 180.0f) y -= 360.0f;
  _initHeadingRad = y * DEG2RADF;
  _initHeadingSigmaRad = sigmaDeg * DEG2RADF;
}

void TriSenseINS::setAutoZupt(bool enable) {
  _cfg.autoZupt = enable;
  if (_begun) nav_suite_set_auto_zupt_zaru_disable(&_suite, !enable);
}

// ===========================================================================
// Life cycle
// ===========================================================================

bool TriSenseINS::begin() { return begin(TriSenseINSConfig()); }

bool TriSenseINS::begin(const TriSenseINSConfig &config) {
  _cfg = config;
  if (_cfg.navRateHz == 0) _cfg.navRateHz = 200;
  if (_cfg.magRateHz == 0) _cfg.magRateHz = 50;
  if (_cfg.baroRateHz == 0) _cfg.baroRateHz = 50;

  if (_cfg.configureSensors) {
    _sensor.imu.setODR(_cfg.imuOdr);
    if (_sensor.imu.getFIFOMode() == FIFO_NONE) _sensor.imu.setFIFOMode(FIFO_16BIT);

    if (_cfg.useBarometer) {
      // 400 kHz I2C so the magnetometer and barometer reads stay cheap.
      _sensor.bmp.setI2CSpeed(400000);
      // 50 Hz, sampled at 50 Hz, IIR off: an IIR would add group delay and,
      // worse for a Kalman filter, correlate samples it treats as independent.
      // Config registers only latch reliably in standby, hence the bracket.
      _sensor.bmp.setPowerMode(BMP580_MODE_STANDBY);
      delay(5);
      _sensor.bmp.setOversampling(BMP580_OSR_x8, BMP580_OSR_x2);
      _sensor.bmp.setODR(BMP580_ODR_50p1Hz);
      _sensor.bmp.setIIRFilter(BMP580_IIR_OFF, BMP580_IIR_OFF);
      _sensor.bmp.setPowerMode(BMP580_MODE_NORMAL);
      delay(20);
    }
  }

  if (_cfg.calibrateGyro) _sensor.autoCalibrateGyro(_cfg.gyroCalibrationSamples);

  return reset();
}

bool TriSenseINS::reset() {
  memset(_accSum, 0, sizeof(_accSum));
  memset(_gyrSum, 0, sizeof(_gyrSum));
  _imuCount = 0;
  memset(&_pending, 0, sizeof(_pending));
  memset(&_meas, 0, sizeof(_meas));
  _droppedEpochs = 0;
  _epochCount = 0;
  _epochHz = 0;
  _navLastUs = _navMaxUs = 0;
  _stackOk = true;
  _lastPressure = NAN;

  _begun = initFilter();
  if (!_begun) return false;

  _sensor.imu.flushFIFO();
  const uint64_t now = nowUs();
  _lastEpochUs = now;
  _lastMagUs = now;
  _lastBaroUs = now;
  _lastStarvedUs = now;
  _rateWindowUs = now;
  return true;
}

bool TriSenseINS::initFilter() {
  memset(&_suite, 0, sizeof(_suite));

  ins_init_t init;
  memset(&init, 0, sizeof(init));
  init.time = (ins_time_us_t)nowUs();
  // No start position: auto_init anchors the NED origin on the first fix that
  // passes the entry gate.
  init.pos_init_stddev_m = 10.0f;
  init.vel_init_stddev_mps = 1.0f;
  init.rpy_init_stddev_rad[0] = 5.0f * DEG2RADF;    // levelled from gravity
  init.rpy_init_stddev_rad[1] = 5.0f * DEG2RADF;
  init.rpy_init_stddev_rad[2] = 180.0f * DEG2RADF;  // unknown at boot

  ins_options_t opt;
  memset(&opt, 0, sizeof(opt));
  opt.auto_init = true;  // bootstrap level, time and position from the data
  opt.kalman_update_dt_sec = (_cfg.covarianceRateHz > 0.0f) ? 1.0f / _cfg.covarianceRateHz : 0.0f;

  // The GNSS gates must agree with the covariance model in feedGnss(), or the
  // filter consumes fixes forever without ever entering 3D.
  opt.gnss_start_max_horizontal_pos_stddev_m = _cfg.gnssStartMaxHorzM;
  opt.gnss_start_max_vertical_pos_stddev_m   = _cfg.gnssStartMaxVertM;
  opt.gnss_stop_max_horizontal_pos_stddev_m  = _cfg.gnssStopMaxHorzM;
  opt.gnss_stop_max_vertical_pos_stddev_m    = _cfg.gnssStopMaxVertM;
  opt.gnss_max_horizontal_pos_stddev_m = _cfg.gnssStopMaxHorzM;
  opt.gnss_max_vertical_pos_stddev_m   = _cfg.gnssStopMaxVertM;

  opt.max_deadreckoning_sec = _cfg.maxDeadReckoningSec;
  opt.mag_field_tolerance   = _cfg.magFieldTolerance;
  opt.auto_zupt_disable     = !_cfg.autoZupt;
  opt.chi2_disable          = !_cfg.outlierRejection;
  // The magnetometer is calibrated here (setMagCalibration), so INSLIB's own
  // mag_fixed_bias / mag_misalignment stay at zero = identity.

  if (nav_suite_init(&_suite, &init, &opt) != 0) return false;

  if (!_cfg.autoZupt) nav_suite_set_auto_zupt_zaru_disable(&_suite, true);
  if (_haveInitHeading) {
    nav_suite_set_init_att_hint(&_suite, 0.0f, 0.0f, 0.0f, _initHeadingRad, _initHeadingSigmaRad);
  }
  if (_magModelSet) {
    nav_suite_set_magnetic_model_position(&_suite, _magModelLat * DEG2RAD, _magModelLon * DEG2RAD,
                                          _magModelYear);
  }
  return true;
}

// 64-bit microseconds from the 32-bit micros(), valid as long as update() runs
// at least once every 71 minutes.
uint64_t TriSenseINS::nowUs() {
  const uint32_t m = micros();
  if (m < _microsLast) _microsHigh += 0x100000000ULL;
  _microsLast = m;
  return _microsHigh | m;
}

// ===========================================================================
// Sensor access. The ONE place that knows how TriSense delivers data, so a new
// board revision (e.g. MMC5603NJ instead of AK09918C on V1.1/V2) changes only
// this section.
// ===========================================================================

// Magnetometer sample in the TriSense's own axes [uT], hard/soft iron applied,
// then rotated into body FRD. False if there is no new sample.
bool TriSenseINS::readMagnetometer(float out[3]) {
  if (!_sensor.mag.readData()) return false;
  if (_sensor.mag.overflow) return false;
  const float rx = _sensor.mag.x, ry = _sensor.mag.y, rz = _sensor.mag.z;

  // Calibration first, in raw sensor axes, then the mount rotation: a MotionCal
  // fit is derived from raw axes, so applying it after an axis flip would feed
  // each offset to the wrong axis. Mirrors TriSenseFusion::applyMagCalibration().
  const float hx = rx - _magHard[0], hy = ry - _magHard[1], hz = rz - _magHard[2];
  const float cx = _magSoft[0][0] * hx + _magSoft[0][1] * hy + _magSoft[0][2] * hz;
  const float cy = _magSoft[1][0] * hx + _magSoft[1][1] * hy + _magSoft[1][2] * hz;
  const float cz = _magSoft[2][0] * hx + _magSoft[2][1] * hy + _magSoft[2][2] * hz;
  toFRD(cx, cy, cz, out);
  return true;
}

int TriSenseINS::freeStackBytes() {
#if defined(ARDUINO_ARCH_RP2040)
  return rp2040.getFreeStack();
#elif defined(ARDUINO_ARCH_ESP32)
  // High-water mark: the least free stack this task has EVER had [bytes].
  return (int)uxTaskGetStackHighWaterMark(NULL);
#else
  return -1;
#endif
}

uint32_t TriSenseINS::getImuLostPackets() const { return _sensor.imu.getLostPacketCount(); }

// ===========================================================================
// The epoch loop
// ===========================================================================

bool TriSenseINS::update() {
  if (!_begun) return false;

  // Drain the IMU FIFO, converting to FRD / SI on the way in. Capped so one
  // call can never starve the epoch clock; without the FIFO every read is a
  // fresh register read, so take one per call.
  const int cap = (_sensor.imu.getFIFOMode() == FIFO_NONE) ? 1 : 64;
  float ax, ay, az, gx, gy, gz;
  for (int n = 0; n < cap && _sensor.imu.readFIFO(ax, ay, az, gx, gy, gz); n++) {
    float a[3], g[3];
    toFRD(ax * G_MPS2, ay * G_MPS2, az * G_MPS2, a);
    toFRD(gx * DEG2RADF, gy * DEG2RADF, gz * DEG2RADF, g);
    _accSum[0] += a[0]; _accSum[1] += a[1]; _accSum[2] += a[2];
    _gyrSum[0] += g[0]; _gyrSum[1] += g[1]; _gyrSum[2] += g[2];
    _imuCount++;
  }

  const uint64_t now = nowUs();
  const uint64_t period = 1000000UL / _cfg.navRateHz;
  if (now - _lastEpochUs < period) return false;

  // No IMU data, no epoch: the strapdown is what carries the state. The epoch
  // clock stays put so dt is right once samples resume.
  if (_imuCount == 0) {
    if (now - _lastStarvedUs >= period) {
      _lastStarvedUs = now;
      _droppedEpochs++;
    }
    return false;
  }

  float dt = (float)(now - _lastEpochUs) * 1e-6f;
  _lastEpochUs = now;
  if (dt > 0.25f) dt = 0.25f;  // a stalled loop must not inject a wild step

  runEpoch(now, dt);

  _epochCount++;
  if (now - _rateWindowUs >= 1000000ULL) {
    _epochHz = _epochCount;
    _epochCount = 0;
    _rateWindowUs = now;
  }
  return true;
}

void TriSenseINS::runEpoch(uint64_t now, float dt) {
  // Aiding fed since the last epoch (GNSS, speed, heading, ZUPT) rides along.
  memcpy(&_meas, &_pending, sizeof(_meas));
  memset(&_pending, 0, sizeof(_pending));
  _meas.timestamp = (ins_time_us_t)now;

  // Time spent waiting for this epoch is extra age on top of the stated delay.
  if (_meas.gnss_pos.is_valid || _meas.gnss_vel.is_valid) {
    const uint64_t waited = now - _gnssFedUs;
    _meas.gnss_delay_us = (int32_t)(waited > 500000ULL ? 500000ULL : waited);
  }
  if (_meas.speed.is_valid) {
    const uint64_t waited = now - _speedFedUs;
    _meas.speed_delay_us = (int32_t)(waited > 500000ULL ? 500000ULL : waited);
  }
  if (_meas.yaw.is_valid) {
    const uint64_t waited = now - _yawFedUs;
    _meas.yaw_delay_us = (int32_t)(waited > 500000ULL ? 500000ULL : waited);
  }

  // --- IMU: the mean of the batch over the batch's own time. Integrating the
  // mean once over dt is the same first-order strapdown step as integrating
  // each sample over dt/N, so the 1 kHz -> 200 Hz decimation is free.
  const float inv = 1.0f / (float)_imuCount;
  _meas.strapdown_dt_sec = dt;
  _meas.acc.is_valid = true;
  _meas.gyr.is_valid = true;
  for (int i = 0; i < 3; i++) {
    _meas.acc.data[i] = _accSum[i] * inv;
    _meas.gyr.data[i] = _gyrSum[i] * inv;
    _meas.acc.Qll_diag[i] = _cfg.accelNoisePSD;
    _meas.gyr.Qll_diag[i] = _cfg.gyroNoisePSD;
    _accSum[i] = 0.0f;
    _gyrSum[i] = 0.0f;
  }
  _imuCount = 0;

  // --- Magnetometer: flagged only on epochs with a genuinely NEW sample. A
  // latched is_valid would re-fuse one reading every epoch and make the filter
  // overconfident about its heading.
  if (_cfg.useMagnetometer && now - _lastMagUs >= 1000000ULL / _cfg.magRateHz) {
    _lastMagUs = now;
    float m[3];
    if (readMagnetometer(m)) {
      _meas.mag.is_valid = true;
      _meas.mag.data[0] = m[0];
      _meas.mag.data[1] = m[1];
      _meas.mag.data[2] = m[2];
    }
  }

  // --- Barometer: raw static pressure, INSLIB owns the conversion. Same "new
  // sample only" rule; the BMP580 driver returns its cached value until a new
  // conversion is ready, so an unchanged reading is not a new sample.
  if (_cfg.useBarometer && now - _lastBaroUs >= 1000000ULL / _cfg.baroRateHz) {
    _lastBaroUs = now;
    const float pa = _sensor.readPressure();
    if (pa > 30000.0f && pa < 125000.0f && pa != _lastPressure) {
      _meas.baro.is_valid = true;
      _meas.baro.pressure_pa = pa;
      _meas.baro.stddev_m = _cfg.baroSigmaM;
    }
    _lastPressure = pa;
  }

  // --- The filter. Check the stack first: running out is silent corruption.
  _stackFree = freeStackBytes();
  const uint32_t t0 = micros();
  nav_suite_update(&_suite, &_meas);
  _navLastUs = micros() - t0;
  if (_navLastUs > _navMaxUs) _navMaxUs = _navLastUs;

#if defined(ARDUINO_ARCH_ESP32)
  _stackFree = freeStackBytes();  // high-water mark, now including the filter call
  if (_stackFree >= 0 && _stackFree < 256) _stackOk = false;
#else
  if (_stackFree >= 0 && _stackFree < TRISENSE_INS_STACK_NEEDED) _stackOk = false;
#endif
}

// ===========================================================================
// Inputs
// ===========================================================================

bool TriSenseINS::feedGnss(double latDeg, double lonDeg, float altMslM, float hdop, uint8_t sats,
                           float speedMps, float courseDeg, uint32_t fixAgeMs, float geoidSeparationM) {
  TriSenseGnssFix f;
  f.latDeg = latDeg;
  f.lonDeg = lonDeg;
  f.altMslM = altMslM;
  f.hdop = hdop;
  f.sats = sats;
  f.speedMps = speedMps;
  f.courseDeg = courseDeg;
  f.ageMs = fixAgeMs;
  f.geoidSeparationM = geoidSeparationM;
  return feedGnss(f);
}

bool TriSenseINS::feedGnss(const TriSenseGnssFix &fix) {
  if (!_begun) return false;
  if (!isfinite(fix.latDeg) || !isfinite(fix.lonDeg) || !isfinite(fix.altMslM)) return false;
  if (fix.sats != 0 && fix.sats < _cfg.gnssMinSats) return false;

  float sigmaH, sigmaV;
  if (isfinite(fix.horizSigmaM) && fix.horizSigmaM > 0.0f) {
    sigmaH = fix.horizSigmaM;
    sigmaV = (isfinite(fix.vertSigmaM) && fix.vertSigmaM > 0.0f) ? fix.vertSigmaM
                                                                 : sigmaH * _cfg.gnssVertOverHorz;
  } else {
    if (!isfinite(fix.hdop) || !(fix.hdop > 0.0f) || fix.hdop > _cfg.gnssMaxHdop) return false;
    sigmaH = _cfg.gnssUereM * fix.hdop;
    if (sigmaH < _cfg.gnssSigmaFloorM) sigmaH = _cfg.gnssSigmaFloorM;
    sigmaV = sigmaH * _cfg.gnssVertOverHorz;
  }

  // MSL altitude + geoid separation = the ellipsoid height INSLIB works in.
  const float geoid = isfinite(fix.geoidSeparationM) ? fix.geoidSeparationM : _cfg.geoidSeparationM;
  _lastGeoidSepM = geoid;

  ins_meas_gnss_pos_t &p = _pending.gnss_pos;
  p.is_valid = true;
  p.llh[0] = fix.latDeg * DEG2RAD;
  p.llh[1] = fix.lonDeg * DEG2RAD;
  p.llh[2] = (double)fix.altMslM + (double)geoid;
  diagCov(p.Qll_ned, sigmaH * sigmaH, sigmaH * sigmaH, sigmaV * sigmaV);

  const uint32_t age = fix.ageMs > 400 ? 400 : fix.ageMs;
  const int delayMs = (int)_cfg.gnssLatencyMs + (int)age;
  _pending.gnss_delay_ms = delayMs;
  _gnssFedUs = nowUs();

  // Full NED velocity only if the receiver really measures it per axis (UBX);
  // NMEA has no vertical velocity, so it goes in as a scalar speed below.
  if (isfinite(fix.velN) && isfinite(fix.velE) && isfinite(fix.velD) &&
      isfinite(fix.velSigmaMps) && fix.velSigmaMps > 0.0f) {
    ins_meas_gnss_vel_t &v = _pending.gnss_vel;
    v.is_valid = true;
    v.vel_ned[0] = fix.velN;
    v.vel_ned[1] = fix.velE;
    v.vel_ned[2] = fix.velD;
    const float s2 = fix.velSigmaMps * fix.velSigmaMps;
    diagCov(v.Qll_ned, s2, s2, s2);
  } else if (_cfg.gnssUseSpeed && isfinite(fix.speedMps) && fix.speedMps >= 0.0f) {
    _pending.speed.is_valid = true;
    _pending.speed.speed_mps = fix.speedMps;
    _pending.speed.stddev_mps = _cfg.gnssSpeedSigmaMps;
    _pending.speed_delay_ms = delayMs;
    _speedFedUs = _gnssFedUs;
  }

  if (_cfg.gnssUseCourseAsHeading && isfinite(fix.courseDeg) && isfinite(fix.speedMps) &&
      fix.speedMps >= _cfg.gnssCourseMinSpeedMps) {
    float c = wrap360(fix.courseDeg);
    if (c > 180.0f) c -= 360.0f;
    _pending.yaw.is_valid = true;
    _pending.yaw.yaw_rad = c * DEG2RADF;  // NED yaw == course over ground
    _pending.yaw.stddev_rad = _cfg.gnssCourseSigmaDeg * DEG2RADF;
    _pending.yaw_delay_ms = delayMs;
    _yawFedUs = _gnssFedUs;
  }

  // Declination and field strength from the World Magnetic Model, once we
  // know where on Earth we are.
  if (!_magModelSet) setMagneticModelPosition(fix.latDeg, fix.lonDeg);
  return true;
}

void TriSenseINS::feedSpeed(float speedMps, float sigmaMps, uint16_t delayMs) {
  if (!isfinite(speedMps) || speedMps < 0.0f) return;
  _pending.speed.is_valid = true;
  _pending.speed.speed_mps = speedMps;
  _pending.speed.stddev_mps = sigmaMps > 0.0f ? sigmaMps : 0.0f;  // 0 -> INSLIB default
  _pending.speed_delay_ms = delayMs;
  _speedFedUs = nowUs();
}

void TriSenseINS::feedHeading(float headingDeg, float sigmaDeg, uint16_t delayMs) {
  if (!isfinite(headingDeg) || !(sigmaDeg > 0.0f)) return;
  float h = wrap360(headingDeg);
  if (h > 180.0f) h -= 360.0f;
  _pending.yaw.is_valid = true;
  _pending.yaw.yaw_rad = h * DEG2RADF;
  _pending.yaw.stddev_rad = sigmaDeg * DEG2RADF;
  _pending.yaw_delay_ms = delayMs;
  _yawFedUs = nowUs();
}

void TriSenseINS::zeroVelocityUpdate() {
  _pending.zero_velocity_update = true;
  _pending.zero_rotation_update = true;
}

// ===========================================================================
// Outputs
// ===========================================================================

// sqrt of one diagonal element of P = U * diag(d) * U' (INSLIB keeps the
// covariance factorised). Same equation as INSLIB's own udu_get_diag().
float TriSenseINS::stateStdDev(int idx) const {
  const ins_t &f = _suite.ins;
  if (!f.is_initialized || idx < 0 || idx >= f.n) return NAN;
  const int n = f.n;
  float s = f.d[idx];
  for (int k = idx + 1; k < n; ++k) {
    const float u = f.U[idx + k * n];  // column-major, n x n
    s += u * u * f.d[k];
  }
  return s > 0.0f ? sqrtf(s) : 0.0f;
}

bool TriSenseINS::getRPYRad(float &roll, float &pitch, float &yaw) const {
  return nav_suite_get_rpy(&_suite, &roll, &pitch, &yaw);
}

bool TriSenseINS::getAttitude(TriSenseINSAttitudeSource source, float &rollDeg, float &pitchDeg,
                              float &yawDeg) const {
  float r, p, y;
  bool ok;
  switch (source) {
    case TriSenseINSAttitudeSource::INS:  ok = nav_suite_get_rpy_ins(&_suite, &r, &p, &y); break;
    case TriSenseINSAttitudeSource::AHRS: ok = nav_suite_get_rpy_ahrs(&_suite, &r, &p, &y); break;
    case TriSenseINSAttitudeSource::ARS:  ok = nav_suite_get_rpy_ars(&_suite, &r, &p, &y); break;
    default:                              ok = nav_suite_get_rpy(&_suite, &r, &p, &y); break;
  }
  if (!ok) return false;
  rollDeg = r * RAD2DEGF;
  pitchDeg = p * RAD2DEGF;
  yawDeg = y * RAD2DEGF;
  // A manual declination only makes sense while the filter references
  // magnetic north, i.e. before a magnetic-model position is known.
  if (!_magModelSet) yawDeg += _declinationDeg;
  yawDeg = wrap360(yawDeg);
  return true;
}

bool TriSenseINS::getRPY(float &rollDeg, float &pitchDeg, float &yawDeg) const {
  return getAttitude(TriSenseINSAttitudeSource::Best, rollDeg, pitchDeg, yawDeg);
}

float TriSenseINS::getHeading() const {
  float r, p, y;
  return getRPY(r, p, y) ? y : NAN;
}

bool TriSenseINS::getRPYStdDev(float &rollDeg, float &pitchDeg, float &yawDeg) const {
  float r, p, y;
  if (!ins_get_rpy_stddev(&_suite.ins, &r, &p, &y)) return false;
  rollDeg = r * RAD2DEGF;
  pitchDeg = p * RAD2DEGF;
  yawDeg = y * RAD2DEGF;
  return true;
}

bool TriSenseINS::getQuaternion(float &w, float &x, float &y, float &z) const {
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  // Same source order as nav_suite_get_rpy(): INS, then AHRS, then ARS.
  bool ok = ins_is_ready(&_suite.ins) && ins_get_quaternion(&_suite.ins, q);
  if (!ok) ok = ahrs_get_quaternion(&_suite.ahrs, q);
  if (!ok) ok = ahrs_get_quaternion(&_suite.ars, q);
  if (!ok) return false;
  w = q[0]; x = q[1]; y = q[2]; z = q[3];
  return true;
}

bool TriSenseINS::getRotationMatrix(float R[3][3]) const {
  float q[4];
  if (!getQuaternion(q[0], q[1], q[2], q[3])) return false;
  quatToMat(q, R);
  return true;
}

bool TriSenseINS::getAngularRate(float &rollRate, float &pitchRate, float &yawRate) const {
  float w[3];
  if (!ins_get_omega_b_nb(&_suite.ins, w)) {
    // INS not running (e.g. no GNSS yet): last gyro epoch minus the ARS bias.
    if (!_meas.gyr.is_valid) return false;
    float b[3] = {0.0f, 0.0f, 0.0f};
    ahrs_get_bias_gyr(&_suite.ars, b);
    for (int i = 0; i < 3; i++) w[i] = _meas.gyr.data[i] - b[i];
  }
  rollRate = w[0] * RAD2DEGF;
  pitchRate = w[1] * RAD2DEGF;
  yawRate = w[2] * RAD2DEGF;
  return true;
}

bool TriSenseINS::getPositionNED(float &north, float &east, float &down) const {
  float p[3];
  if (!ins_get_position_local(&_suite.ins, p)) return false;
  north = p[0]; east = p[1]; down = p[2];
  return true;
}

bool TriSenseINS::getPositionStdDev(float &north, float &east, float &down) const {
  north = stateStdDev(INS_IDX_POS + 0);
  east = stateStdDev(INS_IDX_POS + 1);
  down = stateStdDev(INS_IDX_POS + 2);
  return isfinite(north);
}

bool TriSenseINS::getLatLonH(double &latDeg, double &lonDeg, float &heightEllipsoidM) const {
  if (!_suite.wgs84_anchor_seen) return false;
  double llh[3];
  if (!ins_get_latlonh(&_suite.ins, llh)) return false;
  latDeg = llh[0] * RAD2DEG;
  lonDeg = llh[1] * RAD2DEG;
  heightEllipsoidM = (float)llh[2];
  return true;
}

bool TriSenseINS::getPositionECEF(double &x, double &y, double &z) const {
  if (!_suite.wgs84_anchor_seen) return false;
  double e[3];
  if (!ins_get_position_ecef(&_suite.ins, e)) return false;
  x = e[0]; y = e[1]; z = e[2];
  return true;
}

bool TriSenseINS::localToLatLon(float north, float east, float down, double &latDeg, double &lonDeg,
                                float &hEllM) const {
  const float ned[3] = {north, east, down};
  double la, lo, h;
  if (!nav_suite_local_to_wgs84(&_suite, ned, &la, &lo, &h)) return false;
  latDeg = la * RAD2DEG;
  lonDeg = lo * RAD2DEG;
  hEllM = (float)h;
  return true;
}

bool TriSenseINS::latLonToLocal(double latDeg, double lonDeg, float hEllM, float &north, float &east,
                                float &down) const {
  float ned[3];
  if (!nav_suite_wgs84_to_local(&_suite, latDeg * DEG2RAD, lonDeg * DEG2RAD, hEllM, ned)) return false;
  north = ned[0]; east = ned[1]; down = ned[2];
  return true;
}

bool TriSenseINS::getVelocityNED(float &vn, float &ve, float &vd) const {
  float v[3];
  if (!ins_get_velocity_ned(&_suite.ins, v)) return false;
  vn = v[0]; ve = v[1]; vd = v[2];
  return true;
}

bool TriSenseINS::getVelocityStdDev(float &vn, float &ve, float &vd) const {
  vn = stateStdDev(INS_IDX_VEL + 0);
  ve = stateStdDev(INS_IDX_VEL + 1);
  vd = stateStdDev(INS_IDX_VEL + 2);
  return isfinite(vn);
}

bool TriSenseINS::getVelocityECEF(float &vx, float &vy, float &vz) const {
  float v[3];
  if (!ins_get_velocity_ecef(&_suite.ins, v)) return false;
  vx = v[0]; vy = v[1]; vz = v[2];
  return true;
}

float TriSenseINS::getGroundSpeed() const {
  float vn, ve, vd;
  return getVelocityNED(vn, ve, vd) ? sqrtf(vn * vn + ve * ve) : NAN;
}

float TriSenseINS::getCourse() const {
  float vn, ve, vd;
  if (!getVelocityNED(vn, ve, vd)) return NAN;
  return wrap360(atan2f(ve, vn) * RAD2DEGF);
}

bool TriSenseINS::getAccelerationNED(float &an, float &ae, float &ad) const {
  float a[3];
  if (!ins_get_acc_n(&_suite.ins, a)) {
    // INS not running: rotate the last specific force with the best attitude
    // and add gravity back. No accelerometer bias correction on this path.
    float R[3][3];
    if (!_meas.acc.is_valid || !getRotationMatrix(R)) return false;
    const float *f = _meas.acc.data;
    for (int i = 0; i < 3; i++) a[i] = R[i][0] * f[0] + R[i][1] * f[1] + R[i][2] * f[2];
    a[2] += G_MPS2;
  }
  an = a[0]; ae = a[1]; ad = a[2];
  return true;
}

bool TriSenseINS::getAccelerationBody(float &ax, float &ay, float &az) const {
  float an, ae, ad, R[3][3];
  if (!getAccelerationNED(an, ae, ad) || !getRotationMatrix(R)) return false;
  // body = R^T * ned
  ax = R[0][0] * an + R[1][0] * ae + R[2][0] * ad;
  ay = R[0][1] * an + R[1][1] * ae + R[2][1] * ad;
  az = R[0][2] * an + R[1][2] * ae + R[2][2] * ad;
  return true;
}

bool TriSenseINS::getBaroAltitude(float &heightM, float &verticalSpeedMps) const {
  return nav_suite_get_baro_alt(&_suite, &heightM, &verticalSpeedMps);
}

bool TriSenseINS::getHeight(float &heightM) const { return nav_suite_get_height(&_suite, &heightM); }

bool TriSenseINS::getVerticalSpeed(float &mps) const {
  float v[3];
  if (getMode() == TriSenseINSMode::Full && ins_get_velocity_ned(&_suite.ins, v)) {
    mps = -v[2];
    return true;
  }
  if (nav_suite_get_baro_alt(&_suite, NULL, &mps)) return true;
  if (ins_get_velocity_ned(&_suite.ins, v)) {
    mps = -v[2];
    return true;
  }
  return false;
}

bool TriSenseINS::getHeightEllipsoid(float &m) const { return nav_suite_get_height_ellipsoid(&_suite, &m); }

bool TriSenseINS::getHeightMSL(float &m) const {
  float h;
  if (!nav_suite_get_height_ellipsoid(&_suite, &h)) return false;
  m = h - (isfinite(_lastGeoidSepM) ? _lastGeoidSepM : _cfg.geoidSeparationM);
  return true;
}

bool TriSenseINS::getPressureAltitude(float &m) const {
  if (baro_alt_get_isa_altitude(&_suite.baro_alt, &m)) return true;
  if (!isfinite(_lastPressure)) return false;
  m = baro_alt_pressure_to_altitude(_lastPressure);
  return true;
}

bool TriSenseINS::getAccelBias(float &x, float &y, float &z) const {
  float b[3];
  if (!ins_get_bias_acc(&_suite.ins, b)) return false;
  x = b[0]; y = b[1]; z = b[2];
  return true;
}

bool TriSenseINS::getGyroBias(float &x, float &y, float &z) const {
  float b[3];
  if (!ins_get_bias_gyr(&_suite.ins, b) && !ahrs_get_bias_gyr(&_suite.ars, b)) return false;
  x = b[0] * RAD2DEGF; y = b[1] * RAD2DEGF; z = b[2] * RAD2DEGF;
  return true;
}

bool TriSenseINS::getAccelBiasStdDev(float &x, float &y, float &z) const {
  x = stateStdDev(INS_IDX_ACC + 0);
  y = stateStdDev(INS_IDX_ACC + 1);
  z = stateStdDev(INS_IDX_ACC + 2);
  return isfinite(x);
}

bool TriSenseINS::getGyroBiasStdDev(float &x, float &y, float &z) const {
  x = stateStdDev(INS_IDX_GYR + 0) * RAD2DEGF;
  y = stateStdDev(INS_IDX_GYR + 1) * RAD2DEGF;
  z = stateStdDev(INS_IDX_GYR + 2) * RAD2DEGF;
  return isfinite(x);
}

TriSenseINSMode TriSenseINS::getMode() const {
  switch (nav_suite_get_mode(&_suite)) {
    case NAV_SUITE_MODE_ATTITUDE_ONLY: return TriSenseINSMode::AttitudeOnly;
    case NAV_SUITE_MODE_COASTING:      return TriSenseINSMode::Coasting;
    case NAV_SUITE_MODE_FULL:          return TriSenseINSMode::Full;
    default:                           return TriSenseINSMode::None;
  }
}

const char *TriSenseINS::modeName(TriSenseINSMode mode) {
  switch (mode) {
    case TriSenseINSMode::AttitudeOnly: return "ATT";
    case TriSenseINSMode::Coasting:     return "COAST";
    case TriSenseINSMode::Full:         return "FULL";
    default:                            return "NONE";
  }
}

bool TriSenseINS::isReady() const { return ins_is_ready(&_suite.ins); }

bool TriSenseINS::hasAttitude() const {
  float r, p, y;
  return nav_suite_get_rpy(&_suite, &r, &p, &y);
}

bool TriSenseINS::hasPosition() const {
  const TriSenseINSMode m = getMode();
  return _suite.wgs84_anchor_seen && (m == TriSenseINSMode::Full || m == TriSenseINSMode::Coasting);
}

bool TriSenseINS::isZuptActive() const { return nav_suite_get_zaru_active(&_suite); }
bool TriSenseINS::isVerticalZuptActive() const { return nav_suite_get_vertical_zupt_active(&_suite); }

uint32_t TriSenseINS::getDeadReckoningMs() const {
  const int ms = ins_deadreckoning_ms(&_suite.ins);
  return ms > 0 ? (uint32_t)ms : 0;
}
