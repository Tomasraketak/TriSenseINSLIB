/*
 * TriSenseINS.h - Voltino TriSense + INSLIB navigation filter.
 *
 * Part of TriSenseINSLIB. SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * Copyright (c) 2026 VoltinoLabs (Tomas Michal)
 * Contains INSLIB (c) Jan Zwiener, AGPL-3.0, and KFCore, BSD-3-Clause.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Affero General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version. See LICENSE.
 *
 * READ THIS FIRST: because INSLIB is AGPL-3.0, any firmware you build with this
 * library is AGPL-3.0 too. If you distribute that firmware (or let people use
 * it over a network) you must offer its complete source code. For closed-source
 * commercial use you need a commercial licence from INSLIB's author.
 *
 * ---------------------------------------------------------------------------
 * What it does
 * ---------------------------------------------------------------------------
 * TriSenseINS reads the IMU FIFO, magnetometer and barometer of a TriSense
 * module through the Voltino_TriSense library, converts everything into the
 * units and the body frame INSLIB expects (SI, front-right-down), and drives
 * INSLIB's nav_suite: a 15-state error-state Kalman INS running in parallel
 * with two AHRS filters and a barometric vertical channel.
 *
 *   #include <TriSense.h>
 *   #include <TriSenseINS.h>
 *
 *   TriSense    sensor;
 *   TriSenseINS ins(sensor);
 *
 *   void setup() {
 *     sensor.beginAll(MODE_HYBRID, 17, 10000000);
 *     ins.begin();
 *   }
 *   void loop() {
 *     ins.update();
 *     float roll, pitch, yaw;  ins.getRPY(roll, pitch, yaw);
 *     float h, vz;             ins.getBaroAltitude(h, vz);
 *   }
 *
 * Without GNSS you get attitude (roll, pitch, heading), height above the start
 * point and vertical speed - the rocket / CanSat use case. Feed GNSS fixes with
 * feedGnss() and you get full position, velocity and acceleration.
 *
 * Units: degrees, metres, seconds, unless a function name ends in Rad.
 * Frames: body = FRD (x forward, y right, z down), navigation = NED (north,
 * east, down). Height and vertical speed are reported POSITIVE UP.
 */
#ifndef TRISENSE_INS_H
#define TRISENSE_INS_H

#include <Arduino.h>
#include <TriSense.h>

// Must come before the INSLIB headers, see the note inside.
#include "trisense_inslib_config.h"
#include "inslib/nav_suite.h"

#define TRISENSE_INSLIB_VERSION "0.1.0"
#define TRISENSE_INSLIB_INSLIB_VERSION "1.4.2"

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

// Solution mode, the same as INSLIB's nav_suite_mode_t.
enum class TriSenseINSMode : uint8_t {
  None = 0,       // nothing usable yet (first ~1.5 s after begin())
  AttitudeOnly,   // attitude + baro height available, no position solution
  Coasting,       // INS running on IMU only, position drifting (GNSS lost)
  Full            // INS with recent GNSS aiding
};

// Which filter an attitude comes from.
enum class TriSenseINSAttitudeSource : uint8_t {
  Best = 0,  // INS if ready, else the magnetometer AHRS, else the ARS
  INS,       // the 15-state INS
  AHRS,      // roll/pitch/yaw filter, yaw from the magnetometer
  ARS        // roll/pitch filter, yaw only integrated from the gyro (drifts)
};

// How the TriSense board sits in the vehicle. Fine rotations on top of this
// with setMountingRPY(), arbitrary ones with setMountingRotation().
enum class TriSenseINSMount : uint8_t {
  ComponentsUp = 0,  // board flat, components up, board X = vehicle forward
  ComponentsDown     // board flat, upside down, board X = vehicle forward
};

// ---------------------------------------------------------------------------
// Configuration. Every default is a sensible starting point for a TriSense on
// a Pico 2 with a consumer GNSS receiver; set only what you need.
// ---------------------------------------------------------------------------
struct TriSenseINSConfig {
  // --- Rates ----------------------------------------------------------------
  uint16_t navRateHz  = 200;  // filter epochs per second (IMU samples are averaged per epoch)
  uint16_t magRateHz  = 50;   // magnetometer polling rate
  uint16_t baroRateHz = 50;   // barometer polling rate (match the BMP580 ODR)
  // Covariance prediction rate. The strapdown runs at navRateHz; only the
  // (expensive) covariance propagation is throttled to this rate.
  float covarianceRateHz = 50.0f;

  // --- Sensor setup done by begin() ------------------------------------------
  // true: begin() sets the IMU to imuOdr with the 16-bit FIFO, and the
  // barometer to 50 Hz, OSR x8, IIR off (an IIR would correlate samples the
  // Kalman filter treats as independent). false: leave the sensors as they are.
  bool    configureSensors = true;
  ICM_ODR imuOdr = ODR_1KHZ;
  // Boot-time gyro bias estimate (board must be still). INSLIB keeps
  // estimating the bias afterwards; this only makes the start cleaner.
  bool     calibrateGyro = true;
  uint16_t gyroCalibrationSamples = 1000;

  // --- Which sensors the filter uses -----------------------------------------
  bool useMagnetometer = true;
  bool useBarometer    = true;

  // --- Sensor noise ------------------------------------------------------------
  // 0 = INSLIB's conservative consumer-MEMS defaults. Measure your own with an
  // Allan variance run (INSLIB tools/allan_variance.py) before changing.
  float accelNoisePSD = 0.0f;  // (m/s^2)^2/Hz per axis
  float gyroNoisePSD  = 0.0f;  // (rad/s)^2/Hz per axis
  float baroSigmaM    = 0.6f;  // barometric altitude noise, 1-sigma [m]

  // --- Magnetometer --------------------------------------------------------
  // Reject magnetometer samples whose field strength is off by more than this
  // fraction (needs the magnetic model, i.e. a position). 0 = INSLIB default.
  float magFieldTolerance = 0.30f;
  // Decimal year for the World Magnetic Model (declination, inclination).
  float wmmYear = 2026.5f;

  // --- GNSS ------------------------------------------------------------------
  float gnssUereM        = 2.2f;   // horizontal 1-sigma per unit of HDOP [m]
  float gnssSigmaFloorM  = 1.5f;   // no fix is trusted better than this [m]
  float gnssVertOverHorz = 1.9f;   // vertical / horizontal error ratio (no VDOP in NMEA)
  float gnssMaxHdop      = 6.0f;   // fixes worse than this are ignored
  uint8_t gnssMinSats    = 4;      // fixes with fewer satellites are ignored
  // How old a fix is when the NMEA sentence has finished arriving [ms]. INSLIB
  // applies the correction that far back in its state history.
  uint16_t gnssLatencyMs = 150;
  // INSLIB only enters a 3D solution once the fixes have been better than the
  // "start" limits for a while, and leaves it when they stay worse than the
  // "stop" limits. Keep them consistent with gnssUereM (HDOP 1.0 * 2.2 m must
  // pass the start gate) or the filter never starts.
  float gnssStartMaxHorzM = 6.0f;
  float gnssStartMaxVertM = 12.0f;
  float gnssStopMaxHorzM  = 15.0f;
  float gnssStopMaxVertM  = 30.0f;
  // GNSS ground speed fused as a scalar speed measurement.
  bool  gnssUseSpeed      = true;
  float gnssSpeedSigmaMps = 0.3f;
  // GNSS course over ground fused as heading. ONLY for vehicles that cannot
  // move sideways (car, boat, plane). Wrong for multicopters, people, rockets.
  bool  gnssUseCourseAsHeading = false;
  float gnssCourseMinSpeedMps  = 3.0f;
  float gnssCourseSigmaDeg     = 8.0f;
  // Geoid separation used when feedGnss() is not given one [m]. NMEA altitude
  // is above mean sea level; INSLIB works in ellipsoid height. Czechia ~ +44 m.
  float geoidSeparationM = 44.0f;

  // --- Behaviour ---------------------------------------------------------------
  // How long the INS keeps a position solution without GNSS [s].
  float maxDeadReckoningSec = 120.0f;
  // Automatic zero-velocity / zero-rotation updates when the board is still.
  // Good on the ground, consider disabling in flight (setAutoZupt(false)).
  bool autoZupt = true;
  // Outlier rejection. Leave on.
  bool outlierRejection = true;
};

// A GNSS fix with everything feedGnss() can use. Leave a field NAN (or 0 where
// noted) if your receiver does not report it.
struct TriSenseGnssFix {
  double latDeg = NAN;
  double lonDeg = NAN;
  float  altMslM = NAN;          // altitude above mean sea level [m]
  float  geoidSeparationM = NAN; // NMEA GGA field 11; NAN -> config default
  float  hdop = NAN;             // used for the covariance if horizSigmaM is NAN
  uint8_t sats = 0;              // 0 = unknown (no satellite gate)
  float  horizSigmaM = NAN;      // 1-sigma horizontal accuracy, if the receiver reports it
  float  vertSigmaM  = NAN;      // 1-sigma vertical accuracy
  float  speedMps  = NAN;        // ground speed
  float  courseDeg = NAN;        // course over ground
  // Full NED velocity, for receivers that report it (UBX-NAV-PVT velN/E/D).
  float  velN = NAN, velE = NAN, velD = NAN;
  float  velSigmaMps = NAN;      // 1-sigma per axis for velN/E/D
  uint32_t ageMs = 0;            // time since the fix arrived, on top of gnssLatencyMs
};

// ---------------------------------------------------------------------------
// The filter
// ---------------------------------------------------------------------------
class TriSenseINS {
public:
  explicit TriSenseINS(TriSense &sensor);

  // ---- Life cycle ----------------------------------------------------------
  // Call after sensor.beginAll(). Configures the sensors (see
  // TriSenseINSConfig::configureSensors), calibrates the gyro if asked to (keep
  // the board still), and initialises the filter. False if INSLIB rejected the
  // configuration.
  bool begin();
  bool begin(const TriSenseINSConfig &config);

  // Restart the filter from scratch with the current configuration and
  // calibration. The sensors are not touched.
  bool reset();

  // Call as often as possible from loop(). Drains the IMU FIFO, polls the
  // magnetometer and barometer, and runs one filter epoch whenever
  // 1/navRateHz has elapsed. Returns true when an epoch ran (i.e. the outputs
  // changed). On RP2350 one epoch takes ~0.1-1 ms.
  bool update();

  // ---- Inputs --------------------------------------------------------------
  // GNSS fix, e.g. from TinyGPSPlus. Returns false if the fix fails the
  // HDOP/satellite gate. The fix is used in the next epoch.
  bool feedGnss(double latDeg, double lonDeg, float altMslM, float hdop, uint8_t sats,
                float speedMps = NAN, float courseDeg = NAN, uint32_t fixAgeMs = 0,
                float geoidSeparationM = NAN);
  bool feedGnss(const TriSenseGnssFix &fix);

  // Scalar speed from odometry (wheel, airspeed is NOT ground speed) [m/s].
  void feedSpeed(float speedMps, float sigmaMps = 0.0f, uint16_t delayMs = 0);
  // Absolute heading from an external source (dual-antenna GNSS, ...) [deg].
  void feedHeading(float headingDeg, float sigmaDeg, uint16_t delayMs = 0);
  // "I know the vehicle is not moving" for the next epoch (e.g. on the launch pad).
  void zeroVelocityUpdate();

  // Automatic stillness detection (ZUPT/ZARU) on/off at run time.
  void setAutoZupt(bool enable);

  // Magnetometer calibration from the MotionCal example, in the TriSense's own
  // axes - the same numbers you give TriSenseFusion::setMagCalibration().
  void setMagCalibration(const float hardIron[3], const float softIron[3][3]);
  void setMagHardIron(float x, float y, float z);

  // Position for the World Magnetic Model, so the heading refers to TRUE north
  // and the magnetic disturbance gate is armed. Done automatically on the first
  // GNSS fix; call it yourself without GNSS (e.g. with the launch site).
  void setMagneticModelPosition(double latDeg, double lonDeg);
  void setMagneticModelPosition(double latDeg, double lonDeg, float year);
  // Manual declination [deg, east positive], added to the reported heading
  // ONLY while no magnetic-model position is known. Prefer the call above.
  void setDeclination(float declinationDeg);

  // A known initial heading, e.g. the launch rail azimuth [deg, 1-sigma deg].
  // Applied at begin()/reset(), so call it before those.
  void setInitialHeading(float headingDeg, float sigmaDeg);

  // Board orientation in the vehicle. Call before begin() or call reset()
  // afterwards. ComponentsUp is the default.
  void setMounting(TriSenseINSMount mount);
  // Extra rotation of the board relative to the vehicle, ZYX Euler [deg],
  // applied on top of the setMounting() preset.
  void setMountingRPY(float rollDeg, float pitchDeg, float yawDeg);
  // Full control: v_body_FRD = R * v_trisense (row-major).
  void setMountingRotation(const float R[3][3]);

  // ---- Attitude ------------------------------------------------------------
  // Roll, pitch [-180..180 / -90..90 deg] and yaw as a compass heading
  // [0..360 deg, true north once a position is known]. From the best source.
  bool  getRPY(float &rollDeg, float &pitchDeg, float &yawDeg) const;
  bool  getRPYRad(float &roll, float &pitch, float &yaw) const;
  bool  getAttitude(TriSenseINSAttitudeSource source, float &rollDeg, float &pitchDeg, float &yawDeg) const;
  float getHeading() const;   // [0..360 deg], NAN if unknown
  // 1-sigma attitude uncertainty of the INS [deg].
  bool  getRPYStdDev(float &rollDeg, float &pitchDeg, float &yawDeg) const;
  // Body (FRD) to NED quaternion, Hamilton convention. Use this instead of
  // Euler angles near pitch +-90 deg (a rocket on the pad).
  bool  getQuaternion(float &w, float &x, float &y, float &z) const;
  // Body (FRD) to NED rotation matrix, row-major: v_ned = R * v_body.
  bool  getRotationMatrix(float R[3][3]) const;
  // Angular rate of the body, gyro bias removed [deg/s, FRD].
  bool  getAngularRate(float &rollRate, float &pitchRate, float &yawRate) const;

  // ---- Position (needs GNSS) --------------------------------------------------
  // Relative to the NED origin (the first accepted fix) [m].
  bool  getPositionNED(float &north, float &east, float &down) const;
  bool  getPositionStdDev(float &north, float &east, float &down) const;
  bool  getLatLonH(double &latDeg, double &lonDeg, float &heightEllipsoidM) const;
  bool  getPositionECEF(double &x, double &y, double &z) const;
  // Convert between the local NED frame and WGS84 (e.g. waypoints).
  bool  localToLatLon(float north, float east, float down, double &latDeg, double &lonDeg, float &hEllM) const;
  bool  latLonToLocal(double latDeg, double lonDeg, float hEllM, float &north, float &east, float &down) const;

  // ---- Velocity -------------------------------------------------------------
  bool  getVelocityNED(float &vn, float &ve, float &vd) const;
  bool  getVelocityStdDev(float &vn, float &ve, float &vd) const;
  bool  getVelocityECEF(float &vx, float &vy, float &vz) const;
  float getGroundSpeed() const;  // horizontal [m/s], NAN if unknown
  float getCourse() const;       // direction of travel [0..360 deg], NAN if unknown

  // ---- Acceleration ---------------------------------------------------------
  // Acceleration with gravity removed, in NED [m/s^2] (down positive).
  bool  getAccelerationNED(float &an, float &ae, float &ad) const;
  // The same in the body frame (FRD).
  bool  getAccelerationBody(float &ax, float &ay, float &az) const;

  // ---- Height ------------------------------------------------------------------
  // Barometric vertical channel: height above the start point and vertical
  // speed, both POSITIVE UP. Works without GNSS. This is your rocket altimeter.
  bool  getBaroAltitude(float &heightM, float &verticalSpeedMps) const;
  // Height above the start point from the best source (INS with GNSS, else baro).
  bool  getHeight(float &heightM) const;
  // Vertical speed, positive up, from the best source.
  bool  getVerticalSpeed(float &mps) const;
  // Absolute height above the WGS84 ellipsoid / above mean sea level.
  bool  getHeightEllipsoid(float &m) const;
  bool  getHeightMSL(float &m) const;
  // Standard-atmosphere pressure altitude (QNE, not corrected for weather).
  bool  getPressureAltitude(float &m) const;

  // ---- Estimated sensor errors ------------------------------------------------
  bool  getAccelBias(float &x, float &y, float &z) const;  // [m/s^2, FRD]
  bool  getGyroBias(float &x, float &y, float &z) const;   // [deg/s, FRD]
  bool  getAccelBiasStdDev(float &x, float &y, float &z) const;
  bool  getGyroBiasStdDev(float &x, float &y, float &z) const;

  // ---- Status -------------------------------------------------------------------
  TriSenseINSMode getMode() const;
  static const char *modeName(TriSenseINSMode mode);  // "NONE", "ATT", "COAST", "FULL"
  bool  isReady() const;               // INS converged (attitude + velocity + position usable)
  bool  hasAttitude() const;           // some attitude source is available
  bool  hasPosition() const;           // GNSS-anchored absolute position available
  bool  isZuptActive() const;          // stillness update applied last epoch
  bool  isVerticalZuptActive() const;
  uint32_t getDeadReckoningMs() const; // time since the last absolute aiding (0 = aided)

  // ---- Health -----------------------------------------------------------------
  uint32_t getEpochRateHz() const   { return _epochHz; }
  uint32_t getLastUpdateUs() const  { return _navLastUs; }
  uint32_t getMaxUpdateUs() const   { return _navMaxUs; }
  uint32_t getDroppedEpochs() const { return _droppedEpochs; }
  uint32_t getImuLostPackets() const;
  // Free stack measured right before the last filter call [bytes], -1 if the
  // platform cannot tell. stackOk() is false once it was ever below
  // TRISENSE_INS_STACK_NEEDED - expect corrupted memory then, see README.
  int   getFreeStack() const { return _stackFree; }
  bool  stackOk() const { return _stackOk; }

  // ---- Raw access -----------------------------------------------------------
  // INSLIB's own diagnostic counters (GNSS used/rejected, resets, ...).
  const ins_diag_t &diagnostics() const { return _suite.ins.diag; }
  // The INSLIB nav_suite, for anything not wrapped above. Read-only use is safe.
  nav_suite_t &suite() { return _suite; }
  const nav_suite_t &suite() const { return _suite; }
  // The measurement bundle handed to INSLIB in the last epoch (FRD, SI units).
  // This is what the FlightLogger example records.
  const ins_measurements_t &lastMeasurements() const { return _meas; }
  const TriSenseINSConfig &config() const { return _cfg; }

private:
  TriSense &_sensor;
  TriSenseINSConfig _cfg;

  // The filter. ~15 KB, so it must never live on a stack.
  nav_suite_t _suite;
  ins_measurements_t _meas;
  ins_measurements_t _pending;  // inputs fed between epochs

  // Mounting: v_frd = _mount * v_sensor.
  float _mountBase[3][3];
  float _mountRPY[3];
  float _mount[3][3];
  float _magHard[3];
  float _magSoft[3][3];

  float _declinationDeg = 0.0f;
  bool  _magModelSet = false;
  double _magModelLat = 0.0, _magModelLon = 0.0;
  float _magModelYear = 2026.5f;
  uint64_t _gnssFedUs = 0, _speedFedUs = 0, _yawFedUs = 0;
  bool  _haveInitHeading = false;
  float _initHeadingRad = 0.0f, _initHeadingSigmaRad = 0.0f;
  float _lastGeoidSepM = NAN;
  float _lastPressure = NAN;

  // IMU batch since the last epoch, already FRD / SI.
  float    _accSum[3];
  float    _gyrSum[3];
  uint32_t _imuCount = 0;

  uint64_t _lastEpochUs = 0;
  uint64_t _lastMagUs = 0;
  uint64_t _lastBaroUs = 0;
  uint64_t _lastStarvedUs = 0;
  uint32_t _microsLast = 0;
  uint64_t _microsHigh = 0;

  uint32_t _droppedEpochs = 0;
  uint32_t _epochCount = 0;
  uint32_t _epochHz = 0;
  uint64_t _rateWindowUs = 0;
  uint32_t _navLastUs = 0;
  uint32_t _navMaxUs = 0;
  int      _stackFree = -1;
  bool     _stackOk = true;
  bool     _begun = false;

  uint64_t nowUs();
  bool     initFilter();
  void     rebuildMount();
  void     toFRD(float x, float y, float z, float out[3]) const;
  bool     readMagnetometer(float out[3]);
  void     runEpoch(uint64_t nowUs, float dt);
  float    stateStdDev(int idx) const;
  static int freeStackBytes();
};

#endif // TRISENSE_INS_H
