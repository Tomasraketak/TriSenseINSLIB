// Host test for TriSenseINSLIB: runs the real library and the real INSLIB on a
// PC against a simulated TriSense (stubs/TriSense.h) and checks that the
// outputs match the simulated truth. It verifies the glue - units, axes,
// mounting, magnetometer calibration, baro, GNSS wiring - not the hardware.
//
//   make -C tests/host        (or see the Makefile)
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include "TriSenseINS.h"

uint64_t g_sim_us = 1000000;
SimTruth g_truth;
static std::mt19937 rng(42);
float sim_noise(float sigma) { std::normal_distribution<float> d(0.0f, sigma); return d(rng); }

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } \
                              else { printf("  ok:   "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const float D2R = 0.017453292519943295f;
static const float G = 9.80665f;
// Earth field in NED for central Europe [uT] (WMM ~ 50 N 14 E: dec ~ +5 deg).
static const float MAG_N[3] = {19.9f, 1.7f, 45.3f};

static float isaPressure(float h) { return 101325.0f * powf(1.0f - 2.25577e-5f * h, 5.25588f); }

static void eul(float r, float p, float y, float R[3][3]) {
  float cr = cosf(r), sr = sinf(r), cp = cosf(p), sp = sinf(p), cy = cosf(y), sy = sinf(y);
  R[0][0] = cy * cp; R[0][1] = cy * sp * sr - sy * cr; R[0][2] = cy * sp * cr + sy * sr;
  R[1][0] = sy * cp; R[1][1] = sy * sp * sr + cy * cr; R[1][2] = sy * sp * cr - cy * sr;
  R[2][0] = -sp;     R[2][1] = cp * sr;                R[2][2] = cp * cr;
}

// Truth for a vehicle at attitude rpy [deg] and height h, accelerating a_n
// (NED, m/s^2), on a board mounted components up (sensor = diag(1,-1,-1)*FRD).
// hardIron is added to the sensor-frame magnetometer, like a real board.
static void setTruth(float rollDeg, float pitchDeg, float yawDeg, float h, const float a_n[3],
                     const float hardIron[3]) {
  float R[3][3];
  eul(rollDeg * D2R, pitchDeg * D2R, yawDeg * D2R, R);
  float f_n[3] = {a_n[0], a_n[1], a_n[2] - G};  // specific force = a - g
  float f_b[3], m_b[3];
  for (int i = 0; i < 3; i++) {
    f_b[i] = R[0][i] * f_n[0] + R[1][i] * f_n[1] + R[2][i] * f_n[2];   // R^T
    m_b[i] = R[0][i] * MAG_N[0] + R[1][i] * MAG_N[1] + R[2][i] * MAG_N[2];
  }
  // FRD -> TriSense axes (components up): x = F, y = -R, z = -D. Accel in g.
  g_truth.acc[0] = f_b[0] / G;  g_truth.acc[1] = -f_b[1] / G;  g_truth.acc[2] = -f_b[2] / G;
  g_truth.gyr[0] = g_truth.gyr[1] = g_truth.gyr[2] = 0.0f;
  g_truth.mag[0] = m_b[0] + hardIron[0];
  g_truth.mag[1] = -m_b[1] + hardIron[1];
  g_truth.mag[2] = -m_b[2] + hardIron[2];
  g_truth.pressure = isaPressure(h);
}

static const float ZERO3[3] = {0, 0, 0};

static void run(TriSenseINS &ins, float seconds) {
  const uint64_t end = g_sim_us + (uint64_t)(seconds * 1e6f);
  while (g_sim_us < end) {
    g_sim_us += 250;  // loop() runs at 4 kHz
    ins.update();
  }
}

static void runLoop(TriSenseINS &ins, float seconds, uint32_t loopUs) {
  const uint64_t end = g_sim_us + (uint64_t)(seconds * 1e6f);
  while (g_sim_us < end) {
    g_sim_us += loopUs;
    ins.update();
  }
}

// Angle of the rotation between an estimated and a true body->NED matrix [deg].
static float rotErrDeg(const float A[3][3], const float B[3][3]) {
  // E = A B^T; angle = atan2(|vee(E - E^T)| / 2, (tr E - 1) / 2), exact at
  // small angles where acos of the trace loses everything below ~0.03 deg.
  double E[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      E[i][j] = (double)A[i][0] * B[j][0] + (double)A[i][1] * B[j][1] + (double)A[i][2] * B[j][2];
  const double vx = E[2][1] - E[1][2], vy = E[0][2] - E[2][0], vz = E[1][0] - E[0][1];
  const double sn = 0.5 * sqrt(vx * vx + vy * vy + vz * vz);
  const double cs = 0.5 * (E[0][0] + E[1][1] + E[2][2] - 1.0);
  return (float)(atan2(sn, cs) * 180.0 / M_PI);
}

static float insErrDeg(const TriSenseINS &ins, float rollDeg, float pitchDeg, float yawDeg) {
  float R[3][3], T[3][3];
  if (!ins.getRotationMatrix(R)) return 999.0f;
  eul(rollDeg * D2R, pitchDeg * D2R, yawDeg * D2R, T);
  return rotErrDeg(R, T);
}

// Level board turning about the vertical at g_yawRate [deg/s] from g_motionT0.
static double g_motionT0 = 1e30, g_motionT1 = 1e30;
static float g_yawRate = 0.0f;
static float yawAt(double t) {
  if (t <= g_motionT0) return 0.0f;
  if (t >= g_motionT1) return (float)(g_yawRate * (g_motionT1 - g_motionT0));
  return (float)(g_yawRate * (t - g_motionT0));
}
static void truthYawRate(double t) {
  setTruth(0, 0, yawAt(t), 0, ZERO3, ZERO3);
  const bool turning = t > g_motionT0 && t < g_motionT1;
  g_truth.gyr[2] = turning ? -g_yawRate : 0.0f;  // FRD z = -sensor z
}

static TriSenseINSConfig timingConfig() {
  TriSenseINSConfig c;
  c.navRateHz = 240;
  c.useMagnetometer = false;  // attitude from the ARS: yaw is pure gyro integration
  c.autoZupt = false;
  return c;
}

// ---------------------------------------------------------------------------
// Scripted body motion: a body rate w_b(t) [rad/s, FRD] integrated in double
// precision in 5 us steps. Each IMU sample gets the MEAN rate over its own
// sample interval (what a rate gyro behind its anti-alias filter reports), the
// accelerometer gets gravity in the current body frame (no translation).
// ---------------------------------------------------------------------------
struct BodyMotion {
  double t = 0;
  double q[4] = {1, 0, 0, 0};  // FRD -> NED
  void (*rate)(double t, double w[3]) = nullptr;
  double gyrAvg[3] = {0, 0, 0};
  double accAvg[3] = {0, 0, -9.80665};  // body specific force, interval mean [m/s^2]
};
static BodyMotion g_bm;

static void dqMul(const double a[4], const double b[4], double o[4]) {
  o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}
static void dqToMat(const double q[4], float R[3][3]) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  R[0][0] = 1 - 2 * (y * y + z * z); R[0][1] = 2 * (x * y - w * z);     R[0][2] = 2 * (x * z + w * y);
  R[1][0] = 2 * (x * y + w * z);     R[1][1] = 1 - 2 * (x * x + z * z); R[1][2] = 2 * (y * z - w * x);
  R[2][0] = 2 * (x * z - w * y);     R[2][1] = 2 * (y * z + w * x);     R[2][2] = 1 - 2 * (x * x + y * y);
}
static void bmSetTruth() {
  float R[3][3];
  dqToMat(g_bm.q, R);
  float f_b[3], m_b[3];
  for (int i = 0; i < 3; i++) {
    f_b[i] = (float)g_bm.accAvg[i];
    m_b[i] = R[0][i] * MAG_N[0] + R[1][i] * MAG_N[1] + R[2][i] * MAG_N[2];
  }
  // FRD -> TriSense axes (components up): x = F, y = -R, z = -D.
  g_truth.acc[0] = f_b[0] / G; g_truth.acc[1] = -f_b[1] / G; g_truth.acc[2] = -f_b[2] / G;
  g_truth.gyr[0] = (float)(g_bm.gyrAvg[0] / D2R);
  g_truth.gyr[1] = (float)(-g_bm.gyrAvg[1] / D2R);
  g_truth.gyr[2] = (float)(-g_bm.gyrAvg[2] / D2R);
  g_truth.mag[0] = m_b[0]; g_truth.mag[1] = -m_b[1]; g_truth.mag[2] = -m_b[2];
  g_truth.pressure = isaPressure(0);
}
static void truthBodyMotion(double t) {
  if (t > g_bm.t) {
    const double span = t - g_bm.t;
    const int n = (int)ceil(span / 5e-6);
    const double h = span / n;
    double sum[3] = {0, 0, 0}, fsum[3] = {0, 0, 0};
    for (int k = 0; k < n; k++) {
      double w[3] = {0, 0, 0};
      if (g_bm.rate) g_bm.rate(g_bm.t + (k + 0.5) * h, w);
      for (int i = 0; i < 3; i++) sum[i] += w[i];
      // Specific force R^T (0, 0, -g), sampled with the attitude at the
      // start of the step (the half-step offset averages out over n steps).
      const double *q = g_bm.q;
      fsum[0] += -G * 2 * (q[1] * q[3] - q[0] * q[2]);
      fsum[1] += -G * 2 * (q[2] * q[3] + q[0] * q[1]);
      fsum[2] += -G * (1 - 2 * (q[1] * q[1] + q[2] * q[2]));
      const double a = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * h;
      const double s = a > 1e-12 ? sin(0.5 * a) / (a / h) : 0.5 * h;
      const double e[4] = {cos(0.5 * a), w[0] * s, w[1] * s, w[2] * s};
      double qn[4];
      dqMul(g_bm.q, e, qn);
      const double nr = 1.0 / sqrt(qn[0] * qn[0] + qn[1] * qn[1] + qn[2] * qn[2] + qn[3] * qn[3]);
      for (int i = 0; i < 4; i++) g_bm.q[i] = qn[i] * nr;
    }
    for (int i = 0; i < 3; i++) {
      g_bm.gyrAvg[i] = sum[i] / n;
      g_bm.accAvg[i] = fsum[i] / n;
    }
    g_bm.t = t;
  }
  bmSetTruth();
}
static void bmStart(void (*rate)(double, double[3])) {
  g_bm = BodyMotion();
  g_bm.t = g_sim_us * 1e-6;
  g_bm.rate = rate;
  g_truth.truthFn = truthBodyMotion;
  bmSetTruth();
}
static void bmStop() { g_truth.truthFn = nullptr; g_bm.rate = nullptr; }

// Run until an epoch has consumed every sample produced so far, so the filter
// state and the truth refer to the same instant.
template <typename OnEpoch>
static void runToEpoch(TriSenseINS &ins, TriSense &sensor, OnEpoch onEpoch) {
  for (int i = 0; i < 100; i++) {
    g_sim_us += 250;
    if (!ins.update()) continue;
    onEpoch();
    if (ins.getImuBacklog() == 0 && fabs(sensor.imu.lastReadT - g_bm.t) < 1e-9) return;
  }
}
static void runToEpoch(TriSenseINS &ins, TriSense &sensor) { runToEpoch(ins, sensor, [] {}); }
// Error of the estimate against the truth: total angle, and the component
// about the vertical (the part no accelerometer leveling can see) [deg].
static void bmErr(const TriSenseINS &ins, float &total, float &vertical) {
  float R[3][3], T[3][3], E[3][3];
  total = vertical = 999.0f;
  if (!ins.getRotationMatrix(R)) return;
  dqToMat(g_bm.q, T);
  total = rotErrDeg(R, T);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) E[i][j] = R[i][0] * T[j][0] + R[i][1] * T[j][1] + R[i][2] * T[j][2];
  vertical = 0.5f * (E[1][0] - E[0][1]) / D2R;
}

// Pure strapdown of what TriSenseINS hands INSLIB, see test 11. The simulated
// gyro has no bias, so the rate is integrated as it is handed over. (INSLIB
// subtracts its bias estimate from it; with the mean of the epoch that is
// all it does, and per-sample integration removes the same estimate before
// integrating and adds it back afterwards, so in both modes the raw rate is
// the one to compare against the truth.)
struct StrapdownCheck {
  double q[4];
  explicit StrapdownCheck(const double q0[4]) { for (int i = 0; i < 4; i++) q[i] = q0[i]; }
  double v[3] = {0, 0, 0};  // NED velocity from the specific force [m/s]
  void epoch(const TriSenseINS &ins) {
    const ins_measurements_t &m = ins.lastMeasurements();
    double w[3];
    for (int i = 0; i < 3; i++) w[i] = (double)m.gyr.data[i];
    const double h = m.strapdown_dt_sec;
    // INSLIB's velocity step: R_k (f h + 1/2 (w x f) h^2) + g h, with R_k the
    // attitude before this epoch's rotation.
    {
      float R[3][3];
      dqToMat(q, R);
      const double f[3] = {m.acc.data[0], m.acc.data[1], m.acc.data[2]};
      const double wf[3] = {w[1] * f[2] - w[2] * f[1], w[2] * f[0] - w[0] * f[2], w[0] * f[1] - w[1] * f[0]};
      double ib[3];
      for (int i = 0; i < 3; i++) ib[i] = f[i] * h + 0.5 * wf[i] * h * h;
      for (int i = 0; i < 3; i++) v[i] += R[i][0] * ib[0] + R[i][1] * ib[1] + R[i][2] * ib[2];
      v[2] += G * h;
    }
    const double a = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * h;
    const double s = a > 1e-12 ? sin(0.5 * a) / (a / h) : 0.5 * h;
    const double e[4] = {cos(0.5 * a), w[0] * s, w[1] * s, w[2] * s};
    double qn[4];
    dqMul(q, e, qn);
    const double nr = 1.0 / sqrt(qn[0] * qn[0] + qn[1] * qn[1] + qn[2] * qn[2] + qn[3] * qn[3]);
    for (int i = 0; i < 4; i++) q[i] = qn[i] * nr;
  }
  void err(float &total, float &vertical) const {
    float R[3][3], T[3][3], E[3][3];
    dqToMat(q, R);
    dqToMat(g_bm.q, T);
    total = rotErrDeg(R, T);
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) E[i][j] = R[i][0] * T[j][0] + R[i][1] * T[j][1] + R[i][2] * T[j][2];
    vertical = 0.5f * (E[1][0] - E[0][1]) / D2R;
  }
};

static double g_coneHz = 0, g_coneDeg = 0, g_motionStart = 0;
static void rateCone(double t, double w[3]) {
  // Body rate of constant amplitude whose axis turns in the x-y plane: the
  // z axis sweeps a cone of half-angle ~coneDeg at coneHz, and the body
  // really turns about z at ~Omega alpha^2 / 2 (the coning motion proper).
  if (t < g_motionStart) { w[0] = w[1] = w[2] = 0; return; }
  const double W = 2 * M_PI * g_coneHz, a = W * g_coneDeg * M_PI / 180.0;
  w[0] = a * cos(W * (t - g_motionStart));
  w[1] = a * sin(W * (t - g_motionStart));
  w[2] = 0;
}

static int g_flipMode = 0;  // 0: fixed axis, 1: tumbling axis
static void rateFlip(double t, double w[3]) {
  const double u = t - g_motionStart;
  w[0] = w[1] = w[2] = 0;
  if (u < 0 || u > 1.0) return;
  // 400 deg/s, eased in and out over 20 ms.
  double env = 1.0;
  if (u < 0.02) env = 0.5 - 0.5 * cos(M_PI * u / 0.02);
  if (u > 0.98) env = 0.5 - 0.5 * cos(M_PI * (1.0 - u) / 0.02);
  double ax[3] = {1, 2, 3};
  if (g_flipMode == 1) {
    ax[0] = 1.0;
    ax[1] = 0.8 * sin(2 * M_PI * 2.0 * u);
    ax[2] = 0.6 * cos(2 * M_PI * 1.3 * u);
  }
  const double n = sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
  for (int i = 0; i < 3; i++) w[i] = 400.0 * M_PI / 180.0 * env * ax[i] / n;
}

// Static pose for the body-motion truth.
static void bmPose(float rollDeg, float pitchDeg, float yawDeg) {
  const double r = rollDeg * M_PI / 360.0, p = pitchDeg * M_PI / 360.0, y = yawDeg * M_PI / 360.0;
  const double qz[4] = {cos(y), 0, 0, sin(y)}, qy[4] = {cos(p), 0, sin(p), 0}, qx[4] = {cos(r), sin(r), 0, 0};
  double t[4];
  dqMul(qz, qy, t);
  dqMul(t, qx, g_bm.q);
  bmSetTruth();
}
// The magnetometer AHRS references magnetic north until it gets a position:
// rotate the truth by the simulated field's declination before comparing.
static const double MAG_DEC = atan2(1.7, 19.9);
static void truthMagnetic(float T[3][3]) {
  const double h[4] = {cos(-MAG_DEC / 2), 0, 0, sin(-MAG_DEC / 2)};
  double q[4];
  dqMul(h, g_bm.q, q);
  dqToMat(q, T);
}
// Error about the vertical (heading) against the magnetic truth [deg].
static float errMagneticVertical(const TriSenseINS &ins) {
  float R[3][3], T[3][3];
  if (!ins.getRotationMatrix(R)) return 999.0f;
  truthMagnetic(T);
  float E[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) E[i][j] = R[i][0] * T[j][0] + R[i][1] * T[j][1] + R[i][2] * T[j][2];
  return 0.5f * (E[1][0] - E[0][1]) / D2R;
}
static float errMagnetic(const TriSenseINS &ins) {
  float R[3][3], T[3][3];
  if (!ins.getRotationMatrix(R)) return 999.0f;
  truthMagnetic(T);
  return rotErrDeg(R, T);
}
// Angle between the estimated and the true down direction in the body [deg].
static float tiltErr(const TriSenseINS &ins) {
  float R[3][3], T[3][3];
  if (!ins.getRotationMatrix(R)) return 999.0f;
  dqToMat(g_bm.q, T);
  double c = 0;
  for (int i = 0; i < 3; i++) c += (double)R[2][i] * T[2][i];
  return (float)(acos(c > 1 ? 1 : c) * 180.0 / M_PI);
}
static double g_pitchRateDps = 0, g_pitchT0 = 1e30, g_pitchT1 = 1e30;
static void ratePitch(double t, double w[3]) {
  w[0] = w[2] = 0;
  w[1] = (t > g_pitchT0 && t < g_pitchT1) ? g_pitchRateDps * M_PI / 180.0 : 0.0;
}

static float angDiff(float a, float b) {
  float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return fabsf(d);
}

int main() {
  TriSense sensor;

  // -------------------------------------------------------------------------
  printf("1) Level and still, no GNSS: attitude, heading, baro altimeter\n");
  {
    TriSenseINS ins(sensor);
    setTruth(0, 0, 30, 0, ZERO3, ZERO3);
    CHECK(ins.begin(), "begin()");
    run(ins, 20);
    float r, p, y, h, vz;
    CHECK(ins.getRPY(r, p, y), "attitude available");
    CHECK(fabsf(r) < 1.0f && fabsf(p) < 1.0f, "roll %.2f pitch %.2f within 1 deg of level", r, p);
    // No position known -> magnetic heading. True yaw 30, field declination
    // atan2(1.7, 19.9) = 4.9 deg -> magnetic heading 25.1.
    CHECK(angDiff(y, 30.0f - 4.88f) < 3.0f, "magnetic heading %.1f (expect ~25.1)", y);
    CHECK(ins.getBaroAltitude(h, vz), "baro altitude available");
    CHECK(fabsf(h) < 0.5f && fabsf(vz) < 0.2f, "baro height %.2f m, vz %.2f m/s at rest", h, vz);
    CHECK(ins.getMode() == TriSenseINSMode::AttitudeOnly, "mode %s (expect ATT)", TriSenseINS::modeName(ins.getMode()));
    float an, ae, ad;
    CHECK(ins.getAccelerationNED(an, ae, ad) && fabsf(an) < 0.2f && fabsf(ae) < 0.2f && fabsf(ad) < 0.2f,
          "acceleration NED %.2f %.2f %.2f ~ 0 at rest", an, ae, ad);
    printf("  info: epoch rate %u Hz, stack %d, dropped %u\n", (unsigned)ins.getEpochRateHz(),
           ins.getFreeStack(), (unsigned)ins.getDroppedEpochs());
    CHECK(ins.getEpochRateHz() >= 195 && ins.getEpochRateHz() <= 205, "epoch rate %u Hz", (unsigned)ins.getEpochRateHz());

    // With the World Magnetic Model the heading becomes TRUE heading.
    ins.setMagneticModelPosition(50.08, 14.42);
    run(ins, 30);
    y = ins.getHeading();
    CHECK(angDiff(y, 30.0f) < 4.0f, "true heading %.1f after setMagneticModelPosition (expect ~30)", y);
  }

  // -------------------------------------------------------------------------
  printf("2) Tilted (roll 25, pitch -15), hard iron calibrated\n");
  {
    TriSenseINS ins(sensor);
    const float hard[3] = {-46.0f, -0.85f, -46.0f};
    const float soft[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    ins.setMagCalibration(hard, soft);
    setTruth(25, -15, 200, 0, ZERO3, hard);
    CHECK(ins.begin(), "begin()");
    run(ins, 20);
    float r, p, y;
    ins.getRPY(r, p, y);
    CHECK(fabsf(r - 25.0f) < 1.5f && fabsf(p + 15.0f) < 1.5f, "roll %.2f (25) pitch %.2f (-15)", r, p);
    CHECK(angDiff(y, 200.0f - 4.88f) < 4.0f, "magnetic heading %.1f (expect ~195.1)", y);
    float w, x, qy, z;
    CHECK(ins.getQuaternion(w, x, qy, z) && fabsf(w * w + x * x + qy * qy + z * z - 1.0f) < 1e-3f, "unit quaternion");
  }

  // -------------------------------------------------------------------------
  printf("3) Board upside down, setMounting(ComponentsDown)\n");
  {
    TriSenseINS ins(sensor);
    ins.setMounting(TriSenseINSMount::ComponentsDown);
    setTruth(10, 5, 90, 0, ZERO3, ZERO3);
    // Re-express the truth for an upside-down board: sensor = diag(1,1,1)*FRD.
    g_truth.acc[1] = -g_truth.acc[1]; g_truth.acc[2] = -g_truth.acc[2];
    g_truth.mag[1] = -g_truth.mag[1]; g_truth.mag[2] = -g_truth.mag[2];
    CHECK(ins.begin(), "begin()");
    run(ins, 20);
    float r, p, y;
    ins.getRPY(r, p, y);
    CHECK(fabsf(r - 10.0f) < 1.5f && fabsf(p - 5.0f) < 1.5f, "roll %.2f (10) pitch %.2f (5)", r, p);
  }

  // -------------------------------------------------------------------------
  printf("4) Vertical flight, no GNSS: climb at 1 g for 3 s, then coast\n");
  {
    TriSenseINS ins(sensor);
    setTruth(0, 0, 0, 0, ZERO3, ZERO3);
    CHECK(ins.begin(), "begin()");
    run(ins, 10);
    ins.setAutoZupt(false);  // in flight
    float hTrue = 0, vTrue = 0;
    const float dt = 0.00025f;
    for (int i = 0; i < 8 * 4000; i++) {
      const float aUp = (i < 3 * 4000) ? G : 0.0f;  // boost, then constant velocity
      vTrue += aUp * dt;
      hTrue += vTrue * dt;
      const float a_n[3] = {0, 0, -aUp};
      setTruth(0, 0, 0, hTrue, a_n, ZERO3);
      g_sim_us += 250;
      ins.update();
    }
    float h, vz, hb, vb;
    CHECK(ins.getHeight(h), "height available");
    ins.getBaroAltitude(hb, vb);
    ins.getVerticalSpeed(vz);
    CHECK(fabsf(h - hTrue) < 3.0f, "height %.1f m vs truth %.1f m", h, hTrue);
    CHECK(fabsf(vz - vTrue) < 2.0f, "vertical speed %.2f m/s vs truth %.2f m/s", vz, vTrue);
    printf("  info: baro channel %.1f m / %.2f m/s\n", hb, vb);
  }

  // -------------------------------------------------------------------------
  printf("5) Static with GNSS fixes at 5 Hz: full solution\n");
  {
    TriSenseINS ins(sensor);
    setTruth(0, 0, 60, 0, ZERO3, ZERO3);
    CHECK(ins.begin(), "begin()");
    const double lat0 = 50.0812345, lon0 = 14.4201234;
    for (int s = 0; s < 40 * 5; s++) {
      for (int k = 0; k < 800; k++) { g_sim_us += 250; ins.update(); }
      ins.feedGnss(lat0 + sim_noise(1.0f) / 111e3, lon0 + sim_noise(1.0f) / 71e3, 250.0f + sim_noise(2.0f),
                   1.0f, 10, fabsf(sim_noise(0.05f)), 0.0f, 20);
    }
    CHECK(ins.getMode() == TriSenseINSMode::Full, "mode %s (expect FULL)", TriSenseINS::modeName(ins.getMode()));
    CHECK(ins.isReady(), "INS ready");
    double la, lo; float he;
    CHECK(ins.getLatLonH(la, lo, he), "lat/lon available");
    CHECK(fabs(la - lat0) * 111e3 < 3.0 && fabs(lo - lon0) * 71e3 < 3.0, "position %.7f %.7f within 3 m", la, lo);
    CHECK(fabsf(he - 294.0f) < 6.0f, "ellipsoid height %.1f (expect ~294 = 250 MSL + 44)", he);
    float msl;
    CHECK(ins.getHeightMSL(msl) && fabsf(msl - 250.0f) < 6.0f, "MSL height %.1f (expect ~250)", msl);
    float vn, ve, vd;
    CHECK(ins.getVelocityNED(vn, ve, vd) && fabsf(vn) < 0.3f && fabsf(ve) < 0.3f && fabsf(vd) < 0.3f,
          "velocity %.2f %.2f %.2f ~ 0", vn, ve, vd);
    float sn, se, sd;
    CHECK(ins.getPositionStdDev(sn, se, sd) && sn > 0 && sn < 5, "position 1-sigma %.2f %.2f %.2f m", sn, se, sd);
    float y = ins.getHeading();
    CHECK(angDiff(y, 60.0f) < 5.0f, "true heading %.1f (expect ~60, WMM from first fix)", y);
    const ins_diag_t &d = ins.diagnostics();
    printf("  info: GNSS seen %u used %u, max filter %u us (host)\n", (unsigned)d.n_gnss_seen,
           (unsigned)d.n_gnss_used, (unsigned)ins.getMaxUpdateUs());
    CHECK(d.n_gnss_used > 50, "GNSS fixes fused (%u)", (unsigned)d.n_gnss_used);
  }

  // -------------------------------------------------------------------------
  printf("6) Driving east-north-east with GNSS: velocity, course, acceleration\n");
  {
    TriSenseINS ins(sensor);
    const float yawDeg = 70.0f;  // vehicle points where it drives
    setTruth(0, 0, yawDeg, 0, ZERO3, ZERO3);
    CHECK(ins.begin(), "begin()");
    double lat = 50.0812345, lon = 14.4201234;
    float v = 0;
    const float acc = 1.0f;  // m/s^2 along the heading
    for (int s = 0; s < 60 * 5; s++) {
      const bool moving = s >= 30 * 5;
      const bool accel = moving && s < 45 * 5;
      const float a = accel ? acc : 0.0f;
      const float a_n[3] = {a * cosf(yawDeg * D2R), a * sinf(yawDeg * D2R), 0};
      setTruth(0, 0, yawDeg, 0, a_n, ZERO3);
      g_truth.vib = moving ? 25.0f : 1.0f;  // engine/road vibration once moving
      for (int k = 0; k < 800; k++) {
        g_sim_us += 250;
        v += a * 0.00025f;
        lat += (v * cosf(yawDeg * D2R) * 0.00025f) / 111194.9;
        lon += (v * sinf(yawDeg * D2R) * 0.00025f) / (111194.9 * cos(lat * D2R));
        ins.update();
      }
      ins.feedGnss(lat + sim_noise(1.0f) / 111e3, lon + sim_noise(1.0f) / 71e3, 250.0f + sim_noise(2.0f),
                   1.0f, 10, v + sim_noise(0.1f), yawDeg, 20);
      if (s == 40 * 5) {
        float an, ae, ad;
        ins.getAccelerationNED(an, ae, ad);
        CHECK(fabsf(sqrtf(an * an + ae * ae) - acc) < 0.25f, "horizontal acceleration %.2f m/s^2 while accelerating (1.0)",
              sqrtf(an * an + ae * ae));
      }
    }
    float vn, ve, vd;
    ins.getVelocityNED(vn, ve, vd);
    CHECK(fabsf(ins.getGroundSpeed() - v) < 0.5f, "ground speed %.2f vs truth %.2f m/s", ins.getGroundSpeed(), v);
    CHECK(angDiff(ins.getCourse(), yawDeg) < 3.0f, "course %.1f (expect 70)", ins.getCourse());
    double la, lo; float he;
    ins.getLatLonH(la, lo, he);
    CHECK(fabs(la - lat) * 111e3 < 5.0 && fabs(lo - lon) * 71e3 < 5.0, "position within 5 m of truth after 15 s at %.0f m/s", v);
    CHECK(ins.getMode() == TriSenseINSMode::Full, "mode %s", TriSenseINS::modeName(ins.getMode()));
  }

  g_truth.vib = 1.0f;

  // -------------------------------------------------------------------------
  printf("7) IMU timing at navRateHz 240, ODR 1 kHz, loop 4 kHz\n");
  {
    TriSenseINS ins(sensor);
    setTruth(0, 0, 0, 0, ZERO3, ZERO3);
    CHECK(ins.begin(timingConfig()), "begin()");
    runLoop(ins, 10, 250);
    CHECK(ins.getImuMaxPacketsPerUpdate() == 32, "auto packet cap %u (4 x ceil(1000/240) = 20 -> min 32)",
          (unsigned)ins.getImuMaxPacketsPerUpdate());
    CHECK(ins.getEpochRateHz() >= 238 && ins.getEpochRateHz() <= 242, "epoch rate %u Hz", (unsigned)ins.getEpochRateHz());
    CHECK(ins.getLastEpochSamples() == 4 || ins.getLastEpochSamples() == 5, "last epoch %u samples (4..5)",
          (unsigned)ins.getLastEpochSamples());
    CHECK(fabsf(ins.getLastEpochDt() - ins.getLastEpochSamples() * 0.001f) < 2e-5f, "dt %.5f s = samples x 1 ms",
          ins.getLastEpochDt());
    CHECK(ins.getDtMismatchCount() == 0, "no sample/MCU dt mismatch (%u)", (unsigned)ins.getDtMismatchCount());
    CHECK(ins.getImuLostSamples() == 0 && ins.getImuMaxBacklog() <= 2, "lost %u, max FIFO backlog %u",
          (unsigned)ins.getImuLostSamples(), (unsigned)ins.getImuMaxBacklog());
    CHECK(fabsf(ins.getImuSamplePeriodUs() - 1000.0f) < 0.5f, "sample period %.2f us", ins.getImuSamplePeriodUs());
  }

  // -------------------------------------------------------------------------
  printf("8) IMU sample clock 1.5 %% fast: 10 s at 90 deg/s, yaw from the gyro only\n");
  {
    for (int cal = 1; cal >= 0; cal--) {
      TriSenseINS ins(sensor);
      sensor.imu.odrHz = 1015.0;  // reports 1000 Hz, really runs at 1015 Hz
      g_yawRate = 90.0f;
      g_truth.truthFn = truthYawRate;
      TriSenseINSConfig c = timingConfig();
      c.calibrateImuPeriod = cal;
      g_motionT0 = 1e30;
      CHECK(ins.begin(c), "begin()");
      runLoop(ins, 8, 250);
      g_motionT0 = g_sim_us * 1e-6;
      g_motionT1 = g_motionT0 + 10.0;
      runLoop(ins, 11, 250);
      const float err = insErrDeg(ins, 0, 0, yawAt(g_sim_us * 1e-6));
      if (cal) {
        CHECK(fabsf(ins.getImuSamplePeriodUs() - 1e6f / 1015.0f) < 0.5f, "calibrated period %.2f us (true %.2f)",
              ins.getImuSamplePeriodUs(), 1e6f / 1015.0f);
        CHECK(err < 1.0f, "attitude error %.2f deg after 900 deg of turn (calibrated)", err);
      } else {
        CHECK(err > 5.0f, "without calibration the error is %.1f deg (expect ~13.5) - the test is sensitive", err);
      }
      g_truth.truthFn = nullptr;
      g_motionT0 = g_motionT1 = 1e30;
      sensor.imu.odrHz = 1000.0;
    }
  }

  // -------------------------------------------------------------------------
  printf("9) 200 ms loop stall while turning at 90 deg/s: FIFO overflow\n");
  {
    TriSenseINS ins(sensor);
    g_yawRate = 90.0f;
    g_truth.truthFn = truthYawRate;
    CHECK(ins.begin(timingConfig()), "begin()");
    runLoop(ins, 5, 250);
    g_motionT0 = g_sim_us * 1e-6;
    g_motionT1 = g_motionT0 + 1.5;  // the turn ends during the recovery below
    runLoop(ins, 1, 250);
    const uint32_t driverBefore = ins.getImuLostPackets();
    g_sim_us += 200000;  // loop() blocked (SD card, radio, ...)
    runLoop(ins, 1, 250);
    const uint32_t lost = ins.getImuLostSamples();
    printf("  info: driver count +%u (first overflow after boot only primes it), TriSenseINS %u, "
           "mismatch epochs %u, max backlog %u\n", (unsigned)(ins.getImuLostPackets() - driverBefore),
           (unsigned)lost, (unsigned)ins.getDtMismatchCount(), (unsigned)ins.getImuMaxBacklog());
    CHECK(lost >= 69 && lost <= 75, "lost samples %u (expect 200 - 128 = 72)", (unsigned)lost);
    CHECK(ins.getImuMaxBacklog() >= 126, "max FIFO backlog %u (full)", (unsigned)ins.getImuMaxBacklog());
    CHECK(ins.getImuBacklog() <= 2, "backlog drained again (%u)", (unsigned)ins.getImuBacklog());
    const float err = insErrDeg(ins, 0, 0, yawAt(g_sim_us * 1e-6));
    CHECK(err < 1.0f, "attitude error %.2f deg after the stall (lost time integrated at the batch mean)", err);
    g_truth.truthFn = nullptr;
    g_motionT0 = g_motionT1 = 1e30;
  }

  // -------------------------------------------------------------------------
  printf("10) Packet cap: slow loop (100 Hz) with the automatic and a too-small cap\n");
  {
    for (int pass = 0; pass < 2; pass++) {
      TriSenseINS ins(sensor);
      setTruth(0, 0, 0, 0, ZERO3, ZERO3);
      TriSenseINSConfig c = timingConfig();
      c.imuMaxPacketsPerUpdate = pass == 0 ? 0 : 4;
      CHECK(ins.begin(c), "begin() cap %u", (unsigned)ins.getImuMaxPacketsPerUpdate());
      runLoop(ins, 5, 10000);
      if (pass == 0) {
        CHECK(ins.getImuLostSamples() == 0 && ins.getImuBacklog() <= 12,
              "automatic cap keeps up: lost %u, backlog %u", (unsigned)ins.getImuLostSamples(),
              (unsigned)ins.getImuBacklog());
      } else {
        CHECK(ins.getImuLostSamples() > 1000 && ins.getImuMaxBacklog() >= 126,
              "cap 4 at 100 Hz cannot keep up: lost %u counted, backlog %u", (unsigned)ins.getImuLostSamples(),
              (unsigned)ins.getImuMaxBacklog());
      }
      float r, p, y;
      CHECK(ins.getRPY(r, p, y) && fabsf(r) < 1.0f && fabsf(p) < 1.0f, "still level: roll %.2f pitch %.2f", r, p);
    }
  }

  // -------------------------------------------------------------------------
  // What the integration itself contributes is measured with a pure strapdown:
  // the rate TriSenseINS hands INSLIB each epoch (lastMeasurements(), minus
  // the gyro bias the filter subtracts), integrated exactly the way INSLIB's
  // strapdown does it, with no leveling or other correction on top. The full
  // filter's error is reported alongside: the ARS's accelerometer leveling
  // lags a fast cone/flip (10 Hz low-pass) and adds an error of its own that
  // has nothing to do with how the samples are integrated.
  printf("11) Coning, 60 s: mean of the epoch vs per-sample integration\n");
  {
    const double cases[3][2] = {{10, 5}, {30, 3}, {50, 2}};
    for (int c = 0; c < 3; c++) {
      float sdVert[2], sdTot[2], fVert[2], fTot[2];
      for (int ps = 0; ps < 2; ps++) {
        TriSenseINS ins(sensor);
        TriSenseINSConfig cfg = timingConfig();
        cfg.perSampleIntegration = ps;
        g_coneHz = cases[c][0];
        g_coneDeg = cases[c][1];
        g_motionStart = 1e30;
        bmStart(rateCone);
        ins.begin(cfg);
        runLoop(ins, 5, 250);
        g_motionStart = g_sim_us * 1e-6;
        runToEpoch(ins, sensor);
        StrapdownCheck sd(g_bm.q);
        const uint64_t end = g_sim_us + 60000000ULL;
        while (g_sim_us < end) {
          g_sim_us += 250;
          if (ins.update()) sd.epoch(ins);
        }
        runToEpoch(ins, sensor, [&] { sd.epoch(ins); });
        sd.err(sdTot[ps], sdVert[ps]);
        bmErr(ins, fTot[ps], fVert[ps]);
        bmStop();
      }
      printf("  info: %2.0f Hz %.0f deg: strapdown drift about the vertical %8.3f deg (mean) vs %7.4f deg (per sample),"
             " total %.3f vs %.4f\n", cases[c][0], cases[c][1], sdVert[0], sdVert[1], sdTot[0], sdTot[1]);
      printf("  info:              full ARS filter: vertical %.2f vs %.2f deg, total %.2f vs %.2f deg\n", fVert[0],
             fVert[1], fTot[0], fTot[1]);
      CHECK(sdTot[1] < 0.5f, "%.0f Hz / %.0f deg: per-sample strapdown error %.4f deg < 0.5 deg after 60 s", cases[c][0],
            cases[c][1], sdTot[1]);
      CHECK(sdTot[1] * 10.0f < sdTot[0], "%.0f Hz / %.0f deg: per-sample at least 10x better than the mean (%.3f)",
            cases[c][0], cases[c][1], sdTot[0]);
    }
  }

  // -------------------------------------------------------------------------
  printf("12) Flip: 400 deg/s for 1 s, no magnetometer\n");
  {
    for (g_flipMode = 0; g_flipMode < 2; g_flipMode++) {
      float sdErr[2], sdV, fErr[2], fV, vErr[2];
      for (int ps = 0; ps < 2; ps++) {
        TriSenseINS ins(sensor);
        TriSenseINSConfig cfg = timingConfig();
        cfg.perSampleIntegration = ps;
        g_motionStart = 1e30;
        bmStart(rateFlip);
        ins.begin(cfg);
        runLoop(ins, 5, 250);
        g_motionStart = g_sim_us * 1e-6;
        runToEpoch(ins, sensor);
        StrapdownCheck sd(g_bm.q);
        const uint64_t end = g_sim_us + 1050000ULL;
        while (g_sim_us < end) {
          g_sim_us += 250;
          if (ins.update()) sd.epoch(ins);
        }
        runToEpoch(ins, sensor, [&] { sd.epoch(ins); });
        sd.err(sdErr[ps], sdV);
        vErr[ps] = (float)sqrt(sd.v[0] * sd.v[0] + sd.v[1] * sd.v[1] + sd.v[2] * sd.v[2]);
        bmErr(ins, fErr[ps], fV);
        bmStop();
      }
      const char *name = g_flipMode ? "tumbling axis" : "fixed axis (1,2,3)";
      printf("  info: %s: strapdown error %.4f deg (mean) vs %.4f deg (per sample); full ARS filter %.2f vs %.2f deg\n",
             name, sdErr[0], sdErr[1], fErr[0], fErr[1]);
      printf("  info: %s: strapdown velocity error (truth: at rest) %.3f m/s (mean) vs %.4f m/s (per sample)\n", name,
             vErr[0], vErr[1]);
      CHECK(sdErr[1] < 0.05f, "%s: per-sample strapdown error %.4f deg after 400 deg", name, sdErr[1]);
      CHECK(vErr[1] < 0.02f, "%s: per-sample velocity error %.4f m/s (sculling)", name, vErr[1]);
      CHECK(fErr[1] < 3.0f && fErr[0] < 3.0f, "%s: full filter within 3 deg (%.2f / %.2f)", name, fErr[0], fErr[1]);
    }
  }

  // -------------------------------------------------------------------------
  printf("13) Daidalos on the rail: board -Y to the nose, ComponentsUp -> roll -90, pitch 0\n");
  {
    TriSenseINS ins(sensor);
    bmStart(nullptr);
    bmPose(-90, 0, 123);
    CHECK(ins.begin(), "begin()");
    runLoop(ins, 20, 250);
    float r, p, y;
    ins.getRPY(r, p, y);
    printf("  info: rpy %.3f %.3f %.3f, error vs magnetic truth %.3f deg\n", r, p, y, errMagnetic(ins));
    CHECK(fabsf(r + 90.0f) < 1.0f && fabsf(p) < 1.0f, "roll %.2f (-90) pitch %.2f (0)", r, p);
    CHECK(angDiff(y, 123.0f - (float)(MAG_DEC * 180 / M_PI)) < 2.0f, "magnetic heading %.2f (expect %.2f)", y,
          123.0f - (float)(MAG_DEC * 180 / M_PI));
    CHECK(errMagnetic(ins) < 1.5f, "attitude error %.2f deg", errMagnetic(ins));
    bmStop();
  }

  // -------------------------------------------------------------------------
  printf("14) Pitching through 90 deg (x vertical) with the magnetometer, no GNSS\n");
  {
    TriSenseINS ins(sensor);
    bmStart(ratePitch);
    bmPose(0, 0, 30);
    g_pitchRateDps = 30.0;
    g_pitchT0 = 1e30;
    // In flight: no stillness detection, which would otherwise measure the
    // gyro bias below directly and hide whether the magnetometer is used.
    TriSenseINSConfig c;
    c.autoZupt = false;
    CHECK(ins.begin(c), "begin()");
    runLoop(ins, 15, 250);
    float maxErr = 0, maxStep = 0, prevErr = errMagnetic(ins);
    auto track = [&] {
      const float e = errMagnetic(ins);
      maxErr = fmaxf(maxErr, e);
      maxStep = fmaxf(maxStep, fabsf(e - prevErr));
      prevErr = e;
    };
    // Up to pitch 90 (3 s).
    g_pitchT0 = g_sim_us * 1e-6;
    g_pitchT1 = g_pitchT0 + 3.0;
    for (int k = 0; k < 4 * 3000; k++) {
      g_sim_us += 250;
      if (ins.update()) track();
    }
    CHECK(maxErr < 1.5f, "on the way up to pitch 90: error never above %.2f deg", maxErr);
    // Held there for 60 s while the gyro grows a 0.3 deg/s bias about body x -
    // which is now the vertical, i.e. exactly the heading axis. Without the
    // magnetometer that is 18 deg of heading drift.
    const ahrs_t &a = ins.suite().ahrs;
    const uint32_t fused0 = a.n_mag_fused, weak0 = a.n_mag_skipped_weak_h;
    g_truth.gyrBiasDps[0] = 0.3f;
    for (int k = 0; k < 4 * 60000; k++) {
      g_sim_us += 250;
      if (ins.update()) track();
    }
    const uint32_t fused = a.n_mag_fused - fused0;
    float gb[3];
    ahrs_get_bias_gyr(&a, gb);
    float T[3][3];
    dqToMat(g_bm.q, T);
    const float errHold = errMagnetic(ins);
    printf("  info: body x %.2f deg from vertical; %u magnetometer samples fused in 60 s (%u skipped, weak field);\n"
           "        heading error %.2f deg, tilt %.2f deg; AHRS gyro bias x %.3f deg/s (true 0.3)\n",
           acosf(fabsf(T[2][0])) * 180.0f / (float)M_PI, (unsigned)fused, (unsigned)(a.n_mag_skipped_weak_h - weak0),
           errMagneticVertical(ins), tiltErr(ins), gb[0] / D2R);
    CHECK(fused >= 0.95f * 5 * 60, "magnetometer fused at pitch 90: %u of ~300 samples (5 Hz)", (unsigned)fused);
    // How fast the bias is learnt is AHRS tuning (heading measurement 1-sigma
    // 10 deg at 5 Hz by default); what matters here is that it is learnt at
    // all - before, the heading was simply not observed at pitch 90 (error
    // 9.1 deg after 30 s, bias estimate 0.000 in the same scenario).
    CHECK(gb[0] / D2R > 0.1f, "the heading-axis gyro bias is being learnt (%.3f deg/s, true 0.3)", gb[0] / D2R);
    CHECK(errHold < 9.0f, "attitude error %.2f deg after 60 s at pitch 90 with a 0.3 deg/s bias (unaided: 18 deg)",
          errHold);
    // And on over the top to pitch 150 (upside down, heading flipped).
    g_truth.gyrBiasDps[0] = 0.0f;
    g_pitchT0 = g_sim_us * 1e-6;
    g_pitchT1 = g_pitchT0 + 2.0;
    for (int k = 0; k < 4 * 6000; k++) {
      g_sim_us += 250;
      if (ins.update()) track();
    }
    CHECK(maxStep < 0.2f, "attitude continuous through pitch 90: largest epoch-to-epoch error change %.3f deg", maxStep);
    CHECK(errMagnetic(ins) < 6.0f, "attitude error %.2f deg at pitch 150 (upside down)", errMagnetic(ins));
    g_pitchT0 = g_pitchT1 = 1e30;
    bmStop();
  }

  // -------------------------------------------------------------------------
  printf("15) Initial alignment in 50 random poses, incl. pitch +-90\n");
  {
    std::mt19937 prng(7);
    std::uniform_real_distribution<float> U(-180.0f, 180.0f), UP(-90.0f, 90.0f);
    float worstAlign = 0, worstSettled = 0, worstTilt = 0;
    int bad = 0;
    for (int i = 0; i < 50; i++) {
      float r = U(prng), p = UP(prng), y = U(prng);
      if (i < 8) {
        const float forced[8] = {90, -90, 89.9f, -89.9f, 90, -90, 89.5f, -89.5f};
        p = forced[i];
      }
      for (int withMag = 1; withMag >= 0; withMag--) {
        TriSenseINS ins(sensor);
        TriSenseINSConfig c;
        c.useMagnetometer = withMag;
        bmStart(nullptr);
        bmPose(r, p, y);
        ins.begin(c);
        // Run until the attitude filter has bootstrapped, then once more.
        const ahrs_t &a = withMag ? ins.suite().ahrs : ins.suite().ars;
        for (int k = 0; k < 4 * 10000 && !a.is_initialized; k++) { g_sim_us += 250; ins.update(); }
        float align;
        if (withMag) {
          align = errMagnetic(ins);
          runLoop(ins, 5, 250);
          const float settled = errMagnetic(ins);
          worstAlign = fmaxf(worstAlign, align);
          worstSettled = fmaxf(worstSettled, settled);
          if (settled >= 1.0f) {
            bad++;
            printf("  info: pose %d (r %.1f p %.1f y %.1f): error %.2f at alignment, %.2f after 5 s\n", i, r, p, y,
                   align, settled);
          }
        } else {
          align = tiltErr(ins);
          worstTilt = fmaxf(worstTilt, align);
          if (align >= 1.0f) {
            bad++;
            printf("  info: pose %d (r %.1f p %.1f y %.1f), no magnetometer: tilt error %.2f\n", i, r, p, y, align);
          }
        }
        bmStop();
      }
    }
    // Right after alignment the heading carries the noise of the single
    // magnetometer sample it was taken from (0.3 uT on a 20 uT horizontal
    // field, ~1 deg 1-sigma); the filter averages that out within seconds.
    CHECK(worstAlign < 4.0f, "with magnetometer: worst error right after alignment %.3f deg", worstAlign);
    CHECK(worstSettled < 1.0f, "with magnetometer: worst error 5 s later %.3f deg", worstSettled);
    CHECK(worstTilt < 1.0f, "without magnetometer: worst tilt error after alignment %.3f deg", worstTilt);
    CHECK(bad == 0, "all 100 alignments converge within 1 deg (%d outside)", bad);
  }

  // -------------------------------------------------------------------------
  printf("16) Heading axis: INS with GNSS standing on its x axis (pitch 90), heading fed for FRD z\n");
  {
    // Body x straight up, yaw 40: FRD z then points horizontally to azimuth 40.
    // pass 3: the default axis at an ordinary pitch, where the fed heading is
    // the plain yaw as before.
    for (int pass = 0; pass < 4; pass++) {
      TriSenseINS ins(sensor);
      TriSenseINSConfig c;
      c.useMagnetometer = false;  // the fed heading is the only heading source
      if (pass == 1 || pass == 2) { c.headingAxis[0] = 0; c.headingAxis[2] = 1; }
      const float pitch = pass == 2 ? 60.0f : pass == 3 ? 10.0f : 90.0f;
      bmStart(nullptr);
      bmPose(0, pitch, 40);
      ins.begin(c);
      // Azimuth of FRD z for this pose (= 40 at pitch 90; 220 at pitch 60,
      // where z leans back over the nose).
      float T[3][3];
      dqToMat(g_bm.q, T);
      const int ax = (pass == 1 || pass == 2) ? 2 : 0;
      const float azZ = atan2f(T[1][ax], T[0][ax]) / D2R;
      const double lat0 = 50.0812345, lon0 = 14.4201234;
      for (int s5 = 0; s5 < 60 * 5; s5++) {
        for (int k = 0; k < 800; k++) { g_sim_us += 250; ins.update(); }
        ins.feedGnss(lat0 + sim_noise(1.0f) / 111e3, lon0 + sim_noise(1.0f) / 71e3, 250.0f + sim_noise(2.0f), 1.0f,
                     10, 0.0f, NAN, 20);
        ins.feedHeading(azZ + sim_noise(1.0f), 1.0f);
      }
      float sr, sp, sy;
      ins.getRPYStdDev(sr, sp, sy);
      float R[3][3];
      float err = 999.0f;
      if (ins.isReady() && ins.getRotationMatrix(R)) err = rotErrDeg(R, T);
      printf("  info: axis %s, pitch %.0f: INS ready %d, attitude error %.2f deg, yaw 1-sigma %.1f deg\n",
             ax ? "FRD z" : "FRD x (default)", pitch, ins.isReady(), err, sy);
      if (pass == 3) {
        CHECK(err < 2.0f, "default axis at pitch 10: heading = yaw fused as before, error %.2f deg", err);
      } else if (pass == 0) {
        CHECK(!(err < 5.0f && sy < 5.0f), "default axis FRD x is vertical: its heading is ignored (yaw 1-sigma %.1f)",
              sy);
      } else {
        CHECK(err < 2.0f, "heading of FRD z fused at pitch %.0f: attitude error %.2f deg", pitch, err);
      }
      bmStop();
    }
  }

  printf(failures ?"\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
  return failures ? 1 : 0;
}
