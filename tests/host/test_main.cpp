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
  printf(failures ?"\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
  return failures ? 1 : 0;
}
