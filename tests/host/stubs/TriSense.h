// Simulated TriSense for the host test: implements exactly the members
// TriSenseINS uses, fed from a scripted "truth" (see test_main.cpp).
#pragma once
#include <Arduino.h>

enum ICM_ODR { ODR_8KHZ = 3, ODR_1KHZ = 6, ODR_200HZ = 7 };
enum ICM_FIFO_MODE { FIFO_NONE = 0, FIFO_16BIT = 1, FIFO_20BIT_HIRES = 2 };
enum BMP580_OSR { BMP580_OSR_x2 = 1, BMP580_OSR_x8 = 3 };
enum BMP580_ODR { BMP580_ODR_50p1Hz = 0x0F };
enum BMP580_IIR { BMP580_IIR_OFF = 0 };
enum BMP580_Mode { BMP580_MODE_STANDBY = 0, BMP580_MODE_NORMAL = 1 };

// Truth, written by the test: sensor-frame accel [g], gyro [dps], mag [uT],
// pressure [Pa]. If truthFn is set, the simulated sensors call it with the
// sample time [s] before producing each sample, so a test can script motion
// that changes faster than the loop runs (coning, flips).
struct SimTruth {
  float acc[3], gyr[3], mag[3], pressure;
  float vib = 1.0f;
  void (*truthFn)(double tSec) = nullptr;
  float gyrNoiseDps = 0.05f;   // per-sample 1-sigma
  float accNoiseG = 0.002f;
  float gyrBiasDps[3] = {0, 0, 0};  // sensor axes
};
extern SimTruth g_truth;
float sim_noise(float sigma);

// ICM-42688-P with its 2 KB FIFO in 16-bit mode: 128 packets, Stream mode
// (a full FIFO discards NEW samples and counts them in FIFO_LOST_PKT). The
// lost-packet counter is reported the way the real Voltino driver does it:
// accumulated only on refills that find the FIFO full, and the first such
// refill after boot only primes the baseline (its losses are not counted).
struct SimIMU {
  static const int CAP = 128;
  ICM_FIFO_MODE mode = FIFO_16BIT;
  double odrHz = 1000.0;     // true sample rate (tests can detune it)
  int    reportedOdrHz = 1000;
  uint64_t nextNs = 0;       // time of the next sample to be produced
  struct Pkt { float a[3], g[3]; double t; uint16_t ts; } q[CAP];
  // FIFO timestamps (sensor clock: 1 us per LSB of the sensor's own time, so
  // exact multiples of the nominal period), as Voltino_TriSense >= 1.6 reports
  // them. tsEnabled = false behaves like a driver/sensor without them.
  bool tsEnabled = true;
  uint32_t tsTicks = 0;
  uint16_t lastTs = 0;
  bool lastTsValid = false;
  // Every dropEvery-th packet is corrupt and skipped by the driver (0 = never).
  uint32_t dropEvery = 0, dropped = 0, pktIndex = 0;
  // Filter calls (Voltino_TriSense >= 1.6).
  uint16_t aafGyro = 585, aafAccel = 1163;
  float uiGyro = 250, uiAccel = 250;
  int filterCalls = 0;
  double lastReadT = 0;  // sample time of the last packet handed out [s]
  int head = 0, count = 0;
  uint32_t chipLost = 0;      // the chip's own counter (since flush)
  uint32_t driverLost = 0;    // what getLostPacketCount() reports
  uint32_t chipLostSeen = 0;
  bool primed = false;
  uint64_t produced = 0;

  void setODR(ICM_ODR) {}
  int getODRHz() { return reportedOdrHz; }
  void setFIFOMode(ICM_FIFO_MODE m) { mode = m; }
  ICM_FIFO_MODE getFIFOMode() { return mode; }
  void flushFIFO() { produce(); count = 0; head = 0; chipLost = 0; chipLostSeen = 0; lastTsValid = false; }
  bool hasFIFOTimestamp() { return tsEnabled && lastTsValid; }
  uint16_t getFIFOTimestamp() { return lastTs; }
  bool setAntiAliasFilter(uint16_t g, uint16_t a) { aafGyro = g; aafAccel = a; filterCalls++; return true; }
  bool setUIFilter(float g, float a) { uiGyro = g; uiAccel = a; filterCalls++; return true; }
  // The driver's fallback for an ODR it cannot service: half the rate, FIFO flushed.
  bool stepDownODR() { produce(); odrHz /= 2; reportedOdrHz /= 2; flushFIFO(); return true; }
  uint32_t getLostPacketCount() { return driverLost; }

  void sample(Pkt &p, double t) {
    p.t = t;
    p.ts = (uint16_t)tsTicks;
    if (g_truth.truthFn) g_truth.truthFn(t);
    for (int i = 0; i < 3; i++) {
      p.a[i] = g_truth.acc[i] + sim_noise(g_truth.accNoiseG * g_truth.vib);
      p.g[i] = g_truth.gyr[i] + g_truth.gyrBiasDps[i] + sim_noise(g_truth.gyrNoiseDps * g_truth.vib);
    }
  }
  void produce() {
    const uint64_t nowNs = g_sim_us * 1000ULL;
    if (nextNs == 0) nextNs = nowNs;
    const double stepNs = 1e9 / odrHz;
    while (nextNs <= nowNs) {
      if (count < CAP) {
        sample(q[(head + count) % CAP], nextNs * 1e-9);
        count++;
      } else {
        chipLost++;
      }
      produced++;
      tsTicks += (uint32_t)(1000000 / reportedOdrHz);
      nextNs = (uint64_t)((double)nextNs + stepNs + 0.5);
    }
  }
  // Driver refill: reads the lost counter only when the FIFO is (nearly) full.
  void refillCheck() {
    if (count + 2 >= CAP) {
      if (!primed) primed = true;
      else driverLost += chipLost - chipLostSeen;
      chipLostSeen = chipLost;
    }
  }
  uint16_t availablePackets() {
    if (mode == FIFO_NONE) return 1;
    produce();
    return (uint16_t)count;
  }
  bool readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    if (mode == FIFO_NONE) {
      Pkt p; sample(p, g_sim_us * 1e-6);
      ax = p.a[0]; ay = p.a[1]; az = p.a[2]; gx = p.g[0]; gy = p.g[1]; gz = p.g[2];
      return true;
    }
    produce();
    refillCheck();
    if (dropEvery && count > 0 && (++pktIndex % dropEvery) == 0) {  // corrupt packet, skipped
      head = (head + 1) % CAP;
      count--;
      dropped++;
    }
    if (count == 0) return false;
    const Pkt &p = q[head];
    lastReadT = p.t;
    lastTs = p.ts;
    lastTsValid = true;
    head = (head + 1) % CAP;
    count--;
    ax = p.a[0]; ay = p.a[1]; az = p.a[2]; gx = p.g[0]; gy = p.g[1]; gz = p.g[2];
    return true;
  }
};
struct SimMag {
  float x = 0, y = 0, z = 0; bool overflow = false; uint64_t nextUs = 0;
  bool readData() {
    if (nextUs > g_sim_us) return false;
    nextUs = g_sim_us + 10000;  // 100 Hz
    if (g_truth.truthFn) g_truth.truthFn(g_sim_us * 1e-6);
    x = g_truth.mag[0] + sim_noise(0.3f); y = g_truth.mag[1] + sim_noise(0.3f); z = g_truth.mag[2] + sim_noise(0.3f);
    return true;
  }
};
struct SimBMP {
  uint64_t nextUs = 0; float last = 0;
  void setI2CSpeed(uint32_t) {}
  void setPowerMode(BMP580_Mode) {}
  void setOversampling(BMP580_OSR, BMP580_OSR) {}
  void setODR(BMP580_ODR) {}
  void setIIRFilter(BMP580_IIR, BMP580_IIR) {}
  float readPressure() {
    if (nextUs <= g_sim_us) { nextUs = g_sim_us + 20000; last = g_truth.pressure + sim_noise(0.4f); }
    return last;
  }
};
class TriSense {
public:
  SimBMP bmp; SimMag mag; SimIMU imu;
  void autoCalibrateGyro(uint16_t) {}
  float readPressure() { return bmp.readPressure(); }
};
