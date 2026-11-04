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
// pressure [Pa].
struct SimTruth { float acc[3], gyr[3], mag[3], pressure; float vib = 1.0f; };
extern SimTruth g_truth;
float sim_noise(float sigma);

struct SimIMU {
  ICM_FIFO_MODE mode = FIFO_16BIT;
  uint64_t nextUs = 0;
  void setODR(ICM_ODR) {}
  void setFIFOMode(ICM_FIFO_MODE m) { mode = m; }
  ICM_FIFO_MODE getFIFOMode() { return mode; }
  void flushFIFO() { nextUs = g_sim_us; }
  uint32_t getLostPacketCount() { return 0; }
  // One sample per simulated millisecond (1 kHz ODR).
  bool readFIFO(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    if (nextUs > g_sim_us) return false;
    nextUs += 1000;
    ax = g_truth.acc[0] + sim_noise(0.002f * g_truth.vib); ay = g_truth.acc[1] + sim_noise(0.002f * g_truth.vib); az = g_truth.acc[2] + sim_noise(0.002f * g_truth.vib);
    gx = g_truth.gyr[0] + sim_noise(0.05f * g_truth.vib);  gy = g_truth.gyr[1] + sim_noise(0.05f * g_truth.vib);  gz = g_truth.gyr[2] + sim_noise(0.05f * g_truth.vib);
    return true;
  }
};
struct SimMag {
  float x = 0, y = 0, z = 0; bool overflow = false; uint64_t nextUs = 0;
  bool readData() {
    if (nextUs > g_sim_us) return false;
    nextUs = g_sim_us + 10000;  // 100 Hz
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
