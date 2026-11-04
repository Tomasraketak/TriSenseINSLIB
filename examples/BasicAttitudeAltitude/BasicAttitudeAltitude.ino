/*
 * BasicAttitudeAltitude - TriSense + INSLIB without GNSS.
 *
 * Attitude (roll, pitch, heading), height above the start point and vertical
 * speed: the rocket / CanSat use case. No GNSS receiver needed.
 *
 * Hardware: Raspberry Pi Pico 2 (RP2350) + Voltino TriSense
 *   ICM-42688-P on SPI (CS = GP17), magnetometer + BMP580 on I2C (GP4/GP5).
 *   Also builds for ESP32-S3 and Teensy 4.x - adjust IMU_CS_PIN.
 *
 * Keep the board STILL for the first ~3 seconds (gyro calibration and
 * levelling). Serial: 115200 baud, 10 lines per second.
 *
 * Licence: this sketch and the library are AGPL-3.0 (INSLIB is AGPL-3.0).
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */
#include <TriSense.h>
#include <TriSenseINS.h>

#if defined(ARDUINO_ARCH_ESP32)
// The filter needs ~7 KB of stack; the ESP32 loop task has 8 KB by default.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);
#endif

#define IMU_CS_PIN  17
#define IMU_SPI_HZ  10000000UL

// Magnetometer calibration from the MotionCal example of Voltino_TriSense.
// Leave at zero / identity until you have run it - heading will be off.
static const float MAG_HARD_IRON[3] = { 0.0f, 0.0f, 0.0f };
static const float MAG_SOFT_IRON[3][3] = {
  { 1.0f, 0.0f, 0.0f },
  { 0.0f, 1.0f, 0.0f },
  { 0.0f, 0.0f, 1.0f }
};

// Where you are (e.g. the launch site), for the World Magnetic Model: turns
// the magnetic heading into a TRUE heading. Set USE_SITE to 0 to skip.
#define USE_SITE   1
#define SITE_LAT   50.08
#define SITE_LON   14.42

TriSense    sensor;
TriSenseINS ins(sensor);

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) { delay(10); }

  if (!sensor.beginAll(MODE_HYBRID, IMU_CS_PIN, IMU_SPI_HZ)) {
    Serial.println(F("TriSense not found - check wiring and IMU_CS_PIN."));
    while (true) { delay(1000); }
  }

  ins.setMagCalibration(MAG_HARD_IRON, MAG_SOFT_IRON);
#if USE_SITE
  ins.setMagneticModelPosition(SITE_LAT, SITE_LON);
#endif

  Serial.println(F("Keep still: calibrating gyro..."));
  if (!ins.begin()) {
    Serial.println(F("INSLIB refused the configuration."));
    while (true) { delay(1000); }
  }
  Serial.println(F("roll  pitch  heading [deg] | height [m]  vz [m/s] | mode"));
}

void loop() {
  ins.update();   // call as often as possible

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  float roll, pitch, heading;
  if (ins.getRPY(roll, pitch, heading)) {
    Serial.print(roll, 1);    Serial.print(F("  "));
    Serial.print(pitch, 1);   Serial.print(F("  "));
    Serial.print(heading, 1);
  } else {
    Serial.print(F("-- attitude not ready --"));
  }

  float h, vz;
  if (ins.getBaroAltitude(h, vz)) {
    Serial.print(F(" | "));
    Serial.print(h, 2);  Serial.print(F(" m  "));
    Serial.print(vz, 2); Serial.print(F(" m/s"));
  }

  Serial.print(F(" | "));
  Serial.print(TriSenseINS::modeName(ins.getMode()));
  if (ins.isZuptActive()) Serial.print(F(" still"));
  if (!ins.stackOk()) Serial.print(F(" !!STACK"));
  Serial.println();
}
