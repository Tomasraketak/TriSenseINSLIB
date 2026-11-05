# TriSenseINSLIB

Inertial navigation for the **Voltino TriSense** module: attitude, position,
velocity, acceleration and barometric altitude from
[**INSLIB**](https://github.com/jnz/INSLIB), a 15-state error-state Kalman
filter by Jan Zwiener, with a few lines of Arduino code.

> [!WARNING]
> **Licence: AGPL-3.0.** This library contains INSLIB, which is AGPL-3.0, so
> the whole library is AGPL-3.0, and so is **any firmware you build with it**.
> If you give that firmware to anyone (or let people use it over a network)
> you must offer them its complete source code under the AGPL. For
> closed-source commercial products you need a **commercial licence from
> INSLIB's author**. The `Voltino_TriSense` driver library itself stays MIT;
> nothing from INSLIB goes into it.

> [!CAUTION]
> **Not validated on hardware yet.** Everything compiles in CI for Pico 2,
> ESP32-S3 and Teensy 4.1, and a host test runs the real library and the real
> INSLIB against a simulated TriSense (see [Testing](#testing)). Nobody has
> flown or driven it yet. Treat the output as unverified until you have.

---

## Quick start

```cpp
#include <TriSense.h>
#include <TriSenseINS.h>

TriSense    sensor;
TriSenseINS ins(sensor);

void setup() {
  sensor.beginAll(MODE_HYBRID, 17, 10000000);   // TriSense as usual
  ins.begin();                                  // keep the board still ~2 s
}

void loop() {
  ins.update();                                 // call as often as you can

  float roll, pitch, heading;  ins.getRPY(roll, pitch, heading);   // deg
  float h, vz;                 ins.getBaroAltitude(h, vz);         // m, m/s, up = +

  // With a GNSS receiver (optional):
  // ins.feedGnss(lat, lon, altMSL, hdop, sats, speed, course, fixAgeMs);
  float n, e, d;               ins.getPositionNED(n, e, d);        // m from start
  double lat, lon; float hEll; ins.getLatLonH(lat, lon, hEll);
  float vn, ve, vd;            ins.getVelocityNED(vn, ve, vd);     // m/s
  float an, ae, ad;            ins.getAccelerationNED(an, ae, ad); // m/s^2, no gravity
}
```

**Without GNSS** you get roll, pitch, heading, height above the start point and
vertical speed: a good AHRS and barometric altimeter, which is what a rocket or
CanSat needs. **With GNSS** you also get position, velocity, acceleration, the
estimated sensor biases and the uncertainty of all of it.

## Installation

1. Install **Voltino_TriSense** and **TinyGPSPlus** from the Library Manager
   (installing this library from the Library Manager pulls both in).
2. Install this library (Library Manager once registered, or *Sketch → Include
   Library → Add .ZIP Library* with a ZIP of this repository).
3. Board: **Raspberry Pi Pico 2** with the
   [arduino-pico](https://github.com/earlephilhower/arduino-pico) core, ARM
   architecture. ESP32-S3 and Teensy 4.x also compile (see
   [Platforms](#platforms-and-the-stack)).

## Examples

| Example | What it shows |
|---|---|
| `BasicAttitudeAltitude` | No GNSS: roll / pitch / heading, baro height and vertical speed, 10 Hz |
| `GPS_INS` | TriSense + Quectel L76K over NMEA (L76K-specific PCAS setup): full navigation, accuracy, biases, CSV mode |
| `GPS_INS_UBLOX` | TriSense + u-blox M10 (or M8/M9) over binary UBX: NAV-PVT at 10 Hz, the receiver's own hAcc/vAcc/sAcc, full 3D NED velocity, airborne dynamic model, UART or I2C. Needs the **SparkFun u-blox GNSS v3** library |
| `FlightLogger` | Logs what the filter sees; `tools/split_flight_log.py` turns it into an INSLIB dataset for `inspostgui.py` |

### Wiring (Pico 2, as in the examples)

```
TriSense   ICM-42688-P   SPI0, CS = GP17 (SCK GP18, MOSI GP19, MISO GP16)
           magnetometer  I2C0 (SDA GP4, SCL GP5)
           BMP580        I2C0
L76K       module RX  <- GP0  (Pico TX)
           module TX  -> GP1  (Pico RX)
u-blox M10 same UART pins (GPS_INS_UBLOX), or Qwiic / I2C0 at 0x42
           with GNSS_USE_I2C 1
```

Use `GPS_INS_UBLOX` for a u-blox module. The NMEA `GPS_INS` example sends
L76K-only `$PCAS` commands; on a u-blox it only works with `GPS_FAST_BAUD` set
to 9600, at 1 Hz and with HDOP-based accuracy.

---

## API

Angles in **degrees**, distances in **metres**, unless the name ends in `Rad`.
Body frame **FRD** (x forward, y right, z down), navigation frame **NED**.
Heights and vertical speeds are **positive up**. Every getter returns `false`
(or `NAN`) while that quantity is not available yet.

### Life cycle and inputs

| Call | |
|---|---|
| `begin()` / `begin(TriSenseINSConfig)` | After `sensor.beginAll()`. Sets IMU 1 kHz + FIFO, baro 50 Hz / OSR x8 / no IIR, calibrates the gyro, starts the filter |
| `update()` | Call from `loop()` as often as possible. Drains the IMU FIFO, polls mag + baro, runs a filter epoch at 200 Hz. `true` when an epoch ran |
| `reset()` | Restart the filter, keep calibration and settings |
| `feedGnss(lat, lon, altMSL, hdop, sats, speed, course, ageMs, geoidSep)` | A GNSS fix (e.g. TinyGPSPlus). HDOP → covariance, latency compensated |
| `feedGnss(TriSenseGnssFix)` | Same, with optional receiver accuracies and full NED velocity (u-blox UBX) |
| `feedSpeed(mps, sigma, delayMs)` | Odometry speed |
| `feedHeading(deg, sigmaDeg, delayMs)` | External heading (dual-antenna GNSS, launch rail, …) |
| `zeroVelocityUpdate()` | "Not moving" for the next epoch |
| `setAutoZupt(bool)` | Automatic stillness detection on/off (turn off in flight) |
| `setMagCalibration(hard[3], soft[3][3])` | From the MotionCal example, in TriSense axes |
| `setMagneticModelPosition(lat, lon[, year])` | World Magnetic Model → heading to **true** north. Automatic on the first GNSS fix |
| `setDeclination(deg)` | Manual declination, used only while no position is known |
| `setInitialHeading(deg, sigmaDeg)` | Known start heading (before `begin()`) |
| `setMounting(ComponentsUp / ComponentsDown)`, `setMountingRPY(r,p,y)`, `setMountingRotation(R)` | How the board sits in the vehicle (before `begin()`) |

### Outputs

| Group | Calls |
|---|---|
| Attitude | `getRPY`, `getRPYRad`, `getHeading`, `getRPYStdDev`, `getQuaternion`, `getRotationMatrix`, `getAngularRate`, `getAttitude(source, …)` with source `Best` / `INS` / `AHRS` (magnetometer) / `ARS` (gyro-only yaw) |
| Position | `getPositionNED`, `getPositionStdDev`, `getLatLonH`, `getPositionECEF`, `localToLatLon`, `latLonToLocal` |
| Velocity | `getVelocityNED`, `getVelocityStdDev`, `getVelocityECEF`, `getGroundSpeed`, `getCourse` |
| Acceleration | `getAccelerationNED`, `getAccelerationBody` (gravity removed) |
| Height | `getBaroAltitude(h, vz)`, `getHeight`, `getVerticalSpeed`, `getHeightEllipsoid`, `getHeightMSL`, `getPressureAltitude` |
| Sensor errors | `getAccelBias`, `getGyroBias`, `getAccelBiasStdDev`, `getGyroBiasStdDev` |
| Status | `getMode()` → `None` / `AttitudeOnly` / `Coasting` / `Full`, `modeName()`, `isReady`, `hasAttitude`, `hasPosition`, `isZuptActive`, `isVerticalZuptActive`, `getDeadReckoningMs` |
| Health | `getEpochRateHz`, `getLastUpdateUs`, `getMaxUpdateUs`, `getDroppedEpochs`, `getImuLostPackets`, `getFreeStack`, `stackOk` |
| Raw | `diagnostics()` (INSLIB `ins_diag_t`), `suite()` (the INSLIB `nav_suite_t`), `lastMeasurements()`, `config()` |

Everything in `TriSenseINSConfig` has a working default; see the comments in
[`src/TriSenseINS.h`](src/TriSenseINS.h).

### Modes

| Mode | Meaning |
|---|---|
| `None` | First ~1.5 s, nothing usable yet |
| `AttitudeOnly` | Attitude + baro height. **This is the normal mode without GNSS.** |
| `Coasting` | INS running on the IMU alone (GNSS lost); position drifts, its sigma grows |
| `Full` | INS with recent GNSS aiding |

---

## Things to set before trusting the output

1. **Magnetometer calibration.** Run the `MotionCal` example of Voltino_TriSense
   and pass the result to `setMagCalibration()`. Without it the heading is off.
2. **A position for the magnetic model.** With GNSS this happens on the first
   fix. Without GNSS call `setMagneticModelPosition(lat, lon)` with your
   location, or the heading refers to *magnetic* north.
3. **Mounting.** At rest and level the filter must see the accelerometer as
   `(0, 0, -9.81)` in FRD. A board lying flat with components up and its X axis
   pointing forward is the default. Upside down: `setMounting(ComponentsDown)`.
   Anything else: `setMountingRPY()` or `setMountingRotation()`.
4. **GNSS gates.** INSLIB only enters a 3D solution once fixes have been better
   than `gnssStartMax*` for a while. The defaults fit a consumer receiver with
   `gnssUereM = 2.2` (HDOP 1 → 2.2 m). Change both together, or the filter will
   consume fixes forever and never start, silently.
5. **Stillness detection (ZUPT).** It locks velocity to zero when the IMU says
   "still". In flight, or on anything that moves perfectly smoothly, a constant
   velocity looks exactly like standing still to an IMU: call
   `setAutoZupt(false)` once you are moving (e.g. at launch detection).

**Rockets.** Mount the board with X along the rocket's axis (nose = forward).
On the pad the pitch is then +90°, where roll and yaw Euler angles are
undefined (gimbal lock); use `getQuaternion()` / `getRotationMatrix()` for
attitude there. Height and vertical speed (`getBaroAltitude`) are unaffected.
Expect the barometer to read wrong around transonic speeds and in the ejection
gas; the vertical channel will show it.

---

## Platforms and the stack

| Board | Status | Notes |
|---|---|---|
| **Raspberry Pi Pico 2** (RP2350, ARM) | primary target | 8 KB stack per core, filter needs ~6.8 KB |
| ESP32-S3 | compiles | add `SET_LOOP_TASK_STACK_SIZE(16 * 1024);` (the examples do) |
| Teensy 4.x | compiles | large stack, nothing to do |
| RP2040, RP2350 RISC-V | **refused** with `#error` | no FPU |
| AVR (Uno, Nano, …) | **refused** with `#error` | no FPU, not enough RAM |

### Why the stack matters on the Pico 2

One `nav_suite_update()` call needs **6776 bytes** of stack (measured with
INSLIB's call-graph analysis for cortex-m33, `-Os`, with this library's
configuration of 15 states). arduino-pico gives a core **8192 bytes** of stack,
and that limit is not about free RAM: the stacks live in two separate 4 KB
banks (SCRATCH_X / SCRATCH_Y) above the main RAM, so the 400+ KB of free RAM
does not help. Running past the end is not a crash, it silently overwrites
whatever is below, with the default split the other core's stack.

- **Single core (the examples):** nothing to do, the loop has all 8 KB.
- **Using `setup1()` / `loop1()`:** arduino-pico then splits the 8 KB into 4 KB
  per core, which is **not enough**. Add
  `bool core1_separate_stack = true;` to your sketch: core 1 gets its own 8 KB
  and core 0 keeps the full 8 KB. Run the filter on core 0.
- **More than 8 KB** (e.g. to enable INSLIB's 18-state magnetometer bias
  estimation): build with arduino-pico's FreeRTOS support and run the filter in
  a task with an explicit stack size, which can be as large as free RAM allows.

`getFreeStack()` reports the free stack right before each filter call (Pico)
or the stack high-water mark (ESP32), and `stackOk()` turns false if it ever
dropped below what the filter needs. The examples print a warning.

`src/trisense_inslib_config.h` sets `INS_UNKNOWNS_MAX = 15` and
`KALMAN_MAX_STATE_SIZE / KALMAN_MAX_NOISE_SIZE = 15 / 21`. KFCore's defaults
(32 / 32) need 12.9 KB of stack. Do not change them without reading that file.

### Size

Measured by the CI build (arduino-cli, arduino-pico, default board options,
2026-11-04):

| Board | Example | Flash | RAM (static) |
|---|---|---:|---:|
| Pico 2 | GPS_INS (incl. TinyGPSPlus and the core) | 156 364 B (3 %) | 35 276 B (6 %) |

The other examples are smaller; the CI log of each push lists them all.

The filter state (`nav_suite_t`) is about 15 KB of RAM, and INSLIB's World
Magnetic Model tables about 11 KB of flash.

---

## How INSLIB is built

INSLIB and KFCore are vendored, **unmodified**, in [`src/inslib/`](src/inslib)
(INSLIB **v1.4.2**, exact commits and file hashes in
[`src/inslib/VERSIONS.txt`](src/inslib/VERSIONS.txt)).

The Arduino build compiles every `.c` under `src/` and cannot pass `-D` flags
to a library, but INSLIB's array sizes (and therefore its stack use) are set
by `-D` flags. So the INSLIB `.c` files are stored as `.c.inc` (same bytes,
only the name changed, so Arduino does not compile them directly) and each is
compiled through a one-line wrapper `src/inslib_tu_<name>.c`:

```c
#include "trisense_inslib_config.h"
#include "inslib/ins.c.inc"
```

That does what `-D` would have done, without patching INSLIB.
`TriSenseINS.h` includes the same config header before the INSLIB headers, so
every translation unit agrees on the layout of `ins_t`.

To update INSLIB: `tools/update_inslib.sh v1.x.y`, then run the host test,
build the examples and update the version in this README and in
`TRISENSE_INSLIB_INSLIB_VERSION`.

## Post-processing a flight

1. Flash `FlightLogger`, capture its serial output to a file.
2. `python3 tools/split_flight_log.py flight.log flight/`
3. Open `flight/config.yaml` in INSLIB's `tools/inspostgui.py` (or run
   `tools/replay.py`): plots, KML track, RTS smoother, different tuning.

## Testing

`tests/host` builds the real library and the real INSLIB on a PC against a
simulated TriSense and checks the outputs against the simulated truth: level
and tilted attitude, magnetic vs. true heading, upside-down mounting, hard-iron
calibration, a 1 g vertical boost tracked by the barometric channel, a static
GNSS solution and a vehicle accelerating with GNSS.

```sh
make -C tests/host
```

It runs in CI together with the compile jobs for Pico 2, ESP32-S3 and
Teensy 4.1 and `arduino-lint` in Library Manager submission mode. It checks the
glue (units, axes, frames, wiring of every input and output), **not** the
sensor hardware or real-world tuning.

## Known limitations

- Not validated on hardware (see the top).
- The magnetometer is read through `sensor.mag.readData()` / `.x .y .z`, which
  is today the AK09918C. When TriSense V1.1/V2 boards with the MMC5603NJ get a
  driver in Voltino_TriSense with the same interface, only
  `TriSenseINS::readMagnetometer()` may need a change.
- Sensor noise (`accelNoisePSD`, `gyroNoisePSD`) is left at INSLIB's
  conservative consumer-MEMS defaults. Measure your own with an Allan variance
  run (`tools/allan_variance.py` in INSLIB).
- `feedGnss()` and `update()` must run on the same core (or be protected by
  a mutex): the fix is handed over through a plain struct.
- Heading from NMEA course over ground is off by default (`gnssUseCourseAsHeading`),
  as it is only valid for vehicles that cannot move sideways.
- With INSLIB's logging turned off (`LOG_LEVEL 0`), a filter that refuses to
  start says nothing. `diagnostics()` has the counters to find out why.

## Credits and licences

- **TriSenseINSLIB**: © 2026 VoltinoLabs (Tomáš Michal), AGPL-3.0-or-later
  ([LICENSE](LICENSE)).
- **INSLIB**: © Jan Zwiener, AGPL-3.0 ([src/inslib/LICENSE.INSLIB](src/inslib/LICENSE.INSLIB)),
  <https://github.com/jnz/INSLIB>. Commercial licences: contact the author.
- **KFCore**: © 2024 Jan Zwiener, BSD-3-Clause ([src/inslib/LICENSE.KFCore](src/inslib/LICENSE.KFCore)).
- The TriSense glue follows the `GPS_INS_INSLIB` example of
  [Voltino_TriSense](https://github.com/Tomasraketak/Voltino_TriSense_Library)
  (PR #7), whose README explains the design decisions in more depth.
