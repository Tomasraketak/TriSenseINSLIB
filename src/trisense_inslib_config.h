/*
 * trisense_inslib_config.h - build configuration for the vendored INSLIB.
 *
 * Part of TriSenseINSLIB. SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * The Arduino build cannot pass -D flags to a library, so this header does
 * their job: every src/inslib_tu_*.c wrapper includes it before the INSLIB
 * source it compiles, and TriSenseINS.h includes it before the INSLIB headers.
 *
 * That second include is not cosmetic. INS_UNKNOWNS_MAX changes the layout of
 * ins_t, so every translation unit that sees ins.h has to agree on it. If you
 * write code that includes an INSLIB header directly, include this first (or
 * just include TriSenseINS.h, which does).
 *
 * Do not edit these numbers unless you have read the stack note below.
 */
#ifndef TRISENSE_INSLIB_CONFIG_H
#define TRISENSE_INSLIB_CONFIG_H

/* ---------------------------------------------------------------------------
 * Platform gate
 * ---------------------------------------------------------------------------
 * INSLIB is single-precision float throughout and needs a hardware FPU to run
 * at a useful rate. AVR has neither the FPU nor the RAM (the filter struct
 * alone is ~15 KB). The RP2040 has the RAM but no FPU, and RP2350's RISC-V
 * (Hazard3) cores have no FPU either.
 */
#if defined(__AVR__)
  #error "TriSenseINSLIB needs a 32-bit MCU with a hardware FPU (Raspberry Pi Pico 2 / RP2350, ESP32-S3, Teensy 4.x). AVR boards have neither the FPU nor the RAM for a 15-state Kalman filter - use the fusion filters in Voltino_TriSense instead."
#endif
#if defined(ARDUINO_ARCH_RP2040) && !defined(PICO_RP2350)
  #error "TriSenseINSLIB needs an RP2350 (Raspberry Pi Pico 2). The RP2040 has no FPU, so the filter's float arithmetic would run in software - use the fusion filters in Voltino_TriSense (e.g. examples/GPS_INS_Localization) on RP2040."
#endif
#if defined(ARDUINO_ARCH_RP2040) && defined(PICO_RP2350) && !defined(__ARM_FP)
  #error "Select the ARM (Cortex-M33) architecture for the RP2350, not RISC-V: the Hazard3 cores have no FPU."
#endif
#if defined(ARDUINO) && defined(__riscv) && !defined(__riscv_flen)
  #warning "TriSenseINSLIB: this RISC-V target has no FPU - the filter's float arithmetic runs in software and will be slow."
#endif

/* ---------------------------------------------------------------------------
 * State vector size: the one that decides the stack budget
 * ---------------------------------------------------------------------------
 * INSLIB dimensions its covariance and history arrays from INS_UNKNOWNS_MAX,
 * and KFCore dimensions the scratch matrices inside kalman_udu_predict() from
 * KALMAN_MAX_STATE_SIZE / KALMAN_MAX_NOISE_SIZE. All of it lands on the stack
 * of whoever calls nav_suite_update(). Measured with INSLIB's call-graph stack
 * analysis for a Pico 2 (arm-none-eabi-gcc, cortex-m33, -Os):
 *
 *     INS_UNKNOWNS_MAX=15, KALMAN 15/21  ->  nav_suite_update() needs  6776 B
 *     INS_UNKNOWNS_MAX=18, KALMAN 18/24  ->  nav_suite_update() needs  8528 B
 *     INS_UNKNOWNS_MAX=15, KFCore 32/32  ->  nav_suite_update() needs 12880 B
 *
 * A core on arduino-pico has 8192 B of stack, so 15 states fits with ~1.4 KB to
 * spare, 18 does not fit, and KFCore's desktop-sized defaults overflow inside
 * the first Kalman prediction. On a Cortex-M that is not a crash but silent
 * corruption of whatever lies below the stack.
 *
 * 15 states is the full INS: position, velocity, attitude, accelerometer bias,
 * gyroscope bias. Only INSLIB's optional magnetometer hard-iron estimator
 * (estimate_mag_bias, 18 states) is given up - calibrate the magnetometer with
 * the MotionCal example and pass the result to setMagCalibration() instead.
 */
#ifndef INS_UNKNOWNS_MAX
#define INS_UNKNOWNS_MAX      15
#endif
#ifndef KALMAN_MAX_STATE_SIZE
#define KALMAN_MAX_STATE_SIZE 15
#endif
/* INS_NOISE_COLS_MAX == 12 + INS_UNKNOWNS_MAX - 6 == 21. ins.c turns a wrong
 * value into a _Static_assert failure rather than an overflowing write. */
#ifndef KALMAN_MAX_NOISE_SIZE
#define KALMAN_MAX_NOISE_SIZE 21
#endif

/* Worst-case stack use of one nav_suite_update() call with the sizes above, in
 * bytes, plus a little for TriSenseINS::update() itself. TriSenseINS compares
 * the free stack against this on every update (where the platform can tell).
 */
#define TRISENSE_INS_STACK_NEEDED 7000

/* ---------------------------------------------------------------------------
 * Logging off
 * ---------------------------------------------------------------------------
 * INSLIB's default log sink is printf(), which blocks and drags ~2 KB of extra
 * stack into the filter's worst-case path. At LOG_LEVEL 0 every log call
 * disappears in the preprocessor. (The literal 0 rather than LOG_LEVEL_NONE is
 * deliberate: the symbolic name is defined inside log.h, after this header.)
 */
#ifndef LOG_LEVEL
#define LOG_LEVEL 0
#endif

/* assert() off - the release setting the stack figures were measured with.
 * The size checks that matter are _Static_assert and fire regardless. */
#ifndef NDEBUG
#define NDEBUG 1
#endif

#endif /* TRISENSE_INSLIB_CONFIG_H */
