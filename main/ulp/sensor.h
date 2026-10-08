/*
 * Sensor-Adapter for the LP-Core.
 *
 * All sensors implements this interface. Which implementation is build,
 * is decided by the main/CMakeLists.txt depend on CONFIG_SENSOR_*.
 * A new sensor requires: a new sensor_xxx.c file, an entry in Kconfig.projbuild
 * and an entry in main/CMakeLists.txt.
 *
 * Flow in LP-Core (Pipeline, the LP-Core never waits for a sensor):
 *   Lauf N:    sensor_read()  get the result from sequence N-1
 *              sensor_start() triggers a next messurement
 */
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#if defined(__has_include)
#  if __has_include("sdkconfig.h")
#    include "sdkconfig.h"      /* get CONFIG_SENSOR_I2C_ADDR, if available */
#  endif
#endif

/* Diagnose, is shown as ulp_sensor_id etc. in the main app */
extern volatile uint32_t sensor_id;              /* 0 = not reached yet */
extern volatile uint32_t sensor_i2c_errors;
extern volatile uint32_t sensor_invalid_errors;  /* CRC- or validation error */
extern volatile uint32_t sensor_last_err;

/* Prepare sensor (check ID, read calibration data). Can be called on
 * each sequence, does nothing after success. false = not ready. */
bool sensor_init(void);

/* Trigger a messurement. false on I2C-error. */
bool sensor_start(void);

/* t_c100:  Temperature in 0,01 Grad C.
 * rh_c100: Humidity in 0,01 %RH, or -1 if not available (BMP280, CRC-Fehler).
 * Returns false: if temperature is invalid. */
bool sensor_read(int32_t *t_c100, int32_t *rh_c100);

#endif
