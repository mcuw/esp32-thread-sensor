/* Adapter: Bosch BME280 / BMP280, Temperature, Forced Mode.
 * ATTENTION: LP-I2C-API: lp_core_i2c_master_* aus lp_core_i2c.h. */
#include <stddef.h>
#include "sensor.h"
#include "ulp_lp_core.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_i2c.h"

#ifndef CONFIG_SENSOR_I2C_ADDR
#define CONFIG_SENSOR_I2C_ADDR 0x76
#endif

#define REG_ID          0xD0      /* 0x60 = BME280, 0x58 = BMP280 */
#define REG_CALIB_T     0x88      /* dig_T1..dig_T3, 6 Bytes */
#define REG_CTRL_MEAS   0xF4
#define REG_TEMP        0xFA
#define CTRL_FORCED_T1  0x21      /* osrs_t x1, Pressure off, Forced Mode */
#define I2C_TIMEOUT     5000

volatile uint32_t sensor_id = 0;
volatile uint32_t sensor_i2c_errors = 0;
volatile uint32_t sensor_invalid_errors = 0;
volatile uint32_t sensor_last_err = 0;

static bool    calib_ok = false;
static int32_t dig_T1, dig_T2, dig_T3;

static bool reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    esp_err_t r = lp_core_i2c_master_write_read_device(LP_I2C_NUM_0, CONFIG_SENSOR_I2C_ADDR,
                                                       &reg, 1, buf, len, I2C_TIMEOUT);
    sensor_last_err = (uint32_t)r;
    return r == ESP_OK;
}

static bool reg_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, CONFIG_SENSOR_I2C_ADDR,
                                                  b, 2, I2C_TIMEOUT) == ESP_OK;
}

bool sensor_init(void)
{
    uint8_t id, c[6];
    if (calib_ok) return true;

    if (!reg_read(REG_ID, &id, 1)) { sensor_i2c_errors++; return false; }
    sensor_id = id;
    if (id != 0x60 && id != 0x58) { sensor_invalid_errors++; return false; }
    if (!reg_read(REG_CALIB_T, c, 6)) { sensor_i2c_errors++; return false; }

    uint32_t t1 = (uint32_t)c[0] | ((uint32_t)c[1] << 8);
    if (t1 == 0 || t1 == 0xFFFF) { sensor_invalid_errors++; return false; }
    dig_T1 = (int32_t)t1;
    dig_T2 = (int16_t)((uint32_t)c[2] | ((uint32_t)c[3] << 8));
    dig_T3 = (int16_t)((uint32_t)c[4] | ((uint32_t)c[5] << 8));
    calib_ok = true;
    return true;
}

bool sensor_start(void)
{
    if (!reg_write(REG_CTRL_MEAS, CTRL_FORCED_T1)) { sensor_i2c_errors++; return false; }
    return true;
}

/* Compensation described in the BME280-datasheet (32-Bit-integer), result in 0,01 Grad C */
static int32_t compensate_c100(int32_t adc_T)
{
    int32_t var1 = (((adc_T >> 3) - (dig_T1 << 1)) * dig_T2) >> 11;
    int32_t d    = (adc_T >> 4) - dig_T1;
    int32_t var2 = (((d * d) >> 12) * dig_T3) >> 14;
    int32_t t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

bool sensor_read(int32_t *t_c100)
{
    uint8_t b[3];
    if (!reg_read(REG_TEMP, b, 3)) { sensor_i2c_errors++; return false; }

    int32_t adc = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    if (adc == 0x80000) { sensor_invalid_errors++; return false; }   /* no messurement */

    int32_t t = compensate_c100(adc);
    if (t <= -4000 || t >= 8500) { sensor_invalid_errors++; return false; }
    *t_c100 = t;
    return true;
}

