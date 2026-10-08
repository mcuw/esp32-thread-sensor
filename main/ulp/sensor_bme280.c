/* Adapter: Bosch BME280 / BMP280, Temperature, Humidity (for BME280), Forced Mode. Pressure is not messured.
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
#define REG_CALIB_H1    0xA1      /* dig_H1, 1 Byte */
#define REG_CALIB_H2    0xE1      /* dig_H2..dig_H6, 0xE1..0xE7 = 7 Bytes */
#define REG_CTRL_HUM    0xF2      /* muss VOR ctrl_meas geschrieben werden */
#define REG_CTRL_MEAS   0xF4
#define REG_DATA        0xFA      /* temp (3 Bytes), danach hum (2 Bytes) */
#define CTRL_HUM_X1     0x01      /* osrs_h x1 */
#define CTRL_FORCED_T1  0x21      /* osrs_t x1, Druck aus, Forced Mode */
#define I2C_TIMEOUT     5000

volatile uint32_t sensor_id = 0;
volatile uint32_t sensor_i2c_errors = 0;
volatile uint32_t sensor_invalid_errors = 0;
volatile uint32_t sensor_last_err = 0;

static bool    calib_ok = false;
static bool    is_bme   = false;       /* true = BME280 (has Humidity) */
static int32_t dig_T1, dig_T2, dig_T3;
static int32_t dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6;

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
    esp_err_t r = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, CONFIG_SENSOR_I2C_ADDR,
                                                     b, 2, I2C_TIMEOUT);
    sensor_last_err = (uint32_t)r;
    return r == ESP_OK;
}

bool sensor_init(void)
{
    uint8_t id, c[6], h1, hc[7];
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

    is_bme = (id == 0x60);
    if (is_bme) {
        if (!reg_read(REG_CALIB_H1, &h1, 1)) { sensor_i2c_errors++; return false; }
        if (!reg_read(REG_CALIB_H2, hc, 7))  { sensor_i2c_errors++; return false; }
        dig_H1 = h1;
        dig_H2 = (int16_t)((uint32_t)hc[0] | ((uint32_t)hc[1] << 8));
        dig_H3 = hc[2];
        dig_H4 = (int16_t)((int32_t)(int8_t)hc[3] * 16 | (hc[4] & 0x0F));
        dig_H5 = (int16_t)((int32_t)(int8_t)hc[5] * 16 | (hc[4] >> 4));
        dig_H6 = (int8_t)hc[6];
    }
    calib_ok = true;
    return true;
}

bool sensor_start(void)
{
    if (is_bme && !reg_write(REG_CTRL_HUM, CTRL_HUM_X1)) { sensor_i2c_errors++; return false; }
    if (!reg_write(REG_CTRL_MEAS, CTRL_FORCED_T1))        { sensor_i2c_errors++; return false; }
    return true;
}

/* Compensation described in the BME280-datasheet (32-Bit-integer), result in 0,01 Grad C */
static int32_t compensate_t_c100(int32_t adc_T, int32_t *t_fine)
{
    int32_t var1 = (((adc_T >> 3) - (dig_T1 << 1)) * dig_T2) >> 11;
    int32_t d    = (adc_T >> 4) - dig_T1;
    int32_t var2 = (((d * d) >> 12) * dig_T3) >> 14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;
}

/* datasheet, 32-Bit-integer (Q22.10 in %RH). Result in 0,01 %RH. */
static int32_t compensate_rh_c100(int32_t adc_H, int32_t t_fine)
{
    int32_t v = t_fine - 76800;
    v = (((((adc_H << 14) - (dig_H4 << 20) - (dig_H5 * v)) + 16384) >> 15) *
         (((((((v * dig_H6) >> 10) * (((v * dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
           dig_H2 + 8192) >> 14));
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * dig_H1) >> 4);
    if (v < 0) v = 0;
    if (v > 419430400) v = 419430400;
    uint32_t h1024 = (uint32_t)v >> 12;                 /* %RH * 1024 */
    return (int32_t)((h1024 * 25u + 128u) >> 8);        /* * 100 / 1024 */
}

bool sensor_read(int32_t *t_c100, int32_t *rh_c100)
{
    uint8_t b[5];
    if (!reg_read(REG_DATA, b, is_bme ? 5 : 3)) { sensor_i2c_errors++; return false; }

    int32_t adc_T = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    if (adc_T == 0x80000) { sensor_invalid_errors++; return false; }   /* keine Messung */

    int32_t t_fine;
    int32_t t = compensate_t_c100(adc_T, &t_fine);
    if (t <= -4000 || t >= 8500) { sensor_invalid_errors++; return false; }

    *t_c100  = t;
    *rh_c100 = -1;
    if (is_bme) {
        int32_t adc_H = ((int32_t)b[3] << 8) | b[4];
        if (adc_H != 0x8000) {                          /* 0x8000 = nicht gemessen */
            *rh_c100 = compensate_rh_c100(adc_H, t_fine);
        } else {
            sensor_invalid_errors++;
        }
    }
    return true;
}
