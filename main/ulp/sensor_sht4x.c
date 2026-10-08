/* Adapter: Sensirion SHT4x (SHT40 / SHT41 / SHT45). Temperature and Humidity come from single messurement.
 * ATTENTION: LP-I2C-API: lp_core_i2c_master_* from lp_core_i2c.h. */
#include "sensor.h"
#include "ulp_lp_core.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_i2c.h"

#ifndef CONFIG_SENSOR_I2C_ADDR
#define CONFIG_SENSOR_I2C_ADDR 0x44
#endif

#define CMD_MEASURE_LOW  0xE0     /* low repeatability, ca. 1,7 ms; 0xF6 = medium, 0xFD = high */
#define I2C_TIMEOUT      5000

volatile uint32_t sensor_id = 0;  /* will be set after first successfully read of the address */
volatile uint32_t sensor_i2c_errors = 0;
volatile uint32_t sensor_invalid_errors = 0;
volatile uint32_t sensor_last_err = 0;

static uint8_t crc8(const uint8_t *d)   /* Polynom 0x31, Init 0xFF, 2 Bytes */
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

bool sensor_init(void)
{
    return true;    /* no calibration required; Exist will be shown on sensor_start() */
}

bool sensor_start(void)
{
    uint8_t cmd = CMD_MEASURE_LOW;
    esp_err_t r = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, CONFIG_SENSOR_I2C_ADDR,
                                                     &cmd, 1, I2C_TIMEOUT);
    sensor_last_err = (uint32_t)r;
    if (r != ESP_OK) { sensor_i2c_errors++; return false; }
    return true;
}

bool sensor_read(int32_t *t_c100, int32_t *rh_c100)
{
    uint8_t b[6];
    esp_err_t r = lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, CONFIG_SENSOR_I2C_ADDR,
                                                      b, 6, I2C_TIMEOUT);
    sensor_last_err = (uint32_t)r;
    if (r != ESP_OK) { sensor_i2c_errors++; return false; }
    if (crc8(&b[0]) != b[2]) { sensor_invalid_errors++; return false; }

    uint32_t raw_t = ((uint32_t)b[0] << 8) | b[1];
    /* T = -45 + 175 * raw / 65535 */
    *t_c100 = (int32_t)((raw_t * 17500u + 32767u) / 65535u) - 4500;

    *rh_c100 = -1;
    if (crc8(&b[3]) == b[5]) {
        uint32_t raw_h = ((uint32_t)b[3] << 8) | b[4];
        /* RH = -6 + 125 * raw / 65535, auf 0..100 % begrenzt */
        int32_t rh = (int32_t)((raw_h * 12500u + 32767u) / 65535u) - 600;
        if (rh < 0) rh = 0;
        if (rh > 10000) rh = 10000;
        *rh_c100 = rh;
    } else {
        sensor_invalid_errors++;
    }
    sensor_id = CONFIG_SENSOR_I2C_ADDR;
    return true;
}
