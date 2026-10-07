/*
 * LP-core-application (ESP32-C6): contains sensor independed logic.
 * The sensor is abstracted behind the interface in sensor.h (see sensor_*.c).
 * Two Thresholdpairs (Heat / Fan) with debounce
 * The main CPU is only powered on when a command has changed or on sensor error.
 *
 * LP-I2C-Pins for the ESP32-C6: SDA = GPIO6, SCL = GPIO7.
 */
#include <stdint.h>
#include <stdbool.h>
#include "ulp_lp_core.h"
#include "ulp_lp_core_utils.h"
#include "sensor.h"
#include "ulp_lp_core_lp_timer_shared.h"   /* header-name */

#define EV_HEAT     (1u << 0)
#define EV_FAN      (1u << 1)
#define EV_FAIL     (1u << 2)
#define FAIL_LIMIT  10

/* Initial values set by main core on boot time, all values in 0,01 Grad C */
volatile int32_t  heat_on_c100      = 2300;   /* below -> heat on  */
volatile int32_t  heat_off_c100     = 2400;   /* above -> heat off */
volatile int32_t  fan_on_c100       = 2650;   /* above -> Fan on   */
volatile int32_t  fan_off_c100      = 2600;   /* below -> Fan off  */
volatile int32_t  temp_offset_c100  = 0;      /* adjust to referencethermometer */
volatile uint32_t hit_limit         = 3;      /* Change trigger when MORE then hit_limit times reached */

/* Shared state, which is readable by the main CPU */
volatile uint32_t heat_cmd        = 2;        /* 0 = aus, 1 = an, 2 = unbekannt */
volatile uint32_t fan_cmd         = 2;
volatile uint32_t event_flags     = 0;
volatile int32_t  last_temp_c100  = 0;

volatile uint32_t run_count = 0;
volatile uint32_t period_us = 2000000;

/* only private */
static uint32_t heat_cnt = 0, fan_cnt = 0, fail_cnt = 0, pending = 0;

/* 1 = set on, 0 = set off, -1 = hysteresezone (keep) */
static int heat_target(int32_t t)
{
    if (t < heat_on_c100)  return 1;
    if (t > heat_off_c100) return 0;
    return -1;
}
static int fan_target(int32_t t)
{
    if (t > fan_on_c100)  return 1;
    if (t < fan_off_c100) return 0;
    return -1;
}

/* true on stable change */
static bool debounce(volatile uint32_t *cmd, uint32_t *cnt, int target)
{
    if (*cmd == 2) {                                   /* firsr classification */
        if (target < 0) { *cmd = 0; *cnt = 0; return false; }
        *cmd = (uint32_t)target; *cnt = 0;
        return true;
    }
    if (target < 0 || (uint32_t)target == *cmd) { *cnt = 0; return false; }
    if (++(*cnt) > hit_limit) { *cmd = (uint32_t)target; *cnt = 0; return true; }
    return false;
}

static uint32_t cycle(void)
{
    uint32_t events = 0;
    bool got = false;
    bool expected = (pending != 0);

    if (!sensor_init()) {
        if (++fail_cnt == FAIL_LIMIT) events |= EV_FAIL;
        return events;
    }

    /* Get the result of the previous messurement */
    if (expected) {
        int32_t t;
        pending = 0;
        if (sensor_read(&t)) {
            last_temp_c100 = t + temp_offset_c100;
            got = true;
        }
    }

    if (got) {
        fail_cnt = 0;
        int32_t t = last_temp_c100;
        if (debounce(&heat_cmd, &heat_cnt, heat_target(t))) events |= EV_HEAT;
        if (debounce(&fan_cmd,  &fan_cnt,  fan_target(t)))  events |= EV_FAN;
    } else if (expected) {
        if (++fail_cnt == FAIL_LIMIT) events |= EV_FAIL;
    }

    /* start next messurement */
    if (sensor_start()) {
        pending = 1;
    } else if (++fail_cnt == FAIL_LIMIT) {
        events |= EV_FAIL;
    }
    return events;
}

int main(void)
{
    run_count++;

    uint32_t events = cycle();

    if (events & EV_FAIL) {          /* new classification after sensor failed, when the sensor is reachable again */
        heat_cmd = 2;
        fan_cmd  = 2;
    }
    if (events) {
        event_flags = events;
        ulp_lp_core_wakeup_main_processor();
    }

    ulp_lp_core_lp_timer_set_wakeup_time (period_us);
    ulp_lp_core_halt();
    return 0;
}
