/*
 * ESP32-C6 Sensor node
 * - use LP-core to meassure the temperature
 * - you can select several sensor modules with the menuconfig
 * - main core sendes messages over Thread to the heating-ESP32-C6 (receiver).
 * - Thread is only prepared as a stub by now,
 *   see part "THREAD-INTEGRATION" and marks [THREAD-1] ... [THREAD-6].
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "esp_sleep.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "lp_core_i2c.h"
#include "ulp_lp_core.h"
#include "ulp_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define TEST_AWAKE 0      /* set 0 in production */

/* Single temperature-comparision: set reference-value in Grad C (only with TEST_AWAKE 1).
 * The sensor have to lay beside a reference-thermometer for around minutes.
 * Then set to 0.0f again. */
#define CAL_TEMP_REF_C  0.0f

/* [THREAD-1] Includes (bei aktiviertem Thread einkommentieren)
 * #include "esp_event.h"
 * #include "esp_netif.h"
 * #include "esp_openthread.h"
 * #include "esp_openthread_netif_glue.h"
 * #include "esp_openthread_types.h"
 * #include "openthread/thread.h"
 * #include "openthread/dataset.h"
 * #include "openthread/coap.h"       // wenn CoAP genutzt wird
 * #include "openthread/udp.h"        // wenn rohes UDP genutzt wird
 */

static const char *TAG = "main";

#if CONFIG_SENSOR_BME280
    #define SENSOR_NAME "BME280"
#elif CONFIG_SENSOR_SHT4X
    #define SENSOR_NAME "SHT4x"
#else
    #error "Please select a sensor (see menuconfig -> Thermo Sensor)"
#endif

extern const uint8_t lp_core_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t lp_core_main_bin_end[]   asm("_binary_ulp_main_bin_end");

#define MEASURE_PERIOD_US   (2 * 1000 * 1000)   /* LP-Core meassures every 2s */
#define HEARTBEAT_S         (15 * 60)           /* 15m heartbeat to receiver */
//#define HEARTBEAT_S         10                /* reduce heartbeat to 10s for debugging */

/* Thresdhold range in Grad Celcius */
#define HEAT_ON_C    23.0f
#define HEAT_OFF_C   24.0f
#define FAN_ON_C     26.5f
#define FAN_OFF_C    26.0f
#define HIT_LIMIT    3

RTC_DATA_ATTR static bool     ulp_started = false;   /* keep also after a deep-sleep - for other RTC_DATA_ATTR */
RTC_DATA_ATTR static bool     heat_on = false;
RTC_DATA_ATTR static bool     fan_on  = false;
RTC_DATA_ATTR static uint16_t seq     = 0;      /* Sequencenumber prevents replay/ doublicated */

/* Payload, which will be send over Thread (keep small, no paddings) */
typedef struct __attribute__((packed)) {
    uint8_t  version;     /* Protokollversion = 1                         */
    uint8_t  flags;       /* Bit0 = heat on, Bit1 = fan on,
                             Bit2 = Sensorerror                           */
    uint16_t seq;         /* incremented number                           */
    int16_t  temp_c_x100; /* temperature in 0,01 Grad C                   */
    uint16_t rh_x100;     /* Humidity in 0,01 %RH, 0xFFFF = not available */
} node_msg_t;

/* Configuration from receiver (e. g. from the CoAP-response for the Heartbeat) */
typedef struct __attribute__((packed)) {
    uint8_t version;              /* = 1 */
    int16_t temp_offset_c100;     /* 0,01 Grad C */
    int16_t rh_offset_c100;       /* 0,01 %RH    */
} node_cfg_t;

/* Grad C -> 0,01 Grad C (the LP-core calculates in integers) */
static int32_t c100(float c)
{
    return (int32_t)(c * 100.0f + (c >= 0 ? 0.5f : -0.5f));
}

static float rh_or_nan(void)
{
    int32_t r = (int32_t)ulp_last_rh_c100;
    return r < 0 ? NAN : r / 100.0f;
}

/* ============ Offsets: NVS + LP-Core ============ */
#define NVS_NS  "thermo"

static bool nvs_ready = false;
static void nvs_init_once(void)
{
    if (nvs_ready) return;
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);
    nvs_ready = true;
}

/* Kconfig-value, overwritten with a stored NVS-value (except FROM_KCONFIG) */
static void load_offsets(int32_t *t_off, int32_t *rh_off)
{
    *t_off  = CONFIG_THERMO_TEMP_OFFSET_C100;
    *rh_off = CONFIG_THERMO_RH_OFFSET_C100;
#if !CONFIG_THERMO_OFFSET_FROM_KCONFIG
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        int32_t v;
        if (nvs_get_i32(h, "t_off", &v)  == ESP_OK) *t_off  = v;
        if (nvs_get_i32(h, "rh_off", &v) == ESP_OK) *rh_off = v;
        nvs_close(h);
    }
#endif
}

/* Saves the Offsets persistent (NVS) and activate it inthe LP-Core.
 * Valid for next messurements; the hysterese handle small flaky jumps. */
static esp_err_t apply_offsets(int32_t t_off, int32_t rh_off)
{
    if (t_off < -1000 || t_off > 1000 || rh_off < -2000 || rh_off > 2000)
        return ESP_ERR_INVALID_ARG;
    nvs_init_once();
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_i32(h, "t_off", t_off);
    if (r == ESP_OK) r = nvs_set_i32(h, "rh_off", rh_off);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    if (r == ESP_OK) {
        ulp_temp_offset_c100 = (uint32_t)t_off;
        ulp_rh_offset_c100   = (uint32_t)rh_off;
    }
    return r;
}

/* apply configuration from receiver (see [THREAD-5b]) */
__attribute__((unused))
static void handle_cfg(const node_cfg_t *c)
{
    if (c->version != 1) return;
    esp_err_t r = apply_offsets(c->temp_offset_c100, c->rh_offset_c100);
    ESP_LOGI(TAG, "cfg: t_off=%d rh_off=%d -> %s", c->temp_offset_c100, c->rh_offset_c100,
             esp_err_to_name(r));
}

/* Calibrate the temperature with a reference-thermometer.
 * new Offset = old Offset + (reference - current value) */
__attribute__((unused))
static void calibrate_temp_to_reference(float ref_c)
{
    int32_t shown = (int32_t)ulp_last_temp_c100;       /* contains the current Offset */
    int32_t cur   = (int32_t)ulp_temp_offset_c100;
    int32_t neu   = cur + (c100(ref_c) - shown);
    esp_err_t r = apply_offsets(neu, (int32_t)ulp_rh_offset_c100);
    ESP_LOGI(TAG, "Calibration: current=%.2f ref=%.2f offset %ld -> %ld (%s)",
             shown / 100.0f, ref_c, (long)cur, (long)neu, esp_err_to_name(r));
}

/* ===================== THREAD-INTEGRATION (Vorbereitung) =====================
 *
 * Ablauf bei JEDEM Wakeup der Haupt-CPU, die etwas senden muss:
 *
 *  [THREAD-2] Einmalig beim ersten Boot: Dataset vorgeben (Commissioning).
 *      Der Empfaenger-C6 (Router/FTD) bildet das Netz. Sein Active Dataset
 *      (Channel, PAN-ID, Network Key, ...) traegst du hier ein oder
 *      uebertraegst es per Commissioner/Joiner. Das Dataset landet im NVS
 *      und ueberlebt den Deep-Sleep, solange NVS nicht geloescht wird.
 *          otOperationalDataset ds = { ... };
 *          otDatasetSetActive(instance, &ds);
 *
 *  [THREAD-3] Stack starten (nach jedem Deep-Sleep-Boot noetig):
 *          nvs_flash_init();
 *          esp_event_loop_create_default();
 *          esp_openthread_platform_config_t cfg = { radio: native, host: none,
 *                                                   port: default };
 *          esp_openthread_init(&cfg);
 *          otThreadSetEnabled(instance, true);   // laedt gespeichertes Dataset
 *          // Als MTD/SED: otLinkSetPollPeriod(instance, ...) sinnvoll setzen.
 *          // Mit gespeichertem Dataset und bekanntem Parent ist der Re-Attach
 *          // deutlich schneller als ein Neuaufbau. Reale Dauer MESSEN.
 *
 *  [THREAD-4] Auf Rolle warten (mit Timeout, z. B. 3 bis 5 s):
 *          Warten bis otThreadGetDeviceRole() == CHILD (oder ROUTER/LEADER).
 *          Bei Timeout: Nachricht verwerfen, seq nicht erhoehen, in den Schlaf
 *          gehen und beim naechsten Heartbeat oder Ereignis erneut versuchen.
 *
 *  [THREAD-5] Nachricht senden (node_msg_t):
 *          Variante CoAP (empfohlen, mit Bestaetigung):
 *              otCoapNewMessage(), Pfad z. B. "heat", CON-Request an die
 *              Mesh-Local-Adresse oder per Service/Anycast des Empfaengers.
 *              Auf Bestaetigung (ACK) warten, bei Fehlschlag 1 bis 2 Retries.
 *          Variante UDP:
 *              otUdpOpen(), otUdpSend() an [Empfaenger-IPv6]:Port.
 *          Empfaenger-Adresse: fest eintragen oder ueber DNS-SD/SRP/Anycast
 *          ermitteln und im RTC-Speicher merken (RTC_DATA_ATTR).
 *
 *  [THREAD-5b] Konfiguration zurueck: Der Empfaenger kann in der CoAP-Antwort
 *          ein node_cfg_t mitschicken (Offsets). Der Sensorknoten ruft damit
 *          handle_cfg() auf. Der Benutzer aendert den Wert also am Empfaenger.
 *
 *  [THREAD-6] Vor dem Schlafen sauber beenden:
 *          Erst nach ACK (oder Timeout) weitermachen.
 *          otThreadSetEnabled(instance, false);
 *          esp_openthread_deinit();   // Radio aus, damit Deep-Sleep sparsam bleibt
 *
 * Sicherheit: Thread verschluesselt (AES-CCM) mit dem Network Key auf Mesh-
 * Ebene. Fuer zusaetzlichen Schutz der Anwendungsdaten kannst du seq pruefen
 * (Empfaenger verwirft alte/doppelte seq) oder CoAP ueber DTLS nutzen.
 *
 * Empfaenger (Heizungs-C6): FTD/Router, dauerhaft am Netz.
 *   - CoAP-Server auf Pfad "heat" oeffnen, node_msg_t auswerten (version 2).
 *   - Relais nur schalten, wenn version und seq gueltig sind.
 *   - Watchdog: ohne Nachricht innerhalb von 2 Heartbeats (hier 30 min)
 *     Heizung UND Ventilator abschalten (Fail-safe).
 *   - flags Bit2 (Sensorfehler) -> sofort alles abschalten.
 * ============================================================================ */


static bool thread_send(const node_msg_t *msg)
{
    /* [THREAD-2] bis [THREAD-6] hier umsetzen und true bei erfolgreicher
     * Zustellung (ACK) zurueckgeben. */
    ESP_LOGI(TAG, "SEND(stub) seq=%u flags=0x%02x T=%.2f C RH=%.1f %%",
             msg->seq, msg->flags, msg->temp_c_x100 / 100.0f,
             msg->rh_x100 == 0xFFFF ? NAN : msg->rh_x100 / 100.0f);
    return true;   /* Stub: ohne Thread immer "gesendet" */
}

/* build message object and send it */
static void send_to_receiver(bool heat, bool fan, bool fault)
{
    int32_t rh = (int32_t)ulp_last_rh_c100;
    node_msg_t m = {
        .version     = 1,
        .flags       = (heat ? 1 : 0) | (fan ? 2 : 0) | (fault ? 4 : 0),
        .seq         = seq,
        .temp_c_x100 = (int16_t)(int32_t)ulp_last_temp_c100,
        .rh_x100     = rh < 0 ? 0xFFFF : (uint16_t)rh,
    };
    if (thread_send(&m)) {
        seq++;
    } else {
        ESP_LOGW(TAG, "Failed to a send mesage, will try it again on next wake-up");
    }
}

static void init_first_boot(void)
{
    nvs_init_once();    /* NVS: Thread-dataset und Offsets */

    lp_core_i2c_cfg_t i2c_cfg = LP_CORE_I2C_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c_cfg));

    ESP_ERROR_CHECK(ulp_lp_core_load_binary(lp_core_main_bin_start,
                    (lp_core_main_bin_end - lp_core_main_bin_start)));

    ulp_heat_on_c100  = (uint32_t)c100(HEAT_ON_C);
    ulp_heat_off_c100 = (uint32_t)c100(HEAT_OFF_C);
    ulp_fan_on_c100   = (uint32_t)c100(FAN_ON_C);
    ulp_fan_off_c100  = (uint32_t)c100(FAN_OFF_C);
    ulp_hit_limit     = HIT_LIMIT;
    ulp_period_us     = MEASURE_PERIOD_US;
    ulp_heat_cmd = 2;
    ulp_fan_cmd  = 2;

    int32_t t_off, rh_off;
    load_offsets(&t_off, &rh_off);
    ulp_temp_offset_c100 = (uint32_t)t_off;
    ulp_rh_offset_c100   = (uint32_t)rh_off;
#if CONFIG_THERMO_OFFSET_FROM_KCONFIG
    apply_offsets(t_off, rh_off);                      /* use Kconfig-values from NVS */
#endif

    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER,
        .lp_timer_sleep_duration_us = MEASURE_PERIOD_US,
    };
    ESP_ERROR_CHECK(ulp_lp_core_run(&cfg));

    /* [THREAD-2] save Thread-Dataset once if needed */
}

void app_main(void)
{
    uint32_t causes = esp_sleep_get_wakeup_causes();
    ESP_LOGD(TAG, "causes=0x%lx ulp_started=%d", (unsigned long)causes, (int)ulp_started);

    if (!ulp_started) {
        heat_on = fan_on = false;
        seq = 0;
        init_first_boot();
        ulp_started = true;
    } else if (causes & BIT(ESP_SLEEP_WAKEUP_ULP)) {
        uint32_t ev = ulp_event_flags;
        if (ev & 4) {                               /* Sensorerror */
            heat_on = fan_on = false;
            send_to_receiver(false, false, true);
        } else {
            if (ev & 1) heat_on = (ulp_heat_cmd == 1);
            if (ev & 2) fan_on  = (ulp_fan_cmd  == 1);
            send_to_receiver(heat_on, fan_on, false);
        }
        ulp_event_flags = 0;
    } else if (causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
        /* sensor_id: BME280 0x60, BMP280 0x58, SHT4x = I2C-address, 0 = not available */
        ESP_LOGI(TAG, "%s id=0x%02x T=%.2f C RH=%.1f %% off=%ld/%ld i2c_err=%lu invalid=%lu", SENSOR_NAME,
            (unsigned)(ulp_sensor_id & 0xFF), (int32_t)ulp_last_temp_c100 / 100.0f, rh_or_nan(),
            (long)(int32_t)ulp_temp_offset_c100, (long)(int32_t)ulp_rh_offset_c100,
            (unsigned long)ulp_sensor_i2c_errors, (unsigned long)ulp_sensor_invalid_errors);
        send_to_receiver(heat_on, fan_on, false); /* Heartbeat */
    }


#if TEST_AWAKE
    if (CAL_TEMP_REF_C > 0.0f) {
        vTaskDelay(pdMS_TO_TICKS(8000));            /* wait for a valid messurement */
        calibrate_temp_to_reference(CAL_TEMP_REF_C);
    }
    while (1) {
        ESP_LOGI(TAG, "T=%.2f C RH=%.1f %% heat=%lu fan=%lu ev=0x%lx id=0x%02x i2c_err=%lu inv=%lu run=%lu err=0x%lx off=%ld/%ld",
                 (int32_t)ulp_last_temp_c100 / 100.0f, rh_or_nan(),
                 (unsigned long)ulp_heat_cmd, (unsigned long)ulp_fan_cmd,
                 (unsigned long)ulp_event_flags,
                 (unsigned)(ulp_sensor_id & 0xFF),
                 (unsigned long)ulp_sensor_i2c_errors,
                 (unsigned long)ulp_sensor_invalid_errors,
                 (unsigned long)ulp_run_count,
                 (unsigned long)ulp_sensor_last_err,
                 (long)(int32_t)ulp_temp_offset_c100, (long)(int32_t)ulp_rh_offset_c100);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

    // start deep-sleep mode to reduce the power consumption
    // wake-up by ULP
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup((uint64_t)HEARTBEAT_S * 1000000ULL));
    fflush(stdout);
    esp_deep_sleep_start();
}
