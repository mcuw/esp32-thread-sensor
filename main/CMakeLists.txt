idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ""
                       REQUIRES ulp driver esp_driver_gpio esp_driver_i2c nvs_flash
                       # Fuer Thread einkommentieren:
                       # openthread esp_event esp_netif
                       )

set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_sources "ulp/lp_main.c")

# Sensor-Adapter: only the selected sensor implementation is build for the LP-core
if(CONFIG_SENSOR_BME280)
    list(APPEND ulp_sources "ulp/sensor_bme280.c")
elseif(CONFIG_SENSOR_SHT4X)
    list(APPEND ulp_sources "ulp/sensor_sht4x.c")
else()
    message(FATAL_ERROR "Please select a sensor: idf.py menuconfig -> Thermo Sensor")
endif()

set(ulp_exp_dep_srcs "main.c")

ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
