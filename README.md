| Supported Targets | ESP32-C6 |
| ----------------- | -------- |

# ESP32 Thread Sensor with ESP32-C6 LP-core

Select from a list of sensors (BME/P-280, SHT4x)

## Usecase

- thermostat

## Features

- Heat on and off
- Fan on and off
- Heartbeat
- Deep-Sleep for low energy consumption
- LP-Core with main CPU wakeup for continuous messurements
- RTC-states

## Select a sensor (build-time)
Variant 1: `idf.py menuconfig` -> "Thermo Sensor" -> Sensortype and I2C-Address.

Variant 2: Overlay-file
```sh
    rm -rf sdkconfig build
    idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.sht4x" set-target esp32c6
    idf.py build flash monitor
```
(`sdkconfig.bme280` for BME280/BMP280.) Clean up sdkconfig and build/ on sensor change.

| Kconfig | Sensor | Adresse |
|---|---|---|
| CONFIG_SENSOR_BME280 | BME280 / BMP280 (e. g. GY-BME280) | 0x76 / 0x77 |
| CONFIG_SENSOR_SHT4X | SHT40 / SHT41 / SHT45 | 0x44 / 0x45 / 0x46 |

## Structure
- `main/ulp/sensor.h`            Interface: sensor_init / sensor_start / sensor_read
- `main/ulp/sensor_bme280.c`     Adapter BME280 (Register, integer-compensation)
- `main/ulp/sensor_sht4x.c`      Adapter SHT4x (command + CRC)
- `main/ulp/lp_main.c`           Threshold-paires, debounce, wake-up (sensor independend)
- `main/main.c`                  Main-CPU, Thread-preparation ([THREAD-1] to [THREAD-6])

How to add a new sensor: create a `sensor_xxx.c` file, add entry in `Kconfig.projbuild`,
handle in `main/CMakeLists.txt`.

## Pinouts (LP-I2C)
SDA = GPIO6, SCL = GPIO7, Pull-ups between 4,7k and 10k to 3V3 (GY-modules usually have them integrated).
BME280: CSB to 3V3, SDO to GND (0x76) or VCC (0x77).

## Checks
Log on Heartbeat: `sensor_id`: BME280 0x60, BMP280 0x58 (w/o humidity), SHT4x = Address,
0 = Sensor not available. `TEMP_OFFSET_C100` in main.c calibrates to a reference-thermometer.

Flash w/o `erase-flash`, otherwise the Thread-dataset in NVS will get lost.

```sh
idf.py -p /dev/ttyACM0 build flash monitor
```

## States

| flags | Heat | Fan |
|---|---|---|
| 0x00 | off | off |
| 0x01 | on | off |
| 0x02 | off | on |
| 0x04 | N/A | N/A |

## Troubleshooting

### No device found

The connection will be lost with the native USB-Port on every deep-sleep and reconnects. If there is no serial output then use the other UART-port.

### How to exit the serial monitor

```
CTRL+t
then x
```
