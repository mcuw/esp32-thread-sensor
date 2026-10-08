# ESP32 Thread Sensor with ESP32-C6 LP-core

Select from a list of sensors (BME/P-280, SHT4x)

## Usecase

- thermostat

## Features

Damit sind folgende Funktionen belegt:
- Ventilator an und aus
- Heizung an und aus
- Heartbeat
- Deep-Sleep
- LP-Core-Wakeup
- die RTC-Zustände

## Sensor waehlen (Build-Zeit)
Variante 1: `idf.py menuconfig` -> "Thermo Sensor" -> Sensortyp und I2C-Adresse.

Variante 2: Overlay-Datei
    rm -rf sdkconfig build
    idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.sht4x" set-target esp32c6
    idf.py build flash monitor
(`sdkconfig.bme280` fuer BME280/BMP280.) Beim Wechsel sdkconfig und build loeschen.

| Kconfig | Sensor | Adresse |
|---|---|---|
| CONFIG_SENSOR_BME280 | BME280 / BMP280 (z. B. GY-BME280) | 0x76 / 0x77 |
| CONFIG_SENSOR_SHT4X | SHT40 / SHT41 / SHT45 | 0x44 / 0x45 / 0x46 |

## Aufbau
- `main/ulp/sensor.h`            Schnittstelle: sensor_init / sensor_start / sensor_read
- `main/ulp/sensor_bme280.c`     Adapter BME280 (Register, Ganzzahl-Kompensation)
- `main/ulp/sensor_sht4x.c`      Adapter SHT4x (Kommando + CRC)
- `main/ulp/lp_main.c`           Schwellenpaare, Entprellung, Wakeup (sensorunabhaengig)
- `main/main.c`                  Haupt-CPU, Thread-Vorbereitung ([THREAD-1] bis [THREAD-6])

Neuen Sensor ergaenzen: `sensor_xxx.c` schreiben, Eintrag in `Kconfig.projbuild`,
Zweig in `main/CMakeLists.txt`.

## Verdrahtung (LP-I2C)
SDA = GPIO6, SCL = GPIO7, Pull-ups 4,7k bis 10k nach 3V3 (GY-Module haben oft schon welche).
BME280: CSB an 3V3, SDO an GND (0x76) oder VCC (0x77).

## Pruefen
Log beim Heartbeat: `sensor_id`: BME280 0x60, BMP280 0x58 (ohne Feuchte), SHT4x = Adresse,
0 = Sensor nicht erreicht. `TEMP_OFFSET_C100` in main.c gegen Referenzthermometer einstellen.

Flashen ohne `erase-flash`, sonst geht das Thread-Dataset im NVS verloren.
Nicht getestet: API-Namen (LP-Core, LP-I2C, OpenThread) aus dem Gedaechtnis, gegen die
Beispiele deiner IDF-Version pruefen.

```sh
idf.py -p /dev/ttyACM0 build flash monitor
```

## States

Heizung und Ventilator aus
flags=0x00

Heizung an, Ventilator aus
flags=0x01

Der Ventilator ist an
flags=0x02

Sensorausfall
flags=0x04

## Troubleshooting

### No device found

Beim nativen USB-Port verschwindet die Verbindung bei jedem Einschlafen und meldet sich neu. Wenn dir Zeilen fehlen, nimm den UART-Port.

### How to exit serial monitor

```
CTRL+t then x
```