# Matter BMP280 Temperature and Pressure Sensor

This ESP-IDF project exposes a BMP280 temperature and pressure sensor as Matter sensor endpoints. The BMP280 is read over I2C, then the firmware publishes temperature and pressure through ESP-Matter.

## Repository

- URL: `https://github.com/albert-gee/matter_bmp280.git`
- Folder: `matter_bmp280`

## Supported Target

- `esp32h2`

The checked-in `sdkconfig.defaults.esp32h2` configures the project for ESP32-H2 with Thread/OpenThread. Other targets are not claimed by this README.

## Dependencies

- ESP-IDF 5.3 at `~/esp/esp-idf-5.3`
- ESP-Matter 1.4 at `~/esp/esp-matter-1.4`

The local shell aliases are expected to be available from `~/.bashrc`:

```bash
get_idf
get_matter
```

## Hardware

- ESP32-H2 development board
- BMP280 temperature and pressure sensor module
- 3.3 V power

## Wiring

The I2C pins are currently hardcoded in `main/app_main.cpp`.

| BMP280 Pin | ESP32-H2 Pin |
|------------|--------------|
| VCC        | 3.3V         |
| GND        | GND          |
| SDA        | GPIO27       |
| SCL        | GPIO26       |
| SDO        | GND          |

With SDO tied to GND, the firmware uses BMP280 I2C address `0x76`.

## Build

```bash
get_idf
get_matter
idf.py set-target esp32h2
idf.py build
```

## Flash and Monitor

```bash
idf.py -p <PORT> flash monitor
```

Press `CTRL+]` to exit the monitor.

## Sensor Units

- BMP280 temperature compensation returns Celsius x 100, matching the Matter Temperature Measurement measured value unit.
- BMP280 pressure compensation returns Pascals. The firmware converts pressure to the Matter Pressure Measurement measured value unit, `kPa x 10`, by using `Pa / 100`.

## License

This project is licensed under the MIT License. See `LICENSE` for details.
