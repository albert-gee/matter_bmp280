# Matter-Based BMP280 Pressure and Temperature Sensor

This project implements a Matter-over-Thread pressure and temperature sensor for ESP32-H2 using a BMP280. Matter starts even when the sensor is absent; measurements remain null and the firmware retries the sensor with bounded I2C operations.

---

## Prerequisites

### Hardware

- ESP32-H2-DevKitM-1
- BMP280 Pressure and Temperature Sensor Module
- Breadboard and connecting wires
- 3.3V power supply

### Software

- **Ubuntu 24.04** or **24.10**
- **ESP-IDF 5.3.2**
- **ESP-Matter release/v1.4.2** (`4cedd4ad40821933d6cc237e80ab6604f2953fb0`)

---

## Getting Started

### ESP-IDF Setup

**Step 1. Install Prerequisites for ESP-IDF:**

```bash
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

**Step 2. Get ESP-IDF:**

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.3.2 --recursive https://github.com/espressif/esp-idf.git
```

**Step 3. Set up the Tools:**

```bash
cd ~/esp/esp-idf
./install.sh all
```

**Step 4. Configure Environment Variables:**

```bash
alias get_idf='. $HOME/esp/esp-idf/export.sh'
```

Add this line to `~/.bashrc` to persist the alias.

**Step 5. Refresh Configuration:**

```bash
source ~/.bashrc
```

Running `get_idf` will now set up or refresh the esp-idf environment in any terminal session.

### ESP-Matter Setup

**Step 1. Install Prerequisites for Matter:**

```bash
sudo apt-get install git gcc g++ pkg-config libssl-dev libdbus-1-dev libglib2.0-dev libavahi-client-dev ninja-build python3-venv python3-dev python3-pip unzip libgirepository1.0-dev libcairo2-dev libreadline-dev
```

**Step 2. Clone the esp-matter Repository:**

```bash
cd ~/esp/
git clone -b release/v1.4.2 --recursive https://github.com/espressif/esp-matter.git
```

**Step 3. Bootstrap esp-matter:**

```bash
cd esp-matter
./install.sh
```

**Step 4. Configure Environment Variables:**

```bash
alias get_matter='. $HOME/esp/esp-matter/export.sh'
```

Add this line to `~/.bashrc` to persist the alias.

**Step 5. Refresh Configuration:**

```bash
source ~/.bashrc
```

Running `get_matter` will now set up or refresh the esp-matter environment in any terminal session.

---

## Wiring

To connect the BMP280 sensor to the ESP32-H2, use the following default wiring. The GPIOs are configurable under `BMP280 configuration` in menuconfig.

### BMP280 Pinout and Connections

| BMP280 Pin | ESP32-H2 Pin |
|------------|------------|
| VCC        | 3.3V       |
| GND        | GND        |
| SDA        | GPIO10 (J3 pin 4) |
| SCL        | GPIO11 (J3 pin 5) |
| SDO        | GND        |

### Notes:

1. Ensure that the **VCC pin** of the BMP280 is connected to the 3.3V pin of the ESP32 to avoid overvoltage damage.
2. The **SDO pin** of the BMP280 should be connected to GND for an I2C address of `0x76`.
3. GPIO10 and GPIO11 are exposed on the ESP32-H2-DevKitM-1 J3 header and are not used by this application. GPIO8 is reserved for the board RGB LED, GPIO9 is BOOT, GPIO23/GPIO24 are UART, and GPIO26/GPIO27 are USB.

---

## Build and Flash

### Step 1. Clone the Repository

```bash
git clone git@github.com:albert-gee/matter_bmp280.git
cd matter_bmp280
```

### Step 2. Configure the Project

Use `menuconfig` to configure project-specific settings:

```bash
idf.py menuconfig
```

- Set `BMP280 I2C SDA GPIO` and `BMP280 I2C SCL GPIO` to match your wiring (ESP32-H2-DevKitM-1 defaults: GPIO10/GPIO11).
- Enable or disable features like CHIP Shell or OTA based on your requirements.

### Step 3. Set ESP32-H2 as the Target Device

```bash
idf.py set-target esp32h2
```

### Step 4. Build the Project

```bash
idf.py build
```

### Step 5. Determine Serial Port

Connect the ESP32 board to the computer and check under which serial port the board is visible. Serial ports typically follow the `/dev/tty` pattern.

### Step 6. Flash the Project to the Target

```bash
idf.py -p <PORT> flash
```

### Step 7. Launch the IDF Monitor Application

```bash
idf.py -p <PORT> monitor
```

Press `CTRL+]` to exit.

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

## References

- [Bosch BMP280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)
- [ESP32-H2-DevKitM-1 Header Reference](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32h2/esp32-h2-devkitm-1/user_guide.html#header-block)
- [Espressif Matter Documentation](https://docs.espressif.com/projects/esp-matter/en/latest/)
- [Matter Protocol Specification](https://csa-iot.org/all-solutions/matter/)
