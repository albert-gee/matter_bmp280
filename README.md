# Matter-Based BMP280 Pressure and Temperature Sensor

This project implements a Matter-compatible pressure and temperature sensor using an ESP32 microcontroller and the BMP280 sensor. The firmware integrates the BMP280 driver with ESP-Matter to enable seamless communication over the Matter protocol.

---

## Prerequisites

### Hardware

- ESP32-DevKitM-1 or ESP32-H2 board
- BMP280 Pressure and Temperature Sensor Module
- Breadboard and connecting wires
- 3.3V power supply

### Software

- **Ubuntu 24.04** or **24.10**
- **ESP-IDF 5.2.3** (newer versions should be tested)
- **ESP-Matter 1.3** (newer versions should be tested)

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
git clone -b v5.2.3 --recursive https://github.com/espressif/esp-idf.git
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
git clone -b release/v1.3 --recursive https://github.com/espressif/esp-matter.git
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

To connect the BMP280 sensor to the ESP32, use the following wiring instructions:

### BMP280 Pinout and Connections

| BMP280 Pin | ESP32 Pin  |
|------------|------------|
| VCC        | 3.3V       |
| GND        | GND        |
| SDA        | GPIO21     |
| SCL        | GPIO22     |
| SDO        | GND        |

### Notes:

1. Ensure that the **VCC pin** of the BMP280 is connected to the 3.3V pin of the ESP32 to avoid overvoltage damage.
2. The **SDO pin** of the BMP280 should be connected to GND for an I2C address of `0x76`.

---

## Build and Flash

### Step 1. Clone the Repository

```bash
git clone https://github.com/<your-repo>/matter-bmp280.git
cd matter-bmp280
```

### Step 2. Configure the Project

Use `menuconfig` to configure project-specific settings:

```bash
idf.py menuconfig
```

- Set the I2C GPIO pins to match your wiring (SDA: GPIO21, SCL: GPIO22).
- Enable or disable features like CHIP Shell or OTA based on your requirements.

### Step 3. Set ESP32 as the Target Device

```bash
idf.py set-target esp32
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

- [BMP280 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf)
- [Espressif Matter Documentation](https://docs.espressif.com/projects/esp-matter/en/latest/)
- [Matter Protocol Specification](https://csa-iot.org/all-solutions/matter/)
