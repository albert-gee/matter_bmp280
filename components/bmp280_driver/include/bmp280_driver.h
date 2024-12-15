#ifndef BMP280_DRIVER_H
#define BMP280_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <bmp280_calibration.h>
#include <soc/gpio_num.h>

// ======================= General Constants =======================

// Temperature range
#define BMP280_TEMP_MIN_DEG_CELSIUS -40.0
#define BMP280_TEMP_MAX_DEG_CELSIUS 85.0

// Pressure range
#define BMP280_PRESS_MIN_HPA 300.0
#define BMP280_PRESS_MAX_HPA 1100.0

// Delay for reset
#define BMP280_RESET_DELAY_TICKS 1

// ======================= I2C Settings =======================

// BMP280 I2C Address
#define BMP280_I2C_ADDRESS_76 0x76      // SDO pin connected to GND
#define BMP280_I2C_ADDRESS_77 0x77      // SDO pin connected to VDDIO

// I2C Clock Speed constants supported by BMP280
#define BMP280_I2C_CLK_SPEED_STD 100000         // Standard Mode (100 kHz)
#define BMP280_I2C_CLK_SPEED_FAST 400000        // Fast Mode (400 kHz)
#define BMP280_I2C_CLK_SPEED_FAST_PLUS 1000000  // Fast Mode Plus (1 MHz)
#define BMP280_I2C_CLK_SPEED_HIGH 3400000       // High-Speed Mode (3.4 MHz)

// ======================= Register Map =======================

// Register Addresses
#define BMP280_REG_ID 0xD0              // Chip ID register (read-only)
#define BMP280_REG_RESET 0xE0           // Soft reset register
#define BMP280_REG_STATUS 0xF3          // Status register
#define BMP280_REG_CTRL_MEAS 0xF4       // Control and measurement register
#define BMP280_REG_CONFIG 0xF5          // Configuration register
#define BMP280_REG_PRESS_MSB 0xF7       // Pressure MSB
#define BMP280_REG_PRESS_LSB 0xF8       // Pressure LSB
#define BMP280_REG_PRESS_XLSB 0xF9      // Pressure XLSB
#define BMP280_REG_TEMP_MSB 0xFA        // Temperature MSB
#define BMP280_REG_TEMP_LSB 0xFB        // Temperature LSB
#define BMP280_REG_TEMP_XLSB 0xFC       // Temperature XLSB

// ======================= Specific Register Values =======================

#define BMP280_ID_VALUE 0x58            // Expected ID value for BMP280
#define BMP280_RESET_VALUE 0xB6         // Write this to reset the device

// ======================= Status Register Bits =======================

#define BMP280_STATUS_MEASURING 0x08 // Measurement running
#define BMP280_STATUS_IM_UPDATE 0x01 // NVM data being copied

// ======================= Control Measurement Register =======================

// Register Layout
// Bits [7:5]: osrs_t (Temperature oversampling setting)
// Bits [4:2]: osrs_p (Pressure oversampling setting)
// Bits [1:0]: mode (Sensor operating mode)
#define BMP280_OSRS_T_POS 5          // Starting bit position for temperature oversampling (bits 7:5)
#define BMP280_OSRS_P_POS 2          // Starting bit position for pressure oversampling (bits 4:2)
#define BMP280_MODE_POS 0            // Starting bit position for operating mode (bits 1:0)

// Operating Mode Settings
#define BMP280_MODE_SLEEP 0x00       // Sleep mode: No measurements, low power
#define BMP280_MODE_FORCED 0x01      // Forced mode: Single measurement, then sleep
#define BMP280_MODE_NORMAL 0x03      // Normal mode: Continuous measurements

// ======================= Oversampling Settings =======================

// Temperature Oversampling
#define BMP280_OSRS_T_SKIP  0        // Temperature measurement skipped (output set to 0x80000)
#define BMP280_OSRS_T_X1    1        // x1 oversampling: Low resolution, fast response
#define BMP280_OSRS_T_X2    2        // x2 oversampling: Medium resolution
#define BMP280_OSRS_T_X4    3        // x4 oversampling: High resolution
#define BMP280_OSRS_T_X8    4        // x8 oversampling: Very high resolution
#define BMP280_OSRS_T_X16   5        // x16 oversampling: Maximum resolution, slow response

// Pressure Oversampling
#define BMP280_OSRS_P_SKIP  0        // Pressure measurement skipped (output set to 0x80000)
#define BMP280_OSRS_P_X1    1        // x1 oversampling: Low resolution, fast response
#define BMP280_OSRS_P_X2    2        // x2 oversampling: Medium resolution
#define BMP280_OSRS_P_X4    3        // x4 oversampling: High resolution
#define BMP280_OSRS_P_X8    4        // x8 oversampling: Very high resolution
#define BMP280_OSRS_P_X16   5        // x16 oversampling: Maximum resolution, slow response

// ======================= Configuration Register =======================

// Register Layout
// Bits [7:5]: t_sb (Standby time)
// Bits [4:2]: filter (IIR filter setting)
// Bit [0]: spi3w_en (SPI 3-wire enable)
#define BMP280_T_SB_POS 5            // Standby time position
#define BMP280_FILTER_POS 2          // Filter position
#define BMP280_SPI3W_EN_POS 0        // SPI 3-wire enable position

// Standby Time
#define BMP280_STANDBY_TIME_0_5_MS  0  // 0.5 ms
#define BMP280_STANDBY_TIME_62_5_MS 1  // 62.5 ms
#define BMP280_STANDBY_TIME_125_MS  2  // 125 ms
#define BMP280_STANDBY_TIME_250_MS  3  // 250 ms
#define BMP280_STANDBY_TIME_500_MS  4  // 500 ms
#define BMP280_STANDBY_TIME_1000_MS 5  // 1000 ms
#define BMP280_STANDBY_TIME_2000_MS 6  // 2000 ms
#define BMP280_STANDBY_TIME_4000_MS 7  // 4000 ms

// Filter Coefficients
#define BMP280_FILTER_OFF       0  // Filter off
#define BMP280_FILTER_COEFF_2   1  // Coefficient 2
#define BMP280_FILTER_COEFF_4   2  // Coefficient 4
#define BMP280_FILTER_COEFF_8   3  // Coefficient 8
#define BMP280_FILTER_COEFF_16  4  // Coefficient 16

/**
 * @brief Structure to hold BMP280 device context.
 */
typedef struct {
    BMP280_CalibrationData calibration_data; /**< Calibration data for BMP280. */
    uint8_t oversampling_temp; /**< Oversampling setting for temperature (0-5). */
    uint8_t oversampling_press; /**< Oversampling setting for pressure (0-5). */
    i2c_master_bus_handle_t i2c_master_bus_handle; /**< I2C bus handle. */
    i2c_master_dev_handle_t i2c_master_dev_handle; /**< I2C master device handle. */
    uint32_t i2c_timeout_ticks; /**< Timeout for I2C operations in ticks. */
} BMP280;

/**
 * @brief Configures the I2C communication and initializes the BMP280 sensor.
 *
 * The initialization process includes:
 * - Initializing the I2C bus and verifying communication with the sensor.
 * - Resetting the sensor to ensure a known starting state.
 * - Reading and storing calibration data for temperature and pressure compensation.
 * - Configuring the control measurement register (CTRL_MEAS) for oversampling and mode.
 * - Configuring the configuration register (CONFIG) for standby time and filter settings.
 *
 * @param bmp280 Pointer to a BMP280 structure to hold sensor state and configuration.
 * @param i2c_address I2C address of the BMP280 sensor (0x76 or 0x77 based on SDO pin configuration).
 * @param i2c_port I2C port to use (e.g., I2C_NUM_0 or I2C_NUM_1).
 * @param i2c_timeout_ticks Timeout for I2C operations in FreeRTOS ticks.
 * @param i2c_clk_speed I2C clock speed in Hz (e.g., 100000 for 100 kHz).
 * @param sda_gpio GPIO pin for the I2C SDA line.
 * @param scl_gpio GPIO pin for the I2C SCL line.
 * @param oversampling_temp Oversampling setting for temperature (e.g., BMP280_OSRS_T_X4).
 * @param oversampling_press Oversampling setting for pressure (e.g., BMP280_OSRS_P_X8).
 * @param mode Operating mode for the sensor (e.g., BMP280_MODE_NORMAL).
 * @param standby_time Standby time (t_sb) in CONFIG register, defining delay between measurements in Normal Mode.
 * @param filter Filter coefficient setting in CONFIG register for pressure noise reduction.
 *
 * @return ESP_OK on successful initialization, or an appropriate error code on failure.
 */
esp_err_t bmp280_init(BMP280 *bmp280, uint16_t i2c_address, i2c_port_t i2c_port, uint32_t i2c_timeout_ticks,
    uint32_t i2c_clk_speed, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint8_t oversampling_temp,
    uint8_t oversampling_press, uint8_t mode, uint8_t standby_time, uint8_t filter);

/**
 * @brief Verifies the BMP280 sensor by reading and checking its chip ID.
 *
 * This function reads the BMP280_REG_ID register (0xD0) of the BMP280 sensor to retrieve the
 * chip identification number and verifies that it matches the expected value defined
 * as BMP280_ID_VALUE. This ensures:
 *  - The connected device is a BMP280 sensor.
 *  - The I2C communication is functioning correctly.
 *
 * If the chip ID does not match the expected value, an error is logged, and
 * ESP_ERR_INVALID_RESPONSE is returned.
 *
 * @param bmp280 Pointer to the BMP280 device structure.
 * @return ESP_OK on successful verification, or an appropriate error code.
 */
esp_err_t bmp280_verify(const BMP280 *bmp280);

/**
 * @brief Resets the BMP280 device.
 *
 * @param bmp280 Pointer to BMP280 device context.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bmp280_reset(const BMP280 *bmp280);

/**
 * @brief Sets the oversampling settings for temperature and pressure.
 *
 * @param bmp280 Pointer to BMP280 device context.
 * @param oversampling_temp Oversampling setting for temperature.
 * @param oversampling_press Oversampling setting for pressure.
 * @param mode Power mode to set.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bmp280_set_ctrl_meas(BMP280 *bmp280, int oversampling_temp, int oversampling_press, int mode);

/**
 * @brief Sets the Configuration register (0xF5).
 *
 * @param bmp280 Pointer to BMP280 device context.
 * @param standby_time Standby time setting.
 * @param filter Filter setting.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bmp280_set_config(BMP280 *bmp280, uint8_t standby_time, uint8_t filter);

/**
 * @brief Reads the temperature and pressure from the BMP280 device.
 *
 * @param bmp280 Pointer to BMP280 device context.
 * @param temperature Pointer to store the read temperature.
 * @param pressure Pointer to store the read pressure.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t bmp280_read_temperature_pressure(const BMP280 *bmp280, int32_t *temperature, int32_t *pressure);

/**
 * @brief Reads the raw temperature and pressure data from the BMP280 sensor.
 *
 * This function performs a burst read of 6 bytes starting from the pressure MSB register (0xF7).
 * It retrieves the 20-bit raw pressure and temperature data stored in consecutive registers:
 * - Pressure data: Registers 0xF7 (MSB), 0xF8 (LSB), and 0xF9 (XLSB, upper 4 bits only).
 * - Temperature data: Registers 0xFA (MSB), 0xFB (LSB), and 0xFC (XLSB, upper 4 bits only).
 *
 * The data is extracted from the registers and returned as 32-bit integers in raw_temp
 * and raw_press. These raw values need to be compensated using calibration data before
 * being converted to physical units (e.g., °C and Pa).
 *
 * @param bmp280 Pointer to the BMP280 device context containing I2C configuration.
 * @param raw_temp Pointer to a variable where the raw temperature data will be stored.
 * @param raw_press Pointer to a variable where the raw pressure data will be stored.
 * @return ESP_OK on successful data retrieval, or an error code on failure (e.g., I2C communication issues).
 */
esp_err_t bmp280_read_raw_data(const BMP280 *bmp280, int32_t *raw_temp, int32_t *raw_press);

/**
 * @brief Waits for the BMP280 sensor to complete its ongoing measurement.
 *
 * This function checks the "measuring" bit (Bit 3) of the status register (0xF3) to determine
 * whether the sensor is currently performing a measurement. The "measuring" bit is automatically set to `1`
 * when a measurement is in progress and is cleared to `0` when the measurement result is transferred to the
 * data registers (e.g., pressure and temperature registers starting at 0xF7).
 *
 * Use this function before attempting to read the measurement results to avoid accessing invalid or partial data.
 * This function is particularly useful in **forced mode**, where the sensor performs a single measurement
 * and then enters sleep mode. For **continuous mode** (normal mode), this function is typically not needed since the
 * sensor updates the data registers at regular intervals automatically.
 *
 * @param bmp280 A pointer to the BMP280 structure.
 *
 * @return
 * - `ESP_OK`: Measurement is complete, and the sensor is ready for data access.
 * - `ESP_ERR_TIMEOUT`: If reading the status register fails due to I2C communication timeout.
 * - Other `esp_err_t` error codes indicating I2C communication failures.
 */
esp_err_t bmp280_wait_for_measurement(const BMP280 *bmp280);

#ifdef __cplusplus
}
#endif

#endif // BMP280_DRIVER_H
