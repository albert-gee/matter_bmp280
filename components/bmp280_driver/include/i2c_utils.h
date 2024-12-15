#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include <stdint.h>
#include <driver/i2c_types.h>
#include <soc/gpio_num.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the I2C master bus and device.
 *
 * Configures the specified I2C port with given GPIO pins, clock speed, and device address.
 * Probes the device to verify connectivity and initializes the handles for further I2C operations.
 *
 * @param[in] i2c_port I2C port number.
 * @param[in] i2c_address I2C address of the device.
 * @param[in] sda_gpio GPIO number for the SDA line.
 * @param[in] scl_gpio GPIO number for the SCL line.
 * @param[in] clk_speed Clock speed in Hz.
 * @param[out] master_bus_handle Pointer to the I2C master bus handle.
 * @param[out] master_dev_handle Pointer to the I2C master device handle.
 *
 * @return
 *     - ESP_OK: Successfully initialized the I2C bus and device.
 *     - ESP_ERR_INVALID_ARG: Invalid parameters provided.
 *     - ESP_ERR_NO_MEM: Insufficient memory for initialization.
 *     - ESP_ERR_NOT_FOUND: No available I2C bus resources.
 */
esp_err_t i2c_init(i2c_port_t i2c_port, uint16_t i2c_address, gpio_num_t sda_gpio, gpio_num_t scl_gpio,
                   uint32_t clk_speed, i2c_master_bus_handle_t *master_bus_handle, i2c_master_dev_handle_t *master_dev_handle);

/**
 * @brief Cleans up the I2C master bus and device.
 *
 * Releases resources allocated during I2C initialization. This includes removing the device from the bus,
 * deinitializing the bus, and freeing associated handles.
 *
 * @param[in] master_bus_handle Handle to the I2C master bus.
 * @param[in] master_dev_handle Handle to the I2C master device.
 *
 * @return
 *     - ESP_OK: Successfully cleaned up I2C resources.
 *     - ESP_ERR_INVALID_ARG: Provided device handle is invalid.
 *     - ESP_FAIL: Failed to remove the device or release resources.
 */
esp_err_t i2c_cleanup(i2c_master_bus_handle_t master_bus_handle, i2c_master_dev_handle_t master_dev_handle);

/**
 * @brief Writes a single byte to a specific register on an I2C device.
 *
 * Sends the register address and data byte to the I2C device in a single transaction.
 *
 * @param master_device_handle Handle to the initialized I2C master device.
 * @param register_address Address of the target register on the I2C device.
 * @param data The byte to write to the specified register.
 * @param i2c_timeout_ms Timeout for the I2C transaction in milliseconds.
 *
 * @return
 *      - ESP_OK: Write operation successful.
 *      - ESP_ERR_INVALID_ARG: Invalid argument provided.
 *      - ESP_FAIL: I2C transaction failed.
 */
esp_err_t i2c_write_register(i2c_master_dev_handle_t master_device_handle, uint8_t register_address, uint8_t data, uint16_t i2c_timeout_ms);

/**
 * @brief Reads one or more consecutive registers from an I2C device.
 *
 * This function sends the start register address to the I2C device and reads
 * the specified number of bytes starting from that register in a single I2C transaction.
 *
 * @param master_device_handle Handle to the I2C master device.
 * @param start_register Address of the first register to read.
 * @param data Pointer to a buffer where the read data will be stored.
 * @param length Number of bytes to read from the device.
 * @param i2c_timeout_ms Timeout for the I2C transaction, in FreeRTOS ticks.
 * @return
 *      - ESP_OK: Read operation completed successfully.
 *      - ESP_ERR_INVALID_ARG: Invalid argument (e.g., NULL pointer or length = 0).
 *      - ESP_FAIL: I2C transaction failed (e.g., device did not acknowledge).
 */
esp_err_t i2c_read_registers(i2c_master_dev_handle_t master_device_handle, uint8_t start_register, uint8_t *data, size_t length,
    uint16_t i2c_timeout_ms);

/**
 * @brief Waits for all pending I2C transactions to complete on the specified bus.
 *
 * This function ensures that all I2C transactions are finished before proceeding.
 * Useful for operations like device resets, where ensuring the bus is idle is critical.
 *
 * @param master_bus_handle Handle to the I2C master bus.
 * @param timeout_ms Timeout in milliseconds for waiting. If transactions are not completed
 *                   within this time, the function returns a timeout error.
 * @return
 *      - ESP_OK: All transactions completed successfully.
 *      - ESP_ERR_TIMEOUT: Transactions did not complete within the specified timeout.
 *      - ESP_FAIL: General failure.
 */
esp_err_t i2c_wait_all_done(i2c_master_bus_handle_t master_bus_handle, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // I2C_UTILS_H
