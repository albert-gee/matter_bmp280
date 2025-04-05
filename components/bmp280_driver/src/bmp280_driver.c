#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/projdefs.h>
#include <portmacro.h>

#include "bmp280_driver.h"
#include "i2c_utils.h"

static const char *TAG = "   ***bmp280_driver***   ";

esp_err_t bmp280_init(BMP280 *bmp280, const uint16_t i2c_address, const i2c_port_t i2c_port,
    const uint32_t i2c_timeout_ticks, const uint32_t i2c_clk_speed, const gpio_num_t sda_gpio,
    const gpio_num_t scl_gpio, const uint8_t oversampling_temp, const uint8_t oversampling_press, const uint8_t mode,
    const uint8_t standby_time, const uint8_t filter) {
    // Initialize the I2C bus for communication with the BMP280 sensor
    esp_err_t ret = i2c_init(
        i2c_port,                       // I2C port number
        i2c_address,                    // I2C address of the BMP280 sensor
        sda_gpio,                       // GPIO pin for the I2C SDA line
        scl_gpio,                       // GPIO pin for the I2C SCL line
        i2c_clk_speed,                  // I2C clock speed in Hz
        &bmp280->i2c_master_bus_handle, // Pointer to the I2C master bus handle
        &bmp280->i2c_master_dev_handle  // Pointer to the I2C master device handle
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return ret; // Exit initialization if I2C initialization fails
    }

    // Set the I2C operation timeout in FreeRTOS ticks
    bmp280->i2c_timeout_ticks = pdMS_TO_TICKS(i2c_timeout_ticks);

    // Verify the sensor
    ret = bmp280_verify(bmp280);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to verify BMP280 sensor: %s", esp_err_to_name(ret));
        return ret; // Exit initialization if sensor verification fails
    }

    // Reset the sensor to start in a known state
    ret = bmp280_reset(bmp280);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset BMP280 sensor: %s", esp_err_to_name(ret));
        return ret; // Exit initialization if sensor reset fails
    }

    // Read and store the calibration data
    ret = bmp280_read_calibration_data(
        bmp280->i2c_master_dev_handle,
        &bmp280->calibration_data,
        bmp280->i2c_timeout_ticks
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280 calibration data: %s", esp_err_to_name(ret));
        return ret; // Exit initialization if calibration data cannot be read
    }

    // Validate oversampling and mode settings
    if (oversampling_temp > BMP280_OSRS_T_X16 || oversampling_press > BMP280_OSRS_P_X16 || mode > BMP280_MODE_NORMAL) {
        ESP_LOGE(TAG, "Invalid oversampling or mode configuration: Temp=%d, Press=%d, Mode=%d",
                 oversampling_temp, oversampling_press, mode);
        return ESP_ERR_INVALID_ARG; // Return error for invalid settings
    }

    // Set oversampling and power mode settings
    ret = bmp280_set_ctrl_meas(
        bmp280,
        oversampling_temp,  // Oversampling for temperature
        oversampling_press, // Oversampling for pressure
        mode                // Power mode
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BMP280 control measurement settings: %s", esp_err_to_name(ret));
        return ret; // Exit initialization if control measurement settings fail
    }

    // Configure standby time and filter settings
    ret = bmp280_set_config(bmp280, standby_time, filter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BMP280 configuration settings: %s", esp_err_to_name(ret));
        return ret; // Exit initialization if configuration settings fail
    }

    ESP_LOGI(TAG, "BMP280 initialization complete with Temp_OSR=%d, Press_OSR=%d, Mode=%d",
             oversampling_temp, oversampling_press, mode);
    return ESP_OK; // Return success if all initialization steps are completed
}

esp_err_t bmp280_verify(const BMP280 *bmp280) {
    // Read the chip ID of the BMP280 device
    uint8_t chip_id = 0;
    esp_err_t result = i2c_read_registers(bmp280->i2c_master_dev_handle, BMP280_REG_ID, &chip_id, 1,
                                          bmp280->i2c_timeout_ticks);

    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Unable to read ID register");
        return result; // Early return to avoid additional checks
    }

    if (chip_id != BMP280_ID_VALUE) {
        ESP_LOGE(TAG, "BMP280 chip ID mismatch: 0x%02X", chip_id);
        result = ESP_ERR_INVALID_RESPONSE;
    } else {
        ESP_LOGI(TAG, "BMP280 chip ID verified: 0x%02X", chip_id);
    }

    return result;
}

esp_err_t bmp280_reset(const BMP280 *bmp280) {
    ESP_LOGI(TAG, "Resetting BMP280 sensor");

    // Write the reset command
    esp_err_t ret = i2c_write_register(bmp280->i2c_master_dev_handle, BMP280_REG_RESET, BMP280_RESET_VALUE,
                                       bmp280->i2c_timeout_ticks);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write reset command to BMP280: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for the I2C bus to finish all transactions
    ret = i2c_wait_all_done(bmp280->i2c_master_bus_handle, pdTICKS_TO_MS(bmp280->i2c_timeout_ticks));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wait for BMP280 reset completion: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for initialization (im_update bit to clear)
    uint8_t status;
    do {
        ret = i2c_read_registers(bmp280->i2c_master_dev_handle, BMP280_REG_STATUS, &status, 1, bmp280->i2c_timeout_ticks);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read status register during reset: %s", esp_err_to_name(ret));
            return ret;
        }
    } while (status & BMP280_STATUS_IM_UPDATE); // Wait while im_update (Bit 0) is 1

    ESP_LOGI(TAG, "BMP280 reset complete and ready");
    return ESP_OK;
}

esp_err_t bmp280_set_ctrl_meas(BMP280 *bmp280, const int oversampling_temp, const int oversampling_press,
                               const int mode) {
    ESP_LOGI(TAG, "Setting BMP280 control measurement register");

    // Store the oversampling values in the BMP280 structure
    bmp280->oversampling_temp = oversampling_temp;
    bmp280->oversampling_press = oversampling_press;

    // Construct the CTRL_MEAS register
    const uint8_t ctrl_meas = (oversampling_temp << BMP280_OSRS_T_POS) | // Shift temperature oversampling
                              (oversampling_press << BMP280_OSRS_P_POS) | // Shift pressure oversampling
                              mode; // Set normal mode

    // Write the CTRL_MEAS register to the sensor
    return i2c_write_register(
        bmp280->i2c_master_dev_handle,
        BMP280_REG_CTRL_MEAS,
        ctrl_meas,
        bmp280->i2c_timeout_ticks);
}

esp_err_t bmp280_set_config(BMP280 *bmp280, const uint8_t standby_time, const uint8_t filter) {
    ESP_LOGI(TAG, "Setting BMP280 configuration register");

    // Validate input parameters
    if (standby_time > 7 || filter > 7) {
        ESP_LOGE(TAG, "Invalid standby_time or filter value: standby_time=%d, filter=%d", standby_time, filter);
        return ESP_ERR_INVALID_ARG;
    }

    // Read the current CONFIG register value
    uint8_t config;
    esp_err_t ret = i2c_read_registers(
        bmp280->i2c_master_dev_handle,
        BMP280_REG_CONFIG,
        &config,
        1,
        bmp280->i2c_timeout_ticks
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CONFIG register: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update t_sb (Bits 7:5) and filter (Bits 4:2)
    config = (config & 0x1) | (standby_time << 5) | (filter << 2);

    // Write the updated CONFIG register value
    ret = i2c_write_register(
        bmp280->i2c_master_dev_handle,
        BMP280_REG_CONFIG,
        config,
        bmp280->i2c_timeout_ticks
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CONFIG register: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t bmp280_read_temperature_pressure(const BMP280 *bmp280, int32_t *temperature, int32_t *pressure) {

    // Check if the sensor has completed its measurement
    uint8_t status;
    esp_err_t ret;
    do {
        ret = i2c_read_registers(bmp280->i2c_master_dev_handle, BMP280_REG_STATUS, &status, 1, bmp280->i2c_timeout_ticks);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read status register: %s", esp_err_to_name(ret));
            return ret;
        }
    } while (status & BMP280_STATUS_MEASURING); // Wait while measuring (Bit 3) is 1

    // Read raw temperature and pressure data from the BMP280
    int32_t raw_temp, raw_press;
    ret = bmp280_read_raw_data(bmp280, &raw_temp, &raw_press);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read raw data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Use calibration data to calculate compensated temperature and pressure
    *temperature = bmp280_compensate_temperature(raw_temp, &bmp280->calibration_data);
    *pressure = bmp280_compensate_pressure(raw_press, &bmp280->calibration_data);

    return ESP_OK;
}

esp_err_t bmp280_read_raw_data(const BMP280 *bmp280, int32_t *raw_temp, int32_t *raw_press) {

    // Buffer to store the 6 bytes of data read from the BMP280
    // - 3 bytes for pressure (MSB, LSB, XLSB)
    // - 3 bytes for temperature (MSB, LSB, XLSB)
    uint8_t data[6];

    // Perform a burst read starting from the PRESS_MSB register
    // This reads all the bytes for both pressure and temperature in a single I2C transaction
    const esp_err_t ret = i2c_read_registers(
        bmp280->i2c_master_dev_handle, // I2C device handle for communication
        BMP280_REG_PRESS_MSB, // Starting register for burst read
        data, // Buffer to hold the read data
        sizeof(data), // Number of bytes to read (6 for BMP280)
        bmp280->i2c_timeout_ticks // Timeout duration for the I2C operation
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280 raw data: %d", ret);
        return ret;
    }

    *raw_press = (int32_t) ((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));

    *raw_temp = (int32_t) ((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));

    // Return ESP_OK to indicate the operation was successful
    return ESP_OK;
}

esp_err_t bmp280_wait_for_measurement(const BMP280 *bmp280) {
    uint8_t status;

    ESP_LOGI(TAG, "Checking if BMP280 is currently measuring...");
    do {
        const esp_err_t ret = i2c_read_registers(
            bmp280->i2c_master_dev_handle,
            BMP280_REG_STATUS,
            &status,
            1,
            bmp280->i2c_timeout_ticks);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read status register: %s", esp_err_to_name(ret));
            return ret;
        }
    } while (status & BMP280_STATUS_MEASURING); // Wait while Bit 3 is 1

    return ESP_OK;
}
