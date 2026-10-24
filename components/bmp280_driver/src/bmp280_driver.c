#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/projdefs.h>
#include <freertos/task.h>
#include <portmacro.h>
#include <esp_timer.h>
#include <stdbool.h>
#include <string.h>

#include "bmp280_driver.h"
#include "i2c_utils.h"

static const char *TAG = "   ***bmp280_driver***   ";
#define BMP280_CTRL_MEAS_SETTINGS_MASK 0xFCU
#define BMP280_CTRL_MEAS_MODE_MASK 0x03U
static esp_err_t bmp280_verify_operating_configuration(const BMP280 *bmp280);

esp_err_t bmp280_init(BMP280 *bmp280, const uint16_t i2c_address, const i2c_port_t i2c_port,
    const uint32_t i2c_timeout_ms, const uint32_t i2c_clk_speed, const gpio_num_t sda_gpio,
    const gpio_num_t scl_gpio, const uint8_t oversampling_temp, const uint8_t oversampling_press, const uint8_t mode,
    const uint8_t standby_time, const uint8_t filter) {
    if (bmp280 == NULL || i2c_timeout_ms == 0U ||
        oversampling_temp > BMP280_OSRS_T_X16 || oversampling_press > BMP280_OSRS_P_X16 ||
        (mode != BMP280_MODE_SLEEP && mode != BMP280_MODE_FORCED && mode != BMP280_MODE_NORMAL) ||
        standby_time > BMP280_STANDBY_TIME_4000_MS ||
        filter > BMP280_FILTER_COEFF_16) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bmp280->i2c_master_bus_handle != NULL || bmp280->i2c_master_dev_handle != NULL) {
        const esp_err_t cleanup_ret = bmp280_deinit(bmp280);
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "Cannot reinitialize while I2C cleanup is incomplete: %s",
                     esp_err_to_name(cleanup_ret));
            return cleanup_ret;
        }
    }

    memset(bmp280, 0, sizeof(*bmp280));
    bmp280->i2c_timeout_ms = i2c_timeout_ms;

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

    // Verify the sensor
    ret = bmp280_verify(bmp280);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to verify BMP280 sensor: %s", esp_err_to_name(ret));
        goto fail;
    }

    // Reset the sensor to start in a known state
    ret = bmp280_reset(bmp280);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset BMP280 sensor: %s", esp_err_to_name(ret));
        goto fail;
    }

    // Read and store the calibration data
    ret = bmp280_read_calibration_data(
        bmp280->i2c_master_dev_handle,
        &bmp280->calibration_data,
        bmp280->i2c_timeout_ms
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BMP280 calibration data: %s", esp_err_to_name(ret));
        goto fail;
    }

    // Program CONFIG while sleeping; normal-mode CONFIG writes may be ignored.
    ret = bmp280_set_ctrl_meas(
        bmp280,
        oversampling_temp,
        oversampling_press,
        BMP280_MODE_SLEEP
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BMP280 control measurement settings: %s", esp_err_to_name(ret));
        goto fail;
    }

    // Configure standby time and filter settings
    ret = bmp280_set_config(bmp280, standby_time, filter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set BMP280 configuration settings: %s", esp_err_to_name(ret));
        goto fail;
    }

    // Enter the requested operating mode only after CONFIG has been accepted.
    // Forced mode is armed but left sleeping here; each read triggers exactly
    // one fresh conversion.
    const uint8_t initial_mode = mode == BMP280_MODE_FORCED ? BMP280_MODE_SLEEP : mode;
    ret = bmp280_set_ctrl_meas(bmp280, oversampling_temp, oversampling_press, initial_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable BMP280 operating mode: %s", esp_err_to_name(ret));
        goto fail;
    }
    if (mode == BMP280_MODE_FORCED) {
        bmp280->mode = BMP280_MODE_FORCED;
        bmp280->expected_ctrl_meas =
            (uint8_t)((oversampling_temp << BMP280_OSRS_T_POS) |
                      (oversampling_press << BMP280_OSRS_P_POS) |
                      BMP280_MODE_FORCED);
    }

    ret = bmp280_verify_operating_configuration(bmp280);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 configuration verification failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    ESP_LOGI(TAG, "BMP280 initialization complete with Temp_OSR=%d, Press_OSR=%d, Mode=%d",
             oversampling_temp, oversampling_press, mode);
    return ESP_OK;

fail:
    {
        const esp_err_t cleanup_ret = bmp280_deinit(bmp280);
        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(TAG, "BMP280 initialization cleanup failed: %s",
                     esp_err_to_name(cleanup_ret));
            return cleanup_ret;
        }
    }
    return ret;
}

esp_err_t bmp280_deinit(BMP280 *bmp280) {
    if (bmp280 == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_cleanup(&bmp280->i2c_master_bus_handle,
                       &bmp280->i2c_master_dev_handle);
}

esp_err_t bmp280_verify(const BMP280 *bmp280) {
    // Read the chip ID of the BMP280 device
    uint8_t chip_id = 0;
    esp_err_t result = i2c_read_registers(bmp280->i2c_master_dev_handle, BMP280_REG_ID, &chip_id, 1,
                                          bmp280->i2c_timeout_ms);

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
                                       bmp280->i2c_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write reset command to BMP280: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for the I2C bus to finish all transactions
    ret = i2c_wait_all_done(bmp280->i2c_master_bus_handle, bmp280->i2c_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wait for BMP280 reset completion: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for initialization (im_update bit to clear)
    uint8_t status;
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)bmp280->i2c_timeout_ms * 1000;
    do {
        ret = i2c_read_registers(bmp280->i2c_master_dev_handle, BMP280_REG_STATUS, &status, 1,
                                 bmp280->i2c_timeout_ms);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read status register during reset: %s", esp_err_to_name(ret));
            return ret;
        }
        if ((status & BMP280_STATUS_IM_UPDATE) != 0U) {
            if (esp_timer_get_time() >= deadline_us) {
                ESP_LOGE(TAG, "Timed out waiting for BMP280 NVM copy");
                return ESP_ERR_TIMEOUT;
            }
            vTaskDelay(1);
        }
    } while (status & BMP280_STATUS_IM_UPDATE); // Wait while im_update (Bit 0) is 1

    ESP_LOGI(TAG, "BMP280 reset complete and ready");
    return ESP_OK;
}

esp_err_t bmp280_set_ctrl_meas(BMP280 *bmp280, const int oversampling_temp, const int oversampling_press,
                               const int mode) {
    ESP_LOGI(TAG, "Setting BMP280 control measurement register");

    if (bmp280 == NULL || oversampling_temp < BMP280_OSRS_T_SKIP ||
        oversampling_temp > BMP280_OSRS_T_X16 || oversampling_press < BMP280_OSRS_P_SKIP ||
        oversampling_press > BMP280_OSRS_P_X16 ||
        (mode != BMP280_MODE_SLEEP && mode != BMP280_MODE_FORCED && mode != BMP280_MODE_NORMAL)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Store the oversampling values in the BMP280 structure
    bmp280->oversampling_temp = oversampling_temp;
    bmp280->oversampling_press = oversampling_press;

    // Construct the CTRL_MEAS register
    const uint8_t ctrl_meas = (oversampling_temp << BMP280_OSRS_T_POS) | // Shift temperature oversampling
                              (oversampling_press << BMP280_OSRS_P_POS) | // Shift pressure oversampling
                              mode;

    // Write the CTRL_MEAS register to the sensor
    const esp_err_t ret = i2c_write_register(
        bmp280->i2c_master_dev_handle,
        BMP280_REG_CTRL_MEAS,
        ctrl_meas,
        bmp280->i2c_timeout_ms);
    if (ret == ESP_OK) {
        bmp280->expected_ctrl_meas = ctrl_meas;
        bmp280->mode = (uint8_t)mode;
    }
    return ret;
}

esp_err_t bmp280_set_config(BMP280 *bmp280, const uint8_t standby_time, const uint8_t filter) {
    ESP_LOGI(TAG, "Setting BMP280 configuration register");

    // Validate input parameters
    if (standby_time > BMP280_STANDBY_TIME_4000_MS || filter > BMP280_FILTER_COEFF_16) {
        ESP_LOGE(TAG, "Invalid standby_time or filter value: standby_time=%d, filter=%d", standby_time, filter);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t ctrl_meas = 0;
    esp_err_t ret = i2c_read_registers(
        bmp280->i2c_master_dev_handle, BMP280_REG_CTRL_MEAS, &ctrl_meas, 1,
        bmp280->i2c_timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }
    if ((ctrl_meas & 0x03U) != BMP280_MODE_SLEEP) {
        ESP_LOGE(TAG, "CONFIG can only be written while BMP280 is sleeping");
        return ESP_ERR_INVALID_STATE;
    }

    // Read the current CONFIG register value
    uint8_t config;
    ret = i2c_read_registers(
        bmp280->i2c_master_dev_handle,
        BMP280_REG_CONFIG,
        &config,
        1,
        bmp280->i2c_timeout_ms
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
        bmp280->i2c_timeout_ms
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CONFIG register: %s", esp_err_to_name(ret));
    } else {
        bmp280->expected_config = config;
    }

    return ret;
}

static esp_err_t bmp280_verify_operating_configuration(const BMP280 *bmp280) {
    uint8_t chip_id = 0;
    uint8_t ctrl_meas = 0;
    uint8_t config = 0;

    esp_err_t ret = i2c_read_registers(bmp280->i2c_master_dev_handle, BMP280_REG_ID,
                                       &chip_id, 1, bmp280->i2c_timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_read_registers(bmp280->i2c_master_dev_handle, BMP280_REG_CTRL_MEAS,
                             &ctrl_meas, 1, bmp280->i2c_timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_read_registers(bmp280->i2c_master_dev_handle, BMP280_REG_CONFIG,
                             &config, 1, bmp280->i2c_timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }

    const uint8_t actual_mode = ctrl_meas & BMP280_CTRL_MEAS_MODE_MASK;
    const bool persistent_settings_match =
        (ctrl_meas & BMP280_CTRL_MEAS_SETTINGS_MASK) ==
        (bmp280->expected_ctrl_meas & BMP280_CTRL_MEAS_SETTINGS_MASK);
    const bool mode_matches = bmp280->mode == BMP280_MODE_FORCED
                                  ? (actual_mode == BMP280_MODE_SLEEP ||
                                     actual_mode == BMP280_MODE_FORCED)
                                  : actual_mode == bmp280->mode;

    if (chip_id != BMP280_ID_VALUE || !persistent_settings_match || !mode_matches ||
        config != bmp280->expected_config) {
        ESP_LOGW(TAG,
                 "BMP280 configuration lost: id=0x%02x ctrl=0x%02x/0x%02x config=0x%02x/0x%02x",
                 chip_id, ctrl_meas, bmp280->expected_ctrl_meas,
                 config, bmp280->expected_config);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t bmp280_read_temperature_pressure(const BMP280 *bmp280, int32_t *temperature, int32_t *pressure) {
    if (bmp280 == NULL || temperature == NULL || pressure == NULL ||
        bmp280->i2c_master_dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = bmp280_verify_operating_configuration(bmp280);
    if (ret != ESP_OK) {
        return ret;
    }

    if (bmp280->mode == BMP280_MODE_SLEEP) {
        ESP_LOGW(TAG, "Cannot read a measurement while BMP280 is in sleep mode");
        return ESP_ERR_INVALID_STATE;
    }
    if (bmp280->mode == BMP280_MODE_FORCED) {
        /* A forced conversion automatically returns to sleep. Trigger a fresh
         * conversion for every read, then wait at least one RTOS tick so either
         * the conversion has completed or the measuring bit is observable. */
        ret = i2c_write_register(bmp280->i2c_master_dev_handle,
                                 BMP280_REG_CTRL_MEAS,
                                 bmp280->expected_ctrl_meas,
                                 bmp280->i2c_timeout_ms);
        if (ret != ESP_OK) {
            return ret;
        }
        vTaskDelay(1);
    }

    ret = bmp280_wait_for_measurement(bmp280);
    if (ret != ESP_OK) {
        return ret;
    }

    // Read raw temperature and pressure data from the BMP280
    int32_t raw_temp, raw_press;
    ret = bmp280_read_raw_data(bmp280, &raw_temp, &raw_press);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read raw data: %s", esp_err_to_name(ret));
        return ret;
    }
    if (raw_temp == 0x80000 || raw_press == 0x80000) {
        ESP_LOGW(TAG, "BMP280 returned reset/skipped raw measurement values");
        return ESP_ERR_INVALID_RESPONSE;
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
        bmp280->i2c_timeout_ms // Timeout duration for the I2C operation
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
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)bmp280->i2c_timeout_ms * 1000;

    ESP_LOGI(TAG, "Checking if BMP280 is currently measuring...");
    do {
        const esp_err_t ret = i2c_read_registers(
            bmp280->i2c_master_dev_handle,
            BMP280_REG_STATUS,
            &status,
            1,
            bmp280->i2c_timeout_ms);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read status register: %s", esp_err_to_name(ret));
            return ret;
        }
        if ((status & BMP280_STATUS_MEASURING) != 0U) {
            if (esp_timer_get_time() >= deadline_us) {
                ESP_LOGE(TAG, "Timed out waiting for BMP280 measurement");
                return ESP_ERR_TIMEOUT;
            }
            vTaskDelay(1);
        }
    } while (status & BMP280_STATUS_MEASURING); // Wait while Bit 3 is 1

    return ESP_OK;
}
