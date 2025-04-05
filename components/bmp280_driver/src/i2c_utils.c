#include "i2c_utils.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

#define TAG "   ***i2c_utils***   "

esp_err_t i2c_init(const i2c_port_t i2c_port, const uint16_t i2c_address, const gpio_num_t sda_gpio, const gpio_num_t scl_gpio,
                   const uint32_t clk_speed, i2c_master_bus_handle_t *master_bus_handle, i2c_master_dev_handle_t *master_dev_handle) {
    if (!master_bus_handle || !master_dev_handle) {
        ESP_LOGE(TAG, "Master bus or device handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (sda_gpio < 0 || scl_gpio < 0 || clk_speed == 0) {
        ESP_LOGE(TAG, "Invalid I2C configuration parameters");
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate and initialize an I2C master bus
    ESP_LOGI(TAG, "Initializing I2C master bus...");
    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_config, master_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Probe I2C address to ensure connectivity
    ESP_LOGI(TAG, "Probing I2C device at address 0x%02X...", i2c_address);
    ret = i2c_master_probe(*master_bus_handle, i2c_address, 100); // 100 ms timeout
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to probe I2C device at address 0x%02X: %s", i2c_address, esp_err_to_name(ret));
        i2c_del_master_bus(*master_bus_handle); // Cleanup bus
        return ret;
    }

    // Add device to the bus
    ESP_LOGI(TAG, "Adding I2C device at address 0x%02X to the bus...", i2c_address);
    const i2c_device_config_t sensor_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address,
        .scl_speed_hz = clk_speed,
    };
    ret = i2c_master_bus_add_device(*master_bus_handle, &sensor_config, master_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device to the bus: %s", esp_err_to_name(ret));
        i2c_del_master_bus(*master_bus_handle); // Cleanup bus
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized on port %d with SDA: GPIO%d, SCL: GPIO%d, Speed: %u Hz",
             i2c_port, sda_gpio, scl_gpio, (unsigned int)clk_speed);

    return ESP_OK;
}

esp_err_t i2c_cleanup(i2c_master_bus_handle_t master_bus_handle, i2c_master_dev_handle_t master_dev_handle) {
    if (!master_dev_handle || !master_bus_handle) {
        ESP_LOGE(TAG, "Master bus or device handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Remove the device from the bus
    esp_err_t ret = i2c_master_bus_rm_device(master_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Delete the bus
    ret = i2c_del_master_bus(master_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C cleaned up successfully");
    return ESP_OK;
}

esp_err_t i2c_write_register(i2c_master_dev_handle_t master_device_handle, const uint8_t register_address,
                         const uint8_t data, const uint16_t i2c_timeout_ms) {
    const uint8_t write_data[2] = {register_address, data};

    const esp_err_t ret = i2c_master_transmit(master_device_handle, write_data, sizeof(write_data),
                                        i2c_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write byte to register 0x%02X: %s", register_address, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t i2c_read_registers(i2c_master_dev_handle_t master_device_handle, const uint8_t start_register,
                         uint8_t *data, size_t length, const uint16_t i2c_timeout_ms) {
    if (!master_device_handle) {
        ESP_LOGE(TAG, "Master device handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (!data) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (length == 0) {
        ESP_LOGE(TAG, "Invalid length: must be greater than 0");
        return ESP_ERR_INVALID_ARG;
    }

    // Write the start register address (1 byte) and read the subsequent data
    esp_err_t ret = i2c_master_transmit_receive(master_device_handle, &start_register, 1, data,
                                                length, i2c_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to perform burst read starting at 0x%02X: %s", start_register, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t i2c_wait_all_done(i2c_master_bus_handle_t master_bus_handle, const int timeout_ms) {
    if (!master_bus_handle) {
        ESP_LOGE(TAG, "Master bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_bus_wait_all_done(master_bus_handle, timeout_ms);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C wait timeout after %d ms", timeout_ms);
        } else {
            ESP_LOGE(TAG, "I2C wait failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGI(TAG, "I2C bus transactions completed successfully");
    }
    return ret;
}
