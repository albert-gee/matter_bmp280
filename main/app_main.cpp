#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include <portmacro.h>
#include <FreeRTOSConfig.h>
#include <freertos/projdefs.h>
#include <freertos/task.h>

#include "bmp280_driver.h"
#include "matter_manager.h"

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

// Logging tag
static const char *TAG = "   ***app_main***   ";

// BMP280 Sensor Configuration
#define BMP280_I2C_ADDRESS BMP280_I2C_ADDRESS_76  // BMP280 I2C address (0x76 with SDO connected to GND)
#define GPIO_NUM_SDA GPIO_NUM_27                  // GPIO pin for I2C SDA line
#define GPIO_NUM_SCL GPIO_NUM_26                  // GPIO pin for I2C SCL line
#define I2C_PORT_NUM I2C_NUM_0                    // I2C port number
#define I2C_TIMEOUT_TICKS 1000                    // Timeout for I2C operations (ticks)
#define SCL_SPEED_HZ 100000                       // I2C clock speed (100 kHz)

// BMP280 Measurement Configuration
#define BMP280_OVERSAMPLING_TEMP BMP280_OSRS_T_X4 // Temperature oversampling (x4)
#define BMP280_OVERSAMPLING_PRESS BMP280_OSRS_P_X8 // Pressure oversampling (x8)
#define BMP280_OPERATION_MODE BMP280_MODE_NORMAL  // Operating mode (Normal)
#define BMP280_STANDBY_TIME BMP280_STANDBY_TIME_500_MS // Standby time (500 ms)
#define BMP280_FILTER_COEFFICIENT BMP280_FILTER_COEFF_4 // Filter coefficient (4)

// Matter Configuration
#define MATTER_TEMP_SENSOR_INITIAL_VALUE BMP280_TEMP_MAX_DEG_CELSIUS    // Initial temperature value for Matter (in °C)
#define MATTER_TEMP_MIN BMP280_TEMP_MIN_DEG_CELSIUS                     // Minimum temperature value
#define MATTER_TEMP_MAX BMP280_TEMP_MAX_DEG_CELSIUS                     // Maximum temperature value
#define MATTER_PRESS_INITIAL_VALUE BMP280_PRESS_MAX_HPA                 // Initial temperature value for Matter (in hp
#define MATTER_PRESS_MIN BMP280_PRESS_MIN_HPA                           // Minimum pressure value
#define MATTER_PRESS_MAX BMP280_PRESS_MAX_HPA                           // Maximum pressure value

// Task Configuration
#define SENSOR_READ_DELAY_MS 10000 // Delay between sensor readings (in ms)

extern "C" void app_main() {
    // Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS partition...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGI(TAG, "NVS initialized.");

    // Initialize BMP280
    ESP_LOGI(TAG, "Initializing BMP280 sensor...");
    BMP280 bmp280;
    err = bmp280_init(&bmp280, BMP280_I2C_ADDRESS, I2C_PORT_NUM, I2C_TIMEOUT_TICKS, SCL_SPEED_HZ,
                      GPIO_NUM_SDA, GPIO_NUM_SCL, BMP280_OVERSAMPLING_TEMP, BMP280_OVERSAMPLING_PRESS,
                      BMP280_OPERATION_MODE, BMP280_STANDBY_TIME, BMP280_FILTER_COEFFICIENT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "BMP280 initialized.");



#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                           \
{                                                                                   \
.radio_mode = RADIO_MODE_NATIVE,                                                \
}

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                            \
{                                                                                   \
.host_connection_mode = HOST_CONNECTION_MODE_NONE,                              \
}

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
{                                                                                   \
.storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
}
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif


    // Initialize Matter
    ESP_LOGI(TAG, "Initializing Matter interface...");
    uint16_t temp_sensor_ep_id, press_sensor_ep_id;
    err = matter_init(&temp_sensor_ep_id, &press_sensor_ep_id, MATTER_TEMP_MIN, MATTER_TEMP_MAX,
                      MATTER_TEMP_SENSOR_INITIAL_VALUE, MATTER_PRESS_MIN, MATTER_PRESS_MAX,
                      MATTER_PRESS_INITIAL_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Matter initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Matter initialized.");

    // Sensor read loop
    int32_t temperature = 0, pressure = 0;
    while (true) {
        err = bmp280_read_temperature_pressure(&bmp280, &temperature, &pressure);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(err));
        } else {
            // Scale values as needed (e.g., divide by 100 for human-readable format)
            ESP_LOGI(TAG, "Temperature = %.2f°C, Pressure = %.2f hPa", temperature / 100.0, pressure / 100.0);

            // Update Matter attributes
            err = matter_update_values(temp_sensor_ep_id, press_sensor_ep_id,
                                       static_cast<int16_t>(temperature), static_cast<int16_t>(pressure / 10));
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update Matter attributes: %s", esp_err_to_name(err));
            }
        }

        // Delay for next reading
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_DELAY_MS));
    }
}
