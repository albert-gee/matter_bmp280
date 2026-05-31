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
#define I2C_TIMEOUT_MS 1000                       // Timeout for I2C transactions (ms)
#define SCL_SPEED_HZ 100000                       // I2C clock speed (100 kHz)

// BMP280 Measurement Configuration
#define BMP280_OVERSAMPLING_TEMP BMP280_OSRS_T_X4 // Temperature oversampling (x4)
#define BMP280_OVERSAMPLING_PRESS BMP280_OSRS_P_X8 // Pressure oversampling (x8)
#define BMP280_OPERATION_MODE BMP280_MODE_NORMAL  // Operating mode (Normal)
#define BMP280_STANDBY_TIME BMP280_STANDBY_TIME_500_MS // Standby time (500 ms)
#define BMP280_FILTER_COEFFICIENT BMP280_FILTER_COEFF_4 // Filter coefficient (4)

// Task Configuration
#define SENSOR_READ_DELAY_MS 10000 // Delay between sensor readings (in ms)

static bool is_valid_temperature_centi_celsius(const int32_t temperature) {
    return temperature >= static_cast<int32_t>(BMP280_TEMP_MIN_DEG_CELSIUS * 100) &&
           temperature <= static_cast<int32_t>(BMP280_TEMP_MAX_DEG_CELSIUS * 100);
}

static bool is_valid_pressure_pa(const int32_t pressure) {
    return pressure >= static_cast<int32_t>(BMP280_PRESS_MIN_HPA * 100) &&
           pressure <= static_cast<int32_t>(BMP280_PRESS_MAX_HPA * 100);
}

static int16_t pressure_pa_to_matter_measured_value(const int32_t pressure_pa) {
    // Matter PressureMeasurement MeasuredValue is 10 x pressure in kPa.
    // BMP280 reports Pa: pressure_pa / 1000 = kPa, then x 10 => pressure_pa / 100.
    return static_cast<int16_t>(pressure_pa / 100);
}

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
    err = bmp280_init(&bmp280, BMP280_I2C_ADDRESS, I2C_PORT_NUM, I2C_TIMEOUT_MS, SCL_SPEED_HZ,
                      GPIO_NUM_SDA, GPIO_NUM_SCL, BMP280_OVERSAMPLING_TEMP, BMP280_OVERSAMPLING_PRESS,
                      BMP280_OPERATION_MODE, BMP280_STANDBY_TIME, BMP280_FILTER_COEFFICIENT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "BMP280 initialized.");

    int32_t temperature = 0;
    int32_t pressure = 0;
    err = bmp280_read_temperature_pressure(&bmp280, &temperature, &pressure);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initial BMP280 read failed: %s", esp_err_to_name(err));
        return;
    }
    if (!is_valid_temperature_centi_celsius(temperature) || !is_valid_pressure_pa(pressure)) {
        ESP_LOGE(TAG, "Initial BMP280 reading is out of range: temperature=%ld centi-C, pressure=%ld Pa",
                 static_cast<long>(temperature), static_cast<long>(pressure));
        return;
    }
    ESP_LOGI(TAG, "Initial BMP280 reading: Temperature = %.2f°C, Pressure = %.2f hPa",
             temperature / 100.0, pressure / 100.0);



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
    err = matter_init(&temp_sensor_ep_id, &press_sensor_ep_id,
                      BMP280_TEMP_MIN_DEG_CELSIUS, BMP280_TEMP_MAX_DEG_CELSIUS, temperature / 100.0f,
                      pressure_pa_to_matter_measured_value(static_cast<int32_t>(BMP280_PRESS_MIN_HPA * 100)),
                      pressure_pa_to_matter_measured_value(static_cast<int32_t>(BMP280_PRESS_MAX_HPA * 100)),
                      pressure_pa_to_matter_measured_value(pressure));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Matter initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Matter initialized.");

    // Sensor read loop
    uint32_t consecutive_failures = 0;
    while (true) {
        err = bmp280_read_temperature_pressure(&bmp280, &temperature, &pressure);
        if (err != ESP_OK) {
            consecutive_failures++;
            ESP_LOGE(TAG, "Failed to read sensor data: %s; Matter values are stale for %lu update(s)",
                     esp_err_to_name(err), static_cast<unsigned long>(consecutive_failures));
        } else {
            if (!is_valid_temperature_centi_celsius(temperature)) {
                consecutive_failures++;
                ESP_LOGE(TAG, "Invalid temperature reading %ld centi-C; Matter values are stale for %lu update(s)",
                         static_cast<long>(temperature), static_cast<unsigned long>(consecutive_failures));
            } else if (!is_valid_pressure_pa(pressure)) {
                consecutive_failures++;
                ESP_LOGE(TAG, "Invalid pressure reading %ld Pa; Matter values are stale for %lu update(s)",
                         static_cast<long>(pressure), static_cast<unsigned long>(consecutive_failures));
            } else {
                ESP_LOGI(TAG, "Temperature = %.2f°C, Pressure = %.2f hPa", temperature / 100.0, pressure / 100.0);

                err = matter_update_values(temp_sensor_ep_id, press_sensor_ep_id,
                                           static_cast<int16_t>(temperature),
                                           pressure_pa_to_matter_measured_value(pressure));
                if (err != ESP_OK) {
                    consecutive_failures++;
                    ESP_LOGE(TAG, "Failed to update Matter attributes: %s; Matter values are stale for %lu update(s)",
                             esp_err_to_name(err), static_cast<unsigned long>(consecutive_failures));
                } else if (consecutive_failures > 0) {
                    ESP_LOGI(TAG, "Matter values refreshed after %lu stale update(s)",
                             static_cast<unsigned long>(consecutive_failures));
                    consecutive_failures = 0;
                }
            }
        }

        // Delay for next reading
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_DELAY_MS));
    }
}
