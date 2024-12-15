#include <esp_log.h>
#include <esp_matter.h>

#include "matter_temperature_sensor.h"

static auto TAG = "   ***matter_interface***   ";

esp_err_t create_temperature_sensor_endpoint(esp_matter::node_t *matter_node, uint16_t *endpoint_id,
    const float min_temperature, const float max_temperature, const float initial_temperature) {

    esp_matter::endpoint::temperature_sensor::config_t temperature_sensor_endpoint_config;
    temperature_sensor_endpoint_config.temperature_measurement.min_measured_value = static_cast<nullable<int16_t>>(
        min_temperature * 100);
    temperature_sensor_endpoint_config.temperature_measurement.max_measured_value = static_cast<nullable<int16_t>>(
        max_temperature * 100);
    temperature_sensor_endpoint_config.temperature_measurement.measured_value = static_cast<nullable<int16_t>>(
        initial_temperature * 100);

    esp_matter::endpoint_t *temperature_sensor_endpoint = esp_matter::endpoint::temperature_sensor::create(
        matter_node, &temperature_sensor_endpoint_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    if (temperature_sensor_endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create temperature sensor endpoint.");
        return ESP_FAIL;
    }

    *endpoint_id = esp_matter::endpoint::get_id(temperature_sensor_endpoint);
    ESP_LOGI(TAG, "Temperature sensor endpoint created with id %d", *endpoint_id);

    return ESP_OK;
}
