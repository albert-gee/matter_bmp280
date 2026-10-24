#include <esp_log.h>
#include <esp_matter.h>

#include <cmath>
#include <limits>

#include "matter_temperature_sensor.h"

static auto TAG = "   ***matter_interface***   ";

esp_err_t create_temperature_sensor_endpoint(esp_matter::node_t *matter_node, uint16_t *endpoint_id,
    const float min_temperature, const float max_temperature) {

    if (matter_node == nullptr || endpoint_id == nullptr || !std::isfinite(min_temperature) ||
        !std::isfinite(max_temperature) || min_temperature > max_temperature) {
        return ESP_ERR_INVALID_ARG;
    }

    const float scaled_min = std::round(min_temperature * 100.0f);
    const float scaled_max = std::round(max_temperature * 100.0f);
    if (scaled_min < std::numeric_limits<int16_t>::min() ||
        scaled_max > std::numeric_limits<int16_t>::max()) {
        ESP_LOGE(TAG, "Temperature limits are outside the Matter int16 range");
        return ESP_ERR_INVALID_ARG;
    }

    esp_matter::endpoint::temperature_sensor::config_t temperature_sensor_endpoint_config;
    temperature_sensor_endpoint_config.temperature_measurement.min_measured_value =
        nullable<int16_t>(static_cast<int16_t>(scaled_min));
    temperature_sensor_endpoint_config.temperature_measurement.max_measured_value =
        nullable<int16_t>(static_cast<int16_t>(scaled_max));
    temperature_sensor_endpoint_config.temperature_measurement.measured_value = nullable<int16_t>();

    esp_matter::endpoint_t *temperature_sensor_endpoint = esp_matter::endpoint::temperature_sensor::create(
        matter_node, &temperature_sensor_endpoint_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    if (temperature_sensor_endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create temperature sensor endpoint.");
        return ESP_FAIL;
    }

    *endpoint_id = esp_matter::endpoint::get_id(temperature_sensor_endpoint);
    ESP_LOGI(TAG, "Temperature sensor endpoint created with id %d", *endpoint_id);


    esp_matter::cluster::binding::config_t config;
    esp_matter::cluster::binding::create(temperature_sensor_endpoint, &config, esp_matter::CLUSTER_FLAG_SERVER);


    return ESP_OK;
}
