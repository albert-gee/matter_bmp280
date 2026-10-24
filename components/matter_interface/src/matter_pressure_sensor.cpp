#include <esp_log.h>
#include <esp_matter.h>

#include <cmath>
#include <limits>

#include "matter_pressure_sensor.h"

static auto TAG = "   ***matter_interface***   ";

esp_err_t create_pressure_sensor_endpoint(esp_matter::node_t *matter_node, uint16_t *endpoint_id,
                                         const float min_pressure_hpa, const float max_pressure_hpa) {

    if (matter_node == nullptr || endpoint_id == nullptr || !std::isfinite(min_pressure_hpa) ||
        !std::isfinite(max_pressure_hpa) || min_pressure_hpa > max_pressure_hpa) {
        return ESP_ERR_INVALID_ARG;
    }

    const float rounded_min = std::round(min_pressure_hpa);
    const float rounded_max = std::round(max_pressure_hpa);
    if (rounded_min < std::numeric_limits<int16_t>::min() ||
        rounded_max > std::numeric_limits<int16_t>::max()) {
        ESP_LOGE(TAG, "Pressure limits are outside the Matter int16 range");
        return ESP_ERR_INVALID_ARG;
    }

    const int16_t min_pressure = static_cast<int16_t>(rounded_min);
    const int16_t max_pressure = static_cast<int16_t>(rounded_max);

    // Set up the pressure sensor endpoint
    esp_matter::endpoint::pressure_sensor::config_t pressure_sensor_endpoint_config;
    pressure_sensor_endpoint_config.pressure_measurement.pressure_min_measured_value = static_cast<nullable<int16_t>>(min_pressure);
    pressure_sensor_endpoint_config.pressure_measurement.pressure_max_measured_value = static_cast<nullable<int16_t>>(max_pressure);
    pressure_sensor_endpoint_config.pressure_measurement.pressure_measured_value = nullable<int16_t>();

    esp_matter::endpoint_t *pressure_sensor_endpoint = esp_matter::endpoint::pressure_sensor::create(
        matter_node, &pressure_sensor_endpoint_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    if (pressure_sensor_endpoint == nullptr) {
        ESP_LOGE(TAG, "Failed to create pressure sensor endpoint.");
        return ESP_FAIL;
    }

    *endpoint_id = esp_matter::endpoint::get_id(pressure_sensor_endpoint);
    ESP_LOGI(TAG, "Pressure sensor endpoint created with id %d", *endpoint_id);

    return ESP_OK;
}
