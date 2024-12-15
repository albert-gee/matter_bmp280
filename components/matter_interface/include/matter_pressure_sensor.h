#ifndef MATTER_PRESSURE_SENSOR_H
#define MATTER_PRESSURE_SENSOR_H

#include <esp_err.h>

esp_err_t create_pressure_sensor_endpoint(esp_matter::node_t *matter_node, uint16_t *endpoint_id, int16_t min_pressure,
                                         int16_t max_pressure, int16_t initial_pressure);

esp_err_t pressure_measured_value_attribute_update(uint16_t endpoint_id, float new_temperature);

#endif //MATTER_PRESSURE_SENSOR_H
