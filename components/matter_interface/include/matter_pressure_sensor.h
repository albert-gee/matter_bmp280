#ifndef MATTER_PRESSURE_SENSOR_H
#define MATTER_PRESSURE_SENSOR_H

#include <stdint.h>
#include <esp_err.h>
#include <esp_matter.h>

esp_err_t create_pressure_sensor_endpoint(esp_matter::node_t *matter_node, uint16_t *endpoint_id, int16_t min_pressure,
                                         int16_t max_pressure, int16_t initial_pressure);

#endif //MATTER_PRESSURE_SENSOR_H
