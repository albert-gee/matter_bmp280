#ifndef MATTER_TEMPERATURE_SENSOR_H
#define MATTER_TEMPERATURE_SENSOR_H

/**
 * @brief Create a Matter temperature sensor endpoint.
 *
 * This function sets up a new Matter temperature sensor endpoint with the specified properties.
 *
 * @param[out] matter_node        Pointer to the Node handle.
 * @param[out] endpoint_id        Pointer to store the assigned endpoint ID.
 * @param[in] min_temperature     Minimum allowable temperature in Celsius.
 * @param[in] max_temperature     Maximum allowable temperature in Celsius.
 * @param[in] initial_temperature Initial temperature value in Celsius.
 * @return
 *      - ESP_OK on success.
 *      - ESP_ERR_INVALID_ARG if arguments are invalid.
 *      - Other error codes on failure.
 */
esp_err_t create_temperature_sensor_endpoint(esp_matter::node_t *matter_node, uint16_t *endpoint_id, float min_temperature, float max_temperature, float initial_temperature);

#endif // MATTER_TEMPERATURE_SENSOR_H
