idf_component_register(
        SRCS "src/matter_manager.cpp" "src/matter_temperature_sensor.cpp" "src/matter_pressure_sensor.cpp"
        INCLUDE_DIRS "include"
        REQUIRES esp_matter
)
