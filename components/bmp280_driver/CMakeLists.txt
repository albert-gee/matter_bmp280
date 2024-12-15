idf_component_register(
        SRCS "src/bmp280_driver.c" "src/bmp280_calibration.c" "src/i2c_utils.c"     # list of source files
        INCLUDE_DIRS "include"                                                      # list of directories to include in global include search path for any component which requires this component, and also the main source files
        REQUIRES "driver"                                                           # list of components that this component depends on
)
