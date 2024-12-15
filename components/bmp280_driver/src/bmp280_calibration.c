#include "bmp280_calibration.h"

#include <driver/i2c_master.h>
#include <driver/i2c_types.h>

#include "i2c_utils.h"

static int32_t t_fine;  // Declare t_fine as a global variable for reuse in pressure calculation

esp_err_t bmp280_read_calibration_data(i2c_master_dev_handle_t master_device_handle, BMP280_CalibrationData *calibration_data, const uint32_t i2c_timeout_ticks) {
    uint8_t read_buffer[24]; // Buffer to read all calibration data at once

    // Read 24 bytes of calibration data starting from register 0x88
    const esp_err_t ret = i2c_read_registers(master_device_handle, BMP280_REG_DIG_T1, read_buffer, sizeof(read_buffer), i2c_timeout_ticks);

    if (ret == ESP_OK) {

        // Assign values to each calibration parameter from buffer
        calibration_data->dig_T1 = (uint16_t)((read_buffer[1] << 8) | read_buffer[0]);
        calibration_data->dig_T2 = (int16_t)((read_buffer[3] << 8) | read_buffer[2]);
        calibration_data->dig_T3 = (int16_t)((read_buffer[5] << 8) | read_buffer[4]);
        calibration_data->dig_P1 = (uint16_t)((read_buffer[7] << 8) | read_buffer[6]);
        calibration_data->dig_P2 = (int16_t)((read_buffer[9] << 8) | read_buffer[8]);
        calibration_data->dig_P3 = (int16_t)((read_buffer[11] << 8) | read_buffer[10]);
        calibration_data->dig_P4 = (int16_t)((read_buffer[13] << 8) | read_buffer[12]);
        calibration_data->dig_P5 = (int16_t)((read_buffer[15] << 8) | read_buffer[14]);
        calibration_data->dig_P6 = (int16_t)((read_buffer[17] << 8) | read_buffer[16]);
        calibration_data->dig_P7 = (int16_t)((read_buffer[19] << 8) | read_buffer[18]);
        calibration_data->dig_P8 = (int16_t)((read_buffer[21] << 8) | read_buffer[20]);
        calibration_data->dig_P9 = (int16_t)((read_buffer[23] << 8) | read_buffer[22]);
    }

    return ret;
}

int32_t bmp280_compensate_temperature(const int32_t raw_temp, const BMP280_CalibrationData *cal) {
    // Intermediate calculations for temperature compensation
    int32_t var1 = (((raw_temp >> 3) - ((int32_t)cal->dig_T1 << 1)) * (int32_t)cal->dig_T2) >> 11;
    int32_t var2 = (((((raw_temp >> 4) - (int32_t)cal->dig_T1) *
                      ((raw_temp >> 4) - (int32_t)cal->dig_T1)) >> 12) *
                      (int32_t)cal->dig_T3) >> 14;

    // Store fine temperature for further pressure compensation (if needed)
    t_fine = var1 + var2;

    // Return temperature in hundredths of degrees Celsius (e.g., 2511 means 25.11°C).
    return (t_fine * 5 + 128) >> 8;
}

int32_t bmp280_compensate_pressure(const int32_t raw_press, const BMP280_CalibrationData *cal) {
    if (t_fine == 0) {
        return 0; // Ensure t_fine is initialized properly from temperature compensation.
    }

    // Calculate intermediate variables for compensation (datasheet Section 3.11.3).
    int64_t var1 = ((int64_t) t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t) cal->dig_P6;
    var2 = var2 + ((var1 * (int64_t)cal->dig_P5) << 17);
    var2 = var2 + (((int64_t)cal->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)cal->dig_P3) >> 8) + ((var1 * (int64_t)cal->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)cal->dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // Avoid division by zero.
    }

    // Calculate compensated pressure.
    int64_t p = 1048576 - raw_press;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (((int64_t)cal->dig_P9) * (p >> 13) * (p >> 13)) >> 25; // Higher-order corrections (dig_P9).
    var2 = (((int64_t)cal->dig_P8) * p) >> 19; // Linear correction (dig_P8).
    p = ((p + var1 + var2) >> 8) + (((int64_t)cal->dig_P7) << 4); // Final adjustment (dig_P7).

    // Convert to Pascals (Pa).
    return (int32_t)(p / 256);
}
