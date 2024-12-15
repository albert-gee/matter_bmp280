#ifndef BMP280_CALIBRATION_H
#define BMP280_CALIBRATION_H

#include <esp_err.h>
#include <stdint.h>
#include <driver/i2c_types.h>

// Register addresses for temperature calibration data
#define BMP280_REG_DIG_T1   0x88
#define BMP280_REG_DIG_T2   0x8A
#define BMP280_REG_DIG_T3   0x8C

// Register addresses for pressure calibration data
#define BMP280_REG_DIG_P1   0x8E
#define BMP280_REG_DIG_P2   0x90
#define BMP280_REG_DIG_P3   0x92
#define BMP280_REG_DIG_P4   0x94
#define BMP280_REG_DIG_P5   0x96
#define BMP280_REG_DIG_P6   0x98
#define BMP280_REG_DIG_P7   0x9A
#define BMP280_REG_DIG_P8   0x9C
#define BMP280_REG_DIG_P9   0x9E

/**
 * @brief Structure for storing BMP280 calibration data.
 *
 * This structure holds the calibration data read from the BMP280 sensor.
 * The data is used for temperature and pressure compensation.
 */
typedef struct {
    uint16_t dig_T1; /**< Calibration data for temperature */
    int16_t dig_T2;  /**< Calibration data for temperature */
    int16_t dig_T3;  /**< Calibration data for temperature */
    uint16_t dig_P1; /**< Calibration data for pressure */
    int16_t dig_P2;  /**< Calibration data for pressure */
    int16_t dig_P3;  /**< Calibration data for pressure */
    int16_t dig_P4;  /**< Calibration data for pressure */
    int16_t dig_P5;  /**< Calibration data for pressure */
    int16_t dig_P6;  /**< Calibration data for pressure */
    int16_t dig_P7;  /**< Calibration data for pressure */
    int16_t dig_P8;  /**< Calibration data for pressure */
    int16_t dig_P9;  /**< Calibration data for pressure */
} BMP280_CalibrationData;

/**
 * @brief Reads the calibration data from the BMP280 sensor.
 *
 * This function reads the calibration data from the BMP280 sensor over I2C
 * and stores it in the provided BMP280_CalibrationData structure.
 *
 * @param master_device_handle The I2C master device handle.
 * @param calibration_data Pointer to the structure where the calibration data will be stored.
 * @param i2c_timeout_ticks Timeout for I2C operations in ticks
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Parameter error
 *     - ESP_FAIL: Sending command error, slave doesn't ACK the transfer.
 */
esp_err_t bmp280_read_calibration_data(i2c_master_dev_handle_t master_device_handle, BMP280_CalibrationData *calibration_data, uint32_t i2c_timeout_ticks);

/**
 * @brief Compensates the raw temperature data using the calibration data.
 *
 * This function applies the calibration data to the raw temperature data
 * to obtain the compensated temperature value.
 *
 * @param raw_temp The raw temperature data.
 * @param cal Pointer to the BMP280_CalibrationData structure.
 * @return The compensated temperature value.
 */
int32_t bmp280_compensate_temperature(int32_t raw_temp, const BMP280_CalibrationData *cal);

/**
 * @brief Compensates the raw pressure data using the calibration data.
 *
 * This function applies the calibration data to the raw pressure data
 * to obtain the compensated pressure value.
 *
 * @param raw_press The raw pressure data.
 * @param cal Pointer to the BMP280_CalibrationData structure.
 * @return The compensated pressure value.
 */
int32_t bmp280_compensate_pressure(int32_t raw_press, const BMP280_CalibrationData *cal);

#endif // BMP280_CALIBRATION_H