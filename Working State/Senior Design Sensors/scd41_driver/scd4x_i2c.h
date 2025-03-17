/*
 * THIS FILE IS AUTOMATICALLY GENERATED
 *
 * Generator:     sensirion-driver-generator 1.1.2
 * Product:       scd4x
 * Model-Version: 2.0
 */
/*
 * Copyright (c) 2025, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SCD4X_I2C_H
#define SCD4X_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "sensirion_config.h"
#include "../sps30-i2c-3.1.1/sps30-i2c-3.1.1/sensirion_arch_config.h"

#define SCD40_I2C_ADDR_62 0x62
#define SCD41_I2C_ADDR_62 0x62

typedef enum {
    SCD4X_START_PERIODIC_MEASUREMENT_CMD_ID = 0x21b1,
    SCD4X_READ_MEASUREMENT_RAW_CMD_ID = 0xec05,
    SCD4X_STOP_PERIODIC_MEASUREMENT_CMD_ID = 0x3f86,
    SCD4X_SET_TEMPERATURE_OFFSET_RAW_CMD_ID = 0x241d,
    SCD4X_GET_TEMPERATURE_OFFSET_RAW_CMD_ID = 0x2318,
    SCD4X_SET_SENSOR_ALTITUDE_CMD_ID = 0x2427,
    SCD4X_GET_SENSOR_ALTITUDE_CMD_ID = 0x2322,
    SCD4X_SET_AMBIENT_PRESSURE_RAW_CMD_ID = 0xe000,
    SCD4X_GET_AMBIENT_PRESSURE_RAW_CMD_ID = 0xe000,
    SCD4X_PERFORM_FORCED_RECALIBRATION_CMD_ID = 0x362f,
    SCD4X_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED_CMD_ID = 0x2416,
    SCD4X_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED_CMD_ID = 0x2313,
    SCD4X_SET_AUTOMATIC_SELF_CALIBRATION_TARGET_CMD_ID = 0x243a,
    SCD4X_GET_AUTOMATIC_SELF_CALIBRATION_TARGET_CMD_ID = 0x233f,
    SCD4X_START_LOW_POWER_PERIODIC_MEASUREMENT_CMD_ID = 0x21ac,
    SCD4X_GET_DATA_READY_STATUS_RAW_CMD_ID = 0xe4b8,
    SCD4X_PERSIST_SETTINGS_CMD_ID = 0x3615,
    SCD4X_GET_SERIAL_NUMBER_CMD_ID = 0x3682,
    SCD4X_PERFORM_SELF_TEST_CMD_ID = 0x3639,
    SCD4X_PERFORM_FACTORY_RESET_CMD_ID = 0x3632,
    SCD4X_REINIT_CMD_ID = 0x3646,
    SCD4X_GET_SENSOR_VARIANT_RAW_CMD_ID = 0x202f,
    SCD4X_MEASURE_SINGLE_SHOT_CMD_ID = 0x219d,
    SCD4X_MEASURE_SINGLE_SHOT_RHT_ONLY_CMD_ID = 0x2196,
    SCD4X_POWER_DOWN_CMD_ID = 0x36e0,
    SCD4X_WAKE_UP_CMD_ID = 0x36f6,
    SCD4X_SET_AUTOMATIC_SELF_CALIBRATION_INITIAL_PERIOD_CMD_ID = 0x2445,
    SCD4X_GET_AUTOMATIC_SELF_CALIBRATION_INITIAL_PERIOD_CMD_ID = 0x2340,
    SCD4X_SET_AUTOMATIC_SELF_CALIBRATION_STANDARD_PERIOD_CMD_ID = 0x244e,
    SCD4X_GET_AUTOMATIC_SELF_CALIBRATION_STANDARD_PERIOD_CMD_ID = 0x234b,
} SCD4X_CMD_ID;

typedef enum {
    SCD4X_SENSOR_VARIANT_UNKNOWN = 0,
    SCD4X_SENSOR_VARIANT_SCD40 = 1,
    SCD4X_SENSOR_VARIANT_SCD41 = 2,
} scd4x_sensor_variant;

/**
 * @brief Initialize i2c address of driver
 *
 * @param[in] i2c_address Used i2c address
 *
 */
void scd4x_init(uint8_t i2c_address);

/**
 * @brief scd4x_signal_co2_concentration
 *
 * @param[in] raw_co2_concentration
 *
 * @return CO₂ concentration in ppm
 */
uint16_t scd4x_signal_co2_concentration(uint16_t raw_co2_concentration);

/**
 * @brief scd4x_signal_ambient_pressure
 *
 * @param[in] raw_ambient_pressure
 *
 * @return Pressure in Pa
 */
uint32_t scd4x_signal_ambient_pressure(uint16_t raw_ambient_pressure);

/**
 * @brief Set the ambient pressure around the sensor.
 *
 * The set_ambient_pressure command can be sent during periodic measurements to
 * enable continuous pressure compensation. Note that setting an ambient
 * pressure overrides any pressure compensation based on a previously set sensor
 * altitude. Use of this command is highly recommended for applications
 * experiencing significant ambient pressure changes to ensure sensor accuracy.
 * Valid input values are between 70000 - 120000 Pa. The default value is 101300
 * Pa.
 *
 * @param[in] ambient_pressure Ambient pressure around the sensor in Pa
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_set_ambient_pressure(uint32_t ambient_pressure);

/**
 * @brief Get the ambient pressure around the sensor.
 *
 * @param[out] a_ambient_pressure Pressure in Pa
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_ambient_pressure(uint32_t* a_ambient_pressure);

/**
 * @brief Read if data is ready.
 *
 * Polls the sensor for whether data from a periodic or single shot measurement
 * is ready to be read out.
 *
 * @param[out] arg_0
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_data_ready_status(bool* arg_0);

/**
 * @brief Reads out the SCD4x sensor variant.
 *
 * @param[out] a_sensor_variant
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_sensor_variant(scd4x_sensor_variant* a_sensor_variant);

/**
 * @brief Start periodic measurement mode.
 *
 * Starts the periodic measurement mode. The signal update interval is 5
 * seconds.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_start_periodic_measurement();

/**
 * @brief Read CO₂, temperature, and humidity measurements raw values.
 *
 * Reads the sensor output. The measurement data can only be read out once per
 * signal update interval as the buffer is emptied upon read-out. If no data is
 * available in the buffer, the sensor returns a NACK. To avoid a NACK response,
 * the get_data_ready_status can be issued to check data status. The I2C master
 * can abort the read transfer with a NACK followed by a STOP condition after
 * any data byte if the user is not interested in subsequent data.
 *
 * @param[out] co2_concentration CO₂ concentration in ppm
 * @param[out] temperature Convert to degrees celsius by (175 * value / 65535) -
 * 45
 * @param[out] relative_humidity Convert to relative humidity in % by (100 *
 * value / 65535)
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_read_measurement_raw(uint16_t* co2_concentration,
                                   uint16_t* temperature,
                                   uint16_t* relative_humidity);

/**
 * @brief Read sensor output and convert to pyhsical unit.
 *
 * See @ref scd4x_read_measurement_raw() for more details.
 *
 * @param[out] co2 CO₂ concentration in ppm
 * @param[out] temperature_m_deg_c Temperature in milli degrees celsius (°C *
 * 1000)
 * @param[out] humidity_m_percent_rh Relative humidity in milli percent RH
 * (%RH * 1000)
 * @return 0 on success, an error code otherwise
 */
int16_t scd4x_read_measurement(uint16_t* co2, int32_t* temperature_m_deg_c,
                               int32_t* humidity_m_percent_rh);

/**
 * @brief Stop periodic measurement to change the sensor configuration or to
 * save power.
 *
 * Command returns a sensor running in periodic measurement mode or low power
 * periodic measurement mode back to the idle state, e.g. to then allow changing
 * the sensor configuration or to save power.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_stop_periodic_measurement();

/**
 * @brief Set the temperature compensation offset (raw value).
 *
 * Setting the temperature offset of the SCD4x inside the customer device allows
 * the user to optimize the RH and T output signal. The temperature offset can
 * depend on several factors such as the SCD4x measurement mode, self-heating of
 * close components, the ambient temperature and air flow. Thus, the SCD4x
 * temperature offset should be determined after integration into the final
 * device and under its typical operating conditions (including the operation
 * mode to be used in the application) in thermal equilibrium. By default, the
 * temperature offset is set to 4 °C. To save the setting to the EEPROM, the
 * persist_settings command may be issued. Equation (1) details how the
 * characteristic temperature offset can be calculated using the current
 * temperature output of the sensor (TSCD4x), a reference temperature value
 * (TReference), and the previous temperature offset (Toffset_pervious) obtained
 * using the get_temperature_offset_raw command:
 *
 * Toffset_actual = TSCD4x - TReference + Toffset_pervious.
 *
 * Recommended temperature offset values are between 0 °C and 20 °C. The
 * temperature offset does not impact the accuracy of the CO2 output.
 *
 * @param[in] offset_temperature Temperature offset. Convert Toffset in °C to
 * value by: (Toffset * 65535 / 175)
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 *
 * Example:
 * --------
 *
 * @code{.c}
 *
 *     int16_t local_error = 0;
 *     local_error = scd4x_set_temperature_offset_raw(1498);
 *     if (local_error != NO_ERROR) {
 *         return local_error;
 *     }
 *
 * @endcode
 *
 */
int16_t scd4x_set_temperature_offset_raw(uint16_t offset_temperature);

/**
 * @brief Get the raw temperature compensation offset used by the sensor.
 *
 * @param[out] offset_temperature Convert to °C by (175 * value / 65535)
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_temperature_offset_raw(uint16_t* offset_temperature);

/**
 * @brief Set the altitude of the sensor (in meters above sea level).
 *
 * Typically, the sensor altitude is set once after device installation. To save
 * the setting to the EEPROM, the persist_settings command must be issued. The
 * default sensor altitude value is set to 0 meters above sea level. Note that
 * setting a sensor altitude to the sensor overrides any pressure compensation
 * based on a previously set ambient pressure.
 *
 * @param[in] sensor_altitude Sensor altitude in meters above sea level. Valid
 * input values are between 0 - 3000 m.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 *
 * Example:
 * --------
 *
 * @code{.c}
 *
 *     int16_t local_error = 0;
 *     local_error = scd4x_set_sensor_altitude(0);
 *     if (local_error != NO_ERROR) {
 *         return local_error;
 *     }
 *
 * @endcode
 *
 */
int16_t scd4x_set_sensor_altitude(uint16_t sensor_altitude);

/**
 * @brief Get the sensor altitude used by the sensor.
 *
 * @param[out] sensor_altitude Sensor altitude used by the sensor in meters
 * above sea level.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_sensor_altitude(uint16_t* sensor_altitude);

/**
 * @brief Set the raw ambient pressure value.
 *
 * The set_ambient_pressure command can be sent during periodic measurements to
 * enable continuous pressure compensation. Note that setting an ambient
 * pressure overrides any pressure compensation based on a previously set sensor
 * altitude. Use of this command is highly recommended for applications
 * experiencing significant ambient pressure changes to ensure sensor accuracy.
 * Valid input values are between 70000 - 120000 Pa. The default value is 101300
 * Pa.
 *
 * @param[in] ambient_pressure Convert ambient_pressure in hPa to Pa by
 * ambient_pressure / 100.
 *
 * @note Available during measurements.
 *
 * @return error_code 0 on success, an error code otherwise.
 *
 * Example:
 * --------
 *
 * @code{.c}
 *
 *     int16_t local_error = 0;
 *     local_error = scd4x_set_ambient_pressure_raw(1013);
 *     if (local_error != NO_ERROR) {
 *         return local_error;
 *     }
 *
 * @endcode
 *
 */
int16_t scd4x_set_ambient_pressure_raw(uint16_t ambient_pressure);

/**
 * @brief Get the ambient pressure around the sensor.
 *
 * @param[out] ambient_pressure Convert to Pa by value = ambient_pressure * 100.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_ambient_pressure_raw(uint16_t* ambient_pressure);

/**
 * @brief Perform a forced recalibration (FRC) of the CO₂ concentration.
 *
 * To successfully conduct an accurate FRC, the following steps need to be
 * carried out:
 *
 * 1. Operate the SCD4x in the operation mode later used for normal sensor
 * operation (e.g. periodic measurement) for at least 3 minutes in an
 * environment with a homogenous and constant CO2 concentration. The sensor must
 * be operated at the voltage desired for the application when performing the
 * FRC sequence. 2. Issue the stop_periodic_measurement command. 3. Issue the
 * perform_forced_recalibration command.
 *
 * A return value of 0xffff indicates that the FRC has failed because the sensor
 * was not operated before sending the command.
 *
 * @param[in] target_co2_concentration Target CO₂ concentration in ppm CO₂.
 * @param[out] frc_correction Convert to FRC correction in ppm CO₂ by
 * frc_correction - 0x8000. A return value of 0xFFFF indicates that the FRC has
 * failed because the sensor was not operated before sending the command.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_perform_forced_recalibration(uint16_t target_co2_concentration,
                                           uint16_t* frc_correction);

/**
 * @brief Enable or disable automatic self calibration (ASC).
 *
 * Sets the current state (enabled / disabled) of the ASC. By default, ASC is
 * enabled. To save the setting to the EEPROM, the persist_settings command must
 * be issued. The ASC enables excellent long-term stability of SCD4x without the
 * need for regular user intervention. The algorithm leverages the sensor's
 * measurement history and the assumption of exposure of the sensor to a known
 * minimum background CO₂ concentration at least once over a period of
 * cumulative operation. By default, the ASC algorithm assumes that the sensor
 * is exposed to outdoor fresh air at 400 ppm CO₂ concentration at least once
 * per week of accumulated operation using one of the following measurement
 * modes for at least 4 hours without interruption at a time: periodic
 * measurement mode, low power periodic measurement mode or single shot mode
 * with a measurement interval of 5 minutes (SCD41 only).
 *
 * @param[in] asc_enabled 1 enables ASC, 0 disables ASC.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 *
 * Example:
 * --------
 *
 * @code{.c}
 *
 *     int16_t local_error = 0;
 *     local_error = scd4x_set_automatic_self_calibration_enabled(1);
 *     if (local_error != NO_ERROR) {
 *         return local_error;
 *     }
 *
 * @endcode
 *
 */
int16_t scd4x_set_automatic_self_calibration_enabled(uint16_t asc_enabled);

/**
 * @brief Check if automatic self calibration (ASC) is enabled.
 *
 * @param[out] asc_enabled 1 if ASC is enabled, 0 if ASC is disabled.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_automatic_self_calibration_enabled(uint16_t* asc_enabled);

/**
 * @brief Set the value of ASC baseline target in ppm.
 *
 * Sets the value of the ASC baseline target, i.e. the CO₂ concentration in ppm
 * which the ASC algorithm will assume as lower-bound background to which the
 * SCD4x is exposed to regularly within one ASC period of operation. To save the
 * setting to the EEPROM, the persist_settings command must be issued
 * subsequently. The factory default value is 400 ppm.
 *
 * @param[in] asc_target ASC baseline value in ppm CO₂
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 *
 * Example:
 * --------
 *
 * @code{.c}
 *
 *     int16_t local_error = 0;
 *     local_error = scd4x_set_automatic_self_calibration_target(400);
 *     if (local_error != NO_ERROR) {
 *         return local_error;
 *     }
 *
 * @endcode
 *
 */
int16_t scd4x_set_automatic_self_calibration_target(uint16_t asc_target);

/**
 * @brief Reads out the ASC baseline target concentration parameter.
 *
 * @param[out] asc_target ASC baseline target concentration parameter in ppm
 * CO₂.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_automatic_self_calibration_target(uint16_t* asc_target);

/**
 * @brief Start a low-power periodic measurement (interval 30 s).
 *
 * To enable use-cases with a constrained power budget, the SCD4x features a low
 * power periodic measurement mode with a signal update interval of
 * approximately 30 seconds. The low power periodic measurement mode is
 * initiated using the start_low_power_periodic_measurement command and read-out
 * in a similar manner as the periodic measurement mode using the
 * read_measurement command. To periodically check whether a new measurement
 * result is available for read out, the get_data_ready_status command can be
 * used to synchronize to the sensor's internal measurement interval as an
 * alternative to relying on the ACK/NACK status of the
 * read_measurement_command.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_start_low_power_periodic_measurement();

/**
 * @brief Read if data is ready.
 *
 * Polls the sensor for whether data from a periodic or single shot measurement
 * is ready to be read out.
 *
 * @param[out] data_ready_status If one or more of the 11 least significant bits
 * are 1, then the data is ready.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_data_ready_status_raw(uint16_t* data_ready_status);

/**
 * @brief Store volatile sensor settings in the EEPROM.
 *
 * Configuration settings such as the temperature offset, sensor altitude and
 * the ASC enabled/disabled parameters are by default stored in the volatile
 * memory (RAM) only. The persist_settings command stores the current
 * configuration in the EEPROM of the SCD4x, ensuring the current settings
 * persist after power-cycling. To avoid unnecessary wear of the EEPROM, the
 * persist_settings command should only be sent following configuration changes
 * whose persistence is required. The EEPROM is guaranteed to withstand at least
 * 2000 write cycles. Note that field calibration history (i.e. FRC and ASC) is
 * automatically stored in a separate EEPROM dimensioned for the specified
 * sensor lifetime when operated continuously in either periodic measurement
 * mode, low power periodic measurement mode or single shot mode with 5 minute
 * measurement interval (SCD41 only).
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_persist_settings();

/**
 * @brief Read the sensor's unique serial number.
 *
 * Reading out the serial number can be used to identify the chip and to verify
 * the presence of the sensor. The get_serial_number command returns 3 words,
 * and every word is followed by an 8-bit CRC checksum. Together, the 3 words
 * constitute a unique serial number with a length of 48 bits (in big endian
 * format).
 *
 * @param[out] serial_number 48-bit unique serial number of the sensor.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_serial_number(uint16_t* serial_number,
                                uint16_t serial_number_size);

/**
 * @brief Perform self test to assess sensor functionality and power supply.
 *
 * Can be used as an end-of-line test to check the sensor functionality.
 *
 * @param[out] sensor_status If sensor status is equal to 0, no malfunction has
 * been detected.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_perform_self_test(uint16_t* sensor_status);

/**
 * @brief Perform factory reset to erase the settings stored in the EEPROM.
 *
 * The perform_factory_reset command resets all configuration settings stored in
 * the EEPROM and erases the FRC and ASC algorithm history.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_perform_factory_reset();

/**
 * @brief Reinitialize the sensor by reloading the settings from the EEPROM.
 *
 * The reinit command reinitialize the sensor by reloading user settings from
 * EEPROM. The sensor must be in the idle state before sending the reinit
 * command. If the reinit command does not trigger the desired
 * re-initialization, a power-cycle should be applied to the SCD4x.
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_reinit();

/**
 * @brief Reads out the SCD4x sensor variant.
 *
 * @param[out] sensor_variant Bits[15…12] = 0000 → SCD40 Bits[15…12] = 0001 →
 * SCD41
 *
 * @note This command is only available in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_sensor_variant_raw(uint16_t* sensor_variant);

/**
 * @brief On-demand measurement of the CO₂ concentration, temperature, and
 * humidity.
 *
 * The sensor output is read out by using the read_measurement command. The
 * fastest possible sampling interval for single shot measurements is 5 seconds.
 * The ASC is enabled by default in single shot operation and optimized for
 * single shot measurements performed every 5 minutes. For more details about
 * single shot measurements and optimization of power consumption please refer
 * to the datasheet.
 *
 * @note This command is only available for SCD41.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_measure_single_shot();

/**
 * @brief On-demand measurement of the temperature and humidity only.
 *
 * For more details about single shot measurements and optimization of power
 * consumption please refer to the datasheet.
 *
 * @note This command is only available for SCD41.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_measure_single_shot_rht_only();

/**
 * @brief Put the sensor into sleep mode from idle mode.
 *
 * Put the sensor from idle to sleep to reduce power consumption. Can be used to
 * power down when operating the sensor in power-cycled single shot mode.
 *
 * @note This command is only available in idle mode. Only for SCD41.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_power_down();

/**
 * @brief Wake up sensor from sleep mode to idle mode.
 *
 * Wake up the sensor from sleep mode into idle mode. Note that the SCD4x does
 * not acknowledge the wake_up command. The sensor's idle state after wake up
 * can be verified by reading out the serial number.
 *
 * @note This command is only available for SCD41.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_wake_up();

/**
 * @brief Sets the initial period for ASC correction
 *
 * Sets the duration of the initial period for ASC correction (in hours). By
 * default, the initial period for ASC correction is 44 hours. Allowed values
 * are integer multiples of 4 hours. A value of 0 results in an immediate
 * correction. To save the setting to the EEPROM, the persist_settings command
 * must be issued.
 *
 * For single shot operation, this parameter always assumes a measurement
 * interval of 5 minutes, counting the number of single shots to calculate
 * elapsed time. If single shot measurements are taken more / less frequently
 * than once every 5 minutes, this parameter must be scaled accordingly to
 * achieve the intended period in hours (e.g. for a 10-minute measurement
 * interval, the scaled parameter value is obtained by multiplying the intended
 * period in hours by 0.5).
 *
 * @param[in] asc_initial_period ASC initial period in hours
 *
 * @note This command is available for SCD41 and only in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 *
 * Example:
 * --------
 *
 * @code{.c}
 *
 *     int16_t local_error = 0;
 *     local_error = scd4x_set_automatic_self_calibration_initial_period(44);
 *     if (local_error != NO_ERROR) {
 *         return local_error;
 *     }
 *
 * @endcode
 *
 */
int16_t scd4x_set_automatic_self_calibration_initial_period(
    uint16_t asc_initial_period);

/**
 * @brief Read out the initial period for ASC correction
 *
 * @param[out] asc_initial_period ASC initial period in hours
 *
 * @note This command is only available for SCD41 and only in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_automatic_self_calibration_initial_period(
    uint16_t* asc_initial_period);

/**
 * @brief Sets the standard period for ASC correction.
 *
 * Sets the standard period for ASC correction (in hours). By default, the
 * standard period for ASC correction is 156 hours. Allowed values are integer
 * multiples of 4 hours. Note: a value of 0 results in an immediate correction.
 * To save the setting to the EEPROM, the persist_settings (see Section 3.10.1)
 * command must be issued.
 *
 * For single shot operation, this parameter always assumes a measurement
 * interval of 5 minutes, counting the number of single shots to calculate
 * elapsed time. If single shot measurements are taken more / less frequently
 * than once every 5 minutes, this parameter must be scaled accordingly to
 * achieve the intended period in hours (e.g. for a 10-minute measurement
 * interval, the scaled parameter value is obtained by multiplying the intended
 * period in hours by 0.5).
 *
 * @param[in] asc_standard_period ASC standard period in hours
 *
 * @note This command is only available for SCD41 and only in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 *
 * Example:
 * --------
 *
 * @code{.c}
 *
 *     int16_t local_error = 0;
 *     local_error = scd4x_set_automatic_self_calibration_standard_period(156);
 *     if (local_error != NO_ERROR) {
 *         return local_error;
 *     }
 *
 * @endcode
 *
 */
int16_t scd4x_set_automatic_self_calibration_standard_period(
    uint16_t asc_standard_period);

/**
 * @brief Get the standard period for ASC correction.
 *
 * @param[out] asc_standard_period ASC standard period in hours
 *
 * @note This command is only available for SCD41 and only in idle mode.
 *
 * @return error_code 0 on success, an error code otherwise.
 */
int16_t scd4x_get_automatic_self_calibration_standard_period(
    uint16_t* asc_standard_period);

#ifdef __cplusplus
}
#endif
#endif  // SCD4X_I2C_H
