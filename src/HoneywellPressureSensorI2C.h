#ifndef __HONEYWELL_TRUSTABILITY_I2C_H__
#define __HONEYWELL_TRUSTABILITY_I2C_H__

#include <Wire.h>

/*!
 * @file HoneywellTruStabilityI2C.h
 *
 * @mainpage Honeywell TruStability HSC and SSC digital pressure sensor SPI driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for the Honeywell TruStability
 * HSC and SSC and Honeywell Basic ABP driver for the Arduino platform.
 * It is designed based on the <a href="https://sensing.honeywell.com/i2c-comms-digital-output-pressure-sensors-tn-008201-3-en-final-30may12.pdf">
 * Honeywell technical note</a> for this product.
 *
 * @section dependencies Dependencies
 *
 * This library depends on the <a href="https://www.arduino.cc/en/Reference/Wire"> 
 * Arduino Wire library</a> included in a standard Arduino installation.
 *
 * @section author Author
 *
 * Written by Erik Werner for the Hui Lab (SPI version).
 * Adapted by Lukas Hirschwald for RWTH Aachen University (I2C version).
 *
 * @section license License
 *
 * MIT license
 *
 */

/********** Sensor Configuration *****************/
/* Values taken from honeywell datasheet        */

// Default values are for 10 - 90 % calibrarion */
const float MIN_COUNT = 1638.4;  ///< 1638 counts (10% of 2^14 counts or 0x0666)
const float MAX_COUNT = 14745.6; ///< 14745 counts (90% of 2^14 counts or 0x3999)
/********** Sensor Configuration *****************/

/**************************************************************************/
/*!
    @brief  Class for reading temperature and pressure from a Honeywell TruStability HSC, SSC or ABP sensor
*/
/**************************************************************************/
class TruStabilityPressureSensor
{
    const float _MIN_PRESSURE; ///< minimum calibrated output pressure (10%), in any units
    const float _MAX_PRESSURE; ///< maximum calibrated output pressure (90%), in any units

	const uint8_t _i2c_address;
	const uint32_t _i2c_frequency;

    uint8_t _buf[4];            ///< buffer to hold sensor data
    uint8_t _status = 0;        ///< byte to hold status information.
    // Status codes:
    // status = 0 : normal operation
    // status = 1 : device in command mode
    // status = 2 : stale data
    // status = 3 : diagnostic condition

    int _pressure_count = 0;    ///< hold raw pressure data (14 - bit, 0 - 16384)
    int _temperature_count = 0; ///< hold raw temperature data (11 - bit, 0 - 2048)

  public:
    /**************************************************************************/
    /*!
    @brief  Constructs a new presssure sensor object. min_pressure an max_pressure are
    taken from the datasheet and represent the 10% and 90% calibrated output pressures.
    Subsequent calls to pressure() will return values in the
    units of min_pressure and max_pressure

    @param    min_pressure
              the minimum calibrated output pressure
    @param    max_pressure
              the maximum calibrated output pressure
    @param    i2c_address
              I2C address of sensor. Default ist 0x28
	@param	  i2c_frequency
			  I2C clock frequency. Honeywell sensors work from 100kHz to 400kHz. Default is 400kHz
    */
    /**************************************************************************/
    TruStabilityPressureSensor(const float min_pressure, const float max_pressure, const uint8_t i2c_address=0x28, const uint32_t i2c_frequency=400000)
    : _MIN_PRESSURE(min_pressure), _MAX_PRESSURE(max_pressure), _i2c_address(i2c_address), _i2c_frequency(i2c_frequency) {}

    /**************************************************************************/
    /*!
    @brief  Initializes a pressure sensor object.
            This function must be called in the Arduino setup() function.
    */
    /**************************************************************************/
    void begin()
    {
        Wire.begin(_i2c_address);
    }

    /**************************************************************************/
    /*!
    @brief  Polls the sensor for new data. The raw temperature and
    pressure variables are updated. There is no guarantee that the data retrieved
    from the sensor is fresh data. Check the status

    @return   The status of the sensor
    0 indicates normal operation,
    1 indicates the device is in command mode,
    2 indicates stale data,
    3 indicates a diagnostic condition
    */
    /**************************************************************************/
    uint8_t readSensor()
    {
        uint8_t count = 4; // transfer 4 bytes (the last two are only used by some sensors)
        memset(_buf, 0x00, count); // probably not necessary, sensor is half-duplex
		Wire.requestFrom(_i2c_address, count);
		while(Wire.available())
		{
			_buf[] = Wire.read();
		}

        _status = _buf[0] >> 6 & 0x3;

        // if device is normal and there is new data, bitmask and save the raw data
        if (_status == 0)
        {
            // 14 - bit pressure is the last 6 bits of byte 0 (high bits) & all of byte 1 (lowest 8 bits)
            _pressure_count = ( (uint16_t)(_buf[0]) << 8 & 0x3F00 ) | ( (uint16_t)(_buf[1]) & 0xFF );
            // 11 - bit temperature is all of byte 2 (lowest 8 bits) and the first three bits of byte 3
            _temperature_count = ( ((uint16_t)(_buf[2]) << 3) & 0x7F8 ) | ( ( (uint16_t)(_buf[3]) >> 5) & 0x7 );
        }

        return _status;
    }
    /**************************************************************************/
    /*!
    @brief  Read the most recent status information for the sensor
        This value is updated by readSensor()

    @return  The most recent status information
    0 indicates normal operation,
    1 indicates the device is in command mode,
    2 indicates stale data,
    3 indicates a diagnostic condition
    */
    /**************************************************************************/
    uint8_t status() const {return _status;}

    /**************************************************************************/
    /*!
    @brief  Read the most recently polled pressure value.
        Update this value by calling readSensor() before reading.

    @return  The pressure value from the most recent reading in raw counts
    */
    /**************************************************************************/
    int rawPressure() const { return _pressure_count; }

    /**************************************************************************/
    /*!
    @brief  Read the most recently polled temperature value.
        Update this value by calling readSensor() before reading.

    @return  The temperature value from the most recent reading in raw counts
    */
    /**************************************************************************/
    int rawTemperature() const { return _temperature_count; }

    /**************************************************************************/
    /*!
    @brief  Read the most recently polled pressure value converted to the units
        specified in the constructor (minimum and maximum calibrated output
        values). Update this value by calling readSensor() before reading.
        To avoid using floating point number, /sa rawPressure.

    @return  The pressure value from the most recent reading in units
    */
    /**************************************************************************/
    float pressure() const { return countsToPressure(_pressure_count, _MIN_PRESSURE, _MAX_PRESSURE); }

    /**************************************************************************/
    /*!
    @brief  Read the most recently polled temperature value in degress celcius.
        Update this value by calling readSensor() before reading.
        To avoid using floating point number, /sa rawPressure.

    @return  The temperature value from the most recent reading in degrees C
    */
    /**************************************************************************/
    float temperature() const { return countsToTemperatures(_temperature_count); }

    /**************************************************************************/
    /*!
    @brief  Converts a digital pressure measurement in counts to pressure.
            The output temperature will be in the units of min_pressure and max_pressure.
            This is a helper function to pressure()
    @param    counts
              The raw pressure value
    @param    min_pressure
              The minimum calibrated output pressure for the sensor, in units of choice
    @param    max_pressure
              The maximum calibrated output pressure for the sensor, in units of choice
    @return Pressure value in units of choice
    */
    /**************************************************************************/
    static float countsToPressure(const int counts, const float min_pressure, const float max_pressure)
    {
        return ((((float)counts - MIN_COUNT) * (max_pressure - min_pressure)) / (MAX_COUNT - MIN_COUNT)) + min_pressure;
    }

    /**************************************************************************/
    /*!
    @brief  Converts a digital temperature measurement in counts to temperature in C.
            This is a helper function to temperature()
    @param    counts
              The raw temperature value
    @return Temperature value in degrees C
    */
    /**************************************************************************/
    static float countsToTemperatures(const int counts)
    {
        return (((float)counts / 2047.0) * 200.0) - 50.0;
    }
};

#endif // End __HONEYWELL_TRUSTABILITY_I2C_H__ include guard
