#ifndef __HONEYWELL_TRUSTABILITY_SPI_H__
#define __HONEYWELL_TRUSTABILITY_SPI_H__

#include <SPI.h>

/*!
 * @file HoneywellTruStabilitySPI.h
 *
 * @mainpage Honeywell TruStability HSC and SSC digital pressure sensor SPI driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for the Hui Lab's Honeywell TruStability 
 * HSC and SSC driver for the Arduino platform.  It is designed based on
 * the <a href="https://sensing.honeywell.com/spi-comms-digital-ouptu-pressure-sensors-tn-008202-3-en-final-30may12.pdf">
 * Honeywell technical note</a> for this product.
 *
 * @section dependencies Dependencies
 *
 * This library depends on the  <a href="https://www.arduino.cc/en/Reference/SPI">
 * Arduino SPI library</a>, included in a standard Arduino installation.
 *
 * @section author Author
 *
 * Written by Erik Werner for the Hui Lab.
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
    @brief  Class for reading temperature and pressure from a Honeywell TruStability HSC or SSC sensor
*/
/**************************************************************************/
class TruStability_PressureSensor
{
    const uint8_t _SS_PIN;     ///< slave select pin (active low)
    const float _MIN_PRESSURE; ///< minimum calibrated output pressure (10%), in any units
    const float _MAX_PRESSURE; ///< maximum calibrated output pressure (90%), in any units

    SPISettings _spi_settings;
    uint8_t _buf[4];            ///< buffer to hold sensor data
    int _pressure_count = 0;    ///< hold raw pressure data (14 - bit, 0 - 16384)
    int _temperature_count = 0; ///< hold raw temperature data (11 - bit, 0 - 2048)

  public:
    /**************************************************************************/
    /*!
    @brief  Constructs a new presssure sensor object.
            The constructor defaults SPI settings to use 800 KHz SPI
*/
    /**************************************************************************/
    TruStability_PressureSensor(uint8_t pin, float min_pressure, float max_pressure, SPISettings spi_settings = SPISettings(800000, MSBFIRST, SPI_MODE0))
        : _SS_PIN(pin), _MIN_PRESSURE(min_pressure), _MAX_PRESSURE(max_pressure), _spi_settings(spi_settings) {}

    /**************************************************************************/
    /*!
    @brief  Initializes a pressure sensor object.
            This function must be called in the Arduino setup() function
*/
    /**************************************************************************/
    void begin()
    {
        pinMode(_SS_PIN, OUTPUT);
        digitalWrite(_SS_PIN, HIGH);
        SPI.begin();
    }

    /**************************************************************************/
    /*!
    @brief  Polls the sensor for new data.
            The raw temperature and pressure variables are updated
    @return   The status of the sensor. 0 is normal operation.
*/
    /**************************************************************************/
    uint8_t readSensor()
    {
        SPI.beginTransaction(_spi_settings);
        digitalWrite(_SS_PIN, LOW);
        _buf[0] = SPI.transfer(0x00);
        _buf[1] = SPI.transfer(0x00);
        _buf[2] = SPI.transfer(0x00);
        _buf[3] = SPI.transfer(0x00);
        digitalWrite(_SS_PIN, HIGH);
        SPI.endTransaction();

        byte status = _buf[0] >> 6;
        // Status codes:
        // status = 0 : normal operation
        // status = 1 : device in command mode
        // status = 2 : stale data
        // status = 3 : diagnostic condition

        // if device is normal and there is new data, bitmask and save the raw data
        if (status == 0)
        {
            // 14 - bit pressure is the last 6 bits of byte 1 (high bits) & all of byte 2 (lowest 8 bits)
            _pressure_count = (((int)(_buf[0] & 0x3f)) << 8) | ((int)(_buf[1]));
            // 11 - bit temperature is all of byte 3 (lowest 8 bits) and the first three bites of byte 4
            _temperature_count = ((((int)_buf[2]) << 8) | (int)_buf[3]) >> 5;
        }
        return status;
    }

    // read the most recently polled
    /**************************************************************************/
    /*!
    @brief  Read the most recently polled pressure value.
        Update this value by calling readSensor() before reading.
    
    @return  The pressure value from the most recent reading in raw counts
*/
    /**************************************************************************/
    int rawPressure() { return _pressure_count; }

    /**************************************************************************/
    /*!
    @brief  Read the most recently polled temperature value.
        Update this value by calling readSensor() before reading.
    
    @return  The temperature value from the most recent reading in raw counts
*/
    /**************************************************************************/
    int rawTemperature() { return _temperature_count; }

    /**************************************************************************/
    /*!
    @brief  Read the most recently polled pressure value converted to units
        Update this value by calling readSensor() before reading.
    
    @return  The pressure value from the most recent reading in units
*/
    /**************************************************************************/
    float pressure() { return countsToPressure(_pressure_count, _MIN_PRESSURE, _MAX_PRESSURE); }

    /**************************************************************************/
    /*!
    @brief  Read the most recently polled temperature value.
        Update this value by calling readSensor() before reading.
    
    @return  The temperature value from the most recent reading in degrees C
*/
    /**************************************************************************/

    float temperature() { return countsToTemperatures(_temperature_count); }

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
    static float countsToPressure(int counts, float min_pressure, float max_pressure)
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
    static float countsToTemperatures(int counts)
    {
        return (((float)counts / 2047.0) * 200.0) - 50.0;
    }
    /*
    static float rawToPressure(int raw, int rawMin, int rawMax, float pMin, float pMax) {
      return (float(raw - rawMin) * (pMax - pMin)) / (rawMax - rawMin) + pMin;
    }
    
    static float rawToTemperature(int raw) {
      return float(raw) * 200.0 / 2047 - 50.0;
    }*/
};

#endif // End __HONEYWELL_TRUSTABILITY_SPI_H__ include guard