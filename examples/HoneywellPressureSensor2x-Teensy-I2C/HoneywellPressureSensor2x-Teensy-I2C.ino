/*
 * PressureSensorTest
 *
 * Fetch and print values from two Honeywell 
 * Pressure Sensors over I2C using a Teensy 3.x/LC board.
 *
 * The Teensy has two (or more, depends on version) I2C ports which you can use to
 * fetch values from two sensors that have the same i2c address.
 * 
 * The sensor values used in this demo are 
 * for a -40 to 40 psi/kPa/mbar/bar differential pressure sensor
 * and a 0 to 160 psi/kPa/mbar/bar gage pressure sensor.
 * The unit depends on your sensor.
 */

#include <HoneywellPressureSensorI2CTeensy.h>

#define I2C_ADDRESS_1 0x28    // this is the default address of honeywell i2c pressure sensors
#define I2C_ADDRESS_2 0x28    // this is the default address of honeywell i2c pressure sensors

/* sensor(min_pressure, max_pressure, sda_pin, scl_pin, wire_port, i2c_address, i2c_frequency)
 * min_pressure: the minimum calibrated output pressure [psi/kPa/mbar/bar]
 * max_pressure: the maximum calibrated output pressure [psi/kPa/mbar/bar]
 * sda_pin: pin number of SDA pin [default 18] (see pinout to find working pins, or https://github.com/nox771/i2c_t3#pins)
 * scl_pin: pin number of SCL_pin [default 19]
 * wire_port: number of i2c port to use [default 1, can be 1, 2, 3, 4] see https://github.com/nox771/i2c_t3#pins
 * i2c_address: address of sensor [default 0x28, 7bit address]
 * i2c_frequency: honeywell pressure sensors work from 100000 Hz to 400000 Hz [default 400000 Hz]    */
HoneywellPressureSensorI2CTeensy sensor1( -40.0, 40.0, 18, 19, 1, 0x28, 400000 );  // differential pressure sensor with +-40 psi/kPa/mbar/bar
HoneywellPressureSensorI2CTeensy sensor2( 0.0, 160.0, 30, 29, 2, 0x28, 400000 );     // gage pressure sensor with 0 to 160 psi/kPa/mbar/bar

void setup() {
  Serial.begin(115200);   // start Serial communication
  sensor1.begin();         // run sensor initialization
  sensor2.begin();         // run sensor initialization
}

void loop() {
  // the sensor returns 0 when new data is ready
  if( sensor.readSensor() == 0 ) {
    Serial.print( "temp [C]: " );
    Serial.print( sensor.temperature() );
    Serial.print( "\t pressure [psi/mbar/bar]: " );
    Serial.println( sensor.pressure() );
  }
  
  delay( 500 ); // Slow down sampling to 2 Hz. This is just a test.

}
