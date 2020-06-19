/*
 * PressureSensorTest
 *
 * Fetch and print values from two Honeywell 
 * Pressure Sensors over I2C using a Teensy 3.x/LC board.
 *
 * The Teensy has two (or more, depends on version) I2C ports which you can use to
 * fetch values from multiple sensors that have the same i2c address.
 * 
 * The sensor values used in this demo are 
 * for a -40 to 40 psi/kPa/mbar/bar differential pressure sensor
 * and a 0 to 160 psi/kPa/mbar/bar gage pressure sensor.
 * The unit depends on your sensor.
 */

#include <HoneywellPressureSensorI2C.h>
#include <HoneywellPressureSensorI2CWire1.h>

#define I2C_ADDRESS_1 0x28    // this is the default address of honeywell i2c pressure sensors
#define I2C_ADDRESS_2 0x28    // this is the default address of honeywell i2c pressure sensors
#define I2C_FREQ 400000       // this is the maximum I2C frequency supported by most sensors

/* sensor(min_pressure, max_pressure, i2c_address, i2c_frequency)
 * min_pressure: the minimum calibrated output pressure [psi/kPa/mbar/bar]
 * max_pressure: the maximum calibrated output pressure [psi/kPa/mbar/bar]
 * i2c_address: address of sensor [default 0x28, 7bit address]
 * i2c_frequency: honeywell pressure sensors work from 100000 Hz to 400000 Hz [default 400000 Hz]
 * 
 * The library uses the standard I2C pins of Arduino or Teensy.
 */
HoneywellPressureSensorI2C sensor1( -40.0,  40.0, I2C_ADDRESS_1, I2C_FREQ );  // differential pressure sensor with +-40 psi/kPa/mbar/bar
HoneywellPressureSensorI2CWire1 sensor2(   0.0, 160.0, I2C_ADDRESS_2, I2C_FREQ );  // gage pressure sensor with 0 to 160 psi/kPa/mbar/bar

void setup() {
  Serial.begin(115200);   // start Serial communication
  sensor1.begin();        // run sensor1 initialization
  sensor2.begin();        // run sensor2 initialization
}

void loop() {
  // the sensor returns 0 when new data is ready
  if( sensor1.readSensor() == 0 ) {
    Serial.print( "temp 1 [C]: " );
    Serial.print( sensor1.temperature() );
    Serial.print( "\t pressure 1 [psi/mbar/bar]: " );
    Serial.println( sensor1.pressure() );
  }
  delay(10);
  // the sensor returns 0 when new data is ready
  if( sensor2.readSensor() == 0 ) {
    Serial.print( "temp 2 [C]: " );
    Serial.print( sensor2.temperature() );
    Serial.print( "\t pressure 2 [psi/mbar/bar]: " );
    Serial.println( sensor2.pressure() );
  }
  
  delay( 500 ); // Slow down sampling to 2 Hz. This is just a test.

}
