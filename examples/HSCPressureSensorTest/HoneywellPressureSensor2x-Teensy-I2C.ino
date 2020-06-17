/*
 * PressureSensorTest
 *
 * Fetch and print values from two Honeywell 
 * Pressure Sensors over I2C using a Teensy board.
 *
 * The Teensy has two I2C ports which you can use to
 * fetch values from two sensors that have the same
 * i2c address.
 * 
 * The sensor values used in this demo are 
 * for a -40 to 40 psi/mbar/bar gauge pressure sensor.
 * The unit depends on your sensor.
 */

#include <HoneywellPressureSensor-Teensy-I2C.h>

#define I2C_ADDRESS 0x28    // this is the default address of honeywell i2c pressure sensors
TruStabilityPressureSensor sensor( -40.0, 40.0 );

void setup() {
  Serial.begin(115200);   // start Serial communication
  sensor.begin();         // run sensor initialization
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
