/*
 * PressureSensorTest
 *
 * Fetch and print values from a Honeywell 
 * Pressure Sensor over I2C using an Arduino
 * 
 * The sensor values used in this demo are 
 * for a -40 to 40 psi/mbar/bar gauge pressure sensor.
 * The unit depends on your sensor.
 */

#include <HoneywellPressureSensorI2C.h>

#define I2C_ADDRESS 0x28    // this is the default address of honeywell i2c pressure sensors
#define I2C_FREQ 400000		// this is the maximum I2C frequency supported by most sensors

/*
 * HoneywellPressureSensorI2C sensor( min_pressure, max_pressure, i2c_address, i2c_frequency );
 * min_pressure [psi/mbar/bar]:	The minimum calibrated output pressure. 
 * max_pressure [psi/mbar/bar]:	The maximum calibrated output pressure.
 * I2C_ADDRESS [hex or dec]:	I2C address of sensor. Default is 0x28. Other addresses are 0x38, 0x48, 0x58, 0x68, 0x78, 0x88, 0x98
 * I2C_FREQ [Hz]: 				I2C clock frequency. Sensors work from 100000 Hz to 400000 Hz (see data sheet of sensor for details).
 */
HoneywellPressureSensorI2C sensor( -40.0, 40.0, I2C_ADDRESS, I2C_FREQ );

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
