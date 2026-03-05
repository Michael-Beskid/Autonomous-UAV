/**
 * @file LaserAltimeter.cpp
 *
 * @brief LaserAltimeter class to interface with TFmini Plus Micro Range Finder Sensor.
 * 
 * Wiring:
 *   Red: +5V
 *   White: RX3 (pin 15)
 *   Green: TX3 (pin 14)
 *   Black: GND
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#include "LaserAltimeter.h"

TFMPlus tfmP; 

LaserAltimeter::LaserAltimeter() {}

/**
 * @brief Initialize the laser rangefinder sensor.
 */
void LaserAltimeter::init() {

    Serial3.begin(115200);  // Initialize TFMPLus device serial port.
    delay(20);               // Give port time to initalize
    bool resp = tfmP.begin( &Serial3);   // Initialize device library object and...
                             // pass device serial port to the object.
    if (!resp) {
        Serial.print("Failed to connect to laser altimeter.");
    }
  
    delay(500);  // added to allow the System Rest enough time to complete

    // - - Set the data frame-rate to 20Hz - - - - - - - -
    if( tfmP.sendCommand( SET_FRAME_RATE, FRAME_20))
    {
        //printf( "%2uHz.\r\n", FRAME_20);
    }
    else tfmP.printReply();
    // - - - - - - - - - - - - - - - - - - - - - - - -

    delay(500);            // And wait for half a second.
}

/**
 * @brief Read the laser rangefinder sensor.
 *
 *  Reads the latest altitude measurement from the sensor
 *    and updates the current stored altitude value.
 */
void LaserAltimeter::pollSensorData() {
    // TODO: Implement method to poll sensor data
    tfmP.getData(tfDist, tfFlux, tfTemp);
    altitude = static_cast<float>(tfDist);
}

/**
 * @brief Get the current altitude.
 *
 * @returns most recent altitude reading in milimeters.
 */
float LaserAltimeter::getAltitude() {
   return altitude;
}

/**
 * @brief Print the current altitude in [cm] to the Serial monitor.
 */
void LaserAltimeter::printAltitude() {
   Serial.print(F("Altitude: "));
   Serial.print(altitude);
   Serial.println(F(" cm"));
}