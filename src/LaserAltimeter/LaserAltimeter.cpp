/**
 * @file LaserAltimeter.cpp
 *
 * @brief LaserAltimeter class to interface with TFmini Plus Micro Range Finder Sensor.
 * 
 * Wiring:
 *   Red:
 *   White:
 *   Green:
 *   Black:
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#include "LaserAltimeter.h"

LaserAltimeter::LaserAltimeter() {}

/**
 * @brief Initialize the laser rangefinder sensor.
 */
void LaserAltimeter::init() {
    // TODO: Implement any initial setup for the sensor
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
 * @brief Read the laser rangefinder sensor.
 *
 *  Reads the latest altitude measurement from the sensor
 *    and updates the current stored altitude value.
 */
void LaserAltimeter::readAltitude() {
    // TODO: Implement method to poll sensor data
}

/**
 * @brief Print the current altitude in [cm] to the Serial monitor.
 */
void LaserAltimeter::printAltitude() {
   Serial.print(F("Altitude: "));
   Serial.print(altitude / 10.0);
   Serial.println(F(" cm"));
}