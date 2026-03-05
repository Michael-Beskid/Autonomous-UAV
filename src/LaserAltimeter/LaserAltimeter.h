/**
 * @file LaserAltimeter.h
 *
 * @brief Header file for LaserAltimeter class.
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#ifndef _LaserAltimeter_H_
#define _LaserAltimeter_H_

#include <Arduino.h>
#include "TFMini-Plus/TFMPlus.h"

class LaserAltimeter {
public:
	
	LaserAltimeter();

	void init();
	void pollSensorData();
	float getAltitude();
	void printAltitude();

private:
    float altitude;
	int16_t tfDist = 0;    // Distance to object in centimeters
	int16_t tfFlux = 0;    // Strength or quality of return signal
	int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip

};

#endif