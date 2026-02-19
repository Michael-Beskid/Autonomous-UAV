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

#include "Arduino.h"

class LaserAltimeter {
public:
	
	LaserAltimeter();

	void init();
	float getAltitude();
	void readAltitude();
	void printAltitude();

private:
    float altitude;

};

#endif