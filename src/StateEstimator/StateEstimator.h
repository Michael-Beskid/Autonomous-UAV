/**
 * @file StateEstimator.h
 *
 * @brief Header file for StateEstimator class.
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#ifndef _StateEstimator_H_
#define _StateEstimator_H_

#include "Arduino.h"

class StateEstimator {
public:
	
	StateEstimator();

	void init();
	float getStateVector();
	void updateFilter();
	void printAttitudeEstimate();
    void printFullStateEstimate();

private:
    float x;
    float y;
    float z;
    float theta;
    float phi;
    float psi;
    float dx;
    float dy;
    float dz;
    float dtheta;
    float dphi;
    float dpsi;

};

#endif