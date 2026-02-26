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
#include "Common.h"

class StateEstimator {
public:
	
	StateEstimator();

	void init();
	StateVector getStateVector();
    void predictEKF();
	void updateEKF();
    void updateMadgwick(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
	void printAttitudeEstimate();
    void printFullStateEstimate();

private:

    float invSqrt(float x);
    
    // State vector for publishing
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

    // Initialize quaternion for madgwick filter
    float q0 = 1.0f; 
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    // Madgwick filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
    float B_madgwick = 0.04;  //Madgwick filter parameter

};

#endif