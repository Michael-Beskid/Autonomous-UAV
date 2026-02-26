/**
 * @file AttitudeController.h
 *
 * @brief Header file for attitude controller class.
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#ifndef _ATTITUDECONTROLLER_H_
#define _ATTITUDECONTROLLER_H_

#include "Arduino.h"
#include "Common.h"

class AttitudeController {
public:
	
	AttitudeController();

    MotorSpeeds calcMotorSpeeds(DesiredState desiredState, StateVector stateVector, float dt, bool low_throttle);
    void printPIDoutput();

private:
    MotorSpeeds controlMixer(float thro_des);
    void controlANGLE(DesiredState desiredState, StateVector stateVector, float dt, bool low_throttle);

    float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
    float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
    float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

    // Stabilization controller variables
    float i_limit = 25.0;         //Integrator saturation level, mostly for safety (default 25.0)
    float Kp_roll_angle = 0.2;    //Roll P-gain
    float Ki_roll_angle = 0.3;    //Roll I-gain
    float Kd_roll_angle = 0.05;   //Roll D-gain
    float Kp_pitch_angle = 0.2;   //Pitch P-gain
    float Ki_pitch_angle = 0.3;   //Pitch I-gain
    float Kd_pitch_angle = 0.05;  //Pitch D-gain
    float Kp_yaw = 0.3;           //Yaw P-gain
    float Ki_yaw = 0.05;          //Yaw I-gain
    float Kd_yaw = 0.00015;       //Yaw D-gain

};

#endif