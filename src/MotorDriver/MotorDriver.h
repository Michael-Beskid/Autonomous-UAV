/**
 * @file MotorDriver.h
 *
 * @brief Header file for MotorDriver class.
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#ifndef _MOTORDRIVER_H_
#define _MOTORDRIVER_H_

#include "Arduino.h"
#include <PWMServo.h>   // Commanding any extra actuators, installed with teensyduino installer

class MotorDriver {
public:
	
	MotorDriver();
    
    void init();
    void setMotorCommands(float m1, float m2, float m3, float m4);
    void scaleCommands();
    void commandMotors();
    void armMotors();
    void throttleCut();
    void printMotorCommands();
    void printMotorCommandsScaled();

private:
    static const int m1Pin, m2Pin, m3Pin, m4Pin;
    float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
    int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;

};

#endif