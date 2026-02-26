/**
 * @file Navigation.h
 *
 * @brief Header file for Navigation
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_

#include <Arduino.h>
#include "Common.h"

class Navigation {
public:
	
	Navigation();
    
  void printFlightMode();
  void printDesiredState();
  DesiredState getDesiredState(RadioCommands radioCommands);

private:

  // Desired state
  DesiredState desiredState;

  // Position controller variables
  float error_altitude, error_altitude_prev, altitude_des_prev, integral_altitude, integral_altitude_prev, derivative_altitude, altitude_PID = 0;
  float error_posX, error_posY, posX_control, posY_control = 0;

  // Motion planning
  const float POSITION_DB_RADIUS = 1.00; // Deadband radius for evaluating reached position targets [meters]
  const int ALTITUDE_DB_RADIUS = 250; // Deadband radius for evaluating altitude targets [milimetetrs]
  float altitude_des = 0.0; // mm
  float target_posX = 0.0; // meters
  float target_posY = 0.0; // meters

  // Angle and angular rate limits
  float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
  float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
  float maxYaw = 160.0;     //Max yaw rate in deg/sec

  // Altitude controller variables
  float hover_throttle = 0.525;       //Baseline throttle for hovering
  float Kp_altitude = 1.0;            //Altitude P-gain
  float Ki_altitude = 0.2;            //Altitude I-gain
  float Kd_altitude = 0.5;            //Altitude D-gain
  float i_limit_altitude = 10000.0;   //Integrator saturation level

  // Position controller variables
  float Kp_position = 0.05;  // Full angle at 20m away from target

  // Flight modes enumeration
  enum flightModes {
    MANUAL,
    AUTONOMOUS
  };

  enum flightModes flightMode;

  // Manual states enumeration
  enum manualStates {
    MANUAL_STARTUP,
    NORMAL,
  };

  enum manualStates manualState;

  // Autonomous states enumeration
  enum autoStates {
    AUTO_STARTUP, // Initilaization
    TAKEOFF,      // Take off vertically from start position to an altitude of 1.5 meters
    FORWARD,      // Translate forward a distance of 5.0 meters
    LAND,         // Land on the ground surface
    STOP          // End autonomous mission
  };

  enum autoStates missionState;

  void getDesStateManual(RadioCommands radioCommands);

};

#endif


