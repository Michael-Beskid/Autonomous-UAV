/**
 * @file Navigation.cpp
 *
 * @brief Navigation class for autonomous waypoint navigation.
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#include "Navigation.h"

Navigation::Navigation() {}

// /**
//  * @brief Set target altitude.
//  * 
//  * @param alt Desired altitude in meters.
//  */
// void Navigation::setTargetAltitude(float alt) {
//   altitude_des = alt*1000; // convert meters to mm
// }

// /**
//  * @brief Set target (X,Y) position.
//  * 
//  * @param posX Desired X-position in meters.
//  * @param posY Desired Y-position in meters.
//  */
// void Navigation::setTargetPos(float posX, float posY) {
//   target_posX = posX;
//   target_posY = posY;
// }

// /**
//  * @brief Checks whether the vehicle has reached its target location.
//  * 
//  * During aerial operation, this function checks whether both the vehicle's
//  *   (X,Y) position and altitude are within a tolerance of the target values.
//  * 
//  * The global variable POS_DB_RADIUS can be modified to change the tolerance 
//  *   required to consider a target position "reached."
//  * 
//  * @returns 'true' if vehicle has reached target. 
//  */
// boolean Navigation::reachedTarget() {
//   return abs(target_posX - gps.getPosX()) < POSITION_DB_RADIUS 
//     && abs(target_posY - gps.getPosY()) < POSITION_DB_RADIUS
//     && abs(altitude_des - altimeter.getAltitude()) < ALTITUDE_DB_RADIUS;
// }

/**
 * @brief Print the current flight mode to the Serial monitor.
 */
void Navigation::printFlightMode() {
  Serial.print(F("Flight Mode: "));
  if (flightMode) {
    Serial.println(F("Autonomous"));
  } else {
    Serial.println(F("Manual"));
  }
}

/**
 * @brief Print desired vehicle state to the Serial monitor.
 * 
 * The desired state incudes the desired throttle, deisred roll, 
 *   desired pitch, and desired yaw values.
 */
void Navigation::printDesiredState() {
  Serial.print(F("thro_des: "));
  Serial.print(desiredState.throttle);
  Serial.print(F(" roll_des: "));
  Serial.print(desiredState.roll);
  Serial.print(F(" pitch_des: "));
  Serial.print(desiredState.pitch);
  Serial.print(F(" yaw_des: "));
  Serial.println(desiredState.yaw);
}

/**
 * @brief TODO
 */

// The outer state machine allows for toggling between flight modes, control of which is mapped to the radio transmitter
// The inner state machines for each flight mode are controlled by the computer
// The manual mode begins in STARTUP to set a few variables before transitioning into NORMAL for manual radio control
// The auto mode begins in STARTUP to set a few variables, then transitions between a sequence of pre-defined states to complete the autonomous mission
DesiredState Navigation::getDesiredState(RadioCommands radioCommands) {

  if (radioCommands.flight_mode_ch_pwm > 1500) {
    flightMode = AUTONOMOUS;
  } else {
    flightMode = MANUAL;
  }

  switch (flightMode) {

    case MANUAL:
      switch (manualState) {
        case MANUAL_STARTUP:
          missionState = AUTO_STARTUP; // Reset AUTO mode state machine
          manualState = NORMAL;
          break;
        case NORMAL:
          break;
        default:
          break;
      }

      getDesStateManual(radioCommands);
      break;

    // case AUTONOMOUS: 
    //   switch (missionState) {
    //     case AUTO_STARTUP:
    //       integral_altitude_prev = 0.0;
    //       error_altitude_prev = 0.0;
    //       motorsOff = false;
    //       manualState = MANUAL_STARTUP; // Reset MANUAL mode state machine
    //       missionState = TAKEOFF;      
    //       gps.setStartPos();
    //       setTargetAltitude(1.5);
    //       setTargetPos(0.0, 0.0);
    //       break;
    //     case TAKEOFF: // Uncomment this block when you're ready to fly a path instead of just hover
    //       // if (reachedTarget()) {
    //       //   missionState = FORWARD;
    //       //   setTargetPos(10.0, 0.0);
    //       // }
    //       break;
    //     case FORWARD:
    //       if (reachedTarget()) {
    //         missionState = LAND;
    //         setTargetAltitude(0.0);
    //       }
    //       break;
    //     case LAND:
    //       if (reachedTarget()) {
    //         missionState = STOP;
    //         motorsOff = true;
    //       }
    //       break;
    //     case STOP:
    //       break;
    //     default:
    //       break;
    //   }  

    //   getDesStateAuto();
    //   break;

    default:
      getDesStateManual(radioCommands);
      break;
  }

  return desiredState;
}

// /**
//  * @brief TODO
//  */
// void Navigation::getDesStateAuto() {

//   // PID Altitude Controller
//   error_altitude = altitude_des - altimeter.getAltitude();
//   integral_altitude = integral_altitude_prev + error_altitude*dt;
//   integral_altitude = constrain(integral_altitude, -i_limit_altitude, i_limit_altitude); //Saturate integrator to prevent unsafe buildup
//   derivative_altitude = (error_altitude - error_altitude_prev)/dt; 
//   altitude_PID = 0.00005*(Kp_altitude*error_altitude + Ki_altitude*integral_altitude - Kd_altitude*derivative_altitude); //Scaled by .00005 to bring within 0 to 1 range

//   // Update variables
//   integral_altitude_prev = integral_altitude;
//   error_altitude_prev = error_altitude;

//   // Proportional Position Controller
//   error_posX = target_posX - gps.getPosX();
//   error_posY = target_posY - gps.getPosY();

//   // If needed: Perform a rotaion based on the estiamted yaw angle so translation occurs in body-frame coordiantes to assign pitch/roll angles
//   //    Note: This shouldn't really be needed becasue the yaw is stabilized about zero... so should always be aligned except for dript which can't be controlled anyway
//   //    Would add these lines to do the rotation:
//   //      error_bodyX = error_posX*cos(imu.getYaw()) - error_posY*sin(imu.getYaw());
//   //      error_bodyY = error_posX*sin(imu.getYaw()) + error_posY*cos(imu.getYaw());
//   //    Then change the error values below to those expressed in the body frame
  
//   posX_control = Kp_position*error_posX;
//   posY_control = Kp_position*error_posY;

//   // Set desired throttle value from altitude controller
//   thro_des = hover_throttle + altitude_PID;

//   // TODO: Check conventions to confirm that X and Y are correctly mapped to pitch and roll axes

//   // Set desired roll and pitch angles from position controller
//   roll_des = posY_control ; //Between -1 and 1
//   pitch_des = posX_control; //Between -1 and 1
//   yaw_des = 0;
  
//   //Constrain within normalized bounds
//   thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
//   roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
//   pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
//   yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  
// }

/**
 * @brief Normalizes desired control values to appropriate values
 * 
 * From dRehmFlight:
 *   Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
 *   RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
 *   roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
 *   (rate mode). yaw_des is scaled to be within max yaw in degrees/sec.
 */
void Navigation::getDesStateManual(RadioCommands radioCommands) {
  
  desiredState.throttle = (radioCommands.throttle_ch_pwm - 1000.0)/1000.0; //Between 0 and 1
  desiredState.roll = (radioCommands.roll_ch_pwm  - 1500.0)/500.0; //Between -1 and 1
  desiredState.pitch = (radioCommands.pitch_ch_pwm  - 1500.0)/500.0; //Between -1 and 1
  desiredState.yaw_rate = (radioCommands.yaw_ch_pwm  - 1500.0)/500.0; //Between -1 and 1
  
  //Constrain within normalized bounds
  desiredState.throttle = constrain(desiredState.throttle, 0.0, 1.0); //Between 0 and 1
  desiredState.roll = constrain(desiredState.roll, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  desiredState.pitch = constrain(desiredState.pitch, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  desiredState.yaw_rate = constrain(desiredState.yaw_rate, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  
}