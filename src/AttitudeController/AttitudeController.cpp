/**
 * @file AttitudeController.cpp
 *
 * @brief Attitude controller class for stabilization about desired body angles.
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#include "AttitudeController.h"

AttitudeController::AttitudeController() {}

/**
 * @brief Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
 *   in preparation to be sent to the motor ESCs and servos.
 * 
 * From dRehmFlight:
 *   Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
 *   vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
 *   should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
 *   normalized (0 to 1) thro_des command for throttle control.
 *   in preparation to be sent to the motor ESCs and servos.
 */
MotorSpeeds AttitudeController::controlMixer(float thro_des) {
   
  MotorSpeeds motorSpeeds;
  motorSpeeds.m1 = thro_des - pitch_PID - roll_PID - yaw_PID; //Front right
  motorSpeeds.m2 = thro_des + pitch_PID + roll_PID - yaw_PID; //Back left
  motorSpeeds.m3 = thro_des + pitch_PID - roll_PID + yaw_PID; //Back right
  motorSpeeds.m4 = thro_des - pitch_PID + roll_PID + yaw_PID; //Front left 
  return motorSpeeds;

}

/**
 * @brief Computes control commands based on state error (angle)
 * 
 * From dRehmFlight:
 *   Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
 *   getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
 *   are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
 *   excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
 *   they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
 *   if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
 *   terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
 *   can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
 */
void AttitudeController::controlANGLE(DesiredState desiredState, StateVector stateVector, float dt, bool low_throttle) {
  
  //Roll
  error_roll = desiredState.roll - stateVector.roll;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (low_throttle) integral_roll = 0;   //Don't let integrator build if throttle is too low
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = stateVector.roll_rate;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = desiredState.pitch - stateVector.pitch;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (low_throttle) integral_pitch = 0;   //Don't let integrator build if throttle is too low
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = stateVector.pitch_rate;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = desiredState.yaw_rate - stateVector.yaw_rate;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (low_throttle) integral_yaw = 0;   //Don't let integrator build if throttle is too low
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
  
}

MotorSpeeds AttitudeController::calcMotorSpeeds(DesiredState desiredState, StateVector stateVector, float dt, bool low_throttle)
{
  // Update PID controller
  controlANGLE(desiredState, stateVector, dt, low_throttle);

  // Control mixing
  MotorSpeeds motorSpeeds = controlMixer(desiredState.throttle);

  return motorSpeeds;
}

/**
 * @brief Print PID output values to the Serial monitor.
 * 
 * This includes PID outputs for roll, pitch, and yaw from stabilization controller.
 */
void AttitudeController::printPIDoutput() {
  Serial.print(F("roll_PID: "));
  Serial.print(roll_PID);
  Serial.print(F(" pitch_PID: "));
  Serial.print(pitch_PID);
  Serial.print(F(" yaw_PID: "));
  Serial.println(yaw_PID);
}
