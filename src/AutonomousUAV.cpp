/**
 * @file AutonomousUAV.cpp
 *
 * @brief Flight computer for autonomous UAV
 * 
 * Flight control software development for a quadrotor vehicle that is capable of autonomous station keeping,
 *   waypoint navigation, and the capability to track, intercept, and catch a ball thrown in its vicinity.
 * 
 * The project is hosted on GitHub: https://github.com/Michael-Beskid/Autonomous-UAV.
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#include <Arduino.h>                                  // Arduino library
#include <Wire.h>                                     // I2C communication
#include <SPI.h>                                      // SPI communication
#include <SoftwareSerial.h>                           // Serial communication
#include "AutonomousUAV.h"                            // General variables and function declarations
#include "Common.h"                                   // Common struct definitions for use globally
#include "MotorDriver/MotorDriver.h"                  // Motor commands
#include "StateEstimator/StateEstimator.h"            // State estimation class
#include "IMU/IMU.h"                                  // MPU 6050 IMU (6-axis accel/gyro)
#include "GPS/GPS.h"                                  // NEO-M9N GPS Module
#include "RadioComm/RadioComm.h"                      // Radio communication
#include "LaserAltimeter/LaserAltimeter.h"            // TFmini Plus Micro Range Finder Sensor
#include "AttitudeController/AttitudeController.h"    // Attitude controller to stabilize about body angle setpoints
#include "Navigation/Navigation.h"                    // Navigation class for computing desired body angle setpoints

MotorDriver motors;
RadioComm radio;
IMU imu;
GPS gps;
LaserAltimeter altimeter;         
StateEstimator stateEstimator;
Navigation navigation;
AttitudeController controller;                          

/**
 * @brief Perform setup tasks before entering the main flight control loop.
 * 
 * This is the Arduino built-in setup() function. setup() is used to establish
 *   a USB Serial connection, initilaize communication with sensors, set state
 *   variables to initial conditions, begin receiving radio data, and arm the 
 *   electronic speed controllers (ESCs) for the motors.
 * 
 * The functions "calcualte_IMU_error()" and "Calibrate_ESCs" may be uncommented
 *   for calibration purposes but should ordinarily remain commented out.
 */
void setup() {

  // Begin USB Serial
  Serial.begin(500000);

  // TEMP
  delay(3000);

  // Set built in LED to turn on to signal startup
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Initialize sensors
  // altimeter.init();
  imu.init();
  delay(500);
  Serial.println("Sensors initialized.");

  // TODO: Initialize state estimator?

  delay(50);

  // Initialize motor
  motors.init();
  delay(2000);
  Serial.println("Motors initialized.");

  // Initialize radio communication
  radio.init();
  delay(20);
  //Attach interrupt and point to corresponding ISR function
  attachInterrupt(digitalPinToInterrupt(radio.getPPMpin()), ISR, CHANGE);
  Serial.println("Radio initialized.");

  altimeter.init();
  Serial.println("Altimeter initialized.");

  // gps.init();
  // Serial.println("GPS initialized.");
  // delay(30000);
  // gps.setStartPos();

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  // imu.calculateError();

  // Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  // PROPS OFF. Code will not proceed past this if uncommented.
  // calibrateESCs(); 
  
  // Indicate entering main loop with 3 quick blinks
  Serial.println("Ready to fly!");
  setupBlink(3,160,70); // numBlinks, upTime (ms), downTime (ms)

}

/**
 * @brief Main flight control loop.
 * 
 * This is the Arduino built-in loop() function. The loop runs continuously after the 
 *   setup() function is executed. The main flight control loop performs a number of tasks
 *   such as reading sensor data, computing state estimates, managing the mission state, 
 *   calculating control outputs, and commanding the motors and servos.
 */                                              
void loop() {

  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

  // Get radio commands for current loop iteration
  RadioCommands radioCommands = radio.getAllPWM();
  bool low_throttle = radioCommands.throttle_ch_pwm < 1060;
  bool throttle_cut = radioCommands.throttle_cut_ch_pwm > 1500;

  // Poll sensors for new data if available
  imu.pollSensorData();
  altimeter.pollSensorData();
  //gps.pollSensorData();

  // Update state estimate
      // TODO: Likely multiple update methods dependent on which sensor have new data
  // stateEstimator.predict();
  // stateEstimator.update();
  stateEstimator.updateMadgwick(imu.getGyroX(), -imu.getGyroY(), -imu.getGyroZ(), -imu.getAccX(), imu.getAccY(), imu.getAccZ(), dt);

  // Get current state estimated
  StateVector stateVector = stateEstimator.getStateVector();

  // Compute desired state
  DesiredState desiredState = navigation.getDesiredState(radioCommands);
  
  // TODO: Wrap manual and auto states in here? Host state machines inside nav class?

  // Compute control inputs
  //    TODO: Create struct for motor commands? For return type
  //    TODO: Do scaling and mixing before returning 
  MotorSpeeds motorSpeeds;
  motorSpeeds = controller.calcMotorSpeeds(desiredState, stateVector, dt, low_throttle);

  // Command actuators
  motors.commandMotors(motorSpeeds, throttle_cut); // Send command pulses to each motor pin using OneShot125 protocol
    
  // Get vehicle commands for next loop iteration
  radio.getCommands(); // Pull current available radio commands
  radio.failSafe(); // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  printDebugInfo(); // Print data at 100 Hz for troubleshooting

  // Regulate loop rate
  loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default

}

/**
 * @brief Prints useful information to the Serial monitor for debugging purposes.
 * 
 * Any of the lines below can be uncommented to print the corresponding values to
 *   the Serial monitor for troublehsooting. Only one line should be uncommented
 *   at a time to ensure that the print statements will not slow down the main
 *   flight control loop and impact the vehicle's performance.
 */
void printDebugInfo() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    // radio.printData();
    // printDesiredState();
    // printPIDoutput();
    // navigation.printFlightMode();
    // motors.printMotorCommands();
    // motors.printMotorCommandsScaled();
    // imu.printGyroData();
    // imu.printAccelData();
    // stateEstimator.printAttitudeEstimate();
    // altimeter.printAltitude();
    // gps.printPosition();
    // gps.printLatLong();
    // printLoopRate();
    }
}

/**
 * @brief Used in to allow standard ESC calibration procedure with the radio to take place.
 * 
 * From dRehmFlight:
 *   Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
 *   power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
 *   uncommented when performing an ESC calibration.
 */
void calibrateESCs() {
   while (true) {
      prev_time = current_time;      
      current_time = micros();      
      dt = (current_time - prev_time)/1000000.0;
    
      digitalWrite(13, HIGH); //LED on to indicate we are not in main loop

      // Get radio commands
      RadioCommands radioCommands = radio.getAllPWM();
      bool throttle_cut = radioCommands.throttle_cut_ch_pwm > 1500;

      // Calc desired throttle from radio command
      DesiredState desiredState = navigation.getDesiredState(radioCommands);

      // Set all 4 motors to match throttle input
      MotorSpeeds motorSpeeds {desiredState.throttle, desiredState.throttle, desiredState.throttle, desiredState.throttle};
      motors.commandMotors(motorSpeeds, throttle_cut);
      
      loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}

/**
 * @brief Interupt service rotine for reading radio commands.
 */
void ISR() {
  radio.getPPM();
}


 /**
 * @brief Regulate main loop rate to specified frequency.
 * 
 * From dRehmFlight:
 *   It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
 *   background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
 *   the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
 *   be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
 *   and remain above 2kHz, without needing to retune all of our filtering parameters.
 * 
 * @param freq Loop frequency in Hz.
 */
void loopRate(int freq) {

  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

/**
 * @brief Blink LED on board to indicate main loop is running.
 */
void loopBlink() {
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); //Pin 13 is built in LED
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

/**
 * @brief Simple function to make LED on board blink as desired
 */
void setupBlink(int numBlinks,int upTime, int downTime) {
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

/**
 * @brief Print the loop rate to the Serial monitor.
 */
void printLoopRate() {
  Serial.print(F("dt = "));
  Serial.println(dt*1000000.0);
}
