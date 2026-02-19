/**
 * @file StateEstimator.cpp
 *
 * @brief StateEstimator class which implements an extended Kalman filter.
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#include "StateEstimator.h"

StateEstimator::StateEstimator() {}

/**
 * @brief Initialize the state estimator class.
 */
void StateEstimator::init() {
    // TODO
}

/**
 * @brief Get the current altitude.
 *
 * @returns most recent altitude reading in milimeters.
 */
float StateEstimator::getStateVector() {
   return x;
}

/**
 * @brief Update the state estimate.
 *
 *  EKF fuses system dynaimcs with sensor readings to update optimal state estimate.
 */
void StateEstimator::updateFilter() {
    // TODO: Implement method to poll sensor data
}

/**
 * @brief Print the current attitude estimate to the Serial monitor.
 */
void StateEstimator::printAttitudeEstimate() {
   // TODO
}

/**
 * @brief Print the current full state estimate to the Serial monitor.
 */
void StateEstimator::printAttitudeEstimate() {
   // TODO
}