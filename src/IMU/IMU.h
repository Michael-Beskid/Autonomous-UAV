/**
 * @file IMU.h
 *
 * @brief Header file for IMU class.
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include "MPU6050/MPU6050.h"

// Set gyro full scale range to 250 (deg/sec)
#define GYRO_SCALE MPU6050_GYRO_FS_250
#define GYRO_SCALE_FACTOR 131.0

// Set accelerometer full scale range to 2 (G's)
#define ACCEL_SCALE MPU6050_ACCEL_FS_2
#define ACCEL_SCALE_FACTOR 16384.0

class IMU {
public:
	
	IMU();

    void init();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    float getAccX();
    float getAccY();
    float getAccZ();
    float getRoll();
    float getPitch();
    float getYaw();
    void pollSensorData();
    void calculateError();
    void printGyroData();
    void printAccelData();

private:
    float AccX, AccY, AccZ;
    float AccX_prev, AccY_prev, AccZ_prev;
    float GyroX, GyroY, GyroZ;
    float GyroX_prev, GyroY_prev, GyroZ_prev;
    float roll_IMU, pitch_IMU, yaw_IMU;
    float roll_IMU_prev, pitch_IMU_prev;
    
    //IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
    float AccErrorX = 0.02;
    float AccErrorY = -0.07;
    float AccErrorZ = -0.01;
    float GyroErrorX = -1.70;
    float GyroErrorY = 0.63;
    float GyroErrorZ = -1.46;

    // Madgwick filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
    float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

};

#endif