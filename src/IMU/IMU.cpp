/**
 * @file IMU.cpp
 *
 * @brief IMU class to interface with the MPU6050 inertial measurement unit.
 * 
 * This class utilizes the MPU6050 library created by Jeff Rowberg to interface with the MPU6050 6-axis IMU
 *   which features a 3-axis accelerometer and 3-axis rate gyroscope.
 * 
 * Wiring:
 *   Red: +5V
 *   Black: GND
 *   Blue: SDA (pin 18)
 *   Green: SCL (pin 19)
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#include "IMU.h"

MPU6050 mpu6050;

IMU::IMU() {}

/**
 * @brief Initialize the IMU.
 */
void IMU::init() {
  
  Wire.begin();
  Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
  
  mpu6050.initialize();
  
  if (mpu6050.testConnection() == false) {
    Serial.println("MPU6050 initialization unsuccessful");
    Serial.println("Check MPU6050 wiring or try cycling power");
    while(1) {}
  }

  //From the reset state all registers should be 0x00, so we should be at
  //max sample rate with digital low pass filter(s) off.  All we need to
  //do is set the desired fullscale ranges
  mpu6050.setFullScaleGyroRange(GYRO_SCALE);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
    
}

/**
 * @brief Get the current X-axis gyro reading.
 *
 * @returns most recent X-axis gyro value in dps.
 */
float IMU::getGyroX() {
  return GyroX;
}

/**
 * @brief Get the current Y-axis gyro reading.
 *
 * @returns most recent Y-axis gyro value in dps.
 */
float IMU::getGyroY() {
  return GyroY;
}

/**
 * @brief Get the current Z-axis gyro reading.
 *
 * @returns most recent Z-axis gyro value in dps.
 */
float IMU::getGyroZ() {
  return GyroZ;
}

/**
 * @brief Get the current X-axis accelerometer reading.
 *
 * @returns most recent X-axis accelerometer value in g's.
 */
float IMU::getAccX() {
  return AccX;
}

/**
 * @brief Get the current Y-axis accelerometer reading.
 *
 * @returns most recent Y-axis accelerometer value in g's.
 */
float IMU::getAccY() {
  return AccY;
}

/**
 * @brief Get the current Z-axis accelerometer reading.
 *
 * @returns most recent Z-axis accelerometer value in g's.
 */
float IMU::getAccZ() {
  return AccZ;
}

/**
 * @brief Get the current roll angle.
 *
 * @returns most recent roll angle measurement in degrees.
 */
float IMU::getRoll() {
  return roll_IMU;
}

/**
 * @brief Get the current pitch angle.
 *
 * @returns most recent pitch angle measurement in degrees.
 */
float IMU::getPitch() {
  return pitch_IMU;
}

/**
 * @brief Get the current yaw angle.
 *
 * @returns most recent yaw angle measurement in degrees.
 */
float IMU::getYaw() {
  return yaw_IMU;
}

/**
 * @brief Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data.
 *
 * From dRehmFlight:
 *   Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
 *   These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
 *   low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
 *   off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
 *   the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
 *   the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
 */
void IMU::pollSensorData() {

  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;

  mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

 // Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  // Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

}

/**
 * @brief Computes IMU accelerometer and gyro error on startup.
 *
 * From dRehmFlight:
 *   Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and
 *   accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
 *   measurement. 
 */
void IMU::calculateError() {
  
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

/**
 * @brief Print the 3-axis gyros readings in [dps] to the Serial monitor.
 */
void IMU::printGyroData() {
    Serial.print(F("GyroX: "));
    Serial.print(GyroX);
    Serial.print(F(" GyroY: "));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ: "));
    Serial.println(GyroZ);
}

/**
 * @brief Print the 3-axis accelerometer readings in g's to the Serial monitor.
 */
void IMU::printAccelData() {
    Serial.print(F("AccX: "));
    Serial.print(AccX);
    Serial.print(F(" AccY: "));
    Serial.print(AccY);
    Serial.print(F(" AccZ: "));
    Serial.println(AccZ);
}

/**
 * @brief Print the roll, pitch, and yaw angles in [deg] to the Serial monitor.
 */
void IMU::printRollPitchYaw() {
    Serial.print(F("roll: "));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch: "));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw: "));
    Serial.println(yaw_IMU);
}
