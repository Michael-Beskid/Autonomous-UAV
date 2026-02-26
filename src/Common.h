/**
 * @file Common.h
 *
 * @brief Header file for common structs used globally
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#ifndef _COMMON_H_
#define _COMMON_H_

struct MotorSpeeds {
    float m1;
    float m2;
    float m3;
    float m4;
};

struct StateVector {
    float x;
    float y;
    float z;
    float x_vel;
    float y_vel;
    float z_vel;
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
};

struct DesiredState {
    float throttle;
    float roll;
    float pitch;
    float yaw;
    float yaw_rate;
};

struct RadioCommands {
    unsigned long roll_ch_pwm;
    unsigned long pitch_ch_pwm;
    unsigned long throttle_ch_pwm;
    unsigned long yaw_ch_pwm;
    unsigned long throttle_cut_ch_pwm;
    unsigned long flight_mode_ch_pwm;
};

#endif