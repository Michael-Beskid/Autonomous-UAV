/**
 * @file AutonomousUAV.h
 *
 * @brief Header file for main AutonomousUAV.cpp file.
 *
 * @author Michael Beskid
 * Contact: michael.beskid@gmail.com
 *
 */

#ifndef _AUTONOMOUSUAV_H_
#define _AUTONOMOUSUAV_H_

// General declarations
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
unsigned int slowLoopCounter;
bool blinkAlternate;

// Function declarations
void printDebugInfo();
void calibrateESCs();
void loopRate(int freq);
void loopBlink();
void setupBlink(int numBlinks,int upTime, int downTime);
void printLoopRate();
void ISR();

#endif