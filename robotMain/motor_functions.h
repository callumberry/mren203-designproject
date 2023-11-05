#ifndef MOTOR_FUNCTIONS_H
#define MOTOR_FUNCTIONS_H
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

void decodeEncoderTicksL();
void decodeEncoderTicksL();
void messageCb( const geometry_msgs::Twist& msg);
double computeWheelRotation(long encoderTicks, double deltaT);
double computeWheelSpeed(double omega);
double compute_vehicle_speed(double v_L, double v_R);
double compute_vehicle_rate(double v_L, double v_R);
double computeLeftSpeed(double v, double omega);
double computeRightSpeed(double v, double omega);
double computePWMLeft(double ul, double vdl, double v_L);
double computePWMRight(double ur, double vdr, double v_R);
void driveControl(double u_L, double u_R);

void setupMotors();
void driveLoop(double irDistance);

#endif