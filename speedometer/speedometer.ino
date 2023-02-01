/**
 * @file motor-angular-rate.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to estimate motor speed from encoder.
 * @version 2.0
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2021-2022
 *
 */

#include <Arduino_LSM6DS3.h>

// Variables to store angular rates from the gyro [degrees/s]
float omega_x, omega_y, omega_z;

// Variables to store sample rates from sensor [Hz]
float g_f;

// Motor driver PWM pin
int EA = 9;
int EB = 10; 

// Motor driver direction pin
int I1 = 2;
int I2 = 3;

int I3 = 4;
int I4 = 5;

// Motor PWM command variable [0-255]
byte L = 0;
byte R = 0;

// Left wheel encoder digital pins
const byte SIGNAL_AL = 12;
const byte SIGNAL_BL = 13;
const byte SIGNAL_AR = 6;
const byte SIGNAL_BR = 7;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3020;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticksL = 0;
volatile long encoder_ticksR = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;
double v_L = 0.0;
double v_R = 0.0;

double v_t = 0.0;
double turnRate = 0.0;
double track = 0.2775;


double kr = 1.0, kl = 1.0; 
double ur = 0.0, ul = 0.0;
double vdr = 0.0, vdl = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 100;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicksL(){
    if (digitalRead(SIGNAL_BL) == LOW){
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticksL--;
    }
    else{
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticksL++;
    }
}

void decodeEncoderTicksR(){
    if (digitalRead(SIGNAL_BR) == LOW){
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticksR++;
    }
    else{
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticksR--;
    }
}

void setup(){
    // Open the serial port at 9600 bps
    Serial.begin(9600);
    //Serial.begin(115200);

     if (!IMU.begin()){
        // Print an error message if the IMU is not ready
        Serial.print("Failed to initialize IMU :(");
        Serial.print("\n");
        while (1){
            delay(10);
        }
    }

    g_f = IMU.gyroscopeSampleRate();

    // Set the pin modes for the motor driver
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);

    pinMode(EB, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_AL, INPUT);
    pinMode(SIGNAL_BL, INPUT);

    pinMode(SIGNAL_AR, INPUT);
    pinMode(SIGNAL_BR, INPUT);

    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(SIGNAL_AL), decodeEncoderTicksL, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_AR), decodeEncoderTicksR, RISING);

    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void loop(){
    // Get the elapsed time [ms]
    t_now = millis();

    if (t_now - t_last >= T){
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)encoder_ticksL / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R = 2.0 * PI * ((double)encoder_ticksR / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        v_L = omega_L * RHO;
        v_R = omega_R * RHO;

        turnRate = compute_vehicle_rate(v_L, v_R);

        vdr = 0.5 * (track * turnRate) - compute_vehicle_speed(v_L, v_R); 
        vdl = compute_vehicle_speed(v_L, v_R) - 0.5 * (track * turnRate); 

        ur = kr * (vdr - v_R);
        ul = kl * (vdl - v_L);
    

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticksL = 0;
        encoder_ticksR = 0;
    }

    analogWrite(EA, ur);    // Write left motors command
    analogWrite(EB, ul);

    // Write to the output pins
    digitalWrite(I1, HIGH); // Drive forward (left wheels)
    digitalWrite(I2, LOW);

    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);   
}

double compute_vehicle_speed(double v_L, double v_R){
    double v;
    v = 0.5 * (v_L + v_R);
    return v;
}

double compute_vehicle_rate(double v_L, double v_R){
    double omega;
    omega = 1.0 / track * (v_R - v_L);
    return omega;
}