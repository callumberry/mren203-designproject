#include <Arduino_LSM6DS3.h>

// Motor driver PWM pin
int EA = 9, EB = 10; 

// Motor driver direction pin
int I1 = 2, I2 = 3, I3 = 4, I4 = 5;

// Motor PWM command variable [0-255]
double ul = 0, ur = 0;

// Left wheel encoder digital pins
const byte SIGNAL_AL = 12, SIGNAL_BL = 13, SIGNAL_AR = 6, SIGNAL_BR = 7;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3020;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticksL = 0, encoder_ticksR = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0, omega_R = 0.0;
double v_L = 0.0, v_R = 0.0;

double turnRate = 0;
double track = 0.2775;

double kp = 150, ki = 0.5 * kp; 
double vdr = 0.0, vdl = 0.0;
double vDesired = 0.25;
double integralL = 0.0, integralR = 0.0;

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

        vdr = 0.5 * (track * turnRate) + vDesired; 
        vdl = vDesired - 0.5 * (track * turnRate); 

        integralL = integralL + (vdr - v_R);
        integralR = integralL + (vdl - v_L);

        if(abs(ur) > 255){
            
        }
        else{
            ur = kp * (vdr - v_R) + ki * integralL;
      
        }
        if(abs(ul) > 255){

        }
        else{
            ul = kp * (vdl - v_L) + ki * integralR;
        }

        

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticksL = 0;
        encoder_ticksR = 0;

        

        if(ur > 0){
            RFor(ur);
        }
        if(ur < 0){
            RBack(-ur);
        }
        if(ul > 0){
            LFor(ur);
        }
        if(ul < 0){
            LBack(-ul);
        }
       

        Serial.print("ul: ");
        Serial.print(ul);
        Serial.print("   ");
        Serial.print("ur: ");
        Serial.print(ur);
        Serial.print("\n");
        Serial.print("   ");
        Serial.print("velocity: ");
        Serial.print(compute_vehicle_speed(v_L, v_R));
        Serial.print("\n");
    }  
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

//right wheel
void RFor(int speed){
    digitalWrite(I1, HIGH);  
    digitalWrite(I2, LOW); 
    analogWrite(EA, speed);
}
void RBack(int speed){
    digitalWrite(I1, LOW);  
    digitalWrite(I2, HIGH); 
    analogWrite(EA, speed);
}
//control for left wheels
void LFor(int speed){
    digitalWrite(I3, LOW);  
    digitalWrite(I4, HIGH); 
    analogWrite(EB, speed);
}
void LBack(int speed){
    digitalWrite(I3, HIGH);  
    digitalWrite(I4, LOW); 
    analogWrite(EB, speed);
}