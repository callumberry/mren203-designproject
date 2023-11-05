#include "motor_functions.h"

const byte EA = 9, I1 = 2, I2 = 3,
           EB = 10, I3 = 4, I4 = 5,
           SIGNAL_AL = 12, SIGNAL_BL = 13, 
           SIGNAL_AR = 6, SIGNAL_BR = 7;

const int TPR = 3000, kp = 250, ki = 0.5 * kp,
          T = 100;

const double RHO = 0.0625, track = 0.2775;

double ul = 0.0, ur = 0.0,
       omega_L = 0.0, omega_R = 0.0,
       v_L = 0.0, v_R = 0.0,
       vdr = 0.0, vdl = 0.0,
       currentV = 0.0, currentOmega = 0.0,
       integralL = 0.0, integralR = 0.0,
       vDesired = 0.0, turnRate = 0.0;

volatile long encoder_ticksL = 0,
              encoder_ticksR = 0;

long t_now = 0, t_last = 0;

ros::NodeHandle  nh;

void messageCb( const geometry_msgs::Twist& msg){
  //turnRate = msg.angular.z;
  //vDesired = msg.linear.x;
  turnRate = msg.angular.z / 2;
  vDesired = msg.linear.x / 2;

  if(turnRate == 2|| vDesired == 0.5 || turnRate == -2|| vDesired == -0.5){ //Testing LED
    digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void decodeEncoderTicksL(){
  if (digitalRead(SIGNAL_BL) == LOW){
    encoder_ticksL--;
  }
  else{
    encoder_ticksL++;
  }
}
void decodeEncoderTicksR(){
  if (digitalRead(SIGNAL_BR) == LOW){
    encoder_ticksR++;
  }
  else{
    encoder_ticksR--;
  }
}

double computeWheelRotation(long encoderTicks, double deltaT){
  double omega;
  omega = 2.0 * PI * ((double)encoderTicks / (double)TPR) * 1000.0 / deltaT;
  return omega;    
}
double computeWheelSpeed(double omega){
  double wheelSpeed;
  wheelSpeed = omega * RHO;
  return wheelSpeed;    
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

double computeLeftSpeed(double v, double omega){
  double wheelSpeed = 0.0;
  wheelSpeed = v - 0.5 * (track * omega);
  return wheelSpeed;
}
double computeRightSpeed(double v, double omega){
  double wheelSpeed = 0.0;
  wheelSpeed = v + 0.5 * (track * omega);
  return wheelSpeed;
}

double computePWMLeft(double ul, double vdl, double v_L){
  if(abs(ul) < 255){
    integralL += (vdl- v_L);
  }
  ul = kp * (vdl - v_L) + ki * integralL;

  if( ul  > 255){
    ul  = 255;
  }
  if( ul < -255){
    ul  = -255;
  }
  return ul;
}
double computePWMRight(double ur, double vdr, double v_R){
  if( abs(ur) < 255){
    integralR += (vdr - v_R);
  }
  ur = kp * (vdr - v_R) + ki * integralR;

  if( ur > 255){
    ur = 255;
  }
  if( ur < -255){
    ur = -255;
  }
  return ur;
}

void driveControl(double u_L, double u_R){
  if(ur > 0){
    digitalWrite(I1, HIGH);  
    digitalWrite(I2, LOW); 
    analogWrite(EA, ur);
  }
  if(ur < 0){
    digitalWrite(I1, LOW);  
    digitalWrite(I2, HIGH); 
    analogWrite(EA, -ur);
  }
  if(ul > 0){
    digitalWrite(I3, LOW);  
    digitalWrite(I4, HIGH); 
    analogWrite(EB, ul);
  }
  if(ul < 0){
    digitalWrite(I3, HIGH);  
    digitalWrite(I4, LOW); 
    analogWrite(EB, -ul);
  }  
}

void setupMotors(){
  pinMode(LED_BUILTIN, OUTPUT); //Testing LED

  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  pinMode(SIGNAL_AL, INPUT);
  pinMode(SIGNAL_BL, INPUT);
  pinMode(SIGNAL_AR, INPUT);
  pinMode(SIGNAL_BR, INPUT);

  analogWrite(EA, 0);
  analogWrite(EB, 0);

  digitalWrite(I1, LOW);  
  digitalWrite(I2, LOW); 
  digitalWrite(I3, LOW);  
  digitalWrite(I4, LOW); 

  attachInterrupt(digitalPinToInterrupt(SIGNAL_AL), decodeEncoderTicksL, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_AR), decodeEncoderTicksR, RISING);
  
  nh.initNode();
  nh.subscribe(sub);
}

void driveLoop(double irDistance){
  t_now = millis();
    if (t_now - t_last >= T){
      //set values here for testing 
      //vDesired = 1;
      //turnRate = 0.0;

      omega_L = computeWheelRotation(encoder_ticksL, (double)(t_now - t_last));
      omega_R = computeWheelRotation(encoder_ticksR, (double)(t_now - t_last));    

      v_L = computeWheelSpeed(omega_L);
      v_R = computeWheelSpeed(omega_R);

      t_last = t_now;
      encoder_ticksL = 0;
      encoder_ticksR = 0;

      //Testing
      //currentV = compute_vehicle_speed(v_L, v_R);
      //currentOmega = compute_vehicle_rate(v_L, v_R);
      //Serial.println(currentV);
      //Serial.println(currentOmega);
      if(irDistance < 30 && vDesired > 0){
        vdl = computeLeftSpeed(0, turnRate);
        vdr = computeRightSpeed(0, turnRate);   
        tone(8,100);  
      }
      else{
        vdl = computeLeftSpeed(vDesired, turnRate);
        vdr = computeRightSpeed(vDesired, turnRate);    
        noTone(8);   
      }
      ul = computePWMLeft(ul, vdl, v_L);
      ur = computePWMRight(ur, vdr, v_R);      

      driveControl(ul, ur);
    }

    nh.spinOnce();
}