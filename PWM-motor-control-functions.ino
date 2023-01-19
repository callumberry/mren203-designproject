/**
 * @file PWM-motor-control.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to drive one wheel motor through a motor driver.
 * @version 2.0
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2021-2022
 *
 */

int EA = 9; // Wheel PWM pin (must be a PWM pin)
int EB = 10; // Wheel PWM pin (must be a PWM pin)

int I1 = 2; // Wheel direction digital pin 1
int I2 = 3; // Wheel direction digital pin 2
int I3 = 4; // Wheel direction digital pin 1
int I4 = 5; // Wheel direction digital pin 2

void setup()
{
    // Configure digital pins for output
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT); 
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT); 
    pinMode(I4, OUTPUT);
}

void loop()
{
//    int u; // A variable for the motor PWM command [0-255]

    // Play with this code to write open loop commands to a wheel motor
//    for (u = 0; u <= 255; u += 5)
//    {
//        // Select a direction 
//        digitalWrite(I1, HIGH);  
//        digitalWrite(I2, LOW); 
//        digitalWrite(I3, LOW);  
//        digitalWrite(I4, HIGH); 
//
//        // PWM command to the motor driver
//        analogWrite(EA, u);
//        analogWrite(EB, u);
//
//        // Brief delay (perhaps not necessary)
//       // delay(10);
//    }

//  pause(5000);

  delay(2000);
  rightMove(200);
  delay(2000);
  pause(3000);
  delay(2000);
  rightMove(-200);
  delay(2000);
  pause(3000);
  delay(2000);
//  leftMove(200);
//  delay(2000);
//  leftMove(-200);
//  delay(2000);
//  


//  forward(150, 5000);
//  pause(1000);
//  reverse(150, 2000);
}

// Drives forwards for 2 seconds


void rightMove(int speed){
  if(speed > 0){
    //Right wheels go forward
    digitalWrite(I1, HIGH);  
    digitalWrite(I2, LOW); 
  }

  if(speed < 0){
    //Right wheels go backward
    digitalWrite(I1, LOW);  
    digitalWrite(I2, HIGH); 
  }
  analogWrite(EA, abs(speed));
}



void leftMove(int speed){
  if(speed > 0){
    //Left wheels go forward
    digitalWrite(I3, LOW);  
    digitalWrite(I4, HIGH);  
  }

  if(speed < 0){
    //Left wheels go backward
    digitalWrite(I3, HIGH);  
    digitalWrite(I4, LOW); 
  }
  analogWrite(EB, speed);
}








void reverse(int speed, int time){
        digitalWrite(I1, LOW);  
        digitalWrite(I2, HIGH); 
        digitalWrite(I3, HIGH);  
        digitalWrite(I4, LOW); 
        analogWrite(EA, speed);
        analogWrite(EB, speed);
        delay (time);
}


void pause(int time){
        analogWrite(EA, 0);
        analogWrite(EB, 0);
        delay (time);
}

void forward(int speed, int time){
        digitalWrite(I1, HIGH);  
        digitalWrite(I2, LOW); 
        digitalWrite(I3, LOW);  
        digitalWrite(I4, HIGH); 
        analogWrite(EA, speed);
        analogWrite(EB, speed);
        delay (time);
}
