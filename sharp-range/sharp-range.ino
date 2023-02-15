/**
 * @file sharp-range.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca), Thomas Sears (thomas.sears@queensu.ca)
 * @brief Arduino program to read proximity data from a Sharp GP2Y0A21YK.
 * @version 2.1
 * @date 2022-12-21
 *
 * @copyright Copyright (c) 2022
 *
 */

// Arduino analog input pin to which the Sharp sensor is connected
const byte SHARP_PIN_FRONT = A0;
const byte SHARP_PIN_RIGHT = A1;
const byte SHARP_PIN_LEFT = A2;

// Variables to store the proximity measurement
int sharp_val_front = 0; // integer read from analog pin
int sharp_val_left = 0; // integer read from analog pin
int sharp_val_right = 0; // integer read from analog pin
float sharp_range_front, sharp_range_left, sharp_range_right; // range measurement [cm]
float sharp_front_cm, sharp_left_cm, sharp_right_cm;

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(115200);
}

void loop()
{
    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_val_front = analogRead(SHARP_PIN_FRONT);
    sharp_front_cm = 5450.7*(pow(sharp_val_front,-0.986));
    sharp_val_left = analogRead(SHARP_PIN_LEFT);
    sharp_left_cm = 4482.8*(pow(sharp_val_left,-0.918));
    sharp_val_right = analogRead(SHARP_PIN_RIGHT);
    sharp_right_cm = 4054.2*(pow(sharp_val_right,-0.891));
    //Print all values
    Serial.print("Sharp Value Front, Left, Right (cm): \n");
    Serial.print(sharp_front_cm);
    Serial.print("\n");
    Serial.print(sharp_left_cm);
    Serial.print("\n");
    Serial.print(sharp_right_cm);
    Serial.print("\n");

    // Delay for a bit before reading the sensor again
    delay(500);
}
