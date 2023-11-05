#ifndef LCD_H
#define LCD_H
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

void lcdSetup();
void printReadings(int CO2Reading, int eCO2Reading);

#endif