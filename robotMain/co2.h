#ifndef CO2_H
#define CO2_H
#include <Arduino.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_SCD30.h>

void scd30Setup();
void sgp30Setup();
int scd30Reading();
int sgp30Reading();

#endif