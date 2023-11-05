#include "motor_functions.h"
#include "co2.h"
#include "lcd.h"
#include "ir.h"
#include "publisher.h"

void setup() {
  //scd30Setup();
  //sgp30Setup();
  setupMotors();
  //lcdSetup(); 
  //pubSetup();
}

void loop() {
  driveLoop(irReading());
  //int CO2 = scd30Reading();
  //int eCO2 = sgp30Reading();

  //printReadings(CO2, eCO2);
  //pubLoop(CO2);
}