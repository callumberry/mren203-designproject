#include "lcd.h"

LiquidCrystal_I2C lcd(0x27, 20, 4);
unsigned int timer = millis() + 2000;

void lcdSetup(){
  lcd.init();
  lcd.backlight();
}

void printReadings(int CO2Reading, int eCO2Reading){
  if(millis() > timer){
    int CO2, eCO2;
    lcd.clear();  
    lcd.setCursor(0, 0);
    lcd.print("CO2:"); 
    lcd.setCursor(0,1); 
    CO2 = (int) CO2Reading;
    lcd.print(CO2); 
    lcd.print("ppm"); 

    lcd.setCursor(9, 0); 
    lcd.print("eCO2:"); 
    lcd.setCursor(9,1);  
    eCO2 = (int) eCO2Reading;
    lcd.print(eCO2);
    lcd.print("ppm");
    timer = millis() + 2000;
  }
}
