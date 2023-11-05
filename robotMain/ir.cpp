#include "ir.h"

const byte SHARP_PIN_FRONT = A5;
int sharp_val_front = 0; 


double irReading(){
  double irReading;
  sharp_val_front = analogRead(SHARP_PIN_FRONT);
  irReading = 5450.7*(pow(sharp_val_front,-0.986));

  Serial.print("Sharp Value Front, Left, Right (cm): \n");
  Serial.println(irReading);

  //delay(500);

  return irReading;
}

          