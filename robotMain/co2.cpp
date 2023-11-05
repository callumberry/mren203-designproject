#include "co2.h"


Adafruit_SCD30 scd;
Adafruit_SGP30 sgp;

int counter = 0;
unsigned long period = 2000, measurement_time = millis();


uint32_t getAbsoluteHumidity(float temperature, float humidity){
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]

    return absoluteHumidityScaled;
}

void scd30Setup(){
  // Open the serial port at 115200 bps
  Serial.begin(115200);

  // Wait for serial connection before starting
  while (!Serial){
    delay(10);
  }

  Serial.println("__SCD30 demo__");
  // Initialize the sensor
  if (!scd.begin()){
    Serial.println("Sensor not found :(");
    while (1){
      delay(10); // This will stay here forever if a sensor isn't found
    }
  }
  // Set the measurement interval [2-1800 s]
  scd.setMeasurementInterval(2);

  // Read the measurement interval [s]
  Serial.print("Measurement Interval: ");
  Serial.print(scd.getMeasurementInterval());
  Serial.println(" seconds");  
}
void sgp30Setup(){
  Serial.println("__SGP30 demo__");

  if (!sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1){
    delay(10); // This will stay here forever if a sensor isn't found
    }
  }

  // Each board has a unique serial number that you can read
  // if you want to track which sensor you are using
  //Serial.print("SGP30 serial #");
  //Serial.print(sgp.serialnumber[0], HEX);
  //Serial.print(sgp.serialnumber[1], HEX);
  //Serial.println(sgp.serialnumber[2], HEX);

  // An existing calibration can be hard coded if known
  // sgp.setIAQBaseline(0x8E68, 0x8F41);
}

int scd30Reading(){
  int CO2;
  if (scd.dataReady()){
    // The library also has a handy check if data can't be read
    if (!scd.read()){
      Serial.println("Error reading sensor data");
      return;
    } 

    //Serial.print("Temp: ");
    //Serial.print(scd.temperature);
    //Serial.print(" dC\t");

    //Serial.print("RH: ");
    //Serial.print(scd.relative_humidity);
    //Serial.print(" %\t");

    Serial.print("CO2: ");
    Serial.print(scd.CO2, 3);
    Serial.println(" ppm");

    CO2 = (int) scd.CO2;  
    return CO2;    
  }  
}

int sgp30Reading(){
  int eCO2;
  if (millis() - measurement_time > period){
    measurement_time = millis();

    // If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
    float temperature = scd.temperature; // [°C]
    float humidity = scd.relative_humidity; // [%RH]
    sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

    if (!sgp.IAQmeasure()){
      Serial.println("Measurement failed");
      return;
    }
    Serial.print("TVOC ");
    Serial.print(sgp.TVOC);
    Serial.print(" ppb\t");
    Serial.print("eCO2 ");
    Serial.print(sgp.eCO2);
    Serial.println(" ppm");

    eCO2 = (int )sgp.eCO2;
    
    return eCO2;
  }  
}