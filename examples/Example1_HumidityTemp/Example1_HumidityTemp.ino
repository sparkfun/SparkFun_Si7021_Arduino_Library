/*
  SparkFun Si7021 Breakout Example
  Joel Bartlett @ SparkFun Electronics
  Original Creation Date: May 18, 2015
  Updated November 28th, 2023
  This sketch prints the temperature and humidity to the Serial port.

  Hardware Connections:
      Si7021 ------------- Arduino
      (-) ------------------- GND
      (+) ------------------- 3.3V (VCC)
       CL ------------------- SCL
       DA ------------------- SDA

  This code is beerware; if you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful,
  please buy us a round!
  Distributed as-is; no warranty is given.
*/

#include "SparkFun_Si7021_Breakout_Library.h" //http://librarymanager/All#SparkFun_Si7021
#include <Wire.h>

SI7021 myHumidity;

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println("SparkFun Si7021 Example");

  Wire.begin();

  while (myHumidity.begin() == false)
  {
    Serial.println("Sensor not found. Please check wiring. Freezing...");
    while (true)
      ;
  }
  Serial.println("Humidity sensor detected");
}

void loop()
{
  // Measure Relative Humidity from the Si7021
  float humidity = myHumidity.getRH();

  Serial.print("Humidity:");
  Serial.print(humidity, 1);
  Serial.print("%, ");

  // Measure Temperature from the Si7021
  float tempF = myHumidity.getTemperatureF();

  // Temperature is measured every time RH is requested.
  // You can also read the previous reading with getPreviousTemperature()

  Serial.print("Temperature:");
  Serial.print(tempF, 2);
  Serial.print("F, ");

  Serial.println();

  delay(1000);
}