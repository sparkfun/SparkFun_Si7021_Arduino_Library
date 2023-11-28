/*
  SparkFun Si7021 Breakout Example
  Joel Bartlett @ SparkFun Electronics
  Original Creation Date: May 18, 2015
  Updated November 28th, 2023
  This sketch prints the temperature and humidity the Serial port.

  Hardware Connections:
      HTU21D ------------- Arduino
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

Weather myHumidity;

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun Si7021 Example");

  while (myHumidity.begin() == false)
  {
    Serial.println("Sensor not found. Please check wiring. Freezing...");
    while (true);
  }
  Serial.println("Humidity sensor detected");
}

void loop()
{
  // Measure Relative Humidity from the HTU21D or Si7021
  float humidity = myHumidity.getRH();

  // Measure Temperature from the HTU21D or Si7021
  float tempf = myHumidity.getTempF();

  // Temperature is measured every time RH is requested.
  // It is faster, therefore, to read it from previous RH
  // measurement with getTemp() instead with readTemp()

  Serial.print("Temp:");
  Serial.print(tempf, 2);
  Serial.print("F, ");

  Serial.print("Humidity:");
  Serial.print(humidity, 1);
  Serial.println("%");

  delay(1000);
}