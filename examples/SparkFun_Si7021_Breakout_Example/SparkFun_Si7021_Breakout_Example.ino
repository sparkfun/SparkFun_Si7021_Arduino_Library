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
  Serial.println("SparkFun Si7021 Example");

  while (myHumidity.begin() == false)
  {
    Serial.println("Sensor not found. Please check wiring. Freezing...");
    while (true);
  }
  Serial.println("Humidity sensor detected");

  myHumidity.setResolution(0); // RH 12-bit, Temp 14-bit (Default)
  //myHumidity.setResolution(1); // RH 8-bit, Temp 12-bit (Default)
  //myHumidity.setResolution(2); // RH 10-bit, Temp 13-bit (Default)
  //myHumidity.setResolution(3); // RH 11-bit, Temp 11-bit (Default)
  int resolution = myHumidity.getResolution();

  resolution *= 2;

  myHumidity.heaterOn(); //Turn internal heater on
  //myHumidity.heaterOff(); //Turn heater off
  //myHumidity.setHeater(true); //Another way to turn heater on
  bool isHeaterOn = myHumidity.getHeater();

  if(isHeaterOn) Serial.println("Heater on");

  myHumidity.setHeaterCurrent(0b0000); //Set heater to min 3.09mA (Default)
  //myHumidity.setHeaterCurrent(0b1111); //Set heater to max 94.2mA
  int heaterCurrent = myHumidity.getHeaterCurrent();

  heaterCurrent *= 2;
}

void loop()
{
  // Measure Relative Humidity from the Si7021
  float humidity = myHumidity.getRH();

  // Measure Temperature from the Si7021
  float tempf = myHumidity.getTemperatureF();

  // Temperature is measured every time RH is requested.
  // You can also read the previous reading with getPreviousTemperature()

  Serial.print("Temperature:");
  Serial.print(tempf, 2);
  Serial.print("F, ");

  Serial.print("Humidity:");
  Serial.print(humidity, 1);
  Serial.print("%, ");

  uint64_t deviceSerialNumber = myHumidity.getSerialNumber();
  Serial.print("Serial Number: ");
  Serial.print(deviceSerialNumber);

  Serial.println();

  delay(1000);
}