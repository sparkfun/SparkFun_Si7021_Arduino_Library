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

  //You can call begin() with other Wire ports if needed
  while (myHumidity.begin(Wire) == false)
  {
    Serial.println("Sensor not found. Please check wiring. Freezing...");
    while (true)
      ;
  }
  Serial.println("Humidity sensor detected");

  myHumidity.setResolution(0); // RH 12-bit, Temp 14-bit (Default)
  // myHumidity.setResolution(1); // RH 8-bit, Temp 12-bit
  // myHumidity.setResolution(2); // RH 10-bit, Temp 13-bit
  // myHumidity.setResolution(3); // RH 11-bit, Temp 11-bit

  int resolution = myHumidity.getResolution();
  Serial.print("Resolution: ");
  Serial.println(resolution);

  // myHumidity.heaterOn(); //Turn internal heater on
  myHumidity.heaterOff(); // Turn internal heater off
  // myHumidity.setHeater(true); //Another way to turn heater on

  bool isHeaterOn = myHumidity.getHeater();
  if (isHeaterOn)
    Serial.println("Heater on");

  myHumidity.setHeaterCurrent(0b0000); // Set heater to min 3.09mA (Default)
  // myHumidity.setHeaterCurrent(0b1111); //Set heater to max 94.2mA

  int heaterCurrent = myHumidity.getHeaterCurrent();
  Serial.print("Heater Current: ");
  Serial.println(heaterCurrent);

  uint64_t deviceSerialNumber = myHumidity.getSerialNumber();
  Serial.print("Serial Number: ");
  Serial.println(deviceSerialNumber);
}

void loop()
{
  // You can also call the getRH/getTemperature with a variable. This will return true/false if the reading was
  // successful. This is helpful for checking if the I2C comm timed out or otherwise failed.
  float humidity = 0.0;
  if (myHumidity.getRH(&humidity) == true)
  {
    Serial.print("Humidity was successfully read:");
    Serial.print(humidity, 1);
    Serial.print("%, ");
  }

  // Measure the temperature in F, and check if it's valid
  float tempF = 0.0;
  if (myHumidity.getTemperatureF(&tempF) == true)
  {
    Serial.print("Temperature was successfully read:");
    Serial.print(tempF, 2);
    Serial.print("F, ");
  }
  
  // Measure the temperature in C, and check if it's valid
  float tempC = 0.0;
  if (myHumidity.getTemperature(&tempC) == true)
  {
    Serial.print("Temperature:");
    Serial.print(tempC, 2);
    Serial.print("C, ");
  }

  Serial.println();

  delay(1000);
}