/*
 SparkFun Si7021 Temperature and Humidity Breakout
 By: Joel Bartlett
 SparkFun Electronics
 Date: December 10, 2015

 This is an Arduino library for the Si7021 Temperature and Humidity Sensor Breakout

 This library is based on the following libraries:

 HTU21D Temperature / Humidity Sensor Library
 By: Nathan Seidle
 https://github.com/sparkfun/HTU21D_Breakout/tree/master/Libraries

 Arduino Si7010 relative humidity + temperature sensor
 By: Jakub Kaminski, 2014
 https://github.com/teoqba/ADDRESS

 This Library is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This Library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 For a copy of the GNU General Public License, see
 <http://www.gnu.org/licenses/>.
 */

#ifndef SparkFun_Si7021_Breakout_Library_h
#define SparkFun_Si7021_Breakout_Library_h

#include "Wire.h"
#include <Arduino.h>

const uint8_t SI7021_ADDRESS = 0x40;

#define SI7021_TEMP_MEASURE_HOLD 0xE3
#define SI7021_HUMD_MEASURE_HOLD 0xE5
#define SI7021_TEMP_MEASURE_NOHOLD 0xF3
#define SI7021_HUMD_MEASURE_NOHOLD 0xF5
#define SI7021_TEMP_PREVIOUS 0xE0

#define SI7021_READ_USER_REG 0xE7
#define SI7021_WRITE_USER_REG 0xE6
#define SI7021_HTRE_BIT 2

#define SI7021_READ_HEATER_CONTROL_REG 0x11
#define SI7021_WRITE_HEATER_CONTROL_REG 0x51

#define SI7021_READ_SERIAL_NUMBER_1_A 0xFA
#define SI7021_READ_SERIAL_NUMBER_1_B 0x0F
#define SI7021_READ_SERIAL_NUMBER_2_A 0xFC
#define SI7021_READ_SERIAL_NUMBER_2_B 0xC9

#define SI7021_SOFT_RESET 0xFE

// Measurement results
typedef enum Level {
  SI7021_OK = 0x00,
  SI7021_BAD_CRC,
  SI7021_I2C_ERROR,
  SI7021_READ_TIMEOUT,
} si7021Result;

class SI7021
{
  public:
    bool begin(TwoWire &wirePort = Wire);
    bool isConnected();

    float getRH();
    bool getRH(float *humidity);

    float getTemperature();
    bool getTemperature(float *temperature);
    float getTemperatureF();
    bool getTemperatureF(float *temperatureF);

    float getPreviousTemperature();
    float getPreviousTemperatureF();

    float readTemp();  // Get previous temp reading - Depricated
    float readTempF(); // Get privous temp reading - Depricated

    float getTemp();  // Get temp reading - Depricated
    float getTempF(); // Get temp reading - Depricated

    void heaterOn();
    void heaterOff();
    void setHeater(bool heaterOn);
    bool getHeater();

    void setHeaterCurrent(uint8_t currentLevel);
    uint8_t getHeaterCurrent();

    void setResolution(uint8_t resolutionValue);
    uint8_t getResolution();
    void changeResolution(uint8_t resolutionValue); // Depricated

    void reset();

    uint64_t getSerialNumber();
    uint8_t getDeviceID();
    uint8_t checkID(); // Depricated

  private:
    TwoWire *_i2cPort;

    uint64_t deviceSerialNumber = 0;

    si7021Result getMeasurementNoHold(uint8_t registerAddress, uint16_t *reading);

    void writeRegister8(uint8_t registerAddress, uint8_t value);
    uint8_t readRegister8(uint8_t registerAddress);
    uint16_t readRegister16(uint8_t registerAddress);

    uint8_t checkCrc8(uint8_t *inputBytes, uint8_t inputLength);
};

#endif
