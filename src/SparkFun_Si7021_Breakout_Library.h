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

#include <Arduino.h>

/****************Si7021 & HTU21D Definitions***************************/

#define SI7021_ADDRESS 0x40

#define SI7021_TEMP_MEASURE_HOLD 0xE3
#define SI7021_HUMD_MEASURE_HOLD 0xE5
#define SI7021_TEMP_MEASURE_NOHOLD 0xF3
#define SI7021_HUMD_MEASURE_NOHOLD 0xF5
#define SI7021_TEMP_PREV 0xE0

#define SI7021_WRITE_USER_REG 0xE6
#define SI7021_READ_USER_REG 0xE7

#define SI7021_WRITE_HEATER_CONTROL_REG 0x51
#define SI7021_READ_HEATER_CONTROL_REG 0x11

#define SI7021_SOFT_RESET 0xFE

#define HTRE 0x02
#define _BV(bit) (1 << (bit))

#define SI7021_CRC_POLY 0x988000 // Shifted Polynomial for CRC check

// Error codes
#define SI7021_I2C_TIMEOUT 998
#define SI7021_BAD_CRC 999

/****************Si7021 & HTU21D Class**************************************/
class Weather
{
  public:
    bool begin();

    float getRH();
    float readTemp();
    float getTemp();
    float readTempF();
    float getTempF();
    void heaterOn();
    void heaterOff();
    void changeResolution(uint8_t i);
    
	void reset();
    
	uint8_t getDeviceID();
    uint8_t checkID(); //Depricated

  private:

    uint16_t makeMeasurment(uint8_t command);
    void writeRegister8(uint8_t registerAddress, uint8_t value);
    uint8_t readRegister8(uint8_t registerAddress);
    uint16_t readRegister16(uint8_t registerAddress);
};

#endif
