/*
 SparkFun Si7021 Temperature and HUmidity Breakout
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

#if defined(ARDUINO)
#include "SparkFun_Si7021_Breakout_Library.h"
#include "Wire.h"
#elif defined(SPARK)
#include "SparkFun_Si7021_Breakout_Library/SparkFun_Si7021_Breakout_Library.h"
#endif

bool Weather::begin()
{
    uint8_t deviceID = checkID();

    if (deviceID == 0x15) // Si7021 Found
        return true;
    else if (deviceID == 0x32) // HTU21D Found
        return true;

    return false;
}

/****************Si7021 & HTU21D Functions**************************************/

float Weather::getRH()
{
    // Measure the relative humidity
    uint16_t RH_Code = makeMeasurment(HUMD_MEASURE_NOHOLD);
    float result = (125.0 * RH_Code / 65536) - 6;
    return result;
}

float Weather::readTemp()
{
    // Read temperature from previous RH measurement.
    uint16_t temp_Code = makeMeasurment(TEMP_PREV);
    float result = (175.72 * temp_Code / 65536) - 46.85;
    return result;
}

float Weather::getTemp()
{
    // Measure temperature
    uint16_t temp_Code = makeMeasurment(TEMP_MEASURE_NOHOLD);
    float result = (175.72 * temp_Code / 65536) - 46.85;
    return result;
}
// Give me temperature in fahrenheit!
float Weather::readTempF()
{
    return ((readTemp() * 1.8) + 32.0); // Convert celsius to fahrenheit
}

float Weather::getTempF()
{
    return ((getTemp() * 1.8) + 32.0); // Convert celsius to fahrenheit
}

// Turn on the heater
void Weather::heaterOn()
{
    uint8_t regVal = readRegister8(SI7021_READ_USER_REG);
    regVal |= _BV(HTRE);
    writeRegister8(SI7021_WRITE_USER_REG, regVal);
}

// Turns off the heater
void Weather::heaterOff()
{
    uint8_t regVal = readRegister8(SI7021_READ_USER_REG);
    regVal &= ~_BV(HTRE);
    writeRegister8(SI7021_WRITE_USER_REG, regVal);
}

// Changes to resolution of measurements
void Weather::setResolution(uint8_t resolutionValue)
{
    
    // Set resolutionValue to:
    //      RH         Temp
    // 0: 12 bit       14 bit (default)
    // 1:  8 bit       12 bit
    // 2: 10 bit       13 bit
    // 3: 11 bit       11 bit

    uint8_t regVal = readRegister8(SI7021_READ_USER_REG);
    regVal &= 0b0 1111 1110; //Default 12/14 bit
    switch (resolutionValue)
    {
    case 1:
        regVal |= 0b00000001;
        break;
    case 2:
        regVal |= 0b10000000;
        break;
    case 3:
        regVal |= 0b10000001;
    default:
        break;
    }
    writeRegister8(SI7021_WRITE_USER_REG, regVal);
}

// Changes to resolution of measurements
void Weather::changeResolution(uint8_t resolutionValue)
{
    setResolution(resolutionValue);
}

//Send software reset command
void Weather::reset()
{
    writeRegister8(SI7021_WRITE_USER_REG, SI7021_SOFT_RESET);
}

// Get device ID
uint8_t Weather::getDeviceID()
{
    Wire.beginTransmission(SI7021_ADDRESS);
    Wire.write(0xFC);
    Wire.write(0xC9);
    Wire.endTransmission();

    Wire.requestFrom(SI7021_ADDRESS, 1);

    return (Wire.read());
}

//Depricated - see get device ID
uint8_t Weather::checkID()
{
    return (getDeviceID);
}

// Write single register
void Weather::writeRegister8(uint8_t registerAddress, uint8_t value)
{
    Wire.beginTransmission(SI7021_ADDRESS);
    Wire.write(SI7021_WRITE_USER_REG);
    Wire.write(value);
    Wire.endTransmission();
}

// Read single register
uint8_t Weather::readRegister8(uint8_t registerAddress)
{
    Wire.beginTransmission(SI7021_ADDRESS);
    Wire.write(registerAddress);
    Wire.endTransmission();
    Wire.requestFrom(SI7021_ADDRESS, 1);
    uint8_t regVal = Wire.read();
    return regVal;
}

// Read a 16 bit register
uint16_t Weather::readRegister16(uint8_t registerAddress)
{
    // TODO: implement checksum checking

    Wire.beginTransmission(SI7021_ADDRESS);
    Wire.write(registerAddress);
    Wire.endTransmission();

    // When not using clock stretching (*_NOHOLD commands) delay here
    // is needed to wait for the measurement.
    // According to datasheet the max. conversion time is ~22ms
    delay(100);

    Wire.requestFrom(SI7021_ADDRESS, 2);
    if (Wire.available() != 2)
        return 100;

    uint16_t msb = Wire.read();
    uint16_t lsb = Wire.read();
    uint16_t value = msb << 8 | lsb;
    return value;
}

// Read a given register
uint16_t Weather::makeMeasurment(uint8_t registerAddress)
{
    uin16_t response = readRegister16(registerAddress);
    // Clear the last to bits of LSB to 00.
    // According to datasheet LSB of RH is always xxxxxx10
    response &= 0xFFFC;
    return (response);
}