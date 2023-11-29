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

bool SI7021::begin()
{
    return (isConnected());
}

bool SI7021::isConnected()
{
    uint8_t deviceID = getDeviceID();

    if (deviceID == 0x15) // Si7021 Found
        return (true);

    return (false);
}

// Get relative humidity
float SI7021::getRH()
{
    uint16_t RH_Code = getMeasurementNoHold(SI7021_HUMD_MEASURE_NOHOLD);
    float result = (125.0 * RH_Code / 65536) - 6;

    // Due to normal variations in RH accuracy of the device as described in Table 4, it is possible for the measured
    // value of %RH to be slightly less than 0 when the actual RH level is close to or equal to 0. Similarly, the
    // measured value of %RH may be slightly greater than 100 when the actual RH level is close to or equal to 100.

    // Bound to 0 - 100%
    if (result > 100.0)
        result = 100.0;
    if (result < 0.0)
        result = 0.0;

    return (result);
}

// Get temperature in C
float SI7021::getTemperature()
{
    uint16_t temp_Code = getMeasurementNoHold(SI7021_TEMP_MEASURE_NOHOLD);
    float result = (175.72 * temp_Code / 65536) - 46.85;
    return result;
}

// Get temperature in F
float SI7021::getTemperatureF()
{
    return ((getTemperature() * 1.8) + 32.0);
}

// Read temperature in C from previous RH measurement
float SI7021::getPreviousTemperature()
{
    uint16_t temp_Code = getMeasurementNoHold(SI7021_TEMP_PREVIOUS);
    float result = (175.72 * temp_Code / 65536) - 46.85;
    return result;
}

// Read temperature in F from previous RH measurement
float SI7021::getPreviousTemperatureF()
{
    return ((getPreviousTemperature() * 1.8) + 32.0);
}

// Depricated - Get temperature in C
float SI7021::getTemp()
{
    return (getTemperature());
}

// Depricated - Get temperature in F
float SI7021::getTempF()
{
    return (getTemperatureF());
}

// Depricated - Get previous temp in C
float SI7021::readTemp()
{
    return (getPreviousTemperature());
}

// Depricated - Get previous temp in F
float SI7021::readTempF()
{
    return ((readTemp() * 1.8) + 32.0); // Convert celsius to fahrenheit
}

// Turn on the heater
void SI7021::heaterOn()
{
    setHeater(true);
}

// Turns off the heater
void SI7021::heaterOff()
{
    setHeater(false);
}

void SI7021::setHeater(bool heaterOn)
{
    uint8_t regVal = readRegister8(SI7021_READ_USER_REG);

    regVal &= ~(0x01 << SI7021_HTRE_BIT); // Clear bit
    if (heaterOn)
        regVal |= 1; // Set bit

    writeRegister8(SI7021_WRITE_USER_REG, regVal);
}

// Return true if heater is on
bool SI7021::getHeater()
{
    uint8_t regVal = readRegister8(SI7021_READ_USER_REG);

    regVal &= (0x01 << SI7021_HTRE_BIT); // Clear all other bits
    if (regVal)
        return (true);
    return (false);
}

// Set the heater current
// 0000 = 3.09mA (Default)
// 0001 = 9.18mA
// 0010 = 16.24mA
// ...
// 1111 = 92.20mA
void SI7021::setHeaterCurrent(uint8_t currentLevel)
{
    currentLevel &= 0x0F; // Limit to lower four bits

    uint8_t regVal = readRegister8(SI7021_READ_HEATER_CONTROL_REG);

    regVal &= 0xF0;         // Clear lower bits
    regVal |= currentLevel; // Set bits

    writeRegister8(SI7021_WRITE_HEATER_CONTROL_REG, regVal);
}

uint8_t SI7021::getHeaterCurrent()
{
    uint8_t currentLevel = readRegister8(SI7021_READ_HEATER_CONTROL_REG);
    currentLevel &= 0x0F; // Limit to lower four bits
    return (currentLevel);
}

// Set the measurement resolution
void SI7021::setResolution(uint8_t resolutionValue)
{
    // Set resolutionValue to:
    //      RH         Temperature
    // 0: 12 bit       14 bit (default)
    // 1:  8 bit       12 bit
    // 2: 10 bit       13 bit
    // 3: 11 bit       11 bit

    uint8_t regVal = readRegister8(SI7021_READ_USER_REG);
    regVal &= 0b01111110; // Clear bits

    switch (resolutionValue)
    {
    case 0:
        regVal |= 0b00000000; // 12/14
        break;
    case 1:
        regVal |= 0b00000001; // 8/12
        break;
    case 2:
        regVal |= 0b10000000; // 10/13
        break;
    case 3:
        regVal |= 0b10000001; // 11/11
        break;
    default:
        break;
    }
    writeRegister8(SI7021_WRITE_USER_REG, regVal);
}

// Get the measurement resolution
uint8_t SI7021::getResolution()
{
    //      RH         Temperature
    // 0: 12 bit       14 bit (default)
    // 1:  8 bit       12 bit
    // 2: 10 bit       13 bit
    // 3: 11 bit       11 bit

    uint8_t resolutionValue = readRegister8(SI7021_READ_USER_REG);
    resolutionValue &= 0b10000001; // Clear inner bits

    switch (resolutionValue)
    {
    case 0b00000000:
        return (0); // 12/14
        break;
    case 0b00000001:
        return (1); // 8/12
        break;
    case 0b10000000:
        return (2); // 10/13
        break;
    case 0b10000001:
        return (3); // 11/11
        break;
    default:
        break;
    }
    return (0);
}

// Depricated - Changes to resolution of measurements
void SI7021::changeResolution(uint8_t resolutionValue)
{
    setResolution(resolutionValue);
}

// Send software reset command
void SI7021::reset()
{
    writeRegister8(SI7021_WRITE_USER_REG, SI7021_SOFT_RESET);
}

// Get device ID
uint8_t SI7021::getDeviceID()
{
    if (deviceSerialNumber == 0)
        getSerialNumber();
    return ((deviceSerialNumber >> 24) & 0xFF); // Extract SNB_3

    // // Device ID is the first byte of the 2nd serial number response (ie SNB_3)
    // Wire.beginTransmission(SI7021_ADDRESS);
    // Wire.write(SI7021_READ_SERIAL_NUMBER_2_A);
    // Wire.write(SI7021_READ_SERIAL_NUMBER_2_B);
    // Wire.endTransmission();

    // Wire.requestFrom(SI7021_ADDRESS, 1);

    // return (Wire.read());
}

// Get device's 64-bit serial number
uint64_t SI7021::getSerialNumber()
{
    uint8_t crc = 0;

    Wire.beginTransmission(SI7021_ADDRESS);
    Wire.write(SI7021_READ_SERIAL_NUMBER_1_A);
    Wire.write(SI7021_READ_SERIAL_NUMBER_1_B);
    Wire.endTransmission();

    Wire.requestFrom(SI7021_ADDRESS, 8);

    if (Wire.available() == 0)
    {
        deviceSerialNumber = 0;
        return (deviceSerialNumber);
    }

    // The response alternates between data and CRC bytes
    for (int x = 0; x < 8; x++)
    {
        if (x % 2 == 0)
        {
            deviceSerialNumber <<= 8;
            deviceSerialNumber |= Wire.read();
        }
        else
            crc = Wire.read();
    }

    Wire.beginTransmission(SI7021_ADDRESS);
    Wire.write(SI7021_READ_SERIAL_NUMBER_2_A);
    Wire.write(SI7021_READ_SERIAL_NUMBER_2_B);
    Wire.endTransmission();

    // The B portion is only 6 bytes
    Wire.requestFrom(SI7021_ADDRESS, 6);

    if (Wire.available() == 0)
    {
        deviceSerialNumber = 0;
        return (deviceSerialNumber);
    }

    // The response is two data bytes, then a CRC, then 2, then CRC
    for (int x = 0; x < 6; x++)
    {
        if (x == 2 || x == 5)
            crc = Wire.read();
        else
        {
            deviceSerialNumber <<= 8;
            deviceSerialNumber |= Wire.read();
        }
    }

	if(checkCrc8((uint8_t *)deviceSerialNumber, 8) != crc)
        return (SI7021_BAD_CRC);

    return (deviceSerialNumber);
}

// Depricated - see get device ID
uint8_t SI7021::checkID()
{
    return (getDeviceID());
}

// Write single register
void SI7021::writeRegister8(uint8_t registerAddress, uint8_t value)
{
    Wire.beginTransmission(SI7021_ADDRESS);
    Wire.write(SI7021_WRITE_USER_REG);
    Wire.write(value);
    Wire.endTransmission();
}

// Read single register
uint8_t SI7021::readRegister8(uint8_t registerAddress)
{
    Wire.beginTransmission(SI7021_ADDRESS);
    Wire.write(registerAddress);
    Wire.endTransmission();
    Wire.requestFrom(SI7021_ADDRESS, 1);
    uint8_t regVal = Wire.read();
    return regVal;
}

// Read a given register, polling until device responds
uint16_t SI7021::getMeasurementNoHold(uint8_t registerAddress)
{
    Wire.beginTransmission(SI7021_ADDRESS);
    Wire.write(registerAddress);
    Wire.endTransmission();

    // During a "no hold" register read, we can poll the device until it is complete
    uint8_t maxWait = 100; // In ms
    for (uint8_t x = 0; x < 255; x++)
    {
        delay(1);
        if (isConnected() == true)
            break;
        if (x == maxWait)
            return (0);
    }

    Wire.requestFrom(SI7021_ADDRESS, 3);
    if (Wire.available() != 3)
        return(SI7021_I2C_ERROR);

    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint8_t crc = Wire.read();

    int value = (uint16_t)msb << 8 | lsb;
    if(checkCrc8((uint8_t *)value, 2) != crc)
        return (SI7021_BAD_CRC);

    // LSB of RH is always xxxxxx10 - clear LSB to 00.
    value &= 0xFFFC;
    return ((uint16_t)value);
}

// Based on https://forum.arduino.cc/t/crc-8-i2c-cyclic-redundancy-check/644812/3
uint8_t SI7021::checkCrc8(uint8_t *inputBytes, uint8_t inputLength)
{
    const uint8_t generator = 0b00110001; // CRC polynomial = x^8 + x^5 + x^4 + 1 (x^8 is ignored)
    uint8_t crc = 0;

    while (inputLength--)
    {
        crc ^= *inputBytes++; // XOR-in the next input byte

        for (uint8_t i = 0; i < 8; i++)
        {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ generator);
            else
                crc <<= 1;
        }
    }
    return crc;
}