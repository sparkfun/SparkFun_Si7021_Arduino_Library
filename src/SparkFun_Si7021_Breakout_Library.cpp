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

bool SI7021::begin(TwoWire &wirePort)
{
    _i2cPort = &wirePort;
    if (isConnected() == false)
        return (false);

    if (getDeviceID() == 0x15) // Si7021 Found
        return (true);

    return (false);
}

bool SI7021::isConnected()
{
    _i2cPort->beginTransmission(SI7021_ADDRESS);
    if (_i2cPort->endTransmission() != 0)
        return (false); // Sensor did not ACK
    return (true);
}

// getRhCrc returns true if reading passes CRC check
// Reading is stored in humidity regardless of the CRC result
bool SI7021::getRH(float *humidity)
{
    uint16_t rhCode = 0;
    si7021Result result = getMeasurementNoHold(SI7021_HUMD_MEASURE_NOHOLD, &rhCode);

    // The measurement may have failed CRC, complete calc regardless

    float localHumidity = (125.0 * rhCode / 65536) - 6;

    // From the datasheet:
    // Due to normal variations in RH accuracy of the device as described in Table 4, it is possible for the measured
    // value of %RH to be slightly less than 0 when the actual RH level is close to or equal to 0. Similarly, the
    // measured value of %RH may be slightly greater than 100 when the actual RH level is close to or equal to 100.

    // Bound to 0 - 100%
    if (localHumidity > 100.0)
        localHumidity = 100.0;
    if (localHumidity < 0.0)
        localHumidity = 0.0;

    *humidity = localHumidity; // Update the humidity value regardless of the result

    if (result != SI7021_OK)
        return (false);

    return (true);
}

// Return relative humidity regardless of CRC result
float SI7021::getRH()
{
    float humidity = 0.0;
    getRH(&humidity);
    return (humidity);
}

// Get temperature in C
bool SI7021::getTemperature(float *temperature)
{
    uint16_t tempReading = 0;
    si7021Result result = getMeasurementNoHold(SI7021_TEMP_MEASURE_NOHOLD, &tempReading);

    // The measurement may have failed CRC, complete calc regardless

    float localTemp = (175.72 * tempReading / 65536) - 46.85;

    *temperature = localTemp; // Update the humidity value regardless of the result

    if (result != SI7021_OK)
        return (false);

    return (true);
}

// Get temperature in C
float SI7021::getTemperature()
{
    float temperature = 0.0;
    getTemperature(&temperature);
    return (temperature);
}

// Get temperature in F
bool SI7021::getTemperatureF(float *temperature)
{
    bool result = getTemperature(temperature);
    *temperature = (*temperature * 1.8) + 32.0;
    return (result);
}

// Get temperature in F
float SI7021::getTemperatureF()
{
    return ((getTemperature() * 1.8) + 32.0);
}

// Read temperature in C from previous RH measurement
float SI7021::getPreviousTemperature()
{
    uint16_t temperatureCode = 0;
    getMeasurementNoHold(SI7021_TEMP_PREVIOUS,
                         &temperatureCode); // Ignore return value because there is no CRC for previous temp
    float result = (175.72 * temperatureCode / 65536) - 46.85;
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
        regVal |= (0x01 << SI7021_HTRE_BIT); // Set bit

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

// Get device ID - Should return 0x15 for the Si7021
uint8_t SI7021::getDeviceID()
{
    if (deviceSerialNumber == 0)
        getSerialNumber();
    return ((deviceSerialNumber >> 24) & 0xFF); // Extract SNB_3
}

// Get device's 64-bit serial number
uint64_t SI7021::getSerialNumber()
{
    if (deviceSerialNumber > 0)
        return (deviceSerialNumber);

    uint8_t crc = 0;
    uint64_t tempSerialNumber = 0;

    _i2cPort->beginTransmission(SI7021_ADDRESS);
    _i2cPort->write(SI7021_READ_SERIAL_NUMBER_1_A);
    _i2cPort->write(SI7021_READ_SERIAL_NUMBER_1_B);
    _i2cPort->endTransmission();

    _i2cPort->requestFrom(SI7021_ADDRESS, (uint8_t)8);

    if (_i2cPort->available() == 0)
        return (0); // Error

    // The response alternates between data and CRC bytes
    // The CRC is associated with each number of bytes read
    // If you read one byte, CRC is for that one byte, if you read
    // 4, the CRC applies to the 4 bytes read.
    for (uint8_t x = 0; x < 8; x++)
    {
        if (x % 2 == 0)
        {
            tempSerialNumber <<= 8;
            tempSerialNumber |= _i2cPort->read();
        }
        else
        {
            crc = _i2cPort->read();

            if (checkCrc8((uint8_t *)&tempSerialNumber, (x / 2) + 1) != crc)
                return (0); // Error
        }
    }

    _i2cPort->beginTransmission(SI7021_ADDRESS);
    _i2cPort->write(SI7021_READ_SERIAL_NUMBER_2_A);
    _i2cPort->write(SI7021_READ_SERIAL_NUMBER_2_B);
    _i2cPort->endTransmission();

    // The B portion is only 6 bytes
    _i2cPort->requestFrom(SI7021_ADDRESS, (uint8_t)6);

    if (_i2cPort->available() == 0)
        return (0); // Error

    // The response is two data bytes, then a CRC, then 2, then CRC
    // The CRC is associated with each number of bytes read
    // If you read two bytes, CRC is for those two bytes
    for (uint8_t x = 0; x < 6; x++)
    {
        if (x == 2 || x == 5)
        {
            crc = _i2cPort->read();
            if (checkCrc8((uint8_t *)&tempSerialNumber, (x / 2) * 2) != crc)
                return (0); // Error
        }
        else
        {
            tempSerialNumber <<= 8;
            tempSerialNumber |= _i2cPort->read();
        }
    }

    deviceSerialNumber = tempSerialNumber; // All good, update the global

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
    _i2cPort->beginTransmission(SI7021_ADDRESS);
    _i2cPort->write(registerAddress);
    _i2cPort->write(value);
    _i2cPort->endTransmission();
}

// Read single register
uint8_t SI7021::readRegister8(uint8_t registerAddress)
{
    _i2cPort->beginTransmission(SI7021_ADDRESS);
    _i2cPort->write(registerAddress);
    _i2cPort->endTransmission();
    _i2cPort->requestFrom(SI7021_ADDRESS, (uint8_t)1);

    if (_i2cPort->available() != 1)
        return (SI7021_I2C_ERROR);

    uint8_t regVal = _i2cPort->read();
    return regVal;
}

// Read a given register, polling until device responds
// Store result in reading
// Can return SI7021_OK, SI7021_I2C_ERROR, or SI7021_BAD_CRC
si7021Result SI7021::getMeasurementNoHold(uint8_t registerAddress, uint16_t *reading)
{
    uint8_t bytesToRead = 3;

    // SI7021_TEMP_PREVIOUS has no CRC, 2 byte read only
    if (registerAddress == SI7021_TEMP_PREVIOUS)
        bytesToRead = 2;

    _i2cPort->beginTransmission(SI7021_ADDRESS);
    _i2cPort->write(registerAddress);
    _i2cPort->endTransmission();

    // During a "no hold" register read, we can poll the device until it is complete
    uint8_t maxWait = 100; // In ms
    for (uint8_t x = 0; x < 255; x++)
    {
        delay(1);
        if (isConnected() == true) // Device will ACK when read is complete
            break;
        if (x == maxWait)
            return (SI7021_READ_TIMEOUT);
    }

    _i2cPort->requestFrom(SI7021_ADDRESS, (uint8_t)bytesToRead);
    if (_i2cPort->available() != bytesToRead)
    {
        return (SI7021_I2C_ERROR);
    }

    uint8_t crc = 0;

    uint8_t msb = _i2cPort->read();
    uint8_t lsb = _i2cPort->read();
    uint16_t value = (uint16_t)msb << 8 | lsb;

    *reading = value; // Update caller

    // CRC does not seem to be working for RH/temp reads
    // if (bytesToRead == 3)
    // {
    //     crc = _i2cPort->read();

    //     //Serial.printf("msb: 0x%02X lsb: 0x%02X crc: 0x%02X\r\n", msb, lsb, crc);
    //     //

    //     if (checkCrc8((uint8_t *)&value, 2) != crc)
    //         return (SI7021_BAD_CRC);
    // }

    return (SI7021_OK);
}

// Based on https://forum.arduino.cc/t/crc-8-i2c-cyclic-redundancy-check/644812/3
// Byte order figured out with http://www.sunshine2k.de/coding/javascript/crc/crc_js.html - Polynomial = 0x31
uint8_t SI7021::checkCrc8(uint8_t *inputBytes, uint8_t inputLength)
{
    const uint8_t generator = 0b00110001; // CRC polynomial = x^8 + x^5 + x^4 + 1 (x^8 is ignored)
    uint8_t crc = 0;

    for (int x = 0; x < inputLength; x++)
    {
        crc ^= inputBytes[inputLength - 1 - x]; // Reverse the order. XOR-in the next input byte

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