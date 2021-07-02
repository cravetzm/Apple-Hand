/**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**/
/*!
    This library was adapted from the Adafruit_MPL1152A2 library but
    converted to use I2Cdev.h instead of Wire.h, as Wire.h is improperly
    implemented for I2C on the OpenCM.

    Original information:
    @mainpage Driver for the Adafruit MPL115A2 barometric pressure sensor

    @section intro_sec Introduction
      Driver for the MPL115A2 barometric pressure sensor
      This is a library for the Adafruit MPL115A2 breakout
      ----> https://www.adafruit.com/products/992
      Adafruit invests time and resources providing this open source code,
      please support Adafruit and open-source hardware by purchasing
      products from Adafruit!

    @section author Author
    K.Townsend (Adafruit Industries)

    @section license License
    BSD (see license.txt)

    @section history
      v1.0 - First release
      v1.1 - Rick Sellens added casts to make bit shifts work below 22.6C
      get both P and T with a single call to getPT
      v1.2 - Lisa Dischinger (from IMML at Oregon State University)
      converted this code from using Wire.h to I2Cdev.h.
      v1.3 - Brian Zhang (from SHARE at Oregon State University) split
      functions to allow for parallel conversions.
*/

#include <I2Cdev.h>
#include "MPL115A2.h"

/*!
    @brief  Gets the factory-set coefficients for this particular sensor
*/
void MPL115A2::readCoefficients() {

  int16_t a0coeff;
  int16_t b1coeff;
  int16_t b2coeff;
  int16_t c12coeff;
  
  /* Wire.h version
       _wire->beginTransmission(_i2caddr);
      i2cwrite(_wire, (uint8_t)MPL115A2_REGISTER_A0_COEFF_MSB);
      _wire->endTransmission();

      _wire->requestFrom(_i2caddr, (uint8_t)8);
      a0coeff = (((uint16_t)i2cread(_wire) << 8) | i2cread(_wire));
      b1coeff = (((uint16_t)i2cread(_wire) << 8) | i2cread(_wire));
      b2coeff = (((uint16_t)i2cread(_wire) << 8) | i2cread(_wire));
      c12coeff = (((uint16_t)(i2cread(_wire) << 8) | i2cread(_wire))) >> 2;
  */

  I2Cdev::readByte(_i2caddr, (uint8_t)MPL115A2_REGISTER_A0_COEFF_MSB, buffer);
  uint8_t a0coeff_MSB = buffer[0];
  I2Cdev::readByte(_i2caddr, (uint8_t)MPL115A2_REGISTER_A0_COEFF_LSB, buffer);
  uint8_t a0coeff_LSB = buffer[0];
  I2Cdev::readByte(_i2caddr, (uint8_t)MPL115A2_REGISTER_B1_COEFF_MSB, buffer);
  uint8_t b1coeff_MSB = buffer[0];
  I2Cdev::readByte(_i2caddr, (uint8_t)MPL115A2_REGISTER_B1_COEFF_LSB, buffer);
  uint8_t b1coeff_LSB = buffer[0];
  I2Cdev::readByte(_i2caddr, (uint8_t)MPL115A2_REGISTER_B2_COEFF_MSB, buffer);
  uint8_t b2coeff_MSB = buffer[0];
  I2Cdev::readByte(_i2caddr, (uint8_t)MPL115A2_REGISTER_B2_COEFF_LSB, buffer);
  uint8_t b2coeff_LSB = buffer[0];
  I2Cdev::readByte(_i2caddr, (uint8_t)MPL115A2_REGISTER_C12_COEFF_MSB, buffer);
  uint8_t c12coeff_MSB = buffer[0];
  I2Cdev::readByte(_i2caddr, (uint8_t)MPL115A2_REGISTER_C12_COEFF_LSB, buffer);
  uint8_t c12coeff_LSB = buffer[0];

  a0coeff = (((uint16_t)a0coeff_MSB << 8) | a0coeff_LSB);
  b1coeff = (((uint16_t)b1coeff_MSB << 8) | b1coeff_LSB);
  b2coeff = (((uint16_t)b2coeff_MSB << 8) | b2coeff_LSB);
  c12coeff = (((uint16_t)c12coeff_MSB << 8) | c12coeff_LSB) >> 2;

  _mpl115a2_a0 = (float)a0coeff / 8;
  _mpl115a2_b1 = (float)b1coeff / 8192;
  _mpl115a2_b2 = (float)b2coeff / 16384;
  _mpl115a2_c12 = (float)c12coeff;
  _mpl115a2_c12 /= 4194304.0;
}

/*!
    @brief  Instantiates a new MPL115A2 class
*/
MPL115A2::MPL115A2() {
  _mpl115a2_a0 = 0.0F;
  _mpl115a2_b1 = 0.0F;
  _mpl115a2_b2 = 0.0F;
  _mpl115a2_c12 = 0.0F;
}

/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @return true for success (nonzero readings), false for no success
*/
bool MPL115A2::begin() {
  _i2caddr = MPL115A2_DEFAULT_ADDRESS;
  //_wire = &I2Cdev;
  // Read factory coefficient values (this only needs to be done once)
  readCoefficients();

  return (getPressure() != 50.0F);
}

/*!
    @brief  Runs through whole conversion process, updating internal values
*/
void MPL115A2::read() {
  startConversion();
  delay(3);
  readConversionResults();
}

/*!
    @brief  Begins the conversion - should occur >3ms before reading values
*/
void MPL115A2::startConversion() {
  // This conversion may take up to 3ms
  I2Cdev::writeByte(_i2caddr, (uint8_t)MPL115A2_REGISTER_STARTCONVERSION,
                    (uint8_t)0x00);
}

/*!
    @brief  Reads the sensor's registers and updates internal values
*/
void MPL115A2::readConversionResults() {
  uint8_t pressure_MSB, pressure_LSB, temp_MSB, temp_LSB;
  uint16_t pressure_raw, temperature_raw;
  float pressure_compensated;

  I2Cdev::readByte(_i2caddr,
                   (uint8_t)MPL115A2_REGISTER_PRESSURE_MSB, &pressure_MSB);
  I2Cdev::readByte(_i2caddr,
                   (uint8_t)MPL115A2_REGISTER_PRESSURE_LSB, &pressure_LSB);
  I2Cdev::readByte(_i2caddr,
                   (uint8_t)MPL115A2_REGISTER_TEMP_MSB, &temp_MSB);
  I2Cdev::readByte(_i2caddr,
                   (uint8_t)MPL115A2_REGISTER_TEMP_LSB, &temp_LSB);

  pressure_raw = (((uint16_t)pressure_MSB << 8) | pressure_LSB) >> 6;
  temperature_raw = (((uint16_t)temp_MSB << 8) | temp_LSB) >> 6;

  // See datasheet p.6 for evaluation sequence
  pressure_compensated = _mpl115a2_a0 +
                         (_mpl115a2_b1 +
                          _mpl115a2_c12 * temperature_raw) * pressure_raw +
                         _mpl115a2_b2 * temperature_raw;

  // Return pressure and temperature as floating point values
  _pressure_kpa = ((65.0F / 1023.0F) * pressure_compensated) + 50.0F; // kPa
  _temp_c = ((float)temperature_raw - 498.0F) / -5.35F + 25.0F;   // C
}

/*!
    @brief  Performs a new conversion to get the current pressure
    @return Pressure in kPa
*/
float MPL115A2::getPressure() {
  read();
  return getLastPressure();
}

/*!
    @brief  Retrieves last conversion's pressure
    @return Pressure in kPa
*/
float MPL115A2::getLastPressure() {
  return _pressure_kpa;
}

/*!
    @brief  Performs a new conversion to get the current temperature
    @return Temperature in Centigrade
*/
float MPL115A2::getTemperature() {
  read();
  return getLastTemperature();
}

/*!
    @brief  Retrieves last conversion's pressure
    @return Temperature in Centigrade
*/
float MPL115A2::getLastTemperature() {
  return _temp_c;
}
