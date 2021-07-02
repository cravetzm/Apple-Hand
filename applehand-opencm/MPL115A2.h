#ifndef _MPL1152A2_H_
#define _MPL1152A2_H_

#include <I2Cdev.h>

/**< I2C address **/
#define MPL115A2_DEFAULT_ADDRESS (0x60)
/**< 10-bit Pressure ADC output value most-significant-bit (MSB) **/
#define MPL115A2_REGISTER_PRESSURE_MSB      (0x00)
/**< 10-bit Pressure ADC output value LSB **/
#define MPL115A2_REGISTER_PRESSURE_LSB      (0x01)
/**< 10-bit Temperature ADC output value MSB **/
#define MPL115A2_REGISTER_TEMP_MSB          (0x02)
/**< 10-bit Temperature ADC output value LSB **/
#define MPL115A2_REGISTER_TEMP_LSB          (0x03)

#define MPL115A2_REGISTER_A0_COEFF_MSB      (0x04)  /**< a0 coefficient MSB **/
#define MPL115A2_REGISTER_A0_COEFF_LSB      (0x05)  /**< a0 coefficient LSB **/
#define MPL115A2_REGISTER_B1_COEFF_MSB      (0x06)  /**< b1 coefficient MSB **/
#define MPL115A2_REGISTER_B1_COEFF_LSB      (0x07)  /**< b1 coefficient LSB **/
#define MPL115A2_REGISTER_B2_COEFF_MSB      (0x08)  /**< b2 coefficient MSB **/
#define MPL115A2_REGISTER_B2_COEFF_LSB      (0x09)  /**< b2 coefficient LSB **/
#define MPL115A2_REGISTER_C12_COEFF_MSB     (0x0A) /**< c12 coefficient MSB **/
#define MPL115A2_REGISTER_C12_COEFF_LSB     (0x0B) /**< c12 coefficient LSB **/
#define MPL115A2_REGISTER_STARTCONVERSION   (0x12) /**< conversion start bit **/

class MPL115A2 {
  public:
    MPL115A2();
    bool begin();
    void read();
    void startConversion();
    void readConversionResults();
    float getPressure();
    float getLastPressure();
    float getTemperature();
    float getLastTemperature();

  private:
    uint8_t _i2caddr;
    uint8_t buffer[10];

    float _mpl115a2_a0;
    float _mpl115a2_b1;
    float _mpl115a2_b2;
    float _mpl115a2_c12;

    float _pressure_kpa;
    float _temp_c;

    void readCoefficients();
};

#endif
