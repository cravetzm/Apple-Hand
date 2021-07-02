/**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**/

#ifndef APPLEHAND_SENSORS_H_
#define APPLEHAND_SENSORS_H_

#include "MPL115A2.h"
#include "MPU6050.h"
#include "errors.h"

/********* Constants *********/
// Convert to m/s^2 based on half-scale range (+- 2G, 4G, 8G, 16G)
// Formula: 9.80665 [m/s^2] / [G] * HSR [G] / 2^15
const double SCALING_ACCELEROMETER = 9.80665 * 2.0 / 32768.0;
// Convert to deg/s based on half-scale range (+- 250deg/s, 500, 1000, 2000)
// Formula: HSR [deg/s] / 2^15
const double SCALING_GYROSCOPE = 250.0 / 32768.0;

/********* Communication *********/
// Addresses for ATtiny and pressure sensors
const uint8_t FINGER_MUX_I2C_ADDR = 10;
const uint8_t FINGER_MUX_REG = 0x00;
const uint8_t NUM_PRESSURE_SENSORS = 4;
const uint8_t PRESSURE_SENSOR_ADDR[] = {1, 2, 3, 4};

// Required delay for mux to transfer over, in microseconds
const uint32_t MUX_DELAY = 200;

// Number of reconnects to attempt on beginning
const uint8_t NUM_RECONNECT_ATTEMPTS = 3;

class FingerSensors {
  public:
    FingerSensors(uint8_t finger_address, uint8_t gate_address);

    bool begin(char error[ERROR_MESSAGE_LENGTH]);

    bool readIMU(float data_imu[6], char error[ERROR_MESSAGE_LENGTH]);

    bool getOnePressure(float * pressure, char error[ERROR_MESSAGE_LENGTH]);
    void startPressureConversion();
    bool readPressureConversion(float * pressure,
                                char error[ERROR_MESSAGE_LENGTH]);

    bool calibratePressureOffsets(char error[ERROR_MESSAGE_LENGTH]);
    void setPressureOffsets(float offsets[NUM_PRESSURE_SENSORS]);
    void getPressureOffsets(float offsets[NUM_PRESSURE_SENSORS]);

    void selectPressureSensor(uint8_t i);
    void selectFinger();

    uint8_t _gate_address;
    uint8_t _finger_address;

    uint8_t _pressure_sensor_current;
    float _pressure_offsets[NUM_PRESSURE_SENSORS];

    MPU6050 _imu_sensor;
    MPL115A2 _pressure_sensors[NUM_PRESSURE_SENSORS];
};

#endif // APPLEHAND_SENSORS_H_
