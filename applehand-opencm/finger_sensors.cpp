/**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**/

#include "finger_sensors.h"
#include "errors.h"

#include <I2Cdev.h>
#include "MPL115A2.h"
#include "MPU6050.h"

// Common variables and constants

FingerSensors::FingerSensors(uint8_t finger_address, uint8_t gate_address) {
  _finger_address = finger_address;
  _gate_address = gate_address;
}

// Initialization, should only be run once per finger
bool FingerSensors::begin(char error[ERROR_MESSAGE_LENGTH]) {
  // Success tracking boolean
  bool success = true;

  // Check initialization of IMU
  for (int i = 0; i < NUM_RECONNECT_ATTEMPTS; i++) {
    if (_imu_sensor.begin() == true) {
      break;
    } else {
      createIMUError(_finger_address, error);
      success = false;
    }
  }

  // Check initialization of pressure sensors
  for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
    selectPressureSensor(i);
    _pressure_sensors[i] = MPL115A2();
    for (int j = 0; j < NUM_RECONNECT_ATTEMPTS; j++) {
      if (_pressure_sensors[i].begin() == true) {
        break;
      } else {
        createPressureError(_finger_address, i, error);
        success = success && false;
      }
    }
  }

  // Calibrate the pressure sensors
  calibratePressureOffsets(error);

  return success;
}

// IMU reading, returns success
bool FingerSensors::readIMU(float data_imu[6], char error[ERROR_MESSAGE_LENGTH]) {
//  // Check if we're still connected to the IMU
//  if (_imu_sensor.testConnection() == false) {
//    createIMUError(_finger_address, error);
//    _imu_sensor.begin();
//    return false;
//  }
  
  // Get IMU readings
  int16_t ax, ay, az, gx, gy, gz;
  _imu_sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // If they're all 0's, it was a failed reading somehow
  if (ax == 0.00F && ay == 0.00F && az == 0.00F &&
      gx == 0.00F && gy == 0.00F && gz == 0.00F) {
    // Re-initialize the IMU or it'll read 0's forever
    _imu_sensor.begin();
    return false;
  }

  // Otherwise, apply scaling and convert
  data_imu[0] = ax * SCALING_ACCELEROMETER;
  data_imu[1] = ay * SCALING_ACCELEROMETER;
  data_imu[2] = az * SCALING_ACCELEROMETER;
  data_imu[3] = gx * SCALING_GYROSCOPE;
  data_imu[4] = gy * SCALING_GYROSCOPE;
  data_imu[5] = gz * SCALING_GYROSCOPE;

  return true;
}

bool FingerSensors::getOnePressure(float * pressure,
                                   char error[ERROR_MESSAGE_LENGTH]) {
  *pressure = _pressure_sensors[_pressure_sensor_current].getPressure();
  if (*pressure == 50.0F) {
    createPressureError(_finger_address, _pressure_sensor_current, error);
    return false;
  } else {
    *pressure = *pressure - _pressure_offsets[_pressure_sensor_current];
    return true;
  }
}

void FingerSensors::startPressureConversion() {
  _pressure_sensors[_pressure_sensor_current].startConversion();
}

bool FingerSensors::readPressureConversion(float * pressure,
    char error[ERROR_MESSAGE_LENGTH]) {
  _pressure_sensors[_pressure_sensor_current].readConversionResults();
  *pressure = _pressure_sensors[_pressure_sensor_current].getLastPressure();
  if (*pressure == 50.0F) {
    createPressureError(_finger_address, _pressure_sensor_current, error);
    return false;
  } else {
    *pressure = *pressure - _pressure_offsets[_pressure_sensor_current];
    return true;
  }
}

bool FingerSensors::calibratePressureOffsets(char error[ERROR_MESSAGE_LENGTH]) {
  bool success = true;
  for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
    selectPressureSensor(i);
    if (getOnePressure(&_pressure_offsets[i], error) == false) {
      success = false;
    }
  }
  return success;
}

void FingerSensors::setPressureOffsets(float offsets[NUM_PRESSURE_SENSORS]) {
  for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
    _pressure_offsets[i] = offsets[i];
  }
}

void FingerSensors::getPressureOffsets(float offsets[NUM_PRESSURE_SENSORS]) {
  for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
    offsets[i] = _pressure_offsets[i];
  }
}

void FingerSensors::selectPressureSensor(uint8_t i) {
  I2Cdev::writeByte(FINGER_MUX_I2C_ADDR, FINGER_MUX_REG,
                    PRESSURE_SENSOR_ADDR[i]);
  delayMicroseconds(MUX_DELAY);
  _pressure_sensor_current = i;
}

void FingerSensors::selectFinger() {
  I2Cdev::writeByte(_gate_address, 0, 1 << _finger_address);
  delayMicroseconds(MUX_DELAY);
}
