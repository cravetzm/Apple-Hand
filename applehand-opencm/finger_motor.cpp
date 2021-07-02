/**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**/
// We are using a Dynamixel XM430 motor. Please check to see whether the
// control modes used are applicable to any replacement motor.

#include <Dynamixel2Arduino.h>
#include "finger_motor.h"

FingerMotor::FingerMotor(Dynamixel2Arduino dxl, uint8_t id) {
  _dxl = dxl;
  _id = id;
}

bool FingerMotor::begin() {
  bool success = true;
  success = _dxl.torqueOff(_id) && success;
  success = _dxl.setOperatingMode(_id, OP_CURRENT_BASED_POSITION) && success;
  return success;
}

bool FingerMotor::turnOffTorque() {
  return _dxl.torqueOff(_id);
}

bool FingerMotor::goTo(float position_degrees, float current_milliamps) {
  bool success = true;
  success = _dxl.torqueOn(_id) && success;
  success = _dxl.setGoalPosition(_id, position_degrees, UNIT_DEGREE)
            && success;
  success = _dxl.setGoalCurrent(_id, current_milliamps, UNIT_MILLI_AMPERE)
            && success;
  return success;
}

void FingerMotor::getJointState(float data[3]) {
  // {position_degrees, velocity_degrees_per_second, effort_milliamps}
  data[0] = _dxl.getPresentPosition(_id, UNIT_DEGREE);
  data[1] = _dxl.getPresentVelocity(_id, UNIT_RPM) * 360.0 / 60.0;
  data[2] = _dxl.getPresentCurrent(_id, UNIT_MILLI_AMPERE);
}
