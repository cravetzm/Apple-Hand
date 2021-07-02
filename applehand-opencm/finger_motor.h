/**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**/
// We are using a Dynamixel XM430 motor. Please check to see whether the
// control modes used are applicable to any replacement motor.


#ifndef APPLEHAND_MOTOR_H_
#define APPLEHAND_MOTOR_H_

#include <Dynamixel2Arduino.h>

class FingerMotor {
  public:
    FingerMotor(Dynamixel2Arduino dxl, uint8_t id);
    bool begin();

    bool turnOffTorque();
    bool goTo(float position_degrees, float current_milliamps);
    void getJointState(float data[3]);

    uint8_t _id;
    Dynamixel2Arduino _dxl;
};

#endif // APPLEHAND_MOTOR_H_
