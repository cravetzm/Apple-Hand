/**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**/
// Derived from examples/basic/id as provided by the
// Dynamixel2Arduino package.

#include <Dynamixel2Arduino.h>

/********* Parameters *********/
// Set the change the appropriate motor and baud rates
// Use dynamixel_scanner to detect motors and current baud rates
uint8_t dynamixel_id_old = 3;
// set a new ID for DYNAMIXEL. Do not use ID 200
uint8_t dynamixel_id_new = 2;
float dynamixel_protocol = 2.0;
uint32_t dynamixel_baudrate = 1000000;

/********* Communication *********/
// Set for the OpenCM 9.04 board.
#define DynamixelSerial   Serial1
const uint8_t kDynamixelPin = 28;
// Set as Serial (for USB) or Serial2 (for wired).
// Serial1 is reserved for the Dynamixel TTL and Serial3 is for I2C.
#define SelectedSerial    Serial
// USB serial doesn't actually use this, but Serial2 (and ROS) does.
const bool kSerialBaudRateHz = 250000;

Dynamixel2Arduino dxl(DynamixelSerial, kDynamixelPin);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  SelectedSerial.begin(kSerialBaudRateHz);
  while (!SelectedSerial) {}

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(dynamixel_baudrate);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(dynamixel_protocol);

  SelectedSerial.print("Protocol ");
  SelectedSerial.print(dynamixel_protocol, 1);
  SelectedSerial.print(", baud rate ");
  SelectedSerial.print(dynamixel_baudrate);
  SelectedSerial.print(", ID ");
  SelectedSerial.print(dynamixel_id_old);
  SelectedSerial.print(": ");
  if (dxl.ping(dynamixel_id_old) == true) {
    SelectedSerial.println("ping succeeded!");

    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(dynamixel_id_old);

    if (dxl.setID(dynamixel_id_old, dynamixel_id_new) == true) {
      SelectedSerial.print("ID has been successfully changed to ");
      SelectedSerial.println(dynamixel_id_new);
    } else {
      SelectedSerial.print("Failed to change ID to ");
      SelectedSerial.println(dynamixel_id_old);
    }
  }
  else {
    SelectedSerial.println("ping failed!");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
