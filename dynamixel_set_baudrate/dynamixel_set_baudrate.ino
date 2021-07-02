/**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**/
// Derived from examples/basic/baudrate as provided by the
// Dynamixel2Arduino package.

#include <Dynamixel2Arduino.h>

/********* Parameters *********/
// Set the change the appropriate motor and baud rates
// Use dynamixel_scanner to detect motors and current baud rates
uint8_t dynamixel_id = 2;
float dynamixel_protocol = 2.0;
uint32_t old_baudrate = 1000000;
uint32_t new_baudrate = 3000000; //1Mbsp

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

// This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:

  // Use UART port of DYNAMIXEL Shield to debug.
  SelectedSerial.begin(kSerialBaudRateHz);
  while (!SelectedSerial) {}

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(old_baudrate);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(dynamixel_protocol);

  SelectedSerial.print("Protocol ");
  SelectedSerial.print(dynamixel_protocol, 1);
  SelectedSerial.print(", baud rate ");
  SelectedSerial.print(old_baudrate);
  SelectedSerial.print(", ID ");
  SelectedSerial.print(dynamixel_id);
  SelectedSerial.print(": ");

  if (dxl.ping(dynamixel_id) == true) {
    SelectedSerial.println("ping succeeded!");

    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(dynamixel_id);

    // Set a new baudrate(1Mbps) for DYNAMIXEL
    dxl.setBaudrate(dynamixel_id, new_baudrate);
    SelectedSerial.print("Baud rate has been changed to:");
    Serial.println(new_baudrate);

    // Change to the new baudrate for communication.
    dxl.begin(new_baudrate);

    SelectedSerial.print("Pinging on new baud rate: ");
    // Change back to the initial baudrate
    if (dxl.ping(dynamixel_id) == true) {
      SelectedSerial.println("ping succeeded!");
    } else {
      SelectedSerial.println("ping failed!");
    }
  } else {
    SelectedSerial.println("ping failed!");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
