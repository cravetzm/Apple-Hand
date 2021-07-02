/**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**/
// Derived from examples/basic/scan_dynamixel as provided by the
// Dynamixel2Arduino package.

#include <Dynamixel2Arduino.h>

// Set for the OpenCM 9.04 board.
#define DynamixelSerial   Serial1
const uint8_t kDynamixelPin = 28;
// Set as Serial (for USB) or Serial2 (for wired).
// Serial1 is reserved for the Dynamixel TTL and Serial3 is for I2C.
#define SelectedSerial    Serial
// USB serial doesn't actually use this, but Serial2 (and ROS) does.
const bool kSerialBaudRateHz = 250000;

// Dynamixel baud rates
#define MAX_BAUD  5
const int32_t baud[MAX_BAUD] = {57600, 115200, 1000000, 2000000, 3000000};

Dynamixel2Arduino dxl(DynamixelSerial, kDynamixelPin);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // Use a different serial port to debug
  SelectedSerial.begin(kSerialBaudRateHz);
  while (!SelectedSerial) {}
}

void loop() {
  int8_t index = 0;
  int8_t found_dynamixel = 0;

  // Check all protocols and baud rates.
  for (int8_t protocol = 1; protocol < 3; protocol++) {
    // Set Port Protocol Version. This must match with DYNAMIXEL protocol.
    dxl.setPortProtocolVersion((float)protocol);
    SelectedSerial.print("Scanning using protocol ");
    SelectedSerial.print(protocol);
    SelectedSerial.println(" with baud rate...");

    for (index = 0; index < MAX_BAUD; index++) {
      // Set Port baudrate.
      SelectedSerial.print("  ");
      SelectedSerial.print(baud[index]);
      SelectedSerial.println(": ");
      
      dxl.begin(baud[index]);
      
      for (int id = 0; id < DXL_BROADCAST_ID; id++) {
        //iterate until all ID in each buadrate is scanned.
        if (dxl.ping(id)) {
          SelectedSerial.print("    Found motor with ID ");
          SelectedSerial.print(id);
          SelectedSerial.print(", model number ");
          SelectedSerial.println(dxl.getModelNumber(id));
          found_dynamixel++;
        }
      }
    }
  }

  SelectedSerial.print(found_dynamixel);
  SelectedSerial.println(" DYNAMIXEL(s) found! Rescanning in 5 seconds...");

  delay(5000);
}
