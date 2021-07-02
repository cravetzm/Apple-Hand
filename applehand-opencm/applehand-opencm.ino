/**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**/

/********* Constants *********/
const bool USE_ROSSERIAL = true;
const bool READ_IMUS = true;
const bool READ_PRESSURES = false;
const bool READ_JOINT_STATES = true;

// Set as Serial (for USB) or Serial2 (for wired).
// Serial1 is reserved for the Dynamixel TTL and Serial3 is for I2C.
#define SelectedSerial Serial
// Baud rate in Hz. USBSerial (Serial) is adaptive, but Serial2 (and ROS) does.
const bool SERIAL_BAUD_RATE = 500000;

// I2C baud rate in kHz, 300kHz is the upper limit on stablility
const uint16_t I2C_BAUD_RATE = 300;
// Address, relative to the OpenCM, of the I2C mux that connects to the fingers
const uint8_t PALM_MUX_ADDR = 0x70;

// Number of fingers
const uint8_t NUM_FINGERS = 3;
// Addresses, relative to the I2C mux, of the ATtiny muxes on the fingers
const uint8_t FINGER_MUX_ADDR[NUM_FINGERS] = {0, 1, 2};
// Addresses, relative to the OpenCM, of the Dynamixel motors
const uint8_t SERVO_ADDR[NUM_FINGERS] = {0, 1, 2};

// Set for the OpenCM 9.04 board.
#define DynamixelSerial   Serial1
const uint8_t DYNAMIXEL_PIN = 28;
const uint32_t DYNAMIXEL_BAUD_RATE = 3000000;
const float DYNAMIXEL_PROTOCOL = 2.0;

// This current limit, in mA, prevents over-current on the UR5e.
// Based off of a 12V, 600mA continuous spec. Could be raised if set to
// 24V and stepped down, or through testing to determine real allowable current.
const uint16_t SERVO_CURRENT_LIMIT = 200;
// Servo positions in degrees
const float SERVO_POSITIONS_OPEN[3] = {76, 320, -12.5};
const float SERVO_POSITIONS_CLOSED[3] = {146, 210, -143};

// Note: we are using XM430s, the OP_CURRENT_BASED_POSITION control mode is a
// newer feature that may not be available for all Dynamixels

/******************* Base ROS Setup *******************/
// General ROS packages/nodes
#include <ros.h>
ros::NodeHandle nh;

#include <std_msgs/String.h>
std_msgs::String messages_message;
ros::Publisher publisher_messages("/applehand/messages", &messages_message);

/******************* ROS Services Setup *******************/
#include <std_srvs/Trigger.h>
void closeHandService(const std_srvs::Trigger::Request &req,
                      std_srvs::Trigger::Response &res) {
  res.success = closeHand();
}
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_close("applehand/close_hand", &closeHandService);

void openHandService(const std_srvs::Trigger::Request &req,
                     std_srvs::Trigger::Response &res) {
  res.success = openHand();
}
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_open("applehand/open_hand", &openHandService);

void relaxHandService(const std_srvs::Trigger::Request &req,
                      std_srvs::Trigger::Response &res) {
  res.success = relaxHand();
}
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_relax("applehand/relax_hand", &relaxHandService);

/******************* Sensor Setup *******************/
#include <I2Cdev.h>
#include "finger_sensors.h"
#include <applehand/ImuNaive.h>
#include <applehand/PressureArrayStamped.h>

float data_imu[NUM_FINGERS][6];
applehand::ImuNaive fingertip_imu_data[NUM_FINGERS];

float data_pressure[NUM_FINGERS][NUM_PRESSURE_SENSORS];
applehand::PressureArrayStamped fingertip_pressure_data[NUM_FINGERS];

FingerSensors sensors[NUM_FINGERS] = {
  FingerSensors(FINGER_MUX_ADDR[0], PALM_MUX_ADDR),
  FingerSensors(FINGER_MUX_ADDR[1], PALM_MUX_ADDR),
  FingerSensors(FINGER_MUX_ADDR[2], PALM_MUX_ADDR)
};

ros::Publisher publisher_finger_imu[NUM_FINGERS] = {
  ros::Publisher("/applehand/finger1/imu", &fingertip_imu_data[0]),
  ros::Publisher("/applehand/finger2/imu", &fingertip_imu_data[1]),
  ros::Publisher("/applehand/finger3/imu", &fingertip_imu_data[2])
};

ros::Publisher publisher_finger_pressures[NUM_FINGERS] = {
  ros::Publisher("/applehand/finger1/pressures", &fingertip_pressure_data[0]),
  ros::Publisher("/applehand/finger2/pressures", &fingertip_pressure_data[1]),
  ros::Publisher("/applehand/finger3/pressures", &fingertip_pressure_data[2])
};

/******************* Motor Setup *******************/
#include <Dynamixel2Arduino.h>
#include "finger_motor.h"
#include <applehand/SingleJointState.h>

Dynamixel2Arduino dxl(DynamixelSerial, DYNAMIXEL_PIN);

FingerMotor motors[NUM_FINGERS] = {
  FingerMotor(dxl, SERVO_ADDR[0]),
  FingerMotor(dxl, SERVO_ADDR[1]),
  FingerMotor(dxl, SERVO_ADDR[2])
};

float data_joints[NUM_FINGERS][3];
applehand::SingleJointState joint_state_data[NUM_FINGERS];

ros::Publisher publisher_joint_states[NUM_FINGERS] = {
  ros::Publisher("/applehand/finger1/jointstate", &joint_state_data[0]),
  ros::Publisher("/applehand/finger2/jointstate", &joint_state_data[1]),
  ros::Publisher("/applehand/finger3/jointstate", &joint_state_data[2])
};

/******************* Error Setup *******************/
#include "errors.h"
char error_message[ERROR_MESSAGE_LENGTH];

/******************* Loop Setup *******************/
char serial_message;
uint32_t num_packet = 0;
unsigned long last_time = 0;

/******************* Setup *******************/
void setup() {
  // Only call this ONE TIME or it will lag all I2C comms
  I2Cdev::begin(I2C_BAUD_RATE);

  // Set up communication
  if (USE_ROSSERIAL == true) {
    // Initialize ROS node
    nh.getHardware()->setBaud(SERIAL_BAUD_RATE);
    nh.initNode();

    // Advertise publishers
    nh.advertise(publisher_messages);
    for (int f = 0; f < NUM_FINGERS; f++) {
      nh.advertise(publisher_finger_imu[f]);
      nh.advertise(publisher_finger_pressures[f]);
      nh.advertise(publisher_joint_states[f]);
    }

    // Advertise services
    nh.advertiseService(service_close);
    nh.advertiseService(service_open);
    nh.advertiseService(service_relax);

    // Send start-up message
    messages_message.data = "Starting AppleHand...";
    publisher_messages.publish(&messages_message);
  } else {
    // Communicate through Serial instead
    SelectedSerial.begin(SERIAL_BAUD_RATE);
    while (!SelectedSerial) {}
    SelectedSerial.println("Starting AppleHand on Serial...");
  }

  // Set up sensors, retrying continuously if not successful
  bool success = false;
  while (!success) {
    success = true;
    resetError(error_message);

    // Attempt to start each finger
    for (int i = 0; i < NUM_FINGERS; i++) {
      SelectedSerial.println(i);
      sensors[i].selectFinger();

      if (sensors[i].begin(error_message) == true) {
        success = success && true;
      } else {
        success = success && false;
        // If starting the finger is unsuccessful, error out and retry
        if (USE_ROSSERIAL == true) {
          sendMessageThroughROS(error_message);
          sendMessageThroughROS("Sensor connection failed, will retry...");
        } else {
          SelectedSerial.print("Sensor connection failed: ");
          SelectedSerial.print(error_message);
          SelectedSerial.println(" Will retry...");
        }
      }
    }
  }

  // Set up motors
  success = false;
  while (!success) {
    success = true;

    resetError(error_message);

    dxl.begin(DYNAMIXEL_BAUD_RATE);
    dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL);

    for (int i = 0; i < NUM_FINGERS; i++) {

      if (motors[i].begin() == true) {
        success = success && true;
      } else {
        success = success && false;
        if (USE_ROSSERIAL == true) {
          messages_message.data =
            "AppleHand motor connection failed, retrying...";
          publisher_messages.publish(&messages_message);
        } else {
          SelectedSerial.print("AppleHand motor connection failed: ");
          SelectedSerial.print(error_message);
          SelectedSerial.println("retrying...");
        }
      }
    }
  }

  if (USE_ROSSERIAL == true) {
    messages_message.data = "AppleHand started successfully!";
    publisher_messages.publish(&messages_message);
  } else {
    SelectedSerial.println("AppleHand started successfully!");
  }
}

/******************* Loop *******************/
void loop() {
  // Check for service calls
  if (USE_ROSSERIAL) {
    nh.spinOnce();
  } else {
    // Equivalent to services - accept some serial commands
    if (SelectedSerial.available()) {
      processSerialInput(SelectedSerial.read());
    }
  }

  // Get that data
  bool success;
  //  success = getAllDataInSeries();
  success = getAllDataInParallel();

  // Send that data (and errors if needed)
  if (USE_ROSSERIAL) {
    // Process ROS callbacks, printing takes time
    nh.spinOnce();
    sendDataThroughROS();
    if (success == false) {
      sendMessageThroughROS(error_message);
    }
  } else {
    sendDataThroughSerial();
  }

  num_packet++;
}

/******************* Data Reading Helper Functions *******************/
// Reads all sensors front-to-back, one-by-one
bool getAllDataInSeries() {
  for (int f = 0; f < NUM_FINGERS; f++) {
    if (READ_IMUS == true || READ_PRESSURES == true) {
      sensors[f].selectFinger();
    }

    if (READ_IMUS == true) {
      sensors[f].readIMU(data_imu[f], error_message);
    }

    if (READ_PRESSURES == true) {
      for (int p = 0; p < NUM_PRESSURE_SENSORS; p++) {
        sensors[f].selectPressureSensor(p);
        sensors[f].getOnePressure(&data_pressure[f][p], error_message);
      }
    }

    if (READ_JOINT_STATES == true) {
      motors[f].getJointState(data_joints[f]);
    }
  }
}

// Reads from the pressure sensors in parallel by kicking off conversions,
// then returning to scoop up the data. Faster than getSensorDataInSeries().
bool getAllDataInParallel() {
  // Start all of the conversions and read IMUs
  for (int f = NUM_FINGERS - 1; f >= 0 ; f--) {
    if (READ_IMUS == true || READ_PRESSURES == true) {
      // Default finger is last finger, no need to switch
      if (f != NUM_FINGERS - 1) {
        sensors[f].selectFinger();
      }
    }

    // Select from last to first and start the conversion
    if (READ_PRESSURES == true) {
      for (int p = NUM_PRESSURE_SENSORS - 1; p >= 0 ; p--) {
        if (p != NUM_PRESSURE_SENSORS - 1) {
          sensors[f].selectPressureSensor(p);
        }
        sensors[f].startPressureConversion();
      }
    }

    // Read IMU and motor to save time
    if (READ_IMUS == true) {
      sensors[f].readIMU(data_imu[f], error_message);
    }

    if (READ_JOINT_STATES == true) {
      motors[f].getJointState(data_joints[f]);
    }
  }

  // We assume that getting the IMU and joint data takes > 3 ms, otherwise wait
  if (READ_PRESSURES && !READ_IMUS && !READ_JOINT_STATES) {
    delay(3);
  }

  // Read the data from the pressure sensors
  if (READ_PRESSURES == true) {
    for (int f = 0; f < NUM_FINGERS; f++) {
      if (f != 0) {
        sensors[f].selectFinger();
      }

      for (int p = 0; p < NUM_PRESSURE_SENSORS; p++) {
        if (p != 0) {
          sensors[f].selectPressureSensor(p);
        }
        sensors[f].readPressureConversion(&data_pressure[f][p], error_message);
      }
    }
  }
}

/******************* Control Functions *******************/
bool closeHand() {
  bool success = true;
  for (int f = 0; f < NUM_FINGERS; f++) {
    success = motors[f].goTo(SERVO_POSITIONS_CLOSED[f], SERVO_CURRENT_LIMIT)
              && success;
  }
  return success;
}

bool openHand() {
  bool success = true;
  for (int f = 0; f < NUM_FINGERS; f++) {
    success = motors[f].goTo(SERVO_POSITIONS_OPEN[f], SERVO_CURRENT_LIMIT)
              && success;
  }
  return success;
}

bool relaxHand() {
  bool success = true;
  for (int f = 0; f < NUM_FINGERS; f++) {
    success = motors[f].turnOffTorque() && success;
  }
  return success;
}

/******************* ROS Publishing Helper Functions *******************/
void sendDataThroughROS() {
  ros::Time t = nh.now();
  for (int f = 0; f < NUM_FINGERS; f++) {
    if (READ_IMUS == true) {
      fingertip_imu_data[f].header.seq = num_packet;
      fingertip_imu_data[f].header.stamp = t;

      fingertip_imu_data[f].linear_acceleration.x = data_imu[f][0];
      fingertip_imu_data[f].linear_acceleration.y = data_imu[f][1];
      fingertip_imu_data[f].linear_acceleration.z = data_imu[f][2];

      fingertip_imu_data[f].angular_velocity.x = data_imu[f][3];
      fingertip_imu_data[f].angular_velocity.y = data_imu[f][4];
      fingertip_imu_data[f].angular_velocity.z = data_imu[f][5];

      publisher_finger_imu[f].publish(&fingertip_imu_data[f]);
    }
    if (READ_PRESSURES == true) {
      fingertip_pressure_data[f].header.seq = num_packet;
      fingertip_pressure_data[f].header.stamp = t;

      fingertip_pressure_data[f].pressures_length = NUM_PRESSURE_SENSORS;

      fingertip_pressure_data[f].pressures = data_pressure[f];

      publisher_finger_pressures[f].publish(&fingertip_pressure_data[f]);
    }

    if (READ_JOINT_STATES == true) {
      joint_state_data[f].header.seq = num_packet;
      joint_state_data[f].header.stamp = t;

      joint_state_data[f].position = data_joints[f][0];
      joint_state_data[f].velocity = data_joints[f][1];
      joint_state_data[f].effort = data_joints[f][2];

      publisher_joint_states[f].publish(&joint_state_data[f]);
    }
  }
}

void sendMessageThroughROS(char * message) {
  messages_message.data = message;
  publisher_messages.publish(&messages_message);
}

/******************* Serial Communication Helper Functions *******************/
void sendDataThroughSerial() {
  // Header equivalent block: packet number, time, and frequency (Hz)
  SelectedSerial.print(num_packet); SelectedSerial.print(", ");
  SelectedSerial.print(millis()); SelectedSerial.print(", ");
  SelectedSerial.print(1000.0 / (millis() - last_time));
  SelectedSerial.print(", ");
  last_time = millis();

  // Sensor & motor data
  for (int f = 0; f < NUM_FINGERS; f++) {
    char message[] = {'F', '0', ',', ' ', '\0'};
    message[1] = message[1] + f;
    Serial.print(message);
    if (READ_IMUS == true) {
      Serial.print("IMU, ");
      for (int i = 0; i < 6; i++) {
        SelectedSerial.print(data_imu[f][i]);
        SelectedSerial.print(", ");
      }
    }
    if (READ_PRESSURES == true) {
      Serial.print("Pressure, ");

      for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
        SelectedSerial.print(data_pressure[f][i]); SelectedSerial.print(", ");
      }
    }
    if (READ_JOINT_STATES == true) {
      Serial.print("Joint State, ");
      for (int i = 0; i < 3; i++) {
        SelectedSerial.print(data_joints[f][i]); SelectedSerial.print(", ");
      }
    }
  }

  // Error handling
  SelectedSerial.println(error_message);
  resetError(error_message);
}

void processSerialInput(byte in) {
  // These are custom and must be changed for any major changes to the hand!!!
  //            Close fully    Close a bit    Relax   Open a bit    Open fully
  //  Finger 0:     q               w           e         r             t
  //  Finger 1:     a               s           d         f             g
  //  Finger 2:     z               x           c         v             b
  //
  //  closeHand()   relaxHand()   openHand()
  //      ,             .             /

  switch (in) {
    // Close finger all the way
    case 'q':
      motors[0].goTo(SERVO_POSITIONS_CLOSED[0], SERVO_CURRENT_LIMIT);
      break;
    case 'a':
      motors[1].goTo(SERVO_POSITIONS_CLOSED[1], SERVO_CURRENT_LIMIT);
      break;
    case 'z':
      motors[2].goTo(SERVO_POSITIONS_CLOSED[2], SERVO_CURRENT_LIMIT);
      break;
    // Close finger a little bit
    case 'w':
      motors[0].goTo(data_joints[0][0] + 10, SERVO_CURRENT_LIMIT);
      break;
    case 's':
      motors[1].goTo(data_joints[1][0] - 10, SERVO_CURRENT_LIMIT);
      break;
    case 'x':
      motors[2].goTo(data_joints[2][0] - 10, SERVO_CURRENT_LIMIT);
      break;
    // Relax finger
    case 'e':
      motors[0].turnOffTorque();
      break;
    case 'd':
      motors[1].turnOffTorque();
      break;
    case 'c':
      motors[2].turnOffTorque();
      break;
    // Open finger a little bit
    case 'r':
      motors[0].goTo(data_joints[0][0] - 10, SERVO_CURRENT_LIMIT);
      break;
    case 'f':
      motors[1].goTo(data_joints[1][0] + 10, SERVO_CURRENT_LIMIT);
      break;
    case 'v':
      motors[2].goTo(data_joints[2][0] + 10, SERVO_CURRENT_LIMIT);
      break;
    // Open finger all the way
    case 't':
      motors[0].goTo(SERVO_POSITIONS_OPEN[0], SERVO_CURRENT_LIMIT);
      break;
    case 'g':
      motors[1].goTo(SERVO_POSITIONS_OPEN[1], SERVO_CURRENT_LIMIT);
      break;
    case 'b':
      motors[2].goTo(SERVO_POSITIONS_OPEN[2], SERVO_CURRENT_LIMIT);
      break;
    // Equivalent to ROS callbacks
    case ',':
      closeHand();
      break;
    case '.':
      relaxHand();
      break;
    case '/':
      openHand();
      break;
  }
}
