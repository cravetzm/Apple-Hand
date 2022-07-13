# AppleHand

This repository contains the Arduino and ROS-related code, as well as relevant schematics and models, for the AppleHand project from the Intelligent Machines and Materials Lab at Oregon State University. 

The AppleHand currently relies on an OpenCM 9.04 microcontroller connected to several I2C muxes for sensors and 3 XM430-W350-T servos. It is meant to be mounted on a UR5e robot arm.

## Setup

You should use a computer running Ubuntu 16.04, 18.04, or 20.04 (tested only on 18.04 but should work on the others) for this repository.

### OpenCM 9.04

Follow the most up-to-date instructions on installing the OpenCM 9.04 device rules, Arduino, and the OpenCM 9.04 Arduino board definition here: [https://emanual.robotis.com/docs/en/parts/controller/opencm904/#install-on-linux](https://emanual.robotis.com/docs/en/parts/controller/opencm904/#install-on-linux)

### Robot Operating System (ROS)

Install the appropriate ROS distribution for your operating system version here: [http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation)

Next, install the `ros-<distro>-rosserial-arduino` package. 

### Installing This Repository

Now, in your ROS workspace packages folder (usually `~/catkin_ws/src`), download the repository via:
```
git clone https://github.com/cravetzm/Apple-Hand.git
```
Then, return to your ROS workspace folder (usually `~/catkin_ws/`) and run `catkin_make`.

To make Arduino recognize our custom messages (for future changes), copy the appropriate message header files from the repository to your local Arduino `ros_lib` like so:
```
cp -r ~/catkin_ws/src/applehand/arduino/ros_lib/applehand/ ~/.arduino15/packages/OpenCM904/hardware/OpenCM904/1.5.0/libraries/ros_lib/applehand
```

Lastly, we are transferring a lot of data - we can increase the buffer size of `rosserial` by modifying `~/.arduino15/packages/OpenCM904/hardware/OpenCM904/1.5.0/libraries/ros_lib/ros.h`. In line 53 of that file, we want to change:
```
  typedef NodeHandle_<ArduinoHardware, 25, 25, 256, 256> NodeHandle;
```
to
```
  typedef NodeHandle_<ArduinoHardware, 25, 25, 1024, 1024> NodeHandle;
```
1024 bytes is somewhat arbitrary, but it works (tested with IMU and joint state data). 

## How to Use

### Communication and Sensing Options

At the top of [the main .ino file](https://github.com/cravetzm/applehand/blob/main/arduino/applehand-opencm/applehand-opencm.ino), there are four booleans that control sensing and communication:
```
const bool USE_ROSSERIAL = true;
const bool READ_IMUS = true;
const bool READ_PRESSURES = true;
const bool READ_JOINT_STATES = true;
```
These are fairly self-explanatory. If `USE_ROSSERIAL = false`, messages will be displayed as text in normal Serial communication and basic commands can be sent as letters (see the bottom of [the main .ino file](https://github.com/cravetzm/applehand/blob/main/arduino/applehand-opencm/applehand-opencm.ino) for details).

If any of the `READ_<data> = false`, their respective sensors will not be read from and no messages related to that sensor will be sent. This will impact the overall sample rate of the system.

Be sure to re-upload the .ino file after any changes!

### Communication with ROS

Core communication with ROS only requires the following commands, run in different terminals, when directly connected to the OpenCM 9.04 via USB:

```
roscore
```
```
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=2000000
```

The baud rate of the USBSerial port on the OpenCM 9.04 is adaptive, so the used baud rate will match the ```_baud:=BAUD_RATE``` setting with some exceptions (250000 didn't work for some reason). 

When the previous two commands are operating, you can manually trigger the hand to close, open, or relax via the following commands:
```
rosservice call /applehand/close_hand
```
```
rosservice call /applehand/open_hand
```
```
rosservice call /applehand/relax_hand
```

### Recording Data via ROS

You can easily record data via the `rosbag` command ([more info on ROS wiki](http://wiki.ros.org/rosbag/Commandline)) like so:
```
rosbag record -a
```
Then, press `CTRL + C` to end the recording. This will create a .bag file in your current directory, named based on the current time. Using this file, we can then replay and port that data to a text file with this command:
```
rostopic echo -b <bag_file_name>.bag -p /<topic_name> > <output_file_name>.txt
```
For instance, to record the IMU data on finger 1 you can run:
```
rostopic echo -b 2021-03-02-23-29-13.bag -p /applehand/finger1/imu > imu1.txt
```

## Known Issues and Workarounds

### .ino file won't upload to the OpenCM 9.04

If uploading hangs on 
```
Sketch uses 75228 bytes (64%) of program storage space. Maximum is 116736 bytes.
Global variables use 13740 bytes of dynamic memory.
Enter bootloader
```
for more than 30 seconds, simply press the upload button again. This will seem to not work and say:
```
Sketch uses 75228 bytes (64%) of program storage space. Maximum is 116736 bytes.
Global variables use 13740 bytes of dynamic memory.
An error occurred while uploading the sketch
```
However, if you wait for a few more seconds, it will work regardless. You will know that it works when it says:
```
Enter bootloader
stm32ld ver 1.0.1
OpenCM Download Ver 1.0.4 2015.06.16 
Board Name : CM-904
Ready To download 
Flash : 10% 20% 30% 40% 50% 60% 70% 80% 90% 100% 
Write Size : 75520
CheckSum : Success..
Go Application
```

### /tty/ACM0 is permanently busy

After uploading, the port may take some time to no longer be busy. However, if it remains busy for more than a minute you can unplug both the USB and power cables, then plug the USB back in first and the power second. 

### Arduino IDE cannot upload to the OpenCM 9.04

If many upload attempts and disconnect-reconnect attempts fail, press the small black `USER SW` button on the OpenCM 9.04 as you power it on. A green light should turn on alongside the usual red light, indicating that it is in recovery mode. You should be able to upload to the board now.

### Using the service throws an error

You may see the following error when using the `rosservice` command:
```
ERROR: service [/applehand/home_hand] responded with an error: service cannot process request: service handler returned None
```
This is an issue with the `rosserial` library. The issue can be checked on in [this thread](https://github.com/ros-drivers/rosserial/issues/408). Unless the AppleHand also does not respond to the command, it is working as well as it can.
