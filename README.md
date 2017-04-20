# tinyIMU_arduino

an arduino sketch to read raw values from MPU6050 Acclerometer and publish the data on tinyIMU messages to ROS

Depends on Jeff Rowberg's I2Cdevlib library found at https://github.com/jrowberg/i2cdevlib.  Be sure to install the MPU6050 library and I2Cdev Libraries from the Arduino folder to the folders on your IDE.

Work will eventually be done to convert this into a catkinized package to run with CMake. Initial attempts to do this were frustrating, so the current workaround is to flash this to an arduino independent of ROS, install the rosserial library found at https://github.com/ros-drivers/rosserial into the workspace and build the ROS libraries into the arduino sketchbook folder using the method described in the [rosserial tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup#Install_ros_lib_into_the_Arduino_Environment.)

## Step by Step
1. Create a folder and clone tinyIMU_arduino and tiny_msgs
```
mkdir tinyPkgs
cd tinyPkgs
git clone https://github.com/superjax/tinyIMU_arduino.git
git clone https://github.com/superjax/tiny_msgs.git
```

2. Compile tiny_msgs
```
cmake tiny_msgs/
```

3. source tiny_msgs
```
source tiny_msgs/devel/setup.bash 
```
You will need to run this command on every new shell you open to have access to the tiny_msgs, unless you add this line to your .bashrc (with the full folder path). For this tutorial it's necessary to have tiny_msgs available in every new shell.

4. rebuild Arduino rosserial library
The tiny_msgs have to be included in the Arduino library. The location of the library folder might be different in your installation.
```
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

5. connect the IMU
Connect the MPU6050 to the Arduino like shown in this [tutorial](https://create.arduino.cc/projecthub/Aritro/getting-started-with-imu-6-dof-motion-sensor-96e066#about-project) and test if you can run the example code to gather the raw data without ROS.

6. load the tinyIMU arduino sketch
Upload the tinyIMU.ino sketch to your Arduino using the Arduino software.

7. start roscore
```roscore```

8. start rosserial and test
start rosserial in a new shell. Don't forget step 3 if you have not sourced setup.bash
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
ttyUSB0 might also be another port like ttyUSB1 or ttyACM0

Examine the available topics
```
rostopic list
```
A new topic /tinyImu should be now available

```
rostopic echo /tinyImu
```

This should print a stream of IMU readings in the following style:

```
...
---
header: 
  seq: 8
  stamp: 
    secs: 1492693910
    nsecs: 348656101
  frame_id: �
accel: 
  x: 1392
  y: -2854
  z: -17169
gyro: 
  x: -123
  y: 255
  z: -75
---
...
```

9. Use IMU readings
You can now subscribe to the topic in your own ROS package and use it for your purposes. 
