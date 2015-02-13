# tinyIMU_arduino

an arduino sketch to read raw values from MPU6050 Acclerometer and publish the data on tinyIMU messages to ROS

Depends on Jeff Rowberg's I2Cdevlib library found at https://github.com/jrowberg/i2cdevlib.  Be sure to install the MPU6050 library and I2Cdev Libraries from the Arduino folder to the folders on your IDE.

Work will eventually be done to convert this into a catkinized package to run with CMake.  Initial attempts to do this were frustrating, so the current workaround is to flash this to an arduino independant of ros, install the rosserial library found at https://github.com/ros-drivers/rosserial into the workspace and build the ros libraries into the arduino sketchbook folder using the method described in the rosserial tutorials.
