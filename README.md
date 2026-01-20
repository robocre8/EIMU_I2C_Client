## Easy IMU (EIMU) Arduino I2C Client Library - EIMU_I2C_Client
This library helps communicate with the already setup **`Easy IMU`** with  arduino microcontroller projects via I2C

> you can use it in your Arduino-based robotics project (e.g Arduino UNO, Arduino NANO, Arduino MEGA, Esp32, etc.)

A simple way to get started is simply to try out and follow the example code


## How to Use the Library
- Ensure you have the **`Easy IMU`**

- Download download the library by clicking on the green Code button above (or clone it)
  > if you download it, extract it and change the folder name to `EIMU_I2C_Client`

- Move the downloaded library file to your Arduino library folder
  > e.g on linux: ... home/Arduino/libraries/
  >
  > e.g on windows: ... Documents/Arduino/libraries/
  >
  > (or any where your arduino libraries are stored)

- restart your ArduinoIDE and navigate to examples and run the example code

- you can copy this example code into your project and modify it to your taste.


## Basic Library functions and usage

- connect to epmc_driver shield module
  > EIMU_I2C_Client imu(imuAddress)
  > imu.begin()

  > imu.readQuat(float &w, float &x, float &y, float &z);
  > imu.readRPY(float &x, float &y, float &z);
  > imu.readAccGyro(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

  > imu.readLinearAcc(float &x, float &y, float &z);
  > imu.readGyro(float &x, float &y, float &z);
  > imu.readMag(float &x, float &y, float &z);

  > imu.readRPYVariance(float &x, float &y, float &z);
  > imu.readAccVariance(float &x, float &y, float &z);
  > imu.readGyroVariance(float &x, float &y, float &z);
  
  > imu.setWorldFrameId(int);
  > imu.getWorldFrameId();
  > imu.getFilterGain();
  > imu.clearDataBuffer();
