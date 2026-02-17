// #ifndef EIMU_I2C_H
// #define EIMU_I2C_H

#pragma once

#include <Arduino.h>
#include <Wire.h>

class EIMU_I2C_Client
{
public:
  float dataBuffer[4];
  explicit EIMU_I2C_Client(uint8_t address);
  bool begin(TwoWire &wire=Wire);

  void readQuat();
  void readRPY();
  void readLinearAcc();
  void readGyro();

  // void readAcc();
  // void readMag();

  void readRPYVariance();
  void readAccVariance();
  void readGyroVariance();
  
  void setWorldFrameId(int);
  int getWorldFrameId();
  void setFilterGain(float);
  float getFilterGain();
  bool clearDataBuffer();


private:
  TwoWire *_wire = nullptr;
  uint8_t slaveAddr;
  bool wireReady() const;
  uint8_t computeChecksum(const uint8_t *packet, uint8_t length);
  void send_packet_without_payload(uint8_t cmd);
  void write_data1(uint8_t cmd, float val=0.0, uint8_t pos=100);
  void write_data3(uint8_t cmd, float val0, float val1, float val2);
  void read_data1(float &val0);
  void read_data3(float &val0, float &val1, float &val2);
  void read_data4(float &val0, float &val1, float &val2, float &val3);
  void clearBuffer();

  // Serial Protocol Command IDs -------------
  static constexpr uint8_t START_BYTE = 0xBB;
  static constexpr uint8_t READ_QUAT = 0x01;
  static constexpr uint8_t READ_RPY = 0x02;
  static constexpr uint8_t READ_RPY_VAR = 0x03;
  static constexpr uint8_t READ_ACC = 0x05;
  static constexpr uint8_t READ_ACC_VAR = 0x09;
  static constexpr uint8_t READ_GYRO = 0x0B;
  static constexpr uint8_t READ_GYRO_VAR = 0x0F;
  static constexpr uint8_t READ_MAG = 0x11;
  static constexpr uint8_t SET_FILTER_GAIN = 0x1D;
  static constexpr uint8_t GET_FILTER_GAIN = 0x1E;
  static constexpr uint8_t SET_FRAME_ID = 0x1F;
  static constexpr uint8_t GET_FRAME_ID = 0x20;
  static constexpr uint8_t READ_QUAT_RPY = 0x22;
  static constexpr uint8_t READ_ACC_GYRO = 0x23;
  static constexpr uint8_t CLEAR_DATA_BUFFER = 0x27;
  static constexpr uint8_t READ_LIN_ACC_RAW = 0x2B;
  static constexpr uint8_t READ_LIN_ACC = 0x2C;
  //---------------------------------------------
};

// #endif