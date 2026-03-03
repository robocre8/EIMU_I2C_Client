#include "EIMU_I2C_Client.h"

inline void delayMs(int ms)
{
  for (int i = 0; i < ms; i += 1)
  {
    delayMicroseconds(1000);
  }
}

EIMU_I2C_Client::EIMU_I2C_Client(uint8_t slave_addr)
{
  slaveAddr = slave_addr;
}

bool EIMU_I2C_Client::begin(TwoWire &wire)
{
  _wire = &wire;
  delayMs(3000);

  constexpr int max_attempts = 10;

  for (int i = 0; i < max_attempts; ++i)
  {
      if (getWorldFrameId() < 3){
        return true;
      }
      delayMs(100);
  }

  _wire = nullptr;
  return false;
}

bool EIMU_I2C_Client::wireReady() const {
  return _wire != nullptr;
}

uint8_t EIMU_I2C_Client::computeChecksum(const uint8_t *packet, uint8_t length){
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += packet[i]; 
  }
  return sum & 0xFF; 
}

void EIMU_I2C_Client::send_packet_without_payload(uint8_t cmd)
{
  if (!wireReady()) return;
  // Build packet: start_byte + cmd + length + checksum
  uint8_t packet[4];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 0; // msg length = 0

  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 3);
  packet[3] = checksum;

  _wire->beginTransmission(slaveAddr);
  _wire->write(packet, sizeof(packet));
  _wire->endTransmission(true);
}

void EIMU_I2C_Client::write_data1(uint8_t cmd, float val, uint8_t pos)
{
  if (!wireReady()) return;
  // Build packet: start_byte + cmd + length + pos + float + checksum
  uint8_t packet[1 + 1 + 1 + 1 + 4 + 1];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 5; // msg is uint8 + float = 5byte length
  packet[3] = pos;
  memcpy(&packet[4], &val, sizeof(float));

  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 8);
  packet[8] = checksum;

  _wire->beginTransmission(slaveAddr);
  _wire->write(packet, sizeof(packet));
  _wire->endTransmission(true);
}

void EIMU_I2C_Client::write_data3(uint8_t cmd, float val0, float val1, float val3)
{
  if (!wireReady()) return;
  // Build packet: start_byte + cmd + length + float*3 + checksum
  uint8_t packet[1 + 1 + 1 + 12 + 1];
  packet[0] = START_BYTE;
  packet[1] = cmd;
  packet[2] = 12; // msg is 3 float = 12byte length
  memcpy(&packet[3], &val0, sizeof(float));
  memcpy(&packet[7], &val1, sizeof(float));
  memcpy(&packet[11], &val1, sizeof(float));

  // Compute checksum
  uint8_t checksum = computeChecksum(packet, 15);
  packet[15] = checksum;

  _wire->beginTransmission(slaveAddr);
  _wire->write(packet, sizeof(packet));
  _wire->endTransmission(true);
}

void EIMU_I2C_Client::read_data1(float& val0)
{
  if (!wireReady()) return;
  uint8_t buffer[4];
  uint8_t dataSizeInBytes = _wire->requestFrom(slaveAddr, (uint8_t)4);
  if (dataSizeInBytes != 4) {
    return;
  }
  for (uint8_t i = 0; i < dataSizeInBytes; i += 1)
  {
    buffer[i] = _wire->read();
  }
  memcpy(&val0, &buffer[0], sizeof(float));
}

void EIMU_I2C_Client::read_data3(float &val0, float &val1, float &val2)
{
  if (!wireReady()) return;
  uint8_t buffer[12];
  uint8_t dataSizeInBytes = _wire->requestFrom(slaveAddr, (uint8_t)12);
  if (dataSizeInBytes != 12) {
    return;
  }
  for (uint8_t i = 0; i < dataSizeInBytes; i += 1)
  {
    buffer[i] = _wire->read();
  }
  memcpy(&val0, &buffer[0], sizeof(float));
  memcpy(&val1, &buffer[4], sizeof(float));
  memcpy(&val2, &buffer[8], sizeof(float));
}

void EIMU_I2C_Client::read_data4(float &val0, float &val1, float &val2, float &val3)
{
  if (!wireReady()) return;
  uint8_t buffer[16];
  uint8_t dataSizeInBytes = _wire->requestFrom(slaveAddr, (uint8_t)16);
  if (dataSizeInBytes != 16) {
    return;
  }
  for (uint8_t i = 0; i < dataSizeInBytes; i += 1)
  {
    buffer[i] = _wire->read();
  }
  memcpy(&val0, &buffer[0], sizeof(float));
  memcpy(&val1, &buffer[4], sizeof(float));
  memcpy(&val2, &buffer[8], sizeof(float));
  memcpy(&val3, &buffer[12], sizeof(float));
}

void EIMU_I2C_Client::readQuat(float &w, float &x, float &y, float &z){
  send_packet_without_payload(READ_QUAT);
  read_data4(w, x, y, z);
}

void EIMU_I2C_Client::readRPY(float &r, float &p, float &y){
  send_packet_without_payload(READ_RPY);
  read_data3(r, p, y);
}

void EIMU_I2C_Client::readLinearAcc(float &x, float &y, float &z){
  send_packet_without_payload(READ_LIN_ACC);
  read_data3(x, y, z);
}

void EIMU_I2C_Client::readGyro(float &x, float &y, float &z){
  send_packet_without_payload(READ_GYRO);
  read_data3(x, y, z);
}

void EIMU_I2C_Client::readMag(float &x, float &y, float &z){
  send_packet_without_payload(READ_MAG);
  read_data3(x, y, z);
}

void EIMU_I2C_Client::readAcc(float &x, float &y, float &z){
  send_packet_without_payload(READ_ACC);
  read_data3(x, y, z);
}

void EIMU_I2C_Client::readRPYVariance(float &r, float &p, float &y){
  send_packet_without_payload(READ_RPY_VAR);
  read_data3(r, p, y);
}

void EIMU_I2C_Client::readAccVariance(float &x, float &y, float &z){
  send_packet_without_payload(READ_ACC_VAR);
  read_data3(x, y, z);
}

void EIMU_I2C_Client::readGyroVariance(float &x, float &y, float &z){
  send_packet_without_payload(READ_GYRO_VAR);
  read_data3(x, y, z);
}

void EIMU_I2C_Client::setWorldFrameId(int frame_id){
  write_data1(SET_FRAME_ID, (float)frame_id);
}

int EIMU_I2C_Client::getWorldFrameId(){
  float frame_id = 3.0;
  write_data1(GET_FRAME_ID);
  read_data1(frame_id);
  return (int)frame_id;
}

void EIMU_I2C_Client::setFilterGain(float gain){
  write_data1(SET_FILTER_GAIN, gain);
}

float EIMU_I2C_Client::getFilterGain(){
  float gain = 0.0;
  write_data1(GET_FILTER_GAIN);
  read_data1(gain);
  return gain;
}

bool EIMU_I2C_Client::clearDataBuffer(){
  float res = 0.0;
  write_data1(CLEAR_DATA_BUFFER);
  read_data1(res);
  return ((int)res == 1);
}