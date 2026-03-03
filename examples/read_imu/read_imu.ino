/*
 * Basic example code shows how to read orientation data from the MPU9250 EIMU Module
 * which have been succesfully calibrated with filter and covariances setup
 *
 * The code basically reads r, p, and y values from the MPU9250 EIMU Module connected to it.
 * read printed values from serial monitor or serial plotter.
 *
 * you can copy the code and use it in your project as you will.
 */

// Easy IMU i2c communication library
#include <EIMU_I2C_Client.h>

// please update with the address with that which you set when doing
// calibration and filter setup with the eimu_setup_application
uint8_t imuAddress = 104; // i.e 0x68 in HEX
EIMU_I2C_Client imu(imuAddress);

float toRad = 2 * PI / 360;
float toDeg = 1 / toRad;


long prevSampleTime;
long sampleTime = 15; // millisec

void setup()
{
  // setup serial communication to print result on serial minitor
  Serial.begin(115200);

  // start i2c communication
  Wire.begin();
  bool connect_success = imu.begin(); // wait for about 4 seconds
  if(!connect_success) {
    Serial.println("EIMU Connection was not successful. Check connection and Try Again");
    while(true);
  }

  // imu.clearDataBuffer();
  // check the refence frame the IMU is working in (0 - NWU,  1 - ENU,  2 - NED)
  int ref_frame_id = 1;
  imu.setWorldFrameId(ref_frame_id);
  ref_frame_id = imu.getWorldFrameId();
  if (ref_frame_id == 0)
    Serial.println("Reference Frame is North-West-Up (NWU)");
  else if (ref_frame_id == 1)
    Serial.println("Reference Frame is East-North-Up (ENU)");
  else if (ref_frame_id == 2)
    Serial.println("Reference Frame is North-East-Down (NED)");
  else
    Serial.println("ERROR: Reference Frame not Found");

  prevSampleTime = millis();
}

void loop()
{

  if ((millis() - prevSampleTime) >= sampleTime)
  {
    /* CODE SHOULD GO IN HERE*/
    float r, p, y, ax, ay, az, gx, gy, gz;

    imu.readRPY(r, p, y);
    imu.readLinearAcc(ax, ay, az);
    imu.readGyro(gx, gy, gz);

    // Serial.print(r, 4);
    // Serial.print(", ");
    // Serial.print(p, 4);
    // Serial.print(", ");
    // Serial.println(y, 4);

    Serial.print(r * toDeg, 1);
    Serial.print(", ");
    Serial.print(p * toDeg, 1);
    Serial.print(", ");
    Serial.println(y * toDeg, 1);

    Serial.print(ax, 4);
    Serial.print(", ");
    Serial.print(ay, 4);
    Serial.print(", ");
    Serial.println(az, 4);

    Serial.print(gx, 4);
    Serial.print(", ");
    Serial.print(gy, 4);
    Serial.print(", ");
    Serial.println(gz, 4);

    Serial.println();

    prevSampleTime = millis();
  }
}