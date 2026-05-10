/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-30 10:34:30
 * @LastEditors: Changhua
 * @Description: MPU6050 Data solution
 * @FilePath: 
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "MPU6050_getdata.h"
#include <stdio.h>
#include <math.h>

MPU6050 accelgyro;
MPU6050_getdata MPU6050Getdata;

// static void MsTimer2_MPU6050getdata(void)
// {
//   sei();
//   int16_t ax, ay, az, gx, gy, gz;
//   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //Read the raw values of the six axes
//   float gyroz = -(gz - MPU6050Getdata.gzo) / 131 * 0.005f;
//   MPU6050Getdata.yaw += gyroz;
// }

bool MPU6050_getdata::MPU6050_dveInit(void)
{
  Wire.begin();
  uint8_t chip_id = 0x00;
  uint8_t cout;
  do
  {
    chip_id = accelgyro.getDeviceID();
    Serial.print("MPU6050_chip_id: ");
    Serial.println(chip_id);
    delay(10);
    cout += 1;
    if (cout > 10)
    {
      return true;
    }
  } while (chip_id == 0X00 || chip_id == 0XFF); //Ensure that the slave device is online（Wait forcibly to get the ID）
  accelgyro.initialize();
  // unsigned short times = 100; //Sampling times
  // for (int i = 0; i < times; i++)
  // {
  //   gz = accelgyro.getRotationZ();
  //   gzo += gz;
  // }
  // gzo /= times; //Calculate gyroscope offset
  return false;
}
bool MPU6050_getdata::MPU6050_calibration(void)
{
  // Let the MPU6050 settle after initialize() before sampling.
  // Without this delay, the first ~100ms of readings are unreliable
  // and bias the calibration, causing slow yaw drift in straight-line mode.
  delay(500);

  const unsigned short times = 500; // 5x more samples for a stable bias estimate
  long sum = 0;
  for (unsigned short i = 0; i < times; i++)
  {
    sum += accelgyro.getRotationZ();
    delay(2); // ~1 second total — sample over time, not back-to-back
  }
  gzo = sum / times;

  Serial.print(F("[MPU] gyro-Z bias: "));
  Serial.print(gzo);
  Serial.println(F(" LSB"));
  return false;
}
bool MPU6050_getdata::MPU6050_dveGetEulerAngles(float *Yaw)
{
  unsigned long now = millis();           //Record the current time(ms)
  dt = (now - lastTime) / 1000.0;         //Caculate the derivative time(s)
  lastTime = now;                         //Record the last sampling time(ms)
  gz = accelgyro.getRotationZ();          //Read the raw values of the six axes
  // Compute rotation RATE first (deg/s), apply dead-band on the rate, then
  // integrate. The original code dead-banded the per-tick delta (rate*dt),
  // which silently scaled the threshold with update frequency: at 100Hz
  // (dt=0.01s), 0.05° per tick = 5°/s ignored — enough slow drift to make
  // the car curve away without the controller ever noticing.
  float rate = -(gz - gzo) / 131.0; //deg/s
  if (fabs(rate) < 0.5)             //~0.5 deg/s noise floor
  {
    rate = 0.0;
  }
  agz += rate * dt; //integrate to angle
  *Yaw = agz;
  return false;
}

// Returns true when the car is tilted enough that the gyro-Z "yaw" reading
// becomes unreliable (gimbal-lock-like coupling from pitch/roll into yaw).
// At rest on level ground, |ax| and |ay| are near 0 and az ≈ 16384 (1g).
// 4500 LSB ≈ atan(4500/16384) ≈ 15° tilt.
bool MPU6050_getdata::MPU6050_dveIsTilted(void)
{
  int16_t ax, ay, az;
  accelgyro.getAcceleration(&ax, &ay, &az);
  return (abs(ax) > 4500 || abs(ay) > 4500);
}
