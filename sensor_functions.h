/* 
  this File contains the sensor functions 
*/

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>  //time of flight distance sensor
#include <MPU6050_tockn.h>  //gyro
#include "DFRobot_INA219.h"
// #include <Adafruit_INA219.h> //Current Sensor

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
MPU6050 mpu6050(Wire);
VL53L0X sensor;
// Adafruit_INA219 ina219(0x41); //assign unique address, A0(0x41) or A1(0x44) is bridged on the board!
DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS4);

// Revise the following two paramters according to actual reading of the INA219 and the multimeter
// for linearly calibration
float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;

float y_ang_offset = 0.0, z_ang_offset = 0.0, x_acc_offset = 0.0, y_acc_offset = 0.0, z_acc_offset = 0.0;  
int step_val = 0; 
bool interrupt = false; 

int get_dist(); 
float get_current(); 
float get_accel(int val); 

int get_dist(){ 

  int measured_dist; 
  int sample_size = 10;
  int sum = 0;  
  int mean_dist; 

  measured_dist = sensor.readRangeSingleMillimeters();

  if (measured_dist <= 85)
  {
    interrupt = true; 
    // Serial.print(measured_dist); 
    // Serial.println("End of track reached!"); 
  }

  if (measured_dist > 4000) //input max reading of sensor here 
  {
    interrupt = true; 
    // Serial.print(measured_dist); 
    // Serial.println("Distance sensor read max!"); 
    measured_dist = 9999;
    return measured_dist;
  }

  for (size_t i = 0; i < sample_size; i++)
  {
    sum += sensor.readRangeSingleMillimeters();
    delay(1);
  }

  mean_dist = sum / sample_size; 
  return mean_dist;
}

float get_current()
{
    float current;
    current = -(ina219.getCurrent_mA());
    return current;
}

float get_accel(int val){ 
  mpu6050.update();
  float measured_xaxisValue, measured_yaxisValue, measured_zaxisValue;
  float z_angle, y_angle; 

  // //Mean for more precision --> SLOWER !! 
  // float x[5] = {0.0}; //arrays to store acceleration in X, Y, Z 
  // float y[5] = {0.0};
  // float z[5] = {0.0};

  // for (size_t i = 0 ; i < 5; i++) {
  //   rslt = bmi160.getAccelGyroData(accelGyro); //SLOW !!!
  //       x[i] = accelGyro[3]/16384.0; //get linear acceleration in g-force 
  //       y[i] = accelGyro[4]/16384.0; 
  //       z[i] = accelGyro[5]/16384.0;   
  // }

  // measured_xaxisValue = (x[0] + x[1] + x[2] + x[3] + x[4]) / 5; //Mean to reduce influence of noise of the sensor
  // measured_yaxisValue = (y[0] + y[1] + y[2] + y[3] + y[4]) / 5;
  // measured_zaxisValue = (z[0] + z[1] + z[2] + z[3] + z[4]) / 5;

  //No mean 
  measured_xaxisValue = mpu6050.getAccX();
  measured_yaxisValue = mpu6050.getAccY();
  measured_zaxisValue = mpu6050.getAccZ();

  y_angle = - mpu6050.getAngleX(); 
  z_angle =  mpu6050.getAngleZ(); 
  
  if (val == 1)
  {
    return measured_xaxisValue; 
  }
    if (val == 2)
  {
    return measured_yaxisValue; 
  }
    if (val == 3)
  {
    return measured_zaxisValue; 
  }
  if (val == 4)
  { 
    y_angle = y_angle - y_ang_offset; 
    return y_angle; 
  }
  if (val == 5)
  {
    z_angle = z_angle - z_ang_offset; 
    return z_angle; 
  }
  
  return 0.0; 
}