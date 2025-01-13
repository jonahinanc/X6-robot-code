/* Main Arduino Sketch 
*/

#include <Arduino.h>
#include <Wire.h>
#include <sensor_functions.h>
#include <gaits.h> 
#include <webinterface.h>

void setup() {
  Serial.begin(115200);
  while (!Serial)
  delay(1000);
  Serial.println("Serial is up.");
  Wire.begin(); 

//Wifi 
/* Go to http://192.168.4.1 in a web browser
   connected to the ESP AP to see the webinterface 
*/
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  //callback functions for http requests  
  server.on("/", handle_root); 
  server.on("/X5_mini.csv", download_data); 
  server.begin();

  //Initialize/Settings I2C Devices
  pwm.begin();
  pwm.setPWMFreq(60);

  // Setup for ESP32 servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    hsp.setPeriodHertz(60);
    fsp.setPeriodHertz(60);
// change to B130 or NEEBRC min and max
    hsp.attach(h_s, b130_servo_min, b130_servo_max);
    fsp.attach(f_s, b130_servo_min, b130_servo_max);
    delay(10);

  // Current sensor
  // Current sensor
  Serial.begin(115200);
  while (!Serial)
      ;
  Serial.println();
  while (ina219.begin() != true)
  {
      Serial.println("INA219 begin failed");
      delay(2000);
  }
  ina219.linearCalibrate(ina219Reading_mA, extMeterReading_mA);
  Serial.println();
  //   ina219.begin(); //regular INA Sensor (limit 3.5A)

  // distance Sensor
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize tof sensor!");
    while (1) {}
  }
  //Gyro 
  mpu6050.begin(); 
  mpu6050.calcGyroOffsets(true); 
  Serial.println(); 

  home_pos(); //All Servos to home position 
}


void loop() {

  server.handleClient(); 
}
