#include <Arduino.h>

#include <Wire.h>
#include <ESP32Servo.h>

int pulsewidth = 2000;

Servo b130;

void setup() {
    Serial.begin(115200);
    delay(10);

    Wire.begin();

    // Setup for ESP32 servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    b130.setPeriodHertz(60);
    b130.attach(27);        
}

void loop() {

    b130.writeMicroseconds(pulsewidth);
    delay(1000);
}