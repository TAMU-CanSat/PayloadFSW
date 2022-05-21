#pragma once
#ifndef __COMMON_H__
#define __COMMON_H__

#include <Arduino.h>
#include <TeensyThreads.h>
#include <ArduinoQueue.h>

#define CONTAINER_XBEE_SERIAL Serial3

namespace Common {
  const unsigned long TELEMETRY_DELAY = 250; //4hz 
  
  const byte VOLTAGE_PIN = 16;
  const byte CAMERA_PIN = 22;
  const byte TETHER_SERVO_PIN = 9;
  const byte PIN1_SERVO_PIN = 8;
  const byte PIN2_SERVO_PIN = 7;
  const byte MOTOR_PWM_PIN = 12;
  const byte MOTOR_IN1_PIN = 10;
  const byte MOTOR_IN2_PIN = 11;
  
  const uint16_t BA_ADDR = 0;
  const uint16_t ST_ADDR = 4;
  const float SEA_LEVEL = 1014.6f; //update this before launch
  
  struct Sensor_Data // 36 bytes
  {
    float vbat; // 4 bytes
    float altitude; // 4 bytes
    float temperature; // 4 bytes
    float gyro[3]; // 12 bytes
    float acceleration[3]; // 12 bytes
    float magnetometer[3];
  };
}
#endif
