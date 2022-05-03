#pragma once
#ifndef __COMMON_H__
#define __COMMON_H__

#include <Arduino.h>
#include <TeensyThreads.h>
#include <ArduinoQueue.h>

#define CONTAINER_XBEE_SERIAL Serial1

namespace Common {
  const unsigned long TELEMETRY_DELAY = 250; //4hz
  
  const byte VOLTAGE_PIN = 23;
  const byte I2C_SCL = 24;
  const byte I2C_SDA = 25;
  const byte CAMERA_PIN = 3;

  const float SEA_LEVEL = 1014.6f;

  static bool SIM_ACTIVATE = false;
  static bool SIM_ENABLE = false;
  static int SIM_PRESSURE = 0;

  static uint16_t BA_ADDR = 0;
  static uint16_t ST_ADDR = 4;

  static float EE_BASE_ALTITUDE = 0;
  
  struct Sensor_Data // 36 bytes
  {
    float vbat; // 4 bytes
    float altitude; // 4 bytes
    float temperature; // 4 bytes
    float gyro[3]; // 12 bytes
    float acceleration[3]; // 12 bytes
    float magnetometer[3];
  };

  static void build_packet(String& packet, const String& state, const float error, const Sensor_Data &sensors)
  {
    packet = String(sensors.altitude) + ","; 
    packet += String(sensors.temperature) + ",";
    packet += String(sensors.vbat) + ",";
    packet += String(sensors.gyro[0]) + ",";
    packet += String(sensors.gyro[1]) + ",";
    packet += String(sensors.gyro[2]) + ",";
    packet += String(sensors.acceleration[0]) + ",";
    packet += String(sensors.acceleration[1]) + ",";
    packet += String(sensors.acceleration[2]) + ",";
    packet += String(sensors.magnetometer[0]) + ",";
    packet += String(sensors.magnetometer[1]) + ",";
    packet += String(sensors.magnetometer[2]) + ",";
    packet += String(error) + ",";
    packet += state + "\n";
  }
}
#endif
