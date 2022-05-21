#pragma once
#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include "Common.h"
#include <Arduino.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <TeensyThreads.h>
#include <ArduinoQueue.h>
#include <Servo.h>

namespace Hardware
{
  extern bool SIM_ACTIVATE;
  extern bool SIM_ENABLE;
  extern int SIM_PRESSURE;
  extern float EE_BASE_ALTITUDE;

  extern Adafruit_BMP3XX bmp;
  extern Adafruit_BNO055 bno;
  extern Servo tether_servo;
  extern Servo pin1_servo;
  extern Servo pin2_servo;

  extern ArduinoQueue<String> container_packets;
  extern Threads::Mutex mtx;

  extern elapsedMillis cameraHold;
  extern bool cameraRecording;
  extern bool firstCameraCall;

  void init();

  void update_camera(bool record);
  void start_recording();
  void stop_recording();

  void ascend_tether();
  void descend_tether();
  void release_pins();
  
  void spin_motor(uint16_t duty_cycle, bool ccw = false);
  
  void read_sensors(Common::Sensor_Data &data);

  bool read_container_radio(String &data);
  
  void container_radio_loop();

  static void build_packet(String& packet, const String& state, const float error, const Common::Sensor_Data &sensors)
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
    packet += state;
    packet += "&";
  }
}
#endif
