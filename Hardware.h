#pragma once
#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include "Common.h"
#include <Arduino.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_GPS.h>
#include <TeensyThreads.h>
#include <ArduinoQueue.h>

namespace Hardware
{
  extern Adafruit_BMP3XX bmp;
  extern Adafruit_MPU6050 mpu;
  extern Adafruit_GPS GPS;

  extern ArduinoQueue<Downlink> container_packets;
  extern Threads::Mutex mtx;

  bool ready();
  void init();
  void read_gps(GPS_Data &data);
  void read_sensors(Sensor_Data &data);

  void write_container_radio(Downlink data);
  bool read_container_radio(String &data);
  
  void container_radio_loop();
}
#endif
