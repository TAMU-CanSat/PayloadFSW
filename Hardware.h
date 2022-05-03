#pragma once
#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include "Common.h"
#include <Arduino.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <TeensyThreads.h>
#include <ArduinoQueue.h>

namespace Hardware
{
  extern Adafruit_BMP3XX bmp;
  extern Adafruit_BNO055 bno;

  extern ArduinoQueue<String> container_packets;
  extern Threads::Mutex mtx;

  void init();
  void read_sensors(Common::Sensor_Data &data);

  bool read_container_radio(String &data);
  
  void container_radio_loop();
}
#endif
