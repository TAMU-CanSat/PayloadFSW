#include "Common.h"
#include "Hardware.h"

#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <TeensyThreads.h>
#include <ArduinoQueue.h>

namespace Hardware
{   
  Adafruit_BMP3XX bmp;
  Adafruit_BNO055 bno;

  ArduinoQueue<String> container_packets(20);
  Threads::Mutex mtx;
  
  void init()
  {
    Wire.setSCL(Common::I2C_SCL);
    Wire.setSDA(Common::I2C_SDA);
    Wire.begin();
    bmp.begin_I2C();
    bno.begin();
  }

  bool read_container_radio(String &data)
  {
    if (CONTAINER_XBEE_SERIAL.available())
    {
      data = CONTAINER_XBEE_SERIAL.readStringUntil('\n');
      return true;
    } else
      return false;
  }

  void read_sensors(Common::Sensor_Data &data)
  {
    data.vbat = map(analogRead(Common::VOLTAGE_PIN), 0, 1023, 0, 3.7);
    data.altitude = bmp.readAltitude(Common::SEA_LEVEL);
    data.temperature = bmp.readTemperature();

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    data.gyro[0] = gyro.x();
    data.gyro[1] = gyro.y();
    data.gyro[2] = gyro.z();

    data.acceleration[0] = acceleration.x();
    data.acceleration[1] = acceleration.y();
    data.acceleration[2] = acceleration.z();

    data.magnetometer[0] = magnetometer.x();
    data.magnetometer[1] = magnetometer.y();
    data.magnetometer[2] = magnetometer.z();
  }

  void container_radio_loop()
  {
    while(1)
    {
      mtx.lock();
      while (!container_packets.isEmpty())
      {
        CONTAINER_XBEE_SERIAL.println(container_packets.dequeue());
      }
      mtx.unlock();
  
      String received;
      if (read_container_radio(received))
      {
        for(unsigned int i = 0; i < received.length(); i++)
        {
          // do something with received[i]
        }
      }
      delay(10);
    }
  }
}
