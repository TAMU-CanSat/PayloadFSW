#include "Common.h"
#include "Hardware.h"

#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <TeensyThreads.h>
#include <ArduinoQueue.h>
#include <EEPROM.h>
#include <Servo.h>

namespace Hardware
{ 
  bool SIM_ACTIVATE = false;
  bool SIM_ENABLE = false;
  int SIM_PRESSURE = 0;
  float EE_BASE_ALTITUDE = 0;
  elapsedMillis cameraHold = 0;
  bool cameraRecording = false;
  bool firstCameraCall = true;
    
  Adafruit_BMP3XX bmp;
  Adafruit_BNO055 bno;
  Servo tether_servo;
  Servo pin1_servo;
  Servo pin2_servo;

  ArduinoQueue<String> container_packets(20);
  Threads::Mutex mtx;
  
  void init()
  {
    pinMode(Common::MOTOR_PWM_PIN, OUTPUT);
    pinMode(Common::MOTOR_IN1_PIN, OUTPUT);
    pinMode(Common::MOTOR_IN2_PIN, OUTPUT);
    pinMode(Common::CAMERA_PIN, OUTPUT);
    digitalWrite(Common::CAMERA_PIN, HIGH);
    cameraHold = 0;
    cameraRecording = false;
    firstCameraCall = true;

    //tether_servo.attach(Common::TETHER_SERVO_PIN);
    //pin1_servo.attach(Common::PIN1_SERVO_PIN);
    //pin2_servo.attach(Common::PIN2_SERVO_PIN);

    //tether_servo.write(90);
    //pin1_servo.write(90);
    //pin2_servo.write(90);
    
    Wire.begin();
    bmp.begin_I2C(0x77, &Wire);
    
    bno.begin();
  }

  void update_camera(bool record)
  {
    if (record && !cameraRecording)
    {
      if (firstCameraCall)
      { 
        cameraHold = 0;
        firstCameraCall = false;
      }
      
      start_recording();
    } else if (!record && cameraRecording)
    {
      if (firstCameraCall)
      {
        cameraHold = 0;
        firstCameraCall = false;
      }
      
      stop_recording();
    }
  }

  void start_recording()
  {
    if (cameraHold < 550)
    {
      pinMode(Common::CAMERA_PIN, OUTPUT);
      digitalWrite(Common::CAMERA_PIN, LOW);
    } else
    {
      pinMode(Common::CAMERA_PIN, OUTPUT);
      digitalWrite(Common::CAMERA_PIN, HIGH);
      cameraRecording = true;
      firstCameraCall = true;
    }
  }

  void stop_recording()
  {
    if (cameraHold < 150)
    {
      pinMode(Common::CAMERA_PIN, OUTPUT);
      digitalWrite(Common::CAMERA_PIN, LOW);
    } else
    {
      pinMode(Common::CAMERA_PIN, OUTPUT);
      digitalWrite(Common::CAMERA_PIN, HIGH);
      cameraRecording = false;
      firstCameraCall = true;
    }
  }

  void spin_motor(uint16_t duty_cycle, bool ccw = false)
  {
    if (!ccw)
    {
      digitalWrite(Common::MOTOR_IN1_PIN, HIGH);
      digitalWrite(Common::MOTOR_IN2_PIN, LOW);
      
      //analogWriteFrequency(Common::MOTOR_PWM_PIN, 8000);
      analogWrite(Common::MOTOR_PWM_PIN, duty_cycle);
    } else {
      digitalWrite(Common::MOTOR_IN1_PIN, LOW);
      digitalWrite(Common::MOTOR_IN2_PIN, HIGH);
      
      //analogWriteFrequency(Common::MOTOR_PWM_PIN, 8000);
      analogWrite(Common::MOTOR_PWM_PIN, duty_cycle);
    }
  }

  bool read_container_radio(String &data)
  {
    if (CONTAINER_XBEE_SERIAL.available())
    {
      data = CONTAINER_XBEE_SERIAL.readStringUntil('\r\n');
      return true;
    } else
      return false;
  }

  void read_sensors(Common::Sensor_Data &data)
  {
    data.vbat = ((analogRead(Common::VOLTAGE_PIN) / 1023.0) * 4.2) + 0.35;
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
        CONTAINER_XBEE_SERIAL.print(container_packets.dequeue());
      }
      
      String received;
      if (read_container_radio(received))
      {
        //reset recovery params
        //EEPROM.put(Common::BA_ADDR, 0.0f);
        //EEPROM.put(Common::ST_ADDR, 0);
      }
      mtx.unlock();
      threads.delay(250);
    }
  }
}
