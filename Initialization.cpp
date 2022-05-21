#include "Common.h"
#include "Hardware.h"
#include "States.h"
#include <TimeLib.h>

namespace States
{  
  uint16_t EE_STATE = 0;
  
  void Initialization()
  { 
    Common::Sensor_Data sensor_data;
    Hardware::read_sensors(sensor_data);

    //Hardware::spin_motor(256, true);

    if (millis() < 5000)
    {
        Hardware::update_camera(true);
    } else if (millis() > 10000)
    {
        Hardware::update_camera(false);
    }
    
    String packet;
    Hardware::build_packet(packet, "INITIALIZATION", 0, sensor_data);
    Hardware::container_packets.enqueue(packet);
    //Serial.println(packet);
   }
}
