#include "Common.h"
#include "Hardware.h"
#include "States.h"

namespace States
{  
  void Initialization()
  { 
    unsigned long start = millis();
  
    Common::Sensor_Data sensor_data;
    Hardware::read_sensors(sensor_data);

    String packet;
    Common::build_packet(packet, "INITIALIZATION", 0, sensor_data);
  
    Hardware::mtx.lock();
    Hardware::container_packets.enqueue(packet);
    Hardware::mtx.unlock();
  
    if (Common::TELEMETRY_DELAY > (millis() - start))
      delay(Common::TELEMETRY_DELAY - (millis() - start));
    }
}
