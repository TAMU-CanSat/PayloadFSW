#include "Common.h"
#include "Hardware.h"
#include "States.h"
#include <TeensyThreads.h>

#include <EEPROM.h>

void setup() {
  Hardware::init();
  CONTAINER_XBEE_SERIAL.begin(115200); // must configure the xbee for this
  
  std::thread container(Hardware::container_radio_loop);

  //load recovery params
  EEPROM.get(Common::BA_ADDR, Hardware::EE_BASE_ALTITUDE);
  EEPROM.get(Common::ST_ADDR, States::EE_STATE);  
  
  container.detach();
}

void loop() {
  unsigned long start = millis();
  Hardware::mtx.lock();
  switch (States::EE_STATE)
  {
    case 0:
      States::Initialization();
      break;
    case 1:
      States::Standby();
      break;
    case 2:
      States::Deployment();
      break;
    default:
      States::Initialization();
      break;
  }
  Hardware::mtx.unlock();

  if (Common::TELEMETRY_DELAY > (millis() - start))
    threads.delay(Common::TELEMETRY_DELAY - (millis() - start));
}
