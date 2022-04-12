#include "Common.h"
#include "Hardware.h"
#include "States.h"

void setup() {  
  CONTAINER_XBEE_SERIAL.begin(9600);
  
  std::thread container(Hardware::container_radio_loop);
  
  container.detach();
}

void loop() {
  switch (States::state)
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
}
