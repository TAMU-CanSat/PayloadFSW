#pragma once
#ifndef __STATES_H__
#define __STATES_H__

#include <Arduino.h>
namespace States
{
  extern uint16_t EE_STATE;
  
  void Initialization();
  void Standby();
  void Deployment();
}
#endif
