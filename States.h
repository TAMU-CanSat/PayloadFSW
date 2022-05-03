#pragma once
#ifndef __STATES_H__
#define __STATES_H__

#include <Arduino.h>
namespace States
{
  static int EE_STATE = 0;
  
  void Initialization();
  void Standby();
  void Deployment();
}
#endif
