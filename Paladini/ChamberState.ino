#include "ChamberState.h"


ChamberState& ChamberState::operator=(const ChamberState& other) {

  // Guard self assignment
  if (this == &other) {
      return *this;
  }

  this->temperature = other.temperature;
  this->humidity = other.humidity;
  this->switchLight = other.switchLight;
  this->timeLastChangeSwitchLight = other.timeLastChangeSwitchLight;
  this->switchForceHeater = other.switchForceHeater;
  this->switchForceHumidity = other.switchForceHumidity;
  this->fanSpeed = other.fanSpeed;
  this->mode = other.mode;

  return *this;

}
