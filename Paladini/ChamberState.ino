#include "ChamberState.h"


ChamberState& ChamberState::operator=(const ChamberState& other) {

  // Guard self assignment
  if (this == &other) {
      return *this;
  }

  this->temperature = other.temperature;
  this->heaterState = other.heaterState;
  this->timeLastChangeHeater = other.timeLastChangeHeater;
  this->humidity = other.humidity;
  this->humidifierState = other.humidifierState;
  this->timeLastChangeHumidifier = other.timeLastChangeHumidifier;
  this->switchLight = other.switchLight;
  this->timeLastChangeSwitchLight = other.timeLastChangeSwitchLight;
  this->fanPowerLevel = other.fanPowerLevel;
  this->mode = other.mode;

  return *this;

}
