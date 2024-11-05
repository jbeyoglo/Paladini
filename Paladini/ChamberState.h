#ifndef ChamberState_H
#define ChamberState_H


// Modes:
// 1 - idling: At ease - circulation when humidifier and heater are off
// 2 - Humidifying: During Humidification
// 3 - Heating: During Heating (humidifying or not) 
enum Mode { Idle = 0, Humidifying, Heating };

struct ChamberState {
  int temperature;
  bool heaterState;
  unsigned long timeLastChangeHeater;
  int humidity;
  bool humidifierState;
  unsigned long timeLastChangeHumidifier;
  int switchLight;
  unsigned long timeLastChangeSwitchLight;
  int fanPowerLevel;
  Mode mode;

  ChamberState& operator=(const ChamberState& other);
};

#endif
