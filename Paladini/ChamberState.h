#ifndef ChamberState_H
#define ChamberState_H


// Modes:
// 1 - idling: At ease - circulation when humidifier and heater are off
// 2 - Humidifying: During Humidification
// 3 - Heating: During Heating (humidifying or not) 

enum Mode { IDLING, HUMIDIFYING, HEATING };

struct ChamberState {
  int temperature;
  int humidity;
  int switchLight;
  unsigned long timeLastChangeSwitchLight;
  int switchForceHeater;
  int switchForceHumidity;
  int fanSpeed;
  Mode mode;

  ChamberState& operator=(const ChamberState& other);
};

#endif
