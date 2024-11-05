
// Modes:
// 1 - idling: At ease - circulation when humidifier and heater are off
// 2 - Humidifying: During Humidification
// 3 - Heating: During Heating (humidifying or not) 

struct Configuration {
  volatile int goalTemperature;
  volatile int goalHumidity;
  volatile int goalFanSpeed[3];
  uint32_t crc;
};

void ReadConfiguration(Configuration&);

void UpdateConfiguration(Configuration&);
