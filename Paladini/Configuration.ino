#include <EEPROM.h>
#include "CRC.h"



void ReadConfiguration( Configuration &configuration) {
  EEPROM.get(0x00, configuration);
  uint8_t * data = (uint8_t *) &configuration;
  int crc2Verify = calcCRC32(data, sizeof(Configuration)-sizeof(configuration.crc));
  if( configuration.crc != crc2Verify ) {
    configuration.goalTemperature = 1800;
    configuration.goalHumidity = 60;
    configuration.goalFanSpeed[Idle] = 20;
    configuration.goalFanSpeed[Humidifying] = 60;
    configuration.goalFanSpeed[Heating] = 40;
  }
}

void UpdateConfiguration(Configuration &configuration) {

  uint8_t * data = (uint8_t *) &configuration;
  int crc2Verify = calcCRC32(data, sizeof(Configuration)-sizeof(configuration.crc));

  if( configuration.crc != crc2Verify ) {
    configuration.crc = crc2Verify;
    EEPROM.put(0x00, configuration);
  }
}
