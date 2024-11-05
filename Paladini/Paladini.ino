#include <Wire.h>
#include <TM1637Display.h>
#include <Adafruit_SHT4x.h>
#include <EEPROM.h>
#include "CRC.h"
#include <RGBLed.h>
#include "TM1651.h"

// Switches
#define PIN_SW_LIGHT 39
#define PIN_SW_SENSOR_SELECTOR 41
#define PIN_SW_FORCE_HEATER 43
#define PIN_SW_FORCE_HUMIDIFIER 45

// Define the DIO pins for each display TM1637
#define PIN_DISPLAY_CLK 22
#define PIN_DISP_GOAL_TEMP_DIO 32
#define PIN_DISP_GOAL_HUM_DIO 30
#define PIN_DISP_CURRENT_TEMP_DIO 26
#define PIN_DISP_CURRENT_HUM_DIO 28

// Create instances of the TM1637Display for each display
TM1637Display dispGoalTemp(PIN_DISPLAY_CLK, PIN_DISP_GOAL_TEMP_DIO);
TM1637Display dispGoalHum(PIN_DISPLAY_CLK, PIN_DISP_GOAL_HUM_DIO);
TM1637Display dispCurrentTemp(PIN_DISPLAY_CLK, PIN_DISP_CURRENT_TEMP_DIO);
TM1637Display dispCurrentHum(PIN_DISPLAY_CLK, PIN_DISP_CURRENT_HUM_DIO);

#define PIN_GAUGE_CLK 36
#define PIN_GAUGE_DIO 38
TM1651 gaugeDisplay(PIN_GAUGE_CLK, PIN_GAUGE_DIO);
uint8_t batteryLevel = 0;

#define PIN_LED_HumRed 44
#define PIN_LED_HumGreen 42
#define PIN_LED_HumBlue 46 
#define PIN_LED_HeatRed 50
#define PIN_LED_HeatGreen 48
#define PIN_LED_HeatBlue 52 
RGBLed ledHumidifier(PIN_LED_HumRed, PIN_LED_HumGreen, PIN_LED_HumBlue, RGBLed::COMMON_CATHODE);
RGBLed ledHeater(PIN_LED_HeatRed, PIN_LED_HeatGreen, PIN_LED_HeatBlue, RGBLed::COMMON_CATHODE);

Adafruit_SHT4x sht4Up = Adafruit_SHT4x();
Adafruit_SHT4x sht4Down = Adafruit_SHT4x();

// Rotary encoders: Used for generating interrupts using CLK signal
const int PinE1A = 19;
const int PinE1B = 33;
const int PinE2A = 18;
const int PinE2B = 31;
const int PinE3A = 2;
const int PinE3B = 35;
// Updated by the ISR (Interrupt Service Routine)
// volatile int virtualPositionE3 = 50;

#define PIN_LIGHT 11
#define PIN_HUMIDIFIER 10
const unsigned long MIN_TIME_HUMIDIFIER_OFF = ( 40 * 1000 ); // 20 seconds
const unsigned long MIN_TIME_HUMIDIFIER_ON = ( 20 * 1000 ); // 20 seconds
#define PIN_FAN 13
const unsigned long MIN_TIME_HEATER_OFF = ( 2 * 60 * 1000 ); // 2 minutes
int fanPowerLevel = 0;

#include "Configuration.h"
Configuration configuration;

#include "ChamberState.h"
ChamberState current, nextLoop;

// ------------------------------------------------------------------
// Select I2C BUS
// ------------------------------------------------------------------
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}

// ------------------------------------------------------------------
// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
// ------------------------------------------------------------------
void isrE1 ()  {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 5) {
    if (digitalRead(PinE1B) == LOW) {
      configuration.goalTemperature-=10 ; // Could be -5 or -10
    }
    else {
      configuration.goalTemperature+=10 ; // Could be +5 or +10
    }

    // Restrict value from 0 to +100
    configuration.goalTemperature = min(2200, max(500, configuration.goalTemperature));
  }
  // Keep track of when we were here last (no more than every 5ms)
  lastInterruptTime = interruptTime;
}

// ------------------------------------------------------------------
// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
// ------------------------------------------------------------------
void isrE2 ()  {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 5) {
    if (digitalRead(PinE2B) == LOW) {
      configuration.goalHumidity-- ; // Could be -5 or -10
    }
    else {
      configuration.goalHumidity++ ; // Could be +5 or +10
    }

    // Restrict value from 0 to +100
    configuration.goalHumidity = min(100, max(0, configuration.goalHumidity));
  }
  // Keep track of when we were here last (no more than every 5ms)
  lastInterruptTime = interruptTime;
}


// ------------------------------------------------------------------
// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
// ------------------------------------------------------------------
void isrE3 ()  {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 5) {
    if (digitalRead(PinE3B) == LOW) {
      configuration.goalFanSpeed[current.mode]--;
      // virtualPositionE3-- ;
    }
    else {
      configuration.goalFanSpeed[current.mode]++;
      // virtualPositionE3++ ;
    }

    // Restrict value from 0 to +31
    // virtualPositionE3 = min(31, max(0, virtualPositionE3));
    configuration.goalFanSpeed[current.mode] = min(31, max(0, configuration.goalFanSpeed[current.mode]));
  }
  // Keep track of when we were here last (no more than every 5ms)
  lastInterruptTime = interruptTime;
}

// ------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  ledHumidifier.off();
  ledHeater.off();

  pinMode(PIN_SW_LIGHT, INPUT_PULLUP);
  pinMode(PIN_SW_SENSOR_SELECTOR, INPUT_PULLUP);
  pinMode(PIN_SW_FORCE_HEATER, INPUT_PULLUP);
  pinMode(PIN_SW_FORCE_HUMIDIFIER, INPUT_PULLUP);

  digitalWrite(PIN_LIGHT, HIGH);
  pinMode(PIN_LIGHT, OUTPUT);

  digitalWrite(PIN_HUMIDIFIER, HIGH);
  pinMode(PIN_HUMIDIFIER, OUTPUT);
  current.humidifierState = false;
  current.timeLastChangeHumidifier = millis();
  current.timeLastChangeHeater = millis();
  current.mode = Idle;

  pinMode(PIN_FAN, OUTPUT);

  // Set the display brightness for all displays (0-7)
  dispGoalTemp.setBrightness(7);
  dispGoalHum.setBrightness(7);
  dispCurrentTemp.setBrightness(7);
  dispCurrentHum.setBrightness(7);

  gaugeDisplay.init();
  gaugeDisplay.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;

  // Start I2C communication with the Multiplexer
  Wire.begin();
  
  // Init SHT40
  TCA9548A(0);    
  Serial.println("Adafruit SHT4x test - Sensor UP");
  if (! sht4Up.begin()) {
    Serial.println("Couldn't find SHT4x - UP");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4Up.readSerial(), HEX);
  sht4Up.setPrecision(SHT4X_HIGH_PRECISION);
  sht4Up.setHeater(SHT4X_NO_HEATER);

  TCA9548A(1);    
  Serial.println("Adafruit SHT4x test - Sensor DOWN");
  if (! sht4Down.begin()) {
    Serial.println("Couldn't find SHT4x - DOWN");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4Down.readSerial(), HEX);
  sht4Down.setPrecision(SHT4X_HIGH_PRECISION);
  sht4Down.setHeater(SHT4X_NO_HEATER);

  // Rotary pulses are INPUTs
  pinMode(PinE1A, INPUT);
  pinMode(PinE1B, INPUT);
  // Attach the routine to service the interrupts
  attachInterrupt(digitalPinToInterrupt(PinE1A), isrE1, LOW);

  pinMode(PinE2A, INPUT);
  pinMode(PinE2B, INPUT);
  // Attach the routine to service the interrupts
  attachInterrupt(digitalPinToInterrupt(PinE2A), isrE2, LOW);

  pinMode(PinE3A, INPUT);
  pinMode(PinE3B, INPUT);
  // Attach the routine to service the interrupts
  attachInterrupt(digitalPinToInterrupt(PinE3A), isrE3, LOW);

  ReadConfiguration(configuration);
}


// ------------------------------------------------------------------
// LOOP
// ------------------------------------------------------------------
void loop() {

  UpdateConfiguration(configuration);

  // read the proper temp/hum sensor
  sensors_event_t humidity, temp;
  if( digitalRead(PIN_SW_SENSOR_SELECTOR) == LOW ) {
    TCA9548A(0);
    sht4Up.getEvent(&humidity, &temp);
  } else {
    TCA9548A(1);
    sht4Down.getEvent(&humidity, &temp);
  } 
  nextLoop.temperature = (int)( temp.temperature * 100 );
  nextLoop.humidity = (int)( humidity.relative_humidity );

  // Display temperature and humidty goal and current values
  dispGoalTemp.showNumberDecEx(configuration.goalTemperature, 0b01000000, false, 4, 0);
  dispGoalHum.showNumberDec(configuration.goalHumidity);
  dispCurrentTemp.showNumberDecEx(nextLoop.temperature, 0b01000000, false, 4, 0);
  dispCurrentHum.showNumberDec(nextLoop.humidity);

  // humidifier + status led (red)
  int forceHumidifier = digitalRead(PIN_SW_FORCE_HUMIDIFIER);
  nextLoop.humidifierState = ( forceHumidifier == LOW 
                                || ( configuration.goalHumidity > nextLoop.humidity 
                                    && (current.timeLastChangeHumidifier+MIN_TIME_HUMIDIFIER_OFF) < millis() ) );
  if( nextLoop.humidifierState != current.humidifierState ) {
    if( nextLoop.humidifierState ) {
      digitalWrite(PIN_HUMIDIFIER, LOW);
      ledHumidifier.setColor(RGBLed::BLUE);
    } else {
      digitalWrite(PIN_HUMIDIFIER, HIGH);
      ledHumidifier.off();
      nextLoop.timeLastChangeHumidifier = millis();
    }
  }

  // heater + status led (blue)
  int forceHeater = digitalRead(PIN_SW_FORCE_HEATER);
  nextLoop.heaterState = ( forceHeater == LOW || 
                  ( configuration.goalTemperature > nextLoop.temperature 
                        && (current.timeLastChangeHeater+MIN_TIME_HEATER_OFF) < millis() ) );
  if( nextLoop.heaterState != current.heaterState ) {
    if( nextLoop.heaterState ) {
      //digitalWrite(PIN_HEATER, LOW);
      ledHeater.setColor(RGBLed::RED);
    } else {
      //digitalWrite(PIN_HEATER, HIGH);
      ledHeater.off();
      nextLoop.timeLastChangeHeater = millis();
    }
  }

  // set the right mode
  if( nextLoop.heaterState ) {
    nextLoop.mode = Mode::Heating;
  } else if( nextLoop.humidifierState ) { 
      nextLoop.mode = Mode::Humidifying;
  } else {
      nextLoop.mode = Mode::Idle;
  }

  // fan speed - WIP, introduce modes...
  nextLoop.fanPowerLevel = map( configuration.goalFanSpeed[nextLoop.mode], 0, 31, 0, 255);
  if( nextLoop.fanPowerLevel != current.fanPowerLevel ) {
    analogWrite(PIN_FAN, nextLoop.fanPowerLevel);
    int newGaugeLevel = map( configuration.goalFanSpeed[nextLoop.mode], 0, 31, 0, 8);
    gaugeDisplay.displayLevel(newGaugeLevel);
  }

  // Light
  nextLoop.switchLight = digitalRead(PIN_SW_LIGHT);
  if( current.switchLight != nextLoop.switchLight ) {
    // pressing the switch
    if( nextLoop.switchLight == LOW ) {
      // it could mean it was on and I want to turn it off 
      if( nextLoop.timeLastChangeSwitchLight > 0 ) {
        digitalWrite(PIN_LIGHT, HIGH);
        nextLoop.timeLastChangeSwitchLight = 0;
      } else {
        // or just turn the light on
        digitalWrite(PIN_LIGHT, LOW);
        nextLoop.timeLastChangeSwitchLight = millis();
      }
    } else {
      // releasing the switch, if pressed for less than 1sec it will stay on
      if( (nextLoop.timeLastChangeSwitchLight+1000) < millis() ) {
        digitalWrite(PIN_LIGHT, HIGH);
        nextLoop.timeLastChangeSwitchLight = 0;
      }
    }
  }

  current = nextLoop;

}
