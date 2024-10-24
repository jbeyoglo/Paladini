#include <Wire.h>
#include <TM1637Display.h>
#include <Adafruit_SHT4x.h>
#include <EEPROM.h>
#include "CRC.h"

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

#include "TM1651.h"
#define LEVEL_CLK_PIN 36 //pins definitions for TM1651 and can be changed to other ports       
#define LEVEL_DIO_PIN 38
TM1651 gaugeDisplay(LEVEL_CLK_PIN,LEVEL_DIO_PIN);
uint8_t batteryLevel = 0;

Adafruit_SHT4x sht4Up = Adafruit_SHT4x();
Adafruit_SHT4x sht4Down = Adafruit_SHT4x();

// Used for generating interrupts using CLK signal
const int PinE1A = 19;
const int PinE1B = 33;
const int PinE2A = 18;
const int PinE2B = 31;
const int PinE3A = 2;
const int PinE3B = 35;
// Keep track of last rotary value
int lastCountE1 = 50;
int lastCountE2 = 50;
int lastCountE3 = 50;
// Updated by the ISR (Interrupt Service Routine)
volatile int virtualPositionE1 = 50;
volatile int virtualPositionE2 = 50;
volatile int virtualPositionE3 = 50;

#define PIN_LIGHT 11
bool lightPrevOn = false;
#define PIN_HUMIDIFIER 10
bool humidifierPrevOn = false;
#define PIN_FAN 13
int fanPowerLevel = 0;

const int HumLed_RedPin = 50;
const int HumLed_GreenPin = 48;
const int HumLed_BluePin = 52; 
const int HeatLed_RedPin = 44;
const int HeatLed_GreenPin = 42;
const int HeatLed_BluePin = 46; 

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
    if (digitalRead(PinE1B) == LOW)
    {
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
    if (digitalRead(PinE2B) == LOW)
    {
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
    if (digitalRead(PinE3B) == LOW)
    {
      virtualPositionE3-- ; // Could be -5 or -10
    }
    else {
      virtualPositionE3++ ; // Could be +5 or +10
    }

    // Restrict value from 0 to +100
    virtualPositionE3 = min(100, max(0, virtualPositionE3));
  }
  // Keep track of when we were here last (no more than every 5ms)
  lastInterruptTime = interruptTime;
}

// ------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(PIN_SW_LIGHT, INPUT_PULLUP);
  pinMode(PIN_SW_SENSOR_SELECTOR, INPUT_PULLUP);
  pinMode(PIN_SW_FORCE_HEATER, INPUT_PULLUP);
  pinMode(PIN_SW_FORCE_HUMIDIFIER, INPUT_PULLUP);

  digitalWrite(PIN_LIGHT, HIGH);
  pinMode(PIN_LIGHT, OUTPUT);
  
  digitalWrite(PIN_HUMIDIFIER, HIGH);
  pinMode(PIN_HUMIDIFIER, OUTPUT);
  
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

  pinMode(HumLed_RedPin, OUTPUT);
  pinMode(HumLed_GreenPin, OUTPUT);
  pinMode(HumLed_BluePin, OUTPUT);
  pinMode(HeatLed_RedPin, OUTPUT);
  pinMode(HeatLed_GreenPin, OUTPUT);
  pinMode(HeatLed_BluePin, OUTPUT);

  analogWrite(HumLed_RedPin, 0);
  analogWrite(HumLed_GreenPin, 0);
  analogWrite(HumLed_BluePin, 255);
  analogWrite(HeatLed_RedPin, 255);
  analogWrite(HeatLed_GreenPin, 0);
  analogWrite(HeatLed_BluePin, 0);

  ReadConfiguration(configuration);
}


// ------------------------------------------------------------------
// LOOP
// ------------------------------------------------------------------
void loop() {

  UpdateConfiguration(configuration);

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

  nextLoop.switchLight = digitalRead(PIN_SW_LIGHT);

  // Display different numbers on each display
  dispGoalTemp.showNumberDecEx(configuration.goalTemperature, 0b01000000, false, 4, 0);
  dispGoalHum.showNumberDec(configuration.goalHumidity);
  dispCurrentTemp.showNumberDecEx(nextLoop.temperature, 0b01000000, false, 4, 0);
  dispCurrentHum.showNumberDec(nextLoop.humidity);

  int number3 = virtualPositionE3;
  

  int sensorVal = digitalRead(PIN_SW_FORCE_HEATER);
  if (sensorVal == LOW) {
      number3 = 8888;
  }

  sensorVal = digitalRead(PIN_SW_FORCE_HUMIDIFIER);
  

  gaugeDisplay.displayLevel(batteryLevel);
  batteryLevel = ( batteryLevel < 7 ? batteryLevel+1 : 0 );


  // if( virtualPositionE1 >= 60 ) {
  //   if( !lightPrevOn ) {
  //     digitalWrite(PIN_LIGHT, LOW);
  //     lightPrevOn = true;
  //   }
  // } else {
  //   digitalWrite(PIN_LIGHT, HIGH);
  //   lightPrevOn = false;
  // }
  
  // if( virtualPositionE2 >= 60 ) {
  //   if( !humidifierPrevOn ) {
  //     digitalWrite(PIN_HUMIDIFIER, LOW);
  //     humidifierPrevOn = true;
  //   }
  // } else {
  //   digitalWrite(PIN_HUMIDIFIER, HIGH);
  //   humidifierPrevOn = false;
  // }

  int newPowerLevel = map( virtualPositionE3, 0, 100, 0, 255);
  if( newPowerLevel != fanPowerLevel ) {
    fanPowerLevel = newPowerLevel;
    analogWrite(PIN_FAN, fanPowerLevel);
  }
  //dispGoalHum.showNumberDec(fanPowerLevel);


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
