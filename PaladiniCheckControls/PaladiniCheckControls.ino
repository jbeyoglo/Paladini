#include <Wire.h>
#include <TM1637Display.h>
#include <Adafruit_SHT4x.h>

// Switches
#define BUTON1_PIN 39
#define BUTON2_PIN 41
#define BUTON3_PIN 43
#define BUTON4_PIN 45

// Define the DIO pins for each display TM1637
#define CLK_PIN 22
#define DIO_PIN_1 32
#define DIO_PIN_2 30
#define DIO_PIN_3 26
#define DIO_PIN_4 28

// Create instances of the TM1637Display for each display
TM1637Display display1(CLK_PIN, DIO_PIN_1);
TM1637Display display2(CLK_PIN, DIO_PIN_2);
TM1637Display display3(CLK_PIN, DIO_PIN_3);
TM1637Display display4(CLK_PIN, DIO_PIN_4);

#include "TM1651.h"
#define LEVEL_CLK_PIN 36 //pins definitions for TM1651 and can be changed to other ports       
#define LEVEL_DIO_PIN 38
TM1651 batteryDisplay(LEVEL_CLK_PIN,LEVEL_DIO_PIN);
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

#define LIGHT_PIN 11
bool lightPrevOn = false;
#define HUMIDIFIER_PIN 10
bool humidifierPrevOn = false;
#define FAN_PIN 13
int fanPowerLevel = 0;

const int HumLed_RedPin = 50;
const int HumLed_GreenPin = 48;
const int HumLed_BluePin = 52; 
const int HeatLed_RedPin = 44;
const int HeatLed_GreenPin = 42;
const int HeatLed_BluePin = 46; 

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
      virtualPositionE1-- ; // Could be -5 or -10
    }
    else {
      virtualPositionE1++ ; // Could be +5 or +10
    }

    // Restrict value from 0 to +100
    //virtualPosition = min(100, max(0, virtualPosition));
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
      virtualPositionE2-- ; // Could be -5 or -10
    }
    else {
      virtualPositionE2++ ; // Could be +5 or +10
    }

    // Restrict value from 0 to +100
    //virtualPosition = min(100, max(0, virtualPosition));
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

  pinMode(BUTON1_PIN, INPUT_PULLUP);
  pinMode(BUTON2_PIN, INPUT_PULLUP);
  pinMode(BUTON3_PIN, INPUT_PULLUP);
  pinMode(BUTON4_PIN, INPUT_PULLUP);

  digitalWrite(LIGHT_PIN, HIGH);
  pinMode(LIGHT_PIN, OUTPUT);
  
  digitalWrite(HUMIDIFIER_PIN, HIGH);
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  
  pinMode(FAN_PIN, OUTPUT);

  // Set the display brightness for all displays (0-7)
  display1.setBrightness(7);
  display2.setBrightness(7);
  display3.setBrightness(7);
  display4.setBrightness(7);

  batteryDisplay.init();
  batteryDisplay.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;

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
}


// ------------------------------------------------------------------
// LOOP
// ------------------------------------------------------------------
void loop() {

  sensors_event_t humidityUp, tempUp;
  sensors_event_t humidityDown, tempDown;
  
  TCA9548A(1);
  sht4Up.getEvent(&humidityUp, &tempUp);// populate temp and humidity objects with fresh data
//  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  (int)( tempUp.temperature * 100 );
  TCA9548A(0);
  sht4Up.getEvent(&humidityDown, &tempDown);// populate temp and humidity objects with fresh data
  
  // Display different numbers on each display
  int number1 = virtualPositionE1;
  int number2 = virtualPositionE3;
  int number3 = virtualPositionE3;
  int number4 = 4445;
  
  int sensorVal = digitalRead(BUTON1_PIN);
  if (sensorVal == LOW) {
      number1 = 8888;
  }

  sensorVal = digitalRead(BUTON2_PIN);
  if (sensorVal == HIGH) {
    number3 = (int)( tempUp.temperature * 100 );
    number4 = (int)( humidityUp.relative_humidity );;
  } else {
    number3 = (int)( tempDown.temperature * 100 );
    number4 = (int)( humidityDown.relative_humidity );;    
  }
  
  sensorVal = digitalRead(BUTON3_PIN);
  if (sensorVal == LOW) {
      number3 = 8888;
  }

  sensorVal = digitalRead(BUTON4_PIN);
  if (sensorVal == LOW) {
      number4 = 8888;
  }


  // Display numbers on each display
  display1.showNumberDec(number1);
  //display2.showNumberDec(number2);
  display3.showNumberDecEx(number3, 0b01000000, false, 4, 0);
  display4.showNumberDec(number4);

  batteryDisplay.displayLevel(batteryLevel);
  batteryLevel = ( batteryLevel < 7 ? batteryLevel+1 : 0 );


  if( virtualPositionE1 >= 60 ) {
    if( !lightPrevOn ) {
      digitalWrite(LIGHT_PIN, LOW);
      lightPrevOn = true;
    }
  } else {
    digitalWrite(LIGHT_PIN, HIGH);
    lightPrevOn = false;
  }
  
  if( virtualPositionE2 >= 60 ) {
    if( !humidifierPrevOn ) {
      digitalWrite(HUMIDIFIER_PIN, LOW);
      humidifierPrevOn = true;
    }
  } else {
    digitalWrite(HUMIDIFIER_PIN, HIGH);
    humidifierPrevOn = false;
  }

  int newPowerLevel = map( virtualPositionE3, 0, 100, 0, 255);
  if( newPowerLevel != fanPowerLevel ) {
    fanPowerLevel = newPowerLevel;
    analogWrite(FAN_PIN, fanPowerLevel);
  }
  display2.showNumberDec(fanPowerLevel);
}
