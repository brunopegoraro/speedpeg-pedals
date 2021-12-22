#include <BleGamepad.h> 
#include <EEPROM.h>
#include "HX711.h"

#define numOfButtons        10
#define numOfHatSwitches    0
#define enableX             false
#define enableY             false
#define enableZ             false
#define enableRZ            false
#define enableRX            false
#define enableRY            false
#define enableSlider1       false
#define enableSlider2       false
#define enableRudder        false
#define enableThrottle      true
#define enableAccelerator   true
#define enableBrake         true
#define enableSteering      false

byte previousButtonStates[numOfButtons];
byte currentButtonStates[numOfButtons];
byte buttonPins[numOfButtons] = { 0, 35, 17, 18, 19, 23, 25, 26, 27, 32 };
byte physicalButtons[numOfButtons] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

#define GAS_PIN 4
#define BRAKE_PIN 23
#define CLUTCH_PIN 0
#define TARE_LOADCELL_PIN 19
#define LED_POWER_PIN 17
#define LED_STATUS_PIN 16

BleGamepad bleGamepad("SpeedPeg Driving Controller", "SpeedPeg", 100);

#define BITS_12_RANGE_LIMIT 4095 // 12-bit range

struct Pedal {
  byte pin;
  HX711 loadcell;
  bool useLoadcell;
  float min, max, cur;
  int axis;
};

static Pedal* gasPedal = 0;
static Pedal* brakePedal = 0;
static Pedal* clutchPedal = 0;

struct EepromData
{
  float gasMin, gasMax, brakeMin, brakeMax, clutchMin, clutchMax;
  int neverSalvedIndicator; // 100 = salved, outher = never salved
};

void loadFromEEPROM()
{  
  EepromData fromEEPROM;
  EEPROM.begin(512);
  EEPROM.get(0, fromEEPROM);

  if (fromEEPROM.neverSalvedIndicator == 100) {
    gasPedal->min = fromEEPROM.gasMin;
    gasPedal->max = fromEEPROM.gasMax;
    brakePedal->min = fromEEPROM.brakeMin;
    brakePedal->max = fromEEPROM.brakeMax;
    clutchPedal->min = fromEEPROM.clutchMin;
    clutchPedal->max = fromEEPROM.clutchMax;
  }
}

void saveToEEPROM()
{  
  EepromData toEEPROM;
  
  toEEPROM.gasMin = gasPedal->min;
  toEEPROM.gasMax = gasPedal->max;
  toEEPROM.brakeMin = brakePedal->min;
  toEEPROM.brakeMax = brakePedal->max;
  toEEPROM.clutchMin = clutchPedal->min;
  toEEPROM.clutchMax = clutchPedal->max;
  toEEPROM.neverSalvedIndicator = 100;

  EEPROM.begin(512);
  EEPROM.put(0, toEEPROM);
  EEPROM.commit();
}

void setup()
{
  Serial.begin(115200);

  pinMode(LED_POWER_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT); 

  digitalWrite(LED_POWER_PIN, HIGH);

  bleGamepad.setAutoReport(false);
  bleGamepad.setControllerType(CONTROLLER_TYPE_GAMEPAD);  //CONTROLLER_TYPE_JOYSTICK, CONTROLLER_TYPE_GAMEPAD (DEFAULT), CONTROLLER_TYPE_MULTI_AXIS
  bleGamepad.begin(numOfButtons,numOfHatSwitches,enableX,enableY,enableZ,enableRZ,enableRX,enableRY,enableSlider1,enableSlider2,enableRudder,enableThrottle,enableAccelerator,enableBrake,enableSteering);  

  // BUTTONS

  for (byte currentPinIndex = 0 ; currentPinIndex < numOfButtons ; currentPinIndex++)
  {
    pinMode(buttonPins[currentPinIndex], INPUT_PULLUP);
    previousButtonStates[currentPinIndex] = HIGH;
    currentButtonStates[currentPinIndex] =  HIGH;
  }

  // PEDALS

  // Para poderem ser usados como ponteiros genéricos no parametro de processPedal()
  Pedal* gas = new Pedal();
  Pedal* brake = new Pedal();
  Pedal* clutch = new Pedal();

  gas->pin = GAS_PIN; 
  brake->pin = BRAKE_PIN;
  clutch->pin = CLUTCH_PIN; 
  
  gas->min = -32767;
  gas->max = 32767;

  clutch->min = -32767;
  clutch->max = 32767;

  // Min e max são automaticamente descobertos ao pressionar a loadcell pela primeira vez
  brake->min = 500000;
  brake->max = 0;

  //Set accelerator and brake to min
  bleGamepad.setAccelerator(gas->min);
  bleGamepad.setBrake(brake->min);
  bleGamepad.setThrottle(clutch->min);

  brake->useLoadcell = true;
  brake->loadcell.begin(brake->pin /* dout pin */, 22 /* sck pin */);
  brake->loadcell.set_scale(1.0); 
  brake->loadcell.set_offset(0);

  gasPedal = gas;
  brakePedal = brake;
  clutchPedal = clutch;

  pinMode(GAS_PIN, INPUT);
  pinMode(TARE_LOADCELL_PIN, INPUT);

  loadFromEEPROM();
} 

bool processPedal(struct Pedal* pedal, bool debug = false) {

  bool result = false;

  float raw_value = pedal->cur;
    
  if ((pedal->useLoadcell ) && (pedal->loadcell.is_ready())) {
    raw_value = pedal->loadcell.get_units() * 0.453592; //This multiples the original value by 0.454 to convert from lbs to kg.
  }
  else {
    raw_value = analogRead(pedal->pin);  
  }

  result = raw_value != pedal->cur;

  if (result) {

      pedal->cur = raw_value;

      bool needSalveToEeprom = false;

      if (pedal->useLoadcell) {

        float oldValue;
        if (pedal->cur >= 0) {
          oldValue = pedal->min;
          pedal->min = min(pedal->cur, pedal->min);
          needSalveToEeprom = pedal->min < oldValue; 
        }
  
        oldValue = pedal->max;
        pedal->max = max(pedal->cur, pedal->max);
        needSalveToEeprom = needSalveToEeprom || (pedal->max > oldValue); 
    
        pedal->axis = map(pedal->cur, pedal->min, pedal->max, 0, BITS_12_RANGE_LIMIT);
        pedal->axis = pedal->axis < 0 ? 0 : pedal->axis;
      }
      else {
        pedal->axis = map(pedal->cur, 0, BITS_12_RANGE_LIMIT, -32767, 32767);  
      }

      if (needSalveToEeprom) 
        saveToEEPROM();  

      if (debug) {
        Serial.print("pedal->pin: ");
        Serial.print(pedal->pin);
        
        Serial.print(" | pedal->axis: ");
        Serial.print(pedal->axis);
    
        Serial.print(" | pedal->cur: ");
        Serial.print(pedal->cur);
    
        Serial.print(" | pedal->min: ");
        Serial.print(pedal->min);
    
        Serial.print(" | pedal->max: ");
        Serial.print(pedal->max);

        Serial.print(" | pedal->useLoadcell: ");
        Serial.print(pedal->useLoadcell);
  
        Serial.print(" | Need Salve to EEPROM: ");
        Serial.println(needSalveToEeprom);
      }
   }

   return result;
}

void loop()
{
  bool status_changed = false;
  
  if (bleGamepad.isConnected()) 
  { 
    // PEDALS
      
    if (processPedal(brakePedal)) {        
      bleGamepad.setBrake(brakePedal->axis);
      bleGamepad.sendReport();
      status_changed = true;
    }

    if (processPedal(gasPedal, true)) {        
      bleGamepad.setThrottle(gasPedal->axis);
      bleGamepad.sendReport();
      status_changed = true;
    }

    int buttonTareLoadcellState = digitalRead(TARE_LOADCELL_PIN);
    if (buttonTareLoadcellState == HIGH) {
      if (brakePedal->loadcell.is_ready()) {
        brakePedal->loadcell.tare(); 
        Serial.println("Loadcell tared");
      }
    }

    // BUTTONS

    for (byte currentIndex = 0 ; currentIndex < numOfButtons ; currentIndex++) {
      currentButtonStates[currentIndex]  = digitalRead(buttonPins[currentIndex]);

      if (currentButtonStates[currentIndex] != previousButtonStates[currentIndex]) {
        if(currentButtonStates[currentIndex] == LOW) {
          bleGamepad.press(physicalButtons[currentIndex]);
          status_changed = true;
        }
        else {
          bleGamepad.release(physicalButtons[currentIndex]);
          status_changed = true;
        }
      } 
    }
    
    if (currentButtonStates != previousButtonStates) {
      for (byte currentIndex = 0 ; currentIndex < numOfButtons ; currentIndex++) {
        previousButtonStates[currentIndex] = currentButtonStates[currentIndex]; 
      }
      
      bleGamepad.sendReport();
    }
  }  

  if (status_changed) {
    digitalWrite(LED_STATUS_PIN, HIGH);
  }

  delay(20);
  
  digitalWrite(LED_STATUS_PIN, LOW);
} 
