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
#define CLUTCH_PIN 30
#define TARE_LOADCELL_PIN 19

BleGamepad bleGamepad("RacerPeg Driving Controller", "RacerPeg", 100);

#define MIN_PEDAL_INIT 500000
#define RANGE_LIMIT 4095 // 12-bit range

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

  gas->pin = GAS_PIN; // definir o pino certo no futuro
  brake->pin = BRAKE_PIN;
  clutch->pin = CLUTCH_PIN; // definir o pino certo no futuro
  
  gas->min = brake->min = clutch->min = MIN_PEDAL_INIT;
  gas->max = brake->max = clutch->max = 0;

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

      float oldValue;
      if (pedal->cur >= 0) {
        oldValue = pedal->min;
        pedal->min = min(pedal->cur, pedal->min);
        needSalveToEeprom = pedal->min < oldValue; 
      }

      oldValue = pedal->max;
      pedal->max = max(pedal->cur, pedal->max);
      needSalveToEeprom = needSalveToEeprom || (pedal->max > oldValue); 
  
      pedal->axis = map(pedal->cur, pedal->min, pedal->max, 0, RANGE_LIMIT);
      pedal->axis = pedal->axis < 0 ? 0 : pedal->axis;

      if (needSalveToEeprom) 
        saveToEEPROM();  

      if (debug) {
        Serial.print("Mapped Value: ");
        Serial.print(pedal->axis);
    
        Serial.print(" | Cur Cell Value: ");
        Serial.print(pedal->cur);
    
        Serial.print(" | Min Load Cell Value: ");
        Serial.print(pedal->min);
    
        Serial.print(" | Max Load Cell Value: ");
        Serial.print(pedal->max);
  
        Serial.print(" | Need Salve to EEPROM: ");
        Serial.println(needSalveToEeprom);
      }
   }

   return result;
}

void loop()
{
  if (bleGamepad.isConnected()) 
  {    

    // PEDALS
      
    if (processPedal(brakePedal)) {        
      bleGamepad.setBrake(brakePedal->axis);
      bleGamepad.sendReport();
    }

    if (processPedal(gasPedal, true)) {        
      bleGamepad.setAccelerator(gasPedal->axis);
      bleGamepad.sendReport();
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
        }
        else {
          bleGamepad.release(physicalButtons[currentIndex]);
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

  delay(20);
} 
