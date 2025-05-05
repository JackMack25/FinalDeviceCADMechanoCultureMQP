#include <AccelStepper.h>
#include <EEPROM.h>

#define MotorInterfaceType 1
const int stepPin = 3;
const int dirPin = 2;
AccelStepper stepper = AccelStepper(MotorInterfaceType, stepPin, dirPin);

// EEPROM addresses for storing parameters
const int EEPROM_VALID_FLAG = 0;   // Address to store validation flag (42 if parameters exist)
const int STRAIN_ADDR = 1;         // Start addresses for each parameter
const int UP_SPEED_ADDR = 5;
const int HOLD_UP_ADDR = 9;
const int DOWN_SPEED_ADDR = 13;
const int HOLD_DOWN_ADDR = 17;
const int CYCLES_ADDR = 21;
const int STEP_ADDR = 25;

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

const int maxSpeed = 3000;
int currentSpeed = 0;
int upSpeed = 0;
int holdUp = 0;
int downSpeed = 0;
int holdDown = 0;
int strain = 0;
int cycles = 0;
int step = 0;

long initialPosition = 0;
boolean returningHome = false;
int returnSpeed = 500;
boolean shouldStop = false;

const int motorVoltagePin = A4;
const int powerLossThreshold = 410; // ~2V with 5V ADC
bool motorPowerLost = false;
bool motorPowerRestored = false;

// Define a push button pin to start the saved program
const int startButtonPin = 8;
boolean runningFromEEPROM = false;

void setup() {
  delay(2000);
  stepper.setMaxSpeed(maxSpeed);
  stepper.setCurrentPosition(0);
  stepper.setSpeed(0);
  stepper.stop();
  
  // Set up button input
  pinMode(startButtonPin, INPUT_PULLUP);
  
  Serial.begin(9600);
  
  // Check if we have saved parameters in EEPROM
  byte validFlag = EEPROM.read(EEPROM_VALID_FLAG);
  
  // If valid flag is set, load parameters from EEPROM
  if (validFlag == 42) {
    loadParamsFromEEPROM();
    Serial.println("Parameters loaded from EEPROM");
    
    // If no computer is connected (no serial data for a few seconds), 
    // automatically start the program with saved parameters
    delay(2000);
    if (Serial.available() <= 0) {
      runningFromEEPROM = true;
      Serial.println("Running saved program");
    }
  } else {
    Serial.println("No saved parameters found");
  }
}

void loop() {
  monitorMotorVoltage();

  // Check if button is pressed to start the saved program
  if (!runningFromEEPROM && digitalRead(startButtonPin) == LOW) {
    delay(2000); // Debounce
    if (digitalRead(startButtonPin) == LOW) {
      loadParamsFromEEPROM();
      runningFromEEPROM = true;
      initialPosition = stepper.currentPosition();
      returningHome = false;
      shouldStop = false;
      while (digitalRead(startButtonPin) == LOW) {
        delay(1000); // Wait for button release
      }
    }
  }

  // Check for new serial data
  recvWithStartEndMarkers();
  if (newData == true) {
    parseData();
    if (strain == 0 && upSpeed == 0 && holdUp == 0 && downSpeed == 0 && holdDown == 0 && cycles == 0 && step == 0) {
      shouldStop = true;
    } else {
      // Save parameters to EEPROM
      saveParamsToEEPROM();
      Serial.println("Parameters saved to EEPROM");
      
      initialPosition = stepper.currentPosition();
      returningHome = false;
      shouldStop = false;
      runningFromEEPROM = false;
    }
    newData = false;
  }

  if (motorPowerRestored) {
    // Return to initialPosition
    long delta = initialPosition - stepper.currentPosition();
    if (delta != 0) {
      stepper.setSpeed(delta > 0 ? returnSpeed : -returnSpeed);
      stepper.runSpeed();
    } else {
      motorPowerLost = false;
      motorPowerRestored = false;
      shouldStop = false;
    }
    return;
  }

  if (motorPowerLost) {
    stepper.setSpeed(0);
    stepper.stop();
    return;
  }

  if (shouldStop && !returningHome) returningHome = true;

  if (step > 0 && !returningHome && !shouldStop) {
    stepper.setSpeed(99);
    stepper.runSpeed();
    if (stepper.currentPosition() >= step) {
      step = 0;
      stepper.setCurrentPosition(0);
    }
  } else if (step < 0 && !returningHome && !shouldStop) {
    stepper.setSpeed(-99);
    stepper.runSpeed();
    if (stepper.currentPosition() <= step) {
      step = 0;
      stepper.setCurrentPosition(0);
    }
  } else if (step == 0) {
    if ((cycles <= 0 || shouldStop) && !returningHome) returningHome = true;

    if (returningHome) {
      if (stepper.currentPosition() < initialPosition) {
        stepper.setSpeed(returnSpeed);
      } else if (stepper.currentPosition() > initialPosition) {
        stepper.setSpeed(-returnSpeed);
      } else {
        stepper.setSpeed(0);
        stepper.stop();
        returningHome = false;
        shouldStop = false;
      }
      stepper.runSpeed();
    } else if (strain == 0) {
      stepper.stop();
      delay(100);
    } else if (stepper.currentPosition() >= strain) {
      currentSpeed = downSpeed;
      delay(holdUp);
    } else if (stepper.currentPosition() <= 0) {
      if (currentSpeed != upSpeed) {
        currentSpeed = upSpeed;
        delay(holdDown);
        cycles--;
        if (cycles <= 0 && !returningHome) returningHome = true;
      }
    }

    if (cycles > 0 && !returningHome && !shouldStop) {
      stepper.setSpeed(currentSpeed);
      stepper.runSpeed();
    }
  }
}

// Write an integer (4 bytes) to EEPROM
void writeIntToEEPROM(int address, int value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    EEPROM.write(address + i, p[i]);
  }
}

// Read an integer (4 bytes) from EEPROM
int readIntFromEEPROM(int address) {
  int value = 0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    p[i] = EEPROM.read(address + i);
  }
  return value;
}

// Save parameters to EEPROM
void saveParamsToEEPROM() {
  EEPROM.write(EEPROM_VALID_FLAG, 42);  // Set validation flag
  writeIntToEEPROM(STRAIN_ADDR, strain);
  writeIntToEEPROM(UP_SPEED_ADDR, upSpeed);
  writeIntToEEPROM(HOLD_UP_ADDR, holdUp);
  writeIntToEEPROM(DOWN_SPEED_ADDR, downSpeed);
  writeIntToEEPROM(HOLD_DOWN_ADDR, holdDown);
  writeIntToEEPROM(CYCLES_ADDR, cycles);
  writeIntToEEPROM(STEP_ADDR, step);
}

// Load parameters from EEPROM
void loadParamsFromEEPROM() {
  strain = readIntFromEEPROM(STRAIN_ADDR);
  upSpeed = readIntFromEEPROM(UP_SPEED_ADDR);
  holdUp = readIntFromEEPROM(HOLD_UP_ADDR);
  downSpeed = readIntFromEEPROM(DOWN_SPEED_ADDR);
  holdDown = readIntFromEEPROM(HOLD_DOWN_ADDR);
  cycles = readIntFromEEPROM(CYCLES_ADDR);
  step = readIntFromEEPROM(STEP_ADDR);
  
  // Reset cycles to the stored value if loading from EEPROM
  // This ensures we run the full program
  currentSpeed = upSpeed;
}

void monitorMotorVoltage() {
  int voltageReading = analogRead(motorVoltagePin);
  if (voltageReading < powerLossThreshold && !motorPowerLost) {
    motorPowerLost = true;
    shouldStop = true;
  }
  if (motorPowerLost && voltageReading >= powerLossThreshold) {
    motorPowerRestored = true;
  }
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress) {
      if (rc != endMarker) {
        receivedChars[ndx++] = rc;
        if (ndx >= numChars) ndx = numChars - 1;
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {
  char * strtokIndx;
  strtokIndx = strtok(receivedChars, ","); strain = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); upSpeed = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); holdUp = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); downSpeed = -1 * atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); holdDown = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); cycles = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); step = atoi(strtokIndx);
}