/*
  Program to read the electrical meter using a reflective light sensor
  This is the data acquisition part running on an Arduino Nano.
  It controls the infrared light barrier, detects trigger
  and communicates with a master computer (Raspberry Pi)
  over USB serial.

  Copyright 2015 Martin Kompf
  Copyright 2015 Manuel Mühlig for additional changes for supporting mutiple sensors

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/eeprom.h>

#define NUM_SENSORS (2)

// Comment in if a blink led is attached
//#define BLINK_LED

const int analogInPin[NUM_SENSORS] = {A0, A1};  // Analog input pin that the photo transistor is attached to
const int irOutPin[NUM_SENSORS] = {2, 3}; // Digital output pin that the IR-LED is attached to

#ifdef BLINK_LED
const int ledOutPin[NUM_SENSORS] = {12, 13}; // Signal LED output pin
#endif

int sensorValueOff = 0;  // value read from the photo transistor when ir LED is off
int sensorValueOn = 0;  // value read from the photo transistor when ir LED is on
int sensorDiffValue[NUM_SENSORS]; // difference values

// command line
#define MAX_CMD_LEN (80)
char command[MAX_CMD_LEN]; // command buffer
int inCount = 0; // command buffer index
boolean cmdComplete = false;

// Mode of serial line:
// C - command, D - data output
char mode = 'D';

// Data output mode:
// R - raw data (for plotting in Arduino serial plotter)
// T - trigger events
char dataOutput = 'T';

// trigger state and level
int triggerLevelLow[NUM_SENSORS];
int triggerLevelHigh[NUM_SENSORS];
boolean triggerState[NUM_SENSORS] = {false, false};

// Address of trigger levels in EEPROM
uint16_t triggerLevelLowAddr[NUM_SENSORS] = {0, 8};
uint16_t triggerLevelHighAddr[NUM_SENSORS] = {4, 12};

/**
   Set trigger levels (low high) from the command line
   and store them into EEPROM
*/
void setTriggerLevels() {
  // first value is the sensor id
  char *p = &command[1];
  while (*p != 0 && *p == ' ') {
    ++p;
  }
  char *q = p + 1;
  while (*q != 0 && *q != ' ') {
    ++q;
  }
  *q = 0;

  int sensor = atoi(p);
  if (sensor >= NUM_SENSORS) {
    return;
  }

  // low
  p = q + 1;
  while (*p != 0 && *p == ' ') {
    ++p;
  }
  q = p + 1;
  while (*q != 0 && *q != ' ') {
    ++q;
  }
  *q = 0;
  eeprom_write_word(&triggerLevelLowAddr[sensor], atoi(p));

  // high
  p = q + 1;
  while (*p != 0 && *p == ' ') {
    ++p;
  }
  q = p + 1;
  while (*q != 0 && *q != ' ') {
    ++q;
  }
  *q = 0;
  eeprom_write_word(&triggerLevelHighAddr[sensor], atoi(p));
}

/**
   Read trigger levels from EEPROM
*/
void readTriggerLevels(bool quiet) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    triggerLevelLow[i] = (int)eeprom_read_word(&triggerLevelLowAddr[i]);
    triggerLevelHigh[i] = (int)eeprom_read_word(&triggerLevelHighAddr[i]);
    if(!quiet) {
      Serial.print("Trigger levels sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(triggerLevelLow[i]);
      Serial.print(" ");
      Serial.println(triggerLevelHigh[i]);
    }
  }
}

/**
   Detect and print a trigger event
*/
void detectTrigger(int sensor, int val) {
  boolean nextState = triggerState[sensor];
  if (val > triggerLevelHigh[sensor]) {
    nextState = true;
  } else if (val < triggerLevelLow[sensor]) {
    nextState = false;
  }
  if (nextState != triggerState[sensor]) {
    triggerState[sensor] = nextState;
    Serial.print(sensor);
    Serial.print(" ");
    Serial.println(triggerState[sensor] ? 1 : 0);
#ifdef BLINK_LED
    // control internal LED
    digitalWrite(ledOutPin[sensor], triggerState[sensor]);
#endif
  }
}

/**
   Read one line from serial connection and interpret it as a command
*/
void doCommand() {
  // print prompt
  Serial.print(">");
  while (! cmdComplete) {
    // read input
    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();
      if (inChar == '\n' || inChar == '\r') {
        command[inCount] = 0;
        Serial.println();
        cmdComplete = true;
        break; // End of line
      } else if (inCount < MAX_CMD_LEN - 1) {
        command[inCount] = inChar;
        inCount++;
        // echo
        Serial.print(inChar);
      }
    }
  }

  // interprete command
  switch (command[0]) {
    case 'D':
      // start raw data mode
      mode = 'D';
      dataOutput = 'R';
      break;
    case 'T':
      // start trigger data mode
      mode = 'D';
      dataOutput = 'T';
      break;
    case 'S':
      // set trigger levels
      setTriggerLevels();
      readTriggerLevels(false);
      break;
  }

  // clear command buffer
  command[0] = 0;
  inCount = 0;
  cmdComplete = false;
}

void updateSensor(int sensor)
{
  // perform measurement
  // turn IR LED off
  digitalWrite(irOutPin[sensor], LOW);
  // wait 10 milliseconds
  delay(10);
  // read the analog in value:
  sensorValueOff = analogRead(analogInPin[sensor]);
  // turn IR LED on
  digitalWrite(irOutPin[sensor], HIGH);
  delay(10);
  // read the analog in value:
  sensorValueOn = analogRead(analogInPin[sensor]);

  sensorDiffValue[sensor] = sensorValueOn - sensorValueOff;

  if (dataOutput == 'T') {
    // detect and output trigger
    detectTrigger(sensor, sensorDiffValue[sensor]);
  }
}

/**
   Setup.
*/
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  // initialize the digital pins as an output.
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(irOutPin[i], OUTPUT);
#ifdef BLINK_LED
    pinMode(ledOutPin[i], OUTPUT);
#endif
  }


  // read config from EEPROM
  readTriggerLevels(true);
}

/**
   Main loop.
*/
void loop() {
  if (mode == 'C') {
    doCommand();
  } else if (mode == 'D') {
    if (Serial.available()) {
      char inChar = (char)Serial.read();
      if (inChar == 'C') {
        // exit data mode
        mode = 'C';
      }
    }
  }
  if (mode == 'D') {
    for (int i = 0; i < NUM_SENSORS; i++) {
      updateSensor(i);
    }

    // output raw data in serial plotter format
    if (dataOutput == 'R') {
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorDiffValue[i]);
        if (i != NUM_SENSORS - 1) {
          Serial.print(",");
        }
      }
      Serial.println();
    }
  }
  delay(10);
}
