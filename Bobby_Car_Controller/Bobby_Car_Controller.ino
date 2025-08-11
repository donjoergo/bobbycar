#include <SoftwareSerial.h>
#include <Wire.h>
#include "nunchuk.h"
//#include <WiiChuck.h>
// Accessory nunchuk1;

// ########################## DEFINES ##########################
// General
#define VERSION "0.3 alpha - Macherfestival Edition"

// Debugging mode (enable by uncommenting the line below)
#define DEBUG_MODE  // Comment out to disable debug messages
#ifdef DEBUG_MODE
#define DEBUG_SERIAL_BAUD 115200  // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#endif
// #define SIMULATE_NUNCHUK 1


#define HOVER_SERIAL_BAUD 115200  // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME 0xABCD        // [-] Start frme definition for reliable serial communication
#define TIME_SEND 100             // [ms] Sending time interval

// Pins for hoverboard USART communication (if using SoftwareSerial)
#define RX_PIN 3
#define TX_PIN 2

// BUZZER
#define BUZZER_PIN 6

// Joystick range definitions
#define NUNCHUK_ACC_MIN 129     //
#define NUNCHUK_ACC_MAX 245     //
#define NUNCHUK_BRK_MIN 0       //
#define NUNCHUK_BRK_MAX 127     //
#define NUNCHUK_SIGNAL_MIN 0    //
#define NUNCHUK_SIGNAL_MAX 255  //
bool initialNunchukRead = true;
unsigned int acc_cmd_prev;

struct modeParameters {
  unsigned int MAX_SPEED_FORWARDS;
  unsigned int MAX_SPEED_REVERSE;
  float ACC_FORWARD;
  float ACC_REVERSE;
};

// Speed range definitions
// Here you can tune the speeds and accelerations of the 3 modes:
// Calculate as follows:
// spd = rpm / 31.45
// rpm = spd * 31.45
const modeParameters MODES[4] = {
  { 200, 260, 0.01, 0.01 },   // 4 km/h
  { 400, 260, 0.01, 0.01 },   // 10 km/h
  { 970, 360, 0.005, 0.02 },  // 17 km/h
  { 1000, 500, 2, 0.05 }      // 17 km/h
};

#define FREEWHEELING_DECELERATION 0.0005



// ########################## VARIABLES ##########################
// Loop Variables
unsigned long iTimeSend = 0;
float speed = 0;
unsigned int driveMode = 0;  // Driving Mode. Modes 1-3 possible
bool forceNunchukRelease = false;

// Motorcontroller USART
SoftwareSerial HoverSerial(RX_PIN, TX_PIN);  // RX, TX pins for USART communication

// Command structure (based on the hoverboard packet structure)
typedef struct {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;


// Feedback structure (based on the hoverboard packet structure)
// struct HoverboardFeedback {
//   int16_t cmd1;
//   int16_t cmd2;
//   int16_t speedLeft;
//   int16_t speedRight;
//   int16_t currentLeft;
//   int16_t currentRight;
//   int16_t voltage;
//   int16_t temperature;
//   int16_t cmdLed;
// };
// HoverboardFeedback feedback;  // Create an instance of the feedback structure

void setup() {
  HoverSerial.begin(HOVER_SERIAL_BAUD);  // Start USART for motor controller

#ifdef DEBUG_MODE
  Serial.begin(DEBUG_SERIAL_BAUD);  // Start serial monitor for debugging if debug mode is enabled
  Serial.println(F("=================================================="));
  Serial.println(F("Bobby Car Controller by @donjoergo"));
  Serial.print(F("Version: "));
  Serial.println(F(VERSION));
  Serial.print(F("Build Date: "));
  Serial.println(F(__DATE__));
  Serial.println(F("[!] Debugging mode enabled"));
  Serial.println(F("=================================================="));
  Serial.println(F(""));
#endif

// Initialize nunchuk
#ifndef SIMULATE_NUNCHUK
  // nunchuk1.begin();
  // if (nunchuk1.type == Unknown) {
  //   nunchuk1.type = NUNCHUCK;
  // }
  Wire.begin();
  nunchuk_init();
#endif

  //delay(1000);
  //detectDrivingMode();
  driveMode = 2;
  beepShort(2);
}

void loop() {
  if (driveMode > 0 && driveMode < 5) {
    unsigned long timeNow = millis();

    // Get Nunchuk input
    unsigned int iNunchuk = getNunchukY();
    unsigned int iNunchuckC;
    unsigned int iNunchuckZ;
    if (nunchuk_read()) {
      iNunchuckC = nunchuk_buttonC();
      iNunchuckZ = nunchuk_buttonZ();
    }

    if (iNunchuckC && iNunchuckZ) {
      detectDrivingMode();
    }



    unsigned int acc_cmd = map(constrain(iNunchuk, NUNCHUK_ACC_MIN, NUNCHUK_ACC_MAX), NUNCHUK_ACC_MIN, NUNCHUK_ACC_MAX, NUNCHUK_SIGNAL_MIN, NUNCHUK_SIGNAL_MAX);
    unsigned int brk_cmd = map(constrain(iNunchuk, NUNCHUK_BRK_MIN, NUNCHUK_BRK_MAX), NUNCHUK_BRK_MAX, NUNCHUK_BRK_MIN, NUNCHUK_SIGNAL_MIN, NUNCHUK_SIGNAL_MAX);
    bool ignoreBrk = acc_cmd < acc_cmd_prev ? true : false;

    acc_cmd_prev = acc_cmd;

    // Serial.println(iNunchuk);
    // Serial.println(acc_cmd);
    // Serial.println(brk_cmd);

    // Calculate speed depending on drive mode
    int i = driveMode - 1;

    // TODO Calculate speed from inputted kmh value
    // TODO Make a nunchuk poti release required when going from forward in reverse

    if (speed > 1) {
      forceNunchukRelease = true;
    }

    // Serial.println(acc_cmd);
    // Serial.println(brk_cmd);
    if ((acc_cmd < 6 && brk_cmd < 6) || ignoreBrk) {
      forceNunchukRelease = false;
      speed = speed * (1.0 - FREEWHEELING_DECELERATION);  //(speed > 0 ? MODES[i].ACC_FORWARDS / MODES[i].MAX_SPEED_FORWARDS * FREEWHEELING_DECELERATION * 1.0 : MODES[i].ACC_REVERSE / MODES[i].MAX_SPEED_REVERSE * FREEWHEELING_DECELERATION * 1.0));
      // Serial.println("Freewheeling!");
      // Serial.println(speed);

    } else if (acc_cmd > 6) {
      speed += acc_cmd * MODES[i].ACC_FORWARD * 1.0;  // accelerating forwards
      int maxSpeed = (acc_cmd * 1.0 / (NUNCHUK_SIGNAL_MAX - NUNCHUK_SIGNAL_MIN)) * MODES[i].MAX_SPEED_FORWARDS;
      speed = constrain(speed, -1000, maxSpeed);
    } else if (brk_cmd > 20) {
      speed -= brk_cmd * MODES[i].ACC_REVERSE * 1.0;  // accelerating backwards
      int minSpeed = (brk_cmd * 1.0 / (NUNCHUK_SIGNAL_MAX - NUNCHUK_SIGNAL_MIN)) * MODES[i].MAX_SPEED_REVERSE;
      speed = constrain(speed, forceNunchukRelease ? 0 : -minSpeed, 1000);
      //Serial.println("Braking!");
    }

    // Serial.print("Speed: ");
    // Serial.println(speed);

    // Send commands
    if (iTimeSend > timeNow) return;
    iTimeSend = timeNow + TIME_SEND;
    sendToHoverboard(0, round(speed));
  } else {
#ifdef DEBUG_MODE
    Serial.println(F("[!] Drive Mode has invalid state!"));
    Serial.println(F("[!] Power on and off to try again"));
#endif
    delay(3000);
  }
}

// Move the Nunchuk joystick to the following positions while poweron:
// Drive Mode 1, down:      3 kmh, no Turbo
// Drive Mode 2, default:  10 kmh, no Turbo
// Drive Mode 3, up:       17 kmh, no Turbo
int detectDrivingMode() {
  // when entering the function, beep once
  beepShort(1);

  unsigned int tempDriveMode;
  int xaxis;
  int yaxis;


#ifndef SIMULATE_NUNCHUK

  //nunchuk1.readData();  // Read inputs and update maps
  //yaxis = nunchuk1.values[1];
#endif
#ifdef SIMULATE_NUNCHUK
  yaxis = 255;
#endif

#ifdef DEBUG_MODE
  Serial.println(F("Waiting for drivemode selection..."));
#endif

  int counter = 0;
  while ((yaxis > 120 && yaxis < 136 && xaxis > 120 && xaxis < 136)) {
#ifndef SIMULATE_NUNCHUK
    if (nunchuk_read()) {
      xaxis = nunchuk_joystickX_raw();
      yaxis = nunchuk_joystickY_raw();
    }
    // nunchuk1.readData();  // Read inputs and update maps
    //int yaxis = nunchuk1.values[1];
#endif
#ifdef SIMULATE_NUNCHUK
    yaxis = 128;
#endif
    //Serial.println(counter);
    // #ifdef DEBUG_MODE
    //     Serial.println(yaxis);
    // #endif

    if (counter++ > 30) {
#ifdef DEBUG_MODE
      Serial.println(F("Nunchuk is not around middle position for 5 sec. Powering off..."));
      Serial.println(yaxis);
#endif
      beepShort(10);
      return;
    }
    delay(50);
  }

  bool modeFound = false;
  while (!modeFound) {
    if (nunchuk_read()) {
      xaxis = nunchuk_joystickX_raw();
      yaxis = nunchuk_joystickY_raw();
    }
    if (xaxis > 200 && yaxis > 120 && yaxis < 136) {  // joystick right postition
      tempDriveMode = 1;
      modeFound = true;
    } else if (yaxis < 50 && xaxis > 120 && xaxis < 136) {  // joystick down postition
      tempDriveMode = 2;
      modeFound = true;
    } else if (xaxis < 50 && yaxis > 120 && yaxis < 136) {  // joystick left postition
      tempDriveMode = 3;
      modeFound = true;
    } else if (yaxis > 200 && xaxis > 120 && xaxis < 136) {  // joystick up postition
      tempDriveMode = 4;
      modeFound = true;
    }
  }

#ifdef DEBUG_MODE
  Serial.println(F("Waiting for nunchuk release..."));
#endif

  counter = 0;
  while (!(yaxis > 120 && yaxis < 136 && xaxis > 120 && xaxis < 136)) {
#ifndef SIMULATE_NUNCHUK
    if (nunchuk_read()) {
      xaxis = nunchuk_joystickX_raw();
      yaxis = nunchuk_joystickY_raw();
    }
    // nunchuk1.readData();  // Read inputs and update maps
    //int yaxis = nunchuk1.values[1];
#endif
#ifdef SIMULATE_NUNCHUK
    yaxis = 128;
#endif
    //Serial.println(counter);
    // #ifdef DEBUG_MODE
    //     Serial.println(yaxis);
    // #endif

    if (counter++ > 30) {
#ifdef DEBUG_MODE
      Serial.println(F("Nunchuk is not around middle position for 5 sec. Powering off..."));
      Serial.println(yaxis);
#endif
      beepShort(10);
      return;
    }
    delay(50);
  }

  // Success!
  // Serial.print("Success. Mode: ");
  // Serial.println(tempDriveMode);
  driveMode = tempDriveMode;

  // Beep the buzzer to inidicate drive mode
  beepShort(driveMode);

#ifdef DEBUG_MODE
  // Serial.println(F("Nunchuk released"));
  Serial.println(F("Driving Mode detection done"));
  Serial.print(F("[!] Drive Mode: "));
  Serial.println(driveMode);

  Serial.print(F("[!] Max Speed Fwd: "));
  Serial.print(MODES[driveMode - 1].MAX_SPEED_FORWARDS);
  Serial.print(F(", Max Speed Rev: "));
  Serial.print(MODES[driveMode - 1].MAX_SPEED_REVERSE);
  Serial.print(F(", Acc Fwd: "));
  Serial.print(MODES[driveMode - 1].ACC_FORWARD);
  Serial.print(F(", Acc Rev: "));
  Serial.println(MODES[driveMode - 1].ACC_REVERSE);
#endif
}


void beepShort(unsigned int beeps) {
  for (int i = 0; i < beeps; i++) {
    tone(BUZZER_PIN, 500);
    delay(100);
    noTone(BUZZER_PIN);
    delay(100);
#ifdef DEBUG_MODE
    Serial.println(F("Beep"));
#endif
  }
}


int getNunchukY() {
  int yaxis;

#ifndef SIMULATE_NUNCHUK
  if (nunchuk_read()) {
    yaxis = nunchuk_joystickY_raw();
  }

  // Library Bug Workaround:
  // Read out the Nunchuk again when it was the first read
  if (initialNunchukRead) {
    if (nunchuk_read()) {
      yaxis = nunchuk_joystickY_raw();
      initialNunchukRead = false;
    }
  }


  // nunchuk1.readData();  // Read inputs and update maps
  // yaxis = nunchuk1.values[1];

  // Serial.print("Before: ");
  // Serial.println(yaxis);
  // // Manipulate yaxis because nunchuk delivers the values with a offset...
  // int MAGIC_NUMBER = 27;

  // if (yaxis > MAGIC_NUMBER) {
  //   yaxis -= MAGIC_NUMBER;
  // }
  // else {
  //   yaxis = yaxis + 255 - MAGIC_NUMBER;
  // }
  // Serial.print("After : ");
  // Serial.println(yaxis);

#endif
#ifdef SIMULATE_NUNCHUK
  if (millis() < 4000) {
    yaxis = 255;
  } else if (millis() < 4000) {
    yaxis = 127;
  } else if (millis() < 5000) {
    yaxis = 127;
  } else if (millis() < 6000) {
    yaxis = 0;
  } else if (millis() > 8000) {
    yaxis = 127;
  }
#endif

  // #ifdef DEBUG_MODE
  //   Serial.println(F("-------------------------------------------"));
  //   Serial.println(nunchuk1.values[0]);  // X-Axis
  //   Serial.println(nunchuk1.values[1]);  // Y-Axis
  //   Serial.println(nunchuk1.values[10]); // Z-Button
  //   Serial.println(nunchuk1.values[11]); // C-Button
  // #endif

  return yaxis;
}
