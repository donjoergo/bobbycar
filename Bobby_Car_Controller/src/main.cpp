#include <Arduino.h>
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

#define JOYSTICK_CENTER_MIN 120
#define JOYSTICK_CENTER_MAX 136
#define JOYSTICK_EXTREME_LOW 50
#define JOYSTICK_EXTREME_HIGH 200
#define MODE_SELECTION_TIMEOUT_MS 5000
#define NUNCHUK_CENTER_TIMEOUT_LOOPS 30
#define NUNCHUK_POLL_DELAY_MS 50
#define ACC_ACTIVE_THRESHOLD 6
#define BRK_ACTIVE_THRESHOLD 20


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

// Arduino IDE auto-generates these prototypes for .ino tabs.
// In PlatformIO/C++ they need to be declared explicitly.
/**
 * @brief Sends one command frame to the hoverboard controller over UART.
 *
 * @param uSteer Steering command in hoverboard protocol units.
 * @param uSpeed Speed command in hoverboard protocol units.
 */
void sendToHoverboard(int16_t uSteer, int16_t uSpeed);
/**
 * @brief Runs joystick-based drive mode detection and updates global drive mode.
 *
 * @return true if mode selection was successful, false on timeout/error.
 */
bool detectDrivingMode();
/**
 * @brief Emits a short audible beep pattern on the buzzer.
 *
 * @param beeps Number of short beeps to play.
 */
void beepShort(unsigned int beeps);
/**
 * @brief Reads and returns current Nunchuk Y-axis value.
 *
 * Uses the last valid value as fallback if no fresh sample is available.
 *
 * @return Raw Y-axis value in range 0..255.
 */
int getNunchukY();
/**
 * @brief Returns whether joystick coordinates are around center position.
 */
bool isJoystickCentered(int xaxis, int yaxis);
/**
 * @brief Reads latest Nunchuk joystick axes into provided references.
 */
void readNunchukAxes(int& xaxis, int& yaxis);
/**
 * @brief Maps Nunchuk Y-axis to acceleration command signal.
 */
unsigned int getAccelerationCommand(int yaxis);
/**
 * @brief Maps Nunchuk Y-axis to braking/reverse command signal.
 */
unsigned int getBrakeCommand(int yaxis);


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

/**
 * @brief Initializes serial links, Nunchuk interface and default drive mode.
 */
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

/**
 * @brief Main control loop.
 *
 * Reads Nunchuk input, updates target speed with mode-dependent limits and
 * periodically sends speed commands to the hoverboard controller.
 */
void loop() {
  if (driveMode > 0 && driveMode < 5) {
    unsigned long timeNow = millis();

    // Get latest Nunchuk input (Y axis for throttle/brake, buttons for mode switch).
    int iNunchuk = getNunchukY();
    unsigned int iNunchuckC = 0;
    unsigned int iNunchuckZ = 0;
    if (nunchuk_read()) {
      iNunchuckC = nunchuk_buttonC();
      iNunchuckZ = nunchuk_buttonZ();
    }

    if (iNunchuckC && iNunchuckZ) {
      detectDrivingMode();
    }

    unsigned int acc_cmd = getAccelerationCommand(iNunchuk);
    unsigned int brk_cmd = getBrakeCommand(iNunchuk);
    bool ignoreBrk = acc_cmd < acc_cmd_prev;

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
    if ((acc_cmd < ACC_ACTIVE_THRESHOLD && brk_cmd < ACC_ACTIVE_THRESHOLD) || ignoreBrk) {
      forceNunchukRelease = false;
      speed = speed * (1.0 - FREEWHEELING_DECELERATION);  //(speed > 0 ? MODES[i].ACC_FORWARDS / MODES[i].MAX_SPEED_FORWARDS * FREEWHEELING_DECELERATION * 1.0 : MODES[i].ACC_REVERSE / MODES[i].MAX_SPEED_REVERSE * FREEWHEELING_DECELERATION * 1.0));
      // Serial.println("Freewheeling!");
      // Serial.println(speed);

    } else if (acc_cmd > ACC_ACTIVE_THRESHOLD) {
      speed += acc_cmd * MODES[i].ACC_FORWARD * 1.0;  // accelerating forwards
      int maxSpeed = (acc_cmd * 1.0 / (NUNCHUK_SIGNAL_MAX - NUNCHUK_SIGNAL_MIN)) * MODES[i].MAX_SPEED_FORWARDS;
      speed = constrain(speed, -1000, maxSpeed);
    } else if (brk_cmd > BRK_ACTIVE_THRESHOLD) {
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
/**
 * @brief Detects drive mode from joystick position during selection flow.
 *
 * Sequence:
 * 1) Wait for centered joystick (safety / release detection).
 * 2) Wait for one of four direction positions to choose a mode.
 * 3) Wait for joystick release back to center.
 *
 * @return true if a valid mode was selected and stored in `driveMode`.
 */
bool detectDrivingMode() {
  // when entering the function, beep once
  beepShort(1);

  unsigned int tempDriveMode;
  int xaxis = NUNCHUK_JOYSTICK_X_ZERO;
  int yaxis = NUNCHUK_JOYSTICK_Y_ZERO;


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
  while (isJoystickCentered(xaxis, yaxis)) {
    readNunchukAxes(xaxis, yaxis);
    //Serial.println(counter);
    // #ifdef DEBUG_MODE
    //     Serial.println(yaxis);
    // #endif

    if (counter++ > NUNCHUK_CENTER_TIMEOUT_LOOPS) {
#ifdef DEBUG_MODE
      Serial.println(F("Nunchuk is not around middle position for 5 sec. Powering off..."));
      Serial.println(yaxis);
#endif
      beepShort(10);
      return false;
    }
    delay(NUNCHUK_POLL_DELAY_MS);
  }

  bool modeFound = false;
  unsigned long modeSelectionStart = millis();
  while (!modeFound) {
    readNunchukAxes(xaxis, yaxis);
    if (xaxis > JOYSTICK_EXTREME_HIGH && yaxis > JOYSTICK_CENTER_MIN && yaxis < JOYSTICK_CENTER_MAX) {  // joystick right postition
      tempDriveMode = 1;
      modeFound = true;
    } else if (yaxis < JOYSTICK_EXTREME_LOW && xaxis > JOYSTICK_CENTER_MIN && xaxis < JOYSTICK_CENTER_MAX) {  // joystick down postition
      tempDriveMode = 2;
      modeFound = true;
    } else if (xaxis < JOYSTICK_EXTREME_LOW && yaxis > JOYSTICK_CENTER_MIN && yaxis < JOYSTICK_CENTER_MAX) {  // joystick left postition
      tempDriveMode = 3;
      modeFound = true;
    } else if (yaxis > JOYSTICK_EXTREME_HIGH && xaxis > JOYSTICK_CENTER_MIN && xaxis < JOYSTICK_CENTER_MAX) {  // joystick up postition
      tempDriveMode = 4;
      modeFound = true;
    }
    if (millis() - modeSelectionStart > MODE_SELECTION_TIMEOUT_MS) {
#ifdef DEBUG_MODE
      Serial.println(F("No valid drive mode selection within 5 sec. Powering off..."));
#endif
      beepShort(10);
      return false;
    }
    delay(NUNCHUK_POLL_DELAY_MS);
  }

#ifdef DEBUG_MODE
  Serial.println(F("Waiting for nunchuk release..."));
#endif

  counter = 0;
  while (!isJoystickCentered(xaxis, yaxis)) {
    readNunchukAxes(xaxis, yaxis);
    //Serial.println(counter);
    // #ifdef DEBUG_MODE
    //     Serial.println(yaxis);
    // #endif

    if (counter++ > NUNCHUK_CENTER_TIMEOUT_LOOPS) {
#ifdef DEBUG_MODE
      Serial.println(F("Nunchuk is not around middle position for 5 sec. Powering off..."));
      Serial.println(yaxis);
#endif
      beepShort(10);
      return false;
    }
    delay(NUNCHUK_POLL_DELAY_MS);
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

  return true;
}


/**
 * @brief Emits a sequence of short buzzer beeps.
 *
 * @param beeps Number of short beeps.
 */
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


/**
 * @brief Reads Nunchuk Y-axis with a fallback to the latest valid sample.
 *
 * Includes a one-time double-read workaround for unstable first read from
 * the current Nunchuk library.
 *
 * @return Raw joystick Y-axis value.
 */
int getNunchukY() {
  static int lastY = NUNCHUK_JOYSTICK_Y_ZERO;
  int yaxis = lastY;

#ifndef SIMULATE_NUNCHUK
  if (nunchuk_read()) {
    yaxis = nunchuk_joystickY_raw();
    lastY = yaxis;
  }

  // Library Bug Workaround:
  // Read out the Nunchuk again when it was the first read
  if (initialNunchukRead) {
    if (nunchuk_read()) {
      yaxis = nunchuk_joystickY_raw();
      lastY = yaxis;
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
  lastY = yaxis;
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

/**
 * @brief Sends one packet with start frame, payload and checksum.
 *
 * @param uSteer Steering command.
 * @param uSpeed Speed command.
 */
void sendToHoverboard(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write(reinterpret_cast<const uint8_t*>(&Command), sizeof(Command));
}

/**
 * @brief Checks whether joystick is in center deadband.
 *
 * @param xaxis Raw joystick X-axis.
 * @param yaxis Raw joystick Y-axis.
 * @return true when both axes are inside center deadband.
 */
bool isJoystickCentered(int xaxis, int yaxis) {
  return yaxis > JOYSTICK_CENTER_MIN && yaxis < JOYSTICK_CENTER_MAX
      && xaxis > JOYSTICK_CENTER_MIN && xaxis < JOYSTICK_CENTER_MAX;
}

/**
 * @brief Reads joystick axes from Nunchuk (or simulated values).
 *
 * @param xaxis Output: raw X-axis.
 * @param yaxis Output: raw Y-axis.
 */
void readNunchukAxes(int& xaxis, int& yaxis) {
#ifndef SIMULATE_NUNCHUK
  if (nunchuk_read()) {
    xaxis = nunchuk_joystickX_raw();
    yaxis = nunchuk_joystickY_raw();
  }
#endif
#ifdef SIMULATE_NUNCHUK
  xaxis = NUNCHUK_JOYSTICK_X_ZERO;
  yaxis = NUNCHUK_JOYSTICK_Y_ZERO;
#endif
}

/**
 * @brief Converts Nunchuk Y-axis to forward acceleration command.
 *
 * @param yaxis Raw joystick Y-axis.
 * @return Command in range NUNCHUK_SIGNAL_MIN..NUNCHUK_SIGNAL_MAX.
 */
unsigned int getAccelerationCommand(int yaxis) {
  return map(constrain(yaxis, NUNCHUK_ACC_MIN, NUNCHUK_ACC_MAX),
             NUNCHUK_ACC_MIN, NUNCHUK_ACC_MAX,
             NUNCHUK_SIGNAL_MIN, NUNCHUK_SIGNAL_MAX);
}

/**
 * @brief Converts Nunchuk Y-axis to reverse/brake command.
 *
 * @param yaxis Raw joystick Y-axis.
 * @return Command in range NUNCHUK_SIGNAL_MIN..NUNCHUK_SIGNAL_MAX.
 */
unsigned int getBrakeCommand(int yaxis) {
  return map(constrain(yaxis, NUNCHUK_BRK_MIN, NUNCHUK_BRK_MAX),
             NUNCHUK_BRK_MAX, NUNCHUK_BRK_MIN,
             NUNCHUK_SIGNAL_MIN, NUNCHUK_SIGNAL_MAX);
}
