#include "config.h"
#include "hoverboard.h"
#include "nunchuk_input.h"
#include <Arduino.h>

// ########################## GLOBAL STATE ##########################
unsigned long iTimeSend = 0;
float speed = 0.0f;
unsigned int driveMode = DEFAULT_DRIVE_MODE; // Default driving mode (Modes 1 to 4 available)
bool forceNunchukRelease = false;

// Function prototypes
bool detectDrivingMode();
void beepShort(unsigned int beeps);

// ########################## SETUP ##########################
/**
 * @brief Initializes serial links, Nunchuk interface, and default drive mode.
 */
void setup() {
  hoverboardInit(); // Start USART communication for motor controller

#ifdef DEBUG_MODE
  Serial.begin(DEBUG_SERIAL_BAUD); // Start serial monitor for debugging
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

  nunchukInputInit(); // Initialize I2C and Nunchuk hardware

  // Interactive drive mode selection at boot (currently bypassed; driveMode defaults to DEFAULT_DRIVE_MODE)
  // delay(1000);
  // detectDrivingMode();
  beepShort(driveMode);
}

// ########################## MAIN LOOP ##########################
/**
 * @brief Main control loop.
 *
 * Reads Nunchuk inputs, computes vehicle speed based on the active drive mode,
 * handles safety interlocks, and periodically transmits command packets to the hoverboard.
 */
void loop() {
  if (driveMode > 0 && driveMode < 5) {
    unsigned long timeNow = millis();

    // 1. Read Nunchuk inputs
    int iNunchuk = getNunchukY();
    unsigned int iNunchukC = 0;
    unsigned int iNunchukZ = 0;

    if (nunchukRead()) {
      iNunchukC = nunchukButtonC();
      iNunchukZ = nunchukButtonZ();
    }

    // Trigger interactive drive mode selection if both C and Z buttons are pressed
    if (iNunchukC && iNunchukZ) {
      detectDrivingMode();
    }

    // 2. Decode throttle and brake commands
    unsigned int acc_cmd = getAccelerationCommand(iNunchuk);
    unsigned int brk_cmd = getBrakeCommand(iNunchuk);

    int i = driveMode - 1;

    // Require joystick centering before allowing reverse if forward motion was detected
    if (speed > 1.0f) {
      forceNunchukRelease = true;
    }

    // 3. Speed state machine: Freewheeling vs. Accelerating vs. Braking/Reverse
    if (acc_cmd < ACC_ACTIVE_THRESHOLD && brk_cmd < ACC_ACTIVE_THRESHOLD) {
      // Natural freewheeling deceleration
      forceNunchukRelease = false;
      speed = speed * (1.0f - FREEWHEELING_DECELERATION);
    } else if (acc_cmd > ACC_ACTIVE_THRESHOLD) {
      // Throttle position defines the new set speed
      int setSpeed = (acc_cmd * 1.0f / (NUNCHUK_SIGNAL_MAX - NUNCHUK_SIGNAL_MIN)) * MODES[i].MAX_SPEED_FORWARDS;
      if (speed > setSpeed) {
        // Coast down to the new set speed instead of applying it instantly
        speed = speed * (1.0f - FREEWHEELING_DECELERATION);
      } else {
        // Forward acceleration
        speed += acc_cmd * MODES[i].ACC_FORWARD * 1.0f;
        speed = constrain(speed, -1000.0f, setSpeed);
      }
    } else if (brk_cmd > BRK_ACTIVE_THRESHOLD) {
      // Deceleration / reverse acceleration with safety lockout
      speed -= brk_cmd * MODES[i].ACC_REVERSE * 1.0f;
      int minSpeed = (brk_cmd * 1.0f / (NUNCHUK_SIGNAL_MAX - NUNCHUK_SIGNAL_MIN)) * MODES[i].MAX_SPEED_REVERSE;
      speed = constrain(speed, forceNunchukRelease ? 0.0f : -minSpeed, 1000.0f);
    }

    // 4. Send command packet to hoverboard at configured interval
    if (iTimeSend > timeNow) {
      return;
    }
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

// ########################## HELPER FUNCTIONS ##########################
/**
 * @brief Detects drive mode from joystick deflection.
 *
 * Selection mapping:
 * - Joystick RIGHT: Mode 1 (~4 km/h, beginner)
 * - Joystick DOWN:  Mode 2 (~10 km/h, standard)
 * - Joystick LEFT:  Mode 3 (~17 km/h, sport)
 * - Joystick UP:    Mode 4 (turbo / fast acceleration)
 *
 * @return true if mode was successfully selected, false on timeout/error.
 */
bool detectDrivingMode() {
  beepShort(1);

  unsigned int tempDriveMode = driveMode;
  int xaxis = NUNCHUK_JOYSTICK_X_ZERO;
  int yaxis = NUNCHUK_JOYSTICK_Y_ZERO;

#ifdef SIMULATE_NUNCHUK
  yaxis = 255;
#endif

#ifdef DEBUG_MODE
  Serial.println(F("Waiting for drivemode selection..."));
#endif

  // Step 1: Wait for joystick to be in center position
  int counter = 0;
  while (isJoystickCentered(xaxis, yaxis)) {
    readNunchukAxes(xaxis, yaxis);
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

  // Step 2: Read directional deflection to select mode
  bool modeFound = false;
  unsigned long modeSelectionStart = millis();
  while (!modeFound) {
    readNunchukAxes(xaxis, yaxis);
    if (xaxis > JOYSTICK_EXTREME_HIGH && yaxis > JOYSTICK_CENTER_MIN && yaxis < JOYSTICK_CENTER_MAX) {
      tempDriveMode = 1; // Joystick RIGHT
      modeFound = true;
    } else if (yaxis < JOYSTICK_EXTREME_LOW && xaxis > JOYSTICK_CENTER_MIN && xaxis < JOYSTICK_CENTER_MAX) {
      tempDriveMode = 2; // Joystick DOWN
      modeFound = true;
    } else if (xaxis < JOYSTICK_EXTREME_LOW && yaxis > JOYSTICK_CENTER_MIN && yaxis < JOYSTICK_CENTER_MAX) {
      tempDriveMode = 3; // Joystick LEFT
      modeFound = true;
    } else if (yaxis > JOYSTICK_EXTREME_HIGH && xaxis > JOYSTICK_CENTER_MIN && xaxis < JOYSTICK_CENTER_MAX) {
      tempDriveMode = 4; // Joystick UP
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

  // Step 3: Wait for joystick to return to center before continuing
#ifdef DEBUG_MODE
  Serial.println(F("Waiting for nunchuk release..."));
#endif

  counter = 0;
  while (!isJoystickCentered(xaxis, yaxis)) {
    readNunchukAxes(xaxis, yaxis);
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

  // Success: activate mode and signal with buzzer beeps
  driveMode = tempDriveMode;
  beepShort(driveMode);

#ifdef DEBUG_MODE
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
  for (unsigned int i = 0; i < beeps; i++) {
    tone(BUZZER_PIN, 500);
    delay(100);
    noTone(BUZZER_PIN);
    delay(100);
  }
}
