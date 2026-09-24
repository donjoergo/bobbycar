#pragma once

#include <Arduino.h>

// ########################## BUILD & DEBUG FLAGS ##########################
#define VERSION "0.3 alpha - Macherfestival Edition"

// Debugging mode (comment out to disable debug messages on built-in Serial)
#define DEBUG_MODE

// Simulation mode (uncomment to test without physical Nunchuk hardware)
// #define SIMULATE_NUNCHUK 1

// ########################## HARDWARE PINOUT ##########################
// Arduino digital pin connected to Hoverboard TX (USART receive)
constexpr uint8_t RX_PIN = 3;

// Arduino digital pin connected to Hoverboard RX (USART transmit)
constexpr uint8_t TX_PIN = 2;

// Arduino digital pin connected to active piezo buzzer
constexpr uint8_t BUZZER_PIN = 6;

// ########################## SERIAL BAUD RATES ##########################
#ifdef DEBUG_MODE
// Baud rate for USB Serial debugging output (matches Serial Monitor setting)
constexpr unsigned long DEBUG_SERIAL_BAUD = 115200;
#endif

// Baud rate for communication with the hoverboard motor controller
constexpr unsigned long HOVER_SERIAL_BAUD = 115200;

// ########################## HOVERBOARD PROTOCOL ##########################
// 2-byte synchronization header marking the start of every hoverboard packet
constexpr uint16_t START_FRAME = 0xABCD;

// Transmission interval in milliseconds (~10 Hz update rate to motor controller)
constexpr unsigned long TIME_SEND = 100;

// ########################## NUNCHUK JOYSTICK CALIBRATION ##########################
// Raw Y-axis threshold where forward throttle begins (just above center deadband)
constexpr int NUNCHUK_ACC_MIN = 129;

// Raw Y-axis value corresponding to 100% full forward throttle
constexpr int NUNCHUK_ACC_MAX = 245;

// Raw Y-axis value corresponding to 100% full brake / reverse throttle
constexpr int NUNCHUK_BRK_MIN = 0;

// Raw Y-axis threshold where braking/reverse begins (just below center deadband)
constexpr int NUNCHUK_BRK_MAX = 127;

// Normalized command output minimum (0% throttle/brake command)
constexpr int NUNCHUK_SIGNAL_MIN = 0;

// Normalized command output maximum (100% throttle/brake command)
constexpr int NUNCHUK_SIGNAL_MAX = 255;

// Expected factory resting/center position for Nunchuk X-axis
constexpr int NUNCHUK_JOYSTICK_X_ZERO = 127;

// Expected factory resting/center position for Nunchuk Y-axis
constexpr int NUNCHUK_JOYSTICK_Y_ZERO = 128;

// Lower bound of the joystick center deadband window (ignore micro-movements)
constexpr int JOYSTICK_CENTER_MIN = 120;

// Upper bound of the joystick center deadband window (ignore micro-movements)
constexpr int JOYSTICK_CENTER_MAX = 136;

// Low deflection threshold for detecting directional gestures (Down on Y, Left on X)
constexpr int JOYSTICK_EXTREME_LOW = 50;

// High deflection threshold for detecting directional gestures (Up on Y, Right on X)
constexpr int JOYSTICK_EXTREME_HIGH = 200;

// ########################## TIMING & DELAYS ##########################
// Maximum time in milliseconds to wait for drive mode selection before aborting
constexpr unsigned long MODE_SELECTION_TIMEOUT_MS = 5000;

// Number of consecutive loops joystick must stay centered during safety verification
constexpr int NUNCHUK_CENTER_TIMEOUT_LOOPS = 30;

// Polling delay in milliseconds between consecutive joystick reads during mode selection
constexpr unsigned long NUNCHUK_POLL_DELAY_MS = 50;

// ########################## VEHICLE DYNAMICS ##########################
// Percentage speed reduction per cycle when coasting without throttle or brake
constexpr float FREEWHEELING_DECELERATION = 0.0005f;

// Minimum normalized throttle signal (0..255) required to apply forward drive
constexpr unsigned int ACC_ACTIVE_THRESHOLD = 6;

// Minimum normalized brake signal (0..255) required to trigger braking/reverse drive
constexpr unsigned int BRK_ACTIVE_THRESHOLD = 20;

// ########################## DRIVING MODES ##########################
struct ModeParameters {
  unsigned int MAX_SPEED_FORWARDS; // Maximum forward speed limit in hoverboard protocol units
  unsigned int MAX_SPEED_REVERSE;  // Maximum reverse speed limit in hoverboard protocol units
  float ACC_FORWARD;               // Forward acceleration rate (speed added per loop cycle)
  float ACC_REVERSE;               // Reverse acceleration rate (speed added per loop cycle)
};

// Driving mode selected at boot (1 = Gentle, 2 = Standard, 3 = Sport, 4 = Turbo)
constexpr unsigned int DEFAULT_DRIVE_MODE = 2;

// Driving mode configurations:
// Formula: speed [km/h] = rpm / 31.45  <-->  rpm = speed [km/h] * 31.45
// - Mode 1 (Gentle):    ~4 km/h max, soft acceleration for beginners
// - Mode 2 (Standard):  ~10 km/h max, balanced daily driving speed (boot default)
// - Mode 3 (Sport):     ~17 km/h max, higher top speed with controlled ramp-up
// - Mode 4 (Turbo):     ~17+ km/h max, direct & aggressive throttle response
const ModeParameters MODES[4] = {
    {200, 260, 0.01f, 0.01f},  // Mode 1: ~4 km/h (beginner)
    {400, 260, 0.01f, 0.01f},  // Mode 2: ~10 km/h (standard default)
    {970, 360, 0.005f, 0.02f}, // Mode 3: ~17 km/h (sport)
    {1000, 500, 2.0f, 0.05f}   // Mode 4: Fast/open acceleration
};
