#include "nunchuk_input.h"
#include "config.h"
#include "nunchuk.h"
#include <Arduino.h>
#include <Wire.h>

#ifndef SIMULATE_NUNCHUK
static bool initialNunchukRead = true;
#endif

void nunchukInputInit() {
#ifndef SIMULATE_NUNCHUK
  Wire.begin();
  nunchuk_init();
#endif
}

bool nunchukRead() {
#ifndef SIMULATE_NUNCHUK
  return nunchuk_read();
#else
  return false;
#endif
}

uint8_t nunchukButtonC() {
#ifndef SIMULATE_NUNCHUK
  return nunchuk_buttonC();
#else
  return 0;
#endif
}

uint8_t nunchukButtonZ() {
#ifndef SIMULATE_NUNCHUK
  return nunchuk_buttonZ();
#else
  return 0;
#endif
}

int getNunchukY() {
  static int lastY = NUNCHUK_JOYSTICK_Y_ZERO;
  int yaxis = lastY;

#ifndef SIMULATE_NUNCHUK
  if (nunchuk_read()) {
    yaxis = nunchuk_joystickY_raw();
    lastY = yaxis;
  }

  // Workaround for unstable first read from the Nunchuk library
  if (initialNunchukRead) {
    if (nunchuk_read()) {
      yaxis = nunchuk_joystickY_raw();
      lastY = yaxis;
      initialNunchukRead = false;
    }
  }
#endif

#ifdef SIMULATE_NUNCHUK
  unsigned long currentMillis = millis();
  if (currentMillis < 4000) {
    yaxis = 255; // Full throttle for first 4 seconds
  } else if (currentMillis < 6000) {
    yaxis = 0; // Full brake for next 2 seconds
  } else {
    yaxis = 127; // Center position afterwards
  }
  lastY = yaxis;
#endif

  return yaxis;
}

void readNunchukAxes(int &xaxis, int &yaxis) {
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

bool isJoystickCentered(int xaxis, int yaxis) {
  return yaxis > JOYSTICK_CENTER_MIN && yaxis < JOYSTICK_CENTER_MAX && xaxis > JOYSTICK_CENTER_MIN &&
         xaxis < JOYSTICK_CENTER_MAX;
}

unsigned int getAccelerationCommand(int yaxis) {
  return map(constrain(yaxis, NUNCHUK_ACC_MIN, NUNCHUK_ACC_MAX), NUNCHUK_ACC_MIN, NUNCHUK_ACC_MAX, NUNCHUK_SIGNAL_MIN,
             NUNCHUK_SIGNAL_MAX);
}

unsigned int getBrakeCommand(int yaxis) {
  return map(constrain(yaxis, NUNCHUK_BRK_MIN, NUNCHUK_BRK_MAX), NUNCHUK_BRK_MAX, NUNCHUK_BRK_MIN, NUNCHUK_SIGNAL_MIN,
             NUNCHUK_SIGNAL_MAX);
}
