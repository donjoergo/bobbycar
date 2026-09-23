#pragma once

#include <stdint.h>

/**
 * @brief Initializes the I2C bus and the Nunchuk hardware.
 */
void nunchukInputInit();

/**
 * @brief Reads a new data packet from the Nunchuk over I2C.
 *
 * @return true if read was successful.
 */
bool nunchukRead();

/**
 * @brief Returns the state of Nunchuk button C.
 *
 * @return 1 if pressed, 0 if released.
 */
uint8_t nunchukButtonC();

/**
 * @brief Returns the state of Nunchuk button Z.
 *
 * @return 1 if pressed, 0 if released.
 */
uint8_t nunchukButtonZ();

/**
 * @brief Reads Nunchuk Y-axis with fallback to last valid sample and simulation support.
 *
 * @return Raw Y-axis value (0..255).
 */
int getNunchukY();

/**
 * @brief Reads both X and Y joystick axes from Nunchuk (or simulated values).
 *
 * @param xaxis Output reference for raw X axis.
 * @param yaxis Output reference for raw Y axis.
 */
void readNunchukAxes(int &xaxis, int &yaxis);

/**
 * @brief Checks whether the joystick is currently within the center deadband.
 *
 * @param xaxis Raw X axis value.
 * @param yaxis Raw Y axis value.
 * @return true if both axes are inside the center deadband.
 */
bool isJoystickCentered(int xaxis, int yaxis);

/**
 * @brief Maps Nunchuk Y-axis to forward acceleration command signal.
 *
 * @param yaxis Raw joystick Y-axis.
 * @return Normalized command in range NUNCHUK_SIGNAL_MIN..NUNCHUK_SIGNAL_MAX.
 */
unsigned int getAccelerationCommand(int yaxis);

/**
 * @brief Maps Nunchuk Y-axis to reverse/brake command signal.
 *
 * @param yaxis Raw joystick Y-axis.
 * @return Normalized command in range NUNCHUK_SIGNAL_MIN..NUNCHUK_SIGNAL_MAX.
 */
unsigned int getBrakeCommand(int yaxis);
