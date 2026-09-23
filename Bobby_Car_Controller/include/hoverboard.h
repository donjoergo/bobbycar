#pragma once

#include <stdint.h>

// Command packet structure expected by the hoverboard motor controller firmware
typedef struct {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;

/**
 * @brief Initializes USART / SoftwareSerial communication with the hoverboard.
 */
void hoverboardInit();

/**
 * @brief Sends one command packet with start frame, payload, and XOR checksum.
 *
 * @param uSteer Steering command (-1000..1000).
 * @param uSpeed Speed command (-1000..1000).
 */
void sendToHoverboard(int16_t uSteer, int16_t uSpeed);
