#include "hoverboard.h"
#include "config.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

// Module-local SoftwareSerial instance and command buffer
static SoftwareSerial HoverSerial(RX_PIN, TX_PIN);
static SerialCommand Command;

void hoverboardInit() { HoverSerial.begin(HOVER_SERIAL_BAUD); }

void sendToHoverboard(int16_t uSteer, int16_t uSpeed) {
  Command.start = START_FRAME;
  Command.steer = uSteer;
  Command.speed = uSpeed;
  Command.checksum = static_cast<uint16_t>(Command.start ^ Command.steer ^ Command.speed);

  HoverSerial.write(reinterpret_cast<const uint8_t *>(&Command), sizeof(Command));
}
