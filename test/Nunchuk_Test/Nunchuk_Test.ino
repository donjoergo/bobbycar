/**
 * Tests the Nunchuk controller
 */

#include "nunchuk.h"
#include <Wire.h>
void setup() {
  Serial.begin(115200);
  Wire.begin();
  // nunchuk_init_power(); // A1 and A2 is power supply
  nunchuk_init();
}
void loop() {
  if (nunchuk_read()) {
    // Work with nunchuk_data
    // nunchuk_print();
    Serial.println(nunchuk_joystickY_raw());
    Serial.println(nunchuk_joystickY());
    // Serial.println(nunchuk_joystickY_raw() / nunchuk_joystickY());
  }

  unsigned int iNunchuk = getNunchukData();
  unsigned int acc_cmd = map(
      constrain(iNunchuk, NUNCHUK_ACC_MIN, NUNCHUK_ACC_MAX), NUNCHUK_ACC_MIN,
      NUNCHUK_ACC_MAX, NUNCHUK_SIGNAL_MIN, NUNCHUK_SIGNAL_MAX);
  unsigned int brk_cmd = map(
      constrain(iNunchuk, NUNCHUK_BRK_MIN, NUNCHUK_BRK_MAX), NUNCHUK_BRK_MAX,
      NUNCHUK_BRK_MIN, NUNCHUK_SIGNAL_MIN, NUNCHUK_SIGNAL_MAX);

  delay(100);
}
