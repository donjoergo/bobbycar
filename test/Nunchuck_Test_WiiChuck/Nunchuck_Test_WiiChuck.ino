/**
 * Tests the nunchuck controller with the WiiChuck library.
 */

#include <WiiChuck.h>
Accessory nunchuck1;

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD                                                      \
  115200 // [-] Baud rate for HoverSerial (used to communicate with the
         // hoverboard)
#define SERIAL_BAUD                                                            \
  115200 // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME                                                            \
  0xABCD // [-] Start frme definition for reliable serial communication
#define TIME_SEND 100      // [ms] Sending time interval
#define SPEED_MAX_TEST 300 // [-] Maximum speed for testing
#define SPEED_STEP 20      // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints
// all bytes to serial (comment-out to disable)

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(3, 2); // RX, TX

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

void setup() {
  Serial.begin(SERIAL_BAUD);

  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);

  nunchuck1.begin();
  if (nunchuck1.type == Unknown) {
    /** If the device isn't auto-detected, set the type explicatly
     * 	NUNCHUCK,
     WIICLASSIC,
     GuitarHeroController,
     GuitarHeroWorldTourDrums,
     DrumController,
     DrawsomeTablet,
     Turntable
     */
    nunchuck1.type = NUNCHUCK;
  }
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *)&Command, sizeof(Command));
}

// // ########################## RECEIVE ##########################
// void Receive()
// {
//     // Check for new data availability in the Serial buffer
//     if (HoverSerial.available()) {
//         incomingByte 	  = HoverSerial.read(); // Read the incoming
//         byte bufStartFrame	= ((uint16_t)(incomingByte) << 8) |
//         incomingBytePrev;       // Construct the start frame
//     }
//     else {
//         return;
//     }

//   // If DEBUG_RX is defined print all incoming bytes
//   #ifdef DEBUG_RX
//         Serial.print(incomingByte);
//         return;
//     #endif

//     // Copy received data
//     if (bufStartFrame == START_FRAME) {	                    //
//     Initialize if new data is detected
//         p       = (byte *)&NewFeedback;
//         *p++    = incomingBytePrev;
//         *p++    = incomingByte;
//         idx     = 2;
//     } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new
//     received data
//         *p++    = incomingByte;
//         idx++;
//     }

//     // Check if we reached the end of the package
//     if (idx == sizeof(SerialFeedback)) {
//         uint16_t checksum;
//         checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^
//         NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
//                             ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp
//                             ^ NewFeedback.cmdLed);

//         // Check validity of the new data
//         if (NewFeedback.start == START_FRAME && checksum ==
//         NewFeedback.checksum) {
//             // Copy the new data
//             memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

//             // Print data to built-in Serial
//             Serial.print("1: ");   Serial.print(Feedback.cmd1);
//             Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
//             Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
//             Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
//             Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
//             Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
//             Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
//         } else {
//           Serial.println("Non-valid data skipped");
//         }
//         idx = 0;    // Reset the index (it prevents to enter in this if
//         condition in the next cycle)
//     }

//     // Update previous states
//     incomingBytePrev = incomingByte;
// }

int getNunchuckData() {
  Serial.println("-------------------------------------------");
  nunchuck1.readData(); // Read inputs and update maps

  int yaxis = nunchuck1.values[1];

  Serial.println(nunchuck1.values[0]); // X-Axis
  Serial.println(nunchuck1.values[1]); // Y-Axis

  Serial.println(nunchuck1.values[10]); // Z-Button
  Serial.println(nunchuck1.values[11]); // C-Button

  return map(yaxis, 0, 255, -1000, 1000);
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;

void loop(void) {
  unsigned long timeNow = millis();
  // Check for new received data
  // Receive();

  // Get Nunchuck input
  iTest = getNunchuckData();
  if (nunchuck1.values[10] == 255) {
    iTest = 1000;
  }

  // Send commands
  if (iTimeSend > timeNow)
    return;
  iTimeSend = timeNow + TIME_SEND;
  Send(0, iTest);

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
}
