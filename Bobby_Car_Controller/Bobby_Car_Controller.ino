#include <SoftwareSerial.h>

// Pin for analog X joystick (Nunchuck)
#define JOY_X_PIN A0  // Analog pin for X-axis

// Joystick range definitions
#define JOYSTICK_MIN 0     // Minimum analog value for joystick
#define JOYSTICK_MAX 1023  // Maximum analog value for joystick

// Speed range definitions
#define SPEED_MIN -1000  // Minimum speed (backward)
#define SPEED_MAX 1000   // Maximum speed (forward)

// Pins for hoverboard USART communication (if using SoftwareSerial)
#define RX_PIN 10
#define TX_PIN 11

// Debugging mode (enable by uncommenting the line below)
#define DEBUG_MODE  // Comment out to disable debug messages

// Motorcontroller USART
SoftwareSerial hoverboardSerial(RX_PIN, TX_PIN);  // RX, TX pins for USART communication

// Feedback structure (based on the hoverboard packet structure)
struct HoverboardFeedback {
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedLeft;
  int16_t speedRight;
  int16_t currentLeft;
  int16_t currentRight;
  int16_t voltage;
  int16_t temperature;
  int16_t cmdLed;
};

HoverboardFeedback feedback;  // Create an instance of the feedback structure

void setup() {
  // Start USART communication
  hoverboardSerial.begin(115200);  // Start USART for motor controller

  #ifdef DEBUG_MODE
    Serial.begin(9600);  // Start serial monitor for debugging if debug mode is enabled
    Serial.println("Debugging mode enabled");
  #endif

  // Initialize analog input for the joystick (Nunchuck)
  pinMode(JOY_X_PIN, INPUT);
}

void loop() {
  // Read and map the joystick value to the speed range
  int speed = readJoystick();

  // Send the speed value to the hoverboard via USART
  sendToHoverboard(speed);

  // Read and process feedback from the hoverboard
  readHoverboardFeedback();

  delay(100);  // Delay for smoother communication
}

// Function to read and map the joystick value to the speed range
int readJoystick() {
  // Read joystick (Nunchuck) X-axis value from the analog pin
  int joyX = analogRead(JOY_X_PIN);

  // Map the joystick X-axis value to the speed range (-1000 to 1000)
  int speed = map(joyX, JOYSTICK_MIN, JOYSTICK_MAX, SPEED_MIN, SPEED_MAX);

  return speed;  // Return the mapped speed value
}

// Function to send speed command to the motor controller
void sendToHoverboard(int speed) {
  // Create a data packet (adjust to match motor controller protocol)
  byte buffer[4];

  // Convert speed into a serial packet (adjust based on motor controller documentation)
  buffer[0] = 0xAA;  // Start byte
  buffer[1] = (speed & 0xFF00) >> 8;  // Speed high byte
  buffer[2] = speed & 0x00FF;         // Speed low byte
  buffer[3] = 0x55;  // End byte

  hoverboardSerial.write(buffer, 4);  // Send the packet via USART

  #ifdef DEBUG_MODE
    // Debugging output for the data packet
    Serial.print("Data sent: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  #endif
}

// Function to read and process feedback from the hoverboard
void readHoverboardFeedback() {
  // Check if we have enough bytes for a full packet (14 bytes)
  if (hoverboardSerial.available() >= 14) {
    // Read the first two bytes for sync header
    if (hoverboardSerial.read() == 0xAA && hoverboardSerial.read() == 0x55) {
      // Read the remaining data into the feedback structure
      feedback.cmd1 = readInt16();
      feedback.cmd2 = readInt16();
      feedback.speedLeft = readInt16();
      feedback.speedRight = readInt16();
      feedback.currentLeft = readInt16();
      feedback.currentRight = readInt16();
      feedback.voltage = readInt16();
      feedback.temperature = readInt16();
      feedback.cmdLed = readInt16();

      // Print the feedback for debugging
      #ifdef DEBUG_MODE
        Serial.println("Feedback received:");
        Serial.print("Command 1: "); Serial.println(feedback.cmd1);
        Serial.print("Command 2: "); Serial.println(feedback.cmd2);
        Serial.print("Speed Left: "); Serial.println(feedback.speedLeft);
        Serial.print("Speed Right: "); Serial.println(feedback.speedRight);
        Serial.print("Current Left: "); Serial.println(feedback.currentLeft);
        Serial.print("Current Right: "); Serial.println(feedback.currentRight);
        Serial.print("Voltage: "); Serial.println(feedback.voltage);
        Serial.print("Temperature: "); Serial.println(feedback.temperature);
        Serial.print("LED Command: "); Serial.println(feedback.cmdLed);
      #endif
    }
  }
}

// Helper function to read two bytes and combine them into an int16_t
int16_t readInt16() {
  int16_t value = hoverboardSerial.read();           // Read lower byte
  value |= (hoverboardSerial.read() << 8);  // Read upper byte and shift left
  return value;
}
