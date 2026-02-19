/**
 * Tests different functions of the Bobby Car
 * 128x64 OLED Display
 * WS2812B LED Strips
 * Headlights PWM control
 * Buzzer with melody
 */

#include "FastLED.h"
#include "pitches.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// FASTLED
#define DATA_PIN 4
#define DATA_PIN_2 5
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS 26
#define BRIGHTNESS 120
CRGB leds[NUM_LEDS];

// HEADLIGHTS
#define HEADLIGHTS_PIN 9
unsigned int headlightsBrightness = 0;

// DISPLAY
U8G2_SSD1306_128X64_NONAME_F_SW_I2C
u8g2(U8G2_R0, /* clock=*/SCL, /* data=*/SDA,
     /* reset=*/U8X8_PIN_NONE); // All Boards without Reset of the Display

// BUZZER
#define BUZZER_PIN 8
// notes in the melody:
int melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3,
                NOTE_G3, 0,       NOTE_B3, NOTE_C4};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {4, 8, 8, 4, 4, 4, 4, 4};

void setup(void) {
  // doBuzzer();

  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip)
      .setDither(BRIGHTNESS < 255);
  FastLED.addLeds<LED_TYPE, DATA_PIN_2, COLOR_ORDER>(leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip)
      .setDither(BRIGHTNESS < 255);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  // Start display
  u8g2.begin();

  // Setup pin for headlights
  pinMode(HEADLIGHTS_PIN, OUTPUT);
}

void loop(void) {
  doLed();
  doDisplay();
  doHeadlights();
  // delay(1000);
}

void doLed() {
  pride();
  FastLED.show();
}

void doDisplay() {
  u8g2.clearBuffer();                  // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
  u8g2.drawStr(0, 10, "Hello World!"); // write something to the internal memory
  u8g2.setCursor(40, 40);
  u8g2.print(millis()); // write something to the internal memory
  u8g2.sendBuffer();    // transfer internal memory to the display
}

void doBuzzer() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    // e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(BUZZER_PIN);
  }
}

void doHeadlights() {
  analogWrite(HEADLIGHTS_PIN, 150);
  headlightsBrightness += 20;
}

void pride() {
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  uint8_t sat8 = beatsin88(87, 220, 250);
  uint8_t brightdepth = beatsin88(341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88(203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16; // gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);

  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis;
  sLastMillis = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88(400, 5, 9);
  uint16_t brightnesstheta16 = sPseudotime;

  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16 += brightnessthetainc16;
    uint16_t b16 = sin16(brightnesstheta16) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    CRGB newcolor = CHSV(hue8, sat8, bri8);

    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS - 1) - pixelnumber;

    nblend(leds[pixelnumber], newcolor, 64);
  }
}