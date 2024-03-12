#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_TinyUSB.h"

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD()
};

// USB HID object
Adafruit_USBD_HID usb_hid;

hid_gamepad_report_t    gp;


//---------------------------------------------------------------------------

// Encoder (interrupt) pins.
const int encoderPinA = 6;
const int encoderPinB = 7;

// Button pins
const int buttonPins[5] = {26, 27, 28, 29, 0}; // L/D/R/U/CENTER. Seeed Rp2040 button pins

const uint8_t buttonMappings[5] = {20, 21, 22, 23, 24}; // output button mappings, use whatever you need. I used these random values.

//---------------------------------------------------------------------------

// NeoPixel setup
int Power = 11;
int PIN = 12;
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Encoder variables
volatile long encoderPosition = 0;
volatile int lastEncoded = 0;
unsigned long lastInterruptTime = 0;
unsigned long lastDebounceTime[5] = {0, 0, 0, 0, 0};
unsigned long debounceDelay = 3;

// Button variables
bool buttonStates[5] = {false, false, false, false, false};
bool lastButtonStates[5] = {false, false, false, false, false};

bool udlrPressedRecently = false;
bool centerPressed = false;

void setup() {
  //Serial.begin(115200);
  //while (!Serial) delay(10);

  #if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
  #endif

  // Setup HID
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  // Setup button pins
  for (int i = 0; i < 5; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Wait until device mounted
  while(!TinyUSBDevice.mounted()) delay(1);

  // NeoPixel setup
  pixels.begin();
  pinMode(Power, OUTPUT);
  digitalWrite(Power, HIGH);
  randomSeed(analogRead(0));

  // Fade LED at startup, sick
  uint32_t startColor = pixels.Color(0, 0, 0);
  uint32_t endColor = generateRandomColor();
  fadeBetweenColors(startColor, endColor, 3000);
}

void loop() {
  static long lastPosition = -999;
  unsigned long currentMillis = millis();

  // Check encoder position
  if (encoderPosition != lastPosition) {
    // you can also track the encoder position
    lastPosition = encoderPosition;

    // Change LED color
    uint32_t randomColor = generateRandomColor();
    pixels.setPixelColor(0, randomColor);
    pixels.show();
  }

  // Read buttons and handle debouncing
  for (int i = 0; i < 5; i++) {
    bool reading = !digitalRead(buttonPins[i]);
    if (reading != lastButtonStates[i]) {
      lastDebounceTime[i] = currentMillis;
    }

    if ((currentMillis - lastDebounceTime[i]) > debounceDelay) {
      if (reading != buttonStates[i]) {
        buttonStates[i] = reading;
        handleButtonStateChange(i, buttonStates[i]);
      }
    }
    lastButtonStates[i] = reading;
  }
  udlrPressedRecently = false;
}

void updateEncoder() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > debounceDelay) {
    int MSB = digitalRead(encoderPinA); // MSB = most significant bit
    int LSB = digitalRead(encoderPinB); // LSB = least significant bit
    int encoded = (MSB << 1) | LSB; // Combining the two bits
    int sum = (lastEncoded << 2) | encoded; // Creating a unique identifier for movement

    if (sum == 0b1101 || sum == 0b0010) {
      encoderPosition++;
      gp.buttons |= (1U << 10); // Simulate press
    } else if (sum == 0b1110 || sum == 0b0001) {
      encoderPosition--;
      gp.buttons |= (1U << 11); // Simulate press
    }

    usb_hid.sendReport(0, &gp, sizeof(gp)); // Send the press report

    // Clear the actions immediately
    gp.buttons &= ~(1U << 10); // Clear the bit for one direction
    gp.buttons &= ~(1U << 11); // Clear the bit for the other direction

    usb_hid.sendReport(0, &gp, sizeof(gp)); // Send the clear report

    lastEncoded = encoded;
    lastInterruptTime = currentTime;
  }
}

void handleButtonStateChange(int button, bool pressed) {
    if (button == 4) { // index 4 is the  center button
        centerPressed = pressed;
    } else if (centerPressed) {
        // ignore other funky presses when center is pressed
        return;
    }

    // If not processing the center button and no button is allowed to be processed, return
    if (udlrPressedRecently && button != 4) return;

    // Update the state for UDLR buttons, considering the special logic
    if (button < 4) { // index 0-3 for UDLR buttons
        udlrPressedRecently = pressed;
    }

    uint8_t mappedBit = buttonMappings[button]; // Get the mapped gamepad bit for the button

    // Update the gamepad report based on the button pressed
    if (pressed) {
        // Clear all other bits to ensure only one button is active at a time
        gp.buttons = (1UL << mappedBit);
    } else {
        // If the button is released, clear its bit.
        gp.buttons &= ~(1UL << mappedBit);
    }

    // Send updated report
    usb_hid.sendReport(0, &gp, sizeof(gp));
}


uint32_t generateRandomColor() {
  return pixels.Color(random(0, 256), random(0, 256), random(0, 256));
}

void fadeBetweenColors(uint32_t startColor, uint32_t endColor, unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
    float ratio = (float)(millis() - startTime) / duration;
    uint8_t red = (uint8_t)lerp((startColor >> 16) & 0xFF, (endColor >> 16) & 0xFF, ratio);
    uint8_t green = (uint8_t)lerp((startColor >> 8) & 0xFF, (endColor >> 8) & 0xFF, ratio);
    uint8_t blue = (uint8_t)lerp(startColor & 0xFF, endColor & 0xFF, ratio);
    pixels.setPixelColor(0, pixels.Color(red, green, blue));
    pixels.show();
    delay(10);
  }
}

float lerp(float start, float end, float ratio) {
  return start + ratio * (end - start);
}