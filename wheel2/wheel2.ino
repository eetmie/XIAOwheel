// still very work in progress
// 1 button, 1 funky, paddles
// just bunch of stuff thrown together

#include <Arduino.h>
#include <bluefruit.h>
#include <Adafruit_TinyUSB.h> // for Serial.print()

#define CCW_ROTATION_1 0b0010
#define CCW_ROTATION_2 0b1011
#define CCW_ROTATION_3 0b1101
#define CCW_ROTATION_4 0b0100

#define CW_ROTATION_1 0b0001
#define CW_ROTATION_2 0b0111
#define CW_ROTATION_3 0b1110
#define CW_ROTATION_4 0b1000

// gamepad button numbers!
// work in progress
#define ENCODER_A_OUT   3
#define ENCODER_B_OUT   4
#define BUTTON_A_OUT    12
#define BUTTON_B_OUT    13
#define BUTTON_C_OUT    14
#define BUTTON_D_OUT    15
#define BUTTON_PUSH_OUT 16
#define SPECIAL_BTN_OUT 17
#define PADL_L_OUT      1
#define PADL_R_OUT      2

// Masks
// only 0-15!
const uint16_t PADL_L_MASK = (1U << PADL_L_OUT);
const uint16_t PADL_R_MASK = (1U << PADL_R_OUT);
const uint16_t ENCODER_A_MASK = (1U << ENCODER_A_OUT);
const uint16_t ENCODER_B_MASK = (1U << ENCODER_B_OUT);


// Pin numbers on the NRF52 board
// i soldered the wires in randomly and then just probed with multimeter
const int EXT_LED = 7; // output!
const int SPECIAL_BTN = 6;
const int PADL_R = 5;
const int PADL_L = 10;

// Funky switch pins
const int FUNKY_A =    2;
const int FUNKY_B =    3;
const int FUNKY_C =    4;
const int FUNKY_D =    9; //haha
const int FUNKY_PUSH = 8;
const int ENC_A =      1;
const int ENC_B =      0;


// Settings variables (stored in RAM because XIAO didn't have EEPROM)
int clutchMode = 0;
unsigned long clutchDelay = 200; // total clutch-kick delay in ms


// Encoder variables
volatile bool encoderTriggered = false;
volatile long encoderPosition = 0;
volatile int lastEncoded = 0;

// Debounce conf
unsigned long lastInterruptTime = 0;
const unsigned long debounceDelay = 5;
const unsigned long ENCdebounceDelay = 2;


// Paddle variables
volatile bool PADL_L_state = false;
volatile bool PADL_R_state = false;
volatile unsigned long lastDebounceTimeL = 0;
volatile unsigned long lastDebounceTimeR = 0;
bool lastPadlLPressed = false;
bool lastPadlRPressed = false;

// SPECIAL_BTN state tracking
volatile bool specialBtnState = false;
volatile bool lastSpecialBtnState = false;
unsigned long lastDebounceTimeBTN = 0;


// XIAO battery voltage stuff
#define PIN_VBAT        (32) // D32 battery voltage
#define PIN_VBAT_ENABLE (14) // D14 LOW:read enable
#define PIN_HICHG       (22) // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG         (23) // D23 charge indicator LOW:charge HIGH:no charge

BLEDis bledis;
BLEHidGamepad blegamepad;
hid_gamepad_report_t gp;


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // LED
  pinMode(EXT_LED, OUTPUT);
  digitalWrite(EXT_LED, HIGH);

  // Setup funky buttons
  pinMode(FUNKY_A, INPUT_PULLUP);
  pinMode(FUNKY_B, INPUT_PULLUP);
  pinMode(FUNKY_C, INPUT_PULLUP);
  pinMode(FUNKY_D, INPUT_PULLUP);
  pinMode(FUNKY_PUSH, INPUT_PULLUP);
  
  // Setup special button
  pinMode(SPECIAL_BTN, INPUT_PULLUP);

  // Setup encoder
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, ISR_DEFERRED | CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, ISR_DEFERRED | CHANGE);

  // Setup paddles
  pinMode(PADL_L, INPUT_PULLUP);
  pinMode(PADL_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PADL_L), updatePaddleL, ISR_DEFERRED | CHANGE);
  attachInterrupt(digitalPinToInterrupt(PADL_R), updatePaddleR, ISR_DEFERRED | CHANGE);

  // Charging stuff
  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  pinMode(PIN_HICHG, OUTPUT);
  pinMode(PIN_CHG, INPUT);
  digitalWrite(PIN_HICHG, LOW); // Set charge current to 100mA
  analogReference(AR_DEFAULT); // Default reference voltage
  analogReadResolution(12); // 12-bit resolution

  // BLE
  Bluefruit.begin();
  Bluefruit.setName("OMP Trecento"); // This name will be visible in Windows

  // - nRF52840: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
  Bluefruit.setTxPower(4);
    
  bledis.setManufacturer("iisakki");
  bledis.setModel("02");
  bledis.setSerialNum("0001");
  bledis.begin();
  blegamepad.begin();
  startAdv();

  // BLE callbacks
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Charging and Battery Monitoring Setup
  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  pinMode(PIN_HICHG, OUTPUT);
  pinMode(PIN_CHG, INPUT);
  pinMode(EXT_LED, OUTPUT);

  digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
  digitalWrite(PIN_HICHG, LOW); // Set charge current to 100mA







  Serial.println("Checking clutch...");

  // hold SPECIAL_BTN at startup to enable clutchMode
  if (digitalRead(SPECIAL_BTN) == LOW) {
    Serial.println("Clutch enabled!");
    clutchMode = 1;

    // Wait for the release (because the button will be used soon again)
    while (digitalRead(SPECIAL_BTN) == LOW) {
      blink(1,100);
      delay(100);
    }
  }

  digitalWrite(EXT_LED, LOW);
  delay(1000);

  Serial.println("Checking battery...");


  // blink battery level until SPECIAL_BTN is (again) pressed
  while (digitalRead(SPECIAL_BTN) == HIGH) {
      blinkBatteryLevel();
      delay(1000);
    }


  Serial.println("I am alive.");
}

void loop() {
  unsigned long currentMillis = millis();

    // Check if the encoder was triggered
  if (encoderTriggered) {
    // Check the direction of rotation
    if (encoderPosition > 0) {
      Serial.println("CW Rotation");
      gp.buttons |= ENCODER_B_MASK; // Set the bit for CW rotation using the mask
    } else {
      Serial.println("CCW Rotation");
      gp.buttons |= ENCODER_A_MASK; // Set the bit for CCW rotation using the mask
    }

    // Report the button press
    blegamepad.report(&gp);

    // Clear the actions immediately
    gp.buttons &= ~ENCODER_A_MASK; // Clear the bit for one direction using the mask
    gp.buttons &= ~ENCODER_B_MASK; // Clear the bit for the other direction using the mask
    blegamepad.report(&gp);

    // Reset the flag and the position
    encoderTriggered = false;
    encoderPosition = 0;
  }


    // Paddle L press/release event handling
  if (PADL_L_state != lastPadlLPressed) {
      if (PADL_L_state) {
          Serial.println("Paddle L was pressed");
          gp.buttons |= PADL_L_MASK; // Set the Paddle R button bit to indicate a press
      } else {
          Serial.println("Paddle L was released");
          gp.buttons &= ~PADL_L_MASK; // Clear the Paddle R button bit to indicate a release
      }
      blegamepad.report(&gp); // Report the updated gamepad state
      lastPadlLPressed = PADL_L_state; // Update the last known state
  }

    // Paddle R press/release event handling
  if (PADL_R_state != lastPadlRPressed) {
      if (PADL_R_state) {
          Serial.println("Paddle R was pressed");
          gp.buttons |= PADL_R_MASK; // Set the Paddle R button bit to indicate a press
      } else {
          Serial.println("Paddle R was released");
          gp.buttons &= ~PADL_R_MASK; // Clear the Paddle R button bit to indicate a release
      }
      blegamepad.report(&gp); // Report the updated gamepad state
      lastPadlRPressed = PADL_R_state; // Update the last known state
  }


  // Read funky buttons with logic
  // work in progress


  // Special button handling
  // hold for layer 2 maybe in the future
  bool reading = digitalRead(SPECIAL_BTN);
  // Check if the button state has changed
  if (reading != lastSpecialBtnState) {
    // reset the debouncing timer
    lastDebounceTimeBTN = currentMillis;
  }
  if ((currentMillis - lastDebounceTimeBTN) > debounceDelay) {
    // if the button state has changed:
    if (reading != specialBtnState) {
      specialBtnState = reading;
      // only toggle the LED if the new button state is HIGH
      if (specialBtnState == HIGH) {
        Serial.println("Button Released");
        // gp
      } else {
        Serial.println("Button Pressed");
        // gp
      }
    }
  }
  // ge report
  lastSpecialBtnState = reading;


}




void updateEncoder() {
    unsigned long interruptTime = millis();
    // Debounce check
    if (interruptTime - lastInterruptTime > ENCdebounceDelay) {
        int MSB = digitalRead(ENC_A); // MSB = most significant bit
        int LSB = digitalRead(ENC_B); // LSB = least significant bit
        int encoded = (MSB << 1) | LSB; // Combining the two bits
        int sum = (lastEncoded << 2) | encoded; // Creating a unique identifier for movement

        if (sum == CCW_ROTATION_3 || sum == CCW_ROTATION_1) {
            encoderPosition++;
            encoderTriggered = true;
        } 
        if (sum == CW_ROTATION_3 || sum == CW_ROTATION_1) {
            encoderPosition--;
            encoderTriggered = true;
        }
        lastEncoded = encoded;
    }
    lastInterruptTime = interruptTime; // Update the last interrupt time


}
void handleButtonStateChange(int button, bool pressed) {
  // work in progress
  return;
}

void connect_callback(uint16_t conn_handle) {
  // work in progress
  Serial.println("Connected");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  // work in progress
  Serial.println("Disconnected");
}

void updatePaddleL() {
  if ((millis() - lastDebounceTimeL) > debounceDelay) {
    // Update the last debounce time
    lastDebounceTimeL = millis();
    // Toggle the state
    PADL_L_state = !PADL_L_state;
  }
}

void updatePaddleR() {
  if ((millis() - lastDebounceTimeR) > debounceDelay) {
    lastDebounceTimeR = millis();
    PADL_R_state = !PADL_R_state;
  }
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);
  Bluefruit.Advertising.addService(blegamepad);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void stopAdv() {
  Bluefruit.Advertising.stop();
}

float readBatteryVoltage() {
  // this takes average from [readings] amount of of measurements.
  // my prototype had high voltage fluctuations without it
  digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
  delay(1); // Ensure the pin is settled
  
  float voltageSum = 0;
  int readings = 5; // Number of measurements to average
  for (int i = 0; i < readings; i++) {
    float voltage = analogRead(PIN_VBAT) * 2.961 * 3.6 / 4096.0;
    voltageSum += voltage;
    delay(5); // Short delay between readings to allow for settling
  }
  digitalWrite(PIN_VBAT_ENABLE, HIGH); // VBAT read disable
  
  return voltageSum / readings; // Return the average voltage
}

void blinkBatteryLevel() {
  // Blinks the battery level like (very nice) Anduril flashlight UI.
  // https://ivanthinking.net/thoughts/anduril2-manual/
  // Eg. 3.8V would be 3 long and 8 short blinks.
  float vbattVoltage = readBatteryVoltage();
  int wholePart = (int)vbattVoltage;
  int fractionalPart = (int)((vbattVoltage - wholePart) * 10);
  blink(wholePart, 500);
  delay(1000);
  blink(fractionalPart, 200);
}

void blink(int blinks, int blinkDelay) {
  for (int i = 0; i < blinks; i++) {
    digitalWrite(EXT_LED, HIGH);
    delay(blinkDelay);
    digitalWrite(EXT_LED, LOW);
    delay(blinkDelay);
  }
}
