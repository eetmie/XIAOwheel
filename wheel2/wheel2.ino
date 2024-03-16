// still very work in progress
// 1 button, 1 funky, paddles
// just bunch of stuff thrown together

#include <Arduino.h>
#include <bluefruit.h>
#include <Adafruit_TinyUSB.h> // for Serial.print()

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


// gamepad button numbers!
// change these to your liking
// uint16 so numbers 0-15 available!
#define ENCODER_A_OUT   1
#define ENCODER_B_OUT   2
#define BUTTON_A_OUT    3
#define BUTTON_B_OUT    4
#define BUTTON_C_OUT    5
#define BUTTON_D_OUT    6
#define BUTTON_PUSH_OUT 7
#define SPECIAL_BTN_OUT 8
#define PADL_L_OUT      9
#define PADL_R_OUT      10

// Buttons and encoder debounce delays. Milliseconds.
const unsigned long debounceDelay = 5;
const unsigned long ENCdebounceDelay = 1;

// - nRF52840: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
const int TxPower = 4;


//----------------------------------------------------------
// no need to manually change these values below this line.


// encoder bits. Not all are used
#define CW_ROTATION_1 0b0001
#define CW_ROTATION_2 0b0111
#define CW_ROTATION_3 0b1110
#define CW_ROTATION_4 0b1000

#define CCW_ROTATION_1 0b0010
#define CCW_ROTATION_2 0b1011
#define CCW_ROTATION_3 0b1101
#define CCW_ROTATION_4 0b0100


// Masks
const uint16_t PADL_L_MASK = (1U << PADL_L_OUT);
const uint16_t PADL_R_MASK = (1U << PADL_R_OUT);
const uint16_t ENCODER_A_MASK = (1U << ENCODER_A_OUT);
const uint16_t ENCODER_B_MASK = (1U << ENCODER_B_OUT);
const uint16_t BUTTON_A_MASK = (1U << BUTTON_A_OUT);
const uint16_t BUTTON_B_MASK = (1U << BUTTON_B_OUT);
const uint16_t BUTTON_C_MASK = (1U << BUTTON_C_OUT);
const uint16_t BUTTON_D_MASK = (1U << BUTTON_D_OUT);
const uint16_t BUTTON_PUSH_MASK = (1U << BUTTON_PUSH_OUT);
const uint16_t SPECIAL_BTN_MASK = (1U << SPECIAL_BTN_OUT);


// Settings variables (stored in RAM because XIAO didn't have EEPROM)
int clutchMode = 0;
unsigned long clutchDelay = 200; // total clutch-kick delay in ms
bool isConnected = false;
bool isCharging = false;
bool isAdvertising = false;


// Encoder variables
volatile bool encoderTriggered = false;
volatile long encoderPosition = 0;
volatile int lastEncoded = 0;


// Debounce conf
unsigned long lastInterruptTime = 0;

// Power saving mode
unsigned long lastButtonPressTime = millis();


// Paddle variables
volatile bool PADL_L_state = false;
volatile bool PADL_R_state = false;
volatile unsigned long lastDebounceTimeL = 0;
volatile unsigned long lastDebounceTimeR = 0;
bool lastPadlLPressed = false;
bool lastPadlRPressed = false;


// SPECIAL_BTN state tracking
volatile bool specialBtnState = false;
//volatile bool lastSpecialBtnState = false;
//unsigned long lastDebounceTimeBTN = 0;


// XIAO battery voltage stuff
#define PIN_VBAT        (32) // D32 battery voltage
#define PIN_VBAT_ENABLE (14) // D14 LOW:read enable
#define PIN_HICHG       (22) // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG         (23) // D23 charge indicator LOW:charge HIGH:no charge


// BLE stuff
BLEDis bledis;
BLEHidGamepad blegamepad;
hid_gamepad_report_t gp;


void setup() {

  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("I am alive.");

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
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_Encoder, ISR_DEFERRED | CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), ISR_Encoder, ISR_DEFERRED | CHANGE);

  // Setup paddles
  pinMode(PADL_L, INPUT_PULLUP);
  pinMode(PADL_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PADL_L), ISR_PaddleL, ISR_DEFERRED | CHANGE);
  attachInterrupt(digitalPinToInterrupt(PADL_R), ISR_PaddleR, ISR_DEFERRED | CHANGE);

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
  Bluefruit.setTxPower(TxPower);
  bledis.setManufacturer("iisakki");
  bledis.setModel("02");
  bledis.setSerialNum("0001");
  bledis.begin();
  blegamepad.begin();
  //startAdv();

  // BLE callbacks
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  
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


  // blink battery level until SPECIAL_BTN is (again) pressed
  while (digitalRead(SPECIAL_BTN) == HIGH) {
      Serial.println("Blinkng battery...");
      blinkBatteryLevel();
      delay(1000);
    }
  Serial.flush();  // Ensure this output is sent.
  Serial.println("Exited battery blinking loop.");


  Serial.println("I am ready.");
  startAdv();
}


void loop() {
  checkCharging();

  updateSpecialButton();
  updateFunkyButtons();
  updateEncoder();
  updatePaddles();

  // power saving, WIP
  // If more than 5 (300000) minutes have passed since the last button press, enter sleep mode
  //if ((currentMillis - lastButtonPressTime > 300000) && isAdvertising) {
  //  Serial.println("advertising timeout!")
  //  stopAdv(); // Stop advertising
  //}
  //lastButtonPressTime to start advertising again...

  //delay(5); //let the brother relax
}


void updatePaddles(){
  // Paddle L press/release event handling
  if (PADL_L_state != lastPadlLPressed) {
    lastButtonPressTime = millis(); // Update the last interaction time
    if (clutchMode == 1 && PADL_L_state) {
      executeClutchKick(PADL_L_OUT); // Execute clutch kick with paddle L
    } else {
      // Normal paddle L handling
      handlePaddlePress(PADL_L_MASK, PADL_L_state);
    }
    lastPadlLPressed = PADL_L_state; // Update the last known state
  }

  // Paddle R press/release event handling
  if (PADL_R_state != lastPadlRPressed) {
    lastButtonPressTime = millis(); // Update the last interaction time
    if (clutchMode == 1 && PADL_R_state) {
      executeClutchKick(PADL_R_OUT); // Execute clutch kick with paddle R
    } else {
      // Normal paddle R handling
      handlePaddlePress(PADL_R_MASK, PADL_R_state);
    }
    lastPadlRPressed = PADL_R_state; // Update the last known state
  }
}


void handlePaddlePress(uint16_t mask, bool state) {
    if (state) {
        Serial.print("Paddle/Button with mask ");
        Serial.print(mask);
        Serial.println(" was pressed");
        gp.buttons |= mask; // Set the appropriate bit to indicate a press
    } else {
        Serial.print("Paddle/Button with mask ");
        Serial.print(mask);
        Serial.println(" was released");
        gp.buttons &= ~mask; // Clear the appropriate bit to indicate a release
    }
    blegamepad.report(&gp); // Report the updated gamepad state
}


void executeClutchKick(uint16_t paddle) {

  Serial.println("clutching with");
  Serial.println(paddle);
  
  gp.x = 127; // Press in the clutch
  blegamepad.report(&gp);
  delay(2/3 * clutchDelay); // 2/3 delay before shifting
  
  gp.buttons |= paddle; // Gearswitch with specified button
  blegamepad.report(&gp);

  delay(1/3 * clutchDelay); // 1/3 delay after shifting
  gp.x = 0;// Release the clutch
  blegamepad.report(&gp);
}


void updateEncoder() {
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

    //delay(1); // relax time
    // Clear the actions immediately
    gp.buttons &= ~ENCODER_A_MASK; // Clear the bit for one direction using the mask
    gp.buttons &= ~ENCODER_B_MASK; // Clear the bit for the other direction using the mask
    blegamepad.report(&gp);

    // Reset the flag and the position
    encoderTriggered = false; 
    encoderPosition = 0; // remove to track the number
  }
}


void connect_callback(uint16_t conn_handle) {
  Serial.println("Connected");
  isConnected = true;
}


void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.println("Disconnected");
  isConnected = false;
}


void ISR_Encoder() {
  // again, debounce in interrupt, crazy
  unsigned long interruptTime = millis();
  // Debounce check
  if (interruptTime - lastInterruptTime > ENCdebounceDelay) {
    // not the best way....
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


void ISR_PaddleL() {
  // risky business inside interrupt but im lazy
  if ((millis() - lastDebounceTimeL) > debounceDelay) {
    // Update the last debounce time
    lastDebounceTimeL = millis();
    // Toggle the state
    PADL_L_state = !PADL_L_state;
  }
}


void ISR_PaddleR() {
  // risky business inside interrupt but im lazy
  if ((millis() - lastDebounceTimeR) > debounceDelay) {
    // Update the last debounce time
    lastDebounceTimeR = millis();
    // Toggle the state
    PADL_R_state = !PADL_R_state;
  }
}


void updateSpecialButton() {
    static unsigned long lastDebounceTimeBTN = 0; // Last time the button state was checked
    static bool lastSpecialBtnState = false; // Last known state of the special button
    unsigned long currentMillis = millis(); // Current time
    bool reading = digitalRead(SPECIAL_BTN); // Current reading from the special button

    // Check if the button state has changed since the last read
    if (reading != lastSpecialBtnState) {
        lastDebounceTimeBTN = currentMillis;
    }

    // Check if the debounce period has passed
    if ((currentMillis - lastDebounceTimeBTN) > debounceDelay) {
        // If the button state has changed:
        if (reading != specialBtnState) {
            specialBtnState = reading;

            // Update the gamepad state and print the state to the serial monitor
            if (specialBtnState == HIGH) {
                Serial.println("Button Released");
                gp.buttons &= ~SPECIAL_BTN_MASK; // Clear the button mask bit
            } else {
                Serial.println("Button Pressed");

                gp.buttons |= SPECIAL_BTN_MASK; // Set the button mask bit
            }

            // Report the updated gamepad state
            blegamepad.report(&gp);

        }
    }
    // Update last button press time for power saving feature
    lastButtonPressTime = currentMillis;
    lastSpecialBtnState = reading; // Save the current state as the last state for the next cycle
}


void updateFunkyButtons(){
  // Assuming you're polling the button states in your loop (alternatively, could be in an ISR)
  static bool lastFunkyAState = HIGH;
  static bool lastFunkyBState = HIGH;
  static bool lastFunkyCState = HIGH;
  static bool lastFunkyDState = HIGH;
  static bool lastFunkyPushState = HIGH;

  bool currentFunkyAState = digitalRead(FUNKY_A);
  bool currentFunkyBState = digitalRead(FUNKY_B);
  bool currentFunkyCState = digitalRead(FUNKY_C);
  bool currentFunkyDState = digitalRead(FUNKY_D);
  bool currentFunkyPushState = digitalRead(FUNKY_PUSH);

  if(currentFunkyAState != lastFunkyAState) {
      lastFunkyAState = currentFunkyAState;
      handleFunkyButtons(FUNKY_A, currentFunkyAState == LOW);
  }

  if(currentFunkyBState != lastFunkyBState) {
    lastFunkyBState = currentFunkyBState;
    handleFunkyButtons(FUNKY_B, currentFunkyBState == LOW);
  }

  if(currentFunkyCState != lastFunkyCState) {
    lastFunkyCState = currentFunkyCState;
    handleFunkyButtons(FUNKY_C, currentFunkyCState == LOW);
  }

  if(currentFunkyDState != lastFunkyDState) {
    lastFunkyDState = currentFunkyDState;
    handleFunkyButtons(FUNKY_D, currentFunkyDState == LOW);
  }

  if(currentFunkyPushState != lastFunkyPushState) {
    lastFunkyPushState = currentFunkyPushState;
    handleFunkyButtons(FUNKY_PUSH, currentFunkyPushState == LOW);
  }
}

// WIP
void handleFunkyButtons(int button, bool pressed) {
  //WIP
}

uint16_t determineButtonMask(int button) {
    // This function returns the correct button mask based on the button parameter
    // You need to implement this to return the corresponding mask for each button
    switch(button) {
        case FUNKY_A: return BUTTON_A_MASK;
        case FUNKY_B: return BUTTON_B_MASK;
        case FUNKY_C: return BUTTON_C_MASK;
        case FUNKY_D: return BUTTON_D_MASK;
        case FUNKY_PUSH: return BUTTON_PUSH_MASK;
        default: return 0;
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
  isAdvertising = true;
}


void stopAdv() {
  Bluefruit.Advertising.stop();
  isAdvertising = false;
}


bool checkCharging() {
    static unsigned long lastCheckTime = 0; // Last time the charging status was checked
    unsigned long currentTime = millis(); // Current time
    static bool lastChargingState = false; // Last known charging state

    // Only check charging status if more than 1000 milliseconds (1 second) have passed
    if (currentTime - lastCheckTime > 1000) {
        bool isCurrentlyCharging = digitalRead(PIN_CHG) == LOW; // Check the current charging state

        // If the charging state has changed since the last check, update the LED
        if (isCurrentlyCharging != lastChargingState) {
            digitalWrite(EXT_LED, isCurrentlyCharging ? HIGH : LOW); // Update the LED based on current charging state
            lastChargingState = isCurrentlyCharging; // Remember the new charging state
        }

        lastCheckTime = currentTime; // Update the last check time
    }

    return lastChargingState; // Return the most recent charging state
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
