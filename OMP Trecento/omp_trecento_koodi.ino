#include <Arduino.h>
#include <bluefruit.h>
#include "Button.h"
//#include <Adafruit_TinyUSB.h> // for Serial.print()

// LED output pin
#define EXT_LED         (7)

// XIAO battery voltage stuff
#define PIN_VBAT        (32) // D32 battery voltage
#define PIN_VBAT_ENABLE (14) // D14 LOW:read enable
#define PIN_HICHG       (22) // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG         (23) // D23 charge indicator LOW:charge HIGH:no charge

// timeout shutdown
unsigned long lastActivityTime = millis();  // Initially set to the start time
const unsigned long inactivityThreshold = 1800000; // 30 minutes in milliseconds (30 * 60 * 1000)


// BLE stuff
BLEDis bledis;
BLEHidGamepad blegamepad;
hid_gamepad_report_t gp;

// - nRF52840: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
const int TxPower = 4;
bool isConnected = false;
bool isAdvertising = false;

// Global state variables for the funky buttons
bool stateA = false;
bool stateB = false;
bool stateC = false;
bool stateD = false;
bool statePush = false;

// Define callbacks for each button type
void handlePaddlePress(bool state, int paddleId);
void handleButtonPress(bool state, int buttonId);
void handleFunkyButtonPress(bool state, int buttonId);
void handleEncoderChange(bool isCW);


// button name(GPIO_pin, callback_funtion, id_number)
// buttonButton is mapped as the A-button on an Xbox-style gamepad
Button buttonButton(6, handleButtonPress, 0);

// funkyA is mapped as Dpad UP
Button funkyA(2, handleFunkyButtonPress, 1);

// funkyB is mapped as Dpad LEFT
Button funkyB(3, handleFunkyButtonPress, 2);

// funkyC is mapped as Dpad DOWN
Button funkyC(4, handleFunkyButtonPress, 3);

// funkyD is mapped as Dpad RIGHT
Button funkyD(9, handleFunkyButtonPress, 4);

// funkyPush is mapped as the Y-button
Button funkyPush(8, handleFunkyButtonPress, 5);

// paddleL is mapped to function as the LB (left bumper)
Button paddleL(10, handlePaddlePress, 6);

// paddleR is mapped to function as the RB
Button paddleR(5, handlePaddlePress, 7);

// Encoder variables
volatile int encoderPosition = 0;
int lastEncoded = 0;
volatile unsigned long lastEncoderEvent = 0;
const unsigned long encoderDebounceTime = 50;  // Microseconds
volatile bool cwRotationDetected = false;
volatile bool ccwRotationDetected = false;

// FunkyButton variables
volatile unsigned long lastPushTime = 0;

void ISR_Encoder() {
    unsigned long currentMicros = micros();
    if (currentMicros - lastEncoderEvent > encoderDebounceTime) {
        int MSB = digitalRead(1);  // MSB = most significant bit
        int LSB = digitalRead(0);  // LSB = least significant bit
        int encoded = (MSB << 1) | LSB;
        int sum = (lastEncoded << 2) | encoded;

        if (sum == 0b1101 || sum == 0b0010) {
            cwRotationDetected = true;
        } else if (sum == 0b1110 || sum == 0b0001) {
            ccwRotationDetected = true;
        }
        lastEncoded = encoded;
        lastEncoderEvent = currentMicros;
    }
}

void setup() {
  // LED
  pinMode(EXT_LED, OUTPUT);
  digitalWrite(EXT_LED, LOW);

  //Serial.begin(115200);
  //while (!Serial) delay(10);
  //Serial.println("I am alive.");

  // Setup encoder pins
  pinMode(1, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(1), ISR_Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(0), ISR_Encoder, CHANGE);


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
  bledis.setSerialNum("0002");
  bledis.begin();
  blegamepad.begin();

  // BLE callbacks
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);


  blinkBatteryLevel();
  delay(2000);
  resetGamepadReport();
  startAdv();

}

void loop() {
  unsigned long currentTime = millis();

  // Check for inactivity and shutdown if necessary
  if (currentTime - lastActivityTime >= inactivityThreshold) {
    //Serial.println("Shutting down due to inactivity...");
    sd_power_system_off();  // Call function to power down the system
    return;
  }

  // Check for connection; if not connected, blink the LED
  if (!Bluefruit.connected()) {
      blink(1,100);
      return; // Skip the rest of the loop
  }

  // Update all buttons
  updateAllButtons();

  // Handle encoder changes detected by the ISR
  if (cwRotationDetected) {
    handleEncoderChange(true);
    cwRotationDetected = false;  // Reset the flag
    }
  if (ccwRotationDetected) {
    handleEncoderChange(false);
    ccwRotationDetected = false;  // Reset the flag
    }

  // Check charging status
  isCharging();

  delay(10); // rough 100 hz polling
}


void updateAllButtons() {
    buttonButton.update();
    funkyA.update();
    funkyB.update();
    funkyC.update();
    funkyD.update();
    funkyPush.update();
    paddleL.update();
    paddleR.update();
}


void handlePaddlePress(bool state, int paddleId) {
    String name = (paddleId == 6) ? "Left" : "Right";
    //Serial.print(name + " paddle ");
    //Serial.println(state ? "pressed" : "released");

    //resetGamepadReport(); // Clear existing reports

    // Map left paddle
    if (paddleId == 6) {
        if (state) {
            gp.buttons |= GAMEPAD_BUTTON_2;
        } else {
            gp.buttons &= ~GAMEPAD_BUTTON_2;
        }
    }
    // Map right paddle
    else if (paddleId == 7) {
        if (state) {
            gp.buttons |= GAMEPAD_BUTTON_1;
        } else {
            gp.buttons &= ~GAMEPAD_BUTTON_1;
        }
    }

    blegamepad.report(&gp);
    lastActivityTime = millis();  // Update the last activity time
}


void handleButtonPress(bool state, int buttonId) {
    //Serial.print("Special button ");
    //Serial.println(state ? "pressed" : "released");

    //resetGamepadReport(); // Clear existing reports

    if (state) {
        gp.buttons |= GAMEPAD_BUTTON_0;
    } else {
        gp.buttons &= ~GAMEPAD_BUTTON_0;
    }
    
    blegamepad.report(&gp);
    lastActivityTime = millis();  // Update the last activity time
}


void handleFunkyButtonPress(bool state, int buttonId) {
    // Update the state variables based on buttonId
    switch (buttonId) {
        case 1: stateA = state; break;  // Funky A (Dpad UP)
        case 2: stateB = state; break;  // Funky B (Dpad LEFT)
        case 3: stateC = state; break;  // Funky C (Dpad DOWN)
        case 4: stateD = state; break;  // Funky D (Dpad RIGHT)
        case 5: 
            statePush = state;
            if (state) {  // Log time only when the push is initially pressed
                lastPushTime = millis(); 
            }
            break;
    }

    // Check if the center push is active and if any ABCD button interaction should be ignored
    if (statePush) {
        if ((buttonId == 1 || buttonId == 2 || buttonId == 3 || buttonId == 4)) {
            return; // Ignore both presses and releases of ABCD if the center push is active
        }
    }

    // If any of ABCD are pressed, check to ignore center push
    if ((stateA || stateB || stateC || stateD) && buttonId == 5) {
        return; // Ignore center push if any ABCD is currently pressed
    }

    // Process the button press/release normally and update the gamepad report
    resetGamepadReport(); // Clear previous gamepad state before updating

    // Update the gamepad report based on button states
    if (stateA) gp.buttons |= GAMEPAD_BUTTON_13;
    if (stateB) gp.buttons |= GAMEPAD_BUTTON_15;
    if (stateC) gp.buttons |= GAMEPAD_BUTTON_12;
    if (stateD) gp.buttons |= GAMEPAD_BUTTON_14;
    if (statePush) gp.buttons |= GAMEPAD_BUTTON_3; // Mapping center push to Y/triangle-button

    blegamepad.report(&gp); // Send the updated report
    lastActivityTime = millis();  // Update the last activity time

    // Log button press/release
    //Serial.print("Button ");
    //Serial.print(buttonId);
    //Serial.print(state ? " pressed" : " released");
    //Serial.println();
}


void resetGamepadReport() {
    memset(&gp, 0, sizeof(hid_gamepad_report_t));  // Clear the gamepad report structure
}


void handleEncoderChange(bool isCW) {
  // Decide which button corresponds to the direction of rotation
  uint16_t buttonMask = isCW ? GAMEPAD_BUTTON_10 : GAMEPAD_BUTTON_11;

  // Log the rotation
  //Serial.println(isCW ? "Encoder CW rotation" : "Encoder CCW rotation");

  // Set the corresponding button in the gamepad report
  gp.buttons |= buttonMask;
  blegamepad.report(&gp);  // Send the button press report

  // Clear the button (simulate button release)
  gp.buttons &= ~buttonMask;
  blegamepad.report(&gp);  // Send the button release report
}


float readBatteryVoltage() {
  int readings = 5;
  // this takes average from [readings] amount of of measurements.
  // my prototype had high voltage fluctuations without it
  digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
  delay(1); // Ensure the pin is settled
  
  float voltageSum = 0;
  for (int i = 0; i < readings; i++) {
    float voltage = analogRead(PIN_VBAT) * 2.961 * 3.6 / 4096.0;
    voltageSum += voltage;
    delay(1); // Short delay between readings to allow for settling
  }
  digitalWrite(PIN_VBAT_ENABLE, HIGH); // VBAT read disable
  
  return voltageSum / readings; // Return the average voltage
}


void blinkBatteryLevel() {
  const float BATTERY_MIN_VOLTAGE = 3.0; // minimum voltage
  const float BATTERY_MAX_VOLTAGE = 4.2; // maximum voltage 

  float vbattVoltage = readBatteryVoltage(); // Get the current battery voltage
  // Normalize the battery voltage to a percentage
  int batteryPercent = (int)(((vbattVoltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100);

  // Calculate number of blinks based on 20% increments
  int blinks = (batteryPercent / 20) + 1; // Plus one to ensure at least one blink

  blink(blinks, 300); // Blink with 500ms interval per blink
}


void blink(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(EXT_LED, HIGH);
    delay(duration);
    digitalWrite(EXT_LED, LOW);
    if (i < count - 1) { // Avoid delay after the last blink
      delay(duration);
    }
  }
}

// Not used yet
void connect_callback(uint16_t conn_handle) {
  //Serial.println("Connected");
  isConnected = true;
}

// Not used yet
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  //Serial.println("Disconnected");
  isConnected = false;
}

void startAdv()
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);

  // Include BLE HID service
  Bluefruit.Advertising.addService(blegamepad);

  // There is enough room for the dev name in the advertising packet
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
  isAdvertising = true;
}

// Not used yet
void stopAdv() {
  Bluefruit.Advertising.stop();
  isAdvertising = false;
}


bool isCharging() {
    bool chargingState = digitalRead(PIN_CHG) == LOW;
    digitalWrite(EXT_LED, chargingState ? HIGH : LOW);
    return chargingState;
}
