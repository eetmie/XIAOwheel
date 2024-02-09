#include <bluefruit.h>
#include <Adafruit_TinyUSB.h> // for Serial.print()
#include <Bounce2.h>

unsigned long axisSimulateDelay = 50; // clutch-kick delay in ms

// Pins definition
#define PIN_LBTN        (0)
#define PIN_RPADL       (1)
#define PIN_EXTLED      (2)
#define PIN_RBTN        (3)
#define PIN_LPADL       (4)
#define PIN_BUZZ        (5)
#define NUM_BUTTONS      4

#define PIN_VBAT        (32) // D32 battery voltage
#define PIN_VBAT_ENABLE (14) // D14 LOW:read enable
#define PIN_HICHG       (22) // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG         (23) // D23 charge indicator LOW:charge HIGH:no charge

// Battery voltage range
const float batteryMaxVoltage = 4.2; // Maximum voltage of 1S LiPo
const float batteryMinVoltage = 3.2; // Minimum voltage of 1S LiPo

BLEDis bledis;
BLEHidGamepad blegamepad;
hid_gamepad_report_t gp;

// Settings variables (stored in RAM)
int clutch_mode = 0; // 0 disabled, 1 enabled
bool inSettingsMenu = false;

// Global variable to track the last reported button state
uint32_t lastButtonsState = 0;

// for holding donw buttons
unsigned long rbtnPressedTime = 0;
unsigned long lbtnPressedTime = 0;
bool lpaddlePressedLastFrame = false;
bool rpaddlePressedLastFrame = false;
bool rbtnPreviouslyPressed = false;
bool lbtnPreviouslyPressed = false;
// for entering the menu
unsigned long lastButtonPressTime = 0;
bool isEnteringSettings = false;


Bounce debouncer[NUM_BUTTONS];

void setup() {
    //Serial.begin(115200);
    //while (!Serial) delay(10); // Wait for Serial to be ready

    // BLE Setup
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    bledis.setManufacturer("Eddu");
    bledis.setModel("OMP Trecento");
    bledis.begin();
    blegamepad.begin();
    startAdv();

    // Button Setup with Debounce
    const int buttonPins[NUM_BUTTONS] = {PIN_LBTN, PIN_RPADL, PIN_RBTN, PIN_LPADL}; // Include all button pins
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        pinMode(buttonPins[i], INPUT_PULLUP);
        debouncer[i].attach(buttonPins[i]);
        debouncer[i].interval(3); // Debounce interval in ms
    }

    // Charging and Battery Monitoring Setup
    pinMode(PIN_VBAT, INPUT);
    pinMode(PIN_VBAT_ENABLE, OUTPUT);
    pinMode(PIN_HICHG, OUTPUT);
    pinMode(PIN_CHG, INPUT);
    pinMode(PIN_EXTLED, OUTPUT); // LED pin as output

    digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
    digitalWrite(PIN_HICHG, LOW); // Set charge current to 100mA

    // Initialize ADC for battery voltage reading
    analogReference(AR_DEFAULT); // Default reference voltage
    analogReadResolution(12); // 12-bit resolution

    // i am ready!
    beep(1, 100, 400);
    beep(1, 100, 800);
    blink(1,100);
}

void loop() {
  // nothing to do if not connected
  if (!Bluefruit.connected()) return;

  // Update button states
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    debouncer[i].update();
  }

  // Handle clutch_mode behavior when pressing paddles
  if (clutch_mode == 1) {
    // Check if either paddle button is pressed
    bool isLPaddlePressed = debouncer[1].read() == LOW; // Assuming LPADL is at index 1
    bool isRPaddlePressed = debouncer[3].read() == LOW; // Assuming RPADL is at index 3

    // Execute clutch kick on first press of LPADL
    if (isLPaddlePressed && !lpaddlePressedLastFrame) {
      executeClutchKick();
      lpaddlePressedLastFrame = true;
    } else if (!isLPaddlePressed) {
      lpaddlePressedLastFrame = false;
    }

    // Execute clutch kick on first press of RPADL
    if (isRPaddlePressed && !rpaddlePressedLastFrame) {
      executeClutchKick();
      rpaddlePressedLastFrame = true;
    } else if (!isRPaddlePressed) {
      rpaddlePressedLastFrame = false;
    }
  }

  // Detect if RBTN and LBTN are pressed together for entering settings
  if (debouncer[2].read() == LOW && debouncer[0].read() == LOW) { // Assuming RBTN is at index 2 and LBTN is at index 0
    if (!isEnteringSettings) {
      lastButtonPressTime = millis();
      isEnteringSettings = true;
    } else if (millis() - lastButtonPressTime >= 1000) {
      settingsMenu();
      isEnteringSettings = false; // Reset flag to prevent re-triggering
      // Skip sending any report when entering the settings menu
      lastButtonsState = 0; // Ensure the state is reset to send report next time
      return;
    }
  } else {
    isEnteringSettings = false; // Reset flag if either button is released
  }

  // In Settings Menu Logic
  if (inSettingsMenu) {
    if (debouncer[0].fell()) { // LBTN for enabling clutch_mode
        clutch_mode = 1;
        beep(1, 100, 1100);

        // Simulate clutch engagement for mapping
        gp.x = 127; // Simulate full clutch engagement
        blegamepad.report(&gp);
        delay(200); // Short delay
        gp.x = 0; // Reset clutch simulation
        blegamepad.report(&gp);

        beep(2, 100, 1100);

    } else if (debouncer[2].fell()) { // RBTN for disabling clutch_mode
        clutch_mode = 0;
        beep(1, 100, 900);
    }


    // Exiting settings menu with paddles
    if (debouncer[1].fell() || debouncer[3].fell()) { // Assuming RPADL and LPADL are at index 1 and 3
      settingsMenu(); // Toggle settings menu state
    }
    return; // Prevent further processing when in settings menu
  }

  // Prepare a variable for the current button state outside of settings menu
  uint32_t currentButtonsState = 0;

  // Update the gamepad report with the current button state outside of settings menu
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    if (debouncer[i].read() == LOW) { // Button is active LOW
      currentButtonsState |= (1 << i); // Set bit for button i
    }
  }

  // Only send a report if the button state has changed
  if (currentButtonsState != lastButtonsState) {
    memset(&gp, 0, sizeof(hid_gamepad_report_t)); // Clear previous report
    gp.buttons = currentButtonsState; // Update the gamepad report
    blegamepad.report(&gp); // Send the gamepad report
    lastButtonsState = currentButtonsState; // Update lastButtonsState
  }

  chargeStatus();
  delay(10); // let the brother relax for a while
}

void executeClutchKick() {
  gp.x = 127; // Simulate clutch engagement (full axis movement)
  blegamepad.report(&gp);
  delay(axisSimulateDelay); // Short delay to simulate clutch kick

  gp.x = 0; // Reset axis position
  blegamepad.report(&gp);
}

void settingsMenu() {
    if (!inSettingsMenu) {
        beep(1, 300, 1200);
        readBattery();
        inSettingsMenu = true;
    } else {
        inSettingsMenu = false;
        beep(1, 100, 1200);
        beep(1, 100, 900);
    }
}

void chargeStatus() {
    bool isCharging = digitalRead(PIN_CHG) == LOW;    
    digitalWrite(PIN_EXTLED, isCharging ? HIGH : LOW); // LED on if charging

}

void startAdv(void) {
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

void readBattery() {
    int vbattADC = analogRead(PIN_VBAT);
    float vbattVoltage = 2.961 * 3.6 * vbattADC / 4096; // Convert ADC value to voltage
    float batteryPercentage = (vbattVoltage - batteryMinVoltage) / (batteryMaxVoltage - batteryMinVoltage) * 100;
    int roundedPercentage = round(constrain(batteryPercentage, 0, 100));

    // Calculate the number of blinks: 1 blink for each 25% of battery level
    int blinks = roundedPercentage / 25;
    if (blinks == 0 && roundedPercentage > 0) blinks = 1; // Ensure at least 1 blink for battery > 0%

    // Blink the LED based on the battery level
    for (int i = 0; i < blinks; i++) {
        digitalWrite(PIN_EXTLED, HIGH); // Turn on LED
        delay(200);                     // Keep it on for 200 milliseconds
        digitalWrite(PIN_EXTLED, LOW);  // Turn off LED

        if (i < blinks - 1) {           // If not the last blink, wait 200ms before the next blink
            delay(200);
        }
    }

    // Additional short delay after blinking to visually separate the blink sequences
    delay(500);
}

// beep beep
void beep(int beeps, int beepDelay, int beepTone) {
  for (int i = 0; i < beeps; i++) {
    tone(PIN_BUZZ, beepTone);
    delay(beepDelay);
    noTone(PIN_BUZZ);
    if (i < beeps - 1) {
      delay(beepDelay); // Delay between beeps if not the last beep
    }
  }
}

// blink blink
void blink(int blinks, int blinkDelay) {
  for (int i = 0; i < blinks; i++) {
    digitalWrite(PIN_EXTLED, HIGH);
    delay(blinkDelay);
    digitalWrite(PIN_EXTLED, LOW);
    if (i < blinks - 1) {
      delay(blinkDelay); // Delay between blinks if not the last blink
    }
  }
}
           
