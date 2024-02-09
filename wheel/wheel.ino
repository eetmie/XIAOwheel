#include <bluefruit.h>
#include <Bounce2.h>
// #include <Adafruit_TinyUSB.h> // for Serial.print()

unsigned long ClutchDelay = 120; // clutch-kick delay in ms

// Pins definition
#define PIN_LBTN        (0)
#define PIN_RPADL       (1)
#define PIN_EXTLED      (2)
#define PIN_RBTN        (3)
#define PIN_LPADL       (4)
#define PIN_BUZZ        (5)

#define NUM_BUTTONS     (4)

#define PIN_VBAT        (32) // D32 battery voltage
#define PIN_VBAT_ENABLE (14) // D14 LOW:read enable
#define PIN_HICHG       (22) // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG         (23) // D23 charge indicator LOW:charge HIGH:no charge

// Battery voltage range. 1S LiPo
const float batteryMaxVoltage = 4.2;
const float batteryMinVoltage = 3.2;

BLEDis bledis;
BLEHidGamepad blegamepad;
hid_gamepad_report_t gp;

// Settings variables (stored in RAM because XIAO didn't have EEPROM)
int clutch_mode = 0;
bool inSettingsMenu = false;

// Track the last reported button state
uint32_t lastButtonsState = 0;

// For holding down buttons
unsigned long rbtnPressedTime = 0;
unsigned long lbtnPressedTime = 0;
bool lpaddlePressedLastFrame = false;
bool rpaddlePressedLastFrame = false;
bool rbtnPreviouslyPressed = false;
bool lbtnPreviouslyPressed = false;
// For entering the menu
unsigned long lastButtonPressTime = 0;
bool isEnteringSettings = false;

// Debounce
Bounce debouncer[NUM_BUTTONS];

void setup() {
    //Serial.begin(115200);
    //while (!Serial) delay(10); // Wait for Serial to be ready.

    // BLE Setup
    Bluefruit.begin();
    Bluefruit.setName("OMP Trecento"); // This name will be visible in Windows

    // - nRF52840: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
    Bluefruit.setTxPower(4);
    
    bledis.setManufacturer("Eddu");
    bledis.setModel("02");
    bledis.setSerial("0001")
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
    pinMode(PIN_EXTLED, OUTPUT);

    digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
    digitalWrite(PIN_HICHG, LOW); // Set charge current to 100mA

    // Initialize ADC for battery voltage reading
    analogReference(AR_DEFAULT); // Default reference voltage
    analogReadResolution(12); // 12-bit resolution

    // I am alive!
    beep(1, 100, 400);
    beep(1, 100, 800);
    delay(500);
    readBattery();
}

void loop() {
    handleConnectionStatus();
    delay(10); // Small delay to prevent the loop from running too fast
}


void handleConnectionStatus() {
    // Blink if no connection
    if (!Bluefruit.connected()) {
        static unsigned long lastBlinkTime = 0;
        const unsigned long blinkInterval = 2000;
        if (millis() - lastBlinkTime >= blinkInterval) {
            blink(1, 100);
            lastBlinkTime = millis();
        }
    } else {
        handleButtonActions();
        // go to sleep after 5min if disconnected and charging
        checkSleepCondition();
        // wake up via button press
        wakeUpAndAdvertise();
    }
}

void handleButtonActions() {
  // oh boy this is messy, good luck
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    debouncer[i].update();
  }

  // Handle clutch_mode behavior when pressing paddles
  if (clutch_mode == 1) {
    // Check if either paddle button is pressed
    bool isLPaddlePressed = debouncer[1].read() == LOW; // LPADL
    bool isRPaddlePressed = debouncer[3].read() == LOW; // RPADL

    // Execute clutch kick on first press of LPADL
    if (isLPaddlePressed && !lpaddlePressedLastFrame) {
      executeClutchKick(1);
      lpaddlePressedLastFrame = true;
    } else if (!isLPaddlePressed) {
      lpaddlePressedLastFrame = false;
    }

    // Execute clutch kick on first press of RPADL
    if (isRPaddlePressed && !rpaddlePressedLastFrame) {
      executeClutchKick(3);
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
        // Kick the clutch so it can be assigned ingame
        executeClutchKick();
        beep(2, 100, 1100);

    } else if (debouncer[2].fell()) { // RBTN for disabling clutch_mode
        clutch_mode = 0;
        beep(1, 100, 900);
    }


    // Exiting settings menu with paddles
    if (debouncer[1].fell() || debouncer[3].fell()) {
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


  // Check if the device should enter a low-power state
  checkSleepCondition();
      
  // Check if the device should wake up and start advertising
  wakeUpAndAdvertise();
}
  // if not connected and charging, go sleep. Wake up if some button is held
  delay(10); // Let the brother relax for a while
}

void executeClutchKick(int buttonNumber = -1) {
  // Press in the clutch
  gp.x = 127;
  blegamepad.report(&gp);

  delay(2/3 * clutchDelay); // Wait 2/3 delay before shifting
  
  if (buttonNumber >= 0) {
    // Gearswitch
    gp.buttons |= (1 << buttonNumber); // Press the specified button
    blegamepad.report(&gp);
    delay(5);
    gp.buttons &= ~(1 << buttonNumber); // Release the specified button
    blegamepad.report(&gp);
  }

  delay(1/3 * clutchDelay);

  // Release the clutch
  gp.x = 0;
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

bool chargeStatus() {
    bool isCharging = digitalRead(PIN_CHG) == LOW;    
    digitalWrite(PIN_EXTLED, isCharging ? HIGH : LOW); // LED on if charging
    return isCharging;
}


void checkSleepCondition() {
    static unsigned long disconnectChargeStartTime = 0; // Track when the device started being disconnected and charging
    static bool advertisingStopped = false; // Flag to track if advertising has been stopped

    if (!Bluefruit.connected() && chargeStatus()) { // Nicely hidden charge led call
        if (disconnectChargeStartTime == 0) {
            // Record the time when the device first becomes disconnected and starts charging
            disconnectChargeStartTime = millis();
        } else if (millis() - disconnectChargeStartTime >= 300000 && !advertisingStopped) {
            // Stop advertising after 5 minutes
            Bluefruit.Advertising.stop();
            advertisingStopped = true; // Set the flag to indicate advertising has stopped
        }
    } else {
        // Reset the timer if the device is connected or not charging
        disconnectChargeStartTime = 0;
    }
}


void wakeUpAndAdvertise() {
    bool buttonPressed = false;
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        debouncer[i].update();
        if (debouncer[i].read() == LOW) {
            buttonPressed = true;
            break;
        }
    }

    if (buttonPressed) {
        // Start advertising again if any button is pressed
        beep(1, 100, 400);
        beep(1, 100, 800);
        startAdv();
    }
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
    // You could toggle VBAT pin on/off to save power, but it has risks 
    
    int vbattADC = analogRead(PIN_VBAT);
    float vbattVoltage = 2.961 * 3.6 * vbattADC / 4096; // Convert ADC value to voltage, not accurate
    float batteryPercentage = (vbattVoltage - batteryMinVoltage) / (batteryMaxVoltage - batteryMinVoltage) * 100;
    int roundedPercentage = round(constrain(batteryPercentage, 0, 100));

    // 1 blink for each 25% of battery level
    int blinks = roundedPercentage / 25;

    // Ensure at least 1 blink for battery > 0%
    if (blinks == 0 && roundedPercentage > 0);
    blinks = 1;
    // Couple beeps to let the user know im dying
    beep(3,50,400);

    // Blink the LED based on the battery level
    digitalWrite(PIN_EXTLED, LOW);
    delay(200);
    blink(blinks, 200);
    }
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
           
