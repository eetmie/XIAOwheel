// basically a very simple bluetooth wheel, with bonus clutch kick system added.
// In Assetto Corsa cars with manual gearbox switch very slowly with automatic clutch, so this wheel kicks the clutch gear way faster (I dont have clutch pedal).
// TODO: paddle interrupts for better performance

#include <bluefruit.h>
#include <Bounce2.h>
//#include <Adafruit_TinyUSB.h> // for Serial.print()

unsigned long clutchDelay = 200; // total clutch-kick delay in ms

// Pins definition
#define NUM_BUTTONS     (4)

#define PIN_LBTN        (0)
#define PIN_RPADL       (1)
#define PIN_EXTLED      (2)
#define PIN_RBTN        (3)
#define PIN_LPADL       (4)
#define PIN_BUZZ        (5)

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

// Settings variables (stored in RAM because XIAO didn't have EEPROM haha)
int clutchMode = 0;
int clutchSpeed = 0;
bool inSettingsMenu = false;
bool isEnteringSettings = false;

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

bool isCharging = false;
unsigned long chargeStartTime = 0;
bool shouldAdvertise = true;

// Debounce
Bounce debouncer[NUM_BUTTONS];

void setup() {
    //Serial.begin(115200);
    //while (!Serial) delay(10); // Wait for Serial to be ready.

    Bluefruit.begin();
    Bluefruit.setName("OMP Trecento"); // This name will be visible in Windows

    // - nRF52840: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +2dBm, +3dBm, +4dBm, +5dBm, +6dBm, +7dBm and +8dBm.
    Bluefruit.setTxPower(4);
    
    bledis.setManufacturer("Eddu");
    bledis.setModel("02");
    bledis.setSerialNum("0001");
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
    // Update charge status at defined intervals
    static unsigned long lastChargeCheckTime = 0;
    const unsigned long chargeCheckInterval = 5000;
    if (millis() - lastChargeCheckTime >= chargeCheckInterval) {
        chargeStatus(); // Check and update charging status
        lastChargeCheckTime = millis();
    }

    // Handle blinking for no connection status
    // Charge light messes this up a bit but not bad
    if (!Bluefruit.connected() && shouldAdvertise) {
        static unsigned long lastBlinkTime = 0;
        const unsigned long blinkInterval = 2000;
        if (millis() - lastBlinkTime >= blinkInterval) {
            blink(1, 100); // Perform blinking to indicate no connection
            lastBlinkTime = millis();
        }
    } else if (Bluefruit.connected()) {
        // Handle main functionality when connected
        handleButtonActions();
    }

    // Wake up and start advertising on button press if currently not advertising
    if (!shouldAdvertise && anyButtonPressed()) {
        shouldAdvertise = true; // Set flag to start advertising
        startAdv(); // Start BLE advertising
    }

    delay(10); // Let the guy chill
}


bool anyButtonPressed() {
    // Check if any button is pressed with debouncing
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        debouncer[i].update(); // Update the debouncer state
        if (debouncer[i].read() == LOW) {
            return true; // Button is pressed and debounced
        }
    }
    return false; // No button pressed
}



void executeClutchKick(int buttonNumber = -1) {
  // Press in the clutch
  gp.x = 127;
  blegamepad.report(&gp);
  delay(2/3 * clutchDelay + (clutchSpeed * 200)); // Wait 2/3 delay before shifting
  
  if (buttonNumber >= 0) {
    // Gearswitch
    gp.buttons |= (1 << buttonNumber); // Press the specified button
    blegamepad.report(&gp);
  }

  delay(1/3 * clutchDelay);
  // Release the clutch
  gp.x = 0;
  blegamepad.report(&gp);
}



void handleButtonActions() {
  // oh boy this is messy, good luck
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    debouncer[i].update();
  }

  // Handle clutch_mode behavior when pressing paddles
  if (clutchMode == 1) {
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
  if (debouncer[2].read() == LOW && debouncer[0].read() == LOW) {
    if (!isEnteringSettings) {
        lastButtonPressTime = millis();
        isEnteringSettings = true;
    } else if (millis() - lastButtonPressTime >= 1000) {
        if (!inSettingsMenu) { // Only beep when first entering the settings menu
            beep(1, 300, 2200); // Beep once upon entering the settings
        }
        settingsMenu();
        isEnteringSettings = false; // Reset flag to prevent re-triggering
        lastButtonsState = 0; // Ensure the state is reset to send report next time
        return;
    }
  } else {
    isEnteringSettings = false; // Reset flag if either button is released
  }

  // In Settings Menu Logic
  if (inSettingsMenu) {
    // Toggle clutch mode
    if (debouncer[0].fell()) {
        bool previousClutchMode = clutchMode;
        clutchMode = !clutchMode; // Toggle the value
        if (clutchMode != previousClutchMode) { // Only beep if state changed
            executeClutchKick(); // Optional based on your logic
            beep(clutchMode ? 2 : 1, 100, clutchMode ? 2300 : 2200);
        }
    }

    // Toggle clutch speed
    if (debouncer[2].fell()) {
        bool previousClutchSpeed = clutchSpeed;
        clutchSpeed = !clutchSpeed;
        if (clutchSpeed != previousClutchSpeed) { // Only beep if state changed
            beep(clutchSpeed ? 2 : 1, 100, clutchSpeed ? 2300 : 2200);
        }
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
}


void settingsMenu() {
    if (!inSettingsMenu) {

        readBattery();
        inSettingsMenu = true;
    } else {
        inSettingsMenu = false;
        beep(1, 100, 2400);
        beep(1, 100, 2200);
    }
}


bool chargeStatus() {
    isCharging = digitalRead(PIN_CHG) == LOW;
    digitalWrite(PIN_EXTLED, isCharging ? HIGH : LOW); // LED on if charging
    
    if (isCharging && !Bluefruit.connected()) {
        if (chargeStartTime == 0) { // Start timer if not already started
            chargeStartTime = millis();
        } else if (millis() - chargeStartTime > 30000) {
            shouldAdvertise = false;
            stopAdv(); // Stop advertising
        }
    } else {
        chargeStartTime = 0; // Reset timer if not charging or connected
        shouldAdvertise = true; // Enable advertising if connected
    }
    
    return isCharging;
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


void stopAdv() {
  Bluefruit.Advertising.stop();
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
    if (blinks == 0 && roundedPercentage > 0) {
      blinks = 1;
      // Couple beeps to let the user know im dying
      beep(3,50,2250);
    }


    // Blink the LED based on the battery level
    digitalWrite(PIN_EXTLED, LOW);
    delay(300);
    blink(blinks, 200);
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
      delay(blinkDelay);
    }
  }
}
