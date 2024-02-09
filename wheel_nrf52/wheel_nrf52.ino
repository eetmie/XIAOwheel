#include <bluefruit.h>

BLEDis bledis;
BLEHidGamepad blegamepad;

hid_gamepad_report_t gp;

// Settings variables (stored in RAM)
int clutch_mode = 0;       // 0 disabled, 1 fast, 2 slow
int launchControl_mode = 0; // 0 disabled, 1 setup1, 2 setup2
bool inSettingsMenu = false;

unsigned long buttonPressStartTime = 0;

void setup() {
  Serial.begin(115200);

  while (!Serial) delay(10); // Wait for Serial to be ready

  Bluefruit.begin();
  Bluefruit.setTxPower(-40); // Max power

  bledis.setManufacturer("Eddu");
  bledis.setModel("OMP Trecento");
  bledis.begin();

  blegamepad.begin();

  startAdv();
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

void loop() {
  // Check if connected
  if (!Bluefruit.connected()) return;

  if (inSettingsMenu) {
    updateSettings();
    // Additional settings menu logic here
  } else {
    // Example gamepad logic or other functionalities
    processGamepadActions();
  }
}

void enterSettingsMenu() {
  inSettingsMenu = true;
  Serial.println("Entering Settings Menu");
}

void exitSettingsMenu() {
  inSettingsMenu = false;
  Serial.println("Exiting Settings Menu with settings saved");
}

void updateSettings() {
  // Your settings update logic here
  // For example, pressing a button to cycle through clutch modes
  if (/* condition to check button press for clutch_mode */) {
    clutch_mode = (clutch_mode + 1) % 3; // Cycle through 0, 1, 2
    Serial.print("Clutch Mode: "); Serial.println(clutch_mode);
  }

  // Similarly, add conditions to check button presses for other settings
}

void processGamepadActions() {
  // Reset gamepad state
  memset(&gp, 0, sizeof(hid_gamepad_report_t));

  // Example: Update gamepad state based on your inputs
  // For instance, setting button states, joystick positions, etc.

  // Update the gamepad state
  blegamepad.report(&gp);
  delay(100); // Delay to simulate gamepad polling rate

  // Add your gamepad logic here, possibly including entering/exiting the settings menu
}
