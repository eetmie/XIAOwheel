#define BOUNCE_WITH_PROMPT_DETECTION // Make button state changes available immediately

#include <Arduino.h>
#include <Bounce2.h>
#include <BleGamepad.h>
#include <EEPROM.h>

#define numOfButtons 6
#define BUZZER_PIN 9 //beep(3, 500, 440); // 3 beeps, each 500ms long, at a frequency of 440Hz (A4 note)

struct Button {
    byte index;
    byte pin;
};

// find the real positions!!
Button buttons[numOfButtons] = {
    {0, 25}, // left_paddle (Settings Menu)
    {1, 26}, // right_paddle (Exit Settings Menu)
    {2, 32}, // Lup_button (Cycle Clutch Mode)
    {3, 33}, // Rup_button (Cycle Launch Control Mode)
    {4, 27}, // Ldown_button
    {5, 14}  // Rdown_button (now used to launch the car)
};

Bounce debouncers[numOfButtons];
BleGamepad bleGamepad("Paskaratti0.1", "Eetu Miettinen", 100);

byte physicalButtons[numOfButtons] = {1, 2, 3, 4, 5, 6};

unsigned long buttonPressStartTime = 0;
bool inSettingsMenu = false;

// Settings variables and EEPROM addresses
int clutch_mode = 0;       // 0 disabled, 1 fast, 2 slow
int launchControl_mode = 0;// 0 disabled, 1 setup1, 2 setup2
const int addr1 = 0;       // EEPROM address for clutch_mode
const int addr2 = 1;       // EEPROM address for launchControl_mode

void setup() {
    for (byte currentPinIndex = 0; currentPinIndex < numOfButtons; currentPinIndex++) {
        pinMode(buttons[currentPinIndex].pin, INPUT_PULLUP);

        debouncers[currentPinIndex] = Bounce();
        debouncers[currentPinIndex].attach(buttons[currentPinIndex].pin);
        debouncers[currentPinIndex].interval(3);
    }

    BleGamepadConfiguration bleGamepadConfig;
    bleGamepadConfig.setButtonCount(numOfButtons);
    bleGamepadConfig.setAutoReport(false);
    bleGamepad.begin(&bleGamepadConfig);

    pinMode(BUZZER_PIN, OUTPUT);

    Serial.begin(115200);

    // Read settings from EEPROM
    clutch_mode = EEPROM.read(addr1);
    launchControl_mode = EEPROM.read(addr2);
}

// beep beep
void beep(int beeps = 1, int beepsDelay = 100, int frequency = 440) {
    for (int i = 0; i < beeps; i++) {
        tone(BUZZER_PIN, frequency);
        delay(beepsDelay);
        noTone(BUZZER_PIN);
        delay(beepsDelay);
    }
}

void enterSettingsMenu() {
    Serial.println("Entering Settings Menu");
    inSettingsMenu = true;
}

void exitSettingsMenu() {
    EEPROM.write(addr1, clutch_mode);
    EEPROM.write(addr2, launchControl_mode);
    Serial.println("Exiting Settings Menu with settings saved");
    inSettingsMenu = false;
}

void updateSettings() {
    debouncers[buttons[2].index].update(); // Update Lup_button
    if (debouncers[buttons[2].index].fell()) {
        clutch_mode = (clutch_mode + 1) % 3; // Cycle through 0, 1, 2
        beep(clutch_mode);
    }

    debouncers[buttons[3].index].update(); // Update Rup_button
    if (debouncers[buttons[3].index].fell()) {
        launchControl_mode = (launchControl_mode + 1) % 3;
        beep(launchControl_mode);
    }
}

void loop() {
    if (bleGamepad.isConnected() && !inSettingsMenu) {
        bool sendReport = false;

        for (byte currentIndex = 0; currentIndex < numOfButtons; currentIndex++) {
            debouncers[currentIndex].update();

            byte physicalButtonNumber = currentIndex + 1;

            if (debouncers[currentIndex].fell()) {
                if (buttons[currentIndex].pin == buttons[0].pin) { // Check if it's left_paddle
                    buttonPressStartTime = millis();
                }
                bleGamepad.press(physicalButtonNumber);
                sendReport = true;
                Serial.print("Button ");
                Serial.print(physicalButtonNumber);
                Serial.println(" pushed.");
            } else if (debouncers[currentIndex].rose()) {
                if (buttons[currentIndex].pin == buttons[0].pin && (millis() - buttonPressStartTime >= 2000)) {
                    enterSettingsMenu(); // Enter settings menu if left paddle was held for 2 seconds
                }
                bleGamepad.release(physicalButtonNumber);
                sendReport = true;
                Serial.print("Button ");
                Serial.print(physicalButtonNumber);
                Serial.println(" released.");
            }
        }

        if (sendReport) {
            bleGamepad.sendReport();
        }
    } else if (inSettingsMenu) {
        updateSettings();

        debouncers[buttons[1].index].update(); // Update right paddle for exiting settings
        if (debouncers[buttons[1].index].fell()) {
            exitSettingsMenu();
        }

        debouncers[buttons[5].index].update(); // Update Rdown_button for special exit
        if (debouncers[buttons[5].index].fell()) {
            // Send button press, exit settings menu without saving
            bleGamepad.press(physicalButtons[buttons[5].index]);
            bleGamepad.release(physicalButtons[buttons[5].index]);
            Serial.println("Special Exit from Settings Menu (No Save)");
            inSettingsMenu = false;
        }
    }
}

