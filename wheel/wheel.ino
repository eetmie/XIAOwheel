

#define BOUNCE_WITH_PROMPT_DETECTION // Make button state changes available immediately

#include <Arduino.h>
#include <Bounce2.h>    // https://github.com/thomasfredericks/Bounce2
#include <BleGamepad.h> // https://github.com/lemmingDev/ESP32-BLE-Gamepad

#define numOfButtons 6

Bounce debouncers[numOfButtons];
BleGamepad bleGamepad("Paskaratti0.1", "Eetu Miettinen", 100);

byte buttonPins[numOfButtons] = {25, 26, 32, 33, 27, 14};
byte physicalButtons[numOfButtons] = {1, 2, 3, 4, 5, 6};

void setup()
{
    for (byte currentPinIndex = 0; currentPinIndex < numOfButtons; currentPinIndex++)
    {
        pinMode(buttonPins[currentPinIndex], INPUT_PULLUP);

        debouncers[currentPinIndex] = Bounce();
        debouncers[currentPinIndex].attach(buttonPins[currentPinIndex]); // After setting up the button, setup the Bounce instance :
        debouncers[currentPinIndex].interval(3);
    }

    BleGamepadConfiguration bleGamepadConfig;
    bleGamepadConfig.setButtonCount(numOfButtons);
    bleGamepadConfig.setAutoReport(false);
    bleGamepad.begin(&bleGamepadConfig);

    // changing bleGamepadConfig after the begin function has no effect, unless you call the begin function again

    Serial.begin(115200);
}

void loop()
{
    if (bleGamepad.isConnected())
    {
        bool sendReport = false;

        for (byte currentIndex = 0; currentIndex < numOfButtons; currentIndex++)
        {
            debouncers[currentIndex].update();

            if (debouncers[currentIndex].fell())
            {
                bleGamepad.press(physicalButtons[currentIndex]);
                sendReport = true;
                Serial.print("Button ");
                Serial.print(physicalButtons[currentIndex]);
                Serial.println(" pushed.");
            }
            else if (debouncers[currentIndex].rose())
            {
                bleGamepad.release(physicalButtons[currentIndex]);
                sendReport = true;
                Serial.print("Button ");
                Serial.print(physicalButtons[currentIndex]);
                Serial.println(" released.");
            }
        }

        if (sendReport)
        {
            bleGamepad.sendReport();
        }

        // delay(20);	// (Un)comment to remove/add delay between loops
    }
}
