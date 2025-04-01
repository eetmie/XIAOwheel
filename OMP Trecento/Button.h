#ifndef Button_h
#define Button_h

#include <Arduino.h>

class Button {
public:
    int pin;
    volatile bool state;
    volatile bool lastState;
    volatile unsigned long lastDebounceTime;
    static const unsigned long debounceDelay = 2;

    typedef void (*CallbackFunction)(bool, int); // Use int for flexibility in callback parameters

    CallbackFunction onChange;
    int callbackParam; // This int can be used to pass any type of identifier or index

    Button(int pin, CallbackFunction callback, int callbackParam)
        : pin(pin), state(false), lastState(false), lastDebounceTime(0),
          onChange(callback), callbackParam(callbackParam) {
        pinMode(pin, INPUT_PULLUP);
    }

    void update() {
        // Standard reading method
        bool reading = digitalRead(pin);
        
        if (reading != lastState) {
            lastDebounceTime = millis();
        }
        
        if ((millis() - lastDebounceTime) > debounceDelay) {
            if (reading != state) {
                state = reading;
                // For pull-up configuration, a LOW reading means the button is pressed.
                bool interpretedState = !state;  // Invert the state
                if (onChange != nullptr) {
                    onChange(interpretedState, callbackParam);
                }
            }
        }
        lastState = reading;
    }
};

#endif