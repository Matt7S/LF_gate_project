/**
 * @file HardwareControl.cpp
 * @brief Hardware control implementation.
 */

#include "HardwareControl.hpp"
#include <Arduino.h>
#include <cstdint>
#include "P10Display.hpp"

// ===== Static Variables =====
extern P10Display display;
uint32_t HardwareControl::lastButtonTime = 0;
uint8_t HardwareControl::lastButtonStates = 0xFF;

// ===== Initialization =====

void HardwareControl::initialize() {
    // Configure button inputs with pull-up resistors
    pinMode(BUTTON_ACCEPT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_CANCEL_PIN, INPUT_PULLUP);
    pinMode(BUTTON_FORFEIT_PIN, INPUT_PULLUP);
    
    // Configure LED outputs
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    
    // Configure IR sensor input
    pinMode(IR_IRQ_PIN, INPUT_PULLUP);
    
    // Initialize LED states
    setRedLED(true);    // Red ON (disconnected)
    setGreenLED(false); // Green OFF
}

// ===== LED Control =====

void HardwareControl::setGreenLED(bool on) {
    digitalWrite(LED_GREEN, on ? LOW : HIGH);
}

void HardwareControl::setRedLED(bool on) {
    digitalWrite(LED_RED, on ? LOW : HIGH);
}

void HardwareControl::setConnectionStatus(bool connected) {
    if (connected) {
        setGreenLED(true);   // Green ON = connected
        setRedLED(false);    // Red OFF
    } else {
        setGreenLED(false);  // Green OFF
        setRedLED(true);     // Red ON = disconnected
    }
}

// ===== Button Input =====

bool HardwareControl::isAcceptButtonPressed(uint32_t debounceMs) {
    uint32_t currentTime = millis();
    if (currentTime - lastButtonTime < debounceMs) {
        return false;
    }
    
    if (digitalRead(BUTTON_ACCEPT_PIN) == LOW) {
        lastButtonTime = currentTime;
        return true;
    }
    return false;
}

bool HardwareControl::isCancelButtonPressed(uint32_t debounceMs) {
    uint32_t currentTime = millis();
    if (currentTime - lastButtonTime < debounceMs) {
        return false;
    }
    
    if (digitalRead(BUTTON_CANCEL_PIN) == LOW) {
        lastButtonTime = currentTime;
        return true;
    }
    return false;
}

bool HardwareControl::isForfeitButtonPressed(uint32_t debounceMs) {
    uint32_t currentTime = millis();
    if (currentTime - lastButtonTime < debounceMs) {
        return false;
    }
    
    if (digitalRead(BUTTON_FORFEIT_PIN) == LOW) {
        lastButtonTime = currentTime;
        return true;
    }
    return false;
}

bool HardwareControl::areAllButtonsPressed() {
    uint32_t currentTime = millis();
    if (currentTime - lastButtonTime < BUTTON_SKIP_TIME) {
        return false;
    }
    
    bool acceptPressed = digitalRead(BUTTON_ACCEPT_PIN) == LOW;
    bool cancelPressed = digitalRead(BUTTON_CANCEL_PIN) == LOW;
    bool forfeitPressed = digitalRead(BUTTON_FORFEIT_PIN) == LOW;
    
    if (acceptPressed && cancelPressed && forfeitPressed) {
        lastButtonTime = currentTime;
        return true;
    }
    return false;
}

// ===== IR Sensor =====

static uint32_t irSensorLastTime = 0;
static bool irSensorActive = false;

bool HardwareControl::monitorIRSensor(uint32_t timeout) {
    if (digitalRead(IR_IRQ_PIN) == LOW) {
        if (!irSensorActive) {
            irSensorLastTime = millis();
            irSensorActive = true;
        } else if (millis() - irSensorLastTime > timeout) {
            // Sensor blocked for too long - error condition
            // Could trigger warning on display
        }
    } else {
        irSensorActive = false;
    }
    return true;
}

void HardwareControl::resetIRMonitor() {
    irSensorActive = false;
    irSensorLastTime = 0;
}

// ===== Display Control =====

void HardwareControl::updateDisplay() {
    display.updateDisplay();
}

void HardwareControl::displayMessage(uint8_t line, const String& message,
                                     uint8_t speed, bool scrolling) {
    if (scrolling) {
        display.setLineDynamic(line, message, true, speed, true);
    } else {
        display.setLineStatic(line, message, 1, true);
    }
}

void HardwareControl::displayTimer(uint8_t line, uint64_t startTime, uint32_t divisor) {
    display.setTimer(line, 0, startTime, divisor);
}

void HardwareControl::clearDisplay() {
    display.clearTopPart();
}

void HardwareControl::startBlinking(uint8_t line, uint32_t interval) {
    display.startBlinking(line, interval);
}

void HardwareControl::stopBlinking(uint8_t line) {
    display.stopBlinking(line);
}
