/**
 * @file StateManager.cpp
 * @brief State machine management implementation.
 */

#include "StateManager.hpp"
#include <Arduino.h>

// ===== Static Variables =====
State StateManager::currentState = IDLE;
uint32_t StateManager::lastStateChangeTime = 0;

// ===== State Queries =====

State StateManager::getCurrentState() {
    return currentState;
}

const char* StateManager::getStateName(State state) {
    switch (state) {
        case IDLE:
            return "IDLE";
        case USER_AUTHENTICATION:
            return "USER_AUTHENTICATION";
        case ROBOT_AUTHENTICATION:
            return "ROBOT_AUTHENTICATION";
        case TIME_SYNCHRONIZATION:
            return "TIME_SYNCHRONIZATION";
        case START:
            return "START";
        case FINISH:
            return "FINISH";
        case COUNT_RESULT:
            return "COUNT_RESULT";
        case JUDGE_CONFIRMATION:
            return "JUDGE_CONFIRMATION";
        case CONFIRMATION:
            return "CONFIRMATION";
        case RESET_WAITING:
            return "RESET_WAITING";
        case RESETING:
            return "RESETING";
        default:
            return "UNKNOWN";
    }
}

String StateManager::getCurrentStateDescription() {
    switch (currentState) {
        case IDLE:
            return "Waiting for competitor";
        case USER_AUTHENTICATION:
            return "Scan RFID card";
        case ROBOT_AUTHENTICATION:
            return "Scan robot QR code";
        case TIME_SYNCHRONIZATION:
            return "Syncing with paired gate";
        case START:
            return "Ready for run start";
        case FINISH:
            return "Measuring finish";
        case COUNT_RESULT:
            return "Calculating time";
        case JUDGE_CONFIRMATION:
            return "Waiting for judge approval";
        case CONFIRMATION:
            return "Confirm result";
        case RESET_WAITING:
            return "Requesting reset";
        case RESETING:
            return "Resetting system";
        default:
            return "Unknown state";
    }
}

// ===== State Transitions =====

void StateManager::transitionTo(State newState, const char* logMessage) {
    if (currentState != newState) {
        currentState = newState;
        lastStateChangeTime = millis();
        
        Serial.print("STATE CHANGE: ");
        Serial.print(getStateName(currentState));
        
        if (logMessage) {
            Serial.print(" - ");
            Serial.print(logMessage);
        }
        Serial.println();
    }
}

bool StateManager::waitForStateTransition(unsigned long delayMs) {
    static uint32_t previousTime = 0;
    static bool isWaiting = false;
    
    uint32_t currentTime = millis();
    
    if (!isWaiting) {
        previousTime = currentTime;
        isWaiting = true;
        return false;
    }
    
    if (currentTime - previousTime >= delayMs) {
        previousTime = currentTime;
        isWaiting = false;
        return true;
    }
    
    return false;
}

// ===== State Reset =====

void StateManager::reset() {
    currentState = IDLE;
    lastStateChangeTime = millis();
    Serial.println("State machine reset to IDLE");
}

// ===== Logging =====

void StateManager::logCurrentState() {
    Serial.print("[STATE] ");
    Serial.print(getStateName(currentState));
    Serial.print(" - ");
    Serial.println(getCurrentStateDescription());
}
