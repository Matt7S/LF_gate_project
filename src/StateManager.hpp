/**
 * @file StateManager.hpp
 * @brief Competition state machine management.
 *
 * Manages the 11-state finite state machine:
 * IDLE → USER_AUTH → ROBOT_AUTH → TIME_SYNC → START → FINISH → COUNT_RESULT
 * → JUDGE_CONFIRMATION → CONFIRMATION → RESET_WAITING → RESETING → IDLE
 */

#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <Arduino.h>
#include "main.hpp"
#include "constants.hpp"

/**
 * @class StateManager
 * @brief Manages the competition state machine.
 */
class StateManager {
public:
    /**
     * @brief Get current state.
     * @return Current State enum value
     */
    static State getCurrentState();

    /**
     * @brief Transition to new state.
     * @param newState Target state
     * @param logMessage Optional message to log on state change
     */
    static void transitionTo(State newState, const char* logMessage = nullptr);

    /**
     * @brief Get state name as string.
     * @param state State enum value
     * @return Human-readable state name
     */
    static const char* getStateName(State state);

    /**
     * @brief Check if waiting for state transition delay.
     * Used to implement delays between state changes.
     * @param delayMs Duration to wait in milliseconds
     * @return True when delay has elapsed
     */
    static bool waitForStateTransition(unsigned long delayMs);

    /**
     * @brief Reset state machine to IDLE.
     * Clears all measurement data.
     */
    static void reset();

    /**
     * @brief Log current state to serial output.
     */
    static void logCurrentState();

    /**
     * @brief Get description of current state.
     * @return Human-readable description
     */
    static String getCurrentStateDescription();

private:
    // Private constructor (static methods only)
    StateManager() = delete;

    static State currentState;
    static uint32_t lastStateChangeTime;
};

#endif  // STATE_MANAGER_HPP
