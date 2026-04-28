/**
 * @file HardwareControl.hpp
 * @brief Hardware control interface for LEDs, buttons, and sensors.
 *
 * Manages low-level hardware operations including:
 * - Status LED indicators (Red/Green)
 * - Physical control buttons (Accept, Cancel, Forfeit)
 * - IR sensor monitoring
 * - Display updates
 */

#ifndef HARDWARE_CONTROL_HPP
#define HARDWARE_CONTROL_HPP

#include <Arduino.h>
#include "constants.hpp"

/**
 * @class HardwareControl
 * @brief Singleton class managing all hardware interactions.
 */
class HardwareControl {
public:
    /**
     * @brief Initialize all hardware components.
     * Sets up GPIO, configures interrupts, initializes displays.
     */
    static void initialize();

    // ===== LED Control =====
    /**
     * @brief Set green LED status.
     * @param on True to turn on, false to turn off
     */
    static void setGreenLED(bool on);

    /**
     * @brief Set red LED status.
     * @param on True to turn on, false to turn off
     */
    static void setRedLED(bool on);

    /**
     * @brief Indicate connection status with LED.
     * Green ON = Connected, Red ON = Disconnected
     * @param connected True if connected to server
     */
    static void setConnectionStatus(bool connected);

    // ===== Button Input =====
    /**
     * @brief Read accept button state with debouncing.
     * @param debounceMs Debounce duration in milliseconds
     * @return True if button was pressed
     */
    static bool isAcceptButtonPressed(uint32_t debounceMs = BUTTON_SKIP_TIME);

    /**
     * @brief Read cancel button state with debouncing.
     * @param debounceMs Debounce duration in milliseconds
     * @return True if button was pressed
     */
    static bool isCancelButtonPressed(uint32_t debounceMs = BUTTON_SKIP_TIME);

    /**
     * @brief Read forfeit button state with debouncing.
     * @param debounceMs Debounce duration in milliseconds
     * @return True if button was pressed
     */
    static bool isForfeitButtonPressed(uint32_t debounceMs = BUTTON_SKIP_TIME);

    /**
     * @brief Check if all three buttons pressed simultaneously.
     * Used for emergency reset.
     * @return True if all buttons pressed
     */
    static bool areAllButtonsPressed();

    // ===== IR Sensor =====
    /**
     * @brief Monitor IR sensor for obstructions.
     * @param timeout Timeout in milliseconds
     * @return True if sensor state valid
     */
    static bool monitorIRSensor(uint32_t timeout);

    /**
     * @brief Reset IR sensor monitoring state.
     */
    static void resetIRMonitor();

    // ===== Display Control =====
    /**
     * @brief Update display (called from Core 1).
     * Must be called frequently to refresh LED matrix.
     */
    static void updateDisplay();

    /**
     * @brief Show status message on display.
     * @param line Display line (0 or 1)
     * @param message Message text
     * @param speed Scroll speed (DISPLAY_SPEED_FAST, etc.)
     * @param scrolling Enable text scrolling
     */
    static void displayMessage(uint8_t line, const String& message, 
                               uint8_t speed = DISPLAY_SPEED_FAST, 
                               bool scrolling = true);

    /**
     * @brief Display countdown timer on display.
     * @param line Display line
     * @param startTime Start timestamp in microseconds
     * @param divisor Display divisor (usually 1000 for seconds)
     */
    static void displayTimer(uint8_t line, uint64_t startTime, uint32_t divisor = 1000);

    /**
     * @brief Clear display.
     */
    static void clearDisplay();

    /**
     * @brief Start blinking animation.
     * @param line Display line
     * @param interval Blink interval in milliseconds
     */
    static void startBlinking(uint8_t line, uint32_t interval = DISPLAY_BLINK_INTERVAL);

    /**
     * @brief Stop blinking animation.
     * @param line Display line
     */
    static void stopBlinking(uint8_t line);

private:
    // Private constructor (singleton)
    HardwareControl() = delete;

    // Button debouncing helpers
    static uint32_t lastButtonTime;
    static uint8_t lastButtonStates;
};

#endif  // HARDWARE_CONTROL_HPP
