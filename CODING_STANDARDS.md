# Coding Standards - LF Gate Project

This document outlines the professional coding standards used in the LF Gate Project to ensure code quality, maintainability, and consistency across the codebase.

## 📋 Table of Contents
1. [Naming Conventions](#naming-conventions)
2. [Code Organization](#code-organization)
3. [Documentation](#documentation)
4. [Constants Management](#constants-management)
5. [Error Handling](#error-handling)
6. [Code Style](#code-style)

---

## Naming Conventions

### Variables and Functions
- Use **camelCase** for all variables and functions
- Use descriptive names that clearly indicate purpose
- Avoid single-letter variables except for loop counters

**Good Examples:**
```cpp
uint32_t syncCounter;
bool userCardReceived;
void synchronizeTimeTransmitter();
```

**Bad Examples:**
```cpp
uint32_t sc;  // Too abbreviated
bool u_card_received;  // Mixed snake_case
void SyncTime();  // PascalCase for function
```

### Constants
- Use **UPPERCASE_WITH_UNDERSCORES** for constants
- All constants should be defined in `include/constants.hpp`
- Group related constants with comments

**Good Example:**
```cpp
// ========== Display Configuration ==========
#define DISPLAY_SPEED_FAST 20
#define DISPLAY_SPEED_MEDIUM 50
#define DISPLAY_SPEED_SLOW 75
```

### Structures and Enums
- Use **PascalCase** for struct/enum type names
- Use **camelCase** for struct member names
- Use **UPPERCASE** for enum values

**Good Example:**
```cpp
struct Measurement {
    uint32_t counterMinutes;     // camelCase
    bool syncSuccess;
};

enum State {
    IDLE,
    USER_AUTHENTICATION,
    TIME_SYNCHRONIZATION
};
```

### Classes and Modules
- Use **PascalCase** for class/module names
- Prefix private members with underscore (when applicable)

---

## Code Organization

### File Structure
```cpp
// 1. File header with description
/**
 * @file filename.cpp
 * @brief Short description of file purpose.
 */

// 2. Include guards (for .hpp files)
#ifndef FILENAME_HPP
#define FILENAME_HPP

// 3. Includes
#include <Arduino.h>
#include "constants.hpp"

// 4. Defines and constants (if not in constants.hpp)

// 5. Forward declarations and typedefs

// 6. Global variables

// 7. Function implementations or class definitions

#endif  // FILENAME_HPP
```

### Function/Code Sections
Use clear section headers to organize code:

```cpp
// ========== Interrupt Handlers ==========

// ========== Input Processing Functions ==========

// ========== State Machine Control ==========

// ========== Radio Communication ==========

// ========== Server Communication Functions ==========
```

---

## Documentation

### JSDoc-Style Comments
All functions and structures must have documentation comments:

```cpp
/**
 * @brief Synchronizes time as transmitter (start gate).
 * Performs 2-way handshake with receiver to measure clock offset.
 * 
 * @param maxRetries Maximum number of retry attempts
 * @param waitDuration Time to wait between retries (microseconds)
 * @return True if successful, false on timeout/error
 * 
 * @details
 * The synchronization process:
 * 1. Send request with local timestamp
 * 2. Receive response with peer timestamp
 * 3. Calculate and store time difference
 */
bool synchronizeTimeTransmitter(uint8_t maxRetries, unsigned long waitDuration);
```

### Inline Comments
- Use inline comments sparingly - code should be self-documenting
- Explain "why", not "what"
- Maximum 80 characters per line

**Good:**
```cpp
// Account for NRF communication delay
currMeasurement.timeDifference -= NRF_DELAY_OFFSET;
```

**Bad:**
```cpp
// Subtract 167 from the time difference because NRF is weird
currMeasurement.timeDifference -= 167;
```

### Structure Documentation
```cpp
/**
 * @struct Measurement
 * @brief Current competition run data and timestamps.
 * 
 * Contains all information for the active measurement cycle,
 * including participant data, timestamps, and server responses.
 */
struct Measurement {
    /// RFID card code of competitor
    String playerCardCode = "";
    
    /// Display name of competitor
    String playerName = "";
};
```

---

## Constants Management

### Location
All constants must be defined in `include/constants.hpp`:

```cpp
#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// ========== Pin Configuration ==========
#define LED_RED 12
#define LED_GREEN 13

// ========== Timing and Delays ==========
#define BUTTON_SKIP_TIME 500
#define SERIAL_BAUD_RATE 115200

#endif  // CONSTANTS_HPP
```

### Benefits
- Single source of truth for all project constants
- Easy to modify values without searching code
- Reduces magic numbers in source files
- Improves maintainability and readability

### Avoid Magic Numbers
**Bad:**
```cpp
if (millis() - lastClickTime > 500) { ... }  // What is 500?
radio.setChannel(100);  // What frequency?
```

**Good:**
```cpp
if (millis() - lastClickTime > BUTTON_SKIP_TIME) { ... }
radio.setChannel(RF24_CHANNEL);
```

---

## Error Handling

### Error Messages
Use descriptive, actionable error messages:

```cpp
// Good: Clear context and remediation
Serial.println("ERROR: Failed to read RFID card. Ensure antenna is enabled.");

// Bad: Vague error
Serial.println("Error in card reading");

// Bad: No context
Serial.println("Fail");
```

### Return Values
- Return `true` for success, `false` for failure
- Document return values in function comments
- Validate inputs and communicate failures

```cpp
/**
 * @return True if synchronization successful, false on timeout/error
 */
bool synchronizeTimeTransmitter() {
    if (currMeasurement.syncTime == 0) {
        Serial.println("ERROR: Invalid sync timestamp");
        return false;
    }
    return true;
}
```

---

## Code Style

### Indentation
- Use **4 spaces** per indentation level
- Never use tabs
- Consistent indentation within blocks

### Braces
- Use K&R style (opening brace on same line)
- Always use braces, even for single-statement blocks

```cpp
// Correct
if (gate.active) {
    handleGateState();
}

// Incorrect
if (gate.active)
    handleGateState();
```

### Line Length
- Target maximum 100 characters per line
- Break long lines logically

```cpp
// Good - broken into logical parts
FirebaseJson json;
json.set("serial_number", GATE_SERIAL_NUMBER);
json.set("category_id", gate.categoryID);

// Avoid
json.set("serial_number", GATE_SERIAL_NUMBER); json.set("category_id", gate.categoryID);
```

### Spacing
- One space around operators: `a + b`, not `a+b`
- No space before semicolon: `delay(100);`, not `delay(100) ;`
- One blank line between functions

```cpp
// Good spacing
uint64_t currentTime = millis();
if (currentTime - lastTime > INTERVAL) {
    processEvent();
    lastTime = currentTime;
}

void nextFunction() { ... }
```

### Comments
- Use `//` for single-line comments
- Use `/* */` for multi-line comments only when necessary
- Keep comments aligned and readable

```cpp
// Single line comment
int counter = 0;  // Inline comment

/*
 * Multi-line comment explaining complex logic:
 * - First point
 * - Second point
 */
```

---

## Type Guidelines

### Integer Types
- Use `uint32_t` for timestamps (milliseconds)
- Use `uint64_t` for microsecond timestamps
- Use `uint8_t` for IDs and flags
- Use `int16_t` for signed offsets

### String Types
- Use Arduino `String` class for dynamic text
- Use `const char*` for compile-time constants
- Use `uint8_t[]` for binary data

### Boolean Types
- Use `bool` instead of `int` for logical flags
- Initialize boolean variables explicitly

```cpp
// Good
bool isConnected = false;
bool syncSuccess = true;

// Avoid
int isConnected = 0;  // Less clear
bool syncSuccess;  // Uninitialized
```

---

## Version Control & Commits

### Commit Messages
- Use clear, descriptive commit messages
- Reference issues when applicable
- Start with imperative verb: "Add", "Fix", "Refactor", "Document"

```
Good commit:
- "Add time synchronization for finish gate"
- "Fix RFID card reader initialization"
- "Document interrupt handlers"

Bad commit:
- "updated stuff"
- "fixed bugs"
- "changes"
```

---

## Code Review Checklist

Before submitting code, verify:

- [ ] Follows naming conventions (camelCase for variables)
- [ ] All functions have JSDoc comments
- [ ] Constants are in `include/constants.hpp`
- [ ] No magic numbers in code
- [ ] Error handling with meaningful messages
- [ ] Consistent indentation (4 spaces)
- [ ] Meaningful variable names
- [ ] Code compiles without warnings
- [ ] Consistent with rest of codebase

---

## References
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [Arduino Best Practices](https://github.com/arduino/Arduino/wiki/Development)
- [Doxygen Documentation](https://www.doxygen.nl/)
- Arduino Serial output: Use meaningful messages

---

## Questions or Suggestions?
If you have questions about these standards or suggestions for improvements, please open an issue in the repository.
