# Communication Modules Extraction Summary

This document describes the extraction of communication code from `src/main.cpp` into two new dedicated modules: **RadioCommunication** and **ServerCommunication**.

## Overview

The refactoring improves code organization and maintainability by separating concerns into focused modules:
- **RadioCommunication**: Handles NRF24L01 wireless communication
- **ServerCommunication**: Handles WiFi-based server communication

Both modules use static methods (no instantiation required) for ease of use throughout the application.

## Files Created

### 1. RadioCommunication Module

#### `src/RadioCommunication.hpp`
Header file containing the `RadioCommunication` class with comprehensive JSDoc documentation.

**Public Static Methods:**
- `listenForSignals()` - Receives and processes radio commands (sync, start, finish)
- `synchronizeTimeTransmitter()` - Performs time synchronization as transmitter (start gate)
- `synchronizeTimeReceiver(uint32_t)` - Performs time synchronization as receiver (finish gate)
- `sendStartCommand()` - Transmits start signal with time compensation
- `sendFinishCommand()` - Transmits finish time to paired gate
- `sendTime(uint32_t)` - Sends timestamp to paired gate
- `waitForRadio(unsigned long)` - Waits for radio interrupt with timeout
- `requestTimeWithRetries(uint8_t, unsigned long)` - Requests time with retry mechanism

**Global Dependencies (extern):**
- `RF24 radio` - NRF24L01 radio module instance
- `Measurement currMeasurement` - Current measurement data with interrupt flags

#### `src/RadioCommunication.cpp`
Implementation file containing all radio communication logic extracted from main.cpp.

**Key Features:**
- Time synchronization with 2-way handshake
- Automatic time difference calculation
- Retry mechanism for reliable transmission
- Support for RF24 command codes:
  - `RF24_CMD_TIME_SYNC` (0x01) - Time synchronization request
  - `RF24_CMD_RESEND_TIME` (0x02) - Resend time request
  - `RF24_CMD_START` (0x03) - Start gate signal
  - `RF24_CMD_FINISH` (0x04) - Finish gate signal
  - `123456789` - Synchronization time response marker

### 2. ServerCommunication Module

#### `src/ServerCommunication.hpp`
Header file containing the `ServerCommunication` class with comprehensive JSDoc documentation.

**Public Static Methods:**
- `sendJsonMessage(const char*, FirebaseJson&)` - Sends JSON formatted message to server
- `handleServerResponse()` - Parses and processes server responses
- `checkConnection()` - Monitors WiFi and server connection status

**Global Dependencies (extern):**
- `WiFiClient client` - WiFi connection to server
- `Gate gate` - Current gate configuration
- `Gate newGateSettings` - Pending gate settings from server
- `bool newSettingsAvailable` - Flag for pending settings
- `Measurement currMeasurement` - Current measurement data
- `State currentState` - Current state machine state

#### `src/ServerCommunication.cpp`
Implementation file containing all server communication logic.

**Key Features:**
- JSON message formatting: `{ "co": command, "da": data }`
- Line-based message parsing (newline terminated)
- Support for server commands:
  - `SETTINGS` - Gate configuration update
  - `USER` - Competitor information
  - `ROBOT` - Robot data
  - `JUDGE` - Judge information (if required)
  - `RETRY_JUDGE` - Judge authentication retry
  - `RETRY_SCORE` - Score submission retry
  - `RESET` - System reset command

**Private Helper Method:**
- `applyGateSettings(FirebaseJson&)` - Parses and applies gate settings

## Changes to main.cpp

### Added Includes
```cpp
#include "RadioCommunication.hpp"
#include "ServerCommunication.hpp"
```

### Modified Function Implementations

All the following functions now delegate to the appropriate module class:

**Server Communication Functions:**
```cpp
void sendJsonMessage(const char* command, FirebaseJson& data) {
    ServerCommunication::sendJsonMessage(command, data);
}

bool handleServerResponse() {
    return ServerCommunication::handleServerResponse();
}

void checkConnection() {
    ServerCommunication::checkConnection();
}
```

**Radio Communication Functions:**
```cpp
void listenForSignals() {
    RadioCommunication::listenForSignals();
}

bool synchronizeTimeTransmitter() {
    return RadioCommunication::synchronizeTimeTransmitter();
}

bool synchronizeTimeReceiver(uint32_t lastReceivedInterruptTime) {
    return RadioCommunication::synchronizeTimeReceiver(lastReceivedInterruptTime);
}

bool sendTime(uint32_t time) {
    return RadioCommunication::sendTime(time);
}

bool waitForRadio(unsigned long timeout) {
    return RadioCommunication::waitForRadio(timeout);
}

uint32_t requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration) {
    return RadioCommunication::requestTimeWithRetries(maxRetries, waitDuration);
}

bool sendStartCommand() {
    return RadioCommunication::sendStartCommand();
}

bool sendFinishCommand() {
    return RadioCommunication::sendFinishCommand();
}
```

These wrapper functions maintain backward compatibility with the existing codebase, allowing the state machine logic in main.cpp to remain unchanged.

## Constants Used

All RF24 command codes and timing constants are defined in `include/constants.hpp`:

```cpp
#define RF24_CMD_TIME_SYNC 0x01
#define RF24_CMD_RESEND_TIME 0x02
#define RF24_CMD_START 0x03
#define RF24_CMD_FINISH 0x04
#define RF24_CMD_SYNC_RESPONSE 123456789
#define NRF_DELAY_OFFSET 167
```

## Code Logic Preserved

All original logic and error handling has been preserved:

### RadioCommunication
- ✅ Interrupt-based synchronization handshake
- ✅ Time difference calculation with offset compensation
- ✅ Retry mechanism with configurable attempts
- ✅ Radio state management (startListening/stopListening)
- ✅ All command types and their processing

### ServerCommunication
- ✅ JSON message formatting with proper structure
- ✅ Line-based message parsing with trim
- ✅ All server command handlers (SETTINGS, USER, ROBOT, JUDGE, etc.)
- ✅ WiFi/Server reconnection logic
- ✅ LED status indication for connection state
- ✅ Settings application parsing

## Benefits of This Refactoring

1. **Separation of Concerns**: Radio and server communication logic are now isolated
2. **Improved Maintainability**: Easier to locate and modify specific communication code
3. **Better Code Organization**: Clear module boundaries and responsibilities
4. **Comprehensive Documentation**: Detailed JSDoc comments for all methods
5. **Static Interface**: No object instantiation needed; clean usage pattern
6. **Preserved Compatibility**: Wrapper functions maintain existing API
7. **Reduced main.cpp Complexity**: ~140 lines of communication code removed

## Future Enhancements

These modules are now ready for:
- Unit testing of communication logic
- Timeout and error handling improvements
- Logging and debugging enhancements
- Support for additional RF24 commands
- Mock implementations for testing without hardware

## Verification

To verify the refactoring:

1. **Compilation**: Build with `pio run` to confirm no syntax errors
2. **Functionality**: Test time synchronization between gates
3. **Server Communication**: Verify JSON message format and parsing
4. **Connection Management**: Test WiFi/server reconnection behavior
5. **State Machine**: Verify state transitions work as expected with delegated functions

## Notes

- Both modules use `extern` declarations for global objects defined in main.cpp
- The modules require `main.hpp` for data structures (Measurement, Gate, State, DataPackage)
- Constants are centralized in `include/constants.hpp`
- All original comments and logging statements have been preserved
- Function signatures remain unchanged for backward compatibility
