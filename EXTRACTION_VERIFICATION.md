# Extracted Communication Functions - Verification Checklist

## RadioCommunication Module

### File: src/RadioCommunication.hpp and src/RadioCommunication.cpp

#### ✅ Method: listenForSignals()
- **Signature**: `static void listenForSignals()`
- **Source**: main.cpp lines 826-873 (original)
- **Location**: RadioCommunication.cpp lines 11-57
- **Responsibility**: Listen for incoming RF24 signals and process commands
- **Commands Handled**: 
  - RF24_CMD_TIME_SYNC (0x01)
  - RF24_CMD_RESEND_TIME (0x02)
  - RF24_CMD_START (0x03)
  - RF24_CMD_FINISH (0x04)

#### ✅ Method: synchronizeTimeTransmitter()
- **Signature**: `static bool synchronizeTimeTransmitter()`
- **Source**: main.cpp lines 883-951 (original)
- **Location**: RadioCommunication.cpp lines 59-116
- **Responsibility**: Transmit sync request, measure clock offset
- **Return**: true on success, false on timeout/error

#### ✅ Method: synchronizeTimeReceiver(uint32_t lastReceivedInterruptTime)
- **Signature**: `static bool synchronizeTimeReceiver(uint32_t)`
- **Source**: main.cpp lines 956-978 (original)
- **Location**: RadioCommunication.cpp lines 118-137
- **Responsibility**: Respond to sync request with received timestamp
- **Return**: true on success, false on transmission error

#### ✅ Method: sendTime(uint32_t time)
- **Signature**: `static bool sendTime(uint32_t)`
- **Source**: main.cpp lines 981-996 (original)
- **Location**: RadioCommunication.cpp lines 139-151
- **Responsibility**: Send timestamp to paired gate
- **Return**: true on success, false on error

#### ✅ Method: waitForRadio(unsigned long timeout)
- **Signature**: `static bool waitForRadio(unsigned long)`
- **Source**: main.cpp lines 1000-1005 (original)
- **Location**: RadioCommunication.cpp lines 153-158
- **Responsibility**: Wait for RF24 interrupt with timeout
- **Return**: always true after timeout

#### ✅ Method: requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration)
- **Signature**: `static uint32_t requestTimeWithRetries(uint8_t, unsigned long)`
- **Source**: main.cpp lines 1009-1047 (original)
- **Location**: RadioCommunication.cpp lines 160-206
- **Responsibility**: Request time with automatic retry mechanism
- **Return**: time value or 0 if all retries exhausted

#### ✅ Method: sendStartCommand()
- **Signature**: `static bool sendStartCommand()`
- **Source**: main.cpp lines 1050-1063 (original)
- **Location**: RadioCommunication.cpp lines 208-221
- **Responsibility**: Send start gate signal with time compensation
- **Return**: true on success, false on error

#### ✅ Method: sendFinishCommand()
- **Signature**: `static bool sendFinishCommand()`
- **Source**: main.cpp lines 1066-1080 (original)
- **Location**: RadioCommunication.cpp lines 223-236
- **Responsibility**: Send finish time to start gate
- **Return**: true on success, false on error

---

## ServerCommunication Module

### File: src/ServerCommunication.hpp and src/ServerCommunication.cpp

#### ✅ Method: sendJsonMessage(const char* command, FirebaseJson& data)
- **Signature**: `static void sendJsonMessage(const char*, FirebaseJson&)`
- **Source**: main.cpp lines 667-683 (original)
- **Location**: ServerCommunication.cpp lines 12-28
- **Responsibility**: Format and send JSON message to server
- **Message Format**: `{ "co": command, "da": data }\n`

#### ✅ Method: handleServerResponse()
- **Signature**: `static bool handleServerResponse()`
- **Source**: main.cpp lines 692-817 (original)
- **Location**: ServerCommunication.cpp lines 30-143
- **Responsibility**: Parse JSON responses and update state
- **Commands Handled**:
  - SETTINGS: Update gate configuration
  - USER: Store competitor information
  - ROBOT: Store robot data
  - JUDGE: Store judge information
  - RETRY_JUDGE: Judge retry request
  - RETRY_SCORE: Score retry request
  - RESET: System reset command
- **Return**: true on success, false on error

#### ✅ Private Method: applyGateSettings(FirebaseJson& daJson)
- **Signature**: `static void applyGateSettings(FirebaseJson&)`
- **Source**: main.cpp lines 730-757 (original - inline in handleServerResponse)
- **Location**: ServerCommunication.cpp lines 162-210
- **Responsibility**: Extract and apply gate settings from JSON
- **Fields Extracted**:
  - pair_id, active, start_gate_id, finish_gate_id
  - category_id, stage_id
  - start_nrf, finish_nrf (NRF24 addresses)
  - requires_user_card, requires_user_qr_code, requires_judge_confirmation
  - category_name, stage_name, gate_type

#### ✅ Method: checkConnection()
- **Signature**: `static void checkConnection()`
- **Source**: main.cpp lines 1146-1162 (original)
- **Location**: ServerCommunication.cpp lines 145-159
- **Responsibility**: Monitor and maintain WiFi/server connection
- **Actions**: 
  - Reconnect WiFi if disconnected
  - Reconnect to server if disconnected
  - Request settings on reconnection
  - Control LED status

---

## Main.cpp Wrapper Functions

### File: src/main.cpp

All wrapper functions delegate to the module classes while maintaining backward compatibility.

#### ✅ sendJsonMessage() - Line 666-668
```cpp
void sendJsonMessage(const char* command, FirebaseJson& data) {
    ServerCommunication::sendJsonMessage(command, data);
}
```

#### ✅ handleServerResponse() - Line 674-676
```cpp
bool handleServerResponse() {
    return ServerCommunication::handleServerResponse();
}
```

#### ✅ listenForSignals() - Line 684-686
```cpp
void listenForSignals() {
    RadioCommunication::listenForSignals();
}
```

#### ✅ synchronizeTimeTransmitter() - Line 694-696
```cpp
bool synchronizeTimeTransmitter() {
    return RadioCommunication::synchronizeTimeTransmitter();
}
```

#### ✅ synchronizeTimeReceiver() - Line 705-707
```cpp
bool synchronizeTimeReceiver(uint32_t lastReceivedInterruptTime) {
    return RadioCommunication::synchronizeTimeReceiver(lastReceivedInterruptTime);
}
```

#### ✅ sendTime() - Line 714-716
```cpp
bool sendTime(uint32_t time) {
    return RadioCommunication::sendTime(time);
}
```

#### ✅ waitForRadio() - Line 724-726
```cpp
bool waitForRadio(unsigned long timeout) {
    return RadioCommunication::waitForRadio(timeout);
}
```

#### ✅ requestTimeWithRetries() - Line 734-736
```cpp
uint32_t requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration) {
    return RadioCommunication::requestTimeWithRetries(maxRetries, waitDuration);
}
```

#### ✅ sendStartCommand() - Line 743-745
```cpp
bool sendStartCommand() {
    return RadioCommunication::sendStartCommand();
}
```

#### ✅ sendFinishCommand() - Line 752-754
```cpp
bool sendFinishCommand() {
    return RadioCommunication::sendFinishCommand();
}
```

#### ✅ checkConnection() - Line 824-826
```cpp
void checkConnection() {
    ServerCommunication::checkConnection();
}
```

---

## Code Reduction Summary

| Category | Lines | Extracted |
|----------|-------|-----------|
| RadioCommunication Functions | ~160 | ✅ |
| ServerCommunication Functions | ~130 | ✅ |
| Total Extracted | ~290 | ✅ |
| Total Created (with docs) | ~520 | ✅ |
| Net in main.cpp | -290 | ✅ |

---

## Data Types Used

### Structures (from main.hpp)
- **DataPackage**: RF24 communication packet with command and time
- **Gate**: Gate configuration and state
- **Measurement**: Current run measurement data with timestamps and flags
- **State**: State machine enumeration

### Constants (from constants.hpp)
- RF24_CMD_TIME_SYNC, RF24_CMD_RESEND_TIME, RF24_CMD_START, RF24_CMD_FINISH
- RF24_POWER_LEVEL, RF24_DATA_RATE, RF24_CHANNEL, RF24_RETRY_DELAY, RF24_RETRY_COUNT
- LED_RED, GATE_SERIAL_NUMBER, NRF_DELAY_OFFSET

### Global Objects (extern)
- **Radio**: NRF24L01 module instance
- **WiFiClient client**: Server connection
- **Display**: LED matrix output
- **RFID/QR**: Input devices

---

## Testing Checklist

- [ ] Code compiles without errors
- [ ] Code compiles without warnings
- [ ] Time synchronization works (transmitter mode)
- [ ] Time synchronization works (receiver mode)
- [ ] Start/Finish commands transmit correctly
- [ ] JSON messages format correctly
- [ ] Server responses parse correctly
- [ ] Connection monitoring works
- [ ] LED status indicates connection state
- [ ] State machine transitions work as expected
- [ ] No functionality lost or changed
- [ ] Performance remains unchanged

---

## Notes

1. All original logic and error handling has been preserved
2. All comments and logging remain intact
3. Function signatures are unchanged for compatibility
4. Constants are centralized in constants.hpp
5. Modules use extern for dependency injection
6. No dynamic memory allocation in modules
7. All methods are static (no instantiation required)
8. Comprehensive JSDoc documentation added
9. Private helper method for settings parsing
10. Backward compatible through wrapper functions
