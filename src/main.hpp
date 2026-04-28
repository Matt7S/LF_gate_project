#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include <FirebaseJson.h>
#include "constants.hpp"

/**
 * @struct DataPackage
 * @brief NRF24L01 radio transmission packet structure.
 * 
 * Contains command code and timestamp data for communication
 * between start and finish gates via wireless radio.
 */
struct DataPackage {
    uint32_t command;  ///< Command identifier (synchronization, start, finish, etc.)
    uint64_t time;     ///< Timestamp in microseconds
};

/**
 * @struct Gate
 * @brief Gate configuration and operational state.
 * 
 * Stores the current gate settings received from the server,
 * including role (start/finish), authentication requirements,
 * and wireless communication addresses.
 */
struct Gate {
    uint8_t id = GATE_SERIAL_NUMBER;              ///< Unique gate ID
    uint8_t pairID;                               ///< ID of paired gate
    bool active;                                  ///< Whether gate is currently active
    uint8_t startGateID;                          ///< ID of start gate
    uint8_t finishGateID;                         ///< ID of finish gate
    uint8_t categoryID;                           ///< Competition category ID
    uint8_t stageID;                              ///< Stage/round ID
    uint8_t nrfStartAddress[6];                   ///< NRF24 address for start gate
    uint8_t nrfFinishAddress[6];                  ///< NRF24 address for finish gate
    bool requiredUserCard;                        ///< Whether user authentication card is required
    bool requiredUserQrCode;                      ///< Whether robot QR code scan is required
    bool requiredConfirmation;                    ///< Whether judge confirmation is required
    String categoryName;                          ///< Display name of category
    String stageName;                             ///< Display name of stage/round
    String typeName;                              ///< Gate type: "START" or "FINISH"
    bool isStart = false;                         ///< True if gate is start gate
    bool isFinish = false;                        ///< True if gate is finish gate
};

/**
 * @enum State
 * @brief State machine states for gate operation.
 * 
 * Defines all possible states during competition run processing.
 */
enum State {
    IDLE,                    ///< Waiting for competitor
    USER_AUTHENTICATION,     ///< Scanning competitor RFID card
    ROBOT_AUTHENTICATION,    ///< Scanning robot QR code
    TIME_SYNCHRONIZATION,    ///< Synchronizing clocks between gates
    START,                   ///< Waiting for robot to pass start line
    FINISH,                  ///< Waiting for robot to pass finish line
    COUNT_RESULT,            ///< Calculating final time
    JUDGE_CONFIRMATION,      ///< Waiting for judge verification if required
    CONFIRMATION,            ///< Waiting for operator button confirmation
    RESET_WAITING,           ///< Requesting server reset signal
    RESETING                 ///< Processing reset, returning to IDLE
};

/**
 * @struct Measurement
 * @brief Current competition run data.
 * 
 * Contains all information for the active measurement cycle,
 * including participant data, timestamps, and server responses.
 */
struct Measurement {
    // Competitor Information
    String playerCardCode = "";       ///< RFID card code of competitor
    String playerName = "";           ///< Display name of competitor
    uint8_t playerID = 0;             ///< Server ID of competitor
    
    // Robot Information
    String robotQrCode = "";          ///< QR code of robot
    String robotName = "";            ///< Display name of robot
    uint8_t robotID = 0;              ///< Server ID of robot
    
    // Judge Information (if required)
    String judgeCardCode = "";        ///< RFID card code of judge
    uint8_t judgeID = 0;              ///< Server ID of judge
    
    // Radio Interrupt Data
    volatile uint64_t nrfInterruptTime = 0;   ///< NRF24 interrupt timestamp
    volatile bool nrfInterruptFlag = false;   ///< Flag set when NRF24 interrupt occurs
    
    // Time Synchronization Data
    uint8_t syncCounter = 0;       ///< Number of synchronization attempts
    bool syncSuccess = false;      ///< Whether time sync succeeded
    uint64_t syncTime = 0;         ///< Synchronized time value
    int64_t timeDifference = 0;       ///< Time offset between start and finish gates
    
    // IR Sensor Interrupt Data
    volatile uint64_t startInterruptTime = 0;    ///< Timestamp when robot crosses start line
    volatile uint64_t finishInterruptTime = 0;   ///< Timestamp when robot crosses finish line
    volatile bool startInterruptFlag = false;    ///< Flag set when start line IR triggered
    volatile bool finishInterruptFlag = false;   ///< Flag set when finish line IR triggered
    
    // Calculated Results
    uint32_t counterMinutes = 0;         ///< Minutes component of final time
    uint32_t counterSeconds = 0;         ///< Seconds component of final time
    uint32_t counterMilliseconds = 0;    ///< Milliseconds component of final time
    String finalFormattedTime = "";      ///< Formatted time string (MM:SS:mmm)
    uint64_t finalTime = 0;              ///< Raw final time in microseconds
    
    // Server Response Flags
    bool getUserReceived = false;        ///< Server returned user data
    bool getRobotReceived = false;       ///< Server returned robot data
    bool retryJudgeReceived = false;     ///< Server rejected judge card, retry needed
    bool getJudgeReceived = false;       ///< Server returned judge data
    bool retryScoreReceived = false;     ///< Server rejected score, retry needed
    
    // System State
    bool resetCommandSend = false;       ///< Whether reset command was sent to server
    String lastMessage = "";             ///< Last message from server (for error display)
};


// ========== Function Prototypes ==========

// ----- Interrupt Handlers -----
/**
 * @brief Handles NRF24L01 module interrupt.
 * Records the interrupt timestamp for time synchronization.
 */
void handleInterrupt();

/**
 * @brief Handles IR sensor interrupt when robot passes gate.
 * Records start or finish timestamp based on current gate role.
 */
void handleInterruptIR();

// ----- Hardware Input Processing -----
/**
 * @brief Reads data from RFID card reader.
 * @return String containing card UID in hex format, or empty string if no card detected.
 */
String readDataFromRFIDCard();

/**
 * @brief Processes RFID card scanning with debouncing.
 * @param scanEveryMs Minimum milliseconds between scan attempts.
 * @param cardNumber Reference to store the scanned card code.
 * @return True if new card code was successfully scanned, false otherwise.
 */
bool processRFIDCard(uint32_t scanEveryMs, String &cardNumber);

/**
 * @brief Processes QR code scanning with debouncing.
 * @param scanEveryMs Minimum milliseconds between scan attempts.
 * @param qrCode Reference to store the scanned QR code.
 * @return True if new QR code was successfully scanned, false otherwise.
 */
bool processQRCode(uint32_t scanEveryMs, String &qrCode);

/**
 * @brief Monitors IR sensor for obstructions.
 * @param timeout Timeout in milliseconds for blocked status.
 * @return True if sensor state is valid, false otherwise.
 */
bool checkIR(uint32_t timeout);

// ----- State Machine Control -----
/**
 * @brief Waits for specified interval before state transition.
 * Used to implement state machine delays and feedback timing.
 * @param interval Time to wait in milliseconds.
 * @return True when interval has elapsed, false otherwise.
 */
bool waitForNextState(unsigned long interval);

// ----- Radio Communication -----
/**
 * @brief Sends start command to finish gate via NRF24L01.
 * Transmits synchronized start timestamp.
 * @return True if transmission successful, false otherwise.
 */
bool sendStartCommand();

/**
 * @brief Sends finish command to start gate via NRF24L01.
 * Transmits final measurement time.
 * @return True if transmission successful, false otherwise.
 */
bool sendFinishCommand();

/**
 * @brief Listens for incoming signals from paired gate.
 * Processes time synchronization and start/finish commands.
 */
void listenForSignals();

/**
 * @brief Performs time synchronization as transmitter (start gate).
 * Initiates clock synchronization with finish gate.
 * @return True if synchronization successful, false otherwise.
 */
bool synchronizeTimeTransmitter();

/**
 * @brief Performs time synchronization as receiver (finish gate).
 * Responds to synchronization request from start gate.
 * @param lastReceivedInterruptTime Timestamp of received synchronization signal.
 * @return True if synchronization successful, false otherwise.
 */
bool synchronizeTimeReceiver(uint32_t lastReceivedInterruptTime);

/**
 * @brief Waits for NRF24L01 radio data within timeout period.
 * @param timeout Maximum wait time in microseconds.
 * @return True after timeout period, false otherwise.
 */
bool waitForRadio(unsigned long timeout);

/**
 * @brief Sends timestamp to paired gate.
 * @param time Time value to transmit.
 * @return True if transmission successful, false otherwise.
 */
bool sendTime(uint32_t time);

/**
 * @brief Requests time from receiver with retry mechanism.
 * @param maxRetries Maximum number of retry attempts.
 * @param waitDuration Time to wait for response between retries in microseconds.
 * @return Received time value, or 0 if all attempts failed.
 */
uint32_t requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration);

// ----- Server Communication -----
/**
 * @brief Sends JSON message to server.
 * Formats and transmits command with associated data to central server.
 * @param command Command identifier (e.g., "GET_SETTINGS", "GET_USER").
 * @param data Firebase JSON object containing command data.
 */
void sendJsonMessage(const char* command, FirebaseJson& data);

/**
 * @brief Handles incoming responses from server.
 * Parses JSON messages and updates gate state accordingly.
 * @return True if response was successfully processed, false on error.
 */
bool handleServerResponse();

/**
 * @brief Checks WiFi and server connection status.
 * Reconnects to WiFi or server if connection is lost.
 * Maintains persistent connection for real-time communication.
 */
void checkConnection();

// ----- Measurement and Reset Management -----
/**
 * @brief Resets all measurement data for next competitor run.
 * Clears participant info, timestamps, and server response flags.
 * @param measurement Reference to Measurement struct to reset.
 */
void resetMeasurement(Measurement &measurement);

/**
 * @brief Handles reset when all three buttons pressed simultaneously.
 * Forces system reset and returns to IDLE state.
 */
void handleUserReset();

// ----- Gate Configuration -----
/**
 * @brief Applies new gate settings received from server.
 * Updates gate role, authentication requirements, and radio settings.
 * Configures RF24 as transmitter (start) or receiver (finish).
 */
void applyNewGateSettings();

#endif  // MAIN_HPP