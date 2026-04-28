/**
 * @file main.cpp
 * @brief Main entry point for LF Gate Timing System.
 * 
 * Implements the state machine for managing competition runs,
 * including wireless synchronization, authentication, timing,
 * and communication with central server.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Freenove_RFID_Lib_for_Pico.h>
#include <FirebaseJson.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Wire.h>
#include <QRScanner.hpp>
#include "constants.hpp"
#include "wifi_config.hpp"
#include "P10Display.hpp"
#include "main.hpp"
#include "StateManager.hpp"
#include "HardwareControl.hpp"
#include "Authentication.hpp"
#include "Measurement.hpp"
#include "RadioCommunication.hpp"
#include "ServerCommunication.hpp"

// ========== Global Variables ==========
Gate gate;                                  ///< Current gate configuration
Gate newGateSettings;                       ///< New settings from server pending application
bool newSettingsAvailable = false;          ///< Flag indicating new settings ready to apply

extern WiFiClient client;                   ///< WiFi client for server communication

// Reference to current measurement (managed by MeasurementManager)
// Defined as global for backward compatibility

// ========== Hardware Instances ==========
RF24 radio(NRF_CE, NRF_CSN);                ///< NRF24L01 radio module
RFID rfid(RC522_SS, RC522_RST);             ///< RC522 RFID reader
QRScanner qr(QR_SCANNER_I2C_ADDRESS, BARCODE_SCANNER_SDA, BARCODE_SCANNER_SCL, &Wire);  ///< QR code scanner
P10Display display(LCD_A, LCD_B, LCD_CLK, LCD_DATA, LCD_LATCH, LCD_OE);  ///< LED matrix display


// ========== Setup and Loop ==========

void setup() {
    // Initialize display
    display.setLineDynamic(0, "Connecting to server...", true, DISPLAY_SPEED_FAST, true);
    display.setLineDynamic(1, "Please wait", true, DISPLAY_SPEED_MEDIUM, true);
    delay(1000);

    // Initialize Serial and SPI
    Serial.begin(SERIAL_BAUD_RATE);
    SPI.setSCK(SPI0_PIN_SCK);
    SPI.setTX(SPI0_PIN_MOSI);
    SPI.setRX(SPI0_PIN_MISO);
    SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE0));
    SPI.begin();

    // Initialize all hardware (buttons, LEDs, sensors, etc.)
    HardwareControl::initialize();
    Authentication::initialize();

    // Initialize RF24 radio
    radio.begin();
    radio.setPALevel(RF24_POWER_LEVEL);
    radio.setDataRate(RF24_DATA_RATE);
    radio.setChannel(RF24_CHANNEL);
    radio.setRetries(RF24_RETRY_DELAY, RF24_RETRY_COUNT);
    radio.setAutoAck(true);
    radio.maskIRQ(true, true, false);
    attachInterrupt(digitalPinToInterrupt(NRF_INTERRUPT), handleInterrupt, FALLING);

    // Connect to WiFi and server
    connectToWiFi();
    resolveServerIP();
    connectToServer();
    
    // Request initial gate settings
    FirebaseJson data;
    data.set("serial_number", GATE_SERIAL_NUMBER);
    ServerCommunication::sendJsonMessage("GET_SETTINGS", data);
    
    // Wait for server response
    while (!client.available()) {
        Serial.println("Waiting for server response...");
        delay(1000);
    }
    
    // Indicate successful connection
    HardwareControl::setConnectionStatus(true);
    display.setLineDynamic(0, "Connected to server", true, DISPLAY_SPEED_FAST, true);
    delay(1000);

    Serial.println("Setup complete - system ready");
}

/**
 * @brief Core loop for second processor (dual-core RP2040).
 * Dedicated to display updates and rendering.
 */
void setup1() {
    // Empty setup for second core
}

/**
 * @brief Core 1 loop - handles display rendering.
 */
void loop1() {
    display.updateDisplay();
}

/**
 * @brief Core 0 loop - main state machine.
 * Handles competition timing, user input, and server communication.
 */
void loop() {
  // Check for three-button emergency reset
  if (HardwareControl::areAllButtonsPressed()) {
    FirebaseJson resetData;
    resetData.set("message", "User reset!");
    ServerCommunication::sendJsonMessage("GET_RESET", resetData);
    MeasurementManager::reset();
  }

  // Maintain connection and process server responses
  ServerCommunication::checkConnection();
  ServerCommunication::handleServerResponse();

  State state = StateManager::getCurrentState();

  switch (state) {
  case IDLE:
  {
    applyNewGateSettings();
    
    if (gate.active && gate.requiredUserCard && gate.requiredUserQrCode) {
      StateManager::transitionTo(USER_AUTHENTICATION, "USER_AUTHENTICATION");
      HardwareControl::monitorIRSensor(1000);
    }
    break;
  }

  case USER_AUTHENTICATION:
  {
    HardwareControl::monitorIRSensor(1000);
    
    String prompt = gate.categoryName + " Please scan your card";
    HardwareControl::displayMessage(0, prompt, DISPLAY_SPEED_FAST, true);
    HardwareControl::displayMessage(1, gate.typeName, DISPLAY_SPEED_FAST, true);

    String cardCode;
    if (Authentication::readRFIDCard(500, cardCode)) {
      currMeasurement.playerCardCode = cardCode;
      FirebaseJson message;
      message.set("user_rfid_code", currMeasurement.playerCardCode);
      message.set("category_id", gate.categoryID);
      ServerCommunication::sendJsonMessage("GET_USER", message);
    }
    
    if (currMeasurement.getUserReceived) {
      if (StateManager::waitForStateTransition(200)) {
        StateManager::transitionTo(ROBOT_AUTHENTICATION, "ROBOT_AUTHENTICATION");
        radio.flush_tx();
        radio.flush_rx();
      }
    }
    
    if (newSettingsAvailable) {
      StateManager::transitionTo(IDLE, "IDLE");
    }
    break;
  }

  case ROBOT_AUTHENTICATION:
  {
    if (!currMeasurement.getRobotReceived) {
      String prompt = "Hello " + currMeasurement.playerName + " Please scan Robot QR code";
      HardwareControl::displayMessage(0, prompt, DISPLAY_SPEED_FAST, true);
      HardwareControl::displayMessage(1, gate.typeName, DISPLAY_SPEED_FAST, true);

      String qrCode;
      if (Authentication::readQRCode(100, qrCode)) {
        currMeasurement.robotQrCode = qrCode;
        FirebaseJson data;
        data.set("player_id", currMeasurement.playerID);
        data.set("robot_qr_code", currMeasurement.robotQrCode);
        data.set("category_id", gate.categoryID);
        data.set("stage_id", gate.stageID);
        ServerCommunication::sendJsonMessage("GET_ROBOT", data);
        Authentication::setQRMode(true, false, false);
      }
    }
    else {
      Authentication::setQRMode(true, false, false);
      String greeting = "Hello " + currMeasurement.robotName;
      HardwareControl::displayMessage(0, greeting, DISPLAY_SPEED_FAST, true);
      HardwareControl::displayMessage(1, gate.typeName, DISPLAY_SPEED_FAST, true);

      if (StateManager::waitForStateTransition(2000)) {
        if (gate.isStart) {
          attachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN), handleInterruptIR, RISING);
          String prompt = "Place robot " + currMeasurement.robotName + " on the start gate and good luck!";
          HardwareControl::displayMessage(0, prompt, DISPLAY_SPEED_FAST, true);
          HardwareControl::displayMessage(1, gate.typeName, DISPLAY_SPEED_FAST, true);
          HardwareControl::startBlinking(1, 250);
        } else {
          String prompt = "Place robot " + currMeasurement.robotName + " on the start gate and good luck!";
          HardwareControl::displayMessage(0, prompt, DISPLAY_SPEED_FAST, true);
          HardwareControl::displayMessage(1, gate.typeName, DISPLAY_SPEED_FAST, true);
        }
        StateManager::transitionTo(TIME_SYNCHRONIZATION, "TIME_SYNCHRONIZATION");
      }
    }
    break;
  }

  case TIME_SYNCHRONIZATION:
  {
    if (gate.isStart) {
      if (currMeasurement.syncCounter > 20) {
        HardwareControl::displayMessage(0, "Error! Unable to synchronize time! Please try again!", DISPLAY_SPEED_FAST, true);
        if (StateManager::waitForStateTransition(3000)) {
          StateManager::transitionTo(RESET_WAITING, "RESET_WAITING");
        }
      }
      currMeasurement.syncCounter++;

      if (RadioCommunication::synchronizeTimeTransmitter()) {
        StateManager::transitionTo(START, "START");
        Authentication::setQRMode(true, true, true);
      }
      delay(5);
    } 
    else {
      RadioCommunication::listenForSignals();
      if (currMeasurement.syncSuccess) {
        StateManager::transitionTo(START, "START");
        Authentication::setQRMode(true, true, true);
      }
    }
    break;
  }

  case START:
  {
    HardwareControl::setGreenLED(false);

    if (gate.isStart) {
      if (currMeasurement.startInterruptFlag) {
        HardwareControl::stopBlinking(0);
        HardwareControl::clearDisplay();
        HardwareControl::displayTimer(0, currMeasurement.startInterruptTime, 66);
        HardwareControl::displayMessage(1, "Run in progress!", DISPLAY_SPEED_FAST, true);
        HardwareControl::stopBlinking(1);

        if (RadioCommunication::sendStartCommand()) {
          StateManager::transitionTo(FINISH, "FINISH");
          radio.startListening();
        }
      }
    } 
    else {
      RadioCommunication::listenForSignals();
      if (currMeasurement.startInterruptFlag) {
        attachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN), handleInterruptIR, RISING);
        radio.stopListening();
        HardwareControl::clearDisplay();
        HardwareControl::displayTimer(0, currMeasurement.startInterruptTime, 66);
        HardwareControl::displayMessage(1, "Run in progress!", DISPLAY_SPEED_FAST, true);
        StateManager::transitionTo(FINISH, "FINISH");
      }
    }
    break;
  }

  case FINISH:
  {
    if (gate.isStart) {
      if (currMeasurement.finishInterruptFlag) {
        HardwareControl::displayTimer(0, currMeasurement.finalTime, 66);
        StateManager::transitionTo(COUNT_RESULT, "COUNT_RESULT");
      }
    } 
    else {
      if (currMeasurement.finishInterruptFlag) {
        currMeasurement.finalTime = currMeasurement.finishInterruptTime - currMeasurement.startInterruptTime;
        Serial.println(currMeasurement.finalTime);
        HardwareControl::displayTimer(0, currMeasurement.finalTime, 66);
        StateManager::transitionTo(COUNT_RESULT, "COUNT_RESULT");
      } 
      else if (RadioCommunication::sendFinishCommand()) {
        StateManager::transitionTo(COUNT_RESULT, "COUNT_RESULT");
      }
    }
    break;
  }

  case COUNT_RESULT:
  {
    // Calculate time components
    uint64_t timing = (currMeasurement.finalTime + 500) / 1000;
    uint32_t minutes = (timing / 60000) % 60;
    uint32_t seconds = (timing / 1000) % 60;
    uint32_t milliseconds = timing % 1000;

    char buffer[12];
    sprintf(buffer, "%02u:%02u:%03u", minutes, seconds, milliseconds);
    currMeasurement.finalFormattedTime = String(buffer);

    StateManager::transitionTo(JUDGE_CONFIRMATION, "JUDGE_CONFIRMATION");
    break;
  }

  case JUDGE_CONFIRMATION:
  {
    if (gate.requiredConfirmation) {
      if (!currMeasurement.retryJudgeReceived) {
        HardwareControl::displayMessage(1, "Judge! Please scan your card to confirm!", DISPLAY_SPEED_FAST, true);
      }
      else {
        HardwareControl::displayMessage(1, currMeasurement.lastMessage, DISPLAY_SPEED_FAST, true);
      }

      String judgeCard;
      if (Authentication::readRFIDCard(100, judgeCard)) {
        currMeasurement.judgeCardCode = judgeCard;
        FirebaseJson message;
        message.set("judge_card_code", currMeasurement.judgeCardCode);
        ServerCommunication::sendJsonMessage("GET_JUDGE", message);
      }
      
      if (currMeasurement.getJudgeReceived) {
        StateManager::transitionTo(CONFIRMATION, "CONFIRMATION");
      }
    } 
    else {
      StateManager::transitionTo(CONFIRMATION, "CONFIRMATION");
    }
    break;
  }

  case CONFIRMATION:
  {
    if (!currMeasurement.retryScoreReceived) {
      HardwareControl::displayMessage(1, "Please accept, forfeit or cancel!", DISPLAY_SPEED_FAST, true);
    } 
    else {
      HardwareControl::displayMessage(1, currMeasurement.lastMessage, DISPLAY_SPEED_FAST, true);
    }

    if (HardwareControl::isAcceptButtonPressed()) {
      FirebaseJson data;
      data.set("robot_id", currMeasurement.robotID);
      data.set("result_time", currMeasurement.finalFormattedTime);
      data.set("category_id", gate.categoryID);
      data.set("stage_id", gate.stageID);
      data.set("gate_id", GATE_SERIAL_NUMBER);
      data.set("judge_id", currMeasurement.judgeID);
      data.set("player_id", currMeasurement.playerID);
      data.set("disqualified", false);
      ServerCommunication::sendJsonMessage("SEND_SCORE", data);
    } 
    else if (HardwareControl::isForfeitButtonPressed()) {
      FirebaseJson data;
      data.set("robot_id", currMeasurement.robotID);
      data.set("result_time", currMeasurement.finalFormattedTime);
      data.set("category_id", gate.categoryID);
      data.set("stage_id", gate.stageID);
      data.set("gate_id", gate.id);
      data.set("judge_id", currMeasurement.judgeID);
      data.set("player_id", currMeasurement.playerID);
      data.set("disqualified", true);
      ServerCommunication::sendJsonMessage("SEND_SCORE", data);
    }
    else if (HardwareControl::isCancelButtonPressed()) {
      StateManager::transitionTo(RESET_WAITING, "RESET_WAITING");
    }
    break;
  }

  case RESET_WAITING:
  {
    FirebaseJson data;
    data.set("message", true);
    ServerCommunication::sendJsonMessage("GET_RESET", data);
    StateManager::transitionTo(RESETING, "RESETING");
    break;
  }

  case RESETING:
  {
    HardwareControl::setGreenLED(false);
    HardwareControl::displayMessage(0, currMeasurement.lastMessage, DISPLAY_SPEED_FAST, true);
    HardwareControl::displayMessage(1, gate.typeName, DISPLAY_SPEED_FAST, true);
    HardwareControl::stopBlinking(0);
    HardwareControl::stopBlinking(1);
    
    if (StateManager::waitForStateTransition(5000)) {
      Authentication::setQRMode(true, false, false);
      MeasurementManager::reset();
      StateManager::transitionTo(IDLE, "IDLE");
    }
    break;
  }

  default:
    Serial.println("Unknown state");
    break;
  }
}


// ========== Interrupt Handlers ==========

/**
 * @brief NRF24L01 radio module interrupt handler.
 * Records precise timestamp when RF data becomes available.
 * Called on falling edge of NRF interrupt pin.
 */
void handleInterrupt() {
    noInterrupts();
    currMeasurement.nrfInterruptTime = to_us_since_boot(get_absolute_time());
    currMeasurement.nrfInterruptFlag = true;
    interrupts();
}

/**
 * @brief IR sensor interrupt handler for start/finish detection.
 * Records robot passage with microsecond precision.
 * Determines if this is start gate or finish gate based on gate role.
 */
void handleInterruptIR() {
    noInterrupts();
    detachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN));
    
    uint64_t currentTimestamp = to_us_since_boot(get_absolute_time());
    
    if (gate.isStart) {
        // Start gate: record when robot crosses start line
        currMeasurement.startInterruptTime = currentTimestamp;
        currMeasurement.startInterruptFlag = true;
    } else {
        // Finish gate: record when robot crosses finish line
        currMeasurement.finishInterruptTime = currentTimestamp;
        currMeasurement.finishInterruptFlag = true;
    }
    
    interrupts();
}


// ========== Authentication Wrappers (delegated to Authentication class) ==========

bool processQRCode(uint32_t scanEveryMs, String &qrCode) {
  return Authentication::readQRCode(scanEveryMs, qrCode);
}

bool processRFIDCard(uint32_t scanEveryMs, String &cardNumber) {
  return Authentication::readRFIDCard(scanEveryMs, cardNumber);
}

// ========== Server Communication Wrappers ==========

/**
 * @brief Sends JSON message to central server.
 * Delegates to ServerCommunication class.
 */
void sendJsonMessage(const char* command, FirebaseJson& data) {
    ServerCommunication::sendJsonMessage(command, data);
}

/**
 * @brief Processes incoming messages from central server.
 * Delegates to ServerCommunication class.
 */
bool handleServerResponse() {
    return ServerCommunication::handleServerResponse();
}

// ========== Radio Communication Functions ==========

/**
 * @brief Listens for incoming signals from paired gate.
 * Delegates to RadioCommunication class.
 */
void listenForSignals() {
    RadioCommunication::listenForSignals();
}

// ========== Time Synchronization Functions ==========

/**
 * @brief Synchronizes time as transmitter (start gate).
 * Delegates to RadioCommunication class.
 */
bool synchronizeTimeTransmitter() {
    return RadioCommunication::synchronizeTimeTransmitter();
}




/**
 * @brief Synchronizes time as receiver (finish gate).
 * Delegates to RadioCommunication class.
 */
bool synchronizeTimeReceiver(uint32_t lastReceivedInterruptTime) {
    return RadioCommunication::synchronizeTimeReceiver(lastReceivedInterruptTime);
}


/**
 * @brief Sends timestamp to paired gate.
 * Delegates to RadioCommunication class.
 */
bool sendTime(uint32_t time) {
    return RadioCommunication::sendTime(time);
}





// ========== Helper Functions ==========

/**
 * @brief Applies new gate settings from server.
 * Configures radio addresses and gate role (start/finish).
 */
void applyNewGateSettings() {
  if (!newSettingsAvailable) {
    return;
  }
  Serial.println("Applying new settings...");
  newSettingsAvailable = false;
  
  gate.pairID = newGateSettings.pairID;
  gate.active = newGateSettings.active;
  gate.startGateID = newGateSettings.startGateID;
  gate.finishGateID = newGateSettings.finishGateID;
  gate.categoryID = newGateSettings.categoryID;
  gate.stageID = newGateSettings.stageID;
  gate.requiredUserCard = newGateSettings.requiredUserCard;
  gate.requiredUserQrCode = newGateSettings.requiredUserQrCode;
  gate.requiredConfirmation = newGateSettings.requiredConfirmation;
  gate.categoryName = newGateSettings.categoryName;
  gate.stageName = newGateSettings.stageName;
  gate.typeName = newGateSettings.typeName;
  memcpy(gate.nrfStartAddress, newGateSettings.nrfFinishAddress, sizeof(newGateSettings.nrfFinishAddress));
  memcpy(gate.nrfFinishAddress, newGateSettings.nrfStartAddress, sizeof(newGateSettings.nrfStartAddress));

  if (gate.typeName == "START") {
    gate.isStart = true;
    gate.isFinish = false;
    radio.openWritingPipe(gate.nrfStartAddress);
    radio.openReadingPipe(1, gate.nrfFinishAddress);
    radio.stopListening();
    Serial.println("Role set to TRANSMITTER");
  } else if (gate.typeName == "FINISH") {
    gate.isStart = false;
    gate.isFinish = true;
    radio.openWritingPipe(gate.nrfFinishAddress);
    radio.openReadingPipe(1, gate.nrfStartAddress);
    radio.startListening();
    Serial.println("Role set to RECEIVER");
  }

  Serial.println("New settings applied successfully");
}