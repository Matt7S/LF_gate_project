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

// ========== Global Variables ==========
Gate gate;                                  ///< Current gate configuration
Gate newGateSettings;                       ///< New settings from server pending application
bool newSettingsAvailable = false;          ///< Flag indicating new settings ready to apply

Measurement currMeasurement;                ///< Current run measurement data
State currentState = IDLE;                  ///< Current state machine state

extern WiFiClient client;                   ///< WiFi client for server communication

// ========== Hardware Instances ==========
RF24 radio(NRF_CE, NRF_CSN);                ///< NRF24L01 radio module
RFID rfid(RC522_SS, RC522_RST);             ///< RC522 RFID reader
QRScanner qr(QR_SCANNER_I2C_ADDRESS, BARCODE_SCANNER_SDA, BARCODE_SCANNER_SCL, &Wire);  ///< QR code scanner
P10Display display(LCD_A, LCD_B, LCD_CLK, LCD_DATA, LCD_LATCH, LCD_OE);  ///< LED matrix display


// ========== Setup and Loop ==========

void setup() {
    // Initialize display with startup messages
    display.setLineDynamic(0, "Connecting to server...", true, DISPLAY_SPEED_FAST, true);
    display.setLineDynamic(1, "Please wait", true, DISPLAY_SPEED_MEDIUM, true);
    delay(1000);

    // Initialize Serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Configure SPI bus
    SPI.setSCK(SPI0_PIN_SCK);
    SPI.setTX(SPI0_PIN_MOSI);
    SPI.setRX(SPI0_PIN_MISO);
    SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE0));
    SPI.begin();

    // Configure button inputs with pull-up resistors
    pinMode(BUTTON_ACCEPT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_CANCEL_PIN, INPUT_PULLUP); 
    pinMode(BUTTON_FORFEIT_PIN, INPUT_PULLUP);
    pinMode(NRF_INTERRUPT, INPUT_PULLUP);
    
    // Configure LED outputs
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);

    // Configure IR sensor input
    pinMode(IR_IRQ_PIN, INPUT_PULLUP);
    
    // Initialize LED states: GREEN off (HIGH), RED on (LOW)
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);

    // Initialize hardware modules
    rfid.init();
    radio.begin();
    qr.begin();
    qr.setMode(true, true, false);

    // Connect to WiFi and server
    connectToWiFi();
    resolveServerIP();
    connectToServer();
    
    // Request initial gate settings from server
    FirebaseJson data;
    data.set("serial_number", GATE_SERIAL_NUMBER);
    sendJsonMessage("GET_SETTINGS", data);
    
    // Wait for server response
    while (!client.available()) {
        Serial.println("Waiting for server response...");
        delay(1000);
    }
    
    // Indicate successful connection
    digitalWrite(LED_RED, HIGH);
    display.setLineDynamic(0, "Connected to server", true, DISPLAY_SPEED_FAST, true);
    delay(1000);

    // Configure NRF24L01 radio module
    radio.setPALevel(RF24_POWER_LEVEL);
    radio.setDataRate(RF24_DATA_RATE);
    radio.setChannel(RF24_CHANNEL);
    radio.setRetries(RF24_RETRY_DELAY, RF24_RETRY_COUNT);
    radio.setAutoAck(true);
    radio.maskIRQ(true, true, false);
    
    // Attach interrupt for NRF24L01
    attachInterrupt(digitalPinToInterrupt(NRF_INTERRUPT), handleInterrupt, FALLING);
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
  handleUserReset();
  checkConnection();
  handleServerResponse();

  switch (currentState) {
  case IDLE:  // Handle idle state
  {
    applyNewGateSettings();
    Serial.println("IDLE");
    if (gate.active) {
      if (gate.requiredUserCard && gate.requiredUserQrCode) 
      {
        currentState = USER_AUTHENTICATION;
        Serial.println("RFID_WAITING");
      } 
    }
    
    break;
  }

    

  //                                       CASE RFID_WAITING
  case USER_AUTHENTICATION:
    // Handle RFID waiting state
    {
      checkIR(1000);
      String screen_line1 = gate.categoryName;
      screen_line1 += " Please scan your card";
      display.setLineDynamic(0, screen_line1, true, DISPLAY_SPEED_FAST, true);
      display.setLineStatic(1, gate.typeName, 1, true);

      if (processRFIDCard(500, currMeasurement.playerCardCode)) 
      {
        FirebaseJson message;
        message.set("user_rfid_code", currMeasurement.playerCardCode);
        message.set("category_id", gate.categoryID);
        sendJsonMessage("GET_USER", message);
      }
      if (currMeasurement.getUserReceived) 
      {
        if (waitForNextState(200)) 
        {
          currentState = TIME_SYNCHRONIZATION;
          Serial.println("TIME_SYNCHRONIZATION");
          radio.flush_tx(); // Wyczyść bufor nadawczy
          radio.flush_rx(); // Wyczyść bufor odbiorczy
        }
      }
      if (newSettingsAvailable) 
      {
        currentState = IDLE;
        Serial.println("IDLE");
      }
    break;
  }

    

  //                                       CASE QR_WAITING
  case ROBOT_AUTHENTICATION: // Handle QR waiting state
  { 
    String screen_line1 = "Hello ";
    screen_line1 += currMeasurement.playerName;
    screen_line1 += " Please scan Robot QR code";
    display.setLineDynamic(0, screen_line1, true, DISPLAY_SPEED_FAST, true);
    display.setLineStatic(1, gate.typeName, 1, true);

    if (!currMeasurement.getRobotReceived) 
    {
      if (processQRCode(100, currMeasurement.robotQrCode)) 
      {
        FirebaseJson data;
        data.set("player_id", currMeasurement.playerID);
        data.set("robot_qr_code", currMeasurement.robotQrCode);
        data.set("category_id", gate.categoryID);
        data.set("stage_id", gate.stageID);
        sendJsonMessage("GET_ROBOT", data);
        Serial.println("QR_SCANNED");
        qr.setMode(true, false, false);
      }
    }
    else 
    {
      Serial.println("GET ROBOT RECIEVED");
      qr.setMode(true, false, false);
      String screen_line1 = "Hello ";
      screen_line1 += currMeasurement.robotName;
      display.setLineDynamic(0, screen_line1, true, DISPLAY_SPEED_FAST, true);
      display.setLineStatic(1, gate.typeName, 1, true);
      if (waitForNextState(2000)) 
      {
        if (gate.isStart) 
        {
          attachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN), handleInterruptIR, RISING);
          Serial.println("enable interrupt start");
          String screen_line1 = "Place robot ";
          screen_line1 += currMeasurement.robotName;
          screen_line1 += " on the start gate and good luck!";
          display.setLineDynamic(0, screen_line1, true, DISPLAY_SPEED_FAST, true);
          display.setLineStatic(1, gate.typeName, 1, true);
          display.startBlinking(1, 250);
        }
        else 
        {
          String screen_line1 = "Place robot ";
          screen_line1 += currMeasurement.robotName;
          screen_line1 += " on the start gate and good luck!";
          display.setLineDynamic(0, "Plan", true, DISPLAY_SPEED_FAST, true);
          display.setLineStatic(1, gate.typeName, 1, true);
        }
        currentState = TIME_SYNCHRONIZATION;
        Serial.println("START_WAITING");
      }
    }
    break;
  }

  //                                       CASE TIME_SYNCHRONIZATION
  case TIME_SYNCHRONIZATION:
  { 
    if (gate.isStart) {
      if (currMeasurement.syncCounter > 20)
      {
        display.setLineDynamic(0, "Error! Unable to synchronize time! Please try again!", true, DISPLAY_SPEED_FAST, true);
        if (waitForNextState(3000)) 
          {
            currentState = RESET_WAITING;
            Serial.println("RESET_WAITING");
          }
      }
      currMeasurement.syncCounter++;

      if (synchronizeTimeTransmitter()) 
      {
        currentState = START;
        Serial.println("QR_WAITING");
        qr.setMode(true, true, true);
      }
      delay(5);
    } 
    else
    {
      listenForSignals();
      if (currMeasurement.syncSuccess) {
        currentState = START;
        Serial.println("QR_WAITING");
        qr.setMode(true, true, true);
      }

    }
    break;
  }
    
  //                                       CASE START_WAITING
  case START: // Handle combined state for START_WAITING and STARTED
  {
      digitalWrite(LED_GREEN, LOW); // Green LED off

      if (gate.isStart) {
          if (currMeasurement.startInterruptFlag) {
              // Logic when startInterruptFlag is true
              currentState = START;
              Serial.println("STARTED");
              display.stopBlinking(0);
              display.clearTopPart();
              display.setTimer(0, 1, currMeasurement.startInterruptTime, 66);
              display.setLineDynamic(1, "Run in progress!", true, DISPLAY_SPEED_FAST, true);
              display.stopBlinking(1);

              if (sendStartCommand()) {
                  currentState = FINISH;
                  Serial.println("FINISH_WAITING");
                  radio.startListening();
              }
          }
      } else {
          listenForSignals();
          if (currMeasurement.startInterruptFlag) {
              attachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN), handleInterruptIR, RISING);
              currentState = START;
              Serial.println("STARTED");
              radio.stopListening();
              display.clearTopPart();
              display.setTimer(0, 1, currMeasurement.startInterruptTime, 66);
              display.setLineDynamic(1, "Run in progress!", true, DISPLAY_SPEED_FAST, true);

              currentState = FINISH;
              Serial.println("FINISH_WAITING");
          }
      }
      break;
  }

  case FINISH: // Handle combined state for FINISH_WAITING and FINISHED
  {
      if (gate.isStart) {
           if (currMeasurement.finishInterruptFlag) {
              // Logic for finish interrupt
              display.setTimer(0, 0, currMeasurement.finalTime, 66);
              currentState = COUNT_RESULT;
              Serial.println("CONFIRMATION_WAITING");
          }
      } else {
          if (currMeasurement.finishInterruptFlag) {
              currMeasurement.finalTime = currMeasurement.finishInterruptTime - currMeasurement.startInterruptTime;
              Serial.println(currMeasurement.finalTime);
              display.setTimer(0, 0, currMeasurement.finalTime, 66);
              currentState = COUNT_RESULT;
              Serial.println("CONFIRMATION_WAITING");
          } else if (sendFinishCommand()) {
              currentState = COUNT_RESULT;
              Serial.println("CONFIRMATION_WAITING");
          }
      }
      break;
  }
  case COUNT_RESULT:
  {
    Serial.println(currMeasurement.finalTime);
    uint64_t timing = ((currMeasurement.finalTime + 500) / 1000); // Zaokrąglamy w góręi odcinamy końcowe 3 miejsca po przecinku
    Serial.println(timing); 
    currMeasurement.counterMinutes = (uint32_t)(((timing) / 60000)) % 60;       // Minuty (każde 60 000 ms to 1 minuta)
    currMeasurement.counterSeconds = (uint32_t)(((timing) / 1000) % 60);        // Sekundy (każde 1000 ms to 1 sekunda)
    currMeasurement.counterMilliseconds = (uint32_t)((timing) % 1000);   

    char buffer[12]; // Tablica na wynik (maksymalnie 11 znaków + null terminator)
    Serial.println(currMeasurement.counterMinutes);
    Serial.println(currMeasurement.counterSeconds);
    Serial.println(currMeasurement.counterMilliseconds);

    sprintf(buffer, "%02d:%02d:%03d", currMeasurement.counterMinutes, currMeasurement.counterSeconds, currMeasurement.counterMilliseconds);
    currMeasurement.finalFormattedTime = String(buffer);
    Serial.println(currMeasurement.finalFormattedTime);

    currentState = JUDGE_CONFIRMATION;
    Serial.println("CONFIRMATION_WAITING");
    break;
  }

  case JUDGE_CONFIRMATION: 
  {
    if (gate.requiredConfirmation) 
    {
      if (!currMeasurement.retryJudgeReceived) 
      {
        display.setLineDynamic(1, "Judge! Please scan your card to confirm!", true, DISPLAY_SPEED_FAST, true);
      }
      else {
        display.setLineDynamic(1, currMeasurement.lastMessage, true, DISPLAY_SPEED_FAST, true);
      }

      if (processRFIDCard(100, currMeasurement.judgeCardCode)) 
      {
        FirebaseJson message;
        message.set("judge_card_code", currMeasurement.judgeCardCode);
        sendJsonMessage("GET_JUDGE", message);
      }
      if (currMeasurement.getJudgeReceived) 
      {
        currentState = CONFIRMATION;
        Serial.println("CONFIRMATION");
      }
    } 
    else 
    {
      currentState = CONFIRMATION;
      Serial.println("CONFIRMATION");
    }
    break;
  }
    
  case CONFIRMATION: // Handle confirmation state
  {
    //Serial.println("CONFIRMATION hereeeeee");
    uint32_t currentTime = millis();
    static uint32_t lastClickTime;
    if (!currMeasurement.retryScoreReceived) 
    {
      display.setLineDynamic(1, "Please accept, foreit or cancell!", true, DISPLAY_SPEED_FAST, true);
    } 
    else 
    {
      display.setLineDynamic(1, currMeasurement.lastMessage, true, DISPLAY_SPEED_FAST, true);
    }

      if (millis() - static_cast<uint32_t>(lastClickTime) > BUTTON_SKIP_TIME) 
    {
      if (digitalRead(BUTTON_ACCEPT_PIN) == LOW) 
      {
        lastClickTime = currentTime;
        FirebaseJson data;
        data.set("robot_id", currMeasurement.robotID);
        data.set("result_time", currMeasurement.finalFormattedTime);
        data.set("category_id", gate.categoryID);
        data.set("stage_id", gate.stageID);
        data.set("gate_id", GATE_SERIAL_NUMBER);
        if (gate.requiredConfirmation) 
        {
          data.set("judge_id", currMeasurement.judgeID);
        } else {
          data.set("judge_id", "");
        }
        data.set("judge_id", currMeasurement.judgeID);
        data.set("player_id", currMeasurement.playerID);
        data.set("disqualified", false);
        sendJsonMessage("SEND_SCORE", data);
      } 
      else if (digitalRead(BUTTON_FORFEIT_PIN) == LOW) 
      {
        lastClickTime = currentTime;
        FirebaseJson data;
        data.set("robot_id", currMeasurement.robotID);
        data.set("result_time", currMeasurement.finalFormattedTime);
        data.set("category_id", gate.categoryID);
        data.set("stage_id", gate.stageID);
        data.set("gate_id", gate.id);
        data.set("judge_id", currMeasurement.judgeID);
        data.set("player_id", currMeasurement.playerID);
        data.set("disqualified", true);

        sendJsonMessage("SEND_SCORE", data);
      }
      else if (digitalRead(BUTTON_CANCEL_PIN) == LOW) 
      {
        lastClickTime = currentTime;
        currentState = RESET_WAITING;
        Serial.println("RESET_WAITING");
      }
    }
    break;
  }
  //                                       CASE RESET_WAITING
  case RESET_WAITING: // Handle reset waiting state
  {
    FirebaseJson data;
    data.set("message", true);
    sendJsonMessage("GET_RESET", data);
    currentState = RESETING;
    
    break;
  }
    
  //                                       CASE RESETING
  case RESETING: // Handle reset state
  {
    //Serial.println("RESETING11");
    digitalWrite(LED_GREEN, HIGH);  //high - green led is off
    display.setLineDynamic(0, currMeasurement.lastMessage, true, DISPLAY_SPEED_FAST, true);
    display.setLineStatic(1, gate.typeName, 1, true);
    display.stopBlinking(0);
    display.stopBlinking(1);
    if (waitForNextState(5000)) 
    {
      Serial.println("RESETING22");
      qr.setMode(true, false, false);
      resetMeasurement(currMeasurement);

      currentState = IDLE;
    }
    break;
  }
    
  //                                       DEFAULT
  default:  // Handle unknown state
  {
    Serial.println("Unknown state");
    break;
  }
    
  }
}


bool waitForNextState(unsigned long interval) {
  static uint32_t previousTime = 0;
  static bool state = false;
  uint32_t currentTime = millis();
  if (!state) 
  {
    previousTime = currentTime;
    state = true;
  }
  else 
  {
    if (currentTime - previousTime >= interval) {
      previousTime = currentTime;
      state = false;
      return true;
    }
  }
  return false;
}

//                                                                                                       LOOP 1




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


// ========== Input Processing Functions ==========

/**
 * @brief QR code scanning with debouncing.
 * Reads QR code from scanner with minimum interval between scans.
 */
bool processQRCode(uint32_t scanEveryMs, String &qrCode) {
  static uint32_t repeatQRTimeout = 0;

  if (millis() - repeatQRTimeout > scanEveryMs)
  {
    qrCode = qr.readQRCode(false);
    if (qrCode.length() > 0) 
    {
      Serial.print("QR Code: ");
      Serial.println(qrCode);
      qr.setMode(true, false, false);
      return true;
    }
    repeatQRTimeout = millis();
  }
  return false;
}

/**
 * @brief RFID card scanning with debouncing.
 * Reads RFID card with minimum interval between scans.
 */
bool processRFIDCard(uint32_t scanEveryMs, String &cardNumber) {
  static uint32_t repeatRFIDTimeout = 0;

  if (millis() - repeatRFIDTimeout > scanEveryMs) {
    cardNumber = readDataFromRFIDCard();

    if (cardNumber.length() > 0) 
    {
      Serial.print("RFID Card: ");
      Serial.println(cardNumber);
      return true;
    }
    repeatRFIDTimeout = millis();
  }

  return false;
}

/**
 * @brief Reads RFID card data via RC522 module.
 * Extracts card UID and converts to hexadecimal string.
 * @return Card UID as uppercase hex string, or empty string if no card detected.
 */
String readDataFromRFIDCard() {
  digitalWrite(RC522_SS, LOW);
  rfid.antennaOn();

  static unsigned char status;
  static unsigned char str[MAX_LEN];
  String cardNumber = "";

  // Search card, return card types
  if (rfid.findCard(PICC_REQIDL, str) == MI_OK) {
    Serial.println("Find the card!");

    // Anti-collision detection, reading card serial number
    if (rfid.anticoll(str) == MI_OK) {
      Serial.print("The card's number is: ");
      for (int i = 0; i < 4; i++) {
        if (str[i] < 0x10) {
          cardNumber += "0"; // If the number is less than 0x10, add a "0" to the String
        }
        cardNumber += String(str[i], HEX); // Convert each byte to HEX and append to the String
      }
      cardNumber.toUpperCase();           // Ensure the card number is in uppercase
      Serial.println(cardNumber);
    }

    // Card selection
    rfid.selectTag(str);
  }

  rfid.halt(); // Command the card to enter sleep mode
  rfid.antennaOff();
  digitalWrite(RC522_SS, HIGH);
  return cardNumber; // Return the card number as a String
}

// ========== Server Communication Functions ==========

/**
 * @brief Sends JSON message to central server.
 * Formats command and data into JSON, adds line separator, and transmits via WiFi.
 * 
 * @param command Command identifier (e.g., "GET_SETTINGS", "GET_USER", "SEND_SCORE")
 * @param data FirebaseJson object containing command payload
 */
void sendJsonMessage(const char* command, FirebaseJson& data) {
    if (client.connected()) {
        FirebaseJson json;
        json.set("co", command);
        json.set("da", data);

        String jsonString;
        json.toString(jsonString, true);
        jsonString += "\n";
        client.print(jsonString);

        Serial.print("Sent JSON: ");
        Serial.println(jsonString);
    } else {
        Serial.println("ERROR: Not connected to server. Cannot send JSON.");
    }
}

/**
 * @brief Processes incoming messages from central server.
 * Parses JSON response, extracts command and data,
 * and updates gate state accordingly.
 * 
 * @return True if response processed successfully, false on error
 */
bool handleServerResponse() {
  String command;
  while (client.available()) {
    FirebaseJson responseJson;
    FirebaseJsonData jsonData;

    // Odczytaj odpowiedź jako linię
    String response = client.readStringUntil('\n');
    response.trim(); 
    Serial.print("Otrzymano od serwera: ");
    Serial.println(response);

    // Sprawdź, czy odpowiedź to poprawny JSON
    if (!responseJson.setJsonData(response)) {
        Serial.println("Nieprawidlowy format JSON w odpowiedzi serwera.");
        return false;
    }

    // Przypisanie początkowych danych ("co" i "da")
    if (!responseJson.get(jsonData, "co")) {
        Serial.println("Brak komendy w odpowiedzi.");
        return false;
    }
    command = jsonData.stringValue;

    FirebaseJson daJson;
    if (responseJson.get(jsonData, "da")) {
        daJson = FirebaseJson();
        daJson.setJsonData(jsonData.stringValue);
    } else {
        Serial.println("Brak danych 'da' w odpowiedzi.");
        return false;
    }

    Serial.print("Komenda: ");
    Serial.println(command);

    
    if (command == "SETTINGS") //                                                HANDLE SETTINGS
    {
    newSettingsAvailable = true; // Resetuj flagę
    String startNRFAddress, finishNRFAddress;
    if (daJson.get(jsonData, "pair_id")) { newGateSettings.pairID = jsonData.intValue; }
    if (daJson.get(jsonData, "active")) { newGateSettings.active = jsonData.boolValue; }
    if (daJson.get(jsonData, "start_gate_id")) { newGateSettings.startGateID = jsonData.intValue; }
    if (daJson.get(jsonData, "finish_gate_id")) { newGateSettings.finishGateID = jsonData.intValue; }
    if (daJson.get(jsonData, "category_id")) { newGateSettings.categoryID = jsonData.intValue; }
    if (daJson.get(jsonData, "stage_id")) { newGateSettings.stageID = jsonData.intValue; }
    if (daJson.get(jsonData, "start_nrf")) {
      startNRFAddress = jsonData.stringValue;
      strncpy((char*)newGateSettings.nrfStartAddress, startNRFAddress.c_str(), sizeof(newGateSettings.nrfStartAddress) - 1);
    }
    if (daJson.get(jsonData, "finish_nrf")) {
      finishNRFAddress = jsonData.stringValue;
      strncpy((char*)newGateSettings.nrfFinishAddress, finishNRFAddress.c_str(), sizeof(newGateSettings.nrfFinishAddress) - 1);
    }

    if (daJson.get(jsonData, "requires_user_card")) { newGateSettings.requiredUserCard = jsonData.boolValue; }
    if (daJson.get(jsonData, "requires_user_qr_code")) { newGateSettings.requiredUserQrCode = jsonData.boolValue; }
    if (daJson.get(jsonData, "requires_judge_confirmation")) { newGateSettings.requiredConfirmation = jsonData.boolValue; }
    if (daJson.get(jsonData, "category_name")) { newGateSettings.categoryName = jsonData.stringValue; }
    if (daJson.get(jsonData, "stage_name")) { newGateSettings.stageName = jsonData.stringValue; }
    if (daJson.get(jsonData, "gate_type")) { newGateSettings.typeName = jsonData.stringValue; }

    
    } 
    else if (command == "USER") //                                                HANDLE USER
    {
      currMeasurement.getUserReceived = true;
      if (daJson.get(jsonData, "player_rfid_code")) {currMeasurement.playerCardCode = jsonData.stringValue;}
      if (daJson.get(jsonData, "player_id")) {currMeasurement.playerID = jsonData.intValue;}
      if (daJson.get(jsonData, "player_name")) {currMeasurement.playerName = jsonData.stringValue;}
      
      Serial.print("Dane GET_USER: ");
      Serial.println(currMeasurement.playerName);

    } 
    else if (command == "ROBOT") //                                                HANDLE ROBOT
    {
      currMeasurement.getRobotReceived = true;
      String status;
      if (daJson.get(jsonData, "robot_qr_code")) {currMeasurement.robotQrCode = jsonData.stringValue;}
      if (daJson.get(jsonData, "robot_id")) {currMeasurement.robotID = jsonData.intValue;}
      if (daJson.get(jsonData, "robot_name"))  {currMeasurement.robotName = jsonData.stringValue;}

      Serial.print("Status robota: ");
      Serial.println(currMeasurement.robotName);
      
    } 
    else if (command == "RETRY_JUDGE") //                                          HANDLE RETRY_JUDGE
    {
      if (daJson.get(jsonData, "message")) { currMeasurement.lastMessage = jsonData.stringValue; }
      currMeasurement.retryJudgeReceived = true;
      currMeasurement.lastMessage = jsonData.stringValue;
    }

    else if (command == "JUDGE") //                                               HANDLE JUDGE
    {
      currMeasurement.getJudgeReceived = true;
      if (daJson.get(jsonData, "judge_card_code")) {currMeasurement.judgeCardCode = jsonData.stringValue;}
      if (daJson.get(jsonData, "judge_id")) {currMeasurement.judgeID = jsonData.intValue;}
      Serial.print("Dane GET_JUDGE: ");
      Serial.println(currMeasurement.judgeID);
    }

    else if (command == "RETRY_SCORE") //                                               HANDLE SCORE
    {
      if (daJson.get(jsonData, "message")) {currMeasurement.lastMessage = jsonData.stringValue;}
      currMeasurement.retryScoreReceived = true;
      currMeasurement.lastMessage = jsonData.stringValue;
    }

    else if (command == "RESET") //                                               HANDLE RESET
    {
      Serial.println("RESET command recieved");
      if (daJson.get(jsonData, "message")) {currMeasurement.lastMessage = jsonData.stringValue;}
      currentState = RESETING;
    } 
    else 
    {
      Serial.println("Nieznana komenda.");
      return false;
    }
  }
  return true;
}

// ========== Radio Communication Functions ==========

/**
 * @brief Listens for incoming signals from paired gate.
 * Processes radio commands: time synchronization, start, finish signals.
 * Called periodically when gate is not transmitting.
 */
void listenForSignals() {
    if (radio.available()) {
        Serial.println("Radio data available");

        DataPackage nrfData;
        radio.read(&nrfData, sizeof(DataPackage));

        switch (nrfData.command) {
            // Time synchronization request from transmitter
            case RF24_CMD_TIME_SYNC:
                if (currMeasurement.nrfInterruptTime == 0) {
                    Serial.println("Error: No interrupt timestamp available");
                    break;
                }
                Serial.println("Time synchronization request received");
                Serial.println(nrfData.time);
                currMeasurement.syncSuccess = synchronizeTimeReceiver(currMeasurement.nrfInterruptTime);
                break;

            // Request to resend synchronization time
            case RF24_CMD_RESEND_TIME:
                Serial.println("Resend sync time request received");
                sendTime(currMeasurement.syncTime);
                break;

            // Start gate signal (finish gate receives this)
            case RF24_CMD_START:
                Serial.println("START signal received");
                currMeasurement.startInterruptFlag = true;
                currMeasurement.startInterruptTime = nrfData.time;
                break;

            // Finish gate signal (start gate receives this)
            case RF24_CMD_FINISH:
                Serial.println("FINISH signal received");
                currMeasurement.finishInterruptFlag = true;
                currMeasurement.finalTime = nrfData.time;
                Serial.println(nrfData.time);
                break;

            default:
                break;
        }
    radio.maskIRQ(true, true, false);
    radio.startListening();

  }
}

// ========== Time Synchronization Functions ==========

/**
 * @brief Synchronizes time as transmitter (start gate).
 * Performs 2-way handshake with receiver to measure clock offset.
 * 
 * @return True if synchronization successful, false on timeout/error
 */
bool synchronizeTimeTransmitter() {
    uint64_t receiverTime = 0;
    uint64_t roundTripTimeStart = 0;
    uint64_t roundTripTimeEnd = 0;
    uint64_t roundTripTime = 0;

    currMeasurement.nrfInterruptFlag = false;
    currMeasurement.nrfInterruptTime = 0;
    currMeasurement.syncTime = 0;

    radio.maskIRQ(false, true, true);
    delay(10);
    
    DataPackage nrfData;
    nrfData.command = 0x01;
    nrfData.time = micros();
    Serial.println("Sernding synchronization command");
    Serial.println(nrfData.time);
    radio.stopListening();
    if (!radio.write(&nrfData, sizeof(DataPackage))) {
        Serial.println("Failed to send synchronization command.");
        return false;
    }
    roundTripTimeStart = currMeasurement.nrfInterruptTime;
    currMeasurement.syncTime = currMeasurement.nrfInterruptTime;
    currMeasurement.nrfInterruptFlag = false;

    if (currMeasurement.syncTime == 0) {
      Serial.println("NOT VALID INTERRUPT.");
      return false;
    }

    // Start listening for the response
    radio.maskIRQ(true, true, false);
    radio.startListening();
    
    waitForRadio(50000);
    roundTripTimeEnd = currMeasurement.nrfInterruptTime;
    Serial.println(roundTripTimeEnd);

    // Read the receiver's time
    if (radio.available()) {
      radio.read(&nrfData, sizeof(DataPackage));
      Serial.println(nrfData.command);
      Serial.println(nrfData.time);

      //if (nrfData.command != 123456789) {
      //  Serial.println("Invalid command received.");
      //  receiverTime = requestTimeWithRetries(10, 50000);
      //}
      //else {
        receiverTime = nrfData.time;
      //}
      currMeasurement.timeDifference = (int64_t)currMeasurement.syncTime - (int64_t)receiverTime - 167;
      
      Serial.print("Transmitter time: \t");
      Serial.print(currMeasurement.syncTime);
      Serial.print("\tReceiver time: \t");
      Serial.print(receiverTime);
      Serial.print("\tTime difference: \t");
      Serial.print(currMeasurement.timeDifference);
      Serial.print("\tRound trip time: \t");
      Serial.println(roundTripTimeEnd - roundTripTimeStart - 5000);
    } else {
      Serial.println("No data available.");
      return false;
    }
    return true;
}




bool synchronizeTimeReceiver(uint32_t lastReceivedInterruptTime) {
  currMeasurement.syncTime = 0;
  uint32_t synchro_time = lastReceivedInterruptTime;
  DataPackage nrfData;
  //radio.flush_tx();
  delayMicroseconds(5000);
  nrfData.command = 123456789; // 
  nrfData.time = synchro_time;
  radio.stopListening();
  if (!radio.write(&nrfData, sizeof(DataPackage))) {
    Serial.println("Failed to send current time.");
    currMeasurement.syncTime = 0;
    currMeasurement.syncSuccess = 0;
    currMeasurement.nrfInterruptFlag = false;
    return false;
  }
  else {  
    Serial.print("Time sent successfullyyyyyyy.\n");
    currMeasurement.syncTime = lastReceivedInterruptTime;
    Serial.println(synchro_time);
    return true;
  }
}


bool sendTime(uint32_t time) {
  DataPackage nrfData;
  nrfData.command = 123456789;
  nrfData.time = time;
  delayMicroseconds(5000);
  radio.stopListening();
  if (!radio.write(&nrfData, sizeof(DataPackage))) {
    Serial.println("Failed to send current time.");
    return false;
  }
  else {  
    Serial.println("Time sent successfully.\n");
    Serial.println(time);
    return true;
  }
}



bool waitForRadio(unsigned long timeout) {
  unsigned long start_time = micros();
  while (!currMeasurement.nrfInterruptFlag && (micros() - start_time < timeout)) {}
  currMeasurement.nrfInterruptFlag = false;
  return true;
}



uint32_t requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration) {
    DataPackage nrfData;
    uint32_t validTime = 0;
    

    for (uint8_t attempt = 0; attempt < maxRetries; ++attempt) {
        nrfData.command = 0x02;
        nrfData.time = 0;
        
        radio.stopListening();
        if (!radio.write(&nrfData, sizeof(DataPackage))) {
            Serial.println("Failed to request time.");
        }

        // Start listening for response
        radio.startListening();
        waitForRadio(waitDuration);

        if (radio.available()) {
            radio.read(&nrfData, sizeof(DataPackage));
            if (nrfData.command == 123456789) {
                validTime = nrfData.time;
                Serial.print("Valid time received on attempt ");
                Serial.print(attempt + 1);
                Serial.print(": ");
                Serial.println(validTime);
                return validTime;
            } else {
                Serial.println("Invalid command received, retrying...");
            }
        } else {
            Serial.println("No response received, retrying...");
        }
        radio.stopListening();
    }

    Serial.println("Failed to receive valid time after maximum retries.");
    return 0;
}


bool sendStartCommand() {
    DataPackage nrfData;
    nrfData.command = 0x03;
    nrfData.time = (uint64_t)((int64_t)currMeasurement.startInterruptTime - currMeasurement.timeDifference);
    
    radio.stopListening();
    if (!radio.write(&nrfData, sizeof(DataPackage))) {
        Serial.println("Failed to send start command.");
        radio.startListening();
        return false;
    }
    radio.startListening();
    return true;
}


bool sendFinishCommand() {
    DataPackage nrfData;
    nrfData.command = 0x04;

    nrfData.time = currMeasurement.finalTime;

    radio.stopListening();
    if (!radio.write(&nrfData, sizeof(DataPackage))) {
        Serial.println("Failed to send finish command.");
        radio.startListening();
        return false;
    }
    radio.startListening();
    return true;
}


bool checkIR(uint32_t timeout) {
  static uint32_t timeBrakeSensor = 0;
  static bool irBrakeActive = false;

  if (digitalRead(IR_IRQ_PIN) == LOW) {
    if (!irBrakeActive) {
      timeBrakeSensor = millis();
      irBrakeActive = true;
    } else if (millis() - timeBrakeSensor > timeout) {
      //screen_line2 = "IRerror";
    }
  } else {
    if (irBrakeActive) {
      //screen_line2 = "";
    }
    irBrakeActive = false;
  }
  return true;
}


void resetMeasurement(Measurement &measurement) {
    measurement.playerCardCode = "";
    measurement.playerName = "";
    measurement.playerID = 0;

    measurement.robotQrCode = "";
    measurement.robotName = "";
    measurement.robotID = 0;

    measurement.judgeCardCode = "";
    measurement.judgeID = 0;

    measurement.nrfInterruptTime = 0;
    measurement.nrfInterruptFlag = false;

    measurement.syncCounter = 0;
    measurement.syncSuccess = 0;
    measurement.syncTime = 0;
    measurement.timeDifference = 0;

    measurement.startInterruptTime = 0;
    measurement.finishInterruptTime = 0;
    measurement.startInterruptFlag = false;
    measurement.finishInterruptFlag = false;

    measurement.counterMinutes = 0;
    measurement.counterSeconds = 0;
    measurement.counterMilliseconds = 0;
    measurement.finalFormattedTime = "";
    measurement.finalTime = 0;

    measurement.getUserReceived = false;
    measurement.getRobotReceived = false;
    measurement.getJudgeReceived = false;
    measurement.retryJudgeReceived = false;
    measurement.retryScoreReceived = false;

    measurement.resetCommandSend = false;
    measurement.lastMessage = "";
  }


void checkConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
    
  }

  if (!client.connected()) {
    digitalWrite(LED_RED, LOW);  //low - green led is on
    if (connectToServer()) {
    FirebaseJson data;
    data.set("serial_number", GATE_SERIAL_NUMBER);
    sendJsonMessage("GET_SETTINGS", data);
    }
  } else {
    digitalWrite(LED_RED, HIGH);  //high - green led is off
  }
}

void handleUserReset() 
{
  static unsigned long lastResetTime = 0; // Static pozwala na przechowanie wartości między wywołaniami funkcji
  unsigned long currentTime = millis();

  // Sprawdzenie, czy od ostatniego wywołania minęły 2 sekundy
  if (currentTime - lastResetTime < BUTTON_SKIP_TIME) 
  {
    return; // Jeśli nie minęły 2 sekundy, zakończ funkcję
  }

  if (digitalRead(BUTTON_ACCEPT_PIN) == LOW) 
  {
    if (digitalRead(BUTTON_CANCEL_PIN) == LOW) 
    {
      if (digitalRead(BUTTON_FORFEIT_PIN) == LOW) 
      {
        FirebaseJson data;
        data.set("message", "User reset!");
        sendJsonMessage("GET_RESET", data);
        resetMeasurement(currMeasurement);

        lastResetTime = currentTime; // Zaktualizowanie czasu ostatniego wywołania
      }
    } 
  }
}


void applyNewGateSettings() {
  
  if (!newSettingsAvailable) {
    return;
  }
  Serial.println("Applying new settings...");
  newSettingsAvailable = false;
  // Przypisz wszystkie ustawienia z newGateSettings do gate
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



  // Konfiguracja bramki w zależności od typu
  if (gate.typeName == "START") {
      gate.isStart = true;
      gate.isFinish = false;
      radio.openWritingPipe(gate.nrfStartAddress); // Adres startowy już przypisany w handleServerResponse
      radio.openReadingPipe(1, gate.nrfFinishAddress); // Adres końcowy już przypisany w handleServerResponse
      radio.stopListening();
      Serial.println("Role set to TRANSMITTER");
  } else if (gate.typeName == "FINISH") {
     gate.isStart = false;
      gate.isFinish = true;
      radio.openWritingPipe(gate.nrfFinishAddress); // Adres końcowy już przypisany w handleServerResponse
      radio.openReadingPipe(1, gate.nrfStartAddress); // Adres startowy już przypisany w handleServerResponse
      radio.startListening();
      Serial.println("Role set to RECEIVER");
  }

  // Dodatkowe logi dla debugowania
  Serial.println("New settings applied:");
  Serial.print("Pair ID: "); Serial.println(gate.pairID);
  Serial.print("Active: "); Serial.println(gate.active);
  Serial.print("Start Gate ID: "); Serial.println(gate.startGateID);
  Serial.print("Finish Gate ID: "); Serial.println(gate.finishGateID);
  Serial.print("Category ID: "); Serial.println(gate.categoryID);
  Serial.print("Stage ID: "); Serial.println(gate.stageID);
  Serial.print((char*)gate.nrfStartAddress);
  Serial.print((char*)gate.nrfFinishAddress);
  Serial.print("Required User Card: "); Serial.println(gate.requiredUserCard);
  Serial.print("Required User QR Code: "); Serial.println(gate.requiredUserQrCode);
  Serial.print("Required Confirmation: "); Serial.println(gate.requiredConfirmation);
  Serial.print("Category Name: "); Serial.println(gate.categoryName);
  Serial.print("Stage Name: "); Serial.println(gate.stageName);
  Serial.print("Gate Type: "); Serial.println(gate.typeName);
}