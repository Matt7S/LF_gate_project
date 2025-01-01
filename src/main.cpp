#include <Arduino.h>
#include <SPI.h>
#include <Freenove_RFID_Lib_for_Pico.h>
#include <FirebaseJson.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Wire.h>
#include <QRScanner.hpp>
#include "wifi_config.hpp"
#include "P10Display.hpp"
#include "main.hpp"

#define GATE_SERIAL_NUMBER 1
#define BARCODE_SCANNER_PRESENT_PIN 0
// Define pins for SPI
#define SPI0_PIN_SCK 2
#define SPI0_PIN_MOSI 3
#define SPI0_PIN_MISO 4
// Define pins for RFID
#define RC_522_SDA 5
#define RC_522_RST 6
// Define pins for NRF24L01
#define NRF_INTERRUPT 7
#define NRF_CE 10 //yellow
#define NRF_CSN 11 // orange
// Define pins for barcode scanner
#define BARCODE_SCANNER_SDA 8  // reciever on the scanner
#define BARCODE_SCANNER_SCL 9  // transmiter on the scanner
// Define pins for LEDs
#define LED_RED 12
#define LED_GREEN 13
// Define pins for buttons and LEDs
#define BUTTON_ACCEPT_PIN 26
#define BUTTON_CANCEL_PIN 14
#define BUTTON_FORFEIT_PIN 15
// Define pins for LCD P10
#define LCD_OE 16      // Output Enable
#define LCD_A 17       // Row address A
#define LCD_B 18       // Row address B
#define LCD_CLK 19    // Clock for shifting data
#define LCD_LATCH 20  // Latch to display data
#define LCD_DATA 21   // Serial data input
// Define pin for IR sensor
#define IR_IRQ_PIN 22
// Define BUTTON DEBOUNCE TIME
#define BUTTON_SKIP_TIME 500

Gate gate;
Measurement currMeasurement;
State currentState = IDLE;
extern WiFiClient client; 
RF24 radio(NRF_CE, NRF_CSN);
RFID rfid(RC_522_SDA, RC_522_RST);  
QRScanner qr(0x21, BARCODE_SCANNER_SDA, BARCODE_SCANNER_SCL, &Wire); // Adres I2C, SDA, SCL, Wire
P10Display display(LCD_A, LCD_B, LCD_CLK, LCD_DATA, LCD_LATCH, LCD_OE);

bool QRScanner_present = false;
uint8_t display_speed1 = 20;
uint8_t display_speed2 = 50;
uint8_t display_speed3 = 75;


void setup() {
  display.setLineDynamic(0, "Connecting to server...", true, display_speed1, true);
  display.setLineDynamic(1, "Please wait", true, display_speed2, true);
  delay(1000);

  Serial.begin(115200);
  SPI.setSCK(SPI0_PIN_SCK);
  SPI.setTX(SPI0_PIN_MOSI);
  SPI.setRX(SPI0_PIN_MISO);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.begin();

  pinMode(BUTTON_ACCEPT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CANCEL_PIN, INPUT_PULLUP); 
  pinMode(BUTTON_FORFEIT_PIN, INPUT_PULLUP);
  
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  pinMode(IR_IRQ_PIN, INPUT_PULLUP);
  pinMode(BARCODE_SCANNER_PRESENT_PIN, INPUT_PULLUP);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, LOW);

  QRScanner_present = digitalRead(BARCODE_SCANNER_PRESENT_PIN) == HIGH;


  rfid.init();
  radio.begin();

  if (QRScanner_present) {
    qr.begin();
    Serial.println("QR scanner is present");
  }


  

  connectToWiFi();
  resolveServerIP();
  connectToServer();
  FirebaseJson data;
  data.set("serial_number", GATE_SERIAL_NUMBER);
  sendJsonMessage("GET_SETTINGS", data);
  
  while (!client.available()) {
      Serial.println("Waiting for serwer response...");
      delay(1000);
  }
  display.setLineDynamic(0, "Connected to server", true, display_speed1, true);
  delay(1000);

  // Ustawienia radia
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(100);
  radio.setRetries(0, 0);
  radio.setAutoAck(true); // Wyłącz automatyczne potwierdzenia dla wszystkich rurek
  // Enable interrupt for NRF module
  radio.maskIRQ(true, true, false);
  attachInterrupt(digitalPinToInterrupt(NRF_INTERRUPT), handleInterrupt, FALLING);
  
}






//                                                                                                       SETUP 1


void setup1(){
  
  
  
}


//                                                                                                       LOOP 0

void loop() {
  handleUserReset();
  checkConnection();
  handleServerResponse();

  switch (currentState) {
  //                                       CASE IDLE
  case IDLE:  // Handle idle state
  {
    Serial.println("IDLE");
    if (gate.requiredUserCard && gate.requiredUserQrCode) 
    {
      currentState = RFID_WAITING;
      Serial.println("RFID_WAITING");
    } 
    else if (gate.requiredUserCard && !gate.requiredUserQrCode) 
    {
      currentState = QR_WAITING;
      Serial.println("QR_WAITING");
    }
    
    break;
  }

    

  //                                       CASE RFID_WAITING
  case RFID_WAITING:
    // Handle RFID waiting state
    {
      checkIR(1000);
      String screen_line1 = gate.categoryName;
      screen_line1 += " Please scan your card";
      display.setLineDynamic(0, screen_line1, true, display_speed1, true);
      display.setLineStatic(1, gate.typeName, 1, true);

      if (processRFIDCard(500, currMeasurement.playerCardCode)) 
      {
        FirebaseJson message;
        message.set("user_rfid_code", currMeasurement.playerCardCode);
        message.set("category_id", gate.categoryID);
        sendJsonMessage("GET_USER", message);
      }
      if (currMeasurement.getUserRecieved) 
      {
        currentState = RFID_SCANNED;
        Serial.println("RFID_SCANNED");
      }
    }
    break;
    

  //                                       CASE RFID_SCANNED
  case RFID_SCANNED: // Handle RFID scanned state
  {
    if (currMeasurement.getUserRecieved) 
    {
      if (waitForNextState(200)) 
      {
        currentState = TIME_SYNCHRONIZATION;
        Serial.println("TIME_SYNCHRONIZATION");
        radio.flush_tx(); // Wyczyść bufor nadawczy
        radio.flush_rx(); // Wyczyść bufor odbiorczy
      }
    }
    break;
  }
  
    
  
  //                                       CASE TIME_SYNCHRONIZATION
  case TIME_SYNCHRONIZATION:
  { 
    
    // Handle time synchronization state
    if (gate.typeName == "START") {
      if (currMeasurement.synchroCounter > 20)
      {
        display.setLineDynamic(0, "Error! Unable to synchronize time! Please try again!", true, display_speed1, true);
        if (waitForNextState(3000)) 
          {
            currentState = RESET_WAITING;
            Serial.println("RESET_WAITING");
          }
      }
      currMeasurement.synchroCounter++;

      if (synchronize_time_transmitter()) 
      {
        currentState = QR_WAITING;
        Serial.println("QR_WAITING");
        qr.setMode(true);
      }
    } 
    else
    {
      listenForSignals();
      if (currMeasurement.synchroSuccess) {
        currentState = QR_WAITING;
        Serial.println("QR_WAITING");
        //qr.setMode(true);
      }

    }
    break;
  }
    

  //                                       CASE QR_WAITING
  case QR_WAITING: // Handle QR waiting state
  { 
    String screen_line1 = "Hello ";
    screen_line1 += currMeasurement.playerName;
    screen_line1 += " Please scan Robot QR code";
    display.setLineDynamic(0, screen_line1, true, display_speed1, true);
    display.setLineStatic(1, gate.typeName, 1, true);
    if (gate.typeName == "START") 
    {
      if (processQRCode(100, currMeasurement.robotQrCode)) 
      {
        FirebaseJson data;
        data.set("player_id", currMeasurement.playerID);
        data.set("robot_qr_code", currMeasurement.robotQrCode);
        data.set("category_id", gate.categoryID);
        data.set("stage_id", gate.stageID);
        sendJsonMessage("GET_ROBOT", data);
        
        currentState = QR_SCANNED;
        Serial.println("QR_SCANNED");
        qr.setMode(false);
      }
      if (currMeasurement.getRobotRecieved) 
      {
        currentState = QR_SCANNED;
        Serial.println("QR_SCANNED");
      }
    }
    else {
      if (currMeasurement.getRobotRecieved) {
        currentState = QR_SCANNED;
        Serial.println("QR_SCANNED");
      }
    }
  }
    
    

    
    
    break;

  //                                       CASE QR_SCANNED
  case QR_SCANNED: // Handle QR scanned state
  {
    if (currMeasurement.getRobotRecieved) 
    {
      String screen_line1 = "Hello ";
      screen_line1 += currMeasurement.robotName;
      display.setLineDynamic(0, screen_line1, true, display_speed1, true);
      display.setLineStatic(1, gate.typeName, 1, true);
      if (waitForNextState(2000)) 
      {
        currentState = START_WAITING;
        Serial.println("START_WAITING");

        if (gate.start) 
        {
          attachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN), handleInterruptIR, FALLING);
          Serial.println("enable interrupt start");
          String screen_line1 = "Place robot ";
          screen_line1 += currMeasurement.robotName;
          screen_line1 += " on the start gate and good luck!";
          display.setLineDynamic(0, screen_line1, true, display_speed1, true);
          display.setLineStatic(1, gate.typeName, 1, true);
          display.startBlinking(1, 250);
        }
        else 
        {
          String screen_line1 = "Place robot ";
          screen_line1 += currMeasurement.robotName;
          screen_line1 += " on the start gate and good luck!";
          display.setLineDynamic(0, "Plan", true, display_speed1, true);
          display.setLineStatic(1, gate.typeName, 1, true);
        }
      }
    }
    break;
  }
    
    
  //                                       CASE START_WAITING
  case START_WAITING: // Handle start waiting state
    if (gate.start) {
      if(currMeasurement.startInterruptFlag) 
      {
        currentState = STARTED;
        Serial.println("STARTED");
        display.stopBlinking(0);
        display.clearTopPart();
        display.setTimer(0, 1, currMeasurement.startInterruptTime, 66);
        display.setLineDynamic(1, "Run in progress!", true, display_speed1, true);
        display.stopBlinking(1);
      }
    } else {
      listenForSignals();
      if (currMeasurement.startInterruptFlag)
      {
        attachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN), handleInterruptIR, FALLING);
        currentState = STARTED;
        Serial.println("STARTED");
        radio.stopListening();
        display.clearTopPart();
        display.setTimer(0, 1, currMeasurement.startInterruptTime, 66);
        display.setLineDynamic(1, "Run in progress!", true, display_speed1, true);
      }
    }

      break;

  //                                       CASE STARTED    
  case STARTED: // Handle started state
  {
    if (gate.start) 
    {
      if (sendStartCommand()) 
      {
        currentState = FINISH_WAITING;
        Serial.println("FINISH_WAITING");
        radio.startListening();
      }
    }
    else 
    {
      // POKAZANIE NA EKRANIE CZASU I PRZEJŚCIE DALEJ
      currentState = FINISH_WAITING;
      Serial.println("FINISH_WAITING");
    }
    break;
  }
    
    
  //                                       CASE FINISH_WAITING
  case FINISH_WAITING: // Handle finish waiting state
  {
    if (gate.start) 
    {
      listenForSignals();
      if (currMeasurement.finishInterruptFlag) 
      {
        display.setTimer(0, 0, currMeasurement.finalTime, 66);
        currentState = FINISHED;
        Serial.println("FINISHED");
        
      }
    } 
    else 
    {
      if (currMeasurement.finishInterruptFlag) 
      {
        currMeasurement.finalTime = currMeasurement.finishInterruptTime - currMeasurement.startInterruptTime;
        display.setTimer(0, 0, currMeasurement.finalTime, 66);
        currentState = FINISHED;
        Serial.println("FINISHED"); 
      }
    }
    break;
  }
    
  //                                       CASE FINISHED
  case FINISHED: // Handle finished state
  {
    if (gate.start) 
    {
      // only show time
      currentState = COUNT_RESULT;
      Serial.println("CONFIRMATION_WAITING");
    } 
    else 
    {
      if (sendFinishCommand()) 
      {
        currentState = COUNT_RESULT;
        Serial.println("CONFIRMATION_WAITING");
      }
    }
    break;
  }
  case COUNT_RESULT:
  {
    uint32_t timing = (currMeasurement.finalTime / 1000); // Odcinamy 3 ostatnie zera (optymalnie)
    currMeasurement.counter_m = ((timing) / 60000) % 60;       // Minuty (każde 60 000 ms to 1 minuta)
    currMeasurement.counter_s = ((timing) / 1000) % 60;        // Sekundy (każde 1000 ms to 1 sekunda)
    currMeasurement.counter_ms = (timing) % 1000;   

    char buffer[12]; // Tablica na wynik (maksymalnie 11 znaków + null terminator)
    Serial.println(currMeasurement.counter_m);
    Serial.println(currMeasurement.counter_s);
    Serial.println(currMeasurement.counter_ms);

    sprintf(buffer, "%02d:%02d:%03d", currMeasurement.counter_m, currMeasurement.counter_s, currMeasurement.counter_ms);
    currMeasurement.finalFormatedTime = String(buffer);
    Serial.println(currMeasurement.finalFormatedTime);

    currentState = CONFIRMATION_WAITING;
    Serial.println("CONFIRMATION_WAITING");
    break;
  }

  case CONFIRMATION_WAITING:
  {
    if (gate.requiredConfirmation) 
    {
      if (!currMeasurement.retryJudgeRecieved) 
      {
        display.setLineDynamic(1, "Judge! Please scan your card to confirm!", true, display_speed1, true);
      }
      else {
        display.setLineDynamic(1, currMeasurement.lastMessage, true, display_speed1, true);
      }

      if (processRFIDCard(100, currMeasurement.judgeCardCode)) 
      {
        FirebaseJson message;
        message.set("judge_card_code", currMeasurement.judgeCardCode);
        sendJsonMessage("GET_JUDGE", message);
      }
      if (currMeasurement.getJudgeRecieved) 
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
    if (!currMeasurement.retryScoreRecieved) 
    {
      display.setLineDynamic(1, "Please accept, foreit or cancell!", true, display_speed1, true);
    } 
    else 
    {
      display.setLineDynamic(1, currMeasurement.lastMessage, true, display_speed1, true);
    }

    if (currentTime - lastClickTime > BUTTON_SKIP_TIME) 
    {
      if (digitalRead(BUTTON_ACCEPT_PIN) == LOW) 
      {
        lastClickTime = currentTime;
        FirebaseJson data;
        data.set("robot_id", currMeasurement.robotID);
        data.set("result_time", currMeasurement.finalFormatedTime);
        data.set("category_id", gate.categoryID);
        data.set("stage_id", gate.stageID);
        data.set("gate_id", gate.ID);
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
        data.set("result_time", currMeasurement.finalFormatedTime);
        data.set("category_id", gate.categoryID);
        data.set("stage_id", gate.stageID);
        data.set("gate_id", gate.ID);
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
    display.setLineDynamic(0, currMeasurement.lastMessage, true, display_speed1, true);
    display.setLineStatic(1, gate.typeName, 1, true);
    display.stopBlinking(0);
    display.stopBlinking(1);
    if (waitForNextState(5000)) 
    {
      Serial.println("RESETING22");
      qr.setMode(false);
      resetMeasurement(currMeasurement);

      currentState = IDLE;
      FirebaseJson data;
      data.set("serial_number", GATE_SERIAL_NUMBER);
      sendJsonMessage("GET_SETTINGS", data);
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

void loop1() {
  display.updateDisplay();
}


void handleInterrupt() {
  currMeasurement.NRFInterruptFlag = true;
  currMeasurement.NRFInterruptTime = micros();
}


void handleInterruptIR() {
  if (gate.typeName == "START") {
    currMeasurement.startInterruptTime = (uint32_t)micros();
    currMeasurement.startInterruptFlag = 1;
    Serial.println("START Interrupt");
    Serial.println(currMeasurement.startInterruptTime);
  } else {
    currMeasurement.finishInterruptTime = (uint32_t)micros();
    currMeasurement.finishInterruptFlag = 1;
    Serial.println("FINISH Interrupt");
    Serial.println(currMeasurement.finishInterruptTime);
  }
  detachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN));
}


bool processQRCode(uint32_t scanEveryMs, String &qrCode) {
  static uint32_t repeat_qr = 0;

  if (millis() - repeat_qr > scanEveryMs) 
  {
    if (QRScanner_present) 
    {
      qrCode = qr.readQRCode(false); // Pass 'true' to filter out corrupted data
      if (qrCode.length() > 0) 
      {
        Serial.print("QR Code: ");
        Serial.println(qrCode);
        qr.setMode(false);
        return true;

        
      }
    }
    repeat_qr = millis();
  }
  return false;
}

bool processRFIDCard(uint32_t scanEveryMs, String &cardNumber) {
  static uint32_t repeat_rfid = 0;

  if (millis() - repeat_rfid > scanEveryMs) {
    cardNumber = readDataFromRFIDCard();

    if (cardNumber.length() > 0) 
    {
      Serial.print("RFID Card: ");
      Serial.println(cardNumber);
      return true;
    }
    repeat_rfid = millis();
  }

  return false;
}

String readDataFromRFIDCard() {
  digitalWrite(RC_522_SDA, LOW);
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
  digitalWrite(RC_522_SDA, HIGH);
  return cardNumber; // Return the card number as a String
}




void sendJsonMessage(const char* command, FirebaseJson& data) {
  if (client.connected()) {
    FirebaseJson json;
    json.set("co", command);
    json.set("da", data);

    String jsonString;
    json.toString(jsonString, true);
    jsonString += "\n"; // Dodaj separator wiadomości
    client.print(jsonString);

    Serial.print("Wyslano JSON: ");
    Serial.println(jsonString);
  } else {
      Serial.println("Brak polaczenia z serwerem. Nie mozna wyslac JSON-a.");
  }
}

bool handleServerResponse() {
  String command;
  while (client.available()) {
    FirebaseJson responseJson;
    FirebaseJsonData jsonData;
    bool synchroSuccess = false;

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
      String startAddress, finishAddress;
      if (daJson.get(jsonData, "pair_id")) { gate.pairID = jsonData.intValue; }
      if (daJson.get(jsonData, "active")) { gate.active = jsonData.boolValue; }
      if (daJson.get(jsonData, "start_gate_id")) { gate.startGateID = jsonData.intValue; }
      if (daJson.get(jsonData, "finish_gate_id")) { gate.finishGateID = jsonData.intValue; }
      if (daJson.get(jsonData, "category_id")) { gate.categoryID = jsonData.intValue; }
      if (daJson.get(jsonData, "stage_id")) { gate.stageID = jsonData.intValue; }
      if (daJson.get(jsonData, "start_nrf")) {
          startAddress = jsonData.stringValue;
          strncpy((char*)gate.nrfStartAddress, startAddress.c_str(), sizeof(gate.nrfStartAddress) - 1);
      }
      if (daJson.get(jsonData, "finish_nrf")) {
          finishAddress = jsonData.stringValue;
          strncpy((char*)gate.nrfFinishAddress, finishAddress.c_str(), sizeof(gate.nrfFinishAddress) - 1);
      }
      if (daJson.get(jsonData, "requires_user_card")) { gate.requiredUserCard = jsonData.boolValue; }
      if (daJson.get(jsonData, "requires_user_qr_code")) { gate.requiredUserQrCode = jsonData.boolValue; }
      if (daJson.get(jsonData, "requires_judge_confirmation")) { gate.requiredConfirmation = jsonData.boolValue; }
      if (daJson.get(jsonData, "category_name")) { gate.categoryName = jsonData.stringValue; }
      if (daJson.get(jsonData, "stage_name")) { gate.stageName = jsonData.stringValue; }
      if (daJson.get(jsonData, "gate_type")) { gate.typeName = jsonData.stringValue; }

      // Wyświetlanie ustawień
      Serial.println("Ustawienia:");
      Serial.print("pairID: "); Serial.println(gate.pairID);
      Serial.print("isActive: "); Serial.println(gate.active);
      Serial.print("startGateID: "); Serial.println(gate.startGateID);
      Serial.print("finishGateID: "); Serial.println(gate.finishGateID);
      Serial.print("categoryID: "); Serial.println(gate.categoryID);
      Serial.print("stageID: "); Serial.println(gate.stageID);
      Serial.print("nrfStartAddress: "); Serial.println(startAddress);
      Serial.print("nrfFinishAddress: "); Serial.println(finishAddress);
      Serial.print("requiredUserCard: "); Serial.println(gate.requiredUserCard);
      Serial.print("requiredUserQrCode: "); Serial.println(gate.requiredUserQrCode);
      Serial.print("requiredConfirmation: "); Serial.println(gate.requiredConfirmation);
      Serial.print("categoryName: "); Serial.println(gate.categoryName);
      Serial.print("stageName: "); Serial.println(gate.stageName);
      Serial.print("gateType: "); Serial.println(gate.typeName);

      if (gate.typeName == "START") {
        gate.start = true;
        radio.openWritingPipe(gate.nrfStartAddress);
        radio.openReadingPipe(1, gate.nrfFinishAddress);
        radio.stopListening();
        Serial.println("Role set to TRANSMITTER");
      }
      if (gate.typeName == "FINISH") {
        gate.finish = true;
        radio.openWritingPipe(gate.nrfFinishAddress);
        radio.openReadingPipe(1, gate.nrfStartAddress);
        radio.startListening();
        Serial.println("Role set to RECEIVER");
      }

    } 
    else if (command == "USER") //                                                HANDLE USER
    {
      currMeasurement.getUserRecieved = true;
      if (daJson.get(jsonData, "player_rfid_code")) {currMeasurement.playerCardCode = jsonData.intValue;}
      if (daJson.get(jsonData, "player_id")) {currMeasurement.playerID = jsonData.intValue;}
      if (daJson.get(jsonData, "player_name")) {currMeasurement.playerName = jsonData.stringValue;}
      
      Serial.print("Dane GET_USER: ");
      Serial.println(currMeasurement.playerName);

    } 
    else if (command == "ROBOT") //                                                HANDLE ROBOT
    {
      currMeasurement.getRobotRecieved = true;
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
      currMeasurement.retryJudgeRecieved = true;
      currMeasurement.lastMessage = jsonData.stringValue;
    }

    else if (command == "JUDGE") //                                               HANDLE JUDGE
    {
      currMeasurement.getJudgeRecieved = true;
      if (daJson.get(jsonData, "judge_card_code")) {currMeasurement.judgeCardCode = jsonData.stringValue;}
      if (daJson.get(jsonData, "judge_id")) {currMeasurement.judgeID = jsonData.intValue;}
      Serial.print("Dane GET_JUDGE: ");
      Serial.println(currMeasurement.judgeID);
    }

    else if (command == "RETRY_SCORE") //                                               HANDLE SCORE
    {
      if (daJson.get(jsonData, "message")) {currMeasurement.lastMessage = jsonData.stringValue;}
      currMeasurement.retryScoreRecieved = true;
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


void listenForSignals() {
  // Odbierz dane
  if (radio.available()) {
    Serial.println("Data available");

    //radio.stopListening();     // Przestań nasłuchiwać
    Data_Package nrf_data;
    radio.read(&nrf_data, sizeof(Data_Package));

    switch (nrf_data.command) {
        case 0x01: // Assuming 0x01 represents "TS"
            if (currMeasurement.NRFInterruptTime == 0) {
                Serial.println("No time received by interrupt??");
                break;
            }
            Serial.println("Time synchronization command received");
            currMeasurement.synchroSuccess = synchronize_time_receiver(currMeasurement.NRFInterruptTime);
            break;  

        case 0x02: // Assuming 0x02 represents"
            Serial.println("Resend synchro time command received");
            sendTime(currMeasurement.synchroTime);
            break;  
          
        case 0x03: // Assuming 0x03 represents "START"
            Serial.println("START command received");
            currMeasurement.startInterruptFlag = 1;
            currMeasurement.startInterruptTime = (uint32_t)nrf_data.time;
            break;

        case 0x04: // Assuming 0x04 represents "STOP"
            Serial.println("FINISH command received");
            currMeasurement.finishInterruptFlag = 1;
            currMeasurement.finalTime = (uint32_t)nrf_data.time;
            Serial.println(nrf_data.time);
            break;

        default:  
            break;
    }
    radio.maskIRQ(true, true, false);
    radio.startListening();

  }
}


bool synchronize_time_transmitter() {
    uint32_t receiverTime = 0;
    uint32_t roundTripTimeStart = 0;
    uint32_t roundTripTimeEnd = 0;
    uint32_t roundTripTime = 0;

    currMeasurement.NRFInterruptFlag = false;
    currMeasurement.NRFInterruptTime = 0;
    currMeasurement.synchroTime = 0;

    radio.maskIRQ(false, true, true);
    
    Data_Package nrf_data;
    nrf_data.command = 0x01;
    nrf_data.time = 0;
    radio.stopListening();
    if (!radio.write(&nrf_data, sizeof(Data_Package))) {
        Serial.println("Failed to send synchronization command.");
        return false;
    }
    roundTripTimeStart = currMeasurement.NRFInterruptTime;
    currMeasurement.synchroTime = currMeasurement.NRFInterruptTime;
    currMeasurement.NRFInterruptFlag = false;

    if (currMeasurement.synchroTime == 0) {
      Serial.println("NOT VALID INTERRUPT.");
      return false;
    }

    // Start listening for the response
    radio.maskIRQ(true, true, false);
    radio.startListening();
    
    waitForRadio(50000);
    roundTripTimeEnd = currMeasurement.NRFInterruptTime;
    Serial.println(roundTripTimeEnd);

    // Read the receiver's time
    if (radio.available()) {
      radio.read(&nrf_data, sizeof(Data_Package));
      Serial.println(nrf_data.command);
      Serial.println(nrf_data.time);

      if (nrf_data.command != 123456789) {
        Serial.println("Invalid command received.");
        receiverTime = requestTimeWithRetries(10, 50000);
      }
      else {
        receiverTime = nrf_data.time;
      }
      currMeasurement.timeDiffrence = (int32_t)currMeasurement.synchroTime - (int32_t)receiverTime;
      
      Serial.print("Transmitter time: \t");
      Serial.print(currMeasurement.synchroTime);
      Serial.print("\tReceiver time: \t");
      Serial.print(receiverTime);
      Serial.print("\tTime difference: \t");
      Serial.print(currMeasurement.timeDiffrence);
      Serial.print("\tRound trip time: \t");
      Serial.println(roundTripTimeEnd - roundTripTimeStart - 5000);
    } else {
      Serial.println("No data available.");
      return false;
    }
    return true;
}




bool synchronize_time_receiver(uint32_t last_received_interrupt_time) {
  currMeasurement.synchroTime = 0; 
  uint32_t synchro_time = last_received_interrupt_time;
  Data_Package nrf_data;
  radio.flush_tx();
  delayMicroseconds(5000);
  nrf_data.command = 123456789; // Assuming 0x01 represents "TS"
  nrf_data.time = synchro_time;
  radio.stopListening();
  if (!radio.write(&nrf_data, sizeof(Data_Package))) {
    Serial.println("Failed to send current time.");
    return false;
  }
  else {  
    Serial.print("Time sent successfullyyyyyyy.\n");
    currMeasurement.synchroTime = last_received_interrupt_time;
    Serial.println(synchro_time);
    return true;
  }
}


bool sendTime(uint32_t time) {
  Data_Package nrf_data;
  nrf_data.command = 123456789; // Assuming 0x01 represents "TS"
  nrf_data.time = time;
  delayMicroseconds(5000);
  radio.stopListening();
  if (!radio.write(&nrf_data, sizeof(Data_Package))) {
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
  while (!currMeasurement.NRFInterruptFlag && (micros() - start_time < timeout)) {}
  currMeasurement.NRFInterruptFlag = false;
  return true;
}



uint32_t requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration) {
    Data_Package nrf_data;
    uint32_t validTime = 0;
    

    for (uint8_t attempt = 0; attempt < maxRetries; ++attempt) {
        nrf_data.command = 0x02;
        nrf_data.time = 0;
        
        radio.stopListening();
        if (!radio.write(&nrf_data, sizeof(Data_Package))) {
            Serial.println("Failed to request time.");
        }

        // Start listening for response
        radio.startListening();
        waitForRadio(waitDuration);

        if (radio.available()) {
            radio.read(&nrf_data, sizeof(Data_Package));
            if (nrf_data.command == 123456789) {
                validTime = nrf_data.time;
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
    Data_Package nrf_data;
    nrf_data.command = 0x03; // Assuming 0x02 represents "START"
    nrf_data.time = (uint64_t)((int64_t)currMeasurement.startInterruptTime - (int64_t)currMeasurement.timeDiffrence);
    
    radio.stopListening();
    if (!radio.write(&nrf_data, sizeof(Data_Package))) {
        Serial.println("Failed to send start command.");
        radio.startListening();
        return false;
    }
    return true;
    radio.startListening();
}


bool sendFinishCommand() {
    Data_Package nrf_data;
    nrf_data.command = 0x04; // Assuming 0x03 represents "STOP"

    nrf_data.time = (uint64_t)currMeasurement.finalTime;

    radio.stopListening();
    if (!radio.write(&nrf_data, sizeof(Data_Package))) {
        Serial.println("Failed to send finish command.");
        radio.startListening();
        return false;
    }
    radio.startListening();
    return true;
}


bool checkIR(uint32_t timeout) {
  static uint32_t time_brake_sensor = 0;
  static bool ir_brake_active = false;

  if (digitalRead(IR_IRQ_PIN) == LOW) {
    if (!ir_brake_active) {
      time_brake_sensor = millis();
      ir_brake_active = true;
    } else if (millis() - time_brake_sensor > timeout) {
      //screen_line2 = "IRerror";
    }
  } else {
    if (ir_brake_active) {
      //screen_line2 = "";
    }
    ir_brake_active = false;
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

    measurement.NRFInterruptTime = 0;
    measurement.NRFInterruptFlag = false;

    measurement.synchroCounter = 0;
    measurement.synchroSuccess = 0;
    measurement.synchroTime = 0;
    measurement.timeDiffrence = 0;

    measurement.startInterruptTime = 0;
    measurement.finishInterruptTime = 0;
    measurement.startInterruptFlag = false;
    measurement.finishInterruptFlag = false;

    measurement.counter_m = 0;
    measurement.counter_s = 0;
    measurement.counter_ms = 0;
    measurement.finalFormatedTime = "";
    measurement.finalTime = 0;

    measurement.getUserRecieved = false;
    measurement.getRobotRecieved = false;
    measurement.getJudgeRecieved = false;
    measurement.retryJudgeRecieved = false;
    measurement.retryScoreRecieved = false;

    measurement.resetCommandSend = false;
    measurement.lastMessage = "";
  }


void checkConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
    
  }

  if (!client.connected()) {
    if (connectToServer()) {
    FirebaseJson data;
    data.set("serial_number", GATE_SERIAL_NUMBER);
    sendJsonMessage("GET_SETTINGS", data);
    }
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