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

#define IR_IRQ_PIN 22



Gate gate;
Measurement currMeasurement;
State currentState = IDLE;

extern WiFiClient client; 
RF24 radio(NRF_CE, NRF_CSN);
RFID rfid(RC_522_SDA, RC_522_RST);  
QRScanner qr(0x21, BARCODE_SCANNER_SDA, BARCODE_SCANNER_SCL, &Wire); // Adres I2C, SDA, SCL, Wire
P10Display display(LCD_A, LCD_B, LCD_CLK, LCD_DATA, LCD_LATCH, LCD_OE);


//
volatile bool black_pressed = false;
volatile bool yellow_pressed = false;
volatile bool nr24_interrupt = false;

// Timer variables
volatile int counter_m = 0;  // Minuty
volatile int counter_s = 0;  // Sekundy
volatile int counter_ms = 0; // Milisekundy

// Button debounce time
static int BUTTON_SKIP_TIME = 500; // Debounce time for buttons

// Global variable to store interrupt time and interrupt source
volatile bool NRF_interrupt = false;
volatile uint32_t NRF_interrupt_time = 0;

volatile bool QRScanner_present = false;

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
    sendJsonMessage("GATE_HELLO", data);
    
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
  checkConnection();
  String serwerCommand;
  handleServerResponse(serwerCommand);

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
      display.setLineStatic(1, gate.type, 1, true);

      if (processRFIDCard(500, currMeasurement.cardNumber)) 
      {
          FirebaseJson message;
          message.set("user_rfid_code", currMeasurement.cardNumber);
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
      if (currMeasurement.userFound) 
      {
        if (waitForNextState(3000)) 
        {
          currentState = TIME_SYNCHRONIZATION;
          Serial.println("TIME_SYNCHRONIZATION");
        }

      }
      else {
        display.setLineDynamic(0, "Error! Unknown user or wrong category!", true, display_speed1, true);
        display.setLineStatic(1, gate.type, 1, true);
        if (waitForNextState(3000)) 
        {
          currentState = RESET_WAITING;
          Serial.println("RESET_WAITING");
        }
      }
    }

    break;
  }
  
    
  
  //                                       CASE TIME_SYNCHRONIZATION
  case TIME_SYNCHRONIZATION:
  { 
    
    // Handle time synchronization state
    if (gate.type == "START") {
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
    screen_line1 += currMeasurement.userName;
    screen_line1 += " Please scan Robot QR code";
    display.setLineDynamic(0, screen_line1, true, display_speed1, true);
    display.setLineStatic(1, gate.type, 1, true);
    if (gate.type == "START") 
    {
      if (processQRCode(100, currMeasurement.qrCode)) 
      {
        FirebaseJson data;
        data.set("robot_qr_code", currMeasurement.qrCode);
        data.set("category_id", gate.categoryID);
        data.set("isFinal", gate.final);
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
      if (currMeasurement.robotFound) 
      {
        String screen_line1 = "Hello ";
        screen_line1 += currMeasurement.robotName;
        display.setLineDynamic(0, screen_line1, true, display_speed1, true);
        display.setLineStatic(1, gate.type, 1, true);
        if (waitForNextState(2000)) 
        {
          currentState = START_WAITING;
          Serial.println("START_WAITING");

          if (gate.type == "START") 
          {
            attachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN), handleInterruptIR, FALLING);
            Serial.println("enable interrupt start");
            String screen_line1 = "Place robot ";
            screen_line1 += currMeasurement.robotName;
            screen_line1 += " on the start gate and good luck!";
            display.setLineDynamic(0, screen_line1, true, display_speed1, true);
            display.setLineStatic(1, gate.type, 1, true);
            display.startBlinking(1, 250);
          }
          else 
          {
            String screen_line1 = "Place robot ";
            screen_line1 += currMeasurement.robotName;
            screen_line1 += " on the start gate and good luck!";
            display.setLineDynamic(0, "Plan", true, display_speed1, true);
            display.setLineStatic(1, gate.type, 1, true);
          }
          //screen_line2 = gate.type;
        }
      }
      else 
      {
        display.setLineDynamic(0, currMeasurement.wrongRobotMessage, true, display_speed1, true);
        display.setLineStatic(1, gate.type, 1, true);
        if (waitForNextState(3000)) 
        {
          currentState = RESET_WAITING;
          Serial.println("RESET_WAITING");
        }
      }
    }
    break;
  }
    
    
  //                                       CASE START_WAITING
  case START_WAITING: // Handle start waiting state
    if (gate.type == "START") {
      if(currMeasurement.start_time_status) 
      {
        currentState = STARTED;
        Serial.println("STARTED");
        display.stopBlinking(0);
        display.clearTopPart();
        display.setTimer(0, 150);
        display.setLineDynamic(1, "Run in progress!", true, display_speed1, true);
        display.stopBlinking(1);
      }
    } else {
      listenForSignals();
      if (currMeasurement.start_time_status)
      {
        attachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN), handleInterruptIR, FALLING);
        currentState = STARTED;
        Serial.println("STARTED");
        radio.stopListening();
        display.clearTopPart();
        display.setTimer(0, 150);
        display.setLineDynamic(1, "Run in progress!", true, display_speed1, true);
      }
    }

      break;

  //                                       CASE STARTED    
  case STARTED: // Handle started state
  {
    if (gate.type == "START") 
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
    if (gate.type == "START") 
    {
      listenForSignals();
      if (currMeasurement.finish_time_status) 
      {
        currentState = FINISHED;
        Serial.println("FINISHED");
        //show_timer_line1 = 1;
      }
    } 
    else 
    {
      if (currMeasurement.finish_time_status) 
      {
        currentState = FINISHED;
        Serial.println("FINISHED"); 
        //show_timer_line1 = 1;
      }
      
    }
      
    break;
  }
    
  //                                       CASE FINISHED
  case FINISHED: // Handle finished state
  {
    if (gate.type == "START") 
    {
      // only show time
      currentState = CONFIRMATION;
      Serial.println("CONFIRMATION");
    } 
    else 
    {
      if (sendFinishCommand()) 
      {
        currentState = CONFIRMATION;
        Serial.println("CONFIRMATION");
      }
    }
    break;
  }
    
  case CONFIRMATION: // Handle confirmation state
  {
    display.setLineDynamic(1, "Run is finished please confirm by Judge!", true, display_speed3, true);
    if (digitalRead(BUTTON_ACCEPT_PIN) == LOW) 
    {
      //FirebaseJson data;
      //data.set("ACCEPT", true);
      //sendJsonMessage("CONFIRMATION", data);
      currentState = RESET_WAITING;
      Serial.println("RESET_WAITING");
    } 
    else if (digitalRead(BUTTON_CANCEL_PIN) == LOW) 
    {
      //FirebaseJson data;
      //data.set("ACCEPT", false);
      //sendJsonMessage("CONFIRMATION", data);
      currentState = RESET_WAITING;
      Serial.println("RESET_WAITING");
    }
    else if (digitalRead(BUTTON_FORFEIT_PIN) == LOW) 
    {
      //FirebaseJson data;
      //data.set("ACCEPT", false);
      //sendJsonMessage("CONFIRMATION", data);
      currentState = RESET_WAITING;
      Serial.println("RESET_WAITING");
    }
    break;
  }
  //                                       CASE RESET_WAITING
  case RESET_WAITING: // Handle reset waiting state
  {
    FirebaseJson data;
    data.set("RESET", true);
    sendJsonMessage("RESET", data);
    resetMeasurement(currMeasurement);
    display.stopBlinking(0);
    display.stopBlinking(1);
    break;
  }
    
  //                                       CASE RESETING
  case RESETING: // Handle reset state
  {
    Serial.println("RESETING");
    qr.setMode(false);
    currentState = IDLE;
    break;
  }
    
  //                                       DEFAULT
  default:  // Handle unknown state
  {
    Serial.println("WTF");
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
  updateTime();
  display.updateDisplay();
  display.updateTime(counter_m, counter_s, counter_ms);
}


void handleInterrupt() {
  NRF_interrupt_time = micros();
}


void handleInterruptIR() {
  if (gate.type == "START") {
    currMeasurement.start_time_interrupt = (uint32_t)micros();
    currMeasurement.start_time_status = 1;
    Serial.println("START Interrupt");
    Serial.println(currMeasurement.start_time_interrupt);
  } else {
    currMeasurement.finish_time_interrupt = (uint32_t)micros();
    currMeasurement.finish_time_status = 1;
    Serial.println("FINISH Interrupt");
    Serial.println(currMeasurement.finish_time_interrupt);
  }
  detachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN));
}



void updateTime() {
  uint32_t start = (currMeasurement.start_time_interrupt / 1000); // Odcinamy 3 ostatnie zera (optymalnie)
  uint32_t finish = (currMeasurement.finish_time_interrupt / 1000); // Odcinamy 3 ostatnie zera (optymalnie)

  if (currMeasurement.start_time_status && !currMeasurement.finish_time_status) {
    unsigned long currentMillis = millis(); // Aktualny czas w ms od startu programu
    counter_m = ((currentMillis-start) / 60000) % 60;       // Minuty (każde 60 000 ms to 1 minuta)
    counter_s = ((currentMillis-start) / 1000) % 60;        // Sekundy (każde 1000 ms to 1 sekunda)
    counter_ms = (currentMillis-start) % 1000;              // Milisekundy (reszta z dzielenia przez 1000)
  }
  else {
    counter_m = ((finish-start) / 60000) % 60;       // Minuty (każde 60 000 ms to 1 minuta)
    counter_s = ((finish-start) / 1000) % 60;        // Sekundy (każde 1000 ms to 1 sekunda)
    counter_ms = (finish-start) % 1000;              // Milisekundy (reszta z dzielenia przez 1000)
  }
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

bool handleServerResponse(String command) {
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
        //continue; // Pomijamy tę iterację pętli
    }

    // Przypisanie początkowych danych ("co" i "da")
    if (!responseJson.get(jsonData, "co")) {
        Serial.println("Brak komendy w odpowiedzi.");
        return false;
        //continue;
    }
    command = jsonData.stringValue;

    FirebaseJson daJson;
    if (responseJson.get(jsonData, "da")) {
        daJson = FirebaseJson();
        daJson.setJsonData(jsonData.stringValue);
    } else {
        Serial.println("Brak danych 'da' w odpowiedzi.");
        return false;
        //continue;
    }

    Serial.print("Komenda: ");
    Serial.println(command);

    if (command == "GET_USER") 
    {
      currMeasurement.getUserRecieved = true;
      if (daJson.get(jsonData, "player_name")) 
      {
        currMeasurement.userName = jsonData.stringValue;
      }
      
      Serial.print("Dane GET_USER: ");
      Serial.println(currMeasurement.userName);

      if (currMeasurement.userName == "") 
      {
          currMeasurement.userFound = false;
      } 
      else 
      {
          currMeasurement.userFound = true;
      }

    } 
    else if (command == "GET_ROBOT") 
    {
      currMeasurement.getRobotRecieved = true;
      currMeasurement.robotFound = true;
      String status;
      if (daJson.get(jsonData, "robot_name"))  {currMeasurement.robotName = jsonData.stringValue;}
      Serial.print("Status robota: ");
      Serial.println(currMeasurement.robotName);
      
    } 
    else if (command == "WRONG_ROBOT") 
    {
      currMeasurement.getRobotRecieved = true;
      currMeasurement.robotFound = false;
      Serial.println("Wrong robot scanned");

      if (daJson.get(jsonData, "status_code")) {currMeasurement.wrongRobotCommand = jsonData.stringValue;}
      if (daJson.get(jsonData, "message")) {currMeasurement.wrongRobotMessage = jsonData.stringValue;}
      Serial.println(currMeasurement.wrongRobotMessage);

    } 
    else if (command == "GET_SETTINGS") 
    {
      String startAddress, finishAddress;
      if (daJson.get(jsonData, "gateID")) {gate.ID = jsonData.intValue;}
      if (daJson.get(jsonData, "gateType")) {gate.type = jsonData.stringValue;}
      if (daJson.get(jsonData, "groupID")) {gate.groupID = jsonData.intValue;}
      if (daJson.get(jsonData, "categoryID")) {gate.categoryID = jsonData.intValue;}
      if (daJson.get(jsonData, "gateNrfStartAddress")) {
        startAddress = jsonData.stringValue;
        strncpy((char*)gate.startAddress, startAddress.c_str(), sizeof(gate.startAddress) - 1);
      }
      if (daJson.get(jsonData, "gateNrfEndAddress")) {
        finishAddress = jsonData.stringValue;
        strncpy((char*)gate.finishAddress, finishAddress.c_str(), sizeof(gate.finishAddress) - 1);
      }
      if (daJson.get(jsonData, "isFinal")) {gate.final = jsonData.boolValue;}
      if (daJson.get(jsonData, "requiredUserCard")) {gate.requiredUserCard = jsonData.boolValue;}
      if (daJson.get(jsonData, "requiredUserQrCode")) {gate.requiredUserQrCode = jsonData.boolValue;}
      if (daJson.get(jsonData, "requiredConfirmation")) {gate.requiredConfirmation = jsonData.boolValue;}
      if (daJson.get(jsonData, "isActive")) {gate.active = jsonData.boolValue;}
      if (daJson.get(jsonData, "categoryName")) {gate.categoryName = jsonData.stringValue;}
      // Wyświetlanie ustawień
      Serial.println("Ustawienia:");
      Serial.print("gateID: "); Serial.println(gate.ID);
      Serial.print("gateType: "); Serial.println(gate.type);
      Serial.print("groupID: "); Serial.println(gate.groupID);
      Serial.print("categoryID: "); Serial.println(gate.categoryID);
      Serial.print("gateNrfStartAddress: "); Serial.println(startAddress);
      Serial.print("gateNrfEndAddress: "); Serial.println(finishAddress);
      Serial.print("requiredUserCard: "); Serial.println(gate.requiredUserCard);
      Serial.print("requiredUserQrCode: "); Serial.println(gate.requiredUserQrCode);
      Serial.print("requiredConfirmation: "); Serial.println(gate.requiredConfirmation);
      Serial.print("isActive: "); Serial.println(gate.active);
      Serial.print("categoryName: "); Serial.println(gate.categoryName);

      setRole(gate.type);

    } else if (command == "RESET_RESULT") {
      currentState = RESETING;

    } else {
      Serial.println("Nieznana komenda.");
      return false;
    }
  }
  return true;
}



void setRole(String role) {
    //currentRole = role;
    if (role == "FINISH") {
        radio.openWritingPipe(gate.finishAddress);
        radio.openReadingPipe(1, gate.startAddress);
        radio.startListening();
        Serial.println("Role set to RECEIVER");
    } else if (role == "START") {
        radio.openWritingPipe(gate.startAddress);
        radio.openReadingPipe(1, gate.finishAddress);
        radio.stopListening();
        Serial.println("Role set to TRANSMITTER");
    }
}


void listenForSignals() {
  // Odbierz dane
  if (radio.available()) {
    uint8_t receivedCommand = 0;
    uint32_t recievedTime = NRF_interrupt_time;
    Data_Package nrf_data;
    Serial.println("Data available");

    //radio.stopListening();
    radio.read(&nrf_data, sizeof(Data_Package));

    switch (nrf_data.command) {
        case 0x01: // Assuming 0x01 represents "TS"
            if (recievedTime == 0) {
                Serial.println("No time received by interrupt??");
                break;
            }
            Serial.println("Time synchronization command received");
            currMeasurement.synchroSuccess = synchronize_time_receiver(recievedTime);
            break;  

        case 0x02: // Assuming 0x02 represents"
            Serial.println("Resend synchro time command received");
            sendTime(currMeasurement.timeZero);
            break;  
          
        case 0x03: // Assuming 0x03 represents "START"
            Serial.println("START command received");
            currMeasurement.start_time_status = 1;
            currMeasurement.start_time_interrupt = (uint32_t)nrf_data.time;
            break;

        case 0x04: // Assuming 0x04 represents "STOP"
            Serial.println("FINISH command received");
            currMeasurement.finish_time_status = 1;
            currMeasurement.finish_time_interrupt = (uint32_t)((int32_t)nrf_data.time + (int32_t)currMeasurement.timeDiffrence);
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
    uint32_t transmitterTimeInterrupt = 0;
    uint32_t transmitterTime = micros();
    uint32_t receiverTime = 0;
    uint32_t roundTripTimeStart = 0;
    uint32_t roundTripTimeEnd = 0;
    uint32_t roundTripTime = 0;

    radio.maskIRQ(false, true, true);
    
    
    
    // Send synchronization command ("TS")
    uint32_t command_synchronize = 0x01; // Assuming 0x01 represents "TS"
    Data_Package nrf_data;
    nrf_data.command = command_synchronize;
    nrf_data.time = 0;
    radio.stopListening();
    if (!radio.write(&nrf_data, sizeof(Data_Package))) {
        Serial.println("Failed to send synchronization command.");
        return false;
    }
    //Serial.println("Synchronization command sent.");
    transmitterTimeInterrupt = NRF_interrupt_time;
    roundTripTimeStart = transmitterTimeInterrupt;
    currMeasurement.timeZero = transmitterTimeInterrupt;
    Serial.println(roundTripTimeStart);
    NRF_interrupt = false;

    if (transmitterTime > transmitterTimeInterrupt) {
        Serial.println("NOT VALID INTERRUPT.");
        currMeasurement.timeZero = 0;
        return false;
    }

    // Start listening for the response
    radio.maskIRQ(true, true, false);
    radio.startListening();
    
    waitForRadio(50000);
    roundTripTimeEnd = NRF_interrupt_time;
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

        //receiverTime = nrf_data.time;
        currMeasurement.timeDiffrence = (int32_t)transmitterTimeInterrupt - (int32_t)receiverTime;
        
        Serial.print("Transmitter time: \t");
        Serial.print(transmitterTimeInterrupt);
        Serial.print("\tReceiver time: \t");
        Serial.print(receiverTime);
        Serial.print("\tTime difference: \t");
        Serial.print(currMeasurement.timeDiffrence);
        Serial.print("\tRound trip time: \t");
        Serial.println(roundTripTimeEnd - roundTripTimeStart - 5000);
    } else {
        Serial.println("No data available.");
        currMeasurement.timeZero = 0;
        return false;
    }
    return true;
}




bool synchronize_time_receiver(uint32_t last_received_interrupt_time) {
  currMeasurement.timeZero = 0; 
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
    currMeasurement.timeZero = last_received_interrupt_time;
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
  while (!NRF_interrupt && (micros() - start_time < timeout)) {}
  NRF_interrupt = false;
  return true;
}



uint32_t requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration) {
    Data_Package nrf_data;
    uint32_t validTime = 0;
    

    for (uint8_t attempt = 0; attempt < maxRetries; ++attempt) {
        // Send command 0x02 to request time
        nrf_data.command = 0x02; // Assuming 0x02 represents "START"
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
    nrf_data.time = (uint64_t)((int64_t)currMeasurement.start_time_interrupt - (int64_t)currMeasurement.timeDiffrence);
    
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
    nrf_data.time = (uint64_t)currMeasurement.finish_time_interrupt;

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
    measurement.cardNumber = "";
    measurement.userName = "";
    measurement.qrCode = "";
    measurement.robotName = "";

    measurement.synchroCounter = 0;
    measurement.synchroSuccess = 0;
    measurement.timeZero = 0;
    measurement.timeDiffrence = 0;

    measurement.start_time_interrupt = 0;
    measurement.finish_time_interrupt = 0;
    measurement.start_time_status = false;
    measurement.finish_time_status = false;

    measurement.getUserRecieved = false;
    measurement.userFound = false;

    measurement.getRobotRecieved = false;
    measurement.robotFound = false;
    measurement.wrongRobotCommand = "";
    measurement.wrongRobotMessage = "";
  }