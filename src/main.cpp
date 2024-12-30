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

#define GATE_SERIAL_NUMBER 0


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
#define BUTTON_YELLOW 14
#define BUTTON_BLACK 15

// Define pins for LCD P10
#define LCD_OE 16      // Output Enable
#define LCD_A 17       // Row address A
#define LCD_B 18       // Row address B
#define LCD_CLK 19    // Clock for shifting data
#define LCD_LATCH 20  // Latch to display data
#define LCD_DATA 21   // Serial data input

#define IR_IRQ_PIN 22



struct Data_Package {
    uint32_t command;
    uint64_t time;
};



// Create radio object
extern WiFiClient client; // TCP client object\


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

volatile bool scrollable_line1 = 1;
volatile bool scrollable_line2 = 0;
volatile bool show_timer_line1 = 0;

String screen_line1 = "Welcome to Line Follower Measurement System";
String screen_line2 = "STARTING!";

//uint8_t transmiter_address[6] = "Gat00";
//uint8_t reciever_address[6] = "Gat01";

int gateID = 0;
String gateType;
int groupID = 0;
int categoryID = 0;
uint8_t gateNrfStartAddress[6];
uint8_t gateNrfFinishAddress[6];
bool isFinal = false;
bool requiredUserCard = false;
bool requiredUserQrCode = false;
bool requiredConfirmation = false;
String categoryName = "";

String cardNumber = "";
String qrCode = "";
String robotName = "";

uint32_t timeZero = 0;
int32_t timeDiffrence = 0;

uint32_t start_time_interrupt = 0;
uint32_t finish_time_interrupt = 0;
bool start_time_status = 0;
bool finish_time_status = 0;
bool waintingForCountingTime = 0;
bool countingTimeInProgress = 0;

bool startCommandSuccessfullySend = 0;
bool startCommandSuccessfullyRecieved = 0;
bool finishCommandSuccessfullySend = 0;
bool finishCommandSuccessfullyRecieved = 0;


void setup() {
    delay(2000);
    Serial.begin(115200);
    
    SPI.setSCK(SPI0_PIN_SCK);
    SPI.setTX(SPI0_PIN_MOSI);
    SPI.setRX(SPI0_PIN_MISO);
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    SPI.begin();

    pinMode(BUTTON_YELLOW, INPUT_PULLUP);
    pinMode(BUTTON_BLACK, INPUT_PULLUP); 
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
  
   // Ustawienia radia
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(100);
    radio.setRetries(15, 15);
    radio.setAutoAck(true); // Wyłącz automatyczne potwierdzenia dla wszystkich rurek
    // Enable interrupt for NRF module
    radio.maskIRQ(true, true, false);

    attachInterrupt(digitalPinToInterrupt(BUTTON_YELLOW), handleInterruptYellow, FALLING);
    attachInterrupt(digitalPinToInterrupt(NRF_INTERRUPT), handleInterrupt, FALLING);
  
}






//                                                                                                       SETUP 1


void setup1(){
  
  
  
}


//                                                                                                       LOOP 0

void loop() {
  //Serial.println(micros() - start_time_interrupt);
  checkConnection();
  handleServerResponse();


  static uint32_t time_brake_sensor = 0;
  static bool ir_brake_active = false;

  if (digitalRead(IR_IRQ_PIN) == LOW) {
    if (!ir_brake_active) {
      time_brake_sensor = millis();
      ir_brake_active = true;
    } else if (millis() - time_brake_sensor > 1000) {
      screen_line2 = "IRerror";
    }
  } else {
    if (ir_brake_active) {
      screen_line2 = "";
    }
    ir_brake_active = false;
  }



  if (gateType == "START") {
    if (!startCommandSuccessfullySend) {
      if (start_time_status) {
        show_timer_line1 = 1;
        startCommandSuccessfullySend = sendStartCommand();
        radio.startListening();
      }
    }
    if (startCommandSuccessfullySend && !finishCommandSuccessfullyRecieved) {
      //Serial.println("Słucham");
      
      listenForSignals();
    }
    
    
  } else if (gateType == "FINISH")
  {
    if (startCommandSuccessfullyRecieved) {
      show_timer_line1 = 1;
      if (start_time_status) {
        if (!finishCommandSuccessfullySend) {
          if (finish_time_interrupt) {
            finishCommandSuccessfullySend = sendFinishCommand();
            Serial.println("Wysłano finish");
            Serial.println(finishCommandSuccessfullySend);
          }
        }
        
      }
    }
    else if (!startCommandSuccessfullyRecieved){
      listenForSignals();
    } 
  }
  

    // Read RFID card data   

  
  static unsigned long repeat_rfid = 0;
  if (millis() - repeat_rfid > 500) {

    String cardNumber = readDataFromRFIDCard();

    if (cardNumber.length() > 0) {
      FirebaseJson data;
      data.set("user_rfid_code", cardNumber);
      data.set("category_id", categoryID);
      sendJsonMessage("GET_USER", data);
    }
    repeat_rfid = millis();
  }
  

  // Read QR code data
  if (QRScanner_present) {
    String tmpQrCode = qr.readQRCode(false); // Pass 'true' to filter out corrupted data
    if (tmpQrCode.length() > 0) {
      qrCode = tmpQrCode;
      Serial.print("QR Code: ");
      Serial.println(qrCode);
      qr.setMode(false);

      FirebaseJson data;
      data.set("robot_qr_code", qrCode);
      data.set("category_id", categoryID);
      data.set("isFinal", isFinal);
      sendJsonMessage("GET_ROBOT", data);
    }
  }

  

}





//                                                                                                       LOOP 1

void loop1() {
  static bool last_show_timer_line1 = 0;
    if (show_timer_line1) {
      if (last_show_timer_line1 == 0) {
        display.drawStaticText("           ", 0, 0);
        last_show_timer_line1 = 1;
      }
      else {
        updateTime();
        display.default_timer_screen(66, counter_m, counter_s, counter_ms);
      }

      } else {
      last_show_timer_line1 = 0;
      if (scrollable_line1) {
          display.scrollLine1(screen_line1, 0, 25);
      } else {
          display.drawStaticText(screen_line1, 0, 0);
      }
    }

    if (scrollable_line2) {
        display.scrollLine2(screen_line2, 8, 50);
    } else {
        display.drawStaticText(screen_line2, 0, 8);
    }

    display.refresh();
}


void handleInterrupt() {
  NRF_interrupt_time = micros();
}


void handleInterruptYellow() {

  
  static bool last_state = 0;
  yellow_pressed = true;
}

void handleInterruptBlack() {
  black_pressed = true;
}


void handleInterruptIR() {
  if (gateType == "START") {
    start_time_interrupt = (uint32_t)micros();
    start_time_status = 1;
    Serial.println("START Interrupt");
    Serial.println(start_time_interrupt);
  } else {
    finish_time_interrupt = (uint32_t)micros();
    finish_time_status = 1;
    Serial.println("FINISH Interrupt");
    Serial.println(finish_time_interrupt);
  }
  detachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN));
}



void updateTime() {
  uint32_t start = (start_time_interrupt / 1000); // Odcinamy 3 ostatnie zera (optymalnie)
  uint32_t finish = (finish_time_interrupt / 1000); // Odcinamy 3 ostatnie zera (optymalnie)

  if (start_time_status && !finish_time_status) {
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




String readDataFromRFIDCard() {
  digitalWrite(RC_522_SDA, LOW);
  //rfid.antennaOn();

  static unsigned char status;
  static unsigned char str[MAX_LEN];  // MAX_LEN is 16, the maximum length of the array
  String cardNumber = "";            // Initialize an empty String to store the card number

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

void handleServerResponse() {
    while (client.available()) {
        FirebaseJson responseJson;
        FirebaseJsonData jsonData;
        String command;
        bool synchroSuccess = false;

        // Odczytaj odpowiedź jako linię
        String response = client.readStringUntil('\n');
        response.trim(); 
        Serial.print("Otrzymano od serwera: ");
        Serial.println(response);

        // Sprawdź, czy odpowiedź to poprawny JSON
        if (!responseJson.setJsonData(response)) {
            Serial.println("Nieprawidlowy format JSON w odpowiedzi serwera.");
            continue; // Pomijamy tę iterację pętli
        }

        // Przypisanie początkowych danych ("co" i "da")
        if (!responseJson.get(jsonData, "co")) {
            Serial.println("Brak komendy w odpowiedzi.");
            continue;
        }
        command = jsonData.stringValue;

        FirebaseJson daJson;
        if (responseJson.get(jsonData, "da")) {
            daJson = FirebaseJson();
            daJson.setJsonData(jsonData.stringValue);
        } else {
            Serial.println("Brak danych 'da' w odpowiedzi.");
            continue;
        }

        Serial.print("Komenda: ");
        Serial.println(command);

        if (command == "GET_USER") {
            String user;
            if (daJson.get(jsonData, "player_name")) {user = jsonData.stringValue;}

            Serial.print("Dane GET_USER: ");
            Serial.println(user);
            scrollable_line2 = 1;

            if (user == "") {
                screen_line2 = "Unknown user or wrong category";
            } 
            else {
                screen_line2 = "Scan Robot QR code ";
                screen_line2 += user;
                qr.setMode(true);
            }

        } else if (command == "GET_ROBOT") {
            String status;
            if (daJson.get(jsonData, "robot_name")) {
                robotName = jsonData.stringValue;
                Serial.print("Status robota: ");
                Serial.println(robotName);
                if (gateType == "START") {
                    screen_line1 = "Place robot ";
                    screen_line1 += robotName;
                    screen_line1 += " on the line";
                    scrollable_line1 = 1;
                    scrollable_line2 = 0;
                    screen_line2 = gateType;
                } else
                {
                    screen_line2 = gateType;
                    scrollable_line2 = 0;
                }
            }
            if (gateType == "START") {
              synchroSuccess = synchronize_time_transmitter();
            }
            if (synchroSuccess) {
              attachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN), handleInterruptIR, FALLING);
              waintingForCountingTime = true;
              Serial.println("enable interrupt start");
            }

            

        } else if (command == "WRONG_ROBOT") {
            Serial.println("Wrong robot scanned");
            String errorMsg, errorStatusCode;
            if (daJson.get(jsonData, "status_code")) {errorStatusCode = jsonData.stringValue;}
            if (daJson.get(jsonData, "message")) {errorMsg = jsonData.stringValue;}
            Serial.println(errorMsg);
            screen_line2 = errorMsg;
            scrollable_line2 = 1;
        }
        
        
        
        else if (command == "GET_SETTINGS") {
            String startAddress, finishAddress;
            // Pobieranie danych z JSON-a
            if (daJson.get(jsonData, "gateID")) {gateID = jsonData.intValue;}
            if (daJson.get(jsonData, "gateType")) {gateType = jsonData.stringValue;}
            if (daJson.get(jsonData, "groupID")) {groupID = jsonData.intValue;}
            if (daJson.get(jsonData, "categoryID")) {categoryID = jsonData.intValue;}
            if (daJson.get(jsonData, "gateNrfStartAddress")) {startAddress = jsonData.stringValue;}
            if (daJson.get(jsonData, "gateNrfEndAddress")) {finishAddress = jsonData.stringValue;}
            if (daJson.get(jsonData, "isfinal")) {isFinal = jsonData.boolValue;}
            if (daJson.get(jsonData, "requiredUserCard")) {requiredUserCard = jsonData.boolValue;}
            if (daJson.get(jsonData, "requiredUserQrCode")) {requiredUserQrCode = jsonData.boolValue;}
            if (daJson.get(jsonData, "requiredConfirmation")) {requiredConfirmation = jsonData.boolValue;}
            if (daJson.get(jsonData, "categoryName")) {categoryName = jsonData.stringValue;}

            screen_line2 = gateType;

            // Wyświetlanie ustawień
            Serial.println("Ustawienia:");
            Serial.print("gateID: "); Serial.println(gateID);
            Serial.print("gateType: "); Serial.println(gateType);
            Serial.print("groupID: "); Serial.println(groupID);
            Serial.print("categoryID: "); Serial.println(categoryID);
            Serial.print("gateNrfStartAddress: "); Serial.println(startAddress);
            Serial.print("gateNrfEndAddress: "); Serial.println(finishAddress);
            Serial.print("requiredUserCard: "); Serial.println(requiredUserCard);
            Serial.print("requiredUserQrCode: "); Serial.println(requiredUserQrCode);
            Serial.print("requiredConfirmation: "); Serial.println(requiredConfirmation);
            Serial.print("categoryName: "); Serial.println(categoryName);

            strncpy((char*)gateNrfStartAddress, startAddress.c_str(), sizeof(gateNrfStartAddress) - 1);
            strncpy((char*)gateNrfFinishAddress, finishAddress.c_str(), sizeof(gateNrfFinishAddress) - 1);
            Serial.println((char*)gateNrfStartAddress);
            Serial.println((char*)gateNrfFinishAddress);
            
            
            setRole(gateType);

            

        } else if (command == "ERROR") {
            String errorMsg;
            if (daJson.get(jsonData, "error")) {
                errorMsg = jsonData.stringValue;
                Serial.print("Blad: ");
                Serial.println(errorMsg);
            }
        }
    }
}



void setRole(String role) {
    //currentRole = role;
    if (role == "FINISH") {
        radio.openWritingPipe(gateNrfFinishAddress);
        radio.openReadingPipe(1, gateNrfStartAddress);
        radio.startListening();
        Serial.println("Role set to RECEIVER");
    } else if (role == "START") {
        radio.openWritingPipe(gateNrfStartAddress);
        radio.openReadingPipe(1, gateNrfFinishAddress);
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
            synchronize_time_receiver(recievedTime);
            break;  

        case 0x02: // Assuming 0x02 represents"
            Serial.println("Resend synchro time command received");
            sendTime(timeZero);
            break;  
          
        case 0x03: // Assuming 0x03 represents "STOP"
            Serial.println("START command received - turn on interrupt");
            startCommandSuccessfullyRecieved = 1;
            start_time_status = 1;
            start_time_interrupt = (uint32_t)nrf_data.time;
            Serial.print("START command received local time:\t");
            Serial.print(micros());
            Serial.print("otrzymano czas:\t");
            Serial.println(start_time_interrupt);
            attachInterrupt(digitalPinToInterrupt(IR_IRQ_PIN), handleInterruptIR, FALLING);
            
            break;

        case 0x04: // Assuming 0x04 represents "RESET"
            Serial.println("FINISH command received");
            finishCommandSuccessfullyRecieved = 1;
            finish_time_status = 1;
            Serial.println(nrf_data.time);
            Serial.println(timeDiffrence);
            finish_time_interrupt = (uint32_t)((int32_t)nrf_data.time + (int32_t)timeDiffrence);
            Serial.println(finish_time_interrupt);
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
    timeZero = 0;

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
    timeZero = transmitterTimeInterrupt;
    Serial.println(roundTripTimeStart);
    NRF_interrupt = false;

    if (transmitterTime > transmitterTimeInterrupt) {
        Serial.println("NOT VALID INTERRUPT.");
        timeZero = 0;
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
        timeDiffrence = (int32_t)transmitterTimeInterrupt - (int32_t)receiverTime;
        
        Serial.print("Transmitter time: \t");
        Serial.print(transmitterTimeInterrupt);
        Serial.print("\tReceiver time: \t");
        Serial.print(receiverTime);
        Serial.print("\tTime difference: \t");
        Serial.print(timeDiffrence);
        Serial.print("\tRound trip time: \t");
        Serial.println(roundTripTimeEnd - roundTripTimeStart - 5000);
    } else {
        Serial.println("No data available.");
        return false;
    }
    return true;
}




bool synchronize_time_receiver(uint32_t last_received_interrupt_time) {
  timeZero = 0; 
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
    timeZero = last_received_interrupt_time;
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
    nrf_data.time = (uint64_t)((int64_t)start_time_interrupt - (int64_t)timeDiffrence);
    
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
    nrf_data.time = (uint64_t)finish_time_interrupt;

    radio.stopListening();
    if (!radio.write(&nrf_data, sizeof(Data_Package))) {
        Serial.println("Failed to send finish command.");
        radio.startListening();
        return false;
    }
    radio.startListening();
    return true;
}