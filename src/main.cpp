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
#define BUTTON_YELLOW 14
#define BUTTON_BLACK 15

// Define pins for LCD P10
#define LCD_OE 16      // Output Enable
#define LCD_A 17       // Row address A
#define LCD_B 18       // Row address B
#define LCD_CLK 19    // Clock for shifting data
#define LCD_LATCH 20  // Latch to display data
#define LCD_DATA 21   // Serial data input


enum Role { RECEIVER, TRANSMITTER };
Role currentRole = RECEIVER;

String gate_type = "START"; // Hostname of the server
uint8_t transmiter_address[6] = "Gat00";
uint8_t reciever_address[6] = "Gat01";



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



void setup() {
    delay(2000);
    Serial.begin(115200);
    
    Serial.println(gate_type);
    SPI.setSCK(SPI0_PIN_SCK);
    SPI.setTX(SPI0_PIN_MOSI);
    SPI.setRX(SPI0_PIN_MISO);
    SPI.begin();

    pinMode(BUTTON_YELLOW, INPUT_PULLUP);
    pinMode(BUTTON_BLACK, INPUT_PULLUP); 
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
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
    attachInterrupt(digitalPinToInterrupt(BUTTON_BLACK), handleInterruptBlack, FALLING);
    attachInterrupt(digitalPinToInterrupt(NRF_INTERRUPT), handleInterrupt, FALLING);
  
}






//                                                                                                       SETUP 1


void setup1(){
  
  
  
}


//                                                                                                       LOOP 0

void loop() {
  checkConnection();
  handleServerResponse();


  if (Serial.available()) {
        char key = Serial.read();
        FirebaseJson data;

        switch (key) {
            case '1':
                data.set("message", "Predefiniowana wiadomosc 1");
                sendJsonMessage("GET_SETTINGS", data);
                break;
            case '2':
                data.set("message", "Predefiniowana wiadomosc 2");
                sendJsonMessage("GET_USER", data);
                break;
            case '3':
                data.set("message", "Predefiniowana wiadomosc 3");
                sendJsonMessage("GET_ROBOT", data);
                break;
            default:
                Serial.println("Nieznany klawisz. Uzyj 1, 2 lub 3.");
                break;
        }
    }


  
  static unsigned long repeat_rfid = 0;
  if (millis() - repeat_rfid > 500) {
    digitalWrite(RC_522_SDA, LOW);
    readDataFromRFIDCard();
    digitalWrite(RC_522_SDA, HIGH);
    repeat_rfid = millis();
  }
  

  // Read QR code data
  if (QRScanner_present) {
    String qrCode = qr.readQRCode(true); // Pass 'true' to filter out corrupted data
    if (qrCode.length() > 0) {
      Serial.print("QR Code: ");
      Serial.println(qrCode);
    }
  }
}





//                                                                                                       LOOP 1

void loop1() {
  updateTime();
  display.default_timer_screen(66, counter_m, counter_s, counter_ms);
  
  //display.drawStaticText("Static", 0, 0); // Wyświetl "Static" od pozycji (5, 4)
  display.scrollText("Line Follower Standard!", 8, 25); // Przewijanie od linii 8 z prędkością 100 ms

  display.refresh();
}











void handleInterrupt() {
  NRF_interrupt_time = micros();
  //Serial.println("NRF24L01 interrupt!");
}





void handleInterruptYellow() {
  static bool last_state = 0;
  yellow_pressed = true;
}

void handleInterruptBlack() {
  Serial.println("Black button pressed");
  static bool last_state = 0;
  black_pressed = true;

}



void updateTime() {
  unsigned long currentMillis = millis(); // Aktualny czas w ms od startu programu

  counter_m = (currentMillis / 60000) % 60;       // Minuty (każde 60 000 ms to 1 minuta)
  counter_s = (currentMillis / 1000) % 60;        // Sekundy (każde 1000 ms to 1 sekunda)
  counter_ms = currentMillis % 1000;              // Milisekundy (reszta z dzielenia przez 1000)
}









void readDataFromRFIDCard() {
  static unsigned char status;
  static unsigned char str[MAX_LEN];  //MAX_LEN is 16, the maximum length of the array
  // Search card, return card types
  if (rfid.findCard(PICC_REQIDL, str) == MI_OK) {
    Serial.println("Find the card!");

    // Anti-collision detection, reading card serial number
    if (rfid.anticoll(str) == MI_OK) {
      Serial.print("The card's number is: ");
      for (int i = 0; i < 4; i++) {
        Serial.printf("%02X", str[i]);
      }
      Serial.println();
    }
    // Card selection
    rfid.selectTag(str);
  }


  rfid.halt(); // Command the card to enter sleep mode
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

        if (command == "CUSTOM_MESSAGE") {
            String response;
            if (daJson.get(jsonData, "response")) {
                response = jsonData.stringValue;
                Serial.print("Dane CUSTOM_MESSAGE: ");
                Serial.println(response);
            }
        } else if (command == "ROBOT_INFO") {
            String status;
            if (daJson.get(jsonData, "status")) {
                status = jsonData.stringValue;
                Serial.print("Status robota: ");
                Serial.println(status);
            }
        } else if (command == "GET_SETTINGS") {
            // Deklaracja zmiennych
            int gateID = 0;
            String gateType;
            int groupID = 0;
            int categoryID = 0;
            String gateNrfStartAddress;
            String gateNrfEndAddress;
            bool requiredUserCard = false;
            bool requiredUserQrCode = false;
            bool requiredConfirmation = false;

            // Pobieranie danych z JSON-a
            if (daJson.get(jsonData, "gateID")) {
                gateID = jsonData.intValue;
            }
            if (daJson.get(jsonData, "gateType")) {
                gateType = jsonData.stringValue;
            }
            if (daJson.get(jsonData, "groupID")) {
                groupID = jsonData.intValue;
            }
            if (daJson.get(jsonData, "categoryID")) {
                categoryID = jsonData.intValue;
            }
            if (daJson.get(jsonData, "gateNrfStartAddress")) {
                gateNrfStartAddress = jsonData.stringValue;
            }
            if (daJson.get(jsonData, "gateNrfEndAddress")) {
                gateNrfEndAddress = jsonData.stringValue;
            }
            if (daJson.get(jsonData, "requiredUserCard")) {
                requiredUserCard = jsonData.boolValue;
            }
            if (daJson.get(jsonData, "requiredUserQrCode")) {
                requiredUserQrCode = jsonData.boolValue;
            }
            if (daJson.get(jsonData, "requiredConfirmation")) {
                requiredConfirmation = jsonData.boolValue;
            }

            // Wyświetlanie ustawień
            Serial.println("Ustawienia:");
            Serial.print("gateID: "); Serial.println(gateID);
            Serial.print("gateType: "); Serial.println(gateType);
            Serial.print("groupID: "); Serial.println(groupID);
            Serial.print("categoryID: "); Serial.println(categoryID);
            Serial.print("gateNrfStartAddress: "); Serial.println(gateNrfStartAddress);
            Serial.print("gateNrfEndAddress: "); Serial.println(gateNrfEndAddress);
            Serial.print("requiredUserCard: "); Serial.println(requiredUserCard);
            Serial.print("requiredUserQrCode: "); Serial.println(requiredUserQrCode);
            Serial.print("requiredConfirmation: "); Serial.println(requiredConfirmation);

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



void setRole(Role role) {
    currentRole = role;
    if (role == RECEIVER) {
        radio.openWritingPipe(reciever_address);
        radio.openReadingPipe(1, transmiter_address);
        radio.startListening();
        Serial.println("Role set to RECEIVER");
    } else if (role == TRANSMITTER) {
        radio.openWritingPipe(transmiter_address);
        radio.openReadingPipe(1, reciever_address);
        radio.stopListening();
        Serial.println("Role set to TRANSMITTER");
    }
}


void listenForSignals() {
  // Odbierz dane
  if (radio.available()) {
    uint8_t receivedCommand = 0;
    uint32_t recievedTime = NRF_interrupt_time;

    radio.stopListening();
    radio.read(&receivedCommand, sizeof(receivedCommand));

    switch (receivedCommand) {
        case 0x01: // Assuming 0x01 represents "TS"
            if (recievedTime == 0) {
                Serial.println("No time received by interrupt??");
                break;
            }
            synchronize_time_receiver(transmiter_address, reciever_address, recievedTime);
            break;    

        default:  
            break;
    }
    radio.maskIRQ(true, true, false);
    radio.startListening();

  }
}


bool synchronize_time_transmitter(uint8_t *transmitter_address, uint8_t *receiver_address) {
    unsigned long transmitterTimeInterrupt = 0;
    unsigned long transmitterTime = micros();
    unsigned long receiverTime = 0;
    unsigned long roundTripTimeStart = 0;
    unsigned long roundTripTimeEnd = 0;
    unsigned long roundTripTime = 0;

    radio.maskIRQ(false, true, true);
    radio.stopListening();
    
    
    // Send synchronization command ("TS")
    uint8_t command_synchronize = 0x01; // Assuming 0x01 represents "TS"
    if (!radio.write(&command_synchronize, sizeof(command_synchronize))) {
        Serial.println("Failed to send synchronization command.");
        return false;
    }
    transmitterTimeInterrupt = NRF_interrupt_time;
    NRF_interrupt = false;

    if (transmitterTime > transmitterTimeInterrupt) {
        Serial.println("NOT VALID INTERRUPT.");
        return false;
    }

    // Start listening for the response
    radio.maskIRQ(true, true, false);
    radio.startListening();
    
    waitForRadio(50000);
    roundTripTimeEnd = NRF_interrupt_time;

    // Read the receiver's time
    if (radio.available()) {
        radio.read(&receiverTime, sizeof(receiverTime));
        long timeDiff = (long)receiverTime - (long)transmitterTimeInterrupt;
        roundTripTimeStart = transmitterTimeInterrupt;
        Serial.print("Transmitter time: \t");
        Serial.print(transmitterTimeInterrupt);
        Serial.print("\tReceiver time: \t");
        Serial.print(receiverTime);
        Serial.print("\tTime difference: \t");
        Serial.print(timeDiff);
        Serial.print("\tRound trip time: \t");
        Serial.println(roundTripTimeEnd - roundTripTimeStart-10000);
    } else {
        Serial.println("No data available.");
        return false;
    }

    return true;
}

bool synchronize_time_receiver(uint8_t *transmitter_address, uint8_t *receiver_address, uint32_t last_received_interrupt_time) {
  uint32_t synchro_time = last_received_interrupt_time;
  radio.flush_tx();
  delayMicroseconds(10000);
  
  if (!radio.write(&synchro_time, sizeof(synchro_time))) {
    Serial.println("Failed to send current time.");
    return false;
  }
  else {  
    Serial.print("Time sent successfully.\n");
    Serial.println(synchro_time);
    return true;
  }
}



bool waitForRadio(unsigned long timeout) {
  unsigned long start_time = micros();
  while (!NRF_interrupt && (micros() - start_time < timeout)) {}
  NRF_interrupt = false;
  return true;
}

