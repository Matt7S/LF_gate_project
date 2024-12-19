#include <Arduino.h>
#include <WiFi.h> // WiFi library for Pico W
#include <SPI.h>
#include <Freenove_RFID_Lib_for_Pico.h>
#include <FirebaseJson.h>
#include "fonts.hpp"
#include "wifi_config.hpp"
#include "P10Display.hpp"
#include <RF24.h>
#include <nRF24L01.h>
#include <Wire.h>

/*
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





// Define gate type
String gate_type = "START"; // Hostname of the server
//char *gate_type = "END";

// Define addresses for NRF24L01
uint8_t transmiter_address[6] = "Gat00";
uint8_t reciever_address[6] = "Gat01";

// Create radio object
RF24 radio(NRF_CE, NRF_CSN); // CE, CSN


volatile bool black_pressed = false;
volatile bool yellow_pressed = false;
volatile bool nr24_interrupt = false;

// Timer variables
volatile int counter_m = 0;  // Minuty
volatile int counter_s = 0;  // Sekundy
volatile int counter_ms = 0; // Milisekundy

// Button debounce time
static int BUTTON_SKIP_TIME = 500; // Debounce time for buttons
static unsigned long BUTTON_YELLOW_TIME = 200;
static unsigned long BUTTON_BLACK_TIME = 200;

// WiFi and server variables
const char* serverHostname = "pigate.local"; // Hostname of the server
IPAddress serverIP;                          // Store resolved IP
const int serverPort = 1234;                 // Replace with your server's port number
WiFiClient client; // TCP client object


// RFID variables
RFID rfid(RC_522_SDA, RC_522_RST);   
unsigned char status;
unsigned char str[MAX_LEN];  //MAX_LEN is 16, the maximum length of the array
static unsigned long replay_time = 0;

// Display object
P10Display main_display(LCD_A, LCD_B, LCD_CLK, LCD_DATA, LCD_LATCH, LCD_OE);

uint64_t nrf_interrupt_time = 0;

// Function prototypes
void handleInterrupt();
void handleInterruptYellow();
void handleInterruptBlack();
void updateTime();
void handleButtonPress(int buttonPin, const String& message, unsigned long& lastPressTime);
void handleServerResponse();

void connectToWiFi();
void reconnectWiFi();
void reconnectToServer();
void checkAndReconnectWiFi();
void checkAndReconnectToSerwer();
void readDataFromRFIDCard();
bool synchronize_time(uint8_t *transmiter_address, uint8_t *reciever_address);
bool waitForRadio(unsigned long timeout);



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

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, LOW);

  rfid.init(); //initialization
  //radio.begin();

  
  while (!radio.begin()) {
    Serial.println("nRF24L01 initialization failed!");
    delay(1000);
  }
  Serial.println("nRF24L01 initialized!");

  radio.maskIRQ(false, true, true);

  //connectToWiFi();
  //reconnectToServer();
  attachInterrupt(digitalPinToInterrupt(BUTTON_YELLOW), handleInterruptYellow, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_BLACK), handleInterruptBlack, FALLING);


  
  Serial2.setTX(BARCODE_SCANNER_TX);
  Serial2.setRX(BARCODE_SCANNER_RX);
  Serial2.begin(9600);
  Serial.println("Serial1 initialized!");


}






//                                                                                                       SETUP 1


void setup1(){
  
  
}


//                                                                                                       LOOP 0

void loop() {
  //checkAndReconnectWiFi();
  //checkAndReconnectToSerwer();
  //handleButtonPress(BUTTON_YELLOW, "ON", BUTTON_YELLOW_TIME);
  //handleButtonPress(BUTTON_BLACK, "OFF", BUTTON_BLACK_TIME);
  //handleServerResponse();
  //Serial.println("welll");
  attachInterrupt(digitalPinToInterrupt(BUTTON_BLACK), handleInterruptBlack, FALLING);

  if (black_pressed) {
    black_pressed = false;
    synchronize_time(transmiter_address, reciever_address);
  }
  

  static unsigned long repeat_rfid = 0;
  if (millis() - repeat_rfid > 500) {
    readDataFromRFIDCard();
    repeat_rfid = millis();
  }

  if (Serial2.available()) {
    String barcode = Serial1.readStringUntil('\n');
    Serial.println(barcode);
  }

}





//                                                                                                       LOOP 1

void loop1() {
  updateTime();
  main_display.default_timer_screen(66, counter_m, counter_s, counter_ms);
  main_display.refresh();
}











void handleInterrupt() {
  nrf_interrupt_time = micros();
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




void handleButtonPress(int buttonPin, const String& message, unsigned long& lastPressTime) {
  if ((millis() - lastPressTime) > BUTTON_SKIP_TIME && digitalRead(buttonPin) == LOW) {
    client.println(message); // Send data
    Serial.print("Sent: ");
    Serial.println(message);
    lastPressTime = millis();
    replay_time = micros();
    
  }
}

void handleServerResponse() {
  while (client.available()) {

    String response = client.readStringUntil('\n');
    response.trim(); // Remove newline and whitespace
    Serial.println(micros() - replay_time);
    replay_time = 0;
    Serial.print("Received: ");
    Serial.println(response);
    if (response == "ON") {
      digitalWrite(LED_GREEN, LOW);  // Turn green LED ON
      digitalWrite(LED_RED, HIGH);  // Turn red LED OFF
    } else if (response == "OFF") {
      digitalWrite(LED_GREEN, HIGH); // Turn green LED OFF
      digitalWrite(LED_RED, LOW);    // Turn red LED ON
    }
  }
}

void reconnectWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("Retrying...");
  }
  Serial.println("\nConnected to WiFi!");
}

void reconnectToServer() {
  Serial.println("Connecting to the server...");
  while (!client.connect(serverIP, serverPort)) {
    delay(1000);
    Serial.println("Retrying...");
  }
  Serial.println("Connected to the server!");
}


void checkAndReconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
  }
}

void checkAndReconnectToSerwer() {
if (!client.connected()) {
    reconnectToServer();
  }

}



void readDataFromRFIDCard() {
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



void connectToWiFi() {
  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    Serial.print(".");
  }


  Serial.println("\nConnected to WiFi!");
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);

  Serial.print("Device IP Address: ");
  Serial.println(WiFi.localIP());

  // Resolve server hostname to IP address
  if (WiFi.hostByName(serverHostname, serverIP)) {
    Serial.print("Resolved IP Address of ");
    Serial.print(serverHostname);
    Serial.print(": ");
    Serial.println(serverIP);
  } else {
    Serial.println("Hostname resolution failed!");
    delay(1000); // Stop if resolution fails
    WiFi.hostByName(serverHostname, serverIP);
    Serial.print("Resolved IP Address of ");
    Serial.print(serverHostname);
    Serial.print(": ");
    Serial.println(serverIP);
  }
}



bool synchronize_time(uint8_t *transmitter_address, uint8_t *receiver_address) {
    detachInterrupt(digitalPinToInterrupt(BUTTON_BLACK));
    Serial.println("Synchronizing time...");
    uint64_t sendData1 = 0, sendData2 = 0, receiverTime = 0;
    int64_t timeDiff = 0;

    // Ustawienia radia
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_2MBPS);
    radio.setChannel(100);
    radio.setRetries(0, 0);
    radio.openWritingPipe(receiver_address);
    radio.openReadingPipe(1, transmitter_address);

    // Synchronizacja - wysłanie sygnału "TS"
    radio.stopListening();
    const char *command_synchronize = "TS";
    if (!radio.write(command_synchronize, strlen(command_synchronize) + 1)) {
        Serial.println("Failed to send command!");
        return false;
    }

    Serial.println("Wysłano1, czekanie na potwierdzenie!");
    radio.startListening();
    char receivedText[32] = "";

    if (!waitForRadio(1000)) {
        return false;
    }

    radio.read(&receivedText, sizeof(receivedText));
    if (String(receivedText) != "OK") {
        return false;
    }

    Serial.println("Receiver ready...");
    radio.stopListening();

    // Wysyłanie czasu
    radio.maskIRQ(false, true, true);
    attachInterrupt(digitalPinToInterrupt(NRF_INTERRUPT), handleInterrupt, FALLING);
    //sendData1 = micros();
    
    if (!radio.write(NULL, 0)) {
        return false;
    }
    sendData2 = micros();
    sendData1 = nrf_interrupt_time;
    detachInterrupt(digitalPinToInterrupt(NRF_INTERRUPT));

    radio.startListening();
    if (!waitForRadio(1000)) {
        return false;
    }

    // Odbiór czasu z odbiornika
    radio.read(&receiverTime, sizeof(receiverTime));
    timeDiff = (int64_t)receiverTime - (int64_t)sendData1;

    // Obsługa różnicy czasu
    Serial.print("Receiver is ");
    Serial.print(timeDiff > 0 ? "ahead by " : "behind by ");
    Serial.print(abs(timeDiff));
    Serial.println(" microseconds");

    return true;
}

bool waitForRadio(unsigned long timeout) {
    unsigned long start_time = micros();
    while (!radio.available() && micros() - start_time < timeout) {}
    return radio.available();
}



*/


#include <QRscanner.hpp>

QRScanner qrScanner(0x21, 8, 9, &Wire); // Adres I2C, SDA, SCL, Wire

void setup() {
    Serial.begin(115200);
    qrScanner.begin();
    Serial.println("QR Scanner Initialized.");
    Serial.println("Enter 'a' for Automatic mode or 'm' for Manual mode.");
}

void loop() {
    // Check for user input to toggle mode
    if (Serial.available() > 0) {
        char command = Serial.read();
        if (command == 'a') {
            qrScanner.setMode(true); // Automatic mode
            Serial.println("Switched to Automatic Mode");
        } else if (command == 'm') {
            qrScanner.setMode(false); // Manual mode
            Serial.println("Switched to Manual Mode");
        }
    }

    // Read QR code data
    String qrCode = qrScanner.readQRCode();
    if (qrCode.length() > 0) {
        Serial.print("QR Code: ");
        Serial.println(qrCode);
    }

    delay(100);
}