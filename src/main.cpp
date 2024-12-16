#include <WiFi.h> // WiFi library for Pico W
#include <SPI.h>
#include <Freenove_RFID_Lib_for_Pico.h>
#include <Arduino.h>
#include <FirebaseJson.h>
#include "fonts.hpp"
#include "wifi_config.hpp"
#include "P10Display.hpp"
#include <RF24.h>
#include <nRF24L01.h>



// Definicje pinów
#define RC_522_SCK 2
#define RC_522_MOSI 3
#define RC_522_MISO 4
#define RC_522_SDA 5
#define RC_522_RST 6

#define nrf_ce_yellow 26
#define nrf_cns_orange 27

#define LCD_OE 7      // Output Enable
#define LCD_A 8       // Row address A
#define LCD_B 9       // Row address B
#define LCD_CLK 10    // Clock for shifting data
#define LCD_LATCH 11  // Latch to display data
#define LCD_DATA 12   // Serial data input

#define BUTTON_YELLOW 14
#define BUTTON_BLACK 15

#define LED_RED 13
#define LED_GREEN 22

#define BOOTSEL_PIN 23 // GPIO powiązane z przyciskiem BOOTSEL


char *gate_type = "START";
char *gate_type = "END";


// Tablica użytkownika (1 bit = 1 piksel)
//static bool user_buffer[16][32] = {0}; 


RF24 radio(nrf_ce_yellow, nrf_cns_orange); // CE, CSN
const uint8_t address[6] = "1Node"; // Adres odbiornika


volatile bool beam_brake_sensor = false;

// Zmienne do przechowywania czasu
volatile int counter_m = 0;  // Minuty
volatile int counter_s = 0;  // Sekundy
volatile int counter_ms = 0; // Milisekundy


static int BUTTON_SKIP_TIME = 500; // Debounce time for buttons
static unsigned long BUTTON_YELLOW_TIME = 200;
static unsigned long BUTTON_BLACK_TIME = 200;


const char* serverHostname = "pigate.local"; // Hostname of the server
IPAddress serverIP;                          // Store resolved IP
const int serverPort = 1234;                 // Replace with your server's port number
WiFiClient client; // TCP client object


//D10 - CS Pin、D5 - RST Pin
RFID rfid(RC_522_SDA, RC_522_RST);   
unsigned char status;
unsigned char str[MAX_LEN];  //MAX_LEN is 16, the maximum length of the array
static unsigned long replay_time = 0;


P10Display main_display(LCD_A, LCD_B, LCD_CLK, LCD_DATA, LCD_LATCH, LCD_OE);



void handleInterrupt() {
  beam_brake_sensor = true;
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
    

    // Lokalna reakcja na przycisk
    /*
    if (message == "ON") {
      digitalWrite(LED_GREEN, LOW);  // Turn green LED ON
      digitalWrite(LED_RED, HIGH);  // Turn red LED OFF
    } else if (message == "OFF") {
      digitalWrite(LED_GREEN, HIGH); // Turn green LED OFF
      digitalWrite(LED_RED, LOW);    // Turn red LED ON
    }
    */
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
  Serial.println("Reconnecting to WiFi...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nReconnected to WiFi!");
}

void reconnectToServer() {
  Serial.println("Reconnecting to the server...");
  while (!client.connect(serverIP, serverPort)) {
    delay(1000);
    Serial.println("Retrying...");
  }
  Serial.println("Reconnected to the server!");
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



void setup() {
  Serial.begin(115200);
  SPI.setRX(4);
  SPI.setCS(5);
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.begin();
  rfid.init(); //initialization

  pinMode(BUTTON_YELLOW, INPUT_PULLUP);
  pinMode(BUTTON_BLACK, INPUT_PULLUP); 
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, LOW);

  pinMode(BOOTSEL_PIN, INPUT_PULLUP); // Ustaw GPIO jako wejście z pull-up

  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, HIGH);
    Serial.print(".");
  }

  NTP.begin("pool.ntp.org", "time.nist.gov");


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



  // Connect to the server using TCP
  Serial.println("Connecting to the server...");
  if (client.connect(serverIP, serverPort)) {
    Serial.println("Connected to the server!");
  } else {
    Serial.println("Connection to the server failed!");
    delay(1000); // Stop if connection fails
  }


  //serwer.begin();
  //Serial.println("Serwer uruchomiony");
  //Serial.println(WiFi.localIP());





  // Wprowadź dane do user_buffer jako przykład
  //Serial.println(user_buffer[0][0]);


}


void setup1(){
  attachInterrupt(digitalPinToInterrupt(BUTTON_YELLOW), handleInterrupt, FALLING);

  if (!radio.begin()) {
    Serial.println("nRF24L01 initialization failed!");
    while (1);
  }
  radio.openWritingPipe(address); // Ustaw adres odbiornika
  radio.setPALevel(RF24_PA_HIGH); // Zwiększ moc
  radio.setDataRate(RF24_2MBPS); // Zmniejsz prędkość transmisji
  radio.setChannel(100); // Zmień kanał na mniej zatłoczony
  radio.setRetries(0, 0); // Retransmisja: 5 prób z odstępem 15 x 250 µs
  radio.enableAckPayload(); // Włącz odbieranie ACK payload
  Serial.println("Transmitter ready");

}






void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    reconnectWiFi();
  }
  

  handleButtonPress(BUTTON_YELLOW, "ON", BUTTON_YELLOW_TIME);
  handleButtonPress(BUTTON_BLACK, "OFF", BUTTON_BLACK_TIME);
  handleServerResponse();

  // Ensure connection to the server
  if (!client.connected()) {
    reconnectToServer();
  }

  static unsigned long repeat_rfid = 0;
  if (millis() - repeat_rfid > 500) {
    readDataFromRFIDCard();
    repeat_rfid = millis();
  }

  static unsigned int reapeat_radio = 0;
  if (millis() - reapeat_radio > 1000) {
    unsigned long sendTime = 0;
    unsigned long recievedTime = 0;
    unsigned long receiverTime = 0;
    sendTime = micros(); // Czas wysłania wiadomości
    bool success = radio.write(NULL, 0); // Wyślij pustą wiadomość
    if (success) {
      recievedTime = micros();
      bool ackReceived = radio.isAckPayloadAvailable(); // Sprawdź, czy ACK payload jest dostępny
      if (ackReceived) {
          radio.read(&receiverTime, sizeof(receiverTime)); // Odczytaj czas z ACK payload
          Serial.print("Received ACK with receiver time: ");
          Serial.println(receiverTime);
        }


      unsigned long rtt = recievedTime - sendTime;
      Serial.print("Round trip time (µs): ");
      Serial.println(rtt);
    } else {
      Serial.println("Message failed, no ACK received");
    }
    reapeat_radio = millis();
  }

  
}





void loop1() {

  

  
  updateTime();
  main_display.default_timer_screen(66, counter_m, counter_s, counter_ms);
  main_display.refresh();
}