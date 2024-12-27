/*

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
#include <QRScanner.hpp>






// Klasa dla zarządzania stanami i logiką
class GateDevice {
public:
    // Konstruktor
    GateDevice (
        uint8_t spi0_sck_pin, 
        uint8_t spi0_mosi_pin, 
        uint8_t spi0_miso_pin, 
        uint8_t rc_522_sda_pin, 
        uint8_t rc_522_rst_pin, 
        uint8_t nrf_irq_pin, 
        uint8_t nrf_ce_pin, 
        uint8_t nrf_csn_pin, 
        uint8_t qr_sda_pin, 
        uint8_t qr_scl_pin, 
        uint8_t led_red_pin, 
        uint8_t led_green_pin, 
        uint8_t button_yellow_pin, 
        uint8_t button_black_pin, 
        uint8_t ir_irq_pin) : 
        _spi0_sck_pin(spi0_sck_pin), _spi0_mosi_pin(spi0_mosi_pin), _spi0_miso_pin(spi0_miso_pin), 
        _rc_522_sda_pin(rc_522_sda_pin), _rc_522_rst_pin(rc_522_rst_pin), 
        _nrf_irq_pin(nrf_irq_pin), _nrf_ce_pin(nrf_ce_pin), _nrf_csn_pin(nrf_csn_pin), 
        _qr_sda_pin(qr_sda_pin), _qr_scl_pin(qr_scl_pin), 
        _led_red_pin(led_red_pin), _led_green_pin(led_green_pin), 
        _button_yellow_pin(button_yellow_pin), _button_black_pin(button_black_pin), 
        _ir_irq_pin(ir_irq_pin),
        _rfid(_rc_522_sda_pin, rc_522_rst_pin) {
        //_rfid(_rc_522_sda_pin, rc_522_rst_pin), _qrScanner(0x21, qr_sda_pin, rc_522_rst_pin, &Wire), _radio(nrf_ce_pin, nrf_csn_pin) {
            
        _currentState = INITIAL_SETUP;
    }

    

    // Inicjalizacja pinów
    void begin() {
        SPI.setSCK(_spi0_sck_pin); // Pin wejściowy
        SPI.setTX(_spi0_mosi_pin); // Pin wyjściowy
        SPI.setRX(_spi0_miso_pin); // Pin wyjściowy
        SPI.begin();



        pinMode(_nrf_irq_pin, INPUT_PULLUP); // Pin wejściowy
        pinMode(_led_red_pin, OUTPUT); // Pin wyjściowy
        pinMode(_led_green_pin, OUTPUT); // Pin wyjściowy
        pinMode(_button_yellow_pin, INPUT_PULLUP); // Pin wejściowy
        pinMode(_button_black_pin, INPUT_PULLUP); // Pin wejściowy
        pinMode(_ir_irq_pin, INPUT_PULLUP); // Pin wejściowy

        digitalWrite(_led_red_pin, LOW); // Reset LED
        digitalWrite(_led_green_pin, LOW); // Reset LED

        //_rfid = RFID _rfid(_rc_522_sda_pin, _rc_522_rst_pin);   // Create RFID object
        //_qrScanner = QRScanner _qrScanner(0x21, _qr_sda_pin, _rc_522_rst_pin, &Wire); // Adres I2C, SDA, SCL, Wire
        //_radio = RF24 _radio(_nrf_ce_pin, _nrf_csn_pin); // CE, CSN

        while (!_radio.begin()) {
            Serial.println("nRF24L01 initialization failed!");
            delay(1000);
        }

        _rfid.init(); //initialization
        _qrScanner.begin();

        
    }

    // Wywoływana cyklicznie w loop()
    void update() {
        unsigned long currentMillis = millis(); // Aktualny czas
        String qrCode = _qrScanner.readQRCode(true); // Pass 'true' to filter out corrupted data
        readRfid();
        if (qrCode.length() > 0) {
            Serial.print("QR Code: ");
            Serial.println(qrCode);
  }

        switch (_currentState) {
            case INITIAL_SETUP:
                Serial.println("INITIAL_SETUP");

                break;


            case CONNECTING_WIFI:
                Serial.println("CONNECTING_WIFI");
                connectToWiFi();
                reconnectToServer();
                
                break;

            case CONNECTED_WIFI:
                Serial.println("CONNECTED_WIFI");
                
                break;

            case CONNECTING_SERWER:
                Serial.println("CONNECTING_SERWER");
                
                break;

            case SETUP_RADIO:
                _radio.setPALevel(RF24_PA_HIGH);
                _radio.setDataRate(RF24_1MBPS);
                _radio.setChannel(100);
                _radio.setRetries(0, 0);
                _radio.setAutoAck(false); // Wyłącz automatyczne potwierdzenia dla wszystkich rurek

            default:

                _currentState = CONNECTING_WIFI; // Bezpieczny powrót
                break;
        }
    }

    String readRfid() {
        // Search card, return card types
        static unsigned char status;
        static unsigned char str[MAX_LEN];

        if (_rfid.findCard(PICC_REQIDL, str) == MI_OK) {
            Serial.println("Find the card!");

            // Anti-collision detection, reading card serial number
            if (_rfid.anticoll(str) == MI_OK) {
                Serial.print("The card's number is: ");
                for (int i = 0; i < 4; i++) {
                    Serial.printf("%02X", str[i]);
                }
                Serial.println();
            }
            // Card selection
            _rfid.selectTag(str);
        }
        _rfid.halt(); // Command the card to enter sleep mode
        return "Hello";
    }

    void connectToWiFi() {
        // Connect to WiFi
        Serial.print("Connecting to WiFi...");
        WiFi.begin(_WIFI_SSID, _WIFI_PASSWORD);
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.print(".");
        }


        Serial.println("\nConnected to WiFi!");

        Serial.print("Device IP Address: ");
        Serial.println(WiFi.localIP());

        // Resolve server hostname to IP address
        if (WiFi.hostByName(_serwerHostname, _serwerIP)) {
            Serial.print("Resolved IP Address of ");
            Serial.print(_serwerHostname);
            Serial.print(": ");
            Serial.println(_serwerIP);
        } else {
            Serial.println("Hostname resolution failed!");
            delay(1000); // Stop if resolution fails
            WiFi.hostByName(_serwerHostname, _serwerIP);
            Serial.print("Resolved IP Address of ");
            Serial.print(_serwerHostname);
            Serial.print(": ");
            Serial.println(_serwerIP);
        }
    }



    void reconnectWiFi() {
        Serial.println("Connecting to WiFi...");
        WiFi.disconnect();
        WiFi.begin(_WIFI_SSID, _WIFI_PASSWORD);
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.print("Retrying...");
        }
        Serial.println("\nConnected to WiFi!");
    }

    void reconnectToServer() {
        Serial.println("Connecting to the server...");
        while (!_client.connect(_serwerIP, _serwerPort)) {
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
        if (!_client.connected()) {
            reconnectToServer();
        }

    }

private:
    // Stany
    enum State {
        INITIAL_SETUP,
        CONNECTING_WIFI,
        CONNECTED_WIFI,
        CONNECTING_SERWER,
        CONNECTED_SERWER,
        SETUP_RADIO,
        GET_GATE_DATA,
        READY_TO_START_PROCESS,
        WAITING_FOR_RFID,
        WAITING_FOR_QR_CODE,
        WAITING_FOR_START,
        WAITING_FOR_END,
        WAITING_FOR_CONFIRMATION_BY_JUDGE,
        SEND_DATA_INTO_SERWER,
        READY_TO_FINISH_PROCESS,
    };

    //RFID _rfid;
    //QRScanner _qrScanner;
    RF24 _radio;

    uint8_t _spi0_sck_pin;
    uint8_t _spi0_mosi_pin;
    uint8_t _spi0_miso_pin;
    uint8_t _rc_522_sda_pin;  
    uint8_t _rc_522_rst_pin;
    uint8_t _nrf_irq_pin;      
    uint8_t _nrf_ce_pin;             
    uint8_t _nrf_csn_pin;            
    uint8_t _qr_sda_pin;
    uint8_t _qr_scl_pin;
    uint8_t _led_red_pin;     
    uint8_t _led_green_pin;   
    uint8_t _button_yellow_pin;
    uint8_t _button_black_pin;
    uint8_t _ir_irq_pin;

    State _currentState;         // Aktualny stan
    unsigned long _lastChangeTime; // Ostatni czas zmiany

    const char* _WIFI_SSID = "605a_lab";
    const char* _WIFI_PASSWORD = "ElektroSzmelcMateusza7!";

    IPAddress _serwerIP; // IP serwera
    WiFiClient _client; // Klient TCP
    const char* _serwerHostname = "pigate.local"; // Hostname serwera
    const int _serwerPort = 1234; // Port serwera

    
    
    // Define gate type
    String _gate_type = "START";
    //char *gate_type = "END";

    // Define addresses for NRF24L01
    uint8_t _transmiter_address[6] = "Gat00";
    uint8_t _reciever_address[6] = "Gat01";


    // Opóźnienia dla stanów (w milisekundach)
    static constexpr unsigned long STATE_ONE_DELAY = 500; // 500 ms
    static constexpr unsigned long STATE_TWO_DELAY = 1000; // 1000 ms
};

*/