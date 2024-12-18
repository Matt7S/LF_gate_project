#include <Arduino.h>
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


// spi0
#define SPI0_SCK_PIN 2
#define SPI0_MOSI_PIN 3
#define SPI0_MISO_PIN 4

#define RC_522_SDA_PIN 5
#define RC_522_RST_PIN 6

#define LCD_OE_PIN 7      // Output Enable
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
#define nrf_ce_yellow 26
#define nrf_cns_orange 27


// Klasa dla zarządzania stanami i logiką
class GateDevice {
public:
    // Konstruktor
    GateDevice(uint8_t inputPin, uint8_t outputPin)
        : _inputPin(inputPin), _outputPin(outputPin), _currentState(INITIAL_SETUP) {}

    // Inicjalizacja pinów
    void begin() {
        pinMode(_inputPin, INPUT_PULLUP);
        pinMode(_outputPin, OUTPUT);
        digitalWrite(_outputPin, LOW);
    }

    // Wywoływana cyklicznie w loop()
    void update() {
        unsigned long currentMillis = millis(); // Aktualny czas

        switch (_currentState) {
            case INITIAL_SETUP:


            case CONNECTING_WIFI:
                if (digitalRead(_inputPin) == LOW) { // Warunek wejścia
                    _lastChangeTime = currentMillis;
                    _currentState = STATE_ONE;
                }
                break;

            case STATE_ONE:
                if (currentMillis - _lastChangeTime > STATE_ONE_DELAY) { // Nieblokujące opóźnienie
                    digitalWrite(_outputPin, HIGH); // Zmiana stanu wyjścia
                    _lastChangeTime = currentMillis;
                    _currentState = STATE_TWO;
                }
                break;

            case STATE_TWO:
                if (currentMillis - _lastChangeTime > STATE_TWO_DELAY) {
                    digitalWrite(_outputPin, LOW); // Reset wyjścia
                    _currentState = IDLE; // Powrót do IDLE
                }
                break;

            default:
                _currentState = IDLE; // Bezpieczny powrót
                break;
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
        GET_GATE_DATA,
        READY_TO_START_PROCESS,
        WAITING_FOR_RFID,
        WAITING_FOR_QR_CODE,
        WAITING_FOR_START,
        WAITING_FOR_END,
        WAITING_FOR_CONFIRMATION_BY_JUDGE,
        SEND_DATA_INTO_SERWER,
        READY_TO_FINISH_PROCESS
    };

    char _gate_id;
    char _gate_type;

    uint8_t _inputPin;           // Pin wejściowy
    uint8_t _outputPin;          // Pin wyjściowy
    State _currentState;         // Aktualny stan
    unsigned long _lastChangeTime; // Ostatni czas zmiany

    // Opóźnienia dla stanów (w milisekundach)
    static constexpr unsigned long STATE_ONE_DELAY = 500; // 500 ms
    static constexpr unsigned long STATE_TWO_DELAY = 1000; // 1000 ms
};
