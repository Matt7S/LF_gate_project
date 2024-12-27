/*
#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

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
        uint8_t ir_irq_pin);

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

    // Inicjalizacja pinów
    void begin();

    // Wywoływana cyklicznie w loop()
    void update();

    // Przykładowa funkcja do zmiany stanu
    void changeState(State newState);

    // Przykładowa funkcja do obsługi logiki w zależności od stanu
    void handleState();

    // Dodaj inne funkcje tutaj
    String readRfid();
    void connectToWiFi();
    void reconnectWiFi();
    void reconnectToServer();
    void checkAndReconnectWiFi();
    void checkAndReconnectToSerwer();

    

private:
    
    

    char _gate_id;
    char _gate_type;

    uint8_t _spi0_sck_pin;           // Pin wejściowy
    uint8_t _spi0_mosi_pin;          // Pin wyjściowy
    uint8_t _spi0_miso_pin;          // Pin wyjściowy
    int _rc_522_sda_pin;             // Pin wejściowy
    int _rc_522_rst_pin;             // Pin wejściowy
    uint8_t _nrf_irq_pin;          // Pin wejściowy
    uint8_t _nrf_ce_pin;                 // Pin wyjściowy
    uint8_t _nrf_csn_pin;                // Pin wyjściowy
    uint8_t _qr_sda_pin;    // Pin wejściowy
    uint8_t _qr_scl_pin;    // Pin wejściowy
    uint8_t _led_red_pin;                // Pin wyjściowy
    uint8_t _led_green_pin;              // Pin wyjściowy
    uint8_t _button_yellow_pin;          // Pin wejściowy
    uint8_t _button_black_pin;           // Pin wejściowy
    uint8_t _ir_irq_pin;           // Pin wejściowy

    State _currentState;         // Aktualny stan
    unsigned long _lastChangeTime; // Ostatni czas zmiany

    IPAddress _serwerIP; // IP serwera
    WiFiClient _client; // Klient TCP
    const char* _serwerHostname = "pigate.local"; // Hostname serwera
    const int _serwerPort = 1234; // Port serwera

    RF24 _radio; // CE, CSN
    RFID _rfid; // SDA, RST
    QRScanner _qrScanner; // Adres I2C, SDA, SCL, Wire

    // Define gate type
    String gate_type = "START"; // Hostname of the server
    //char *gate_type = "END";

    // Define addresses for NRF24L01
    uint8_t transmiter_address[6] = "Gat00";
    uint8_t reciever_address[6] = "Gat01";

    // Opóźnienia dla stanów (w milisekundach)
    static constexpr unsigned long STATE_ONE_DELAY = 500; // 500 ms
    static constexpr unsigned long STATE_TWO_DELAY = 1000; // 1000 ms
};

#endif // STATEMACHINE_HPP

*/