#ifndef P10_DRIVER_HPP
#define P10_DRIVER_HPP

#include <Arduino.h>




class P10Display {
private:
    // Numery pinów
    uint8_t pinA;
    uint8_t pinB;
    uint8_t pinCLK;
    uint8_t pinDATA;
    uint8_t pinLATCH;
    uint8_t pinOE;


    String line1String = "";
    String line2String = "";
    bool line1Scroll = false;
    bool line2Scroll = false;
    uint8_t line1Speed = 0;
    uint8_t line2Speed = 0;
    bool line1Show = true;
    bool line2Show = true;
    bool line1Showtimer = false;
    bool line2Showtimer = false;
    uint8_t line1Position = 0;
    uint8_t line2Position = 0;
    // Miganie linii 1
    bool line1Blink = false;               // Czy linia 1 ma migać
    bool line1BlinkState = true;           // Aktualny stan widoczności linii 1
    uint16_t line1BlinkInterval = 500;     // Interwał migania linii 1 w ms
    uint64_t line1LastBlinkTime = 0;       // Czas ostatniej zmiany stanu migania linii 1

    // Miganie linii 2
    bool line2Blink = false;               // Czy linia 2 ma migać
    bool line2BlinkState = true;           // Aktualny stan widoczności linii 2
    uint16_t line2BlinkInterval = 500;     // Interwał migania linii 2 w ms
    uint64_t line2LastBlinkTime = 0;       // Czas ostatniej zmiany stanu migania linii 2


    uint8_t startingScrollOffset = 20;

    uint16_t refresh_time_ms; 
    uint8_t counter_m;
    uint8_t counter_s;
    uint8_t counter_ms;

    bool update_timer = false;
    uint32_t given_time_us = 0;


    // Aktualny wiersz wyświetlacza
    uint8_t refresh_row = 0;
    bool user_buffer[16][32] = {}; 



public:
    /**
     * @brief Konstruktor przyjmujący numery pinów.
     * @param a Pin A (adres wiersza bit 0)
     * @param b Pin B (adres wiersza bit 1)
     * @param clk Pin zegara
     * @param data Pin danych
     * @param latch Pin blokady (latch)
     * @param oe Pin włączania/wyłączania wyjścia (Output Enable)
     */
    P10Display(uint8_t a, uint8_t b, uint8_t clk, uint8_t data, uint8_t latch, uint8_t oe);


    /**
     * @brief Wyświetla dane z bufora na matrycy LED.
     * @param userBuffer Bufor danych do wyświetlenia (16x32 piksele).
     */
    void refresh();


    /**
     * @brief Wyświetla dane z bufora na matrycy LED.
     * @param minutes Czas w minutach
     * @param seconds Czas w sekundach
     * @param miliseconds Czas w milisekundach
     * @param output Opowiednio ustawione piksele timera 5x32
     */
    void default_timer(byte output[5][32]);


    /**
     * @brief Wyświetla dane z bufora na matrycy LED.
     * @param minutes Czas w minutach
     * @param seconds Czas w sekundach
     * @param miliseconds Czas w milisekundach
     * @param output Opowiednio ustawione piksele timera 5x32
     */
    void default_timer_screen();

    void drawStaticText(const char* text, uint8_t x, uint8_t y);
    void drawStaticText(String text, uint8_t x, uint8_t y);

    void scrollLine1(const char* text, uint8_t y, uint16_t scroll_speed_ms);
    void scrollLine1(String text, uint8_t y, uint16_t scroll_speed_ms);
    void scrollLine2(const char* text, uint8_t y, uint16_t scroll_speed_ms);
    void scrollLine2(String text, uint8_t y, uint16_t scroll_speed_ms);

    void setLineDynamic(uint8_t line, String text, bool scroll, uint8_t speed, bool show);
    void setLineStatic(uint8_t line, String text, uint8_t position, bool show);
    void setTimer(uint8_t line, bool update, uint32_t givenTimerUs, uint32_t refresh_time_ms);
    void updateDisplay();
    void clearTopPart();
    void clearBottomPart();
    void startBlinking(uint8_t line, uint16_t interval_ms);
    void stopBlinking(uint8_t line);
    void countTime();

    
};

#endif // P10_DRIVER_HPP
