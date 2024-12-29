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
    void default_timer(uint8_t minutes, uint8_t seconds, uint8_t milliseconds, byte output[5][32]);


    /**
     * @brief Wyświetla dane z bufora na matrycy LED.
     * @param minutes Czas w minutach
     * @param seconds Czas w sekundach
     * @param miliseconds Czas w milisekundach
     * @param output Opowiednio ustawione piksele timera 5x32
     */
    void default_timer_screen(uint16_t refresh_time_ms, uint8_t counter_m, uint8_t counter_s, uint8_t counter_ms);

    void drawStaticText(const char* text, uint8_t x, uint8_t y);
    void drawStaticText(String text, uint8_t x, uint8_t y);

    void scrollLine1(const char* text, uint8_t y, uint16_t scroll_speed_ms);
    void scrollLine1(String text, uint8_t y, uint16_t scroll_speed_ms);

    void scrollLine2(const char* text, uint8_t y, uint16_t scroll_speed_ms);
    void scrollLine2(String text, uint8_t y, uint16_t scroll_speed_ms);

    
};

#endif // P10_DRIVER_HPP
