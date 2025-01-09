#include "P10Display.hpp"
#include "fonts.hpp"

// Tablica cyfr i separatora ":"
byte digits[10][5][3] = {
  {{1, 1, 1}, {1, 0, 1}, {1, 0, 1}, {1, 0, 1}, {1, 1, 1}},  // 0
  {{0, 1, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 0}, {1, 1, 1}},  // 1
  {{1, 1, 1}, {0, 0, 1}, {1, 1, 1}, {1, 0, 0}, {1, 1, 1}},  // 2
  {{1, 1, 1}, {0, 0, 1}, {0, 1, 1}, {0, 0, 1}, {1, 1, 1}},  // 3
  {{1, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 0, 1}, {0, 0, 1}},  // 4
  {{1, 1, 1}, {1, 0, 0}, {1, 1, 1}, {0, 0, 1}, {1, 1, 1}},  // 5
  {{1, 1, 1}, {1, 0, 0}, {1, 1, 1}, {1, 0, 1}, {1, 1, 1}},  // 6
  {{1, 1, 1}, {0, 0, 1}, {0, 0, 1}, {0, 1, 0}, {0, 1, 0}},  // 7
  {{1, 1, 1}, {1, 0, 1}, {1, 1, 1}, {1, 0, 1}, {1, 1, 1}},  // 8
  {{1, 1, 1}, {1, 0, 1}, {1, 1, 1}, {0, 0, 1}, {1, 1, 1}},  // 9
};

// Separator ":" w formacie 5x1
byte separator[5] = {0, 1, 0, 1, 0};


P10Display::P10Display(uint8_t a, uint8_t b, uint8_t clk, uint8_t data, uint8_t latch, uint8_t oe)
    : pinA(a), pinB(b), pinCLK(clk), pinDATA(data), pinLATCH(latch), pinOE(oe), refresh_row(0) 
{
    // Inicjalizacja pinów w konstruktorze
    pinMode(pinOE, OUTPUT);
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(pinCLK, OUTPUT);
    pinMode(pinLATCH, OUTPUT);
    pinMode(pinDATA, OUTPUT);

    // Ustawienie początkowych stanów
    digitalWrite(pinOE, HIGH); // Wyłącz wyjście
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    digitalWrite(pinCLK, LOW);
    digitalWrite(pinLATCH, LOW);
    digitalWrite(pinDATA, LOW);

    // Wyczyść bufor
    //for (uint8_t row = 0; row < 16; row++) {
    //    for (uint8_t col = 0; col < 32; col++) {
    //        user_buffer[row][col] = false;
    //    }
    //}
}


void P10Display::refresh() {
  // wyłącz wyjście
  digitalWrite(pinOE, LOW);
  // wwybierz wierse
  digitalWrite(pinA, !(refresh_row & 0b01));
  digitalWrite(pinB, !((refresh_row & 0b10) >> 1));
  uint8_t start_idx = 15 - refresh_row;
  // prześlij dane do wierszy
  for (uint8_t col_idx = 0; col_idx < 32; col_idx += 8) {
    for (uint8_t byte_idx = start_idx; byte_idx >= 0; byte_idx -= 4) {
      for (uint8_t i = col_idx; i < col_idx + 8; i++) {
        bool transfer = !user_buffer[byte_idx][i];
        digitalWrite(pinDATA, transfer);
        digitalWrite(pinCLK, HIGH);
        delayMicroseconds(50);
        digitalWrite(pinCLK, LOW);
      }
    }
  }
  // zatrzaśnij dane
  digitalWrite(pinLATCH, HIGH);
  delayMicroseconds(50);
  digitalWrite(pinLATCH, LOW);
  // włącz wyjście
  digitalWrite(pinOE, HIGH);
  // odczekaj między wierszami
  delayMicroseconds(1000);
  // zmień wiersz
  refresh_row = (refresh_row + 1) % 4;
}


void P10Display::default_timer(byte output[5][32]) {
  // Funkcja pomocnicza do wstawiania cyfr do tablicy pikseli
  auto addDigit = [&](int digit, int colOffset) {
    for (int row = 0; row < 5; row++) {
      for (int col = 0; col < 3; col++) {
        output[row][colOffset + col] = digits[digit][row][col];
      }
    }
  };

  // Funkcja pomocnicza do wstawiania separatora ":"
  auto addSeparator = [&](int colOffset) {
    for (int row = 0; row < 5; row++) {
      output[row][colOffset] = separator[row];
    }
  };

  // Minuty
  addDigit(counter_m / 10, 0);    // Pierwsza cyfra minut
  addDigit(counter_m % 10, 4);    // Druga cyfra minut

  // Separator
  addSeparator(8);

  // Sekundy
  addDigit(counter_s / 10, 10);   // Pierwsza cyfra sekund
  addDigit(counter_s % 10, 14);   // Druga cyfra sekund

  // Separator
  addSeparator(18);

  // Milisekundy (tylko pierwsze trzy cyfry)
  addDigit(counter_ms / 100, 20);              // Setki milisekund
  addDigit((counter_ms / 10) % 10, 24);        // Dziesiątki milisekund
  addDigit(counter_ms % 10, 28);               // Jednostki milisekund


}

void P10Display::default_timer_screen() {
  static uint64_t last_refresh = 0;
  
  if (millis() - last_refresh > refresh_time_ms) {
    byte pixelArray[5][32] = {0};
    default_timer(pixelArray);

    for (int i = 0; i < 5; i++) {
      for (int j = 0; j < 32; j++) {
        user_buffer[i][j] = pixelArray[i][j];
      }
    }
    last_refresh = millis();
  }
}


void P10Display::drawStaticText(const char* text, uint8_t x, uint8_t y) {
    // Czyszczenie obszaru docelowego w buforze
    for (int i = y; i < y + 8 && i < 16; i++) { // Wysokość tekstu ograniczona do 8 pikseli
        for (int j = x; j < x + 32 && j < 32; j++) {
            user_buffer[i][j] = 0; // Czyszczenie odpowiednich pikseli
        }
    }

    // Rysowanie tekstu
    int xOffset = x;
    for (int i = 0; text[i] != '\0'; i++) {
        char c = text[i];
        if (c < 32 || c > 126) continue; // Ignoruj znaki poza zakresem ASCII

        const tChar* charInfo = Arial10_c.chars + (c - 32);
        for (uint8_t row = 0; row < charInfo->image->height; row++) {
            uint8_t rowData = charInfo->image->data[charInfo->image->height - 1 - row];
            for (uint8_t col = 0; col < charInfo->image->width; col++) {
                if (xOffset + col < 32 && y + row < 16) { // Sprawdzenie granic ekranu
                    user_buffer[y + row][xOffset + col] = (rowData >> (7 - col)) & 0x01;
                }
            }
        }
        xOffset += charInfo->image->width + 1; // Przesunięcie na kolejny znak
        if (xOffset >= 32) break; // Jeśli tekst wykracza poza ekran, przerwij
    }
}



void P10Display::scrollLine1(const char* text, uint8_t y, uint16_t scroll_speed_ms) {
    int start_offset = startingScrollOffset;
    static uint64_t last_scroll_time = 0;
    static int scroll_offset = 0;

    static String last_text = ""; // Przechowuje ostatni tekst

    // Sprawdzenie, czy tekst się zmienił
    if (last_text != String(text)) {
        last_text = String(text); // Aktualizacja ostatniego tekstu
        scroll_offset = -start_offset; // Resetowanie przesunięcia z uwzględnieniem start_offset
    }

    // Obliczenie szerokości tekstu
    int textWidth = 0;
    for (int i = 0; text[i] != '\0'; i++) {
        const tChar* charInfo = Arial10_c.chars + (text[i] - 32);
        textWidth += charInfo->image->width + 1; // Szerokość znaku + odstęp
    }

    // Czyszczenie odpowiednich linii
    for (int i = y; i < y + 8 && i < 16; i++) {
        for (int j = 0; j < 32; j++) {
            user_buffer[i][j] = 0;
        }
    }

    // Rysowanie przewijanego tekstu
    for (int i = 0, xOffset = -scroll_offset; text[i] != '\0'; i++) {
        const tChar* charInfo = Arial10_c.chars + (text[i] - 32);
        for (uint8_t row = 0; row < charInfo->image->height; row++) {
            uint8_t rowData = charInfo->image->data[charInfo->image->height - 1 - row];
            for (uint8_t col = 0; col < charInfo->image->width; col++) {
                int screenX = xOffset + col;
                if (screenX >= 0 && screenX < 32 && y + row < 16) {
                    user_buffer[y + row][screenX] = (rowData >> (7 - col)) & 0x01;
                }
            }
        }
        xOffset += charInfo->image->width + 1; // Przesunięcie na kolejny znak
    }

    // Aktualizacja scrollowania
    if (millis() - last_scroll_time > scroll_speed_ms) {
        scroll_offset++;
        if (scroll_offset > textWidth + start_offset) {
            scroll_offset = -32; // Reset przewijania
        }
        last_scroll_time = millis();
    }
}





void P10Display::scrollLine2(const char* text, uint8_t y, uint16_t scroll_speed_ms) {
    int start_offset = startingScrollOffset;
    static uint64_t last_scroll_time = 0;
    static int scroll_offset = 0;

    static String last_text = ""; // Przechowuje ostatni tekst

    // Sprawdzenie, czy tekst się zmienił
    if (last_text != String(text)) {
        last_text = String(text); // Aktualizacja ostatniego tekstu
        scroll_offset = -start_offset; // Resetowanie przesunięcia z uwzględnieniem start_offset
    }

    // Obliczenie szerokości tekstu
    int textWidth = 0;
    for (int i = 0; text[i] != '\0'; i++) {
        const tChar* charInfo = Arial10_c.chars + (text[i] - 32);
        textWidth += charInfo->image->width + 1; // Szerokość znaku + odstęp
    }

    // Czyszczenie odpowiednich linii
    for (int i = y; i < y + 8 && i < 16; i++) {
        for (int j = 0; j < 32; j++) {
            user_buffer[i][j] = 0;
        }
    }

    // Rysowanie przewijanego tekstu
    for (int i = 0, xOffset = -scroll_offset; text[i] != '\0'; i++) {
        const tChar* charInfo = Arial10_c.chars + (text[i] - 32);
        for (uint8_t row = 0; row < charInfo->image->height; row++) {
            uint8_t rowData = charInfo->image->data[charInfo->image->height - 1 - row];
            for (uint8_t col = 0; col < charInfo->image->width; col++) {
                int screenX = xOffset + col;
                if (screenX >= 0 && screenX < 32 && y + row < 16) {
                    user_buffer[y + row][screenX] = (rowData >> (7 - col)) & 0x01;
                }
            }
        }
        xOffset += charInfo->image->width + 1; // Przesunięcie na kolejny znak
    }

    // Aktualizacja scrollowania
    if (millis() - last_scroll_time > scroll_speed_ms) {
        scroll_offset++;
        if (scroll_offset > textWidth + start_offset) {
            scroll_offset = -32; // Reset przewijania
        }
        last_scroll_time = millis();
    }
}




void P10Display::drawStaticText(String text, uint8_t x, uint8_t y) {
    drawStaticText((char*)text.c_str(), x, y);
}


void P10Display::scrollLine1(String text, uint8_t y, uint16_t scroll_speed_ms) {
    scrollLine1((char*)text.c_str(), y, scroll_speed_ms);
}


void P10Display::scrollLine2(String text, uint8_t y, uint16_t scroll_speed_ms) {
    scrollLine2((char*)text.c_str(), y, scroll_speed_ms);
}


void P10Display::setLineDynamic(uint8_t line, String text, bool scroll, uint8_t speed, bool show) {
    if (line == 0) {
        line1String = text;
        line1Scroll = scroll;
        line1Speed = speed;
        line1Show = show;
        line1Showtimer = false;
    } else if (line == 1) {
        line2String = text;
        line2Scroll = scroll;
        line2Speed = speed;
        line2Show = show;
        line2Showtimer = false;
    }
}

void P10Display::setLineStatic(uint8_t line, String text, uint8_t position, bool show) {
    if (line == 0) {
        line1String = text;
        line1Scroll = false;
        line1Speed = 0;
        line1Show = show;
        line1Showtimer = false;
        line1Position = position;
    } else if (line == 1) {
        line2String = text;
        line2Scroll = false;
        line2Speed = 0;
        line2Show = show;
        line2Showtimer = false;
        line2Position = position;
    }
}

void P10Display::setTimer(uint8_t line, bool update, uint64_t givenTimeUs, uint32_t refresh_time_ms) {
    update_timer = update;
    given_time_us = givenTimeUs;
    if (line == 0) {
        line1String = "";
        line1Scroll = false;
        line1Speed = 0;
        line1Show = true;
        line1Showtimer = true;
        line1Position = 0;
        // Additional logic to set the timer values can be added here
    } else if (line == 1) {
        line1String = "";
        line2Scroll = false;
        line2Speed = 0;
        line2Show = true;
        line2Showtimer = true;
        line2Position = 0;
        // Additional logic to set the timer values can be added here
    }
}


void P10Display::clearTopPart() {
    for (int i = 0; i < 8; i++) { // Górna część to pierwsze 8 wierszy
        for (int j = 0; j < 32; j++) {
            user_buffer[i][j] = 0; // Czyszczenie pikseli
        }
    }
}

void P10Display::clearBottomPart() {
    for (int i = 8; i < 16; i++) { // Dolna część to wiersze od 8 do 15
        for (int j = 0; j < 32; j++) {
            user_buffer[i][j] = 0; // Czyszczenie pikseli
        }
    }
}


void P10Display::startBlinking(uint8_t line, uint16_t interval_ms) {
    if (line == 0) {
        line1Blink = true;
        line1BlinkInterval = interval_ms;
        line1LastBlinkTime = millis();
        line1BlinkState = true; // Początkowy stan widoczności tekstu
    } else if (line == 1) {
        line2Blink = true;
        line2BlinkInterval = interval_ms;
        line2LastBlinkTime = millis();
        line2BlinkState = true; // Początkowy stan widoczności tekstu
    }
}

void P10Display::stopBlinking(uint8_t line) {
    if (line == 0) {
        line1Blink = false;
        line1BlinkState = true; // Upewnij się, że tekst jest widoczny po zatrzymaniu migania
    } else if (line == 1) {
        line2Blink = false;
        line2BlinkState = true; // Upewnij się, że tekst jest widoczny po zatrzymaniu migania
    }
}


void P10Display::updateDisplay() {
    // Aktualizacja migania dla linii 1
    if (line1Blink) {
        if (millis() - line1LastBlinkTime > line1BlinkInterval) {
            line1BlinkState = !line1BlinkState; // Zmień stan migania
            line1LastBlinkTime = millis();
        }
    }

    // Aktualizacja migania dla linii 2
    if (line2Blink) {
        if (millis() - line2LastBlinkTime > line2BlinkInterval) {
            line2BlinkState = !line2BlinkState; // Zmień stan migania
            line2LastBlinkTime = millis();
        }
    }

    // Handle line 1
    if (line1Show) {
        if (line1Blink && !line1BlinkState) {
            clearTopPart(); // Jeśli linia ma migać i jest w stanie "ukrytym"
        } else {
            if (line1Showtimer) {
                clearTopPart();
                countTime(); // Adjust position as needed
                default_timer_screen();
            } else {
                if (line1Scroll) {
                    scrollLine1(line1String, 0, line1Speed);
                } else {
                    clearTopPart();
                    drawStaticText(line1String, line1Position, 0);
                }
            }
        }
    } else {
        clearTopPart();
    }

    // Handle line 2
    if (line2Show) {
        if (line2Blink && !line2BlinkState) {
            clearBottomPart(); // Jeśli linia ma migać i jest w stanie "ukrytym"
        } else {
            if (line2Showtimer) {
                clearBottomPart();
                countTime();
                default_timer_screen(); // Adjust position as needed
            } else {
                if (line2Scroll) {
                    scrollLine2(line2String, 8, line2Speed);
                } else {
                    clearBottomPart();
                    drawStaticText(line2String, line2Position, 8);
                }
            }
        }
    } else {
        clearBottomPart();
    }

    refresh();
}

void P10Display::countTime() {
    uint64_t given_time_ms = ((given_time_us + 500) / 1000);  
  
    if (update_timer) {
        uint64_t currentTime = (uint64_t)to_ms_since_boot(get_absolute_time());
        // Odcinamy 3 ostatnie zera (optymalnie)
        counter_m = ((currentTime - given_time_ms) / 60000) % 60;       // Minuty (każde 60 000 ms to 1 minuta)
        counter_s = ((currentTime - given_time_ms) / 1000) % 60;        // Sekundy (każde 1000 ms to 1 sekunda)
        counter_ms = (currentTime - given_time_ms) % 1000;              // Milisekundy (reszta z dzielenia przez 1000)
    }
    else {       // Odcinamy 3 ostatnie zera (optymalnie)
        counter_m = ((given_time_ms) / 60000) % 60;       // Minuty (każde 60 000 ms to 1 minuta)
        counter_s = ((given_time_ms) / 1000) % 60;        // Sekundy (każde 1000 ms to 1 sekunda)
        counter_ms = (given_time_ms) % 1000;              // Milisekundy (reszta z dzielenia przez 1000)
    }
}