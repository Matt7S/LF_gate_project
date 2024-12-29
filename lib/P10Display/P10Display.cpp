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
  digitalWrite(pinOE, LOW); // Włącz wyjście

  uint8_t start_idx;
  switch (refresh_row) {
      case 0:
          digitalWrite(pinA, HIGH);
          digitalWrite(pinB, HIGH);
          start_idx = 15;
          break;
      case 1:
          digitalWrite(pinA, LOW);
          digitalWrite(pinB, HIGH);
          start_idx = 14;
          break;
      case 2:
          digitalWrite(pinA, HIGH);
          digitalWrite(pinB, LOW);
          start_idx = 13;
          break;
      case 3:
          digitalWrite(pinA, LOW);
          digitalWrite(pinB, LOW);
          start_idx = 12;
          break;
  }

  for (int col_idx = 0; col_idx < 32; col_idx += 8) {
    for (int byte_idx = start_idx; byte_idx >= 0; byte_idx -= 4) {
      int end_idx = col_idx + 8;
      for (int i = col_idx; i < end_idx; i++) {
        bool transfer = !user_buffer[byte_idx][i];
        digitalWrite(pinDATA, transfer);
        digitalWrite(pinCLK, HIGH);
        digitalWrite(pinCLK, LOW);
      }
    }
  }

  digitalWrite(pinLATCH, HIGH);
  digitalWrite(pinLATCH, LOW);
  digitalWrite(pinOE, HIGH);
  delayMicroseconds(1000);

  // Aktualizacja wiersza
  refresh_row++;
  if (refresh_row == 4) {
      refresh_row = 0;
  }
}


void P10Display::default_timer(uint8_t minutes, uint8_t seconds, uint8_t milliseconds, byte output[5][32]) {
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
  addDigit(minutes / 10, 0);    // Pierwsza cyfra minut
  addDigit(minutes % 10, 4);    // Druga cyfra minut

  // Separator
  addSeparator(8);

  // Sekundy
  addDigit(seconds / 10, 10);   // Pierwsza cyfra sekund
  addDigit(seconds % 10, 14);   // Druga cyfra sekund

  // Separator
  addSeparator(18);

  // Milisekundy (tylko pierwsze trzy cyfry)
  addDigit(milliseconds / 100, 20);              // Setki milisekund
  addDigit((milliseconds / 10) % 10, 24);        // Dziesiątki milisekund
  addDigit(milliseconds % 10, 28);               // Jednostki milisekund


}



void P10Display::default_timer_screen(uint16_t refresh_time_ms, uint8_t counter_m, uint8_t counter_s, uint8_t counter_ms) {
  static uint64_t last_refresh = 0;
  
  if (millis() - last_refresh > refresh_time_ms) {
    byte pixelArray[5][32] = {0};
    default_timer(counter_m, counter_s, counter_ms, pixelArray);

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
    static uint64_t last_scroll_time = 0;
    static int scroll_offset = 0;

    static String last_text = ""; // Przechowuje ostatni tekst

    // Sprawdzenie, czy tekst się zmienił
    if (last_text != text) {
        last_text = text; // Aktualizacja ostatniego tekstu
        scroll_offset = 0; // Resetowanie przesunięcia
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
        if (scroll_offset > textWidth) {
            scroll_offset = -32; // Reset przewijania
        }
        last_scroll_time = millis();
    }
}


void P10Display::scrollLine2(const char* text, uint8_t y, uint16_t scroll_speed_ms) {
    static uint64_t last_scroll_time = 0;
    static int scroll_offset = 0;

    static String last_text = ""; // Przechowuje ostatni tekst

    if (last_text != text) {
        last_text = text; // Aktualizacja ostatniego tekstu
        scroll_offset = 0; // Resetowanie przesunięcia
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
        if (scroll_offset > textWidth) {
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