/**
 * @file constants.hpp
 * @brief Project-wide constants and configuration definitions.
 * 
 * Centralized location for all pin definitions, configuration values,
 * timeouts, and other project constants.
 */

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// ========== Gate Configuration ==========
#define GATE_SERIAL_NUMBER 2

// ========== SPI Configuration ==========
/** @brief SPI0 Clock pin (GPIO 2) */
#define SPI0_PIN_SCK 2
/** @brief SPI0 MOSI pin (GPIO 3) */
#define SPI0_PIN_MOSI 3
/** @brief SPI0 MISO pin (GPIO 4) */
#define SPI0_PIN_MISO 4

// ========== RFID (RC522) Pin Configuration ==========
/** @brief RFID Slave Select / SDA pin (GPIO 5) */
#define RC522_SS 5
/** @brief RFID Reset pin (GPIO 6) */
#define RC522_RST 6

// ========== NRF24L01 Pin Configuration ==========
/** @brief NRF24 Interrupt pin (GPIO 7) */
#define NRF_INTERRUPT 7
/** @brief NRF24 Chip Enable pin (GPIO 10) */
#define NRF_CE 10
/** @brief NRF24 Chip Select pin (GPIO 11) */
#define NRF_CSN 11

// ========== Barcode/QR Scanner (I2C) Pin Configuration ==========
/** @brief QR Scanner I2C SDA pin (GPIO 8) */
#define BARCODE_SCANNER_SDA 8
/** @brief QR Scanner I2C SCL pin (GPIO 9) */
#define BARCODE_SCANNER_SCL 9

// ========== LED Pin Configuration ==========
/** @brief Red status LED pin (GPIO 12) */
#define LED_RED 12
/** @brief Green status LED pin (GPIO 13) */
#define LED_GREEN 13

// ========== Button Pin Configuration ==========
/** @brief Accept result button pin (GPIO 26) */
#define BUTTON_ACCEPT_PIN 26
/** @brief Cancel/Reset button pin (GPIO 14) */
#define BUTTON_CANCEL_PIN 14
/** @brief Forfeit result button pin (GPIO 15) */
#define BUTTON_FORFEIT_PIN 15

// ========== P10 LED Display Pin Configuration ==========
/** @brief Display Output Enable pin (GPIO 16) */
#define LCD_OE 16
/** @brief Display Row address A pin (GPIO 17) */
#define LCD_A 17
/** @brief Display Row address B pin (GPIO 18) */
#define LCD_B 18
/** @brief Display Clock/Shift pin (GPIO 19) */
#define LCD_CLK 19
/** @brief Display Latch pin (GPIO 20) */
#define LCD_LATCH 20
/** @brief Display Data pin (GPIO 21) */
#define LCD_DATA 21

// ========== IR Sensor Pin Configuration ==========
/** @brief IR sensor interrupt pin (GPIO 22) */
#define IR_IRQ_PIN 22

// ========== Timing and Delay Constants ==========
/** @brief Button debounce/skip time in milliseconds */
#define BUTTON_SKIP_TIME 500
/** @brief Serial monitor baud rate */
#define SERIAL_BAUD_RATE 115200
/** @brief SPI clock speed in Hz */
#define SPI_CLOCK_SPEED 2000000
/** @brief QR Scanner I2C address */
#define QR_SCANNER_I2C_ADDRESS 0x21

// ========== RF24 Configuration ==========
/** @brief NRF24 Data rate: 2 Mbps */
#define RF24_DATA_RATE RF24_2MBPS
/** @brief NRF24 Power level: High */
#define RF24_POWER_LEVEL RF24_PA_HIGH
/** @brief NRF24 Channel (frequency: 2400 + value MHz) */
#define RF24_CHANNEL 100
/** @brief NRF24 Retry delay (×250µs) */
#define RF24_RETRY_DELAY 15
/** @brief NRF24 Retry count */
#define RF24_RETRY_COUNT 15

// ========== Display Configuration ==========
/** @brief Display speed 1 (fast scrolling) */
#define DISPLAY_SPEED_FAST 20
/** @brief Display speed 2 (medium scrolling) */
#define DISPLAY_SPEED_MEDIUM 50
/** @brief Display speed 3 (slow scrolling) */
#define DISPLAY_SPEED_SLOW 75
/** @brief Display blink interval in milliseconds */
#define DISPLAY_BLINK_INTERVAL 250

// ========== State Machine Configuration ==========
/** @brief Maximum time synchronization attempts before error */
#define MAX_SYNC_ATTEMPTS 20
/** @brief Time synchronization error delay in milliseconds */
#define SYNC_ERROR_DELAY 3000
/** @brief Synchronization command delay in microseconds */
#define SYNC_COMMAND_DELAY 5000
/** @brief Radio wait timeout in microseconds */
#define RADIO_WAIT_TIMEOUT 50000

// ========== RF24 Command Codes ==========
/** @brief Command: Time synchronization request */
#define RF24_CMD_TIME_SYNC 0x01
/** @brief Command: Resend synchronization time */
#define RF24_CMD_RESEND_TIME 0x02
/** @brief Command: Start gate signal */
#define RF24_CMD_START 0x03
/** @brief Command: Finish gate signal */
#define RF24_CMD_FINISH 0x04
/** @brief Command: Synchronization time response marker */
#define RF24_CMD_SYNC_RESPONSE 123456789

// ========== Time Calculation Constants ==========
/** @brief Milliseconds to microseconds conversion factor */
#define MS_TO_US 1000
/** @brief Microseconds to milliseconds conversion factor */
#define US_TO_MS 1000
/** @brief Milliseconds per second */
#define MS_PER_SECOND 1000
/** @brief Milliseconds per minute */
#define MS_PER_MINUTE 60000
/** @brief Time rounding offset (for rounding to nearest millisecond) */
#define TIME_ROUNDING_OFFSET 500
/** @brief NRF24 communication delay offset in microseconds */
#define NRF_DELAY_OFFSET 167

#endif // CONSTANTS_HPP
