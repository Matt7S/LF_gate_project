/**
 * @file Authentication.hpp
 * @brief RFID and QR code authentication interface.
 *
 * Manages scanner operations for:
 * - RFID card reader (RC522)
 * - QR code scanner (I2C)
 */

#ifndef AUTHENTICATION_HPP
#define AUTHENTICATION_HPP

#include <Arduino.h>

/**
 * @class Authentication
 * @brief Handles RFID and QR code scanning.
 */
class Authentication {
public:
    /**
     * @brief Initialize RFID and QR scanners.
     */
    static void initialize();

    // ===== RFID Operations =====
    /**
     * @brief Read RFID card with debouncing.
     * @param scanIntervalMs Minimum time between scan attempts (milliseconds)
     * @param cardCode Reference to store card UID
     * @return True if new card was read, false otherwise
     */
    static bool readRFIDCard(uint32_t scanIntervalMs, String& cardCode);

    // ===== QR Code Operations =====
    /**
     * @brief Read QR code with debouncing.
     * @param scanIntervalMs Minimum time between scan attempts (milliseconds)
     * @param qrCode Reference to store QR code data
     * @return True if new QR code was read, false otherwise
     */
    static bool readQRCode(uint32_t scanIntervalMs, String& qrCode);

    /**
     * @brief Set QR scanner mode/operation.
     * @param mode1 Scanner mode parameter 1
     * @param mode2 Scanner mode parameter 2
     * @param mode3 Scanner mode parameter 3
     */
    static void setQRMode(bool mode1, bool mode2, bool mode3);

private:
    // Private constructor (static methods only)
    Authentication() = delete;

    /**
     * @brief Read raw RFID card data.
     * @return Card UID as uppercase hex string
     */
    static String readRFIDCardRaw();
};

#endif  // AUTHENTICATION_HPP
