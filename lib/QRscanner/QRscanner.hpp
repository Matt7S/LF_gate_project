#ifndef QR_SCANNER_H
#define QR_SCANNER_H

#include <Arduino.h>
#include <Wire.h>

// Register addresses
#define READY_REG 0x0010 ///< Register address for checking if QR code is ready.
#define LENGTH_REG 0x0020 ///< Register address for reading the length of the QR code data.
#define DATA_REG 0x1000 ///< Register address for reading the QR code data.
#define TRIGGER_MODE_REG 0x0030 ///< Register address for setting the trigger mode.

/**
 * @brief Class representing a QR scanner using I2C communication.
 */
class QRScanner {
private:
    uint8_t i2cAddress; ///< I2C address of the QR scanner.
    uint8_t sdaPin; ///< SDA pin number.
    uint8_t sclPin; ///< SCL pin number.
    TwoWire *wire; ///< Pointer to the TwoWire instance.

    /**
     * @brief Enumeration for the scanning mode.
     */
    enum ScanMode {
        AUTOMATIC = 0, ///< Automatic scanning mode.
        MANUAL = 1 ///< Manual scanning mode.
    };

    ScanMode currentMode = AUTOMATIC; ///< Current scanning mode.

    /**
     * @brief Writes data to a specific register of the QR scanner.
     * 
     * @param reg The register address to write to.
     * @param data Pointer to the data to be written.
     * @param len Length of the data to be written.
     */
    void writeRegister(uint16_t reg, uint8_t *data, uint8_t len);

    /**
     * @brief Reads data from a specific register of the QR scanner.
     * 
     * @param reg The register address to read from.
     * @param data Pointer to the buffer where the read data will be stored.
     * @param len Length of the data to be read.
     */
    void readRegister(uint16_t reg, uint8_t *data, uint16_t len);

public:
    /**
     * @brief Constructor for the QRScanner class.
     * 
     * @param addr I2C address of the QR scanner.
     * @param sda SDA pin number.
     * @param scl SCL pin number.
     * @param wirePort Pointer to the TwoWire instance.
     */
    QRScanner(uint8_t addr = 0x21, uint8_t sda = 8, uint8_t scl = 9, TwoWire *wirePort = &Wire);

    /**
     * @brief Initializes the QR scanner by setting up the I2C communication.
     */
    void begin();

    /**
     * @brief Sets the mode of the QR scanner.
     * 
     * @param automatic If true, sets the scanner to automatic mode; otherwise, sets it to manual mode.
     */
    void setMode(bool automatic);

    /**
     * @brief Reads a QR code from the scanner.
     * 
     * @return A string containing the QR code data. Returns an empty string if no QR code is ready.
     */
    String readQRCode();
};

#endif