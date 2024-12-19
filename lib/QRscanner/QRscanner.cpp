#include "QRscanner.hpp"


QRScanner::QRScanner(uint8_t addr, uint8_t sda, uint8_t scl, TwoWire *wirePort)
    : i2cAddress(addr), sdaPin(sda), sclPin(scl), wire(wirePort) {}

void QRScanner::begin() {
    wire->setSDA(sdaPin);
    wire->setSCL(sclPin);
    wire->begin();
    delay(100);
}

void QRScanner::setMode(bool automatic) {
    uint8_t modeValue = automatic ? AUTOMATIC : MANUAL;
    writeRegister(TRIGGER_MODE_REG, &modeValue, 1);
    currentMode = automatic ? AUTOMATIC : MANUAL;
}

String QRScanner::readQRCode(bool filterCorrupted) {
    uint8_t ready = 0;
    readRegister(READY_REG, &ready, 1);

    if (ready) {
        uint8_t lengthData[2] = {0};
        readRegister(LENGTH_REG, lengthData, 2);
        uint16_t length = (lengthData[1] << 8) | lengthData[0];

        if (length > 0) {
            uint8_t qrData[length];
            readRegister(DATA_REG, qrData, length);

            String qrCode = "";
            for (uint16_t i = 0; i < length; i++) {
                qrCode += (char)qrData[i];
            }

            if (filterCorrupted) {
                for (size_t i = 0; i < qrCode.length(); i++) {
                    if (qrCode[i] < 32 || qrCode[i] > 126) { // ASCII printable range
                        return ""; // Corrupted data detected
                    }
                }
            }
            return qrCode;
        }
    }
    return "";
}


void QRScanner::writeRegister(uint16_t reg, uint8_t *data, uint8_t len) {
    wire->beginTransmission(i2cAddress);
    wire->write(reg & 0xFF);
    wire->write((reg >> 8) & 0xFF);
    for (uint8_t i = 0; i < len; i++) {
        wire->write(data[i]);
    }
    wire->endTransmission();
}

void QRScanner::readRegister(uint16_t reg, uint8_t *data, uint16_t len) {
    wire->beginTransmission(i2cAddress);
    wire->write(reg & 0xFF);
    wire->write((reg >> 8) & 0xFF);
    wire->endTransmission(false);
    wire->requestFrom(i2cAddress, len);
    for (uint16_t i = 0; i < len; i++) {
        data[i] = wire->read();
    }
}


