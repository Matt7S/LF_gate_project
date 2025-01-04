#include "QRscanner.hpp"
#include <Arduino.h>
#include <Wire.h>

QRScanner::QRScanner(uint8_t addr, uint8_t sda, uint8_t scl, TwoWire *wirePort)
    : i2cAddress(addr), sdaPin(sda), sclPin(scl), wire(wirePort) {}

void QRScanner::begin() {
    wire->setSDA(sdaPin);
    wire->setSCL(sclPin);
    wire->begin();
    delay(100);
}

bool QRScanner::isConnected() {
    wire->beginTransmission(i2cAddress);
    return (wire->endTransmission() == 0);
}



void QRScanner::setMode(bool clearBuffer, bool startDecoding, bool useAutomaticMode) {
    if (!isConnected()) {
        Serial.println("Device not connected! Cannot set mode.");
        return;
    }

    // 1. Czyszczenie bufora QRCode (jeśli wymagane)
    if (clearBuffer) {
        uint8_t clearStatus = 0;
        writeRegister(READY_REG, &clearStatus, 1);
        Serial.println("Cleared QRCode data buffer.");
    }

    uint8_t triggerValue = startDecoding ? 1 : 0;
    writeRegister(TRIGGER_REG, &triggerValue, 1);
    Serial.println(startDecoding ? "Started decoding manually." : "Stopped decoding manually.");

    // 2. Ustawienie trybu (automatyczny/manualny)
    uint8_t modeValue = useAutomaticMode ? 0 : 1; // 0 = Auto Trigger, 1 = Manual Trigger
    writeRegister(TRIGGER_MODE_REG, &modeValue, 1);
    Serial.println(useAutomaticMode ? "Set to automatic mode." : "Set to manual mode.");

    // 3. Jeśli manualny tryb, ustaw QRCode Trigger
    if (!useAutomaticMode) {
        currentMode = MANUAL;
    } else {
        currentMode = AUTOMATIC;
    }

    writeRegister(TRIGGER_REG, &triggerValue, 1);
    Serial.println(startDecoding ? "Started decoding manually." : "Stopped decoding manually.");


    

}





String QRScanner::readQRCode(bool filterCorrupted) {
    if (!isConnected()) {
        Serial.println("Device not connected! Cannot read QR Code.");
        return "";
    }

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
    if (!isConnected()) {
        Serial.println("Device not connected! Cannot write to register.");
        return;
    }

    wire->beginTransmission(i2cAddress);
    wire->write(reg & 0xFF);
    wire->write((reg >> 8) & 0xFF);
    for (uint8_t i = 0; i < len; i++) {
        wire->write(data[i]);
    }
    wire->endTransmission();
}

void QRScanner::readRegister(uint16_t reg, uint8_t *data, uint16_t len) {
    if (!isConnected()) {
        Serial.println("Device not connected! Cannot read from register.");
        return;
    }

    wire->beginTransmission(i2cAddress);
    wire->write(reg & 0xFF);
    wire->write((reg >> 8) & 0xFF);
    wire->endTransmission(false);
    wire->requestFrom(i2cAddress, len);
    for (uint16_t i = 0; i < len; i++) {
        data[i] = wire->read();
    }
}
