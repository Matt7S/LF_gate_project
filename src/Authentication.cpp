/**
 * @file Authentication.cpp
 * @brief RFID and QR code authentication implementation.
 */

#include "Authentication.hpp"
#include <Arduino.h>
#include <cstdint>
#include <Freenove_RFID_Lib_for_Pico.h>
#include <QRScanner.hpp>
#include "constants.hpp"

extern RFID rfid;
extern QRScanner qr;

// ===== Timing Variables =====
static uint32_t lastRFIDScanTime = 0;
static uint32_t lastQRScanTime = 0;

void Authentication::initialize() {
    rfid.init();
    qr.begin();
    qr.setMode(true, true, false);
}

// ===== RFID Operations =====

bool Authentication::readRFIDCard(uint32_t scanIntervalMs, String& cardCode) {
    if (millis() - lastRFIDScanTime > scanIntervalMs) {
        cardCode = readRFIDCardRaw();
        
        if (cardCode.length() > 0) {
            Serial.print("RFID Card: ");
            Serial.println(cardCode);
            lastRFIDScanTime = millis();
            return true;
        }
        lastRFIDScanTime = millis();
    }
    
    return false;
}

String Authentication::readRFIDCardRaw() {
    digitalWrite(RC522_SS, LOW);
    rfid.antennaOn();

    static unsigned char status;
    static unsigned char str[MAX_LEN];
    String cardNumber = "";

    // Search for card
    if (rfid.findCard(PICC_REQIDL, str) == MI_OK) {
        Serial.println("RFID card found!");

        // Anti-collision detection and read card serial number
        if (rfid.anticoll(str) == MI_OK) {
            Serial.print("Card UID: ");
            for (int i = 0; i < 4; i++) {
                if (str[i] < 0x10) {
                    cardNumber += "0";
                }
                cardNumber += String(str[i], HEX);
            }
            cardNumber.toUpperCase();
            Serial.println(cardNumber);
        }

        // Card selection
        rfid.selectTag(str);
    }

    rfid.halt();
    rfid.antennaOff();
    digitalWrite(RC522_SS, HIGH);
    
    return cardNumber;
}

// ===== QR Code Operations =====

bool Authentication::readQRCode(uint32_t scanIntervalMs, String& qrCode) {
    if (millis() - lastQRScanTime > scanIntervalMs) {
        qrCode = qr.readQRCode(false);
        
        if (qrCode.length() > 0) {
            Serial.print("QR Code: ");
            Serial.println(qrCode);
            qr.setMode(true, false, false);
            lastQRScanTime = millis();
            return true;
        }
        lastQRScanTime = millis();
    }
    
    return false;
}

void Authentication::setQRMode(bool mode1, bool mode2, bool mode3) {
    qr.setMode(mode1, mode2, mode3);
}
