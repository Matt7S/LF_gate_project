/**
 * @file RadioCommunication.cpp
 * @brief Implementation of NRF24L01 radio communication module.
 *
 * Implements time synchronization, command transmission, and signal
 * listening for wireless communication between start and finish gates.
 */

#include "RadioCommunication.hpp"

void RadioCommunication::listenForSignals() {
    if (radio.available()) {
        Serial.println("Radio data available");

        DataPackage nrfData;
        radio.read(&nrfData, sizeof(DataPackage));

        switch (nrfData.command) {
            // Time synchronization request from transmitter
            case RF24_CMD_TIME_SYNC:
                if (MeasurementManager::getCurrentMeasurement().nrfInterruptTime == 0) {
                    Serial.println("Error: No interrupt timestamp available");
                    break;
                }
                Serial.println("Time synchronization request received");
                Serial.println(nrfData.time);
                MeasurementManager::getCurrentMeasurement().syncSuccess = synchronizeTimeReceiver(MeasurementManager::getCurrentMeasurement().nrfInterruptTime);
                break;

            // Request to resend synchronization time
            case RF24_CMD_RESEND_TIME:
                Serial.println("Resend sync time request received");
                sendTime(MeasurementManager::getCurrentMeasurement().syncTime);
                break;

            // Start gate signal (finish gate receives this)
            case RF24_CMD_START:
                Serial.println("START signal received");
                MeasurementManager::getCurrentMeasurement().startInterruptFlag = true;
                MeasurementManager::getCurrentMeasurement().startInterruptTime = nrfData.time;
                break;

            // Finish gate signal (start gate receives this)
            case RF24_CMD_FINISH:
                Serial.println("FINISH signal received");
                MeasurementManager::getCurrentMeasurement().finishInterruptFlag = true;
                MeasurementManager::getCurrentMeasurement().finalTime = nrfData.time;
                Serial.println(nrfData.time);
                break;

            default:
                break;
        }
        radio.maskIRQ(true, true, false);
        radio.startListening();
    }
}

bool RadioCommunication::synchronizeTimeTransmitter() {
    uint64_t receiverTime = 0;
    uint64_t roundTripTimeStart = 0;
    uint64_t roundTripTimeEnd = 0;
    uint64_t roundTripTime = 0;

    MeasurementManager::getCurrentMeasurement().nrfInterruptFlag = false;
    MeasurementManager::getCurrentMeasurement().nrfInterruptTime = 0;
    MeasurementManager::getCurrentMeasurement().syncTime = 0;

    radio.maskIRQ(false, true, true);
    delay(10);
    
    DataPackage nrfData;
    nrfData.command = RF24_CMD_TIME_SYNC;
    nrfData.time = micros();
    Serial.println("Sending synchronization command");
    Serial.println(nrfData.time);
    radio.stopListening();
    if (!radio.write(&nrfData, sizeof(DataPackage))) {
        Serial.println("Failed to send synchronization command.");
        return false;
    }
    roundTripTimeStart = MeasurementManager::getCurrentMeasurement().nrfInterruptTime;
    MeasurementManager::getCurrentMeasurement().syncTime = MeasurementManager::getCurrentMeasurement().nrfInterruptTime;
    MeasurementManager::getCurrentMeasurement().nrfInterruptFlag = false;

    if (MeasurementManager::getCurrentMeasurement().syncTime == 0) {
        Serial.println("NOT VALID INTERRUPT.");
        return false;
    }

    // Start listening for the response
    radio.maskIRQ(true, true, false);
    radio.startListening();
    
    waitForRadio(50000);
    roundTripTimeEnd = MeasurementManager::getCurrentMeasurement().nrfInterruptTime;
    Serial.println(roundTripTimeEnd);

    // Read the receiver's time
    if (radio.available()) {
        radio.read(&nrfData, sizeof(DataPackage));
        Serial.println(nrfData.command);
        Serial.println(nrfData.time);

        receiverTime = nrfData.time;
        MeasurementManager::getCurrentMeasurement().timeDifference = (int64_t)MeasurementManager::getCurrentMeasurement().syncTime - (int64_t)receiverTime - 167;
        
        Serial.print("Transmitter time: \t");
        Serial.print(MeasurementManager::getCurrentMeasurement().syncTime);
        Serial.print("\tReceiver time: \t");
        Serial.print(receiverTime);
        Serial.print("\tTime difference: \t");
        Serial.print(MeasurementManager::getCurrentMeasurement().timeDifference);
        Serial.print("\tRound trip time: \t");
        Serial.println(roundTripTimeEnd - roundTripTimeStart - 5000);
    } else {
        Serial.println("No data available.");
        return false;
    }
    return true;
}

bool RadioCommunication::synchronizeTimeReceiver(uint32_t lastReceivedInterruptTime) {
    MeasurementManager::getCurrentMeasurement().syncTime = 0;
    uint32_t synchro_time = lastReceivedInterruptTime;
    DataPackage nrfData;
    
    delayMicroseconds(5000);
    nrfData.command = 123456789;
    nrfData.time = synchro_time;
    radio.stopListening();
    if (!radio.write(&nrfData, sizeof(DataPackage))) {
        Serial.println("Failed to send current time.");
        MeasurementManager::getCurrentMeasurement().syncTime = 0;
        MeasurementManager::getCurrentMeasurement().syncSuccess = 0;
        MeasurementManager::getCurrentMeasurement().nrfInterruptFlag = false;
        return false;
    } else {  
        Serial.print("Time sent successfully.\n");
        MeasurementManager::getCurrentMeasurement().syncTime = lastReceivedInterruptTime;
        Serial.println(synchro_time);
        return true;
    }
}

bool RadioCommunication::sendTime(uint32_t time) {
    DataPackage nrfData;
    nrfData.command = 123456789;
    nrfData.time = time;
    delayMicroseconds(5000);
    radio.stopListening();
    if (!radio.write(&nrfData, sizeof(DataPackage))) {
        Serial.println("Failed to send current time.");
        return false;
    } else {  
        Serial.println("Time sent successfully.\n");
        Serial.println(time);
        return true;
    }
}

bool RadioCommunication::waitForRadio(unsigned long timeout) {
    unsigned long start_time = micros();
    while (!MeasurementManager::getCurrentMeasurement().nrfInterruptFlag && (micros() - start_time < timeout)) {}
    MeasurementManager::getCurrentMeasurement().nrfInterruptFlag = false;
    return true;
}

uint32_t RadioCommunication::requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration) {
    DataPackage nrfData;
    uint32_t validTime = 0;
    
    for (uint8_t attempt = 0; attempt < maxRetries; ++attempt) {
        nrfData.command = RF24_CMD_RESEND_TIME;
        nrfData.time = 0;
        
        radio.stopListening();
        if (!radio.write(&nrfData, sizeof(DataPackage))) {
            Serial.println("Failed to request time.");
        }

        // Start listening for response
        radio.startListening();
        waitForRadio(waitDuration);

        if (radio.available()) {
            radio.read(&nrfData, sizeof(DataPackage));
            if (nrfData.command == 123456789) {
                validTime = nrfData.time;
                Serial.print("Valid time received on attempt ");
                Serial.print(attempt + 1);
                Serial.print(": ");
                Serial.println(validTime);
                return validTime;
            } else {
                Serial.println("Invalid command received, retrying...");
            }
        } else {
            Serial.println("No response received, retrying...");
        }
        radio.stopListening();
    }

    Serial.println("Failed to receive valid time after maximum retries.");
    return 0;
}

bool RadioCommunication::sendStartCommand() {
    DataPackage nrfData;
    nrfData.command = RF24_CMD_START;
    nrfData.time = (uint64_t)((int64_t)MeasurementManager::getCurrentMeasurement().startInterruptTime - MeasurementManager::getCurrentMeasurement().timeDifference);
    
    radio.stopListening();
    if (!radio.write(&nrfData, sizeof(DataPackage))) {
        Serial.println("Failed to send start command.");
        radio.startListening();
        return false;
    }
    radio.startListening();
    return true;
}

bool RadioCommunication::sendFinishCommand() {
    DataPackage nrfData;
    nrfData.command = RF24_CMD_FINISH;
    nrfData.time = MeasurementManager::getCurrentMeasurement().finalTime;

    radio.stopListening();
    if (!radio.write(&nrfData, sizeof(DataPackage))) {
        Serial.println("Failed to send finish command.");
        radio.startListening();
        return false;
    }
    radio.startListening();
    return true;
}
