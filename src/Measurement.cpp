/**
 * @file Measurement.cpp
 * @brief Measurement calculations and data management.
 */

#include "Measurement.hpp"

// ===== Static Instance =====
Measurement MeasurementManager::measurement;

Measurement& MeasurementManager::getCurrentMeasurement() {
    return measurement;
}

void MeasurementManager::reset() {
    measurement.playerCardCode = "";
    measurement.playerName = "";
    measurement.playerID = 0;

    measurement.robotQrCode = "";
    measurement.robotName = "";
    measurement.robotID = 0;

    measurement.judgeCardCode = "";
    measurement.judgeID = 0;

    measurement.nrfInterruptTime = 0;
    measurement.nrfInterruptFlag = false;

    measurement.syncCounter = 0;
    measurement.syncSuccess = false;
    measurement.syncTime = 0;
    measurement.timeDifference = 0;

    measurement.startInterruptTime = 0;
    measurement.finishInterruptTime = 0;
    measurement.startInterruptFlag = false;
    measurement.finishInterruptFlag = false;

    measurement.counterMinutes = 0;
    measurement.counterSeconds = 0;
    measurement.counterMilliseconds = 0;
    measurement.finalFormattedTime = "";
    measurement.finalTime = 0;

    measurement.getUserReceived = false;
    measurement.getRobotReceived = false;
    measurement.retryJudgeReceived = false;
    measurement.getJudgeReceived = false;
    measurement.retryScoreReceived = false;

    measurement.resetCommandSend = false;
    measurement.lastMessage = "";
}

// ===== Time Calculations =====

String MeasurementManager::calculateFinalTime(uint64_t startTime, uint64_t finishTime) {
    uint64_t duration = calculateDuration(startTime, finishTime);
    return formatTimeString(duration);
}

uint64_t MeasurementManager::calculateDuration(uint64_t startTime, uint64_t finishTime) {
    // Convert microseconds to milliseconds with rounding
    return (finishTime - startTime + TIME_ROUNDING_OFFSET) / MS_TO_US;
}

String MeasurementManager::formatTimeString(uint64_t milliseconds) {
    uint32_t minutes = getMinutes(milliseconds);
    uint32_t seconds = getSeconds(milliseconds);
    uint32_t ms = getMilliseconds(milliseconds);

    char buffer[12];
    sprintf(buffer, "%02d:%02d:%03d", minutes, seconds, ms);
    
    return String(buffer);
}

uint32_t MeasurementManager::getMinutes(uint64_t milliseconds) {
    return (milliseconds / MS_PER_MINUTE) % 60;
}

uint32_t MeasurementManager::getSeconds(uint64_t milliseconds) {
    return (milliseconds / MS_PER_SECOND) % 60;
}

uint32_t MeasurementManager::getMilliseconds(uint64_t milliseconds) {
    return milliseconds % MS_PER_SECOND;
}
