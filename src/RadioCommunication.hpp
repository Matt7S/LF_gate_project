/**
 * @file RadioCommunication.hpp
 * @brief NRF24L01 radio communication module for wireless timing synchronization.
 *
 * Handles all RF24 radio communication including:
 * - Time synchronization between start and finish gates
 * - Start and finish command transmission
 * - Signal listening and data reception
 */

#ifndef RADIO_COMMUNICATION_HPP
#define RADIO_COMMUNICATION_HPP

#include <Arduino.h>
#include <RF24.h>
#include "main.hpp"
#include "Measurement.hpp"
#include "constants.hpp"

// Forward declarations for extern global objects
extern RF24 radio;

/**
 * @class RadioCommunication
 * @brief Static interface for NRF24L01 wireless communication.
 *
 * Provides methods for time synchronization, command transmission,
 * and signal listening between paired start and finish gates.
 */
class RadioCommunication {
public:
    /**
     * @brief Listens for incoming signals from paired gate.
     * 
     * Processes radio commands:
     * - RF24_CMD_TIME_SYNC: Time synchronization request from transmitter
     * - RF24_CMD_RESEND_TIME: Request to resend synchronization time
     * - RF24_CMD_START: Start gate signal (finish gate receives)
     * - RF24_CMD_FINISH: Finish gate signal (start gate receives)
     * 
     * Called periodically when gate is not transmitting.
     */
    static void listenForSignals();

    /**
     * @brief Synchronizes time as transmitter (start gate).
     * 
     * Performs 2-way handshake with receiver to measure clock offset.
     * Sends current timestamp to receiver and calculates time difference
     * based on received response time.
     * 
     * @return True if synchronization successful, false on timeout/error
     */
    static bool synchronizeTimeTransmitter();

    /**
     * @brief Synchronizes time as receiver (finish gate).
     * 
     * Responds to synchronization request from start gate.
     * Sends back the interrupt timestamp received at RF24_CMD_TIME_SYNC.
     * 
     * @param lastReceivedInterruptTime Timestamp of received synchronization signal
     * @return True if synchronization successful, false on transmission error
     */
    static bool synchronizeTimeReceiver(uint32_t lastReceivedInterruptTime);

    /**
     * @brief Sends start command to finish gate.
     * 
     * Transmits synchronized start timestamp with time difference compensation.
     * Used to notify finish gate that robot has crossed the start line.
     * 
     * @return True if transmission successful, false on error
     */
    static bool sendStartCommand();

    /**
     * @brief Sends finish command to start gate.
     * 
     * Transmits final measurement time when robot crosses finish line.
     * Used to provide finish time to start gate for display/processing.
     * 
     * @return True if transmission successful, false on error
     */
    static bool sendFinishCommand();

    /**
     * @brief Sends timestamp to paired gate.
     * 
     * Transmits a time value via RF24 with command identifier 123456789.
     * Used during time synchronization to send receiver's timestamp back.
     * 
     * @param time Time value to transmit (usually in microseconds)
     * @return True if transmission successful, false on error
     */
    static bool sendTime(uint32_t time);

    /**
     * @brief Waits for NRF24L01 radio interrupt within timeout period.
     * 
     * Polls nrfInterruptFlag until it's set or timeout expires.
     * Clears the interrupt flag after waiting.
     * 
     * @param timeout Maximum wait time in microseconds
     * @return True after timeout period expires
     */
    static bool waitForRadio(unsigned long timeout);

    /**
     * @brief Requests time from receiver with retry mechanism.
     * 
     * Sends RF24_CMD_RESEND_TIME command and waits for response.
     * Retries up to maxRetries times if no valid response received.
     * Valid response has command identifier 123456789.
     * 
     * @param maxRetries Maximum number of retry attempts
     * @param waitDuration Time to wait for response between retries in microseconds
     * @return Received time value, or 0 if all attempts failed
     */
    static uint32_t requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration);

private:
    // Private constructor - static methods only
    RadioCommunication() = delete;
};

#endif  // RADIO_COMMUNICATION_HPP
