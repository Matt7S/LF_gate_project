/**
 * @file ServerCommunication.hpp
 * @brief Server communication module for WiFi-based messaging.
 *
 * Handles all server communication including:
 * - JSON message formatting and transmission
 * - Server response parsing and processing
 * - WiFi connection status monitoring
 */

#ifndef SERVER_COMMUNICATION_HPP
#define SERVER_COMMUNICATION_HPP

#include <Arduino.h>
#include <FirebaseJson.h>
#include <WiFi.h>
#include "main.hpp"
#include "constants.hpp"

// Forward declarations for extern global objects
extern WiFiClient client;
extern Gate gate;
extern Gate newGateSettings;
extern bool newSettingsAvailable;
extern Measurement currMeasurement;
extern State currentState;

/**
 * @class ServerCommunication
 * @brief Static interface for server communication via WiFi.
 *
 * Provides methods for sending JSON messages to central server,
 * parsing responses, and maintaining connection status.
 */
class ServerCommunication {
public:
    /**
     * @brief Sends JSON message to central server.
     *
     * Formats command and data into JSON structure:
     * { "co": command, "da": data }
     * Adds line separator and transmits via WiFi client.
     *
     * @param command Command identifier (e.g., "GET_SETTINGS", "GET_USER", "SEND_SCORE")
     * @param data FirebaseJson object containing command payload
     *
     * @note Only sends if client is connected. Prints error if connection lost.
     * @note Adds newline terminator to JSON message for line-based parsing.
     */
    static void sendJsonMessage(const char* command, FirebaseJson& data);

    /**
     * @brief Processes incoming messages from central server.
     *
     * Reads JSON responses from server socket line by line.
     * Parses JSON structure { "co": command, "da": data }.
     * Updates gate state and measurement data based on command type:
     * - SETTINGS: Updates gate configuration from server
     * - USER: Stores competitor information
     * - ROBOT: Stores robot data
     * - JUDGE: Stores judge information if required
     * - RETRY_JUDGE: Judge authentication retry
     * - RETRY_SCORE: Score submission retry
     * - RESET: Server initiates system reset
     *
     * @return True if response processed successfully, false on error
     *
     * @note Processes all available data in one call
     * @note Updates currMeasurement and gate state directly
     * @note Prints diagnostic information to Serial
     */
    static bool handleServerResponse();

    /**
     * @brief Checks WiFi and server connection status.
     *
     * Monitors connection state and performs automatic reconnection:
     * - If WiFi disconnected: attempts to reconnect to WiFi network
     * - If server disconnected: attempts to reconnect to server
     * - If reconnected: requests fresh gate settings from server
     *
     * Controls LED status:
     * - RED LED LOW when disconnected (attempting connection)
     * - RED LED HIGH when connected
     *
     * @note Called periodically in main loop
     * @note Reconnection is handled by other modules (connectToWiFi, connectToServer)
     */
    static void checkConnection();

private:
    // Private constructor - static methods only
    ServerCommunication() = delete;

    /**
     * @brief Applies received gate settings from server.
     * @param daJson FirebaseJson containing settings data
     */
    static void applyGateSettings(FirebaseJson& daJson);
};

#endif  // SERVER_COMMUNICATION_HPP
