/**
 * @file ServerCommunication.cpp
 * @brief Implementation of server communication module.
 *
 * Implements JSON message transmission, response parsing,
 * and WiFi/server connection management.
 */

#include "ServerCommunication.hpp"

// Forward declarations for functions defined in main.cpp
extern void connectToWiFi();
extern void reconnectWiFi();
extern bool connectToServer();

void ServerCommunication::sendJsonMessage(const char* command, FirebaseJson& data) {
    if (client.connected()) {
        FirebaseJson json;
        json.set("co", command);
        json.set("da", data);

        String jsonString;
        json.toString(jsonString, true);
        jsonString += "\n";
        client.print(jsonString);

        Serial.print("Sent JSON: ");
        Serial.println(jsonString);
    } else {
        Serial.println("ERROR: Not connected to server. Cannot send JSON.");
    }
}

bool ServerCommunication::handleServerResponse() {
    String command;
    while (client.available()) {
        FirebaseJson responseJson;
        FirebaseJsonData jsonData;

        // Read response as line
        String response = client.readStringUntil('\n');
        response.trim();
        Serial.print("Received from server: ");
        Serial.println(response);

        // Check if response is valid JSON
        if (!responseJson.setJsonData(response)) {
            Serial.println("Invalid JSON format in server response.");
            return false;
        }

        // Extract command ("co" field)
        if (!responseJson.get(jsonData, "co")) {
            Serial.println("No command in response.");
            return false;
        }
        command = jsonData.stringValue;

        // Extract data ("da" field)
        FirebaseJson daJson;
        if (responseJson.get(jsonData, "da")) {
            daJson = FirebaseJson();
            daJson.setJsonData(jsonData.stringValue);
        } else {
            Serial.println("No 'da' field in response.");
            return false;
        }

        Serial.print("Command: ");
        Serial.println(command);

        // Handle different command types
        if (command == "SETTINGS") {
            newSettingsAvailable = true;
            applyGateSettings(daJson);
        } 
        else if (command == "USER") {
            currMeasurement.getUserReceived = true;
            if (daJson.get(jsonData, "player_rfid_code")) {
                currMeasurement.playerCardCode = jsonData.stringValue;
            }
            if (daJson.get(jsonData, "player_id")) {
                currMeasurement.playerID = jsonData.intValue;
            }
            if (daJson.get(jsonData, "player_name")) {
                currMeasurement.playerName = jsonData.stringValue;
            }
            
            Serial.print("GET_USER data: ");
            Serial.println(currMeasurement.playerName);
        } 
        else if (command == "ROBOT") {
            currMeasurement.getRobotReceived = true;
            if (daJson.get(jsonData, "robot_qr_code")) {
                currMeasurement.robotQrCode = jsonData.stringValue;
            }
            if (daJson.get(jsonData, "robot_id")) {
                currMeasurement.robotID = jsonData.intValue;
            }
            if (daJson.get(jsonData, "robot_name")) {
                currMeasurement.robotName = jsonData.stringValue;
            }

            Serial.print("Robot status: ");
            Serial.println(currMeasurement.robotName);
        } 
        else if (command == "RETRY_JUDGE") {
            if (daJson.get(jsonData, "message")) {
                currMeasurement.lastMessage = jsonData.stringValue;
            }
            currMeasurement.retryJudgeReceived = true;
        }
        else if (command == "JUDGE") {
            currMeasurement.getJudgeReceived = true;
            if (daJson.get(jsonData, "judge_card_code")) {
                currMeasurement.judgeCardCode = jsonData.stringValue;
            }
            if (daJson.get(jsonData, "judge_id")) {
                currMeasurement.judgeID = jsonData.intValue;
            }
            Serial.print("GET_JUDGE data: ");
            Serial.println(currMeasurement.judgeID);
        }
        else if (command == "RETRY_SCORE") {
            if (daJson.get(jsonData, "message")) {
                currMeasurement.lastMessage = jsonData.stringValue;
            }
            currMeasurement.retryScoreReceived = true;
        }
        else if (command == "RESET") {
            Serial.println("RESET command received");
            if (daJson.get(jsonData, "message")) {
                currMeasurement.lastMessage = jsonData.stringValue;
            }
            currentState = RESETING;
        } 
        else {
            Serial.println("Unknown command.");
            return false;
        }
    }
    return true;
}

void ServerCommunication::checkConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        reconnectWiFi();
    }

    if (!client.connected()) {
        digitalWrite(LED_RED, LOW);
        if (connectToServer()) {
            FirebaseJson data;
            data.set("serial_number", GATE_SERIAL_NUMBER);
            sendJsonMessage("GET_SETTINGS", data);
        }
    } else {
        digitalWrite(LED_RED, HIGH);
    }
}

void ServerCommunication::applyGateSettings(FirebaseJson& daJson) {
    FirebaseJsonData jsonData;
    String startNRFAddress, finishNRFAddress;

    if (daJson.get(jsonData, "pair_id")) {
        newGateSettings.pairID = jsonData.intValue;
    }
    if (daJson.get(jsonData, "active")) {
        newGateSettings.active = jsonData.boolValue;
    }
    if (daJson.get(jsonData, "start_gate_id")) {
        newGateSettings.startGateID = jsonData.intValue;
    }
    if (daJson.get(jsonData, "finish_gate_id")) {
        newGateSettings.finishGateID = jsonData.intValue;
    }
    if (daJson.get(jsonData, "category_id")) {
        newGateSettings.categoryID = jsonData.intValue;
    }
    if (daJson.get(jsonData, "stage_id")) {
        newGateSettings.stageID = jsonData.intValue;
    }
    if (daJson.get(jsonData, "start_nrf")) {
        startNRFAddress = jsonData.stringValue;
        strncpy((char*)newGateSettings.nrfStartAddress, startNRFAddress.c_str(), sizeof(newGateSettings.nrfStartAddress) - 1);
    }
    if (daJson.get(jsonData, "finish_nrf")) {
        finishNRFAddress = jsonData.stringValue;
        strncpy((char*)newGateSettings.nrfFinishAddress, finishNRFAddress.c_str(), sizeof(newGateSettings.nrfFinishAddress) - 1);
    }
    if (daJson.get(jsonData, "requires_user_card")) {
        newGateSettings.requiredUserCard = jsonData.boolValue;
    }
    if (daJson.get(jsonData, "requires_user_qr_code")) {
        newGateSettings.requiredUserQrCode = jsonData.boolValue;
    }
    if (daJson.get(jsonData, "requires_judge_confirmation")) {
        newGateSettings.requiredConfirmation = jsonData.boolValue;
    }
    if (daJson.get(jsonData, "category_name")) {
        newGateSettings.categoryName = jsonData.stringValue;
    }
    if (daJson.get(jsonData, "stage_name")) {
        newGateSettings.stageName = jsonData.stringValue;
    }
    if (daJson.get(jsonData, "gate_type")) {
        newGateSettings.typeName = jsonData.stringValue;
    }
}
