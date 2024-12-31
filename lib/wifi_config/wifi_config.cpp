#include <Arduino.h>
#include <WiFi.h>
#include "wifi_config.hpp"

// Konfiguracja WiFi
const char* MY_WIFI_SSID = "Orange-A713";
const char* MY_WIFI_PASSWORD = "kamperJJ2713";
const char* MY_SERVER_HOSTNAME = "192.168.8.111";
const int   MY_SERVER_PORT = 1234;

IPAddress MY_SERVER_IP; // IP serwera
WiFiClient client; // TCP client object



bool connectToWiFi() {
  WiFi.begin(MY_WIFI_SSID, MY_WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi!");
  Serial.print("Device IP Address: ");
  Serial.println(WiFi.localIP());

    return true;
}

bool reconnectWiFi() {
  WiFi.disconnect();
  connectToWiFi();
  return true;
}

bool resolveServerIP() {
    for (int i = 0; i < 5; i++) {
        if (WiFi.hostByName(MY_SERVER_HOSTNAME, MY_SERVER_IP)) {
            Serial.print("Resolved IP Address of ");
            Serial.print(MY_SERVER_HOSTNAME);
            Serial.print(": ");
            Serial.println(MY_SERVER_IP);
            return true;
        } else {
            Serial.println("Hostname resolution failed!");
            delay(500); // Stop if resolution fails
        }
    }
    Serial.println("Hostname resolution failed 5 times!");
    return false;
}


bool connectToServer() {
  Serial.println("Connecting to the server...");
  while (!client.connect(MY_SERVER_IP, MY_SERVER_PORT)) {
    delay(1000);
    Serial.println("Retrying...");
  }
  Serial.println("Connected to the server!");
  return true;
}