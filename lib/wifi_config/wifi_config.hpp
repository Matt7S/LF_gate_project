#ifndef WIFI_CONFIG_HPP
#define WIFI_CONFIG_HPP

#include <Arduino.h>
#include <WiFi.h>





void checkConnection();
bool connectToWiFi();
bool reconnectWiFi();
bool resolveServerIP();
bool connectToServer();

#endif // WIFI_CONFIG_HPP