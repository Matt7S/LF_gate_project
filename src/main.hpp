#include <Arduino.h>

// Function prototypes
void handleInterrupt();
void handleInterruptIR();
bool sendStartCommand();
bool sendFinishCommand();

void handleInterruptYellow();
void handleInterruptBlack();
void updateTime();
void handleButtonPress(int buttonPin, const String& message, unsigned long& lastPressTime);
String readDataFromRFIDCard();

void sendJsonMessage(const char* command, FirebaseJson& data);
void handleServerResponse();

uint32_t requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration);
bool sendTime(uint32_t time);
void setRole(String gateType);
void listenForSignals();
bool synchronize_time_receiver(uint32_t last_received_interrupt_time);
bool synchronize_time_transmitter();
void handleInterrupt();
bool waitForRadio(unsigned long timeout);
