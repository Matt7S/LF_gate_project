#include <Arduino.h>

// Function prototypes
void handleInterrupt();
void handleInterruptYellow();
void handleInterruptBlack();
void updateTime();
void handleButtonPress(int buttonPin, const String& message, unsigned long& lastPressTime);
void readDataFromRFIDCard();

void sendJsonMessage(const char* command, FirebaseJson& data);
void handleServerResponse();

bool synchronize_time_receiver(uint8_t *transmiter_address, uint8_t *reciever_address, uint32_t last_received_interrupt_time);
bool synchronize_time_transmitter(uint8_t *transmitter_address, uint8_t *receiver_address);
void handleInterrupt();
bool waitForRadio(unsigned long timeout);