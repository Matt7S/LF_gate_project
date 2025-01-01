#include <Arduino.h>


struct Data_Package {
    uint32_t command;
    uint64_t time;
};

struct Gate {
    uint8_t ID;
    uint8_t pairID;
    bool active;
    uint8_t startGateID;
    uint8_t finishGateID;
    uint8_t categoryID;
    uint8_t stageID;
    uint8_t nrfStartAddress[6];
    uint8_t nrfFinishAddress[6];
    bool requiredUserCard;
    bool requiredUserQrCode;
    bool requiredConfirmation;
    String categoryName;
    String stageName;
    String typeName;
    bool start = false;
    bool finish = false;
};

enum State {
    IDLE,
    RFID_WAITING,
    RFID_SCANNED,
    TIME_SYNCHRONIZATION,
    QR_WAITING,
    QR_SCANNED,
    START_WAITING,
    STARTED,
    FINISH_WAITING,
    FINISHED,
    CONFIRMATION_WAITING,
    CONFIRMATION,
    RESET_WAITING,
    RESETING
};



struct Measurement {
  String playerCardCode = "";
  String playerName = "";
  uint8_t playerID = 0;

  String robotQrCode = "";
  String robotName = "";
  uint8_t robotID = 0;

  String judgeCardNumber = "";
  uint8_t judgeID = 0;

  bool rfidScanned = false;

  uint8_t synchroCounter = 0;
  bool synchroSuccess = 0;
  uint32_t timeZero = 0;
  int32_t timeDiffrence = 0;

  uint32_t start_time_interrupt = 0;
  uint32_t finish_time_interrupt = 0;
  uint32_t final_time = 0;

  bool start_time_status = false;
  bool finish_time_status = false;

  bool getUserRecieved =  false;
  bool getRobotRecieved = false;
  bool retryJudgeRecieved = false;
  bool getJudgeRecieved = false;
  bool retryScoreRecieved = false;

  bool resetCommandSend = false;
  
  String lastMessage = "";
};


// Function prototypes
void handleInterrupt();
void handleInterruptIR();
bool sendStartCommand();
bool sendFinishCommand();


void updateTime();
String readDataFromRFIDCard();
bool processRFIDCard(uint32_t scanEveryMs, String &cardNumber);
bool processQRCode(uint32_t scanEveryMs, String &qrCode);

bool waitForNextState(unsigned long interval);
void sendJsonMessage(const char* command, FirebaseJson& data);

bool handleServerResponse(String command);

uint32_t requestTimeWithRetries(uint8_t maxRetries, unsigned long waitDuration);
bool sendTime(uint32_t time);
void listenForSignals();
bool synchronize_time_receiver(uint32_t last_received_interrupt_time);
bool synchronize_time_transmitter();
bool waitForRadio(unsigned long timeout);

bool checkIR(uint32_t timeout);
void resetMeasurement(Measurement &measurement);
void checkConnection();
void handleUserReset();