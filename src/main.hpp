#include <Arduino.h>

#define GATE_SERIAL_NUMBER 2

struct Data_Package {
    uint32_t command;
    uint64_t time;
};

struct Gate {
    uint8_t ID = GATE_SERIAL_NUMBER;
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
    USER_AUTHENTICATION,
    TIME_SYNCHRONIZATION,
    ROBOT_AUTHENTICATION,
    START_WAITING,
    STARTED,
    FINISH_WAITING,
    FINISHED,
    COUNT_RESULT,
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

  String judgeCardCode = "";
  uint8_t judgeID = 0;

  uint64_t NRFInterruptTime = 0;
  bool NRFInterruptFlag = false;

  uint8_t synchroCounter = 0;
  bool synchroSuccess = 0;
  uint64_t synchroTime = 0;
  int64_t timeDiffrence = 0;

  uint64_t startInterruptTime = 0;
  uint64_t finishInterruptTime = 0;
  bool startInterruptFlag = false;
  bool finishInterruptFlag = false;

  uint32_t counter_m = 0;
  uint32_t counter_s = 0;
  uint32_t counter_ms = 0;
  String finalFormatedTime = "";
  uint64_t finalTime = 0;

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

bool handleServerResponse();

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
void applyNewGateSettings();