#ifndef BCM_PREFERENCES
#define BCM_PREFERENCES

#define PRINT_STATUS // uncomment - print status
// #define PRINT_CAN    // uncomment - print CAN

#define FURY // uncomment for Link Fury, comment for PE3

#define CAN_BAUD 250000
#define SERIAL_BAUD 921600
#define ECU_TIMEOUT_MS 2000
#define CAN_TIMEOUT_MS 2000
#define COOLING_KILL_MS 10000

// Cooldown times in milliseconds
#define FAN_COOLDOWN_MS 30000
#define WATERPUMP_COOLDOWN_MS 60000

// Active duty cycles
#define WATERPUMP_ACTIVE_DC 1.0f
#define FAN_ACTIVE_DC 0.7f

// Cooldown duty cycles
#define WATERPUMP_COOLDOWN_DC .7f
#define FAN_COOLDOWN_DC .5f

// For cooling fan and pump
#define PWM_PERIOD_US 100

// Paramaters
#define ENGINE_WARM_F 175       // Fahrenheit
#define ENGINE_TEMP_DEADBAND 20 // Fahreneheit

// State Machine states
#define safetyState 0
#define engineOffState 1
#define cooldownState 2
#define coldRunningState 3
#define hotRunningState 4

// ethrottle safety
#define ETHROTTLE_MAX_ERROR_COUNT 10
#define APPS_VS_APPS_MAX_ERROR 10
#define TPS_VS_TPS_MAX_ERROR 10

#define APPS_VS_TPS_MAX_ERROR_COUNT 50
#define APPS_VS_TPS_MAX_ERROR 20 // include idle offset

// Used only for printing purposes
#ifdef PRINT_STATUS
char stateNames[5][20] = {"safetyState", "engineOffState",
                          "cooldownState", "coldRunningState", "hotRunningState"};
#endif

// Globals
volatile bool CANConnected = false;
volatile bool engineRunning = false;
volatile bool ECUConnected = false;

volatile float waterTemp = 0.0;
volatile float batteryVoltage = 0.0;
volatile int rpm = 0;
volatile int state = 0;

// For EThrotttle error checks
volatile int APPS1 = 0;
volatile int APPS2 = 0;
volatile int TPS1 = 0;
volatile int TPS2 = 0;
volatile int APPSerrorCount = 0;
volatile int TPSerrorCount = 0;
volatile int APPSvsTPSerrorCount = 0;
volatile bool eThrottleErrorOccurred = false;

// Function Prototypes
void initBCM();
void initTimers();
void parseCANmessage();
void beepMotors();
void initGPIO();
void initCANMessages();
void checkTimers();
void coolingControl();
void upShift();
void downShift();
void halfShift();
void sendStatusMsg();
void updateState();
void eThrottleSafety();

#endif