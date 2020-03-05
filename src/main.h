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
#define WATERPUMP_ACTIVE_DC 0.9
#define FAN_ACTIVE_DC 0.7

// Cooldown duty cycles
#define WATERPUMP_COOLDOWN_DC .5
#define FAN_COOLDOWN_DC .7

// For cooling fan and pump
#define PWM_PERIOD_US 100

// Used for to reset ECU online timer
#define ECU_HEARTBEAT_ID 0x0CFFF048

// Paramaters
#define ENGINE_WARM_F 195       // Fahrenheit
#define ENGINE_TEMP_DEADBAND 20 // Fahreneheit

// State Machine states
#define safetyState 0
#define engineOffState 1
#define cooldownState 2
#define coldRunningState 3
#define hotRunningState 4
#define coolingKillState 5

// Function Prototypes
void CANCallback(); // Used for CAN frame interrupt
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

#endif