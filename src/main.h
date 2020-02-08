#ifndef BCM_PREFERENCES
#define BCM_PREFERENCES

#define PRINT_STATUS // uncomment - print status
// #define PRINT_CAN    // uncomment - print CAN

#define CAN_BAUD 250000
#define SERIAL_BAUD 921600
#define ECU_TIMEOUT_MS 250
#define CAN_TIMEOUT_MS 250
#define COOLING_KILL_MS 10000

// Cooldown times in milliseconds
#define FAN_COOLDOWN_MS 10000
#define WATERPUMP_COOLDOWN_MS 20000

// Active duty cycles
#define WATERPUMP_ACTIVE_DC 0.9
#define FAN_ACTIVE_DC 0.7

// Cooldown duty cycles
#define WATERPUMP_COOLDOWN_DC .5
#define FAN_COOLDOWN_DC .4

#define PWM_PERIOD_US 100

#define ECU_HEARTBEAT_ID 0x0CFFF548

// Paramaters
#define ENGINE_WARM_F 90 // Fahrenheit

// State Machine states
#define safetyState 0
#define engineOffState 1
#define cooldownState 2
#define engineCrankState 3
#define coldRunningState 4
#define hotRunningState 5
#define coolingKillState 6

// Function Prototypes
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