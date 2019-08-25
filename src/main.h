#ifndef BCM_PREFERENCES
#define BCM_PREFENERENCES

#define PRINT_STATUS // uncomment - print status
// #define PRINT_CAN    // uncomment - print CAN

#define CAN_BAUD 250000
#define SERIAL_BAUD 115200
#define ECU_TIMEOUT_MS 250
#define CAN_TIMEOUT_MS 250

// Cooldown times in seconds
#define FAN_COOLDOWN_S 2
#define WATERPUMP_COOLDOWN_S 5

// Active duty cycles
#define WATERPUMP_ACTIVE_DC 0.9
#define FAN_ACTIVE_DC 0.7

// Cooldown duty cycles
#define WATERPUMP_COOLDOWN_DC .5
#define FAN_COOLDOWN_DC .4

#define PWM_PERIOD_US 100

#define ECU_HEARTBEAT_ID 8 // test val

// Function Prototypes
void beepMotors();
void initGPIO();
void initCANMessages();
void checkTimers();
void coolingControl();
void upShift();
void downShift();
void sendStatusMsg();

#endif