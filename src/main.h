#ifndef BCM_PREFERENCES
#define BCM_PREFENERENCES

#define CAN_BAUD 250000
#define ECU_TIMEOUT_MS 500

#define FAN_COOLDOWN_S 30
#define WATERPUMP_COOLDOWN_S 15

#define PWM_PERIOD_US 100

#define ECU_HEARTBEAT_ID 8 //test val

// Function Prototypes
void beepMotors();
void initGPIO();
void initCANMessages();
void checkTimers();
void coolingControl();
void upShift();
void downShift();

#endif