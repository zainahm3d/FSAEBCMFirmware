#include "main.h"
#include <mbed.h>
#include "rtos.h"

DigitalOut led(LED1);

PwmOut fan(PA_10);
PwmOut waterPump(PA_8);
PwmOut starter(PA_1);

DigitalOut sparkCut(PA_0);
DigitalOut upShiftPin(PB_1);
DigitalOut downShiftPin(PB_0);
DigitalOut ECUPower(PB_4);
DigitalOut ETCEnable(PA_5);

DigitalIn neutral(PA_7);
AnalogIn anIn(PA_3);

CAN can0(PA_11, PA_12, CAN_BAUD);
CANMessage inMsg;
CANMessage statusMsg;

Serial ser(USBTX, USBRX, SERIAL_BAUD);

Thread checkTimerThread;
Thread coolingControlThread;
Thread sendStatusThread;
Thread updateStateThread;

Timer ECUTimer;
Timer CANTimer;
Timer starterTimer;
Timer CANResetTimer;
Timer coolingKillTimer;

volatile bool CANConnected = false;
volatile bool engineRunning = false;
volatile bool ECUConnected = false;
volatile bool coolDownFlag = false;
volatile bool coolingKillFlag = false;
volatile bool haltStateMachine = false;
volatile bool coolingDone = false;
volatile bool engineWasRunning = false;

volatile float waterTemp = 0.0;
volatile float batteryVoltage = 0.0;
volatile int rpm = 0;
volatile int state = 0;

// Used only for printing purposes
#ifdef PRINT_STATUS
char stateNames[7][20] = {"safetyState", "engineOffState",
                          "cooldownState", "engineCrankState",
                          "coldRunningState", "hotRunningState"};
#endif

int main()
{
  ECUTimer.reset();
  ECUTimer.start();

  CANTimer.reset();
  CANTimer.start();

  starterTimer.reset();
  starterTimer.start();

  CANResetTimer.reset();
  CANResetTimer.start();

  initCANMessages();
  initGPIO();
  beepMotors();

  coolingControlThread.start(coolingControl);
  checkTimerThread.start(checkTimers);
  sendStatusThread.start(sendStatusMsg);
  updateStateThread.start(updateState);

  while (1)
  {

    // recover CAN bus after significant amount of errors,
    // up to once per second
    if ((can0.tderror() > 5 || can0.rderror() > 5) &&
        CANResetTimer.read() > 1)
    {
      can0.reset();
      CANResetTimer.reset();
    }

    if (can0.read(inMsg))
    {

#ifdef PRINT_CAN
      ser.printf("ID: %d", inMsg.id);
      ser.printf(" Data: ");
      for (int i = 0; i < inMsg.len; i++)
      {
        ser.printf("%d ", inMsg.data[i]);
      }
      ser.printf("\n");
#endif

      // Reset watchdog timers
      CANTimer.reset();
      if (inMsg.id == ECU_HEARTBEAT_ID)
      {
        ECUTimer.reset();
      }

      if (inMsg.id == 98)
      {
        if (inMsg.data[0] == 91)
        {
          starter.write(1);
          starterTimer.reset();
        }
      }

      // Steering wheel ID is 0
      if (inMsg.id == 0)
      {
        if (inMsg.data[0] == 10)
        {
          upShift();
        }
        else if (inMsg.data[0] == 11)
        {
          downShift();
        }
        else if (inMsg.data[0] == 14)
        {
          halfShift();
        }
      }

      // Read in ECU frames
      // PE1
      if (inMsg.id == 0x0CFFF048)
      {
        rpm = ((inMsg.data[1] << 8) + inMsg.data[0]);
        if (rpm > 0)
        {
          coolingDone = false;
        }
      }
      // PE6
      else if (inMsg.id == 0x0CFFF548)
      {
        uint16_t newTemp = ((inMsg.data[5] << 8) + inMsg.data[4]);
        if (newTemp > 32767)
        {
          newTemp = newTemp - 65536;
        }
        waterTemp = ((newTemp / 10.0) * 1.8) + 32;
      }
    }
  }
}

void beepMotors()
{
  // Beep water Pump

  waterPump.period_us(200);
  waterPump.write(0);
  ThisThread::sleep_for(100);

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 100; j++)
    {
      waterPump.write(.2);
      wait_us(800);
      waterPump.write(0);
      wait_us(400);
    }
    ThisThread::sleep_for(20);
  }

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 400; j++)
    {
      waterPump.write(.2);
      wait_us(200);
      waterPump.write(0);
      wait_us(150);
    }
    ThisThread::sleep_for(20);
  }

  waterPump.period_us(PWM_PERIOD_US);
  waterPump.write(0);

  // Beep cooling fan

  fan.period_us(200);
  fan.write(0);
  ThisThread::sleep_for(100);

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 400; j++)
    {
      fan.write(.2);
      wait_us(200);
      fan.write(0);
      wait_us(150);
    }
    ThisThread::sleep_for(20);
  }

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 100; j++)
    {
      fan.write(.2);
      wait_us(800);
      fan.write(0);
      wait_us(400);
    }
    ThisThread::sleep_for(20);
  }

  fan.period_us(PWM_PERIOD_US);
  fan.write(0);
}

void initGPIO()
{
  // PWMs
  waterPump.period_us(PWM_PERIOD_US);
  fan.period_us(PWM_PERIOD_US);
  starter.period_us(PWM_PERIOD_US);

  waterPump.write(0);
  fan.write(0);
  starter.write(0);

  // DigitalOuts
  sparkCut.write(1); // Active LOW
  upShiftPin.write(0);
  downShiftPin.write(0);
  ECUPower.write(1);
  ETCEnable.write(1); // TESTING ONLY

  // DigitalIns
  neutral.mode(PullUp);
}

void checkTimers()
{
  while (1)
  {
    if (ECUTimer.read_ms() > ECU_TIMEOUT_MS)
    {
      ECUConnected = false;
      engineRunning = false;

      // if ECU shuts down, this data must be reset
      rpm = 0;
      waterTemp = 0;
    }
    else
    {
      ECUConnected = true;
      engineRunning = true;
    }

    if (CANTimer.read_ms() > CAN_TIMEOUT_MS)
    {
      led.write(0);
      CANConnected = false;
    }
    else
    {
      led.write(1);
      CANConnected = true;
    }

    if (starterTimer.read_ms() > 100)
    { // timeout starter motor after 100ms
      starter.write(0);
    }

    if (coolingKillTimer.read_ms() > COOLING_KILL_MS)
    {
      haltStateMachine = false;
    }
  }
}

//states for fan and water pump
void coolingControl()
{
  while (1)
  {

    switch (state)
    {

    case safetyState:
    {
      engineWasRunning = false;
      fan.write(FAN_ACTIVE_DC);
      waterPump.write(WATERPUMP_ACTIVE_DC);
      coolingDone = false;
      break;
    }

    case engineOffState:
    {
      fan.write(0);
      waterPump.write(0);

      break;
    }

    case cooldownState:
    {

      fan.write(FAN_COOLDOWN_DC);
      waterPump.write(WATERPUMP_COOLDOWN_DC);

      for (int i = 0; i < (FAN_COOLDOWN_MS / 1000); i++)
      {

        if (rpm == 0)
        {
          ThisThread::sleep_for(1000);
        }
        else
        {
          break;
        }
      }

      fan.write(0);

      for (int i = 0; i < ((WATERPUMP_COOLDOWN_MS - FAN_COOLDOWN_MS) / 1000); i++)
      {

        if (rpm == 0)
        {
          ThisThread::sleep_for(1000);
        }
        else
        {
          break;
        }
      }

      waterPump.write(0);
      state = engineOffState;
      coolingDone = true;

      break;
    }

    case engineCrankState:
    {
      fan.write(0);
      waterPump.write(0);
      coolingDone = false;
      engineWasRunning = true;
      break;
    }

    case coldRunningState:
    {
      waterPump.write(WATERPUMP_ACTIVE_DC);
      fan.write(0);
      coolingDone = false;
      engineWasRunning = true;
      break;
    }

    case hotRunningState:
    {
      waterPump.write(WATERPUMP_ACTIVE_DC);
      fan.write(FAN_ACTIVE_DC);
      coolingDone = false;
      engineWasRunning = true;
      break;
    }

    case coolingKillState:
    {
      coolingKillTimer.start();
      fan.write(0);
      waterPump.write(0);
      coolingDone = false;
      break;
    }

    default:
    {
      fan.write(FAN_ACTIVE_DC);
      waterPump.write(WATERPUMP_ACTIVE_DC);
      break;
    }
    }
  }
}

//updates states based on rpm, waterTemp, if engine is on
void updateState()
{

  while (1)
  {
    if (rpm == 0)
    {
      engineWasRunning = false;
    }

    if (coolingKillFlag)
    {
      state = coolingKillState;
      haltStateMachine = true;
    }

    while (!haltStateMachine)
    {
      if (!CANConnected)
      {
        state = safetyState;
      }
      else if ((waterTemp > ENGINE_WARM_F && rpm == 0) ||
               (waterTemp > ENGINE_WARM_F && !ECUConnected && CANConnected))

      {
        state = cooldownState;
      }

      else if ((!engineWasRunning && rpm == 0) ||
               (waterTemp < ENGINE_WARM_F && !ECUConnected && CANConnected) ||
               coolingDone || (state == engineCrankState && rpm == 0))
      {
        state = engineOffState;
      }

      else if (rpm > 10 && rpm <= 1000 && waterTemp < ENGINE_WARM_F)
      {
        state = engineCrankState;
      }
      else if (rpm > 1000 && waterTemp < ENGINE_WARM_F)
      {
        state = coldRunningState;
      }
      else if (rpm > 1000 && waterTemp > ENGINE_WARM_F + 1)
      {
        state = hotRunningState;
      }
      else
      {
        state = safetyState;
      }
    }
  }
}

void sendStatusMsg()
{
  while (1)
  {
    statusMsg.data[0] = neutral.read();
    statusMsg.data[1] = ETCEnable.read();
    can0.write(statusMsg);
#ifdef PRINT_STATUS
    ser.printf("\n");
    ser.printf("Water Temp:\t %f\n", waterTemp);
    ser.printf("RPM:\t\t %d\n", rpm);
    ser.printf("Water Pump DC:\t %f\n", waterPump.read());
    ser.printf("Fan DC:\t\t %f\n", fan.read());
    ser.printf("Starter DC:\t %f\n", starter.read());
    ser.printf("CAN Status:\t %d\n", CANConnected);
    ser.printf("ECU Status:\t %d\n", ECUConnected);
    ser.printf("State:\t\t %d, %s\n", state, stateNames[state]);
    ser.printf("SMHalted:\t %d\n", haltStateMachine);
    ser.printf("killCooling:\t %d\n", coolingKillFlag);
    ser.printf("ETCEnabled:\t %d\n", ETCEnable.read());
#endif
    ThisThread::sleep_for(100);
  }
}

void initCANMessages()
{
  statusMsg.id = 20;
  statusMsg.len = 8;
}

void upShift()
{

#ifdef PRINT_STATUS
  ser.printf("\n---------- UPSHIFT ----------\n");
#endif

  sparkCut.write(0);
  ThisThread::sleep_for(10);
  upShiftPin.write(1);
  ThisThread::sleep_for(70);
  upShiftPin.write(0);
  sparkCut.write(1);
}

void downShift()
{

#ifdef PRINT_STATUS
  ser.printf("\n---------- DOWNSHIFT ----------\n");
#endif

  sparkCut.write(0);
  ThisThread::sleep_for(10);
  downShiftPin.write(1);
  ThisThread::sleep_for(150);
  downShiftPin.write(0);
  sparkCut.write(1);
}

void halfShift()
{

#ifdef PRINT_STATUS
  ser.printf("\n---------- HALFSHIFT ----------\n");
#endif

  upShiftPin.write(1);
  ThisThread::sleep_for(7);
  downShiftPin.write(1);
  ThisThread::sleep_for(150);
  upShiftPin.write(0);
  downShiftPin.write(0);
}