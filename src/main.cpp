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

bool ECUConnected = false;
bool CANConnected = false;
bool engineRunning = false;
bool coolDownFlag = false;
bool coolingKillFlag = false;
bool haltStateMachine = false;

double waterTemp = 0;
double batteryVoltage = 0;
int rpm = 0;
int state = 0;

// Used only for printing purposes
char stateNames[7][20] = {"safetyState", "engineOffState",
                          "cooldownState", "engineCrankState",
                          "coldRunningState", "hotRunningState"};

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

      if (inMsg.id == 9)
      {
        rpm = inMsg.data[0] * 100; // testing only
        waterTemp = inMsg.data[1]; // testing only
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
          // half shift
        }
      }

      // Read in ECU frames
      // PE1
      if (inMsg.id == 0x0CFFF048)
      {
        rpm = ((inMsg.data[1] << 8) + inMsg.data[0]);
      }
      // PE6
      else if (inMsg.id == 0x0CFFF548)
      {
        double newTemp = ((inMsg.data[5] << 8) + inMsg.data[4]);
        if (newTemp > 32767)
        {
          newTemp = newTemp - 65536;
        }
        waterTemp = newTemp / 10;
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
      fan.write(FAN_ACTIVE_DC);
      waterPump.write(WATERPUMP_ACTIVE_DC);
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
      ThisThread::sleep_for(FAN_COOLDOWN_MS);
      fan.write(0);
      ThisThread::sleep_for(WATERPUMP_COOLDOWN_MS - FAN_COOLDOWN_MS);
      waterPump.write(0);
      state = postCooldownState;
      break;
    }

    case engineCrankState:
    {
      fan.write(0);
      waterPump.write(0);
      break;
    }

    case coldRunningState:
    {
      waterPump.write(WATERPUMP_ACTIVE_DC);
      fan.write(0);
      break;
    }

    case hotRunningState:
    {
      waterPump.write(WATERPUMP_ACTIVE_DC);
      fan.write(FAN_ACTIVE_DC);
      break;
    }

    case coolingKillState:
    {
      coolingKillTimer.start();
      fan.write(0);
      waterPump.write(0);
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
      else if ((waterTemp < 150 && rpm == 0) ||
               (waterTemp < 150 && !ECUConnected && CANConnected))
      {
        state = engineOffState;
      }
      else if ((waterTemp > 150 && rpm == 0) ||
               (waterTemp > 150 && !ECUConnected && CANConnected))
      {
        state = cooldownState;
      }
      else if (rpm > 10 && rpm < 1000)
      {
        state = engineCrankState;
      }
      else if (rpm > 1000 && waterTemp < 150)
      {
        state = coldRunningState;
      }
      else if (rpm > 1000 && waterTemp > 155)
      {
        state = hotRunningState;
      }
    }
  }
}

void sendStatusMsg()
{
  while (1)
  {
    statusMsg.data[0] = neutral.read();
    can0.write(statusMsg);
#ifdef PRINT_STATUS
    ser.printf("\n");
    ser.printf("Water Temp: %f\n", waterTemp);
    ser.printf("RPM: %d\n", rpm);
    ser.printf("Water Pump DC: %f\n", waterPump.read());
    ser.printf("Fan DC: %f\n", fan.read());
    ser.printf("Starter DC: %f\n", starter.read());
    ser.printf("CAN Status: %d\n", CANConnected);
    ser.printf("ECU Status: %d\n", ECUConnected);
    ser.printf("State: %d, %s\n", state, stateNames[state]);
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
  sparkCut.write(0);
  ThisThread::sleep_for(10);
  upShiftPin.write(1);
  ThisThread::sleep_for(70);
  upShiftPin.write(0);
  sparkCut.write(1);
}

void downShift()
{
  sparkCut.write(0);
  ThisThread::sleep_for(10);
  downShiftPin.write(1);
  ThisThread::sleep_for(150);
  downShiftPin.write(0);
  sparkCut.write(1);
}

void halfShift()
{
  upShiftPin.write(1);
  ThisThread::sleep_for(7);
  downShiftPin.write(1);
  ThisThread::sleep_for(150);
  upShiftPin.write(0);
  downShiftPin.write(0);
}