#include "main.h"
#include <mbed.h>
#include "rtos.h"
#include "../BR-CAN-IDs/BR_CAN_IDs.h"

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

Thread sendStatusThread;
Thread updateStateThread;

Timer ECUTimer;
Timer CANTimer;
Timer starterTimer;
Timer CANResetTimer;
Timer eThrottleSafetyTimer;

int main()
{

  initBCM();

  while (1)
  {
    checkTimers();

    /* Recover CAN bus after significant amount of tx/rx errors,
     up to once per second */
    if ((can0.tderror() > 5 || can0.rderror() > 5) && CANResetTimer.read() > 1)
    {
      can0.reset();
      CANResetTimer.reset();
    }

    if (can0.read(inMsg))
    {
      Watchdog::get_instance().kick();
      parseCANmessage();
    }
  }
}

/// @brief Setup GPIOs, timers, CAN messages, and threads.
void initBCM()
{
  can0.frequency(CAN_BAUD);
  initGPIO();
  initTimers();
  initCANMessages();
  beepMotors();

  sendStatusThread.start(sendStatusMsg);
  updateStateThread.start(updateState);

  // osThreadSetPriority(osThreadGetId(), osPriorityNormal1);

  // when watchdog timer elapses without a kick, mcu will soft reset.
  // the watchdog is kicked during the most critical code (ETC safety)
  Watchdog::get_instance().start(5000);
  Watchdog::get_instance().kick();
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

  wait_us(250 * 1000);
  ETCEnable.write(1);
  wait_us(100 * 1000);

  // DigitalIns
  neutral.mode(PullUp);
}

void initTimers()
{
  ECUTimer.reset();
  ECUTimer.start();

  CANTimer.reset();
  CANTimer.start();

  starterTimer.reset();
  starterTimer.start();

  CANResetTimer.reset();
  CANResetTimer.start();

  eThrottleSafetyTimer.reset();
  eThrottleSafetyTimer.start();
}

void initCANMessages()
{
  statusMsg.id = 20;
  statusMsg.len = 8;
}

void checkTimers()
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
  {
    // timeout starter motor after 100ms
    starter.write(0);
  }

  if (eThrottleSafetyTimer.read_ms() >= 10)
  {
    eThrottleSafety();

    eThrottleSafetyTimer.reset();
    eThrottleSafetyTimer.start();
  }
}

/**
 *  @brief Updates target state and outputs based on CAN data.
 *  @note THREAD
*/
void updateState()
{

  while (1)
  {

    ThisThread::sleep_for(1000); // Prevent rapid state changes

    if (!CANConnected)
    {
      state = safetyState;
    }

    if (CANConnected && ECUConnected && rpm == 0)
    {
      state = engineOffState;
    }

    // Cooling system states (only with engine running)
    if (rpm > 1000)
    {
      if (waterTemp >= ENGINE_WARM_F + ENGINE_TEMP_DEADBAND)
      {
        state = hotRunningState;
      }

      if (waterTemp <= ENGINE_WARM_F)
      {
        state = coldRunningState;
      }
    }

    // --------- SET OUTPUTS ----------
    switch (state)
    {
      case safetyState:                           // when we lose CAN bus connection
        fan.write(FAN_ACTIVE_DC);
        waterPump.write(WATERPUMP_ACTIVE_DC);
        ETCEnable.write(0);
        break;

      case engineOffState:
        if (waterTemp >= 190)
        {
          fan.write(FAN_COOLDOWN_DC);
          waterPump.write(WATERPUMP_COOLDOWN_DC);
        }
        else
        {
          fan.write(0);
          waterPump.write(0);
        }
        break;

      case coldRunningState:
        waterPump.write(WATERPUMP_ACTIVE_DC);
        fan.write(0);
        break;

      case hotRunningState:
        waterPump.write(WATERPUMP_ACTIVE_DC);
        fan.write(FAN_ACTIVE_DC);
        break;

      default:
        fan.write(FAN_ACTIVE_DC);
        waterPump.write(WATERPUMP_ACTIVE_DC);
        break;
    }
  }
}

/**
 * @brief Check for a fault lasting 100ms by doing 10 10ms checks.
 * Check APPS1 vs APPS2
 * Check TPS1 vs TPS2
 * Check APPS vs TPS
 * Any fault lasting x_MAX_ERROR consecutive function calls will cause a latching disable of EThrottle safety output.
 *
 * @note ECU must be configured to send out all APPS/TPS values as 0 to 100 (100 being WOT)
*/
void eThrottleSafety()
{
  // Check APPS1 vs APPS2
  if (abs(APPS1 - APPS2) >= APPS_VS_APPS_MAX_ERROR)
  {
    APPSerrorCount++;
  }
  else
  {
    APPSerrorCount = 0;
  }

  // Check TPS1 vs TPS2
  if (abs(TPS1 - TPS2) >= TPS_VS_TPS_MAX_ERROR)
  {
    TPSerrorCount++;
  }
  else
  {
    TPSerrorCount = 0;
  }

  // Check APPS vs TPS
  // Only do this check if either TPS is above idle % so that check is not performed during idle
  if ((abs(APPS1 - TPS1) >= APPS_VS_TPS_MAX_ERROR) &&
      ((TPS1 > APPS_VS_TPS_ENABLE_THRESHOLD) || (TPS2 > APPS_VS_TPS_ENABLE_THRESHOLD)) )
  {
    APPSvsTPSerrorCount++;
  }
  else
  {
    APPSvsTPSerrorCount = 0;
  }

  // If significant errors have ocurred, shut off ETC SAFE power rail
  if ((APPSerrorCount >= ETHROTTLE_MAX_ERROR_COUNT) || (TPSerrorCount >= ETHROTTLE_MAX_ERROR_COUNT) || (APPSvsTPSerrorCount >= APPS_VS_TPS_MAX_ERROR_COUNT))
  {
    ETCEnable.write(0);
    eThrottleErrorOccurred = true;

    // if (APPSerrorCount >= ETHROTTLE_MAX_ERROR_COUNT)
    // {
    //   ser.printf("********** ETC KILLED - REASON: APPS1 vs APPS2 **********\n");
    // }
    // else if (TPSerrorCount >= ETHROTTLE_MAX_ERROR_COUNT)
    // {
    //   ser.printf("********** ETC KILLED - REASON: TPS1 vs TPS2 **********\n");
    // }
    // else if (APPSvsTPSerrorCount >= APPS_VS_TPS_MAX_ERROR_COUNT)
    // {
    //   ser.printf("********** ETC KILLED - REASON: APPS vs TPS **********\n");
    // }

  }
}

/// @brief Read in CAN message and set variable with data. Reset timeouts if necessary.
void parseCANmessage()
{

  CANTimer.reset();

#ifdef PRINT_CAN
  ser.printf("ID: %d", inMsg.id);
  ser.printf(" Data: ");
  for (int i = 0; i < inMsg.len; i++)
  {
    ser.printf("%d ", inMsg.data[i]);
  }
  ser.printf("\n");
#endif

  switch (inMsg.id)
  {
  case STEERING_WHEEL_ID:
    switch (inMsg.data[0])
    {
    case 10:
      upShift();
      break;

    case 11:
      downShift();
      break;

    case 14:
      halfShift();
      break;

    default:
      break;
    }
    break;

  case PE1_ID:
    ECUTimer.reset();
    rpm = ((inMsg.data[1] << 8) + inMsg.data[0]);
    break;

  case PE6_ID:
  {
    uint16_t newTemp = ((inMsg.data[5] << 8) + inMsg.data[4]);
    if (newTemp > 32767)
    {
      newTemp = newTemp - 65536;
    }
#ifdef FURY
    waterTemp = ((newTemp / 10.0) * 1.8) + 32;
#else
    waterTemp = newTemp / 10.0;
#endif
    break;
  }

  case DBW_SENSORS_ID:      // each value is 0 - 100 as an integer
    APPS1 = inMsg.data[0];
    APPS2 = inMsg.data[1];
    TPS1 = inMsg.data[2];
    TPS2 = inMsg.data[3];

    break;

  default:
    break;
  }
}

/// @brief Print a status message over UART and CAN
void sendStatusMsg()
{
  while (1)
  {
    statusMsg.data[0] = neutral.read();
    statusMsg.data[1] = ETCEnable.read();
    // statusMsg.data[2] = (uint8_t)fan.read();
    // statusMsg.data[3] = (uint8_t)waterPump.read();
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

    ser.printf("\n");
    ser.printf("APPS1:\t\t %d\n", APPS1);
    ser.printf("TPS1:\t\t %d\n", TPS1);
    ser.printf("APPS vs TPS:\t %d\n", APPS1 - TPS1);
    ser.printf("ETCEnabled:\t %d\n", ETCEnable.read());
#endif

    ThisThread::sleep_for(100);
  }
}

void upShift()
{

#ifdef PRINT_STATUS
  ser.printf("\n---------- UPSHIFT ----------\n");
#endif

  sparkCut.write(0);
  ThisThread::sleep_for(10);
  upShiftPin.write(1);
  ThisThread::sleep_for(UPSHIFT_TIME); //
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
  ThisThread::sleep_for(DOWNSHIFT_TIME);
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
  ThisThread::sleep_for(HALFSHIFT_TIME);
  upShiftPin.write(0);
  downShiftPin.write(0);
}