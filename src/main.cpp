#include "main.h"
#include <mbed.h>

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

CAN can0(PA_11, PA_12);
CANMessage inMsg;
CANMessage statusMsg;

Serial ser(USBTX, USBRX);

Thread checkTimerThread;
Thread coolingControlThread;
Thread sendStatusThread;

Timer ECUTimer;
Timer CANTimer;
Timer starterTimer;

bool ECUConnected = false;
bool CANConnected = false;
bool engineRunning = false;

double waterTemp = 0;
int rpm = 0;

int main() {
  wait_ms(100);

  ECUTimer.reset();
  ECUTimer.start();

  CANTimer.reset();
  CANTimer.start();

  starterTimer.reset();
  starterTimer.start();

  can0.frequency(CAN_BAUD);
  ser.baud(SERIAL_BAUD);

  initCANMessages();
  initGPIO();
  beepMotors();

  coolingControlThread.start(coolingControl);
  checkTimerThread.start(checkTimers);
  sendStatusThread.start(sendStatusMsg);

  while (1) {
    if (can0.read(inMsg)) {

#ifdef PRINT_CAN
      ser.printf("ID: %d", inMsg.id);
      ser.printf(" Data: ");
      for (int i = 0; i < inMsg.len; i++) {
        ser.printf("%d ", inMsg.data[i]);
      }
      ser.printf("\n");
#endif

      // Reset watchdog timers
      CANTimer.reset();
      if (inMsg.id == ECU_HEARTBEAT_ID) {
        ECUTimer.reset();
      }

      if (inMsg.id == 9) {
        rpm = inMsg.data[0] * 100; // testing only
        waterTemp = inMsg.data[1]; // testing only
      }

      if (inMsg.id == 98) {
        if (inMsg.data[0] == 91) {
          starter.write(1);
          starterTimer.reset();
        }
      }
    }
  }
}

void beepMotors() {
  // Beep water Pump

  waterPump.period_us(200);
  waterPump.write(0);
  wait_ms(100);

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 100; j++) {
      waterPump.write(.2);
      wait_us(800);
      waterPump.write(0);
      wait_us(400);
    }
    wait_ms(20);
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 400; j++) {
      waterPump.write(.2);
      wait_us(200);
      waterPump.write(0);
      wait_us(150);
    }
    wait_ms(20);
  }

  waterPump.period_us(PWM_PERIOD_US);
  waterPump.write(0);

  // Beep cooling fan

  fan.period_us(200);
  fan.write(0);
  wait_ms(100);

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 400; j++) {
      fan.write(.2);
      wait_us(200);
      fan.write(0);
      wait_us(150);
    }
    wait_ms(20);
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 100; j++) {
      fan.write(.2);
      wait_us(800);
      fan.write(0);
      wait_us(400);
    }
    wait_ms(20);
  }

  fan.period_us(PWM_PERIOD_US);
  fan.write(0);
}

void initGPIO() {
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

void checkTimers() {
  while (1) {
    if (ECUTimer.read_ms() > ECU_TIMEOUT_MS) {
      ECUConnected = false;
      engineRunning = false;

      // if ECU shuts down, this data must be reset
      rpm = 0;
      waterTemp = 0;
    } else {
      ECUConnected = true;
      engineRunning = true;
    }

    if (CANTimer.read_ms() > CAN_TIMEOUT_MS) {
      led.write(0);
      CANConnected = false;
    } else {
      led.write(1);
      CANConnected = true;
    }

    if (starterTimer.read_ms() > 100) { // timeout starter motor after 100ms
      starter.write(0);
    }
  }
}

void coolingControl() {
  while (1) {
    wait_ms(100);

    if (rpm > 1500 && ECUConnected) { // water pump speed is based on RPM
      if (waterPump.read() == 0) {    // if water pump is off, soft start it
        for (double i = 0; i < WATERPUMP_ACTIVE_DC; i += 0.01) {
          waterPump.write(i);
          wait_ms(20);
        }
      } else {
        waterPump.write(WATERPUMP_ACTIVE_DC);
      }
    } else {
      waterPump.write(0);
    }

    if (waterTemp > 150) {   // Fan is based on water temp
      if (fan.read() == 0) { // if the fan is off, soft start it
        for (double i = 0; i < FAN_ACTIVE_DC; i += 0.01) {
          fan.write(i);
          wait_ms(20);
        }
      } else {
        fan.write(FAN_ACTIVE_DC); // if the pump is on, keep it on
      }
    } else {
      fan.write(0);
    }

    // CAN bus disconnect protection (3 seconds)
    // if CAN bus disconnects, turn on pump and fan at active power level
    if (CANTimer.read_ms() > 3000) {
      waterPump.write(WATERPUMP_ACTIVE_DC);
      fan.write(FAN_ACTIVE_DC);
    }
  }
}

void sendStatusMsg() {
  while (1) {
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
#endif
    wait_ms(100);
  }
}

void initCANMessages() {
  statusMsg.id = 20;
  statusMsg.len = 8;
}

void upShift() {
  sparkCut.write(0);
  wait_ms(10);
  upShiftPin.write(1);
  wait_ms(70);
  upShiftPin.write(0);
  sparkCut.write(1);
}

void downShift() {
  sparkCut.write(0);
  wait_ms(10);
  downShiftPin.write(1);
  wait_ms(150);
  downShiftPin.write(0);
  sparkCut.write(1);
}