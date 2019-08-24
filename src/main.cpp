#include "main.h"
#include <mbed.h>

DigitalOut led(LED1);

PwmOut fan(PA_10);
PwmOut waterPump(PA_8);
PwmOut aux(PA_1);

DigitalOut sparkCut(PA_7);
DigitalOut upShiftPin(PB_1);
DigitalOut downShiftPin(PB_0);
DigitalOut ECUPower(PB_4);

DigitalIn neutral(PA_0);
AnalogIn anIn(PA_3);

CAN can0(PA_11, PA_12);
CANMessage inMsg;

Thread checkTimerThread;
Thread coolingControlThread;

Timer ECUTimer;
Timer CANTimer;

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

  can0.frequency(CAN_BAUD);

  initCANMessages();
  initGPIO();
  beepMotors();

  coolingControlThread.start(coolingControl);
  checkTimerThread.start(checkTimers);

  while (1) {

    if (can0.read(inMsg)) {
      CANTimer.reset();

      if (inMsg.id == ECU_HEARTBEAT_ID) {
        ECUTimer.reset();
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
  aux.period_us(PWM_PERIOD_US);

  waterPump.write(0);
  fan.write(0);
  aux.write(0);

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
    if (ECUTimer.read_ms() > 250) {
      ECUConnected = false;
      engineRunning = false;
    } else {
      ECUConnected = true;
      engineRunning = true;
    }

    if (CANTimer.read_ms() > 250) {
      led.write(0);
      CANConnected = false;
    } else {
      led.write(1);
      CANConnected = true;
    }
  }
}

void coolingControl() {
  while (1) {
    wait_ms(1000);

    if (waterTemp > 150) {   // Fan is based on water temp
      if (fan.read() == 0) { // if the pump is off, soft start it
        for (double i = 0; i < FAN_ACTIVE_DS; i += 0.01) {
          fan.write(i);
          wait_ms(20);
        }
      } else {
        fan.write(FAN_ACTIVE_DS); // if the pump is on, keep it on
      }
    }

    if (rpm > 1500 && ECUConnected) { // water pump speed is based on RPM
      if (waterPump.read() == 0) {    // if water pump was off, soft start it
        for (double i = 0; i < WATERPUMP_ACTIVE_DS; i += 0.01) {
          waterPump.write(i);
          wait_ms(20);
        }
      } else {
        waterPump.write(WATERPUMP_ACTIVE_DS);
      }
    } else {
      waterPump.write(0);
    }
  }

  // CAN bus disconnect protection (3 seconds)
  // if CAN bus disconnects, turn on pump and fan at active power level
  if (!CANConnected && CANTimer.read_ms() > 3000) {
    waterPump.write(WATERPUMP_ACTIVE_DS);
    fan.write(FAN_ACTIVE_DS);
  }
}

void initCANMessages() {}