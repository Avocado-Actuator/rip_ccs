#ifndef COMMS_H_
#define COMMS_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "utils/uartstdio.h"
#include "crc.h"
#include "driverlib/timer.h"

void CommsInit(uint32_t);
void ConsoleInit(void);
void ConsoleIntHandler(void);
void UARTIntHandler(void);
void UARTSend(const uint8_t*, uint32_t);
void TimerInit(void);

// C Library Functions

void heartbeat(void);

// <<<< set >>>>

// logistics
void setAddress(uint8_t);
void setMaxCurrent(uint8_t, float);
void setEStopBehavior(uint8_t, uint8_t);

// movement
void rotateToPosition(uint8_t, float);
void rotateAtVelocity(uint8_t, float);
void rotateAtCurrent(uint8_t, float);

// <<<< get >>>>

// logistics
void getStatus(uint8_t);
void getMaxCurrent(uint8_t);
void getStopBehavior(uint8_t);

// sensors
void getPosition(uint8_t);
void getVelocity(uint8_t);
void getCurrent(uint8_t);
void getTemperature(uint8_t);

uint8_t msgID, recvIndex, STOP_BYTE, MAX_PAR_VAL, CMD_MASK, PAR_MASK;
uint32_t uartSysClock, sendMsgFlag;

// data structures
union Flyte {
  float f;
  uint8_t bytes[sizeof(float)];
};

enum Command {
    Get = 0,
    Set = 1
};

enum Parameter {
    Adr     = 0,
    Tmp     = 1,
    Cur     = 2,
    Vel     = 3,
    Pos     = 4,
    MaxCur  = 5,
    EStop   = 6,
    Status  = 7
};

#endif /* COMMS_H_ */
