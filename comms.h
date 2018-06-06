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

void CommsInit(uint32_t);
void ConsoleInit(void);
void ConsoleIntHandler(void);
void UARTIntHandler(void);
void UARTSend(const uint8_t*, uint32_t);

uint8_t recvIndex, STOP_BYTE;
uint32_t uartSysClock;

#endif /* COMMS_H_ */
