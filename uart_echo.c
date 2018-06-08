#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

#include "comms.h"

// System clock rate in Hz.
uint32_t g_ui32SysClock;

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

int main(void) {
    // Set the clocking to run directly from the crystal at 120MHz.
    g_ui32SysClock = MAP_SysCtlClockFreqSet(
            (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
            120000000);
    // Enable the GPIO port that is used for the on-board LED.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    // Enable the GPIO pins for the LED (PN0).
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    // Enable processor interrupts.
    ROM_IntMasterEnable();

    ConsoleInit();
    CommsInit(g_ui32SysClock);
    TimerInit();
    UARTprintf("Initialization done.\n");
    uint8_t addr = 0x1;
    uint8_t newID = 0;
//    UARTprintf("\nSet Address **********\n\n");
//    newID = setAddress(addr);
//    UARTprintf("\nSet EStopBehavior **********\n\n");
//    newID = setEStopBehavior(addr, 0x1);
//    UARTprintf("\nSet MaxCurrent **********\n\n");
//    newID = setMaxCurrent(addr, 11.1);
//    UARTprintf("\nGet Status **********\n\n");
//    newID = getStatus(addr);
//    UARTprintf("\nGet MaxCurrent **********\n\n");
//    newID = getMaxCurrent(addr);
//    UARTprintf("\nGet EStopBehavior **********\n\n");
//    newID = getStopBehavior(addr);
//    UARTprintf("\nGet Position **********\n\n");
//    newID = getPosition(addr);
//    UARTprintf("\nGet Velocity **********\n\n");
//    newID = getVelocity(addr);
//    UARTprintf("\nGetCurrent **********\n\n");
//    newID = getCurrent(addr);
//    UARTprintf("\nGet Temp **********\n\n");
//    newID = getTemperature(addr);
//    UARTprintf("\nRotate Position **********\n\n");
//    newID = rotateToPosition(addr, 11.1);
//    UARTprintf("\nRotate Velocity **********\n\n");
//    newID = rotateAtVelocity(addr, 11.1);
//    UARTprintf("\nRotate Current **********\n\n");
//    newID = rotateAtCurrent(addr, 11.1);
    // Loop forever echoing data through the UART.
    while(1) {
        if(sendMsgFlag == 1) {
            sendMsgFlag = 0;
            newID = setEStopBehavior(addr, 0x1);
//            UARTprintf("Sent message ID: %d\n", newID);
        }
    }
}
