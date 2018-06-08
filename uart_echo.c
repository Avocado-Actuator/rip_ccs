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
    uint8_t ids[13];
    UARTprintf("\nSet Address **********\n\n");
    ids[0] = setAddress(addr);
    SysCtlDelay(10'000'000);
    UARTprintf("\nSet EStopBehavior **********\n\n");
    ids[1] = setEStopBehavior(addr, 0x1);
    SysCtlDelay(10'000'000);
    UARTprintf("\nSet MaxCurrent **********\n\n");
    ids[2] = setMaxCurrent(addr, 11.1);
    SysCtlDelay(10'000'000);
    UARTprintf("\nRotate Position **********\n\n");
    ids[3] = rotateToPosition(addr, 12.1);
    SysCtlDelay(10'000'000);
    UARTprintf("\nRotate Velocity **********\n\n");
    ids[4] = rotateAtVelocity(addr, 13.1);
    SysCtlDelay(10'000'000);
    UARTprintf("\nRotate Current **********\n\n");
    ids[5] = rotateAtCurrent(addr, 14.1);
    SysCtlDelay(10'000'000);
    UARTprintf("\nGet Status **********\n\n");
    ids[6] = getStatus(addr);
    SysCtlDelay(10'000'000);
    UARTprintf("\nGet EStopBehavior **********\n\n");
    ids[7] = getStopBehavior(addr);
    SysCtlDelay(10'000'000);
    UARTprintf("\nGet MaxCurrent **********\n\n");
    ids[8] = getMaxCurrent(addr);
    SysCtlDelay(10'000'000);
    UARTprintf("\nGet Position **********\n\n");
    ids[9] = getPosition(addr);
    SysCtlDelay(10'000'000);
    UARTprintf("\nGet Velocity **********\n\n");
    ids[10] = getVelocity(addr);
    SysCtlDelay(10'000'000);
    UARTprintf("\nGetCurrent **********\n\n");
    ids[11] = getCurrent(addr);
    SysCtlDelay(10'000'000);
    UARTprintf("\nGet Temp **********\n\n");
    ids[12] = getTemperature(addr);
    SysCtlDelay(10'000'000);

//    UARTprintf("**********\n");
//    UARTprintf("ID: 0\n");
//    UARTprintf("Addr set status: %x\n", response_buffer[(uint8_t)ids[0]].bytes[0]);
//    UARTprintf("**********\n");
//    UARTprintf("ID: 3\n");
//    UARTprintf("Get status: %x\n", response_buffer[(uint8_t)ids[3]].bytes[0]);
    int i;
    for(i=0; i<13; i++){
        UARTprintf("********\n");
        UARTprintf("ID: %x\n", ids[i]);
        if (i<8){
            UARTprintf("Value: %x\n", response_buffer[(uint8_t)ids[i]].bytes[0]);
        }
        else {
            UARTprintf("Value: ");
            UARTPrintFloat(response_buffer[(uint8_t)ids[i]].f, false);
        }
    }
    uint8_t newID;
    // Loop forever echoing data through the UART.
    while(1) {
        if(sendMsgFlag == 1) {
            sendMsgFlag = 0;
            newID = setAddress(addr);
//            UARTprintf("Sent message ID: %d\n", newID);
        }
    }
}
