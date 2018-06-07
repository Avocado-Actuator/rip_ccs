#include "comms.h"

uint8_t ADDR, BRAIN_ADDR, BROADCAST_ADDR, ADDRSET_ADDR;
uint8_t recv[10];

uint8_t buffer_time_flag;
uint32_t TIME, BUFFER_TIME, HEARTBEAT_TIME;

uint32_t heartbeat_counter = 0;

// <<<<<<<<<<<<<<<>>>>>>>>>>>>>>
// <<<<<<<<<<<< TIMER >>>>>>>>>>
// <<<<<<<<<<<<<<<>>>>>>>>>>>>>>
void Timer0IntHandler(void) {
    // Clear the timer interrupt.
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Update the interrupt status.
    ++TIME;

    // island time except for heartbeats
    ++HEARTBEAT_TIME;
    // send heartbeat every 100 ms, twice as fast as they're expected
    if(HEARTBEAT_TIME % 10'000 == 0) {
        heartbeat_counter++;
        heartbeat();
    }

    // island time except for a buffer
    ++BUFFER_TIME;
    // 200 chosen arbitrarily
    if(BUFFER_TIME % 200 == 0) { buffer_time_flag = 1; }
}

void TimerInit(void) {
    /************** Initialization for timer (1ms)  *****************/
    //Enable the timer peripherals
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // Configure 32-bit periodic timers.
    //1ms timer
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, uartSysClock/100000);//was 1000, trigger every 1ms, 1000Hz
    // Setup the interrupts for the timer timeouts.
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Enable the timers.
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);

    TIME = 0;
    BUFFER_TIME = 0;
    HEARTBEAT_TIME = 0;

    buffer_time_flag = 0;
    UARTprintf("Communication initialized\n");
}

// <<<<<<<<<<<<<<<>>>>>>>>>>>>>>
// <<<<<<<<<<<< INITS >>>>>>>>>>
// <<<<<<<<<<<<<<<>>>>>>>>>>>>>>

void CommsInit(uint32_t g_ui32SysClock){
    // Copy over the clock created in main
    uartSysClock = g_ui32SysClock;
    // Enable the peripherals used
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC4_U7RX);
    GPIOPinConfigure(GPIO_PC5_U7TX);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // Enable GPIO port C pin 6 as the RS-485 transceiver rx/tx pin
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    // enable tied pin as input to read output of enable pin
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7);
    // Write transceiver enable pin low for listening
    //UARTSetRead();
    // Configure the UART for 115,200, 8-N-1 operation.
    ROM_UARTConfigSetExpClk(UART7_BASE, g_ui32SysClock, 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    // Enable the UART interrupt.
    ROM_IntEnable(INT_UART7);
    ROM_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);

    recvIndex = 0;
    STOP_BYTE = '!';

    BRAIN_ADDR = 0x0;
    ADDR = 0x1;
    BROADCAST_ADDR = 0xFF;
    ADDRSET_ADDR = 0xFE;

    CMD_MASK = 0b10000000; // 1 is SET and 0 is GET
    PAR_MASK = 0b00000111; // gives just parameter selector bits

    UARTprintf("Communication initialized\n");
}

//*****************************************************************************
// Initializes UART0 for console output using UARTStdio
//*****************************************************************************
void ConsoleInit(void) {
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 9600, 16000000);

    // Enable the UART interrupt.
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTprintf("Console initialized\n");
}


// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
// <<<<<<<<<< UTILITIES >>>>>>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>


/**
 * Prints given float
 *
 * @param val - float to print
 * @param verbose - how descriptive to be in printing
 */
void UARTPrintFloat(float val, bool verbose) {
    char str[100]; // pretty arbitrarily chosen
    sprintf(str, "%f", val);
    verbose
        ? UARTprintf("val, length: %s, %d\n", str, strlen(str))
        : UARTprintf("%s\n", str);
}


// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
// <<<<<<< MESSAGE HANDLING >>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>

/**
 * Takes actions on message as appropriate.
 *
 * If address does not match our own, bail out and send message on.
 *
 * @param buffer - pointer to the message
 * @param length - the length of the message
 * @param verbose - if true print to console for debugging
 * @param echo - if true simply echo the message, can also be helpful for debugging
 * @return if we successfully handled a message meant for us
 */
bool handleUART(uint8_t* buffer, uint32_t length, bool verbose, bool echo) {
    if(echo) {
//        UARTSend((uint8_t *) buffer, length);
        int i;
        UARTprintf("Address: %x\n", buffer[0]);
        if (length == 4) {
            UARTprintf("Value: %x\n", buffer[1]);
        } else if (length == 7) {
            union Flyte val;
            for (i = 0; i < 4; i++){
                val.bytes[i] = buffer[i+1];
            }
            UARTprintf("Value: ");
            UARTPrintFloat(val.f, false);
        } else {
            UARTprintf("Message:\n");
            for (i = 0; i < length; i++){
                UARTprintf("[%d]: %x\n", i, buffer[i]);
            }
        }
        return false;
    }

    return false;
}

/**
 * Send message to UART connection.
 *
 * @param buffer - pointer to the message
 * @param length - the length of the message
 */
void UARTSend(const uint8_t *buffer, uint32_t length) {
    ROM_TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // So we can't be interrupted by the heartbeat trying to send
    // Add CRC byte to message
    uint8_t crc = crc8(0, (const unsigned char*) buffer, length);
    bool space;
    int i;
    for (i = 0; i < length; i++) {
        // write the next character to the UART.
        // putchar returns false if the send FIFO is full
        space = ROM_UARTCharPutNonBlocking(UART7_BASE, buffer[i]);
        // if send FIFO is full, wait until we can put the char in
        while (!space) {
            space = ROM_UARTCharPutNonBlocking(UART7_BASE, buffer[i]);
        }
    }
    space = ROM_UARTCharPutNonBlocking(UART7_BASE, crc);
    // if send FIFO is full, wait until we can put the char in
    while (!space) {
        space = ROM_UARTCharPutNonBlocking(UART7_BASE, crc);
    }
    space = ROM_UARTCharPutNonBlocking(UART7_BASE, STOP_BYTE);
    // if send FIFO is full, wait until we can put the char in
    while (!space) {
        space = ROM_UARTCharPutNonBlocking(UART7_BASE, STOP_BYTE);
    }

    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
// <<<<<<<<<<<< HANDLERS >>>>>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>

//*****************************************************************************
// The console interrupt handler.
//*****************************************************************************
void ConsoleIntHandler(void) {
    // Get the interrupt status.
    uint32_t ui32Status = ROM_UARTIntStatus(UART0_BASE, true);
    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(ROM_UARTCharsAvail(UART0_BASE)) {
        // Read the next character from the UART and write it back to the UART.
        ROM_UARTCharPutNonBlocking(UART7_BASE, ROM_UARTCharGetNonBlocking(UART0_BASE));
        // Blink the LED to show a character transfer is occuring.
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        SysCtlDelay(uartSysClock / (1000 * 3));
        // Turn off the LED
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
    }
}

//*****************************************************************************
// The UART interrupt handler.
//*****************************************************************************
void UARTIntHandler(void) {
    // Get the interrrupt status.
    uint32_t ui32Status = ROM_UARTIntStatus(UART7_BASE, true);
    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART7_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
//    while(ROM_UARTCharsAvail(UART7_BASE)) {
    // Read the next character from the UART and write it back to the UART.
    uint8_t character = ROM_UARTCharGetNonBlocking(UART7_BASE);
    recv[recvIndex++] = character;
//    UARTprintf("Byte: %x\n", character);

    if(character == STOP_BYTE) {
        handleUART(recv, recvIndex, true, true);
        recvIndex = 0;
    }

//    ROM_UARTCharPutNonBlocking(UART0_BASE, character);
    // Blink the LED to show a character transfer is occuring.
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
    // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
    SysCtlDelay(uartSysClock / (1000 * 3));
    // Turn off the LED
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
//    }
}

// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
// <<<<<<<<<<< C LIBRARY >>>>>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>


/**
 * Send empty message on address BROADCAST_ADDR so avocados know to keep working.
 */
void heartbeat() { UARTSend((uint8_t[]) { BROADCAST_ADDR }, 1); }
