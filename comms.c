#include "comms.h"

uint8_t ADDR, BRAIN_ADDR, BROADCAST_ADDR, ADDRSET_ADDR;
uint8_t recv[10];

uint32_t TIME, HEARTBEAT_TIME, sendMsgFlag;

uint32_t heartbeat_counter = 0;

// <<<<<<<<<<<<<<<>>>>>>>>>>>>>>
// <<<<<<<<<<<< TIMER >>>>>>>>>>
// <<<<<<<<<<<<<<<>>>>>>>>>>>>>>
void Timer0IntHandler(void) {
    // Clear the timer interrupt.
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Update the interrupt status.
    ++TIME;
    // Send a message every 50ms
    if (TIME % 50 == 0){
        sendMsgFlag = 1;
    }
    // island time except for heartbeats
    ++HEARTBEAT_TIME;
    // send heartbeat every 500 ms, twice as fast as they're expected
    if(HEARTBEAT_TIME % 200 == 0) {
        heartbeat_counter++;
        heartbeat();
    }
}

void TimerInit(void) {
    /************** Initialization for timer (1ms)  *****************/
    //Enable the timer peripherals
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // Configure 32-bit periodic timers.
    //1ms timer
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, uartSysClock/1000);//was 1000, trigger every 1ms, 1000Hz
    // Setup the interrupts for the timer timeouts.
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Enable the timers.
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);

    TIME = 0;
    HEARTBEAT_TIME = 0;
    sendMsgFlag = 0;

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

    msgID = 0;
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
    if(verbose) {
//        UARTSend((uint8_t *) buffer, length);
        int i;
        UARTprintf("*************************************************\n");
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

    uint8_t crcin = buffer[length-2];
    if (crc8(0, (uint8_t *)buffer, length-2) != crcin){
        // ********** ERROR ***********
        // Handle corrupted message
        UARTprintf("Corrupted message, panic!\n");
        return false;
    } else if (buffer[0] != BRAIN_ADDR){
        UARTprintf("Not my address, abort\n");
        return false;
    }
    // RESUME handle the responses, probably by including the parameter in the message, so we can switch on that and save into global

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

    ROM_UARTIntDisable(UART7_BASE, UART_INT_RX | UART_INT_RT); // So we can't be interrupted by another character arriving

    // Loop while there are characters in the receive FIFO.
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
    ROM_UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);
}

// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
// <<<<<<<<<<< C LIBRARY >>>>>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>

// <<<<<<<<<<<<< SENDS >>>>>>>>>>>>>

/**
 * Send empty message on address BROADCAST_ADDR so avocados know to keep working.
 */
void heartbeat() { UARTSend((uint8_t[]) { BROADCAST_ADDR }, 1); /*UARTprintf("%d*\n", heartbeat_counter);*/ }

/**
 * Send a get message.
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 */
void sendGet(uint8_t addr, uint8_t pParMask) {
    // since get is 0 in msb of pParMask no need to do anything
    uint8_t msg[2] = { addr, pParMask };
    UARTSend(msg, 2);
}

/**
 * Send a set message.
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 * @param pParVal - float value to set
 */
void sendSetFloatPar(uint8_t addr, uint8_t pParMask, float pParVal) {
    uint8_t msgLen = 6; // 1 byte for addr, 1 for mask, 4 for val
    uint8_t msg[msgLen];
    msg[0] = addr;
    msg[1] = 0b10000000 | pParMask;

    union Flyte parVal;
    parVal.f = pParVal;
    int i;
    for(i = 0; i < 4; ++i)
        msg[i+2] = parVal.bytes[i];

    UARTSend(msg, msgLen);
}

/**
 * Send a set message.
 *
 * @param addr - address of actuator
 * @param pParMask - mask of parameter to get
 * @param pParVal - byte value to set
 */
void sendSetBytePar(uint8_t addr, uint8_t pParMask, uint8_t pParVal) {
    uint8_t msgLen = 3; // 1 byte for addr, 1 for mask, 1 for val
    uint8_t msg[msgLen];
    msg[0] = addr;
    msg[1] = 0b10000000 | pParMask;
    msg[2] = pParVal;

    UARTSend(msg, msgLen);
}


// <<<<<<<<<<<<< MESSAGES >>>>>>>>>>>>>

// <<<<<<< SET >>>>>>>

/**
 * Set address of any other devices on bus.
 *
 * @param addr - address to set on device
 */
void setAddress(uint8_t addr) {
    // Special case doesn't use a parameter mask byte in protocol structure
    uint8_t msglen = 2;
    uint8_t msg[msglen];
    msg[0] = ADDRSET_ADDR;
    msg[1] = addr;
    UARTSend(msg, 2);
}

/**
 * Set maximum current for actuator.
 *
 * @param addr - address of actuator
 * @param maxCurr - maximum current in amps that actuator should throttle to
 */
void setMaxCurrent(uint8_t addr, float maxCurr) {
    sendSetFloatPar(addr, (uint8_t) MaxCur, maxCurr);
}

/**
 * Set behavior to take in case of brain failure (identified through lack of
 * heartbeats).
 *
 * @param addr - address of actuator
 * @param eStopBehavior - bitmask indicating behavior to take in case of failure. 1 = hold position, 0 = kill motor power
 */
void setEStopBehavior(uint8_t addr, uint8_t eStopBehavior) {
    sendSetBytePar(addr, (uint8_t) EStop, eStopBehavior);
}

/**
 * Rotate given actuator to given position.
 *
 * @param addr - address of actuator
 * @param pos - angle to rotate actuator to (in radians)
 */
void rotateToPosition(uint8_t addr, float pos) {
    sendSetFloatPar(addr, (uint8_t) Pos, pos);
}

/**
 * Rotate given actuator at given velocity.
 *
 * @param addr - address of actuator
 * @param vel - velocity to rotate at (in rpm)
 */
void rotateAtVelocity(uint8_t addr, float vel) {
    sendSetFloatPar(addr, (uint8_t) Vel, vel);
}

/**
 * Rotate given actuator at given current.
 *
 * @param addr - address of actuator
 * @param cur - current to rotate at (in amps)
 */
void rotateAtCurrent(uint8_t addr, float cur) {
    sendSetFloatPar(addr, (uint8_t) Cur, cur);
}

// <<<<<<< GET >>>>>>>

/**
 * Get status of avocado.
 *
 * @param addr - address to set on device
 */
void getStatus(uint8_t addr) {
    sendGet(addr, (uint8_t) Status);
}

/**
 * Get current maximum current.
 *
 * @param addr - address of actuator
 */
void getMaxCurrent(uint8_t addr) {
    sendGet(addr, (uint8_t) MaxCur);
}

/**
 * Get current behavior to take in case of brain failure (identified through
 * lack of heartbeats).
 *
 * @param addr - address of actuator
 */
void getStopBehavior(uint8_t addr) {
    sendGet(addr, (uint8_t) EStop);
}

/**
 * Get current position.
 *
 * @param addr - address of actuator
 */
void getPosition(uint8_t addr) {
    sendGet(addr, (uint8_t) Pos);
}

/**
 * Get current velocity.
 *
 * @param addr - address of actuator
 */
void getVelocity(uint8_t addr) {
    sendGet(addr, (uint8_t) Vel);
}

/**
 * Get current current.
 *
 * @param addr - address of actuator
 */
void getCurrent(uint8_t addr) {
    sendGet(addr, (uint8_t) Cur);
}

/**
 * Get temperature of actuator.
 *
 * @param addr - address of actuator
 */
void getTemperature(uint8_t addr) {
    sendGet(addr, (uint8_t) Tmp);
}
