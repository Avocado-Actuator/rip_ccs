#include "comms.h"

uint8_t ADDR, BRAIN_ADDR, BROADCAST_ADDR, ADDRSET_ADDR;
uint8_t recv[10];

// <<<<<<<<<<<<<<<>>>>>>>>>>>>>>
// <<<<<<<<<<<< INITS >>>>>>>>>>
// <<<<<<<<<<<<<<<>>>>>>>>>>>>>>

/**
 * Initializes UART7 for communication between boards
 *
 * @param g_ui32SysClock - system clock to sync with
 */
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
    MAX_PAR_VAL = 9;
    STOP_BYTE = '!';

    BRAIN_ADDR = 0x0;
    ADDR = 0x1;
    BROADCAST_ADDR = 0xFF;
    ADDRSET_ADDR = 0xFE;

    CMD_MASK = 0b10000000; // 1 is SET and 0 is GET
    PAR_MASK = 0b00000111; // gives just parameter selector bits

    UARTprintf("Communication initialized\n");
}

/**
 * Initializes UART0 for console output using UARTStdio
 */
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
// <<<<<<<<<<< UTILITIES >>>>>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>

/**
 * Sets new status
 *
 * @param newflag - new status flag
 */
uint8_t STATUS;
void setStatus(uint8_t newflag) { STATUS &= newflag; }

/**
 * Returns string corresponding to given enum value.
 *
 * @param par - enum value whose name to return
 * @return the name of the enum value given
 */
const char* getCommandName(enum Command cmd) {
    switch(cmd) {
        case Get: return "Get";
        case Set: return "Set";
        default: return "NOP";
    }
}

/**
 * Returns string corresponding to given enum value.
 *
 * @param par - enum value whose name to return
 * @return the name of the enum value given
 */
const char* getParameterName(enum Parameter par) {
    switch(par) {
        case Pos: return "Pos";
        case Vel: return "Vel";
        case Cur: return "Cur";
        case Tmp: return "Tmp";
        case MaxCur: return "MaxCur"; // max current
        case Status: return "Status"; // status register
        case EStop: return "EStop"; // emergency stop behavior
        case Adr: return "Adr";
        default: return "NOP";
    }
}

/**
 * Get personal address
 *
 * @return our address
 */
uint8_t getAddress() { return ADDR; }

/**
 * Set personal address
 *
 * @param our new address
 */
void setAddress(uint8_t addr) { ADDR = addr; }

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
// <<<<<<<<<<<< ACTIONS >>>>>>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>

/**
 * Sets given parameter to given value.
 *
 * @param par - parameter to set
 * @param value - value to set parameter to
 */
void setData(enum Parameter par, union Flyte * value) {
    UARTprintf("\nin setData\n");
    UARTprintf("Target value: %d\n", value->f);
    switch(par) {
        case Pos: {
//            setTargetAngle(value->f);
//            UARTprintf("New value: ");
//            UARTPrintFloat(getTargetAngle(), false);
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case Vel: {
//            setTargetVelocity(value->f);
//            UARTprintf("New value: ");
//            UARTPrintFloat(getTargetVelocity(), false);
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case Cur: {
//            setTargetCurrent(value->f);
//            UARTprintf("New value: ");
//            UARTPrintFloat(getTargetCurrent(), false);
//            setStatus(COMMAND_SUCCESS);
            break;
        }
        case Adr: {
            setAddress(value->bytes[0]);
//            UARTprintf("New value: ");
//            uint8_t temp = getAddress();
//            UARTprintf((const char*) &temp, false);
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case MaxCur: {
//            setMaxCurrent(value->f);
//            UARTprintf("New value: ");
//            UARTPrintFloat(getMaxCurrent(), false);
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case EStop: {
//            setEStop(value->bytes[0]);
            UARTprintf("New value: %x\n", value->bytes[0]);
//            uint8_t temp = getEStop();
//            UARTprintf((const char*) &temp, false);
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case Tmp: {
            UARTprintf("Invalid set, user tried to set temperature\n");
            setStatus(COMMAND_FAILURE);
            break;
        }
        default: {
            UARTprintf("Tried to set invaliad parameter, aborting\n");
            setStatus(COMMAND_FAILURE);
            break;
        }
    }
    return;
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
        UARTprintf("\nNew message:\n");
        int i;
        for (i = 0; i < length; ++i) {
    //            UARTprintf("Text[%d]: %c\n", i, buffer[i]);
            UARTprintf("%x", buffer[i]);
        }
        UARTprintf("\n");
    }

    uint8_t crcin = recv[length-2];
    if(verbose) UARTprintf("CRC byte: %d\n", crcin);
    if (crc8(0, (uint8_t *)recv, length-2) != crcin){
        // ********** ERROR ***********
        // Handle corrupted message
        UARTprintf("Corrupted message, panic!\n");
        return false;
    }

    // get address
    uint8_t tempaddr = buffer[0];
    if(verbose) UARTprintf("Address: %x\n", tempaddr);

    if(tempaddr == BROADCAST_ADDR){
        //TODO: Handle heartbeat from brain
        return false;
    } else if (tempaddr == ADDRSET_ADDR){
        setAddress(buffer[1]);
        if(verbose) UARTprintf("Set address to %x\n", getAddress());
        setStatus(COMMAND_SUCCESS);
        return true;
    }

    if(tempaddr != getAddress()) {
        if(verbose) UARTprintf("Not my address, abort\n");
        return false;
    }

    enum Command type = buffer[1] & CMD_MASK ? Set : Get;
    if(verbose) UARTprintf("Command: %s\n", getCommandName(type));


    enum Parameter par = buffer[1] & PAR_MASK;
    if(par > MAX_PAR_VAL) {
        if(verbose) UARTprintf("No parameter specified, abort");
//        setStatus(COMMAND_FAILURE);
        return true;
    }

    if(verbose) UARTprintf("Parameter: %s\n", getParameterName(par));

    if(type == Set) {
        if (length < 5){
            if(verbose) UARTprintf("No value provided, abort\n");
//            setStatus(COMMAND_FAILURE);
            return true;
        }

        // if the cmd is Set then the next entity is a value. This value is either a single float
        // which takes the next four bytes of buffer, or a single byte
        union Flyte setval;
        int i;
        for (i = 0; i < length - 4; i++)
            setval.bytes[i] = buffer[i+2];

        if(verbose) {
            UARTprintf("Setting value\n");
            UARTPrintFloat(setval.f, true);
        }
        setData(par, &setval);
        return true;
    } else {
//        sendData(par);
        return false;
    }
}

/**
 * Send message to UART connection.
 *
 * @param buffer - pointer to the message
 * @param length - the length of the message
 */
void UARTSend(const uint8_t *buffer, uint32_t length) {
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
}

// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>
// <<<<<<<<<<<< HANDLERS >>>>>>>>>>
// <<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>

/**
 * Console interrupt handler
 */
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

/**
 * UART interrupt handler
 */
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

    if(character == STOP_BYTE) {
        handleUART(recv, recvIndex, true, false);
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
