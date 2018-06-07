#include "comms.h"

uint8_t ADDR, BRAIN_ADDR, BROADCAST_ADDR, ADDRSET_ADDR;
uint8_t recv[10];

uint8_t buffer_time_flag;
uint32_t TIME, BUFFER_TIME, HEARTBEAT_TIME;
uint32_t panic_counter;

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
    // expect heartbeat every 200 ms
    // user should send multiple in that time in case of corruption
    if(HEARTBEAT_TIME % 20'000 == 0) {
        UARTprintf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n%d PANIC ESTOP, NO HEARTBEAT\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n", panic_counter++);
    }

    // island time except for a buffer
    ++BUFFER_TIME;
    // 200 chosen arbitrarily
    if(BUFFER_TIME % 200) { buffer_time_flag = 1; }
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

    // matching flags should be inverse
    ESTOP_HOLD      = 0b00000001;
    ESTOP_KILL      = 0b11111110;

    COMMAND_SUCCESS = 0b00000010;
    COMMAND_FAILURE = 0b11111101;

    OUTPUT_LIMITING = 0b00000100;
    OUTPUT_FREE     = 0b11111011;

    STATUS          = 0b00000000;

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
void setStatus(uint8_t newflag) { STATUS |= newflag; }
void clearStatus(uint8_t newflag) { STATUS &= newflag; }
uint8_t getStatus() { return STATUS; }

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
 * @param verbose - if true print to console for debugging
 */
void setData(enum Parameter par, union Flyte * value, bool verbose) {
    UARTprintf("\nin setData\n");
    UARTprintf("Target value: %d\n", value->f);
    switch(par) {
        case Pos: {
//            setTargetAngle(value->f);
            if(verbose) {
                UARTprintf("New target angle: ");
//                UARTPrintFloat(getTargetAngle(), false);
            }
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case Vel: {
//            setTargetVelocity(value->f);
            if(verbose) {
                UARTprintf("New target velocity: ");
//                UARTPrintFloat(getTargetVelocity(), false);
            }
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case Cur: {
//            setTargetCurrent(value->f);
            if(verbose) {
                UARTprintf("New target current: ");
//                UARTPrintFloat(getTargetCurrent(), false);
            }
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case Adr: {
            setAddress(value->bytes[0]);
            if(verbose) {
                UARTprintf("New address: ");
                uint8_t temp = getAddress();
                UARTprintf((const char*) &temp, false);
            }
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case MaxCur: {
//            setMaxCurrent(value->f);
            if(verbose) {
                UARTprintf("New max current: ");
//                UARTPrintFloat(getMaxCurrent(), false);
            }
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case EStop: {
//            setEStop(value->bytes[0]);
            if(verbose) {
                UARTprintf("New estop behavior: %x\n", value->bytes[0]);
//                uint8_t temp = getEStop();
//                UARTprintf((const char*) &temp, false);
            }
            setStatus(COMMAND_SUCCESS);
            break;
        }
        case Tmp: {
            if(verbose) UARTprintf("Invalid set, user tried to set temperature\n");
            clearStatus(COMMAND_FAILURE);
            break;
        }
        default: {
            if(verbose) UARTprintf("Tried to set invaliad parameter, aborting\n");
            clearStatus(COMMAND_FAILURE);
            break;
        }
    }
}

/**
 * Sends value of given parameter on UART.
 *
 * @param par - parameter to send
 */
void sendData(enum Parameter par) {
    UARTprintf("\nin sendData\n");
    union Flyte value;
    switch(par) {
//        case Pos: {
//            UARTprintf("Current pos: ");
//            UARTPrintFloat(getAngle(), false);
//            value.f = getAngle();
//            setStatus(COMMAND_SUCCESS);
//            break;
//        }
//        case Vel: {
//            UARTprintf("Current vel: ");
//            UARTPrintFloat(getVelocity(), false);
//            value.f = getVelocity();
//            setStatus(COMMAND_SUCCESS);
//            break;
//        }
//
//        case Cur: {
//            UARTprintf("Current current: ");
//            UARTPrintFloat(getCurrent(), false);
//            value.f = getCurrent();
//            setStatus(COMMAND_SUCCESS);
//            break;
//        }
//
        case Tmp: {
            UARTprintf("Current temperature: \n");
//            UARTPrintFloat(getTemp(), false);
//            value.f = getTemp();
            value.f = 123.987;
            UARTprintf("getStatus() before = %x\n", getStatus());
            setStatus(COMMAND_SUCCESS);
            UARTprintf("getStatus() after = %x\n", getStatus());
            break;
        }

//        case MaxCur: {
//            UARTprintf("Max Current Setting: ");
//            UARTPrintFloat(getMaxCurrent(), false);
//            value.f = getMaxCurrent();
//            setStatus(COMMAND_SUCCESS);
//            break;
//        }
//
//        case EStop: {
//            UARTprintf("Emergency Stop Behaviour: ");
//            uint8_t temp = getEStop();
//            UARTprintf((const char*) &temp, false);
//            value.bytes[0] = temp;
//            setStatus(COMMAND_SUCCESS);
//            uint8_t bytes = value.bytes[0];
//            UARTSend((const uint8_t*) &bytes, 1);
//            return;
//        }
//
//        case Status: {
//            UARTprintf("Status Register: ");
//            uint8_t temp = getStatus();
//            UARTprintf((const char*) &temp, false);
//            value.bytes[0] = temp;
//            setStatus(COMMAND_SUCCESS);
//            uint8_t bytes = value.bytes[0];
//            UARTSend((const uint8_t*) &bytes, 1);
//            return;
//        }

        default: {
            UARTprintf("Asked for invalid parameter, aborting\n");
             clearStatus(COMMAND_FAILURE);
             break;
        }
    }
    if (!(getStatus() & ~COMMAND_FAILURE)){ // true if command failed
        UARTprintf("Get failed :(\n");
        UARTprintf("getStatus() = %x\n", getStatus());
        uint8_t temp = getStatus();
        UARTSend(&temp, 1);
    } else {
        UARTprintf("Get succeeded! :)\n");
        UARTPrintFloat(value.f, true);
        UARTSend(value.bytes, 4);
    }
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
            UARTprintf("Byte %d: %x", i, buffer[i]);
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
//        clearStatus(COMMAND_FAILURE);
        return true;
    }

    if(verbose) UARTprintf("Parameter: %s\n", getParameterName(par));

    if(type == Set) {
        if (length < 5){
            if(verbose) UARTprintf("No value provided, abort\n");
//            clearStatus(COMMAND_FAILURE);
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
        setData(par, &setval, verbose);
        return true;
    } else {
        sendData(par);
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
    UARTprintf("In UARTSend, message:\n");
    uint8_t msg[8];
    msg[0] = BRAIN_ADDR;
    int i, len;
    len = 1;
    for (i = 0; i < length; i++){
        msg[i+1] = buffer[i];
        len++;
    }
    // Add CRC byte to message
    uint8_t crc = crc8(0, (const unsigned char*) &msg, len);
    msg[len++] = crc;
    msg[len++] = STOP_BYTE;

    for(i = 0; i < len; i++) {
        UARTprintf("Byte %d: %x\n", i, msg[i]);
    }

    bool space;
    for (i = 0; i < len; i++) {
        // write the next character to the UART.
        // putchar returns false if the send FIFO is full
        space = ROM_UARTCharPutNonBlocking(UART7_BASE, msg[i]);
        // if send FIFO is full, wait until we can put the char in
        while (!space)
            space = ROM_UARTCharPutNonBlocking(UART7_BASE, msg[i]);
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
