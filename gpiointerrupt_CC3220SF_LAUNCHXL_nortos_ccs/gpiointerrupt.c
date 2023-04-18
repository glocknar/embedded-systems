/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 *
 *  ======== gpiointerrupt.c ========
 *
 *  Modified by Michael Joseph Honie
 *  Southern New Hampshire University
 *  Emerging Systems, Architectures & Technologies
 *  Professor Roland Morales
 *  April 16th, 2023
 *
 *  Blink "SOS" or "OK" based on button press
 *
 */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

volatile bool lowerTempButtonPressed;

/*  GPIO interrupt on CONFIG_GPIO_BUTTON_0 (lower temperature) */
void lowerTempButtonPress(uint_least8_t index) {
    // Clear the interrupt and update the button state
    uint_fast8_t rising = GPIO_read(index);
    // Toggle button state
    if(rising) {
        lowerTempButtonPressed = true;
    }
    else {
        lowerTempButtonPressed = false;
    }
}

volatile bool raiseTempButtonPressed;

/*  GPIO interrupt on CONFIG_GPIO_BUTTON_1 (raise temperature) */
void raiseTempButtonPress(uint_least8_t index) {
    // Clear the interrupt and update the button state
    uint_fast8_t rising = GPIO_read(index);
    // Toggle button state
    if(rising) {
        raiseTempButtonPressed = true;
    }
    else {
        raiseTempButtonPressed = false;
    }
}

/* Initialize general purpose input/output driver */
void initGPIO(void) {
    // Initialize the input/output driver
    GPIO_init();
    // We need the red LED to show that the thermostat is ON (heating or cooling) or OFF
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    // The thermostat is off by default
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    // We need two buttons to lower and raise the temperature
    // Interrupt on both rising and falling edges ("hold" button)
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_BOTH_EDGES | GPIO_CFG_INT_ENABLE);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_BOTH_EDGES | GPIO_CFG_INT_ENABLE);
    // Functions to handle button-presses
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, lowerTempButtonPress);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, raiseTempButtonPress);
}

// We need a handle to UART to asynchronously report the current
// state of the thermostat, i.e., print data to standard output
UART_Handle uart;

// UART Global Variables
char output[64];
int bytesToSend;

// Print to standard output UART macro
#define DISPLAY(x) UART_write(uart, &output, x);

void initUART(void) {
    UART_Params uartParams;
    // Initialize the driver
    UART_init();
    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
    /* UART_open() failed */
        while (1);
    }
}

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// We need a handle to I2C to interact with the temperature sensor via USB
I2C_Handle i2c;

void initI2C(void) {

    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))
    // Initialize the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    for (i=0; i<3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }

        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

/* The task scheduler need a timer trigger tasks on time */
Timer_Handle task_timer;

/* Flag set when the smallest relevant time interval as elapsed (tasksPeriodGCD_U) */
volatile unsigned char TimerFlag = 0;

/* Lets the task scheduler know when the smallest relevant time interval has elapsed (tasksPeriodGCD_U) */
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;
}

/* smallest relevant time interval */
const unsigned long tasksPeriodGCD_U = 100000;

/* Create and start the task scheduler timer */
void initTimer(void) {

    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);

    params.period = tasksPeriodGCD_U;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    task_timer = Timer_open(CONFIG_TIMER_0, &params);

    if (task_timer == NULL) {
        // If creating timer failed, we get stuck here.
        while (1) {}
    }
    if (Timer_start(task_timer) == Timer_STATUS_ERROR) {
        // If starting timer failed, we get stuck here.
        while (1) {}
    }
}

int16_t readTemp(void) {

    int16_t temperature = 0;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction)) {
        // Extract degrees C from the received data;
        // see TMP sensor data sheet
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        // If the MSB is set '1', then we have a 2's complement
        // negative value which needs to be sign extended
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    }
    else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
}

/* Temperature measurements are in Celsius */
#define DEFAULT_TARGET_TEMP 28

/* The thermostat works to drive the ambient temperature to the target temperature */
int targetTemp = DEFAULT_TARGET_TEMP;

bool cooling, heating = false;

/* The reporting task doesn't really have a state. We include one for completeness */
enum ReportStates { REPORT_STATE } ReportState = REPORT_STATE;

/* We print the thermostat state to the standard output */
int reportResponse(int reportState) {
    int16_t ambientTemp = readTemp();
    uint32_t ticks = Timer_getCount(task_timer);

    DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", ambientTemp, targetTemp, heating, ticks))

    return reportState;
}

/* Not implemented */
void coldLEDOff() {
    // GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
}

/* Not implemented */
void coldLEDOn() {
    // GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
}

void heatLEDOff() {
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
}

void heatLEDOn() {
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
}

/* LEDs are OFF by default */
enum CoolLEDStates { COOL_LED_OFF, COOL_LED_ON } CoolLEDState = COOL_LED_OFF;
/* LEDs are OFF by default */
enum HeatLEDStates { HEAT_LED_OFF, HEAT_LED_ON } HeatLEDState = HEAT_LED_OFF;

/* State machine for cooling LED (not implemented) */
int coolLEDResponse(int coolLEDState) {

    // Transitions
    switch(coolLEDState) {
        case COOL_LED_OFF:
            break;
        case COOL_LED_ON:
            break;
        default:
            break;
    }

    // Actions
    switch(coolLEDState) {
        case COOL_LED_OFF:
            break;
        case COOL_LED_ON:
            break;
        default:
            break;
     }

    return coolLEDState;
}

/* State machine for heating LED */
int heatLEDResponse(int heatLEDState) {

    // Transitions
    switch(heatLEDState) {
        case HEAT_LED_OFF:
            if(heating)
                heatLEDState = HEAT_LED_ON;
            break;
        case HEAT_LED_ON:
            if(!heating)
                heatLEDState = HEAT_LED_OFF;
            break;
        default:
            break;
    }

    // Actions
    switch(heatLEDState) {
        case HEAT_LED_OFF:
            heatLEDOff();
            break;
        case HEAT_LED_ON:
            heatLEDOn();
            break;
        default:
            break;
     }

    return heatLEDState;
}

/* Thermostat is OFF by default */
enum ThermStates { THERM_OFF, THERM_COOL, THERM_HEAT } ThermState = THERM_OFF;

/* State machine for heating/cooling */
int temperatureResponse(int thermState) {

    int16_t ambientTemp = readTemp();
    // Transitions
    switch(thermState) {
        case THERM_OFF:
            if(targetTemp < ambientTemp)
                thermState = THERM_COOL;
            if(targetTemp > ambientTemp)
                thermState = THERM_HEAT;
            break;
        case THERM_COOL:
            if(targetTemp >= ambientTemp)
                thermState = THERM_OFF;
            break;
        case THERM_HEAT:
            if(targetTemp <= ambientTemp)
                thermState = THERM_OFF;
            break;
        default:
            thermState = THERM_OFF;
            break;
    }

    // Actions
    switch(thermState) {
        case THERM_OFF:
            heating = false;
            cooling = false;
            break;
        case THERM_COOL:
            heating = false;
            cooling = true;
            break;
        case THERM_HEAT:
            cooling = false;
            heating = true;
            break;
        default:
            break;
     }

    return thermState;
}

void lowerTemperature(Timer_Handle myHandle, int_fast16_t status) {
    targetTemp--;
}

void raiseTemperature(Timer_Handle myHandle, int_fast16_t status) {
    targetTemp++;
}

// Interval at which we lower target temperature while button is held
const unsigned long periodLowerTemperature_U = 200000;
// Interval at which we raise target temperature while button is held
const unsigned long periodRaiseTemperature_U = 200000;

/* Create instance of the "lower target temperature" timer */
Timer_Handle createLowerTargetTempTimer() {

    Timer_Handle timer;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);

    // Lower the temperature by 1 degree every 200 milliseconds
    params.period = periodLowerTemperature_U;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_ONESHOT_CALLBACK;
    params.timerCallback = lowerTemperature;

    timer = Timer_open(CONFIG_TIMER_1, &params);

    if (timer == NULL) {
        // If starting timer failed, we get stuck here.
        while (1) {}
    }

    return timer;
}

/* Create instance of the "raise target temperature" timer*/
Timer_Handle createRaiseTargetTempTimer() {

    Timer_Handle timer;
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);

    // Raise the temperature by 1 degree every 200 milliseconds
    params.period = periodRaiseTemperature_U;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_ONESHOT_CALLBACK;
    params.timerCallback = raiseTemperature;

    timer = Timer_open(CONFIG_TIMER_2, &params);

    if (timer == NULL) {
        // If starting timer failed, we get stuck here.
        while (1) {}
    }

    return timer;
}

/* Calls a function that lowers the target temperature 1 degree every 200 milliseconds */
Timer_Handle lower_target_temp_timer;
/* Calls a function that raises the target temperature 1 degree every 200 milliseconds */
Timer_Handle raise_target_temp_timer;

/* Get a singleton instance of the "lower target temperature" timer */
Timer_Handle getLowerTargetTempTimer() {

    if(lower_target_temp_timer == NULL)
        lower_target_temp_timer = createLowerTargetTempTimer();

    return lower_target_temp_timer;
}

/* Get a singleton instance of the "raise target temperature" timer */
Timer_Handle getRaiseTargetTempTimer() {

    if(raise_target_temp_timer == NULL)
        raise_target_temp_timer = createRaiseTargetTempTimer();

    return raise_target_temp_timer;
}

enum ButtonStates { BUTTON_DOWN, BUTTON_UP };

/* State machine for lower target temperature */
int lowerTempButtonReponse(int buttonState) {
    // Transitions
    switch(buttonState) {
        case BUTTON_DOWN:
            if(!lowerTempButtonPressed)
                buttonState = BUTTON_UP;
            break;
        case BUTTON_UP:
            if(lowerTempButtonPressed)
                buttonState = BUTTON_DOWN;
            break;
        default:
            break;
    }

    Timer_Handle lower_target_temp_timer = getLowerTargetTempTimer();

    // Actions
    switch(buttonState) {
        case BUTTON_DOWN:
            Timer_start(lower_target_temp_timer);
            break;
        case BUTTON_UP:
            Timer_stop(lower_target_temp_timer);
            break;
        default:
            break;
     }

    return buttonState;
}

/* State machine for raise target temperature */
int raiseTempButtonReponse(int buttonState) {
    // Transitions
    switch(buttonState) {
        case BUTTON_DOWN:
            if(!raiseTempButtonPressed)
                buttonState = BUTTON_UP;
            break;
        case BUTTON_UP:
            if(raiseTempButtonPressed)
                buttonState = BUTTON_DOWN;
            break;
        default:
            break;
    }

    Timer_Handle raise_target_temp_timer = getRaiseTargetTempTimer();

    // Actions
    switch(buttonState) {
        case BUTTON_DOWN:
            Timer_start(raise_target_temp_timer);
            break;
        case BUTTON_UP:
            Timer_stop(raise_target_temp_timer);
            break;
        default:
            break;
     }

    return buttonState;
}

/* Task periods in microseconds */
const unsigned long periodCheckLowerTempButton_U = 200000;
const unsigned long periodCheckRaiseTempButton_U = 200000;
const unsigned long periodCheckTemperature_U = 500000;
const unsigned long periodUpdateHeatLED_U = 1000000;
const unsigned long periodUpdateCoolLED_U = 1000000;
const unsigned long periodReportState_U = 1000000;

/*
 * Checking buttons state and temperature, updating LED, and
 * reporting state are "tasks" handled by task scheduler
 */
typedef struct Task {

   int state;                  // current state (enum)
   unsigned long period;       // time between task executions
   unsigned long elapsedTime;  // time elapsed since last task
   int (*TickFct)(int);        // pointer to task function

} Task;

Task tasks[6];

const unsigned int tasksNum = 6;

/* Setup tasks handled by task scheduler */
void initTasks() {
    unsigned int i = 0;

    tasks[i].state = BUTTON_UP;
    tasks[i].period = periodCheckLowerTempButton_U;
    tasks[i].elapsedTime = 0;
    tasks[i].TickFct = &lowerTempButtonReponse;

    i++;
    tasks[i].state = BUTTON_UP;
    tasks[i].period = periodCheckRaiseTempButton_U;
    tasks[i].elapsedTime = 0;
    tasks[i].TickFct = &raiseTempButtonReponse;

    i++;
    tasks[i].state = THERM_OFF;
    tasks[i].period = periodCheckTemperature_U;
    tasks[i].elapsedTime = 0;
    tasks[i].TickFct = &temperatureResponse;

    i++;
    tasks[i].state = HEAT_LED_OFF;
    tasks[i].period = periodUpdateHeatLED_U;
    tasks[i].elapsedTime = 0;
    tasks[i].TickFct = &heatLEDResponse;

    i++;
    tasks[i].state = COOL_LED_OFF;
    tasks[i].period = periodUpdateCoolLED_U;
    tasks[i].elapsedTime = 0;
    tasks[i].TickFct = &coolLEDResponse;

    i++;
    tasks[i].state = REPORT_STATE;
    tasks[i].period = periodReportState_U;
    tasks[i].elapsedTime = 0;
    tasks[i].TickFct = &reportResponse;
}

/*
 * runThermostat - Thermostat Task Scheduler
 *
 * Monitor input and temperature, print current state,
 * and activate/deactivate heating/cooling as needed
 */
void runThermostat() {
    // Run thermostat forever
    while (1) {
        // Lower flag raised by timer and wait for timer period
        TimerFlag = 0;
        while (!TimerFlag) { }
        // For each task, call task tick function if task's period is up
        unsigned int i = 0;
        for (i=0; i < tasksNum; i++) {
            // Check if task is ready to execute. If so, execute and reset
           if (tasks[i].elapsedTime >= tasks[i].period) {
              tasks[i].state = tasks[i].TickFct(tasks[i].state);
              tasks[i].elapsedTime = 0;
           }
           // If task is not ready to execute, update elapsed time
           tasks[i].elapsedTime += tasksPeriodGCD_U;
        }
    }
}

/* Configure drivers/devices and run thermostat */
void *mainThread(void *arg0) {
    // Initialize general purpose input/output driver
    initGPIO();
    // Initialize asynchronous receiver/transmitter
    initUART();
    // Initialize interaction with temperature sensor
    initI2C();
    // Initialize task scheduler timer
    initTimer();
    // Initialize thermostat tasks
    initTasks();
    // Run thermostat (forever)
    runThermostat();
    // Bail if thermostat fails
    return (NULL);
}
