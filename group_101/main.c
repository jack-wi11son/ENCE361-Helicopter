/*
 * main.c
 *
 *  Created on: 21/03/2024
 *      Authors: jwi182, hrc48
 */


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"
#include "display.h"
#include "ADC.h"
#include "quadrature.h"
#include "pwmRotor.h"
#include "uart.h"
#include "heliState.h"


//Task flags
static volatile bool flagController = false;
static volatile bool flagButtons = false;
static volatile bool flagDisplay = false;
static volatile bool flagUART = false;

//********************************************************
// Global variables
//********************************************************

char statusStr[MAX_STR_LEN + 1];

#define CONTROL_PERIOD 4    //Corrosponds to 250Hz
#define BUTTON_PERIOD 10    //Corrosponds to 100Hz
#define DISPLAY_PERIOD 15   //Corrosponds to 66.67Hz
#define UART_PERIOD 200     //Corrosponds to 5Hz
#define START_DELAY 5       //200 ms delay



//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}


//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    static uint8_t controllerCounter = 0;
    static uint8_t buttonsCounter = 0;
    static uint8_t displayCounter = 0;
    static uint8_t uartCounter = 0;

    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);


    //Set task flags based on desired task frequency
    if (controllerCounter >= CONTROL_PERIOD) {
        flagController = true;
        controllerCounter = 0;
    }

    if (buttonsCounter >= BUTTON_PERIOD) {
        flagButtons = true;
        buttonsCounter = 0;
    }

    if (displayCounter >= DISPLAY_PERIOD) {
        flagDisplay = true;
        displayCounter = 0;
    }

    if (uartCounter >= UART_PERIOD) {
        flagUART = true;
        uartCounter = 0;
    }

    //Increment task timers
    controllerCounter++;
    buttonsCounter++;
    displayCounter++;
    uartCounter++;
}


int
main(void)
{
    uint16_t currentAlt;
    int32_t currentYaw;
    uint16_t initLandedADC;
    int32_t mainDuty;
    int32_t tailDuty;
    enum DisplayMode displayCycle = PROCESSED; //Display altitude percentage and yaw degrees
    HelicopterState heliState = LANDED;
    bool sweepEn = 0;   //Sweep yaw to find yaw ref point


    //Initialise all functions
    initClock ();
    initButtons();
    initADC ();
    initDisplay ();
    initQuad();
    initialisePWM();
    initialiseUSB_UART();
    initialiseSwitch();
    initialiseResetButton();
    initialiseYawRef();


    // Enable interrupts to the processor.
    IntMasterEnable();

    SysCtlDelay (SysCtlClockGet() / START_DELAY);  // ~200ms delay for buffer to populate

    
    // Grab initial altitude ADC input to use for percentage converting
    initLandedADC = getAltMean();

    //Set inital Max and Min altitudes 
    initAltLimits(initLandedADC);

    while (1)
    {
        //Pole heli soft reset button
        readResetButtonState();

        //Run main tasks
        //Run PID controller and set PWM levels
        if (flagController) {
            //Update current sensor values
            currentAlt = getAltMean();
            currentYaw = getYawPosition();

            mainDuty = controllerMain(currentAlt);
            tailDuty = controllerTail(mainDuty, currentYaw, sweepEn);

            setDuty(mainDuty, tailDuty);

            flagController = false;
        }

        //Pole buttons and state switch and update heli state
        if (flagButtons) {

            heliState = updateHelicopterState(currentYaw, currentAlt);
            if (heliState == TAKING_OFF) {
                sweepEn = true;
            } else {
                sweepEn = false;
            }
            flagButtons = false;
        }

        //Refresh BoosterPack OLED display
        if (flagDisplay) {
            displayWrite(initLandedADC, currentAlt, currentYaw, displayCycle);
            flagDisplay = false;
        }

        //Send new Heli stats to PC via UART
        if (flagUART) {
            
            //Convert raw values to nice values for UART
            int32_t actualAlt = getAltPercent(initLandedADC, currentAlt);
            int32_t desireAlt = getAltPercent(initLandedADC, getAltSet());

            int32_t actualYaw = getYawDegree(currentYaw) / SCALE_BY_100;
            int32_t desireYaw = getYawDegree(getYawSet()) / SCALE_BY_100;

            //Grab current Heli state as a string
            char *heliString = getHeliState();

            //Update UART string
            usprintf (statusStr, "Alt(Actual/Set) %d/%d  \r\n", actualAlt, desireAlt);
            UARTSend (statusStr);

            usprintf (statusStr, "Yaw(Actual/Set) %d/%d  \r\n", actualYaw, desireYaw);
            UARTSend (statusStr);

            usprintf (statusStr, "Main %% %d | Tail %% %d | Mode %s \r\n", mainDuty, tailDuty, heliString);
            UARTSend (statusStr);


            flagUART = false;
        }

    }
}
