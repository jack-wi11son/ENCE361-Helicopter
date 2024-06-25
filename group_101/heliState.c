/*
 * heliState.c
 *
 *  Created on: 12/05/2024
 *      Author: jwi182, hrc48
 */

#include "heliState.h"

// Initialize state
static HelicopterState heliState = LANDED; //Helicopters state instance
static bool landedLock = true;  //Start locked in landed mode until Switch1 pulled LOW
bool scanFlag = true;           //Start in yaw scanning mode to locate physical yaw reference point

// Corresponding array of strings for helicopter state enum
static char 
*HELISTATE_STRING[] = {
    "LANDED",
    "TAKING OFF",
    "FLYING",
    "LANDING"
};

//Initialise SW1 to read input for changing heli state
void 
initialiseSwitch (void) {
    // Enable Peripheral for GPIO Port A (for PA7)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the GPIO pin for the mode slider switch (PA7)
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

}

//Initialise soft reset pin on BoosterPack SW2 PIN PA6
void 
initialiseResetButton (void) {
    // Configure the GPIO pin for the virtual reset button
    // Set the direction of the pin as input and enable the pull-down resistor.
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
}


//Initialise physical yaw reference input pin as GPIO interupt PIN PC4
void 
initialiseYawRef (void) {

    GPIOIntRegister(GPIO_PORTC_BASE, yawRefHandler);

    GPIOPinTypeGPIOInput (GPIO_PORTC_BASE, GPIO_PIN_4);

    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_LOW_LEVEL);

    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);
}


bool 
readSwitchState(void) {
    // Read the current state of SW1 switch for Heli state
    return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7);
}

//Pole soft reset pin on SW2 on PA6
void 
readResetButtonState(void) {
    if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0) {
        SysCtlReset();
    }
}

//Yaw reference interupt handler function, set current yaw and yaw setpoint to 0
void 
yawRefHandler (void) {
    //Scan flag on by default
    if (scanFlag) {
        setYawZero();
        setYaw(0);
        scanFlag = false;
    }

    //Clear and disable interupt so it only occurs once on startup
    uint32_t status = GPIOIntStatus(GPIO_PORTC_BASE, true);
    GPIOIntClear(GPIO_PORTC_BASE, status);
    GPIOIntDisable(GPIO_PORTC_BASE, GPIO_PIN_4);
}



//Pole UP DN LF RT buttons for heli control
void
poleButtons(void) {

    updateButtons();

    // Left button decreases Yaw
    if (checkButton(LEFT) == PUSHED) {
        decYaw();
    }

    // Right button decreases Yaw
    if (checkButton(RIGHT) == PUSHED) {
        incYaw();
    }

    // Right button decreases Yaw
    if (checkButton(UP) == PUSHED) {
        incAlt();
    }

    // Right button decreases Yaw
    if (checkButton(DOWN) == PUSHED) {
        decAlt();
    }
}

/********************************************************
 * Function to set the Helicopter states
 ********************************************************/
HelicopterState 
updateHelicopterState(int32_t currentYaw, uint16_t currentAlt) {
    bool sw1High = readSwitchState();

    switch (heliState) {
        case LANDED:
            //Unlock when switch is low
            if (!sw1High) {
                landedLock = false;
            }
            //Keep heli off when in LANDED
            if (!landedLock && sw1High) {
                heliState = TAKING_OFF;
            } else {
                PWM_OFF();
            }

            break;

        case TAKING_OFF:
            // Activate motors to take off
            // Transition to FLYING after successful takeoff
            PWM_ON();
            if (takeoffComplete(currentYaw, currentAlt)) {
                heliState = FLYING;
            }
            break;

        case FLYING:
            //Allow full user heli control after take off
            poleButtons();
            if (!sw1High) {
                heliState = LANDING;
            }
            break;

        case LANDING:
            // Set yaw to 0 to land on yaw ref point
            setYaw(0);
            // Remain in LANDING until landing is complete
            if (landingComplete(currentYaw, currentAlt)) {
                heliState = LANDED;
            }
            break;
    }
    return heliState;
}

//Auto landing function, yaw set to zero then wait for altitude to decrease to 0-2% then turn off PWM
bool 
landingComplete(int32_t yaw, uint16_t altitude) {
    // Logic to check if landing is complete
    uint16_t minAlt = getmin_alt();

    //Wait for yaw to go to zero
    if (yaw >= -YAW_LIMIT && yaw < YAW_LIMIT) {
        setAlt(minAlt); //Set altitude to 0%
        //Wait for altitude to go <2%
        if (altitude <= minAlt && altitude >= minAlt - ALT_LAND) {
            return true;
        }
        else {
            return false;
        }

    }
    else {
        return false;
    }
}


//Auto take off to 5% then rotate clockwise a full rev to find yaw reference point
bool 
takeoffComplete (int32_t yaw, uint16_t altitude) {
    uint16_t minAlt = getmin_alt();

    setAlt(minAlt - ALT_TAKEOFF_5_PERCENT);
    //Rise to >3% 
    if (altitude < minAlt - ALT_TAKEOFF_5_PERCENT + ALT_LAND) {
        //scanFlag changed by yaw ref point interupt
        //Wait to pass over yaw ref point
        if (!scanFlag) {
            return true;
        }
        else {
            //Do full revolution
            setYaw(223);
            return false;
        }

    } else {
        return false;
    }


}

//Return heliState as a string for printing
char* 
getHeliState (void) {
    return HELISTATE_STRING[heliState];
}
