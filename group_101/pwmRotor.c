/*
 * pwmRotor.c
 *
 *  Created on: 7/05/2024
 *      Author: hrc48, jwi182
 */

#include <pwmRotor.h>

//Heli controller set points
static int16_t altSetPoint = 0;
static int16_t yawSetPoint = -223;

//Altitude ADC Max and Min 
static uint16_t max_alt = 0; 
static uint16_t min_alt = 0;



/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 * M1PWM5 (J3, PF1) is used for the tail rotor motor
 *********************************************************/
void
initialisePWM (void)
{
    //System clock
    uint32_t sysClock = SysCtlClockGet();

    //init main
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    uint32_t ui32Period = sysClock / PWM_DIVIDER / PWM_MAIN_FREQ;

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, 0);

    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);


    //init Tail
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32PeriodTail = sysClock / PWM_DIVIDER / PWM_TAIL_FREQ;

    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32PeriodTail);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, 0);


    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}


//Initialise min and max ADC heights
void initAltLimits (uint16_t initLandedADC) {
    min_alt = initLandedADC;
    max_alt = initLandedADC - ADC_STEP_FOR_1V;

    altSetPoint = initLandedADC;
}



//Function to set the Main and Tail duty cycles
void
setDuty (uint32_t mainDuty, uint32_t tailDuty)
{
    //Get System clock value once, no point running function all the time
    static uint32_t sysClock = 0;
    if (sysClock == 0) {
        sysClock = SysCtlClockGet();
    }

    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32PeriodMain = sysClock / PWM_DIVIDER / PWM_MAIN_FREQ;

    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, ui32PeriodMain * mainDuty / 100);

    uint32_t ui32PeriodTail = sysClock / PWM_DIVIDER / PWM_TAIL_FREQ;

    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM, ui32PeriodTail * tailDuty / 100);
}


//PID controller function for main rotor, returns a duty cycle %
int32_t
controllerMain (uint16_t sensor) {
    //Record integral and prev sensor value
    static float dI = 0;
    static uint16_t prevSensor = 0;

    float error = altSetPoint - sensor;
    float P = KPM * error;
    float I = KIM * error * DELTA_T;
    float D = KDM * (prevSensor - sensor) / DELTA_T;

    //Gravity is constant offset, controller is negative as ADC is opposite to height
    int32_t control = GRAVITY - P - (dI + I) - D;

    //Cap controller output
    if (control > PWM_DUTY_MAIN_MAX) {
        control = PWM_DUTY_MAIN_MAX;
    } else if (control < PWM_DUTY_MAIN_MIN) {
        control = PWM_DUTY_MAIN_MIN;
    } else {
        //Accumulate integral
        I = I + dI;
    }
    prevSensor = sensor;
    return control;
}


//PID controller function for tail rotor, returns a duty cycle %
int32_t
controllerTail (int32_t mainControl, int16_t sensor, bool sweepEn) {
    int16_t error = yawSetPoint - sensor;
    
    //Turn off controller wrap around at 180 deg when in take off sweeping mode so sweep can complete 360 turn
    if (!sweepEn) {
        //Adjust error values when crossing 180 deg mark
        if (error < -YAW_ERROR_LIMIT) {
            error = YAW_REV + error;
        } else if (error > YAW_ERROR_LIMIT) {
            error = -YAW_REV + error;
        }
    }
    
    //Record integral and prev sensor value
    static float dI = 0;
    static int16_t prevSensor = 0;

    //PID controller calc
    float P = KPT * error;
    float I = KIT * error * DELTA_T;
    float D = KDT * (prevSensor - sensor) / DELTA_T;
    float PID_TAIL = P + I + D;

    //Limit PID effort without limiting coupling
    if (PID_TAIL > PID_TAIL_MAX) {
        PID_TAIL = PID_TAIL_MAX;
    }

    //Couple tail rotor to main rotor duty
    float coupling = mainControl * KC;

    int32_t control = PID_TAIL + coupling;

    //Cap controller output
    if (control > PWM_DUTY_TAIL_MAX) {
        control = PWM_DUTY_TAIL_MAX;
    } else if (control < PWM_DUTY_TAIL_MIN) {
        control = PWM_DUTY_TAIL_MIN;
    } else {
        I = I + dI;
    }

    prevSensor = sensor;
    return control;
}


//Increase altitude setpoint
void incAlt (void) {
    altSetPoint -= ALT_STEP;
    if (altSetPoint < max_alt){
        altSetPoint = max_alt;
    }
}


//Decrease altitude setpoint
void decAlt (void) {
    altSetPoint += ALT_STEP;
    if (altSetPoint > min_alt - ALT_STEP){
        altSetPoint = min_alt - ALT_STEP; //Set min button altitude to 10%
    }
}


//Set altitude setpoint
void setAlt (int16_t setPoint) {
    altSetPoint = setPoint;
}

//Increase yaw setpoint
void incYaw (void) {
    yawSetPoint += YAW_STEP;
    //special case - wrap around at 180 deg
    if (yawSetPoint > YAW_ERROR_LIMIT) {
        yawSetPoint = -YAW_ERROR_LIMIT + (yawSetPoint - YAW_ERROR_LIMIT);
    }
}


//Decrease yaw setpoint
void decYaw (void) {
    yawSetPoint -= YAW_STEP;
    //special case - wrap around at 180 deg
    if (yawSetPoint < -YAW_ERROR_LIMIT) {
        yawSetPoint = YAW_ERROR_LIMIT + (yawSetPoint + YAW_ERROR_LIMIT);
    }
}


//Set yaw setpoint
void setYaw (int16_t setPoint) {
    yawSetPoint = setPoint;
}

//Get altitude setpoint
int32_t getAltSet (void) {
    return altSetPoint;
}

//Get yaw setpoint
int32_t getYawSet (void) {
    return yawSetPoint;
}

//Get min alt setpoint
uint16_t getmin_alt (void) {
    return min_alt;
}

//Get max alt setpoint
uint16_t getmax_alt (void) {
    return max_alt;

}





void 
PWM_ON (void) {
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);
}

void 
PWM_OFF (void) {
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}
