/*
 * display.h
 *
 *  Created on: 21/03/2024
 *      Author: hrc48
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
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"


//Global constants
#define ADC_STEP_FOR_1V 1240
#define YAW_STEPS 448
#define DEG_REV 360
#define SCALE_BY_100 100

//Display mode enum
enum DisplayMode { PROCESSED, RAW, DISPLAY_OFF, CYCLE_BACK};

#ifndef DISPLAY_H_
#define DISPLAY_H_



void displayWrite(uint16_t baseAlt, uint16_t currentAlt, int32_t currentYaw, enum DisplayMode displayCycle);

int32_t getAltPercent (uint16_t baseAltitude, int32_t altitude);

int32_t getYawDegree(int32_t currentYaw);

void initDisplay (void);


#endif /* DISPLAY_H_ */
