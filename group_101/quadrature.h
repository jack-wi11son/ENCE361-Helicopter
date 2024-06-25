/*
 * quadrature.h
 *
 *  Created on: 21/03/2024
 *      Author: jwi182, hrc48
 */

#include <stdint.h>
#include <stdbool.h>
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

#define WRAPSTEP 224 //Number of quadrature steps  before degrees wrap around at +180 and -180 (448/2)
#define INITIAL_YAW_POSITION -223

#ifndef QUADRATURE_H_
#define QUADRATURE_H_


void initQuad (void);

void setYawZero (void);

int32_t getYawPosition (void);

void GPIOYawHandler (void);



#endif /* DISPLAY_H_ */
