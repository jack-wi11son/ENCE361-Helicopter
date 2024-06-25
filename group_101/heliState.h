/*
 * heliState.h
 *
 *  Created on: 12/05/2024
 *      Author: jwi182, hrc48
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "pwmRotor.h"
#include "buttons4.h"
#include "quadrature.h"

// Define states for the helicopter
typedef enum {
    LANDED,
    TAKING_OFF,
    FLYING,
    LANDING
} HelicopterState;


#ifndef HELISTATE_H_
#define HELISTATE_H_

void initialiseSwitch (void);

void initialiseResetButton (void);

void initialiseYawRef (void);

void readResetButtonState(void);

bool readSwitchState(void);

void yawRefHandler (void);

void poleButtons(void);

HelicopterState updateHelicopterState(int32_t currentYaw, uint16_t currentAlt);

char* getHeliState (void);

bool landingComplete(int32_t yaw, uint16_t altitude);

bool takeoffComplete (int32_t yaw, uint16_t altitude);

#endif /* HELISTATE_H_ */
