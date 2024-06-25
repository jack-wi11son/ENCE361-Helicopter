/*
 * pwmRotor.h
 *
 *  Created on: 7/05/2024
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

//ALT and YAW
#define ADC_STEP_FOR_1V 1240
#define ALT_STEP 124    // 1240/10  For 10% of 1V step
#define ALT_LAND 24     // 2% of Max height
#define ALT_TAKEOFF_5_PERCENT 62 //5% of max a
#define YAW_STEP 19  //448 *15/360 degrees rounded
#define YAW_LIMIT 6     // ~5 degrees
#define YAW_ERROR_LIMIT 224
#define YAW_REV 448
#define GRAVITY 31
#define KC 0.8



//Delta time HZ 250Hz
#define DELTA_T 0.004 //seconds

// PID config
//MAIN ROTOR
#define KPM 0.06 //Real rig
//#define KPM 1.5
#define KIM 0.08
#define KDM 0.0001

//TAIL ROTOR
#define KPT 1.2 //Real rig
//#define KPT 5
#define KIT 0.01
#define KDT 0


// PWM configuration
#define PWM_START_RATE_HZ  250

#define PWM_DUTY_MAIN_MIN   15
#define PWM_DUTY_MAIN_MAX    80
#define PWM_DUTY_TAIL_MIN    5
#define PWM_DUTY_TAIL_MAX    85
#define PID_TAIL_MAX 25


#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_1
#define PWM_DIVIDER        1

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5
#define PWM_MAIN_FREQ        300


//  PWM Hardware Details M1PWM5 (gen 2)
//  ---Tail Rotor PWM: PF1, J3
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1
#define PWM_TAIL_FREQ        300



#ifndef PWMROTOR_H_
#define PWMROTOR_H_

void
initialisePWM (void);

void initAltLimits (uint16_t initLandedADC);

void
setDuty (uint32_t mainDuty, uint32_t tailDuty);

int32_t
controllerMain (uint16_t sensor);

int32_t
controllerTail (int32_t mainControl, int16_t sensor, bool sweepEn);

void incAlt (void);

void decAlt (void);

void setAlt (int16_t setPoint);

void incYaw (void);

void decYaw (void);

void setYaw (int16_t setPoint);

int32_t getAltSet (void);

int32_t getYawSet (void);

uint16_t getmin_alt (void);

uint16_t getmax_alt (void);

void PWM_ON (void);

void PWM_OFF (void);

#endif /* PWMOTOR_H_ */

