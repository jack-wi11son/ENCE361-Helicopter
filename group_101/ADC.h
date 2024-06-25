/*
 * ADC.h
 *
 *  Created on: 21/03/2024
 *      Author: jwi182, hrc48
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "circBufT.h"


//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 60
#define SAMPLE_RATE_HZ 1000

// ADC configuration
#define ADC_SEQUENCE_NUM         3    // ADC sequence number
#define ADC_SEQUENCE_STEP        0    // Step index for ADC sequence



#ifndef ADC_H_
#define ADC_H_



void ADCIntHandler(void);

//*****************************************************************************
// initADC: The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//*****************************************************************************
void initADC (void);

// ************************************************************
// getAltMean: Calculates the mean altitude from a circular buffer.
// Assumes g_inBuffer is initialized and BUF_SIZE is defined.
uint16_t getAltMean (void);

void SysTickIntHandler(void);

#endif /* ADC_H_ */
