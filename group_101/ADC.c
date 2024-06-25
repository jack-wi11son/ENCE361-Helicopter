/*
 * ADC.c
 *
 *  Created on: 21/03/2024
 *      Author: jwi182, hrc48
 */

#include "ADC.h"

static circBuf_t g_inBuffer;        // Create buffer instance


//*****************************************************************************
//
// initADC: The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
//*****************************************************************************

//Initialise ADC and GPIO
//Function written by UCECE
void
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE_NUM, ADC_TRIGGER_PROCESSOR, ADC_SEQUENCE_STEP);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE_NUM, ADC_SEQUENCE_STEP, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE_NUM);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, ADC_SEQUENCE_NUM, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, ADC_SEQUENCE_NUM);

    //Initialise the circular buffer
    initCircBuf (&g_inBuffer, BUF_SIZE);
}


// ************************************************************
// ADCIntHandler: Interrupt handler for ADC conversion completion on the Tiva
// processor. Retrieves the ADC value from a completed conversion,
// stores it in a circular buffer, and clears the ADC interrupt.
//Function written by UCECE
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQUENCE_NUM, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf(&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE_NUM);
}

// ************************************************************
// getAltMean: Calculates the mean altitude from a circular buffer.
// Assumes g_inBuffer is initialized and BUF_SIZE is defined.
uint16_t 
getAltMean (void) {
    uint16_t i;
    uint32_t sum = 0;
    for (i = 0; i < BUF_SIZE; i++) {
        sum += readCircBuf(&g_inBuffer);
    }
    uint16_t currentMean = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
    return currentMean;
}








