/*
 * quadrature.c
 *
 *  Created on: 21/03/2024
 *      Author: jwi182, hrc48
 */

#include "quadrature.h"

static volatile int32_t yawPosition = INITIAL_YAW_POSITION;



// *******************************************************
// initQuad: Initialise the hardware and interrupt settings for a quadcopter's yaw control.
// This function configures GPIO ports and pins and registers interrupt handlers
// specific to the yaw movement controls of the quadcopter.
void initQuad(void)
{
    // Enable the peripheral clock for GPIO port B, which is used for yaw control.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Register the interrupt handler function 'GPIOYawHandler' for GPIO port B.
    GPIOIntRegister(GPIO_PORTB_BASE, GPIOYawHandler);

    // Configure pins 0 and 1 on GPIO port B as input pins, to be used for reading yaw control signals.
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Set the interrupt type for pins 0 and 1 on GPIO port B to trigger on both rising and falling edges,
    // which allows detection of all changes in the yaw control signal.
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES);

    // Enable interrupts on pins 0 and 1 on GPIO port B, allowing the system to respond to yaw control signals.
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}


void setYawZero (void) {
    yawPosition = 0;
}

int32_t getYawPosition (void)
{
    return yawPosition;
}



// Interrupt handler for GPIO Port B Pins 0 and 1
void GPIOYawHandler(void)
{
    // Variables to keep track of the current and last state of the encoder
    static uint8_t last_state = 0;
    uint8_t state;

    // Read and clear the interrupt status for GPIO Port B
    uint32_t status = GPIOIntStatus(GPIO_PORTB_BASE, true);
    GPIOIntClear(GPIO_PORTB_BASE, status);

    // Read the current state of the pins connected to the encoder (PB0 and PB1)
    state = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Return immediately if there is no change in state
    if (state == last_state) return;

    // Update the yaw position based on the transition between states
    // These transitions are determined based on the expected behavior of a quadrature encoder
    if (((last_state == 0x00) && (state == 0x01)) ||  // Transition from 00 to 01
        ((last_state == 0x01) && (state == 0x03)) ||  // Transition from 01 to 11
        ((last_state == 0x03) && (state == 0x02)) ||  // Transition from 11 to 10
        ((last_state == 0x02) && (state == 0x00))) {  // Transition from 10 to 00
        // Counter-clockwise rotation: decrement yaw position
        yawPosition--;
    } else {
        // Clockwise rotation: Increment yaw position
        yawPosition++;
    }


    // Handle wrap around at 180 degrees
    if (yawPosition > WRAPSTEP) {
        yawPosition = -WRAPSTEP + (yawPosition - WRAPSTEP);
    }
    if (yawPosition < -WRAPSTEP) {
        yawPosition = WRAPSTEP + (yawPosition + WRAPSTEP);
    }

    // Update last_state to the current state for the next interrupt
    last_state = state;

}

