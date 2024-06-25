/*
 * display.c
 *
 *  Created on: 21/03/2024
 *      Author: hrc48, jwi182
 */

#include "display.h"


// *******************************************************
// displayWrite: Updates the OLED display with altitude and yaw information based on the current display mode.
// This function handles the formatting and output of altitude and yaw data onto an OLED display.
// The display mode can be either PROCESSED, RAW, or DISPLAY_OFF, which affects what and how information is displayed.
void 
displayWrite(uint16_t baseAlt, uint16_t currentAlt, int32_t currentYaw, enum DisplayMode displayCycle) {

    // Get Altitude as a percentage
    int32_t altPercentage = getAltPercent(baseAlt, currentAlt);

    // Get yaw as a degree
    int32_t yawDegree = getYawDegree(currentYaw);

    //Create string instance
    char lineString[17]; // 16 characters across the display

    switch (displayCycle) {
        case PROCESSED:
        {
            //Special case when 0/100=0 and not -0 when Deg = -0.8
            bool negZero = false;

            //Title
            OLEDStringDraw ("helicopter Stats", 0, 0);

            // Display ADC input as a height percentage
            usnprintf(lineString, sizeof(lineString), "Altitude: %3d%% ", altPercentage);
            OLEDStringDraw (lineString, 0, 1);

            // Calculate Yaw decimal point using modulo
            int32_t yawInt = yawDegree / SCALE_BY_100;
            int32_t yawDecimal = yawDegree % SCALE_BY_100;

            // Prevent the decimal portion from displaying as negative
            if (yawDecimal < 0) {
                yawDecimal *= -1;
                negZero = true;
            }

            if (negZero && yawInt == 0) {
                //Display a -ve zero as integer -ve 0 does not exist
                usnprintf(lineString, sizeof(lineString), "Yaw(deg):-0.%d   ", yawDecimal);
            } else {
                //Display yaw in degrees to 2dp
                usnprintf(lineString, sizeof(lineString), "Yaw(deg):%d.%d   ", yawInt, yawDecimal);
            }
            //Clear display line
            OLEDStringDraw (lineString, 0, 2);

            negZero = false;
            break;
        }
        case RAW:
        {
            OLEDStringDraw ("Helicopter Stats", 0, 0);

            // Display the mean ADC value
            usnprintf(lineString, sizeof(lineString), "Mean ADC: %4d ", currentAlt);
            OLEDStringDraw (lineString, 0, 1);
            //Clear Yaw line
            OLEDStringDraw ("                ", 0, 2);

            break;
        }
        case DISPLAY_OFF:
        {   
            //Blank display
            OrbitOledClear();
            break;
        }
        default:
            break;

    }

}

// *******************************************************
//getAltPercent: Convert raw altitude ADC value to a percentage of maximum altitude based
// off 1 volt between MIN and MAX
int32_t 
getAltPercent (uint16_t baseAltitude, int32_t altitude)
{
    // Calculate the altitude as a percentage (integer math)
    int32_t delta = baseAltitude - altitude; // Difference from the baseline
    int32_t altPercentage = (delta * SCALE_BY_100) / ADC_STEP_FOR_1V; //Scale by 100 for percentage

    return altPercentage;
}

// *******************************************************
// getYawDegree: Scale by 100 before division to include 2 decimal point for modulo division
int32_t 
getYawDegree(int32_t currentYaw)
{   
    return (currentYaw * DEG_REV * SCALE_BY_100) / YAW_STEPS; 
}


void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}



