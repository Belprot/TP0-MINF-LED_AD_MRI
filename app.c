/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "Mc32DriverLcd.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
S_ADCResults AdcRes;
BSP_LED arrLEDs[8] = {  PORTS_BIT_POS_0,
                        PORTS_BIT_POS_1,
                        PORTS_BIT_POS_4,
                        PORTS_BIT_POS_5,
                        PORTS_BIT_POS_6,
                        PORTS_BIT_POS_7,
                        PORTS_BIT_POS_15,
                        PORTS_BIT_POS_10};

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
}

void APP_UpdateState(APP_STATES NewState){
    
    appData.state = NewState;
}

void APP_LEDsState(bool state){
    
    int i;
    if(state){ // Turn ON the 8 LEDs
        for (i = 0; i < 8; i++) {
            
            BSP_LEDOn(arrLEDs[i]);
        }
    }
    else{ // Turn OFF the 8 LEDs
        for (i = 0; i < 8; i++) {
            
            BSP_LEDOff(arrLEDs[i]);
        }
    }
}

void APP_UpdateChennillard(){
    
    static uint8_t led = 0;
    
    // Turn ON the LED corresponding to the counter
    BSP_LEDOn(arrLEDs[led]);
    // Turn OFF the LED corresponding to the previous iteration
    if(led == 0) BSP_LEDOff(arrLEDs[7]);
    else BSP_LEDOff(arrLEDs[led-1]);
    
    led += 1;
    if(led >= 8){
        led = 0;
    }
}

void APP_Tasks ( void )
{
    static bool appRunOneTime = false;
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        //====================================================================// APP_STATE_INIT
        case APP_STATE_INIT:
        {
            // Init LCD
            lcd_init();
            lcd_bl_on();
            lcd_gotoxy(1, 1);
            printf_lcd("TP0 LED+AD 2022-2Y");
            lcd_gotoxy(1, 2);
            printf_lcd("M.Ricchieri");
            // Init ADC
            BSP_InitADC10();
            // Start Timer0
            DRV_TMR0_Start();
            // Turn ON all LEDs
            APP_LEDsState(ON);
            // Update state machine
            appData.state = APP_STATE_WAIT;
            break;
        }
        //====================================================================// APP_STATE_WAIT
        case APP_STATE_WAIT:
        {
            // Nothing there, the wind blows
            break;
        }
        //====================================================================// APP_STATE_SERVICE_TASKS
        case APP_STATE_SERVICE_TASKS:
        {
            // Read both adc channels (CH0, CH1))
            AdcRes = BSP_ReadAllADC();
            // Print ADC values on the third line of the LCD
            lcd_gotoxy(1, 3);
            printf_lcd("CH0 %04d CH1 %04d", AdcRes.Chan0, AdcRes.Chan1);
            // Turn OFF all LEDs, ONLY the first time
            if(!appRunOneTime){
                appRunOneTime = true;
                APP_LEDsState(OFF);
            }
            // Call chenillard function
            APP_UpdateChennillard();
            // Update state machine
            APP_UpdateState(APP_STATE_WAIT);
            break;
        }

        /* The default state should never be executed. */
        //====================================================================// default
        default:
        {
            break;
        }
    }
}

/*******************************************************************************
 End of File
 */
