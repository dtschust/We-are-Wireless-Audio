// Title: IRQ
// Version: 1.0
// Filename: IRQ.c
// Author: Drew Schuster  (loosely based on MCB2300 development tools provided by Keil)
// Purpose/Function of program: Handles all interrupt routines needed by WaWA
// How Program is Run on Target System: Loaded using JTAG interface, called via interrupt after
     // Initialization by WaWA.c
// Date Started: March 14th
// Update History:  See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio

#define BASE
//#define DEVBOARD
//#define REMOTE
#include <LPC23xx.H>                    /* LPC23xx definitions                */
#include <stdio.h>


short AD_last;                          /* Last converted value               */
int I2SRX=4;
unsigned char clock_1s;                 /* Flag activated each second         */
int currentButton;
int voiceCode;




// Function Name: TO_IRQHandler()
// Author: Drew Schuster
// Called by: timer 0 interrupt
// Purpose: Starts A/D Conversion
// Calling convention:  Not called directly, called via interrupt
// Conditions at exit: A/D Conversion started, interrupt acknowledged
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
/* Timer0 IRQ: Executed periodically                                          */
__irq void T0_IRQHandler (void) {

  AD0CR |= 0x01000000;                  /* Start A/D Conversion               */
 
  T0IR        = 1;                      /* Clear interrupt flag               */
  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
}


// Function Name: TO_IRQHandler2()
// Author: Drew Schuster
// Called by: timer 1 interrupt
// Purpose: Handles user input, seconds flag
// Calling convention:  Not called directly, called via interrupt
// Conditions at exit: User input updated, seconds flag updated
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
__irq void T0_IRQHandler2 (void) {	 /* Timer interrupt to interface with display*/
  static int clk_cntr;
  	int buttons;
  
  if (clk_cntr%100==0)	 /*Update push buttons 10 times a second*/
  {
#ifdef DEVBOARD
   currentButton = FIO3PIN;
#else
   currentButton = (FIO1PIN >>15) & 0x7; //P1[15-17]
#endif
   currentButton +=0;
  }

  if (clk_cntr%1000==0)
  {
#ifdef DEVBOARD
   buttons = (FIO2PIN>>5) & 0x0F;
#else
   buttons = (FIO1PIN>>20)& 0x1F;
#endif
   voiceCode = buttons;
  }

  clk_cntr++;
  if (clk_cntr >= 1000) {
    clk_cntr = 0;
    clock_1s = 1;                       /* Activate flag every 1 second       */
  }

  T1IR        = 1;                      /* Clear interrupt flag               */
  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
}


// Function Name: ADC_IRQHandler()
// Author: Drew Schuster
// Called by: ADC completion interrupt interrupt
// Purpose: Reads A/D Conversion result, stores in variable read by main
// Calling convention:  Not called directly, called via interrupt
// Conditions at exit: A/D Conversion result stored, interrupt acknowledged
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
/* A/D IRQ: Executed when A/D Conversion is done                              */
__irq void ADC_IRQHandler(void) {

  AD_last = (AD0DR0 >> 6) & 0x3FF;      /* Read Conversion Result             */

  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
}
