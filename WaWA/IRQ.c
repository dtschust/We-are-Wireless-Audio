/******************************************************************************/
/* IRQ.C: IRQ Handler                                                         */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <LPC23xx.H>                    /* LPC23xx definitions                */


short AD_last;                          /* Last converted value               */
unsigned char clock_1s;                 /* Flag activated each second         */

/* Import function for turning LEDs on or off                                 */
extern void LED_On (unsigned int num);
extern void LED_Off(unsigned int num);


/* Timer0 IRQ: Executed periodically                                          */
__irq void T0_IRQHandler (void) {
 // static int clk_cntr;
 // unsigned int n;
 // unsigned int v;

//  clk_cntr++;
//  if (clk_cntr >= 1000) {
//    clk_cntr = 0;
//    clock_1s = 1;                       /* Activate flag every 1 second       */
//  }

//  v = (5*AD_last) >> 9;                 /* Scale the Value                    */
//  for (n = 0; n < 8; n++)               /* Show the Value                     */
//    if (n < v) LED_On (n);              /* Turn LED On                        */
//    else       LED_Off(n);              /* Turn LED Off                       */

  AD0CR |= 0x01000000;                  /* Start A/D Conversion               */

  T0IR        = 1;                      /* Clear interrupt flag               */
  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
}

__irq void T0_IRQHandler2 (void) {
  static int clk_cntr;


  clk_cntr++;
  if (clk_cntr >= 1000) {
    clk_cntr = 0;
    clock_1s = 1;                       /* Activate flag every 1 second       */
  }


  T1IR        = 1;                      /* Clear interrupt flag               */
  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
}

/* A/D IRQ: Executed when A/D Conversion is done                              */
__irq void ADC_IRQHandler(void) {

  AD_last = (AD0DR0 >> 6) & 0x3FF;      /* Read Conversion Result             */

  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
}
