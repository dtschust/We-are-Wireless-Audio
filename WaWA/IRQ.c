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
#include <stdio.h>


short AD_last;                          /* Last converted value               */
int I2SRX=4;
unsigned char clock_1s;                 /* Flag activated each second         */
int currentButton;
int voiceCode;
/* Import function for turning LEDs on or off                                 */
extern void LED_On (unsigned int num);
extern void LED_Off(unsigned int num);


__irq void I2S_IRQHandler (void) 
{
  //unsigned long RxCount = 0;
  //int I2SRX;
  int temp;
  //if (1==0)
  if ( I2S_STATE & 0x01 )
  {
	/*RxCount = (I2S_STATE >> 8) & 0xFF;
	if ( (RxCount != 0) )//&& !I2SRXDone )
	{
	  while ( RxCount > 0 )
	  {
		if ( I2SReadLength == BUFSIZE )
		{
		  // Stop RX channel 
		  I2S_DAI |= ((0x01 << 3) | (0x01 << 4));
		  I2S_IRQ &= ~(0x01 << 0);	// Disable RX 	
		  I2SRXDone = 1;
		  break;
		}
		else
		{
		  I2SRXBuffer[I2SReadLength++] = I2S_RX_FIFO;
		}
		RxCount--;
	  }
	} */
	temp = I2S_RX_FIFO;
	if (temp != 0)
	{
	I2SRX = I2S_RX_FIFO;
	I2S_TX_FIFO = I2SRX;
	}
  }
  else
  {
    I2SRX= I2S_RX_FIFO;
  }
  VICVectAddr = 0;		/* Acknowledge Interrupt */
}

/* Timer0 IRQ: Executed periodically                                          */
__irq void T0_IRQHandler (void) {
    static int clk_cntr=0;
	static int yesno=0;

		clk_cntr++;
  if (clk_cntr>=156)
  {
    if (yesno == 1)
	{
    FIO2SET = 1 <<11; //Pin goes high
	yesno=0;
	clk_cntr=0;
	}
	else
	{
	FIO2CLR = 1<<11 ; // Pin goes low.
	clk_cntr=0;
	yesno=1;
	}
  }







  AD0CR |= 0x01000000;                  /* Start A/D Conversion               */
 
  T0IR        = 1;                      /* Clear interrupt flag               */
  VICVectAddr = 0;                      /* Acknowledge Interrupt              */
}

__irq void T0_IRQHandler2 (void) {	 /* Timer interrupt to interface with display*/
  static int clk_cntr;
  	int something;//,one,two,four,eight;
  
  if (clk_cntr%100==0)	 /*Update push buttons 10 times a second*/
  {
   currentButton = FIO3PIN;
   currentButton +=0;
  }

  if (clk_cntr%1000==0)
  {
   something = (FIO2PIN>>5) &0xF;
   //something = FIO2PIN & 0x0E0;
  /* one = something & 0x1;
   two = (something & 0x2)>>2;
   four =(something & 0x4)>>3;
   eight = (something & 0x8)>>4;
   voiceCode = one*1+two*2+four*4+eight*8; */
   voiceCode = something;
  }

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
