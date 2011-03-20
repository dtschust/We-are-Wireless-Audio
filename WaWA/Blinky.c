/******************************************************************************/
/* BLINKY.C: LED Flasher                                                      */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/
                  
#include <stdio.h>
#include <LPC23xx.H>                    /* LPC23xx definitions                */
#include "LCD.h"                        /* Graphic LCD function prototypes    */


/* Function that initializes LEDs                                             */
void LED_Init(void) {
  PINSEL10 = 0;                         /* Disable ETM interface, enable LEDs */
  FIO2DIR  = 0x000000FF;                /* P2.0..7 defined as Outputs         */
  FIO2MASK = 0x00000000;
}

/* Function that turns on requested LED                                       */
void LED_On (unsigned int num) {
  FIO2SET = (1 << num);
}

/* Function that turns off requested LED                                      */
void LED_Off (unsigned int num) {
  FIO2CLR = (1 << num);
}

/* Function that outputs value to LEDs                                        */
void LED_Out(unsigned int value) {
  FIO2CLR = 0xFF;                       /* Turn off all LEDs                  */
  FIO2SET = (value & 0xFF);             /* Turn on requested LEDs             */
}


/* Function for displaying bargraph on the LCD display                        */
void Disp_Bargraph(int pos_x, int pos_y, int value) {
  int i;

  set_cursor (pos_x, pos_y);
  for (i = 0; i < 16; i++)  {
    if (value > 5)  {
      lcd_putchar (0x05);
      value -= 5;
    }  else  {
      lcd_putchar (value);
      value = 0;
    }
  }
}

/* Import external IRQ handlers from IRQ.c file                               */
extern __irq void T0_IRQHandler  (void);
extern __irq void T0_IRQHandler2  (void);
extern __irq void ADC_IRQHandler (void);

/* Import external functions from Serial.c file                               */
extern       void init_serial    (void);
extern       int sendchar    (int ch);
extern       int sendchar0    (int ch);
extern       int getkey3    (void);
extern       int getkey    (void);

/* Import external variables from IRQ.c file                                  */
extern short AD_last;

extern unsigned char clock_1s;


int main (void) {
//int i;
int in;
char out;
int volume=5;


 

  LED_Init();                           /* LED Initialization                 */
   /* Enable and setup timer interrupt, start timer                            */
  //T0MR0         = 11999;                       /* 1msec = 12000-1 at 12.0 MHz */

  T0MR0         = 300;                       /* 1msec = 12000-1 at 12.0 MHz */
  T0MCR         = 3;                           /* Interrupt and Reset on MR0  */
  T0TCR         = 0;                           /* Timer0 Enable             */
  VICVectAddr4  = (unsigned long)T0_IRQHandler;/* Set Interrupt Vector        */
  VICVectCntl4  = 15;                          /* use it for Timer0 Interrupt */
  VICIntEnable  = (1  << 4);                   /* Enable Timer0 Interrupt     */

  

  /* Power enable, Setup pin, enable and setup AD converter interrupt         */
  PCONP        |= (1 << 12);                   /* Enable power to AD block    */
  PINSEL1       = 0x00204000;                      /* AD0.0 pin function select   */
  AD0INTEN      = (1 <<  0);                   /* CH0 enable interrupt        */
  AD0CR         = 0x00200301;                  /* Power up, PCLK/4, sel AD0.0 */
  VICVectAddr18 = (unsigned long)ADC_IRQHandler;/* Set Interrupt Vector       */
  VICVectCntl18 = 14;                          /* use it for ADC Interrupt    */
  VICIntEnable  = (1  << 18);                  /* Enable ADC Interrupt        */

  init_serial();                               /* Init UART                   */
 
  lcd_init();
  lcd_clear();
  lcd_print ("We are Wireless Audio!");
  set_cursor (0, 1);
  lcd_print ("WaWA");
  

  /*Initialize the party I mean LCD*/
  // Let's arbitrarily make the LCD nReset pin P3[1] AKA pin 140!
  PINSEL6 &= 0xFFFFFFF3;
  FIO3DIR = 1 << 1 ; //pin configured as output
  FIO3CLR =1<<1 ; // pin goes Low.

 /*Let's make a new timer!	 */
  T1MR0         = 11999;                       /* 1msec = 12000-1 at 12.0 MHz */
  T1MCR         = 3;                           /* Interrupt and Reset on MR0  */
  T1TCR         = 1;                           /* Timer0 Enable               */
  VICVectAddr5  = (unsigned long)T0_IRQHandler2;/* Set Interrupt Vector        */
  VICVectCntl5  = 16;                          /* use it for Timer0 Interrupt */
  VICIntEnable  = (1  << 5);                   /* Enable Timer0 Interrupt     */ 


  //set nReset to low.
  while (clock_1s ==0);
      clock_1s=0;
  while (clock_1s ==1);
  while (clock_1s ==0);
      clock_1s=0;
      //set nReset to high
	  FIO3SET =1<<1; //pin goes High
	  while (clock_1s==1);
	  while (clock_1s==0);
	  clock_1s=0;
	  sendchar(0x55);
	  //while (clock_1s==1);
	  //while (clock_1s==0);
	  //clock_1s=0;
	  in = getkey3();
	  if (in != 6)
	{
	  lcd_print("Trouble in paradise");
    }

	T1TCR         = 0;                           /* Timer1 Disable               */
  /*Initialization complete*/

	//Set a black background
  	  sendchar(0x42);
	  sendchar(0xFF);
	  sendchar(0xFF);
	  in = getkey3();

	  /*Formatted text*/
	  sendchar(0x73);
	  sendchar(0x03);
	  sendchar(0x04);
	  sendchar(0x02);
	  sendchar(0x00);
	  sendchar(0x00);	 
	  printf("Volume=%d", volume);
	  sendchar(0x00);

  T0TCR         = 1;                           /* Timer0 Enable              */
  while (1) {                           /* Loop forever                       */
   /* Audio Code*/
    out = AD_last>>2;
    sendchar0(out);
	/*End Audio Code*/




	/*Deprecated Code!*/
	//in = getkey();
	//DACR= (out) << 8 | 0x00010000;	
	
  }
}
