/******************************************************************************/
/* SERIAL.C: Low Level Serial Routines                                        */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <LPC23xx.H>                     /* LPC23xx definitions               */
#include "LCD.h" 
//#define UART0                            /* Use UART 0 for printf             */

/* If UART 0 is used for printf                                               */
/*#ifdef UART0
  #define UxFDR  U0FDR
  #define UxLCR  U0LCR
  #define UxDLL  U0DLL
  #define UxDLM  U0DLM
  #define UxLSR  U0LSR
  #define UxTHR  U0THR
  #define UxRBR  U0RBR*/
/* If UART 1 is used for printf                                               */
/*#elif defined(UART1)
  #define UxFDR  U1FDR
  #define UxLCR  U1LCR
  #define UxDLL  U1DLL
  #define UxDLM  U1DLM
  #define UxLSR  U1LSR
  #define UxTHR  U1THR
  #define UxRBR  U1RBR
#endif	*/
extern       void LED_On (unsigned int num);
extern       void LED_Off (unsigned int num);

int getkey3 (void)  {                     /* Read character from Serial Port   */

  while (!(U3LSR & 0x01));

  return (U3RBR);
}

int getkey (void)  {                     /* Read character from Serial Port   */

  while (!(U0LSR & 0x01));

  return (U0RBR);
}
void int_serial (void) __irq {
  short in;
  in=getkey();
 
	DACR= (in) << 9 | 0x00010000;  //shift 8! 6 + 2!
    VICVectAddr = 0;                    /* Acknowledge Interrupt               */

                                      /* Transmit interrupt                  */
}


void init_serial (void)  {               /* Initialize Serial Interface       */
    PCONP        |= (1 << 25);                   /* Enable power to UART3   */


    PINSEL0 |= 0x00000050;               /* Enable TxD0 and RxD0              */

    PINSEL0 |= 0x40000000;               /* Enable TxD1                       */
    PINSEL1 |= 0x00000001;               /* Enable RxD1                       */

//	PINSEL1 |= 0xF<<18; 				 /*Enable RxD3 and TxD3*/
    PINSEL9 |= 0xF<<24;

  U0FDR    = 0;                          /* Fractional divider not used       */
  U0LCR    = 0x83;                       /* 8 bits, no Parity, 1 Stop bit     */
  //U0DLL    = 78;                         /* 9600 Baud Rate @ 12.0 MHZ PCLK    */
  U0DLL    = 13;                         /* 57600 Baud Rate @ 12.0 MHZ PCLK    */
  U0DLM    = 0;                          /* High divisor latch = 0            */
  U0LCR    = 0x03;                       /* DLAB = 0                          */


  U1FDR    = 0;                          /* Fractional divider not used       */
  U1LCR    = 0x83;                       /* 8 bits, no Parity, 1 Stop bit     */
  //U1DLL    = 78;                         /* 9600 Baud Rate @ 12.0 MHZ PCLK    */
  U1DLL    = 13;                         /* 57600 Baud Rate @ 12.0 MHZ PCLK    */
  U1DLM    = 0;                          /* High divisor latch = 0            */
  U1LCR    = 0x03;                       /* DLAB = 0                          */

  U3FDR    = 0;                          /* Fractional divider not used       */
  U3LCR    = 0x83;                       /* 8 bits, no Parity, 1 Stop bit     */
  //U0DLL    = 78;                         /* 9600 Baud Rate @ 12.0 MHZ PCLK    */
  U3DLL    = 13;                         /* 57600 Baud Rate @ 12.0 MHZ PCLK    */
  U3DLM    = 0;                          /* High divisor latch = 0            */
  U3LCR    = 0x03;                       /* DLAB = 0                          */
   
	/*I don't think we need the THRE interrupts.  We shall see	 */
   // U0IER = 0x03;                       /* Enable RDA and THRE interrupts      */

   /*UART0 Interrupt*/
       U0IER = 0x01;                       /* Enable RDA interrupts      */
	VICVectAddr6 = (unsigned long)int_serial;     /* Set Interrupt Vector                */
    VICVectCntl6 = 6;                   /* use it for UART0 Interrupt          */
    VICIntEnable  = (1  << 6);          /* Enable Interrupt                    */


}


/* Implementation of putchar (also used by printf function to output data)    */
int sendchar (int ch)  {                 /* Write character to Serial Port    */

  while (!(U3LSR & 0x20));

  return (U3THR = ch);
}

/* Implementation of putchar (also used by printf function to output data)    */
int sendchar0 (int ch)  {                 /* Write character to Serial Port    */

  while (!(U0LSR & 0x20));

  return (U0THR = ch);
}

