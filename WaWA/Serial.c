/******************************************************************************/
/* SERIAL.C: Low Level Serial Routines                                        */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/
//#define DEVBOARD
#define BASE
//#define REMOTE
#include <LPC23xx.H>                     /* LPC23xx definitions               */
//#include "LCD.h" 

#define SPIF		1 << 7
//unsigned char SPIWRData[0x2];
extern       void LED_On (unsigned int num);
extern       void LED_Off (unsigned int num);
void SPISend( unsigned char *buf, unsigned long Length )
{
  unsigned long i;
  unsigned char Dummy;

  if ( Length == 0 )
	return;
  for ( i = 0; i < Length; i++ )
  {
	S0SPDR = *buf;
	while ( !(S0SPSR & SPIF) );
	Dummy = S0SPDR;		/* Flush the RxFIFO */
	Dummy += 0;
	buf++;
  }
  return; 
}
int getkey3 (void)  {                     /* Read character from Serial Port (UART3)   */

  while (!(U3LSR & 0x01));

  return (U3RBR);
}

int getkey0 (void)  {                     /* Read character from Serial Port (UART0)   */

  while (!(U0LSR & 0x01));

  return (U0RBR);
}
void int_serial0 (void) __irq {		/*UART0 receive interrupt*/
  short in;
  in=getkey0();
 
	DACR= (in) << 9 | 0x00010000;  //shift 8! 6 + 2!
    VICVectAddr = 0;                    /* Acknowledge Interrupt               */

                                      /* Transmit interrupt                  */
}


void init_serial (void)  {               /* Initialize Serial Interface       */
    PCONP        |= (1 << 25);                   /* Enable power to UART3   */
	PCONP        &= 0xFFFFFFEF;          //Disable UART1


    PINSEL0 |= 0x00000050;               /* Enable TxD0 and RxD0              */

    PINSEL0 |= 0x40000000;               /* Enable TxD1                       */
    PINSEL1 |= 0x00000001;               /* Enable RxD1                       */

//	PINSEL1 |= 0xF<<18; 				 /*Enable RxD3 and TxD3*/

#ifdef DEVBOARD
    PINSEL9 |= 0xF<<24;
#else
	PINSEL1 |= 0xF<<18;
#endif
  //U0FDR    = 0;                          /* Fractional divider not used       */
  U0LCR    = 0x83;                       /* 8 bits, no Parity, 1 Stop bit     */
  //U0DLL    = 78;                         /* 9600 Baud Rate @ 12.0 MHZ PCLK    */
  //U0DLL    = 13;                         /* 57600 Baud Rate @ 12.0 MHZ PCLK    */
 
 
  U0DLL = 3; /*115200 Baud Rate*/												  //LOLCRYSTAL
  U0FDR = 0x67	   ;
 // U0DLL = 1;
 // U0FDR = 0x22;
  U0DLM    = 0;                          /* High divisor latch = 0            */
  U0LCR    = 0x03;                       /* DLAB = 0                          */



  		//U1FDR    = 0;                          /* Fractional divider not used       */
  //U1LCR    = 0x83;                       /* 8 bits, no Parity, 1 Stop bit     */
  		//U1DLL    = 78;                         /* 9600 Baud Rate @ 12.0 MHZ PCLK    */
  		//U1DLL    = 13;                         /* 57600 Baud Rate @ 12.0 MHZ PCLK    */
  //U1DLL = 3; /*115200 Baud Rate*/
  //U1FDR = 0x67 ;
  //U1DLM    = 0;                          /* High divisor latch = 0            */
  //U1LCR    = 0x03;                       /* DLAB = 0                          */

  //U3FDR    = 0;                          /* Fractional divider not used       */
  U3LCR    = 0x83;                       /* 8 bits, no Parity, 1 Stop bit     */
  //U0DLL    = 78;                         /* 9600 Baud Rate @ 12.0 MHZ PCLK    */
  //U3DLL    = 13;                         /* 57600 Baud Rate @ 12.0 MHZ PCLK    */
  U3DLL = 3; /*115200 Baud Rate*/											 
  U3FDR = 0x67	;
  U3DLM    = 0;                          /* High divisor latch = 0            */
  U3LCR    = 0x03;                       /* DLAB = 0                          */
   
	/*I don't think we need the THRE interrupts.  We shall see	 */
   // U0IER = 0x03;                       /* Enable RDA and THRE interrupts      */

   /*UART0 Interrupt*/
#ifdef REMOTE
    U0IER = 0x01;                       /* Enable RDA interrupts      */
	VICVectAddr6 = (unsigned long)int_serial0;     /* Set Interrupt Vector                */
    VICVectCntl6 = 6;                   /* use it for UART0 Interrupt          */
    VICIntEnable  = (1  << 6);          /* Enable Interrupt                    */
#endif


   /*SPI enable*/


  PCONP |= (1 << 8);	/* by default, it's enabled already, for safety reason */

  S0SPCR = 0x00;
  /* Port 0.15 SPI SCK, port0.16 uses GPIO SPI_SEL, 
  port0.17 MISO, port0.18 MOSI */
  PINSEL0 |= 0xC0000000;
  PINSEL1 |= 0x0000003C; //Used to be 3C but they probably lied.	   I think it should be 3F
  PINSEL1 &= 0xFFFFFFFC;
  //IODIR0 = 1 << 16;		/* P0.16 is used as GPIO, CS signal to SPI EEPROM */
  //IOSET0 = 1 << 16;		/* P0.16 is used as GPIO, CS signal to SPI EEPROM */

  /* Setting SPI0 clock, for Atmel SEEPROM, SPI clock should be no more 
  than 3Mhz on 4.5V~5.5V, no more than 2.1Mhz on 2.7V~5.5V */
  S0SPCCR = 0x8;
  S0SPCR = 0x00000020;	   //0x20
  FIO0DIR = 1 << 16 ; //pin configured as output

  /*End SPI Enable*/


}
	   //Let's send FF07



/* Implementation of putchar (also used by printf function to output data)    */
int sendchar3 (int ch)  {                 /* Write character to Serial Port  (UART3)  */

  while (!(U3LSR & 0x20));

  return (U3THR = ch);
}

/* Implementation of putchar (also used by printf function to output data)    */
int sendchar0 (int ch)  {                 /* Write character to Serial Port (UART0)   */

  while (!(U0LSR & 0x20));

  return (U0THR = ch);
}

