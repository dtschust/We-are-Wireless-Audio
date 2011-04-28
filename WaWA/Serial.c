// Title: Serial
// Version: 1.0
// Filename: Serial.c
// Author: Drew Schuster (loosely based on MCB2300 development tools provided by Keil)
// Purpose/Function of program: This file controls all serial interactions needed by WaWA
// How Program is Run on Target System: Loaded using JTAG interface, functions called by WaWA.c.
// Date Started: March 14th
// Update History:  See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio


//#define DEVBOARD
#define BASE
//#define REMOTE
#include <LPC23xx.H>                     /* LPC23xx definitions               */


#define SPIF		1 << 7
extern       void LED_On (unsigned int num);
extern       void LED_Off (unsigned int num);

// Function Name: SPISend
// Author: Drew Schuster
// Called by: main
// Purpose: sends a message to the potentiometer
// Calling convention:    P[0]16 has to go low before sending, and go high after
                         //FIO0CLR = 1 << 16;
                        //SPISend(SPIWRData, 0x2);
                        //FIO0SET = 1 << 16;
// Conditions at exit: potentiometer message will have been sent
// Date Started: April 10th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
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


// Function Name: getkey3()
// Author: Drew Schuster
// Called by: display_init
// Purpose: Receives an acknowledgement from display
// Calling convention:   value =  getkey3()	;
// Conditions at exit: Returns message from display
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
int getkey3 (void)  {                     /* Read character from Serial Port (UART3)   */

  while (!(U3LSR & 0x01));

  return (U3RBR);
}

// Function Name: getkey0()
// Author: Drew Schuster
// Called by: display_init
// Purpose: Receives messages from XBee module
// Calling convention:   value =  getkey0()	 ;
// Conditions at exit: Returns message from XBee
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
int getkey0 (void)  {                     /* Read character from Serial Port (UART0)   */

  while (!(U0LSR & 0x01));

  return (U0RBR);
}

// Function Name: int_serial0()
// Author: Drew Schuster
// Called by: called on UART0 interrupt
// Purpose: Reads a message from XBee, outputs it to amplifier via DAC
// Calling convention:    Called via interrupt
// Conditions at exit: Acknowledges interrupt
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
void int_serial0 (void) __irq {		/*UART0 receive interrupt*/
  short in;
  in=getkey0();
 
	DACR= (in) << 9 | 0x00010000;  //shift 8! 6 + 2!
    VICVectAddr = 0;                    /* Acknowledge Interrupt               */

                                      /* Transmit interrupt                  */
}


// Function Name: init_serial()
// Author: Drew Schuster
// Called by: main
// Purpose: Initializes all serial communication, enables appropriate interrupts
// Calling convention:    init_serial()
// Conditions at exit: all serial communication is properly enabled
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
void init_serial (void)  {               /* Initialize Serial Interface       */
    PCONP        |= (1 << 25);                   /* Enable power to UART3   */
	PCONP        &= 0xFFFFFFEF;          //Disable UART1


    PINSEL0 |= 0x00000050;               /* Enable TxD0 and RxD0              */

    PINSEL0 |= 0x40000000;               /* Enable TxD1                       */
    PINSEL1 |= 0x00000001;               /* Enable RxD1                       */

#ifdef DEVBOARD
    PINSEL9 |= 0xF<<24;
#else
	PINSEL1 |= 0xF<<18;
#endif
  U0LCR    = 0x83;                       /* 8 bits, no Parity, 1 Stop bit     */

 
 
  U0DLL = 3; /*38400 Baud Rate*/												  
  U0FDR = 0x67	   ;
  U0DLM    = 0;                          /* High divisor latch = 0            */
  U0LCR    = 0x03;                       /* DLAB = 0                          */

  U3LCR    = 0x83;                       /* 8 bits, no Parity, 1 Stop bit     */
  U3DLL = 3; /*38400 Baud Rate*/											 
  U3FDR = 0x67	;
  U3DLM    = 0;                          /* High divisor latch = 0            */
  U3LCR    = 0x03;                       /* DLAB = 0                          */
   

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
  PINSEL1 |= 0x0000003C; 
  PINSEL1 &= 0xFFFFFFFC;

  /* Setting SPI0 clock */
  S0SPCCR = 0x8;
  S0SPCR = 0x00000020;	   //0x20
  FIO0DIR = 1 << 16 ; //pin configured as output

  /*End SPI Enable*/


}

/* Implementation of putchar (also used by printf function to output data)    */
// Function Name: sendchar3()
// Author: Drew Schuster
// Called by: display_init,preprint,main
// Purpose: sends a message to display
// Calling convention:    sendchar3(char)
// Conditions at exit: Message sent to display
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
int sendchar3 (int ch)  {                 /* Write character to Serial Port  (UART3)  */

  while (!(U3LSR & 0x20));

  return (U3THR = ch);
}

/* Implementation of putchar (also used by printf function to output data)    */
// Function Name: sendchar0()
// Author: Drew Schuster
// Called by: main
// Purpose: Sends audio data to the XBee (Base Station)
// Calling convention:    sendchar0(char)
// Conditions at exit: Message sent to XBee
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
int sendchar0 (int ch)  {                 /* Write character to Serial Port (UART0)   */

  while (!(U0LSR & 0x20));

  return (U0THR = ch);
}

