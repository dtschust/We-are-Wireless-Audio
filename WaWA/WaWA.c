/* WaWA.c : Wireless Audio System	   */
// Title: WaWA
// Version: 1.0
// Filename: WaWA.c
// Author: Drew Schuster  (loosely based on MCB2300 development tools provided by Keil)
// Purpose/Function of program: This is the main file controlling WaWA.
//   It controls the volume, and the initializations, and calls functions
//   In other programs to interact with peripherals and deal with ADC/DAC.
// How Program is Run on Target System: Loaded using JTAG interface, executes on power on.
// Date Started: March 14th
// Update History:  See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
/******************************************************************************/
#define BASE
//#define DEVBOARD
//#define REMOTE

                  
#include <stdio.h>
#include <LPC23xx.H>                    /* LPC23xx definitions                */
#define BUFSIZE			0x2
unsigned char SPIWRData[BUFSIZE];

/* Import external IRQ handlers from IRQ.c file                               */
extern __irq void T0_IRQHandler  (void);
extern __irq void T0_IRQHandler2  (void);
extern __irq void ADC_IRQHandler (void);
extern __irq void I2S_IRQHandler (void);

/* Import external functions from Serial.c file                               */
extern       void init_serial    (void);
extern       int sendchar3    (int ch);
extern       int sendchar0    (int ch);
extern       int getkey3    (void);
extern       int getkey0    (void);
extern       void SPISend( unsigned char *buf, unsigned long Length )  ;
/* Import external variables from IRQ.c file                                  */
extern short AD_last;
extern unsigned char clock_1s;
extern int currentButton;
extern int voiceCode;



int in;
char out;
int volume;


/* Function that initializes the display and puts an initial value on the screen*/
// Function Name: display_init
// Author: Drew Schuster
// Called by: main
// Purpose: Initializes the display
// Calling convention: display_init();
// Conditions at exit: Display will be initialized, text can now be displayed
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
void display_init(void)
{
  //Reset pin for the LED display
  PINSEL6 &= 0xFFFFFCFF;
  FIO3DIR = 1 << 4 ; //pin configured as output
  FIO3SET =1<<4; //pin goes High


  //Let's setup the volume input pins! P3[2-4]
  PINSEL6 &=		 0xFFFFFC0F ;	 // Configure all 3 pins as I/O

  
  while (clock_1s ==0);
  clock_1s=0;
  FIO3CLR =1<<4 ; // pin goes Low.
  while (clock_1s ==1);
  while (clock_1s ==0);
  clock_1s=0;
  //set nReset to high
  FIO3SET =1<<4; //pin goes High
  while (clock_1s==1);
  while (clock_1s==0);
  clock_1s=0;
  sendchar3(0x55);
  in = getkey3();
  if (in != 6)
  {	   
    //We have a problem, ack was not received properly!
  }

    
  /*Initialization complete*/
  //Set a black background
  sendchar3(0x42);
  sendchar3(0x00); 
  sendchar3(0x00); 
  in = getkey3();
  

}


// Function Name: preprint
// Author: Drew Schuster
// Called by: main
// Purpose: sends the initial commands to display onscreen.  Afterwards
//   a printf command can write text to the screen.
// Calling convention: preprint();
// Conditions at exit: printf command will now display text onscreen.
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
void preprint(void)
{
  sendchar3(0x4F);		//Set as opaque background
  sendchar3(0x01);		//Set as opaque background
  sendchar3(0x73);		// Unformatted text
  sendchar3(0x03);		// Unformatted Text
  sendchar3(0x04);	    // Row Number
  sendchar3(0x02);		// Column Number
  sendchar3(0xFF);	 // White text
  sendchar3(0xFF);	 // White text
}

// Function Name: checkVolumeChange
// Author: Drew Schuster
// Called by: main
// Purpose: Checks buttons and voicerecognition flags for volume change commands
// Calling convention: checkVolumeChange();
// Conditions at exit: volume variable will have been updated
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
void checkVolumeChange(void)
{
  static int lastButton;
  //Buttons
  if (lastButton != currentButton)
  {	
	if ( (lastButton&0x1)  != (currentButton&0x1) && (currentButton&0x1 == 0x1))
    {
       volume++;
    } 
	else if ((lastButton&0x2) != (currentButton&0x2) && ((currentButton&0x2) == 0x2))
    {
      volume--;
    }
	else if ( (lastButton&0x4) != (currentButton&0x4) && (currentButton&0x4) == 0x4)
    {
      volume = 0;
    }
	   	
    else
    {
	  //This occurs on press of button instead of release.  Do nothing here
    }	  
  }
  //Voice
  if (clock_1s == 1)
  {				  
    clock_1s = 0;	 
    if (voiceCode == 1)
    {
     volume++;
    }
	else if (voiceCode == 2)
	{
	 volume--;
	}
	else if (voiceCode == 4)
	{
	 volume=0;
	}
	else
	{
	}	 
  }
  if (volume <0)
  {
    volume=0;
  }
  else if (volume > 5)
  {
    volume=5;
  }
  lastButton = currentButton;

}

// Function Name: main
// Author: Drew Schuster
// Called by: runs on powerup
// Purpose: Initializes registers and peripherals.  Handles volume changes
     // And updates to the display
// Calling convention: is not called by any other function
// Conditions at exit: No exit, infinite while loop
// Date Started: March 14th
// Update History: 	See Github repository at https://github.com/dr3wster/We-are-Wireless-Audio
int main (void) {
#ifdef BASE
int lastVolume=-1;
#endif
SCS |= 1;  //Enable fast GPIO
#ifdef BASE
  volume=5;
   /* Enable and setup timer interrupt, start timer                            */
  T0MR0         = 99;                       /* 0.025msec = 100-1 at 4.0 MHz*/
  T0MCR         = 3;                           /* Interrupt and Reset on MR0  */
  T0TCR         = 0;                           /* Timer0 Enable             */
  VICVectAddr4  = (unsigned long)T0_IRQHandler;/* Set Interrupt Vector        */
  VICVectCntl4  = 15;                          /* use it for Timer0 Interrupt */
  VICIntEnable  |= (1  << 4);                   /* Enable Timer0 Interrupt     */

  /* Enable and setup timer interrupt (useful for communicating with display)*/
  T1MR0         = 3999;                       /* 1msec = 4000-1 at 4.0 MHz*/
  T1MCR         = 3;                           /* Interrupt and Reset on MR0  */
  T1TCR         = 1;                           /* Timer1 Enable            */
  VICVectAddr5  = (unsigned long)T0_IRQHandler2;/* Set Interrupt Vector        */
  VICVectCntl5  = 16;                          /* use it for Timer1 Interrupt */
  VICIntEnable  |= (1  << 5);                   /* Enable Timer1 Interrupt     */ 
#endif
		
  /* Power enable, Setup pin, enable and setup AD converter interrupt         */
  PCONP        |= (1 << 12);                   /* Enable power to AD block    */
  PINSEL1       = 0x00204000;                      /* AD0.0 pin function select   */
  AD0INTEN      = (1 <<  0);                   /* CH0 enable interrupt        */
  AD0CR         = 0x00200301;                  /* Power up, PCLK/4, sel AD0.0 */
#ifdef BASE
  VICVectAddr18 = (unsigned long)ADC_IRQHandler;/* Set Interrupt Vector       */
  VICVectCntl18 = 14;                          /* use it for ADC Interrupt    */
  VICIntEnable  |= (1  << 18);                  /* Enable ADC Interrupt        */
#endif
  init_serial();                               /* Init UART                   */
 


#ifdef BASE
  /*Initialize the display*/
  #ifdef DEVBOARD
  // development board doesn't have display connected
  #else
  display_init();
  #endif
#endif  

#ifdef BASE  
  /* Let's initialize the voice recognition pins: p1.20-24	 */
#ifdef DEVBOARD
  PINSEL4 &=  0xFFFC03FF;
#else
  //p1[20-24]
  PINSEL3 &=   0xFFFC00FF;
  FIO3DIR = 0;

  //Pushbuttons	p1[15-17]
  PINSEL2 &= 0x3FFFFFFF;
  PINSEL3 &= 0xFFFFFFF0; 
  FIO1DIR &= 0xFFFC7FFF;

#endif

  T0TCR         = 1;                           /* Timer0 Enable (Start A-D conversions!)              */
#endif

 		   
#ifdef REMOTE
//Set pin P3[26] to high (for amplifier)
PINSEL4 &= 0x0;
FIO3DIR = 1 << 26 ; //pin configured as output
FIO3SET = 1 << 26;


#endif  
#ifdef BASE
 		 
//Initialize the potentiometer 	 
  SPIWRData[0]=0x24;  
  SPIWRData[1]=0x00;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16; 
  
  SPIWRData[0]=0x1C;  	
  SPIWRData[1]=0x03;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;
   
  SPIWRData[0]=0x05;
  SPIWRData[1]=0x40;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;

#endif
  while (1) {                           /* Loop forever                       */

#ifdef BASE   	   
   /* Audio Code*/

   if (volume == 5)
   {
    out = AD_last>>2;
   }
   else if (volume == 4)
   {
    out = (AD_last>>3) & 0x7F;
   }
   else if (volume == 3)
   {
    out = (AD_last>>4) & 0x3F;
   }
      else if (volume == 2)
   {
    out = (AD_last>>5) & 0x1F;
   }
      else if (volume == 1)
   {
    out = (AD_last>>6) & 0x0F;
   }
   else
   {
    out = AD_last>>2;
   }
	if (volume !=0)
      sendchar0(out);
	/*End Audio Code*/

    /*Volume control*/
    checkVolumeChange();
   
	if (lastVolume != volume)
	{
	  if (volume !=0)
	  {
	     U0IER = 0x01;                       /* Enable RDA interrupts      */
	  }
	  if (volume != 5)
	  {
	  SPIWRData[0]=0x24;  
	  SPIWRData[1]=0x00;
	  FIO0CLR = 1 << 16;
	  SPISend(SPIWRData, 0x2);
	  FIO0SET = 1 << 16; 
	  }			  
	  if (volume == 5){	
	  //Maximum volume		 			
	  SPIWRData[0]=0x24;  
	  SPIWRData[1]=0x01;
	  FIO0CLR = 1 << 16;
	  SPISend(SPIWRData, 0x2);
	  FIO0SET = 1 << 16;
	  }
 	  else if (volume == 4)
  		{
    	SPIWRData[0]=0x04; //05FF
  		SPIWRData[1]=0x80;
 		FIO0CLR = 1 << 16;
 		SPISend(SPIWRData, 0x2);
 		FIO0SET = 1 << 16;
  		}
      else if (volume == 3)
 		 {
    	SPIWRData[0]=0x04; //05FF
  		SPIWRData[1]=0x40;
  		FIO0CLR = 1 << 16;
  		SPISend(SPIWRData, 0x2);
  		FIO0SET = 1 << 16;
  		}
    else if (volume == 2)
  		{
    	SPIWRData[0]=0x04; //05FF
  		SPIWRData[1]=0x08;
 	 	FIO0CLR = 1 << 16;
  		SPISend(SPIWRData, 0x2);
  		FIO0SET = 1 << 16;
  		}
    else if (volume == 1)
  		{
  		SPIWRData[0]=0x04; //05FF
  		SPIWRData[1]=0x00;
  		FIO0CLR = 1 << 16;
  		SPISend(SPIWRData, 0x2);
  		FIO0SET = 1 << 16;
  		}
   		  
  	else if (volume == 0)
  		{
  		SPIWRData[0]=0x04;  
  		SPIWRData[1]=0x00;
  		FIO0CLR = 1 << 16;
  		SPISend(SPIWRData, 0x2);
  		FIO0SET = 1 << 16; 
  }	 

       //Update Display
       preprint();
	   printf("Volume: %2d",volume);
	   sendchar3(0x00);

	}
	lastVolume=volume;	  
    /*End Volume control*/

#endif	
  }
}
