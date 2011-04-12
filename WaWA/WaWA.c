/* WaWA.c : Wireless Audio System	   */
/******************************************************************************/
                  
#include <stdio.h>
#include <LPC23xx.H>                    /* LPC23xx definitions                */
//#include "LCD.h"                        /* Graphic LCD function prototypes    */


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

/* Import external variables from IRQ.c file                                  */
extern short AD_last;
extern unsigned char clock_1s;
extern int currentButton;
extern int voiceCode;

extern int I2SRX;

int in;
char out;
int volume;

int lastonoff;
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

/* Function that initializes the display and puts an initial value on the screen*/
void display_init(void)
{
  // Let's ghetto make pin 75 2[11] the clock for pcm3010!
  PINSEL4 &= 0xFF9FFFFF;
  FIO2DIR = 1 << 11 ; //pin configured as output
  FIO2CLR = 1 << 11;
  // Let's make the LCD nReset pin P3[1] AKA pin 140!
  PINSEL6 &= 0xFFFFFFF3;
  FIO3DIR = 1 << 1 ; //pin configured as output
  FIO3CLR =1<<1 ; // pin goes Low.

  //Let's setup the volume input pins! P3[2-4]

  PINSEL6 &=		 0xFFFFFC0F ;	 // Configure all 3 pins as I/O
  //FIO3DIR = 0;				(Already done above?)         // Configure all 3 pins as input

  //set nReset to low.
  LED_On(3);
  while (clock_1s ==0);
  clock_1s=0;
  while (clock_1s ==1);
  while (clock_1s ==0);
  LED_Off(3);
  clock_1s=0;
  //set nReset to high
  FIO3SET =1<<1; //pin goes High
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
  sendchar3(0xFF);
  sendchar3(0xFF);
  in = getkey3();
  

}

void preprint(void)
{
  sendchar3(0x4F);		//Set as opaque background
  sendchar3(0x01);		//Set as opaque background
  sendchar3(0x73);
  sendchar3(0x03);
  sendchar3(0x04);
  sendchar3(0x02);
  sendchar3(0x00);
  sendchar3(0x00);
}

void checkVolumeChange(void)
{
  static int lastButton;
  //Buttons
  if ((lastButton & 0x1C)!= (currentButton & 0x1C))	 //If any of the 3 inputs changed

  {	
    if ( (lastButton&0x4) != (currentButton&0x4) && (currentButton&0x4) != 0x4)
    {
       volume++;
    } 
    else if ( (lastButton&0x8) != (currentButton&0x8 && (currentButton&0x8) !=0x8))
    {
      volume--;
    }
    else if ( (lastButton&0x10) != (currentButton&0x10) && (currentButton&0x10) !=0x10)
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
	else if (voiceCode == 3)
	{
	 volume=0;
	}
	else
	{
	 //volume+=2;
	}
  }
  if (volume <0)
  {
    volume=0;
  }
  else if (volume > 11)
  {
    volume=11;
  }
  lastButton = currentButton;

}

int main (void) {
  int count=0;
  int lastVolume;
  LED_Init();                           /* LED Initialization                 */
  volume=5;
   /* Enable and setup timer interrupt, start timer                            */
  //T0MR0         = 11999;                       /* 1msec = 12000-1 at 12.0 MHz */
  T0MR0         = 299;                       /* 0.025msec = 300-1 at 12.0 MHz */
  T0MCR         = 3;                           /* Interrupt and Reset on MR0  */
  T0TCR         = 0;                           /* Timer0 Enable             */
  VICVectAddr4  = (unsigned long)T0_IRQHandler;/* Set Interrupt Vector        */
  VICVectCntl4  = 15;                          /* use it for Timer0 Interrupt */
  VICIntEnable  |= (1  << 4);                   /* Enable Timer0 Interrupt     */

  /* Enable and setup timer interrupt (useful for communicating with display)*/
  T1MR0         = 11999;                       /* 1msec = 12000-1 at 12.0 MHz */
  T1MCR         = 3;                           /* Interrupt and Reset on MR0  */
  T1TCR         = 1;                           /* Timer1 Enable            */
  VICVectAddr5  = (unsigned long)T0_IRQHandler2;/* Set Interrupt Vector        */
  VICVectCntl5  = 16;                          /* use it for Timer1 Interrupt */
  VICIntEnable  |= (1  << 5);                   /* Enable Timer0 Interrupt     */ 

  			
  /* Power enable, Setup pin, enable and setup AD converter interrupt         */
  PCONP        |= (1 << 12);                   /* Enable power to AD block    */
  PINSEL1       = 0x00204000;                      /* AD0.0 pin function select   */
  AD0INTEN      = (1 <<  0);                   /* CH0 enable interrupt        */
  AD0CR         = 0x00200301;                  /* Power up, PCLK/4, sel AD0.0 */
  VICVectAddr18 = (unsigned long)ADC_IRQHandler;/* Set Interrupt Vector       */
  VICVectCntl18 = 14;                          /* use it for ADC Interrupt    */
  VICIntEnable  |= (1  << 18);                  /* Enable ADC Interrupt        */

  init_serial();                               /* Init UART                   */
 



  /*Initialize the display*/
  display_init();


  /*Let's try to turn on I2S :-/ */
  //PCONP |= (1 << 27);  /*Enable power to I2S partay */

  /*Enable pins 0[4] - 0[9]: I don't trust these pins*/
  //PINSEL0 &= ~0x000FFF00;
  //PINSEL0 |= 0x00055500;  

  //I2S_RXRATE = 0x6;
  //I2S_TXRATE = 0x6;

  //I2S_DAO = 0x7C3;
  //I2S_DAI = 0x7C3;

  //I2S_IRQ = (1) | (1<<8) ; /*Enable interrupt for receive*/

  //VICVectAddr31 = (unsigned long)I2S_IRQHandler;/* Set Interrupt Vector       */
  //VICVectCntl6 = 14;                          /* use it for ADC Interrupt    */
  //VICIntEnable  |= (unsigned int)(0x1  << 31);                  /* Enable I2S Interrupt        */





  /* Let's initialize the voice recognition pins: p1.19-22	 */
  // In a perfect world those would be the pins.  But yeah not so much.
  // p2[5-8] = 97,96,95,93
  PINSEL4 &=  0xFFFC03FF;
  //FIO2DIR not necessary
  
   PINSEL4 &= 0xFF9FFFFF;
  FIO2DIR = 1 << 11 ; //pin configured as output
  FIO2CLR = 1 << 11;


  T0TCR         = 1;                           /* Timer0 Enable (Start A-D conversions!)              */
  while (1) {                           /* Loop forever                       */
   /* Audio Code*/
    out = AD_last>>2;
    sendchar0(out);
	/*End Audio Code*/

    /*Volume control*/
    checkVolumeChange();

	if (lastVolume != volume)
	{
       preprint();
	   //printf("Volume: %2d %x",volume,(voiceCode>>5)& 0xF);
	   printf("Volume: %2d",volume);
	  // printf("Volume: %2d %2d",volume,voiceCode);
	   //printf("Blah: %x, %d",I2SRX,count++);
	   sendchar3(0x00);
	}
	lastVolume=volume;	  
    /*End Volume control*/



	/*Deprecated Code! Now goes in IRQ*/
	//in = getkey0();
	//DACR= (out) << 8 | 0x00010000;	
	
  }
}
