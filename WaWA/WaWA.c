/* WaWA.c : Wireless Audio System	   */
/******************************************************************************/
#define BASE
//#define DEVBOARD
//#define REMOTE

                  
#include <stdio.h>
#include <LPC23xx.H>                    /* LPC23xx definitions                */
//#include "LCD.h"                        /* Graphic LCD function prototypes    */
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

extern int I2SRX;

int in;
char out;
int volume;

int lastonoff;

/*void LED_Init(void) {
  PINSEL10 = 0;                       
  FIO2DIR  = 0x000000FF;              
  FIO2MASK = 0x00000000;
}


void LED_On (unsigned int num) {
  FIO2SET = (1 << num);
}


void LED_Off (unsigned int num) {
  FIO2CLR = (1 << num);
}

                                    
void LED_Out(unsigned int value) {
  FIO2CLR = 0xFF;                       
  FIO2SET = (value & 0xFF);           
} */

/* Function that initializes the display and puts an initial value on the screen*/
void display_init(void)
{
  // Let's ghetto make pin 75 2[11] the clock for pcm3010!
  /*PINSEL4 &= 0xFF9FFFFF;
  FIO2DIR = 1 << 11 ; //pin configured as output
  FIO2CLR = 1 << 11; */
  // Let's make the LCD nReset pin P3[1] AKA pin 140!
  //PINSEL6 &= 0xFFFFFFF3;
  PINSEL6 &= 0xFFFFFCFF;
  FIO3DIR = 1 << 4 ; //pin configured as output
  //FIO3CLR =1<<4 ; // pin goes Low.
  FIO3SET =1<<4; //pin goes High
  //Let's setup the volume input pins! P3[2-4]

  PINSEL6 &=		 0xFFFFFC0F ;	 // Configure all 3 pins as I/O

  //FIO3DIR = 0;				(Already done above?)         // Configure all 3 pins as input

  //set nReset to low.
  //LED_On(3);
  while (clock_1s ==0);
  clock_1s=0;
  FIO3CLR =1<<4 ; // pin goes Low.
  while (clock_1s ==1);
  while (clock_1s ==0);
  //LED_Off(3);
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
  sendchar3(0x00); //0xFF
  sendchar3(0x00); //0xFF
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
  sendchar3(0xFF);	 //0xFF
  sendchar3(0xFF);	 //0xFF
}

void checkVolumeChange(void)
{
  static int lastButton;
  //Buttons
 // if ((lastButton & 0x1C)!= (currentButton & 0x1C))	 //If any of the 3 inputs changed
  if (lastButton != currentButton)
  {	
    //if ( (lastButton&0x4) != (currentButton&0x4) && (currentButton&0x4) != 0x4)
	if ( (lastButton&0x1)  != (currentButton&0x1) && (currentButton&0x1 == 0x1))
    {
       volume++;
    } 
    //else if ( (lastButton&0x8) != (currentButton&0x8 && (currentButton&0x8) !=0x8))
	else if ((lastButton&0x2) != (currentButton&0x2) && ((currentButton&0x2) == 0x2))
    {
      volume--;
    }
    //else if ( (lastButton&0x10) != (currentButton&0x10) && (currentButton&0x10) !=0x10)
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

int main (void) {
#ifdef BASE
int lastVolume=-1;
#endif
SCS |= 1;  //Enable fast GPIO
#ifdef BASE
  volume=5;
   /* Enable and setup timer interrupt, start timer                            */
  //T0MR0         = 11999;                       /* 1msec = 12000-1 at 12.0 MHz */
  //T0MR0         = 299;                       /* 0.025msec = 300-1 at 12.0 MHz */
  
  
  T0MR0         = 99;                       /* 0.025msec = 100-1 at 4.0 MHz LOLCRYSTAL */
  T0MCR         = 3;                           /* Interrupt and Reset on MR0  */
  T0TCR         = 0;                           /* Timer0 Enable             */
  VICVectAddr4  = (unsigned long)T0_IRQHandler;/* Set Interrupt Vector        */
  VICVectCntl4  = 15;                          /* use it for Timer0 Interrupt */
  VICIntEnable  |= (1  << 4);                   /* Enable Timer0 Interrupt     */

  /* Enable and setup timer interrupt (useful for communicating with display)*/
  //T1MR0         = 11999;                       /* 1msec = 12000-1 at 12.0 MHz */
  T1MR0         = 3999;                       /* 1msec = 4000-1 at 4.0 MHz LOLCRYSTAL */
  T1MCR         = 3;                           /* Interrupt and Reset on MR0  */
  T1TCR         = 1;                           /* Timer1 Enable            */
  VICVectAddr5  = (unsigned long)T0_IRQHandler2;/* Set Interrupt Vector        */
  VICVectCntl5  = 16;                          /* use it for Timer1 Interrupt */
  VICIntEnable  |= (1  << 5);                   /* Enable Timer1 Interrupt     */ 
#endif
#ifdef REMOTE
//  PINSEL1       = 0x00204000;                      /* AD0.0 pin function select   */

#endif
//#ifdef BASE  			
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
  #else
  display_init();
  #endif
#endif  

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
  //VICVectCntl6 = 14;                          /* use it for I2S Interrupt    */
  //VICIntEnable  |= (unsigned int)(0x1  << 31);                  /* Enable I2S Interrupt        */






  /*PINSEL4 &= 0xFF9FFFFF;
  FIO2DIR = 1 << 11 ; //pin configured as output
  FIO2CLR = 1 << 11;  */

#ifdef BASE  
  /* Let's initialize the voice recognition pins: p1.19-22	 */

#ifdef DEVBOARD
  // In a perfect world those would be the pins.  But yeah not so much.
  // p2[5-8] = 97,96,95,93
  PINSEL4 &=  0xFFFC03FF;
#else
  //p1[20-24]
  PINSEL3 &=   0xFFFC00FF;
  FIO3DIR = 0;

  //Pushbuttons	p1[15-17]
  PINSEL2 &= 0x3FFFFFFF;
  PINSEL3 &= 0xFFFFFFF0; 
  FIO1DIR &= 0xFFFC7FFF;

  //FIO2DIR not necessary
#endif


  T0TCR         = 1;                           /* Timer0 Enable (Start A-D conversions!)              */
#endif

					//2401 is shut down, 2400 is turn on.
					// 1c03 sets appropriate permissions
					// 0500 should set the potentiometer to 1/4 or something.


 		   
#ifdef REMOTE
//Set pin P3[26] to high
PINSEL4 &= 0x0;
FIO3DIR = 1 << 26 ; //pin configured as output
FIO3SET = 1 << 26;


#endif  
#ifdef BASE
			  		 
 	 
  SPIWRData[0]=0x24;  
  SPIWRData[1]=0x00;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16; 
  
  SPIWRData[0]=0x1C;  		//1C03	inverted is C038
  SPIWRData[1]=0x03;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;
   
  SPIWRData[0]=0x05; //05FF
  SPIWRData[1]=0x40;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;

#endif
  while (1) {                           /* Loop forever                       */
  /*SPIWRData[0]=0x24;  
  SPIWRData[1]=0x01;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;*/	  
#ifdef BASE   	   
/*
  SPIWRData[0]=0x24;  
  SPIWRData[1]=0x00;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16; */
   /* Audio Code*/

   /*Fake Volume*/
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
  if (volume != 11)
  {
  SPIWRData[0]=0x24;  
  SPIWRData[1]=0x00;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16; 
  }			  
  if (volume == 11){			 			
  SPIWRData[0]=0x24; //05FF
  SPIWRData[1]=0x01;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;
  }
  else if (volume == 10)
  {
  SPIWRData[0]=0x05; //05FF
  SPIWRData[1]=0xFF;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;
  }
    else if (volume == 9)
  {
    SPIWRData[0]=0x05; //05FF
  SPIWRData[1]=0x00;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;
  }
    else if (volume == 8)
  {
    SPIWRData[0]=0x05; //05FF
  SPIWRData[1]=0x00;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;
  }
    else if (volume == 7)
  {
    SPIWRData[0]=0x05; //05FF
  SPIWRData[1]=0x00;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;
  }
    else if (volume == 6)
  {
    SPIWRData[0]=0x05; //05FF
  SPIWRData[1]=0x00;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16;
  }
    else if (volume == 5)
  {
    SPIWRData[0]=0x05; //05FF
  SPIWRData[1]=0x40;
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
  //U0IER = 0x00;                       /* Disable RDA interrupts      */
  SPIWRData[0]=0x04;  
  SPIWRData[1]=0x00;
  FIO0CLR = 1 << 16;
  SPISend(SPIWRData, 0x2);
  FIO0SET = 1 << 16; 
  

  }	 
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
#endif	
  }
}
