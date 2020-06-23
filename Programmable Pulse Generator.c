// Name: Surabhi Chythanya Kumar
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz



//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"

#include <stdlib.h>
#include<math.h>

//#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define A7           (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))

#define GREEN_LED_MASK 8
//#define RED_LED_MASK 2

#define MAX_CHARS 80
#define MAX_FIELDS 6

// PortE masks
#define AIN0_MASK 8
#define AIN1_MASK 4

#define PI 3.14
#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")
//#define delay12Cycles() __asm(" NOP\n NOP\n NOP\n NOP\n NOP\n NOP\n NOP\n NOP\n NOP\n NOP\n NOP\n NOP\n")

#define TX_MASK 128
#define FSS_MASK 32
#define CLK_MASK 16
#define GPIO_MASK 128


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
    char str[MAX_CHARS+1];
    uint8_t position[MAX_FIELDS];
    uint8_t argCount=0;
    uint8_t argNo=0;
    uint8_t min_args=0;

    float volt=0;
    uint8_t out;
    uint8_t mode0=0;
    uint8_t mode1=0;
    uint32_t ph_acc0=0;
    uint32_t ph_acc1=0;
    uint32_t delta_ph0=0;
    uint32_t delta_ph1=0;
    float freq=0;
    float f0,f1;
    float amp=0;
    float amp1,amp0;
    float ofs=0;
    float ofs1,ofs0;
    float dtc;
    float dtc1,dtc0;

   uint8_t gainmode=0;
   float freq1=0;
   float freq2=0;
   uint8_t g=0;
   uint16_t rawA,rawB;
   float vA=0,vB=0;
   float gain=0,gainDB=0;

//    bool timeMode = false;
//    uint32_t frequency = 0;
// uint32_t time = 0;

    uint16_t LUT[3][4096];
    uint8_t LUT_OUT1=0;
    uint8_t LUT_OUT2=1;
    uint16_t DACval=0;

    uint8_t num_cyc=0;
    uint16_t count=0;
    uint8_t cyc=0;
    int16_t k=0;

    uint8_t diff=0;
    uint8_t hil=0;
//    typedef struct uart
//    {
//        char str[MAX_CHARS+1];
//            uint8_t position[MAX_FIELDS];
//            uint8_t argCount;
//            uint8_t argNo;
//            uint8_t min_args;
//
//    }data;

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);


      SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB  | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOD;

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
      SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0|SYSCTL_RCGCADC_R1;




    // Configure LED pins
    GPIO_PORTF_DIR_R = GREEN_LED_MASK ;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK ; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK;  // enable LEDs



    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
    delay4Cycles();                                  // wait 4 clock cycles
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;

                                                     // enable TX, RX, and module
    // Configure AIN0 and AIN1 as an analog input
        GPIO_PORTE_AFSEL_R |= AIN0_MASK|AIN1_MASK;                 // select alternative functions for AN3 (PE0)
        GPIO_PORTE_DEN_R &= ~(AIN0_MASK|AIN1_MASK);                  // turn off digital operation on pin PE0
        GPIO_PORTE_AMSEL_R |= AIN0_MASK|AIN1_MASK;                 // turn on analog operation on pin PE0

  /*  // Configure AIN1 as an analog input
        GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AN3 (PE0)
        GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE0
        GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE0*/

        // Configure ADC0
           ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
           ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
           ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
           ADC0_SSMUX3_R = 0;                               // set first sample to AIN3
           ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
           ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

        // Configure ADC1
         ADC1_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
         ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
         ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
         ADC1_SSMUX3_R = 1;                               // set first sample to AIN1
         ADC1_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
         ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation


}



// Request and read one sample from SS3
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}


// Request and read one sample from SS2
int16_t readAdc1Ss3()
{
    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS2 is not busy
    return ADC1_SSFIFO3_R;                           // get single result from the FIFO
}


void initDAC()
{


    // Enable clocks
        SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;

          // Configure A7 for LDAC
               GPIO_PORTA_DIR_R |= GPIO_MASK;                       // make bit 2 an output
             //GPIO_PORTA_DR2R_R |= GPIO_MASK;                      // set drive strength to 2mA
               GPIO_PORTA_DEN_R |= GPIO_MASK;         // enable bit 7 for digital


               // Configure SSI1 pins for SPI configuration
                  GPIO_PORTB_DIR_R |= TX_MASK | FSS_MASK | CLK_MASK; // make SSI1 TX, FSS, and CLK outputs
                 // GPIO_PORTB_DR2R_R |= TX_MASK | FSS_MASK | CLK_MASK; // set drive strength to 2mA
                  GPIO_PORTB_AFSEL_R |= TX_MASK | FSS_MASK | CLK_MASK; // select alternative functions
        //     GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB7_M | GPIO_PCTL_PB5_M | GPIO_PCTL_PB4_M);
                  GPIO_PORTB_PCTL_R= GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB5_SSI2FSS | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI1
                  GPIO_PORTB_DEN_R |= TX_MASK | FSS_MASK | CLK_MASK; // enable digital operation
                  GPIO_PORTB_PUR_R |= CLK_MASK|FSS_MASK;          // SCLK must be enabled when SPO=1 (see 15.4)


      // Configure the SSI1 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
         SSI2_CR1_R &= ~SSI_CR1_SSE;                        // turn off SSI1 to allow re-configuration
         SSI2_CR1_R = 0;                                    // select master mode
         SSI2_CC_R = 0;                                     // select system clock as the clock source
         SSI2_CPSR_R = 10;                                  // set bit rate to 1 MHz (if SR=0 in CR0)
         SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
         SSI2_CR1_R |= SSI_CR1_SSE;                         // turn on SSI1

//         A7=1;
}

void initTimer()
{
    // Enable clocks
         SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
         SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    // Configure Timer 1 as the time base
               TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
               TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
               TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
               TIMER1_TAILR_R = 400;                       // set load value to 40e6 for 1 Hz interrupt rate
               TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
               NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
               TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int len(char str[])
{
    uint8_t i=0,len=0;
    for (i = 0; str[i] != '\0'; i++)
               {
                   len++;
               }
    return len;
}


bool cmp(char str1[], char str2[])
{

    uint8_t i=0;
    while(str1[i]!='\0' || str2[i]!='\0')
    {
        if(str1[i]==str2[i])
        {
            i++;
            continue;
        }
        else

            return false;

    }
    return true;


}
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
}

char* getString(char str[],uint8_t MAX)
{

   int cnt;
   char c;
   for (cnt=0;cnt<MAX;cnt++)

   {
       if(cnt<MAX)
       {
       c=getcUart0();
       if(c==8||c==127)
       {
           if(cnt>0)
               cnt--;
           cnt--;
           continue;
       }
       if(c==10 ||c==13)
       {
           str[cnt]=0;
           break;
       }
       if(c>=' ')
       {
           str[cnt]=c;
       }
       }
       else
       {
           str[cnt]=0;
       break;
       }


 }
   return str;
}

char* parseString(char str[],uint8_t pos[],uint8_t maxField,uint8_t *argCnt)
{
     *argCnt=0;
    int i=0;
    int j=0;
    int len=0;
    for (i = 0; str[i] != '\0'; i++)
           {
               len++;
           }
    //len=strlen(str);

    for ( i=0;i<=len;i++)
    {
        if((str[i]>=48 && str[i]<=57)||(str[i]>=65 && str[i]<=90)||( str[i]>=97 && str[i]<=122)||( str[i]==46)||(str[i]==45))
        {
            if(!((str[i-1]>=48 && str[i-1]<=57)||(str[i-1]>=65 && str[i-1]<=90)||( str[i-1]>=97 && str[i-1]<=122)||( str[i-1]==46)||(str[i-1]==45)))
            {

                pos[j]=i;
                j++;
                (*argCnt)++;
                if (*argCnt==maxField)
                {
                    break;
                }
            }
        }
         else if(!((str[i]>=48 && str[i]<=57)||(str[i]>=65 && str[i]<=90)||( str[i]>=97 && str[i]<=122)||( str[i]==46)||(str[i]==45)))
         {

                   str[i]=0;

         }

}

    min_args=argCount-1;
return str;
}

char* getArgString(char str[],uint8_t posi[],uint8_t argNum)
{

        return &str[posi[argNum]];

}





bool isCommand(char strCmd[],uint8_t minArgs)
{

           if(cmp(strCmd,getArgString(str,position,0)))
           {
               if(minArgs<argCount && minArgs==(argCount-1))
               {
                   return true;
               }
           }
   return false;
}
//reffered from geeksforgeeks
void reverse(char* s, uint8_t l)
{
    uint8_t i=0,j=l-1;
    char temp;
    for(i=0;i<j;i++)
    {
       temp=s[i];
       s[i]=s[j];
       s[j]=temp;
       j--;
    }
}
//reffered from geeksforgeeks
uint8_t itoa(uint16_t n, char s1[], uint8_t d)
{
    uint8_t i=0;
    while(n)
    {
        s1[i++]=n%10+'0';
        n=n/10;
    }
    while(i<d)
    {
        s1[i++]='0';
    }
    reverse(s1,i);
    s1[i]='\0';
    return i;
}
uint16_t pow10(uint8_t x)
{
    uint16_t y=1;uint8_t i;
   for ( i=0 ; i<x;i++)
   {
        y=10*y;
   }
  return y;
}
//referred from geeksforgeeks
void ftoa(float n1,char* res, int d1)
{
    uint8_t integer= (uint8_t) n1;
    float fraction=n1-(float) integer;
    uint8_t i=itoa(integer,res,0);
    if(d1!=0)
    {
        res[i]='.';
    }
    fraction=fraction*pow10(d1);
    itoa((uint16_t)fraction,res+i+1,d1);


}

void sendDACData(uint16_t data)
{

        SSI2_DR_R = data;                  // write data
        while (SSI2_SR_R & SSI_SR_BSY);    // wait for transmission to stop

    A7 =0;
    delay4Cycles();
    A7=1;

}

void setDC(uint8_t chan,float volts)
{
    uint16_t R=0;
    // A7=1;
     if(chan==0)
     {
  R=-386.2661*volts+14310;
       // R=volts;
     sendDACData(R);
    // A7=1;
     }
     if(chan==1)
     {
          R=-386.2661*volts+47077;
       // R=volts;
           sendDACData(R);

     }

}

uint64_t pow2(uint8_t x)
{
    uint64_t z=1;uint8_t i;
   for ( i=0 ; i<x;i++)
   {
        z=2*z;
   }
  return z;
}
//float freqHz(float freq)
//{
//
//    delta_ph=(freq*4294967296)/(100000);
//    ph_acc=ph_acc+delta_ph;
//    return ph_acc;
//}
uint16_t *lookup2()
{
    uint16_t r=0;uint16_t j=0;



               //f0=freq;





    for(j=0;j<4096;j++)
                     {
                         if(mode0==1)
                         {
                           LUT[2][j]=2022-386.2661*(sin((2*PI*j)/4096)*amp0+ofs0);
                         }

                         if(mode0==2)
                         {
                             if(j<(((dtc0/100)*4096)))
                             {
                                 LUT[2][j]=2022-386.2661*(1*amp0+ofs0);
                             }
                             if(j>=((dtc0/100)*4096))
                             {
                                 LUT[2][j]=2022-386.2661*(0*amp0+ofs0);
                             }
                          }
                         if(mode0==3)
                         {

                             //  LUT[2][j]=2022-386.2661*(((0.5 *PI*j)/4096)*(amp-0.6)+ofs)+12288;
                            //   LUT[2][j]=2022-386.2661*(((0.35 *PI*j)/4096)*(amp)+(ofs-0.15))+12288;
                             LUT[2][j]=(((amp0/4096)*j+ofs0)*(-386.2661))+2022;

                         }
                         if(mode0==4)
                               {

                                  if(j<2048)
                                        {
                                        //   LUT[2][j]=2022-386.2661*(sin((2*PI*j)/4096)*amp+ofs)+12288;
                                        //    LUT[2][j]=2022-386.2661*(((0.5*2*j*amp)/2048)+(ofs-1+1))+12288;
                                      LUT[2][j]=(((amp0/2048)*j+ofs0)*(-386.2661))+2022;
                                         }
                                  if(j>=2048)
                                         {
                                        //   LUT[2][j]=2022-386.2661*(sin(1-((2*PI*j)/4096)*amp+ofs))+12288;
                                         //  LUT[2][j]=2022-386.2661*(((-0.5*(2*j)*amp)/2048)+(ofs+3+1))+12288;
                                      LUT[2][j]=(((-amp0/2048)*(j)+4+ofs0)*(-386.2661))+2022;
                                          }
                               }
                     }
    for(r=0;r<4096;r++)
        {
          LUT[0][r]=LUT[2][r];
        }
    for (r=0;r<4096;r++)
             {
                LUT[2][r]=0;
             }


   // f1=freq;


       for(j=0;j<4096;j++)
                        {
                          if(mode1==1)
                          {

                            if(hil==0)
                              {
                              LUT[2][j]=2021-386.2661*(sin((2*PI*j)/4096)*amp1+ofs1);
                              }
                            if(hil==1)
                            {
                                LUT[2][j]=2021-386.2661*(-cos((2*PI*j)/4096)*amp1+ofs1);
                            }

                          }
                          if(mode1==2)
                          {
                              if(j<(((dtc1/100)*4096)))
                               {
                                  LUT[2][j]=2021-386.2661*(1*amp1+ofs1);
                               }
                              if(j>=((dtc1/100)*4096))
                                {
                                   LUT[2][j]=2021-386.2661*(0*amp1+ofs1);
                                }
                           }
                           if(mode1==3)
                            {

                                 //  LUT[2][j]=2022-386.2661*(((0.5 *PI*j)/4096)*(amp-0.6)+ofs)+12288;
                                 //   LUT[2][j]=2021-386.2661*(((0.35 *PI*j)/4096)*(amp)+(ofs-0.15))+45056;
                               LUT[2][j]=(((amp1/4096)*j+ofs1)*(-386.2661))+2021;

                            }
                           if(mode1==4)
                             {

                                 if(j<2048)
                                   {
                                 //   LUT[2][j]=2022-386.2661*(sin((2*PI*j)/4096)*amp+ofs)+12288;
                                   //   LUT[2][j]=2021-386.2661*(((0.5*2*j*amp)/2048)+(ofs-1+1))+45056;
                                     LUT[2][j]=(((amp1/2048)*j+ofs1)*(-386.2661))+2021;
                                   }
                                 if(j>=2048)
                                   {
                                 //   LUT[2][j]=2022-386.2661*(sin(1-((2*PI*j)/4096)*amp+ofs))+12288;
                                 //     LUT[2][j]=2021-386.2661*(((-0.5*(2*j)*amp)/2048)+(ofs+3+1))+45056;
                                     if(diff==0)
                                     {
                                    LUT[2][j]=(((-amp1/2048)*(j)+4+ofs1)*(-386.2661))+2021;
                                     }
                                     if(diff==1)
                                     {
                                     LUT[2][j]=((((-amp1/2048)*j)+ofs1-4)*(-386.2661))+2021;
                                     }

                                   }
                             }
                        }
       for(r=0;r<4096;r++)
            {
              LUT[1][r]=LUT[2][r];
            }
       for (r=0;r<4096;r++)
                {
                   LUT[2][r]=0;
                }



return LUT;
}



void timer1Isr()
{
    uint16_t x;
    if(gainmode==0)
    {
    delta_ph0=(f0*4294967296)/(100000);
    delta_ph1=(f1*4294967296)/(100000);
    }
        if(gainmode==1  && freq1<=freq2)
          {
            f0=0;
            f1=0;
            delta_ph0=(freq1*4294967296)/(100000);

          }
      if(cyc==0)
       {
           ph_acc0=ph_acc0+delta_ph0 ;

                   SSI2_DR_R= LUT[LUT_OUT1][ph_acc0>>20]+12288;
                   //while (SSI2_SR_R & SSI_SR_BSY);    // wait for transmission to stop
                   A7 =0;
                //   delay4Cycles();
                   A7=1;
           ph_acc1=ph_acc1+delta_ph1;
                   SSI2_DR_R= LUT[LUT_OUT2][ph_acc1>>20]+45056;
                  // while (SSI2_SR_R & SSI_SR_BSY);    // wait for transmission to stop
                   A7 =0;
                  // delay4Cycles();
                   A7=1;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
       }
     if(cyc==1)
      {
         if(k>0 )
             {
             if(out==0)
             {
                 ph_acc0=ph_acc0+delta_ph0 ;
                 k--;
                 SSI2_DR_R= LUT[0][ph_acc0>>20]+12288;
                 A7=0;
                 A7=1;
             }
             if(out==1)
             {
                 ph_acc1=ph_acc1+delta_ph1;
                  k--;
                  SSI2_DR_R= LUT[1][ph_acc1>>20]+45056;
                  A7=0;
                  A7=1;
             }
              }

              if(k==0)
                {
                 for (x=0;x<4096;x++)
                  {
                      LUT[out][x]=0;

                  }

                   cyc=0;
                //   k=count;
                   TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
                  TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

                                            //k=count;

                  }
      }

}

int main(void)
{

    // Initialize hardware
    initHw();

    initDAC();
    GREEN_LED = 1;
    waitMicrosecond(500000);
    GREEN_LED = 0;
char strout[MAX_CHARS];
uint16_t raw;
float v=0;

uint16_t x=0;

    while(1)
    {
    // Display greeting
    putsUart0("\r\n");
    putsUart0("Enter any string\r\n");
    putsUart0("\r \n");

    getString(str,MAX_CHARS);
    parseString(str,position,MAX_FIELDS, &argCount);
    //  getArgString(str,position,MAX_FIELDS,argCount,argNo);



      if(isCommand("reset",0))
      {
          NVIC_APINT_R= NVIC_APINT_VECTKEY|NVIC_APINT_SYSRESETREQ |NVIC_APINT_VECT_RESET;
      }

      if(isCommand("voltage",1))
      {
         uint8_t ch=atoi(getArgString(str,position,1));
        if(ch==0)
         {
          raw=readAdc0Ss3();

           v=(3.3*raw)/4096;

         }
         if(ch==1)
          {

               raw=readAdc1Ss3();
               v=(3.3*raw)/4096;

          }

        ftoa(v,strout,4);
        putsUart0("\r \n");
        putsUart0(strout);
      }

      if(isCommand("dc",2))
        {
          //   mode=0;

            uint8_t out=atoi(getArgString(str,position,1));
            if(out==0)
            {
                mode0=0;
            }
            if(out==1)
            {
                mode1=0;
            }
             if(out==0)
             {
             volt=atof(getArgString(str,position,2));

            setDC(out,volt);

            putsUart0("\r \n vout a");
             }
             if(out==1)
             {
                 float volt=atof(getArgString(str,position,2));
                             setDC(out,volt);
                             putsUart0("\r \n vout b");
             }
        }

         if(isCommand("sine",4))
         {

            // mode=1;
             out=atoi(getArgString(str,position,1));


                 freq=atof(getArgString(str,position,2));

                 amp=atof(getArgString(str,position,3));
                 ofs=atof(getArgString(str,position,4));
                 if(out==0)
                  {
                     mode0=1;
                     f0=freq;
                     amp0=amp;
                     ofs0=ofs;

                  }
                 if(out==1)
                   {
                      mode1=1;
                      f1=freq;
                      amp1=amp;
                      ofs1=ofs;

                   }
                 lookup2();

           }



         if(isCommand("run",0))
            {

              initTimer();

             }

         if(isCommand("stop",0))
               {

                       for (x=0;x<4096;x++)
                         {
                           LUT[0][x]=0+12288+2022;
                           LUT[1][x]=0+45056+2021;
                           f0=0;
                           f1=0;

                         }

                       TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

                }

             if(isCommand("square",5))
              {

               //  mode=2;
                 out=atoi(getArgString(str,position,1));

                 freq=atof(getArgString(str,position,2));

                 amp=atof(getArgString(str,position,3));
                 ofs=atof(getArgString(str,position,4));
                 dtc=atof(getArgString(str,position,5));
                 if(out==0)
                    {
                         mode0=2;
                         f0=freq;
                         amp0=amp;
                         ofs0=ofs;
                         dtc0=dtc;
                    }
                 if(out==1)
                    {
                         mode1=2;
                         f1=freq;
                         amp1=amp;
                         ofs1=ofs;
                        dtc1=dtc;
                   }
                 lookup2();

             }

             if(isCommand("sawtooth",4))
             {

            //   mode=3;
              out=atoi(getArgString(str,position,1));

               freq=atof(getArgString(str,position,2));

               amp=atof(getArgString(str,position,3));
               ofs=atof(getArgString(str,position,4));
               if(out==0)
                  {
                      mode0=3;
                      f0=freq;
                       amp0=amp;
                       ofs0=ofs;

                   }
                if(out==1)
                   {
                      mode1=3;
                      f1=freq;
                      amp1=amp;
                      ofs1=ofs;
                   }
               lookup2();

             }

             if(isCommand("triangle",4))
              {

               //  mode=4;
                 out=atoi(getArgString(str,position,1));


                 freq=atof(getArgString(str,position,2));

                 amp=atof(getArgString(str,position,3));
                 ofs=atof(getArgString(str,position,4));
                 if(out==0)
                  {
                      mode0=4;
                      f0=freq;
                      amp0=amp;
                      ofs0=ofs;

                  }
                if(out==1)
                  {
                      mode1=4;
                      f1=freq;
                      amp1=amp;
                      ofs1=ofs;

                  }
                 lookup2();
               }

             if(isCommand("cycles",2))
             {
               cyc=1;
                 out=atoi(getArgString(str,position,1));
                 num_cyc=atoi(getArgString(str,position,2));
               //  count=((((((100000*num_cyc)-6)/freq))/8)/4)/2;
                // count=(((((100000*num_cyc)-6)/freq))/4)+1;
                 count=((100000*num_cyc)/freq);
                k=count;
             }

             if(isCommand("gain",2))
              {

                  gainmode=1;
                   freq1=atof(getArgString(str,position,1));
                  freq2=atof(getArgString(str,position,2));

                             while(freq1<=freq2)
                             {
                             freq1=freq1+(10*g);
                            // lookup2();
                                 g++;
                              if(freq1>freq2)
                              {
                                  g=0;
                                  gainmode=0;
                              }
                                rawA=readAdc0Ss3();
                                vA=(3.3*rawA)/4096;



                                rawB=readAdc1Ss3();
                                vB=(3.3*rawB)/4096;

                                waitMicrosecond(50000);

                                gain= vB/vA;
                                gainDB =20*log10(gain);

                                ftoa(freq1,strout,4);
                                putsUart0("\r \n");
                                putsUart0(strout);

                                ftoa(gain,strout,4);
                                putsUart0("\t \t");
                                putsUart0(strout);


                             }

                         }
             if(isCommand("alc",1))
                          {

                           if( cmp("on",getArgString(str,position,1)))
                           {
                             if(out==0)
                              {
                                 rawA=readAdc0Ss3();
                                 vA=(3.3*rawA)/4096;
                                 ftoa(vA,strout,4);
                                 putsUart0("\r \n");
                                 putsUart0("vout a");
                                 putsUart0("\r \n");
                                 putsUart0(strout);

                              }


                             if(out==1)
                             {
                              rawB=readAdc1Ss3();
                              vB=(3.3*rawB)/4096;

                              ftoa(vB,strout,4);
                              putsUart0("\r \n");
                              putsUart0("vout b");
                              putsUart0("\r \n");
                              putsUart0(strout);
                             }



                           }

                            if( cmp("off",getArgString(str,position,1)))
                             {
                                if(out==0)
                                {
                                    rawA=readAdc0Ss3();
                                    vA=(3.3*rawA)/4096;

                                    ftoa(vA,strout,4);
                                    putsUart0("\r \n");
                                    putsUart0("vout a");
                                    putsUart0("\r \n");
                                    putsUart0(strout);

                                }

                                 if(out==1)
                                 {
                                 rawB=readAdc1Ss3();
                                 vB=(3.3*rawB)/4096;

                                 // waitMicrosecond(50000);

                                  ftoa(vB,strout,4);
                                  putsUart0("\r \n");
                                  putsUart0("vout b");
                                  putsUart0("\r \n");
                                  putsUart0(strout);
                                 }
                               }

                         }
             if(isCommand("diff",1))
             {

                 if( cmp("on",getArgString(str,position,1)))
                       {
                     diff=1;
                         if(out==0)
                         {
                             amp1=-amp0;
                             ofs1=ofs0;
                             f1=f0;
                             ph_acc1=ph_acc0+delta_ph0;
                             if(mode0==1)
                             {
                                 mode1=1;
                                 lookup2();
                             }
                             if(mode0==2)
                             {
                                 dtc1=dtc0;
                                  mode1=2;
                                  lookup2();
                             }
                             if(mode0==3)
                               {
                                   mode1=3;
                                   lookup2();
                               }
                             if(mode0==4)
                              {
                                 ofs1=ofs0;
                         //        amp1=-amp0;
                               //  ph_acc1=ph_acc0+(1*delta_ph0);
                                   mode1=4;
                                  lookup2();
                              }

                         }
                       }
                 if( cmp("off",getArgString(str,position,1)))
                 {
                     if(diff==1)
                     {
                     diff=0;
                     amp1=0;
                     ofs1=0;
                     f1=0;
                     dtc1=0;
                     mode1=0;
                     }
                 }
             }
             if(isCommand("hilbert",1))
               {
                   if( cmp("on",getArgString(str,position,1)))
                      {
                       hil=1;
                       if(out==0)
                       {
                           amp1=amp0;
                           ofs1=ofs0;
                           f1=f0;
                           delta_ph1=delta_ph0;
                           ph_acc1=ph_acc0+delta_ph0;
                           if(mode0==1)
                           {
                               mode1=1;

                               lookup2();
                           }
                       }
                      }
                   if( cmp("off",getArgString(str,position,1)))
                       {
                             if(hil==1)
                               {
                                  hil=0;
                                  amp1=0;
                                  ofs1=0;
                                  f1=0;
                                  mode1=0;
                                }
                         }
                  }



     if(!(isCommand("reset",0) || isCommand("voltage",1) || isCommand("dc",2) || isCommand("sine",4) || isCommand("run",0) || isCommand("stop",0) || isCommand("square",5) || isCommand("sawtooth",4) || isCommand("triangle",4) || isCommand("cycles",2) || isCommand("gain",2) || isCommand("diff",1) || isCommand("hilbert",1) || isCommand("alc",1)))
     {
         putsUart0("\r \n invalid cmd");
     }
    }


    return 0;
}







