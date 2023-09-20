//  ==========================================================================================
//  main.c for STM32F030-IR-Remote
//  ------------------------------------------------------------------------------------------
//  Universal Remote Capture and Playback using CMSIS (no HAL)
//  ------------------------------------------------------------------------------------------
//  https://github.com/EZdenki/STM32F030-CMSIS-IR-Remote
//  Released under the MIT License
//  Copyright (c) 2023
//  Mike Shegedin, EZdenki.com
//  Version 1.0   4 Aug 2023    Started
//  ------------------------------------------------------------------------------------------
//  Target Microcontroller and Devices:
//    * STM32F030Fxxx
//    * 24LC64 I2C 64 Kbit EEPROM with 5.6K pullup resistors on pins 18-SDA and 17-SCL
//    * TSOP4838 38 KHz IR Sensor -- TIM3 for input capture to read pulse stream on
//      the output pin of the sensor.
//    * IR333A Infrared LED with 2N7000 MOSFET driver -- TIM14 for PWM (38 kHz) output and
//      TIM16 for individual pulse timing.
//    * Status LED on pin 2-PF0
//    * Tactile Button and pulldown resistor for reset button on NRST pin 4
//    * Tactile buttons for: Prog/Pin 3/PF1, B1/Pin 6/PA0, B2/Pin 7/PA1, B3/Pin 10/PA4
//                           B4/Pin 11/PA5, B5/Pin 13/PA7
//    * USB Serial Dongle (for Debugging) on pins 8-TX / 9-RX
//    
//  ------------------------------------------------------------------------------------------
//  Hardware Setup:
//    Wire an LED and 1k resistor between pin 6 and ground.
//    Connect the four pins from the ST-Link V2 as shown below:
//
//
//                                        STM32F030F4xx     
//                                         ╭────╮╭────╮
//                                   BOOT0 │1       20│ SWCLK -- [SWCLK│ST-Link V2]
//    GND -- [1K] -- [-Status LED+] -- PF0 │2       19│ SWCLK -- [SWDIO│          ]
//             GND -- [Prog Button] -- PF1 │3       18│ PA10:I2C1_SDA [SDA|24LC64]
//  GND -- [10K] -- [Reset Button] -- NRST │4       17│ PA9: I2C1_SCL [SCL|      ]
//                                    VDDA │5       16│ VCC -- VCC
//               GND -- [B1 Button] -- PA0 │6       15│ GND -- GND
//               GND -- [B2 Button] -- PA1 │7       14│ PB1 TIM14_CH1(AF0) [IR333 LED]
//      [USB Serial|RX] USART1_TX(AF1) PA2 │8       13│ PA7 -- [B5 Button] -- GND
//      [Dongle    |TX] USART1_RX(AF1) PA3 │9       12│ PA6 TIM3_CH1(AF1) [IR Photodiode]
//               GND -- [B3 Button] -- PA4 │10      11│ PA5 -- [B4 Button] -- GND
//                                         ╰──────────╯
//
//  ==========================================================================================

#include "stm32f030x6.h"
#include "STM32F030-CMSIS-USART-lib.c"
#include "STM32F030-CMSIS-I2C-EEPROM-lib.c"

//  Global Defines
#define MAX_PULSE_CNT 126   // Maximum number of pulses to scan and write to EEPROM. Note that
                            // there is a hard limit due to the fact that only 256 bytes can
                            // be written to or read from the EEPROM in a single read/write
                            // statement. Initially at least, all reads/writes will be 16-bit
                            // (2-byte) words. So if the the first word is the count, that
                            // leaves 127 words remaining, which is word[0] to word[126].
                            // For this reason, the hard limit for MAX_PULSE_CNT is 126.


//  Macros to turn on/off Status and Program LEDs
#define StatusLEDON()  GPIOF->ODR |=  GPIO_ODR_0
#define StatusLEDOFF() GPIOF->ODR &= ~GPIO_ODR_0
#define StatusLEDTog() GPIOF->ODR ^=  GPIO_ODR_0
//  Macros to poll Program and Send buttons
#define DEBOUNCE_DELAY 5000    // Number of microseconds for button debounce

// #define ProgButON() !(GPIOA->IDR & GPIO_IDR_4)


//  uint32_t ProgON( void )
//  uint32_t BxON( void )
//  Polls the Program or Bx buttons and returns 1 if pressed or 0 if not being pressed.
//  Does some super-simple debounce via a delay.
uint32_t
ProgON( void )
{
  if(  (GPIOF->IDR & GPIO_IDR_1 ))
    return 0;
  delay_us( DEBOUNCE_DELAY );
  if(  (GPIOF->IDR & GPIO_IDR_1 ))
    return 0;
  return 1;
}

uint32_t
B1ON( void )
{
  if(  (GPIOA->IDR & GPIO_IDR_0 ))
    return 0;
  delay_us( DEBOUNCE_DELAY );
  if(  (GPIOA->IDR & GPIO_IDR_0 ))
    return 0;
  return 1;
}

uint32_t
B2ON( void )
{
  if(  (GPIOA->IDR & GPIO_IDR_1 ))
    return 0;
  delay_us( DEBOUNCE_DELAY );
  if(  (GPIOA->IDR & GPIO_IDR_1 ))
    return 0;
  return 1;
}

uint32_t
B3ON( void )
{
  if(  (GPIOA->IDR & GPIO_IDR_4 ))
    return 0;
  delay_us( DEBOUNCE_DELAY );
  if(  (GPIOA->IDR & GPIO_IDR_4 ))
    return 0;
  return 1;
}

uint32_t
B4ON( void )
{
  if(  (GPIOA->IDR & GPIO_IDR_5 ))
    return 0;
  delay_us( DEBOUNCE_DELAY );
  if(  (GPIOA->IDR & GPIO_IDR_5 ))
    return 0;
  return 1;
}

uint32_t
B5ON( void )
{
  if(  (GPIOA->IDR & GPIO_IDR_7 ))
    return 0;
  delay_us( DEBOUNCE_DELAY );
  if(  (GPIOA->IDR & GPIO_IDR_7 ))
    return 0;
  return 1;
}


//  uint32_t butScan( void )
//  Scan Program and Send buttons. Returns 0 if no button pressed, 99 if Program button
//  pressed, or else the number of the send button from 1 to 5.
uint32_t
ButScan( void )
{
  if( !(GPIOA->IDR & GPIO_IDR_0) )
  {
    delay_us( DEBOUNCE_DELAY );
    if( !(GPIOA->IDR & GPIO_IDR_0) )
      return 1;                           // Button 1 pressed
  }
  else if( !(GPIOA->IDR & GPIO_IDR_1) )
  {
    delay_us( DEBOUNCE_DELAY );
    if( !(GPIOA->IDR & GPIO_IDR_1) )
      return 2;                           // Button 2 pressed
  }
  else if( !(GPIOA->IDR & GPIO_IDR_4) )
  {
    delay_us( DEBOUNCE_DELAY );
    if( !(GPIOA->IDR & GPIO_IDR_4) )
      return 3;                           // Button 3 pressed
  }
  else if( !(GPIOA->IDR & GPIO_IDR_5) )
  {
    delay_us( DEBOUNCE_DELAY );
    if( !(GPIOA->IDR & GPIO_IDR_5) )
      return 4;                           // Button 4 pressed
  }
  else if( !(GPIOA->IDR & GPIO_IDR_7) )
  {
    delay_us( DEBOUNCE_DELAY );
    if( !(GPIOA->IDR & GPIO_IDR_7) )
      return 5;                           // Button 5 pressed
  }
  else if( !(GPIOF->IDR & GPIO_IDR_1) )
  {
    delay_us( DEBOUNCE_DELAY );
    if( !(GPIOF->IDR & GPIO_IDR_1) )
      return 99;                          // Program button pressed
  }
  return 0;   // No button pressed or debounce fault
}



//  void ButtonLightTest( void )
//  Short routine to test button and indicator LED functionality. Pressing the Program button
//  should light the Program LED and pressing the Send button should light the Status LED.
void
ButtonLightTest( void )
{
  while( 1 )
  {
    uint32_t butNum = ButScan();
    if( butNum )
    {
      if( butNum == 99 )
        butNum = 6;
      for( uint32_t x = 0; x< butNum; x++ )
      {
        StatusLEDON();
        delay_us(50E3);
        StatusLEDOFF();
        delay_us(50E3);
      }
      delay_us(2e6);
    }
  }
}

#define IRPulseON()  TIM14->CNT = 0; TIM14->CCMR1 |=  TIM_CCMR1_OC1M_1
#define IRPulseOFF() TIM14->CNT = 0; TIM14->CCMR1 &= ~TIM_CCMR1_OC1M_1
#define IRPulseTog() TIM14->CNT = 0; TIM14->CCMR1 ^=  TIM_CCMR1_OC1M_1
//  void IRLEDSetup( void )
//  Set up PWM output for IR LED (pin 14, PB1/TIM14_CH1(AF0) at 38 KHz at a 50% duty cycle.
//  After setup, there will be no output. Use the following above-defined macros to start/stop
//  the output:  IRPulseON()    -- start modulation
//               IRPulseOFF()   -- stop modoulation, making sure output is LOW
//  Also set up TIM16 to time individual pulses of modulated output.

void
IRLEDSetup( void )
{
  RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;   // Enable GPIO Port B
  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;  // Enable TIM14
  GPIOB->MODER |= GPIO_MODER_MODER1_1;  // Set GPIO B1 as alternate function
  // Don't need to set GPIOA->AFR[0] because AF0 is default alternate function
  
  // CCMR1:OC1M[2:0] bits -> 0b100 = inactive (output forced low)
  //                         0b110 = PWM mode 1
  // Therefore, by changing the OC1M[1] bit between 0 and 1, we can turn ON and OFF PWM,
  // making sure that when the PWM is OFF, the output pin is LOW. Start wit PWM OFF.
  TIM14->CCMR1 |= (0b100 << TIM_CCMR1_OC1M_Pos);  // TIM14/Ch1 output compare PWM mode 1
  TIM14->PSC    = 0;                    // Set prescaler to 0 (1)
  TIM14->ARR    = 210;                  // Set Auto Reload Register to 210 (period=ARR+1)
  TIM14->CCR1   = 105;                  // Set duty cycle to 104 (50%).
  TIM14->CCER  |= TIM_CCER_CC1E;        // Associate timer compare with GPIO output pin
  TIM14->CR1   |= TIM_CR1_CEN;          // Enable IR LED modulation timer
  IRPulseOFF();                         // Force modulation OFF for now
  
  // Set up TIM16 used to time modulation pulses
  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;  // Enable TIM16. (Don't need GPIO for this timer!)
  TIM16->PSC    = 79;                   // Same prescaler used in capture
  TIM16->ARR    = 1;                    // Overflow the timer once to enable the prescaler
  TIM16->CR1   |= TIM_CR1_CEN;          // Start the timer
}

void
IRLEDSendStream( uint16_t pulseStream[] )
{

  TIM16->ARR    = pulseStream[1];       // Set 1st pulse time
  TIM16->CNT    = 1;                    // Clear counter (just for beginning of 1st pulse)
  TIM16->SR    &= ~TIM_SR_UIF;          // Clear overflow flag
  StatusLEDON();                        // Turn on modulation
  IRPulseON();
  while( !( TIM16->SR & TIM_SR_UIF) ) ; // Wait for end of pulse time
  uint32_t psCnt = pulseStream[0];

  for( uint32_t x = 2; x<psCnt; x++ )
  {
    TIM16->ARR    =  pulseStream[x];                 // Set new pulse time
    TIM16->SR    &= ~TIM_SR_UIF;          // Clear overflow flag
    StatusLEDTog();                       // Toggle modulation
    IRPulseTog();
    while( !( TIM16->SR & TIM_SR_UIF) ) ; // Wait for end of pulse time
  }


  StatusLEDOFF();                       // Turn off modulation
  IRPulseOFF();
}


//  void InputCaptureSetup( void )
//  Set up Timer Capture on pin 12, PA6/TIM3_CH1(AF1) to capture IR input from photodiode
//  Uses Input Capture on Single Channel, Capturing on both Rising/Falling Edges
void
InputCaptureSetup( void )
{
  
  // Enable GPIO Port A (Done above)
  // Set GPIO Mode as alternate function
  GPIOA->MODER |= (0b10 << GPIO_MODER_MODER6_Pos);
  
  // Set GPIO A6 as alternate function 1 (Use register name: AFR[0] -- used to be AFRL)
  GPIOA->AFR[0]  |= (0b0001 << GPIO_AFRL_AFSEL6_Pos);
	
  // Enable Timer 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  
  // Configure capture compare channel 1 as input, and map IC1 to TI1
  TIM3->CCMR1  |= (0b01 << TIM_CCMR1_CC1S_Pos);
  
  // CC1E=1: Enable capture compare channel 1.
  // CC1NP=1 and CC1P=1: Set capture compare channel 1 to trigger on TIxFP1 rising and
  // falling edges.
  TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NP | TIM_CCER_CC1P;

  // Set Timer 3 prescaler to generate capture values of desired range.
  // A ÷80 prescaler (PSC=79) with an 8 MHz clock yields a 10 μs counter.
	TIM3->PSC	 = 79;

  // Set the maximum number of counts before overflowing and generating the Update Interrupt
  // Flag (UIF).
  TIM3->ARR = 1000;

	// Enable/Start Timer 3
	TIM3->CR1    |= TIM_CR1_CEN;
}


//  void GetNewPulseStream( uint16_t pulseStream[] )
//  Waits for an IR pulse stream from a remote control and then reads in all of the pulse
//  lengths using the timer input-capture functionality. Stops receiving after a timeout or
//  when the maximum number of pulses has been read in. The routine is passed a pointer to
//  an array of 16-bit integers used to hold the pulse stream. The first 16-bit word contains
//  the number of pulses in the stream.

void
GetNewPulseStream( uint16_t pulseStream[] )
{
  TIM3->SR &= ~(TIM_SR_CC1IF ) ;            // Clear capture compare interrupt flag
		
  while( ! (TIM3->SR & TIM_SR_CC1IF) ) ;    // Wait for initial edge (beginning of first
                                              // pulse).
  TIM3->CNT = 0;                            // Reset timer counter
  TIM3->SR &= ~(TIM_SR_CC1IF | TIM_SR_UIF) ;   // Clear capture and overcapture flags
  
  for( uint8_t x=1; x<MAX_PULSE_CNT; x++ )
  {
	  while( ! ((TIM3->SR & TIM_SR_CC1IF) | (TIM3->SR & TIM_SR_UIF)) ) ;    // Wait for next edge (end of 1st pulse, and
    TIM3->CNT = 0;                            // Reset main timer counter
    pulseStream[x] = TIM3->CCR1;              // Save pulse length and reset capture flag
    if( TIM3->SR & TIM_SR_UIF )
    {
      pulseStream[0] = x;                // Store count in first word of pulse stream
      x = MAX_PULSE_CNT;                      // This gets us out of the loop
    }
  }
}


void
SavePulseStream( uint16_t pulseStream[], uint32_t program )
{
  EE24_write( 256 * program, (uint8_t *) pulseStream, MAX_PULSE_CNT*2, 0);
}


uint32_t
LoadPulseStream( uint16_t pulseStream[], uint32_t program )
{
  return EE24_read( 256 * program, (uint8_t *) pulseStream, MAX_PULSE_CNT*2 );
}


void
DispPulseStream( uint16_t pulseStream[] )
{
  if( (pulseStream[0] != 0) && (pulseStream[0] <= MAX_PULSE_CNT) )
  {
    for( uint8_t x=0; x<pulseStream[0]; x++ )
    {
      USART_puts( "Pulse " );
      if( x<10 )
        USART_putc(' ');
      USART_puti( x, 10 );
      USART_puts( ": " );
      if( pulseStream[x]<100 )
        USART_putc(' ');
      USART_puti( pulseStream[x], 10 );
      USART_puts( "  " );
      if(!( (x+1) & 3) )
        USART_putc( '\n' );
    }
    USART_puts("\n\n");
  }
}


void
ClearPulseStream( uint16_t pulseStream[] )
{
  for( uint32_t x=0; x<MAX_PULSE_CNT; x++ )
    pulseStream[x] = 0;
}

//  ==========================================================================================
//  main
//  ==========================================================================================
int
main( void )
{
  uint16_t pulseStream[MAX_PULSE_CNT];    // Array holding a single IR pulse stream

  // Set up Serial IO via USART1
  USART_init( USART1, 115200 );

  // Set up EEPROM library
  EE24_init( I2C1, 100e3, 0x50, 0x2000, 32 );

  // Set up Status LED on PF0 (pin 2)
  RCC->AHBENR |= RCC_AHBENR_GPIOFEN; 
  GPIOF->MODER |= ( 0b01 << GPIO_MODER_MODER0_Pos );

  // Set up Program and Bx buttons
  GPIOF->PUPDR |= (0b01 << GPIO_PUPDR_PUPDR1_Pos);
  GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPDR0_Pos) | (0b01 << GPIO_PUPDR_PUPDR1_Pos) |
                  (0b01 << GPIO_PUPDR_PUPDR4_Pos) | (0b01 << GPIO_PUPDR_PUPDR5_Pos) |
                  (0b01 << GPIO_PUPDR_PUPDR7_Pos);

  // Set up PWM for output LED
  IRLEDSetup();

  // Set up TIM3 for input capture for IR receiver
  InputCaptureSetup();

  USART_puts( "Setup done until InputCaptureSetup!\n" );


//  ------------------------------------------------------------------------------------------
//  Timing Diagram
//
//  --------,                  .----------,   ,---,   ,------,   ,--/ /--,   ,---------------
//          |       Pre-A      |  Pre-B   | 1 | 4 | 5 |  6   | 7 |  \ \  | n |    Timeout(*)
//          |                  |          |   |   |   |      |   |  / /  |   |
//          '------------------'          '---'   '---'      '---'  \ \  '---'
//  Trig->  *                  *          *   *   *   *      *   *  / /  *   *   Ch2-Overflow
//
// (*) Note that timeout may end on high or low pulse
//
//  ------------------------------------------------------------------------------------------
 


  USART_puts( "\nPress Send or Prog button ...\n");  
  while( 1 )                                  // *** Main Program Loop ***
  {	
    uint32_t butNum = ButScan();
    if( butNum == 99 )                        // Process Program button
    {
      while( !(GPIOF->IDR & GPIO_IDR_1) ) ;   // Wait 'till Program button released
      USART_puts("\nProgram Button Pressed.\n Press button on IR remote...\n");
      StatusLEDON();
      butNum = 0;
      uint32_t flashCnt = 0;
      while( butNum == 0 )                    // Flash Status LED while polling the buttons
      {
        butNum = ButScan();
        if( flashCnt++ > 8E3 )
        {
          StatusLEDTog();
          flashCnt = 0;
        }
      }
      if( butNum < 6 )
      {
        StatusLEDON();
        GetNewPulseStream( pulseStream );
        USART_puts("\n Saving to pulse Stream for button: ");
        USART_puti( butNum, 10 );
        
        SavePulseStream( pulseStream, butNum-1 );
        USART_puts(".\nSaved pulse stream!\n");
        StatusLEDOFF();
        USART_puts("\n New pulse stream saved to EEPROM\n");
        DispPulseStream( pulseStream);
        USART_puts( "\nPress Send or Prog button ...\n");  
      }
      else
      {
        while( !(GPIOF->IDR & GPIO_IDR_1) ) ;   // Wait 'till Program button released
        butNum = 0;      // Probably hit Program button to escape program sequence
        StatusLEDOFF();
      }
    }
    else if( butNum>0 )
    {                             // Send programmed pulse stream
      LoadPulseStream( pulseStream, butNum-1 );
      if( pulseStream[0] == 0 )
        for(uint32_t x=0; x<6; x++ )
        {
          StatusLEDTog();
          delay_us( 200E3 );
        }
      else
      {
        IRLEDSendStream( pulseStream );
        StatusLEDON();
        USART_puts("\nSend Button Pressed. Loading pulse stream from EEPROM.\n");
        DispPulseStream( pulseStream );
        StatusLEDOFF();
        USART_puts( "\nPress Send or Prog button ...\n");
      }
    }
  }   // End of main while loop
} // End of main()
