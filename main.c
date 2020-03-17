/* 
  Will Song and Launnie Ginn
  BEE 425 Function Generator
  Winter 2020
  This program controls the AD9837 programmable waveform generator using: 
   - Tiva C Series TM4C123GH6PM Microcontroller
   - KY-040 Rotary Encoder Switch to control frequency adjustment
     * Turns increment/decrement frequency
     * Switch toggles between 1 Hz and 1 kHz increments
   - EG4208A Switch to select Square or Sine/Triangle wave 
   - A11JV Switch to select Sine or Triangle wave
  
  The function generator uses external components to implement the following specifications:
  Output voltage range: 0 to +/- 5 V
  Frequency range: 10 Hz to 100 kHz
*/ 

#include "AD9837.h"	        // AD9837 definitions.
#include "tm4c123gh6pm.h"       // MCU definitions.
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/lcd.h"
#include "driverlib/sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include <stdio.h>
#include <math.h>
#include "driverlib/cpu.h"
#include "inc/Communication.h"
#include "string.h"

/* 
******************************************
 ADWaveform Pin Definitions
 Outputs sine, triangle, or square wave
 PA2->(CLK)
 PA3->(FSYNC)
 PA5->(SDATA) 
****************************************** 
*/ 
//#define WAVE_CLK_PERIPH SYSCTL_PERIPH_GPIOA
#define WAVE_CLK_BASE GPIO_PORTA_BASE
#define WAVE_CLK GPIO_PIN_2   
//#define WAVE_FSYNC_PERIPH SYSCTL_PERIPH_GPIOA
#define WAVE_FSYNC_BASE GPIO_PORTA_BASE
#define WAVE_FSYNC GPIO_PIN_3   
//#define SDATA_PERIPH SYSCTL_PERIPH_GPIOA
#define SDATA_BASE GPIO_PORTA_BASE
#define SDATA GPIO_PIN_5   

/* 
******************************************
  KY-040 Rotary encoder pin definitions
  Controls frequency and granularity
  pinA -> PF3;  --Connected to CLK (A) 
  pinB -> PF4;  --Connected to DT  (B) 
  pinC -> GND
  FREQ_SW -> PF1 --Connected to SW
****************************************** 
*/ 
// PIN A -- CLK PIN Definitions
#define PINA_BASE GPIO_PORTF_BASE
#define PINA GPIO_PIN_3 
#define PINA_INT GPIO_INT_PIN_3

// PIN B -- DT PIN Definitions
#define PINB_BASE GPIO_PORTF_BASE
#define PINB GPIO_PIN_4   

// FREQ_SW -- Encoder Switch PIN Definitions
#define FREQ_SW_BASE GPIO_PORTF_BASE
#define FREQ_SW GPIO_PIN_1  
#define FREQ_SW_INT GPIO_INT_PIN_1

// EG4208A: Square <--> Sine/Triange Wave Switch Definitions
#define SQR_WAVE_BASE GPIO_PORTA_BASE
#define SQR_WAVE GPIO_PIN_7

// A11JV: Sine <--> Triange Wave Switch Definitions
#define TRI_SIN_BASE GPIO_PORTC_BASE
#define TRI_SIN GPIO_PIN_6

// Onboard LED pin definitions for debugging
#define LED_BASE GPIO_PORTF_BASE
#define BLUE_LED GPIO_PIN_2    

// Define waveform defaults for setup 
#define WAVE_SETUP AD9837_OUT_SINUS 
#define FREQ_SETUP AD9837_FSEL0
#define PHASE_SETUP AD9837_PSEL0

// Define waveform register values to write to
#define FREQ_REG AD9837_REG_FREQ0  
#define PHASE_REG AD9837_REG_PHASE0 

// Define selectable waveforms
#define SINE AD9837_OUT_SINUS
#define SQUARE AD9837_OUT_MSB
#define TRIANGLE AD9837_OUT_TRIANGLE

// Define frequency parameters (in Hz)
#define FREQ_DEFAULT 10000  // Hz
#define FREQ_MIN 10 // Hz
#define FREQ_MAX 100000  // Hz


/* 
******************************************
// Global variables 
****************************************** 
*/ 

// Switch Toggle variable initialization
uint32_t value = 0;
uint8_t state = 0;
volatile uint32_t g_ui32Counter = 0;

//Encoder variable initialization
uint32_t freqPosCount = FREQ_DEFAULT;

// encoder interrupt variables
bool encoderDebounce = false;
void encoderInterrupt (void);
void SysTickIntHandler(void);

void GPIO_Init() { // Configures the GPIO
  // Enable peripherals that we will need for the application 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);   
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  
  SysCtlDelay(33); // wait until ready
  
  // Configure encoder SW pin to a input with internal pull-up  
  GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, FREQ_SW | PINA | PINB);
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, SQR_WAVE);
  GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, TRI_SIN);
  
  //GPIOPadConfigSet(FREQ_SW_BASE, FREQ_SW, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
  GPIOPadConfigSet(FREQ_SW_BASE, FREQ_SW, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  
  // Configures the Blue LED to output
  GPIOPinTypeGPIOOutput(LED_BASE, BLUE_LED);
 
  // ***Interrupt Enable for Encoder CLK GPIO.****
  GPIOIntEnable(PINA_BASE,PINA_INT);
  GPIOIntRegister(PINA_BASE, &encoderInterrupt);
  
 // ***Interrupt Enable for Encoder Switch ****
//  GPIOIntEnable(FREQ_SW_BASE,FREQ_SW_INT);
//  GPIOIntRegister(FREQ_SW_BASE, &encoderInterrupt);  

  // SPI is enabled and defined in the SPI library 
}

void LCD_output() {
    // Encoder toggles state on and off
  value = GPIOPinRead(FREQ_SW_BASE,FREQ_SW);
  if((value & GPIO_PIN_1) == 0){
    state = state ^ BLUE_LED;  // state is HIGH only if either is HIGH -- not both
  }
  GPIOPinWrite(LED_BASE,BLUE_LED, state); // Toggle to onboard LED for debugging 
  SysCtlDelay(10); //cheap debounce
  
    // LCD Menu Output for Function Generator (rough draft)
  // Display Frequency Increment Amount: (0) Hz or (2) KHz
  //          (0) Hz:   encoder dial is incrementing +/- 1 Hz
  //          (4) KHz:  encoder dial is incrementing +/- 1 KHz                
  //
  // Convert state from int to char and output to LCD 
  char state_char[10];
  sprintf(state_char,"%d",state); // :D
  
  // Output toggled state to LCD 
  LCD_PrintJustify(0, "Adj 0-Hz 4-KHz:", state_char); 
  //SysCtlDelay(33); //cheap debounce
  
  // Display Frequency in Hz
  // Convert state from int to char and output to LCD
  char freqPosCount_char[11];
  sprintf(freqPosCount_char,"%d",freqPosCount); // :D
  // Output frequency in Hz to LCD    
  LCD_Cursor(1,0);
  LCD_Puts("Freq(Hz): ");
  switch(strlen(freqPosCount_char)){
  case 6:
    LCD_Cursor(1,10);
    LCD_Puts(freqPosCount_char);     
    break;
  case 5:
    LCD_Cursor(1,10);
    LCD_Puts(freqPosCount_char);     
    LCD_Cursor(1,15);
    LCD_Puts(" ");
    break;
  case 4:
    LCD_Cursor(1,10);
    LCD_Puts(freqPosCount_char);   
    LCD_Cursor(1,14);
    LCD_Puts("  ");
    break;
  case 3:
    LCD_Cursor(1,10);
    LCD_Puts(freqPosCount_char); 
    LCD_Cursor(1,13);
    LCD_Puts("  ");    
    break;    
  case 2:
    LCD_Cursor(1,10);
    LCD_Puts(freqPosCount_char);     
    LCD_Cursor(1,12);
    LCD_Puts("    ");
    break;   
    }
  
}

void waveform_control(){
    int square_wave = GPIOPinRead(SQR_WAVE_BASE,SQR_WAVE); // Read square wave switch
    int sine_triangle = GPIOPinRead(TRI_SIN_BASE,TRI_SIN); // Read sine_triangle switch
    
    // EG4208A switch controls Square or Sine_Triangle:
    if (!square_wave){ // square wave switch is LOW
        AD9837_SetWave(SQUARE); // enable square wave 
    }
    else { // square wave switch is HIGH
        // A11JV switch controls Sine or Triangle
        if (sine_triangle){// sine_triangle switch HIGH
            AD9837_SetWave(SINE); // enable sign wave
        }
        else {// sine_triangle switch LOW
            AD9837_SetWave(TRIANGLE); // enable triangle wave
        } 
    }
  }

int main() //  Main Function 
{
  /////////////////////////////////////////////////////////////////
  // Set the clocking to run directly from the internal oscillator
  ////////////////////////////////////////////////////////////////
//#if defined(TARGET_IS_TM4C129_RA0) ||                                   \
//    defined(TARGET_IS_TM4C129_RA1) ||                                      \
//    defined(TARGET_IS_TM4C129_RA2)
//      ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
//                                         SYSCTL_OSC_MAIN |
//                                           SYSCTL_USE_OSC), 25000000);
//#else // This is where we define our clock speed -> using internal oscillator right now
      // the internal clock is 16 MHz (65.2 ns per clock cycle)
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_INT);
//#endif
      //
    // Initialize the interrupt counter.
    //
    g_ui32Counter = 0;

  SysTickPeriodSet(SysCtlClockGet()/200); // 5 ms system tick
  SysTickIntRegister(&SysTickIntHandler);
  IntMasterEnable();
  SysTickIntEnable();
  SysTickEnable();

  /////////////////////////////////////////////////////
  //                 Initialize Peripherals
  /////////////////////////////////////////////////////
  GPIO_Init();
  AD9837_Init(); // function also initializes the AD pins 
  LCD_init(); 
  
  
  // Set initial frequency, phase,  wave type
  AD9837_Setup(FREQ_SETUP, PHASE_SETUP, WAVE_SETUP); //check parameter types   
  
  while(1)    //Main loop
  {      
    waveform_control();
    LCD_output();
    AD9837_SetFrequency(FREQ_REG, freqPosCount);
      
  } // End main while loop
} // End main function

  
  void encoderInterrupt (void){
    GPIOIntClear(PINA_BASE,PINA_INT); // Clear interrupt 
    volatile int CLK = GPIOPinRead(PINA_BASE,PINA); // Read clock pin 
    volatile int DT = GPIOPinRead(PINB_BASE,PINB); // Read Data Pin 
    
    if (!CLK && DT){    //Only act when clock is low, if not equal going clockwise
      switch(state){
      case 0:           
        if (freqPosCount < FREQ_MAX)  // if frequency will not exceed maximum
          freqPosCount++; //increment by 1 Hz
        else 
          freqPosCount = FREQ_MAX;
        break;    
      case 4: 
        if (freqPosCount <= (FREQ_MAX - 1000))  // if frequency will not exceed maximum  
          freqPosCount += 1000; //increment by 1 KHz
        else 
          freqPosCount = FREQ_MAX;
        break; 
      }//end switch         
     // if (freqPosCount > FREQ_MAX) //Prevent going over limit 
      //    freqPosCount = FREQ_MAX;      
      encoderDebounce = true; //set debounce flag
      GPIOIntDisable(PINA_BASE,PINA_INT); //disable interrupts on pin for debounce   
      g_ui32Counter = 0; //Clear the debounce counter
    } 
    else if (!CLK && !DT){ // Only act when clock is low. If equal, going counter-clockwise
      switch(state){
      case 0: 
        if (freqPosCount > FREQ_MIN)  // if frequency is not zero   
          freqPosCount--; // decrement by 1 KHz
        else 
          freqPosCount = FREQ_MIN;
        break; 
      case 4: 
        if (freqPosCount >= 1000)  // if frequency is not going to go beyond zero
          freqPosCount -= 1000; // decrement by 1 KHz
        else 
          freqPosCount = FREQ_MIN;
        break;  
      }//end switch      
     // if (freqPosCount < FREQ_MIN) //Prevent going under limit
     //   freqPosCount = FREQ_MIN;     
      encoderDebounce = true;     
      GPIOIntDisable(PINA_BASE,PINA_INT);
      g_ui32Counter = 0; //Clear the debounce counter
    }
  }

void SysTickIntHandler(void)
{ 
  // if debouncing and counter is on a second cycle, then re-enable encoder interrupt
  if (!(g_ui32Counter++%2) && encoderDebounce){
    GPIOIntEnable(PINA_BASE, PINA_INT);
    encoderDebounce = false;
  }
}
