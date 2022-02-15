#include "efm32gg990f1024.h"
#include "lcd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include "em_chip.h"    // required for CHIP_Init() function

#define LED_PORT 4            // gpioPortE
#define TOP_VAL_PWM 1000      // sets PWM frequency to 1kHz (1MHz timer clock)
#define TOP_VAL_GP_TIMER 1000 // sets general purpose timer overflow frequency to 1kHz (1MHz timer clock)

#define COM_PORT 3 // gpioPortD (USART location #1: PD'0 and PD1)
#define ADC_PORT 3 // gpioPortD (ADC Channel 6 location #0: PD6)
#define TX_pin 0
#define ADC_pin 6 // ADC Channel 6

uint16_t ms_counter = 0;

void Delay(uint32_t delay)
{
    volatile uint32_t counter;
    int i;

    for (i = 0; i < delay; i++)
    {
        counter = 100000;
        while (counter)
            counter--;
    }
}
void TIMER0_IRQHandler(void)
{
    TIMER0->IFC = 1; // Clear overflow flag
    ms_counter++;    // Increment counter
}

void setupADC()
{
    // Initialize Clock Tree
    CMU->CTRL |= (1 << 14); // Set HF clock divider to /2 to keep core frequency <32MHz
    CMU->OSCENCMD |= 0x4;   // Enable XTAL Oscillator
    while (!(CMU->STATUS & 0x8))
        ;                                                           // Wait for XTAL osc to stabilize
    CMU->CMD = 0x2;                                                 // Select HF XTAL osc as system clock source. 48MHz XTAL, but we divided the system clock by 2, therefore our HF clock should be 24MHz
    CMU->HFPERCLKEN0 = (1 << 16) | (1 << 13) | (1 << 5) | (1 << 1); // Enable GPIO, TIMER0, USART1, and ADC0 peripheral clocks

    // Initialize GPIO
    GPIO->P[COM_PORT].MODEL = (1 << 24) | (1 << 4) | (4 << 0); // Configure PD0 as digital output, PD1 and PD6 as input
    GPIO->P[COM_PORT].DOUTSET = (1 << TX_pin);                 // Initialize PD0 high since UART TX idles high (otherwise glitches can occur)

    // Setup UART Port for asynch mode, frame format 8-none-1-none
    USART1->CLKDIV = (152 << 6);                               // 152 will give 38400 baud rate (using 16-bit oversampling with 24MHz peripheral clock)
    USART1->CMD = (1 << 11) | (1 << 10) | (1 << 2) | (1 << 0); // Clear RX/TX buffers and shif regs, Enable Transmitter and Receiver
    USART1->IFC = 0x1FF9;                                      // clear all USART interrupt flags
    USART1->ROUTE = 0x103;                                     // Enable TX and RX pins, use location #1 (UART TX and RX located at PD0 and PD1, see EFM32GG990 datasheet for details)

    // Setup ADC
    // Timebase bit field = 24, defines ADC warm up period (must be greater than or equal to number of clock cycles in 1us)
    // Prescaler setting = 1: ADC clock = HFPERCLK/2 = 12MHz (ADC clock should be between 32kHz and 13MHz)
    // Oversampling set to 2, no input filter, no need for Conversion Tailgating
    // Warm-up mode = NORMAL (ADC is not kept warmed up between conversions)
    ADC0->CTRL = (24 << 16) | (1 << 8);

    // Don't use PRS as input
    // Can use single-cycle acquisition time since we are spacing out our conversions using a timer
    // Use buffered Vdd as reference, use Channel 6 as input to single conversion
    // 12-bit resolution, right-justified, single-ended input, single conversion
    ADC0->SINGLECTRL = (2 << 16) | (6 << 8);
    ADC0->IEN = 0x0; // Disable ADC interrupts

    // Setup Timer to trigger conversions
    TIMER0->TOP = 24000;         // Set TOP value for Timer0
    TIMER0->IEN = 1;             // Enable Timer0 overflow interrupt
    NVIC_EnableIRQ(TIMER0_IRQn); // Enable TIMER0 interrupt vector in NVIC
    TIMER0->CMD = 0x1;           // Start timer0
}

void setupLED()
{
    CMU->HFRCOCTRL = 0x8;
    CMU->HFPERCLKEN0 = CMU->HFPERCLKEN0 | (1 << 8);
    GPIO->P[LED_PORT].MODEL = (4 << 12) | (4 << 8);

    TIMER0->TOP = TOP_VAL_GP_TIMER;
    TIMER3->TOP = TOP_VAL_PWM;

    TIMER0->CNT = 0;
    TIMER3->CNT = 0;

    TIMER3->CC[2].CCV = 0;
    TIMER3->CC[2].CCVB = 0;

    TIMER3->CC[2].CTRL = 0x3;
    TIMER3->ROUTE = (1 << 16) | (1 << 2);

    TIMER0->CMD = 0x1;
    TIMER3->CMD = 0x1;
}

int main()
{
    int adc_result = 0; // Temp variable for storing ADC conversion results

    setupADC();
    setupLED();

    Delay(3);

    // Configure LCD
    LCD_Init();

    LCD_SetAll();
    Delay(3);

    LCD_ClearAll();
    Delay(3);

    TIMER3->CC[2].CCVB = 200;

    while (1)
    {
        if (ms_counter == 500)
        {
            ADC0->CMD = 0x1; // Start Single Conversion
            while (!(ADC0->STATUS & (1 << 16)))
                ;                          // Wait for single conversion data to become valid
            adc_result = ADC0->SINGLEDATA; // Store conversion result

            adc_result = adc_result * 100 / 4095;

            char result_str[10];
            itoa(adc_result, result_str, 10);

            TIMER3->CC[2].CCVB = adc_result;

            LCD_WriteString(result_str);

            ms_counter = 0; // reset counter
        }
    }
}
