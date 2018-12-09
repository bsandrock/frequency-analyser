#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "fix_fft.h"
#include "fix_fft.cpp"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include <math.h>


// Makros
#define FSAMPLE  10000
#define BUFFER_BITS 9
#define BUFFER_SIZE (1<<BUFFER_BITS)
#define OVERDRIVE_DURATION 200
#define OVERDRIVE_COUNT_INIT (OVERDRIVE_DURATION*FSAMPLE/1000)

// globale Variable
int overdrive_counter = 0;
uint32_t buffer_index = 0;
int im[BUFFER_SIZE], data[BUFFER_SIZE];
// Prototypen
void ADC_int_handler(void);
void pwm_init(){

    //Activate PWM0 Module
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
        //Activate GPIO Outputs and configure for PWM usage
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
        GPIOPinConfigure(GPIO_PB6_M0PWM0);
        GPIOPinConfigure(GPIO_PB7_M0PWM1);
        GPIOPinConfigure(GPIO_PB4_M0PWM2);
        GPIOPinConfigure(GPIO_PB5_M0PWM3);
        GPIOPinConfigure(GPIO_PE4_M0PWM4);
        GPIOPinConfigure(GPIO_PE5_M0PWM5);
        GPIOPinConfigure(GPIO_PC4_M0PWM6);
        GPIOPinConfigure(GPIO_PC5_M0PWM7);
        GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5);
        GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
        GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
        // Wait for the PWM0 module to be ready.
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))   {
        }
        //
        // Configure the PWM generators for count down mode with immediate updates
        // to the parameters.
        //
        PWMGenConfigure(PWM0_BASE, PWM_GEN_0 ,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        PWMGenConfigure(PWM0_BASE, PWM_GEN_1 ,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        PWMGenConfigure(PWM0_BASE, PWM_GEN_2 ,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        PWMGenConfigure(PWM0_BASE, PWM_GEN_3 ,PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        //
        // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
        // microseconds. For a 20 MHz clock, this translates to 400 clock ticks.
        // Use this value to set the period.
        //
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 256);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 256);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 256);
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 256);
        //
        // Set pulse width to minimum Value
        //
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 1);
        //
        // Start the timers in generators.
        //
        PWMGenEnable(PWM0_BASE, PWM_GEN_0);
        PWMGenEnable(PWM0_BASE, PWM_GEN_1);
        PWMGenEnable(PWM0_BASE, PWM_GEN_2);
        PWMGenEnable(PWM0_BASE, PWM_GEN_3);
        //
        // Enable the outputs.
        //
        PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT), true);
        //Activate GPIO for red LED
        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);

}
void set_led(int pin, int value){
    //sets output of LED connected to PWM-Pin pin to value
if(value > 255){value = 255;}
if(value < 1){value = 1;}
switch(pin){
    case 0:
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, value);
    break;
    case 1:
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, value);
    break;
    case 2:
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, value);
    break;
    case 3:
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, value);
    break;
    case 4:
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, value);
    break;
    case 5:
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, value);
    break;
    case 6:
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, value);
    break;
    case 7:
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, value);
    break;

}
}
int main(void)
{
    // SystemClock konfigurieren
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    uint32_t ui32Period = SysCtlClockGet()/FSAMPLE;

    // Peripherie aktivieren
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // GPIO konfigurieren
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_2);
    pwm_init();
    //Timer0 konfigurieren
    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
    TimerControlTrigger(TIMER0_BASE,TIMER_A,true);
    TimerEnable(TIMER0_BASE,TIMER_A);

    // ADC konfigurieren
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_RATE_FULL,1);

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE,3);
    ADCIntRegister(ADC0_BASE,3,ADC_int_handler);
    ADCIntEnable(ADC0_BASE,3);

    while(1) {
        if(buffer_index == BUFFER_SIZE){ //start FFT if Buffer is filled
            fix_fft(data, im, BUFFER_BITS, 0);
            uint16_t index = 0;
            for(int i = 0; i<8;i++){ //calculate values for each LED
               int pegel = 0;
               int data_value;
               int im_value;

               for(int j = 0; j<BUFFER_SIZE/16;j++){
                   data_value = data[index];
                   im_value = im[index];
                   pegel+= sqrtf(data_value*data_value+im_value*im_value);//calculate level of frequency spectrum
                   index += 1;
               }
               set_led(i, (pegel/(BUFFER_SIZE/16))>>1);
            }
            buffer_index = 0;
        }
    }
}

// Interrupt handler
void ADC_int_handler(void)
{
    uint32_t ui32ADC0Value;
    ADCIntClear(ADC0_BASE, 3);  // delete interrupt flag
    ADCSequenceDataGet(ADC0_BASE, 3, &ui32ADC0Value); // Wert auslesen
    if(buffer_index<BUFFER_SIZE){
        data[buffer_index] = (ui32ADC0Value<<4)-(1<<15);
        im[buffer_index]=0;
        buffer_index++;
    }
    if(ui32ADC0Value>4000){
        overdrive_counter = OVERDRIVE_COUNT_INIT;
    }
    if(overdrive_counter>0){
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0xFF);
        overdrive_counter--;
    }else{
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
    }
}
