#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#define servotiltLimit_h 2000  //1600
#define servotiltLimit_l 780
#define servoazimuthLimit_h 1800
#define servoazimuthLimit_l 280

uint32_t pui32ADC0_s1_Value[4];
uint32_t pui32ADC0_s2_Value[4];
uint32_t pui32ADC0_s3_Value[4];
uint32_t pui32ADC0_s4_Value[4];
uint32_t servotilt_val = 780;
uint32_t servoazimuth_val = 280;

int topl;
int topr;
int botl;
int botr;
void delay(float); // Delay
void servotilt(uint32_t); // Tilt Angle Control
void servoazimuth(uint32_t); // Azimuth Angle Control

int main(void){
//    int i=0;
    volatile uint32_t ui32Adjust;
    SysCtlClockSet(SYSCTL_SYSDIV_16 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // System Clock Set 12.5MHz
    /* PWM Interfacing
     * PWM PB5 M0 GEN 1 OUT 3
     * PWM PB6 M0 GEN 0 OUT 0 */
    SysCtlPWMClockSet( SYSCTL_PWMDIV_16 ); // PWM CLock Set 781.25KHz
    SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM0 ); // Peripheral PWM0 Enable
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB ); // GPIO B Enable for PWM
    GPIOPinTypePWM( GPIO_PORTB_BASE, GPIO_PIN_5); // Pin Number for PWM Gen 1 for PB5
    GPIOPinTypePWM( GPIO_PORTB_BASE, GPIO_PIN_6); // Pin Number for PWM Gen 0 for PB6
    GPIOPinConfigure(  GPIO_PB5_M0PWM3 ); // For PB5
    GPIOPinConfigure(  GPIO_PB6_M0PWM0 ); // For PB6
    PWMGenConfigure( PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN ); // For PB5
    PWMGenConfigure( PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN ); // For PB6
    PWMGenPeriodSet( PWM0_BASE, PWM_GEN_1, 15625 ); //Total Pulse Period
    PWMGenPeriodSet( PWM0_BASE, PWM_GEN_0, 15625 ); //Total Pulse Period
    PWMOutputState( PWM0_BASE, PWM_OUT_3_BIT, true ); // Output for PB5
    PWMOutputState( PWM0_BASE, PWM_OUT_0_BIT, true ); // Output for PB6
    PWMGenEnable( PWM0_BASE, PWM_GEN_1 ); // Enable the Gen 1
    PWMGenEnable( PWM0_BASE, PWM_GEN_0 ); // Enable the Gen 0

    /* ADC Interfacing
     *
     * */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // Peripheral ADC0 Enable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1); // Peripheral ADC1 Enable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // GPIO E Enable for ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // Pin Number for ADC 0
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); // Pin Number for ADC 1
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1); // Pin Number for ADC 1
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // Pin Number for ADC 1
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCSequenceEnable(ADC1_BASE, 1);
    ADCSequenceEnable(ADC1_BASE, 2);
    ADCIntClear(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 2);
    ADCIntClear(ADC1_BASE, 1);
    ADCIntClear(ADC1_BASE, 2);
    // Infinite Loop
    while(1){
        /* ADC */
        // Trigger the ADC conversion.
        ADCProcessorTrigger(ADC0_BASE, 1);
        ADCProcessorTrigger(ADC0_BASE, 2);
        ADCProcessorTrigger(ADC1_BASE, 1);
        ADCProcessorTrigger(ADC1_BASE, 2);
        // Wait for conversion to be completed.
        while(!ADCIntStatus(ADC0_BASE, 1, false))
        {
        }
        while(!ADCIntStatus(ADC0_BASE, 2, false))
        {
        }
        while(!ADCIntStatus(ADC1_BASE, 1, false))
        {
        }
        while(!ADCIntStatus(ADC1_BASE, 2, false))
        {
        }
        // Clear the ADC interrupt flag.
        ADCIntClear(ADC0_BASE, 1);
        ADCIntClear(ADC0_BASE, 2);
        ADCIntClear(ADC1_BASE, 1);
        ADCIntClear(ADC1_BASE, 2);
        // Read ADC Value.
        ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0_s1_Value);
        ADCSequenceDataGet(ADC0_BASE, 2, pui32ADC0_s2_Value);
        ADCSequenceDataGet(ADC1_BASE, 1, pui32ADC0_s3_Value);
        ADCSequenceDataGet(ADC1_BASE, 2, pui32ADC0_s4_Value);
//        delay(0.1); // 0.1 sec delay
//
//        topl = pui32ADC0_s3_Value[0];
//        topr = pui32ADC0_s1_Value[0];
//        botl = pui32ADC0_s2_Value[0];
//        botr = pui32ADC0_s4_Value[0];

         topl = pui32ADC0_s3_Value[0]; //PE0
         topr = pui32ADC0_s1_Value[0];//PE1
         botl = pui32ADC0_s2_Value[0];//PE2
         botr = pui32ADC0_s4_Value[0];//PE3

        int avgtop = (topl + topr) ; //average of top
        int avgbot = (botl + botr) ; //average of bottom
        int avgleft = (topl + botl) ; //average of left
        int avgright = (topr + botr) ; //average of right

          if (avgtop > avgbot)
          {
              if(servotilt_val < servotiltLimit_h){
                  servotilt_val = servotilt_val + 5; // ++
                  servotilt(servotilt_val);
              }
              else servotilt_val = servotiltLimit_h;
          }
          else if (avgbot > avgtop)
          {
              if(servotilt_val > servotiltLimit_l){
                  servotilt_val = servotilt_val - 5; // --
                  servotilt(servotilt_val);
              }
              else servotilt_val = servotiltLimit_l;
          }
//          else cond




          if (avgleft < avgright)
          {
              if(servoazimuth_val < servoazimuthLimit_h){
                  servoazimuth_val = servoazimuth_val + 5; // ++
                  servoazimuth(servoazimuth_val);
              }
              else servoazimuth_val = servoazimuthLimit_h;
          }
          else if (avgright < avgleft)
          {
              if(servoazimuth_val > servoazimuthLimit_l){
                  servoazimuth_val = servoazimuth_val - 5; // --
                  servoazimuth(servoazimuth_val);
              }
              else servoazimuth_val = servoazimuthLimit_l;

          }
      //    else Cond

          delay(0.1);

        /* PWM
        Pulse Width = 280 => 0 deg
        Pulse Width = 1800 => 180 deg
        Tilt 0 deg = 780
        Tilt 90 deg  = 1600
        */
//
//        for(i=780;i<1600;i=i+10){
//                   ui32Adjust = i;
//                   servotilt(ui32Adjust );
//                   servoazimuth(ui32Adjust );
//                   delay(0.05);
//             }
    }
}

void delay(float i){
    // i is in seconds
    SysCtlDelay((SysCtlClockGet()*i)/3);
}

void servotilt(uint32_t ui32Adjust){
    PWMPulseWidthSet( PWM0_BASE, PWM_OUT_3,  ui32Adjust ); // PB5
}

void servoazimuth(uint32_t ui32Adjust){
    PWMPulseWidthSet( PWM0_BASE, PWM_OUT_0,  ui32Adjust ); //PB6
}
