
#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "Clock.h"
#include "UART0.h"
#include "CortexM.h"
#include <stdint.h>
#include "msp.h"
int tick = 0;
int n = 100;


// capture P10.4, P10.5:


void TimerA3Capture_Init() {

  // initialize P10.4, P10.5 and make them Timer A inputs
  P10->SEL0 |= 0x30;
  P10->SEL1 &= ~0x30;                 // configure as Timer A
  P10->DIR &= ~0x30;                  // configure as input

  TIMER_A3->CTL &= ~0x0030;          // halt Timer A3

  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=00,       input clock divider /1
  // bits5-4=00,       stop mode
  // bit3=X,           reserved
  // bit2=0,           set this bit to clear
  // bit1=0,           interrupt disable
  // bit0=0,           clear interrupt pending
  TIMER_A3->CTL = 0x0200;

  // bits15-14=01,     capture on rising edge
  // bits13-12=00,     capture/compare input on CCI0A
  // bit11=1,          synchronous capture source
  // bit10=X,          synchronized capture/compare input
  // bit9=X,           reserved
  // bit8=1,           capture mode
  // bits7-5=XXX,      output mode
  // bit4=1,           enable capture/compare interrupt
  // bit3=X,           read capture/compare input from here
  // bit2=X,           output this value in output mode 0
  // bit1=X,           capture overflow status
  // bit0=0,           clear capture/compare interrupt pending
  TIMER_A3->CCTL[0] = 0x4910;
  TIMER_A3->CCTL[1] = 0x4910;

  TIMER_A3->EX0 &= ~0x0007;       // configure for input clock divider /1

  NVIC->IP[3] = (NVIC->IP[3]&0xFF00FFFF)|0x00400000; // set TIMER_A3 interrupt 0 to priority 2
  NVIC->IP[3] = (NVIC->IP[3]&0x00FFFFFF)|0x40000000; // set TIMER_A3 interrupt N to priority 2
  NVIC->ISER[0] = 0x0000C000; // enable TIMER_A3 interrupt 0 and N (interrupts 14 & 15) in NVIC

  // interrupts will be enabled in the main program after all devices are initialized

  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=00,       input clock divider /1
  // bits5-4=10,       continuous mode
  // bit3=X,           reserved
  // bit2=1,           set this bit to clear
  // bit1=1,           interrupt enable (interrupt on rollover)
  // bit0=0,           clear interrupt pending
  TIMER_A3->CTL |= 0x0026;        // reset and start Timer A3 in continuous mode

}

//////////////////////////////////
//------------TimerCapture_Init------------
// Initialize Timer A0 in edge time mode to request interrupts on
// the rising edge of P7.3 (TA0CCP0).
//////////////////////////////////

void TimerA0Capture_Init() {

  // initialize P7.3 and make it input (P7.3 TA0CCP0)
  P7->SEL0 |= 0x08;
  P7->SEL1 &= ~0x08;                 // configure P7.3 as TA0CCP0
  P7->DIR &= ~0x08;                  // make P7.3 in

  TIMER_A0->CTL &= ~0x0030;          // halt Timer A0

  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=00,       input clock divider /1
  // bits5-4=00,       stop mode
  // bit3=X,           reserved
  // bit2=0,           set this bit to clear
  // bit1=0,           interrupt disable
  // bit0=0,           clear interrupt pending
  TIMER_A0->CTL = 0x0200;

  // bits15-14=01,     capture on rising edge
  // bits13-12=00,     capture/compare input on CCI0A
  // bit11=1,          synchronous capture source
  // bit10=X,          synchronized capture/compare input
  // bit9=X,           reserved
  // bit8=1,           capture mode
  // bits7-5=XXX,      output mode
  // bit4=1,           enable capture/compare interrupt
  // bit3=X,           read capture/compare input from here
  // bit2=X,           output this value in output mode 0
  // bit1=X,           capture overflow status
  // bit0=0,           clear capture/compare interrupt pending
  TIMER_A0->CCTL[0] = 0x4910;

  TIMER_A0->EX0 &= ~0x0007;       // configure for input clock divider /1

  NVIC->IP[2] = (NVIC->IP[2]&0xFFFFFF00)|0x00000040; // compare interrupt priority 2
  NVIC->IP[2] = (NVIC->IP[2]&0xFFFF00FF)|0x00004000; // rollover interrupt priority 2

  // interrupts enabled in the main program after all devices initialized
  NVIC->ISER[0] = 0x00000300; // enable interrupts 8, 9 in NVIC

  // bits15-10=XXXXXX, reserved
  // bits9-8=10,       clock source to SMCLK
  // bits7-6=00,       input clock divider /1
  // bits5-4=10,       continuous mode
  // bit3=X,           reserved
  // bit2=1,           set this bit to clear
  // bit1=1,           interrupt disable (no interrupt on rollover)
  // bit0=0,           clear interrupt pending
  TIMER_A0->CTL |= 0x0026;        // reset and start Timer A0 in continuous mode

}

uint16_t Period=0, Period1=0, Period2=0;              // (1/SMCLK) units = 83.3 ns units
uint16_t Previous=0, Previous1=0, Previous2=0;               // Timer A0 first edge
uint16_t Time=0, Time1=0, Time2=0, Type=0;
uint32_t N=0, N1=0, N2=0, Roll0=0, LastRoll0=0, Roll3=0, LastRoll3=0, Period32;

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us

void TA3_0_IRQHandler(void){
  TIMER_A3->CCTL[0] &= ~0x0001;      // acknowledge capture/compare interrupt 0
  Time1 = TIMER_A3->CCR[0];
  Period1 = (Time1 - Previous1) & 0xFFFF; // 16 bits, 83.3 ns resolution
  Previous1 = Time1;                   // setup for next
  N1++;
}
void TA3_N_IRQHandler(void){
    P2->OUT |= BIT0;
  Type = TIMER_A3->IV;
  if (Type == (uint16_t) 0x000E) {
      TIMER_A3->CTL &= ~0x0001;                // acknowledge rollover interrupt
  } else {
      TIMER_A3->CCTL[1] &= ~0x0001;          // acknowledge capture/compare interrupt 1
      Time2 = TIMER_A3->CCR[1];
      Period2 = (Time2 - Previous2)&0xFFFF;  // 16 bits, 83.3 ns resolution
      Previous2 = Time2;                     // setup for next
      N2++;
  }
  P2->OUT &= ~BIT0;
}

void TA0_0_IRQHandler(void){
  TIMER_A0->CCTL[0] &= ~0x0001;                 // acknowledge capture/compare interrupt 0
  Time = TIMER_A0->CCR[0];
  Period = (Time - Previous) & 0xFFFF;          // 16 bits, 83.3 ns resolution
  Previous = Time;                              // setup for next
  N++;
  tick++;
}

void TA0_N_IRQHandler(void){
  TIMER_A0->CTL &= ~0x0001;                     // acknowledge rollover interrupt 0
}

int main(void){

  Clock_Init48MHz();      // makes SMCLK=12 MHz
  TimerA0Capture_Init();  // initialize Timer A0 in capture mode
  TimerA3Capture_Init();  // initialize Timer A3 in capture mode
  UART0_Initprintf();     // initialize UART and printf
  UART0_OutString("\nCapture demo");
  WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
              WDT_A_CTL_HOLD;
  // Configure GPIO
  P1->DIR |= BIT0;
  P1->OUT |= BIT0;
  P2->DIR |= BIT0;
  P2->OUT |= BIT0;
  P2->DIR |= BIT6 | BIT7;
  P2->SEL0 |= BIT6 | BIT7;
  P2->SEL1 &= ~(BIT6 | BIT7);
  TIMER_A0->CCR[0] = 1000 - 1;            // PWM Period
  TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
  TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7; // CCR1 reset/set
  TIMER_A0->CCR[3] = 195;                 // CCR1 PWM duty cycle
  TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7; // CCR2 reset/set
  TIMER_A0->CCR[4] = 195;                 // CCR2 PWM duty cycle
  TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK
                TIMER_A_CTL_MC__UP |            // Up mode
                TIMER_A_CTL_CLR | TIMER_A_CTL_IE;                // Clear TAR
  EnableInterrupts();

  NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);
  while (1) {
    printf("%12d, %6d, %12d, %6d\r\n", N1, Period1, N2, Period2);
    if ((EUSCI_A0->IFG&0x01) != 0) {
      printf("%c\r\n", (char)(EUSCI_A0->RXBUF));
    }
    P1->OUT ^= BIT0;
  }

}


// Modified from the following:

// UARTtestmain.c
// Runs on MSP432
// Test UART and printf functions.
// Daniel and Jonathan Valvano
// September 23, 2017

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/
