/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP432 CODE EXAMPLE DISCLAIMER
 *
 * MSP432 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see http://www.ti.com/tool/mspdriverlib for an API functional
 * library & https://dev.ti.com/pinmux/ for a GUI approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//*******************************************************************************
//  MSP432P401 Demo - Timer1_A3, PWM TA1.1-2, Up Mode, DCO SMCLK
//
//  Description: This program generates two PWM outputs on P7.6, P7.7 using
//  Timer1_A configured for up mode. The value in CCR0, 1000-1, defines the PWM
//  period and the values in CCR1 and CCR2 the PWM duty cycles. Using ~3MHz
//  SMCLK as TACLK, the timer period is ~1ms with a 75% duty cycle on P7.7
//  and 25% on P7.6.
//  ACLK = n/a, SMCLK = MCLK = TACLK = 3MHz
//
//
//           MSP432P401
//         ---------------
//     /|\|               |
//      | |               |
//      --|RST            |
//        |               |
//        |     P7.7/TA1.1|--> CCR1 - 75% PWM
//        |     P7.6/TA1.2|--> CCR2 - 25% PWM
//
//
//   William Goh
//   Texas Instruments Inc.
//   June 2016 (updated) | June 2014 (created)
//   Built with CCSv6.1, IAR, Keil, GCC
//******************************************************************************
#include "ti/devices/msp432p4xx/inc/msp.h"
int tick = 0;
int n = 100;
int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
            WDT_A_CTL_HOLD;

    // Configure GPIO
//    P7->DIR |= BIT6 | BIT7;                 // P7.6~7 set TA1.1~2
//    P7->SEL0 |= BIT6 | BIT7;
//    P7->SEL1 &= ~(BIT6 | BIT7);

    P2->DIR |= BIT6 | BIT7;
    P2->SEL0 |= BIT6 | BIT7;
    P2->SEL1 &= ~(BIT6 | BIT7);
    //Pin7
//    TIMER_A1->CCR[0] = 1000 - 1;            // PWM Period
//    TIMER_A1->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7; // CCR1 reset/set
//    TIMER_A1->CCR[1] = 750;                 // CCR1 PWM duty cycle
//    TIMER_A1->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7; // CCR2 reset/set
//    TIMER_A1->CCR[2] = 250;                 // CCR2 PWM duty cycle
    //Pin2
    TIMER_A0->CCR[0] = 1000 - 1;            // PWM Period
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7; // CCR1 reset/set
    TIMER_A0->CCR[3] = 750;                 // CCR1 PWM duty cycle
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7; // CCR2 reset/set
    TIMER_A0->CCR[4] = 250;                 // CCR2 PWM duty cycle
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK
            TIMER_A_CTL_MC__UP |            // Up mode
            TIMER_A_CTL_CLR | TIMER_A_CTL_IE;                // Clear TAR

   // __DSB();
    __enable_irq();

    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);
    while(1){
        if(tick>=n){
            tick = 0;
            TIMER_A0->CCR[3] += n;
            TIMER_A0->CCR[4] += n;
        }
        else if(TIMER_A0->CCR[3]>=999){
            TIMER_A0->CCR[3]=0;
        }
        else if(TIMER_A0->CCR[4]>=999){
            TIMER_A0->CCR[4]=0;
        }
    }
    // Enter LPM0
    //__sleep();
    //__no_operation();                       // For debugger
}
void TA0_0_IRQHandler(void) {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    tick++;
}
