
#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "Clock.h"
#include "UART0.h"
#include "CortexM.h"
#include <stdint.h>
#include "msp.h"



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
  TIMER_A0->CTL |= 0x0026;       // reset and start Timer A0 in continuous mode

}

uint16_t Period=0, Period_R=0, Period_L=0;              // (1/SMCLK) units = 83.3 ns units
uint16_t Previous=0, Previous_L=0, Previous_R=0;               // Timer A0 first edge
uint16_t Time=0, Time_L=0, Time_R=0, Type=0;
uint32_t N=0, N_L=0, N_R=0, Roll0=0, LastRoll0=0, Roll3=0, LastRoll3=0, Period32;
uint32_t Acc_L = 0, Acc_R = 0;
uint32_t Rev_L = 0, Rev_R = 0;
uint32_t tick=0;

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us

void TA3_0_IRQHandler(void){
  P1->OUT |= BIT0;
  TIMER_A3->CCTL[0] &= ~0x0001;      // acknowledge capture/compare interrupt 0
  Time_R = TIMER_A3->CCR[0];
  Period_R = (Time_R - Previous_R) & 0xFFFF; // 16 bits, 83.3 ns resolution
  Acc_R += Period_R;
  Previous_R = Time_R;                   // setup for next
  N_R++;
  if (0==(N_R % 3)) {
      Rev_R = Acc_R;
      Acc_R = 0;
  }
  P1->OUT &= ~BIT0;
}
void TA3_N_IRQHandler(void){
  P1->OUT |= BIT0;
  Type = TIMER_A3->IV;
  if (Type == (uint16_t) 0x000E) {
      TIMER_A3->CTL &= ~0x0001;                // acknowledge rollover interrupt
      tick++;
  } else {
      TIMER_A3->CCTL[1] &= ~0x0001;          // acknowledge capture/compare interrupt 1
      Time_L = TIMER_A3->CCR[1];
      Period_L = (Time_L - Previous_L) & 0xFFFF;  // 16 bits, 83.3 ns resolution
      Acc_L += Period_L;
      Previous_L = Time_L;                     // setup for next
      N_L++;
      if (0 == (N_L % 3)) {
          Rev_L = Acc_L;
          Acc_L = 0;
      }
  }
  P1->OUT &= ~BIT0;
}


void TA0_0_IRQHandler(void){
  TIMER_A0->CCTL[0] &= ~0x0001;                 // acknowledge capture/compare interrupt 0
}

void TA0_N_IRQHandler(void){
  TIMER_A0->CTL &= ~0x0001;                     // acknowledge rollover interrupt 0
}



uint32_t PWM_L = 200 << 16;
uint32_t PWM_R = 200 << 16;
uint32_t last_N_L=0, last_N_R=0;
int32_t p;

float Setpoint_L= 1.0;
float Setpoint_R= 1.0;
float Error_L, Error_R;
float Speed_L, Speed_R;

#define PRINT_INTERVAL 10
#define DATA_INTERVAL 10
#define STEP_INTERVAL 10

#define DATA_LENGTH 10000
#define PWM_PERIOD 1000

float Data[DATA_LENGTH];
int Data_i=0;

float Steps[] = { 0.0, 0.0, 50,
                  1.0, 1.0, 80,
                  -1.0, -1.0, 80,
                  0.0,  0.0,  -1 };

int Step_i = 0;
int Step_t = 0;

#define I_MIN (-150.0)
#define I_MAX (150.0)
#define I_COEFF (1e6)
#define P_COEFF (1e7)
#define OFFSET (2.0e7)
#define STOP_TICKS (300)
#define MIN_SET (1.0)

int main(void){

    Clock_Init48MHz();      // makes SMCLK=12 MHz
    TimerA3Capture_Init();  // initialize Timer A3 in capture mode
    UART0_Initprintf();     // initialize UART and printf
    UART0_OutString("\r\nControl demo!\r\n");

    // Configure GPIO
    P1->DIR |= BIT0;
    P1->OUT &= ~BIT0;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // Stop WDT

    // Configure GPIO
    P2->DIR |= BIT6 | BIT7;                 // P7.6~7 set TA1.1~2
    P2->SEL0 |= BIT6 | BIT7;
    P2->SEL1 &= ~(BIT6 | BIT7);

    TIMER_A0->CCR[0] = PWM_PERIOD - 1;     // PWM Period
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; //interrupt enabled
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7; // CCR3 reset/set
    TIMER_A0->CCR[3] = 100;                 // CCR3 PWM duty cycle
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7; // CCR4 reset/set
    TIMER_A0->CCR[4] = 100;                 // CCR4 PWM duty cycle
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK
              TIMER_A_CTL_MC__UP | // Up mode
              TIMER_A_CTL_IE |   //interrupt enable
              TIMER_A_CTL_CLR;                // Clear TAR

    P4->SEL0 &= ~0xFD;
    P4->SEL1 &= ~0xFD;   // 1) configure P4 as GPIO (except .1)
    P4->DIR &= ~0xFD;    // 2) make P4 in
    P4->REN |= 0xFD;     // 3) enable pull resistors on P4

    P5->DIR |= BIT4|BIT5;
    __enable_irq();

    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);

    EnableInterrupts();

    char c;
    int stopped = 0, open_loop=0;
    uint32_t last_tick;
    int data_len = 0;
    int dump_n, dump_i, bump;
    int Step_duration;
    int stepped=0;
    int bump_start;
    int tick_now = 0;
    int stopped_L=0, stopped_R=0;
    float I_L=0.0, P_L=0.0, out_L = 0.0;
    float I_R=0.0, P_R=0.0, out_R = 0.0;
    float IL, PL, IR, PR;
    Setpoint_L = Steps[0];
    Setpoint_R = Steps[1];
    Step_duration = (int) Steps[2];
    bump_start = (int) (P4->IN&0xFD);
    while (1) {
        if (tick != last_tick) {
            tick_now = tick;
            bump = (int) (P4->IN&0xFD);
            if (0 == (tick_now % PRINT_INTERVAL)) {
               // printf("tick %7d, PWM_L %9d, Set_L %9.3f, Speed_L %9.3f, Error_L %9.3f, Rev_L %7d, IL %9.3f, PL %9.3f\r\n",   //PWM_R %9d, Set_R %9.3f, Speed_R %9.3f, Error_R %9.3f, Rev_R %9d, I_R %9.3f\r\n",
                 //      tick,       PWM_L,      Setpoint_L, Speed_L,      Error_L,      Rev_L,     IL,       PL); // ,       PWM_R,       Setpoint_R, Speed_R,     Error_R,      Rev_R,      I_R);
            }
            if (0 == (tick_now % DATA_INTERVAL) && 0==stopped && bump==bump_start ) {
                data_len = Data_i;
                Data[Data_i++ % DATA_LENGTH] = (float) tick;
                Data[Data_i++ % DATA_LENGTH] = (float) N_L;
                Data[Data_i++ % DATA_LENGTH] = (float) PWM_L;
                Data[Data_i++ % DATA_LENGTH] = (float) Setpoint_L;
                Data[Data_i++ % DATA_LENGTH] = (float) Speed_L;
                Data[Data_i++ % DATA_LENGTH] = (float) Error_L;
                Data[Data_i++ % DATA_LENGTH] = (float) N_R;
                Data[Data_i++ % DATA_LENGTH] = (float) PWM_R;
                Data[Data_i++ % DATA_LENGTH] = (float) Setpoint_R;
                Data[Data_i++ % DATA_LENGTH] = (float) Speed_R;
                Data[Data_i++ % DATA_LENGTH] = (float) Error_R;
                data_len = Data_i - data_len;
            }
            if (0 == (tick_now % STEP_INTERVAL) && 0==stopped && bump==bump_start) {
                if (0==stepped) {
                    Step_t++;
                    if ((Step_t == Step_duration) && (Step_duration != 0)) {
                        Step_i++;
                        if (Steps[Step_i*3+2] < -0.5) {
                            Step_i = 0;
                        }
                        Setpoint_L = Steps[Step_i*3];
                        Setpoint_R = Steps[Step_i*3+1];
                        Step_duration = Steps[Step_i*3+2];
                        Step_t = 0;
                        if(Setpoint_L < 0){
                            P5->OUT |= BIT4;
                            Setpoint_L = Setpoint_L*-1;
                        }
                        else{
                            P5->OUT &= ~BIT4;
                        }
                        if(Setpoint_R < 0){
                            P5->OUT |= BIT5;
                            Setpoint_R = Setpoint_R*-1;
                        }
                        else{
                            P5->OUT &= ~BIT5;
                        }
                        printf("Step %d\r\n", Step_i);
                    }
                    stepped=1;
                }
            } else {
                stepped=0;
            }
        }
        if ((EUSCI_A0->IFG&0x01) != 0) {
            c = (char)(EUSCI_A0->RXBUF);
            printf("%c\r\n", c);
            if ('+' == c) {
                Setpoint_L += 0.1;
                Setpoint_R += 0.1;
            }
            if ('-' == c) {
                Setpoint_L -= 0.1;
                Setpoint_R -= 0.1;
            }
            if ('0' == c) {
                stopped = 1-stopped;
                printf("stopped=%d\r\n", stopped);
            }
            if ('o' == c) { // go open-loop
                open_loop = 1-open_loop;
                printf("open_loop=%d\r\n", open_loop);
            }
            if ('d' == c) {
                dump_n = (DATA_LENGTH / data_len)*data_len;
                dump_i = Data_i - dump_n - 1;
                if (dump_i < 0) {
                    dump_i += DATA_LENGTH;
                }
                for (int n=0; n<dump_n; n++) {
                    printf("%f", Data[dump_i++ % DATA_LENGTH]);
                    if (0 == (n % data_len)) {
                        printf("\n");
                    } else {
                        printf("\t");
                    }
                 }
                 printf("\n");
            }
        }

        if (stopped || (bump != bump_start)) {
            TIMER_A0->CCR[4] = 0;
            TIMER_A0->CCR[3] = 0;
        } else {
            if (tick_now != last_tick) {

                // right

                 if (N_R == last_N_R) {
                     stopped_R++;
                     if (stopped_R > STOP_TICKS) {
                         Speed_R = 0.0;
                         I_R = 0;
                     }
                 } else {
                     if (Rev_R>0) {
                         Speed_R = 100000.0/(float) Rev_R;
                     } else {
                         Speed_R = 0.0;
                         I_R = 0;
                     }
                     stopped_R=0;
                 }
                 last_N_R = N_R;

                 Error_R = Speed_R - Setpoint_R;

                 I_R += Error_R;

                 if (I_R<I_MIN) {
                     I_R = I_MIN;
                 }
                 if (I_R>I_MAX) {
                      I_R = I_MAX;
                  }

                 PR = - Error_R * P_COEFF;
                 IR = - I_R * I_COEFF;

                 out_R = OFFSET + PR + IR;

                 if (out_R < 0.0) {
                     //out_R = 0.0;
                 }
                 if (out_R > (float) (999<16) ) {
                     //out_R = (float) (999<16);
                 }

                 if (Setpoint_R < MIN_SET) {
                     PWM_R = 0;
                     I_R = 0.0;
                 } else {
                     PWM_R = (uint32_t) out_R;
                 }

                // left

                if (N_L == last_N_L) {
                    stopped_L++;
                    if (stopped_L > STOP_TICKS) {
                        Speed_L = 0.0;
                        I_L=0;
                    }
                } else {
                    if (Rev_L>0) {
                        Speed_L = 100000.0/(float) Rev_L;
                    } else {
                        Speed_L = 0.0;
                        I_L=0;
                    }
                    stopped_L=0;
                }
                last_N_L = N_L;

                Error_L = Speed_L - Setpoint_L;

                I_L += Error_L;

                if (I_L<I_MIN) {
                    I_L = I_MIN;
                }
                if (I_L>I_MAX) {
                     I_L = I_MAX;
                 }

                PL = - Error_L * P_COEFF;
                IL = - I_L * I_COEFF;


                out_L = OFFSET + PL + IL;

                if (out_L < 0.0) {
                    //out_L = 0.0;
                }
                if (out_L > (float) (999<16) ) {
                    //out_L = (float) (999<16);
                }

                if (Setpoint_L < MIN_SET) {
                    PWM_L = 0;
                    I_L = 0.0;
                } else {
                    PWM_L = (uint32_t) out_L;
                }


                // update PWM
                if (!open_loop) {
                    TIMER_A0->CCR[4] = (uint16_t) (PWM_L >> 16);
                    TIMER_A0->CCR[3] = (uint16_t) (PWM_R >> 16);
                }
                last_tick = tick_now;
            }
        }
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
