
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "msp.h"
#include "Clock.h"
#include "UART0.h"
#include "CortexM.h"
#include <stdint.h>
#include "msp.h"
#include "ADC14.h"

int Step_i = 0;
int Step_speed = 0;

uint32_t v17, v21, v22, v23;


// L move and R move are the amount each wheel will turn in degrees
// Speed is in arbitrary machine units and is proportional to the
// maximum speed the wheel will be controlled to turn
// A negative speed starts the sequence over.
// To stop, use L and R move of zero.  Then "speed" indicates the
// duration of the stop in ticks.

//                L move    R move  speed
int32_t Steps[] = { 800,     800,     10,
                   -240,     240,     10,
 //                     0,       0,   1000,
                    800,     800,     10,
                   -240,     240,     10,
                    800,     800,     10,
                   -240,     240,     10,
                      0,       0,    -1 };

#define DATA_INTERVAL 10
#define DATA_LENGTH 10000
int32_t Data[DATA_LENGTH];
int Data_i = 0;

#define PRINT_INTERVAL 10

// PWM and control loop parameters

#define PWM_PERIOD 1000

#define ERROR_SUM_MAX (1000000)
#define OUT_MAX ((PWM_PERIOD-1) << 16)

#define I_COEFF (0.0)
#define P_COEFF (3200000)

#define NCH (2)
int32_t Setpoint[NCH], Error[NCH], ErrorSum[NCH], P[NCH], I[NCH];
int32_t D[NCH], Out[NCH], N[NCH], Dir[NCH], Move[NCH], Start[NCH], Finish[NCH];
float Ease[NCH], Ease_min, f_Setpoint[NCH];
int Stopped=0;

// TA3 ISRs keep track of encoder counts on ERA, ELA
// and monitor encoder phase inputsP5.0 (ERB) and P5.2 (ELB)
// to determine the direction to count

uint16_t Type=0;
uint32_t tick=0;

void TA3_0_IRQHandler(void){
    TIMER_A3->CCTL[0] &= ~0x0001;      // acknowledge capture/compare interrupt 0
    if (P5->IN & (BIT0)) { // if ERB is high, decrement
        N[1]++;
    } else {                // else increment
        N[1]--;
    }
}

void TA3_N_IRQHandler(void){
  Type = TIMER_A3->IV;
  if (Type == (uint16_t) 0x000E) {
      TIMER_A3->CTL &= ~0x0001;                // acknowledge rollover interrupt
      tick++;
  } else {
      TIMER_A3->CCTL[1] &= ~0x0001;          // acknowledge capture/compare interrupt 1
      if (P5->IN & (BIT2)) { // if ELB is high, increment
          N[0]++;
      } else {                // else decrement
          N[0]--;
      }
  }
}

// TA0 interrupts not used - TA0 just generates PWM signals

void TA0_0_IRQHandler(void){
  TIMER_A0->CCTL[0] &= ~0x0001;                 // acknowledge capture/compare interrupt 0
}

void TA0_N_IRQHandler(void){
  TIMER_A0->CTL &= ~0x0001;                     // acknowledge rollover interrupt 0
}

#define N_INTERVALS 9

uint16_t intervals[9] = { 3000, 300, 300, 3000, 300, 300, 3000, 300, 30000 };
int interval_i = 0;

// TA1 ISR services interrupts at set intervals to create an IR sensor pulsing and
// sampling schedule

void TA1_0_IRQHandler(void) {
    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    P1->OUT &= ~BIT0;
    Stopped = 0;
    int data1 = v23;
    int data2 = v21;
    int data3 = v17;
    switch (interval_i) {
        case 0: // LED1 on
            //P6->OUT |= BIT2;
            P10->OUT |= BIT2;
            P10->OUT |= BIT0;
            P6->OUT |= BIT2;
            break;
        case 1: // sample ADC
            //ADC0_InitSWTriggerCh17_21_23();
            break;
        case 2: // LED1 off
            ADC0_InitSWTriggerCh17_21_23();
            //P6->OUT &= ~BIT2;
            break;
        case 3: // LED2 on
            //P10->OUT |= BIT0;
            break;
        case 4: // sample ADC
            ADC_In17_21_23(&v17, &v21, &v23);
            //printf("Left: %6d Center:%6d Right: %6d\n",data1,data2,data3);
            break;
        case 5: // LED2 off
            //P10->OUT &= ~BIT0;
            break;
        case 6: // LED3 on
            //P10->OUT |= BIT2;
            P6->OUT &= ~BIT2;
            P10->OUT &= ~BIT2;
            P10->OUT &= ~BIT0;
            break;
        case 7: // sample ADC
            //ADC_In17_21_23(&v17, &v21, &v23);
            break;
        case 8: // LED3 off
            //P10->OUT &= ~BIT2;
    }
    if(data1>15000 || data2>14000 || data3>4300){
        P1->OUT ^= BIT0;
        Stopped = 1;
    }
    TIMER_A1->CCR[0] += intervals[interval_i];
    interval_i = (interval_i+1) % N_INTERVALS;
}


#define MIN_SPEED_FRAC (0.04)

// Given the fraction of how far the robot
// is along a path, calculate at what fraction
// of max speed it should be traveling

float ease(float f) {
    // easing function
    float v;
    v = 1 - fabs(f - 0.5) * 2.0;
    return v < MIN_SPEED_FRAC ? MIN_SPEED_FRAC : v;
}

#define CLOSE_COUNTS 2

// see if all values of n arrays are within
// CLOSE_COUNTS of each other

int close(int32_t *act, int32_t *set, int n) {
    for (int i=0; i<n; i++) {
        if (abs(act[i]-set[i]) > CLOSE_COUNTS) {
            return 0;
        }
    }
    return 1;
}

int main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // Stop WDT
    Clock_Init48MHz();      // makes SMCLK=12 MHz
    UART0_Initprintf();     // initialize UART and printf
    UART0_OutString("\r\nControl demo!\r\n");

    // Set P1.0 (LED1) to output
    P1->DIR |= BIT0;
    P1->OUT &= ~BIT0;

    // Set P2.6, P2.7 to be outputs controlled by TIMER_A0
    P2->DIR |= BIT6 | BIT7;
    P2->SEL0 |= BIT6 | BIT7;
    P2->SEL1 &= ~(BIT6 | BIT7);

    // Set P6.2, P10.0, P10.2 to outputs driving LED switches
    // J5 pin 12 P6.2, J5 pin 28 P10.0, J5 pin 30 P10.2
    P6->DIR |= BIT2;
    P6->OUT |= BIT2;
    P10->DIR |= (BIT0|BIT2);
    P10->OUT |= (BIT0|BIT2);

    // initialize P10.4 (ERA), P10.5 (ELA) and make them TIMER_A3 inputs
    P10->SEL0 |= 0x30;
    P10->SEL1 &= ~0x30;                 // configure as Timer A
    P10->DIR &= ~0x30;                  // configure as input

    // Configure P4 (except .1) as GPIO inputs for bumper switches
    P4->SEL0 &= ~0xFD;
    P4->SEL1 &= ~0xFD;
    P4->DIR &= ~0xFD;
    P4->REN |= 0xFD;     // enable pull resistors on P4

    // Configure P5.0 (ERB), P5.2 (ELB) as GPIO inputs to read encoder phase
    P5->SEL0 &= ~(BIT0 | BIT2);
    P5->SEL1 &= ~(BIT0 | BIT2);
    P5->DIR &= ~(BIT0 | BIT2);

    // Configure P5.4 (DIRR) and P5.5 (DIRL) as GPIO outputs controlling motor direction
    P5->SEL0 &= ~(BIT4 | BIT5);
    P5->SEL1 &= ~(BIT4 | BIT5);
    P5->DIR |= (BIT4 | BIT5);
    P5->OUT &= ~(BIT4 | BIT5);

    // Timer_A0 generates motor PWM signals
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

    // Timer_A1 generates scheduled interrupts for IR sensor readout
    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
    TIMER_A1->CCR[0] = 50000;
    TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK, continuous mode
            TIMER_A_CTL_MC__CONTINUOUS;

    // TIMER_A3 maintains encoder position
    TIMER_A3->CTL &= ~0x0030;          // halt Timer A3
    TIMER_A3->CTL = 0x0200;
    TIMER_A3->CCTL[0] = 0x4910;
    TIMER_A3->CCTL[1] = 0x4910;
    TIMER_A3->EX0 &= ~0x0007;       // configure for input clock divider /1
    NVIC->IP[3] = (NVIC->IP[3]&0xFF00FFFF)|0x00400000; // set TIMER_A3 interrupt 0 to priority 2
    NVIC->IP[3] = (NVIC->IP[3]&0x00FFFFFF)|0x40000000; // set TIMER_A3 interrupt N to priority 2
    TIMER_A3->CTL |= 0x0026;        // reset and start Timer A3 in continuous mode

    // enable TIMER_A3 interrupt 0 and N (interrupts 14 & 15)
    // and TIMER_A1 interrupt 0 (interrupt 10) in NVIC
    NVIC->ISER[0] = 0x0000C400;

    __enable_irq();

    char c;
    int open_loop=0;
    uint32_t last_tick;
    int data_len = 0;
    int dump_n, dump_i, bump;
    int bump_start;
    int tick_now = 0;
    int32_t max_move;


    N[0] = 0;
    N[1] = 0;
    Setpoint[0] = 0;
    f_Setpoint[0] = 0.0;
    Setpoint[1] = 1;
    f_Setpoint[0] = 0.0;
    Step_i = -1;
    ErrorSum[0] = 0;
    ErrorSum[1] = 0;
    Move[0] = 0;
    Move[1] = 0;
    Start[0] = 0;
    Start[1] = 0;
    Finish[0] = 0;
    Finish[1] = 0;

    bump_start = (int) (P4->IN&0xFD);
    bump = bump_start;
    while (1) {
        if (tick != last_tick) {
            tick_now = tick;
            bump = (int) (P4->IN&0xFD);
        }
        if (bump != bump_start) {
            Stopped = 1;
        }
        if (Stopped) {
            TIMER_A0->CCR[4] = 0;
            TIMER_A0->CCR[3] = 0;
        }

        if ((tick_now != last_tick) && Stopped) {
            if (0==Move[0] && 0==Move[1]) {
                if (Step_speed>0) {
                    Stopped=1;
                    Step_speed--;
                } else {
                    Stopped=0;
                }
            }
        }

        if (tick_now != last_tick && !Stopped) {

            for (int i=0; i<=1; i++) { // 0 = left, 1 = right
                Error[i] = N[i] - Setpoint[i];
                ErrorSum[i] += Error[i];

                if (ErrorSum[i] < -ERROR_SUM_MAX) {
                    ErrorSum[i] = -ERROR_SUM_MAX;
                }
                if (ErrorSum[i] > ERROR_SUM_MAX) {
                    ErrorSum[i] = ERROR_SUM_MAX;
                }

                P[i] = (int32_t) (((float) (- Error[i]))    * P_COEFF);
                I[i] = (int32_t) (((float) (- ErrorSum[i])) * I_COEFF);

                Out[i] = P[i] + I[i];

                if (Out[i] < -OUT_MAX) {
                    Out[i] = -OUT_MAX;
                }
                if (Out[i] > OUT_MAX) {
                    Out[i] = OUT_MAX;
                }
            }

            // calculate how fast we should move based on how far along the
            // step distance we are, weighted by an easing function
            if (Move[0] != 0) {
                Ease[0] = ease(((float) (N[0]-Start[0])) / ((float) (Move[0])));
            } else {
                Ease[0] = 1.0;
            }
            if (Move[1] != 0) {
                Ease[1] = ease(((float) (N[1]-Start[1])) / ((float) (Move[1])));
            } else {
                Ease[1] = 1.0;
            }
            Ease_min = (Ease[0]<Ease[1]) ? Ease[0] : Ease[1];

            // if our position is close to where we are supposed to be at the end of the
            // step, move to the next step


            if (close(N, Finish, NCH) || 0==Step_speed) {
                Step_i++;
                if (Steps[Step_i*3 + 2] < 0) { // end of sequence, wrap around
                    Step_i = 0;
                }
                Step_speed = Steps[Step_i*3 + 2];
                Move[0] = Steps[Step_i*3 + 0]; // left
                Move[1] = Steps[Step_i*3 + 1]; // right
                Start[0] = N[0];
                Start[1] = N[1];
                Setpoint[0] = N[0];
                Setpoint[1] = N[1];
                f_Setpoint[0] = (float) N[0];
                f_Setpoint[1] = (float) N[1];
                Finish[0] = Finish[0] + Move[0];
                Finish[1] = Finish[1] + Move[1];
                if (0==Move[0] && 0==Move[1]) {
                    Stopped = 1;
                }
            }
            // if we're close to the next incremental setpoint, calculate the next increment
            if (close(N, Setpoint, NCH)) {
                max_move = abs(Move[0]) > abs(Move[1]) ? abs(Move[0]) : abs(Move[1]);
                if (max_move > 0) {
                    f_Setpoint[0] = f_Setpoint[0] + ((float) Move[0]/(float) max_move) * Step_speed * Ease_min;
                    f_Setpoint[1] = f_Setpoint[1] + ((float) Move[1]/(float) max_move) * Step_speed * Ease_min;
                }
                Setpoint[0] = (int32_t) f_Setpoint[0];
                Setpoint[1] = (int32_t) f_Setpoint[1];
            }
            // if increment at the calculated speed would make us overshoot, set
            // setpoint to the finish line
            if ( ((Move[0] > 0) && (Setpoint[0] > Finish[0])) ||
                 ((Move[0] < 0) && (Setpoint[0] < Finish[0])) ) {
                Setpoint[0] = Finish[0];
            }
            if ( ((Move[1] > 0) && (Setpoint[1] > Finish[1])) ||
                 ((Move[1] < 0) && (Setpoint[1] < Finish[1])) ) {
                Setpoint[1] = Finish[1];
            }

            if (!open_loop) {
                // update left wheel output
                if (Out[0] <= 0) {
                    Dir[0] = 1;
                    P5->OUT |= BIT4;  // reverse direction
                    TIMER_A0->CCR[4] = (uint16_t) ((-Out[0]) >> 16);
                } else {
                    Dir[0] = 0;
                    P5->OUT &= ~BIT4;  // forward direction
                    TIMER_A0->CCR[4] = (uint16_t) ((Out[0]) >> 16);
                }
                // update right wheel output
                if (Out[1] <= 0) {
                    Dir[1] = 1;
                    P5->OUT |= BIT5;  // reverse direction
                    TIMER_A0->CCR[3] = (uint16_t) ((-Out[1]) >> 16);
                } else {
                    Dir[1] = 0;
                    P5->OUT &= ~BIT5;  // forward direction
                    TIMER_A0->CCR[3] = (uint16_t) ((Out[1]) >> 16);
                }
            }
        }
        if (tick != last_tick) {
            if (0 == (tick_now % PRINT_INTERVAL)) {
               // printf("tick %7d, PWM_L %9d, Set_L %9.3f, Speed_L %9.3f, Error_L %9.3f, Rev_L %7d, IL %9.3f, PL %9.3f\r\n",   //PWM_R %9d, Set_R %9.3f, Speed_R %9.3f, Error_R %9.3f, Rev_R %9d, I_R %9.3f\r\n",
                 //      tick,       PWM_L,      Setpoint_L, Speed_L,      Error_L,      Rev_L,     IL,       PL); // ,       PWM_R,       Setpoint_R, Speed_R,     Error_R,      Rev_R,      I_R);
            }
            if (0 == (tick_now % DATA_INTERVAL) && 0==Stopped && bump==bump_start ) {
                data_len = Data_i;
                Data[Data_i++ % DATA_LENGTH] = tick;
                Data[Data_i++ % DATA_LENGTH] = Setpoint[0];
                Data[Data_i++ % DATA_LENGTH] = N[0];
                Data[Data_i++ % DATA_LENGTH] = Out[0];
                Data[Data_i++ % DATA_LENGTH] = Error[0];
                Data[Data_i++ % DATA_LENGTH] = Setpoint[1];
                Data[Data_i++ % DATA_LENGTH] = N[1];
                Data[Data_i++ % DATA_LENGTH] = Out[1];
                Data[Data_i++ % DATA_LENGTH] = Error[1];
                data_len = Data_i - data_len;
            }
        }

        if ((EUSCI_A0->IFG&0x01) != 0) {
            c = (char)(EUSCI_A0->RXBUF);
            printf("%c\r\n", c);
            if ('+' == c) {
                Setpoint[0] += 1;
                Setpoint[1] += 1;
            }
            if ('-' == c) {
                Setpoint[0] -= 1;
                Setpoint[1] -= 1;
            }
            if ('0' == c) {
                Stopped = 1-Stopped;
                printf("stopped=%d\r\n", Stopped);
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
                    printf("%d", Data[dump_i++ % DATA_LENGTH]);
                    if (0 == (n % data_len)) {
                        printf("\n");
                    } else {
                        printf("\t");
                    }
                 }
                 printf("\n");
            }
        }
        last_tick = tick_now;
    }
}


// Includes some content from the following:
// UARTtestmain.c
// Daniel and Jonathan Valvano
// September 23, 2017

