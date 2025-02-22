// ADC14.c
// Runs on MSP432, TI-RSLK MAX 1.1
// ADC input, software trigger, 14-bit conversion,
// student version
// Jonathan Valvano
// July 11, 2019

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

#include <stdint.h>
#include "msp.h"
#include "ADC14.h" // "../inc/ADC14.h"

// P4.7 = A6
// single conversion, 3.3V reference
void ADC0_InitSWTriggerCh6(void){
  ADC14->CTL0 &= ~0x00000002;        // 2) ADC14ENC = 0 to allow programming
  while(ADC14->CTL0&0x00010000){};   // 3) wait for BUSY to be zero
  ADC14->CTL0 = 0x04203310;          // 4) single, SMCLK, on, disabled, /1, 32 SHM
  // 31-30 ADC14PDIV  predivider,            00b = Predivide by 1
  // 29-27 ADC14SHSx  SHM source            000b = ADC14SC bit
  // 26    ADC14SHP   SHM pulse-mode          1b = SAMPCON the sampling timer
  // 25    ADC14ISSH  invert sample-and-hold  0b =  not inverted
  // 24-22 ADC14DIVx  clock divider         000b = /1
  // 21-19 ADC14SSELx clock source select   100b = SMCLK
  // 18-17 ADC14CONSEQx mode select          00b = Single-channel, single-conversion
  // 16    ADC14BUSY  ADC14 busy              0b (read only)
  // 15-12 ADC14SHT1x sample-and-hold time 0011b = 32 clocks
  // 11-8  ADC14SHT0x sample-and-hold time 0011b = 32 clocks
  // 7     ADC14MSC   multiple sample         0b = not multiple
  // 6-5   reserved                          00b (reserved)
  // 4     ADC14ON    ADC14 on                1b = powered up
  // 3-2   reserved                          00b (reserved)
  // 1     ADC14ENC   enable conversion       0b = ADC14 disabled
  // 0     ADC14SC    ADC14 start             0b = No start (yet)
  ADC14->CTL1 = 0x00000030;          // 5) ADC14MEM0, 14-bit, ref on, regular power
  // 20-16 STARTADDx  start addr          00000b = ADC14MEM0
  // 15-6  reserved                  0000000000b (reserved)
  // 5-4   ADC14RES   ADC14 resolution       11b = 14 bit, 16 clocks
  // 3     ADC14DF    data read-back format   0b = Binary unsigned
  // 2     REFBURST   reference buffer burst  0b = reference on continuously
  // 1-0   ADC14PWRMD ADC power modes        00b = Regular power mode
  ADC14->MCTL[0] = 0x00000086;         // 6) 0 to 3.3V, channel 6
  // 15   ADC14WINCTH Window comp threshold   0b = not used
  // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
  // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
  // 12   reserved                            0b (reserved)
  // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = AVCC, V(R-) = AVSS
  // 7    ADC14EOS    End of sequence         1b = End of sequence
  // 6-5  reserved                           00b (reserved)
  // 4-0  ADC14INCHx  Input channel       00110b = A6, P4.7

  ADC14->IER0 = 0; // 7) no interrupts
  ADC14->IER1 = 0; // no interrupts
  P4->SEL1 |= 0x80;                  // 8) analog mode on A6, P4.7
  P4->SEL0 |= 0x80;
  ADC14->CTL0 |= 0x00000002;         // 9) enable
}

// ADC14IFGR0 bit 0 is set when P4.7 = A6 conversion done
//                  cleared on read ADC14MEM0
// ADC14CLRIFGR0 bit 0, write 1 to clear flag
// ADC14IVx is 0x0C when ADC14MEM0 interrupt flag; Interrupt Flag: ADC14IFG0
// ADC14MEM0 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
uint32_t ADC_In6(void){
  while(ADC14->CTL0&0x00010000){};    // 1) wait for BUSY to be zero
  ADC14->CTL0 |= 0x00000001;          // 2) start single conversion
  while((ADC14->IFGR0&0x01) == 0){};  // 3) wait for ADC14IFG0
  return ADC14->MEM[0];      // 4) return result 0 to 16383
}

// P4.6 = A7
// P4.7 = A6
// 3.3V reference
void ADC0_InitSWTriggerCh67(void){
  ADC14->CTL0 &= ~0x00000002;      // 2) ADC14ENC = 0 to allow programming
  while(ADC14->CTL0&0x00010000){}; // 3) wait for BUSY to be zero
  ADC14->CTL0 = 0x04223390;        // 4) single, SMCLK, on, disabled, /1, 32 SHM
  // 31-30 ADC14PDIV  predivider,            00b = Predivide by 1
  // 29-27 ADC14SHSx  SHM source            000b = ADC14SC bit
  // 26    ADC14SHP   SHM pulse-mode          1b = SAMPCON the sampling timer
  // 25    ADC14ISSH  invert sample-and-hold  0b = not inverted
  // 24-22 ADC14DIVx  clock divider         000b = /1
  // 21-19 ADC14SSELx clock source select   100b = SMCLK
  // 18-17 ADC14CONSEQx mode select          01b = Sequence-of-channels
  // 16    ADC14BUSY  ADC14 busy              0b (read only)
  // 15-12 ADC14SHT1x sample-and-hold time 0011b = 32 clocks
  // 11-8  ADC14SHT0x sample-and-hold time 0011b = 32 clocks
  // 7     ADC14MSC   multiple sample         1b = continue conversions automatically after first SHI signal trigger
  // 6-5   reserved                          00b (reserved)
  // 4     ADC14ON    ADC14 on                1b = powered up
  // 3-2   reserved                          00b (reserved)
  // 1     ADC14ENC   enable conversion       0b = ADC14 disabled
  // 0     ADC14SC    ADC14 start             0b = No start (yet)
  ADC14->CTL1 = 0x00000030;        // 5) ADC14MEM0, 14-bit, ref on, regular power
  // 20-16 STARTADDx  start addr          00000b = ADC14MEM0
  // 15-6  reserved                  0000000000b (reserved)
  // 5-4   ADC14RES   ADC14 resolution       11b = 14 bit, 16 clocks
  // 3     ADC14DF    data read-back format   0b = Binary unsigned
  // 2     REFBURST   reference buffer burst  0b = reference on continuously
  // 1-0   ADC14PWRMD ADC power modes        00b = Regular power mode
  ADC14->MCTL[0] = 0x00000006;     // 6a) 0 to 3.3V, channel 6
  // 15   ADC14WINCTH Window comp threshold   0b = not used
  // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
  // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
  // 12   reserved                            0b (reserved)
  // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = AVCC, V(R-) = AVSS
  // 7    ADC14EOS    End of sequence         0b = Not end of sequence
  // 6-5  reserved                           00b (reserved)
  // 4-0  ADC14INCHx  Input channel        0110b = A6, P4.7
  ADC14->MCTL[1] = 0x00000087;     // 6b) 0 to 3.3V, channel 7
  // 15   ADC14WINCTH Window comp threshold   0b = not used
  // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
  // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
  // 12   reserved                            0b (reserved)
  // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = AVCC, V(R-) = AVSS
  // 7    ADC14EOS    End of sequence         1b = End of sequence
  // 6-5  reserved                           00b (reserved)
  // 4-0  ADC14INCHx  Input channel        0111b = A7, P4.6

  ADC14->IER0 = 0;                 // 7) no interrupts
  ADC14->IER1 = 0;                 //    no interrupts
  P4->SEL1 |= 0xC0;                // 8) analog mode on P4.7/A6 and P4.6/A7
  P4->SEL0 |= 0xC0;
  ADC14->CTL0 |= 0x00000002;       // 9) enable
}

// ADC14IFGR0 bit 1 is set when conversion done
//                  cleared on read ADC14MEM1
// ADC14CLRIFGR0 bit 1, write 1 to clear flag
// ADC14IVx is 0x0E when ADC14MEM1 interrupt flag; Interrupt Flag: ADC14IFG1
// ADC14MEM0 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
// ADC14MEM1 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
void ADC_In67(uint32_t *ch6, uint32_t *ch7){
  while(ADC14->CTL0&0x00010000){}; // 1) wait for BUSY to be zero
  ADC14->CTL0 |= 0x00000001;       // 2) start single conversion
                                   // 3) wait for ADC14IFG1
  while((ADC14->IFGR0&0x02) == 0){};
  *ch6 = ADC14->MEM[0];            // 4) P4.7/A6 result 0 to 16383
  *ch7 = ADC14->MEM[1];            //    P4.6/A7 result 0 to 16383
}

//**********software below is part of Lab 15**************
// P6.1 = A14
// single conversion, 3.3V reference, RSLK1.1
void ADC0_InitSWTriggerCh14(void){
    // write this for Lab 15
}

// ADC14IFGR0 bit 0 is set when P6.1 = A14 conversion done
//                  cleared on read ADC14MEM0
// ADC14CLRIFGR0 bit 0, write 1 to clear flag
// ADC14IVx is 0x0C when ADC14MEM0 interrupt flag; Interrupt Flag: ADC14IFG0
// ADC14MEM0 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
uint32_t ADC_In14(void){
    // write this for Lab 15
	return 0; // replace this line
}

// P9.0 = A17
// P6.1 = A14
// P9.1 = A16
// Lab 15 assignment RSLK 1.1, use software trigger, 3.3V reference, RSLK1.1
void ADC0_InitSWTriggerCh17_14_16(void){
// you write this as part of Lab 15 RSLK 1.1
// you can use any of the MEM[n], MCTL[n] except n=6 (6 is used by TExaS)

}

// ADC14IFGR0 bit 4 is set when conversion done
//                  cleared on read ADC14MEM4
// ADC14CLRIFGR0 bit 4, write 1 to clear flag
// ADC14IVx is 0x14 when ADC14MEM4 interrupt flag; Interrupt Flag: ADC14IFG4
// ADC14MEM2 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
// ADC14MEM3 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
// ADC14MEM4 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
// Lab 15 assignment RSLK 1.1, use software trigger, 3.3V reference
void ADC_In17_14_16(uint32_t *ch17, uint32_t *ch14, uint32_t *ch16){
// you write this as part of Lab 15 RSLK 1.1

}

// P8.4 = A21
// P8.3 = A22
// P8.2 = A23
// use software trigger, 3.3V reference
void ADC0_InitSWTriggerCh21_22_23(void){
// you can use any of the MEM[n], MCTL[n] except n=6 (6 is used by TExaS)
                                   // 1) reference module is not used here
  ADC14->CTL0 &= ~0x00000002;      // 2) ADC14ENC = 0 to allow programming
  while(ADC14->CTL0&0x00010000){}; // 3) wait for BUSY to be zero
  ADC14->CTL0 = 0x04223390;        // 4) single, SMCLK, on, disabled, /1, 32 SHM
  // 31-30 ADC14PDIV  predivider,            00b = Predivide by 1
  // 29-27 ADC14SHSx  SHM source            000b = ADC14SC bit
  // 26    ADC14SHP   SHM pulse-mode          1b = SAMPCON the sampling timer
  // 25    ADC14ISSH  invert sample-and-hold  0b = not inverted
  // 24-22 ADC14DIVx  clock divider         000b = /1
  // 21-19 ADC14SSELx clock source select   100b = SMCLK
  // 18-17 ADC14CONSEQx mode select          01b = Sequence-of-channels
  // 16    ADC14BUSY  ADC14 busy              0b (read only)
  // 15-12 ADC14SHT1x sample-and-hold time 0011b = 32 clocks
  // 11-8  ADC14SHT0x sample-and-hold time 0011b = 32 clocks
  // 7     ADC14MSC   multiple sample         1b = continue conversions automatically after first SHI signal trigger
  // 6-5   reserved                          00b (reserved)
  // 4     ADC14ON    ADC14 on                1b = powered up
  // 3-2   reserved                          00b (reserved)
  // 1     ADC14ENC   enable conversion       0b = ADC14 disabled
  // 0     ADC14SC    ADC14 start             0b = No start (yet)
  ADC14->CTL1 = 0x00020030;        // 5) ADC14MEM2, 14-bit, ref on, regular power
  // 20-16 STARTADDx  start addr          00010b = ADC14MEM2
  // 15-6  reserved                  0000000000b (reserved)
  // 5-4   ADC14RES   ADC14 resolution       11b = 14 bit, 16 clocks
  // 3     ADC14DF    data read-back format   0b = Binary unsigned
  // 2     REFBURST   reference buffer burst  0b = reference on continuously
  // 1-0   ADC14PWRMD ADC power modes        00b = Regular power mode
  ADC14->MCTL[2] = 0x00000015;     // 6a) 0 to 3.3V, channel 21
  // 15   ADC14WINCTH Window comp threshold   0b = not used
  // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
  // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
  // 12   reserved                            0b (reserved)
  // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = AVCC, V(R-) = AVSS
  // 7    ADC14EOS    End of sequence         0b = Not end of sequence
  // 6-5  reserved                           00b (reserved)
  // 4-0  ADC14INCHx  Input channel       10101b = A21, P8.4
  ADC14->MCTL[3] = 0x00000016;     // 6b) 0 to 3.3V, channel 22
  // 15   ADC14WINCTH Window comp threshold   0b = not used
  // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
  // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
  // 12   reserved                            0b (reserved)
  // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = VCC, V(R-) = AVSS
  // 7    ADC14EOS    End of sequence         0b = Not end of sequence
  // 6-5  reserved                           00b (reserved)
  // 4-0  ADC14INCHx  Input channel       10110b = A22, P8.3
  ADC14->MCTL[4] = 0x00000097;     // 6c) 0 to 3.3V, channel 23, end of sequence
  // 15   ADC14WINCTH Window comp threshold   0b = not used
  // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
  // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
  // 12   reserved                            0b (reserved)
  // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = VCC, V(R-) = AVSS
  // 7    ADC14EOS    End of sequence         1b = End of sequence
  // 6-5  reserved                           00b (reserved)
  // 4-0  ADC14INCHx  Input channel       10111b = A23, P8.2
  ADC14->IER0 = 0;                 // 7) no interrupts
  ADC14->IER1 = 0;                 //    no interrupts
  P8->SEL1 |= 0x1C;                // 8) analog mode on P8.4/A21, P8.3/A22, and P8.2/A23
  P8->SEL0 |= 0x1C;
  ADC14->CTL0 |= 0x00000002;       // 9) enable
}

// ADC14IFGR0 bit 4 is set when conversion done
//                  cleared on read ADC14MEM4
// ADC14CLRIFGR0 bit 4, write 1 to clear flag
// ADC14IVx is 0x14 when ADC14MEM4 interrupt flag; Interrupt Flag: ADC14IFG4
// ADC14MEM2 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
// ADC14MEM3 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
// ADC14MEM4 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
void ADC_In21_22_23(uint32_t *ch21, uint32_t *ch22, uint32_t *ch23){
// 1) wait for BUSY to be zero
  while(ADC14->CTL0&0x00010000){};
// 2) start single conversion
  ADC14->CTL0 |= 0x00000001;
// 3) wait for ADC14IFG4
  while((ADC14->IFGR0&0x10) == 0){};
// 4) read three results from MEM
  *ch21 = ADC14->MEM[2];           // P8.4/A21 result 0 to 16383
  *ch22 = ADC14->MEM[3];           // P8.3/A22 result 0 to 16383
  *ch23 = ADC14->MEM[4];           // P8.2/A23 result 0 to 16383
}

void ADC0_InitSWTriggerCh17_21_23(void){
// you can use any of the MEM[n], MCTL[n] except n=6 (6 is used by TExaS)
                                   // 1) reference module is not used here
  ADC14->CTL0 &= ~0x00000002;      // 2) ADC14ENC = 0 to allow programming
  while(ADC14->CTL0&0x00010000){}; // 3) wait for BUSY to be zero
  ADC14->CTL0 = 0x04223390;        // 4) single, SMCLK, on, disabled, /1, 32 SHM
  // 31-30 ADC14PDIV  predivider,            00b = Predivide by 1
  // 29-27 ADC14SHSx  SHM source            000b = ADC14SC bit
  // 26    ADC14SHP   SHM pulse-mode          1b = SAMPCON the sampling timer
  // 25    ADC14ISSH  invert sample-and-hold  0b = not inverted
  // 24-22 ADC14DIVx  clock divider         000b = /1
  // 21-19 ADC14SSELx clock source select   100b = SMCLK
  // 18-17 ADC14CONSEQx mode select          01b = Sequence-of-channels
  // 16    ADC14BUSY  ADC14 busy              0b (read only)
  // 15-12 ADC14SHT1x sample-and-hold time 0011b = 32 clocks
  // 11-8  ADC14SHT0x sample-and-hold time 0011b = 32 clocks
  // 7     ADC14MSC   multiple sample         1b = continue conversions automatically after first SHI signal trigger
  // 6-5   reserved                          00b (reserved)
  // 4     ADC14ON    ADC14 on                1b = powered up
  // 3-2   reserved                          00b (reserved)
  // 1     ADC14ENC   enable conversion       0b = ADC14 disabled
  // 0     ADC14SC    ADC14 start             0b = No start (yet)
  ADC14->CTL1 = 0x00020030;        // 5) ADC14MEM2, 14-bit, ref on, regular power
  // 20-16 STARTADDx  start addr          00010b = ADC14MEM2
  // 15-6  reserved                  0000000000b (reserved)
  // 5-4   ADC14RES   ADC14 resolution       11b = 14 bit, 16 clocks
  // 3     ADC14DF    data read-back format   0b = Binary unsigned
  // 2     REFBURST   reference buffer burst  0b = reference on continuously
  // 1-0   ADC14PWRMD ADC power modes        00b = Regular power mode
  ADC14->MCTL[3] = 0x00000015;     // 6a) 0 to 3.3V, channel 21
  // 15   ADC14WINCTH Window comp threshold   0b = not used
  // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
  // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
  // 12   reserved                            0b (reserved)
  // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = AVCC, V(R-) = AVSS
  // 7    ADC14EOS    End of sequence         0b = Not end of sequence
  // 6-5  reserved                           00b (reserved)
  // 4-0  ADC14INCHx  Input channel       10101b = A21, P8.4
  ADC14->MCTL[2] = 0x00000011;     // 6b) 0 to 3.3V, channel 17
  // 15   ADC14WINCTH Window comp threshold   0b = not used
  // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
  // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
  // 12   reserved                            0b (reserved)
  // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = VCC, V(R-) = AVSS
  // 7    ADC14EOS    End of sequence         0b = Not end of sequence
  // 6-5  reserved                           00b (reserved)
  // 4-0  ADC14INCHx  Input channel       10001b = A17, P9.0
  ADC14->MCTL[4] = 0x00000097;     // 6c) 0 to 3.3V, channel 23, end of sequence
  // 15   ADC14WINCTH Window comp threshold   0b = not used
  // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
  // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
  // 12   reserved                            0b (reserved)
  // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = VCC, V(R-) = AVSS
  // 7    ADC14EOS    End of sequence         1b = End of sequence
  // 6-5  reserved                           00b (reserved)
  // 4-0  ADC14INCHx  Input channel       10111b = A23, P8.2
  ADC14->IER0 = 0;                 // 7) no interrupts
  ADC14->IER1 = 0;                 //    no interrupts
  P8->SEL1 |= 0x14;                // 8) analog mode on P8.4/A21, P8.2/A23
  P8->SEL0 |= 0x14;
  P9->SEL1 |= 0x01;                // analog mode on P9.0
  P9->SEL1 |= 0x01;
  ADC14->CTL0 |= 0x00000002;       // 9) enable
}

void ADC_In17_21_23(uint32_t *ch17, uint32_t *ch21, uint32_t *ch23){
// 1) wait for BUSY to be zero
  while(ADC14->CTL0&0x00010000){};
// 2) start single conversion
  ADC14->CTL0 |= 0x00000001;
// 3) wait for ADC14IFG4
  while((ADC14->IFGR0&0x10) == 0){};
// 4) read three results from MEM
  *ch17 = ADC14->MEM[2];           // P9.0/A17 result 0 to 16383
  *ch21 = ADC14->MEM[3];           // P8.4/A21 result 0 to 16383
  *ch23 = ADC14->MEM[4];           // P8.2/A23 result 0 to 16383
}

// P4.1 = A12
// single conversion, 3.3V reference
void ADC0_InitSWTriggerCh12(void){
      // write this for Lab 15

}

// ADC14IFGR0 bit 0 is set when P4.1 = A12 conversion done
//                  cleared on read ADC14MEM0
// ADC14CLRIFGR0 bit 0, write 1 to clear flag
// ADC14IVx is 0x0C when ADC14MEM0 interrupt flag; Interrupt Flag: ADC14IFG0
// ADC14MEM0 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
uint32_t ADC_In12(void){
    // write this for Lab 15
  return 0; // replace this line
}


// channel 19, P8.6
// single conversion, 3.3V reference
void ADC0_InitSWTriggerCh19(void){
  ADC14->CTL0 &= ~0x00000002;        // 2) ADC14ENC = 0 to allow programming
  while(ADC14->CTL0&0x00010000){};   // 3) wait for BUSY to be zero
  ADC14->CTL0 = 0x04203310;          // 4) single, SMCLK, on, disabled, /1, 32 SHM
  // 31-30 ADC14PDIV  predivider,            00b = Predivide by 1
  // 29-27 ADC14SHSx  SHM source            000b = ADC14SC bit
  // 26    ADC14SHP   SHM pulse-mode          1b = SAMPCON the sampling timer
  // 25    ADC14ISSH  invert sample-and-hold  0b =  not inverted
  // 24-22 ADC14DIVx  clock divider         000b = /1
  // 21-19 ADC14SSELx clock source select   100b = SMCLK
  // 18-17 ADC14CONSEQx mode select          00b = Single-channel, single-conversion
  // 16    ADC14BUSY  ADC14 busy              0b (read only)
  // 15-12 ADC14SHT1x sample-and-hold time 0011b = 32 clocks
  // 11-8  ADC14SHT0x sample-and-hold time 0011b = 32 clocks
  // 7     ADC14MSC   multiple sample         0b = not multiple
  // 6-5   reserved                          00b (reserved)
  // 4     ADC14ON    ADC14 on                1b = powered up
  // 3-2   reserved                          00b (reserved)
  // 1     ADC14ENC   enable conversion       0b = ADC14 disabled
  // 0     ADC14SC    ADC14 start             0b = No start (yet)
  ADC14->CTL1 = 0x00000030;          // 5) ADC14MEM0, 14-bit, ref on, regular power
  // 20-16 STARTADDx  start addr          00000b = ADC14MEM0
  // 15-6  reserved                  0000000000b (reserved)
  // 5-4   ADC14RES   ADC14 resolution       11b = 14 bit, 16 clocks
  // 3     ADC14DF    data read-back format   0b = Binary unsigned
  // 2     REFBURST   reference buffer burst  0b = reference on continuously
  // 1-0   ADC14PWRMD ADC power modes        00b = Regular power mode
  ADC14->MCTL[0] = 0x00000093;         // 6) 0 to 3.3V, channel 19=0x13
  // 15   ADC14WINCTH Window comp threshold   0b = not used
  // 14   ADC14WINC   Comparator enable       0b = Comparator disabled
  // 13   ADC14DIF    Differential mode       0b = Single-ended mode enabled
  // 12   reserved                            0b (reserved)
  // 11-8 ADC14VRSEL  V(R+) and V(R-)      0000b = V(R+) = AVCC, V(R-) = AVSS
  // 7    ADC14EOS    End of sequence         1b = End of sequence
  // 6-5  reserved                           00b (reserved)
  // 4-0  ADC14INCHx  Input channel       10011b = A19, P8.6

  ADC14->IER0 = 0; // 7) no interrupts
  ADC14->IER1 = 0; // no interrupts
  P8->SEL1 |= 0x40;                  // 8) analog mode on A19, P8.6
  P8->SEL0 |= 0x40;
  ADC14->CTL0 |= 0x00000002;         // 9) enable
}

// ADC14IFGR0 bit 0 is set when P4.7 = A6 conversion done
//                  cleared on read ADC14MEM0
// ADC14CLRIFGR0 bit 0, write 1 to clear flag
// ADC14IVx is 0x0C when ADC14MEM0 interrupt flag; Interrupt Flag: ADC14IFG0
// ADC14MEM0 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
uint32_t ADC_In19(void){
  while(ADC14->CTL0&0x00010000){};    // 1) wait for BUSY to be zero
  ADC14->CTL0 |= 0x00000001;          // 2) start single conversion
  while((ADC14->IFGR0&0x01) == 0){};  // 3) wait for ADC14IFG0
  return ADC14->MEM[0];      // 4) return result 0 to 16383
}

// P8.2 = A23
// use software trigger, 3.3V reference
void ADC0_InitSWTriggerCh23(void){
// you can use any of the MEM[n], MCTL[n] except n=6 (6 is used by TExaS)
    // write this for Lab 15
}

// ADC14IFGR0 bit 0 is set when P8.2 = A23 conversion done
//                  cleared on read ADC14MEM0
// ADC14CLRIFGR0 bit 0, write 1 to clear flag
// ADC14IVx is 0x0C when ADC14MEM0 interrupt flag; Interrupt Flag: ADC14IFG0
// ADC14MEM0 14-bit conversion in bits 13-0 (31-16 undefined, 15-14 zero)
uint32_t ADC_In23(void){
    // write this for Lab 15
  return 0; // replace this line
}

