/* 
 * FreeBSD License
 * Copyright (c) 2016, Guenael 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */


#include "cpu.h"
#include "pll.h"

#include "twi.h"
#include "usart.h"

#include <avr/io.h>
#include <math.h>   // FIXME: Floor bypass with simple cast?
#include <util/delay.h>


#define NUM_REGS_MAX 8

typedef struct Reg_Data{
   unsigned char Reg_Addr;
   unsigned char Reg_Val;
} Reg_Data;

Reg_Data const Reg_Store1[NUM_REGS_MAX] = {
{ 26,0x89},
{ 27,0x68},
{ 28,0x00},
{ 29,0x29},
{ 30,0x59},
{ 31,0x95},
{ 32,0xBB},
{ 33,0x58},
};

Reg_Data const Reg_Store2[NUM_REGS_MAX] = {
{ 26,0x89},
{ 27,0x68},
{ 28,0x00},
{ 29,0x29},
{ 30,0x59},
{ 31,0x95},
{ 32,0xBB},
{ 33,0xD8},
};


/* Global definition for the I2C GPS address */
static uint8_t pllAddr;    

static uint8_t pllBankSettings[4][8]; 


void pllSetAddr(uint8_t addr) {
    pllAddr = addr;
}


void pllPushA() {
    for (uint8_t i=0; i<NUM_REGS_MAX; i++) { // Skip the header for the checksum
        pllSendRegister(Reg_Store1[i].Reg_Addr, Reg_Store1[i].Reg_Val);
        //twi_writeTo(pllAddr, &Reg_Store[i], 2, 1, 0);
    }
}


void pllPushB() {
    for (uint8_t i=0; i<NUM_REGS_MAX; i++) { // Skip the header for the checksum
        pllSendRegister(Reg_Store2[i].Reg_Addr, Reg_Store2[i].Reg_Val);
        //twi_writeTo(pllAddr, &Reg_Store[i], 2, 1, 0);
    }
}


void pllInit() {
    DDRB   |= _BV(DDB2);    // PLL LE - Enable output
    PORTB  &= ~_BV(PORTB2);  // Enable PLL

    /* PA output status port -- NOT USED */
    DDRD |= _BV(DDD6);

    /* PA output disable at start */
    PORTD &= ~_BV(PORTD6);

    /*
    for (uint8_t i=0; i<NUM_REGS_MAX; i++) { // Skip the header for the checksum
        pllSendRegister(Reg_Store1[i].Reg_Addr, Reg_Store[i].Reg_Val);
        //twi_writeTo(pllAddr, &Reg_Store[i], 2, 1, 0);
    */

    pllSendRegister(SI_CLK_ENABLE, 0xFF);     // Disable all output

    pllSendRegister(SI_PLL_INPUT_SRC, 0x04);  // FIXME -- Debug avec XTAL first

    pllSendRegister(SI_CLK0_CONTROL, 0x4F);   // FIXME -- config option todo
    pllSendRegister(SI_CLK1_CONTROL, 0x84);   // Turn off
    pllSendRegister(SI_CLK2_CONTROL, 0x84);   // Turn off 
    pllSendRegister(SI_CLK3_CONTROL, 0x84);   // Turn off
    pllSendRegister(SI_CLK4_CONTROL, 0x84);   // Turn off
    pllSendRegister(SI_CLK5_CONTROL, 0x84);   // Turn off
    pllSendRegister(SI_CLK6_CONTROL, 0x84);   // Turn off
    pllSendRegister(SI_CLK7_CONTROL, 0x84);   // Turn off

    pllSendRegister(SI_CLK_ENABLE, 0xFE);     // Disable all output exept CLK0 (CLK0_OEB)
    // pllSendRegister(SI_VCXO_PARAM, 0x??);  // TODO VCXO balloon

    // Diviser -- toujours 1
    pllSendRegister(SI_SYNTH_MS_0 + 1, 0x01); 
    pllSendRegister(SI_SYNTH_MS_0 + 3, 0x01); 
}


void pllShutdown() {
}


void pllSendRegister(uint8_t reg, uint8_t data) {
    uint8_t tmp[2] = {0};
    tmp[0] = reg;
    tmp[1] = data;

    twi_writeTo(pllAddr, tmp, sizeof(tmp), 1, 0);
}


void pllUpdate(uint8_t bank) {
    pllSendRegister(SI_CLK0_CONTROL, 0x84); // Turn off // glitch

    pllSendRegister(SI_SYNTH_PLL_A + 0, pllBankSettings[bank][0]);
    pllSendRegister(SI_SYNTH_PLL_A + 1, pllBankSettings[bank][1]);
    pllSendRegister(SI_SYNTH_PLL_A + 2, pllBankSettings[bank][2]);
    pllSendRegister(SI_SYNTH_PLL_A + 3, pllBankSettings[bank][3]);
    pllSendRegister(SI_SYNTH_PLL_A + 4, pllBankSettings[bank][4]);
    pllSendRegister(SI_SYNTH_PLL_A + 5, pllBankSettings[bank][5]);
    pllSendRegister(SI_SYNTH_PLL_A + 6, pllBankSettings[bank][6]);
    pllSendRegister(SI_SYNTH_PLL_A + 7, pllBankSettings[bank][7]);

    pllSendRegister(SI_PLL_RESET, 0xA0);    // Reset both PLL // glitch ??
    pllSendRegister(SI_CLK0_CONTROL, 0x4F); // Turn on // glitch ??

    _delay_us(468);  // Align ...
}


void pllSetFreq(uint32_t freq, uint8_t bank) { // ATTENTION : uint64_t vs uint32_t
    uint32_t xtalFreq = XTAL_FREQ;

    uint32_t divider = 900000000 / freq;// Calculate the division ratio. 900,000,000 is the maximum internal 
                                    // PLL frequency: 900MHz
    if (divider % 2) divider--;     // Ensure an even integer division ratio

    uint32_t pllFreq = divider * freq;  // Calculate the pllFrequency: the divider * desired output frequency

    uint8_t mult = pllFreq / xtalFreq;      // Determine the multiplier to get to the required pllFrequency
    uint32_t l = pllFreq % xtalFreq;         // It has three parts:
    float f = l;                          // mult is an integer that must be in the range 15..90
    f *= 1048575;                   // num and denom are the fractional parts, the numerator and denominator
    f /= xtalFreq;                  // each is 20 bits (range 0..1048575)
    uint32_t num = f;                        // the actual multiplier is  mult + num / denom
    uint32_t denom = 1048575;                // For simplicity we set the denominator to the maximum 1048575


    uint32_t P1,P2,P3;
    P1 = (uint32_t)(128 * ((float)num / (float)denom));
    P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
    P2 = (uint32_t)(128 * ((float)num / (float)denom));
    P2 = (uint32_t)(128 * num - denom * P2);
    P3 = denom;

    /* Packing */
    pllBankSettings[bank][0] = (P3 & 0x0000FF00) >> 8;
    pllBankSettings[bank][1] = (P3 & 0x000000FF);
    pllBankSettings[bank][2] = (P1 & 0x00030000) >> 16;
    pllBankSettings[bank][3] = (P1 & 0x0000FF00) >> 8;
    pllBankSettings[bank][4] = (P1 & 0x000000FF);
    pllBankSettings[bank][5] = ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
    pllBankSettings[bank][6] = (P2 & 0x0000FF00) >> 8;
    pllBankSettings[bank][7] = (P2 & 0x000000FF);
}


void pllRfOutput(uint8_t enable) {
    if (enable) {
        pllSendRegister(SI_CLK_ENABLE, 0xFE);
        //PORTB |= _BV(PORTB2);
    }
    else {
        pllSendRegister(SI_CLK_ENABLE, 0xFF);
        //PORTB &= ~_BV(PORTB2);
    }
}


// Not USED !!!
void pllPA(uint8_t enable) {
    if (enable)
        PORTD |= _BV(PORTD6);
    else
        PORTD &= ~_BV(PORTD6);
}
