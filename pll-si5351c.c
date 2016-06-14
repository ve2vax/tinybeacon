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
#include "pll-si5351c.h"

#include "twi.h"
#include "usart.h"

#include <avr/io.h>
#include <math.h>   // FIXME: Floor bypass with simple cast?
#include <util/delay.h>


/* Global definition for the I2C GPS address */
static uint8_t pll_si5351c_Addr;

static uint8_t pll_si5351c_BankSettings[4][8];


void pll_si5351c_SetAddr(uint8_t addr) {
    pll_si5351c_Addr = addr;
}


void pll_si5351c_Init() {
    DDRB   |= _BV(DDB2);    // PLL LE - Enable output
    PORTB  &= ~_BV(PORTB2);  // Enable PLL
    _delay_ms(100);

    pll_si5351c_SendRegister(SI_CLK_ENABLE, 0xFF);       // Disable all output

    pll_si5351c_SendRegister(SI_PLL_INPUT_SRC, 0x04);    // FIXME -- Debug avec XTAL first

    pll_si5351c_SendRegister(SI_CLK_CONTROL+0, 0x4F);   // FIXME -- config option todo
    pll_si5351c_SendRegister(SI_CLK_CONTROL+1, 0x84);   // Turn off
    pll_si5351c_SendRegister(SI_CLK_CONTROL+2, 0x84);   // Turn off
    pll_si5351c_SendRegister(SI_CLK_CONTROL+3, 0x84);   // Turn off
    pll_si5351c_SendRegister(SI_CLK_CONTROL+4, 0x84);   // Turn off
    pll_si5351c_SendRegister(SI_CLK_CONTROL+5, 0x84);   // Turn off
    pll_si5351c_SendRegister(SI_CLK_CONTROL+6, 0x84);   // Turn off
    pll_si5351c_SendRegister(SI_CLK_CONTROL+7, 0x84);   // Turn off
    
    pll_si5351c_SendRegister(SI_SYNTH_MS_0+0, 0x00);
    pll_si5351c_SendRegister(SI_SYNTH_MS_0+1, 0x01);
    pll_si5351c_SendRegister(SI_SYNTH_MS_0+2, 0x00);
    pll_si5351c_SendRegister(SI_SYNTH_MS_0+3, 0x01);
    pll_si5351c_SendRegister(SI_SYNTH_MS_0+4, 0x00);
    pll_si5351c_SendRegister(SI_SYNTH_MS_0+5, 0x00);
    pll_si5351c_SendRegister(SI_SYNTH_MS_0+6, 0x00);
    pll_si5351c_SendRegister(SI_SYNTH_MS_0+7, 0x00);

    pll_si5351c_SendRegister(SI_CLK_ENABLE, 0xFE);     // Disable all output exept CLK0 (CLK0_OEB)
}


void pll_si5351c_Shutdown() {
}


void pll_si5351c_SendRegister(uint8_t reg, uint8_t data) {
    uint8_t tmp[2] = {0};
    tmp[0] = reg;
    tmp[1] = data;

    twi_writeTo(pll_si5351c_Addr, tmp, sizeof(tmp), 1, 0);
}


void pll_si5351c_Update(uint8_t bank) {
    pll_si5351c_SendRegister(SI_CLK_CONTROL, 0x84); // Turn off // glitch

    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 0, pll_si5351c_BankSettings[bank][0]);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 1, pll_si5351c_BankSettings[bank][1]);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 2, pll_si5351c_BankSettings[bank][2]);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 3, pll_si5351c_BankSettings[bank][3]);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 4, pll_si5351c_BankSettings[bank][4]);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 5, pll_si5351c_BankSettings[bank][5]);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 6, pll_si5351c_BankSettings[bank][6]);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 7, pll_si5351c_BankSettings[bank][7]);

    pll_si5351c_SendRegister(SI_PLL_RESET, 0xA0);    // Reset both PLL // glitch ??
    pll_si5351c_SendRegister(SI_CLK_CONTROL, 0x4F); // Turn on // glitch ??

    _delay_us(468);  // Align ...
}


void pll_si5351c_Update1(uint8_t bank) {
    pll_si5351c_SendRegister(SI_CLK_CONTROL, 0x84); // Turn off // glitch

    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 0, 0xA1);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 1, 0x20);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 2, 0x00);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 3, 0x29);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 4, 0x58);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 5, 0x76);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 6, 0x5A);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 7, 0x80);

    pll_si5351c_SendRegister(SI_PLL_RESET, 0xA0);    // Reset both PLL // glitch ??
    pll_si5351c_SendRegister(SI_CLK_CONTROL, 0x4F); // Turn on // glitch ??

    _delay_us(468);  // Align ...
}


void pll_si5351c_Update2(uint8_t bank) {
    pll_si5351c_SendRegister(SI_CLK_CONTROL, 0x84); // Turn off // glitch

    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 0, 0x01);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 1, 0xF4);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 2, 0x00);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 3, 0x29);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 4, 0x58);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 5, 0x00);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 6, 0x01);
    pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 7, 0xA0);

    pll_si5351c_SendRegister(SI_PLL_RESET, 0xA0);    // Reset both PLL // glitch ??
    pll_si5351c_SendRegister(SI_CLK_CONTROL, 0x4F); // Turn on // glitch ??

    _delay_us(468);  // Align ...
}


void pll_si5351c_SetFreq(uint32_t freq, uint8_t bank) { // ATTENTION : uint64_t vs uint32_t
    uint32_t xtalFreq = XTAL_FREQ;

    uint32_t divider = 900000000 / freq;// Calculate the division ratio. 900,000,000 is the maximum internal
    // PLL frequency: 900MHz
    if (divider % 2) divider--;         // Ensure an even integer division ratio

    uint32_t pllFreq = divider * freq;  // Calculate the pllFrequency: the divider * desired output frequency

    uint8_t mult = pllFreq / xtalFreq;  // Determine the multiplier to get to the required pllFrequency
    uint32_t l = pllFreq % xtalFreq;    // It has three parts:
    float f = l;                        // mult is an integer that must be in the range 15..90
    f *= 1048575;                       // num and denom are the fractional parts, the numerator and denominator
    f /= xtalFreq;                      // each is 20 bits (range 0..1048575)
    uint32_t num = f;                   // the actual multiplier is  mult + num / denom
    uint32_t denom = 1048575;           // For simplicity we set the denominator to the maximum 1048575


    uint32_t P1,P2,P3;
    P1 = (uint32_t)(128 * ((float)num / (float)denom));
    P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
    P2 = (uint32_t)(128 * ((float)num / (float)denom));
    P2 = (uint32_t)(128 * num - denom * P2);
    P3 = denom;

    /* Packing */
    pll_si5351c_BankSettings[bank][0] = (P3 & 0x0000FF00) >> 8;
    pll_si5351c_BankSettings[bank][1] = (P3 & 0x000000FF);
    pll_si5351c_BankSettings[bank][2] = (P1 & 0x00030000) >> 16;
    pll_si5351c_BankSettings[bank][3] = (P1 & 0x0000FF00) >> 8;
    pll_si5351c_BankSettings[bank][4] = (P1 & 0x000000FF);
    pll_si5351c_BankSettings[bank][5] = ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16);
    pll_si5351c_BankSettings[bank][6] = (P2 & 0x0000FF00) >> 8;
    pll_si5351c_BankSettings[bank][7] = (P2 & 0x000000FF);
}


void pll_si5351c_RfOutput(uint8_t enable) {
    if (enable) {
        pll_si5351c_SendRegister(SI_CLK_ENABLE, 0xFE);
        //PORTB |= _BV(PORTB2);
    } else {
        pll_si5351c_SendRegister(SI_CLK_ENABLE, 0xFF);
        //PORTB &= ~_BV(PORTB2);
    }
}


// Not USED !!!
void pll_si5351c_PA(uint8_t enable) {
    if (enable)
        PORTD |= _BV(PORTD6);
    else
        PORTD &= ~_BV(PORTD6);
}


/*
#define NUM_REGS_MAX 17

typedef struct Reg_Data {
    unsigned char Reg_Addr;
    unsigned char Reg_Val;
} Reg_Data;


Reg_Data const Reg_Store1[NUM_REGS_MAX] = {
{ 15,0x04},

{ 26,0x01},
{ 27,0xF4},
{ 28,0x00},
{ 29,0x29},
{ 30,0x58},
{ 31,0x00},
{ 32,0x01},
{ 33,0xA0},

{ 42,0x00},
{ 43,0x01},
{ 44,0x00},
{ 45,0x01},
{ 46,0x00},
{ 47,0x00},
{ 48,0x00},
{ 49,0x00},
};

Reg_Data const Reg_Store2[NUM_REGS_MAX] = {
{ 15,0x04},

{ 26,0xA1},
{ 27,0x20},
{ 28,0x00},
{ 29,0x29},
{ 30,0x58},
{ 31,0x76},
{ 32,0x5A},
{ 33,0x80},

{ 42,0x00},
{ 43,0x01},
{ 44,0x00},
{ 45,0x01},
{ 46,0x00},
{ 47,0x00},
{ 48,0x00},
{ 49,0x00},
};

void pll_si5351c_PushA() {
    for (uint8_t i=0; i<NUM_REGS_MAX; i++) { // Skip the header for the checksum
        pll_si5351c_SendRegister(Reg_Store1[i].Reg_Addr, Reg_Store1[i].Reg_Val);
        //twi_writeTo(pllAddr, &Reg_Store[i], 2, 1, 0);
        _delay_ms(1);
    }
}


void pll_si5351c_PushB() {
    for (uint8_t i=0; i<NUM_REGS_MAX; i++) { // Skip the header for the checksum
        pll_si5351c_SendRegister(Reg_Store2[i].Reg_Addr, Reg_Store2[i].Reg_Val);
        //twi_writeTo(pllAddr, &Reg_Store[i], 2, 1, 0);
        _delay_ms(1);
    }
}

*/