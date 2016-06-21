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
#include <util/delay.h>
#include <math.h>   // FIXME: Floor bypass with simple cast?


/* === ADF4355 CODE === */
#ifdef ADI

    #define COUNTER_RESET         4  //  5th bit, Register 4
    #define AUTOCAL              21  // 22th bit, Register 0
    #define RF_OUTPUT_ENABLE      6  //  7th bit, Register 6


    /* Precalculated settings for the PLL */
    static uint32_t pllGeneralSettings[13] = {
        0x00201CC0,  // Register 0
        0x0CCCCCC1,  // Register 1
        0x00000012,  // Register 2
        0x40000003,  // Register 3
        0x3000C184,  // Register 4
        0x00800025,  // Register 5
        0x35C02CF6,  // Register 6  // FIXME : use flags !
        0x12000007,  // Register 7
        0x102D0428,  // Register 8
        0x14053CF9,  // Register 9
        0x60C017FA,  // Register 10
        0x0061300B,  // Register 11
        0x0000041C   // Register 12
    };


    /* Precalculated settings for the PLL, using 4 Banks */
    static uint32_t pllCustomSettings[6][2];  // 6 Tones MAX


    void pllTransmitByte(uint8_t data) {
        /* Enable PLL LE */
        PORTB &= ~_BV(PORTB2);

        /* Start transmission */
        SPDR = data;

        /* Wait for transmission complete */
        while(!(SPSR & _BV(SPIF)));

        /* Disable PLL LE */
        PORTB |= _BV(PORTB2);
    }


    void pllTransmitWord(uint32_t data) {
        /* Enable PLL LE */
        PORTB &= ~_BV(PORTB2);

        uint8_t *p = (uint8_t*)&data;
        for (uint8_t i=0; i<4; i++) {
            /* Start transmission */
            SPDR = p[3-i];  // Little endian

            /* Wait for transmission complete */
            while(!(SPSR & _BV(SPIF)));
        }

        /* Disable PLL LE */
        PORTB |= _BV(PORTB2);
    }


    void pllInit() {
        DDRB   |= _BV(DDB3);    // MOSI   - Enable output
        DDRB   |= _BV(DDB5);    // SCK    - Enable output

        DDRB   |= _BV(DDB2);    // PLL LE - Enable output
        PORTB  |= _BV(PORTB2);  // Disable PLL LE

        /* PA output status port */
        DDRD |= _BV(DDD6);

        /* PA output disable at start */
        PORTD &= ~_BV(PORTD6);

        /* Enable SPI, as Master, prescaler = Fosc/16 */
        SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);

        /* First initialisation of the PLL, with the default 0 */
        _delay_ms(100);
        pllTransmitWord(pllGeneralSettings[12]);
        pllTransmitWord(pllGeneralSettings[11]);
        pllTransmitWord(pllGeneralSettings[10]);
        pllTransmitWord(pllGeneralSettings[9]);
        pllTransmitWord(pllGeneralSettings[8]);
        pllTransmitWord(pllGeneralSettings[7]);
        pllTransmitWord(pllGeneralSettings[6]);
        pllTransmitWord(pllGeneralSettings[5]);
        pllTransmitWord(pllGeneralSettings[4]);
        pllTransmitWord(pllGeneralSettings[3]);
        pllTransmitWord(pllGeneralSettings[2]);
        pllTransmitWord(pllGeneralSettings[1]);
        pllTransmitWord(pllGeneralSettings[0]);
        _delay_ms(100);
    }


    void pllShutdown() {
    }


    void pllUpdate(uint8_t bank) {
        /* Regular way to update the PLL : Documentation ADF4355-2, page 29
           http://www.analog.com/media/en/technical-documentation/data-sheets/ADF4355-2.pdf */

        pllGeneralSettings[4] |= (1UL<<COUNTER_RESET);        // Counter reset enabled [DB4 = 1]
        pllTransmitWord(pllGeneralSettings[4]);               // Register 4 (counter reset enabled [DB4 = 1])

        //pllTransmitWord(pllGeneralSettings[bank][2]);         // Register 2

        pllTransmitWord(pllCustomSettings[bank][1]);          // Register 1

        pllCustomSettings[bank][0] &= ~(1UL<<AUTOCAL);        // Autocal enable
        pllTransmitWord(pllCustomSettings[bank][0]);          // Register 0 (autocal disabled [DB21 = 0])

        pllGeneralSettings[4] &= ~(1UL<<COUNTER_RESET);       // Counter reset disable [DB4 = 0]
        pllTransmitWord(pllGeneralSettings[4]);               // Register 4 (counter reset disabled [DB4 = 0])

        _delay_us(500);                                       // Sleep FIXME
        pllCustomSettings[bank][0] |= (1UL<<AUTOCAL);         // Autocal enable
        pllTransmitWord(pllCustomSettings[bank][0]);          // Register 0 (autocal enabled [DB21 = 1])

        _delay_us(178);  // Align on 1ms
    }


    void pllUpdateTiny(uint8_t bank) {
        /* Quick and dirty update if the delta is very low */

        //pllTransmitWord(pllGeneralSettings[bank][2]);          // Register 2
        pllTransmitWord(pllCustomSettings[bank][1]);          // Register 1

        pllCustomSettings[bank][0] |= (1UL<<AUTOCAL);         // Autocal enable
        pllTransmitWord(pllCustomSettings[bank][0]);          // Register 0 (autocal enabled [DB21 = 1])
        _delay_us(870);  // Align on 1ms
    }


    void pllSetFreq(uint64_t freq, uint8_t bank) {
        /* Calculate the frequency register -- Application 144MHz (Usable only beetween : 106.25 - 212.5 MHz) */
        /* NOTE : AVR do NOT support double, and precision of float are insuffisant, so I use uint64... */

        uint64_t pllVcoFreq  = freq * 128;
        uint64_t pllN        = pllVcoFreq / 10000000;

        uint64_t pllNint1    = pllN / 1000000;
        uint64_t pllNfrac1   = ((pllN - (pllNint1*1000000)) * 16777216)/1000000;

        uint32_t intN        = (uint32_t)pllNint1;
        uint32_t intFrac1    = (uint32_t)pllNfrac1;

        pllCustomSettings[bank][0] = (intN<<4);
        pllCustomSettings[bank][1] = (intFrac1<<4) | 0x00000001;
    }


    void pllRfOutput(uint8_t enable) {
        if (enable)
            pllGeneralSettings[6] |= (1UL<<RF_OUTPUT_ENABLE);  // Bank 0 used by default
        else
            pllGeneralSettings[6] &= ~(1UL<<RF_OUTPUT_ENABLE);

        pllTransmitWord(pllGeneralSettings[6]);
    }


    void pllPA(uint8_t enable) {
        if (enable)
            PORTD |= _BV(PORTD6);
        else
            PORTD &= ~_BV(PORTD6);
    }


/* === Si5351 CODE === */
#else

    /* Global definition for the I2C GPS address */
    static uint8_t pll_si5351c_Addr;

    static uint8_t pll_si5351c_BankSettings[4][8];


    void pllSetAddr(uint8_t addr) {
        pll_si5351c_Addr = addr;
    }


    void pllSendRegister(uint8_t reg, uint8_t data) {
        uint8_t tmp[2] = {0};
        tmp[0] = reg;
        tmp[1] = data;

        twi_writeTo(pll_si5351c_Addr, tmp, sizeof(tmp), 1, 0);
        _delay_ms(1);
    }


    void pllInit() {
        DDRB   |= _BV(DDB2);     // PLL LE - Enable output
        PORTB  &= ~_BV(PORTB2);  // Enable PLL
        _delay_ms(100);

        pll_si5351c_SendRegister(SI_CLK_ENABLE, 0xFF);      // Disable all output
        pll_si5351c_SendRegister(SI_PLL_INPUT_SRC, 0x00);   // FIXME -- Debug avec XTAL first

        pll_si5351c_SendRegister(SI_CLK_CONTROL+0, 0x4F);   // Turn on CLK0
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

        pll_si5351c_SendRegister(SI_CLK_ENABLE, 0xFE);      // Disable all output exept CLK0 (CLK0_OEB)
        pll_si5351c_SendRegister(SI_PLL_RESET, 0xA0);
    }


    void pllShutdown() {
    }


    void pllUpdate(uint8_t bank) {
        pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 0, pll_si5351c_BankSettings[bank][0]);
        pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 1, pll_si5351c_BankSettings[bank][1]);
        pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 2, pll_si5351c_BankSettings[bank][2]);
        pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 3, pll_si5351c_BankSettings[bank][3]);
        pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 4, pll_si5351c_BankSettings[bank][4]);
        pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 5, pll_si5351c_BankSettings[bank][5]);
        pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 6, pll_si5351c_BankSettings[bank][6]);
        pll_si5351c_SendRegister(SI_SYNTH_PLL_A + 7, pll_si5351c_BankSettings[bank][7]);
        //pll_si5351c_SendRegister(SI_PLL_RESET, 0xA0);  // Reset both PLL -- make glitch!!

        _delay_us(468);  // TODO Align ...
    }


    void pllUpdate(uint8_t bank) {
        pllUpdate(bank);
    }


    void pllSetFreq(uint32_t freq, uint8_t bank) { // ATTENTION : uint64_t vs uint32_t
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


    void pllRfOutput(uint8_t enable) {
        if (enable) {
            PORTB |= _BV(PORTB2);
        } else {
            PORTB &= ~_BV(PORTB2);
        }
    }


    // Not USED !!!
    void pllPA(uint8_t enable) {
        if (enable)
            PORTD |= _BV(PORTD6);
        else
            PORTD &= ~_BV(PORTD6);
    }

#endif
