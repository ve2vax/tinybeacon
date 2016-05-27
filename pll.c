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

#include <avr/io.h>
#include <util/delay.h>


#define COUNTER_RESET         4  //  5th bit, Register 4
#define AUTOCAL              21  // 22th bit, Register 0
#define RF_OUTPUT_ENABLE      6  //  7th bit, Register 6


/* TEST
static uint32_t pllGeneralSettings[13] = {
    0x0025A000,  // Register 0 
    0x00000001,  // Register 1
    0x00000012,  // Register 2
    0x00000003,  // Register 3
    0x30196584,  // Register 4 
    0x00800025,  // Register 5
    0x35A00CF6,  // Register 6  // FIXME : use flags !
    0x12000007,  // Register 7
    0x102D0428,  // Register 8
    0x0200BCC9,  // Register 9
    0x60C017FA,  // Register 10
    0x0061300B,  // Register 11
    0x0001041C   // Register 12
};
*/



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


//void spiInit(void) {  // faire un fichier separe
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
}


void pllShutdown() {
}


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


void pllProgramInit() {
    /* First initialisation of the PLL, with the default 0 */
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
