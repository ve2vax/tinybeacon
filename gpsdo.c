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


/* |                                                                   |
   |  TinyBeacon project                                               |
   |                                                                   |
   |  - VHF/UHF Beacon (design available for 50, 144, 220 & 440 MHz)   |
   |  - Compact design / Credit card size                              |
   |  - QRP, 5W output power                                           |
   |  - 10 MHz oscillator stabilized by GPS (GPSDO)                    |
   |  - DC-DC Power supply within 10-15V, 1.5A max                     |
   |  - Compatible with WSPR & PI4 protocols                           |
   |                                                                   |
   |                                                                   |
   |  IO Mapping uController, rev.C                                    |
   |                                                                   |
   |  - PC4/SDA  (pin 27) | I2C SDA                                    |
   |  - PC5/SCL  (pin 28) | I2C SCL                                    |
   |  - PD0/RXD  (pin 30) | USART RX                                   |
   |  - PD1/TXD  (pin 31) | USART TX                                   |
   |  - PD5      (pin  9) | GPS INT                                    |
   |  - PD6      (pin 10) | PA EN                                      |
   |  - PD7      (pin 11) | INFO LED                                   |
   |  - PB0      (pin 12) | PLL LOCK                                   |
   |  - PB2      (pin 14) | PLL EN                                     |
   |                                                                   |
   |                                                                   |
   |  Example : VA2NQ Beacon -- Frequency band plan                    |
   |                                                                   |
   |   BAND | CW/PI4 Frequency | WSPR Frequency | PLL Reg. 6           |
   |--------|------------------|----------------|----------------------|
   |  50MHz | 50295000.0       | 50294450.0     | 0x35C02CF6 (/128!)   |
   |  70MHz | NA, Region 2     | NA, Region 2   | 0x35C02CF6 (/64)     |
   | 144MHz | 144491000.0      | 144490450.0    | 0x35A02CF6 (/32)     |
   | 222MHz | 222295000.0 +1?  | 222294450.0    | 0x35802CF6 (/16)     |
   | 440MHz | 432302000.0      | 432301450.0    | 0x35602CF6 ( /8)     |      FIXME : Mod flag & multiplier...
   |                                                                   | */

#include "cpu.h"

#include "twi.h"
#include "gps.h"
#include "pll-adf4355.h"
#include "pll-si5351c.h"
#include "usart.h"

#include "morse.h"
#include "pi4.h"
#include "wspr.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


void timeAlignPI4() {
    /* Get GPS data for the next time sync */
    gpsGetTime();

    /* Align on an minute for the next message */
    gpsTimeAling1Mb();
}


void timeAlignWSPR() {
    /* Get GPS data for the next time sync */
    gpsGetTime();

    /* Align on odd minute for the next message */
    gpsTimeAling2Mb();
}


void pi4sequence() {
    /* 1st part : Send PI4 message, 25 sec */
    pi4Send();

    /* 2nd part : Send morse message */
    morse2TonesSendMessage();

    /* 3th part : Send a carrier, 10 sec, same frequency */
    pllUpdate(1);
    pllPA(1);
    pllRfOutput(1);
    _delay_ms(10000);
    pllRfOutput(0);
    pllPA(0);
}


int main (void) {
    /* CKDIV8 fuse is set -- Frequency is divided by 8 at startup : 2.5MHz */
    cli();
    CLKPR = _BV(CLKPCE);  // Enable change of CLKPS bits
    CLKPR = 0;            // Set prescaler to 0 = Restore system clock to 10 MHz
    sei();

    /* DEBUG Enable I2C output */
    DDRC   |= _BV(DDC4);  // Enable output
    DDRC   |= _BV(DDC5);  // Enable output

    /* LED : Set pin 11 of PORT-PD7 for output*/
    DDRD |= _BV(DDD7);

    /* For now, used for DEBUG purpose only. Future : CLI for freq settings & modes */
    usartInit();
    _delay_ms(10);

    /* Peform I2C modules init */
    twi_init();
    _delay_ms(10);

    /* uBlox/I2C : Set the default I2C address of the GPS */
    gpsSetAddr(0x42);

    /* uBlox : GPS IO init. */
    gpsInit();  // I2C Have to be init before the PLL !

    /* uBlox : Rstrict DDC port only */
    gpsSet_CFG_PRT();

    /* uBlox : 10MHz timing setup */
    gpsSet_CFG_TP5();

    /* uBlox : Refresh rate for internal GPSDO alignment */
    gpsSet_CFG_RATE();

    /* uBlox : Wait on a full GPS sync (+ info req. for message encoding)*/
    gpsGetPVT();
    gpsExtractStrings();
    gpsGetTime();

    /* ADF4355 PLL Init */
    //pllInit();

    /* ADF4355 conf & settings */
    //pllProgramInit();

    /* Prepare the message to encode for PI4 message */
    pi4Encode();

    /* Prepare the message to encode for WSPR message */
    wsprEncode();

    /* End of init sequence : Turn on the LED (pin 11) */
    PORTD |= _BV(PORTD7);

    /*** DEBUG ***/
    //wsprSend();
    //DDRD  &= ~_BV(DDD2);    // FIX PB soudure...
    DDRB   |= _BV(DDB2);    // PLL LE - Enable output
    PORTB  |= _BV(PORTB2);  // Enable PLL
    pll_si5351c_SetAddr(0x60);
    pll_si5351c_Init();
    while(1) {
      //pll_si5351c_RfOutput(1);

      pll_si5351c_PushA();
      _delay_ms(1000); 

      pll_si5351c_PushB();
      _delay_ms(1000); 
   
      //pll_si5351c_RfOutput(0);
    }

    /* Loop sequence :
       - PI4 + Morse + Tone (1 minute)
       - PI4 + Morse + Tone (1 minute)
       - WSPR (2 minutes)
    */
    while(1) {
        timeAlignPI4();
        pi4sequence();

        timeAlignPI4();
        pi4sequence();

        timeAlignWSPR();
        wsprSend();
    }

    /* This case never happens :) Useless without powermanagement... */
    return 0;
}


