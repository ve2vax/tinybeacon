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
   |  VA2NQ Beacon -- Frequency band plan                              |
   |                                                                   |
   |   BAND | CW/PI4 Frequency | WSPR Frequency | PLL Reg. 6           |
   |--------|------------------|----------------|----------------------|
   |  50MHz | 50295000.0       | 50294450.0     | 0x35C02CF6 (/128!)   |
   |  70MHz | NA, Region 2     | NA, Region 2   | 0x35C02CF6 (/64)     |
   | 144MHz | 144491000.0      | 144490450.0    | 0x35A02CF6 (/32)     |
   | 222MHz | 222295000.0 +1?  | 222294450.0    | 0x35802CF6 (/16)     |
   | 440MHz | 432302000.0      | 432301450.0    | 0x35602CF6 ( /8)     |      FIXME : Mod avec flag & multiplier...
   |                                                                   | */

#include "cpu.h"

#include "twi.h"
#include "gps.h"
#include "pll.h"
#include "usart.h"

#include "morse.h"
#include "pi4.h"
#include "wspr.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


void timeAlignPI4() {
	/* Update the GPS data for the next time align */
	gpsGetNMEA();
	gpsExtractStrings();

	/* Align on an minute for the next message */
	gpsTimeAling1Mb();
}


void timeAlignWSPR() {
	/* Update the GPS data for the next time align */
	gpsGetNMEA();
	gpsExtractStrings();

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
	/* CKDIV8 fuse is set -- Frequency is divided by 8 at start : 2.5MHz */
	cli();
	CLKPR = _BV(CLKPCE);  // Enable change of CLKPS bits
	CLKPR = 0;            // Set prescaler to 0 = Restore system clock to 10 MHz
	sei();

    /* DEBUG Enable I2C output */
    DDRC   |= _BV(DDC4);  // Enable output
    DDRC   |= _BV(DDC5);  // Enable output

	/* LED : Set pin 11 of PORT-PD7 for output*/
	DDRD |= _BV(DDD7);

	/* Peform the sub-modules initialisation */
	twi_init();
	_delay_ms(10);

	/* Enable GPS for time sync, osc & locator */
	gpsInit();  // I2C Have to be init before the PLL !
	_delay_ms(10);

	/* Init for the AD PLL */
	pllInit();
	_delay_ms(10);
	
	/* Set the default I2C address of the PLL */
	gpsSetAddr(0x42);
	_delay_ms(10);

	/* For now, used for DEBUG purpose only. Future : CLI for freq settings & modes */
	usartInit();
	_delay_ms(10);

	/* ADF4355-2 init & settings */
	pllProgramInit();
	_delay_ms(10);

	/* Prepare the message to encode for PI4 message */
	pi4Encode();

	/* Prepare the message to encode for WSPR message */
	wsprEncode();

	/* Update the GPS data for the next time align */
	gpsGetNMEA();
	gpsExtractStrings();

	/* uBlox 10MHz timing settings */
	gpsSet_CFG_TP5();
	_delay_ms(10);

	/* uBlox high refresh rate for timing */
	gpsSet_CFG_RATE();
	_delay_ms(10);

	/* Loop sequence :
	   - PI4 + Morse + Tone (1 minute)
	   - PI4 + Morse + Tone (1 minute)
	   - WSPR (2 minutes)
	*/
	while(1) {
	   	/* Start SEQ : Turn on the LED (pin 11) */
		PORTD |= _BV(PORTD7);

		//wsprSend(); // DEBUG

		timeAlignPI4();
		pi4sequence();

		timeAlignPI4();
		pi4sequence();

		timeAlignWSPR(); 
		wsprSend();

		/* End SEQ : Turn off the LED (pin 11) */
		PORTD &= ~_BV(PORTD7);
	}

	/* This case never happens :) Useless without powermanagement... */
	return 0;
}
