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


#pragma once

#include "cpu.h"

#define SI_CLK_ENABLE    3
#define SI_PLL_INPUT_SRC 15
#define SI_CLK_CONTROL   16
#define SI_SYNTH_PLL_A	 26 // Multisynth NA
#define SI_SYNTH_PLL_B	 34 // Multisynth NB
#define SI_SYNTH_MS_0	 42
#define SI_VCXO_PARAM	 162 // TODO
#define SI_PLL_RESET	 177

#define SI_R_DIV_1		0b00000000
#define SI_R_DIV_2		0b00010000
#define SI_R_DIV_4		0b00100000
#define SI_R_DIV_8		0b00110000
#define SI_R_DIV_16		0b01000000
#define SI_R_DIV_32		0b01010000
#define SI_R_DIV_64		0b01100000
#define SI_R_DIV_128	0b01110000

#define SI_CLK_SRC_PLL_A	0b00000000
#define SI_CLK_SRC_PLL_B	0b00100000

#define XTAL_FREQ	27000000


void pll_si5351c_SetAddr(uint8_t addr);
void pll_si5351c_Init();
void pll_si5351c_Shutdown();
void pll_si5351c_SendRegister(uint8_t reg, uint8_t data);
void pll_si5351c_Update(uint8_t bank);
void pll_si5351c_Update1(uint8_t bank);
void pll_si5351c_Update2(uint8_t bank);
void pll_si5351c_SetFreq(uint32_t freq, uint8_t bank);
void pll_si5351c_RfOutput(uint8_t enable);
void pll_si5351c_PA(uint8_t enable);

void pll_si5351c_PushA();
void pll_si5351c_PushB();