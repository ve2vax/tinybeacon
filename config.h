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

#define PI4_MESSAGE      "X1ABC   "        // UPDATE with your Callsign (8 chars, padding with spaces)
#define PI4_FREQUENCY    50295000.0        // UPDATE with frequency aligned with the frequency bands

#define MORSE_MESSAGE    "X1ABC  AB12CD "  // UPDATE with your Callsign + Locator
#define MORSE_FREQUENCY  50295000.0        // UPDATE with frequency aligned with the frequency bands ( Propagation Beacons Exclusive )

#define WSPR_CALLSIGN    "X1ABC "          // Exactly 6 characters (Padding with space at start. Ex " K1AB " or " K1ABC" or "VE1ABC")
#define WSPR_LOCATOR     "AB12"            // Exactly 4 characters (First part of the locator)
#define WSPR_POWER       37                // Numerical value in dBm (range 0-60, check allowed values)
#define WSPR_FREQUENCY   50294450.0

