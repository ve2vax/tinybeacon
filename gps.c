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
#include "gps.h"

#include "twi.h"
#include "usart.h"

#include <avr/io.h>
#include <math.h>  // FIXME : Floor a bypasser par un simple cast ?!
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Global definition for the I2C GPS address */
static uint8_t gpsAddr;     


/* Local/Private structrues for this processing code */
typedef struct gpsDataStruct {
    uint16_t  year;
    uint8_t   month;
    uint8_t   day;
    uint8_t   hours;
    uint8_t   minutes;
    uint8_t   seconds;
    int32_t   nano;
    int32_t   lon;     // factor 1e-7
    int32_t   lat;     // factor 1e-7
    int32_t   height;  // in mm, factor 1e-3
    int32_t   speed;   // in mm/s, factor 1e-3
    uint8_t   numsat;
    int32_t   head;    // factor 1e-5, in deg.
} gpsData;


typedef struct gpsStringStruct {
    // Extracted / Calculated outputs
    char posLocator[7];
    char posLat[8];
    char posLatDir[2];
    char posLon[9];
    char posLonDir[2];
    char day[3];
    char hours[3];
    char minutes[3];
    char seconds[3];
    char altitude[7];
    char speed[4];
    char head[4];
    char numsat[4];
    char seq[4];
} gpsString;


/* Declaration of goblal structrues & variables */
gpsData    lGpsData;    // Structure with all raw GPS data
gpsString  lGpsString;  // Structure with all extracted/computed GPS data


// FIXME PASSER en progmem pour gagner en RAM
/* Data block used for change the output PPS (10 MHz) */
static uint8_t payload10M[] = {
    0xB5, 0x62,             // Header
    0x06, 0x31,             // ID
    0x20, 0x00,             // Length
    0x00,                   // tpIdx
    0x01,                   // version
    0x00, 0x00,             // reserved1 
    0x00, 0x00,             // antCableDelay
    0x00, 0x00,             // rfGroupDelay
    0x01, 0x00, 0x00, 0x00, // freqPeriod = 1Hz
    0x80, 0x96, 0x98, 0x00, // freqPeriodLock = 10MHz
    0x00, 0x00, 0x00, 0x00, // pulseLenRatio = 0%
    0x00, 0x00, 0x00, 0x80, // pulseLenRatioLock = 50%
    0x00, 0x00, 0x00, 0x00, // userConfigDelay
    0x2F, 0x00, 0x00, 0x00, // flags : 0010 1111
    0x00, 0x00              // CRC
};


static uint8_t payload_CFG_PRT[] = {
    0xB5, 0x62,             // Header
    0x06, 0x00,             // ID
    0x14, 0x00,             // Length
    0x00,                   // Port ID (0 = DDC)
    0x00,                   // Reserved1
    0x00, 0x00,             // txReady (0 = Disable)
    0x84, 0x00, 0x00, 0x00, // mode // Slave address (0x42)
    0x00, 0x00, 0x00, 0x00, // reserved2
    0x01, 0x00,             // inProtoMask  (1 = in UBX only)
    0x01, 0x00,             // outProtoMask (1 = out UBX only)
    0x00, 0x00,             // flags (0 = Extended TX timeout )
    0x00, 0x00,             // reserved5
    0x00, 0x00              // CRC
};


// Payload to enable UBX-NAV-PVT on DDC/I2C
static uint8_t payload_CFG_MSG[] = {
    0xB5, 0x62,             // Header
    0x06, 0x01,             // ID
    0x06, 0x00,             // Length
    0x01, 0x07,             // Message Class + Message Identifier
    0x01,                   // Rate on port 0 (DDC)
    0x00,                   // Rate on port 1 (UART 1)
    0x00,                   // Rate on port 2
    0x00,                   // Rate on port 3 (USB)
    0x00, 0x00              // CRC
};


void gpsInit() {
    /* GPS Reset port : set pin 9 of PORT-PD5 for output*/
    DDRD |= _BV(DDD5);

    /* GPS not disable  */
    PORTD |= _BV(PORTD5);

    /* Note : I2C bus Init is done by i2c.c */
}


void gpsShutdown() {
     
}


void gpsSetAddr(uint8_t addr) {
    gpsAddr = addr;
}


void gpsSetPulseTimer() {
    gpsCrcUpdate(payload10M, sizeof(payload10M));
    twi_writeTo(gpsAddr, payload10M, sizeof(payload10M), 1, 0);
    _delay_ms(1);
}


void gpsSetup_CFG_PRT() {
    payload_CFG_PRT[18] = 0x01;
    payload_CFG_PRT[20] = 0x01;
    gpsCrcUpdate(payload_CFG_PRT, sizeof(payload_CFG_PRT));
    twi_writeTo(gpsAddr, payload_CFG_PRT, sizeof(payload_CFG_PRT), 1, 0);
    _delay_ms(1);
}


void gpsSetup_CFG_MSG() {
    payload_CFG_MSG[8] = 0x01;
    gpsCrcUpdate(payload_CFG_MSG, sizeof(payload_CFG_MSG));
    twi_writeTo(gpsAddr, payload_CFG_MSG, sizeof(payload_CFG_MSG), 1, 0);
    _delay_ms(1);
}


void gpsShutdown_CFG_MSG() {
    payload_CFG_MSG[8] = 0x00;
    gpsCrcUpdate(payload_CFG_MSG, sizeof(payload_CFG_MSG));
    twi_writeTo(gpsAddr, payload_CFG_MSG, sizeof(payload_CFG_MSG), 1, 0);
    _delay_ms(1);
}


void gpsCrcUpdate(uint8_t *payload, uint8_t payloadSize) {
    /* https://www.ietf.org/rfc/rfc1145.txt */
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    for (uint8_t i=2; i<(payloadSize-2); i++) { // Skip the header for the checksum
        ck_a = ck_a + payload[i];
        ck_b = ck_b + ck_a;
    }

    payload[payloadSize-1] = ck_b;
    payload[payloadSize-2] = ck_a;
}


void gpsGetNMEA() {
    uint16_t byteToRead;
    uint8_t  *data;
    uint8_t  garbage;

    uint8_t  cmd = 0xFD;
    uint8_t  cmd2 = 0xFF;

    /* Received data or not (timeout) */
    uint8_t  counter = 60;

    /* GPS locked flag (incremented at each valid line) */
    uint8_t  valid = 0;     

    /* Clean the structures */
    memset(&lGpsData, 0, sizeof(lGpsData));  
    memset(&lGpsString, 0, sizeof(lGpsString));  
    
    /* Free space for the raw input */
    data = malloc(100 * sizeof(uint8_t));
    memset(data, 0, 100 * sizeof(uint8_t)); 

    /* Setup & restrict the DDC port only */
    gpsSetup_CFG_PRT();
    
    /* Enable UBX-NAV-PVT messages on DDC/I2C only */
    gpsSetup_CFG_MSG();
 
    // FIXME : Recherche de pattern meilleur que le skip de taille...

    while ((!valid) && counter) {  // 3 minute max to get a full sync
        _delay_ms(250);   
        
        /* Point on the bytes available (Register Adressing) */
        twi_writeTo(gpsAddr, &cmd, 1, 1, 0);  // 0xFD & 0xFE for the 16 bit register
        _delay_ms(1);
        
        /* Read the effective buffered byte on the GPS uProcessor */
        if(!twi_readFrom(gpsAddr, (uint8_t*) &byteToRead, 2, 0))
            continue;
        
        byteToRead = ((byteToRead>>8) | (byteToRead<<8));  // Little endian conversion

        /* Free the buffer if unexpected size */
        if (byteToRead != 100) {
            while (byteToRead--)
                twi_readFrom(gpsAddr, &garbage, 1, 0);
            continue;
        } else {
            /* Point on the stream buffer (Register Adressing) */
            twi_writeTo(gpsAddr, &cmd2, 1, 1, 0);  // 0xFF = StreamBuffer
            _delay_ms(1);

            /* Read all the buffer */
            for (uint8_t i=0; i<byteToRead; i++) {
                twi_readFrom(gpsAddr, &data[i], 1, 0);
            }

            /* Check the validity of the data */
            if ((data[17] & 0x07) == 0x07) {
                /* Extract usefull data */
                lGpsData.year    = *(uint16_t*) &data[10];
                lGpsData.month   =               data[12];
                lGpsData.day     =               data[13];
                lGpsData.hours   =               data[14];
                lGpsData.minutes =               data[15];
                lGpsData.seconds =               data[16];
                lGpsData.nano    = *(int32_t*)  &data[16];
                lGpsData.lon     = *(int32_t*)  &data[30];
                lGpsData.lat     = *(int32_t*)  &data[34];
                lGpsData.height  = *(int32_t*)  &data[38];
                lGpsData.speed   = *(int32_t*)  &data[66];
                lGpsData.numsat  =               data[29];
                lGpsData.head    = *(int32_t*)  &data[70];

                /* Set the stop condition */
                valid++;
            }
        }
        counter--;
    }

    /* Stop the DDC port to avoid to fill the buffer */
    gpsShutdown_CFG_MSG();

    /* Raw data no longer needed */
    free(data);
}


void gpsExtractStrings() {
    /* Latitude conversion deg -> ddmm.mm */
    uint32_t latInt  = labs(lGpsData.lat / 10000000);
    uint32_t latFrac = labs(lGpsData.lat) - (latInt * 10000000);

    sprintf(lGpsString.posLat, "%02lu", latInt);
    sprintf(lGpsString.posLat+2, "%04lu", 6 * latFrac / 1000);

    /* sprintf do no work with float on avr... dumb trick */
    lGpsString.posLat[6] = lGpsString.posLat[5];
    lGpsString.posLat[5] = lGpsString.posLat[4];
    lGpsString.posLat[4] = '.';

    /* Longitude conversion deg -> dddmm.mm */
    uint32_t lonInt  = labs(lGpsData.lon / 10000000);
    uint32_t lonFrac = labs(lGpsData.lon) - (lonInt * 10000000);

    sprintf(lGpsString.posLon, "%03lu", lonInt);
    sprintf(lGpsString.posLon+3, "%04lu", 6 * lonFrac / 10000);

    /* sprintf do no work with float on avr... dumb trick */
    lGpsString.posLon[7] = lGpsString.posLon[6];
    lGpsString.posLon[6] = lGpsString.posLon[5];
    lGpsString.posLon[5] = '.';
    
    /* N/S E/W flags */
    lGpsString.posLatDir[0] = (lGpsData.lat >= 0)? 'N' : 'S';
    lGpsString.posLonDir[0] = (lGpsData.lon >= 0)? 'E' : 'W';

    /* Basic other convesion */
    sprintf(lGpsString.day, "%02d", lGpsData.day);
    sprintf(lGpsString.hours, "%02d", lGpsData.hours);
    sprintf(lGpsString.minutes, "%02d", lGpsData.minutes);
    sprintf(lGpsString.seconds, "%02d", lGpsData.seconds);
    sprintf(lGpsString.altitude, "%06d", (int)lGpsData.height/1000);
    sprintf(lGpsString.speed, "%03d", (int)lGpsData.speed/1000);
    sprintf(lGpsString.head, "%03li", (long int)lGpsData.head/100000);
    sprintf(lGpsString.numsat, "%03d", lGpsData.numsat);
}


void gpsExtractLocator() {
    // latitude
    int32_t iLat = lGpsData.lat + 900000000;
    char lat_M = (char)(iLat/100000000) + 'A';
    char lat_D = (char)((iLat%100000000) / 10000000) + '0';
    char lat_m = (char)(((iLat%10000000) * 24 ) / 10000000) + 'A';

    // longitude
    int32_t iLong = lGpsData.lon + 1800000000;
    iLong /= 2;
    char long_M = (char)(iLong/100000000) + 'A';
    char long_D = (char)((iLong%100000000) / 10000000) + '0';
    char long_m = (char)(((iLong%10000000) * 24 ) / 10000000) + 'A';

    lGpsString.posLocator[0] = long_M;
    lGpsString.posLocator[1] = lat_M;
    lGpsString.posLocator[2] = long_D;
    lGpsString.posLocator[3] = lat_D;
    lGpsString.posLocator[4] = long_m;
    lGpsString.posLocator[5] = lat_m;
    lGpsString.posLocator[6] = 0x00;
}


/*
void gpsTimeAling1M() {
    uint32_t msec = 60000 - (lGpsData.iTOW % 60000);
    
    while (msec--)
        _delay_ms(1);
}
*/


void gpsTimeAling1M() {
    uint8_t sec = 59 - lGpsData.seconds;
    int32_t nano = 1000 - (lGpsData.nano/1000000);

    // _delay_ms function support only const... :(
    while (sec--)
      _delay_ms(1000);

    while (nano--)
      _delay_ms(1);
}


void gpsTimeAling2M() {
    uint8_t min = lGpsData.minutes;
    uint8_t sec = 59 - lGpsData.seconds;
    int32_t nano = 1000 - (lGpsData.nano/1000000);
    
    if (!(min % 2))
         _delay_ms(60000);
    
    while (sec--)
        _delay_ms(1000);

    while (nano--)
      _delay_ms(1);    
}


char* getLocator() {
    return lGpsString.posLocator;
}


char* getLatitude() {
    return lGpsString.posLat;
}


char* getLongitude() {
    return lGpsString.posLon;
}


char* getLatitudeDir() {
    return lGpsString.posLatDir;
}


char* getLongitudeDir() {
    return lGpsString.posLonDir;
}


char* getAltitude() {
    return lGpsString.altitude;
}


char* getSpeed() {
    return lGpsString.speed;
}


char* getHead() {
    return lGpsString.head;
}


char* getDay() {
    return lGpsString.day;
}


char* getHours() {
    return lGpsString.hours;
}


char* getMinutes() {
    return lGpsString.minutes;
}


char* getSeconds() {
    return lGpsString.seconds;
}


char* getNumSat() {
    return lGpsString.numsat;
}


char* getSeq() {
    return lGpsString.seq;
}
