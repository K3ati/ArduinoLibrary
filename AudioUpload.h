/**
 * @file AudioUpload.h
 * @author Christoph Haberer
 * @version 2.0
 *
 * @section LICENSE
 *
 * Copyright (C) 2011  -C-H-R-I-S-T-O-P-H-   -H-A-B-E-R-E-R-
 * 
 * http://www.hobby-roboter.de/forum/viewtopic.php?f=4&t=127
 *
 * v0.1	19.6.2008	C. -H-A-B-E-R-E-R- 	Bootloader for IR-Interface
 * v1.0	03.9.2011	C. -H-A-B-E-R-E-R-	Bootloader for audio signal
 * v1.1	05.9.2011	C. -H-A-B-E-R-E-R-	changing pin setup, comments, and exitcounter=3
 * v1.2	12.5.2012	C. -H-A-B-E-R-E-R-	Atmega8 Support added, java programm has to be addeptet too
 * v1.3	20.5.2012	C. -H-A-B-E-R-E-R-	now interrupts of user programm are working
 * v1.4	05.6.2012	C. -H-A-B-E-R-E-R-  signal coding changed to differential manchester code
 * v2.0	13.6.2012	C. -H-A-B-E-R-E-R-	setup for various MCs
 * 
 * AudioUpload is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * AudioUpload is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with AudioUpload.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef AUDIO_UPLOAD_H
#define	AUDIO_UPLOAD_H

#include <Arduino.h>
#include <stdint.h>
#include <avr/wdt.h>
#include <Wire.h>

#define MAX_ADDRESS 0x7FFF
#define PAGE_MAX 64

#define ATMEGA168_MICROCONTROLLER

//#define ARDUINO_DEBUG

const int ledPin =  3;    // LED connected to digital pin 13

#define LEDON  { digitalWrite(ledPin, HIGH );   }
#define LEDOFF { digitalWrite(ledPin, LOW  );   }
#define TOGGLELED { digitalWrite(ledPin, !digitalRead(ledPin));}
//#define INPUTAUDIOPIN (1<<PB4)
//#define PINVALUE (PINB&INPUTAUDIOPIN)
#define PINVALUE (PINB & _BV (PB4))

#define PINLOW (PINVALUE==0)
#define PINHIGH (!PINLOW)

//***************************************************************************************
// main loop
//***************************************************************************************


#define TIMER TCNT2 // we use timer2 for measuring time

// frame format definition
#define COMMAND         0
#define PAGEINDEXLOW 	1  // page address lower part
#define PAGEINDEXHIGH 	2  // page address higher part
#define CRCLOW          3  // checksum lower part 
#define CRCHIGH 	4  // checksum higher part 
#define DATAPAGESTART   5  // start of data
#define PAGESIZE 	PAGE_MAX
#define FRAMESIZE       (PAGESIZE+DATAPAGESTART)// size of the data block to be received

// bootloader commands
#define NOCOMMAND     0
#define TESTCOMMAND   1
#define PROGCOMMAND   2
#define RUNCOMMAND    3
#define RESETCOMMAND  4
#define WRITEEEPROM   5
//#define READEEPROM    6

void writeEeprom(uint16_t address, uint8_t data);

uint8_t readEeprom(uint16_t address);

int8_t tryReceive();

uint8_t receiveFrame();

void beginAudioProcessing();

void exit();

#endif	/* AUDIO_UPLOAD_H */
