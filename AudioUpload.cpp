/**
 * @file AudioUpload.cpp
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

#include "AudioUpload.h"

void(* resetFunc) (void) = 0;

uint8_t FrameData[FRAMESIZE];

void writeEeprom(uint16_t address, uint8_t data){
	Wire.beginTransmission(0x50);
	Wire.write((uint8_t)(address >> 8));    // Address High Byte
	Wire.write((uint8_t)(address & 0xFF));  // Address Low Byte
	Wire.write(data);
	Wire.endTransmission();
}

uint8_t readEeprom(uint16_t address){
	uint8_t rdata = 0xFF;
	Wire.beginTransmission(0x50);
	Wire.write((uint8_t)(address >> 8));    // Address High Byte
	Wire.write((uint8_t)(address & 0xFF));  // Address Low Byte
	Wire.endTransmission();
	Wire.requestFrom(0x50, 1);
	if (Wire.available()) rdata = Wire.read();
	return rdata;
}

int8_t tryReceive(){
  beginAudioProcessing();

  Serial.println("waiting for signal");

  uint8_t p;
#define WAITBLINKTIME 10000
  uint16_t time = WAITBLINKTIME;
  uint8_t timeout = 6;

  //*************** wait for toggling input pin or timeout ******************************
  uint8_t exitcounter = 6;
  while (1) {
    if (TIMER > 100) { // timedelay ==> frequency @16MHz= 16MHz/8/100=20kHz
      TIMER = 0;
      time--;
      if (time == 0) {
        TOGGLELED;
        time = WAITBLINKTIME;
        timeout--;
        if (timeout == 0) {
          LEDOFF; // timeout,
          // leave bootloader and run program
          Serial.println("timeout");
          exit();
          return 1;
        }
      }
    }
    if (p != PINVALUE) {
      p = PINVALUE;
      exitcounter--;
    }
    if (exitcounter == 0) break; // signal received, leave this loop and go on
  }

  //*************** start command interpreter *************************************
  LEDON;
  while (1) {
    if (!receiveFrame()) {
      //*****  error: blink fast, press reset to restart *******************
      Serial.println("error");
      exit();
      return -1;
    } else { // succeed
      TOGGLELED;
      switch (FrameData[COMMAND]) {
        case TESTCOMMAND: // not used yet
          {
            //Serial.println("testcommand");
            return 0;
          }
          break;
        case RUNCOMMAND:
          {
            //Serial.println("runcommand");
            exit();
            return 0;
          }
          break;
        case RESETCOMMAND:
          {
            //Serial.println("reset");
            delay(1000);
            wdt_enable(WDTO_2S);
            for (;;);
            //resetFunc();
          }
          break;
        case WRITEEEPROM:
          {
            int address = (int) FrameData[PAGEINDEXLOW] + 256 * FrameData[PAGEINDEXHIGH];
            address *= PAGE_MAX;
            //Serial.print("writing eeprom :");
            Serial.println(address);
            for (int j = 0; j < PAGE_MAX; j++) {
              writeEeprom(address + j, FrameData[DATAPAGESTART + j]);
              delay(5);
            }
            boolean ok = true;
            for (int j = 0; j < PAGE_MAX; j++) {
              if (FrameData[DATAPAGESTART + j] != readEeprom(address + j)) {
                ok = false;
              }
            }
            //Serial.println(ok);
            exit();
            return 0;
          }
          break;
      }
      FrameData[COMMAND] = NOCOMMAND; // delete command
    }
  }
}

uint8_t receiveFrame(){
	uint16_t store[16];
  uint16_t counter = 0;
  volatile uint16_t time = 0;
  volatile uint16_t delayTime;
  uint8_t p, t;
  uint8_t k = 8;
  uint8_t dataPointer = 0;
  uint16_t n;
  //*** synchronisation and bit rate estimation **************************
  time = 0;
  // wait for edge
  p = PINVALUE;
  while (p == PINVALUE);

  p = PINVALUE;

  TIMER = 0; // reset timer
  for (n = 0; n < 16; n++) {
    // wait for edge
    while (p == PINVALUE);
    t = TIMER;
    TIMER = 0; // reset timer
    p = PINVALUE;

    store[counter++] = t;

    if (n >= 8)time += t; // time accumulator for mean period calculation only the last 8 times are used
  }

  delayTime = time * 3 / 4 / 8;
  // delay 3/4 bit
  while (TIMER < delayTime);

  //p=1;

  //****************** wait for start bit ***************************
  while (p == PINVALUE) { // while not startbit ( no change of pinValue means 0 bit )
    // wait for edge
    while (p == PINVALUE);
    p = PINVALUE;
    TIMER = 0;

    // delay 3/4 bit
    while (TIMER < delayTime);
    TIMER = 0;

    counter++;
  }
  p = PINVALUE;
  //****************************************************************
  //receive data bits
  k = 8;
  for (n = 0; n < (FRAMESIZE * 8); n++) {
    // wait for edge
    while (p == PINVALUE);
    TIMER = 0;
    p = PINVALUE;

    // delay 3/4 bit
    while (TIMER < delayTime);

    t = PINVALUE;

    counter++;

    FrameData[dataPointer] = FrameData[dataPointer] << 1;
    if (p != t) FrameData[dataPointer] |= 1;
    p = t;
    k--;
    if (k == 0) {
      dataPointer++;
      k = 8;
    };
  }
  uint16_t crc = (uint16_t)FrameData[CRCLOW] + FrameData[CRCHIGH] * 256;

#ifdef ARDUINO_DEBUG
  //********************** debug ********************************
  for (n = 0; n < 10; n++) {
    Serial.println(store[n]);
  }
  Serial.print("mean time:");
  Serial.println(time / 8);
  Serial.print("3/4 time:");
  Serial.println(delayTime);
  Serial.print("counter:");
  Serial.println(counter);
  Serial.println("*********************");
  for (n = 0; n < FRAMESIZE; n++) {
    Serial.print(FrameData[n], HEX);
    Serial.print(" ");

    Serial.println((int)FrameData[n]);
  }
  Serial.println("*********************");

  Serial.print("COMMAND:");
  Serial.println((int)FrameData[COMMAND]);
  Serial.print("PAGEINDEX:");
  Serial.println((int) FrameData[PAGEINDEXLOW] + 256 * FrameData[PAGEINDEXHIGH]);
  Serial.print("CRC:");

  Serial.println(crc, HEX);
  Serial.println("*********************");
#endif

  return (crc == 0x55AA);
}

void beginAudioProcessing(){
	//Serial.println(INPUTAUDIOPIN);
  // Timer 2 normal mode, clk/8, count up from 0 to 255
  // ==> frequency @16MHz= 16MHz/8/256=7812.5Hz

#ifdef ATMEGA8_MICROCONTROLLER
  TCCR2 = _BV(CS21);
#endif

#ifdef ATMEGA168_MICROCONTROLLER
  TCCR2B = _BV(CS21);
#endif
}

void exit(){
	// reintialize registers to default
  //DDRB=0;
  //DDRC=0;
  //DDRD=0;
  //cli();
#ifdef ATMEGA8_MICROCONTROLLER
  TCCR2 = 0; // turn off timer2
  // make sure that interrupt vectors point to user space
  GICR = (1 << IVSEL);
  GICR = 0;
#endif
#ifdef ATMEGA168_MICROCONTROLLER
  TCCR2B = 0; // turn off timer2
#endif
}
