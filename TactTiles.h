/**
 * @file TactTiles.h
 * @author Anderson Antunes <anderson.utf@gmail.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Copyright (C) 2015 by Anderson Antunes <anderson.utf@gmail.com>
 * 
 * TactTiles is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * TactTiles is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with TactTiles.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef TACTTILES_H
#define	TACTTILES_H

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <Wire.h>
#include <AudioUpload.h>

#define OUTPUT_REGISTERS 2
#define SERIAL_BAUD  9600
//#define NOSERIAL
#define DEBUG

#define Z0  A0
#define Z1  A1
//#define Z2
//#define Z3

#define S0  6
#define S1  7
#define S2  8

#define LATCH  9
#define DATA   10
#define CLOCK  11

#define LED             4
#define MOSFET          3
#define POWER           2
#define CHARGING_STATE  A6
#define BUTTON          A7
#define AUDIO           12 // MISO

#define BUFFER_SIZE  64

//EEPROM HEADER
#define INPUT_LENGTH   0
#define TWO_BYTE_OUT   1
#define CATEGORY_LEN   2
#define ALPHABET_ADD   3
#define DFA_ADD        5

//PLAY COMMANDS
#define EMPTY           0
#define WAIT            1
#define SWITCH_ON       2
#define SWITCH_ALL_ON   3
#define SWITCH_OFF      4
#define SWITCH_ALL_OFF  5
#define PWM_VALUE       6
#define END_SECTION     7

//COMMUNICATION PROTOCOL
#define PLAY                 1
#define PRINT_CHAR           2
#define BLINK                3
#define SEND_AGAIN           4
#define ENABLE_AUDIO_UPDATE  5
#define SET_DEBUG_MODE       6
#define GET_DFA_STATE        7
#define GET_FREE_RAM         8
#define GET_VCC              9
#define READ_PIN             10
#define EPROM_DUMP           11
#define POWER_OFF            12
#define RESET                13
#define SET_THRESHOLD        14

class TTDevice {
public:
  TTDevice(uint8_t in, uint8_t out) :
    outputState(0),
    inputs(in),
    outputs(out),
    //playBuffer(NULL),
    playBufferLength(0),
    playHead(NULL),
    clock(0),
    ledClock(0),
    time(0),
    newMessage(1),
    msgLenght(0),
    isInputValid(false) {}
  
  void begin();
  
  void step();
  
  void play (uint8_t * data, uint8_t size);
  
  void stopDrawing();
  
  void softBlink(uint8_t times, uint16_t duration);
  
  void hardBlink(uint8_t times, uint16_t duration);

  bool isInputDetected();
  
  uint32_t getInput(int threshold);
  
  uint8_t readMuxPinCapacitance(int pin);
  
  bool isSymbolParsed();
  
  int16_t getParsedSymbol();
  
  bool isMessageReceived();
  
  uint8_t getMessageSize();
  
  const uint8_t * getMessageData();

public:
  
  virtual void inputDetected (uint32_t input){};
  
  virtual void symbolParsed (int16_t id){};
  
  virtual void messageReceived(const uint8_t * data, uint8_t size){};

protected:

  bool defaultMessageParser(uint8_t * data, uint8_t size);

protected:

  void shiftZero();
  
  void shiftOuputState();

protected:
  
  /*
   * void readPinCapacitance(uint8_t arduinoPin)
   * 
   * http://playground.arduino.cc/Code/CapacitiveSensor
   * 
   * Copyright 2014 by Gabriel Staples
   * Copyright 20?? by InvScribe
   * Copyright 20?? by Martin Renold
   * Copyright 20?? by Casey Rodarmor
   * Copyright 20?? by Paul Stoffregen
   * Copyright 20?? by Alan Chatham
   * Copyright 2007 by Mario Becker
   */
  
  uint8_t readPinCapacitance(uint8_t arduinoPin);
  
protected:
  
  void buildInputArray(uint32_t in);
  
  int16_t parseInput();
  
  int16_t parseInputNonZero();
  
  int16_t changeState(int16_t input);

protected:
  
  void debugSerialTools();
  
  void kill();

  void * check (void * p);

  uint16_t getFreeRam();
  
  long readVCC();
  
  void shutdown();

protected:

  void writeEeprom(uint16_t address, uint8_t data);
  
  void writeEeprom(uint16_t address, uint8_t * data, uint8_t length, uint8_t startIndex);
  
  uint8_t readEeprom(uint16_t address);
  
  int16_t readEepromInt(uint16_t address);
  
  void readEeprom(uint16_t address, uint8_t * buffer, uint16_t length);
  
  void dumpEeprom(uint16_t address, uint8_t length);

protected:

  uint32_t outputState;
  
  uint8_t defaultThresholdValue = 10;
  
 //aqui temp. - colocar como private
  int16_t clock;
  uint8_t highInputs = 0;
  uint8_t inputArray [32];
  bool btn = false;
  uint16_t v = 0;
  uint16_t k = 0;
private:

  const uint8_t inputs;
  const uint8_t outputs;
  
  uint8_t inputLength;
  boolean twoByteOutput;
  uint8_t inputCategoryLength;
  uint16_t alphabetAddress;
  uint16_t dfaAddress;
  
  uint8_t playBuffer[BUFFER_SIZE];
  uint8_t playBufferLength;
  uint8_t pos = 0;
  uint8_t * playHead;
  uint8_t pwmValue = 0;
  
  int16_t ledClock;
  uint16_t time;
  
  uint8_t newMessage;
  uint8_t msgLenght;
  
  uint16_t currentStateAddress;
  
  bool goToNext = false;
  long lastRead = 0;
  
  int16_t lastInputToState = 0;
  
  bool isInputValid;
  uint8_t buffer[BUFFER_SIZE]; /**< Buffer para o recebimento das mensagens, o mesmo é encaminhado para \c messageReceived. */
  
  int debugMode = 0;
};

#endif	/* TACTTILES_H */
