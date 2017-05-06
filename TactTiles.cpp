/**
 * @file TactTiles.cpp
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

#include "TactTiles.h"

#define MAX_DELAY_ERROR 80
#define MIN_DELAY_ERROR 20

void TTDevice::begin(){
	pinMode(POWER,OUTPUT);
	digitalWrite(POWER, LOW);
	pinMode(MOSFET,OUTPUT);
	digitalWrite(MOSFET, HIGH);
	pinMode(LED,OUTPUT);
	digitalWrite(LED, LOW);
	#ifdef Z0
		pinMode(Z0,INPUT);
	#endif
	#ifdef Z1
		pinMode(Z1,INPUT);
	#endif
	#ifdef Z2
		pinMode(Z2,INPUT);
	#endif
	#ifdef Z3
		pinMode(Z3,INPUT);
	#endif
	pinMode(LATCH, OUTPUT);
	pinMode(DATA, OUTPUT);
	pinMode(CLOCK, OUTPUT);
	shiftZero();
	pinMode(S0, OUTPUT);
	pinMode(S1, OUTPUT);
	pinMode(S2, OUTPUT);
	
	Wire.begin();
	#ifndef NOSERIAL
		Serial.begin(SERIAL_BAUD);
		Serial.setTimeout(10000);
		//#pragma message "SERIAL IS ENABLED"
	#endif
	
	inputLength = readEeprom(INPUT_LENGTH);
	twoByteOutput = readEeprom(TWO_BYTE_OUT) != 0;
	inputCategoryLength = readEeprom(CATEGORY_LEN);
	alphabetAddress = readEepromInt(ALPHABET_ADD);
	dfaAddress = readEepromInt(DFA_ADD);
	
	digitalWrite(LED, HIGH);
	hardBlink(1, 200);
}  

void TTDevice::step(){
	#ifdef TEST_OUTPUT
		static bool write = true;
		static int counter = 1;
		if (clock <= 0){
			if (write){
				if (outputState == 0 || counter == OUTPUT_REGISTERS*8){
					outputState = 1;
					counter = 1;
				} else {
					outputState <<= 1;
					counter++;
				}
				digitalWrite(LED, HIGH);
				#ifdef SERIAL
					Serial.println(counter);
				#endif
				shiftOuputState();
				clock = 150;
			} else {
				digitalWrite(LED, LOW);
				shiftZero();
				clock = 500;
			}
			write = !write;
		}
	#endif
	if (playHead && clock <= 0){
		uint8_t increment = 1;
		uint8_t i;
		switch(playHead[0]){
			case WAIT:
				time = millis();
				//TODO use 2 bytes
				clock = playHead[2];
				increment += 2;
				break;
			case SWITCH_ON:
				for (i = 0; i < playHead[1]; i++){
					bitWrite(outputState, playHead[2 + i], 1);
				}
				shiftOuputState();
				increment += playHead[1] + 1;
				break;
			case SWITCH_ALL_ON:
				outputState = ~0;
				shiftOuputState();
				break;
			case SWITCH_OFF:
				for (i = 0; i < playHead[1]; i++){
					bitWrite(outputState, playHead[2 + i], 0);
				}
				shiftOuputState();
				increment += playHead[1] + 1;
				break;
			case SWITCH_ALL_OFF:
				shiftZero();
				outputState = 0;
				break;
			case PWM_VALUE:
				pwmValue = playHead[1];
				increment += 1;
				analogWrite(MOSFET, pwmValue);
				break;
			case END_SECTION:
				Serial.println(F("S:DF"));
				digitalWrite(MOSFET, HIGH);
				break;
			case EMPTY:
			default:
				digitalWrite(MOSFET, HIGH);
				increment = 0;
				playHead = NULL;
				Serial.println(F("E:Invalid gesture token!"));
		}
		if (playHead && playHead + increment < playBuffer + playBufferLength){
			playHead += increment;
		} else {
			stopDrawing();
		}
	}
	
	uint16_t dt = millis() - time;
	clock -= dt;
	ledClock -= dt;
	time = millis();
	delay(1);
	
	#ifndef NOSERIAL
	if (Serial.available() > 0){
		if (newMessage) {
			msgLenght = Serial.read();
			newMessage = 0;
		}

		if (Serial.available() >= msgLenght) {
			uint8_t lenght = (msgLenght < BUFFER_SIZE) ? msgLenght : BUFFER_SIZE;
			Serial.readBytes((char *) buffer, lenght);
			msgLenght -= lenght;
			newMessage = 1;
			
			if (lenght > 0){
				Serial.print(F("D:MR"));
				Serial.print(lenght);
				Serial.print(F("["));
				for (int i = 0; i < lenght; i++){
					Serial.print(buffer[i]);
				Serial.print(F(" "));
				}
				Serial.println(F("]"));
				defaultMessageParser(buffer, lenght);
				messageReceived(buffer, lenght);
			}
		}
	}
	#endif
	buildInputArray(getInput(defaultThresholdValue));
	
	if (!debugMode){
		int16_t i = parseInputNonZero();//parseInput();
		int16_t symbol = 0;//changeState(i);
		if (symbol > 0) {
			symbolParsed(symbol);
			#ifndef NOSERIAL
			Serial.print(F("G:ID"));
			Serial.println(symbol);
			#endif
		}
	} else if (debugMode == 1){
		//input --> output
		
		if (highInputs >= 1){
			uint8_t i;
			for (i = 0; i < highInputs; i++){
				bitWrite(outputState, inputArray[i] - 1, 1);
			}
			shiftOuputState();
			if (clock <= 0){
				Serial.print(F("D:OS"));
				Serial.println(outputState, BIN);
				clock = 100;
			}
		} else {
			shiftZero();
			outputState = 0;
		}
		
	} else if (debugMode == 2){
		//Gove mode
		if (highInputs == 1){
		  if (inputArray[0] != v){
			k = 1;
			v = inputArray[0];
			clock = 300;
		  } else {
			  if (clock > 0 && k == 2){
				  k = 3;
			  }
		  }
		} else if (clock < 250 && highInputs == 0 && k == 1){
			k = 2;
		} else if (highInputs == 0 && k == 3){
			Serial.print("G:DT");
			Serial.println(v);
			clock = 0;
			k = 0;
			v = 0;
		}
		
		if (highInputs == 0 && clock <= 0 && k == 2){
			Serial.print("G:ST");
			Serial.println(v);
			k = 0;
			v = 0;
		}
	}
	
	
	if (analogRead(BUTTON) < 500){
		clock = 30;
		btn = true;
	} else {
		if (btn && clock <= 0){
			Serial.println(F("G:BP"));
			btn = false;
		}
	}	
	
	if (analogRead(CHARGING_STATE) > 500){
		//charging
		Serial.println(F("S:CC"));
		hardBlink(3, 100);
		shutdown();
	}
	
	#ifdef NOSERIAL
	if (!digitalRead(AUDIO)) {
		//TODO: timeout while 0's are received and then pause of 500 ms after the last 0
		if (!digitalRead(AUDIO)) {
			delay(600);
			int returnValue;
			do {
			returnValue = tryReceive();
			} while (returnValue == 0);
		}
	}
	#endif
}
  
void TTDevice::play(uint8_t * data, uint8_t size){
	uint8_t i;
	for (i = 0; i < playBufferLength; i++){
		if (playBuffer[i] = EMPTY){
			break;
		}
	}
	playBufferLength = size;
	uint8_t j;
	for (j = 0; j < size; i++, j++){
		playBuffer[i] = data[j];
	}
	if (!playHead){ //isPlaying
		playHead = (uint8_t*) playBuffer;
	}
		
	Serial.print(F("D:DG"));
	Serial.print(size);
	Serial.print(F("["));
	for (i = 0; i < size; i++){
		Serial.print(playBuffer[i]);
	Serial.print(F(" "));
	}
	Serial.println(F("]"));
	
	
	clock = 0;
}
  
void TTDevice::stopDrawing(){
	shiftZero();
	outputState = 0;
	playHead = NULL;
	uint8_t i;
	for (i = 0; i < playBufferLength; i++){
		playBuffer[i] = EMPTY;
	}
	playBufferLength = 0;
}

void TTDevice::softBlink(uint8_t times, uint16_t duration){
	// TODO
}

void TTDevice::hardBlink(uint8_t times, uint16_t duration){
	times++;
	for (;times > 0; times--){
		digitalWrite(LED, 0);
		delay(duration);
		digitalWrite(LED, 1);
		delay(duration);
	}
}

bool TTDevice::isInputDetected(){
	// TODO
}
  
uint32_t TTDevice::getInput(int threshold){
	int r0, r1, r2;
	int v;
	uint32_t b = 0;
	highInputs = 0;
	for (int i = 0; i < 8; i++) {
		r0 = bitRead(i, 0);
		r1 = bitRead(i, 1);
		r2 = bitRead(i, 2);

		digitalWrite(S0, r0);
		digitalWrite(S1, r1);
		digitalWrite(S2, r2);

		#ifdef Z0
			v = readPinCapacitance(Z0);
			highInputs += (v >= threshold);
			bitWrite(b, i, (v >= threshold));
		#endif
		#ifdef Z1
			v = readPinCapacitance(Z1);
			highInputs += (v >= threshold);
			bitWrite(b, i + 8, (v >= threshold));
		#endif
		#ifdef Z2
			v = readPinCapacitance(Z2);
			highInputs += (v >= threshold);
			bitWrite(b, i + 16, (v >= threshold));
		#endif
		#ifdef Z3
			v = readPinCapacitance(Z3);
			highInputs += (v >= threshold);
			bitWrite(b, i + 24, (v >= threshold));
		#endif
	}
	if (b != 0){
		inputDetected(b);
	}
	return b;
}
  
uint8_t readMuxPinCapacitance(int pin){
	// TODO
}
  
bool isSymbolParsed(){
	// TODO
}
  
int16_t getParsedSymbol(){
	// TODO
}
  
bool isMessageReceived(){
	// TODO
}
  
uint8_t getMessageSize(){
	// TODO
}
  
const uint8_t * getMessageData(){
	// TODO
}

bool TTDevice::defaultMessageParser(uint8_t * data, uint8_t size){
	#ifndef NOSERIAL
	uint8_t offset = 0;
	while (offset < size){
		uint8_t cmd = data[offset];
		offset++;
		switch (cmd){
			case PLAY:
				{
					uint8_t length = data[offset];
					offset++;
					if (offset + length < BUFFER_SIZE){
						play(data + offset, length);
						offset += length;
					} else {
						Serial.println(F("E:Buffer underflow!"));
					}
					break;
				}
			case PRINT_CHAR:
				{
					break;
				}
			case BLINK:
				{
					uint8_t softMode = data[offset];
					offset++;
					uint8_t times = data[offset];
					offset++;
					uint16_t duration;
					memcpy(&duration, buffer + offset, sizeof(uint16_t));
					offset+=2;
					if (softMode){
						softBlink(times,duration);
					} else {
						hardBlink(times,duration);
					}
					break;
				}
			case SEND_AGAIN:
				{
					Serial.println(F("S:PM"));
					break;
				}
			case ENABLE_AUDIO_UPDATE:
				{
					uint8_t returnValue;
					do {
						returnValue = tryReceive();
					} while (returnValue == 0);
					break;
				}
			case SET_DEBUG_MODE:
				{
					debugMode = data[offset];
					offset++;
					break;
				}
			case GET_DFA_STATE:
				{
					Serial.println(F("D:SA"));
					Serial.println(currentStateAddress);
					break;
				}
			case GET_FREE_RAM:
				{
					Serial.println(F("S:FR"));
					Serial.println(getFreeRam());
					break;
				}
			case GET_VCC:
				{
					Serial.println(F("S:SV"));
					Serial.println(readVCC());
					break;
				}
			case READ_PIN:
				{
					uint8_t pin = data[offset];
					offset++;
					if (data[offset]){ //analog
						Serial.println(F("S:AR"));
						Serial.println(analogRead(pin));
					} else { //digital
						Serial.println(F("S:DR"));
						Serial.println(digitalRead(pin));
					}
					offset++;
					break;
				}
			case EPROM_DUMP:
				{
					uint16_t address;
					memcpy(&address, buffer + offset, sizeof(uint16_t));
					offset+=2;
					uint8_t length = data[offset];
					offset++;
					dumpEeprom(address, length);
					break;
				}
			case RESET:
				{
					break;
				}
			case POWER_OFF:
				{	
					Serial.println(F("S:PO"));
					hardBlink(3, 100);
					shutdown();
					break;
				}
			case SET_THRESHOLD:
				{
					defaultThresholdValue = data[offset];
					offset++;
					break;
				}
			default:
				return false;
		}
	}
	#endif
	return true;
}

void TTDevice::shiftZero(){
	digitalWrite(MOSFET, HIGH);
	digitalWrite(LATCH, LOW);
	shiftOut(DATA, CLOCK, MSBFIRST, 0);
	shiftOut(DATA, CLOCK, MSBFIRST, 0);
	shiftOut(DATA, CLOCK, MSBFIRST, 0);
	shiftOut(DATA, CLOCK, MSBFIRST, 0);
	digitalWrite(LATCH, HIGH);
}

void TTDevice::shiftOuputState(){
	analogWrite(MOSFET, pwmValue);
	digitalWrite(LATCH, LOW);
	#if defined OUTPUT_REGISTERS && OUTPUT_REGISTERS == 4
		shiftOut(DATA, CLOCK, MSBFIRST, (uint8_t)(outputState >> 24));
	#endif
	#if defined OUTPUT_REGISTERS && OUTPUT_REGISTERS >= 3
		shiftOut(DATA, CLOCK, MSBFIRST, (uint8_t)(outputState >> 16));
	#endif
	#if defined OUTPUT_REGISTERS && OUTPUT_REGISTERS >= 2
		shiftOut(DATA, CLOCK, MSBFIRST, (uint8_t)(outputState >> 8));
	#endif
	#if defined OUTPUT_REGISTERS && OUTPUT_REGISTERS >= 1
		shiftOut(DATA, CLOCK, MSBFIRST, (uint8_t)(outputState));
	#endif
	digitalWrite(LATCH, HIGH);
}

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

uint8_t TTDevice::readPinCapacitance(uint8_t arduinoPin){
	// Variables used to translate from Arduino to AVR pin naming
	volatile uint8_t* port;
	volatile uint8_t* ddr;
	volatile uint8_t* pin;
	// Here we translate the input pin number from
	//  Arduino pin number to the AVR PORT, PIN, DDR,
	//  and which bit of those registers we care about.
	byte bitmask;
	port = portOutputRegister(digitalPinToPort(arduinoPin));
	ddr = portModeRegister(digitalPinToPort(arduinoPin));
	bitmask = digitalPinToBitMask(arduinoPin);
	pin = portInputRegister(digitalPinToPort(arduinoPin));
	// Discharge the pin first by setting it low and output
	*port &= ~(bitmask);
	*ddr  |= bitmask;
	delay(1);
	uint8_t SREG_old = SREG; //back up the AVR Status Register
	// Prevent the timer IRQ from disturbing our measurement
	noInterrupts();
	// Make the pin an input with the internal pull-up on
	*ddr &= ~(bitmask);
	*port |= bitmask;

	// Now see how long the pin to get pulled up. This manual unrolling of the loop
	// decreases the number of hardware cycles between each read of the pin,
	// thus increasing sensitivity.
	uint8_t cycles = 17;
	   if (*pin & bitmask) { cycles =  0;}
	else if (*pin & bitmask) { cycles =  1;}
	else if (*pin & bitmask) { cycles =  2;}
	else if (*pin & bitmask) { cycles =  3;}
	else if (*pin & bitmask) { cycles =  4;}
	else if (*pin & bitmask) { cycles =  5;}
	else if (*pin & bitmask) { cycles =  6;}
	else if (*pin & bitmask) { cycles =  7;}
	else if (*pin & bitmask) { cycles =  8;}
	else if (*pin & bitmask) { cycles =  9;}
	else if (*pin & bitmask) { cycles = 10;}
	else if (*pin & bitmask) { cycles = 11;}
	else if (*pin & bitmask) { cycles = 12;}
	else if (*pin & bitmask) { cycles = 13;}
	else if (*pin & bitmask) { cycles = 14;}
	else if (*pin & bitmask) { cycles = 15;}
	else if (*pin & bitmask) { cycles = 16;}

	// End of timing-critical section; turn interrupts back on if they were on before, or leave them off if they were off before
	SREG = SREG_old;

	// Discharge the pin again by setting it low and output
	//  It's important to leave the pins low if you want to 
	//  be able to touch more than 1 sensor at a time - if
	//  the sensor is left pulled high, when you touch
	//  two sensors, your body will transfer the charge between
	//  sensors.
	*port &= ~(bitmask);
	*ddr  |= bitmask;

	return cycles;
}

void TTDevice::buildInputArray(uint32_t in){
	int index = 0;
	for (int i = 0; i < 32; i++, in >>= 1) {
		if (in & 1) {
			inputArray[index] = i + 1;
			index++;
		}
	}
	inputArray[index] = 0;
}

int16_t TTDevice::parseInput(){
	if (highInputs == 0) {
		return 0;
	}
	for (int i = 0; i < inputCategoryLength; i++) { //TODO: search category in header
		byte l = readEeprom(alphabetAddress + i * 3);
		if (l == highInputs) {
			byte lowerIndex = readEeprom(alphabetAddress + i * 3 + 1);
			byte higherIndex = readEeprom(alphabetAddress + i * 3 + 2);

			for (int j = lowerIndex; j <= higherIndex; j += highInputs + 1) {
				byte output = readEeprom(alphabetAddress + j);
				boolean ok = true;
				for (int k = 0; k < highInputs; k++) {
					byte key = readEeprom(alphabetAddress + j + k + 1);
					if (inputArray[k] != key) {
						ok = false;
						break;
					}
				}
				if (ok) {
					return output;
				}
			}
			break;
		}
	}
	return -1;
}

int16_t TTDevice::parseInputNonZero(){
	int i = parseInput();
	if (i == 0) {
		long t = millis();
		while (millis() - t < 60 && i == 0) {
			buildInputArray(getInput(defaultThresholdValue));
			i = parseInput();
		}
	}

	if (i != 0) {
		lastRead = millis();
	}
	if (i == 0) {
		goToNext = true;
		if (millis() - lastRead > 500) {
			goToNext = false;
		}
	}
	return i;
}

int16_t TTDevice::changeState(int16_t input){
	if ((!goToNext && input != lastInputToState) || (goToNext && input != 0)) {
		goToNext = false;
		if (input >= 0) {
			int out;
			if (twoByteOutput) {
				out = readEepromInt(currentStateAddress + (input << 2));
			} else {
				out = readEeprom(currentStateAddress + input);
				if (out >= 128) {
					out -= 256;
				}
			}
			if (out < 0) {
				currentStateAddress = dfaAddress; //go back to 0
				if (out == -1) {
					lastInputToState = 0;
					return -1; //invalid state
				} else {
					return -(out + 1); //final state
				}
			} else {
				lastInputToState = input;
				currentStateAddress = dfaAddress + out * (inputLength + 1);
				return 0; //change state
			}
		} else {
			return -2; //invalid input
		}
	} else {
		return -3; //do nothing
	}
}

void TTDevice::debugSerialTools(){
	// TODO
}
  
void TTDevice::kill(){
  bool increase = true;
  bool led_on = true;
  uint16_t pulse_time = MIN_DELAY_ERROR;
  pinMode(LED,OUTPUT);
  for (;;){
    digitalWrite(LED,led_on);
    delay(pulse_time);
    led_on = !led_on;
    if (increase){
      if (pulse_time < MAX_DELAY_ERROR){
	pulse_time++;
      } else {
	increase = false;
      }
    } else {
      if (pulse_time > MIN_DELAY_ERROR){
	pulse_time--;
      } else {
	increase = true;
      }
    }
  }
}

void * TTDevice::check (void * p){
  if (!p) kill();
  return p;
}

uint16_t TTDevice::getFreeRam() {
  extern uint16_t __heap_start, *__brkval; 
  uint16_t v; 
  return (uint16_t) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

long TTDevice::readVCC() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void TTDevice::shutdown(){
	digitalWrite(POWER, HIGH);
}

void TTDevice::writeEeprom(uint16_t address, uint8_t data){
	writeEeprom(address,data);
}

void TTDevice::writeEeprom(uint16_t address, uint8_t * data, uint8_t length, uint8_t startIndex){
	Wire.beginTransmission(0x50);
	Wire.write((uint8_t)(address >> 8));
	Wire.write((uint8_t)(address & 0xFF));
	for (uint8_t c = startIndex; c < startIndex + length; c++) {
		Wire.write(data[c]);
	}
	Wire.endTransmission();
	delay(10);
}

uint8_t TTDevice::readEeprom(uint16_t address){
	uint8_t data = 0;
	readEeprom(address, &data, 1);
	return data;
}

int16_t TTDevice::readEepromInt(uint16_t address){
	uint16_t rdata = 0;
	Wire.beginTransmission(0x50);
	Wire.write((uint8_t)(address >> 8));
	Wire.write((uint8_t)(address & 0xFF));
	Wire.endTransmission();
	Wire.requestFrom(0x50, 2);
	if (Wire.available()) rdata = Wire.read();
	if (Wire.available()) rdata |= Wire.read() << 8;
	return rdata;
}

void TTDevice::readEeprom(uint16_t address, uint8_t * buffer, uint16_t length){
	Wire.beginTransmission(0x50);
	Wire.write((uint8_t)(address >> 8));
	Wire.write((uint8_t)(address & 0xFF));
	Wire.endTransmission();
	Wire.requestFrom(0x50, length);
	for (uint8_t c = 0; c < length; c++ ) {
		if (Wire.available()) buffer[c] = Wire.read();
	}
}

void TTDevice::dumpEeprom(uint16_t address, uint8_t length){
	#ifndef NOSERIAL
	// block to 10
	address = address / 10 * 10;
	length = (length + 9) / 10 * 10;
	byte b = readEeprom(address);
	for (int i = 0; i < length; i++) {
		if (address % 10 == 0) {
			Serial.println();//TODO Fix 1st empty line
			Serial.print(F("S:ED"));
			if (address < 100) {
				if (address < 10) {
					Serial.print(F("00"));
				} else {
					Serial.print(F("0"));
				}
			}
			Serial.print(address);
			Serial.print(F(":  "));
		}
		if (b < 100) {
			if (b < 10) {
				Serial.print(F("00"));
			} else {
				Serial.print(F("0"));
			}
		}
		Serial.print(b);
		b = readEeprom(++address);
		Serial.print(F("  "));
	}
	Serial.println();
	#endif
}
