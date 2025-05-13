/*
	TUSS44x0_Ultrasonic.h
	
	BSD 2-clause "Simplified" License
	Copyright (c) 2020, Texas Instruments
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	   list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright notice,
	   this list of conditions and the following disclaimer in the documentation
	   and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY TEXAS INSTRUMENTS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL TEXAS INSTRUMENTS BE LIABLE FOR
	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	The views and conclusions contained in the software and documentation are those
	of the authors and should not be interpreted as representing official policies,
	either expressed or implied, of the FreeBSD Project.
	
	Last Updated: Feb 2020
	By: A. Whitehead <make@energia.nu>
*/
 
//#pragma once
#include <Energia.h>
#include <string.h>
 
	// *** GLOBAL VARIABLES *** (see TUSS44x0_Ultrasonic.cpp for descriptions)
 
    // TOF Configuration Variables
	extern unsigned int pulseFrequency;
	extern byte recordLength;
	extern byte burstPulse;
	extern unsigned int speedOfSound;

	// Driver Variables
	extern byte tofMode;
	extern byte driverType;
	extern byte IO2_toggleCount;
	extern bool timerBurstDone;
	extern byte numberOfHalfPulses;
   
	// VOUT ADC Capture Variables
	extern bool voutCaptureEnable;
	extern byte voutBuf[];
	extern unsigned int capturedSamples;
   
	// Echo Interrupt at O4 Variables
	extern bool o4EchoIntEnable;
	extern byte o4Trig;
	extern const byte o4TrigMax;
	extern unsigned long o4Micros[];
	
	// SPI Variables
	extern byte misoBuf[];
	extern byte inByteArr[];
	extern byte parityVal;
      
	// Serial Terminal Variables
	extern byte resultDisplayFlag;
	extern const byte numRegs;
	extern String regName[];
	extern String regBitFields[];	
   
class tuss44x0
{

  public:  
	tuss44x0(); 

	// TUSS44x0_ultrasonic.cpp FUNCTIONS
	void initTUSS44x0BP(uint32_t baud);
	byte tuss44x0_regRead(byte addr);
	void tuss44x0_regWrite(byte addr, byte data);
	void spiTransfer(byte* mosi, byte sizeOfArr );
	byte tuss44x0_parity(byte* spi16Val);
	byte parity16(unsigned int ino);
	unsigned int BitShiftCombine( unsigned char x_high, unsigned char x_low);
	void tofConfig(byte* tofCfg);
	void tofExecute(unsigned int pF, byte rL, byte bM);
	void drivePhase(byte bM, unsigned int burstPeriodUs, byte burstPulse, byte ioMode);
	void burstBitBang(unsigned int bPU, byte nP);
	void burstBitBangDual(unsigned int bPU, byte nP);
	void burstTimer(unsigned int bPU, byte nP);
	void captureADC(byte delta);
	double tofToDistance(unsigned long tofTime);
	void captureAndPrintCheck(bool show);
	static void interrupt_o4();
	void configDrvTimer();
	void timerB_interruptConfig();

};