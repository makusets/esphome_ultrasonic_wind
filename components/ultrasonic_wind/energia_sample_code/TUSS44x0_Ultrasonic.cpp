/*
	TUSS44x0_Ultrasonic.cpp
	
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

#include "TUSS44x0_Ultrasonic.h"
#include "Energia.h"
#include "SPI.h"
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)  || defined(__MSP430F5529__)
	#include "msp430.h"
#endif

/*------------------------------------------------- Global Variables -----
 |  Global Variables
 |
 |  Purpose:  Variables shared throughout the tuss44x0_Ultrasonic.cpp functions
 *-------------------------------------------------------------------*/
	// Pin mapping of BOOSTXL-TUSS44x0 to LaunchPad by pin name
	#pragma region globals
		#define VOUT 2
		#define SPI_SCLK 7
		#define SPI_CS 8
		#define SPI_MISO 14
		#define SPI_MOSI 15
		#define O4 35
		#define O3 36
		#define IO1 39
		#define IO2 40	
	#pragma endregion globals

   // *** GLOBAL VARIABLES ***
   
   // TOF Configuration Variables
   unsigned int pulseFrequency = 0x28; // default frequency  of 40 kHz
   byte recordLength = 0x18;           // default record length of 24 ms
   byte burstPulse = 0;				   // default number of pulses of 0
   unsigned int speedOfSound = 0x0157; // default speed of sound through air at room temperature is 343 m/s
   
   // Driver Variables
   byte tofMode = 1;                   // default time of flight mode: 0=listen_only, 1=burst_and_listen
   byte driverType = 0;                // default pulse driver generation type: 0=bit_bang, 1= timer_interrupt
   byte IO2_toggleCount = 0;           // index counter for the number of times IO2 toggles during interrupt routine
   bool timerBurstDone = false;        // flag to indicate timer_interrupt burst is on going or completed
   byte numberOfHalfPulses = 0;        // number of pulses multiplied by two
   
   // VOUT ADC Capture Variables
   bool voutCaptureEnable = true;      // enable state of VOUT ADC capture
   byte voutBuf[4095];                 // VOUT ADC capture buffer
   unsigned int capturedSamples = 0;   // number VOUT ADC samples captured within record length time
   
   // Echo Interrupt at O4 Variables
   bool o4EchoIntEnable = true;        // enable state of echo interrupt capture
   byte o4Trig = 0;                    // index counter for the number of o4Micros state CHANGE interrupts at pin O4
   const byte o4TrigMax = 0x0A;        // maximum allowable number of state CHANGE interrupts at pin O4
   unsigned long o4Micros[o4TrigMax];  // O4 echo interrupt microsecond timer array (one burst, four objects)
      
   // SPI Variables
   byte misoBuf[2];         		   // master received SPI read buffer
   byte inByteArr[2];			   	   // master transmitted SPI data array
   byte parityVal = 0;				   // master calculated SPI parity value
   
   // Serial Terminal Variables
   byte resultDisplayFlag = 0x07;      // LS bit decode: 0=TOFCfgSummary, 1=EIResult, 2=ADCVOUTResult
   const byte numRegs = 0x0F;          // number of registers in user space is 15 for TUSS44x0 devices
   String regName [] = 				   // register name array for TUSS44x0 devices
  {
	  "BPF_CONFIG_1",
	  "BPF_CONFIG_2",
	  "DEV_CTRL_1",
	  "DEV_CTRL_2",
	  "DEV_CTRL_3",
	  "RESERVED",
	  "VDRV_CTRL",
	  "ECHO_INT_CONFIG",
	  "ZC_CONFIG",
	  "XFMR_DRV_LIM",
	  "BURST_PULSE",
	  "TOF_CONFIG",
	  "DEV_STAT",
	  "DEVICE_ID",
	  "REV_ID  "
  };
   String regBitFields [] = 		   // register bit field array for TUSS44x0 devices
  {
	  "7=BPF_FC_TRIM_FRC, 6=BPF_BYPASS, 5:0=BPF_HPF_FREQ",
	  "7:6=RESERVED, 5:4=BPF_Q_SEL. 3:0=BPF_FC_TRIM",
	  "7=LOGAMP_FRC, 6:4=LOGAMP_SLOPE_ADJ, 3:0=LOGAMP_INT_ADJ",
	  "7=LOGAMP_DIS_FIRST, 6=LOGAMP_DIS_LAST, 5:3=RESERVED, 2=VOUT_SCALE_SEL, 1:0=LNA_GAIN",
	  "7:5=RESERVED, 4:2=DRV_PLS_FLT_DT, 1:0=IO_MODE",
	  "7:0=RESERVED",
	  "7=RESERVED, 6=DIS_VDRV_REG_LSTN, 5=VDRV_HI_Z, 4=VDRV_CURRENT_LEVEL, 3:0=VDRV_VOLTAGE_LEVEL",
	  "7:5=RESERVED, 4=ECHO_INT_CMP_EN, 3:0=ECHO_INT_THR_SEL",
	  "7=ZC_CMP_EN, 6=ZC_EN_ECHO_INT, 5=ZC_CMP_IN_SEL, 4:3=ZC_CMP_STG_SEL, 2:0=ZC_CMP_HYST",
	  "7:6=RESERVED, 5:0=XFMR_DRV_ILIM",
	  "7=HALF_BRG_MODE, 6=RESERVED, 5:0=BURST_PULSE",
	  "7=SLEEP_MODE_EN, 6=STDBY_MODE_EN, 5:2=RESERVED, 1=VDRV_TRIGGER, 0=CMD_TRIGGER",
	  "7:4=RESERVED, 3=VDRV_READY, 2=PULSE_NUM_FLT, 1=DRV_PULSE_FLT, 0=EE_CRC_FLT",
	  "7:0=DEVICE_ID",
	  "7:0=REV_ID"
  };


/*------------------------------------------------- tuss44x0 Top Level -----
 |  tuss44x0 Top Level Scope Resolution Operator
 |
 | Use the double colon operator (::) to qualify a C++ member function, a top
 | level function, or a variable with global scope with:
 | • An overloaded name (same name used with different argument types)
 | • An ambiguous name (same name used in different classes)
 *-------------------------------------------------------------------*/
tuss44x0::tuss44x0(){}

/*------------------------------------------------- initTUSS44x0BP -----
 |  Function initBoostXLtuss44x0
 |
 |  Purpose:  Configure the master communication mode and BOOSTXL-TUSS44x0 
 |			hardware to operate in SPI mode.
 |
 |  Parameters:
 | 		baud (IN) -- tuss44x0 accepts a baud rate of 9.6k to 115.2k bps
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void tuss44x0::initTUSS44x0BP(uint32_t baud)
{
  // Configure SPI for TUSS44x0
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  //The SDO line is sampled on the falling edge of the SCLK pin.
  //The SDI line is shifted out on the rising edge of the SCLK pin.
  SPI.setDataMode(SPI_MODE1); //CPOL=0, CPHA=1
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  // Configure GPIOs for TUSS44x0
  pinMode(IO1, OUTPUT);
  digitalWrite(IO1, HIGH);
  pinMode(IO2, OUTPUT);
  digitalWrite(IO2, HIGH);
  pinMode(O3, INPUT);
  pinMode(O4, INPUT);
}

/*------------------------------------------------- tuss44x0_regRead -----
 |  Function tuss44x0_regRead
 |
 |  Purpose:  Perform a single register read.
 |
 |  Parameters:
 |  		addr (IN) -- valid register address byte between 0x10 to 0x1E
 |
 |  Returns:  byte representation of address's data
 *-------------------------------------------------------------------*/
byte tuss44x0::tuss44x0_regRead(byte addr)
{
  inByteArr[0] = 0x80 + ((addr & 0x3F) << 1); // shift addr to MSB position
  inByteArr[1] = 0x00;                        // null data byte during read
  inByteArr[0] |= tuss44x0_parity(inByteArr); // apply parity bit
  tuss44x0::spiTransfer(inByteArr, sizeof(inByteArr));

  return misoBuf[1];
}

/*------------------------------------------------- tuss44x0_regWrite -----
 |  Function tuss44x0_regWrite
 |
 |  Purpose:  Perform a single register write.
 |
 |  Parameters:
 |  		addr (IN) -- valid register address byte between 0x10 to 0x1E
 |  		data (IN) -- valid register data byte between 0x00 to 0xFF
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
 void tuss44x0::tuss44x0_regWrite(byte addr, byte data)
{  
  inByteArr[0] = (addr & 0x3F) << 1;          // shift addr to MSB position
  inByteArr[1] = data;                        // null data byte during read
  inByteArr[0] |= tuss44x0_parity(inByteArr); // apply parity bit
  tuss44x0::spiTransfer(inByteArr, sizeof(inByteArr));
}

/*------------------------------------------------- spiTransfer -----
 |  Function spiTransfer
 |
 |  Purpose:  Transfers byte(s) over the SPI bus, both sending and receiving. 
 |			Captures MISO data in global byte-array. Toggles CS pin
 |			active low during transfer.
 |
 |  Parameters:
 |		mosi (IN) -- MOSI data byte array to transmit over SPI
 |		size (IN) -- size of MOSI data byte array
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void tuss44x0::spiTransfer(byte* mosi, byte sizeOfArr )
{
  memset(misoBuf, 0x00, sizeof(misoBuf));	// clear receive buffer data

  digitalWrite(SPI_CS, LOW);
  for (int i = 0; i<sizeOfArr; i++)
  {
    misoBuf[i] = SPI.transfer(mosi[i]);
  }
  digitalWrite(SPI_CS, HIGH);
}

/*------------------------------------------------- tuss44x0_parity -----
 |  Function tuss44x0_parity
 |
 |  Purpose:  Calculates odd parity bit for given SPI data.
 |
 |  Parameters:
 |		spi16Val (IN) -- 16 bit SPI data byte array
 |
 |  Returns:  byte representation of SPI data array element 0's LSB
 |	 		containing calculated parity bit value
 *-------------------------------------------------------------------*/
byte tuss44x0::tuss44x0_parity(byte * spi16Val) 
{
  // SPI frame comprised of: 1 RW bit, 6 bits for the register address, 1 ODD parity bit for entire SPI frame, 8 bits for data
  return tuss44x0::parity16(tuss44x0::BitShiftCombine(spi16Val[0],spi16Val[1]));
}

/*------------------------------------------------- tuss44x0_parity -----
 |  Function tuss44x0_parity
 |
 |  Purpose:  Determines the number of ones in a given unsigned integer
 |			to return odd parity bit result.
 |
 |  Parameters:
 |		ino (IN) -- 16 bit unsigned integer
 |
 |  Returns:  parity bit value
 *-------------------------------------------------------------------*/
 byte tuss44x0::parity16(unsigned int ino)
{
  byte noofones = 0;
  for(unsigned int i = 0; i < 16; i++)
  {
    if(((ino>>i) & 1) == 1)
    {
      noofones++;
    }
  }
  // if remainder of one, add one to parity bit field
  return ((noofones+1) % 2);
}

/*------------------------------------------------- BitShiftCombine -----
 |  Function BitShiftCombine
 |
 |  Purpose:  Combines two byte values into a single unsigned integer value.
 |
 |  Parameters:
 |		x_high (IN) -- MSB input byte
  |		x_low (IN) -- LSB input byte
 |
 |  Returns:  unsigned integer of combined MSB and LSB bytes
 *-------------------------------------------------------------------*/
 unsigned int tuss44x0::BitShiftCombine( unsigned char x_high, unsigned char x_low)
{
  unsigned int combined;
  combined = x_high;              // send x_high to rightmost 8 bits
  combined = combined<<8;         // shift x_high over to leftmost 8 bits
  combined |= x_low;              // logical OR keeps x_high intact in combined and fills in rightmost 8 bits
  return combined;
}

/*------------------------------------------------- tofConfig -----
 |  Function tofConfig
 |
 |  Purpose:  Configure the time of flight parameters, such as pulse
 |			frequency, record length, speed of sound, TOF mode,
 |			and driver type.
 |
 |  Parameters:
 |		tofCfg (IN) -- 7 byte array of user defined tofConfig values
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void tuss44x0::tofConfig(byte*tofCfg)
{
  pulseFrequency = (tofCfg[0] << 8) + tofCfg[1]; 	// pulse frequency during burst
  recordLength = tofCfg[2]; 						// configure record length
  speedOfSound = (tofCfg[3] << 8) + tofCfg[4]; 		// configure speed of sound for TOF to distance conversion
  tofMode = tofCfg[5]; 								// configure tof mode type  
  driverType = tofCfg[6]; 							// configure drive type  
  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__) || defined(__MSP430F5529__)
	  // TIMER B interrupt configuration
	  if(driverType == 1)
	  {
		tuss44x0::configDrvTimer();
	  }
  #else
	  driverType = 0; // always force bit-bang drive type if not MSP430F5529 MCU
  #endif
}
 
/*------------------------------------------------- tofExecute -----
 |  Function tofExecute
 |
 |  Purpose:  Perform a time of flight command execution, and capture
 |			results of VOUT ADC and/or Echo Interrupt at pin O4.
 |
 |  Parameters:
 |  		pF (IN) -- pulse frequency to drive transducer
 |  		rL (IN) -- record length time to capture resulting data
 |  		tM (IN) -- TOF mode for listen (0) or burst (1) operation
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void tuss44x0::tofExecute(unsigned int pF, byte rL, byte tM)
{
  // read IO_MODE and BURST_PULSE
  byte ioMode = tuss44x0_regRead(0x14) & 0x03;  	// read DEV_CTRL_3's IO_MODE bits
  burstPulse = tuss44x0_regRead(0x1A) & 0x1F;		// read DEV_CTRL_2's BURST_PULSE bits

  // calculate burst period based on frequency
  unsigned int burstPeriodUs = (1 / (pF*0.001));	// convert freq to us
	
  if((resultDisplayFlag) & 0x01 == 1)
  {
    Serial.println("   --- Configuration Summary");
    Serial.print("\tIO_MODE = ");
    Serial.println(ioMode);
    Serial.print("\tBURST_PULSE = ");
    Serial.println(burstPulse);
    Serial.print("\tXDCR Frequency (kHz) = ");
    Serial.println(pF,DEC);
    Serial.print("\tRecord Length (ms) = ");
    Serial.println(rL,DEC);
    Serial.print("\tTOF Mode = ");
    Serial.println(tM);   
    Serial.print("\tBurst Period (us) = ");
    Serial.println(burstPeriodUs,DEC);
  }

  // for VOUT ADC capture, use 8-bit resolution
  analogReadResolution(8);
  
  // when pin O4 Echo Interrupt enabled
  o4Trig = 0;
  memset(o4Micros, 0, sizeof(o4Micros)); // clear receive buffer data 
  
  switch(ioMode)
  {
    case 0:
      tuss44x0::tuss44x0_regWrite(0x1B, 0x00); // TOF_CONFIG's CMD_TRIGGER to 0
      tuss44x0::tuss44x0_regWrite(0x1B, 0x01); // TOF_CONFIG's CMD_TRIGGER to 1
      tuss44x0::drivePhase(tM, burstPeriodUs,burstPulse,ioMode);
      tuss44x0::captureADC(rL);
      tuss44x0::tuss44x0_regWrite(0x1B, 0x00); // TOF_CONFIG's CMD_TRIGGER to 0
      break;
    case 1:
      digitalWrite(IO1, LOW);
      tuss44x0::drivePhase(tM, burstPeriodUs,burstPulse,ioMode);
      tuss44x0::captureADC(rL);
      digitalWrite(IO1, HIGH);
      break;
    case 2:
	case 3:
      tuss44x0::drivePhase(tM, burstPeriodUs,burstPulse,ioMode);
      tuss44x0::captureADC(rL);
      break;
    default:
      break;
  }
  
  // zero offset echo interrupt timer results
  unsigned long startOfBurst = o4Micros[0];
  for(int i = 0; i < o4Trig; i++)
  {
    o4Micros[i] = o4Micros[i]-startOfBurst;
  }
}

/*------------------------------------------------- drivePhase -----
 |  Function drivePhase
 |
 |  Purpose:  Configures the driver of the TOF command execution.
 |
 |  Parameters:
 |  		tM (IN) -- TOF mode for listen (0) or burst (1) operation
 |  		burstPeriodUs (IN) -- burst period in microseconds
 |  		burstPulse (IN) -- number of burst pulses
 |		ioMode (IN) -- TUSS44x0 IO_MODE 0-3
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
 void tuss44x0::drivePhase(byte tM, unsigned int burstPeriodUs, byte burstPulse, byte ioMode)
{
  timerBurstDone = false;
  if(tM = 0x01)
  {
    switch (driverType)
    {
      case 0: // bit bang
        noInterrupts(); 	// disable interrupts
        if(ioMode != 2)
        {          
          tuss44x0::burstBitBang(burstPeriodUs,burstPulse);          
        }
        else
        {
          tuss44x0::burstBitBangDual(burstPeriodUs,burstPulse);
        }
        interrupts(); 		// re enable interrupts 
        timerBurstDone = true;
        break;
      case 1: // timer interrupt
        if(ioMode != 2)
        {        
          tuss44x0::burstTimer(burstPeriodUs,burstPulse);    
        }
        else
        {
          // TODO
        }
        break;
    }
  }
}

/*------------------------------------------------- burstBitBang -----
 |  Function burstBitBang
 |
 |  Purpose:  Performs uninterrupted GPIO toggle of TUSS44x0's IO2 
 |			with frequency based delay to drive transducer.
 |
 |  Parameters:
 |  		bPU (IN) -- burst period in microseconds
 |  		nP (IN) -- number of burst pulses
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void tuss44x0::burstBitBang(unsigned int bPU, byte nP)
{ 
  unsigned int bPUHalf = bPU * 0.5;
  //bPUHalf -= 3; // compensate delay delta for digitalWrite for MSP430F5529 at 16 MHz which requires 3us
  int nPPlus = nP + 1;
  if(!(bPU==0))
  {
    for(int i = 0; i < nPPlus; i++)
    {
		#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__) || defined(__MSP430F5529__)
			P2OUT ^= 0x20;
			delayMicroseconds(bPUHalf);
			P2OUT ^= 0x20;
			delayMicroseconds(bPUHalf);
		#else 
			digitalWrite(IO2, LOW);
			delayMicroseconds(bPUHalf);
			digitalWrite(IO2, HIGH);
			delayMicroseconds(bPUHalf);
		#endif      
    }
  }
  else
  {
    // println("Warning: BURST_PULSE = 0");
  }
}

/*------------------------------------------------- burstBitBangDual -----
 |  Function burstBitBangDual
 |
 |  Purpose:  Performs uninterrupted GPIO toggle of TUSS44x0's IO1 and IO2 
 |			with frequency based delay to drive transducer.
 |
 |  Parameters:
 |  		bPU (IN) -- burst period in microseconds
 |  		nP (IN) -- number of burst pulses
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
 void tuss44x0::burstBitBangDual(unsigned int bPU, byte nP) //bPU=BurstPulsesUs, nP=NumberPulses
{ 
  unsigned int bPUHalf = bPU * 0.5;
  //bPUHalf -= 3; // compensate delay delta for digitalWrite for MSP430F5529 at 16 MHz which requires 3us
  int nPPlus = nP + 0;
  if(!(bPU==0))
  {
    for(int i = 0; i < nPPlus; i++)
    {
      digitalWrite(IO1, LOW);
      delayMicroseconds(bPUHalf);
      digitalWrite(IO1, HIGH);
      digitalWrite(IO2, LOW);
      delayMicroseconds(bPUHalf);
      digitalWrite(IO2, HIGH);
    }
  }
  else
  {
    //println("Warning: BURST_PULSE = 0");
  }
}

/*------------------------------------------------- burstTimer -----
 |  Function burstTimer
 |
 |  Purpose:  Enables interrupt routine to toggle IO2 pin based on 
 |			driver frequency.
 |
 |  Parameters:
 |  		bPU (IN) -- burst period in microseconds
 |  		nP (IN) -- number of burst pulses
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
 void tuss44x0::burstTimer(unsigned int bPU, byte nP)
{
  IO2_toggleCount = 0; 	//reset toggle count
  numberOfHalfPulses = (nP+1)*2;
  TBCCTL0 = CCIE;		// TBCCR0 interrupt enabled
  TBCTL = TBSSEL_2 + MC_1 + TBCLR;
  unsigned long startTime = micros();
  // while interrupt toggling, wait until number of pulses has been reached
  while(timerBurstDone == false)
  {
    //do nothing while bursting
    delayMicroseconds(1);
  }
}
 
/*------------------------------------------------- captureADC -----
 |  Function captureADC
 |
 |  Purpose:  Capture VOUT signal with ADC during TOF command for  the
 |			duration of the user defined record length.
 |
 |  Parameters:
 |  		delta (IN) -- active TOF record length
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void tuss44x0::captureADC(byte delta)
{
  unsigned int sample = 0;
  unsigned long startMillis = millis();  	// capture start time
  while ((millis() - startMillis) < delta)  // test whether the record length has elapsed
  {
    if(voutCaptureEnable == true)
    {
      if(sample < sizeof(voutBuf))
      {     
		voutBuf[sample++] =  analogRead(2);
      }
    }
  }
  capturedSamples = sample;
}
 
/*------------------------------------------------- tofToDistance -----
 |  Function tofToDistance
 |
 |  Purpose:  Convert TOF value in microseconds into distance in millimeters
 | 			based on the user defined speed of sound value.
 |
 |  Parameters:
 |  		tofTime (IN) -- time of flight in microseconds
 |
 |  Returns:  double representation of distance in millimeters
 *-------------------------------------------------------------------*/
double tuss44x0::tofToDistance(unsigned long tofTime)
{
  return (tofTime * (speedOfSound * 0.5)) * 0.001;  
}
 
/*------------------------------------------------- captureAndPrintCheck -----
 |  Function captureAndPrintCheck
 |
 |  Purpose:  Define what resulting data is to be captured and/or displayed.
 |
 |  Parameters:
 |  		show (IN) -- when true, prints existing enable state of result data
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
//capture and print check
void tuss44x0::captureAndPrintCheck(bool show)
{
    
  byte eiCheck = resultDisplayFlag & 0x02;
  if(eiCheck == 0x02) 		// Echo Interrupt Enabled
  {
    o4EchoIntEnable = true;
    attachInterrupt(O4, &tuss44x0::interrupt_o4, CHANGE);
  }
  else
  {
    o4EchoIntEnable = false;
    detachInterrupt(O4);
  }
  
  byte voutCheck = resultDisplayFlag & 0x04;
  if(voutCheck == 0x04) 	// VOUT ADC Enabled
  {
    voutCaptureEnable = true;
  }
  else
  {
    voutCaptureEnable = false;
  }
  
  // Print verify enable states
  if(show== true)
  {
    if(resultDisplayFlag & 0x01 == 0x01)
    {
      Serial.println("\tBit 0 = Configuration Display Enabled");
    }
    if(o4EchoIntEnable== true)
    {
      Serial.println("\tBit 1 = Echo Interrupt Enabled");
    }
    if(voutCaptureEnable== true)
    {
      Serial.println("\tBit 2 = VOUT ADC Enabled");
    }        
  }
}

/*------------------------------------------------- interrupt_o4 -----
 |  Function interrupt_o4
 |
 |  Purpose:  Interrupt routine for echo interrupt data capture from pin O4
 |
 |  Parameters:
 |      none
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void tuss44x0::interrupt_o4() { // INTERRUPT
  if(o4Trig < o4TrigMax)
  {
    o4Micros[o4Trig++] = micros();
  }
  
}

/*------------------------------------------------- configDrvTimer -----
 |  Function configDrvTimer
 |
 |  Purpose:  Configure Timer B0 of MSP430F5529 for timer based driver.
 |
 |  Parameters:
 |      none
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void tuss44x0::configDrvTimer()
{
  //  Description: Toggle P2.5 using software and TB_0 ISR. Timer_B is
  //  configured for up mode, thus the timer overflows when TBR counts
  //  to CCR0. In this example, CCR0 is loaded with 200 for 40kHz operation.
  //  ACLK = n/a, MCLK = SMCLK = TBCLK = default DCO ~1.045MHz

  #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)  || defined(__MSP430F5529__)
	// if XDCR=40kHz, then P=25us; thus, TBCCR0 = (ccr0/SMCLK) / 2 --> ccr0=time*SMCLK = (25us * 16MHz) / 2 = 200
	TBCCR0 = ((1 / (pulseFrequency *0.001)) * 16) / 2;
	TBCTL = TBSSEL_2 + MC_0 + TBCLR;	// SMCLK, stopmode=MC_0, clear TBR
  #endif
}

/*------------------------------------------------- TIMERB0_ISR -----
 |  Function TIMERB0_ISR
 |
 |  Purpose:  Interrupt routine to toggle GPIO attached to IO2 during
 |			timer based drive phase.
 |
 |  Parameters:
 |      none
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
// Timer B0 interrupt service routine specific to MSP430F5529
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)  || defined(__MSP430F5529__)
#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMERB0_VECTOR))) TIMERB0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  P2OUT ^= 0x20;	// Toggle P2.5
  IO2_toggleCount++;
  if (IO2_toggleCount >= numberOfHalfPulses)
  {
    timerBurstDone = true;
	
    // stop and reset Timer B0
    TBCTL |= TBCLR;
    TBCTL &= ~MC_3;
    TBCTL &= ~TBIE;
    TBCCTL0 &= ~CCIE;
    TBCCTL0 &= ~CCIFG;
  }
}
 
/*------------------------------------------------- FUNCTION_NAME -----
 |  Function FUNCTION_NAME
 |
 |  Purpose:  EXPLAIN WHAT THIS FUNCTION DOES TO SUPPORT THE CORRECT
 |      OPERATION OF THE PROGRAM, AND HOW IT DOES IT.
 |
 |  Parameters:
 |      parameter_name (IN, OUT, or IN/OUT) -- EXPLANATION OF THE
 |              PURPOSE OF THIS PARAMETER TO THE FUNCTION.
 |                      (REPEAT THIS FOR ALL FORMAL PARAMETERS OF
 |                       THIS FUNCTION.
 |                       IN = USED TO PASS DATA INTO THIS FUNCTION,
 |                       OUT = USED TO PASS DATA OUT OF THIS FUNCTION
 |                       IN/OUT = USED FOR BOTH PURPOSES.)
 |
 |  Returns:  IF THIS FUNCTION SENDS BACK A VALUE VIA THE RETURN
 |      MECHANISM, DESCRIBE THE PURPOSE OF THAT VALUE HERE.
 *-------------------------------------------------------------------*/