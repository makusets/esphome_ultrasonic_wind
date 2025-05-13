/*------------------------------------------------- VOUT_ADC_Processing -----
 PROJECT:     TUSS44x0 SPI Ultrasonic VOUT Display in Processing
 DESCRIPTION: Transmits and receives ultrasonic echo data to display ADC 
              captured VOUT echo time of flight envelope in Processing GUI.
 CREATED:     19 February 2020
 UPDATED:     19 February 2020
 REVISION:    A
 AUTHOR:      A. Whitehead
 NOTES:       This example code is in the public domain.
*-------------------------------------------------------------------*/
#include <TUSS44x0_Ultrasonic.h>

/*------------------------------------------------- Global Variables -----
|  Local Variables
|
|  Purpose:  Variables shared throughout the GetDistance sketch for 
|    both userInput and standAlone modes. Hard-code these values to
|    the desired conditions when automatically updating the device
|    in standAlone mode.
*-------------------------------------------------------------------*/

  // TUSS44x0_Ultrasonic library class
  tuss44x0 tuss;
  
  // Serial COM terminal settings
  uint32_t baudRate = 115200;       // UART baud rate: 9600, 19200, 38400, 57600, 74800, 115200

  // Configuration variables
  const int buttonPin = PUSH2;      // the number of the pushbutton pin
  int buttonState = 0;              // variable for reading the pushbutton status

/*------------------------------------------------- setup -----
|  function Setup
|
|  Purpose: (see funciton initTUSS44x0 for details)
*-------------------------------------------------------------------*/
void setup() 
{                
  // put your setup code here, to run once
  initTUSS44x0();    
}

/*------------------------------------------------- initTUSS44x0 -----
|  function initTUSS44x0
|
|  Purpose: One-time setup of TUSS44x0-Q1 EVM hardware and software 
|      in the following steps: 
|    1) Configure the LaunchPad to operate in SPI communication mode.
|    2) Configure the LaunchPad pins for IO1 and IO2 as outputs.
|    3) Configure the LaunchPad pins for O3 and O4 as inputs.
|    4) TUSS44x0 hardcoded with VDRV enabled, 40kHz BPF center, and 8 pulses.
*-------------------------------------------------------------------*/
void initTUSS44x0() 
{  
    Serial.begin(baudRate);                   // initialize COM UART serial channel
    delay(1000);    
    analogReadResolution(8);                  // ADC resolution is set to 8 bits
    pinMode(buttonPin, INPUT_PULLUP);         // initialize the pushbutton pin as an input
    while (digitalRead(buttonPin) == HIGH){}  // wait until user presses PUSH2 button to run standalone mode  
    tuss.initTUSS44x0BP(baudRate);            // initialize SPI and GPIO pins on LaunchPad for TUSS44x0 device
    tuss.captureAndPrintCheck(false);         // update result and display settings for time of flight commands

    // initialize device settings to enable VDRV for TOF burst mode of 40kHz and 8 pulses
    tuss.tuss44x0_regWrite(0x10, 0x00);       // BPF  center frequency of 40kHz
    tuss.tuss44x0_regWrite(0x16, 0x00);       // VDRV not Hi-Z
    tuss.tuss44x0_regWrite(0x1A, 0x08);       // number of burst pulses is 8
}

/*------------------------------------------------- main loop -----
|  main loop  VOUT_ADC_Processing
|
|   The time of flight and VOUT ADC capture routines are executed once 
|     per loop. ADC recorded data is sent to the Processing script
|     over COM port serial communication from the LaunchPad to the PC.
|     The Processing script will plot the VOUT data over time for
|     graphical representation of the resulting time of flight command.
|
*-------------------------------------------------------------------*/
void loop()
{   
  // execute time of flight command
  tuss.tuss44x0_regWrite(0x1B, 0x00);       // TOF_CONFIG's CMD_TRIGGER to 0
  tuss.tuss44x0_regWrite(0x1B, 0x01);       // TOF_CONFIG's CMD_TRIGGER to 1
  noInterrupts();                           // disable interrupts
  tuss.burstBitBang(25,8);                  // bit-bang driver at 25us burst period (40kHz) for 8 pulses
  interrupts();                             // re enable interrupts
  tuss.captureADC(recordLength);            // capture VOUT at ADC
  tuss.tuss44x0_regWrite(0x1B, 0x00);       // TOF_CONFIG's CMD_TRIGGER to 0
  
  // send ADC VOUT data to Processing via serial COM port
  for(int comPrint = 0; comPrint < capturedSamples; comPrint++)
  {
    Serial.println(voutBuf[comPrint]);
  }
  Serial.println("E");                      // end deliminator character for Processing
  delay(100);                               // delay 100ms until next cycle

  // wait for user to press PUSH2 button to execute time of flight command
  pinMode(buttonPin, INPUT_PULLUP);         // initialize the pushbutton pin as an input
  while (digitalRead(buttonPin) == HIGH){}  // wait until user presses PUSH2 button to run standalone mode
}  // end of loop