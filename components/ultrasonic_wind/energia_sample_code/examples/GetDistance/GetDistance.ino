/*------------------------------------------------- GetDistance -----
 PROJECT:     TUSS44x0 SPI Ultrasonic Time-of-Flight
 DESCRIPTION: Transmits and receives ultrasonic echo data to measure 
              time-of-flight and converts to distance based on speed
              sound.
 CREATED:     19 February 2020
 UPDATED:     19 February 2020
 REVISION:    A
 AUTHOR:      A. Whitehead
 NOTES:       This example code is in the public domain.
*-------------------------------------------------------------------*/
#include <TUSS44x0_Ultrasonic.h>

/*------------------------------------------------- run mode -----
|  userInputMode
|
|  Purpose:  This code can be operated in two run modes: 
|    • userInputMode = allows the user to configure the device using
|      the COM serial terminal. Resulting data is printed in the
|      terminal view. Recommended run mode for debug.
|    • standAloneMode = waits for the user to press the
|      LaucnhPad's PUSH2 button to automatically execute the
|      initializaiton routine, and begin the burst-and/or-listen captures.
|      The device is configured based on the hard-coded global 
|      variables. Comment out the run mode to use standAloneMode.
*-------------------------------------------------------------------*/
#define userInputMode      

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
  uint32_t baudRate = 115200;       // UART baud rate: 9600, 19200, 38400, 57600, 74800, or 115200

  // Configuration variables
  const unsigned int MAX_INPUT = 3; // how much serial data we expect before a newline (size = twoChars + newline = 2 + 1 = 3)
  byte inByte = 0;                  // incoming byte from serial terminal
  bool cmdRdy = false;              // flag to indicate when command is received
  bool inRdy = false;               // flag to indicate when input to command is received

  #ifndef userInputMode
    //PUSH BUTTON used for standAlone mode
    const int buttonPin = PUSH2;    // the number of the pushbutton pin
    int buttonState = 0;            // variable for reading the pushbutton status
  #endif

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
|  
|  In userInput mode, the user is prompted to enter values through 
|   the Serial COM terminal to configure the device.
|
|  In standAlone mode, the user must hard-code the configuration 
|   variables in the globals section for the device to 
|   auto-configure in the background.
*-------------------------------------------------------------------*/
void initTUSS44x0() 
{  
  #ifdef userInputMode  
    int inByte = 0;                           // incoming serial byte   
    Serial.begin(baudRate);                   // initialize COM UART serial channel
    delay(1000);
    Serial.println("TUSS44x0-Q1 EVM SPI Energia Demo for Ultrasonic Time-of-Flight");
    Serial.println("-------------------------------------------------------------------------");
    Serial.println("Instructions: Configure the TUSS44x0 by typing a value through the serial terminal.");
    Serial.println("--- Inputs must be entered as a two digit hex byte");
    Serial.println("--- Command listing:");
    Serial.println("   00 COM Ping Test"); 
    Serial.println("   01 Register Read --> Address (0x10-1E)");     
    Serial.println("   02 Register Write --> Address (0x10-1E) --> Data");             
    Serial.println("   03 TOF Config");   
    Serial.println("   04 TOF Execute");  
    Serial.println("   05 Capture & Print Options");
    Serial.println("   06 Register List");
    delay(300);
    
  #else                                       // standAlone mode  
    Serial.begin(baudRate);                   // initialize COM UART serial channel
    delay(1000);    
    pinMode(buttonPin, INPUT_PULLUP);         // initialize the pushbutton pin as an input
    while (digitalRead(buttonPin) == HIGH){}  // wait until user presses PUSH2 button to run standalone mode
    
  #endif
  
  tuss.initTUSS44x0BP(baudRate);
  tuss.captureAndPrintCheck(false);
}

/*------------------------------------------------- main loop -----
|  main loop  GetDistance
|
|   Serial command listing and descriptions:
|     "00" - COM Ping Test
|       Verify if the serial terminal is properly communicating with the MCU.
|     "01" - Register Read
|       Single regsiter read of a given address.
|     "02" - Register Write
|       Single regsiter write of a given address and data byte.
|     "03" - TOF Config
|       Configure the drive frequency, record length, speed of sound, TOF mode, and driver mode.
|     "04" - TOF Execute
|       Execute TOF command, then return ADC VOUT capture results and/or ehco interrupt capture results.
|     "05" - Capture & Print Options
|       Select what data to capture during TOF command, and what results to print on serial terminal.
|     "06" - Register List
|       Performs background register read of all user registers, and prints the latest data value on serial terminal.
|
*-------------------------------------------------------------------*/
void loop()
  {
    #ifdef userInputMode
    // if serial data available, process it
    // need to enable newline terminator in serial terminal
    GetComInput();

    // decode incoming command
    if(cmdRdy == true)
    {
      Serial.flush();
      cmdRdy = false;
      byte inTemp[7]; // temporary buffer for serial port data transfer
      unsigned int comPrint = 0;
      switch (inByte)
      {        
        case 0x00: // COM Ping Test
          Serial.println("COM PING TEST");
          break;
          
        case 0x01: // Register Read
          Serial.println("REGISTER READ");
          Serial.print("\tRead Address: ");
          inTemp[0] = GetComInput();
          Serial.println(inTemp[0],HEX);          
          Serial.print("\tRead Data: ");
          Serial.println(tuss.tuss44x0_regRead(inTemp[0]),HEX);
          break;
          
        case 0x02: // Register Write
          Serial.println("REGISTER WRITE");
          Serial.print("\tWrite Address: "); 
          inTemp[0] = GetComInput();
          Serial.println(inTemp[0],HEX);
          Serial.print("\tWrite Data: "); 
          inTemp[1] = GetComInput();
          Serial.println(inTemp[1],HEX);
          tuss.tuss44x0_regWrite(inTemp[0], inTemp[1]);
          break;
          
        case 0x03: // TOF Config
          Serial.println("TOF CONFIG");
          Serial.print("\tTransducer Frequency (0xXX..) kHz [MSB]: "); 
          inTemp[0] = GetComInput();
          Serial.println(inTemp[0],HEX);
          Serial.print("\tTransducer Frequency (0x..XX) kHz [LSB]: "); 
          inTemp[1] = GetComInput();
          Serial.println(inTemp[1],HEX);
          Serial.print("\tTOF Record Length (0xXX) ms: "); 
          inTemp[2] = GetComInput();
          Serial.println(inTemp[2],HEX);
          Serial.print("\tSpeed of Sound (0xXX..) m/s [MSB]: ");
          inTemp[3] = GetComInput();
          Serial.println(inTemp[3],HEX);
          Serial.print("\tSpeed of Sound (0x..XX) m/s [LSB]: ");
          inTemp[4] = GetComInput();
          Serial.println(inTemp[4],HEX);
          Serial.print("\tTOF Listen (0x00) or Burst (0x01) Mode: "); 
          inTemp[5] = GetComInput();
          Serial.println(inTemp[5],HEX);
          Serial.print("\tBit-Bang (0x00) or Timer Interrupt (0x01) Driver Type: "); 
          inTemp[6] = GetComInput();
          Serial.println(inTemp[6],HEX);
          tuss.tofConfig(inTemp);
          break;
          
        case 0x04: // TOF Execute
          Serial.println("TOF EXECUTE");          
          tuss.tofExecute(pulseFrequency, recordLength, tofMode);
          if((resultDisplayFlag >> 1) & 0x01 == 1 || (resultDisplayFlag >> 2) & 0x01 == 1)
          {
            Serial.println("   --- Result Summary");
          }
                         
          // Echo Interrupt at O4 Display
          if((resultDisplayFlag >> 1) & 0x01 == 1)
          {
            Serial.println("\tEcho Interrupts");
            for(comPrint = 0; comPrint < o4Trig; comPrint++)
            {
              if(comPrint == 1) // burst
              {
                Serial.print("\t\tBurst and Decay Time (us) = ");
                Serial.println(o4Micros[comPrint],DEC);
              }
              else if(comPrint > 1) //object(s)
              {
                if(o4Micros[comPrint] <= (recordLength * 1000)) // ignore result if exceeds max record length
                {
                  Serial.print("\t\tObject ");
                  Serial.println(comPrint/2,DEC);
                  Serial.print("\t\t\tTime (us) = ");
                  if(driverType == 0)
                  {
                    // add pulse time
                    Serial.println(o4Micros[comPrint] + (burstPulse/(pulseFrequency*0.001)),DEC);
                  }
                  else
                  {
                    Serial.println(o4Micros[comPrint],DEC);
                  }
                  Serial.print("\t\t\tDistance (mm) = ");
                  Serial.println(tuss.tofToDistance(o4Micros[comPrint]),DEC);
                  Serial.print("\t\t\tWidth (us) = ");
                  Serial.println(o4Micros[comPrint+1]-o4Micros[comPrint],DEC);
                  comPrint++; //skip to next low to high transition
                }
              }
              else
              {
                // no operation
              }
            }
          }
          
          // ADC VOUT Capture Display
          if((resultDisplayFlag >> 2) & 0x01 == 1)
          {
            Serial.println("\tADC VOUT Capture");
            Serial.print("\t\t8-Bit Sampled Array = ");
            for(comPrint = 0; comPrint < capturedSamples; comPrint++)
            {
              Serial.print(voutBuf[comPrint],DEC);
              Serial.print(",");
            }
            Serial.println();
          }

          break;
          
        case 0x05: // Display Options
          Serial.println("CAPTURE & PRINT OPTIONS");
          Serial.println("\tCapture and Result Display Flags: ");
          Serial.println("\t\tBit 0 = Configuration");
          Serial.println("\t\tBit 1 = Echo Interrupt");
          Serial.println("\t\tBit 2 = VOUT ADC");
          Serial.print("\t\t");
          inTemp[0] = GetComInput();      
          resultDisplayFlag = inTemp[0];
          Serial.println(resultDisplayFlag, HEX);
          tuss.captureAndPrintCheck(true);                    
          break;

        case 0x06: // Register Map
          Serial.println("Register Map");
          Serial.println("Address\tName\t\tValue\tBit Fields");
          for(comPrint = 0; comPrint < numRegs; comPrint++)
          {
            Serial.print(comPrint+0x10,HEX);
            Serial.print("\t");     
            Serial.print(regName[comPrint]);
            Serial.print("\t");              
            Serial.print(tuss.tuss44x0_regRead(comPrint+0x10),HEX);
            Serial.print("\t");              
            Serial.println(regBitFields[comPrint]);
          }
          break; 
          
        default:
          Serial.println("INVALID CMD");
          processIncomingByte('\n');         
          break;                
        }
    }

    #else  // standAlone mode  
      // ENTER YOUR STANDALONE MODE CODE HERE
    #endif

  }  // end of loop

// -+-+-+-+-+-+-+-+-+-+-  SERIAL MONITORING FUNCTIONS  -+-+-+-+-+-+-+-+-+-+- //
// capture incoming serial data
byte GetComInput()
{
  while (Serial.available () == 0 )
  {
    delay(100);
  }
  
  while (Serial.available () > 0)
  {              
    processIncomingByte (Serial.read());
  }

  return inByte;
}

// process incoming serial data
void processIncomingByte (const byte inByte)
{
    static char input_line [MAX_INPUT];
    static unsigned int input_pos = 0;  
    switch (inByte)
    {  
      case '\n':                      // end of text
        input_line [input_pos] = 0;   // terminating null byte               
        process_data (input_line);    // terminator reached! process input_line here ...        
        input_pos = 0;                // reset buffer for next time  
        break;  
      case '\r':                      // discard carriage return
        break;  
      default:
        // keep adding if not full, but allow for terminating null byte
        if (input_pos < (MAX_INPUT - 1))
        {
          input_line [input_pos++] = inByte;
        }
        break;  
    }  
}

// process incoming serial data after a terminator received
void process_data (char * data)
{ 
  inByte = string_to_byte(data);
  cmdRdy = true;    
}

// convert string char to hex char
char hexdig(char ch) 
{
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  return -1;
}

// convert two digit string to hex byte
int string_to_byte(char *data) 
{
  // do not preface string with "0x"
  int a = hexdig(data[0]);
  if (a < 0) return -1;
  int b = hexdig(data[1]);
  if (b < 0) return -1;
  return (a << 4) + b;
}
