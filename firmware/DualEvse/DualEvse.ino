/*
Project: DualEVSE
Date:    September 2019

Copyright (c) 2019- AJ vd Werken

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <EEPROM.h>
#include "DualEvse.h"

byte stateEVSE[2];
strConfig config;

// ID of the settings block
#define CONFIG_VERSION "cfg"
#define CONFIG_START 32

void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
  
    for (unsigned int t=0; t<sizeof(config); t++)
      *((char*)&config + t) = EEPROM.read(CONFIG_START + t);
  else
  {
    // Config has never been stored

    saveConfig();
  }
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(config); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&config + t));
}


void initialize()
{
  // Analog inputs
  pinMode(PIN_CP_READ1, INPUT);
  pinMode(PIN_CP_READ2, INPUT);
  pinMode(PIN_PP_READ1, INPUT);
  pinMode(PIN_PP_READ2, INPUT);

  // Digital outputs and inputs
  pinMode(PIN_PWM1, OUTPUT);
  pinMode(PIN_PWM2, OUTPUT);

  // Digital input
  pinMode(PIN_EXT1, INPUT_PULLUP);

  // Digital output
  pinMode(PIN_EXT2, OUTPUT);
  
  pinMode(PIN_SSR1, OUTPUT);
  pinMode(PIN_SSR2, OUTPUT);
  pinMode(PIN_RELAY1, OUTPUT);
  pinMode(PIN_RELAY2, OUTPUT);
}

void setup()
{
  initialize();
}

void loop()
{


//A: CP=12V   No charging plug connected

//B: CP=9V    Charging plug connected 
//  Voltage at the Control Pilot signal (CP) drops to 9 V.
//  R2 in vehicle detected.
//  The voltage value at CP is the result of the series connection of resistor R1 in the charging controller, diode D in the vehicle, and resistor R2 in the vehicle at 12 V.
//  In status B, the oscillator with pulse width modulation (PWM) is
//  switched on. The pulse width codes the permissible charging current that the vehicle may take from the charging infrastructure.
//  The coding is shown in the table below.
//  B1 (9 V DC): EVSE not ready yet
//  B2 (9 V PWM): EVSE ready
   
//C: CP=6V    Charging without ventilation  
//  If the vehicle has detected the PWM signal, 
//  it connects another resistor R3 parallel to R2 via switch S2. 
//  The resulting voltage value is 6 V (ventilation not required) 
//  The charging controller connects the mains voltage to the vehicle
//  via a contactor and charging cable. The charging process begins.

//D: CP=3V    Charging with ventilation required 
//  If the vehicle has detected the PWM signal, 
//  it connects another resistor R3 parallel to R2 via switch S2. 
//  The resulting voltage value is 3 V (ventilation required).
//  The charging controller connects the mains voltage to the vehicle
//  via a contactor and charging cable. The charging process begins.   
  
//B: CP=9V    Charging completed
//  The vehicle disconnects resistor R3 again via S2.
//  The charging controller disconnects the contactor again and with it
//  the voltage from the charging cable.
//  Conversely, the charging controller can also indicate to the vehicle
//  that the charging process should be completed by switching off the
//  PWM signal.

  if (stateEVSE == ST_A)      
  {
    // Set PWM to OFF
    // Set Contactor OFF
    // Set CP to continuous 12V
    // DebugLog "state = (A) inactive"
                                               // Mark as inactive
    short cp = readCP(e)
    
    if (cp == CP_12V)                                            // Check if we are disconnected, or forced to State A, but still connected to the EV
    {
                                   
    }
    if (cp == CP_9V)                                             // switch to State B ?
    {
      
    }
  

}
