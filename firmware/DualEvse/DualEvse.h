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

// Not Set, Not Connected, Connected Not Ready, Connected Ready, Connected Requires Ventilation
enum STATE { ST__, ST_A, ST_B, ST_C, ST_D } 

// Possible CP values
enum PILOT { CP_ERROR, CP_12V, CP_9V, CP_6V, CP_DIODE }

// Hardware PIN layout - Arduino NANO
#define PIN_CP_READ1
#define PIN_CP_READ2
#define PIN_PP_READ1
#define PIN_PP_READ2
#define PIN_PWM1
#define PIN_PWM2
#define PIN_EXT1
#define PIN_EXT2
#define PIN_SSR1
#define PIN_SSR2
#define PIN_RELAY1
#define PIN_RELAY2

// Configuration values
struct Config
{
  int   MinCurrentEV[2];
  int   MaxCurrentEV[2];
  bool  Contacter[2];
  bool  Solenoid[2];
  bool  CableLocked; //EVSE2 is always fixed
}
 
