/*
Project: TESTPWM
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

void setup()
{
  uint8_t oldSREG = SREG;
  cli();
  

  pinMode(11, OUTPUT);
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20); //| _BV(COM2B1) ;
  TCCR2B = _BV(CS22);
  OCR2B = 50;
    
  OCR2A = 50 * 256 / 100; //96% pwm on port 11


  pinMode(10,OUTPUT);
  TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); //| _BV(COM1A0) 
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  OCR1A = 249;

  // 10% = 24 , 96% = 239
  OCR1B = 40 * 256 / 100; // pwm on port 10

  SREG = oldSREG;

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
   
  digitalWrite( 2, LOW );
  digitalWrite( 3, HIGH );
}

void loop()
{
  
}
