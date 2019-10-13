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
#include "DualEvse.h"

void SetCurrent(unsigned int);
unsigned int CalcCurrent();


// Configuration settings
#pragma  config FCMEN = OFF, IESO = OFF, PRICLKEN = ON
#pragma config PLLCFG = OFF, FOSC = HSMP                                        // High Speed Medium power (4-16Mhz), PLL Off
#pragma config BORV = 285, BOREN = ON, PWRTEN = ON
#pragma config WDTPS = 2048, WDTEN = OFF                                        // WDT timeout
#pragma config CCP2MX = PORTB3, PBADEN = OFF, CCP3MX = PORTC6                   // PortB digital IO
#pragma config HFOFST = OFF, T3CMX = PORTB5, P2BMX = PORTC0, MCLRE = INTMCLR

#pragma config XINST = OFF, DEBUG = OFF, LVP = OFF, STVREN = ON
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF, CPD = OFF, CPB = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
#pragma config WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
#pragma config EBTRB = OFF


const unsigned int EE_MaxMains @ 0xf00000 = MAX_MAINS;
const unsigned int EE_MaxCurrent @ 0xf00002 = MAX_CURRENT;
const unsigned int EE_MinCurrent @ 0xf00004 = MIN_CURRENT;
const double EE_ICal @ 0xf00006 = ICAL;
const unsigned int EE_Mode @ 0xf0000a = MODE + (LOCK<<8);
const unsigned int EE_CableLimit @ 0xf0000c = CABLE_LIMIT;
const unsigned int EE_Config_LoadBl @ 0xf0000e = CONFIG + (LOADBL<<8);
const unsigned int EE_Access @ 0xf00010 = ACCESS;
const unsigned int EE_RCmon @ 0xf00012 = RC_MON;

// Global data
char    U1buffer[50];                                                           // Uart1 Receive buffer /RS485
char    U1TXbuffer[50];                                                         // Uart1 Transmit buffer /RS485
char    U2buffer[50];                                                           // Uart2 buffer /Serial CLI
char    Tbuffer[50];                                                            // temp buffer
char    GLCDbuf[256];                                                           // GLCD buffer (one row double height text only)


// The following data will be updated by eeprom data at powerup:
unsigned int MaxMains;                                                          // Max Mains Amps (hard limit, limited by the MAINS connection)
unsigned int MaxCurrent;                                                        // Max Charge current
unsigned int MinCurrent;                                                        // Minimal current the EV is happy with
double ICal;                                                                    // CT calibration value
char Mode;                                                                      // EVSE mode
char Lock;                                                                      // Cable lock enable/disable
unsigned int CableLimit;                                                        // Fixed Cable Current limit (only used when config is set to Fixed Cable)
char Config;                                                                    // Configuration (Fixed Cable or Type 2 Socket)
char LoadBl;                                                                    // Load Balance Setting (Disable, Master or Slave1-3)
char Access;                                                                    // External Start/Stop button on I/O 2
char RCmon;                                                                     // Residual Current Monitor on I/O 3

// total 17 bytes

double Irms[3]={0,0,0};                                                         // Momentary current per Phase (Amps *10) (23= 2.3A)
                                                                                // Max 3 phases supported

unsigned int crc16;
unsigned char State = STATE_A;
unsigned char Error = NO_ERROR;
unsigned char NextState;

unsigned int MaxCapacity;                                                       // Cable limit (Amps)(limited by the wire in the charge cable, set automatically, or manually if Config=Fixed Cable)
unsigned int ChargeCurrent;                                                     // Calculated Charge Current
unsigned int Imeasured=0;                                                       // Max of all CT inputs (Amps *10)
// Load Balance variables
int IsetBalanced=0;                                                             // Max calculated current available for all EVSE's
int Balanced[4]={0,0,0,0};                                                      // Amps value per EVSE (max 4)
int BalancedMax[4]={0,0,0,0};                                                   // Max Amps value per EVSE (max 4)
char BalancedState[4]={0,0,0,0};                                                // State of all EVSE's 0=not active (state A), 1=charge request (State B), 2= Charging (State C) 

unsigned char RX1byte, TX1byte;
unsigned char idx=0,idx2=0,ISRFLAG=0,ISR2FLAG=0,ISRTXFLAG=0;
unsigned char menu=0;
unsigned int locktimer=0,unlocktimer=0;                                         // solenoid timers
unsigned long Timer=0;                                                          // mS counter
unsigned char BacklightTimer=0;                                                 // Backlight timer (sec)
unsigned int ChargeTimer=0;                                                     // Counts seconds in STATE C (Charging) (unused)
unsigned char LCDTimer=0;
unsigned char TempEVSE=0;                                                       // Temperature EVSE in deg C (0-125)
unsigned char ButtonState=0x0f;                                                 // Holds latest push Buttons state (LSB 3:0)
unsigned char OldButtonState=0x0f;                                              // Holds previous push Buttons state (LSB 3:0)
unsigned char LCDNav=0;
unsigned char SubMenu=0;
unsigned long ScrollTimer=0;
unsigned char LCDpos=8;
unsigned char ChargeDelay=0;                                                    // Delays charging at least 60 seconds in case of not enough current available.
unsigned char NoCurrent=0;                                                      // counts overcurrent situations.
unsigned char TestState=0;
unsigned char LedTimer=0;                                                       // LED on I01 uses TMR2 and a PWM signal to fade in/out
unsigned char LedUpdate=0;                                                      // Flag that LED PWM data has been updated
unsigned char LedCount=0;                                                       // Raw Counter before being converted to PWM value
unsigned char LedPwm=0;                                                         // PWM value 0-255

unsigned char Access_bit=0;
unsigned int AccessTimer=0;         



void interrupt high_isr(void)
{
    // Determine what caused the interrupt
    while (PIR1bits.RC1IF)                                                      // Uart1 receive interrupt? RS485
    {
        RX1byte = RCREG1;                                                       // copy received byte 
    // check for start/end of data packet byte, and max number of bytes in buffer 
        if (idx == 50) idx--;
        if (RX1byte == 0x7E)                                                    // max 50 bytes in buffer
        {
            if (idx > 7)                                                        // end of packet detected?
            {
                ISRFLAG = idx;                                                  // flag complete packet for main routine
            }
            idx = 0;                                                            // reset index
        } else if (RX1byte == 0x7D)                                             // escape character found?
        {
            ISRFLAG = 1;                                                        // yes, mark next byte
        } else                                                                  // normal characters
        {
            if (ISRFLAG == 1)                                                   // was previous byte a escape character?
            {
                ISRFLAG = 0;
                RX1byte = 0x20 ^ RX1byte;
            }
            U1buffer[idx++] = RX1byte;
        }
    }

    if (PIR1bits.TX1IF && PIE1bits.TX1IE)                                       // Uart1 transmit interrupt? RS485
    {
        TX1byte = U1TXbuffer[ISRTXFLAG];
        TXREG1 = TX1byte;                                                       // send character
        if ((ISRTXFLAG && TX1byte == 0x7E) || ISRTXFLAG == 49)                  // end of buffer
        {
            PIE1bits.TX1IE = 0;                                                 // clear transmit Interrupt for RS485 after sending last character
            ISRTXFLAG = 0;                                                      // end of transmission.
            // we switch of the transmitter in the main loop, after the final character has been sent..
        } else ISRTXFLAG++;
    }

    // Uart2 receive interrupt?
    while (PIR3bits.RC2IF)                                                      
    {
        // Check for BREAK character, then Reset

        if (RCSTA2bits.FERR && RCONbits.POR && State == STATE_A) {              // Make sure any data during a POR is ignored
            RX1byte = RCREG2;                                                   // copy received byte
            if (!RX1byte) Reset();                                              // Only reset if not charging...
        } else RX1byte = RCREG2;
        
        TXREG2 = RX1byte;                                                       // echo to UART2 port, don't check for overflow here.
        if (idx2 == 50) idx2--;
        if ((RX1byte == 0x08) && (idx2 > 0)) {
            idx2--;                                                             // backspace
        } else {
            if (RX1byte == 0x0d || RX1byte == 0x0a)                             // CR or LF?
            {
                RX1byte = 0;
                ISR2FLAG = idx2 + 1;                                            // ENTER, process data
            }
            U2buffer[idx2++] = RX1byte;                                         // store byte
        }
    }

    // Timer 4 interrupt, called 1000 times/sec
    while (PIR5bits.TMR4IF)                                                     
    {
        if (Lock == 1)                                                          // Cable lock type Solenoid?
        {
            if (Error || (State != STATE_C)) {
                if (unlocktimer < 300)                                          // 300ms pulse    
                {
                    SOLENOID_UNLOCK;
                } else SOLENOID_OFF;
                if (unlocktimer++ > 400) {
                    if (PORTCbits.RC1 == 0)                                     // still locked...
                    {
                        if (unlocktimer > 5000) unlocktimer = 0;                //try to unlock again in 5 seconds
                    } else unlocktimer = 400;
                }
                locktimer = 0;
            }
            else                                                                // State C
            {
                if (locktimer < 300)                                            // 300ms pulse
                {
                    SOLENOID_LOCK;
                }
                else SOLENOID_OFF;
                if (locktimer++ > 400) {
                    if (PORTCbits.RC1 == 1)                                     // still unlocked...
                    {
                        if (locktimer > 5000) locktimer = 0;                    //try to lock again in 5 seconds
                    } else locktimer = 400;
                }
                unlocktimer = 0;
            }
        }
        else if (Lock == 2)                                                     // Cable lock type Motor?
        {
            if (Error || (State != STATE_C)) {
                if (unlocktimer < 600)                                          // 600ms pulse    
                {
                    SOLENOID_UNLOCK;
                } else SOLENOID_OFF;
                if (unlocktimer++ > 700) {
                    if (PORTCbits.RC1 == 1)                                     // still locked...
                    {
                        if (unlocktimer > 5000) unlocktimer = 0;                //try to unlock again in 5 seconds
                    } else unlocktimer = 700;
                }
                locktimer = 0;
            }
            else                                                                // State C
            {
                if (locktimer < 600)                                            // 600ms pulse
                {
                    SOLENOID_LOCK;
                }
                else SOLENOID_OFF;
                if (locktimer++ > 700) {
                    if (PORTCbits.RC1 == 0)                                     // still unlocked...
                    {
                        if (locktimer > 5000) locktimer = 0;                    //try to lock again in 5 seconds
                    } else locktimer = 700;
                }
                unlocktimer = 0;
            }
        }
         

        Timer++;                                                                // mSec counter (overflows in 1193 hours)
        if (AccessTimer) AccessTimer--;

        if (LedTimer-- == 0) {
            CCPR2L = LedPwm;                                                    // MSB of DutyCycle, Lsb 0-1 are part of CCP2CON, but not used
                                                                                // LedPwm is calculated in the main loop
            LedTimer = 10;                                                      // Led is updated every 10ms (1ms*10)
            LedUpdate = 1;                                                      // Flag that LED PWM value has been updated
        }
        PIR5bits.TMR4IF = 0;                                                    // clear interrupt flag
    }

}

/* triwave8: triangle (sawtooth) wave generator.  Useful for
           turning a one-byte ever-increasing value into a
           one-byte value that oscillates up and down.

           input         output
           0..127        0..254 (positive slope)
           128..255      254..0 (negative slope)
 */
unsigned char triwave8(unsigned char in) {
    if (in & 0x80) {
        in = 255 - in;
    }
    unsigned char out = in << 1;
    return out;
}

unsigned char scale8(unsigned char i, unsigned char scale) {
    return (((unsigned int) i) * (1 + (unsigned int) (scale))) >> 8;
}

/* easing functions; see http://easings.net

    ease8InOutQuad: 8-bit quadratic ease-in / ease-out function
 */
unsigned char ease8InOutQuad(unsigned char i) {
    unsigned char j = i;
    if (j & 0x80) {
        j = 255 - j;
    }
    unsigned char jj = scale8(j, j);
    unsigned char jj2 = jj << 1;
    if (i & 0x80) {
        jj2 = 255 - jj2;
    }
    return jj2;
}



// calculates 16-bit CRC of given data
// used for Frame Check Sequence on data frame
// Poly used is x^16+x^12+x^5+x

unsigned int calc_crc16(char* start, char len) {
    unsigned int crc = 0xffff, c;
    int i;
    while (len--) {
        c = *start;
        for (i = 0; i < 8; i++) {
            if ((crc ^ c) & 1) crc = (crc >> 1)^0x8408;
            else crc >>= 1;
            c >>= 1;
        }
        start++;
    }
    crc = (unsigned int) (crc ^ 0xFFFF);
    return (crc);
}

void eeprom_read_object(void *obj_p, size_t obj_size) {
    unsigned char *p = obj_p;

    EECON1 = 0;                                                                 // select EEprom
                                                                                // EEADR needs to be initialized
    while (obj_size--) {
        EECON1bits.RD = 1;
        *p++ = EEDATA;
        EEADR++;
    }
}

void eeprom_write_object(void *obj_p, size_t obj_size) {
    unsigned char *p = obj_p;

    while (obj_size--) {
        EECON1 = 0;                                                             //ensure CFGS=0 and EEPGD=0
        EECON1bits.WREN = 1;                                                    //enable write to EEPROM

        EEDATA = *p++;                                                          // set data
        if (!INTCONbits.GIE)                                                    // Interrupts should have been disabled!
        {
            EECON2 = 0x55;                                                      // required sequence #1
            EECON2 = 0xAA;                                                      // #2
            EECON1bits.WR = 1;                                                  // #3 = actual write
            while (EECON1bits.WR);                                              // blocking
        }    
        EECON1bits.WREN = 0;                                                    // disable write to EEPROM
        EEADR++;
    }
}

void read_settings(void) {
    char savint;

    savint = INTCON;                                                            // Save interrupts state
    INTCONbits.GIE = 0;                                                         // Disable interrupts

    EEADR = 0;                                                                  // start from adr 0 in eeprom
    EEADRH = 0;                                                                 // we only use the first 256 bytes for now.

    eeprom_read_object(&MaxMains, sizeof MaxMains);
    eeprom_read_object(&MaxCurrent, sizeof MaxCurrent);
    eeprom_read_object(&MinCurrent, sizeof MinCurrent);
    eeprom_read_object(&ICal, sizeof ICal);
    eeprom_read_object(&Mode, sizeof Mode);
    eeprom_read_object(&Lock, sizeof Lock);
    eeprom_read_object(&CableLimit, sizeof CableLimit);
    eeprom_read_object(&Config, sizeof Config);
    eeprom_read_object(&LoadBl, sizeof LoadBl);
    eeprom_read_object(&Access, sizeof Access);
    eeprom_read_object(&RCmon, sizeof RCmon);

    INTCON = savint; // Restore interrupts
}

void write_settings(void) {
    char savint;

    savint = INTCON;                                                            // Save interrupts state
    INTCONbits.GIE = 0;                                                         // Disable interrupts

    EEADR = 0;                                                                  // start from adr 0 in eeprom
    EEADRH = 0;                                                                 // we only use the first 256 bytes for now.

    eeprom_write_object(&MaxMains, sizeof MaxMains);
    eeprom_write_object(&MaxCurrent, sizeof MaxCurrent);
    eeprom_write_object(&MinCurrent, sizeof MinCurrent);
    eeprom_write_object(&ICal, sizeof ICal);
    eeprom_write_object(&Mode, sizeof Mode);
    eeprom_write_object(&Lock, sizeof Lock);
    eeprom_write_object(&CableLimit, sizeof CableLimit);
    eeprom_write_object(&Config, sizeof Config);
    eeprom_write_object(&LoadBl, sizeof LoadBl);
    eeprom_write_object(&Access, sizeof Access);
    eeprom_write_object(&RCmon, sizeof RCmon);

    INTCON = savint;                                                            // Restore interrupts
    printf("\r\nsettings saved\r\n");

}



unsigned char ReadPilot(void)                                                   // Read Pilot Signal 
{
    ADCON0bits.GO = 1;                                                          // initiate ADC conversion on the selected channel
    while (ADCON0bits.GO);
    if (ADRES > 980) return PILOT_12V;                                          // Pilot at 12V (min 11.0V)
    if ((ADRES > 860) && (ADRES < 915)) return PILOT_9V;                        // Pilot at 9V
    if ((ADRES > 720) && (ADRES < 800)) return PILOT_6V;                        // Pilot at 6V
    if ((ADRES > 25) && (ADRES < 95)) return PILOT_DIODE;                       // Diode Check OK
    return PILOT_NOK;                                                           // Pilot NOT ok
}

void ProximityPin(void) {
    ADCON0 = 0b00000101;                                                        // ADC input AN1 (Proximity Pin)
    ADCON2 = 0b10100101;                                                        // Right justify, Tacq = 8 uS, FOSC/16
    delay(100);
    ADCON0bits.GO = 1;                                                          // initiate ADC conversion on the selected channel
    while (ADCON0bits.GO);

    MaxCapacity = 13;                                                           // No resistor, Max cable current = 13A
    if ((ADRES > 394) && (ADRES < 434)) MaxCapacity = 16;                       // Max cable current = 16A  680R
    if ((ADRES > 175) && (ADRES < 193)) MaxCapacity = 32;                       // Max cable current = 32A  220R
    if ((ADRES > 88) && (ADRES < 98)) MaxCapacity = 63;                         // Max cable current = 63A  100R

    if (Config) MaxCapacity = CableLimit;                                       // Override when Fixed Cable is used.  

    ADCON0 = 0b00000001;                                                        // ADC input AN0 (Pilot)
    ADCON2 = 0b10000101;                                                        // Right justify, Tacq = 0 uS, FOSC/16
}

void SetCurrent(unsigned int current)                                           // current in Amps (16= 16A)
{
    unsigned int DutyCycle;

    current = current * 10;                                                     // multiply by 10 (current in Amps x10  (160= 16A) )
    if ((current >= 60) && (current <= 510)) DutyCycle = (unsigned int) (current / 0.6);
                                                                                // calculate DutyCycle from current
    else if ((current > 510) && (current <= 800)) DutyCycle = (unsigned int) (current / 2.5) + 640;
    else DutyCycle = 100;                                                       // invalid, use 6A
    CCPR1L = DutyCycle >> 2;                                                    // Msb of DutyCycle
                                                                                // 2 Lsb are part of CCP1CON, use Timer 2 
    CCP1CON = (((DutyCycle & 0x03) << 4) | 0x0C);                               // PWM Pilot signal enabled
}

// Is there atleast 6A(configurable MinCurrent) available for a EVSE?

char IsCurrentAvailable(void) {
    unsigned char n, ActiveEVSE = 0;
    int Baseload, TotalCurrent = 0;

    for (n = 0; n < 4; n++) if (BalancedState[n] == 2)                          // must be in STATE_C
    {
        ActiveEVSE++;                                                           // Count nr of Active EVSE's
        TotalCurrent += Balanced[n];                                            // Calculate total max charge current for all active EVSE's
    }
    if (ActiveEVSE == 0) {
        if (Imeasured > ((MaxMains - MinCurrent)*10)) {
            return 1;                                                           // Not enough current available!, return with error
        }
    } else {
        ActiveEVSE++;                                                           // Do calculations with one more EVSE
        Baseload = Imeasured - (TotalCurrent * 10);                             // Calculate Baseload (load without any active EVSE)
        if (Baseload < 0) Baseload = 0;

        if (ActiveEVSE > 4) ActiveEVSE = 4;
        if ((ActiveEVSE * (MinCurrent * 10) + Baseload) > (MaxMains * 10)) {
            return 1;                                                           // Not enough current available!, return with error
        }
    }
    return 0;
}


// Calculates Balanced PWM current for each EVSE
// mod =0 normal
// mod =1 we have a new EVSE requesting to start charging.

void CalcBalancedCurrent(char mod) {
    int Average, MaxBalanced, Idifference;
    int BalancedLeft = 0;
    int ActiveMax = 0, TotalCurrent = 0, Baseload;
    char CurrentSet[4] = {0, 0, 0, 0};
    char n;

    if (!LoadBl)                                                                // Load balancing disabled?
    {
        for (n = 1; n < 4; n++) BalancedState[n] = 0;                           // Yes, disable old active Slave states
    }
                                                                                // Do not modify MaxCurrent as it is a config setting. (fix 2.05)
    if (BalancedState[0] == 2 && MaxCurrent > MaxCapacity) ChargeCurrent = MaxCapacity;
    else ChargeCurrent = MaxCurrent;                                            // Instead use new variable ChargeCurrent.

    if (LoadBl < 2) BalancedMax[0] = ChargeCurrent;                             // Load Balancing Disabled or Master: 
                                                                                // update BalancedMax[0] if the MAX current was adjusted using buttons or CLI

    for (n = 0; n < 4; n++) if (BalancedState[n] == 2) {
            BalancedLeft++;                                                     // Count nr of Active (Charging) EVSE's
            ActiveMax += BalancedMax[n];                                        // Calculate total Max Amps for all active EVSEs
            TotalCurrent += Balanced[n];                                        // Calculate total of all set charge currents
        }

    if (!mod) {
        Idifference = (MaxMains * 10) - Imeasured;                              // Difference between MaxMains and Measured current (can be negative)

        if (Idifference > 0) IsetBalanced += (Idifference / 4);                 // increase with 1/4th of difference (slowly increase current)
        else IsetBalanced += Idifference;                                       // last PWM setting + difference (immediately decrease current)
        if (IsetBalanced < 0) IsetBalanced = 0;
    }

    Baseload = Imeasured - (TotalCurrent * 10);                                 // Calculate Baseload (load without any active EVSE)
    if (Baseload < 0) Baseload = 0;

    if (!Mode)                                                                  // Normal Mode
    {
        if (LoadBl) IsetBalanced = MaxMains * 10;                               // Load Balancing active? MAINS is max current for all active EVSE's
        else IsetBalanced = ChargeCurrent * 10;                                 // No Load Balancing in Normal Mode. Set current to ChargeCurrent (fix: v2.05)
    }

    if (BalancedLeft)                                                           // Only if we have active EVSE's
    {

        if (mod) IsetBalanced = (MaxMains * 10) - Baseload;                     // Set max combined charge current to MaxMains - Baseload   

        if (IsetBalanced < 0 || IsetBalanced < (BalancedLeft * (MinCurrent * 10))) {
            NoCurrent++;                                                        // Flag NoCurrent left
            printf("No Current!!\n\r");
            IsetBalanced = (BalancedLeft * (MinCurrent * 10));                  // set minimal "MinCurrent" charge per active EVSE
        } else NoCurrent = 0;

        if (IsetBalanced > (ActiveMax * 10)) IsetBalanced = ActiveMax * 10;     // limit to total maximum Amps (of all active EVSE's)

        MaxBalanced = (IsetBalanced / 10);                                      // convert to Amps

        DEBUG_PRINT(("Imeasured:%3u IsetBalanced:%3i Baseload:%3u ", Imeasured, IsetBalanced, Baseload));

        // Calculate average current per EVSE
        n = 0;
        do {
            Average = MaxBalanced / BalancedLeft;                               // Average current for all active EVSE's

        // Check for EVSE's that have a lower MAX current
            if ((BalancedState[n] == 2) && (!CurrentSet[n]) && (Average >= BalancedMax[n])) // Active EVSE, and current not yet calculated?
            {
                Balanced[n] = BalancedMax[n];                                   // Set current to Maximum allowed for this EVSE
                CurrentSet[n] = 1;                                              // mark this EVSE as set.
                BalancedLeft--;                                                 // decrease counter of active EVSE's
                MaxBalanced -= Balanced[n];                                     // Update total current to new (lower) value
                n = 0;                                                          // check all EVSE's again
            } else n++;
        } while (n < 4 && BalancedLeft);

        // All EVSE's which had a Max current lower then the average are set.
        // Now calculate the current for the EVSE's which had a higher Max current
        n = 0;
        if (BalancedLeft)                                                       // Any Active EVSE's left?
        {
            do {                                                                // Check for EVSE's that are not set yet
                if ((BalancedState[n] == 2) && (!CurrentSet[n]))                // Active EVSE, and current not yet calculated?
                {
                    Balanced[n] = MaxBalanced / BalancedLeft;                   // Set current to Average
                    CurrentSet[n] = 1;                                          // mark this EVSE as set.
                    BalancedLeft--;                                             // decrease counter of active EVSE's
                    MaxBalanced -= Balanced[n];                                 // Update total current to new (lower) value
                }
            } while (++n < 4 && BalancedLeft);
        }

        for (n = 0; n < 4; n++) DEBUG_PRINT(("EVSE%u[%u]:%2uA  ", n, BalancedState[n], Balanced[n]));
        DEBUG_PRINT(("\n\r"));
    } // BalancedLeft

}



void delay(unsigned int d) {
    unsigned long x;
    x = Timer;                                                                  // read Timer value (increased every ms)
    while (Timer < (x + d)) {
    }
}



void init(void) {
    OSCCON = 0b01101100;                                                        // setup external oscillator
    OSCCON2 = 0b00000100;                                                       // primary Oscillator On.

    RCON = 0b10011111;                                                          // Set Interrupt priority 

    PMD0 = 0b00000000;                                                          // Perhiperal Module Enable/Disable
    PMD1 = 0b00000000;                                                          // All enabled
    PMD2 = 0b00000000;

    PORTA = 0;                                                                  // Init PORTA
    ANSELA = 0b00000111;                                                        // RA0, RA1, RA2 are analog inputs (pin 2,3,4)
    TRISA = 0b00000111;                                                         // Set RA0,RA1,RA2 as inputs

    PORTB = 0;
    ANSELB = 0;                                                                 // All digital IO
    TRISB = 0b10000111;                                                         // RB7(RX2), RB0-RB2 inputs. all other output
    WPUB = 0b10000111;                                                          // weak pullup on RB7 and RB0-RB2    
    INTCON2bits.RBPU = 0;                                                       // Enable weak pullups on PORTB

    PORTC = 0;
    ANSELC = 0;                                                                 // All digital IO
    TRISC = 0b10000010;                                                         // RC1 and RC7 input (RX1), all other output

    SPBRGH1 = 13;                                                               // Initialize UART 1 (RS485)
    SPBRG1 = 4;                                                                 // Baudrate 1200 
    BAUDCON1 = 0b00001000;                                                      // 16 bit Baudrate register is used
    TXSTA1 = 0b00100100;                                                        // Enable TX, 8 bit, Asynchronous mode
    RCSTA1 = 0b10010000;                                                        // Enable serial port TX and RX, 8 bit. 

    SPBRGH2 = 0;                                                                // Initialize UART 2
    SPBRG2 = 34;                                                                // Baudrate 115k2 (114285)
    //  SPBRG2 = 207;                                                           // Baudrate 19k2
    BAUDCON2 = 0b00001000;                                                      // 16 bit Baudrate register is used
    TXSTA2 = 0b00100100;                                                        // Enable TX, 8 bit, Asynchronous mode
    RCSTA2 = 0b10010000;                                                        // Enable serial port TX and RX, 8 bit. 

    VREFCON0 = 0b10100000;                                                      // Fixed Voltage reference set to 2.048V

    ADCON0 = 0b00000001;                                                        // ADC On input AN0 (Pilot)
    ADCON1 = 0;
    ADCON2 = 0b10000101;                                                        // Right justify, Tacq = 0 uS, FOSC/16

    T0CON = 0b10000111;                                                         // Timer 0 @ 16Mhz => 32 uS Timer0, 16 bit counter, 1:256 prescaler

    PR2 = 249;                                                                  // Timer 2 frequency value -> 1Khz @ 16 Mhz
    T2CON = 0b00000110;                                                         // Timer 2 ON, prescaler 1:16
    CCP1CON = 0;                                                                // PWM off (Control Pilot signal)
    CCP2CON = 0;                                                                // PWM off (Led on I/O 1)

    PR4 = 249;                                                                  // Timer 4 frequency value -> 1Khz @ 16 Mhz
    T4CON = 0b00000110;                                                         // Timer 4 ON, prescaler 1:16

    // SPI registers
    SSP1STAT = 0b00000000;                                                      // 0 = Input data sampled at middle of data output time
                                                                                // 0 = Transmit occurs on transition from Idle to active clock state
    SSP1CON1 = 0b00010000;                                                      // Idle state for clock is a high level, SPI Master mode, clock = FOSC/4
    SSP1CON1 = 0b00110000;                                                      // SPI enabled,Idle state for clock is a high level, SPI Master mode, clock = FOSC/4

    PIE1bits.RC1IE = 1;                                                         // enable receive Interrupt for UART1
    PIE3bits.RC2IE = 1;                                                         // enable receive Interrupt for UART2
    PIE5bits.TMR4IE = 1;                                                        // enable Timer4 Interrupt


    INTCONbits.GIEH = 1;                                                        // global High Priority interrupts enabled
    INTCONbits.GIEL = 0;                                                        // global Low Priority interrupts disabled

    SOLENOID_OFF;                                                               // R and W outputs held at Capacitor voltage (+12V) 

    CCPR2L = 0;                                                                 // LED DutyCycle 0%
    CCP2CON = 0x0C;                                                             // LED PWM on

    printf("\r\nSmart EVSE powerup.\r\n");

}

void main(void) {
    char *pBytes;
    char x, n;
    unsigned char pilot, count = 0, timeout = 5;
    char DiodeCheck = 0;
    char SlaveAdr, Command, Broadcast = 0, Switch_count = 0;
    unsigned int Current;

    init();                                                                     // initialize ports, ADC, UARTs etc

    read_settings();                                                            // from EEprom
    IsetBalanced = MaxMains * 10;                                               // Initially set to MaxMains

    GLCD_init();
    GLCD_version();                                                             // Display Version

    RCONbits.POR = 1;                                                           // flag that future resets are not POR resets

    while (1)                                                                   // MAIN loop
    {

                
        if (TestState) TestIO();                                                // TestMode. Test all I/O of Module

        if (ISR2FLAG) RS232cli();                                               // RS232 command line interface

        if (!ISRTXFLAG && TXSTA1bits.TRMT) LATBbits.LATB5 = 0;                  // set RS485 transceiver to receive if the last character has been sent

        BlinkLed();                                                             // Handle the blinking of the 12V LED

        TRISC = 0b10100011;                                                     // Set RC5 and RC0 to input. Make sure there are pull-ups on these pins.
        NOP();
        NOP();
        x = (PORTC & 0b00100001);                                               // Read Two Button Inputs on RC5(>) and RC0(select)
        ButtonState = (x >> 3);
        ButtonState = ButtonState | ((x << 1) & 0x02);                          // arranged to lowest bits
        ButtonState = ButtonState | (PORTB & 0x01);                             // Read the state of the last button RB0(<).
        TRISC = 0b10000010;                                                     // RC1 and RC7 input (RX1), all other output

        //printf("ButtonState %02x\r",ButtonState);

        if ((ButtonState != 0x07) || (ButtonState != OldButtonState)) GLCDMenu(ButtonState); // Any button pressed or just released?

        if (LCDNav && (ScrollTimer + 5000 < Timer) && (!SubMenu)) GLCDHelp();   // Update/Show Helpmenu


        if (PORTBbits.RB2 == 0)                                                 // Switch input pulled low?
        {
            if (Switch_count++ > 5) {
                if (AccessTimer == 0) {
                    if (Access)                                                 // Menu option Access is enabled (set to Switch))
                    {
                        if (Access_bit) {
                            Access_bit = 0;                                     // Toggle Access bit on/off
                            State = STATE_A;                                    // Switch back to state A
                        } else Access_bit = 1;

                        printf("access: %d ", Access_bit);
                    } else if (State == STATE_C)                                // Menu option Access is set to Disabled
                    {                                                           // We only use the switch/button now to STOP charging
                        State = STATE_A;
                        if (!TestState) ChargeDelay = 15;                       // Keep in State A for 15 seconds, so the Charge cable can be removed.
                    }

                    if (RCmon == 1 && Error == RCD_TRIPPED && PORTBbits.RB1 == 0) // RCD was tripped, but RCD level is back to normal
                    {
                        Error = NO_ERROR;                                       // Clear error , by pressing the button
                    }
                }                                                               // Reset timer while button is pressed.
                AccessTimer = 200;                                              // this de-bounces the switch, and makes sure we don't toggle between Access and No-Access.    
                Switch_count = 0;                                               // make sure that noise on the input does not switch off charging
            }
        } else Switch_count = 0;

        if (RCmon == 1 && PORTBbits.RB1 == 1)                                   // RCD monitor active, and RCD DC current > 6mA ?
        {
            State = STATE_A;
            Error = RCD_TRIPPED;
            LCDTimer = 0;                                                       // display the correct error message on the LCD
        }


        if ((State == STATE_COMM_A) && (Timer > ACK_TIMEOUT))                   // Wait for response from Master
        {
            SendRS485(LoadBl - 1, 0x01, 0x00, 0x00);                            // Send command to Master
            printf("01 sent to Master, charging stopped\r\n");
            Timer = 0;                                                          // Clear the Timer
        }

        if (State == STATE_A)                                                   // ############### EVSE State A #################
        {
            CCP1CON = 0;                                                        // PWM off
            PORTCbits.RC2 = 1;                                                  // Control pilot static +12V
            CONTACTOR_OFF;                                                      // Contactor OFF
            BalancedState[0] = 0;                                               // Mark as inactive

            pilot = ReadPilot();
            if (pilot == PILOT_12V)                                             // Check if we are disconnected, or forced to State A, but still connected to the EV
            {
                ChargeDelay = 0;                                                // Clear ChargeDelay when disconnected.
            }
            if (pilot == PILOT_9V)                                              // switch to State B ?
            {
                if ((NextState == STATE_B) && (Access_bit || Access == 0))      // Access is permitted when Access is disabled or Access_bit=1
                {
                    if (count++ > 25)                                           // repeat 25 times (changed in v2.05)
                    {
                        if (IsCurrentAvailable() == 1) Error = NOCURRENT;       // Enough current available to start Charging?

                        if (ChargeDelay == 0 && Error == NO_ERROR) {
                            DiodeCheck = 0;
                            ProximityPin();                                     // Sample Proximity Pin
                            printf("Cable limit: %uA  Max: %uA \r\n", MaxCapacity, MaxCurrent);
                            if (MaxCurrent > MaxCapacity) ChargeCurrent = MaxCapacity; // Do not modify Max Cable Capacity or MaxCurrent (fix 2.05)
                            else ChargeCurrent = MaxCurrent;                    // Instead use new variable ChargeCurrent

                            if (LoadBl > 1)                                     // Load Balancing : Slave 
                            {
                                SendRS485(LoadBl - 1, 0x02, 0x00, ChargeCurrent); // Send command to Master, followed by Max Charge Current
                                printf("02 sent to Master, requested %uA\r\n", ChargeCurrent);
                                State = STATE_COMM_B;
                                Timer = 0;                                      // Clear the Timer
                            } else {                                            // Load Balancing: Master or Disabled
                                BalancedMax[0] = MaxCapacity;
                                BalancedState[0] = 1;                           // Mark as active
                                State = STATE_B;                                // switch to State B
                                BacklightTimer = BACKLIGHT;                     // Backlight ON
                                BACKLIGHT_ON;
                                printf("STATE A->B\r\n");
                            }
                        }
                    }
                } else {
                    NextState = STATE_B;
                    count = 0;
                }

            }
        }

        if (State == STATE_COMM_B)                                              // Wait for response from Master
        {
            if (Timer > ACK_TIMEOUT) State = STATE_A;
        }


        if (State == STATE_B)                                                   // ############### EVSE State B #################
        {
                                                                                // measure voltage at ~5% and ~90% of PWM cycle
            if ((TMR2 > 7) && (TMR2 < 24))                                      // PWM cycle 3% - 9% (should be high)
            {
                pilot = ReadPilot();
                if (pilot == PILOT_12V)                                         // Disconnected?
                {
                    if (NextState == STATE_A) {
                        if (count++ > 25)                                       // repeat 25 times (changed in v2.05)
                        {
                            State = STATE_A;                                    // switch to STATE_A
                            printf("STATE B->A\r\n");
                            if (LoadBl > 1)                                     // Load Balancing : Slave 
                            {
                                State = STATE_COMM_A;                           // Tell Master we switched to State A
                                Timer = ACK_TIMEOUT + 1;                        // Set the timer to Timeout value, so that it expires immediately
                            }
                        }
                    } else {
                        NextState = STATE_A;
                        count = 0;
                    }
                } else if (pilot == PILOT_6V) {
                    if ((NextState == STATE_C) && (DiodeCheck == 1)) {
                        if (count++ > 25)                                       // repeat 25 times (changed in v2.05)
                        {
                            if ((Error == NO_ERROR) && (ChargeDelay == 0)) {
                                if (LoadBl > 1)                                 // Load Balancing : Slave 
                                {
                                    SendRS485(LoadBl - 1, 0x03, 0x00, ChargeCurrent); // Send command to Master, followed by Charge Current
                                    printf("03 sent to Master, requested %uA\r\n", ChargeCurrent);
                                    State = STATE_COMM_C;
                                    Timer = 0;                                  // Clear the Timer
                                } else {                                        // Load Balancing: Master or Disabled
                                    BalancedMax[0] = ChargeCurrent;
                                    if (IsCurrentAvailable() == 0) {
                                        BalancedState[0] = 2;                   // Mark as Charging
                                        Balanced[0] = 0;                        // For correct baseload calculation set current to zero
                                        CalcBalancedCurrent(1);                 // Calculate charge current for all connected EVSE's

                                        CONTACTOR_ON;                           // Contactor ON
                                        DiodeCheck = 0;
                                        State = STATE_C;                        // switch to STATE_C
                                        LCDTimer = 0;
                                        Timer = 0;                              // reset msTimer and ChargeTimer
                                        if (!LCDNav)                            // Don't update the LCD if we are navigating the menu
                                        {
                                            GLCD();                             // immediately update LCD
                                        }
                                        printf("STATE B->C\r\n");
                                    }
                                    else Error = NOCURRENT;
                                }
                            }
                        }
                    } else {
                        NextState = STATE_C;
                        count = 0;
                    }
                } else                                                          // PILOT_9V
                {
                    if (NextState == STATE_B)                                   // Did the EV switch from State_C to State_B?
                    {                                                           // then there was probably not enough current available
                                                                                // or the charging was finished.
                    } else NextState = 0;                                       // no State to switch to
                }
            }
            if (TMR2 > 230)                                                     // PWM > 92%
            {
                while (TMR2 < 242);                                             // wait till TMR2 is in range, otherwise we'll miss it (blocking)
                if ((TMR2 > 241) && (TMR2 < 249));                              // PWM cycle >= 96% (should be low)
                {
                    pilot = ReadPilot();
                    if (pilot == PILOT_DIODE) DiodeCheck = 1;                   // Diode found, OK
                    else DiodeCheck = 0;
                }
            }
        }

        if ((State == STATE_COMM_C) && (Timer > ACK_TIMEOUT)) {
            DiodeCheck = 0;
            State = STATE_B;                                                    // switch back to STATE_B
            printf("No ack, STATE C->B\r\n");
        }


        if (State == STATE_C)                                                   // ############### EVSE State C #################
        {
                                                                                // measure voltage at ~5% of PWM cycle
            if ((TMR2 > 7) && (TMR2 < 24))                                      // cycle 3% - 9% (should be high)
            {
                pilot = ReadPilot();
                if ((pilot == PILOT_12V) || (pilot == PILOT_NOK))               // Disconnected or Error?
                {
                    if (NextState == STATE_A) {
                        if (count++ > 25)                                       // repeat 25 times (changed in v2.05)
                        {
                            State = STATE_A;                                    // switch back to STATE_A
                            printf("STATE C->A\r\n");
                            GLCD_init();                                        // Re-init LCD
                            if (LoadBl > 1)                                     // Load Balancing : Slave 
                            {
                                State = STATE_COMM_A;                           // Tell Master we switched to State A
                                Timer = ACK_TIMEOUT + 1;                        // Set the timer to Timeout value, so that it expires immediately
                            }
                            else BalancedState[0] = 0;                          // Master or Disabled
                                                                                // Mark EVSE as disconnected
                        }
                    } else {
                        NextState = STATE_A;
                        count = 0;
                    }
                } else if (pilot == PILOT_9V) {
                    if (NextState == STATE_B) {
                        if (count++ > 25)                                       // repeat 25 times
                        {

                            CONTACTOR_OFF;                                      // Contactor OFF
                            GLCD_init();                                        // Re-init LCD
                            DiodeCheck = 0;
                            State = STATE_B;                                    // switch back to STATE_B
                            if (LoadBl > 1)                                     // Load Balancing : Slave 
                            {
                                State = STATE_COMM_CB;                          // Send 04 command to Master
                                Timer = ACK_TIMEOUT + 1;                        // Set the timer to Timeout value, so that it expires immediately
                            } else BalancedState[0] = 0;                        // Master or Disabled
                                                                                // Mark EVSE as inactive (still State B)
                            printf("STATE C->B\r\n");
                        }
                    } else {
                        NextState = STATE_B;
                        count = 0;
                    }
                } else                                                          // PILOT_6V
                {
                    NextState = 0;                                              // no State to switch to          
                }
            }

        } // end of State C code

        if ((State == STATE_COMM_CB) && (Timer > ACK_TIMEOUT)) {
            SendRS485(LoadBl - 1, 0x04, 0x00, 0x00);                            // Send command to Master
            printf("04 sent to Master, charging stopped\r\n");
            Timer = 0;                                                          // Clear the Timer
        }

        if (Error == NOCURRENT) {
            if (ChargeDelay == 0) printf("Not enough current available!\r\n");
            Error = LESS_6A;
            State = STATE_A;
            ChargeDelay = CHARGEDELAY;                                          // Set Chargedelay after the error clears
        }

        if (RCSTA1bits.OERR)                                                    // Uart1 Overrun Error?
        {
            RCSTA1bits.CREN = 0;
            RCSTA1bits.CREN = 1;                                                // Restart Uart
        }
        if (RCSTA2bits.OERR)                                                    // Uart2 Overrun Error?
        {
            RCSTA2bits.CREN = 0;
            RCSTA2bits.CREN = 1;                                                // Restart Uart
        }

        x = TMR0L;
        if (TMR0H >= 0x3d)                                                      // 1 second timer
        {
            TMR0H = 0;
            TMR0L = 0;

            Temp();                                                             // once a second, measure temperature

            if (ChargeDelay > 0) ChargeDelay--;                                 // Decrease Charge Delay counter

            if ((TempEVSE < 55) && (Error == TEMP_HIGH))                        // Temperature below limit?
            {
                Error = NO_ERROR;                                               // clear Error
            }

            if ((Error == LESS_6A) && (LoadBl < 2) && (IsCurrentAvailable() == 0)) {
                Error = NO_ERROR;                                               // Clear Errors if there is enough current available
            }

            if ((timeout == 0) && (Error == NO_ERROR))                          // timeout if CT current measurement takes > 10 secs
            {
                Error = CT_NOCOMM;
                State = STATE_A;                                                // switch back to state A
                printf("Error, communication error!\r\n");
                for (x = 0; x < 4; x++) BalancedState[x] = 0;                   // reset all states
            } else if (timeout) timeout--;

            if (TempEVSE >= 65)                                                 // Temperature too High?
            {
                Error = TEMP_HIGH;
                State = STATE_A;                                                // ERROR, switch back to STATE_A
                printf("Temperature too High!\r\n");
                for (x = 0; x < 4; x++) BalancedState[x] = 0;                   // reset all states
            }

            GLCD();                                                             // once a second, update LCD

            //      if (State==STATE_C) ChargeTimer=Timer/1000; // Update ChargeTimer (unused)
            //      printf("STATE:%c Pilot:%u ChargeDelay:%u CT1:%3u.%01uA CT2:%3u.%01uA CT3:%3u.%01uA Imeas:%3u.%01uA Iset:%u.%01uA\r\n",State-1+'A',pilottest, ChargeDelay, (unsigned int)Irms[0]/10, (unsigned int)Irms[0]%10, (unsigned int)Irms[1]/10, (unsigned int)Irms[1]%10, (unsigned int)Irms[2]/10, (unsigned int)Irms[2]%10,(unsigned int)Imeasured/10,(unsigned int)Imeasured%10,(unsigned int)Iset/10,(unsigned int)Iset%10);

            if (!Mode)                                                          // Normal mode
            {
                Imeasured = 0;                                                  // No measurements, so we set it to zero
                if (Broadcast) Broadcast--;                                     // once every two seconds, Broadcast charge current to all EVSE's

                if (LoadBl < 2 && !Broadcast)                                   // Load Balancing mode: Master or Disabled
                {
                    CalcBalancedCurrent(0);                                     // Calculate charge current for connected EVSE's
                    if (LoadBl == 1) BroadcastCurrent();                        // Send to all EVSE's (only in Master mode)

                    if ((State == STATE_B) || (State == STATE_C)) SetCurrent(Balanced[0]); // set PWM output for Master
                    Broadcast = 2;                                              // reset counter to 2 seconds
                    timeout = 10;                                               // reset timeout counter (not checked for Master)
                }
            }

        } // end 1 second timer



 

    } // end of while(1) loop
}
