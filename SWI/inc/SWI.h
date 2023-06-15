/*
   Frekvens Engineering Copyright (c) 2019
-----------------------------------------------------------------------------
       SWI.h
-----------------------------------------------------------------------------

  This file contains definitions for functions defined in SWI.c
-----------------------------------------------------------------------------
*/
#ifndef SWI_H_
#define SWI_H_


// Macros for Bit-Banged SWI Timing

// bits at 230.4 kbps.


// delay macro for width of one pulse (start pulse or zero pulse)
#define BIT_DELAY_1L        DelayUs(3)
#define BIT_DELAY_1H        DelayUs(3)

// time to keep pin high for five pulses plus stop bit (used to bit-bang 'zero' bit)
// considering pin set delay
#define BIT_DELAY_5        DelayUs(26)    

// time to keep pin high for seven bits plus stop bit (used to bit-bang 'one' bit)
// considering pin set delay
#define BIT_DELAY_7        DelayUs(34)

// turn around time when switching from receive to transmit
// should be 15 us, is 15.58 us
#define RX_TX_DELAY        DelayUs(15)


// Lets set the timeout value for start pulse detection to the uint8_t maximum.
// This value is decremented while waiting for the falling edge of a start pulse.
// Note: 50 iterations for every 80 uS of wait time
//       So 600 gives a wait time of about a millisecond. 
#define START_PULSE_TIME_OUT    (600)

// Maximum time between rising edge of start pulse
// and falling edge of zero pulse is 8.6 us. Therefore, a value of 40 (around 15 us)
// This value is decremented while waiting for the falling edge of a zero pulse.
// (Working range seems to be 7 .. 55, so 30 is the mid-point)
#define ZERO_PULSE_TIME_OUT     (30)


//
// Global functions defined in SWI.c
//
void SwiInit(void);
void SwiDeInit(void);
void SwiSleep(void);
void SwiIdle(void);
fv_return_t SwiReceive(uint8_t *pData, uint16_t len);
void SwiSend(uint8_t *pData, uint16_t len);

#endif // SWI_H_

