/*
       Frekvens Engineering copyright (c) 2019
-----------------------------------------------------------------------------
       SWI.c
-----------------------------------------------------------------------------

  This file contains functions that drive the Single Wire Interface (SWI)
  on the MCU as a bit banged communications port.

*/
/*-------------- HEADER INCLUDE FILES ---------------------------------------*/
#include <asf.h>
#include "Timers.h"
#include "WatchDogTimer.h"
#include "SWI.h"


/*
 Private Data Structures
*/
static uint8_t swiFlagCmd = 0x77;    // flag preceding a command
static uint8_t swiFlagTx = 0x88;     // flag requesting a response
static uint8_t swiFlagIdle = 0xBB;   // flag requesting to go into Idle mode
static uint8_t swiFlagSleep = 0xCC;  // flag requesting to go into Sleep mode


/*
 Private Function Prototypes
*/
static void swiSendBytes(uint8_t count, uint8_t *buffer);
static fv_return_t swiReceiveBytes(uint8_t count, uint8_t *buffer);
static void sclLow(void);
static void sdaHigh(void);


/*
 Global Function Definitions
*/
/*

  Function Name: SwiInit

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description:  This function sets up the SWI port as a master device.

*/
void SwiInit(void)
{
    sclLow();
    sdaHigh();
}

/*

  Function Name: SwiDeInit

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description:  This function sets the SWI port to a low power state.

*/
void SwiDeInit(void)
{
    sclLow();
    sdaHigh();
}


/*

  Function Name: SwiSleep

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description:  This function sends an SWI sleep flag.

*/
void SwiSleep(void)
{
	sdaHigh();
	swiSendBytes(1, &swiFlagSleep);
}


/*

  Function Name: SwiIdle

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description:  This function sends an SWI idle flag.

*/
void SwiIdle(void)
{
	sdaHigh();
	swiSendBytes(1, &swiFlagIdle);
}


/*

  Function Name: SwiReceive

  Inputs:
    len - Expected number of bytes to read from the device

  Outputs
    pData - Points to location to store the data read from the device

  Return Values
    None

  Description: Receive byte(s) via SWI.

*/
fv_return_t SwiReceive(uint8_t *pData, uint16_t len)
{
	fv_return_t retVal = TIMED_OUT;

	int retries = 3;

// Set SWI pin
	sdaHigh();

	while (retries-- > 0 && retVal != SUCCESS) 
	{
		swiSendBytes(1, &swiFlagTx);
		retVal = swiReceiveBytes(len, pData);
	}

	return retVal;
}	


/*

  Function Name: SwiSend

  Inputs:
    pData - Array of data to write to the device
    len - The number of valid bytes in pData[]

  Outputs
    None

  Return Values
    None

  Description: Send byte(s) via SWI.

*/
void SwiSend(uint8_t *pData, uint16_t len)
{
	// Set SWI pin
	sdaHigh();

// Send Command Flag
	swiSendBytes(1, &swiFlagCmd);

// Send the remaining bytes
	swiSendBytes(len, pData);
}	

/*
 Private Function Definitions
*/
/*

  Function Name: swiSendBytes

  Inputs:
    count - number of bytes to send.
    buffer - pointer to buffer containing bytes to send

  Outputs
    None

  Return Values
    None

  Description: Send a number of bytes.

*/
static void swiSendBytes(uint8_t count, uint8_t *buffer)
{
	uint8_t i, bit_mask;
    struct port_config pin_conf; 

	port_get_config_defaults(&pin_conf); 
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT; 
	port_pin_set_config(SWI_SDA_PIN, &pin_conf); 

// Wait turn around time.
	RX_TX_DELAY;
	cpu_irq_disable();


	for (i = 0; i < count; i++) 
	{
		for (bit_mask = 1; bit_mask > 0; bit_mask <<= 1) 
		{
			if (bit_mask & buffer[i]) 
			{
				port_pin_set_output_level(SWI_SDA_PIN, false);
				BIT_DELAY_1L;
				port_pin_set_output_level(SWI_SDA_PIN, true);
				BIT_DELAY_7;
			}else 
			{
				port_pin_set_output_level(SWI_SDA_PIN, false);
				BIT_DELAY_1L;
				port_pin_set_output_level(SWI_SDA_PIN, true);
				BIT_DELAY_1H;
				port_pin_set_output_level(SWI_SDA_PIN, false);
				BIT_DELAY_1L;
				port_pin_set_output_level(SWI_SDA_PIN, true);
				BIT_DELAY_5;
			}
		}
	}
	port_get_config_defaults(&pin_conf);
	port_pin_set_config(SWI_SDA_PIN, &pin_conf);
	cpu_irq_enable();
}


/*

  Function Name: swiReceiveBytes

  Inputs:
    count - number of bytes to receive

  Outputs
    buffer - pointer to receive buffer

  Return Values
    None

  Description: Send a number of bytes.

*/
static fv_return_t swiReceiveBytes(uint8_t count, uint8_t *buffer)
{
	fv_return_t retVal = SUCCESS;

	uint8_t i;
	uint8_t bit_mask;
	uint8_t pulse_count;
	uint16_t timeout_count;
	struct port_config pin_conf;
	
	port_get_config_defaults(&pin_conf);
	port_pin_set_config(SWI_SDA_PIN, &pin_conf);
	
	cpu_irq_disable();
// Receive bits and store in buffer.
	for (i = 0; i < count; i++) 
	{
		buffer[i]=0;
		for (bit_mask = 1; bit_mask > 0; bit_mask <<= 1) 
		{
			pulse_count = 0;

			timeout_count = START_PULSE_TIME_OUT;
// Detect start bit.

			while (--timeout_count > 0) 
			{
// Wait for falling edge.
				if ( port_pin_get_input_level(SWI_SDA_PIN) == 0)
					break;
			}
			if (timeout_count == 0) 
			{
				retVal = TIMED_OUT;
				break;
			}
			
			timeout_count = START_PULSE_TIME_OUT;
			
			do 
			{
// Wait for rising edge.
				if ( port_pin_get_input_level(SWI_SDA_PIN) != 0) 
				{
					pulse_count = 1;
					break;
				}
			} while (--timeout_count > 0);

			if (pulse_count == 0) 
			{
				retVal = TIMED_OUT;
				break;
			}

//  let's just wait the maximum time for the falling edge of a zero bit
// to arrive after we have detected the rising edge of the start bit.
			timeout_count = ZERO_PULSE_TIME_OUT;
			
// Detect possible edge indicating zero bit.
			do {
				if ( port_pin_get_input_level(SWI_SDA_PIN) == 0) 
				{
					pulse_count = 2;
					break;
				}
			} while (--timeout_count > 0);
			

// Wait for rising edge of zero pulse before returning. Otherwise we might interpret
// its rising edge as the next start pulse.
			if (pulse_count == 2) 
			{
				timeout_count = ZERO_PULSE_TIME_OUT;
				do
				if ( port_pin_get_input_level(SWI_SDA_PIN) != 0)
				{
					break;
				}
				while (timeout_count-- > 0);
				
			}
// Update byte at current buffer index.
			else
// received "one" bit
			{
				buffer[i] |= bit_mask;
			}
		}
		

		if (retVal != SUCCESS)
		break;
	}

	if (retVal == TIMED_OUT) 
	{
		if (i > 0)
// Indicate that we timed out after having received at least one byte.
		retVal = FAILED;
	}

	cpu_irq_enable();

	return retVal;
}


/*

  Function Name: sclLow

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description: Sets the SCL pin as an output and drives the SCL2 line low.

*/
static void sclLow(void) 
{
	struct port_config pin_conf;

    //
	// Set the SCL pin as an output
	//
	port_get_config_defaults(&pin_conf);
    pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(SWI_SCL_PIN, &pin_conf);
	
	//
	// Drive the SCL pin low.
	//
    port_pin_set_output_level(SWI_SCL_PIN, false);
}	


/*

  Function Name: sdaHigh

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description: Sets the SDA pin as an input and lets the
               SDA2 line float high (pull-up needed).

*/
static void sdaHigh(void)
{
	struct port_config pin_conf;

	//
	// Set the SDA pin as an pulled-up input
	//
	port_get_config_defaults(&pin_conf);
	port_pin_set_config(SWI_SDA_PIN, &pin_conf);
}
