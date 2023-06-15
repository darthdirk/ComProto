/*
       Frekvens Engineering Copyright (c) 2016
-----------------------------------------------------------------------------
       uart.c
-----------------------------------------------------------------------------

  contents =f(x) service the UARTs of the MCU
----------------------------------------------------------------------------
*/
#include <asf.h>
#include "FvTypes.h"
#include "fifo.h"
#include "feature.h"
#include "HostInterface.h"
#include "Timers.h"
#include "FvGlobals.h"
#include "uart.h"

/*
* Private Data Structures
*/
volatile FIFO_T rx1Fifo;
volatile uint8_t rx1FifoBuffer[MAX_UART_BUF_SIZE];
volatile FIFO_T rx2Fifo;
volatile uint8_t rx2FifoBuffer[EXT_BLE_MAX_DATA_SIZE];
volatile FIFO_T rx3Fifo;
volatile uint8_t rx3FifoBuffer[FEATURE_BUF_SIZE];

// Must be globals
volatile uint16_t rx1Word;
volatile uint16_t rx2Word;
volatile uint16_t rx3Word;
struct usart_module usart1Instance;
struct usart_module usart2Instance;
struct usart_module usart3Instance;



/*
 Global Function Definitions
*/

/*

  Function Name: UartMapBaudRate

Inputs:
    br - Indicates which standard baudrate to use

  Outputs
    None

  Return Values
    32 bit baud rate

  Description:  This function maps a baudrate index value to a 
                32 bit baud rate.

*/
uint32_t UartMapBaudRate(uint8_t br)
{
	uint32_t baudRate;
	
	switch (br)
	{
		case BR_19200:
		baudRate = 19200L;
		break;
		
		case BR_38400:
		baudRate = 38400L;
		break;
		
		case BR_57600:
		baudRate = 57600L;
		break;
		
		case BR_115200:
		baudRate = 115200L;
		break;
		
		case BR_230400:
		baudRate = 230400L;
		break;
		
		case BR_460800:
		baudRate = 460800L;
		break;
		
		case BR_9600:  // Fall Through
		default:
			baudRate = 9600L;
		break;
	}
	
	return baudRate;
}


/*

  Function Name: InitUart1

  Inputs:
    br - Baud rate Indicator
    fClearFifo - TRUE means initialize the FIFO buffer.

  Outputs
    None

  Return Values
    None

  Description:  This function initializes UART1 for interrupt driven input,
                and polled handling of output and error conditions.

*/
void InitUart1(uint8_t br, boolean_t fClearFifo)
{
	struct usart_config configUsart;
	enum system_interrupt_priority_level pLevel;

	if (TRUE == fClearFifo)
	{
		//
		// Initialize the receive FIFO
		//
		InitFifo((FIFO_T *)(&rx1Fifo), (uint8_t *)&rx1FifoBuffer[0], MAX_UART_BUF_SIZE);		
	}
	
	//
	// Configure settings for the UART1 instance specific to STM32F103
	//
	usart_get_config_defaults(&configUsart);
	
	configUsart.baudrate = UartMapBaudRate(br);
	configUsart.mux_setting = USART_RX_3_TX_0_XCK_1;
	configUsart.pinmux_pad0 = PINMUX_PA22C_SERCOM3_PAD0;
	configUsart.pinmux_pad1 = PINMUX_UNUSED;
	configUsart.pinmux_pad2 = PINMUX_UNUSED;
	configUsart.pinmux_pad3 = PINMUX_PA25C_SERCOM3_PAD3;
	
	//
	// Initialize the UART1 instance.
	//
	while (usart_init(&usart1Instance, SERCOM3, &configUsart) != STATUS_OK) {}
	
	//
	// Enable the UART1 instance.
	//
	usart_enable(&usart1Instance);
	
	//
	// Register and enable the callback for the receive module.
	//
	usart_register_callback(&usart1Instance, Uart1RxCallback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart1Instance, USART_CALLBACK_BUFFER_RECEIVED);
	
	pLevel = NVIC_GetPriority(SERCOM3_IRQn);
	if (pLevel != SYSTEM_INTERRUPT_PRIORITY_LEVEL_3)
	{
		NVIC_SetPriority(SERCOM3_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_3);
	}

	//
	// Start the receive interrupt.
	//
	while (STATUS_BUSY == usart_read_job(&usart1Instance, (uint16_t *)&rx1Word)) ;
}


/*

 Function Name: DeInitUart1

  Inputs:

  Outputs
    None

  Return Values
    None.

  Description:  This function de-initializes UART1. This function is needed
                for supporting the osdp_COMSET command.

*/
void DeInitUart1(void)
{
	//
	// Disable the UART1 instance.
	//
	usart_disable(&usart1Instance);

	//
	// Initialize the receive FIFO
	//
	InitFifo((FIFO_T *)(&rx1Fifo), (uint8_t *)&rx1FifoBuffer[0], MAX_UART_BUF_SIZE);
}


/*

  Function Name: InitUart2

  Inputs:
    br - Baud rate Indicator

  Outputs
    None

  Return Values
    None

  Description:  This function initializes UART2 for interrupt driven input,
                and polled handling of output and error conditions.

*/
void InitUart2(uint8_t br)
{
	struct usart_config configUsart;

	//
	// Initialize the receive FIFO
	//
	InitFifo((FIFO_T *)(&rx2Fifo), (uint8_t *)&rx2FifoBuffer[0], EXT_BLE_MAX_DATA_SIZE);

	//
	// Configure settings for the UART2 instance specific to STM32F103
	//
	usart_get_config_defaults(&configUsart);
	configUsart.baudrate = UartMapBaudRate(br);
	configUsart.mux_setting = USART_RX_0_TX_2_XCK_3;
	configUsart.pinmux_pad0 = PINMUX_PB30D_SERCOM5_PAD0;
	configUsart.pinmux_pad1 = PINMUX_UNUSED;
	configUsart.pinmux_pad2 = PINMUX_PB00D_SERCOM5_PAD2;
	configUsart.pinmux_pad3 = PINMUX_UNUSED;
	
	//
	// Initialize the UART2 instance.
	//
	while (usart_init(&usart2Instance, SERCOM5, &configUsart) != STATUS_OK) {}
	
	//
	// Enable the UART2 instance.
	//
	usart_enable(&usart2Instance);
	
	//
	// Register and enable the callback for the receive module.
	//
	usart_register_callback(&usart2Instance, Uart2RxCallback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart2Instance, USART_CALLBACK_BUFFER_RECEIVED);
	
	//
	// Start the receive interrupt.
	//
	while (STATUS_BUSY == usart_read_job(&usart2Instance, (uint16_t *)&rx2Word)) ;
}


/*

  Function Name: InitUart3

  Inputs:
    br - Baud rate Indicator


  Outputs
    None

  Return Values
    None

  Description:  This function initializes UART3 for interrupt driven input,
                and polled handling of output and error conditions.

*/
void InitUart3(uint8_t br)
{
	struct usart_config configUsart;

	//
	// Initialize the receive FIFO
	//
	InitFifo((FIFO_T *)(&rx3Fifo), (uint8_t *)&rx3FifoBuffer[0], UINTER_BUF_SIZE);

	//
	// Configure settings for the UART3 instance specific to STM32F103
	//
	usart_get_config_defaults(&configUsart);
	configUsart.baudrate = UartMapBaudRate(br);
	configUsart.mux_setting = USART_RX_1_TX_0_XCK_1;
	configUsart.pinmux_pad0 = PINMUX_PA12C_SERCOM2_PAD0;
	configUsart.pinmux_pad1 = PINMUX_PA13C_SERCOM2_PAD1;
	configUsart.pinmux_pad2 = PINMUX_UNUSED;
	configUsart.pinmux_pad3 = PINMUX_UNUSED;
	
	//
	// Initialize the UART3 instance.
	//
	while (usart_init(&usart3Instance, SERCOM2, &configUsart) != STATUS_OK) {}

	//
	// Enable the UART3 instance.
	//
	usart_enable(&usart3Instance);
	
	//
	// Register and enable the callback for the receive module.
	//
	usart_register_callback(&usart3Instance, Uart3RxCallback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart3Instance, USART_CALLBACK_BUFFER_RECEIVED);
	
	//
	// Start the receive interrupt.
	//
	while (STATUS_BUSY == usart_read_job(&usart3Instance, (uint16_t *)&rx3Word)) ;
}


/*

  Function Name: SleepUart1

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description:  This function sets up UART1 to sleep and to awake
                on the first bit transition of and incoming byte
                on the Rx on UART1.

*/
void SleepUart1(void)
{
}


/*

  Function Name: SleepUart2

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description:  This function sets up UART2 to sleep and to awake
                on the first bit transition of and incoming byte
                on the Rx on UART2.

*/
void SleepUart2(void)
{
}


/*

  Function Name: SleepUart3

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description:  This function sets up UART3 to sleep and to awake
                on the first bit transition of and incoming byte
                on the Rx on UART3.

*/
void SleepUart3(void)
{
}


/*

  Function Name: PutCharUart1

  Inputs:
    txChar - byte to transmit

  Outputs
    None

  Return Values
    None

  Description:  This function transmits one byte out the UART1 Tx line.

  Note: This function is blocking.

*/
void PutCharUart1(uint8_t txChar)
{
	usart_write_buffer_wait(&usart1Instance, &txChar, 1);
}


/*

  Function Name: PutCharUart2

  Inputs:
    txChar - byte to transmit

  Outputs
    None

  Return Values
    None

  Description:  This function transmits one byte out the UART2 Tx line.

  Note: This function is blocking.

*/
void PutCharUart2(uint8_t txChar)
{
	usart_write_buffer_wait(&usart2Instance, &txChar, 1);
}


/*

  Function Name: PutCharUart3

  Inputs:
    txChar - byte to transmit

  Outputs
    None

  Return Values
    None

  Description:  This function transmits one byte out the UART3 Tx line.

  Note: This function is blocking.

*/
void PutCharUart3(uint8_t txChar)
{
	usart_write_buffer_wait(&usart3Instance, &txChar, 1);
}


/*

  Function Name: PutStringUart1

  Inputs:
    pString - Points to the string of bytes to transmit byte to transmit
    len - Number of valid bytes in pString[].

  Outputs
    None

  Return Values
    None

  Description:  This function transmits an array of bytes out the
                UART1 Tx line.

  Note: This function is blocking.

*/
void PutStringUart1(uint8_t *pString, uint16_t len)
{
	usart_write_buffer_wait(&usart1Instance, pString, len);
}


/*

  Function Name: PutStringUart2

  Inputs:
    pString - Points to the string of bytes to transmit byte to transmit
    len - Number of valid bytes in pString[].

  Outputs
    None

  Return Values
    None

  Description:  This function transmits an array of bytes out the
                UART2 Tx line.

  Note: This function is blocking.

*/
void PutStringUart2(uint8_t *pString, uint16_t len)
{
	usart_write_buffer_wait(&usart2Instance, pString, len);
}


/*

  Function Name: PutStringUart3

  Inputs:
    pString - Points to the string of bytes to transmit byte to transmit
    len - Number of valid bytes in pString[].

  Outputs
    None

  Return Values
    None

  Description:  This function transmits an array of bytes out the
                UART3 Tx line.

  Note: This function is blocking.

*/
void PutStringUart3(uint8_t *pString, uint16_t len)
{
	usart_write_buffer_wait(&usart3Instance, pString, len);
}


/*

  Function Name: GetCharUart1

  Inputs:
    None

  Outputs
    pRxChar - Location to store the incoming byte

  Return Values
    SUCCESS - byte retrieved from FIFO
    FAILED -  no byte available for retrieval

  Description:  This function updates the FIFO from any data available in the
                receive buffer of the UART and then, if a byte is available,
                it retrieves one byte from the FIFO.

  Note: The receive buffer is populated via interrupt.
        The FIFO may be updated with multiple bytes from the receive buffer,
        but on exit from this function only one byte is retrieved from the
        FIFO.

*/
fv_return_t GetCharUart1(uint8_t *pRxChar)
{
    fv_return_t retState = FAILED;
	
    *pRxChar = 0;
    if (FIFO_EMPTY != rx1Fifo.fifoState)
    {
        GetCharFifo((FIFO_T *)(&rx1Fifo), pRxChar);
        retState = SUCCESS;
    }

    return retState;
}


/*

  Function Name: GetCharUart2

  Inputs:
    None

  Outputs
    pRxChar - Location to store the incoming byte

  Return Values
    SUCCESS - byte retrieved from FIFO
    FAILED -  no byte available for retrieval

  Description:  This function updates the FIFO from any data available in the
                receive buffer of the UART and then, if a byte is available,
                it retrieves one byte from the FIFO.

  Note: The receive buffer is populated via interrupt.
        The FIFO may be updated with multiple bytes from the receive buffer,
        but on exit from this function only one byte is retrieved from the
        FIFO.

*/
fv_return_t GetCharUart2(uint8_t *pRxChar)
{
	fv_return_t retState = FAILED;

	*pRxChar = 0;
	if (FIFO_EMPTY != rx2Fifo.fifoState)
	{
		GetCharFifo((FIFO_T *)(&rx2Fifo), pRxChar);
		retState = SUCCESS;
	}

	return retState;
}


/*

  Function Name: GetCharUart3

  Inputs:
    None

  Outputs
    pRxChar - Location to store the incoming byte

  Return Values
    SUCCESS - byte retrieved from FIFO
    FAILED -  no byte available for retrieval

  Description:  This function updates the FIFO from any data available in the
                receive buffer of the UART and then, if a byte is available,
                it retrieves one byte from the FIFO.

  Note: The receive buffer is populated via interrupt.
        The FIFO may be updated with multiple bytes from the receive buffer,
        but on exit from this function only one byte is retrieved from the
        FIFO.

*/
fv_return_t GetCharUart3(uint8_t *pRxChar)
{
	fv_return_t retState = FAILED;

	*pRxChar = 0;
	if (FIFO_EMPTY != rx3Fifo.fifoState)
	{
		GetCharFifo((FIFO_T *)(&rx3Fifo), pRxChar);
		retState = SUCCESS;
	}

	return retState;
}


/*

  Function Name: SneakUart1

  Inputs:
    None

  Outputs
    None

  Return Values
    SUCCESS - at lease one byte available in receive buffer
    FAILED -  no bytes available in receive buffer

  Description:  This function indicates if one or more bytes are available
                in the incoming receive buffer of the UART.

  Note: The receive buffer is populated via interrupt.

*/
fv_return_t SneakUart1(void)
{
    fv_return_t retState;

    if (FIFO_EMPTY != rx1Fifo.fifoState)
    {
        retState = SUCCESS;
    }
    else
    {
        retState = FAILED;
    }

    return retState;
}


/*

  Function Name: SneakUart2

  Inputs:
    None

  Outputs
    None

  Return Values
    SUCCESS - at lease one byte available in receive buffer
    FAILED -  no bytes available in receive buffer

  Description:  This function indicates if one or more bytes are available
                in the incoming receive buffer of the UART.

  Note: The receive buffer is populated via interrupt.

*/
fv_return_t SneakUart2(void)
{
	fv_return_t retState;

	if (FIFO_EMPTY != rx2Fifo.fifoState)
	{
		retState = SUCCESS;
	}
	else
	{
		retState = FAILED;
	}

	return retState;
}


/*

  Function Name: SneakUart3

  Inputs:
    None

  Outputs
    None

  Return Values
    SUCCESS - at lease one byte available in receive buffer
    FAILED -  no bytes available in receive buffer

  Description:  This function indicates if one or more bytes are available 
                in the incoming receive buffer of the UART. 

  Note: The receive buffer is populated via interrupt.

*/
fv_return_t SneakUart3(void)
{
	fv_return_t retState;

	if (FIFO_EMPTY != rx3Fifo.fifoState)
	{
		retState = SUCCESS;
	}
	else
	{
		retState = FAILED;
	}

	return retState;
}


/*

  Function Name: Uart1RxCallback

 Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description:   This callback function stores incoming data from UART1 and
                 stores the data in the UART1 Receive FIFO.

*/
void Uart1RxCallback(struct usart_module *const usartModule)
{
	uint8_t b;
	
	b = (uint8_t)rx1Word;
	while (STATUS_BUSY == usart_read_job(usartModule, (uint16_t *)&rx1Word)) ;
	
	if (HOST_OSDP_PROTOCOL == ActiveHostProtocol)
	{
		OsdpRxInt(b);
	}
	else
	{
		//
		// Place the received byte into the receive FIFO buffer.
		// PutCharFifo() cannot be used here because this function
		// is called by an interrupt.
		//
		if ( FIFO_FULL != rx1Fifo.fifoState )
		{
			rx1Fifo.fifoState = FIFO_IN_USE;
			rx1Fifo.pFifoBuffer[rx1Fifo.fifoHead] = b;
			rx1Fifo.fifoHead++;
			if (rx1Fifo.fifoHead >= rx1Fifo.fifoMaxLen)
			{
				rx1Fifo.fifoHead = 0;
			}
			if (rx1Fifo.fifoHead == rx1Fifo.fifoTail)
			{
				rx1Fifo.fifoState = FIFO_FULL;
			}
		}
	}
}


/*

  Function Name: Uart2RxCallback

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description:   This callback function stores incoming data from UART1 and
                 stores the data in the UART2 Receive FIFO.

*/
void Uart2RxCallback(struct usart_module *const usartModule)
{
	//
	// Place the received byte into the receive FIFO buffer.
	// PutCharFifo() cannot be used here because this function
	// is called by an interrupt.
	//
	if ( FIFO_FULL != rx2Fifo.fifoState )
	{
		rx2Fifo.fifoState = FIFO_IN_USE;
		rx2Fifo.pFifoBuffer[rx2Fifo.fifoHead] = (uint8_t)rx2Word;
		rx2Fifo.fifoHead++;
		if (rx2Fifo.fifoHead >= rx2Fifo.fifoMaxLen)
		{
			rx2Fifo.fifoHead = 0;
		}
		if (rx2Fifo.fifoHead == rx2Fifo.fifoTail)
		{
			rx2Fifo.fifoState = FIFO_FULL;
		}
	}
	
	//
	// Restart the interrupt
	//
	while (STATUS_BUSY == usart_read_job(usartModule, (uint16_t *)&rx2Word)) ;
}


/*

  Function Name: Uart3RxCallback

  Inputs:
    None

  Outputs
    None

  Return Values
    None

  Description:   This callback function stores incoming data from UART1 and
                 stores the data in the UART3 Receive FIFO.

*/
void Uart3RxCallback(struct usart_module *const usartModule)
{
	//
	// Place the received byte into the receive FIFO buffer.
	// PutCharFifo() cannot be used here because this function
	// is called by an interrupt.
	//
	if ( FIFO_FULL != rx3Fifo.fifoState )
	{
		rx3Fifo.fifoState = FIFO_IN_USE;
		rx3Fifo.pFifoBuffer[rx3Fifo.fifoHead] = (uint8_t)rx3Word;
		rx3Fifo.fifoHead++;
		if (rx3Fifo.fifoHead >= rx3Fifo.fifoMaxLen)
		{
			rx3Fifo.fifoHead = 0;
		}
		if (rx3Fifo.fifoHead == rx3Fifo.fifoTail)
		{
			rx3Fifo.fifoState = FIFO_FULL;
		}
	}

	//
	// Restart the interrupt
	//
	while (STATUS_BUSY == usart_read_job(usartModule, (uint16_t *)&rx3Word)) ;
}
