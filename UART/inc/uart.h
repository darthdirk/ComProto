/*
       Frekvens Engineering Copyright (c) 2016
-----------------------------------------------------------------------------
       uart.h
-----------------------------------------------------------------------------

  contents = def for functions in uart.c
-----------------------------------------------------------------------------
*/

#define MAX_UART_BUF_SIZE  1300

//
// Baud Rate Map
//
#define BR_9600          0
#define BR_19200         1
#define BR_38400         2
#define BR_57600         3
#define BR_115200        4
#define BR_230400        5
#define BR_460800        6

/*
* Global Function Prototypes
*/
uint32_t UartMapBaudRate(uint8_t br);
void InitUart1(uint8_t br, boolean_t fClearFifo);
void DeInitUart1(void);
void InitUart2(uint8_t br);
void InitUart3(uint8_t br);
void SleepUart1(void);
void SleepUart2(void);
void SleepUart3(void);
void PutCharUart1(uint8_t txChar);
void PutCharUart2(uint8_t txChar);
void PutCharUart3(uint8_t txChar);
void PutStringUart1(uint8_t *pString, uint16_t len);
void PutStringUart2(uint8_t *pString, uint16_t len);
void PutStringUart3(uint8_t *pString, uint16_t len);
fv_return_t GetCharUart1(uint8_t *pRxChar);
fv_return_t GetCharUart2(uint8_t *pRxChar);
fv_return_t GetCharUart3(uint8_t *pRxChar);
fv_return_t PeekUart1(void);
fv_return_t PeekUart2(void);
fv_return_t PeekUart3(void);
void Uart1RxCallback(struct usart_module *const usartModule);
void Uart2RxCallback(struct usart_module *const usartModule);
void Uart3RxCallback(struct usart_module *const usartModule);

