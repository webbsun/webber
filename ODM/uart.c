//Copyright(c) 2013 PAGA CORP. All rights reserved.

#include <stdio.h>
#include <stdarg.h>
#include "uart.h"
#include "msgDef.h"
#include "genApi.h"

#include "stm32l4xx_hal.h"
//#include "sys_type.h"
//#include "M051Series.h"

#define MAX_UART_BUF_TX  (32 * 8)		/*reserve 8 times*/
#define PRINT_BUF_LEN 12
#define UART_TX_OFF	0
#define UART_TX_ON	1

extern UART_HandleTypeDef hlpuart1;
static char uart_tx_buf[MAX_UART_BUF_TX];
static volatile uint16_t	uart_tx_state = UART_TX_OFF;
static volatile uint16_t	uart_tx_head = 0U;
static volatile uint16_t    uart_tx_tail = 0U;

static char uart_rx_buf[MAX_UART_RX_LEN];
//static volatile uint16_t	uart_rx_state = 0;
static volatile uint16_t	uart_rx_head = 0U;
static volatile uint16_t    uart_rx_tail = 0U;
/* UART handler declaration */

#ifdef UART_DBG

int odmGetStr(char *buf, int maxLen)
{
	int i;
	
	if(uart_rx_tail == uart_rx_head)
		return 0;
	/* enter critical section */
	CLEAR_BIT(hlpuart1.Instance->CR1, USART_CR1_RXNEIE);

	for(i=0; i<maxLen; i++)
	{
		*(buf + i) = uart_rx_buf[uart_rx_head++];
		if(i >= (MAX_UART_RX_LEN -1))
			break;
		if(uart_rx_head >= MAX_UART_RX_LEN)
			uart_rx_head = 0;
		if(uart_rx_head == uart_rx_tail)
		{
			i++;
			break;
		}
	}
    /* leave critical section */
    SET_BIT(hlpuart1.Instance->CR1, USART_CR1_RXNEIE);

	return i;
}

void send_char_to_uart(char ch)
{	
    /* enter critical section */
    CLEAR_BIT(hlpuart1.Instance->CR1, USART_CR1_TXEIE);
    /*check if uart is  busy */
    if(uart_tx_state == UART_TX_ON)
    {
        uart_tx_buf[uart_tx_tail] = ch;
        uart_tx_tail++;
        if (uart_tx_tail >= MAX_UART_BUF_TX)
        {
            if (uart_tx_head == 0)
                uart_tx_tail = MAX_UART_BUF_TX - 1;
            else
                uart_tx_tail = 0;
        }
        else
        {
            /* buffer full */
            if (uart_tx_head == uart_tx_tail)
                uart_tx_tail--;
        }
    }
    else
    {
        /* Send the character */
        uart_tx_state = UART_TX_ON;
        hlpuart1.Instance->TDR = (uint8_t)ch;
        /* Enable the UART Transmit Data Register Empty Interrupt */
        /* If leaving critical section will do the samething, we needn't do it here. */
     }
    /* leave critical section */
    SET_BIT(hlpuart1.Instance->CR1, USART_CR1_TXEIE);
}

void odmPrintf(char *buf)
{
	int i;
	
	osThreadSuspendAll();
	for(i=0; i<MAX_UART_TX_LEN; i++)
	{
		if(*(buf + i) == '\0')
			break;
		send_char_to_uart(*(buf + i));
	}
	osThreadResumeAll();
}

/* for test */
void uartPrintf(char *fmt, ...)
{
    int len=0;
    va_list args;
	char buf[MAX_UART_TX_LEN];

    va_start (args, fmt);
    len = vsprintf (buf, fmt, args);
    if(len)
		odmPrintf(buf);
    va_end (args);
}

void uartPrintfBlock(char *fmt, ...)
{
    int i;
    int len=0;
	char buf[64];
    va_list args;

    va_start (args, fmt);
    len = vsprintf (buf, fmt, args);
    CLEAR_BIT(hlpuart1.Instance->CR1, USART_CR1_TXEIE);
    CLEAR_BIT(hlpuart1.Instance->CR1, USART_CR1_RXNEIE);
    hlpuart1.Instance->TDR = (uint8_t)*(buf + 0);
    for(i=1; i<len; i++)
    {
    	while((READ_REG(hlpuart1.Instance->ISR) & USART_ISR_TXE) == 0);
    	hlpuart1.Instance->TDR = (uint8_t)*(buf + i);
    }
    /*send CR char*/
    /*sci_a_putchar('\r');*/
    /* needn't enable interrupts, we will enter while(1);
    SET_BIT(hlpuart1.Instance->CR1, USART_CR1_TXEIE);
    SET_BIT(hlpuart1.Instance->CR1, USART_CR1_RXNEIE);
    */
    va_end (args);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */

void ODM_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its;
  uint32_t errorflags;
  //uint8_t testrxd;
  trkMsg *pISRMsg;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
  if (errorflags == RESET)
  {
    /* UART in mode Receiver ---------------------------------------------------*/
    if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
      //__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

      uart_rx_buf[uart_rx_tail++] = (uint8_t)(huart->Instance->RDR & 0x0FF);
      if(uart_rx_tail >= MAX_UART_RX_LEN)
        uart_rx_tail = 0;
      if(uart_rx_tail == uart_rx_head)
      {
      	uart_rx_head++; /* remove the oldest char */
        if(uart_rx_head >= MAX_UART_RX_LEN)
          uart_rx_head = 0;
      }

		pISRMsg = gMem32Alloc();
		if(pISRMsg && testQueueHandle) {
			pISRMsg->hdr.id = TEST_MSG_FROM_ISR;
			pISRMsg->hdr.len = 32-sizeof(msg_hdr);
			//pISRMsg->param[0] = (uint8_t)(huart->Instance->RDR & 0x0FF);
			if(osOK != osMessagePut(testQueueHandle, (uint32_t)pISRMsg, osWaitForever))
				gMemFree(pISRMsg);
		}

      return;
    }
  }  

  /* If some errors occur */
  cr3its = READ_REG(huart->Instance->CR3);
if((errorflags != RESET) && ((cr3its & (USART_CR3_EIE | USART_CR1_PEIE)) != RESET))
//if((errorflags != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    }
    
    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if(((isrflags & USART_ISR_ORE) != RESET) &&
       (((cr1its & USART_CR1_RXNEIE) != RESET) || ((cr3its & USART_CR3_EIE) != RESET)))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    }
  } /* End if some error occurs */

  /* UART in mode Transmitter ------------------------------------------------*/
  if(((isrflags & USART_ISR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
  {
        /* check for more chars to send */
        if(uart_tx_tail == uart_tx_head)
        {
            uart_tx_state = UART_TX_OFF;
            uart_tx_head = 0x0U;
            uart_tx_tail = 0x0U;
            CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
        }
        else
        {
            huart->Instance->TDR = ((uint16_t)uart_tx_buf[uart_tx_head]);
            uart_tx_head++;
            if(uart_tx_head >= MAX_UART_BUF_TX)
            	uart_tx_head = 0;
        }
  }
}

void uart_rx_enable(void)
{
	/* enable UART RX/ERR interrupt */
    SET_BIT(hlpuart1.Instance->CR1, USART_CR1_PEIE);
    SET_BIT(hlpuart1.Instance->CR3, USART_CR3_EIE);
    SET_BIT(hlpuart1.Instance->CR1, USART_CR1_RXNEIE);
}

void uart_rx_disable(void)
{
	CLEAR_BIT(hlpuart1.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
	CLEAR_BIT(hlpuart1.Instance->CR3, USART_CR3_EIE);
}
#endif 

