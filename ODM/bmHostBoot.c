/******************************************************************************
    Copyright (C) Cambridge Silicon Radio Ltd 2011

FILE
    csr1k_host_boot.c

DESCRIPTION
    This file contains example code to boot CSR1000 or CSR1001 over the
    debug SPI port.

    This code has been developed for an NXP LPC210x processor and run on
    an Olimex LPC-P2106-B low cost development board.

    The tool chain is WINARM and used the "Programmers Notepad 2" IDE
    although a makefile build should be possible.

DISCLAIMER
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 
    It is only for use with Cambridge Silicon Radio's CSR1000/1001 parts.
 
    It must not be disclosed to any 3rd parties without the express written
    permission of CSR.

******************************************************************************/


#include "bmHostBoot.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "genApi.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <uart.h>

/* We need to "borrow" the SPI clock pin for wiggling by hand */
#define PINSEL0_SCK_MASK            0x00000300
#define PINSEL0_SCK_SPI             0x00000100
#define PINSEL0_SCK_PIO             0x00000000

#define PINSEL0_SPI_MOSI_MISO       0x00001400
/* These are done automatically when we set the relevant bits in PCB_PINSEL0 */
#define SET_SPI_MOSI_AS_OUTPUT()    ((void) 0)
#define SET_SPI_MISO_AS_INPUT()     ((void) 0)

/* register bit definitions */
#define SPI_SPSR_SPIF               (1 << 7)


static void CSRspiTXword(uint16 addr, uint16 tx_data16);
static void CSRspiTXblock(uint16 addr, uint16 *ptx_data16, uint16 length);
static void spiStartTransaction(uint8 r_or_w, uint16 addr);
static void spiClock2Cyles(void);
static void spiStopTransaction(void);
static void spiTXword(uint16 tx_data16);
static void spiTXbyte(uint8 tx_data8);
static void SET_SPI_CSB_1(void);
static void SET_SPI_CSB_0(void);

/*--------------------------------------------------------------------------*/
/* CSR SPI LPC210x initialisation code lives here                           */
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/* CSR1000/1001 boot code is here  lives here                               */
/*--------------------------------------------------------------------------*/

/* Include auto generated header file of constants, approx 50 kb as 25 kw   */
#define UWORD16 uint16
#include "bmImg2hCode.h"

SPI_HandleTypeDef hspi1;


void SPI_CLK_SET_GPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void SPI_CLK_SET_CLK(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void CSR1Kboot(void)
{
	uint16 i;
	uint16 addr;
	uint16 len;
	uint16 *pbuf;

	int XAP_CODE_STRUCT_SIZE=sizeof(xap_code_struct);

	gLog(_INFO_LOG_|_BM_LOG_, "CSR1Kboot : Start ");
	gLog(_INFO_LOG_|_BM_LOG_, "XAP_CODE_STRUCT_SIZE = %d\r\n",XAP_CODE_STRUCT_SIZE);

	/* Reset the chip.  No need to clear this later, it is self clearing */
	CSRspiTXword(0xF82F, 1);


	/* wait for the interal boot hardware in CSR1000 to give up trying to find
	 * an image in external EEPROM or SPI Flash to download (that takes ~11ms).
	 * While the chip is checking for memory we can't boot the chip over SPI.
	 */
	// timeDelayInMS(15);
	osDelay(15);

	/* STOP the processor */
	CSRspiTXword(0xF81D, 2);

	HAL_Delay(2);

	for (i = 0; i < XAP_CODE_STRUCT_SIZE/2; i += len)
	{
		addr = xap_code_struct[i++];
		len  = xap_code_struct[i++];
		pbuf = (uint16 *) &xap_code_struct[i];

		CSRspiTXblock(addr, pbuf, len);
	}
	HAL_Delay(2);

	// Magic toggle
	CSRspiTXword(0x0018, 1);
	CSRspiTXword(0x0018, 0);

	// Set PC to zero
	CSRspiTXword(0xFFEA, 0);
	CSRspiTXword(0xFFE9, 0);

	// And GO (which is really an "un-stop" :)
	CSRspiTXword(0xF81D, 0);

	gLog(_INFO_LOG_|_BM_LOG_, "CSR1Kboot : End \r\n");
    //De-Initialization SPI
    HAL_SPI_MspDeInit(&hspi1);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


/*--------------------------------------------------------------------------*/
/* Basic CSR SPI primitives live here                                       */
/*--------------------------------------------------------------------------*/

/* CSR SPI write command.  Completely proprietary */
#define CSR_SPI_CMD_WRITE           0x02

static void CSRspiTXword(uint16 addr, uint16 tx_data16)
{

    spiStartTransaction(CSR_SPI_CMD_WRITE, addr);
    spiTXword(tx_data16);
    spiStopTransaction();
}

static void spiStopTransaction(void)
{
   // timeDelayInUS(2);

    SET_SPI_CSB_1();    /* VITAL - end of "official" SPI transaction */

   // timeDelayInUS(2);

    spiClock2Cyles();   /* so called "clocking out */

    //timeDelayInUS(2);
}


/*--------------------------------------------------------------------------*/
/* CSR SPI helper functions live here                                       */
/*--------------------------------------------------------------------------*/

static void spiStartTransaction(uint8 r_or_w, uint16 addr)
{
	/* Note:  It is assummed that CSB is already high */

	spiClock2Cyles();       /* so called "clocking in" */

	//timeDelayInUS(2);

	SET_SPI_CSB_0();

	// timeDelayInUS(2);

	spiTXbyte(r_or_w);

	spiTXword(addr);
}

static void CSRspiTXblock(uint16 addr, uint16 *ptx_data16, uint16 length)
{
    spiStartTransaction(CSR_SPI_CMD_WRITE, addr);

	while (length--)
	{
		spiTXword(*ptx_data16++);
	}

    spiStopTransaction();
}

static void spiClock2Cyles(void)
{
	SPI_CLK_SET_GPIO();
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
	{__IO uint32_t i;i=0xF;while(i--);}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
	{__IO uint32_t i;i=0xF;while(i--);}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
	{__IO uint32_t i;i=0xF;while(i--);}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET);
	{__IO uint32_t i;i=0xF;while(i--);}
	SPI_CLK_SET_CLK();
}


static void SET_SPI_CSB_0(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET);
}

static void SET_SPI_CSB_1(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_SET);
}
/* the LPC210x doesn't support 16 bit SPI so do two 8 bit ones back to back */
static void spiTXword(uint16 tx_data16)
{
    spiTXbyte((uint8)(0xff & (tx_data16 >> 8)));
    spiTXbyte((uint8)(0xff &  tx_data16));
}

/* Lowest level SPI primitive. All the TX bits go through this function */
static void spiTXbyte(uint8 tx_data8)
{
	uint8_t pTxData[]={0};
	uint8_t pRxData[]={0};
	//int val;
	pTxData[0]=tx_data8;

	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)pTxData,(uint8_t *)pRxData,1,100);
	//val=HAL_SPI_TransmitReceive(&hspi1,tx_data8,(uint8_t *)pRxData,1,100);

	//printf("vale %d == pTxData 0x%x \r\n",val,pTxData[0]);
}


/* csr1k_host_boot.c */
