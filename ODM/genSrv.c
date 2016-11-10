//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "msgDef.h"
#include "errDef.h"
#include "genApi.h"
#include "genSrv.h"
#include "fmApi.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <uart.h>

extern osMessageQId pmQueueHandle;
extern QSPI_HandleTypeDef hqspi;

static uint16_t ExtiFlag = 0;

void odmExtiDisableIntr(uint16_t GPIO_Pin)
{
	/* needn't to change SYSCFG->EXTICR, RTSR1, FTSR1 */
	uint32_t temp;

	temp = EXTI->IMR1;
	temp &= ~((uint32_t)GPIO_Pin);
	EXTI->IMR1 = temp;
}

void odmExtiEnableIntr(uint16_t GPIO_Pin)
{
	/* needn't to change SYSCFG->EXTICR, RTSR1, FTSR1 */
	uint32_t temp;

	temp = EXTI->IMR1;
	temp |= (uint32_t)GPIO_Pin;
	EXTI->IMR1 = temp;
}

void odmExtiEn(uint16_t GPIO_Pin)
{
	vPortEnterCritical();
	ExtiFlag |= GPIO_Pin;
	vPortExitCritical();
	odmExtiEnableIntr(GPIO_Pin);

	switch(GPIO_Pin)
	{
		case GPIO_PIN_0:
			HAL_NVIC_EnableIRQ(EXTI0_IRQn);
			break;
		case GPIO_PIN_1:
			HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			break;
		case GPIO_PIN_2:
			HAL_NVIC_EnableIRQ(EXTI2_IRQn);
			break;
		case GPIO_PIN_3:
			HAL_NVIC_EnableIRQ(EXTI3_IRQn);
			break;
		case GPIO_PIN_4:
			HAL_NVIC_EnableIRQ(EXTI4_IRQn);
			break;
		case GPIO_PIN_5:
		case GPIO_PIN_6:
		case GPIO_PIN_7:
		case GPIO_PIN_8:
		case GPIO_PIN_9:
			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
			break;
		case GPIO_PIN_10:
		case GPIO_PIN_11:
		case GPIO_PIN_12:
		case GPIO_PIN_13:
		case GPIO_PIN_14:
		case GPIO_PIN_15:
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
			break;
	}
}

void odmExtiDis(uint16_t GPIO_Pin)
{
	vPortEnterCritical();
	ExtiFlag &= (~GPIO_Pin);
	vPortExitCritical();
	odmExtiDisableIntr(GPIO_Pin);

	switch(GPIO_Pin)
	{
		case GPIO_PIN_0:
			HAL_NVIC_DisableIRQ(EXTI0_IRQn);
			break;
		case GPIO_PIN_1:
			HAL_NVIC_DisableIRQ(EXTI1_IRQn);
			break;
		case GPIO_PIN_2:
			HAL_NVIC_DisableIRQ(EXTI2_IRQn);
			break;
		case GPIO_PIN_3:
			HAL_NVIC_DisableIRQ(EXTI3_IRQn);
			break;
		case GPIO_PIN_4:
			HAL_NVIC_DisableIRQ(EXTI4_IRQn);
			break;
		case GPIO_PIN_5:
		case GPIO_PIN_6:
		case GPIO_PIN_7:
		case GPIO_PIN_8:
		case GPIO_PIN_9:
			if(!(ExtiFlag & 0x03E0))
				HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
			break;
		case GPIO_PIN_10:
		case GPIO_PIN_11:
		case GPIO_PIN_12:
		case GPIO_PIN_13:
		case GPIO_PIN_14:
		case GPIO_PIN_15:
			if(!(ExtiFlag & 0xFC00))
				HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
			break;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	_Bool key;
	trkMsg *pISRMsg;
	
	if(!(ExtiFlag & GPIO_Pin))
		return;

	switch(GPIO_Pin)
	{
		case GYRO_G_INT1_Pin:
				pISRMsg = gMem32Alloc();
				if(pISRMsg && smQueueHandle) 
				{
					pISRMsg->hdr.id = SM_MSG_ACCE_ISR1;
					pISRMsg->hdr.len = 32-sizeof(msg_hdr);
					if(osOK != osMessagePut(smQueueHandle, (uint32_t)pISRMsg, osWaitForever))
					{
						gMemFree(pISRMsg);
						break;	
					}		
				}			
		break;
			
		case PRESSURE_INT_DRDY_Pin:		
				pISRMsg = gMem32Alloc();
				if(pISRMsg && smQueueHandle) 
				{
					pISRMsg->hdr.id = SM_MSG_TEMP_ISR;
					pISRMsg->hdr.len = 32-sizeof(msg_hdr);
					if(osOK != osMessagePut(smQueueHandle, (uint32_t)pISRMsg, osWaitForever))
					{
						gMemFree(pISRMsg);
						break;	
					}		
				}				
		break;
			
		case BLE_INT_Pin:			
		break;
		
		case CHARGE_CHG__Pin:			
		break;
		
		case GYRO_G_INT2_Pin:			
			pISRMsg = gMem32Alloc();
			if(pISRMsg && smQueueHandle) 
			{
				pISRMsg->hdr.id = SM_MSG_ACCE_ISR2;
				pISRMsg->hdr.len = 32-sizeof(msg_hdr);
				if(osOK != osMessagePut(smQueueHandle, (uint32_t)pISRMsg, osWaitForever))
				{
					gMemFree(pISRMsg);
					break;	
				}		
			}			
		break;
			
		case PWR_KEY_Pin:
			if(pmQueueHandle)
			{
				key = HAL_GPIO_ReadPin(PWR_KEY_GPIO_Port, PWR_KEY_Pin);
				if(key)
					osMessagePut(pmQueueHandle, (uint32_t)PM_MSG_KEY_RLS, osWaitForever);
				else
					osMessagePut(pmQueueHandle, (uint32_t)PM_MSG_KEY_PRESS, osWaitForever);
			}
			break;
			
		case WIFI_INT_Pin:		
		break;
		
		case GPS_IRQ_Pin:			
		break;
	}
}

void put_out(unsigned int data,unsigned int *addr)
{
	unsigned char wrD[4]={0}; 
		
	wrD[3]=data>>24;
	wrD[2]=data>>16;
	wrD[1]=data>>8;
	wrD[0]=data;	
	QSPI_Write(&hqspi,*addr,4,wrD);
	*addr+=4;
	//HAL_UART_Transmit(&huart1, wrD, 4, 0xFFFF);	
}

void HardFault_Stack_Dump(unsigned int *hardfault_stack)
{
	unsigned int  sp = ((unsigned long) hardfault_stack);
	unsigned int  r0 = ((unsigned long) hardfault_stack[0]);
	unsigned int  r1 = ((unsigned long) hardfault_stack[1]);
	unsigned int  r2 = ((unsigned long) hardfault_stack[2]);
	unsigned int  r3 = ((unsigned long) hardfault_stack[3]);

	unsigned int  r12 = ((unsigned long) hardfault_stack[4]);
	unsigned int  lr  = ((unsigned long) hardfault_stack[5]);
	unsigned int  pc  = ((unsigned long) hardfault_stack[6]);
	unsigned int  psr = ((unsigned long) hardfault_stack[7]);
	unsigned int  SCB_SHCSR = SCB->SHCSR;
	
	//unsigned char wrD[4]={0};
	unsigned int addr=0x3DF000;

	uint8_t *pData;
	int i=0;
 
	 QSPI_SectorErase(&hqspi,addr);			//4k
	 QSPI_BlockErase(&hqspi,0x3E0000); //64k
	 QSPI_BlockErase(&hqspi,0x3F0000); //64k

	//SARM1 96KB 0x2000_0000 - 0x2001_7FFF
	 pData = (uint8_t*)0x20000000;
	 for (i=0;i<0x18000/256 ;i++)
	 {
		 QSPI_Write(&hqspi,addr,256,pData);
		 addr+=256;
		 pData+=256;
	 }

	 //SARM2 32KB 0x1000_0000 - 0x1000_7FFF
	 pData = (uint8_t*)0x10000000;
	 for (i=0;i<0x8000/256 ;i++)
	 {
		 QSPI_Write(&hqspi,addr,256,pData);
		 addr+=256;
		 pData+=256;
	 }
		put_out(sp,&addr);
		put_out(r0,&addr);
		put_out(r1,&addr);
		put_out(r2,&addr);
		put_out(r3,&addr);
		put_out(r12,&addr);
		put_out(lr,&addr);
		put_out(pc,&addr);
		put_out(psr,&addr);
		put_out(SCB_SHCSR,&addr);

		while(1);
}
#if defined ( __CC_ARM   )
__asm void HardFault_GetStack()
{		
	TST LR, #4		;
	ITE EQ			;	
	MRSEQ r0, msp	;
	MRSNE r0, psp		;
	B __cpp(HardFault_Stack_Dump);
}
#endif

__weak void HAL_UART1_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UART1_RxCpltCallback can be implemented in the corresponding service file.
     */
}

__weak void HAL_UART2_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UART2_RxCpltCallback can be implemented in the corresponding service file.
     */
}

__weak void HAL_UART3_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UART3_RxCpltCallback can be implemented in the corresponding service file.
     */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    extern void HAL_UART1_RxCpltCallback(UART_HandleTypeDef *huart);
    extern void HAL_UART2_RxCpltCallback(UART_HandleTypeDef *huart);
    extern void HAL_UART3_RxCpltCallback(UART_HandleTypeDef *huart);

    if (huart->Instance == USART1)
        HAL_UART1_RxCpltCallback(huart);
    else if (huart->Instance == USART2)
        HAL_UART2_RxCpltCallback(huart);
    else if (huart->Instance == USART3)
        HAL_UART3_RxCpltCallback(huart);
}

__weak void HAL_UART1_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UART1_TxCpltCallback can be implemented in the corresponding service file.
     */
}

__weak void HAL_UART2_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UART2_TxCpltCallback can be implemented in the corresponding service file.
     */
}

__weak void HAL_UART3_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UART3_TxCpltCallback can be implemented in the corresponding service file.
     */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    extern void HAL_UART1_TxCpltCallback(UART_HandleTypeDef *huart);
    extern void HAL_UART2_TxCpltCallback(UART_HandleTypeDef *huart);
    extern void HAL_UART3_TxCpltCallback(UART_HandleTypeDef *huart);

    if (huart->Instance == USART1)
        HAL_UART1_TxCpltCallback(huart);
    else if (huart->Instance == USART2)
        HAL_UART2_TxCpltCallback(huart);
    else if (huart->Instance == USART3)
        HAL_UART3_TxCpltCallback(huart);
}

__weak void HAL_UART1_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UART1_ErrorCallback can be implemented in the corresponding service file.
     */
}

__weak void HAL_UART2_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UART2_ErrorCallback can be implemented in the corresponding service file.
     */
}

__weak void HAL_UART3_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE : This function should not be modified, when the callback is needed,
              the HAL_UART3_ErrorCallback can be implemented in the corresponding service file.
     */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    extern void HAL_UART1_ErrorCallback(UART_HandleTypeDef *huart);
    extern void HAL_UART2_ErrorCallback(UART_HandleTypeDef *huart);
    extern void HAL_UART3_ErrorCallback(UART_HandleTypeDef *huart);

    if (huart->Instance == USART1)
        HAL_UART1_ErrorCallback(huart);
    else if (huart->Instance == USART2)
        HAL_UART2_ErrorCallback(huart);
    else if (huart->Instance == USART3)
        HAL_UART3_ErrorCallback(huart);
}
