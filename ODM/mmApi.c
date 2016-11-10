//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#include "mmApi.h"

#include "stm32l4xx.h"

#include "genApi.h"
#include "uart.h"

extern UART_HandleTypeDef huart2;
/************************************************
* 				ALL gobal API       			*
************************************************/
int mmUartSendData(uint8_t *pData, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef uart_result;
    gLog(_DEBUG_LOG_|_MM_LOG_, "mmUartSend \"%s\"\r\n", pData);
	uart_result = HAL_UART_Transmit(&huart2, pData, size, timeout);
	if(uart_result != HAL_OK)
	{
		gLog(_CRIT_LOG_|_MM_LOG_, "mmUartSend ret = %d\r\n", uart_result);
		return -1;
	}

	return 0;
}

int mmSetPowerState(mmState state, mmSetPowerStateCallback *pFunc, void *ctx)
{
    return 0;
}

int mmRegisterUartReceiveCallback(mmUartReceiveCallback *pFunc, void *ctx)
{
    return 0;
}

int mmRegisterIntCallback(mmIntCallback *pFunc, void *ctx)
{
    return 0;
}