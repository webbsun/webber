//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//

#ifndef __MMAPI_H
#define __MMAPI_H
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/**
* @brief  global variable
*/
typedef enum
{
  MM_RESET        = 0,
  MM_SET          = 1
} mmState;

typedef void (*mmSetPowerStateCallback) (void *ctx, mmState state);
typedef void (*mmUartReceiveCallback)(void *ctx, uint8_t *pData, uint16_t size);
typedef void (*mmIntCallback)(void *ctx, mmState state);
/* Exported functions ------------------------------------------------------- */


/* Exported functions for Netgear API ------------------------------------------------------- */
extern int mmUartSendData(uint8_t *pData, uint16_t size, uint32_t timeout);
extern int mmSetPowerState(mmState state, mmSetPowerStateCallback *pFunc, void *ctx);
extern int mmRegisterUartReceiveCallback(mmUartReceiveCallback *pFunc, void *ctx);
extern int mmRegisterIntCallback(mmIntCallback *pFunc, void *ctx);
#endif

