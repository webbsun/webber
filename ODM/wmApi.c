//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//

#include "wmApi.h"
#include "errDef.h"
#include "uart.h"

int wmSetPowerState(uint8_t state, WifiSetPowerStateCallback *pFunc, void *ctx)
{
	uartPrintf("wmSetPowerState\r\n");

	return E_OK;
}

int wmScanRequest(wifi_scan_config_t config, WifiScanResultCallback *pScanResultCb, WifiScanRequestCallback *pScanRequestCb, void *ctx)
{
	uartPrintf("wmScanRequest\r\n");

	return E_OK;
}
