//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//

#ifndef WMAPI_H
#define WMAPI_H

#include "wmSrv.h"
#include "cmsis_os.h"
#include "errDef.h"

#define MAX_CHANNEL_NUM 14
#define MAX_SCAN_SSIDS 16

typedef struct {
	int scanInterval;
	int numChannels;
	int channelList[MAX_CHANNEL_NUM];
} wifi_scan_config_t;

typedef struct {
	int numSsids;
	uint8_t bssids[MAX_SCAN_SSIDS][6];
	uint8_t snr[MAX_SCAN_SSIDS];
	uint8_t channel[MAX_SCAN_SSIDS];
} wifi_scan_result_t;

typedef void (*WifiSetPowerStateCallback)(uint8_t status, void *ctx);
typedef void (*WifiScanResultCallback)(wifi_scan_result_t result, void *ctx);
typedef void (*WifiScanRequestCallback)(uint8_t status, void *ctx);

int wmSetPowerState(uint8_t state, WifiSetPowerStateCallback *pFunc, void *ctx);
int wmScanRequest(wifi_scan_config_t config, WifiScanResultCallback *pScanResultCb, WifiScanRequestCallback *pScanRequestCb, void *ctx);

#endif
