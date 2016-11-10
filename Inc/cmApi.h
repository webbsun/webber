//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#ifndef CMAPI_H
#define CMAPI_H
#include "cmSrv.h"

#define CM_SERIAL ((uint8_t)0x3)
#define CM_WWAN ((uint8_t)0x4)
#define CM_BLE ((uint8_t)0x5)
#define CM_WIFI ((uint8_t)0x6)
#define CM_CONSOLE ((uint8_t)0x7)
#define CM_LOG ((uint8_t)0x8)
#define CM_PAIRING_KEY ((uint8_t)0x9)

typedef struct{
	cmHDR		header;			//	header of struct - 44 Bytes

	uint8_t		serial[16];		//	serial number of factory - 16 Bytes

	uint8_t		wWanMAC[6];		//	wWAN MAC address - 6 Bytes

	uint8_t		BLEMAC[6];		//	BLE MAC address - 6 Bytes

	uint8_t		WifiMAC[6];		//	Wifi MAC address - 6 Bytes

	uint32_t	debugConMask;	//	console mask - 4 Bytes

	uint32_t	debugLogMask;	//	log mask - 4 Bytes

	uint64_t	pairingKey;		//	key of pair link - 8 Bytes
}cmPtr;

extern cmPtr *cm;

void cm_Init(void);
int cm_CRC_Check(cmPtr *ptr);
int cm_Earse_Data(uint32_t address);
int cmRead(uint32_t id, void *pData, uint32_t *length);
int cmWrite(uint32_t id, void *pData, uint32_t length);
int cm_Reset_to_Defalut(void);
int cm_Program(void);

#endif
