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
#include "cmApi.h"
#include "errdef.h"
#include <string.h>

cmPtr *cm = NULL;
static cmPtr *curPtr = (cmPtr *)ADDR_FLASH_PAGE_1;
static cmPtr *noUsePtr = (cmPtr *)ADDR_FLASH_PAGE_2;
static cmPtr tempPtr;

void cm_Init(void){
	uint8_t status;
	status = cmChoosePtrByID(curPtr, noUsePtr);
	if(status == CM_SWITCH){
		if(cm_CRC_Check(noUsePtr) == E_OK){
			curPtr = (cmPtr *)ADDR_FLASH_PAGE_2;
			noUsePtr = (cmPtr *)ADDR_FLASH_PAGE_1;
		}else{
			status = CM_INIT;
		}
	}else if(status == CM_NOSWITCH){
		if(cm_CRC_Check(curPtr) != E_OK){
			status = CM_INIT;
		}
	}else if(status == CM_ALL_RIGHT){
		status = cmChoosePtrByIndex(curPtr, noUsePtr);
		if(status == CM_SWITCH){
			if(cm_CRC_Check(noUsePtr) == E_OK){
				curPtr = (cmPtr *)ADDR_FLASH_PAGE_2;
				noUsePtr = (cmPtr *)ADDR_FLASH_PAGE_1;
			}else{
				if(cm_CRC_Check(curPtr) != E_OK){
					status = CM_INIT;
				}
			}
		}else{
			if(cm_CRC_Check(curPtr) == E_OK);
			else{
				if(cm_CRC_Check(noUsePtr) == E_OK){
					curPtr = (cmPtr *)ADDR_FLASH_PAGE_2;
					noUsePtr = (cmPtr *)ADDR_FLASH_PAGE_1;
				}else{
					status = CM_INIT;
				}
			}
		}
	}else;

	cmInitTable(&tempPtr, curPtr, status);
	if(status == CM_INIT){
		cm_Reset_to_Defalut();
		cm = &tempPtr;
	}else{
		cm = curPtr;
	}
}

int cm_CRC_Check(cmPtr *ptr){
	uint32_t cmCRCData[CM_CRC_BUFFER_DATASIZE] = {0};
	uint32_t CRC_number = 0;
	CRC_number = cmGetCRCNumber(ptr->header);
	cmGetData(cmCRCData, ptr->serial, CM_CRC_BUFFER_DATASIZE);
	__IO uint32_t uwCRCValue = 0;

	uwCRCValue = cmCaculateTableCRCNumber(cmCRCData);

	if(CRC_number == uwCRCValue){
		return E_OK;
	}else{
		return -1;
	}
}

int cm_Earse_Data(uint32_t address){
	if(cmEarseTable(address) == E_OK)
		return E_OK;
	else
		return E_MEM;
}

int cmRead(uint32_t id, void *pData, uint32_t *length){
	if( id == CM_SERIAL){
		uint8_t *ptr = pData;
		memcpy(ptr, curPtr->serial, CM_SERIAL_LENGTH);
//		for(i=0; i<CM_SERIAL_LENGTH; i++){
//			*(ptr + i) = curPtr->serial[i];
//		}
		(*length) = CM_SERIAL_LENGTH;
	}else if( id == CM_WWAN){
		uint8_t *ptr = pData;
		memcpy(ptr, curPtr->wWanMAC, CM_WWAN_LENGTH);
//		for(i=0; i<CM_WWAN_LENGTH; i++){
//			*(ptr + i) = curPtr->wWanMAC[i];
//		}
		(*length) = CM_WWAN_LENGTH;
	}else if( id == CM_BLE){
		uint8_t *ptr = pData;
		memcpy(ptr, curPtr->BLEMAC, CM_BLE_LENGTH);
//		for(i=0; i<CM_BLE_LENGTH; i++){
//			*(ptr + i) = curPtr->BLEMAC[i];
//		}
		(*length) = CM_BLE_LENGTH;
	}else if( id == CM_WIFI){
		uint8_t *ptr = pData;
		memcpy(ptr, curPtr->WifiMAC, CM_WIFI_LENGTH);
//		for(i=0; i<CM_WIFI_LENGTH; i++){
//			*(ptr + i) = curPtr->WifiMAC[i];
//		}
		(*length) = CM_WIFI_LENGTH;
	}else if( id == CM_CONSOLE){
		uint32_t *ptr = pData;
		(*ptr) = curPtr->debugConMask;
		(*length) = CM_CONSOLE_LENGTH;
	}else if( id == CM_LOG){
		uint32_t *ptr = pData;
		(*ptr) = curPtr->debugLogMask;
		(*length) = CM_LOG_LENGTH;
	}else if( id == CM_PAIRING_KEY){
		uint64_t *ptr = pData;
		(*ptr) = curPtr->pairingKey;
		(*length) = CM_PAIRING_KEY_LENGTH;
	}else{
		return E_ID;
	}
	return E_OK;
}

int cmWrite(uint32_t id, void *pData, uint32_t length){
	//int i;
	if( id == CM_SERIAL){
		uint8_t *ptr = pData;
		memcpy(tempPtr.serial, ptr, length);
//		for(i=0; i<length; i++){
//			tempPtr.serial[i] = *(ptr + i);
//		}
	}else if( id == CM_WWAN){
		uint8_t *ptr = pData;
		memcpy(tempPtr.wWanMAC, ptr, length);
//		for(i=0; i<length; i++){
//			tempPtr.wWanMAC[i] = *(ptr + i);
//		}
	}else if( id == CM_BLE){
		uint8_t *ptr = pData;
		memcpy(tempPtr.BLEMAC, ptr, length);
//		for(i=0; i<length; i++){
//			tempPtr.BLEMAC[i] = *(ptr + i);
//		}
	}else if( id == CM_WIFI){
		uint8_t *ptr = pData;
		memcpy(tempPtr.WifiMAC, ptr, length);
//		for(i=0; i<length; i++){
//			tempPtr.WifiMAC[i] = *(ptr + i);
//		}
	}else if( id == CM_CONSOLE){
		uint32_t *ptr = pData;
		tempPtr.debugConMask = (*ptr);
	}else if( id == CM_LOG){
		uint32_t *ptr = pData;
		tempPtr.debugLogMask = (*ptr);
	}else if( id == CM_PAIRING_KEY){
		uint64_t *ptr = pData;
		tempPtr.pairingKey = (*ptr);
	}else{
		return E_ID;
	}
	return E_OK;
}

int cm_Reset_to_Defalut(void){
	tempPtr.debugConMask = RESET_CONSOLE;
	tempPtr.debugLogMask = RESET_LOG;
	return E_OK;
}

int cm_Program(void){
	if( cmEarseTable((uint32_t)noUsePtr) != E_OK){
		return E_MEM;
	}
	uint32_t cmCRCData[CM_CRC_BUFFER_DATASIZE] = {0};
	uint32_t CRC_number = 0;
	cmGetData(cmCRCData, tempPtr.serial, CM_CRC_BUFFER_DATASIZE);
	CRC_number = cmCaculateTableCRCNumber(cmCRCData);

	cmCRCNumberUpdate(&tempPtr.header, CRC_number);
	if(cmUpdateTable(&tempPtr, (uint32_t)noUsePtr) != E_OK){
		return E_MEM;
	}

	if(curPtr == (cmPtr *)ADDR_FLASH_PAGE_1){
		curPtr = (cmPtr *)ADDR_FLASH_PAGE_2;
		noUsePtr = (cmPtr *)ADDR_FLASH_PAGE_1;
	}else{
		curPtr = (cmPtr *)ADDR_FLASH_PAGE_1;
		noUsePtr = (cmPtr *)ADDR_FLASH_PAGE_2;
	}
	cm = curPtr;
	return E_OK;
}
