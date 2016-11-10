//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#ifndef CMSRV_H
#define CMSRV_H

#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08006000) /* Base @ of Page 1, 2 Kbytes */
#define ADDR_FLASH_PAGE_2     ((uint32_t)0x08006800) /* Base @ of Page 2, 2 Kbytes */

#define CM_TABLE_UID (char *) "eb8eb403389d499984fd74fde32cdd0a"

#define CM_TABLE_VERSION 0x1

#define CM_UID_LENGTH 32
#define CM_INDEX_LENGTH 4
#define CM_CRC_LENGTH 4
#define CM_VERSION_LENGTH 4
#define CM_SERIAL_LENGTH 16
#define CM_WWAN_LENGTH 6
#define CM_BLE_LENGTH 6
#define CM_WIFI_LENGTH 6
#define CM_GAP_SIZE 2
#define CM_CONSOLE_LENGTH 4
#define CM_LOG_LENGTH 4
#define CM_PAIRING_KEY_LENGTH 8

#define CM_INDEX_INDEX CM_UID_LENGTH
#define CM_VERSION_INDEX (CM_INDEX_INDEX+CM_INDEX_LENGTH+CM_CRC_LENGTH)
#define CM_SERIAL_INDEX (CM_VERSION_INDEX+CM_VERSION_LENGTH)

#define CM_TABLE_TOTAL_SIZE 96
#define CM_PAGE_SIZE 12
#define CM_CRC_BUFFER_DATASIZE 13

#define CM_SWITCH 0x0
#define CM_NOSWITCH 0x1
#define CM_INIT 0x2
#define CM_ALL_RIGHT 0x3

#define RESET_CONSOLE 0xFFFFFFFF
#define RESET_LOG 0xCFFFFFFF

typedef struct{
	char uid[32];
	uint32_t Index;
	uint32_t CRC_number;
	uint32_t version;
}cmHDR;

uint8_t cmChoosePtrByID(void *curPtr, void *noUsePtr2);
uint8_t cmChoosePtrByIndex(void *curPtr, void *noUsePtr2);
void cmInitTable(void *tempPtr, void *curPtr, uint8_t status);
uint32_t cmGetCRCNumber(cmHDR header);
void cmGetData(void *pData, void *data, uint32_t length);
void cmCRCNumberUpdate(cmHDR *ptr, uint32_t CRC_number);
uint32_t cmUpdateTable(void *ptr, uint32_t address);
uint32_t cmCaculateTableCRCNumber(uint32_t data[CM_CRC_BUFFER_DATASIZE]);
uint32_t cmEarseTable(uint32_t address);

#endif
