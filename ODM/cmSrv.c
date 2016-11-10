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
#include "cmSrv.h"
#include "uart.h"
#include "errdef.h"

#include <string.h>

static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
static FLASH_EraseInitTypeDef EraseInitStruct;
extern CRC_HandleTypeDef hcrc;

uint8_t cmChoosePtrByID(void *curPtr, void *noUsePtr2){
	//int i;
	cmHDR *ptr1 = curPtr;
	cmHDR *ptr2 = noUsePtr2;

	int cmp1 = memcmp(ptr1->uid, CM_TABLE_UID, sizeof(ptr1->uid));
	int cmp2 = memcmp(ptr2->uid, CM_TABLE_UID, sizeof(ptr2->uid));

	if(cmp1==0 && cmp2!=0){
		return CM_NOSWITCH;
	}else if(cmp1!=0 && cmp2==0){
		return CM_SWITCH;
	}else if(cmp1!=0 && cmp2!=0){
		return CM_INIT;
	}else{
		return CM_ALL_RIGHT;
	}
//	for(i = 0; i < 32; i++){
//		if((ptr2->uid[i] != (*(CM_TABLE_UID+i))) && (ptr1->uid[i] != (*(CM_TABLE_UID+i)))){
//			return CM_INIT;
//		}else if((ptr2->uid[i] != (*(CM_TABLE_UID+i))) && (ptr1->uid[i] == (*(CM_TABLE_UID+i)))){
//			if(flag == CM_SWITCH)
//				return CM_INIT;
//			flag = CM_NOSWITCH;
//		}else if((ptr2->uid[i] == (*(CM_TABLE_UID+i))) && (ptr1->uid[i] != (*(CM_TABLE_UID+i)))){
//			if(flag == CM_NOSWITCH)
//				return CM_INIT;
//			flag = CM_SWITCH;
//		}else{
//			continue;
//		}
//	}
//	if(flag == CM_ALL_RIGHT){
//		if(ptr1->Index >= ptr2->Index){
//			return CM_NOSWITCH;
//		}else{
//			return CM_SWITCH;
//		}
//	}else{
//		return flag;
//	}

}

uint8_t cmChoosePtrByIndex(void *curPtr, void *noUsePtr2){
	cmHDR *ptr1 = curPtr;
	cmHDR *ptr2 = noUsePtr2;
	if(ptr1->Index >= ptr2->Index){
		return CM_NOSWITCH;
	}else{
		return CM_SWITCH;
	}
}

void cmInitTable(void *tempPtr, void *curPtr, uint8_t status){
	uint32_t *ptr = tempPtr;
	cmHDR *table = tempPtr;

	int i= 0;
	for(i=0; i<CM_UID_LENGTH; i++){
		table->uid[i] = *(CM_TABLE_UID+i);
	}
	table->version = CM_TABLE_VERSION;
	if(status == CM_INIT){
		table->Index = 0;
		for(i=CM_SERIAL_INDEX; i<CM_TABLE_TOTAL_SIZE; i++){
			*(ptr+i) = 0;
		}
	}else{
		uint32_t *dataPtr = curPtr;
		uint32_t *header = dataPtr + (CM_INDEX_INDEX / 4);
		table->Index = (*header) + 1;
		uint32_t *data = curPtr;
		for(i=(CM_SERIAL_INDEX/4); i<(CM_TABLE_TOTAL_SIZE/4); i++){
			*(ptr+i) = *(data+i);
		}
	}
}

uint32_t cmGetCRCNumber(cmHDR header){
	return header.CRC_number;
}

uint32_t cmCaculateTableCRCNumber(uint32_t data[CM_CRC_BUFFER_DATASIZE]){
	return HAL_CRC_Calculate(&hcrc, (uint32_t *)data, CM_CRC_BUFFER_DATASIZE);
}

void cmGetData(void *pData, void *data, uint32_t length){
	uint32_t *ptr = pData;
	uint32_t *tablePtr = data;
	int i;
	for(i=0; i<length; i++){
		*(ptr+i) = *(tablePtr+i);
	}
}

void cmCRCNumberUpdate(cmHDR *ptr, uint32_t CRC_number){
	ptr->CRC_number = CRC_number;
}

uint32_t cmUpdateTable(void *ptr, uint32_t address){
	uint32_t Address = 0;
	Address = address;
	int i=0;
	uint64_t data[CM_PAGE_SIZE];
	uint64_t *info = ptr;
	for(i=0; i<CM_PAGE_SIZE; i++){
		data[i] = *(info+i);
	}
	i = 0;
	while (Address <  address + CM_TABLE_TOTAL_SIZE)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, data[i]) == HAL_OK)
		{
		  Address = Address + 8;
		}
		else
		{
			return E_MEM;
		}
		i++;
	}
	HAL_FLASH_Lock();
	return E_OK;
}

uint32_t cmEarseTable(uint32_t address){
	HAL_FLASH_Unlock();
	uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
	uint32_t PAGEError = 0;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	/* Get the 1st page to erase */
	FirstPage = GetPage(address);
	/* Get the number of pages to erase from 1st page */
	NbOfPages = 1;
	/* Get the bank */
	BankNumber = GetBank(address);
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		return E_MEM;
	}

	return E_OK;
}

static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}
