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
#include "fatfs.h"
#include "genApi.h"
#include "genSrv.h"
#include "memDef.h"
#include "errDef.h"
#include "fmApi.h"
#include "cmApi.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <uart.h>

uint8_t ucHeap[ configTOTAL_HEAP_SIZE ] __attribute__ ((section(".ccm")));

uint8_t iCode[16] __attribute__ ((section(".image_uuid"))) = {0x29, 0xAA, 0xF5, 0xA8, 0x12, 0x35, 0x44, 0xD4, 
	0x84, 0xE8, 0x00, 0xB5, 0x1A, 0xB6, 0x30, 0xAE};
uint32_t iCRC  __attribute__ ((section(".image_crc"))) = 0xADDEADDE;
uint32_t iLen  __attribute__ ((section(".image_len"))) = 0xFFFFFFFF;
uint32_t majVer __attribute__ ((section(".image_maj"))) = 0;
uint32_t minVer __attribute__ ((section(".image_min"))) = 2;

osMutexId Mutex_I2C = NULL;
	/* Memory pool id */
static osPoolId odmMem32byteId = NULL, odmMem64byteId = NULL;

extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern FATFS SPIFatFs;

extern uint8_t logSuppress;
/**
* @brief Show log on console or store it into SPI Flash
* @param  mPlc       Police of message
* @param  pMsgData   Message pointer
* @retval  status code that indicates the execution status of the function.
* @note   The maximum length of message is 32 bytes 
*/
int gLog(uint32_t mPlc, char *pMsgData, ...) 
{
	va_list argptr;
	int cnt;
	char gLogBuf[MAX_LOG_LEN];
	
	va_start(argptr, pMsgData); 
	cnt = vsprintf(gLogBuf, pMsgData, argptr);
	va_end(argptr);

	if(cnt > MAX_LOG_LEN)
		return E_LEN;
	
	//HAL_UART_Transmit(&hlpuart1, (uint8_t *)gLogBuf, strlen(gLogBuf), HAL_MAX_DELAY);
	if(!logSuppress && (mPlc == (mPlc & cm->debugConMask)))
		odmPrintf(gLogBuf);

	return E_OK;
}

/**
* @brief Return MCU software version
* @param  pMaj   Major version
* @param  pMin   Minor version
* @retval  status code that indicates the execution status of the function.
* @note   Can't assign NULL pointer to pMaj or pMin 
*/
int gSwVer(uint32_t *pMaj, uint32_t *pMin)
{
	*pMaj = majVer;
	*pMin = minVer;

	return E_OK;
}

/**
* @brief	Return 32 bytes memory pointer
* @param	N/A
* @retval	32 bytes memory pointer from memory pool
* @note		Return NULL if no valid memroy  
*/
void *gMem32Alloc(void)
{
	mem_hdr *pHdr;

	if(odmMem32byteId == NULL)
		return NULL;
	
	pHdr = osPoolCAlloc(odmMem32byteId);
	if(pHdr) {
		pHdr->magic = 0xDE3232AD;
		return pHdr+1;
	}
	gLog(_WARN_LOG_|_GEN_LOG_, "Mem32Pool empty!\r\n");
	return NULL;
}

/**
* @brief	Return 64 bytes memory pointer
* @param	N/A
* @retval	64 bytes memory pointer from memory pool
* @note		Return NULL if no valid memroy  
*/
void *gMem64Alloc(void)
{
	mem_hdr *pHdr;

	if(odmMem64byteId == NULL)
		return NULL;
	
	pHdr = osPoolCAlloc(odmMem64byteId);
	if(pHdr) {
		pHdr->magic = 0xDE6464AD;
		return pHdr+1;
	}
	gLog(_WARN_LOG_|_GEN_LOG_, "Mem64Pool empty!\r\n");
	return NULL;
}

/**
* @brief	Free memory to pool
* @param  	pBuf	memory pointer of memory pool
* @retval	status code that indicates the execution status of the function.
* @note		N/A
*/
int gMemFree(void *pBuf)
{
	osStatus ret = osErrorParameter;
	mem_hdr *pHdr = pBuf;

	if(NULL == pHdr)
		return E_ILLEGAL;

	pHdr--;

	switch(pHdr->magic)
	{
		case 0xDE3232AD:
			ret = osPoolFree(odmMem32byteId, pHdr);
			break;
		case 0xDE6464AD:
			ret = osPoolFree(odmMem64byteId, pHdr);
			break;
		default:
			gLog(_CRIT_LOG_|_GEN_LOG_, "gMemFree: avoid magic code\r\n");
	}
	if(ret == osOK)
		return E_OK;
	else
		return E_ILLEGAL;
}

/**
* @brief Control blue/amber LED
* @param blue		Blue LED control
* @param amber	Amber LED control
**/
int ledCtrl(_Bool blue, _Bool amber){
	TIM_OC_InitTypeDef sConfigOC;

	gLog(_DEBUG_LOG_|_GEN_LOG_, "%s begin\r\n", __func__);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	if(blue){
		sConfigOC.Pulse = 500;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		{
			gLog(_WARN_LOG_|_GEN_LOG_, "PWM1_2 Conf");
		}

		if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
		{
			gLog(_WARN_LOG_|_GEN_LOG_, "PWM1_2 Start");
		}
	}else{
		if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2) != HAL_OK)
		{
			gLog(_WARN_LOG_|_GEN_LOG_, "PWM1_2 Stop");
		}
	}
	
	if(amber){
		sConfigOC.Pulse = 500;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
		{
			gLog(_WARN_LOG_|_GEN_LOG_, "PWM1_3 Conf");
		}

		if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
		{
			gLog(_WARN_LOG_|_GEN_LOG_, "PWM1_3 Start");
		}
	}else{
		if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3) != HAL_OK)
		{
			gLog(_WARN_LOG_|_GEN_LOG_, "PWM1_3 Stop");
		}
	}

	return E_OK;
}

/**
* @brief Control Buzzer
* @param opcode		Buzzer control
**/
int bzCtrl(_Bool opcode){
	TIM_OC_InitTypeDef sConfigOC;

	gLog(_DEBUG_LOG_|_GEN_LOG_, "%s begin\r\n", __func__);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	if(opcode){
		sConfigOC.Pulse = 500;
		if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		{
			gLog(_WARN_LOG_|_GEN_LOG_, "PWM3_2 Conf");
			return E_HAL;
		}

		if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
		{
			gLog(_WARN_LOG_|_GEN_LOG_, "PWM3_2 Start");
			return E_HAL;
		}
	}else{
		if (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2) != HAL_OK)
		{
			gLog(_WARN_LOG_|_GEN_LOG_, "PWM3_2 Stop");
			return E_HAL;
		}
	}
	return E_OK;
}

/**
* @brief get debugConMask and debugLogMask from DCT
* @param  	conMask	debug console mask
* @param  	logMask debug log mask
* @retval	status	code that indicates the execution status of the function.
* @note  N/A
*/
int gLogGetMask(uint32_t *conMask, uint32_t *logMask)
{
	gLog(_DEBUG_LOG_|_GEN_LOG_, "%s begin\r\n", __func__);
	if(NULL == conMask || NULL == logMask || NULL == cm)
		return E_ILLEGAL;

	*conMask = cm->debugConMask;
	*logMask = cm->debugLogMask;
	gLog(_DEBUG_LOG_|_GEN_LOG_, "conMask: 0x%08X\r\n", *conMask);
	gLog(_DEBUG_LOG_|_GEN_LOG_, "logMask: 0x%08X\r\n", *logMask);

	return E_OK;
}

/**
* @brief set debugConMask and debugLogMask to DCT
* @param  	conMask	debug console mask
* @param  	logMask debug log mask
* @retval	status	code that indicates the execution status of the function.
* @note  N/A
*/
int gLogSetMask(uint32_t conMask, uint32_t logMask)
{
	gLog(_DEBUG_LOG_|_GEN_LOG_, "%s begin\r\n", __func__);
	if((E_OK != cmWrite(CM_CONSOLE, (void *)&conMask, sizeof(conMask))) ||
			(E_OK != cmWrite(CM_LOG, (void *)&logMask, sizeof(logMask))) ||
			(E_OK != cm_Program())) {
		gLog(_DEBUG_LOG_|_GEN_LOG_, "%s failed\r\n", __func__);
		return E_HAL;
	}

	return E_OK;
}

/**
* @brief verify image store in file system is valid or not
* @param  	path	file name and path of image
* @retval	status	code that indicates the execution status of the function.
* @note  N/A
*/
int gFwVerify(char *path)
{
	gLog(_DEBUG_LOG_|_GEN_LOG_, "%s begin\r\n", __func__);
	if(NULL == path)
		return E_FS;
	gLog(_DEBUG_LOG_|_GEN_LOG_, "%s end\r\n", __func__);
	return E_ILLEGAL;
}

/**
* @brief ODM entry to create resources of FreeRTOS
* @param  N/A
* @retval  N/A
* @note  N/A
*/
void odmEntry(void)
{
	odmExtiDisableIntr(GPIO_PIN_All);
	odmExtiEn(PWR_KEY_Pin); /* must be moved to pmThread when it is ready. */

	cm_Init();

	/* Pega create tasks, queues and resources of FreeRTOS at this function */
	gLog(_CRIT_LOG_|_GEN_LOG_, "\r\nPet Tracker BSP Ver:%02d.%02d\r\n", majVer, minVer);

	/* Create 32 and 64 bytes memory pool */
	//osPoolDef(odmMem32byte, 32, 32);
	const osPoolDef_t os_pool_def_odmMem32byte = { 32, 32+sizeof(mem_hdr), NULL };
	odmMem32byteId = osPoolCreate(osPool(odmMem32byte));
	const osPoolDef_t os_pool_def_odmMem64byte = { 16, 64+sizeof(mem_hdr), NULL };
	odmMem64byteId = osPoolCreate(osPool(odmMem64byte));
	
	/*enable SPI FlASH into Quad mode */
	if(fmQuadEnable()!= 0)
	{
		gLog(_CRIT_LOG_|_FM_LOG_, "QuadEnable Fail\r\n");
	}
	
	/* Probe driver to USER_Path "0:/ */
	FATFS_LinkDriver(&USER_Driver, USER_Path);
	if(f_mount(&SPIFatFs, USER_Path, 0) != FR_OK)
	{
		gLog(_CRIT_LOG_|_GEN_LOG_, "Mount FATFS Failed\r\n");
	}
	
	
	osMutexDef(osMutexi2c);
  Mutex_I2C = osMutexCreate(osMutex(osMutexi2c));
	
	osMutexDef(osMutexqspi);
  Mutex_QSPI = osMutexCreate(osMutex(osMutexqspi));
}

