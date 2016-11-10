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
#include "msgDef.h"
#include "pmApi.h"
#include "pmSrv.h"
#include "genApi.h"
#include "genSrv.h"
#include "uart.h"

#define MAX_PWR_POLL_TIME (60 * 1000) /* ms */

osMessageQId pmQueueHandle = NULL;
osTimerId pmBattTimerHandle;
osTimerId pmKeyTimerHandle;

/* callback functioin parameter. */
static void (*pPwrKeyCallback)(void *ctx, _Bool key);
static void (*pBatteryCallback)(void *ctx, uint8_t status);
static void *ctxKey;
static void *ctxBatt;

static void CallbackPmBattTm(void const * argument);
static void CallbackPmKeyTm(void const * argument);
static void pmUpdateBattInfo(uint8_t *batt_st, uint8_t *batt_cap);
static uint8_t  batt_status;
static uint8_t  batt_rsoc;
static uint16_t batt_volt;
static int16_t  batt_temp;
static uint32_t  rcc_csr = 0x0;

/* task */
void StartTaskPm(void const * argument)
{
	osEvent getRet;
	uint8_t  batt_status_new;
    _Bool pwr_key;

	/* init */
    svPmGetMcuRccCsr(&rcc_csr);
    osMessageQDef(pmQueue, 8, uint32_t);
    pmQueueHandle = osMessageCreate(osMessageQ(pmQueue), NULL);
    osTimerDef(pmBattTimer, CallbackPmBattTm);
    pmBattTimerHandle = osTimerCreate(osTimer(pmBattTimer), osTimerOnce, NULL);
    osTimerDef(pmKeyTimer, CallbackPmKeyTm);
    pmKeyTimerHandle = osTimerCreate(osTimer(pmKeyTimer), osTimerOnce, NULL);
    pwr_key = GPIO_PIN_SET;
    pmUpdateBattInfo(&batt_status, &batt_rsoc);
    odmExtiEn(PWR_KEY_Pin);
    osTimerStart (pmBattTimerHandle, MAX_PWR_POLL_TIME);

	/* Infinite loop */
	for(;;)
	{
		getRet = osMessageGet(pmQueueHandle, osWaitForever);
        switch(getRet.value.v)
        {
            case PM_MSG_KEY_RLS:
                pwr_key = GPIO_PIN_SET;
                gLog(_DEBUG_LOG_|_PM_LOG_, "pmApi PWR_KEY-%d\r\n", pwr_key);
                if(pPwrKeyCallback != NULL)
                	pPwrKeyCallback(ctxKey, POWER_KEY_RELEASE);
                osTimerStop (pmKeyTimerHandle);
                break;
            case PM_MSG_KEY_PRESS:
                pwr_key = GPIO_PIN_RESET;
                gLog(_DEBUG_LOG_|_PM_LOG_, "pmApi PWR_KEY-%d\r\n", pwr_key);
                if(pPwrKeyCallback != NULL)
                	pPwrKeyCallback(ctxKey, POWER_KEY_PRESS);
                osTimerStart (pmKeyTimerHandle, 500);
                break;
            case PM_MSG_KEY_TIMEOUT:
                gLog(_DEBUG_LOG_|_PM_LOG_, "pmApi keytimeout\r\n");
                if(pPwrKeyCallback != NULL)
                {
                	pPwrKeyCallback(ctxKey, POWER_KEY_HOLD_500MS);
                }
                osTimerStart (pmKeyTimerHandle, 500);
                break;
            case PM_MSG_BATT_TIMEOUT:
            	pmUpdateBattInfo(&batt_status_new, &batt_rsoc);
            	if(batt_status_new != batt_status)
            	{
            		batt_status = batt_status_new;
            		if(pBatteryCallback != NULL)
            			pBatteryCallback(ctxBatt, batt_status);
            	}
                gLog(_DEBUG_LOG_|_PM_LOG_, "pmApi 0x%x v=%d t=%d, r=%d\r\n", batt_status, batt_volt, batt_temp, batt_rsoc);
                osTimerStart (pmBattTimerHandle, MAX_PWR_POLL_TIME);
                break;
            default:
                gLog(_WARN_LOG_|_PM_LOG_, "pmApi unknown msg.\r\n");
                break;
        }
	}
}

void pmPwrKeyRegister(PwrKeyCallback pFunc, void *ctx)
{
    pPwrKeyCallback = (void (*)(void *, _Bool))pFunc;
    ctxKey = ctx;
}

void pmBatteryRegister(BatteryCallback pFunc, void *ctx)
{
    pBatteryCallback = (void (*)(void *, uint8_t))pFunc;
    ctxBatt = ctx;
}

void pmGetPwrKey(_Bool *key)
{
	_Bool mykey;
	
	svPmGetPwrKey(&mykey);
	*key = mykey;
	return;
}

void pmGetMcuRccCsr(uint32_t *csr)
{
	uint32_t mycsr;
	
	if(rcc_csr == 0x0)
    {
        svPmGetMcuRccCsr(&mycsr);
        *csr = mycsr;
    }
	else
        *csr = rcc_csr;
	return;
}

void pmGetChgPin(_Bool *chg)
{
	_Bool mychg;
	
	svPmGetChgPin(&mychg);
	*chg = mychg;
	return;
}

void pmSetMcuOff(_Bool opcode)
{	
	svPmSetMcuOff(opcode);
	return;
}

/* for test only */
void pmSetBattAdcOn(_Bool opcode)
{	
	svPmSetBattAdcOn(opcode);
	return;
}

/* for test only */
int pmGetBattTempAdc(int16_t *temp)
{	
	int ret;
	int16_t mytemp;
	
	svPmSetBattAdcOn(1);
	ret = svPmGetBattTempAdc(&mytemp);
	svPmSetBattAdcOn(0);
	*temp = mytemp;
	return ret;
}

/* for test only */
int pmGetBattVbatAdc(uint16_t *volt)
{	
	int ret;
	uint16_t myvolt;
	
	svPmSetBattAdcOn(1);
	ret = svPmGetBattVbatAdc(&myvolt);
	svPmSetBattAdcOn(0);
	*volt = myvolt;
	return ret ;
}

/* for test only */
int pmGetBattGauge(uint8_t cmd, uint16_t *gauge)
{	
	int ret;
	uint16_t mygauge;
	
	ret = svPmGetBattGauge(cmd, &mygauge);
	*gauge = mygauge;
	return ret ;
}

void pmSetMcuReset(uint8_t rsv)
{
    svPmSetMcuReset(rsv);
}

int pmSetPwrState(uint8_t state)
{
	int ret;

    ret = svPmSetPwrState(state);
    return ret;
}

void pmGetBattStatus(uint8_t *status, uint8_t *rsoc)
{
    *status = batt_status;
    *rsoc = batt_rsoc;        
}

void CallbackPmKeyTm(void const * argument)
{
    osMessagePut(pmQueueHandle, (uint32_t) PM_MSG_KEY_TIMEOUT, osWaitForever);
}

void CallbackPmBattTm(void const * argument)
{
    osMessagePut(pmQueueHandle, (uint32_t) PM_MSG_BATT_TIMEOUT, osWaitForever);
}

void pmUpdateBattInfo(uint8_t *batt_st, uint8_t *batt_cap)
{
    uint8_t status;
    uint8_t rsoc;
    
    pmGetBattVbatAdc(&batt_volt);
    pmGetBattTempAdc(&batt_temp);
    status = 0;
    if(batt_volt < BATT_THRESHOLD_VLOW)
    {
        rsoc = 0;
        status = batt_status | BATT_RSOC_VLOW | BATT_RSOC_LOW;
    }
    else if(batt_volt < BATT_THRESHOLD_LOW)
    {
        rsoc = 20;
        status = batt_status | BATT_RSOC_LOW;
    }
    else if(batt_volt < BATT_THRESHOLD_NOR)
    {
        rsoc = 40;
    }
    else if(batt_volt < BATT_THRESHOLD_HIGH)
    {
        rsoc = 60;
    }
    else if(batt_volt < BATT_THRESHOLD_FULL)
    {
        rsoc = 80;
    }
    else
    {
        rsoc = 100;
    }
    
    if(batt_temp < BATT_THRESHOLD_LTEMP)
    {
        status = status | BATT_TEMP_LOW;
    }
    else if(batt_temp > BATT_THRESHOLD_HTEMP)
    {
        status = status | BATT_TEMP_HIGH;
    }
    
    osThreadSuspendAll();
    *batt_st = status;
    *batt_cap = rsoc;
    osThreadResumeAll();
}
