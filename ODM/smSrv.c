//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#include "smSrv.h"
#include "smApi.h"
               
#include "stm32l4xx.h" 
#include "uart.h"
#include "msgDef.h"
#include "errDef.h"
#include "genApi.h"
#include "genSrv.h"
#include <math.h>   // trunc

#include <stdlib.h>
#include <string.h>
#include <ctype.h>


osMessageQId smQueueHandle = NULL;

/************************************************
* 				Query sensor data task		 						*
************************************************/
static void smTimer1Callback(void const *argument)
{
	// Temperature	
	
		trkMsg *pISRMsg;
	
			pISRMsg = gMem32Alloc();
		if(pISRMsg && smQueueHandle) {
			pISRMsg->hdr.id = SM_MSG_TEMP_TM;
			pISRMsg->hdr.len = 32-sizeof(msg_hdr);
			//pISRMsg->param[0] = (uint8_t)(huart->Instance->RDR & 0x0FF);
			if(osOK != osMessagePut(smQueueHandle, (uint32_t)pISRMsg, osWaitForever))
				gMemFree(pISRMsg);
		}
	
}

static void smTimer2Callback(void const *argument)
{
	// accelerometer	
	trkMsg *pISRMsg;
	
			pISRMsg = gMem32Alloc();
		if(pISRMsg && smQueueHandle) {
			pISRMsg->hdr.id = SM_MSG_ACCE_TM;
			pISRMsg->hdr.len = 32-sizeof(msg_hdr);
			if(osOK != osMessagePut(smQueueHandle, (uint32_t)pISRMsg, osWaitForever))
				gMemFree(pISRMsg);
		}
	
}

void smTask(void const * argument)
{
	osEvent smEvent;
	trkMsg *pMsg;
	
	smDev_st tsens;
	smDev_st gsens;

	osMessageQDef(smQueue, 8, uint32_t);
	smQueueHandle = osMessageCreate(osMessageQ(smQueue), NULL);
	

	osTimerDef(smTimer1, smTimer1Callback);
	osTimerId osTimer1 = osTimerCreate(osTimer(smTimer1), osTimerOnce, NULL);
	
	osTimerDef(smTimer2, smTimer2Callback);
	osTimerId osTimer2 = osTimerCreate(osTimer(smTimer2), osTimerOnce, NULL);	
	
	while(1)
	{
		smEvent = osMessageGet(smQueueHandle, osWaitForever);
		pMsg = (trkMsg *)smEvent.value.p;
		
			switch(pMsg->hdr.id)
			{
				case SM_MSG_INITIAL:

						if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)
						{		
									if(pMsg->param[0] == SM_FIFOIRQ )
									{
											TempConfig_IT(&tsens);
										if(TempInit(&tsens)!= E_OK)
											gLog(_CRIT_LOG_|_SM_LOG_, "tConfig FAIL\n");
										else
											gLog(_DEBUG_LOG_|_SM_LOG_, "tSensorID:  0x%02x\r\n",tsens.devid);	

											AccConfig_IT(&gsens);
										if(AccInit(&gsens)!= E_OK)
											gLog(_CRIT_LOG_|_SM_LOG_, "gConfig FAIL\n");
										else
											gLog(_DEBUG_LOG_|_SM_LOG_, "gSensorID:  0x%02x\r\n",gsens.devid);			
									
									}
									else
									{
										TempConfig(&tsens);							
										if(TempInit(&tsens)!= E_OK)
											gLog(_CRIT_LOG_|_SM_LOG_, "tConfig FAIL\n");
										else
										{
											gLog(_DEBUG_LOG_|_SM_LOG_, "tSensorID:  0x%02x\r\n",tsens.devid);	
											osTimerStart(osTimer1, 500);
										}
										
										AccConfig(&gsens);							
										if(AccInit(&gsens)!= E_OK)
											gLog(_CRIT_LOG_|_SM_LOG_, "gConfig FAIL\n");
										else
										{
											gLog(_DEBUG_LOG_|_SM_LOG_, "gSensorID:  0x%02x\r\n",gsens.devid);			
											osTimerStart(osTimer2, 500);	
										}												
									}	

									pMsg->hdr.cbk(pMsg->hdr.ctx,pdTRUE);
									osMutexRelease(Mutex_I2C);
							}
						else
						{
							gLog(_DEBUG_LOG_|_SM_LOG_, "SM_MSG_INITIAL smMutex timout \n");	 
							pMsg->hdr.cbk(pMsg->hdr.ctx,pdFALSE);
						}		
				break;
				case SM_MSG_TEMP_TM:
					if(tsens.pwr_flg == pdTRUE)
					{
						 float temperatur;
						 int32_t d3, d4;
						 if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)
						 { 					
								if(GetTempVal(&temperatur)!=E_OK)
										gLog(_CRIT_LOG_|_SM_LOG_, "GetTempData FAIL\n");
								else
								{
									floatToInt(temperatur, &d3, &d4, 2);
									gLog(_INFO_LOG_|_SM_LOG_, "Tempe:  %d.%02d\r\n\n",(int)d3, (int)d4);		
									if(tsens.pfun.temp != NULL)
										tsens.pfun.temp(tsens.ctx_da,temperatur);
									
								}
								osMutexRelease(Mutex_I2C);	
							}
						 else
							 	gLog(_DEBUG_LOG_|_SM_LOG_, "SM_MSG_TEMP_TM smMutex timout \n");	 
					}							
					osTimerStart(osTimer1, tsens.querytm);				
				break;
					
				case SM_MSG_ACCE_TM:
					if(gsens.pwr_flg == pdTRUE)
					{
						static int16_t data[3];	
						uint8_t skip_flg=0;
						if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)
						{		
								if(gsens.pfun.acc[0] != NULL)
								{
										if(GetAccValRaw(&data[0]) != E_OK)
										gLog(_CRIT_LOG_|_SM_LOG_, "GetACCRData FAIL\n");
										else
										gLog(_INFO_LOG_|_SM_LOG_, "ACC: X=%d Y=%d Z=%d \r\n", data[0],data[1], data[2]);		
										
										gsens.pfun.acc[0](gsens.ctx_da,&data[0]);
										skip_flg=1;
								}
								
								if(gsens.pfun.acc[1] != NULL)
								{
										if(GetAccVal(&data[0]) != E_OK)
											gLog(_CRIT_LOG_|_SM_LOG_, "GetACCData FAIL\n");
										else
											gLog(_INFO_LOG_|_SM_LOG_, "ACC: X=%d Y=%d Z=%d \r\n", data[0],data[1], data[2]);		
										
										gsens.pfun.acc[1](gsens.ctx_da,&data[0]);
										skip_flg=1;
								}
								
								if(!skip_flg)
								{
									if(GetAccVal(&data[0]) != E_OK)
											gLog(_CRIT_LOG_|_SM_LOG_, "GetACCData FAIL\n");
										else
											gLog(_INFO_LOG_|_SM_LOG_, "ACC: X=%d Y=%d Z=%d \r\n", data[0],data[1], data[2]);												
								}
								osMutexRelease(Mutex_I2C);	
							}
							else
								gLog(_DEBUG_LOG_|_SM_LOG_, "SM_MSG_ACCE_TM smMutex timout \n");	
						
					}
					osTimerStart(osTimer2, gsens.querytm);								
				break;
				case SM_MSG_TEMP_ISR:
					if(tsens.pwr_flg == pdTRUE)
					{					
						 float temperatur,humidity;
						 int32_t d3, d4;
							if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)
							{		
								if(GetTempVal(&temperatur)!=E_OK)
									gLog(_CRIT_LOG_|_SM_LOG_, "GetTempData FAIL\n");
								else
								{
									GetHumiVal(&humidity);
									floatToInt(temperatur, &d3, &d4, 2);
									//gLog(_INFO_LOG_|_SM_LOG_, "Tempe:  %d.%02d\r\n\n",(int)d3, (int)d4);	
									if(tsens.pfun.temp != NULL)
										tsens.pfun.temp(tsens.ctx_da,temperatur);			
								}	
								osMutexRelease(Mutex_I2C);	
							}
							else
								gLog(_DEBUG_LOG_|_SM_LOG_, "SM_MSG_TEMP_ISR smMutex timout \n");
					}
				break;	
					
				case SM_MSG_ACCE_ISR1:
				
					if(gsens.pwr_flg == pdTRUE)
					{
							static int16_t data[3];
							uint8_t FIFOSRC,depth;
						
							if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)
							{								
									GetFifoSRC(&FIFOSRC);
									GetFifoSFSS(&depth);	
									depth--;
									while(depth--)
									{
										if(GetAccValRaw(&(data[0])) != E_OK)
											gLog(_CRIT_LOG_|_SM_LOG_, "GetACCData FAIL\n");
					
									}
									
									uint8_t skip_flg=0;				
									if(gsens.pfun.acc[0] != NULL)
									{
											if(GetAccValRaw(&data[0]) != E_OK)
											gLog(_CRIT_LOG_|_SM_LOG_, "GetACCRData FAIL\n");
											else
											gLog(_INFO_LOG_|_SM_LOG_, "ACC: X=%d Y=%d Z=%d \r\n", data[0],data[1], data[2]);		
											
											gsens.pfun.acc[0](gsens.ctx_da,&data[0]);
											skip_flg=1;
									}
									
									if(gsens.pfun.acc[1] != NULL)
									{
											if(GetAccVal(&data[0]) != E_OK)
												gLog(_CRIT_LOG_|_SM_LOG_, "GetACCData FAIL\n");
											else
												gLog(_INFO_LOG_|_SM_LOG_, "ACC: X=%d Y=%d Z=%d \r\n", data[0],data[1], data[2]);		
											
											gsens.pfun.acc[1](gsens.ctx_da,&data[0]);
											skip_flg=1;
									}
									
									if(!skip_flg)
									{
										if(GetAccVal(&data[0]) != E_OK)
												gLog(_CRIT_LOG_|_SM_LOG_, "GetACCData FAIL\n");
											else
												gLog(_INFO_LOG_|_SM_LOG_, "ACC: X=%d Y=%d Z=%d \r\n", data[0],data[1], data[2]);												
									}	
									osMutexRelease(Mutex_I2C);	
							}
							else
								gLog(_DEBUG_LOG_|_SM_LOG_, "SM_MSG_ACCE_ISR1 smMutex timout \n");
					}						
					break;
					
				case SM_MSG_ACCE_ISR2:
					if(gsens.pwr_flg == pdTRUE)
					{
							static int16_t data[3];	
							
							if(osMutexWait(Mutex_I2C, mutex_smDelay)== osOK)
							{			
								if(GetAccVal(&data[0]) != E_OK)
								gLog(_CRIT_LOG_|_SM_LOG_, "GetACCRData FAIL\n");
								else
								gLog(_INFO_LOG_|_SM_LOG_, "ACC_TH: X=%d Y=%d Z=%d \r\n", data[0],data[1], data[2]);		
							
								if(gsens.pTHD.acc != NULL)
									gsens.pTHD.acc(&data[0],gsens.ctx_th);
								
								osMutexRelease(Mutex_I2C);
							}
							else
								gLog(_DEBUG_LOG_|_SM_LOG_, "smSetTempPwr smMutex timout \n");										
					}
				
					break;
				default:
					gLog(_WARN_LOG_|_SM_LOG_, "SM: Illegal msg id:0x%04x\r\n", pMsg->hdr.id);
			}		
			gMemFree(pMsg);		
	}	
}



