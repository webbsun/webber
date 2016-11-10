//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#include "smApi.h"
#include "HTS221_Driver.h"
#include "LIS2DH12_driver.h"
 

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_i2c_ex.h"

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
int __errno; // Workaround for libm


static smDev_st * pTempDev;
static smDev_st * pAccDev;
/*ams ic*/


int AS6200_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data )
{
	  HAL_StatusTypeDef status = HAL_OK;
	
  status = HAL_I2C_Mem_Read( handle, 0x90, ( uint16_t )RegAddr, I2C_MEMADD_SIZE_8BIT, Data, NumByteToRead, 0xfffff );
                             
  if( status != HAL_OK ) 
    return -1;
  else
    return 0;
}


int AS6200_GetTempVal(float *Temperature)
{
	//float temp;
	int ret=0;
	uint8_t val[2]={0};
	if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)
	{	
			ret=AS6200_ReadReg( &hi2c1,0x0, 2,val);
		if (ret<0)
			uartPrintf("AS6200_ReadReg fail\r\n");
		int TemperatureSum = ((val[0] << 8) | val[1]) >> 4; 
		*Temperature=TemperatureSum*0.0625;
		
		osMutexRelease(Mutex_I2C);
	return E_OK;		
	}

	return E_HAL;	
}

/************************************************
* 				ALL Sensor gobal API			 						*
************************************************/

int smSensorInt(smOperMd_et md,smSensorCallback pfun, void * ctx) //NetGeter
{
		trkMsg *pISRMsg;
			
		if(pfun == NULL)
			return E_ILLEGAL;	
	
		pISRMsg = gMem32Alloc();
	if(pISRMsg && smQueueHandle) {
		pISRMsg->hdr.id = SM_MSG_INITIAL;
		pISRMsg->hdr.cbk = (void*)pfun ;
		pISRMsg->hdr.ctx = ctx ;
		pISRMsg->hdr.len = 32-sizeof(msg_hdr);
		pISRMsg->param[0] = (uint8_t)md;

		if(osOK != osMessagePut(smQueueHandle, (uint32_t)pISRMsg, osWaitForever))
		{
			gMemFree(pISRMsg);
			return E_HAL;	
		}		
	}
	
	return E_OK;
}	



/************************************************
* 				Temperature API			 									*
************************************************/
void TempConfig(smDev_st *dev)
{
	dev->init_flg = pdFALSE;
	dev->pwr_flg = pdFALSE;
	dev->irq1_flg = pdFALSE;
	dev->irq2_flg = pdFALSE;
	dev->devid = 0xFF;
	dev->opermod = SM_TIMPERIOD;
	dev->wkmod = HTS221_ODR_1HZ;
	dev->querytm = 500;
	dev->state = DEV_NINT;
	dev->pfun.temp = NULL;
	dev->ctx_da = NULL;
	dev->pTHD.temp= NULL;
	dev->ctx_th = NULL;
	pTempDev = dev;
	
	return ;
}

void TempConfig_IT(smDev_st *dev)
{
	dev->init_flg = pdFALSE;
	dev->pwr_flg 	= pdFALSE;
	dev->irq1_flg = pdFALSE;
	dev->irq2_flg = pdFALSE;
	dev->devid = 0xFF;
	dev->opermod = SM_FIFOIRQ;
	dev->wkmod = HTS221_ODR_1HZ;
	dev->querytm = 500;
	dev->state = DEV_NINT;
	dev->pfun.temp = NULL;
	dev->ctx_da = NULL;
	dev->pTHD.temp= NULL;
	dev->ctx_th = NULL;
	pTempDev = dev;
	
	return ;
}

int TempInit(smDev_st *dev)
{
	uint8_t deviceid;
		
		if(HTS221_Get_DeviceID(&hi2c1,&deviceid)== HTS221_ERROR)
				return E_HAL;
			else
				dev->devid=deviceid;
			
			if(deviceid != HTS221_WHO_AM_I_VAL)
				return E_PERAL_DEVICE;
		
		if(dev->wkmod == SM_TIMPERIOD)
		{
			if(HTS221_DeActivate(&hi2c1)== HTS221_ERROR)
				return E_HAL;

			if(HTS221_Set_BduMode(&hi2c1,HTS221_ENABLE)==HTS221_ERROR )
				return E_HAL;
				
			if(HTS221_Set_Odr(&hi2c1,dev->wkmod)==HTS221_ERROR )
				return E_HAL;
		}
		else
		{		
				HTS221_Init_st HTS221_handle;
				HTS221_handle.avg_h= HTS221_AVGH_32;            /*!< Humidity average */
				HTS221_handle.avg_t= HTS221_AVGT_16;            /*!< Temperature average */
				HTS221_handle.odr=HTS221_ODR_1HZ;              /*!< Output data rate */
				HTS221_handle.bdu_status=HTS221_ENABLE;       /*!< HTS221_ENABLE/HTS221_DISABLE the block data update */
				HTS221_handle.heater_status=HTS221_DISABLE;    /*!< HTS221_ENABLE/HTS221_DISABLE the internal heater */
				HTS221_handle.irq_level=HTS221_HIGH_LVL;        /*!< HTS221_HIGH_LVL/HTS221_LOW_LVL the level for DRDY pin */
				HTS221_handle.irq_output_type=HTS221_PUSHPULL;  /*!< Output configuration for DRDY pin */
				HTS221_handle.irq_enable=HTS221_ENABLE;       /*!< HTS221_ENABLE/HTS221_DISABLE interrupt on DRDY pin */
				
			if(HTS221_Set_InitConfig(&hi2c1,&HTS221_handle)==HTS221_ERROR)
				return E_HAL;	

			 HTS221_DeActivate(&hi2c1);
			
					/* Clear data */
			float temperatur,humidity;
			GetTempVal(&temperatur);
			GetHumiVal(&humidity);
		 
			odmExtiEn(PRESSURE_INT_DRDY_Pin);
			dev->init_flg=pdFALSE;
		}
		
		dev->init_flg = pdTRUE;
		return E_OK;
}

int smSetTempPwr(smState_et state)  //NetGeter
{
	if(pTempDev == NULL)
			return E_NINIT;
	
 if(pTempDev->init_flg == pdFALSE)
		return E_NINIT;
	
	if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)
	{	
		if(state == SM_ENABLE)
		{
				if(HTS221_Activate(&hi2c1)==HTS221_ERROR )
				{
					osMutexRelease(Mutex_I2C);
					return E_HAL;
				}			
				else
				 pTempDev->pwr_flg = pdTRUE;
		}
		else if(state == SM_DISABLE)
		{
				if(HTS221_DeActivate(&hi2c1)==HTS221_ERROR )
				{
					osMutexRelease(Mutex_I2C);
					return E_HAL;
				}		
				else
				 pTempDev->pwr_flg = pdFALSE;			
		}
		else
		{
					osMutexRelease(Mutex_I2C);
					return E_HAL;
		}	
		 osMutexRelease(Mutex_I2C);	
		return E_OK;
	
	}
	else
	{	
		gLog(_DEBUG_LOG_|_SM_LOG_, "smSetTempPwr smMutex timout \n");
		return E_BUSY;
	}
}

int smSetTempAvg(smAvgt_et avgt) //NetGer
{
	if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)
	{	
	
		if(HTS221_Set_AvgT(&hi2c1,(HTS221_Avgt_et)avgt)==HTS221_ERROR)
		{	
			 osMutexRelease(Mutex_I2C);	
			 return E_HAL;
		}
		osMutexRelease(Mutex_I2C);	
	}
	else
	{
		gLog(_DEBUG_LOG_|_SM_LOG_, "smSetTempAvg smMutex timout \n");
		return E_BUSY;
	}
	return E_OK;
}

int smSetTempQueryTime(uint32_t mesec) //NetGer
{
	if(mesec<10)
	 pTempDev->querytm = 10;
	else
	  pTempDev->querytm = mesec;
	return E_OK;
}

int GetTempVal(float *temperature)
{
	int16_t tmper;
  if((HTS221_Get_Temperature(&hi2c1,&tmper)==HTS221_ERROR))
	  return E_HAL;
  *temperature = ( float )tmper / 10.0f;
  return E_OK;
}

int GetHumiVal(float *humidity)
{
	uint16_t humi;
  if((HTS221_Get_Humidity(&hi2c1,&humi)==HTS221_ERROR))
	  return E_HAL;
  *humidity = ( float )humi / 10.0f;
  return E_OK;
}

int smGetTempData(smTempCallback pFun,void *ctx) //NetGer
{
	if(pFun == NULL)
		return E_ILLEGAL;
	
	pTempDev->pfun.temp=pFun;
	pTempDev->ctx_da=ctx;
	return E_OK;
}

int smGetTempData_remove(void) //NetGer
{
	
	pTempDev->pfun.temp=NULL;
	
	return E_OK;
}



void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec)
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}






/************************************************
* 				Accelerometer API			 							*
************************************************/
void AccConfig(smDev_st *dev)
{
	dev->init_flg = pdFALSE;
	dev->pwr_flg = pdFALSE;
	dev->irq1_flg = pdFALSE;
	dev->irq2_flg = pdFALSE;
	dev->devid = 0xFF;
	dev->opermod = SM_TIMPERIOD;
	dev->wkmod = LIS2DH12_LOW_POWER;
	dev->state = 0;
	dev->querytm = 500;
	dev->pfun.acc[0] = NULL;
	dev->pfun.acc[1] = NULL;
	dev->pTHD.acc	=	NULL;
	pAccDev = dev;	
	return ;
}

void AccConfig_IT(smDev_st *dev)
{
	dev->init_flg = pdFALSE;
	dev->pwr_flg = pdFALSE;
	dev->irq1_flg = pdFALSE;
	dev->irq2_flg = pdFALSE;
	dev->devid = 0xFF;
	dev->opermod = SM_FIFOIRQ;
	dev->wkmod = LIS2DH12_LOW_POWER;
	dev->querytm = 500;
	dev->state = DEV_NINT;
	dev->pfun.acc[0] = NULL;
	dev->pfun.acc[1] = NULL;
	dev->ctx_da = NULL;
	dev->pTHD.acc	=	NULL;
	pAccDev = dev;	
	return ;
}

void GetFifoSRC(uint8_t* val)
{
		LIS2DH12_GetInt2Src(val);	
}
void	GetFifoSFSS(uint8_t* val)
{
	LIS2DH12_GetFifoSourceFSS(val);
}

int AccInit(smDev_st *dev)
{
	uint8_t deviceid;

		if(dev->opermod == SM_TIMPERIOD )
		{
				if(LIS2DH12_GetWHO_AM_I(&deviceid)== MEMS_ERROR)
					return E_HAL;
				else
					dev->devid=deviceid;
				
				if(deviceid != LIS2DH12_WHO_AM_I_VAL)
					return E_PERAL_DEVICE;

				if(LIS2DH12_SetODR(LIS2DH12_ODR_10Hz)== MEMS_ERROR)
					return E_HAL;
						
				if(LIS2DH12_SetMode(dev->wkmod)== MEMS_ERROR)
					return E_HAL;
				
				if(LIS2DH12_SetFullScale(LIS2DH12_FULLSCALE_2)==MEMS_ERROR )
					return E_HAL;

				if(LIS2DH12_SetBDU(MEMS_ENABLE)==MEMS_ERROR )
					return E_HAL;

				if(LIS2DH12_SetAxis(LIS2DH12_X_ENABLE | LIS2DH12_Y_ENABLE | LIS2DH12_Z_ENABLE)==MEMS_ERROR )
					return E_HAL;
				
				if(LIS2DH12_SetMode(LIS2DH12_POWER_DOWN)== MEMS_ERROR)
					return E_HAL;		
					odmExtiEn(GYRO_G_INT2_Pin);	
		}
		else	
		{				
				if(LIS2DH12_GetWHO_AM_I(&deviceid)== MEMS_ERROR)
					return E_HAL;
				else
					dev->devid=deviceid;
				
				if(deviceid != LIS2DH12_WHO_AM_I_VAL)
					return E_PERAL_DEVICE;

				if(LIS2DH12_SetODR(LIS2DH12_ODR_10Hz)== MEMS_ERROR)
					return E_HAL;
						
				if(LIS2DH12_SetMode(dev->wkmod)== MEMS_ERROR)
					return E_HAL;
				
				if(LIS2DH12_SetFullScale(LIS2DH12_FULLSCALE_2)==MEMS_ERROR )
					return E_HAL;

				if(LIS2DH12_SetBDU(MEMS_ENABLE)==MEMS_ERROR )
					return E_HAL;

				if(LIS2DH12_SetAxis(LIS2DH12_X_ENABLE | LIS2DH12_Y_ENABLE | LIS2DH12_Z_ENABLE)==MEMS_ERROR )
					return E_HAL;
				
				if(LIS2DH12_SetMode(LIS2DH12_POWER_DOWN)== MEMS_ERROR)
					return E_HAL;		
					
				LIS2DH12_FIFOModeEnable(LIS2DH12_FIFO_DISABLE); // de-inital
				LIS2DH12_SetInt1Pin(LIS2DH12_WTM_ON_INT1_ENABLE);		
				LIS2DH12_SetWaterMark(8);
				LIS2DH12_SetTriggerInt(LIS2DH12_TRIG_INT1);
				LIS2DH12_FIFOModeEnable(LIS2DH12_FIFO_MODE);					
		}		
		pAccDev->init_flg = pdTRUE;
		return E_OK;

}

int smSetAccPwr(smState_et state) //NetGer
{
	if(pAccDev == NULL)
			return E_NINIT;
	
	if(pAccDev->init_flg == pdFALSE)
			return E_NINIT;

	if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)	
		{	
			if(state == SM_ENABLE)
			{		
				 odmExtiEn(GYRO_G_INT1_Pin);
				 odmExtiEn(GYRO_G_INT2_Pin);
				
					if(LIS2DH12_SetMode(pAccDev->wkmod)==MEMS_ERROR )
					{
						osMutexRelease(Mutex_I2C);
						return E_HAL;
					}		
					else
						pAccDev->pwr_flg = pdTRUE;		 
				 
			}
			else if(state == SM_DISABLE)
			{
					odmExtiDis(GYRO_G_INT1_Pin);
					odmExtiDis(GYRO_G_INT2_Pin);
				
				
				 if(LIS2DH12_SetMode(LIS2DH12_POWER_DOWN)==MEMS_ERROR )
					{
						osMutexRelease(Mutex_I2C);
						return E_HAL;
					}		
					else
						pAccDev->pwr_flg = pdFALSE;
				 
			}
			else
			{
						osMutexRelease(Mutex_I2C);
						return E_HAL;
			}	
				osMutexRelease(Mutex_I2C);
				return E_OK;
		}
		else
		{
			gLog(_DEBUG_LOG_|_SM_LOG_, "smSetAccPwr smMutex timout \n");		
			return E_BUSY;
		}	
}

int smSeAccFS(smFS_et fsval) //NetGer
{
		
		if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)	
		{	 
				if(LIS2DH12_SetFullScale((LIS2DH12_Fullscale_t)fsval) == MEMS_ERROR )
				{
					 osMutexRelease(Mutex_I2C);
					 return E_HAL;
				}
				 osMutexRelease(Mutex_I2C);
		}
		else
		{
			gLog(_DEBUG_LOG_|_SM_LOG_, "smAccMvDt smMutex timout \n");	
			return E_BUSY;
		}
		return E_OK;
}

int smSetAccQueryTime(uint32_t mesec) //NetGer
{
		if(mesec<10)
		 pAccDev->querytm = 10;
		else
			pAccDev->querytm = mesec;
		return E_OK;
}

int GetAccValRaw(int16_t *accelerometer)
{

	if(LIS2DH12_GetAccAxesRaw((AxesRaw_t*)accelerometer) == MEMS_ERROR )
	  return E_HAL;
  return E_OK;
}

int GetAccVal(int16_t *accelerometer)
{
	if(LIS2DH12_GetAccAxes((Axes_t*)accelerometer) == MEMS_ERROR )
	  return E_HAL;
  return E_OK;
}

int smGetAccDataRaw(smAccCallback pFun,void *ctx) //NetGer
{

	if(pFun == NULL)
		return E_ILLEGAL;
	
	pAccDev->pfun.acc[0]=pFun;
	pAccDev->ctx_da= ctx;
	
	return E_OK;
	
}

int smGetAccData (smAccCallback pFun,void *ctx) //NetGer
{
	if(pFun == NULL)
		return E_ILLEGAL;
	
	pAccDev->pfun.acc[1]=pFun;
	pAccDev->ctx_da= ctx;
	
	return E_OK;
}

int smGetAccData_remove (void) //NetGer
{
	
	pAccDev->pfun.acc[1]=NULL;
	
	return E_OK;
}

int smAccMvDt(uint8_t THS,uint8_t MvAxis ,smAccMovCallback  pFun, void * ctx) //NetGer
{
	if(pFun == NULL)
		return E_ILLEGAL;
	
	 if(THS > 128 )
		 THS =128;
	 
	 smSetAccPwr(SM_DISABLE);	
		if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)	
		{	 
		 LIS2DH12_SetInt2Threshold(THS);	
		 LIS2DH12_SetInt2Configuration(MvAxis&0x3f); 
		 LIS2DH12_SetInt2Mode(LIS2DH12_INT_MODE_6D_POSITION);	
		 LIS2DH12_SetInt2Pin(LIS2DH12_I2_INT2_ON_PIN_INT2_ENABLE);   
		 pAccDev->pTHD.acc = pFun;
		 pAccDev->ctx_th = ctx;		
		 osMutexRelease(Mutex_I2C);
		}
		else
		{
			gLog(_DEBUG_LOG_|_SM_LOG_, "smAccMvDt smMutex timout \n");	
			return E_BUSY;
		}
	 smSetAccPwr(SM_ENABLE);
	return E_OK;
}


int smAccMvDt_remove(void) //NetGer
{

	 	 		
	  smSetAccPwr(SM_DISABLE);	
		if(osMutexWait(Mutex_I2C, mutex_smDelay) == osOK)	
		{	 
		 LIS2DH12_SetInt2Pin(LIS2DH12_I2_INT2_ON_PIN_INT2_DISABLE); 
		 LIS2DH12_SetInt2Configuration(0); 	   
		 pAccDev->pTHD.acc = NULL;
		 pAccDev->ctx_th = NULL;
		 osMutexRelease(Mutex_I2C);
		}
		else
		{
			gLog(_DEBUG_LOG_|_SM_LOG_, "smAccMvDt smMutex timout \n");	
			return E_BUSY;
		}
	 smSetAccPwr(SM_ENABLE);
	return E_OK;
}


