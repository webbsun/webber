//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#ifndef __SMAPI_H
#define __SMAPI_H
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "HTS221_Driver.h"
#include "LIS2DH12_Driver.h"
#include "cmsis_os.h"

/**
* @brief  Accelerometer Threshold Enable interrupt generation on axix_n low event or on direction recognition
*/
#define SMZUPE 	 0x20
#define SMZDOWNE 0x10
#define SMYUPE 	 0x08
#define SMYDOWNE 0x04
#define SMXUPE 	 0x02
#define SMXDOWNE 0x01


#define mutex_smDelay 1000

typedef enum {SM_DISABLE = (uint8_t)0, SM_ENABLE = (uint8_t)1} smState_et;

/**
* @brief  global variable 
*/
typedef enum
{
  SM_TIMPERIOD	 	 = 0,       
  SM_FIFOIRQ       = 1       
} smOperMd_et;

/**
* @brief  Temperature average.
*/
typedef enum
{
  SM_AVGT2         = 	HTS221_AVGT_2,        /*!< Internal average on 2 samples */
  SM_AVGT4         = 	HTS221_AVGT_4,        /*!< Internal average on 4 samples */
  SM_AVGT8         = 	HTS221_AVGT_8,        /*!< Internal average on 8 samples */
  SM_AVGT16        = 	HTS221_AVGT_16,        /*!< Internal average on 16 samples */
  SM_AVGT32        = 	HTS221_AVGT_32,        /*!< Internal average on 32 samples */
  SM_AVGT64        = 	HTS221_AVGT_64,        /*!< Internal average on 64 samples */
  SM_AVGT128       = 	HTS221_AVGT_128,        /*!< Internal average on 128 samples */
  SM_AVGT256       = 	HTS221_AVGT_256        /*!< Internal average on 256 samples */
} smAvgt_et;

/**
* @brief  Accelerometer Full scale.
*/
typedef enum {
  SM_FS2                =   LIS2DH12_FULLSCALE_2,
  SM_FS4                =   LIS2DH12_FULLSCALE_4,
  SM_FS8                =   LIS2DH12_FULLSCALE_8,
  SM_FS16               =   LIS2DH12_FULLSCALE_16
} smFS_et;

typedef void (*smSensorCallback)(void *ctx,_Bool sat);

typedef void (*smTempCallback)(void *ctx, float value);
typedef void (*smTempTHDCallback  )(void *ctx, float minth,float maxth);

typedef void (*smAccCallback)(void *ctx,int16_t *value);
typedef void (*smAccMovCallback) ( void *ctx, int16_t *value);


typedef enum 
{
	DEV_NINT = 0,	
	DEV_IDLE = 1,
	DEV_ACTE = 2,	
  DEV_BUSY = 3, 
	DEV_PDON = 4	
} DevSt_et;


typedef struct
{
	_Bool  pwr_flg;
	_Bool  init_flg;
	_Bool  irq1_flg;
	_Bool  irq2_flg;
	uint8_t  devid;
	uint8_t  opermod;
	uint8_t  wkmod;
	uint32_t querytm;
	DevSt_et  state;
	union{
		smTempCallback temp;
		smAccCallback  acc[2];
		}pfun;
	void * ctx_da;
	union{
		smTempTHDCallback temp;
		smAccMovCallback  acc;
		}pTHD;
	void * ctx_th;
} smDev_st;

typedef struct
{
	smSensorCallback pInt;
	void * ctx_int;
} SensInt_st;


extern I2C_HandleTypeDef hi2c1;
extern smDev_st AccleDev;
extern smDev_st TemperDev;

extern int AS6200_GetTempVal(float *Temperature);
/* Exported functions ------------------------------------------------------- */
extern void TempConfig(smDev_st *dev);
extern void TempConfig_IT(smDev_st *dev);
extern int  TempInit(smDev_st *dev);
extern void AccConfig(smDev_st *dev);
extern void AccConfig_IT(smDev_st *dev);
extern int 	AccInit(smDev_st *dev);
extern void GetFifoSRC(uint8_t* val);
extern void	GetFifoSFSS(uint8_t* val);
extern	int	GetTempVal(float *temperature);
extern	int GetHumiVal(float *humidity);
extern void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);

/* Exported functions for Netgear API ------------------------------------------------------- */
extern int smSensorInt(smOperMd_et md,smSensorCallback pfun, void * ctx);

extern	int	smSetTempPwr(smState_et state);
extern	int	smSetTempAvg( smAvgt_et  avgt);
extern  int smSetTempQueryTime(uint32_t mesec);
extern	int smGetTempData(smTempCallback pFun,void *ctx);
extern	int smGetTempData_remove(void);

extern  int smSetAccPwr(smState_et state);
extern  int GetAccValRaw(int16_t *accelerometer);
extern  int GetAccVal(int16_t *accelerometer);
extern 	int smGetAccDataRaw(smAccCallback pFun,void *ctx);
extern 	int smGetAccData (smAccCallback pFun,void *ctx);
extern	int smGetAccData_remove (void);
extern  int smSetAccQueryTime(uint32_t mesec);
extern	int smAccMvDt(uint8_t THS,uint8_t MvAxis ,smAccMovCallback  pFun, void * ctx);
extern	int smAccMvDt_remove(void);

#endif
