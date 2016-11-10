//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#ifndef PMAPI_H
#define PMAPI_H

#include "stm32l4xx_hal.h"

//extern void pmPwrKeyRegister(PwrKeyCallback *pFunc, void *ctx);
//extern void pmBattStatusRegister(BatteryCallback *pFunc, void *ctx);
extern int  pmSetPwrState(uint8_t state);
extern void pmSetMcuOff(_Bool opcode);
extern void pmGetMcuRccCsr(uint32_t *csr);
extern void pmGetPwrKey(_Bool *key);
extern void pmGetBattStatus(uint8_t *status, uint8_t *rsoc);
extern void pmSetMcuReset (uint8_t rsv);

extern void pmGetChgPin(_Bool *chg);         /* for test only */
extern void pmSetBattAdcOn(_Bool adc);       /* ctrl=PC2(high=connected), for test only */
extern int pmGetBattTempAdc(int16_t *temp);  /* unit=C, ctrl=PC2(high=connected), adc1=PA5=channel 10 */
extern int pmGetBattVbatAdc(uint16_t *volt); /* unit=mV, ctrl=PC2(high=connected), adc1=PA4=channel 9 */
extern int pmGetBattGauge(uint8_t cmd, uint16_t *gauge); /* for test only */
extern int pmSetPwrState(uint8_t state);

typedef void (*PwrKeyCallback)(void *ctx, _Bool key);
typedef void (*BatteryCallback)(void *ctx, uint8_t status);

extern void pmPwrKeyRegister(PwrKeyCallback pFunc, void *ctx);
extern void pmBatteryRegister(BatteryCallback pFunc, void *ctx);

#define BATT_TEMP_HIGH  ((uint8_t)0x01)  /* bit0=1, battery temperature is too high */
#define BATT_TEMP_LOW   ((uint8_t)0x02)  /* bit1=1, battery temperature is too low */
#define BATT_RSOC_LOW   ((uint8_t)0x04)  /* bit2=1, battery remaining capacity low alarm */
#define BATT_RSOC_VLOW  ((uint8_t)0x08)  /* bit3=1, battery remaining capacity critical low alarm */

#define BATT_THRESHOLD_VLOW  3300  /* mV, remaining capacity critical low alarm */
#define BATT_THRESHOLD_LOW   3500  /* mV, remaining capacity too low alarm */
#define BATT_THRESHOLD_NOR   3800  /* mV, remaining capacity normal */
#define BATT_THRESHOLD_HIGH  4000  /* mV, remaining capacity high */
#define BATT_THRESHOLD_FULL  4100  /* mV, remaining capacity full */

#define BATT_THRESHOLD_LTEMP 0     /* degree-C, temperature low alarm */
#define BATT_THRESHOLD_HTEMP 50    /* degree-C, temperature high alarm */

typedef enum
{
    GAUGE_CMD_TEMP    = 0x06,
    GAUGE_CMD_VOLTAGE = 0x08,
    GAUGE_CMD_STATUS  = 0x0a,
    GAUGE_CMD_CURRENT = 0x0c,
    GAUGE_CMD_RSOC    = 0x2c
} PWR_GAUGE_Cmd;

typedef enum
{
	POWER_MODE_STOP2 = 0,
	POWER_MODE_SHUTDOWN,
	POWER_MODE_OTHER
} PWR_MODE_State;

typedef enum
{
	POWER_KEY_PRESS = 0,
	POWER_KEY_RELEASE,
	POWER_KEY_HOLD_500MS
} PWR_Key_State;

#endif //PMAPI_H

