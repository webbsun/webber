//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#ifndef PMSRV_H
#define PMSRV_H

#include "stm32l4xx_hal.h"

//extern void svPmPwrKeyRegister(PwrKeyCallback *pFunc, void *ctx);
//extern void svPmBattStatusRegister(BatteryCallback *pFunc, void *ctx);
extern int  svPmSetPwrState(uint8_t state);
extern void svPmSetMcuOff(_Bool opcode);
extern void svPmGetMcuRccCsr(uint32_t *csr);
extern void svPmGetPwrKey(_Bool *key);
extern void svPmGetBattStatus(uint8_t *status, uint8_t *rsoc);
extern void svPmSetMcuReset(uint8_t rsv);

extern void svPmGetChgPin(_Bool *chg);
extern void svPmSetBattAdcOn(_Bool adc);       /* ctrl=PC2(high=connected)*/
extern int svPmGetBattTempAdc(int16_t *temp); /* ctrl=PC2(high=connected), adc1=PA5=channel 10 */
extern int svPmGetBattVbatAdc(uint16_t *volt); /* ctrl=PC2(high=connected), adc1=PA4=channel 9 */
extern int svPmGetBattGauge(uint8_t cmd, uint16_t *gauge);  /* I2C 0xAA/0xAB */
extern int svPmSetPwrState(uint8_t state);

#endif //PMSRV_H

