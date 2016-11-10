//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//

#ifndef __GMAPI_H
#define __GMAPI_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

//Structure for UTC time injection
typedef struct {
    uint16_t utcYear;
    uint8_t utcMonth;
    uint8_t utcDay;
    uint8_t utcHour;
    uint8_t utcMin;
    uint8_t utcSec;
} gmUtcTime_st;

//Structure for approximate position injection
typedef struct {
    int16_t latDeg;
    uint8_t latMin;
    uint8_t latSec;
    int16_t lonDeg;
    uint8_t lonMin;
    uint8_t lonSec;
} gmLatLon_st;

//Callback function definition
typedef void (*GpsNmeaCallback)(char *nmea, void *ctx);
typedef void (*GpsSetNmeaTypesCallback)(int result, void *ctx);
typedef void (*GpsSetSvSysCallback)(int result, void *ctx);
typedef void (*GpsColdStartCallback)(int result, void *ctx);
typedef void (*GpsWarmStartCallback)(int result, void *ctx);
typedef void (*GpsHotStartCallback)(int result, void *ctx);
typedef void (*GpsStopCallback)(int result, void *ctx);
typedef void (*GpsGetFwVerCallback)(char *FwVersion, void *ctx);
typedef void (*GpsSetSleepStateCallback)(int result, void *ctx);
typedef void (*GpsWakeCallback)(int result, void *ctx);
typedef void (*GpsInjectTimeCallback)(int result, void *ctx);
typedef void (*GpsInjectPosCallback)(int result, void *ctx);
typedef void (*GpsInjectCepCallback)(int result, void *ctx);

//API for applications
int gmInitialize(GpsNmeaCallback pFunc, void *ctx);
int gmSetNmeaTypes(uint32_t typeMask, GpsSetNmeaTypesCallback pFunc, void *ctx);
int gmSetSvSystem(uint8_t svSysMask, GpsSetSvSysCallback pFunc, void *ctx);
int gmSetPowerState(uint8_t state);
int gmGetPowerState(void);
int gmStartColdStart(GpsColdStartCallback pFunc, void *ctx);
int gmStartWarmStart(GpsWarmStartCallback pFunc, void *ctx);
int gmStartHotStart(GpsHotStartCallback pFunc, void *ctx);
int gmStopPositioning(GpsStopCallback pFunc, void *ctx);
int gmGetFirmwareVersion(GpsGetFwVerCallback pFunc, void *ctx);
int gmSetSleepState(GpsSetSleepStateCallback pFunc, void *ctx);
int gmWakeUp(GpsWakeCallback pFunc, void *ctx);
int gmInjectTime(gmUtcTime_st time, GpsInjectTimeCallback pFunc, void *ctx);
int gmInjectPos(gmLatLon_st pos, GpsInjectPosCallback pFunc, void *ctx);
int gmInjectCep(GpsInjectCepCallback pFunc, void *ctx);

#endif
