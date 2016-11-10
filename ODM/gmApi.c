//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//

#include "cmsis_os.h"
#include "gmApi.h"
#include "uart.h"

//#include "genApi.h"

int gmInitialize(GpsNmeaCallback pFunc, void *ctx)
{
    uartPrintf("gmInitialize\r\n");
    return 0;
}

int gmSetNmeaTypes(uint32_t typeMask, GpsSetNmeaTypesCallback pFunc, void *ctx)
{
    uartPrintf("gmSetNmeaTypes: %d\r\n", typeMask);
    return 0;
}

int gmSetSvSystem(uint8_t svSysMask, GpsSetSvSysCallback pFunc, void *ctx)
{
    uartPrintf("gmSetSvSystem: %d\r\n", svSysMask);
    return 0;
}

int gmSetPowerState(uint8_t state)
{
    uartPrintf("gmSetPowerState: %d\r\n", state);
    return 0;
}

int gmGetPowerState(void)
{
    uartPrintf("gmGetPowerState\r\n");
    return 0;
}

int gmStartColdStart(GpsColdStartCallback pFunc, void *ctx)
{
    uartPrintf("gmStartColdStart\r\n");
    return 0;
}

int gmStartWarmStart(GpsWarmStartCallback pFunc, void *ctx)
{
    uartPrintf("gmStartWarmStart\r\n");
    return 0;
}

int gmStartHotStart(GpsHotStartCallback pFunc, void *ctx)
{
    uartPrintf("gmStartHotStart\r\n");
    return 0;
}

int gmStopPositioning(GpsStopCallback pFunc, void *ctx)
{
    uartPrintf("gmStopPositioning\r\n");
    return 0;
}

int gmGetFirmwareVersion(GpsGetFwVerCallback pFunc, void *ctx)
{
    uartPrintf("gmGetFirmwareVersion\r\n");
    return 0;
}

int gmSetSleepState(GpsSetSleepStateCallback pFunc, void *ctx)
{
    uartPrintf("gmSetSleepState\r\n");
    return 0;
}

int gmWakeUp(GpsWakeCallback pFunc, void *ctx)
{
    uartPrintf("gmWakeUp\r\n");
    return 0;
}

int gmInjectTime(gmUtcTime_st time, GpsInjectTimeCallback pFunc, void *ctx)
{
    uartPrintf("time inject\r\n");
    return 0;
}

int gmInjectPos(gmLatLon_st pos, GpsInjectPosCallback pFunc, void *ctx)
{
    uartPrintf("pos inject\r\n");
    return 0;
}

int gmInjectCep(GpsInjectCepCallback pFunc, void *ctx)
{
    uartPrintf("cep inject\r\n");
    return 0;
}
