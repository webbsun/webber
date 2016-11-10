//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#include "bmApi.h"

#include "msgDef.h"
#include "errDef.h"
#include "genApi.h"

/***** Configuration *****/
int bmSetDeviceName(char *name, BleSetDeviceNameCallback *pFunc, void *ctx)
{
    return E_OK;
}
int bmGetDeviceName(BleGetDeviceNameCallback *pFunc, void *ctx)
{
    return E_OK;
}
int bmSetPairingkey(char *key, BleSetPairingKeyCallback *pFunc, void *ctx)
{
    return E_OK;
}
int bmSetRssiThreshold(uint8_t rssiThreshold, BleSetRssiThresholdCallback *pFunc, void *ctx)
{
    return E_OK;
}
int bmSetPreferredWhitelistDevice(char *bdaddr, BleSetPreferredWhitelistDeviceCallback *pFunc, void *ctx)
{
    return E_OK;
}
int bmRegisterService(char *serviceUuid, BleServiceDataCallback *pFunc, void *ctx)
{
    return E_OK;
}
int bmDeregisterService(int instanceId, char *serviceUuid)
{
    return E_OK;
}
int bmRegisterEvtCallback(BleEvtCallback *pFunc, void *ctx)
{
    return E_OK;
}

/***** Functional *****/
int bmStartAdvertising(ble_adv_config_t config, ble_adv_data_t *advData, BleAdvertisingCallback *pFunc, void *ctx)
{
    return E_OK;
}
int bmStopAdvertising(BleAdvertisingCallback *pFunc, void *ctx)
{
    return E_OK;
}
int bmSendData(int instanceId, char *serviceUuid, ble_data_t *data, BleSendDataCallback *pFunc, void *ctx)
{
    return E_OK;
}
int bmDisconnect(void)
{
    return E_OK;
}
int bmGetPairedDeviceList(BleGetPairedDeviceListCallback *pFunc, void *ctx)
{
    return E_OK;
}
int bmAcceptPairing(_Bool accept)
{
    return E_OK;
}
int bmRemovePairing(char *bdaddr, BleRemovePairingCallback *pFunc, void *ctx)
{
    return E_OK;
}
