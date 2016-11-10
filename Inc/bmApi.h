//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#ifndef __BMAPI_H
#define __BMAPI_H

#include <stdint.h>


typedef struct
{
    uint8_t dataLength;
    uint8_t * data;
} ble_data_t;

typedef struct {
    int fastAdvInterval;        /* in milliseconds, multiple of 0.625, 20ms to 10240ms */
    int fastAdvTimeout;         /* in seconds, default 30 */
    int slowAdvInterval;        /* in milliseconds, multiple of 0.625, 20ms to 10240ms */
    int slowAdvTimeout;         /* in seconds, default 60 */
    uint8_t advChanMask;        /* a bit field of advertising channels, 1 to 7, refer to BT spec v4 Volume 2 Part E section 7.8.5 */
} ble_adv_config_t;

typedef struct {
    char *deviceName;                                   /* CSS Part A 1.2 */
    uint8_t manufacturerSpecificDataLength;
    uint8_t *manufacturerSpecificData ;                 /* CSS Part A 1.4,  first 2 bytes are manufacturer Id */
} ble_adv_data_t;

typedef void (*BleSetDeviceNameCallback)(int status, void *ctx);
typedef void (*BleGetDeviceNameCallback)(int status, char *name, void *ctx);
typedef void (*BleSetPairingKeyCallback)(int status, void *ctx);
typedef void (*BleSetRssiThresholdCallback)(int status, void *ctx);
typedef void (*BleSetPreferredWhitelistDeviceCallback)(int status, void *ctx);
typedef void (*BleServiceDataCallback)(char *serviceUuid, ble_data_t *data, void *ctx);
typedef void (*BleEvtCallback)(int event, void *eventData, void *ctx);
typedef void (*BleAdvertisingCallback)(int status, void *ctx);
typedef void (*BleSendDataCallback)(int status, void *ctx);
typedef void (*BleGetPairedDeviceListCallback)(int status, uint8_t numDevices, char *bdaddr[], void *ctx);
typedef void (*BleRemovePairingCallback)(int status, char *bdaddr, void *ctx);


/* Exported functions for Netgear API ------------------------------------------------------- */
/***** Configuration *****/
extern int bmSetDeviceName(char *name, BleSetDeviceNameCallback *pFunc, void *ctx);
extern int bmGetDeviceName(BleGetDeviceNameCallback *pFunc, void *ctx);
extern int bmSetPairingkey(char *key, BleSetPairingKeyCallback *pFunc, void *ctx);
extern int bmSetRssiThreshold(uint8_t rssiThreshold, BleSetRssiThresholdCallback *pFunc, void *ctx);
extern int bmSetPreferredWhitelistDevice(char *bdaddr, BleSetPreferredWhitelistDeviceCallback *pFunc, void *ctx);
extern int bmRegisterService(char *serviceUuid, BleServiceDataCallback *pFunc, void *ctx);
extern int bmDeregisterService(int instanceId, char *serviceUuid);
extern int bmRegisterEvtCallback(BleEvtCallback *pFunc, void *ctx);

/***** Functional *****/
extern int bmStartAdvertising(ble_adv_config_t config, ble_adv_data_t *advData, BleAdvertisingCallback *pFunc, void *ctx);
extern int bmStopAdvertising(BleAdvertisingCallback *pFunc, void *ctx);
extern int bmSendData(int instanceId, char *serviceUuid, ble_data_t *data, BleSendDataCallback *pFunc, void *ctx);
extern int bmDisconnect(void);
extern int bmGetPairedDeviceList(BleGetPairedDeviceListCallback *pFunc, void *ctx);
extern int bmAcceptPairing(_Bool accept);
extern int bmRemovePairing(char *bdaddr, BleRemovePairingCallback *pFunc, void *ctx);

#endif
