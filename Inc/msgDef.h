//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#ifndef __MSG_DEF_H
#define __MSG_DEF_H
#include "cmsis_os.h"

// ==== Enumeration, structures, defines ====

typedef struct {
    void (*cbk) (void * ctx, ...);
    /* Callback function pointer for asynchronous API */

    uint32_t *ctx;
    /* context from NTGR */

    uint16_t id;
    /*  Message Id decide which function will be launched */
    
    uint16_t len;
    /* Length of param */
} msg_hdr;

typedef struct {
	msg_hdr hdr;
	uint8_t  param[1];
} trkMsg;

/*	Message Id */
/*	Configuration Management: 0x0000-0x00FF */

/*	File Management: 0x01000-0x01FF */
/*	Power Management: 0x0200-0x02FF */
#define PM_MSG_BATT_TIMEOUT 0x0200
#define PM_MSG_KEY_TIMEOUT  0x0201
#define PM_MSG_KEY_RLS      0x0210
#define PM_MSG_KEY_PRESS  	0x0211
/*	Sensor Management: 0x0300-0x03FF */
#define SM_MSG_INITIAL		0x0300
#define SM_MSG_TEMP_TM		0x0301
#define SM_MSG_ACCE_TM		0x0302
#define SM_MSG_TEMP_ISR		0x0303
#define SM_MSG_ACCE_ISR1	0x0304
#define SM_MSG_ACCE_ISR2	0x0305
#define SM_MSG_TEMP_QUERY	0x0306
#define SM_MSG_ACCE_QUERY	0x0307

/*	Modem Management: 0x0400-0x04FF */
/*	Wi-Fi Management: 0x0500-0x05FF */
/*	GPS Management: 0x0600-0x06FF */
/*	BLE Management: 0x0700-0x07FF*/

/*	Testing Application: 0x0800-0x08FF */
#define TEST_MSG_TIMEOUT	0x0800
#define TEST_MSG_FROM_ISR	0x0801

/* Response Message Id = Message Id | RESP_MSG_ID */
#define RESP_MSG_ID 0x8000

// ==== Variables ====
extern osMessageQId testQueueHandle;
extern osMessageQId smQueueHandle;
// ==== Functions ====

#endif /* __MSG_DEF_H */

