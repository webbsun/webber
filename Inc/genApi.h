//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#ifndef __GEN_API_H
#define __GEN_API_H
#include "cmsis_os.h"

// ==== Enumeration, structures, defines ====
#define MAX_LOG_LEN	32
/*	Debug Log level	*/
#define _CRIT_LOG_	0x80000000
#define _WARN_LOG_	0x40000000
#define _DEBUG_LOG_	0x20000000
#define _INFO_LOG_	0x10000000

/*	Module Id for Debug Log 
	Pega Zone:
		0x00000001, 0x00000002, ..., 0x00008000
	NTGR Zone:
		0x00010000, 0x00020000, ..., 0x08000000
*/
#define _CM_LOG_	0x00000001	/*	Configuration Management	*/
#define _FM_LOG_	0x00000002	/*	File Management	*/
#define _PM_LOG_	0x00000004	/*	Power Management	*/
#define _SM_LOG_	0x00000008	/*	Sensor Management	*/

#define _MM_LOG_	0x00000010	/*	Modem Management	*/
#define _WM_LOG_	0x00000020	/*	Wi-Fi Management	*/
#define _GM_LOG_	0x00000040	/*	GPS Management	*/
#define _BM_LOG_	0x00000080	/*	BLE Management	*/

#define _MF_LOG_	0x00000100	/*	Modem Framework	*/
#define _WF_LOG_	0x00000200	/*	Wi-Fi Framework	*/
#define _GF_LOG_	0x00000400	/*	GPS Framework	*/
#define _BF_LOG_	0x00000800	/*	BLE Framework	*/

#define _BAT_LOG_	0x00001000	/*	Battery Framework	*/
#define	_TA_LOG_	0x00002000	/*	Testing Application	*/
#define _GEN_LOG_	0x00008000	/*	General APIs	*/

// ==== Variables ====

extern osMutexId Mutex_I2C;


// ==== Functions ====
int gLog(uint32_t mPlc, char *pMsgData, ...);
int gSwVer(uint32_t *pMaj, uint32_t *pMin);
int ledCtrl(_Bool blue, _Bool amber);
int bzCtrl(_Bool opcode);

void *gMem32Alloc(void);
void *gMem64Alloc(void);
int gMemFree(void *pBuf);

int gLogGetMask(uint32_t *conMask, uint32_t *logMask);
int gLogSetMask(uint32_t conMask, uint32_t logMask);
int gFwVerify(char *path);

void odmEntry(void);

#endif /* __GEN_API_H */
