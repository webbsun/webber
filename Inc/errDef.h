//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#ifndef __ERR_DEF_H
#define __ERR_DEF_H

// ==== Enumeration, structures, defines ====
#define E_OK			0
#define E_RANGE			-1	/* Argument range */
#define E_ILLEGAL		-2	/* Illegal Argument */
#define E_LEN			-3	/* data length */
#define E_BUSY			-4	/* Service busy */
#define E_TIMEOUT		-5	/* Service timeout */
#define E_HAL			-6	/* HAL driver return error */
#define E_MEM			-7	/* Can't allocate memory */
#define E_ID			-8	/* Message id no support */
#define E_PERAL_DEVICE	-9	/* peripheral device error */
#define E_HW			-10 /* Hardware level error */
#define E_FS			-11 /* File system error */
#define E_NINIT		-12 /* NO initial */

// ==== Variables ====

// ==== Functions ====

#endif /* __ERR_DEF_H */



