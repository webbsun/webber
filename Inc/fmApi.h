//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//
#ifndef __FMAPI_H
#define __FMAPI_H
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"



/* MX25R3235F MXIC SPI Flash */
/* Size of the flash */
#define QSPI_FLASH_SIZE                      ((uint8_t)22)
#define QSPI_PAGE_SIZE                       ((uint8_t)256)
#define QSPI_SECTOR_SIZE                     4096
#define QSPI_SECTOR_COUNT                   864U //863 //864 //768
#define BLOCK_SIZE 							 						65536
#define FLASH_PAGES_PER_SECTOR SECTOR_SIZE/PAGE_SIZE

#define SPIFLASH4M_ID 						 ((uint32_t)0x001628C2)
#define SPIFLASH8M_ID 						 ((uint32_t)0x001728C2)

/* Reset Operations */
#define RESET_ENABLE_CMD                     ((uint8_t)0x66)
#define RESET_MEMORY_CMD                     ((uint8_t)0x99)

/* Identification Operations */
#define RDID_CMD                          	  ((uint8_t)0x9F)
#define RES_CMD                          	  ((uint8_t)0xAB)
#define REMS_CMD                          	  ((uint8_t)0x90)
#define READ_SFDP_CMD    					  ((uint8_t)0x5A)

/* Read Operations */
#define READ_CMD                              ((uint8_t)0x03)
#define FAST_READ_CMD                         ((uint8_t)0x0B)

#define DUAL_OUT_FAST_READ_CMD                ((uint8_t)0x3B)
#define DUAL_INOUT_FAST_READ_CMD              ((uint8_t)0xBB)

#define QUAD_4READ_CMD             			  ((uint8_t)0xEB)
#define QUAD_QREAD_CMD            			  ((uint8_t)0x6B)


/* Write Operations */
#define WRITE_ENABLE_CMD                      ((uint8_t)0x06)
#define WRITE_DISABLE_CMD                     ((uint8_t)0x04)

/* Register Operations */
#define READ_STATUS_REG_CMD                   ((uint8_t)0x05)
#define READ_CONFIG_REG_CMD                   ((uint8_t)0x15)
#define WRITE_STATUS_REG_CMD                  ((uint8_t)0x01)

#define READ_LOCK_REG_CMD                     ((uint8_t)0x2B)
#define WRITE_LOCK_REG_CMD                    ((uint8_t)0x2F)


/* Program Operations */
#define PAGE_PROG_CMD                         ((uint8_t)0x02)
#define QUAD_PAGE_PROG_CMD            		  ((uint8_t)0x38)


/* Erase Operations */
#define SECTOR_ERASE_CMD                      ((uint8_t)0x20)
#define BLOCK_ERASE_CMD                       ((uint8_t)0x52)
#define BULK_ERASE_CMD                        ((uint8_t)0xD8)
#define CHIP_ERASE_CMD                        ((uint8_t)0xC7)

#define PROG_ERASE_RESUME_CMD                 ((uint8_t)0x7A) /*0x30*/
#define PROG_ERASE_SUSPEND_CMD                ((uint8_t)0x75) /*0xB0*/

/* One-Time Programmable Operations */
#define ENTER_OTP_CMD                         ((uint8_t)0xB1)
#define EXIT_OTP_CMD                          ((uint8_t)0xC1)

#define DEEP_POWER_CMD                        ((uint8_t)0xB9)
#define SET_BURST_LEN_CMD                     ((uint8_t)0xC0)



/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_FREAD             1
#define DUMMY_CLOCK_CYCLES_4READ             6



/* End address of the QSPI memory */
#define QSPI_END_ADDR              (1 << QSPI_FLASH_SIZE)

/* Size of buffers */
//#define BUFFERSIZE                 (COUNTOF(aTxBuffer) - 1)
#define BUFFERSIZE 10
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)        (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
	
// ==== Variables ====
extern osMutexId Mutex_QSPI;

/* Exported functions ------------------------------------------------------- */
extern int QSPI_Read_IT(QSPI_HandleTypeDef *pHspi, uint32_t addr,uint32_t Bcount, uint8_t* data);
extern int QSPI_Write_IT(QSPI_HandleTypeDef *hqspi, uint32_t addr,uint32_t Bcount, uint8_t* data);
extern int QSPI_SectorErase_IT(QSPI_HandleTypeDef *pHspi, uint32_t addr);
extern int QSPI_BlockErase_IT(QSPI_HandleTypeDef *hqspi, uint32_t addr);
extern int QSPI_Read(QSPI_HandleTypeDef *pHspi, uint32_t addr,uint32_t Bcount, uint8_t* data);
extern int QSPI_Write(QSPI_HandleTypeDef *hqspi, uint32_t addr,uint32_t Bcount, uint8_t* data);
extern int QSPI_SectorErase(QSPI_HandleTypeDef *hqspi, uint32_t addr);
extern int QSPI_BlockErase(QSPI_HandleTypeDef *hqspi, uint32_t addr);
extern int QSPI_RDID(QSPI_HandleTypeDef *hqspi, uint32_t* data);
extern int QSPI_RDSR(QSPI_HandleTypeDef *hqspi, uint8_t* data);
extern int QSPI_WRSR(QSPI_HandleTypeDef *hqspi, uint8_t data);
extern int QSPI_DP(QSPI_HandleTypeDef *hqspi);
extern int QSPI_DPResume(QSPI_HandleTypeDef *hqspi);
extern void FlashTestApp(char *arg);
extern void fmSampCode(char *arg);


/* Exported functions for Netgear API ------------------------------------------------------- */
extern int fmRead(uint32_t addr,uint32_t Blen, uint8_t* data);
extern int fmProg(uint32_t addr,uint32_t Blen, uint8_t* data);
extern int fmBkErase(uint32_t addr);
extern int fmSectErase(uint32_t addr);
extern int fmQuadEnable(void);
extern int fmRDID(uint32_t* data);
#endif
