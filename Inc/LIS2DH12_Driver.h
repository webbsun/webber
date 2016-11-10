/******************** (C) COPYRIGHT 2016 PEGATRONCORP ********************
* File Name          : LIS2DH12_driver.h
* Author             : BSP Application Team
* Author             : Webber Sun
* Version            : $Revision:$
* Date               : $Date:$
* Description        : Descriptor Header for LIS2DH12_driver.c driver file
*
* HISTORY:
* Date        | Modification                                | Author
* 06/09/2016  | Initial Revision                            | Webber Sun
*
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, PEGATRONCORP SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS2DH12_DRIVER__H
#define __LIS2DH12_DRIVER__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported types ------------------------------------------------------------*/


//these could change accordingly with the architecture

// #ifndef __ARCHDEP__TYPES
// #define __ARCHDEP__TYPES

// typedef unsigned char uint8_t;
// typedef unsigned short int uint16_t;
// typedef short int int16_t;
// typedef signed char int8_t;

// #endif /*__ARCHDEP__TYPES*/

typedef uint8_t LIS2DH12_IntPinConf_t;
typedef uint8_t LIS2DH12_Axis_t;
typedef uint8_t LIS2DH12_Int1Conf_t;
typedef uint8_t LIS2DH12_Int2Conf_t;

//define structure
#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef enum {
  MEMS_SUCCESS				=		0x00,
  MEMS_ERROR				=		0x01	
} status_t;

typedef enum {
  MEMS_ENABLE				=		0x01,
  MEMS_DISABLE				=		0x00	
} State_t;

typedef struct {
  int16_t AXIS_X;
  int16_t AXIS_Y;
  int16_t AXIS_Z;
} AxesRaw_t,Axes_t;

#endif /*__SHARED__TYPES*/

typedef enum {  
  LIS2DH12_ODR_1Hz		        =		0x01,		
  LIS2DH12_ODR_10Hz                      =		0x02,
  LIS2DH12_ODR_25Hz		        =		0x03,
  LIS2DH12_ODR_50Hz		        =		0x04,
  LIS2DH12_ODR_100Hz		        =		0x05,	
  LIS2DH12_ODR_200Hz		        =		0x06,
  LIS2DH12_ODR_400Hz		        =		0x07,
  LIS2DH12_ODR_1620Hz_LP		        =		0x08,
  LIS2DH12_ODR_1344Hz_NP_5367HZ_LP       =		0x09	
} LIS2DH12_ODR_t;

typedef enum {
  LIS2DH12_POWER_DOWN             =		0x00,
  LIS2DH12_LOW_POWER 							=		0x01,
  LIS2DH12_NORMAL									=		0x02,
	LIS2DH12_HIGH_RESOLUTION				=		0x03
} LIS2DH12_Mode_t;

typedef enum {
  LIS2DH12_HPM_NORMAL_MODE_RES           =               0x00,
  LIS2DH12_HPM_REF_SIGNAL                =               0x01,
  LIS2DH12_HPM_NORMAL_MODE               =               0x02,
  LIS2DH12_HPM_AUTORESET_INT             =               0x03
} LIS2DH12_HPFMode_t;

typedef enum {
  LIS2DH12_HPFCF_0                       =               0x00,
  LIS2DH12_HPFCF_1                       =               0x01,
  LIS2DH12_HPFCF_2                       = 		0x02,
  LIS2DH12_HPFCF_3                       =               0x03
} LIS2DH12_HPFCutOffFreq_t;

typedef struct {
  uint16_t AUX_1;
  uint16_t AUX_2;
  uint16_t AUX_3;
} LIS2DH12_Aux123Raw_t;

typedef enum {
  LIS2DH12_FULLSCALE_2                   =               0x00,
  LIS2DH12_FULLSCALE_4                   =               0x01,
  LIS2DH12_FULLSCALE_8                   =               0x02,
  LIS2DH12_FULLSCALE_16                  =               0x03
} LIS2DH12_Fullscale_t;

typedef enum {
  LIS2DH12_BLE_LSB			=		0x00,
  LIS2DH12_BLE_MSB			=		0x01
} LIS2DH12_Endianess_t;

typedef enum {
  LIS2DH12_SELF_TEST_DISABLE             =               0x00,
  LIS2DH12_SELF_TEST_0                   =               0x01,
  LIS2DH12_SELF_TEST_1                   =               0x02
} LIS2DH12_SelfTest_t;

typedef enum {
  LIS2DH12_FIFO_BYPASS_MODE              =               0x00,
  LIS2DH12_FIFO_MODE                     =               0x01,
  LIS2DH12_FIFO_STREAM_MODE              =               0x02,
  LIS2DH12_FIFO_TRIGGER_MODE             =               0x03,
  LIS2DH12_FIFO_DISABLE                  =               0x04
} LIS2DH12_FifoMode_t;

typedef enum {
  LIS2DH12_TRIG_INT1                     =		0x00,
  LIS2DH12_TRIG_INT2 			               =		0x01
} LIS2DH12_TrigInt_t;

typedef enum {
  LIS2DH12_SPI_4_WIRE                    =               0x00,
  LIS2DH12_SPI_3_WIRE                    =               0x01
} LIS2DH12_SPIMode_t;

typedef enum {
  LIS2DH12_X_ENABLE                      =               0x01,
  LIS2DH12_X_DISABLE                     =               0x00,
  LIS2DH12_Y_ENABLE                      =               0x02,
  LIS2DH12_Y_DISABLE                     =               0x00,
  LIS2DH12_Z_ENABLE                      =               0x04,
  LIS2DH12_Z_DISABLE                     =               0x00    
} LIS2DH12_AXISenable_t;

typedef enum {
  LIS2DH12_INT1_6D_4D_DISABLE            =               0x00,
  LIS2DH12_INT1_6D_ENABLE                =               0x01,
  LIS2DH12_INT1_4D_ENABLE                =               0x02 
} LIS2DH12_INT_6D_4D_t;

typedef enum {
  LIS2DH12_UP_SX                         =               0x44,
  LIS2DH12_UP_DX                         =               0x42,
  LIS2DH12_DW_SX                         =               0x41,
  LIS2DH12_DW_DX                         =               0x48,
  LIS2DH12_TOP                           =               0x60,
  LIS2DH12_BOTTOM                        =               0x50
} LIS2DH12_POSITION_6D_t;

typedef enum {
  LIS2DH12_INT_MODE_OR                   =               0x00,
  LIS2DH12_INT_MODE_6D_MOVEMENT          =               0x01,
  LIS2DH12_INT_MODE_AND                  =               0x02,
  LIS2DH12_INT_MODE_6D_POSITION          =               0x03  
} LIS2DH12_Int1Mode_t,LIS2DH12_Int2Mode_t;


//interrupt click response
//  b7 = don't care   b6 = IA  b5 = DClick  b4 = Sclick  b3 = Sign  
//  b2 = z      b1 = y     b0 = x
typedef enum {
LIS2DH12_DCLICK_Z_P                      =               0x24,
LIS2DH12_DCLICK_Z_N                      =               0x2C,
LIS2DH12_SCLICK_Z_P                      =               0x14,
LIS2DH12_SCLICK_Z_N                      =               0x1C,
LIS2DH12_DCLICK_Y_P                      =               0x22,
LIS2DH12_DCLICK_Y_N                      =               0x2A,
LIS2DH12_SCLICK_Y_P                      =               0x12,
LIS2DH12_SCLICK_Y_N											 =							 0x1A,
LIS2DH12_DCLICK_X_P                      =               0x21,
LIS2DH12_DCLICK_X_N                      =               0x29,
LIS2DH12_SCLICK_X_P                      =               0x11,
LIS2DH12_SCLICK_X_N                      =               0x19,
LIS2DH12_NO_CLICK                        =               0x00
} LIS2DH12_Click_Response; 

//TODO: start from here and manage the shared macros etc before this

/* Exported constants --------------------------------------------------------*/

#ifndef __SHARED__CONSTANTS
#define __SHARED__CONSTANTS

#define MEMS_SET                                        0x01
#define MEMS_RESET                                      0x00

#endif /*__SHARED__CONSTANTS*/

/**
* @brief  I2C address.
*/
#define LIS2DH12_I2C_ADDRESS  (uint8_t)0x32

//Register Definition
#define LIS2DH12_WHO_AM_I				0x0F  // device identification register


/**
* @brief Device Identification value.
*/
#define LIS2DH12_WHO_AM_I_VAL         (uint8_t)0x33

// CONTROL REGISTER 1
#define LIS2DH12_CTRL_REG1				0x20
#define LIS2DH12_ODR_BIT				        BIT(4)
#define LIS2DH12_LPEN					BIT(3)
#define LIS2DH12_ZEN					BIT(2)
#define LIS2DH12_YEN					BIT(1)
#define LIS2DH12_XEN					BIT(0)

//CONTROL REGISTER 2
#define LIS2DH12_CTRL_REG2				0x21
#define LIS2DH12_HPM     				BIT(6)
#define LIS2DH12_HPCF					BIT(4)
#define LIS2DH12_FDS					BIT(3)
#define LIS2DH12_HPCLICK					BIT(2)
#define LIS2DH12_HPIS2					BIT(1)
#define LIS2DH12_HPIS1					BIT(0)

//CONTROL REGISTER 3
#define LIS2DH12_CTRL_REG3				0x22
#define LIS2DH12_I1_CLICK				BIT(7)
#define LIS2DH12_I1_AOI1					BIT(6)
#define LIS2DH12_I1_AOI2				        BIT(5)
#define LIS2DH12_I1_DRDY1				BIT(4)
#define LIS2DH12_I1_DRDY2				BIT(3)
#define LIS2DH12_I1_WTM					BIT(2)
#define LIS2DH12_I1_ORUN					BIT(1)

//CONTROL REGISTER 6
#define LIS2DH12_CTRL_REG6				0x25
#define LIS2DH12_I2_CLICK				BIT(7)
#define LIS2DH12_I2_INT1					BIT(6)
#define LIS2DH12_I2_INT2					BIT(5)
#define LIS2DH12_I2_BOOT         			BIT(4)
#define LIS2DH12_P2_ACT         			BIT(2)
#define LIS2DH12_H_LACTIVE				BIT(1)

//TEMPERATURE CONFIG REGISTER
#define LIS2DH12_TEMP_CFG_REG				0x1F
#define LIS2DH12_ADC_PD				        BIT(7)
#define LIS2DH12_TEMP_EN					BIT(6)

//CONTROL REGISTER 4
#define LIS2DH12_CTRL_REG4				0x23
#define LIS2DH12_BDU					BIT(7)
#define LIS2DH12_BLE					BIT(6)
#define LIS2DH12_FS					BIT(4)
#define LIS2DH12_HR					BIT(3)
#define LIS2DH12_ST       				BIT(1)
#define LIS2DH12_SIM					BIT(0)

//CONTROL REGISTER 5
#define LIS2DH12_CTRL_REG5				0x24
#define LIS2DH12_BOOT                                    BIT(7)
#define LIS2DH12_FIFO_EN                                 BIT(6)
#define LIS2DH12_LIR_INT1                                BIT(3)
#define LIS2DH12_D4D_INT1                                BIT(2)
#define LIS2DH12_LIR_INT2                                BIT(1)
#define LIS2DH12_D4D_INT2                                BIT(0)

//REFERENCE/DATA_CAPTURE
#define LIS2DH12_REFERENCE_REG		                0x26
#define LIS2DH12_REF		                	BIT(0)

//STATUS_REG_AXIES
#define LIS2DH12_STATUS_REG				0x27
#define LIS2DH12_ZYXOR                                   BIT(7)
#define LIS2DH12_ZOR                                     BIT(6)
#define LIS2DH12_YOR                                     BIT(5)
#define LIS2DH12_XOR                                     BIT(4)
#define LIS2DH12_ZYXDA                                   BIT(3)
#define LIS2DH12_ZDA                                     BIT(2)
#define LIS2DH12_YDA                                     BIT(1)
#define LIS2DH12_XDA                                     BIT(0)

//STATUS_REG_AUX
#define LIS2DH12_STATUS_AUX				0x07

//INTERRUPT 1 CONFIGURATION
#define LIS2DH12_INT1_CFG				0x30
#define LIS2DH12_ANDOR                                   BIT(7)
#define LIS2DH12_INT_6D                                  BIT(6)
#define LIS2DH12_ZHIE                                    BIT(5)
#define LIS2DH12_ZLIE                                    BIT(4)
#define LIS2DH12_YHIE                                    BIT(3)
#define LIS2DH12_YLIE                                    BIT(2)
#define LIS2DH12_XHIE                                    BIT(1)
#define LIS2DH12_XLIE                                    BIT(0)


//INTERRUPT 2 CONFIGURATION
#define LIS2DH12_INT2_CFG				0x34



//FIFO CONTROL REGISTER
#define LIS2DH12_FIFO_CTRL_REG                           0x2E
#define LIS2DH12_FM                                      BIT(6)
#define LIS2DH12_TR                                      BIT(5)
#define LIS2DH12_FTH                                     BIT(0)

//CONTROL REG3 bit mask
#define LIS2DH12_CLICK_ON_PIN_INT1_ENABLE                0x80
#define LIS2DH12_CLICK_ON_PIN_INT1_DISABLE               0x00
#define LIS2DH12_I1_INT1_ON_PIN_INT1_ENABLE              0x40
#define LIS2DH12_I1_INT1_ON_PIN_INT1_DISABLE             0x00
#define LIS2DH12_I1_INT2_ON_PIN_INT1_ENABLE              0x20
#define LIS2DH12_I1_INT2_ON_PIN_INT1_DISABLE             0x00
#define LIS2DH12_I1_DRDY1_ON_INT1_ENABLE                 0x10
#define LIS2DH12_I1_DRDY1_ON_INT1_DISABLE                0x00
#define LIS2DH12_I1_DRDY2_ON_INT1_ENABLE                 0x08
#define LIS2DH12_I1_DRDY2_ON_INT1_DISABLE                0x00
#define LIS2DH12_WTM_ON_INT1_ENABLE                      0x04
#define LIS2DH12_WTM_ON_INT1_DISABLE                     0x00
#define LIS2DH12_INT1_OVERRUN_ENABLE                     0x02
#define LIS2DH12_INT1_OVERRUN_DISABLE                    0x00

//CONTROL REG6 bit mask
#define LIS2DH12_CLICK_ON_PIN_INT2_ENABLE                0x80
#define LIS2DH12_CLICK_ON_PIN_INT2_DISABLE               0x00
#define LIS2DH12_I2_INT1_ON_PIN_INT2_ENABLE              0x40
#define LIS2DH12_I2_INT1_ON_PIN_INT2_DISABLE             0x00
#define LIS2DH12_I2_INT2_ON_PIN_INT2_ENABLE              0x20
#define LIS2DH12_I2_INT2_ON_PIN_INT2_DISABLE             0x00
#define LIS2DH12_I2_BOOT_ON_INT2_ENABLE                  0x10
#define LIS2DH12_I2_BOOT_ON_INT2_DISABLE                 0x00
#define LIS2DH12_INT_ACTIVE_HIGH                         0x00
#define LIS2DH12_INT_ACTIVE_LOW                          0x02

//INT1_CFG bit mask
#define LIS2DH12_INT1_AND                                0x80
#define LIS2DH12_INT1_OR                                 0x00
#define LIS2DH12_INT1_ZHIE_ENABLE                        0x20
#define LIS2DH12_INT1_ZHIE_DISABLE                       0x00
#define LIS2DH12_INT1_ZLIE_ENABLE                        0x10
#define LIS2DH12_INT1_ZLIE_DISABLE                       0x00
#define LIS2DH12_INT1_YHIE_ENABLE                        0x08
#define LIS2DH12_INT1_YHIE_DISABLE                       0x00
#define LIS2DH12_INT1_YLIE_ENABLE                        0x04
#define LIS2DH12_INT1_YLIE_DISABLE                       0x00
#define LIS2DH12_INT1_XHIE_ENABLE                        0x02
#define LIS2DH12_INT1_XHIE_DISABLE                       0x00
#define LIS2DH12_INT1_XLIE_ENABLE                        0x01
#define LIS2DH12_INT1_XLIE_DISABLE                       0x00

//INT1_SRC bit mask
#define LIS2DH12_INT1_SRC_IA                             0x40
#define LIS2DH12_INT1_SRC_ZH                             0x20
#define LIS2DH12_INT1_SRC_ZL                             0x10
#define LIS2DH12_INT1_SRC_YH                             0x08
#define LIS2DH12_INT1_SRC_YL                             0x04
#define LIS2DH12_INT1_SRC_XH                             0x02
#define LIS2DH12_INT1_SRC_XL                             0x01

//INT1 REGISTERS
#define LIS2DH12_INT1_THS                                0x32
#define LIS2DH12_INT1_DURATION                           0x33

//INTERRUPT 1 SOURCE REGISTER
#define LIS2DH12_INT1_SRC				0x31

//INT1 REGISTERS
#define LIS2DH12_INT2_THS                                0x36
#define LIS2DH12_INT2_DURATION                           0x37

//INTERRUPT 2 SOURCE REGISTER
#define LIS2DH12_INT2_SRC					0x35


//FIFO Source Register bit Mask
#define LIS2DH12_FIFO_SRC_WTM                            0x80
#define LIS2DH12_FIFO_SRC_OVRUN                          0x40
#define LIS2DH12_FIFO_SRC_EMPTY                          0x20
  
//INTERRUPT CLICK REGISTER
#define LIS2DH12_CLICK_CFG				0x38
//INTERRUPT CLICK CONFIGURATION bit mask
#define LIS2DH12_ZD_ENABLE                               0x20
#define LIS2DH12_ZD_DISABLE                              0x00
#define LIS2DH12_ZS_ENABLE                               0x10
#define LIS2DH12_ZS_DISABLE                              0x00
#define LIS2DH12_YD_ENABLE                               0x08
#define LIS2DH12_YD_DISABLE                              0x00
#define LIS2DH12_YS_ENABLE                               0x04
#define LIS2DH12_YS_DISABLE                              0x00
#define LIS2DH12_XD_ENABLE                               0x02
#define LIS2DH12_XD_DISABLE                              0x00
#define LIS2DH12_XS_ENABLE                               0x01
#define LIS2DH12_XS_DISABLE                              0x00

//INTERRUPT CLICK SOURCE REGISTER
#define LIS2DH12_CLICK_SRC                               0x39
//INTERRUPT CLICK SOURCE REGISTER bit mask
#define LIS2DH12_IA                                      0x40
#define LIS2DH12_DCLICK                                  0x20
#define LIS2DH12_SCLICK                                  0x10
#define LIS2DH12_CLICK_SIGN                              0x08
#define LIS2DH12_CLICK_Z                                 0x04
#define LIS2DH12_CLICK_Y                                 0x02
#define LIS2DH12_CLICK_X                                 0x01

//Click-click Register
#define LIS2DH12_CLICK_THS                               0x3A
#define LIS2DH12_TIME_LIMIT                              0x3B
#define LIS2DH12_TIME_LATENCY                            0x3C
#define LIS2DH12_TIME_WINDOW                             0x3D

//OUTPUT REGISTER
#define LIS2DH12_OUT_X_L					0x28
#define LIS2DH12_OUT_X_H					0x29
#define LIS2DH12_OUT_Y_L					0x2A
#define LIS2DH12_OUT_Y_H					0x2B
#define LIS2DH12_OUT_Z_L					0x2C
#define LIS2DH12_OUT_Z_H					0x2D

//AUX REGISTER
#define LIS2DH12_OUT_1_L					0x08
#define LIS2DH12_OUT_1_H					0x09
#define LIS2DH12_OUT_2_L					0x0A
#define LIS2DH12_OUT_2_H					0x0B
#define LIS2DH12_OUT_3_L					0x0C
#define LIS2DH12_OUT_3_H					0x0D






//STATUS REGISTER bit mask
#define LIS2DH12_STATUS_REG_ZYXOR                        0x80    // 1	:	new data set has over written the previous one
							// 0	:	no overrun has occurred (default)	
#define LIS2DH12_STATUS_REG_ZOR                          0x40    // 0	:	no overrun has occurred (default)
							// 1	:	new Z-axis data has over written the previous one
#define LIS2DH12_STATUS_REG_YOR                          0x20    // 0	:	no overrun has occurred (default)
							// 1	:	new Y-axis data has over written the previous one
#define LIS2DH12_STATUS_REG_XOR                          0x10    // 0	:	no overrun has occurred (default)
							// 1	:	new X-axis data has over written the previous one
#define LIS2DH12_STATUS_REG_ZYXDA                        0x08    // 0	:	a new set of data is not yet avvious one
                                                        // 1	:	a new set of data is available 
#define LIS2DH12_STATUS_REG_ZDA                          0x04    // 0	:	a new data for the Z-Axis is not availvious one
                                                        // 1	:	a new data for the Z-Axis is available
#define LIS2DH12_STATUS_REG_YDA                          0x02    // 0	:	a new data for the Y-Axis is not available
                                                        // 1	:	a new data for the Y-Axis is available
#define LIS2DH12_STATUS_REG_XDA                          0x01    // 0	:	a new data for the X-Axis is not available

#define LIS2DH12_DATAREADY_BIT                           LIS2DH12_STATUS_REG_ZYXDA


//STATUS AUX REGISTER bit mask
#define LIS2DH12_STATUS_AUX_321OR                         0x80
#define LIS2DH12_STATUS_AUX_3OR                           0x40
#define LIS2DH12_STATUS_AUX_2OR                           0x20
#define LIS2DH12_STATUS_AUX_1OR                           0x10
#define LIS2DH12_STATUS_AUX_321DA                         0x08
#define LIS2DH12_STATUS_AUX_3DA                           0x04
#define LIS2DH12_STATUS_AUX_2DA                           0x02
#define LIS2DH12_STATUS_AUX_1DA                           0x01

#define LIS2DH12_MEMS_I2C_ADDRESS			        0x33

//FIFO REGISTERS
#define LIS2DH12_FIFO_CTRL_REG			        0x2E
#define LIS2DH12_FIFO_SRC_REG			        0x2F


/* Exported macro ------------------------------------------------------------*/

#ifndef __SHARED__MACROS

#define __SHARED__MACROS
#define ValBit(VAR,Place)         (VAR & (1<<Place))
#define BIT(x) ( (x) )

#endif /*__SHARED__MACROS*/

/* Exported functions --------------------------------------------------------*/
//Sensor Configuration Functions
status_t LIS2DH12_SetODR(LIS2DH12_ODR_t ov);
status_t LIS2DH12_SetMode(LIS2DH12_Mode_t md);
status_t LIS2DH12_SetAxis(LIS2DH12_Axis_t axis);
status_t LIS2DH12_SetFullScale(LIS2DH12_Fullscale_t fs);
status_t LIS2DH12_SetBDU(State_t bdu);
status_t LIS2DH12_SetBLE(LIS2DH12_Endianess_t ble);
status_t LIS2DH12_SetSelfTest(LIS2DH12_SelfTest_t st);
status_t LIS2DH12_SetTemperature(State_t state);
status_t LIS2DH12_SetADCAux(State_t state);

//Filtering Functions
status_t LIS2DH12_HPFClickEnable(State_t hpfe);
status_t LIS2DH12_HPFAOI1Enable(State_t hpfe);
status_t LIS2DH12_HPFAOI2Enable(State_t hpfe);
status_t LIS2DH12_SetHPFMode(LIS2DH12_HPFMode_t hpf);
status_t LIS2DH12_SetHPFCutOFF(LIS2DH12_HPFCutOffFreq_t hpf);
status_t LIS2DH12_SetFilterDataSel(State_t state);

//Interrupt Functions
status_t LIS2DH12_SetInt1Pin(LIS2DH12_IntPinConf_t pinConf);
status_t LIS2DH12_SetInt2Pin(LIS2DH12_IntPinConf_t pinConf);
status_t LIS2DH12_Int1LatchEnable(State_t latch);
status_t LIS2DH12_ResetInt1Latch(void);
status_t LIS2DH12_SetIntConfiguration(LIS2DH12_Int1Conf_t ic);
status_t LIS2DH12_SetInt1Threshold(uint8_t ths);
status_t LIS2DH12_SetInt1Duration(LIS2DH12_Int1Conf_t id);
status_t LIS2DH12_SetIntMode(LIS2DH12_Int1Mode_t ic);
status_t LIS2DH12_SetClickCFG(uint8_t status);
status_t LIS2DH12_SetInt6D4DConfiguration(LIS2DH12_INT_6D_4D_t ic);
status_t LIS2DH12_GetInt1Src(uint8_t* val);
status_t LIS2DH12_GetInt1SrcBit(uint8_t statusBIT, uint8_t* val);

status_t LIS2DH12_GetInt2Src(uint8_t* val);
status_t LIS2DH12_SetInt2Threshold(uint8_t ths);
status_t LIS2DH12_SetInt2Configuration(LIS2DH12_Int2Conf_t ic);
status_t LIS2DH12_SetInt2Mode(LIS2DH12_Int2Mode_t ic);

//FIFO Functions
status_t LIS2DH12_FIFOModeEnable(LIS2DH12_FifoMode_t fm);
status_t LIS2DH12_SetWaterMark(uint8_t wtm);
status_t LIS2DH12_SetTriggerInt(LIS2DH12_TrigInt_t tr);
status_t LIS2DH12_GetFifoSourceReg(uint8_t* val);
status_t LIS2DH12_GetFifoSourceBit(uint8_t statusBIT, uint8_t* val);
status_t LIS2DH12_GetFifoSourceFSS(uint8_t* val);

//Other Reading Functions
status_t LIS2DH12_GetStatusReg(uint8_t* val);
status_t LIS2DH12_GetStatusBit(uint8_t statusBIT, uint8_t* val);
status_t LIS2DH12_GetStatusAUXBit(uint8_t statusBIT, uint8_t* val);
status_t LIS2DH12_GetStatusAUX(uint8_t* val);
status_t LIS2DH12_GetAccAxesRaw(AxesRaw_t* buff);
status_t LIS2DH12_GetAuxRaw(LIS2DH12_Aux123Raw_t* buff);
status_t LIS2DH12_GetClickResponse(uint8_t* val);
status_t LIS2DH12_GetTempRaw(int8_t* val);
status_t LIS2DH12_GetWHO_AM_I(uint8_t* val);
status_t LIS2DH12_Get6DPosition(uint8_t* val);
status_t LIS2DH12_GetAccAxes(Axes_t* buff);
//Generic
// i.e. uint8_t LIS2DH12_ReadReg(uint8_t Reg, uint8_t* Data);
// i.e. uint8_t LIS2DH12_WriteReg(uint8_t Reg, uint8_t Data);


#endif /* __LIS2DH12_H */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/



