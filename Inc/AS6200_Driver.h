
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AS6200_DRIVER__H
#define __AS6200_DRIVER__H

#include <stdint.h>

/* Uncomment the line below to expanse the "assert_param" macro in the drivers code */
#define USE_FULL_ASSERT_AS6200

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT_AS6200


#ifdef __cplusplus
extern "C" {
#endif



/**
* @brief  Error code type.
*/
typedef enum {AS6200_OK = (uint8_t)0, AS6200_ERROR = !AS6200_OK} AS6200_Error_et;

/**
* @brief  State type.
*/
typedef enum {AS6200_DISABLE = (uint8_t)0, AS6200_ENABLE = !AS6200_DISABLE} AS6200_State_et;
#define IS_AS6200_State(MODE) ((MODE == AS6200_ENABLE) || (MODE == AS6200_DISABLE))

/**
* @brief  Bit status type.
*/
typedef enum {AS6200_RESET = (uint8_t)0, AS6200_SET = !AS6200_RESET} AS6200_BitStatus_et;
#define IS_AS6200_BitStatus(MODE) ((MODE == AS6200_RESET) || (MODE == AS6200_SET))


/**
* @brief  Temperature average.
*/
typedef enum
{
  AS6200_AVGT_1         = (uint8_t)0x00,        /*!< Internal average on 2 samples */
  AS6200_AVGT_2         = (uint8_t)0x01,        /*!< Internal average on 4 samples */
  AS6200_AVGT_4         = (uint8_t)0x02,        /*!< Internal average on 8 samples */
  AS6200_AVGT_6         = (uint8_t)0x03,        /*!< Internal average on 16 samples */
  
} AS6200_AvgT_et;


/**
* @brief  Output data rate configuration.
*/
typedef enum
{
  AS6200_ODR_0_25HZ    = (uint8_t)0x00,         /*!< Output Data Rate: one shot */
  AS6200_ODR_1HZ       = (uint8_t)0x01,         /*!< Output Data Rate: 1Hz */
  AS6200_ODR_4HZ  	   = (uint8_t)0x02,         /*!< Output Data Rate: 7Hz */
  AS6200_ODR_8HZ   	   = (uint8_t)0x03,         /*!< Output Data Rate: 12.5Hz */
} AS6200_Odr_et;

/**
* @brief  AS6200 Init structure definition.
*/
typedef struct
{

  AS6200_Avgt_et        avg_t;            /*!< Temperature average */
  AS6200_Odr_et         odr;              /*!< Output data rate */
 
} AS6200_Init_st;

/**
* @}
*/


/* Exported Constants ---------------------------------------------------------*/
/** @defgroup AS6200_Exported_Constants
* @{
*/
/**
* @brief  Bitfield positioning.
*/
#define AS6200_BIT(x) ((uint16_t)x)

/**
* @brief  I2C address.
*/
#define AS6200_I2C_ADDRESS  (uint8_t)0x90

#define AS6200_CONFIG      (uint8_t)0x01

#define AS6200_CR_BIT         AS6200_BIT(6)
#define AS6200_CR_MASK       (uint16_t)0x00C0

#define AS6200_SM_BIT          AS6200_BIT(8)
#define AS6200_SM_MASK        (uint16_t)0x0100

#define AS6200_IM_BIT        AS6200_BIT(9)
#define AS6200_IM_MASK      (uint16_t)0x0200

#define AS6200_POL_BIT        AS6200_BIT(10)
#define AS6200_POL_MASK      (uint16_t)0x0400

#define AS6200_CF_BIT        AS6200_BIT(11)
#define AS6200_CF_MASK      (uint16_t)0x0C00

#define AS6200_SS_BIT     	AS6200_BIT(15)
#define AS6200_SS_MASK  	(uint16_t)0x8000


#define AS6200_TLOW     (uint8_t)0x02

#define AS6200_THIGH     (uint8_t)0x03




/* Exported Functions -------------------------------------------------------------*/
/** @defgroup AS6200_Exported_Functions
* @{
*/

AS6200_Error_et AS6200_Get_Temperature(void *handle, float *value);
AS6200_Error_et AS6200_Get_TemperatureRaw(void *handle, int16_t* value);
AS6200_Error_et AS6200_Set_PowerDownMode(void *handle, AS6200_BitStatus_et status);
AS6200_Error_et AS6200_Get_PowerDownMode(void *handle, AS6200_BitStatus_et* status);
AS6200_Error_et AS6200_Set_Odr(void *handle, AS6200_Odr_et odr);
AS6200_Error_et AS6200_Get_Odr(void *handle, AS6200_Odr_et* odr);
AS6200_Error_et AS6200_Set_AvgT(void *handle, AS6200_AvgT_et avgt);
AS6200_Error_et AS6200_Get_OAvgT(void *handle, AS6200_AvgT_et *avgt);
AS6200_Error_et AS6200_StartOneShotMeasurement(void *handle);
AS6200_Error_et AS6200_Set_IrqEnable(void *handle, , AS6200_BitStatus_et* status);
AS6200_Error_et AS6200_Get_IrqEnable(void *handle, AS6200_BitStatus_et* status);
AS6200_Error_et AS6200_Set_IrqActHigh(void *handle, AS6200_BitStatus_et status);
AS6200_Error_et AS6200_Get_IrqActHigh(void *handle, AS6200_BitStatus_et* status);
AS6200_Error_et AS6200_Set_THIGH(void *handle,float Ths);
AS6200_Error_et AS6200_Set_TLOW(void *handle,float Ths);


#ifdef __cplusplus
}
#endif


/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

#endif /* __AS6200_DRIVER__H */

/******************* (C) COPYRIGHT 2016 Pegatroncorp *****END OF FILE****/

