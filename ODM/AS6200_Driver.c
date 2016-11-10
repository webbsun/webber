
/* Includes ------------------------------------------------------------------*/
#include "AS6200_Driver.h"

#ifdef  USE_FULL_ASSERT_AS6200
#include <stdio.h>
#endif
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_i2c_ex.h"


int AS6200_ReadReg( void *handle, uint8_t RegAddr, uint16_t NumByteToRead, uint8_t *Data )
{
	
 HAL_StatusTypeDef status = HAL_I2C_Mem_Read( handle, 0x90, ( uint16_t )RegAddr, I2C_MEMADD_SIZE_8BIT, Data, NumByteToRead, 0xfffff );
                             
  if( status != HAL_OK ) 
    return -1;
  else
    return 0;
}



int AS6200_WriteReg( void *handle, uint8_t RegAddr, uint16_t NumByteToWrite, uint8_t *Data )
{
  
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write( handle, AS6200_I2C_ADDRESS, ( uint16_t )RegAddr, I2C_MEMADD_SIZE_8BIT, Data, NumByteToWrite, 0xfffff );
                              
  /* Check the communication status */
  if( status != HAL_OK ) 
    return AS6200_ERROR;
  else
    return AS6200_OK;
}

/**
* @}
*/



/** @defgroup AS6200_Public_Functions
* @{
*/


AS6200_Error_et AS6200_Set_InitConfig(void *handle, AS6200_Init_st* pxInit)
{
  uint8_t buffer[3];
  
  AS6200_assert_param(IS_AS6200_AVGH(pxInit->avg_h));
  AS6200_assert_param(IS_AS6200_AVGT(pxInit->avg_t));
  AS6200_assert_param(IS_AS6200_ODR(pxInit->odr));
  AS6200_assert_param(IS_AS6200_State(pxInit->bdu_status));
  AS6200_assert_param(IS_AS6200_State(pxInit->heater_status));
  
  AS6200_assert_param(IS_AS6200_DrdyLevelType(pxInit->irq_level));
  AS6200_assert_param(IS_AS6200_OutputType(pxInit->irq_output_type));
  AS6200_assert_param(IS_AS6200_State(pxInit->irq_enable));
  
  if(AS6200_ReadReg(handle, AS6200_AV_CONF_REG, 1, buffer))
    return AS6200_ERROR;
    
  buffer[0] &= ~(AS6200_AVGH_MASK | AS6200_AVGT_MASK);
  buffer[0] |= (uint8_t)pxInit->avg_h;
  buffer[0] |= (uint8_t)pxInit->avg_t;
  
  if(AS6200_WriteReg(handle, AS6200_AV_CONF_REG, 1, buffer))
    return AS6200_ERROR;
    
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 3, buffer))
    return AS6200_ERROR;
    
  buffer[0] &= ~(AS6200_BDU_MASK | AS6200_ODR_MASK);
  buffer[0] |= (uint8_t)pxInit->odr;
  buffer[0] |= ((uint8_t)pxInit->bdu_status) << AS6200_BDU_BIT;
  
  buffer[1] &= ~AS6200_HEATHER_BIT;
  buffer[1] |= ((uint8_t)pxInit->heater_status) << AS6200_HEATHER_BIT;
  
  buffer[2] &= ~(AS6200_DRDY_H_L_MASK | AS6200_PP_OD_MASK | AS6200_DRDY_MASK);
  buffer[2] |= ((uint8_t)pxInit->irq_level) << AS6200_DRDY_H_L_BIT;
  buffer[2] |= (uint8_t)pxInit->irq_output_type;
  buffer[2] |= ((uint8_t)pxInit->irq_enable) << AS6200_DRDY_BIT;
  
  if(AS6200_WriteReg(handle, AS6200_CONFIG, 3, buffer))
    return AS6200_ERROR;
    
  return AS6200_OK;
}





AS6200_Error_et AS6200_DeInit(void *handle)
{
  uint8_t buffer[4];
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, buffer))
    return AS6200_ERROR;
    
  /* AS6200 in power down */
  buffer[0] |= 0x01 << AS6200_PD_BIT;
  
  /* Make AS6200 boot */
  buffer[1] |= 0x01 << AS6200_BOOT_BIT;
  
  if(AS6200_WriteReg(handle, AS6200_CONFIG, 2, buffer))
    return AS6200_ERROR;
    
  /* Dump of data output */
  if(AS6200_ReadReg(handle, AS6200_HR_OUT_L_REG, 4, buffer))
    return AS6200_ERROR;
    
  return AS6200_OK;
}


AS6200_Error_et AS6200_Get_Temperature(void *handle, float *value)
{  
	uint8_t val[2]={0};
	if (=AS6200_ReadReg( handle,0x0, 2,val))
	 return AS6200_ERROR;
	
	int TemperatureSum = ((val[0] << 8) | val[1]) >> 4; 
	
	*Temperature=TemperatureSum*0.0625;	
	
  return AS6200_OK;
}


AS6200_Error_et AS6200_Get_TemperatureRaw(void *handle, int16_t* value)
{
	uint8_t val[2]={0};
	if (=AS6200_ReadReg( handle,0x0, 2,val))
	 return AS6200_ERROR;
    
  *value = (int16_t)((((uint16_t)val[0]) << 8) | (uint16_t)val[1]);
  
  return AS6200_OK;
}



AS6200_Error_et AS6200_Set_PowerDownMode(void *handle, AS6200_BitStatus_et status)
{
  uint16_t tmp;
   
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint8_t*)tmp))
    return AS6200_ERROR;

    
  tmp &= ~AS6200_SM_MASK;
  tmp |= ((uint8_t)status) << AS6200_SM_BIT;
  
  if(AS6200_WriteReg(handle, AS6200_CONFIG, 2, &(uint8_t*)tmp))
    return AS6200_ERROR;
    
  return AS6200_OK;
}

AS6200_Error_et AS6200_Get_PowerDownMode(void *handle, AS6200_BitStatus_et* status)
{
  uint16_t tmp;
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint8_t*)tmp))
    return AS6200_ERROR;
    
  *status = (AS6200_BitStatus_et)((tmp & AS6200_SM_MASK) >> AS6200_SM_BIT);
  
  return AS6200_OK;
}


AS6200_Error_et AS6200_Set_Odr(void *handle, AS6200_Odr_et odr)
{
  uint16_t tmp;
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  tmp &= ~AS6200_CR_MASK;
  tmp |= ((uint16_t)odr) << AS6200_CR_BIT;

 if(AS6200_WriteReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  return AS6200_OK;
}


AS6200_Error_et AS6200_Get_Odr(void *handle, AS6200_Odr_et* odr)
{
  uint16_t tmp;
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  *odr =  (AS6200_BitStatus_et)((tmp & AS6200_CR_MASK) >> AS6200_CR_BIT);

  return AS6200_OK;
}


AS6200_Error_et AS6200_Set_AvgT(void *handle, AS6200_AvgT_et avgt)
{
  uint16_t tmp;
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  tmp &= ~AS6200_CF_MASK;
  tmp |= ((uint16_t)avgt) << AS6200_CF_BIT;

 if(AS6200_WriteReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  return AS6200_OK;
}


AS6200_Error_et AS6200_Get_OAvgT(void *handle, AS6200_AvgT_et *avgt)
{
  uint16_t tmp;
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  *avgt =  (AS6200_AvgT_et)((tmp & AS6200_CF_MASK) >> AS6200_CF_BIT);

  return AS6200_OK;
}



AS6200_Error_et AS6200_StartOneShotMeasurement(void *handle)
{
    uint16_t tmp;
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;

  tmp &= ~AS6200_SS_MASK;
  tmp |= 0x0001 << AS6200_SS_BIT;  

  
  if(AS6200_WriteReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  return AS6200_OK;
  
}



AS6200_Error_et AS6200_Set_IrqEnable(void *handle, , AS6200_BitStatus_et* status)
{
  uint16_t tmp;
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  tmp &= ~AS6200_IM_MASK;
  tmp |= status << AS6200_IM_BIT;  
  
 if(AS6200_WriteReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  return AS6200_OK;
}


AS6200_Error_et AS6200_Get_IrqEnable(void *handle, AS6200_BitStatus_et* status)
{
    uint16_t tmp;
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint8_t*)tmp))
    return AS6200_ERROR;
    
  *status = (AS6200_BitStatus_et)((tmp & AS6200_IM_MASK) >> AS6200_IM_BIT);
  
  
  return AS6200_OK;
}


AS6200_Error_et AS6200_Set_IrqActHigh(void *handle, AS6200_BitStatus_et status)
{
  uint16_t tmp;
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  tmp &= ~AS6200_POL_MASK;
  tmp |= status << AS6200_POL_BIT;  
  
 if(AS6200_WriteReg(handle, AS6200_CONFIG, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  return AS6200_OK;
}


AS6200_Error_et AS6200_Get_IrqActHigh(void *handle, AS6200_BitStatus_et* status)
{
    uint16_t tmp;
  
  if(AS6200_ReadReg(handle, AS6200_CONFIG, 2, &(uint8_t*)tmp))
    return AS6200_ERROR;
    
  *status = (AS6200_BitStatus_et)((tmp & AS6200_POL_MASK) >> AS6200_POL_BIT);
  
  
  return AS6200_OK;
}


AS6200_Error_et AS6200_Set_THIGH(void *handle,float Ths)
{
  uint16_t tmp;
  
tmp = (uint16_t)(Ths / 0.0625);  

 if(AS6200_WriteReg(handle, AS6200_THIGH, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  return AS6200_OK;
}

AS6200_Error_et AS6200_Set_TLOW(void *handle,float Ths)
{
  uint16_t tmp;
  
tmp = (uint16_t)(Ths / 0.0625);  

 if(AS6200_WriteReg(handle, AS6200_TLOW, 2, &(uint16_t*)tmp))
    return AS6200_ERROR;
    
  return AS6200_OK;
}

#ifdef  USE_FULL_ASSERT_AS6200
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void AS6200_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters value: file %s on line %d\r\n", file, (int)line);
  
  /* Infinite loop */
  while (1)
  {
  }
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

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
