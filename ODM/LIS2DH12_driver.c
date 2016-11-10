/******************** (C) COPYRIGHT 2016 PEGATRONCORP ********************
* File Name          : LIS2DH12_driver.c
* Author             : BSP Application Team
* Author             : Webber Sun
* Version            : $Revision:$
* Date               : $Date:$
* Description        : LIS2DH12 driver file
*                      
* HISTORY:
* Date               |	Modification                    |	Author
* 24/06/2011         |	Initial Revision                |	Webber Sun


********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "LIS2DH12_driver.h"
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_i2c_ex.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static LIS2DH12_Mode_t LIS2DH12_WorkMode = LIS2DH12_POWER_DOWN;

/* Private function prototypes -----------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
/*******************************************************************************
* Function Name		: LIS2DH12_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*			: I2C or SPI reading functions					
* Input			: Register Address
* Output		: Data REad
* Return		: None
*******************************************************************************/
uint8_t LIS2DH12_ReadReg(uint8_t Reg, uint8_t* Data) {
  
    HAL_StatusTypeDef status = HAL_OK;
	
  //  if ( NumByteToRead > 1 ) Reg |= 0x80;
	
	
  status = HAL_I2C_Mem_Read( &hi2c1, LIS2DH12_I2C_ADDRESS, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, Data, 1, 0xfffff );


                             
	return status;
}


/*******************************************************************************
* Function Name		: LIS2DH12_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*			: I2C or SPI writing function
* Input			: Register Address, Data to be written
* Output		: None
* Return		: None
*******************************************************************************/
uint8_t LIS2DH12_WriteReg(uint8_t WriteAddr, uint8_t Data) {
  
    HAL_StatusTypeDef status = HAL_OK;  
	
	// if ( NumByteToWrite > 1 ) WriteAddr |= 0x80;
	  status = HAL_I2C_Mem_Write( &hi2c1, LIS2DH12_I2C_ADDRESS, ( uint16_t )WriteAddr, I2C_MEMADD_SIZE_8BIT, &Data, 1, 0xfffff );
								  
		return status;
}


/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : LIS2DH12_GetWHO_AM_I
* Description    : Read identification code by WHO_AM_I register
* Input          : Char to empty by Device identification Value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LIS2DH12_GetWHO_AM_I(uint8_t* val){
  
  if(LIS2DH12_ReadReg(LIS2DH12_WHO_AM_I, val))
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_GetStatusAUX
* Description    : Read the AUX status register
* Input          : Char to empty by status register buffer
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetStatusAUX(uint8_t* val) {
  
  if( LIS2DH12_ReadReg(LIS2DH12_STATUS_AUX, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}



/*******************************************************************************
* Function Name  : LIS2DH12_GetStatusAUXBIT
* Description    : Read the AUX status register BIT
* Input          : LIS2DH12_STATUS_AUX_321OR, LIS2DH12_STATUS_AUX_3OR, LIS2DH12_STATUS_AUX_2OR, LIS2DH12_STATUS_AUX_1OR,
                   LIS2DH12_STATUS_AUX_321DA, LIS2DH12_STATUS_AUX_3DA, LIS2DH12_STATUS_AUX_2DA, LIS2DH12_STATUS_AUX_1DA
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS2DH12_GetStatusAUXBit(uint8_t statusBIT, uint8_t* val) {
//  uint8_t value;  
//  
//  if( LIS2DH12_ReadReg(LIS2DH12_STATUS_AUX, &value) )
//    return MEMS_ERROR;
//  
//  if(statusBIT == LIS2DH12_STATUS_AUX_321OR){
//    if(value &= LIS2DH12_STATUS_AUX_321OR){     
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{  
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }   
//  }
//  
//  if(statusBIT == LIS2DH12_STATUS_AUX_3OR){
//    if(value &= LIS2DH12_STATUS_AUX_3OR){     
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{  
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }     
//  }
//  
//  if(statusBIT == LIS2DH12_STATUS_AUX_2OR){
//    if(value &= LIS2DH12_STATUS_AUX_2OR){     
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{  
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }    
//  }
//  
//  if(statusBIT == LIS2DH12_STATUS_AUX_1OR){
//    if(value &= LIS2DH12_STATUS_AUX_1OR){     
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{  
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }   
//  }
//  
//  if(statusBIT == LIS2DH12_STATUS_AUX_321DA){
//    if(value &= LIS2DH12_STATUS_AUX_321DA) {     
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{  
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }   
//  }
//  
//  if(statusBIT == LIS2DH12_STATUS_AUX_3DA){
//    if(value &= LIS2DH12_STATUS_AUX_3DA){     
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{  
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }   
//  }
//  
//  if(statusBIT == LIS2DH12_STATUS_AUX_2DA){
//    if(value &= LIS2DH12_STATUS_AUX_2DA){     
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{  
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }  
//  }
//  
//  if(statusBIT == LIS2DH12_STATUS_AUX_1DA){
//    if(value &= LIS2DH12_STATUS_AUX_1DA){     
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{  
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }  
//  }  
//  return MEMS_ERROR;
//}


/*******************************************************************************
* Function Name  : LIS2DH12_SetODR
* Description    : Sets LIS2DH12 Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetODR(LIS2DH12_ODR_t ov){
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG1, &value) )
    return MEMS_ERROR;
  
  value &= 0x0f;
  value |= ov<<LIS2DH12_ODR_BIT;
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG1, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetTemperature
* Description    : Sets LIS2DH12 Output Temperature
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Note           : For Read Temperature by LIS2DH12_OUT_AUX_3, LIS2DH12_SetADCAux and LIS2DH12_SetBDU 
				   functions must be ENABLE
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetTemperature(State_t state){
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_TEMP_CFG_REG, &value) )
    return MEMS_ERROR;
  
  value &= 0xBF;
  value |= state<<LIS2DH12_TEMP_EN;
  
  if( LIS2DH12_WriteReg(LIS2DH12_TEMP_CFG_REG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetADCAux
* Description    : Sets LIS2DH12 Output ADC
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS2DH12_SetADCAux(State_t state){
//  uint8_t value;
//  
//  if( LIS2DH12_ReadReg(LIS2DH12_TEMP_CFG_REG, &value) )
//    return MEMS_ERROR;
//  
//  value &= 0x7F;
//  value |= state<<LIS2DH12_ADC_PD;
//  
//  if( LIS2DH12_WriteReg(LIS2DH12_TEMP_CFG_REG, value) )
//    return MEMS_ERROR;
//  
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS2DH12_GetAuxRaw
* Description    : Read the Aux Values Output Registers
* Input          : Buffer to empty
* Output         : Aux Values Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS2DH12_GetAuxRaw(LIS2DH12_Aux123Raw_t* buff) {
//  uint8_t valueL;
//  uint8_t valueH;
//  
//  if( LIS2DH12_ReadReg(LIS2DH12_OUT_1_L, &valueL) )
//    return MEMS_ERROR;
//  
//  if( LIS2DH12_ReadReg(LIS2DH12_OUT_1_H, &valueH) )
//    return MEMS_ERROR;
//  
//  buff->AUX_1 = (uint16_t)( (valueH << 8) | valueL )/16;
//  
//  if( LIS2DH12_ReadReg(LIS2DH12_OUT_2_L, &valueL) )
//    return MEMS_ERROR;
//  
//  if( LIS2DH12_ReadReg(LIS2DH12_OUT_2_H, &valueH) )
//    return MEMS_ERROR;
//  
//  buff->AUX_2 = (uint16_t)( (valueH << 8) | valueL )/16;
//  
//  if( LIS2DH12_ReadReg(LIS2DH12_OUT_3_L, &valueL) )
//    return MEMS_ERROR;
//  
//  if( LIS2DH12_ReadReg(LIS2DH12_OUT_3_H, &valueH) )
//    return MEMS_ERROR;
//  
//  buff->AUX_3 = (uint16_t)( (valueH << 8) | valueL )/16;
//  
//  return MEMS_SUCCESS;  
//}


/*******************************************************************************
* Function Name  : LIS2DH12_GetTempRaw
* Description    : Read the Temperature Values by AUX Output Registers OUT_3_H
* Input          : Buffer to empty
* Output         : Temperature Values Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetTempRaw(int8_t* buff) {
  uint8_t valueL;
  uint8_t valueH;
  
  if( LIS2DH12_ReadReg(LIS2DH12_OUT_3_L, &valueL) )
    return MEMS_ERROR;
  
  if( LIS2DH12_ReadReg(LIS2DH12_OUT_3_H, &valueH) )
    return MEMS_ERROR;
  
  *buff = (int8_t)( valueH );
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetMode
* Description    : Sets LIS2DH12 Operating Mode
* Input          : Modality (LIS2DH12_NORMAL, LIS2DH12_LOW_POWER, LIS2DH12_POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetMode(LIS2DH12_Mode_t md) {
  uint8_t value;
  uint8_t value2;
  static   uint8_t ODR_old_value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG1, &value) )
    return MEMS_ERROR;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG4, &value2) )
    return MEMS_ERROR;
  
  if((value & 0xF0)==0) 
    value = value | (ODR_old_value & 0xF0); //if it comes from POWERDOWN  
  
  switch(md) {
    
  case LIS2DH12_POWER_DOWN:
    ODR_old_value = value;
    value &= 0x0F;
    break;
    
  case LIS2DH12_NORMAL:
    value &= 0xF7;
    value |= (MEMS_RESET<<LIS2DH12_LPEN);
    value2 &= 0xF7;
    value2 |= (MEMS_RESET<<LIS2DH12_HR);   //00
    break;
    
  case LIS2DH12_LOW_POWER:		
    value &= 0xF7;
    value |=  (MEMS_SET<<LIS2DH12_LPEN);
    value2 &= 0xF7;
    value2 |= (MEMS_RESET<<LIS2DH12_HR); //10
    break;
	
	case LIS2DH12_HIGH_RESOLUTION:		
    value &= 0xF7;
    value |=  (MEMS_RESET<<LIS2DH12_LPEN);
    value2 &= 0xF7;
    value2 |= (MEMS_SET<<LIS2DH12_HR); // 01
    break;
    
  default:
    return MEMS_ERROR;
  }
  
	LIS2DH12_WorkMode = md;
	
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG1, value) )
    return MEMS_ERROR;
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG4, value2) )
    return MEMS_ERROR;  
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetAxis
* Description    : Enable/Disable LIS2DH12 Axis
* Input          : LIS2DH12_X_ENABLE/DISABLE | LIS2DH12_Y_ENABLE/DISABLE | LIS2DH12_Z_ENABLE/DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetAxis(LIS2DH12_Axis_t axis) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG1, &value) )
    return MEMS_ERROR;
  value &= 0xF8;
  value |= (0x07 & axis);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG1, value) )
    return MEMS_ERROR;   
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetFullScale
* Description    : Sets the LIS2DH12 FullScale
* Input          : LIS2DH12_FULLSCALE_2/LIS2DH12_FULLSCALE_4/LIS2DH12_FULLSCALE_8/LIS2DH12_FULLSCALE_16
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetFullScale(LIS2DH12_Fullscale_t fs) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xCF;	
  value |= (fs<<LIS2DH12_FS);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


status_t LIS2DH12_GetFullScale(LIS2DH12_Fullscale_t *fs) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= (~0xCF);	
  *fs =(LIS2DH12_Fullscale_t)(value>>LIS2DH12_FS);
    
  return MEMS_SUCCESS;
}



/*******************************************************************************
* Function Name  : LIS2DH12_SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetBDU(State_t bdu) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  value |= (bdu<<LIS2DH12_BDU);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : BLE_LSB / BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetBLE(LIS2DH12_Endianess_t ble) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xBF;	
  value |= (ble<<LIS2DH12_BLE);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetSelfTest
* Description    : Set Self Test Modality
* Input          : LIS2DH12_SELF_TEST_DISABLE/ST_0/ST_1
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetSelfTest(LIS2DH12_SelfTest_t st) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xF9;
  value |= (st<<LIS2DH12_ST);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_HPFClick
* Description    : Enable/Disable High Pass Filter for click
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_HPFClickEnable(State_t hpfe) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xFB;
  value |= (hpfe<<LIS2DH12_HPCLICK);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_HPFAOI1
* Description    : Enable/Disable High Pass Filter for AOI on INT_1
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_HPFAOI1Enable(State_t hpfe) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xFE;
  value |= (hpfe<<LIS2DH12_HPIS1);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_HPFAOI2
* Description    : Enable/Disable High Pass Filter for AOI on INT_2
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_HPFAOI2Enable(State_t hpfe) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xFD;
  value |= (hpfe<<LIS2DH12_HPIS2);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetHPFMode
* Description    : Set High Pass Filter Modality
* Input          : LIS2DH12_HPM_NORMAL_MODE_RES/LIS2DH12_HPM_REF_SIGNAL/
				   LIS2DH12_HPM_NORMAL_MODE/LIS2DH12_HPM_AUTORESET_INT
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetHPFMode(LIS2DH12_HPFMode_t hpm) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0x3F;
  value |= (hpm<<LIS2DH12_HPM);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : HPFCF [0,3]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetHPFCutOFF(LIS2DH12_HPFCutOffFreq_t hpf) {
  uint8_t value;
  
  if (hpf > 3)
    return MEMS_ERROR;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xCF;
  value |= (hpf<<LIS2DH12_HPCF);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetFilterDataSel
* Description    : Set Filter Data Selection bypassed or sent to FIFO OUT register
* Input          : MEMS_SET, MEMS_RESET
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetFilterDataSel(State_t state) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xF7;
  value |= (state<<LIS2DH12_FDS);
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          :  LIS2DH12_CLICK_ON_PIN_INT1_ENABLE/DISABLE    | LIS2DH12_I1_INT1_ON_PIN_INT1_ENABLE/DISABLE |              
                    LIS2DH12_I1_INT2_ON_PIN_INT1_ENABLE/DISABLE  | LIS2DH12_I1_DRDY1_ON_INT1_ENABLE/DISABLE    |              
                    LIS2DH12_I1_DRDY2_ON_INT1_ENABLE/DISABLE     | LIS2DH12_WTM_ON_INT1_ENABLE/DISABLE         |           
                    LIS2DH12_INT1_OVERRUN_ENABLE/DISABLE  
* example        : SetInt1Pin(LIS2DH12_CLICK_ON_PIN_INT1_ENABLE | LIS2DH12_I1_INT1_ON_PIN_INT1_ENABLE |              
                    LIS2DH12_I1_INT2_ON_PIN_INT1_DISABLE | LIS2DH12_I1_DRDY1_ON_INT1_ENABLE | LIS2DH12_I1_DRDY2_ON_INT1_ENABLE |
                    LIS2DH12_WTM_ON_INT1_DISABLE | LIS2DH12_INT1_OVERRUN_DISABLE   ) 
* Note           : To enable Interrupt signals on INT1 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetInt1Pin(LIS2DH12_IntPinConf_t pinConf) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG3, &value) )
    return MEMS_ERROR;
  
  value &= 0x00;
  value |= pinConf;
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetInt2Pin
* Description    : Set Interrupt2 pin Function
* Input          : LIS2DH12_CLICK_ON_PIN_INT2_ENABLE/DISABLE   | LIS2DH12_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LIS2DH12_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS2DH12_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LIS2DH12_INT_ACTIVE_HIGH/LOW
* example        : LIS2DH12_SetInt2Pin(LIS2DH12_CLICK_ON_PIN_INT2_ENABLE/DISABLE | LIS2DH12_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LIS2DH12_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS2DH12_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LIS2DH12_INT_ACTIVE_HIGH/LOW)
* Note           : To enable Interrupt signals on INT2 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetInt2Pin(LIS2DH12_IntPinConf_t pinConf) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG6, &value) )
    return MEMS_ERROR;
  
  value &= 0x00;
  value |= pinConf;
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG6, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}                       


/*******************************************************************************
* Function Name  : LIS2DH12_SetClickCFG
* Description    : Set Click Interrupt config Function
* Input          : LIS2DH12_ZD_ENABLE/DISABLE | LIS2DH12_ZS_ENABLE/DISABLE  | LIS2DH12_YD_ENABLE/DISABLE  | 
                   LIS2DH12_YS_ENABLE/DISABLE | LIS2DH12_XD_ENABLE/DISABLE  | LIS2DH12_XS_ENABLE/DISABLE 
* example        : LIS2DH12_SetClickCFG( LIS2DH12_ZD_ENABLE | LIS2DH12_ZS_DISABLE | LIS2DH12_YD_ENABLE | 
                               LIS2DH12_YS_DISABLE | LIS2DH12_XD_ENABLE | LIS2DH12_XS_ENABLE)
* Note           : You MUST use all input variable in the argument, as example
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetClickCFG(uint8_t status) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CLICK_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0xC0;
  value |= status;
  
  if( LIS2DH12_WriteReg(LIS2DH12_CLICK_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}  


/*******************************************************************************
* Function Name  : LIS2DH12_SetClickTHS
* Description    : Set Click Interrupt threshold
* Input          : Click-click Threshold value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetClickTHS(uint8_t ths) {
  
  if(ths>127)     
    return MEMS_ERROR;
  
  if( LIS2DH12_WriteReg(LIS2DH12_CLICK_THS, ths) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : LIS2DH12_SetClickLIMIT
* Description    : Set Click Interrupt Time Limit
* Input          : Click-click Time Limit value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetClickLIMIT(uint8_t t_limit) {
  
  if(t_limit>127)     
    return MEMS_ERROR;
  
  if( LIS2DH12_WriteReg(LIS2DH12_TIME_LIMIT, t_limit) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : LIS2DH12_SetClickLATENCY
* Description    : Set Click Interrupt Time Latency
* Input          : Click-click Time Latency value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetClickLATENCY(uint8_t t_latency) {
  
  if( LIS2DH12_WriteReg(LIS2DH12_TIME_LATENCY, t_latency) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : LIS2DH12_SetClickWINDOW
* Description    : Set Click Interrupt Time Window
* Input          : Click-click Time Window value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetClickWINDOW(uint8_t t_window) {
  
  if( LIS2DH12_WriteReg(LIS2DH12_TIME_WINDOW, t_window) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_GetClickResponse
* Description    : Get Click Interrupt Response by CLICK_SRC REGISTER
* Input          : char to empty by Click Response Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetClickResponse(uint8_t* res) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CLICK_SRC, &value) ) 
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  if((value & LIS2DH12_IA)==0) {        
    *res = LIS2DH12_NO_CLICK;     
    return MEMS_SUCCESS;
  }
  else {
    if (value & LIS2DH12_DCLICK){
      if (value & LIS2DH12_CLICK_SIGN){
        if (value & LIS2DH12_CLICK_Z) {
          *res = LIS2DH12_DCLICK_Z_N;   
          return MEMS_SUCCESS;
        }
        if (value & LIS2DH12_CLICK_Y) {
          *res = LIS2DH12_DCLICK_Y_N;   
          return MEMS_SUCCESS;
        }
        if (value & LIS2DH12_CLICK_X) {
          *res = LIS2DH12_DCLICK_X_N;   
          return MEMS_SUCCESS;
        }
      }
      else{
        if (value & LIS2DH12_CLICK_Z) {
          *res = LIS2DH12_DCLICK_Z_P;   
          return MEMS_SUCCESS;
        }
        if (value & LIS2DH12_CLICK_Y) {
          *res = LIS2DH12_DCLICK_Y_P;   
          return MEMS_SUCCESS;
        }
        if (value & LIS2DH12_CLICK_X) {
          *res = LIS2DH12_DCLICK_X_P;   
          return MEMS_SUCCESS;
        }
      }       
    }
    else{
      if (value & LIS2DH12_CLICK_SIGN){
        if (value & LIS2DH12_CLICK_Z) {
          *res = LIS2DH12_SCLICK_Z_N;   
          return MEMS_SUCCESS;
        }
        if (value & LIS2DH12_CLICK_Y) {
          *res = LIS2DH12_SCLICK_Y_N;   
          return MEMS_SUCCESS;
        }
        if (value & LIS2DH12_CLICK_X) {
          *res = LIS2DH12_SCLICK_X_N;   
          return MEMS_SUCCESS;
        }
      }
      else{
        if (value & LIS2DH12_CLICK_Z) {
          *res = LIS2DH12_SCLICK_Z_P;   
          return MEMS_SUCCESS;
        }
        if (value & LIS2DH12_CLICK_Y) {
          *res = LIS2DH12_SCLICK_Y_P;   
          return MEMS_SUCCESS;
        }
        if (value & LIS2DH12_CLICK_X) {
          *res = LIS2DH12_SCLICK_X_P;   
          return MEMS_SUCCESS;
        }
      }
    }
  }
  return MEMS_ERROR;
} 


/*******************************************************************************
* Function Name  : LIS2DH12_Int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_Int1LatchEnable(State_t latch) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG5, &value) )
    return MEMS_ERROR;
  
  value &= 0xF7;
  value |= latch<<LIS2DH12_LIR_INT1;
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_ResetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_ResetInt1Latch(void) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_INT1_SRC, &value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetIntConfiguration
* Description    : Interrupt 1 Configuration (without LIS2DH12_6D_INT)
* Input          : LIS2DH12_INT1_AND/OR | LIS2DH12_INT1_ZHIE_ENABLE/DISABLE | LIS2DH12_INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetIntConfiguration(LIS2DH12_Int1Conf_t ic) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_INT1_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic;
  
  if( LIS2DH12_WriteReg(LIS2DH12_INT1_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 

status_t LIS2DH12_SetInt2Configuration(LIS2DH12_Int2Conf_t ic) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_INT2_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic;
  
  if( LIS2DH12_WriteReg(LIS2DH12_INT2_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 
     
/*******************************************************************************
* Function Name  : LIS2DH12_SetIntMode
* Description    : Interrupt 1 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : LIS2DH12_INT_MODE_OR, LIS2DH12_INT_MODE_6D_MOVEMENT, LIS2DH12_INT_MODE_AND, 
				   LIS2DH12_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetIntMode(LIS2DH12_Int1Mode_t int_mode) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_INT1_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<LIS2DH12_INT_6D);
  
  if( LIS2DH12_WriteReg(LIS2DH12_INT1_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

 status_t LIS2DH12_SetInt2Mode(LIS2DH12_Int2Mode_t int_mode) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_INT2_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<LIS2DH12_INT_6D);
  
  if( LIS2DH12_WriteReg(LIS2DH12_INT2_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}   
/*******************************************************************************
* Function Name  : LIS2DH12_SetInt6D4DConfiguration
* Description    : 6D, 4D Interrupt Configuration
* Input          : LIS2DH12_INT1_6D_ENABLE, LIS2DH12_INT1_4D_ENABLE, LIS2DH12_INT1_6D_4D_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetInt6D4DConfiguration(LIS2DH12_INT_6D_4D_t ic) {
  uint8_t value;
  uint8_t value2;
  
  if( LIS2DH12_ReadReg(LIS2DH12_INT1_CFG, &value) )
    return MEMS_ERROR;
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG5, &value2) )
    return MEMS_ERROR;
  
  if(ic == LIS2DH12_INT1_6D_ENABLE){
    value &= 0xBF; 
    value |= (MEMS_ENABLE<<LIS2DH12_INT_6D);
    value2 &= 0xFB; 
    value2 |= (MEMS_DISABLE<<LIS2DH12_D4D_INT1);
  }
  
  if(ic == LIS2DH12_INT1_4D_ENABLE){
    value &= 0xBF; 
    value |= (MEMS_ENABLE<<LIS2DH12_INT_6D);
    value2 &= 0xFB; 
    value2 |= (MEMS_ENABLE<<LIS2DH12_D4D_INT1);
  }
  
  if(ic == LIS2DH12_INT1_6D_4D_DISABLE){
    value &= 0xBF; 
    value |= (MEMS_DISABLE<<LIS2DH12_INT_6D);
    value2 &= 0xFB; 
    value2 |= (MEMS_DISABLE<<LIS2DH12_D4D_INT1);
  }
  
  if( LIS2DH12_WriteReg(LIS2DH12_INT1_CFG, value) )
    return MEMS_ERROR;
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG5, value2) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_Get6DPosition
* Description    : 6D, 4D Interrupt Position Detect
* Input          : Byte to empty by POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_Get6DPosition(uint8_t* val){
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_INT1_SRC, &value) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  switch (value){
  case LIS2DH12_UP_SX:   
    *val = LIS2DH12_UP_SX;    
    break;
  case LIS2DH12_UP_DX:   
    *val = LIS2DH12_UP_DX;    
    break;
  case LIS2DH12_DW_SX:   
    *val = LIS2DH12_DW_SX;    
    break;
  case LIS2DH12_DW_DX:   
    *val = LIS2DH12_DW_DX;    
    break;
  case LIS2DH12_TOP:     
    *val = LIS2DH12_TOP;      
    break;
  case LIS2DH12_BOTTOM:  
    *val = LIS2DH12_BOTTOM;   
    break;
  }
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetInt1Threshold(uint8_t ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
  if( LIS2DH12_WriteReg(LIS2DH12_INT1_THS, ths) )
    return MEMS_ERROR;    
  
  return MEMS_SUCCESS;
}

status_t LIS2DH12_SetInt2Threshold(uint8_t ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
  if( LIS2DH12_WriteReg(LIS2DH12_INT2_THS, ths) )
    return MEMS_ERROR;    
  
  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS2DH12_SetInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetInt1Duration(LIS2DH12_Int1Conf_t id) {
  
  if (id > 127)
    return MEMS_ERROR;
  
  if( LIS2DH12_WriteReg(LIS2DH12_INT1_DURATION, id) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : LIS2DH12_FIFO_DISABLE, LIS2DH12_FIFO_BYPASS_MODE, LIS2DH12_FIFO_MODE, 
				   LIS2DH12_FIFO_STREAM_MODE, LIS2DH12_FIFO_TRIGGER_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_FIFOModeEnable(LIS2DH12_FifoMode_t fm) {
  uint8_t value;  
  
  if(fm == LIS2DH12_FIFO_DISABLE) { 
    if( LIS2DH12_ReadReg(LIS2DH12_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1F;
    value |= (LIS2DH12_FIFO_BYPASS_MODE<<LIS2DH12_FM);                     
    
    if( LIS2DH12_WriteReg(LIS2DH12_FIFO_CTRL_REG, value) )           //fifo mode bypass
      return MEMS_ERROR;   
    if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG5, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;    
    
    if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG5, value) )               //fifo disable
      return MEMS_ERROR;   
  }
  
  if(fm == LIS2DH12_FIFO_BYPASS_MODE)   {  
    if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG5, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS2DH12_FIFO_EN;
    
    if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG5, value) )               //fifo enable
      return MEMS_ERROR;  
    if( LIS2DH12_ReadReg(LIS2DH12_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS2DH12_FM);                     //fifo mode configuration
    
    if( LIS2DH12_WriteReg(LIS2DH12_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LIS2DH12_FIFO_MODE)   {
    if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG5, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS2DH12_FIFO_EN;
    
    if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG5, value) )               //fifo enable
      return MEMS_ERROR;  
    if( LIS2DH12_ReadReg(LIS2DH12_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS2DH12_FM);                      //fifo mode configuration
    
    if( LIS2DH12_WriteReg(LIS2DH12_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LIS2DH12_FIFO_STREAM_MODE)   {  
    if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG5, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS2DH12_FIFO_EN;
    
    if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG5, value) )               //fifo enable
      return MEMS_ERROR;   
    if( LIS2DH12_ReadReg(LIS2DH12_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS2DH12_FM);                      //fifo mode configuration
    
    if( LIS2DH12_WriteReg(LIS2DH12_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LIS2DH12_FIFO_TRIGGER_MODE)   {  
    if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG5, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS2DH12_FIFO_EN;
    
    if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG5, value) )               //fifo enable
      return MEMS_ERROR;    
    if( LIS2DH12_ReadReg(LIS2DH12_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS2DH12_FM);                      //fifo mode configuration
    
    if( LIS2DH12_WriteReg(LIS2DH12_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetTriggerInt
* Description    : Trigger event liked to trigger signal INT1/INT2
* Input          : LIS2DH12_TRIG_INT1/LIS2DH12_TRIG_INT2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetTriggerInt(LIS2DH12_TrigInt_t tr) {
  uint8_t value;  
  
  if( LIS2DH12_ReadReg(LIS2DH12_FIFO_CTRL_REG, &value) )
    return MEMS_ERROR;
  
  value &= 0xDF;
  value |= (tr<<LIS2DH12_TR); 
  
  if( LIS2DH12_WriteReg(LIS2DH12_FIFO_CTRL_REG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_SetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetWaterMark(uint8_t wtm) {
  uint8_t value;
  
  if(wtm > 31)
    return MEMS_ERROR;  
  
  if( LIS2DH12_ReadReg(LIS2DH12_FIFO_CTRL_REG, &value) )
    return MEMS_ERROR;
  
  value &= 0xE0;
  value |= wtm; 
  
  if( LIS2DH12_WriteReg(LIS2DH12_FIFO_CTRL_REG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

  
/*******************************************************************************
* Function Name  : LIS2DH12_GetStatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetStatusReg(uint8_t* val) {
  if( LIS2DH12_ReadReg(LIS2DH12_STATUS_REG, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LIS2DH12_GetStatusBIT
* Description    : Read the status register BIT
* Input          : LIS2DH12_STATUS_REG_ZYXOR, LIS2DH12_STATUS_REG_ZOR, LIS2DH12_STATUS_REG_YOR, LIS2DH12_STATUS_REG_XOR,
                   LIS2DH12_STATUS_REG_ZYXDA, LIS2DH12_STATUS_REG_ZDA, LIS2DH12_STATUS_REG_YDA, LIS2DH12_STATUS_REG_XDA, 
				   LIS2DH12_DATAREADY_BIT
				   val: Byte to be filled with the status bit	
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetStatusBit(uint8_t statusBIT, uint8_t* val) {
  uint8_t value;  
  
  if( LIS2DH12_ReadReg(LIS2DH12_STATUS_REG, &value) )
    return MEMS_ERROR;
  
  switch (statusBIT){
  case LIS2DH12_STATUS_REG_ZYXOR:     
    if(value &= LIS2DH12_STATUS_REG_ZYXOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case LIS2DH12_STATUS_REG_ZOR:       
    if(value &= LIS2DH12_STATUS_REG_ZOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case LIS2DH12_STATUS_REG_YOR:       
    if(value &= LIS2DH12_STATUS_REG_YOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }                                 
  case LIS2DH12_STATUS_REG_XOR:       
    if(value &= LIS2DH12_STATUS_REG_XOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }     
  case LIS2DH12_STATUS_REG_ZYXDA:     
    if(value &= LIS2DH12_STATUS_REG_ZYXDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case LIS2DH12_STATUS_REG_ZDA:       
    if(value &= LIS2DH12_STATUS_REG_ZDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case LIS2DH12_STATUS_REG_YDA:       
    if(value &= LIS2DH12_STATUS_REG_YDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case LIS2DH12_STATUS_REG_XDA:       
    if(value &= LIS2DH12_STATUS_REG_XDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }                                  
    
  }
  return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LIS2DH12_GetAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetAccAxesRaw(AxesRaw_t* buff) {
  int16_t value;
  uint8_t *valueL = (uint8_t *)(&value);
  uint8_t *valueH = ((uint8_t *)(&value)+1);
	uint8_t outoffest=0;
	
	
switch(LIS2DH12_WorkMode) 
	{   
		case LIS2DH12_POWER_DOWN:
			 outoffest=0;
    break;   
		case LIS2DH12_NORMAL:
				outoffest=6;
    break;  
		case LIS2DH12_LOW_POWER:		
				outoffest=8;
    break;
		case LIS2DH12_HIGH_RESOLUTION:		
				outoffest=4;
    break;    
		default:
    return MEMS_ERROR;
  }
	 
  if( LIS2DH12_ReadReg(LIS2DH12_OUT_X_L, valueL) )
    return MEMS_ERROR;
  
  if( LIS2DH12_ReadReg(LIS2DH12_OUT_X_H, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_X = (value >> outoffest);
  
  if( LIS2DH12_ReadReg(LIS2DH12_OUT_Y_L, valueL) )
    return MEMS_ERROR;
  
  if( LIS2DH12_ReadReg(LIS2DH12_OUT_Y_H, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_Y = (value >> outoffest);
  
  if( LIS2DH12_ReadReg(LIS2DH12_OUT_Z_L, valueL) )
    return MEMS_ERROR;
  
  if( LIS2DH12_ReadReg(LIS2DH12_OUT_Z_H, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_Z = (value >> outoffest);
 
  return MEMS_SUCCESS; 
}
/*******************************************************************************
* Function Name  : LIS2DH12_GetAccAxes
* Description    : Read the Acceleration Values Output Registers 
* Input          : buffer to empity by Axes_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetAccAxes(Axes_t* buff) 
{	
	AxesRaw_t tmp;
	uint8_t sensitivity = 1;
	LIS2DH12_Fullscale_t tmpfs;	
	
	if(LIS2DH12_GetFullScale(&tmpfs)!= MEMS_SUCCESS )
			return MEMS_ERROR;
	
	if(LIS2DH12_GetAccAxesRaw(&tmp) != MEMS_SUCCESS )
			return MEMS_ERROR;
		
	switch(tmpfs)
	{
		case LIS2DH12_FULLSCALE_2:
			if(LIS2DH12_WorkMode == LIS2DH12_HIGH_RESOLUTION)
				sensitivity = 1;
			else if(LIS2DH12_WorkMode == LIS2DH12_NORMAL)
				sensitivity = 4;
			else if(LIS2DH12_WorkMode == LIS2DH12_LOW_POWER)
				sensitivity = 16;					
		break;
		case LIS2DH12_FULLSCALE_4:
			if(LIS2DH12_WorkMode == LIS2DH12_HIGH_RESOLUTION)
				sensitivity = 2;
			else if(LIS2DH12_WorkMode == LIS2DH12_NORMAL)
				sensitivity = 8;
			else if(LIS2DH12_WorkMode == LIS2DH12_LOW_POWER)
				sensitivity = 32;	
		break;
		case LIS2DH12_FULLSCALE_8:
			if(LIS2DH12_WorkMode == LIS2DH12_HIGH_RESOLUTION)
				sensitivity = 4;
			else if(LIS2DH12_WorkMode == LIS2DH12_NORMAL)
				sensitivity = 16;
			else if(LIS2DH12_WorkMode == LIS2DH12_LOW_POWER)
				sensitivity = 64;	
		break;	
		case LIS2DH12_FULLSCALE_16:
			if(LIS2DH12_WorkMode == LIS2DH12_HIGH_RESOLUTION)
				sensitivity = 12;
			else if(LIS2DH12_WorkMode == LIS2DH12_NORMAL)
				sensitivity = 48;
			else if(LIS2DH12_WorkMode == LIS2DH12_LOW_POWER)
				sensitivity = 192;	 			
		break;			
	}	
		
	buff->AXIS_X= tmp.AXIS_X * sensitivity;
	buff->AXIS_Y= tmp.AXIS_Y * sensitivity;
	buff->AXIS_Z= tmp.AXIS_Z * sensitivity;
	
  return MEMS_SUCCESS; 
}
/*******************************************************************************
* Function Name  : LIS2DH12_GetInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : Char to empty by Int1 source value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetInt1Src(uint8_t* val) {
  
  if( LIS2DH12_ReadReg(LIS2DH12_INT1_SRC, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

status_t LIS2DH12_GetInt2Src(uint8_t* val) {
  
  if( LIS2DH12_ReadReg(LIS2DH12_INT2_SRC, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS2DH12_GetInt1SrcBit
* Description    : Reset Interrupt 1 Latching function
* Input          : statusBIT: LIS2DH12_INT_SRC_IA, LIS2DH12_INT_SRC_ZH, LIS2DH12_INT_SRC_ZL.....
*                  val: Byte to be filled with the status bit
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetInt1SrcBit(uint8_t statusBIT, uint8_t* val) {
  uint8_t value;  
   
  if( LIS2DH12_ReadReg(LIS2DH12_INT1_SRC, &value) )
      return MEMS_ERROR;
   
  if(statusBIT == LIS2DH12_INT1_SRC_IA){
    if(value &= LIS2DH12_INT1_SRC_IA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS2DH12_INT1_SRC_ZH){
    if(value &= LIS2DH12_INT1_SRC_ZH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS2DH12_INT1_SRC_ZL){
    if(value &= LIS2DH12_INT1_SRC_ZL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS2DH12_INT1_SRC_YH){
    if(value &= LIS2DH12_INT1_SRC_YH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS2DH12_INT1_SRC_YL){
    if(value &= LIS2DH12_INT1_SRC_YL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  if(statusBIT == LIS2DH12_INT1_SRC_XH){
    if(value &= LIS2DH12_INT1_SRC_XH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS2DH12_INT1_SRC_XL){
    if(value &= LIS2DH12_INT1_SRC_XL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LIS2DH12_GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : Byte to empty by FIFO source register value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetFifoSourceReg(uint8_t* val) {
  
  if( LIS2DH12_ReadReg(LIS2DH12_FIFO_SRC_REG, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS2DH12_GetFifoSourceBit
* Description    : Read Fifo WaterMark source bit
* Input          : statusBIT: LIS2DH12_FIFO_SRC_WTM, LIS2DH12_FIFO_SRC_OVRUN, LIS2DH12_FIFO_SRC_EMPTY
*				   val: Byte to fill  with the bit value
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_GetFifoSourceBit(uint8_t statusBIT,  uint8_t* val){
  uint8_t value;  
  
  if( LIS2DH12_ReadReg(LIS2DH12_FIFO_SRC_REG, &value) )
    return MEMS_ERROR;
  
  
  if(statusBIT == LIS2DH12_FIFO_SRC_WTM){
    if(value &= LIS2DH12_FIFO_SRC_WTM){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS2DH12_FIFO_SRC_OVRUN){
    if(value &= LIS2DH12_FIFO_SRC_OVRUN){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  if(statusBIT == LIS2DH12_FIFO_SRC_EMPTY){
    if(value &= statusBIT == LIS2DH12_FIFO_SRC_EMPTY){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LIS2DH12_GetFifoSourceFSS
* Description    : Read current number of unread samples stored in FIFO
* Input          : Byte to empty by FIFO unread sample value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LIS2DH12_GetFifoSourceFSS(uint8_t* val){
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_FIFO_SRC_REG, &value) )
    return MEMS_ERROR;
  
  value &= 0x1F;
  
  *val = value;
  
  return MEMS_SUCCESS;
}

      
/*******************************************************************************
* Function Name  : LIS2DH12_SetSPIInterface
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* Input          : LIS2DH12_SPI_3_WIRE, LIS2DH12_SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS2DH12_SetSPIInterface(LIS2DH12_SPIMode_t spi) {
  uint8_t value;
  
  if( LIS2DH12_ReadReg(LIS2DH12_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xFE;
  value |= spi<<LIS2DH12_SIM;
  
  if( LIS2DH12_WriteReg(LIS2DH12_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

