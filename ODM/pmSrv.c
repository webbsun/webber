//*****************************************************************//
//*****************************************************************//
//**                                                             **//
//**       (C)Copyright 2016, Pegatron Computer, Inc.            **//
//**                                                             **//
//**                     All Rights Reserved.                    **//
//**                                                             **//
//*****************************************************************//
//*****************************************************************//

#include "cmsis_os.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"
#include "uart.h"
#include "pmAPI.h"

#define CON_STM32L476RG_Nucleo 0

#if (CON_STM32L476RG_Nucleo == 1)
#define BATT_I2C_ADDR 0xBE
#else
#define BATT_I2C_ADDR 0xAA /* 0xAA for write and 0xAB for read */
#endif

extern ADC_HandleTypeDef hadc1;;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim5;

const uint16_t adcTable[] =
{
    3870, /* -40 C*/
    3511, /* -20 C*/
    2884, /*  00 C*/
    2099, /*  20 C*/
    1377, /*  40 C*/
     856, /*  60 C*/
     526, /*  80 C*/
     327, /* 100 C*/
     209, /* 120 C*/
};

void SystemClock_Config(void);

#if (CON_STM32L476RG_Nucleo == 1)
void SetPinPullUp(GPIO_TypeDef *port, uint16_t pin)
{
	uint32_t position;
	
	position = 0;
	for(position=0; position<32; position++)
	{
		if(((pin >> position) & 0x01) != RESET)
			break;
	}
	port->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (position * 2));
	port->PUPDR |= GPIO_PUPDR_PUPD0_0 << (position * 2);
}
#endif

void svPmGetPwrKey(_Bool *key)
{	
	*key = HAL_GPIO_ReadPin(PWR_KEY_GPIO_Port, PWR_KEY_Pin);
	return;
}

void svPmGetMcuRccCsr(uint32_t *csr)
{	
	*csr = RCC->CSR;
	RCC->CSR = RCC->CSR | RCC_CSR_RMVF;
	return;
}

void svPmGetChgPin(_Bool *chg)
{	
#if (CON_STM32L476RG_Nucleo == 1)
	/* enable pull-up on nucleo64 for input pin test. */
	SetPinPullUp(CHARGE_CHG__GPIO_Port, CHARGE_CHG__Pin);
#endif
	*chg = HAL_GPIO_ReadPin(CHARGE_CHG__GPIO_Port, CHARGE_CHG__Pin);
	return;
}

void svPmSetMcuOff(_Bool opcode)
{	
#if (CON_STM32L476RG_Nucleo == 1)
	/* enable pull-up on nucleo64 for input pin test. */
	SetPinPullUp(MCU_OFF_GPIO_Port, MCU_OFF_Pin);
#endif
	HAL_GPIO_WritePin(MCU_OFF_GPIO_Port, MCU_OFF_Pin, (GPIO_PinState)opcode);
	return;
}

void svPmSetBattAdcOn(_Bool opcode)
{	
#if (CON_STM32L476RG_Nucleo == 1)
	/* enable pull-up on nucleo64 for input pin test. */
	SetPinPullUp(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin);
#endif
	HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, (GPIO_PinState)opcode);
	return;
}

/* ex: D:\ST\STM32Cube_FW_L4_V1.5.0\Projects\STM32L476G_EVAL\Examples\ADC\ADC_RegularConversion_Polling\Src */
int svPmGetBattTempAdc(int16_t *temp)
{
  int i;
  int ret;
  uint16_t adcTemp;
  ADC_ChannelConfTypeDef sConfig;
	
  ret = 0;
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    ret = -1;
  }
  /*##-3- Calibrate ADC then Start the conversion process ####################*/  
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
    /* ADC Calibration Error */
    ret = -2;
  }
  
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    /* Start Conversation Error */
    ret = -3;
  }

  /*##-4- Wait for the end of conversion #####################################*/
  /*  For simplicity reasons, this example is just waiting till the end of the
      conversion, but application may perform other tasks while conversion
      operation is ongoing. */
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
    /* End Of Conversion flag not set on time */
    ret = -4;
  }

  /* Check if the continous conversion of regular channel is finished */
  if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
  {
    /*##-5- Get the converted value of regular channel  ########################*/
    adcTemp = HAL_ADC_GetValue(&hadc1);
    if(adcTemp >= adcTable[0])
    {
        *temp = -40;
    }
    else if(adcTemp <= adcTable[sizeof(adcTable)/2])
    {
        *temp = 125;
    }
    else
    {
        for(i=0; i<(sizeof(adcTable)/2); i++)
        {
            if(adcTemp > adcTable[i+1])
            break;
        }
        *temp = ((i - 1) * 20) - (adcTemp - adcTable[i+1]) / ((adcTable[i] - adcTable[i+1]) / 20);
    }
  }
  return ret;
}

int svPmGetBattVbatAdc(uint16_t *temp)
{
  int ret;
  uint16_t adcVolt;
  ADC_ChannelConfTypeDef sConfig;
	
  ret = 0;
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    ret = -1;
  }
  /*##-3- Calibrate ADC then Start the conversion process ####################*/  
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) !=  HAL_OK)
  {
    /* ADC Calibration Error */
    ret = -2;
  }
  
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    /* Start Conversation Error */
    ret = -3;
  }

  /*##-4- Wait for the end of conversion #####################################*/
  /*  For simplicity reasons, this example is just waiting till the end of the
      conversion, but application may perform other tasks while conversion
      operation is ongoing. */
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
    /* End Of Conversion flag not set on time */
    ret = -4;
  }

  /* Check if the continous conversion of regular channel is finished */
  if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
  {
    /*##-5- Get the converted value of regular channel  ########################*/
    adcVolt = HAL_ADC_GetValue(&hadc1);
    *temp = (adcVolt * 18 * 244) / 4095;;
  }
  return ret;
}

int svPmGetBattGauge(uint8_t cmd, uint16_t *gauge)
{
	uint16_t mygauge;
	
	 //if (HAL_I2C_Mem_Read( &hi2c1, BATT_I2C_ADDR, BATT_CMD_TEMP, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&mygauge, 2, 0x4000 )==HAL_OK)
	 if (HAL_I2C_Mem_Read( &hi2c1, BATT_I2C_ADDR, cmd, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&mygauge, 2, 0x4000 )==HAL_OK)
	 {
		*gauge = mygauge;
		return 0;
	 }
	 else
	 {
		*gauge = 0;
		return -1;
	 }
}

void svPmSetMcuReset(uint8_t rsv)
{
	NVIC_SystemReset();
    /* software reset flag rcc_csr=0x14000600 */
}

void svPmSetPin(GPIO_TypeDef *GPIOx, uint32_t position, GPIO_PinState PinState)
{
	uint32_t temp;

	/* power consumption is better if we use no pull and low for LED off. */
	if(PinState == GPIO_PIN_SET)
	{
		temp = GPIOx->PUPDR;
		temp &= ~(GPIO_PUPDR_PUPD0 << (position * 2));
		temp |= GPIO_PULLUP << (position * 2);
		GPIOx->PUPDR = temp;
	}
	else
	{
		GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (position * 2));
	}
	HAL_GPIO_WritePin(GPIOx, 1 << position, PinState);
}

int svPmSetPwrState(uint8_t state)
{
	GPIO_InitTypeDef GPIO_InitStruct;

    while(1)
    {
		//HAL_GPIO_WritePin(GPIOC, BLE_WIFI_ANT_SW_Pin|MODEM_PWR_SW_EN_Pin|GPS_INT_WAKE_Pin|BAT_ADC_EN_Pin
		//					  |WIFI_3V3_EN_Pin|GPS_PMIC_CE_Pin, GPIO_PIN_RESET);
		svPmSetPin(GPIOC, 12, GPIO_PIN_RESET);
		svPmSetPin(GPIOC, 9, GPIO_PIN_RESET);
		svPmSetPin(GPIOC, 8, GPIO_PIN_RESET);
		svPmSetPin(GPIOC, 2, GPIO_PIN_RESET);
		svPmSetPin(GPIOC, 5, GPIO_PIN_RESET);
		svPmSetPin(GPIOC, 4, GPIO_PIN_RESET);

		/*Configure GPIO pin Output Level */
		//HAL_GPIO_WritePin(GPIOG, BLE_WAKE_Pin|WIFI_SPI3_CS_3V3_Pin, GPIO_PIN_RESET);
		svPmSetPin(GPIOG, 13, GPIO_PIN_RESET);
		svPmSetPin(GPIOG, 12, GPIO_PIN_RESET);

		/*Configure GPIO pin Output Level */
		//HAL_GPIO_WritePin(WIFI_PWD_L_GPIO_Port, WIFI_PWD_L_Pin, GPIO_PIN_RESET);
		svPmSetPin(WIFI_PWD_L_GPIO_Port, 12, GPIO_PIN_RESET);

		/*Configure GPIO pin Output Level */
		//HAL_GPIO_WritePin(MCU_OFF_GPIO_Port, MCU_OFF_Pin, GPIO_PIN_RESET);
		svPmSetPin(MCU_OFF_GPIO_Port, 2, GPIO_PIN_RESET);
		if(1)
		{
			GPIO_InitStruct.Pin = GPIO_PIN_10;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			GPIO_InitStruct.Pin = GPIO_PIN_9;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

		uartPrintf(" enter stop2...\r\n");
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		svPmSetPin(GPIOA, 10, GPIO_PIN_SET);
		osDelay(2000);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		svPmSetPin(GPIOA, 10, GPIO_PIN_RESET);
		svPmSetPin(GPIOA, 9, GPIO_PIN_RESET);
		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
		SystemClock_Config();
		if(state == POWER_MODE_STOP2)
		{
			GPIO_InitStruct.Pin = GPIO_PIN_10;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			GPIO_InitStruct.Pin = GPIO_PIN_9;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			return 0;
		}
		uartPrintf("enter shutdown mode...\r\n");
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		svPmSetPin(GPIOA, 9, GPIO_PIN_SET);
		osDelay(2000);
		//__asm volatile( "cpsid i" );
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		svPmSetPin(GPIOA, 9, GPIO_PIN_RESET);
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_LOW);
		HAL_PWREx_EnableInternalWakeUpLine();
		HAL_PWREx_EnterSHUTDOWNMode();
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
		HAL_PWREx_DisableInternalWakeUpLine();
		//__asm volatile( "cpsie i" );
		/* the shutdown mode will be waked up from reset. */
	}
	return 0;
}




