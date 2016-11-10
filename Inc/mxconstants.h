/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define GYRO_G_INT1_Pin GPIO_PIN_2
#define GYRO_G_INT1_GPIO_Port GPIOD
#define WIFI_INT_Pin GPIO_PIN_14
#define WIFI_INT_GPIO_Port GPIOG
#define BLE_WIFI_ANT_SW_Pin GPIO_PIN_12
#define BLE_WIFI_ANT_SW_GPIO_Port GPIOC
#define BLE_WAKE_Pin GPIO_PIN_13
#define BLE_WAKE_GPIO_Port GPIOG
#define PWR_KEY_Pin GPIO_PIN_13
#define PWR_KEY_GPIO_Port GPIOC
#define WIFI_PWD_L_Pin GPIO_PIN_12
#define WIFI_PWD_L_GPIO_Port GPIOA
#define WIFI_SPI3_CS_3V3_Pin GPIO_PIN_12
#define WIFI_SPI3_CS_3V3_GPIO_Port GPIOG
#define CHARGE_CHG__Pin GPIO_PIN_11
#define CHARGE_CHG__GPIO_Port GPIOA
#define AMB_LED_PWM_Pin GPIO_PIN_10
#define AMB_LED_PWM_GPIO_Port GPIOA
#define MODEM_PWR_SW_EN_Pin GPIO_PIN_9
#define MODEM_PWR_SW_EN_GPIO_Port GPIOC
#define BLE_INT_Pin GPIO_PIN_8
#define BLE_INT_GPIO_Port GPIOA
#define BLUE_LED_PWM_Pin GPIO_PIN_9
#define BLUE_LED_PWM_GPIO_Port GPIOA
#define BUZZER_PWM_Pin GPIO_PIN_7
#define BUZZER_PWM_GPIO_Port GPIOC
#define GPS_INT_WAKE_Pin GPIO_PIN_8
#define GPS_INT_WAKE_GPIO_Port GPIOC
#define PRESSURE_INT_DRDY_Pin GPIO_PIN_6
#define PRESSURE_INT_DRDY_GPIO_Port GPIOC
#define BAT_ADC_EN_Pin GPIO_PIN_2
#define BAT_ADC_EN_GPIO_Port GPIOC
#define GPS_IRQ_Pin GPIO_PIN_15
#define GPS_IRQ_GPIO_Port GPIOB
#define BAT_V_ADC_Pin GPIO_PIN_4
#define BAT_V_ADC_GPIO_Port GPIOA
#define MCU_AC_PLUGIN__Pin GPIO_PIN_3
#define MCU_AC_PLUGIN__GPIO_Port GPIOC
#define GYRO_G_INT2_Pin GPIO_PIN_12
#define GYRO_G_INT2_GPIO_Port GPIOB
#define BAT_TEMP_ADC_Pin GPIO_PIN_5
#define BAT_TEMP_ADC_GPIO_Port GPIOA
#define MCU_OFF_Pin GPIO_PIN_2
#define MCU_OFF_GPIO_Port GPIOB
#define WIFI_3V3_EN_Pin GPIO_PIN_5
#define WIFI_3V3_EN_GPIO_Port GPIOC
#define GPS_PMIC_CE_Pin GPIO_PIN_4
#define GPS_PMIC_CE_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
#define USE_FULL_ASSERT

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
