/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define pedalADC_Pin GPIO_PIN_1
#define pedalADC_GPIO_Port GPIOA
#define regenADC_Pin GPIO_PIN_2
#define regenADC_GPIO_Port GPIOA
#define chargeEnable_Pin GPIO_PIN_3
#define chargeEnable_GPIO_Port GPIOA
#define pedalDAC_Pin GPIO_PIN_4
#define pedalDAC_GPIO_Port GPIOA
#define regenDAC_Pin GPIO_PIN_5
#define regenDAC_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define auxDCDCDisable_Pin GPIO_PIN_7
#define auxDCDCDisable_GPIO_Port GPIOE
#define hvDCDCEnable_Pin GPIO_PIN_8
#define hvDCDCEnable_GPIO_Port GPIOE
#define keyIGN_Pin GPIO_PIN_9
#define keyIGN_GPIO_Port GPIOE
#define keyACC_Pin GPIO_PIN_10
#define keyACC_GPIO_Port GPIOE
#define preChargeRelay_Pin GPIO_PIN_11
#define preChargeRelay_GPIO_Port GPIOE
#define hvContactor_Pin GPIO_PIN_12
#define hvContactor_GPIO_Port GPIOE
#define chargeContactor_Pin GPIO_PIN_13
#define chargeContactor_GPIO_Port GPIOE
#define disChargeEnable_Pin GPIO_PIN_14
#define disChargeEnable_GPIO_Port GPIOE
#define chargeEnableE15_Pin GPIO_PIN_15
#define chargeEnableE15_GPIO_Port GPIOE
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define hvContactorB15_Pin GPIO_PIN_15
#define hvContactorB15_GPIO_Port GPIOB
#define spareInput2_Pin GPIO_PIN_8
#define spareInput2_GPIO_Port GPIOD
#define spareInput1_Pin GPIO_PIN_9
#define spareInput1_GPIO_Port GPIOD
#define spareInput3_Pin GPIO_PIN_10
#define spareInput3_GPIO_Port GPIOD
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define keyIGND14_Pin GPIO_PIN_14
#define keyIGND14_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SCK_Pin GPIO_PIN_10
#define I2S3_SCK_GPIO_Port GPIOC
#define faultIndicator_Pin GPIO_PIN_12
#define faultIndicator_GPIO_Port GPIOC
#define spareOutput2_Pin GPIO_PIN_0
#define spareOutput2_GPIO_Port GPIOD
#define killSwitch_Pin GPIO_PIN_1
#define killSwitch_GPIO_Port GPIOD
#define brakeSwitchInput_Pin GPIO_PIN_2
#define brakeSwitchInput_GPIO_Port GPIOD
#define faultIndicatorD3_Pin GPIO_PIN_3
#define faultIndicatorD3_GPIO_Port GPIOD
#define chargeIndicator_Pin GPIO_PIN_4
#define chargeIndicator_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define spareOutput3_Pin GPIO_PIN_6
#define spareOutput3_GPIO_Port GPIOD
#define spareOutput1_Pin GPIO_PIN_7
#define spareOutput1_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define canRX_Pin GPIO_PIN_8
#define canRX_GPIO_Port GPIOB
#define canTX_Pin GPIO_PIN_9
#define canTX_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

// Inputs

#define keyIGN keyIGN_Pin					//PE9
#define keyIGNPort GPIOE

#define keyACC keyACC_Pin					// PE10
#define keyACCPort GPIOE

#define killSwitch killSwitch_Pin			// PD1
#define killSwitchPort GPIOD

// ChargeSafety = ChargeContactor
#define chargeContactor chargeContactor_Pin	//PE13
#define chargeContactorPort GPIOE

#define chargeEnable chargeEnable_Pin		//PE15
#define chargeEnablePort GPIOE

#define disChargeEnable disChargeEnable_Pin	// PE14
#define disChargeEnablePort GPIOE

#define brakeSwitchInput brakeSwitchInput_Pin	// PD2
#define brakeSwitchInputPort GPIOD

#define pedalADC pedalADC_Pin			// PA1
#define pedalADCPort GPIOA

#define regenADC regenADC_Pin			// PA2 - ADC2_IN2
#define regenADCPort GPIOA

#define spareInput1 spareInput1_Pin		// PD9
#define spareInput1Port GPIOD

#define spareInput2 spareInput2_Pin		// PD8
#define spareInput2Port GPIOD

#define spareInput3 spareInput3_Pin		// PD10
#define spareInput3Port GPIOD


// Outputs
#define hvDCDCEnable hvDCDCEnable_Pin	//PE8
#define hvDCDCEnablePort GPIOE

#define auxDCDCDisable auxDCDCDisable_Pin	//PE7
#define auxDCDCDisablePort GPIOE

#define chargeIndicator chargeIndicator_Pin	// PD4
#define chargeIndicatorPort GPIOD

#define faultIndicator faultIndicator_Pin	// PC12
#define falutIndicatorPort GPIOC

#define hvContactor hvContactor_Pin			// PE12
#define hvContactorPort GPIOE

#define preChargeRelay preChargeRelay_Pin	//PE11
#define preChargeRelayPort GPIOE

#define pedalDAC pedalDAC_Pin		// PA4 DAC_OUT1
#define pedalDACPort GPIOA

#define regenDAC regenDAC_Pin		// PA5 DAC_OUT2
#define regenDACPort GPIOA

#define spareOutput1 spareOutput1_Pin		// PD7
#define spareOutput1Port GPIOD

#define spareOutput2 spareOutput2_Pin		// PD0
#define spareOutput2Port GPIOD

#define spareOutput3 spareOutput3_Pin		// PD6
#define spareOutput3Port GPIOD



// CAN
#define canRX canRX_Pin			// PD8
#define canRX_Port GPIOD

#define canTX canTX_Pin 		//PB9
#define canTX_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
