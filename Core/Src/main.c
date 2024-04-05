/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <unistd.h>
#include "stm32f4xx_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//Enum For the State Machine
typedef enum {
	OFF,
	ACC,
	IGN,
	DCDC,
	ON,
	CHARGE,
	FAULT
} State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

/* USER CODE BEGIN PV */

// sets current state
static State_t currentState = OFF;
static State_t lastState = OFF;
uint32_t enterStateTime = 0;
uint32_t currentTime = 0;
uint32_t extraTime = 0;
char faultString[255] = "";

/* All faultCodes are two digits, first digit is lastState,
 second is code, see table below

faultCode = 00: 	No fault

fautlCode = 21: i_keyACC == 0 in IGN
faultCode = 22: i_disChargeEnable == 0 in IGN
faultCode = 23: i_ChargeEnable == 0 in IGN
faultCode = 24: i_killSwitch == 1 in IGN
faultCode = 29: Undefined fault in IGN

fautlCode = 31: i_keyACC == 0 in DCDC
faultCode = 32: i_disChargeEnable == 0 in DCDC
faultCode = 33: i_ChargeEnable == 0 in DCDC
faultCode = 34: i_killSwitch == 1 in DCDC
faultCode = 39: Undefined fault in DCDC

fautlCode = 41: i_keyACC == 0 in ON
faultCode = 42: i_disChargeEnable == 0 in ON -> Charger throw this error
faultCode = 43: i_ChargeEnable == 0 in ON
faultCode = 44: i_killSwitch == 1 in DCDC
faultCode = 49: Undefined fault in ON

fautlCode = 51: i_keyACC == 0 in CHARGE
faultCode = 52: i_disChargeEnable == 1 in CHARGE
faultCode = 53: i_ChargeEnable == 0 in CHARGE
faultCode = 54: i_chargeContactor == 0 in CHARGE
faultCode = 55: i_killSwitch == 1 in CHARGE
faultCode = 59: Undefined fault in CHARGE

*/
uint8_t faultCode = 0;
uint8_t debugMode = 1;
uint8_t faultBlinkCounter = 0;

GPIO_PinState i_keyIGN;
GPIO_PinState i_keyACC;
GPIO_PinState i_killSwitch;
GPIO_PinState i_chargeContactor;
GPIO_PinState i_chargeEnable;
GPIO_PinState i_disChargeEnable;
GPIO_PinState i_brakeSwitchInput;

GPIO_PinState o_hvDCDCEnable;
GPIO_PinState o_auxDCDCDisable;
GPIO_PinState o_chargeIndicator;
GPIO_PinState o_faultIndicator;
GPIO_PinState o_hvContactor;
GPIO_PinState o_preChargeRelay;
uint8_t o_pedalDAC;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void off_state(void);
void acc_state(void);
void ign_state(void);
void dcdc_state(void);
void on_state(void);
void charge_state(void);
void fault_state(void);
void stateMachine_init(void);
void debugMonitor(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void printActivatedVariable(uint8_t activatedVariable) {
//    char buffer[50];
//    snprintf(buffer, sizeof(buffer), "Activated variable: %d\r\n", activatedVariable);
//    CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer));
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_USB_DEVICE_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  i_keyIGN 				= HAL_GPIO_ReadPin(keyIGNPort, keyIGN);
	  i_keyACC 				= HAL_GPIO_ReadPin(keyACCPort, keyACC);
	  i_killSwitch 			= HAL_GPIO_ReadPin(killSwitchPort, killSwitch);
	  i_chargeContactor 	= !HAL_GPIO_ReadPin(chargeContactorPort, chargeContactor);
	  i_chargeEnable 		= !HAL_GPIO_ReadPin(chargeEnablePort, chargeEnable);
	  i_disChargeEnable 	= !HAL_GPIO_ReadPin(disChargeEnablePort, disChargeEnable);
	  i_brakeSwitchInput 	= HAL_GPIO_ReadPin(brakeSwitchInputPort, brakeSwitchInput);


	switch(currentState) {
			case OFF:
				off_state();
				break;
			case ACC:
				acc_state();
				break;
			case IGN:
				ign_state();
				break;
			case DCDC:
				dcdc_state();
				break;
			case ON:
				on_state();
				break;
			case CHARGE:
				charge_state();
				break;
			case FAULT:
				fault_state();
				break;
		}

	if (1)
	{
		debugMonitor();
	}

  }
}



/*
 * 		HAL_GPIO_WritePin(preChargeRelayPort,preChargeRelay,RESET);
		HAL_GPIO_WritePin(hvContactorPort,hvContactor,RESET);
		HAL_GPIO_WritePin(auxDCDCDisablePort,auxDCDCDisable,RESET);
		HAL_GPIO_WritePin(hvDCDCEnablePort,hvDCDCEnable,RESET);
 * 		HAL_GPIO_WritePin(GPIO(charge indicator
 * 		HAL_GPIO_WritePin(GPIO(Fault INDECATOR
 */
void off_state(void){		// State 0
	currentTime = HAL_GetTick();
	lastState = ON;
	HAL_GPIO_WritePin(preChargeRelayPort,preChargeRelay,GPIO_PIN_RESET);
	o_preChargeRelay = GPIO_PIN_RESET;
	HAL_GPIO_WritePin(hvContactorPort,hvContactor,GPIO_PIN_RESET);
	o_hvContactor = GPIO_PIN_RESET;
	HAL_GPIO_WritePin(auxDCDCDisablePort,auxDCDCDisable,GPIO_PIN_RESET);
	o_auxDCDCDisable = GPIO_PIN_RESET;
	HAL_GPIO_WritePin(hvDCDCEnablePort,hvDCDCEnable,GPIO_PIN_RESET);
	o_hvDCDCEnable = GPIO_PIN_RESET;
	// HAL_GPIO_WritePin(chargeIndicatorPort,chargeIndicator,RESET);
	// HAL_GPIO_WritePin(faultIndicatorPort,faultIndicator,RESET);

	//Changes States
	if ((i_keyACC != 0 ) && (i_chargeEnable != 0) && (i_disChargeEnable != 0) && (i_killSwitch != 1)){
		currentState = ACC;
	}
}
void acc_state(void){		// State 1
	if( lastState == ON )
	{
		enterStateTime = HAL_GetTick();
	}
	lastState = ACC;

	currentTime = HAL_GetTick();
	// goes back to off if something one wrong
	if((i_keyACC != 1) || (i_chargeEnable != 1) || (i_disChargeEnable != 1) || (i_killSwitch != 0))
		currentState = OFF; 			// No fault occurs

	//Changes States
	if ((i_keyIGN != 0) && (i_disChargeEnable != 0) && (i_keyACC != 0) && (i_chargeEnable != 0))
	{
		currentState = IGN;
	}
}
void ign_state(void){		// State 2

	if( lastState == ACC )
	{
		enterStateTime = HAL_GetTick();
		HAL_GPIO_WritePin(preChargeRelayPort, preChargeRelay, GPIO_PIN_SET);
		o_preChargeRelay = GPIO_PIN_SET;
	}
	lastState = IGN;
	currentTime = HAL_GetTick();

	// Key was not hold long enough
	if (i_keyIGN != 1)
	{
		currentState = ACC;
	}
	// Check for any missing inputs --> Fault
	else if ((i_keyACC != 1) || (i_disChargeEnable != 1) || (i_chargeEnable != 1) || (i_killSwitch != 0))
	{
		currentState = FAULT;
		if (i_keyACC != 1)
		{
			faultCode = 21;
		}
		else if (i_disChargeEnable != 1)
		{
			faultCode = 22;
		}
		else if (i_chargeEnable != 1)
		{
			faultCode = 23;
		}
		else if (i_killSwitch != 0)
		{
			faultCode = 24;
		}
		else
		{
			faultCode = 29;
		}

	}
	// If 5 sec are gone, activate HVContactor and switch in ON State
	else if(currentTime - enterStateTime >= 5000)
	{
		HAL_GPIO_WritePin(hvContactorPort, hvContactor, GPIO_PIN_SET);
		o_hvContactor = GPIO_PIN_SET;
		currentState = DCDC;
	}
}

//Short state for the transition between the two DCDC's
void dcdc_state(void){		// State 3

	if ( lastState == IGN )
	{
		enterStateTime = HAL_GetTick();
		HAL_GPIO_WritePin(hvDCDCEnablePort,hvDCDCEnable, GPIO_PIN_SET);
		o_hvDCDCEnable = GPIO_PIN_SET;
		HAL_GPIO_WritePin(preChargeRelayPort,preChargeRelay, GPIO_PIN_RESET);
		o_preChargeRelay = GPIO_PIN_RESET;
	}
	lastState = DCDC;

	currentTime = HAL_GetTick();
	// Check if any signal is missing --> FAULT State
	if ((i_keyACC != 1) || (i_disChargeEnable != 1) || (i_chargeEnable != 1) || (i_killSwitch != 0))
	{
		currentState = FAULT;
		if (i_keyACC != 1)
		{
			faultCode = 31;
		}
		else if (i_disChargeEnable != 1)
		{
			faultCode = 32;
		}
		else if (i_chargeEnable != 1)
		{
			faultCode = 33;
		}
		else if (i_killSwitch != 0)
		{
			faultCode = 34;
		}
		else
		{
			faultCode = 39;
		}
	}
	// Switch off the AUX-DCDC after 200 ms
	else if (currentTime - enterStateTime >= 200)
	{
		HAL_GPIO_WritePin(auxDCDCDisablePort,auxDCDCDisable, GPIO_PIN_SET);
		o_auxDCDCDisable = GPIO_PIN_SET;
		currentState = ON;
	}
}

void on_state(void){		// State 4


	if ( lastState == DCDC )
	{
		enterStateTime = HAL_GetTick();
	}
	lastState = ON;

	currentTime = HAL_GetTick();


	//SETS THE OUTPUTS!!!
	// Throttle pedal output function

	// Check if any signal is missing --> FAULT State
	if ((i_keyACC != 1) || (i_chargeEnable != 1) || (i_disChargeEnable != 1 && i_chargeContactor != 1) || (i_killSwitch != 0))
	{
		currentState = FAULT;
		if (i_keyACC != 1)
		{
			faultCode = 41;
		}
		else if (i_disChargeEnable != 1)
		{
			faultCode = 42;
		}
		else if (i_chargeEnable != 1)
		{
			faultCode = 43;
		}
		else if (i_killSwitch != 0)
		{
			faultCode = 44;
		}
		else
		{
			faultCode = 49;
		}
	}
	//Changes States
	else if ((i_disChargeEnable != 1) && (i_chargeContactor != 0))
	{
		currentState = CHARGE;
	}
}

void charge_state(void){

	// Start charging routine by enabling AUX DCDC
	if (lastState == ON)
	{
		enterStateTime = HAL_GetTick();
	}
	lastState = CHARGE;

	currentTime = HAL_GetTick();


	// After 0.5 sec, disconnect contactor and switch hvDCDC off
	if (currentTime - enterStateTime >= 500)
	{
		HAL_GPIO_WritePin(hvContactorPort, hvContactor, GPIO_PIN_RESET);
		o_hvContactor = GPIO_PIN_RESET;
		HAL_GPIO_WritePin(hvDCDCEnablePort, hvDCDCEnable, GPIO_PIN_RESET);
		o_hvDCDCEnable = GPIO_PIN_RESET;
	}


	//SETS THE OUTPUTS --> in this case basically doing nothing, maybe outputting for long
	// we are charging
	//HAL_GPIO_WritePin(chargeIndicatorPort, chargeIndicator, GPIO_PIN_SET);							// Charge indictor light


	//Changes States
	if((i_keyACC != 1) || (i_chargeEnable != 1) || (i_disChargeEnable != 0) || (i_chargeContactor != 1) || (i_killSwitch != 0))
	{
		currentState = FAULT;
		if (i_keyACC != 1)
		{
			faultCode = 51;
		}
		else if (i_disChargeEnable != 0)
		{
			faultCode = 52;
		}
		else if (i_chargeEnable != 1)
		{
			faultCode = 53;
		}
		else if (i_chargeContactor != 1)
		{
			faultCode = 54;
		}
		else if (i_killSwitch != 0)
		{
			faultCode = 55;
		}
		else
		{
			faultCode = 59;
		}
	}

}



void fault_state(void){

	// Compare strings for first enter
	if (strcmp(faultString, ""))
	{
		// Fault indicator light on
		HAL_GPIO_WritePin(faultIndicatorPort, faultIndicator, GPIO_PIN_RESET);
		o_faultIndicator = GPIO_PIN_SET;

		sprintf(faultString, "\n\n\n\rFAULT!!! FaultTime: %lu -"
				" old enterStateTime: %lu -- lastState = %d \n\n\n",
				currentTime, enterStateTime, lastState);

		CDC_Transmit_FS((uint8_t*)faultString, strlen((char*)faultString));
		debugMode;
		// Safe enterStateTime for measure 200 ms until HV Contactor will be open
		enterStateTime = HAL_GetTick();
	}

	currentTime = HAL_GetTick();

	// Switch AUX-DCDC on
	HAL_GPIO_WritePin(auxDCDCDisablePort,auxDCDCDisable, GPIO_PIN_RESET);
	o_auxDCDCDisable = GPIO_PIN_RESET;


	// Switch of HV-Contactor and HV-DCDC off
	uint32_t deltaT = currentTime - enterStateTime;
	if (deltaT >= 500)
	{
		// HV-DCDC off
		HAL_GPIO_WritePin(hvDCDCEnablePort,hvDCDCEnable, GPIO_PIN_RESET);
		o_hvDCDCEnable = GPIO_PIN_RESET;

		// HV Contactor open
		HAL_GPIO_WritePin(hvContactorPort, hvContactor, GPIO_PIN_RESET);
		o_hvDCDCEnable = GPIO_PIN_RESET;
	}

	// Blinking light math for 2 Hz
	// Calculating whether light is on or not
	if (((deltaT + extraTime) / 250) % 4 == 0)
	{
		HAL_GPIO_WritePin(faultIndicatorPort, faultIndicator, GPIO_PIN_SET);
		o_faultIndicator = 1;
	}
	else if (((deltaT + extraTime) / 250) % 4 == 1)
	{
		if (o_faultIndicator == 1)
		{
			o_faultIndicator = 0;					// not as an output used here!
			faultBlinkCounter++;
			uint8_t highDigit = faultCode / 10;
			uint8_t lowDigit = faultCode % 10;
			uint8_t startBlinks = 8;
			uint8_t betweenBlinks = 3;
			if (((faultBlinkCounter > startBlinks) &&
					(faultBlinkCounter <= startBlinks + highDigit)) ||
					((faultBlinkCounter > startBlinks + highDigit + betweenBlinks) &&
					(faultBlinkCounter <= startBlinks + highDigit + betweenBlinks + lowDigit)))
			{
				extraTime += 500;
			}
			if (faultBlinkCounter > startBlinks + highDigit + betweenBlinks + lowDigit)
			{
				faultBlinkCounter = 0;
			}
		}
	}
	else if (((deltaT + extraTime) / 250) % 4 == 3)
	{
		HAL_GPIO_WritePin(faultIndicatorPort, faultIndicator, GPIO_PIN_RESET);
		o_faultIndicator = 0;
	}
	else
	{
//		uint8_t highDigit = faultCode / 10;
//		uint8_t lowDigit = faultCode % 10;
//		uint8_t startBlinks = 8;
//		uint8_t betweenBlinks = 3;
//		if (faultBlinkCounter < startBlinks)
//		{
//			HAL_GPIO_WritePin(faultIndicatorPort, faultIndicator, GPIO_PIN_RESET);
//		}
//		else if (faultBlinkCounter - startBlinks > highDigit)
//		{
//			HAL_GPIO_WritePin(faultIndicatorPort, faultIndicator, GPIO_PIN_SET);
//		}
//		else if (faultBlinkCounter - startBlinks - highDigit > betweenBlinks)
//		{
//			HAL_GPIO_WritePin(faultIndicatorPort, faultIndicator, GPIO_PIN_RESET);
//		}
//		else if (faultBlinkCounter - startBlinks - highDigit - betweenBlinks > lowDigit)
//		{
//			HAL_GPIO_WritePin(faultIndicatorPort, faultIndicator, GPIO_PIN_SET);
//		}
//		else
//		{
//			faultBlinkCounter = 0;
//		}
	}

	// Switch off serial monitor in fault state if nothing happens anymore (after 300 ms)
	if (deltaT >= 1000)
	{
		debugMode;
	}


	//FALUT INDICATOR

}
//will change to what ever kill switch is set to
	//KILL SWITCH INTERUPT
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == B1_Pin) {
		switch(currentState) {
			case OFF:
				currentState = FAULT;
				break;
			case ACC:
				currentState = FAULT;
				break;
			case IGN:
				currentState = FAULT;
				break;
			case DCDC:
				currentState = FAULT;
				break;
			case ON:
				currentState = FAULT;
				break;
			case CHARGE:
				currentState = FAULT;
				break;
			case FAULT:
				currentState = FAULT;
				break;
		}
	}
}

void debugMonitor(void)
{
	char inBuffer[240], outBuffer[240], stateBuffer[240];
	sprintf(inBuffer, "\r%lu --- %lu --- State = %d keyIGN: %d\t keyACC: %d\t killSwitch:"
			" %d chargeContactor: %d chargeEnable: %d disChargeEnable: %d brakeSwitch: "
			"%d HV-DCDC-En: %d\t Aux-DCDC: %d\t \n", currentTime, enterStateTime, currentState,
			i_keyIGN, i_keyACC, i_killSwitch, i_chargeContactor, i_chargeEnable,
			i_disChargeEnable, i_brakeSwitchInput, o_hvDCDCEnable, o_auxDCDCDisable);
	CDC_Transmit_FS((uint8_t*)inBuffer, strlen((char*)inBuffer));
	// HAL_Delay(50);
	sprintf(outBuffer, "\rHV-DCDC-En: %d\t Aux-DCDC: %d\t chargeInd: %d\t faultInd: %d\t HV+_Contactor: %d\t preCharge: %d \n", o_hvDCDCEnable, o_auxDCDCDisable, o_chargeIndicator, o_faultIndicator, o_hvContactor, o_preChargeRelay);
	sprintf(stateBuffer, "\r%lu State = %d", currentTime, currentState);

//	CDC_Transmit_FS((uint8_t*)outBuffer, strlen((char*)outBuffer));
	// CDC_Transmit_FS((uint8_t*)stateBuffer, strlen((char*)stateBuffer));





/* keyIGN = HAL_GPIO_ReadPin(keyIGNPort, keyIGN);
	  keyACC = HAL_GPIO_ReadPin(keyACCPort, keyACC);
	  chargeSafety = HAL_GPIO_ReadPin(chargeContactorPort, chargeContactor);
	  chargeEnable = HAL_GPIO_ReadPin(chargeEnablePort, chargeEnable);
	  disChargeEnable = HAL_GPIO_ReadPin(disC
	  */



  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|auxDCDCDisable_Pin|hvDCDCEnable_Pin|preChargeRelay_Pin
                          |hvContactor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(hvContactorB15_GPIO_Port, hvContactorB15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD6_Pin|spareOutput2_Pin
                          |faultIndicatorD3_Pin|chargeIndicator_Pin|spareOutput3_Pin|spareOutput1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(faultIndicator_GPIO_Port, faultIndicator_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin auxDCDCDisable_Pin hvDCDCEnable_Pin preChargeRelay_Pin
                           hvContactor_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|auxDCDCDisable_Pin|hvDCDCEnable_Pin|preChargeRelay_Pin
                          |hvContactor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin faultIndicator_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|faultIndicator_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : chargeEnable_Pin */
  GPIO_InitStruct.Pin = chargeEnable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(chargeEnable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_MISO_Pin */
  GPIO_InitStruct.Pin = SPI1_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(SPI1_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : keyIGN_Pin keyACC_Pin chargeContactor_Pin disChargeEnable_Pin
                           chargeEnableE15_Pin */
  GPIO_InitStruct.Pin = keyIGN_Pin|keyACC_Pin|chargeContactor_Pin|disChargeEnable_Pin
                          |chargeEnableE15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : hvContactorB15_Pin */
  GPIO_InitStruct.Pin = hvContactorB15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(hvContactorB15_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : spareInput2_Pin spareInput1_Pin spareInput3_Pin killSwitch_Pin
                           brakeSwitchInput_Pin OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = spareInput2_Pin|spareInput1_Pin|spareInput3_Pin|killSwitch_Pin
                          |brakeSwitchInput_Pin|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD6_Pin spareOutput2_Pin
                           faultIndicatorD3_Pin chargeIndicator_Pin spareOutput3_Pin spareOutput1_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD6_Pin|spareOutput2_Pin
                          |faultIndicatorD3_Pin|chargeIndicator_Pin|spareOutput3_Pin|spareOutput1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_SCK_Pin */
  GPIO_InitStruct.Pin = I2S3_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
