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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>		// To include uintXX_t type
#include <stdbool.h>	// To include the bool type
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RxBufferSize 100
#define MAX_ANGLE_LENGTH 4  // Adjust if needed (4 digits + comma)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t RxData[RxBufferSize];
char send[100];
int ReadyToSend = 1;

struct Servo {
	int Angle;
	int SetAngle;
	float MovingAngle;
	float Step;
};

int globalcounter = 0;

uint32_t CurrentMillis = 0;
uint32_t PreviousMillis, PreviousMillis_LED = 0;

struct Servo Base_Shoulder;
struct Servo Shoulder_Elbow;
struct Servo Elbow_Wrist;
struct Servo Wrist_Ulna;
struct Servo Ulna_Febur;
struct Servo Febur_Gripper;
struct Servo Right_finger;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	// Ensure we're using UART1
	if (huart->Instance == USART1) {
		RxData[Size] = '\0';  // Null-terminate received data
		char *token;
		int servo_angles[7] = { 0 };  // Initialize angles array to zero
		int index = 0;

		// Parse RxData for comma-separated values
		token = strtok((char*) RxData, ",");
		while (token != NULL && index < 7) {  // Parse up to 5 angles
			servo_angles[index] = atoi(token); // Convert angle string to integer
			if (servo_angles[index] > 180) {
				servo_angles[index] = 180;     // Cap angle to max 180
			}
			index++;
			token = strtok(NULL, ",");         // Move to next token
		}

		// Assign angles to respective servos (ensure order is correct)
		if (index > 0)
			Base_Shoulder.SetAngle = servo_angles[0];
		if (index > 1)
			Shoulder_Elbow.SetAngle = servo_angles[1];
		if (index > 2)
			Elbow_Wrist.SetAngle = servo_angles[2];
		if (index > 3)
			Wrist_Ulna.SetAngle = servo_angles[3];
		if (index > 4)
			Ulna_Febur.SetAngle = servo_angles[4];
		if (index > 5)
			Febur_Gripper.SetAngle = servo_angles[5];
		if (index > 6)
			Right_finger.SetAngle = servo_angles[6];

		// Clear RxData buffer for the next command
		memset(RxData, 0, RxBufferSize);
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxData, RxBufferSize);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	ReadyToSend = 1;

}

void SetServoAngle(TIM_HandleTypeDef *htim, uint32_t channel, int angle) {
	int pulseWidth = 500 + (angle * (2500 - 500) / 180); // Map angle to 500-2500μs range
	__HAL_TIM_SetCompare(htim, channel, pulseWidth);
}

/* Initialize Servos */
void ServoInit() {
	// Base_Shoulder Servo on TIM4 CH4 (PB9)
	HAL_TIM_PWM_Init(&htim4);
	HAL_TIM_PWM_Init(&htim3);
	HAL_TIM_PWM_Init(&htim2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //Base_Shoulder
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); //Shoulder_elbow
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //Elbow_wrist
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //Wrist_ulna
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Ulna_febur
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //Febur_gripper
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //Right_finger

	// Repeat initialization for other servos...

	// Set all servos to 90 degrees (Home position)
	SetServoAngle(&htim4, TIM_CHANNEL_4, 90);
	Base_Shoulder.Angle = 90;
	Base_Shoulder.SetAngle = 90;
	Base_Shoulder.MovingAngle = 90.0;
	SetServoAngle(&htim4, TIM_CHANNEL_3, 90);
	Shoulder_Elbow.Angle = 90;
	Shoulder_Elbow.SetAngle = 90;
	Shoulder_Elbow.MovingAngle = 90.0;
	SetServoAngle(&htim4, TIM_CHANNEL_2, 90);
	Elbow_Wrist.Angle = 90;
	Elbow_Wrist.SetAngle = 90;
	Elbow_Wrist.MovingAngle = 90.0;
	SetServoAngle(&htim4, TIM_CHANNEL_1, 90);
	Wrist_Ulna.Angle = 90;
	Wrist_Ulna.SetAngle = 90;
	Wrist_Ulna.MovingAngle = 90.0;
	SetServoAngle(&htim3, TIM_CHANNEL_2, 90);
	Ulna_Febur.Angle = 90;
	Ulna_Febur.SetAngle = 90;
	Ulna_Febur.MovingAngle = 90.0;
	SetServoAngle(&htim3, TIM_CHANNEL_1, 90);
	Febur_Gripper.Angle = 90;
	Febur_Gripper.SetAngle = 90;
	Febur_Gripper.MovingAngle = 90.0;
	SetServoAngle(&htim2, TIM_CHANNEL_2, 90);
	Right_finger.Angle = 90;
	Right_finger.SetAngle = 90;
	Right_finger.MovingAngle = 90.0;
}

// TEST YOUR SERVO MOTORS
void Servo_Test(TIM_HandleTypeDef *htim, uint32_t channel) {
	int x;
	for (x = 500; x < 2500; x = x + 1) {
		__HAL_TIM_SET_COMPARE(htim, channel, x);
		HAL_Delay(3);
	}
}

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxData, RxBufferSize);

	ServoInit();
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	CurrentMillis = 0;
	PreviousMillis = HAL_GetTick();
	PreviousMillis_LED = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		CurrentMillis = HAL_GetTick();

		if (Base_Shoulder.Angle != Base_Shoulder.SetAngle) {
			if (Base_Shoulder.SetAngle > Base_Shoulder.Angle) {
				Base_Shoulder.Angle += 1;
			} else {
				Base_Shoulder.Angle -= 1;
			}
			SetServoAngle(&htim4, TIM_CHANNEL_4, Base_Shoulder.Angle);
			HAL_Delay(2);
		}

		if (Shoulder_Elbow.Angle != Shoulder_Elbow.SetAngle) {
			if (Shoulder_Elbow.SetAngle > Shoulder_Elbow.Angle) {
				Shoulder_Elbow.Angle += 1;
			} else {
				Shoulder_Elbow.Angle -= 1;
			}
			SetServoAngle(&htim4, TIM_CHANNEL_3, Shoulder_Elbow.Angle);
			HAL_Delay(2);
		}

		if (Elbow_Wrist.Angle != Elbow_Wrist.SetAngle) {
			if (Elbow_Wrist.SetAngle > Elbow_Wrist.Angle) {
				Elbow_Wrist.Angle += 1;
			} else {
				Elbow_Wrist.Angle -= 1;
			}
			SetServoAngle(&htim4, TIM_CHANNEL_2, Elbow_Wrist.Angle);
			HAL_Delay(2);
		}

		if (Wrist_Ulna.Angle != Wrist_Ulna.SetAngle) {
			if (Wrist_Ulna.SetAngle > Wrist_Ulna.Angle) {
				Wrist_Ulna.Angle += 1;
			} else {
				Wrist_Ulna.Angle -= 1;
			}
			SetServoAngle(&htim4, TIM_CHANNEL_1, Wrist_Ulna.Angle);
			HAL_Delay(2);
		}

		if (Ulna_Febur.Angle != Ulna_Febur.SetAngle) {
			if (Ulna_Febur.SetAngle > Ulna_Febur.Angle) {
				Ulna_Febur.Angle += 1;
			} else {
				Ulna_Febur.Angle -= 1;
			}
			SetServoAngle(&htim3, TIM_CHANNEL_2, Ulna_Febur.Angle);
			HAL_Delay(2);
		}

		if (Febur_Gripper.Angle != Febur_Gripper.SetAngle) {
			if (Febur_Gripper.SetAngle > Febur_Gripper.Angle) {
				Febur_Gripper.Angle += 1;
			} else {
				Febur_Gripper.Angle -= 1;
			}
			SetServoAngle(&htim3, TIM_CHANNEL_1, Febur_Gripper.Angle);
			HAL_Delay(2);
		}

		if (Right_finger.Angle != Right_finger.SetAngle) {
			if (Right_finger.SetAngle > Right_finger.Angle) {
				Right_finger.Angle += 1;
			} else {
				Right_finger.Angle -= 1;
			}
			SetServoAngle(&htim2, TIM_CHANNEL_2, Right_finger.Angle);
			HAL_Delay(2);
		}

		if ((CurrentMillis - PreviousMillis) >= 16 && ReadyToSend == 1) {

			sprintf(send, "B_S %d,S_E %d,E_W %d,W_U %d,U_F %d,F_G %d,R_f %d\n",
					Base_Shoulder.Angle, Shoulder_Elbow.Angle,
					Elbow_Wrist.Angle, Wrist_Ulna.Angle, Ulna_Febur.Angle, Febur_Gripper.Angle, Right_finger.Angle);
			HAL_UART_Transmit_IT(&huart1, (uint8_t*) send, strlen(send));
			ReadyToSend = 0;
			PreviousMillis = CurrentMillis;
		}

		if ((CurrentMillis - PreviousMillis_LED) >= 1000) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

			PreviousMillis_LED = CurrentMillis;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
	while (1) {
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
