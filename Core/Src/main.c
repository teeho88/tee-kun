/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AT_Esp8266.h"
#include "tinhtoan.h"
#include "TJ_MPU6050.h"
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
char request[BUFFER_SIZE] = {0};
char TXDATA[100];
uint8_t clientID = 0;
uint8_t numClient = 0;
char *myssid = "ahaha";
char *mypass = "tanho8888";
char *serverIP = "192.168.1.100";
uint16_t serverPort = 80;

float T = 0; //s
float Pitch = 0, Roll = 0, Head = 0; //-pi->pi
float Vx = 0, Vy = 0;
float X = 0, Y = 0;
float ax = 0, ay = 0, az = 0; // m/s^2
float wx = 0, wy = 0, wz = 0; // rad/s

extern float accelScalingFactor, gyroScalingFactor;
extern RawData_Def Accel;
extern RawData_Def Gyro;

//test
uint32_t countTest = 0;
char *ptest;
uint16_t lentest = 0;
uint16_t errtest = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)request, BUFFER_SIZE);
		__HAL_UART_DISABLE_IT(&huart1,DMA_IT_HT);
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	//1. Initialise the MPU6050 module and I2C
  MPU6050_Init(&hi2c1);
  //2. Configure Accel and Gyro parameters
  myMpuConfig.Accel_Full_Scale = AFS_SEL_2g;
  myMpuConfig.ClockSource = Internal_8MHz;
  myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
  myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
  myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
  MPU6050_Config(&myMpuConfig);
	//3. Set communication to server
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)request, BUFFER_SIZE);
	__HAL_UART_DISABLE_IT(&huart1,DMA_IT_HT);

	ESP_Init(&huart1,STA_AP);
	HAL_StatusTypeDef state = ESP_CheckWifiConnect();
	while(state != HAL_OK) 
  {
		if(state == HAL_ERROR)
		{
			ESP_WifiConnect(myssid,mypass);
			break;
		}
		state = ESP_CheckWifiConnect();
	}
	ESP_TCP_CreateTransparentMode(serverIP, serverPort);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int32_t at[3] = {0};
	uint8_t count = 0;
	float z[3] = {0};
	uint32_t t_last = HAL_GetTick();
	for(int i = 0; i < 10; i++)
	{
		if(ReadI2C_MPU(&myMpuConfig) == HAL_OK)
		{
			at[0] += Accel.x;
			at[1] += Accel.y;
			at[2] += Accel.z;
			count++;
		}
	}
	z[0] = at[0]*accelScalingFactor/count; z[1] = at[1]*accelScalingFactor/count; z[2] = at[2]*accelScalingFactor/count;
	Q_init(z[0],z[1],z[2]);
  while (1)
  {
		if(strstr(request,"ket thuc"))
		{
			ESP_CloseTransparent();
			memset(request,0,BUFFER_SIZE);
		}
		else
		{
			int32_t at[3] = {0}, wt[3] = {0};
			uint8_t count = 0;
			float z[6] = {0};
			uint32_t t_cur = HAL_GetTick();
			T = (t_cur - t_last)*0.001f;
			t_last = t_cur;
			for(int i = 0; i < 3; i++)
			{
				if(ReadI2C_MPU(&myMpuConfig) == HAL_OK)
				{
					at[0] += Accel.x;
					at[1] += Accel.y;
					at[2] += Accel.z;
					wt[0] += Gyro.x;
					wt[1] += Gyro.y;
					wt[2] += Gyro.z;
					count++;
				}
			}
			if(!count) continue;
			// kalman filter
			z[0] = at[0]*accelScalingFactor/count; z[1] = at[1]*accelScalingFactor/count; z[2] = at[2]*accelScalingFactor/count;
			z[3] = wt[0]*gyroScalingFactor/count; z[4] = wt[1]*gyroScalingFactor/count; z[5] = wt[2]*gyroScalingFactor/count;
			float *x = KalmanFilter(z);
			ax = *x; ay = *(x+1); az = *(x+2); wx = *(x+3); wy = *(x+4); wz= *(x+5);
			updateQ();
			goc_Euler_Quat();
			updateV_Quat();
			X += Vx*T; Y += Vy*T; Pitch = Pitch*180/3.14; Roll = Roll*180/3.14; Head = Head*180/3.14;
			if(countTest%3==0)
			{
				sprintf(TXDATA,"%.1f,%.1f,%.1f,%.1f,%.1f\r\n",X,Y,Pitch,Roll,Head);
				ESP_TransparentSend(TXDATA);
			}
			countTest++;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 230400;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
