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
#include "arp.h"
#include "scan.h"
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
extern int ARP_table_index_defend;
extern const  uint8_t macaddr[6];
extern  struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}ARP_table_defend[5];
struct
{
   uint8_t ip[4];
   uint8_t mac[6];
}ARP_table_static[5];
extern void ARP_send_request_fake(uint8_t *ip_dest, uint8_t *ip_source, uint8_t * MAC_target);
extern void ARP_send_request_reborn(uint8_t *ip_dest, uint8_t *ip_source,uint8_t *MAC_dest, uint8_t * MAC_source);
extern void ARP_table_setIP_defend(uint8_t *ip_set);
extern int8_t ARP_table_checkIP_defend(uint8_t *ip_check);
extern void ARP_send_request(uint8_t *ip_dest);
uint8_t ip_server[4]={192,168,0,0};
uint8_t ip_ref[4]={192,168,0,0};
int count =0;
int led =1;
uint8_t mac_siemens[6]={0xE0,0xDC,0xA0};
extern int a[2];
extern int k;
extern uint8_t alarm;
uint8_t zero_ip[4]={0,0,0,0};
extern char debug_string[60];
int m =0,n=1;
//extern int check;
//		uint8_t ip1[4] = {192,168,0,102};
//		uint8_t mac1 [6] = {0xE0,0xDC,0xA0,0xB0,0xE9,0xBE};
//		uint8_t ip2[4] = {192,168,0,200};
//		uint8_t mac2 [6] = {0xE0,0xDC,0xA0,0xB0,0x8E,0x47};
//		uint8_t ip3[4] = {192,168,0,201};
//		uint8_t mac3 [6] = {0xE0,0xDC,0xA0,0xB0,0x8B,0x33};
//		uint8_t ip4[4] = {192,168,0,202};
//		uint8_t mac4 [6] = {0xE0,0xDC,0xA0,0xB0,0x8D,0xCE};
//		uint8_t ip5[4] = {192,168,0,50};
//		uint8_t mac5 [6] = {0x08,0x8F,0xC3,0x83,0x4A,0xA6};
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int t =0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	 if(htim->Instance == TIM2)
	 {
		 //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		 if(alarm == 1 || alarm == 3)
		 {
			 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
			 led++;
		 }
		 
			
			
		 
	 }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{

	if(GPIO_Pin == GPIO_PIN_0) 
	{
		if (GPIO_Pin == GPIO_PIN_11)  // Thay dúng chân INT c?a b?n
    {
        // Ð?t c? ho?c x? lý ng?t t?i dây
        // Ví d? don gi?n:
        extern volatile uint8_t enc_irq_flag;
        enc_irq_flag = 1;
    }
		led=1;
//		ARP_send_request_reborn(ip4, ip5, mac5);
//		ARP_send_request_reborn(ip5, ip4, mac4);
		GPIO_Pin = GPIO_PIN_0 -1;
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
//		memcpy(ARP_table_static[0].ip, ip1, 4);
//		memcpy(ARP_table_static[1].ip, ip2, 4);
//		memcpy(ARP_table_static[2].ip, ip3, 4);
//		memcpy(ARP_table_static[3].ip, ip4, 4);
//		memcpy(ARP_table_static[4].ip, ip5, 4);
//		memcpy(ARP_table_static[0].mac, mac1, 6);
//		memcpy(ARP_table_static[1].mac, mac2, 6);
//		memcpy(ARP_table_static[2].mac, mac3, 6);
//		memcpy(ARP_table_static[3].mac, mac4, 6);
//		memcpy(ARP_table_static[4].mac, mac5, 6);
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
	MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	UART_init(&huart1);
  ENC29J600_ini(&hspi1);
	for (int i=0;i<=255;i++)
		{
			ip_server[3]=i;
			ARP_send_request(ip_server);
      HAL_Delay(10);
		}
	UART_putString("Khoi dong thanh cong\r\nIn ra dia chi cac thiet bi luu trong bang\r\n\n");
//		for(int j =0;j<5;j++)
//		{
//		sprintf(debug_string,"Thiet bi %i Ip %i.%i.%i.%i = Mac %02X:%02X:%02X:%02X:%02X:%02X\r\n",j+1,ARP_table_static[j].ip[0],ARP_table_static[j].ip[1],
//		ARP_table_static[j].ip[2],	ARP_table_static[j].ip[3],ARP_table_static[j].mac[0],ARP_table_static[j].mac[1],ARP_table_static[j].mac[2],
//			ARP_table_static[j].mac[3],	ARP_table_static[j].mac[4],ARP_table_static[j].mac[5]);
//		UART_putString(debug_string);
//		}
//	UART_putString("\r\n");
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//ARP_table_checkIP(ip_fake);
	//*ARP_table_setIP(ip_fake,mac_fake);
	//ARP_send_request(ip_server);
		
		//ARP_send_request_reborn(ip1, ip2, mac1, mac2);		

HAL_TIM_Base_Start_IT(&htim2);

 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
		for (int i=200;i<=215;i++)
		{
			ip_server[3]=i;
			ARP_send_request(ip_server);
      HAL_Delay(10);
		}
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
			volatile uint8_t enc_irq_flag = 0;
  while (1)
  {
		
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
    /* USER CODE END WHILE */
			NET_loop();
			//HAL_Delay(20);
		
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
  htim2.Init.Period = 2499;
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_11;;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
