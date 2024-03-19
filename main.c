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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	#define BUFFER_SIZE 128
	#define INT16_TO_FLOAT 1.0f / (32768.0f)
	#define FLOAT_TO_INT16 32768.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
static const uint8_t CHIP_ADDRESS = 0b00100000;
static const uint32_t spi_timeout_write = 1000;

enum SLAVE_MASTER { SLAVE, MASTER };
enum CONTROL_MODE_REGISTERS { MODE_CONTROL_1 = 1, DAC_CONTROL, DAC_VOL_N_MIX,
	DAC_CH_A_VOL, DAC_CH_B_VOL, ADC_CONTROL, MODE_CONTROL_2 };
enum MODE_CONTROL_1_BITS { DAC_DIF0, DAC_DIF1, DAC_DIF2, MSBAR, RATIO0, RATIO1, M0, M1};
//MSBAR & DAC_DIF0
enum DAC_CONTROL_BITS { INV_B, INV_A, RMP_DN, RMP_UP, DEM0, DEM1, FILT_SEL, AMUTE};
//AMUTE
enum DAC_VOL_N_MIX_BITS { ATAPI0, ATAPI1, ATAPI2, ATAPI3, ZEROCROSS, SOFT, BEQA};

enum DAC_CH_X_VOL_BITS { VOL0, VOL1, VOL2, VOL3, VOL4, VOL5, VOL6, MUTE};

enum ADC_CONTROL_BITS { HPFDISABLEB, HPFDISABLEA, MUTEB, MUTEA, ADC_DIF, DITHER16};
//ADC_DIF, MUTEB

enum MODE_CONTROL_2_BITS { PDN, CPEN, FREEZE, MUTECAEQB, LOOP};

uint8_t MODE1_BITS = (1 << DAC_DIF0);// | ( 1 << RATIO1);// & (1 << MSBAR);// & (1<<RATIO0) & (1<<RATIO1);
uint8_t MODE2_BITS = (1 << CPEN) | (1 << PDN);
uint8_t MODE2_BITS_POWER_UP = (1 << CPEN);
uint8_t ADC_BITS = (1 << ADC_DIF) | (1 << MUTEB);
volatile uint8_t register_read = 0;

int16_t adcData[BUFFER_SIZE];
int16_t dacData[BUFFER_SIZE];

static volatile int16_t* inBufferPointer;
static volatile int16_t* outBufferPointer = &dacData[0];

uint8_t dataReadyFlag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Activate_Control_Port(void){
	HAL_I2C_Mem_Write(&hi2c1, CHIP_ADDRESS, MODE_CONTROL_2, 1, &MODE2_BITS, 1, spi_timeout_write);
	HAL_Delay(1);
}

void Set_Remaining_Registers(void){
	//I2S for DAC, AUTOMUTE
	HAL_I2C_Mem_Write(&hi2c1, CHIP_ADDRESS, MODE_CONTROL_1, 1, &MODE1_BITS, 1, spi_timeout_write);
	HAL_Delay(1);
	//I2S for ADC, Mute Channel B
	HAL_I2C_Mem_Write(&hi2c1, CHIP_ADDRESS, ADC_CONTROL, 1, &ADC_BITS, 1, spi_timeout_write);
	HAL_Delay(1);
}

void TestRead(void){

	HAL_I2C_Mem_Read(&hi2c1, CHIP_ADDRESS, MODE_CONTROL_1, 1, &register_read, 1, spi_timeout_write);
}

void Power_Up(void){
	HAL_I2C_Mem_Write(&hi2c1, CHIP_ADDRESS, MODE_CONTROL_2, 1, &MODE2_BITS_POWER_UP, 1, spi_timeout_write);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
}

enum buffer {back, front = BUFFER_SIZE / 2};

void Transmit_Data(enum buffer selection){
	//HAL_UART_Transmit_DMA(&huart1, &adcData[selection], BUFFER_SIZE/2);
	return;
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	//Pointer for which half to process (front half}
	inBufferPointer = &adcData[0];
	outBufferPointer = &dacData[0];
	Transmit_Data(back);
	dataReadyFlag = 1;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
	//Pointer for which half to process (back half}
	inBufferPointer  = &adcData[BUFFER_SIZE / 2];
	outBufferPointer = &dacData[BUFFER_SIZE / 2];
	Transmit_Data(front);
	dataReadyFlag = 1;
}

void Process_Data(){
	static float leftIn, leftOut;
	static float rightIn, rightOut;

	for(uint8_t n = 0; n < (BUFFER_SIZE / 2) - 1; n += 2){
		leftIn = INT16_TO_FLOAT * inBufferPointer[n];
		if (leftIn > 1.0f){
			leftIn -= 2.0f;
		}
		//transform data
		leftOut = leftIn;

		outBufferPointer[n] = (int16_t) (FLOAT_TO_INT16 * leftOut);

		rightIn = INT16_TO_FLOAT * inBufferPointer[n + 1];
		if (rightIn > 1.0f){
			rightIn -= 2.0f;
		}
		//transform data
		rightOut = rightIn;

		outBufferPointer[n + 1] = (int16_t) (FLOAT_TO_INT16 * rightOut);
	}
	dataReadyFlag = 0;
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
  MX_I2S2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef status = HAL_I2SEx_TransmitReceive_DMA(&hi2s2, (uint16_t *) dacData, (uint16_t *) adcData, BUFFER_SIZE);
  if (status != HAL_OK){
	  Error_Handler();
  }
  HAL_GPIO_WritePin(CODEC_RST_GPIO_Port, CODEC_RST_Pin, 0);
  HAL_Delay(5000);

  HAL_GPIO_WritePin(CODEC_RST_GPIO_Port, CODEC_RST_Pin, 1);
  HAL_Delay(3);
  Activate_Control_Port();
  Set_Remaining_Registers();
  TestRead();
  Power_Up();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ( dataReadyFlag ){
		  Process_Data();
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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(8);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 16;
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  huart1.Init.Mode = UART_MODE_TX;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSB_GPIO_Port, CSB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CODEC_RST_GPIO_Port, CODEC_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CSB_Pin */
  GPIO_InitStruct.Pin = CSB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CSB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin CODEC_RST_Pin */
  GPIO_InitStruct.Pin = LED_Pin|CODEC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(1000);
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(250);
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(250);
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(250);
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
