/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usbd_hid.h"
#include "usbd_def.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef __packed struct _mouseReport_t
{
  //byte 0
  uint8_t leftButton  : 1;
  uint8_t rightButton : 1;
  uint8_t midButton   : 1;
  uint8_t res_0003    : 5;
  //byte 1
  int8_t x;
  //byte 2
  int8_t y;
  //byte 3
  int8_t wheel;
} mouseReport_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_ADC_VALUE_X       	4095
#define MIN_ADC_VALUE_X       	0
#define MAX_ADC_VALUE_Y       	4095
#define MIN_ADC_VALUE_Y       	0
#define MAX_NATIVE_MOUSE_RANGE	20.f										//127.f - max for this constant
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
volatile uint8_t adcConversationFlag = 0;
volatile uint8_t buttonPressedFlag = 0;
volatile uint16_t adcValues[2];
mouseReport_t mouseReport;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_ClassTypeDef  USBD_HID;
uint8_t adcConversationCounter = 0;
uint8_t zeroSetupFlag = 0;
uint16_t zeroMinX = 65535;
uint16_t zeroMaxX = 0;
uint16_t zeroMinY = 65535;
uint16_t zeroMaxY = 0;
uint16_t rangeMinusX = 0;
uint16_t rangePlusX = 0;
uint16_t rangeMinusY = 0;
uint16_t rangePlusY = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief adc conversation callback (DMA)
  * @param hadc pointer to ADC descriptor
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
        adcConversationFlag = 1;
}

/**
  * @brief EXTI change state callback
  * @param GPIO_Pin number of EXTI pin
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_PIN_2 == GPIO_Pin)
    buttonPressedFlag = 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t tmpButtonState = 0;
  int8_t tmpX = 0;
  int8_t tmpY = 0;
  uint8_t *descriptor = 0;
  uint16_t length = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  descriptor = USBD_HID.GetHSConfigDescriptor(&length);
  descriptor[7] = 0x80;// self power - 0
  descriptor = USBD_HID.GetFSConfigDescriptor(&length);
  descriptor[7] = 0x80;// self power - 0
  descriptor = USBD_HID.GetOtherSpeedConfigDescriptor(&length);
  descriptor[7] = 0x80;// self power - 0
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  memset(&mouseReport, 0, sizeof(mouseReport_t));
  HAL_ADCEx_Calibration_Start(&hadc1);                    //calibrate adc
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adcValues, 2);   //start adc
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (0 != adcConversationFlag)
    {
      adcConversationFlag = 0;
      if (adcConversationCounter++ >= 100)                 // 2ms to approximate conversation
      {
        adcConversationCounter = 0;
        if (0 == zeroSetupFlag)
        {
          zeroSetupFlag = 1;
          rangeMinusX = zeroMinX - MIN_ADC_VALUE_X;
          rangePlusX = MAX_ADC_VALUE_X - zeroMaxX;
          rangeMinusY = zeroMinY - MIN_ADC_VALUE_Y;
          rangePlusY = MAX_ADC_VALUE_Y - zeroMaxY;
        }
         
        tmpX = (adcValues[0] >= zeroMinX && adcValues[0] <= zeroMaxX) ? 0 : (adcValues[0] < zeroMinX) ? 0 - (int8_t)((MAX_NATIVE_MOUSE_RANGE / rangeMinusX) * (zeroMinX - adcValues[0])) :
                                                                                                        (int8_t)((MAX_NATIVE_MOUSE_RANGE / rangePlusX) * (adcValues[0] - zeroMaxX));
        tmpY = (adcValues[1] >= zeroMinY && adcValues[1] <= zeroMaxY) ? 0 : (adcValues[1] < zeroMinY) ? 0 - (int8_t)((MAX_NATIVE_MOUSE_RANGE / rangeMinusY) * (zeroMinY - adcValues[1])) :
                                                                                                        (int8_t)((MAX_NATIVE_MOUSE_RANGE / rangePlusY) * (adcValues[1] - zeroMaxY));
        if (0 != tmpX || 0 != mouseReport.x || 0 != tmpY || 0 != mouseReport.y || tmpButtonState != mouseReport.leftButton || tmpButtonState == 1)
        {
          mouseReport.x = tmpX;
          mouseReport.y = tmpY;
          mouseReport.leftButton = tmpButtonState;
          USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&mouseReport, sizeof(mouseReport_t));
        }
      }
      else if (0 == zeroSetupFlag)
      {
        //X
        if (adcValues[0] > zeroMaxX)
          zeroMaxX = adcValues[0];
        if (adcValues[0] < zeroMinX)
          zeroMinX = adcValues[0];
        //Y
        if (adcValues[1] > zeroMaxY)
          zeroMaxY = adcValues[1];
        if (adcValues[1] < zeroMinY)
          zeroMinY = adcValues[1];
      }
      //TODO: may be get average value for report?
    }
    if (0 != buttonPressedFlag)
    {
      buttonPressedFlag = 0;
      tmpButtonState = (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)) ? 0 : 1;
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

  /*Configure GPIO pin : leftButton_Pin */
  GPIO_InitStruct.Pin = leftButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(leftButton_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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
