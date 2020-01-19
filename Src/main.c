/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdbool.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Текущее состояние работы драйвера линейных двигателей. */
enum TypeWork {
  /* Режим инициализации. В данном режиме происходит
  расчет минимальных и максимальных положений кисти и пальцев протеза. */
  InitializationMode,
  
  /* Режим ожидания. Механические действия не исполняются. */
  SleepMode,
  
  /* Режим установки новых положений. */
  SettingPositionMode,
  
  /* Режим ошибки. Работа невозможна. */
  ErrorMode
};

/* Структура для работы с движущимися частями протеза (пальцами и кистью). 
* Хранит информацию о расчетах энкордера и физическом подключении протеза.
*/
typedef struct {
  /* Текущая позиция пальца в угле. */
  uint16_t Position;
  
  /* Структура GPIO к которой подключен вывод мотора для движения вперед
  (Движение вперед - при подаче логической единицы на этот пин мотор будет
  разжимать палец). */
  GPIO_TypeDef* GPIO_MotorForward;
  
  /* Номер пина к которому подключен вывод мотора для движения вперед. */
  uint16_t GPIO_Motor_PinForward;
  
  /* Структура GPIO к которой подключен вывод мотора для движения назад
  (Движение назад - при подаче логической единицы на этот пин мотор будет
  сжимать палец). */
  GPIO_TypeDef* GPIO_MotorBackward;
  
  /* Номер пина к которому подключен вывод мотора для движения назад. */
  uint16_t GPIO_Motor_PinBackward;
} FingerStruct;

/* Позиции всех движущихся частей протеза.*/
typedef struct {
  enum TypeWork CurrentRegime;
  FingerStruct* PointerFinger;
  FingerStruct* MiddleFinger;
  FingerStruct* RingFinder;
  FingerStruct* LittleFinger;
  FingerStruct* ThumbFinger;
  FingerStruct* Brush;
} HandStruct;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Основные переменные */

/* Основная конфигурация протеза для осуществления движений. 
 * На ее основе работает вся прошивка МК. */
HandStruct* HandConfig;

/* CAN BUS PV Variables */
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

/* ADC PV Variables */
volatile uint16_t ADC_Data[6];

/* UART data receive */
uint8_t dataRx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Объявление функций. */

/** 
* @brief Выполняет инициализацию конфигурации протеза.
* Устанавливает пины, к которым подключены моторы пальцев.
* @retval Структура, в которой содержится конфигурация протеза.
*/
HandStruct* Hand_Init();

/** 
* @brief Выполняет инициализацию конфигурации отдельного пальца. Выполняет 
*       установку конфигурации пина в соотвесвтующие поля структуры.
* @param  motorForward Структура GPIO к которой подключен вывод мотора для движения вперед.
*       (Движение вперед - при подаче логической единицы на этот пин мотор будет разжимать палец).
* @param  pinForward Номер пина к которому подключен вывод мотора для движения вперед.
* @param  motorBackward Структура GPIO к которой подключен вывод мотора для движения назад. 
*       (Движение назад - при подаче логической единицы на этот пин мотор будет сжимать палец).
* @param  pinBackward Номер пина к которому подключен вывод мотора для движения вперед.
* @retval Структура, в которой содержится конфигурация отдельного пальца.
*/
FingerStruct* Finger_Init(GPIO_TypeDef *motorForward, uint16_t pinForward, GPIO_TypeDef *motorBackward, uint16_t pinBackward);

/** 
* @brief Выполняет инициализацию конфигурации отдельного пальца. Выполняет 
* @retval None
*/
void SendTelemetryByCAN();

/* Реализация функций. */

void SendTelemetryByCAN(HandStruct* handConfig)
{
  TxData[0] = handConfig->CurrentRegime;
  TxData[1] = handConfig->PointerFinger->Position;
  TxData[2] = handConfig->MiddleFinger->Position;
  TxData[3] = handConfig->RingFinder->Position;
  TxData[4] = handConfig->LittleFinger->Position;
  TxData[5] = handConfig->ThumbFinger->Position;
  TxData[6] = handConfig->Brush->Position;
  TxData[7] = handConfig->Brush->Position >> 8;
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

HandStruct* Hand_Init()
{
  HandStruct* newHandConfig = (HandStruct*)malloc(sizeof(HandStruct));
  memset(newHandConfig, 0, sizeof(HandStruct));
  
  newHandConfig->PointerFinger = Finger_Init(GPIOB, GPIO_PIN_0, 
                                             GPIOB, GPIO_PIN_1);
  
  return newHandConfig;
}

FingerStruct* Finger_Init(GPIO_TypeDef *motorForward, uint16_t pinForward, GPIO_TypeDef *motorBackward, uint16_t pinBackward)
{
  FingerStruct* configFinger = (FingerStruct*)malloc(sizeof(FingerStruct));
  memset(configFinger, 0, sizeof(FingerStruct));
  
  configFinger->GPIO_MotorForward = motorForward;
  configFinger->GPIO_Motor_PinForward = pinForward;
  configFinger->GPIO_MotorBackward = motorBackward;
  configFinger->GPIO_Motor_PinBackward = pinBackward;
  
  return configFinger;
}

/**
* @brief  Остановка движения мотора.
*/
void Finger_Stop(FingerStruct* finger)
{
  HAL_GPIO_WritePin(finger->GPIO_MotorForward, finger->GPIO_Motor_PinForward, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(finger->GPIO_MotorBackward, finger->GPIO_Motor_PinBackward, GPIO_PIN_RESET);

  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

/**
* @brief  Движение мотора вперед.
*/
void Finger_Forward(FingerStruct* finger)
{
  HAL_GPIO_WritePin(finger->GPIO_MotorForward, finger->GPIO_Motor_PinForward, GPIO_PIN_SET);
  HAL_GPIO_WritePin(finger->GPIO_MotorBackward, finger->GPIO_Motor_PinBackward, GPIO_PIN_RESET);

//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

/**
* @brief  Движение мотора назад.
*/
void Finger_Backward(FingerStruct* finger)
{
  HAL_GPIO_WritePin(finger->GPIO_MotorForward, finger->GPIO_Motor_PinForward, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(finger->GPIO_MotorBackward, finger->GPIO_Motor_PinBackward, GPIO_PIN_SET);
  
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

/**
* @brief  Выполняет расчет текущего кол-ва оборотов устройства.
* @param  resistorValue Значение, полученное 
* с резистивного датчика, определяющего поворот мотора.
*/
void Calculate_Turns(int resistorValue)
{
  
}

/**
* @brief  Выполняет расчет текущего кол-ва оборотов устройства.
* @param  resistorValue Значение, полученное 
* с резистивного датчика, определяющего поворот мотора.
*/
void UART_Transmit_Text(char* text)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)text, strlen(text)+1, 0xFFFF);
}

/**
* @brief Выполняет необходимую программную иницилизацию CAN и 
* переменных для работы с ней.
* @retval None
*/
void CAN_Init()
{
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0X0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_CAN_ActivateNotification (&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    Error_Handler();
  }
  
  TxHeader.StdId = 0x3E9;
  TxHeader.ExtId = 0x01;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;
  TxData[0] = 0;
  TxData[1] = 0;
  TxData[2] = 0;
  TxData[3] = 0;
  TxData[4] = 0;
  TxData[5] = 0;
  TxData[6] = 0;
  TxData[7] = 0;
}

void CAN_HandlePackage(uint8_t* dataPackage)
{
  if (dataPackage[0] == 0x04)
  {
    // Запрос установки новых положений.
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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  HandConfig = Hand_Init();
  
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &ADC_Data,6);
  // HAL_ADC_Stop_DMA(&hadc1);
  
  CAN_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  
  HAL_UART_Receive_IT(&huart1, &dataRx, 1);
  
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    for(int i=0;i<6;i++)
    {
      //float u;
      char str[9];
      //u =((float)ADC_Data[i])*3/4096;//занесём результат преобразований в переменную
      //sprintf(str,"%.2fv",u);//преобразуем результат в строку
      sprintf(str, "%d", ADC_Data[i]);//преобразуем результат в строку
      
      UART_Transmit_Text(str);
      break;
      // UART_Transmit_Text(" ");
    }
    
    UART_Transmit_Text("\n");
    
    //HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

    HAL_Delay(5);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* 
* @brief Обработчик прерываний таймеров. 
* htim2 отвечает за отправку телеметрии на контроллер управления.
* @param htim таймер, вызвавший прерывание.
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  if (htim == &htim2)
  {
    SendTelemetryByCAN(HandConfig);
  }
}

/**
* @brief  Обработчик прерываний АЦП. По прерыванию АЦП 1 извлекается
значение резистивного датчика и передается в функцию-обработчик для расчет текущего поворота двигателя. 
* @param  hadc1 АЦП, вызвавшее прерывание.
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
  int ADC_Data = HAL_ADC_GetValue(hadc1);
  Calculate_Turns(ADC_Data);
}

///**
//* @brief  Обработчик прерываний успешной отправки сообщения по CAN.
//* @param  hcan CAN, вызвавший прерывание.
//*/
//void HAL_CAN_TxMailBox0CompleteCallback(CAN_HandleTypeDef *hcan)
//{
//  
//}

/**
* @brief  Обработчик принятия сообщения по CAN.
* @param  hcan CAN, вызвавший прерывание.
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
  CAN_HandlePackage(RxData);
}

/**
* @brief  Обработчик принятия сообщения по UART.
* @param  huart UART, вызвавший прерывание.
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    switch(dataRx)
    {
    case '0':
      Finger_Stop(HandConfig->PointerFinger);
      break;
    case '1':
      Finger_Forward(HandConfig->PointerFinger);
      break;
    case '2':
      Finger_Backward(HandConfig->PointerFinger);
      break;
    }
    
    HAL_UART_Receive_IT(&huart1, &dataRx, 1);
  }
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
