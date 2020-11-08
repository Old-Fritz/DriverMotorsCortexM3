/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
#define CONTROL_M1_PIN_PLUS     GPIO_PIN_0
#define CONTROL_M1_PIN_MINUS    GPIO_PIN_1
#define CONTROL_M2_PIN_PLUS     GPIO_PIN_2
#define CONTROL_M2_PIN_MINUS    GPIO_PIN_3
#define CONTROL_M3_PIN_PLUS     GPIO_PIN_4
#define CONTROL_M3_PIN_MINUS    GPIO_PIN_5
#define CONTROL_M4_PIN_PLUS     GPIO_PIN_6    
#define CONTROL_M4_PIN_MINUS    GPIO_PIN_7    
#define CONTROL_M5_PIN_PLUS     GPIO_PIN_8    
#define CONTROL_M5_PIN_MINUS    GPIO_PIN_9
#define CONTROL_M6_PIN_PLUS     GPIO_PIN_10
#define CONTROL_M6_PIN_MINUS    GPIO_PIN_11

#define CONTROL_GPIO_TYPE_DEF   GPIOB

#define CONTROL_COUNT 6

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA7 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
static uint16_t GetControlPin(uint8_t controlId, ControlType type)
{
  switch(controlId)
  {
  case 1:
    return type == MINUS ? CONTROL_M1_PIN_MINUS : CONTROL_M1_PIN_PLUS;
  case 2:
    return type == MINUS ? CONTROL_M2_PIN_MINUS : CONTROL_M2_PIN_PLUS;
  case 3:
    return type == MINUS ? CONTROL_M3_PIN_MINUS : CONTROL_M3_PIN_PLUS;
  case 4:
    return type == MINUS ? CONTROL_M4_PIN_MINUS : CONTROL_M4_PIN_PLUS;
  case 5:
    return type == MINUS ? CONTROL_M5_PIN_MINUS : CONTROL_M5_PIN_PLUS;
  case 6:
    return type == MINUS ? CONTROL_M6_PIN_MINUS : CONTROL_M6_PIN_PLUS;
  default:
    return 0;
  }
}

static void SetState(uint8_t controlId, ControlType type, GPIO_PinState  state)
{
  uint16_t pin = GetControlPin(controlId, type);
  HAL_GPIO_WritePin(CONTROL_GPIO_TYPE_DEF, pin, state);
}

void EnableControl(uint8_t controlId, ControlType type)
{
  GPIO_PinState newState = type == MINUS ? GPIO_PIN_RESET :  GPIO_PIN_SET;
  SetState(controlId, type, newState);
}

void DisableControl(uint8_t controlId, ControlType type)
{
  GPIO_PinState newState = type == MINUS ? GPIO_PIN_SET :  GPIO_PIN_RESET;
  SetState(controlId, type, newState);
}

uint8_t GetControlState(uint8_t controlId, ControlType type)
{
  uint16_t pin = GetControlPin(controlId, type);
  GPIO_PinState enableState = type == MINUS ? GPIO_PIN_RESET :  GPIO_PIN_SET;
  GPIO_PinState currentState = HAL_GPIO_ReadPin(CONTROL_GPIO_TYPE_DEF, pin);
  return currentState == enableState;
}
void ResetControl(uint8_t controlId, ControlType type)
{
  uint16_t pin = GetControlPin(controlId, type);
  HAL_GPIO_DeInit(CONTROL_GPIO_TYPE_DEF, pin);
}
void ResetControlAll()
{
  for( int i = 0; i < CONTROL_COUNT; i++ )
  {
    uint16_t pin = GetControlPin(i, PLUS);
    HAL_GPIO_DeInit(CONTROL_GPIO_TYPE_DEF, pin);
    pin = GetControlPin(i, MINUS);
    HAL_GPIO_DeInit(CONTROL_GPIO_TYPE_DEF, pin);
  }
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
