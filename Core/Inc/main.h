/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define START_STOP_S3_Pin GPIO_PIN_13
#define START_STOP_S3_GPIO_Port GPIOC
#define US1_ECHO_Pin GPIO_PIN_0
#define US1_ECHO_GPIO_Port GPIOC
#define US1_ECHO_EXTI_IRQn EXTI0_IRQn
#define US2_ECHO_Pin GPIO_PIN_1
#define US2_ECHO_GPIO_Port GPIOC
#define US2_ECHO_EXTI_IRQn EXTI1_IRQn
#define US3_ECHO_Pin GPIO_PIN_2
#define US3_ECHO_GPIO_Port GPIOC
#define US3_ECHO_EXTI_IRQn EXTI2_IRQn
#define US4_ECHO_Pin GPIO_PIN_3
#define US4_ECHO_GPIO_Port GPIOC
#define US4_ECHO_EXTI_IRQn EXTI3_IRQn
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define MOTOR1_IN1_Pin GPIO_PIN_6
#define MOTOR1_IN1_GPIO_Port GPIOA
#define MOTOR1_IN2_Pin GPIO_PIN_7
#define MOTOR1_IN2_GPIO_Port GPIOA
#define US4_TRIG_Pin GPIO_PIN_4
#define US4_TRIG_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define LED_D1_Pin GPIO_PIN_14
#define LED_D1_GPIO_Port GPIOB
#define LCD_SDI_Pin GPIO_PIN_15
#define LCD_SDI_GPIO_Port GPIOB
#define ENC1_B_Pin GPIO_PIN_8
#define ENC1_B_GPIO_Port GPIOC
#define ENC1_B_EXTI_IRQn EXTI9_5_IRQn
#define ENC1_A_Pin GPIO_PIN_9
#define ENC1_A_GPIO_Port GPIOC
#define ENC1_A_EXTI_IRQn EXTI9_5_IRQn
#define LCD_RESET_Pin GPIO_PIN_8
#define LCD_RESET_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_9
#define LCD_DC_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define US1_TRIG_Pin GPIO_PIN_10
#define US1_TRIG_GPIO_Port GPIOC
#define US2_TRIG_Pin GPIO_PIN_11
#define US2_TRIG_GPIO_Port GPIOC
#define US3_TRIG_Pin GPIO_PIN_12
#define US3_TRIG_GPIO_Port GPIOC
#define MOTOR2_IN1_Pin GPIO_PIN_4
#define MOTOR2_IN1_GPIO_Port GPIOB
#define MOTOR2_IN2_Pin GPIO_PIN_5
#define MOTOR2_IN2_GPIO_Port GPIOB
#define ENC2_B_Pin GPIO_PIN_6
#define ENC2_B_GPIO_Port GPIOB
#define ENC2_A_Pin GPIO_PIN_7
#define ENC2_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
