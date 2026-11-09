/**
  ******************************************************************************
  * @file    motor_encoder_test.h
  * @brief   モータ + エンコーダ動作確認用テスト（Issue #18）
  *          原本: 講義配布の診断プログラム Motor_Current_Measurement.c
  *          原本のロジックは変えず、ピン配置だけ本プロジェクト用に変更している。
  ******************************************************************************
  */
#ifndef MOTOR_ENCODER_TEST_H
#define MOTOR_ENCODER_TEST_H

#include "main.h"

/* メニューを表示してテストを繰り返す（戻らない）。USART2 115200bps で操作する。 */
void MotorEncoderTest_Run(void);

/* Encoder1（PC8/PC9）の EXTI 割り込みから呼ぶ。main.c の HAL_GPIO_EXTI_Callback に配線済み。 */
void MotorEncoderTest_EXTI_Callback(uint16_t gpio_pin);

#endif /* MOTOR_ENCODER_TEST_H */
