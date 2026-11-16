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

#endif /* MOTOR_ENCODER_TEST_H */
