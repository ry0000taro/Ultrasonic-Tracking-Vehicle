/**
  ******************************************************************************
  * @file    ultrasonic.h
  * @brief   超音波計測ドライバ（HC-SR04 x4、非同期計測）
  *
  * 仕様: Notion「超音波計測 機能モジュール表」の関数仕様表に従う。
  *   センサ番号 0:外側左 1:内側左 2:内側右 3:外側右（回路図の U1〜U4 に対応と仮定）
  *   組番号     0:センサ0・2  1:センサ1・3（隣り合うセンサは同じ組に入れない）
  ******************************************************************************
  */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define US_SENSOR_NUM      4
#define US_PAIR_NUM        2

/* 測定有効範囲 [mm]（範囲判定は Distance_Process 側。接続確認でも使用） */
#define US_RANGE_MIN_MM    20.0f
#define US_RANGE_MAX_MM    4000.0f

/** TIM5 を 1us カウントで起動する。HAL_Init / クロック設定 / GPIO・EXTI 初期化の後に1回呼ぶ。 */
void Ultrasonic_Init(void);

/** 前の組の未応答センサを応答なしにし、pair_id の組の Trig に同時に 10us パルスを出す。
 *  0:開始成功 / -1:組番号が範囲外。60ms ごとに呼ぶ。 */
int Ultrasonic_Trigger(int pair_id);

/** Echo 立ち下がりで呼ぶ。距離 = pulse_us * 0.343 / 2 を保存。0:成功 / -1:センサ番号が範囲外。 */
int Ultrasonic_OnEcho(int sensor_id, float pulse_us);

/** 最新の距離[mm]と応答なしフラグ(1:応答なし)をコピーして返す。常に 0。 */
int Ultrasonic_GetResult(float dist_mm[US_SENSOR_NUM], int no_resp[US_SENSOR_NUM]);

/** EXTI 割り込み（Echo 両エッジ）から呼ぶ。HAL_GPIO_EXTI_Callback の中で呼ぶこと。 */
void Ultrasonic_EXTI_Callback(uint16_t gpio_pin);

/** 接続確認。センサを1個ずつ順に鳴らし、応答して距離が 20〜4000mm に入るかを調べる。
 *  ok_mask: bit i = 1 ならセンサ i が正常（NULL可）。
 *  0:4個とも正常 / -1:応答のないセンサあり。
 *  周期計測（Ultrasonic_Trigger）を始める前に呼ぶ。約 4 x 60ms かかる。 */
int Ultrasonic_CheckConnection(uint8_t *ok_mask);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H */
