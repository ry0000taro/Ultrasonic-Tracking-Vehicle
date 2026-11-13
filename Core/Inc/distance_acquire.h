/**
  ******************************************************************************
  * @file    distance_acquire.h
  * @brief   距離データの周期取得（超音波の発火 → 結果の読み出し → 距離データ処理）
  *
  * Notion「関数一覧（層別）」7章の 60ms 周期処理
  *   Ultrasonic_Trigger → Ultrasonic_GetResult → Distance_Process
  * を、HAL_Delay で待たずに（ノンブロッキングで）回す。
  * main ループや各モードの周期処理から DistanceAcq_Task() を呼び続け、
  * 最新の距離と位置 (X, Y) を DistanceAcq_Get() で読む。
  *
  * 周期の基準は HAL_GetTick()（SysTick, 1ms）。TIM7(10ms) への移行は SOFT-13 で決める。
  ******************************************************************************
  */
#ifndef DISTANCE_ACQUIRE_H
#define DISTANCE_ACQUIRE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ultrasonic.h"
#include "distance_process.h"

/* 取得周期 [ms]。組0・組1 を交互に発火するので、各センサは 2 倍（120ms）ごとに更新される。 */
#define DISTANCE_ACQ_PERIOD_MS  60U

/** 取得を初期化する（Ultrasonic_Init を呼ぶ）。周期処理を始める前に1回呼ぶ。
 *  Ultrasonic_CheckConnection を使うときは、その後（周期取得の前）に DistanceAcq_Init を呼んでよい。 */
void DistanceAcq_Init(void);

/** 周期処理。メインループなどから頻繁に呼ぶ（待たずにすぐ戻る）。
 *  DISTANCE_ACQ_PERIOD_MS が経過していれば、結果の読み出しと距離データ処理を行い、次の組を発火する。
 *  1:今回の呼び出しで新しいデータができた / 0:まだ（周期の途中、または最初の発火のみ）。 */
int DistanceAcq_Task(void);

/** 最新の距離[mm]・応答なしフラグ(1:応答なし)・距離データ処理の結果を返す。
 *  いずれも NULL 可。データがまだ一度もできていないときは -1（出力は変更しない）、できていれば 0。 */
int DistanceAcq_Get(float dist_mm[US_SENSOR_NUM], int no_resp[US_SENSOR_NUM], UsResult *res);

#ifdef __cplusplus
}
#endif

#endif /* DISTANCE_ACQUIRE_H */
