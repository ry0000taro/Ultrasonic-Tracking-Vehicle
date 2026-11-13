/**
  ******************************************************************************
  * @file    distance_process.h
  * @brief   距離データ処理（制御層）。超音波4個の距離から追従対象の位置(X,Y)を推定する
  *
  * 仕様: Notion「超音波計測 機能モジュール表」の Distance_Process に従う。
  *   座標系: 原点=底板前端の線と中心線の交点 / X右が正 / Y前方が正 [mm]
  *   センサ番号 0:外側左 1:内側左 2:内側右 3:外側右（ultrasonic.h と同じ）
  ******************************************************************************
  */
#ifndef DISTANCE_PROCESS_H
#define DISTANCE_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DP_SENSOR_NUM          4

/* 測定有効範囲 [mm]（範囲外は無効） */
#define DP_RANGE_MIN_MM        20.0f
#define DP_RANGE_MAX_MM        4000.0f

/* 対象検出距離 [mm]。有効値のどれかがこれ以内なら detect=1 */
#define DP_DETECT_MM           300.0f

/* 推定の初回初期値 (0, 追従距離) の追従距離 [mm]。仮の値（Notion に未記載） */
#define DP_INIT_FOLLOW_MM      500.0f

/* ガウス・ニュートン法の打ち切り条件 */
#define DP_GN_MAX_ITER         20
#define DP_GN_EPS_MM           1.0f

/* センサの取り付け位置 (xi, yi) [mm]。図面の取付ボルト穴から読んだ仮の値。
 * 発音面中心に直す（TBD）ときはこの2つの配列だけ直す。 */
#define DP_SENSOR_X_MM         { -88.0f, -38.0f,  38.0f,  88.0f }
#define DP_SENSOR_Y_MM         {  37.0f,  20.0f,  20.0f,  37.0f }

/** 処理結果 */
typedef struct {
    float dist_mm[DP_SENSOR_NUM]; /* センサ0〜3の距離[mm]。無効は -1 */
    int   detect;                 /* 対象検出フラグ 1:検出 */
    float x_mm;                   /* 対象の位置X[mm]（estimated=0 のときは 0） */
    float y_mm;                   /* 対象の位置Y[mm]。位置制御の現在距離（同上） */
    int   estimated;              /* 位置の推定に成功したか 1:成功 */
} UsResult;

/** 距離の有効判定・対象検出・位置(X,Y)推定を行い out に入れる。
 *  0:更新成功 / -1:位置を推定できない（有効値が1個以下・収束しない。他の値は更新する）
 *  / -2:out が NULL。
 *  位置推定の前回値は内部(static)で保持する。60msごとの制御周期から呼ぶ。 */
int Distance_Process(const float dist_mm[DP_SENSOR_NUM], UsResult *out);

#ifdef __cplusplus
}
#endif

#endif /* DISTANCE_PROCESS_H */
