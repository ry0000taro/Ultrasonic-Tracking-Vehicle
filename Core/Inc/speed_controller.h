/**
  ******************************************************************************
  * @file    speed_controller.h
  * @brief   速度制御器（Issue #33 / 親 #5）。速度PI制御。
  *
  * 入力を「偏差（目標速度 − 現在速度）」、出力を「操作量（デューティ比）」とする。
  * 制御の中身（PI）をこのファイルに閉じ込め、呼び出し側は偏差を渡して戻り値を使うだけにする。
  *
  * 単位は呼び出し側が決める（例：偏差 [mm/s]、周期 dt [s]、出力 duty [%]。
  * このとき kp は [%/(mm/s)]、ki は [%/(mm/s·s)]）。
  * 積分値には上限を設ける（出力が飽和したまま積分が増え続けるのを防ぐ）。
  * 出力の上下限（後退の禁止など）は、ここでは扱わず、呼び出し側が行う。
  ******************************************************************************
  */
#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

typedef struct
{
    float kp;           /* 比例ゲイン */
    float ki;           /* 積分ゲイン */
    float integ;        /* 積分値（偏差 × 時間の累計） */
    float integ_max;    /* 積分項（ki × integ）の上限。±この値で止める（出力と同じ単位） */
} SpeedCtrl;

void SpeedCtrl_Init(SpeedCtrl *c, float kp, float ki, float integ_max);

/* 積分値を 0 にする（制御の開始時・停止時に呼ぶ） */
void SpeedCtrl_Reset(SpeedCtrl *c);

/* 偏差と周期 dt [s] を受け取り、操作量を返す */
float SpeedCtrl_Update(SpeedCtrl *c, float error, float dt);

#endif /* SPEED_CONTROLLER_H */
