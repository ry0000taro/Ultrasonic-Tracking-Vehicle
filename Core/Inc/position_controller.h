/**
  ******************************************************************************
  * @file    position_controller.h
  * @brief   位置制御器（Issue #31 / 親 #3）
  *
  * 入力を「偏差」、出力を「ゲイン後の操作量」とする。
  * 今は P 制御だけだが、後で PI / PID に変える可能性があるため、制御の中身をこのファイルに閉じ込める。
  * 呼び出し側（テストなど）は PosCtrl_Update() に偏差を渡して戻り値を使うだけなので、
  * 中身を差し替えても呼び出し側は変えなくてよい。
  *
  * 単位は呼び出し側が決める（例：偏差 [mm] → 出力 duty [%]。このときゲインは [%/mm]）。
  * 出力の上下限・最小値・不感帯・後退の禁止は、ここでは扱わず、呼び出し側が行う。
  ******************************************************************************
  */
#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

typedef struct
{
    float kp;       /* 比例ゲイン */
    /* PI / PID にするときは、ki, kd と積分値・前回偏差などの内部状態をここに足す */
} PosCtrl;

/* 制御器を初期化する（内部状態があればここで 0 にする） */
void PosCtrl_Init(PosCtrl *c, float kp);

/* 制御開始時などに内部状態を捨てる（P 制御では何もしない。PI / PID にしたとき積分値を 0 にする） */
void PosCtrl_Reset(PosCtrl *c);

/* 偏差を受け取り、ゲインをかけた操作量を返す */
float PosCtrl_Update(PosCtrl *c, float error);

#endif /* POSITION_CONTROLLER_H */
