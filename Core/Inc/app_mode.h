/**
  ******************************************************************************
  * @file    app_mode.h
  * @brief   起動モードの選択（Test / SpeedControl / PositionControl / Combined）
  *
  * 起動直後に USART2(115200bps) でモードを選ぶ。
  * 注意: 制御層の CtrlMode（MODE_POS / MODE_SPEED / MODE_POS_SPEED、common_types.h 予定）とは別物。
  *       こちらは「このファームウェアを何として動かすか」の起動モード。
  ******************************************************************************
  */
#ifndef APP_MODE_H
#define APP_MODE_H

#include "main.h"

typedef enum {
    APP_MODE_TEST = 0,       /* これまでの動作確認（Test メニュー） */
    APP_MODE_SPEED,          /* 速度制御モード */
    APP_MODE_POSITION,       /* 位置制御モード */
    APP_MODE_COMBINED        /* 位置・速度制御モード */
} AppMode;

/* メニューを表示してモードが選ばれるまで待つ（ブロッキング）。 */
AppMode AppMode_Select(void);

/* 選ばれたモードを実行する。
 *  - Test は戻らない（リセットでモード選択に戻る）。
 *  - それ以外は現在未実装。メッセージを出して戻る（呼び出し側がモード選択をやり直す）。 */
void AppMode_Run(AppMode mode);

#endif /* APP_MODE_H */
