/**
  ******************************************************************************
  * @file    emergency_stop.h
  * @brief   緊急停止（ソフトウェアでモータを止める）
  *
  * 緊急停止は、モータ電源の押しボタン（SW PUSHSW。PWR-03）を押して 12V を切る。このボタンはノーマルクローズ形で
  * 自己保持しない（押している間だけ 12V が切れ、離すと戻る。2026-11-20 に実機で確認）。
  * マイコンはその 12V を分圧した信号を PB1（ESTOP_SENSE）で読む（HIGH = モータ電源 ON、LOW = OFF）。
  *   - PB1 は EXTI1 を Echo（PC1）が使っているため割り込みにできない。1ms ごとの SysTick から
  *     EmergencyStop_Tick1ms() を呼び、10ms 周期で読む。raw が 5 回連続で LOW なら OFF と確定して
  *     モータを止める（約 50ms）。ON への確定は 2 回連続で HIGH（約 20ms）。1 回だけの LOW（ノイズ）では止めない。
  *   - モータ電源が OFF と確定したら、モータの IN1/IN2 を全部 Low にして停止状態を保持する。
  *     解除は EmergencyStop_Clear() だけ（ボタンを離して 12V が戻っただけでは解除されない）。
  * ファームウェアが固まると効かない。マイコンに頼らない停止（12V を切るボタン自体）は別に残すこと。
  ******************************************************************************
  */
#ifndef EMERGENCY_STOP_H
#define EMERGENCY_STOP_H

#include "main.h"

/* 緊急停止（モータ出力の 0 化とラッチ）を有効/無効にする。起動時は無効。
 * 無効の間も、モータ電源の状態の検出（PowerOn / OffCount）は続くが、モータには触らない。 */
void EmergencyStop_Enable(uint8_t enable);

/* 緊急停止中なら 1。モータを動かす処理は、これが 1 の間は出力を 0 にすること。 */
uint8_t EmergencyStop_IsActive(void);

/* 緊急停止を解除する（操作者が明示的に再開するときだけ呼ぶ）。 */
void EmergencyStop_Clear(void);

/* モータ電源の状態（確定後）。1 = ON、0 = OFF。OFF→ON の検出（INP-01 の再開）に使う。 */
uint8_t EmergencyStop_PowerOn(void);

/* モータ電源が ON → OFF に変わった回数（確定後）。テスト用。 */
uint32_t EmergencyStop_OffCount(void);

/* SysTick（1ms）から呼ぶ。stm32f4xx_it.c の SysTick_Handler に配線済み。 */
void EmergencyStop_Tick1ms(void);

#endif /* EMERGENCY_STOP_H */
