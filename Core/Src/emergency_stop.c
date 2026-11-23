/**
  ******************************************************************************
  * @file    emergency_stop.c
  * @brief   緊急停止（ソフトウェアでモータを止める）。説明は emergency_stop.h を参照。
  ******************************************************************************
  */
#include "emergency_stop.h"

/* モータの PWM タイマ（CubeMX 生成）: Motor1 = TIM13 CH1 / TIM14 CH1、Motor2 = TIM2 CH2 / CH1 */
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim2;

/* PB1（ESTOP_SENSE）: モータ 12V の分圧（約 3.0V）。HIGH = モータ電源 ON */
#define ESTOP_POWER_ON_LEVEL   GPIO_PIN_SET
#define ESTOP_SAMPLE_PERIOD_MS 10U      /* 読み取り周期 [ms] */
#define ESTOP_OFF_CONFIRM_SAMPLES 5U    /* raw が 5 回連続で LOW（0）なら OFF と確定して止める（10ms x 5 = 約 50ms） */
#define ESTOP_ON_CONFIRM_SAMPLES  2U    /* raw が 2 回連続で HIGH（1）なら ON と確定する（10ms x 2 = 約 20ms） */

static volatile uint8_t  s_enabled   = 0U;   /* 1 = 緊急停止が有効（起動時は無効） */
static volatile uint8_t  s_active    = 0U;   /* 1 = 緊急停止中 */
static volatile uint8_t  s_power_on  = 0U;   /* 確定したモータ電源の状態 */
static volatile uint32_t s_off_count = 0U;
static uint8_t           s_ready     = 0U;   /* 起動直後の最初の読み取りを済ませたか */
static uint8_t           s_diff      = 0U;   /* 確定状態と違う値が続いたサンプル数 */
static uint8_t           s_ms        = 0U;

void     EmergencyStop_Enable(uint8_t enable) { s_enabled = enable ? 1U : 0U; }
uint8_t  EmergencyStop_IsActive(void) { return s_active; }
void     EmergencyStop_Clear(void)    { s_active = 0U; }
uint8_t  EmergencyStop_PowerOn(void)  { return s_power_on; }
uint32_t EmergencyStop_OffCount(void) { return s_off_count; }

static void stop_motors(void)
{
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2,  TIM_CHANNEL_2, 0U);
}

void EmergencyStop_Tick1ms(void)
{
    if (++s_ms < ESTOP_SAMPLE_PERIOD_MS) {
        return;
    }
    s_ms = 0U;

    uint8_t now = (HAL_GPIO_ReadPin(ESTOP_SENSE_GPIO_Port, ESTOP_SENSE_Pin) == ESTOP_POWER_ON_LEVEL) ? 1U : 0U;

    if (!s_ready) {                 /* 起動直後の状態を初期値にする（起動時の OFF は停止として数えない） */
        s_power_on = now;
        s_ready    = 1U;
        return;
    }

    if (now == s_power_on) {
        s_diff = 0U;
    } else if (++s_diff >= (now ? ESTOP_ON_CONFIRM_SAMPLES : ESTOP_OFF_CONFIRM_SAMPLES)) {
        s_power_on = now;
        s_diff     = 0U;
        if (!s_power_on) {          /* ON → OFF */
            s_off_count++;
            if (s_enabled) {        /* 有効なときだけ、モータを止めてラッチする */
                stop_motors();
                s_active = 1U;
            }
        }
    }
}
