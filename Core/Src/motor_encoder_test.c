/**
  ******************************************************************************
  * @file    motor_encoder_test.c
  * @brief   モータ + エンコーダ動作確認用テスト（Issue #18）
  *
  * 原本 Motor_Current_Measurement.c の Test 5 / Test 6 とその部品
  * （Motor*_Set, Encoder*_Get/Reset, Serial_*）を、ピン配置の変更だけで移植した。
  *
  * [ピン配置（原本 → 本プロジェクト）]
  *  Motor1 IN1/IN2 : PA6/PA7 (TIM3 CH1/CH2)  -> PA6/PA7 (TIM13_CH1 / TIM14_CH1, AF9)
  *  Motor2 IN1/IN2 : PA0/PA1 (TIM2)          -> PA1/PA0 (TIM2 CH2/CH1, CubeMX 生成の htim2。IN1=PA1, IN2=PA0)
  *  Encoder1 A/B   : PA8/PA9 (TIM1)          -> PA9/PA8 (TIM1 CH2=A相, CH1=B相, ラベル ENC1_A=PA9, ENC1_B=PA8, htim1)
  *  Encoder2 A/B   : PB6/PB7 (TIM4 CH1/CH2)  -> PB7/PB6 (TIM4 CH2=A相, CH1=B相, htim4)
  *  USART2         : PA2/PA3 (変更なし)
  *
  * 原本の LED(PB12-15)・スイッチ(PC10-13)・ADC(PC0-3) は液晶・超音波と競合するため移植していない。
  ******************************************************************************
  */
#include "motor_encoder_test.h"
#include "lcd_ili9341.h"
#include "ultrasonic.h"
#include "position_controller.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim1;     /* Encoder1（CubeMX 生成） */
extern TIM_HandleTypeDef htim2;     /* Motor2 PWM（CubeMX 生成） */
extern TIM_HandleTypeDef htim4;     /* Encoder2（CubeMX 生成） */
extern UART_HandleTypeDef huart2;   /* Serial（CubeMX 生成） */

extern TIM_HandleTypeDef htim13;    /* Motor1 IN1 (PA6)（CubeMX 生成） */
extern TIM_HandleTypeDef htim14;    /* Motor1 IN2 (PA7)（CubeMX 生成） */

/* ============================================================
   Constants（原本のまま）
   ============================================================ */
#define PWM_PERIOD      55999UL     /* CubeMX の TIM2/TIM13/TIM14 の Period と同値 */
#define PWM_DUTY_MAX       50       /* Maximum duty cycle [%] */
#define ENCODER_TEST_DUTY  50       /* Duty cycle for encoder test [%] */
#define UART_TIMEOUT    100U        /* UART transmit timeout [ms] */

/* 位置制御テスト（Test 5, Issue #31）のパラメータ */
#define MM_PER_COUNT_1     0.039f   /* TODO(#31): 仮の値（2048PPR x4逓倍・出力軸側の計算値）。実機で測った値に置き換える（Encoder1） */
#define MM_PER_COUNT_2     0.039f   /* TODO(#31): 同上（Encoder2）。左右別々に測る */
#define POS_ENC1_SIGN      1.0f     /* 前進でカウントが増えるなら +1、減るなら -1（Encoder1） */
#define POS_ENC2_SIGN      1.0f     /* 同上（Encoder2） */
#define POS_KP             0.10f    /* 比例ゲイン [duty % / mm]（仮。偏差 100mm で duty 10%） */
#define POS_DEADBAND_MM    5.0f     /* 残り偏差がこれ以下で止める [mm] */
#define POS_DUTY_MIN       10.0f    /* 動き出せる最小 duty [%]（仮） */
#define POS_TARGET_MAX_MM  2000     /* 入力できる目標距離の上限 [mm] */
#define POS_LOOP_MS        50U      /* 制御周期 [ms] */

static char g_tx_buf[256];


/* ============================================================
   Function Prototypes
   ============================================================ */
static void MotorEncoderTest_Init(void);
static void Test_PositionControl(void);
static void Test_Motor2_Encoder2(void);
static void Test_Encoder_Monitor(void);
static void Test_Buzzer(void);
static void Test_Lcd(void);
static void Test_Ultrasonic(void);
static void Print_Menu(void);
static void Serial_Print(const char *str);
static void Serial_Println(const char *str);
static uint8_t Serial_ReceiveChar(char *c, uint32_t timeout_ms);
static uint8_t Serial_WaitEnter(void);
static uint8_t Serial_ReadNumber(uint32_t *value);
static void Motor1_Set(int16_t duty_percent);
static void Motor2_Set(int16_t duty_percent);
static int32_t Encoder1_Get(void);
static int32_t Encoder2_Get(void);
static void Encoder1_Reset(void);
static void Encoder2_Reset(void);

/* ============================================================
   Entry
   ============================================================ */
void MotorEncoderTest_Run(void)
{
    MotorEncoderTest_Init();
    Print_Menu();

    while (1)
    {
        char cmd = 0;
        if (Serial_ReceiveChar(&cmd, 200))
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)&cmd, 1, UART_TIMEOUT);
            Serial_Println("");

            switch (cmd)
            {
                case '3': Test_Ultrasonic();        break;
                case '5': Test_PositionControl();   break;
                case '6': Test_Motor2_Encoder2();   break;
                case '7': Test_Encoder_Monitor();   break;
                case '8': Test_Buzzer();            break;
                case '9': Test_Lcd();               break;
                default:
                    Serial_Println("[!] Invalid input. Enter 3, 5, 6, 7, 8 or 9.");
                    break;
            }
            Print_Menu();
        }
    }
}

static void MotorEncoderTest_Init(void)
{
    /* TODO(#18 レビュー指摘・見送り): Encoder1 (PA9/PA8) と Encoder2 (PB6/PB7) のプルアップは、
     *   CubeMX の .ioc に反映できなかった（生成コードが GPIO_NOPULL のまま）ため、ここで手書きで再設定している。
     *   この初期化を通らない本番コードではピンが浮く。.ioc で Pull-up にできたら、この再設定を削除すること。 */

    /* Encoder2 (PB6/PB7): CubeMX 生成の MspInit は NOPULL なので、プルアップに再設定する */
    GPIO_InitTypeDef enc2_gpio = {0};
    enc2_gpio.Pin       = ENC_2B_Pin | ENC_2A_Pin;
    enc2_gpio.Mode      = GPIO_MODE_AF_PP;
    enc2_gpio.Pull      = GPIO_PULLUP;
    enc2_gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    enc2_gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &enc2_gpio);

    /* Encoder1 (PA9/PA8): CubeMX の MspInit は NOPULL なので、プルアップに再設定する */
    GPIO_InitTypeDef enc1_gpio = {0};
    enc1_gpio.Pin       = ENC1_A_Pin | ENC1_B_Pin;
    enc1_gpio.Mode      = GPIO_MODE_AF_PP;
    enc1_gpio.Pull      = GPIO_PULLUP;
    enc1_gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    enc1_gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &enc1_gpio);

    /* Encoder1: TIM1, Encoder2: TIM4, Motor2: TIM2（いずれも CubeMX 生成） */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);          /* Motor2 IN1 (PA1) */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);          /* Motor2 IN2 (PA0) */
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);         /* Motor1 IN1 (PA6) */
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);         /* Motor1 IN2 (PA7) */

    Motor1_Set(0);
    Motor2_Set(0);
}

/* ============================================================
   Menu
   ============================================================ */
static void Print_Menu(void)
{
    Serial_Println("\r\n================================================");
    Serial_Println("   Motor + Encoder test  (Issue #18)");
    Serial_Println("   STM32F446RE  Nucleo-64");
    Serial_Println("================================================");
    Serial_Println(" 3 : Ultrasonic (4 sensors, distance in mm)");
    Serial_Println(" 5 : Position control (enter target distance, Motor1 + Motor2)");
    Serial_Println(" 6 : Motor2 + Encoder2 test");
    Serial_Println(" 7 : Encoder monitor (no motor, rotate by hand)");
    Serial_Println(" 8 : Buzzer test (PB10)");
    Serial_Println(" 9 : LCD test (color fill + color bands)");
    Serial_Println("------------------------------------------------");
    Serial_Print  (" Enter number > ");
}

/* ============================================================
   Test 7: Encoder monitor（モータは回さない。手で軸を回して確認する）
   ============================================================ */
static void Test_Encoder_Monitor(void)
{
    Serial_Println("\r\n[TEST 7] Encoder monitor (motors OFF)");
    Serial_Println("Rotate the shaft by hand. 'q' to stop, 'r' to reset counts.");

    Motor1_Set(0);
    Motor2_Set(0);
    Encoder1_Reset();
    Encoder2_Reset();

    while (1)
    {
        char c = 0;
        if (Serial_ReceiveChar(&c, 10))
        {
            if (c == 'q' || c == 'Q') break;
            if (c == 'r' || c == 'R') { Encoder1_Reset(); Encoder2_Reset(); }
        }

        /* 切り分け用: ピンの生のレベル（0/1）も表示する。軸を回して変化すれば配線・電源は生きている */
        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "\r  Enc1 cnt=%7ld (PA9 A=%d PA8 B=%d)   Enc2 cnt=%7ld (PB7 A=%d PB6 B=%d)   ",
                 (long)Encoder1_Get(),
                 (int)HAL_GPIO_ReadPin(ENC1_A_GPIO_Port, ENC1_A_Pin),
                 (int)HAL_GPIO_ReadPin(ENC1_B_GPIO_Port, ENC1_B_Pin),
                 (long)Encoder2_Get(),
                 (int)HAL_GPIO_ReadPin(ENC_2A_GPIO_Port, ENC_2A_Pin),
                 (int)HAL_GPIO_ReadPin(ENC_2B_GPIO_Port, ENC_2B_Pin));
        Serial_Print(g_tx_buf);

        HAL_Delay(50);
    }

    Serial_Println("\r\n[TEST 7] Done.");
}

/* ============================================================
   Test 8: Buzzer test（PB10 = BUZZER）
   PB10 を GPIO 出力で HIGH/LOW する（アクティブブザー想定。ブザーは 2SC1815L-Y 経由で駆動）。
   ============================================================ */
static void Test_Buzzer(void)
{
    Serial_Println("\r\n[TEST 8] Buzzer test (PB10)");
    Serial_Println("Press Enter to start, 'q' to cancel...");

    if (!Serial_WaitEnter()) { Serial_Println("Cancelled."); return; }

    /* 短く3回: 200ms ON / 200ms OFF */
    Serial_Println("  Short beep x3");
    for (int i = 0; i < 3; i++)
    {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
    HAL_Delay(500);

    /* 長く1回: 1000ms ON */
    Serial_Println("  Long beep x1");
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

    Serial_Println("[TEST 8] Done. Check that the buzzer sounded 3 short + 1 long.");
}

/* ============================================================
   Test 3: 超音波（本処理）。接続確認のあと、距離[mm]を 60ms ごとに表示する。'q' で終了
   ============================================================ */
static void Test_Ultrasonic(void)
{
    Serial_Println("\r\n[TEST 3] Ultrasonic. 'q' to stop.");

    Ultrasonic_Init();      /* TIM5 を 1us のストップウォッチとして動かす（何度呼んでも問題ない） */

    uint8_t ok_mask = 0U;
    int check = Ultrasonic_CheckConnection(&ok_mask);
    for (int i = 0; i < US_SENSOR_NUM; i++)
    {
        snprintf(g_tx_buf, sizeof(g_tx_buf), "sensor%d: %s\r\n", i,
                 (ok_mask & (1U << i)) ? "OK" : "NG (no response)");
        Serial_Print(g_tx_buf);
    }
    snprintf(g_tx_buf, sizeof(g_tx_buf), "result=%d ok_mask=0x%02X\r\n", check, ok_mask);
    Serial_Print(g_tx_buf);

    int pair_id = 0;
    while (1)
    {
        char c = 0;
        if (Serial_ReceiveChar(&c, 10) && (c == 'q' || c == 'Q')) break;

        /* 60ms ごとに組を交互に鳴らす（受信待ち 10ms + 待ち 50ms） */
        Ultrasonic_Trigger(pair_id);
        pair_id = (pair_id + 1) % US_PAIR_NUM;
        HAL_Delay(50);

        float dist[US_SENSOR_NUM];
        int   no_resp[US_SENSOR_NUM];
        Ultrasonic_GetResult(dist, no_resp);

        Serial_Print("\r");
        for (int i = 0; i < US_SENSOR_NUM; i++)
        {
            if (no_resp[i]) snprintf(g_tx_buf, sizeof(g_tx_buf), "   -  ");
            else            snprintf(g_tx_buf, sizeof(g_tx_buf), "%4d  ", (int)dist[i]);
            Serial_Print(g_tx_buf);
        }
    }

    Serial_Println("\r\n[TEST 3] Done.");
}

/* ============================================================
   Test 9: LCD test（色の全画面塗り → 色の帯）
   ============================================================ */
static void Test_Lcd(void)
{
    Serial_Println("[TEST 9] LCD test");
    Serial_Println("Fill: red, green, blue, white, black (0.5 s each), then 4 color bands.");
    Lcd_Test();
    Serial_Println("[TEST 9] Done. Expect bands from top: red, green, blue, yellow; a white square at top-left.");
}

/* ============================================================
   Test 5: 位置制御テスト（Issue #31 / 親 #3）
   目標距離 [mm] を入力し、Motor1 + Motor2 をエンコーダで測った距離で制御して、目標に着いたら止める。
   制御器（P 制御）は position_controller.c にあり、ここでは偏差を渡して出力を受け取るだけ。
   ============================================================ */

/* Return: 1=入力できた（*value に値）, 0='q' でキャンセル。数字と Enter だけ受け付ける（BS で 1 文字消す） */
static uint8_t Serial_ReadNumber(uint32_t *value)
{
    uint32_t v = 0U;
    int digits = 0;
    char c = 0;

    while (1)
    {
        if (!Serial_ReceiveChar(&c, HAL_MAX_DELAY)) continue;

        if (c == 'q' || c == 'Q') return 0U;
        if ((c == 13 || c == 10) && digits > 0) { *value = v; return 1U; }
        if (c >= '0' && c <= '9' && digits < 5)         /* 5 桁まで（桁あふれ防止） */
        {
            v = v * 10U + (uint32_t)(c - '0');
            digits++;
            HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, UART_TIMEOUT);
        }
        else if ((c == 8 || c == 127) && digits > 0)
        {
            v /= 10U;
            digits--;
            Serial_Print("\b \b");
        }
    }
}

static void Test_PositionControl(void)
{
    Serial_Println("\r\n[TEST 5] Position control (Motor1 + Motor2, Encoder1 + Encoder2)");
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "Enter target distance [mm] (1-%d) then Enter, 'q' to cancel. While running, 'q' stops.\r\n", POS_TARGET_MAX_MM);
    Serial_Print(g_tx_buf);
    Serial_Print("Target [mm] > ");

    uint32_t target = 0U;
    while (1)
    {
        if (!Serial_ReadNumber(&target)) { Serial_Println("\r\nCancelled."); return; }
        if (target >= 1U && target <= (uint32_t)POS_TARGET_MAX_MM) break;
        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "\r\n[!] Out of range. Enter 1-%d.\r\nTarget [mm] > ", POS_TARGET_MAX_MM);
        Serial_Print(g_tx_buf);
    }
    snprintf(g_tx_buf, sizeof(g_tx_buf), "\r\nTarget = %lu mm. Press Enter to start, 'q' to cancel...\r\n",
             (unsigned long)target);
    Serial_Print(g_tx_buf);
    if (!Serial_WaitEnter()) { Serial_Println("Cancelled."); return; }

    /* 左右それぞれに制御器を持つ（今は同じ P ゲイン） */
    PosCtrl ctrl[2];
    PosCtrl_Init(&ctrl[0], POS_KP);
    PosCtrl_Init(&ctrl[1], POS_KP);

    Encoder1_Reset();
    Encoder2_Reset();

    uint8_t done[2] = { 0U, 0U };
    uint8_t stopped = 0U;           /* 1: 'q' で途中停止した */
    float   dist[2] = { 0.0f, 0.0f };
    float   duty[2] = { 0.0f, 0.0f };

    while (!(done[0] && done[1]))
    {
        dist[0] = (float)Encoder1_Get() * POS_ENC1_SIGN * MM_PER_COUNT_1;
        dist[1] = (float)Encoder2_Get() * POS_ENC2_SIGN * MM_PER_COUNT_2;

        for (int i = 0; i < 2; i++)
        {
            float error = (float)target - dist[i];                  /* 偏差 [mm] */

            if (done[i] || error <= POS_DEADBAND_MM)                /* 着いた、または行き過ぎ → 止めるだけ（後退しない） */
            {
                done[i] = 1U;
                duty[i] = 0.0f;
                continue;
            }

            float out = PosCtrl_Update(&ctrl[i], error);            /* 偏差 → 操作量 [%] */
            if (out < POS_DUTY_MIN)               out = POS_DUTY_MIN;
            if (out > (float)PWM_DUTY_MAX)        out = (float)PWM_DUTY_MAX;
            duty[i] = out;
        }

        Motor1_Set((int16_t)(duty[0] + 0.5f));
        Motor2_Set((int16_t)(duty[1] + 0.5f));

        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "\r  L: %5ld mm (duty %2d%%)   R: %5ld mm (duty %2d%%)   target %lu mm   ",
                 (long)dist[0], (int)(duty[0] + 0.5f), (long)dist[1], (int)(duty[1] + 0.5f),
                 (unsigned long)target);
        Serial_Print(g_tx_buf);

        /* 制御周期の待ちを兼ねて、'q' が押されたら途中でも止める */
        char key = 0;
        if (Serial_ReceiveChar(&key, POS_LOOP_MS) && (key == 'q' || key == 'Q'))
        {
            stopped = 1U;
            break;
        }
    }

    Motor1_Set(0);
    Motor2_Set(0);
    if (stopped) Serial_Println("\r\n  Stopped by user ('q').");

    /* 止まった後、惰性で動いた分まで含めて最終距離を測る */
    HAL_Delay(500);
    dist[0] = (float)Encoder1_Get() * POS_ENC1_SIGN * MM_PER_COUNT_1;
    dist[1] = (float)Encoder2_Get() * POS_ENC2_SIGN * MM_PER_COUNT_2;
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "\r\n  Final  L = %ld mm (error %+ld)   R = %ld mm (error %+ld)\r\n",
             (long)dist[0], (long)dist[0] - (long)target,
             (long)dist[1], (long)dist[1] - (long)target);
    Serial_Print(g_tx_buf);
    Serial_Println(stopped ? "[TEST 5] Stopped before reaching the target."
                           : "[TEST 5] Done. Check that the error is within +/-50 mm.");
}

/* ============================================================
   Test 6: Motor2 + Encoder2 Test（LED 表示のみ除いて原本のまま）
   ============================================================ */
static void Test_Motor2_Encoder2(void)
{
    Serial_Println("\r\n[TEST 6] Motor2 + Encoder2 Test");
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "Motor2 ramps 0->%d%% duty then holds. Encoder2 count is displayed.\r\n",
             ENCODER_TEST_DUTY);
    Serial_Print(g_tx_buf);
    Serial_Println("Press Enter to start, 'q' to stop...");

    char c = 0;
    if (!Serial_WaitEnter()) { Serial_Println("Cancelled."); return; }

    Encoder2_Reset();

    snprintf(g_tx_buf, sizeof(g_tx_buf), "  Ramp up: 0 -> %d%%\r\n", ENCODER_TEST_DUTY);
    Serial_Print(g_tx_buf);
    for (int d = 0; d <= ENCODER_TEST_DUTY; d += 5)
    {
        Motor2_Set((int16_t)d);
        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "  Motor2 duty = %2d%%\r\n", d);
        Serial_Print(g_tx_buf);
        HAL_Delay(300);
    }
    snprintf(g_tx_buf, sizeof(g_tx_buf), "  Hold at %d%% -- press 'q' to stop\r\n", ENCODER_TEST_DUTY);
    Serial_Print(g_tx_buf);

    int32_t prev_count = 0;
    int32_t count = 0;

    while (1)
    {
        if (Serial_ReceiveChar(&c, 10))
        {
            if (c == 'q' || c == 'Q') break;
        }

        count = Encoder2_Get();
        int32_t delta = count - prev_count;
        prev_count = count;

        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "\r  Encoder2 count = %7ld  (delta/50ms = %+ld)   ",
                 (long)count, (long)delta);
        Serial_Print(g_tx_buf);

        HAL_Delay(50);
    }

    Motor2_Set(0);

    count = Encoder2_Get();
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "\r\n  Final count = %ld\r\n", (long)count);
    Serial_Print(g_tx_buf);
    Serial_Println("[TEST 6] Done. Check that count changed while motor was running.");
}

/* ============================================================
   Serial（原本のまま）
   ============================================================ */
static void Serial_Print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, (uint16_t)strlen(str), UART_TIMEOUT);
}

static void Serial_Println(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, (uint16_t)strlen(str), UART_TIMEOUT);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, UART_TIMEOUT);
}

/* Return: 1=received, 0=timeout */
static uint8_t Serial_ReceiveChar(char *c, uint32_t timeout_ms)
{
    return (HAL_UART_Receive(&huart2, (uint8_t *)c, 1, timeout_ms) == HAL_OK) ? 1U : 0U;
}

/* 開始待ち: Enter（CR または LF）で 1、'q' / 'Q' で 0 を返す。それ以外の入力は無視して待ち続ける */
static uint8_t Serial_WaitEnter(void)
{
    char c = 0;
    while (1)
    {
        if (!Serial_ReceiveChar(&c, HAL_MAX_DELAY)) continue;
        if (c == 13 || c == 10) return 1U;
        if (c == 'q' || c == 'Q') return 0U;
    }
}

/* ============================================================
   Motor Control（TB67H450）
   positive -> forward (IN1=PWM, IN2=0) / negative -> reverse (IN1=0, IN2=PWM) / zero -> coast
   ============================================================ */

/* duty をクランプし、正転側 CCR / 逆転側 CCR に振り分ける共通処理 */
static void Motor_Set(TIM_HandleTypeDef *h_in1, uint32_t ch_in1,
                      TIM_HandleTypeDef *h_in2, uint32_t ch_in2, int16_t duty_percent)
{
    if (duty_percent >  PWM_DUTY_MAX) duty_percent =  PWM_DUTY_MAX;
    if (duty_percent < -PWM_DUTY_MAX) duty_percent = -PWM_DUTY_MAX;

    uint32_t cmp = (uint32_t)((PWM_PERIOD + 1U) * (uint32_t)abs(duty_percent) / 100U);

    __HAL_TIM_SET_COMPARE(h_in1, ch_in1, (duty_percent > 0) ? cmp : 0U);
    __HAL_TIM_SET_COMPARE(h_in2, ch_in2, (duty_percent < 0) ? cmp : 0U);
}

/* Motor1: IN1=PA6 (TIM13_CH1), IN2=PA7 (TIM14_CH1) */
static void Motor1_Set(int16_t duty_percent)
{
    Motor_Set(&htim13, TIM_CHANNEL_1, &htim14, TIM_CHANNEL_1, duty_percent);
}

/* Motor2: IN1=PA1 (TIM2_CH2), IN2=PA0 (TIM2_CH1) */
static void Motor2_Set(int16_t duty_percent)
{
    Motor_Set(&htim2, TIM_CHANNEL_2, &htim2, TIM_CHANNEL_1, duty_percent);
}

/* ============================================================
   Encoder Functions（x4 resolution）
   ============================================================ */

/* 16bit タイマのカウンタは 65535 の次に 0 へ戻る（一周する）。
 * そのまま使うと、境界をまたいだ瞬間に値と差分が大きく飛ぶ。
 * そこで「前回の読み値との差」を一周を考慮して求め、32bit の累積カウントに足していく。
 * 前提: 読み出しの間隔で進むカウントが 32768 未満であること（テストでは 50ms ごとに読むので十分）。 */
typedef struct
{
    TIM_HandleTypeDef *htim;
    uint16_t last;      /* 前回読んだカウンタ値 */
    int32_t  total;     /* 累積カウント */
} EncoderCount;

static EncoderCount s_enc1 = { &htim1, 0U, 0 };
static EncoderCount s_enc2 = { &htim4, 0U, 0 };

static int32_t Encoder_Get(EncoderCount *e)
{
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(e->htim);
    e->total += (int16_t)(uint16_t)(now - e->last);     /* 16bit の引き算で一周を吸収し、符号付きにする */
    e->last   = now;
    return e->total;
}

static void Encoder_Reset(EncoderCount *e)
{
    __HAL_TIM_SET_COUNTER(e->htim, 0);
    e->last  = 0U;
    e->total = 0;
}

/* Encoder1: TIM1 エンコーダモード */
static int32_t Encoder1_Get(void)   { return Encoder_Get(&s_enc1); }
static void    Encoder1_Reset(void) { Encoder_Reset(&s_enc1); }

/* Encoder2: TIM4 エンコーダモード */
static int32_t Encoder2_Get(void)   { return Encoder_Get(&s_enc2); }
static void    Encoder2_Reset(void) { Encoder_Reset(&s_enc2); }
