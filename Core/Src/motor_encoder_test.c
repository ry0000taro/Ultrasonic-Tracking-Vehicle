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
#include "lcd.h"
#include "ultrasonic.h"
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
#define PWM_PERIOD      55999UL     /* CubeMX の TIM3/TIM13/TIM14 の Period と同値 */
#define PWM_DUTY_MAX       50       /* Maximum duty cycle [%] */
#define ENCODER_TEST_DUTY  50       /* Duty cycle for encoder test [%] */
#define UART_TIMEOUT    100U        /* UART transmit timeout [ms] */

static char g_tx_buf[256];


/* ============================================================
   Function Prototypes
   ============================================================ */
static void MotorEncoderTest_Init(void);
static void Test_Motors_Encoders(void);
static void Test_Motor1_Encoder1(void);
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
                case '4': Test_Motors_Encoders();   break;
                case '5': Test_Motor1_Encoder1();   break;
                case '6': Test_Motor2_Encoder2();   break;
                case '7': Test_Encoder_Monitor();   break;
                case '8': Test_Buzzer();            break;
                case '9': Test_Lcd();               break;
                default:
                    Serial_Println("[!] Invalid input. Enter 3, 4, 5, 6, 7, 8 or 9.");
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

    /* Encoder1: TIM1, Encoder2: TIM4, Motor2: TIM3（いずれも CubeMX 生成） */
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
    Serial_Println(" 4 : Motor1 + Motor2 + Encoder1 + Encoder2 (all at once)");
    Serial_Println(" 5 : Motor1 + Encoder1 test");
    Serial_Println(" 6 : Motor2 + Encoder2 test");
    Serial_Println(" 7 : Encoder monitor (no motor, rotate by hand)");
    Serial_Println(" 8 : Buzzer test (PB10)");
    Serial_Println(" 9 : LCD test (color fill, then STOP / FOLLOW / ERROR)");
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
   Test 9: LCD test（MSP2807。CS=PB12, RESET=PA10, DC/RS=PB14, SCK=PB13, SDI=PB15）
   単色(赤→緑→青)のあと、状態表示(停止中 → 追従中 → エラー)を順に表示する。
   ============================================================ */
static void Test_Lcd(void)
{
    Serial_Println("\r\n[TEST 9] LCD test (MSP2807, ILI9341 assumed)");
    Serial_Println("Press Enter to start, 'q' to cancel...");
    if (!Serial_WaitEnter()) { Serial_Println("Cancelled."); return; }

    Lcd_Init();

    /* 単色 赤→緑→青 を順に表示（表示されるか、色が合っているかの確認） */
    Serial_Println("  Fill: red -> green -> blue");
    Lcd_FillScreen(LCD_RGB(255, 0, 0));
    HAL_Delay(600);
    Lcd_FillScreen(LCD_RGB(0, 255, 0));
    HAL_Delay(600);
    Lcd_FillScreen(LCD_RGB(0, 0, 255));
    HAL_Delay(600);

    /* 状態表示: 停止中 → 追従中 → エラー（各2秒） */
    Serial_Println("  State: STOP (red / square)");
    Lcd_DrawState(LCD_STATE_STOP);
    HAL_Delay(2000);
    Serial_Println("  State: FOLLOW (green / triangle)");
    Lcd_DrawState(LCD_STATE_FOLLOW);
    HAL_Delay(2000);
    Serial_Println("  State: ERROR (yellow / warning)");
    Lcd_DrawState(LCD_STATE_ERROR);

    Serial_Println("[TEST 9] Done. Check: R,G,B fills, then STOP(red), FOLLOW(green), ERROR(yellow). Last one stays.");
}

/* ============================================================
   Test 4: モータ 2 個とエンコーダ 2 個を同時に動かす
   Motor1 と Motor2 を同じデューティで同時にランプアップして保持し、Encoder1/Encoder2 の値を同時に表示する。
   ============================================================ */
static void Test_Motors_Encoders(void)
{
    Serial_Println("\r\n[TEST 4] Motor1 + Motor2 + Encoder1 + Encoder2 (simultaneous)");
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "Motor1 and Motor2 ramp 0->%d%% duty together, then hold. Both encoder counts are displayed.\r\n",
             ENCODER_TEST_DUTY);
    Serial_Print(g_tx_buf);
    Serial_Println("Press Enter to start, 'q' to stop...");

    if (!Serial_WaitEnter()) { Serial_Println("Cancelled."); return; }

    Encoder1_Reset();
    Encoder2_Reset();

    /* 300ms(=50ms x 6) ごとに 5% ずつ上げ、ENCODER_TEST_DUTY% に達したら保持する。毎周回 'q' を確認する */
    Serial_Println("  Ramp up in steps of 5% -- press 'q' to stop");

    int32_t prev1 = Encoder1_Get();
    int32_t prev2 = Encoder2_Get();
    int d = 0;
    char c = 0;

    Motor1_Set(0);
    Motor2_Set(0);
    for (int tick = 0; ; tick++)
    {
        if (Serial_ReceiveChar(&c, 10))
        {
            if (c == 'q' || c == 'Q') break;
        }

        if (tick % 6 == 0 && d <= ENCODER_TEST_DUTY)
        {
            Motor1_Set((int16_t)d);
            Motor2_Set((int16_t)d);
            d += 5;
        }

        int32_t count1 = Encoder1_Get();
        int32_t count2 = Encoder2_Get();
        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "\r  Enc1 = %7ld (d=%+5ld)   Enc2 = %7ld (d=%+5ld)   ",
                 (long)count1, (long)(count1 - prev1), (long)count2, (long)(count2 - prev2));
        Serial_Print(g_tx_buf);
        prev1 = count1;
        prev2 = count2;

        HAL_Delay(50);
    }

    Motor1_Set(0);
    Motor2_Set(0);

    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "\r\n  Final: Enc1 = %ld, Enc2 = %ld\r\n", (long)Encoder1_Get(), (long)Encoder2_Get());
    Serial_Print(g_tx_buf);
    Serial_Println("[TEST 4] Done. Check that both counts changed while both motors ran.");
}

/* ============================================================
   Test 5: Motor1 + Encoder1 Test（LED 表示のみ除いて原本のまま）
   ============================================================ */
static void Test_Motor1_Encoder1(void)
{
    Serial_Println("\r\n[TEST 5] Motor1 + Encoder1 Test");
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "Motor1 ramps 0->%d%% duty then holds. Encoder1 count is displayed.\r\n",
             ENCODER_TEST_DUTY);
    Serial_Print(g_tx_buf);
    Serial_Println("Press Enter to start, 'q' to stop...");

    char c = 0;
    if (!Serial_WaitEnter()) { Serial_Println("Cancelled."); return; }

    Encoder1_Reset();
    Encoder2_Reset();

    /* 0% から ENCODER_TEST_DUTY% まで 300ms(=50ms x 6) ごとに 5% ずつ上げ、到達後は保持する。
     * 昇圧中も含め、ループのたびに 'q' を確認する。 */
    Serial_Println("  Ramp up in steps of 5% -- press 'q' to stop");

    int32_t prev_count = Encoder1_Get();
    int32_t count = 0;
    int d = 0;

    Motor1_Set(0);
    for (int tick = 0; ; tick++)
    {
        if (Serial_ReceiveChar(&c, 10))
        {
            if (c == 'q' || c == 'Q') break;
        }

        if (tick % 6 == 0 && d <= ENCODER_TEST_DUTY)
        {
            Motor1_Set((int16_t)d);
            d += 5;
        }

        count = Encoder1_Get();
        int32_t delta = count - prev_count;
        prev_count = count;

        /* Motor1 に付いているエンコーダがどちらか分からないため、Encoder2 も併記する */
        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "\r  Encoder1 count = %7ld  (delta/50ms = %+ld)   Encoder2 count = %7ld   ",
                 (long)count, (long)delta, (long)Encoder2_Get());
        Serial_Print(g_tx_buf);

        HAL_Delay(50);
    }

    Motor1_Set(0);

    count = Encoder1_Get();
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "\r\n  Final count = %ld\r\n", (long)count);
    Serial_Print(g_tx_buf);
    Serial_Println("[TEST 5] Done. Check that count changed while motor was running.");
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
