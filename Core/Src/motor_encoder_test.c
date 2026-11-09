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
  *  Motor2 IN1/IN2 : PA0/PA1 (TIM2)          -> PB4/PB5 (TIM3 CH1/CH2, CubeMX 生成の htim3)
  *  Encoder1 A/B   : PA8/PA9 (TIM1)          -> PC9/PC8 (EXTI9/EXTI8 のソフト4逓倍)
  *  Encoder2 A/B   : PB6/PB7 (TIM4 CH1/CH2)  -> PB7/PB6 (TIM4 CH2=A相, CH1=B相, htim4)
  *  USART2         : PA2/PA3 (変更なし)
  *
  * 原本の LED(PB12-15)・スイッチ(PC10-13)・ADC(PC0-3) は液晶・超音波と競合するため移植していない。
  ******************************************************************************
  */
#include "motor_encoder_test.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim3;     /* Motor2 PWM（CubeMX 生成） */
extern TIM_HandleTypeDef htim4;     /* Encoder2（CubeMX 生成） */
extern UART_HandleTypeDef huart2;   /* Serial（CubeMX 生成） */

static TIM_HandleTypeDef htim13;    /* Motor1 IN1 (PA6)。ここで初期化する */
static TIM_HandleTypeDef htim14;    /* Motor1 IN2 (PA7)。ここで初期化する */

/* ============================================================
   Constants（原本のまま）
   ============================================================ */
#define PWM_PRESCALER   4U          /* CubeMX の TIM3 と同値 */
#define PWM_PERIOD      55999UL     /* 84MHz / 300Hz / 5 - 1 */
#define PWM_DUTY_MAX       50       /* Maximum duty cycle [%] */
#define ENCODER_TEST_DUTY  50       /* Duty cycle for encoder test [%] */
#define UART_TIMEOUT    100U        /* UART transmit timeout [ms] */

static char g_tx_buf[256];

/* Encoder1 のソフトカウント（EXTI 割り込みで更新） */
static volatile int32_t s_enc1_count = 0;
static volatile uint8_t s_enc1_prev  = 0;

/* ============================================================
   Function Prototypes
   ============================================================ */
static void MotorEncoderTest_Init(void);
static void MX_TIM13_14_PWM_Init(void);
static void Test_Motor1_Encoder1(void);
static void Test_Motor2_Encoder2(void);
static void Print_Menu(void);
static void Serial_Print(const char *str);
static void Serial_Println(const char *str);
static uint8_t Serial_ReceiveChar(char *c, uint32_t timeout_ms);
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
                case '5': Test_Motor1_Encoder1();   break;
                case '6': Test_Motor2_Encoder2();   break;
                default:
                    Serial_Println("[!] Invalid input. Enter 5 or 6.");
                    break;
            }
            Print_Menu();
        }
    }
}

static void MotorEncoderTest_Init(void)
{
    MX_TIM13_14_PWM_Init();

    /* Encoder2: TIM4（CubeMX 生成）, Motor2: TIM3（CubeMX 生成） */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);          /* Motor2 IN1 (PB4) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);          /* Motor2 IN2 (PB5) */
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);         /* Motor1 IN1 (PA6) */
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);         /* Motor1 IN2 (PA7) */

    /* Encoder1 の現在の A/B 状態を初期値にする（A=PC9 を bit1, B=PC8 を bit0） */
    s_enc1_prev = (uint8_t)(((HAL_GPIO_ReadPin(ENC1_A_GPIO_Port, ENC1_A_Pin) == GPIO_PIN_SET) << 1)
                          |  (HAL_GPIO_ReadPin(ENC1_B_GPIO_Port, ENC1_B_Pin) == GPIO_PIN_SET));
    s_enc1_count = 0;

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
    Serial_Println(" 5 : Motor1 + Encoder1 test");
    Serial_Println(" 6 : Motor2 + Encoder2 test");
    Serial_Println("------------------------------------------------");
    Serial_Print  (" Enter number > ");
}

/* ============================================================
   Test 5: Motor1 + Encoder1 Test（LED 表示のみ除いて原本のまま）
   ============================================================ */
static void Test_Motor1_Encoder1(void)
{
    Serial_Println("\r\n[TEST 5] Motor1 + Encoder1 Test");
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "Motor1 ramps 0->%d%%%% duty then holds. Encoder1 count is displayed.\r\n",
             ENCODER_TEST_DUTY);
    Serial_Print(g_tx_buf);
    Serial_Println("Press Enter to start, 'q' to stop...");

    char c = 0;
    Serial_ReceiveChar(&c, HAL_MAX_DELAY);

    Encoder1_Reset();

    /* Ramp up: 0% -> ENCODER_TEST_DUTY% in steps of 5% */
    snprintf(g_tx_buf, sizeof(g_tx_buf), "  Ramp up: 0 -> %d%%\r\n", ENCODER_TEST_DUTY);
    Serial_Print(g_tx_buf);
    for (int d = 0; d <= ENCODER_TEST_DUTY; d += 5)
    {
        Motor1_Set((int16_t)d);
        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "  Motor1 duty = %2d%%\r\n", d);
        Serial_Print(g_tx_buf);
        HAL_Delay(300);
    }
    snprintf(g_tx_buf, sizeof(g_tx_buf), "  Hold at %d%%%% -- press 'q' to stop\r\n", ENCODER_TEST_DUTY);
    Serial_Print(g_tx_buf);

    int32_t prev_count = 0;
    int32_t count = 0;

    while (1)
    {
        if (Serial_ReceiveChar(&c, 10))
        {
            if (c == 'q' || c == 'Q') break;
        }

        count = Encoder1_Get();
        int32_t delta = count - prev_count;
        prev_count = count;

        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "\r  Encoder1 count = %7ld  (delta/50ms = %+ld)   ",
                 (long)count, (long)delta);
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
             "Motor2 ramps 0->%d%%%% duty then holds. Encoder2 count is displayed.\r\n",
             ENCODER_TEST_DUTY);
    Serial_Print(g_tx_buf);
    Serial_Println("Press Enter to start, 'q' to stop...");

    char c = 0;
    Serial_ReceiveChar(&c, HAL_MAX_DELAY);

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
    snprintf(g_tx_buf, sizeof(g_tx_buf), "  Hold at %d%%%% -- press 'q' to stop\r\n", ENCODER_TEST_DUTY);
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

/* ============================================================
   Motor Control（TB67H450）
   positive -> forward (IN1=PWM, IN2=0) / negative -> reverse (IN1=0, IN2=PWM) / zero -> coast
   ============================================================ */

/* Motor1: IN1=PA6 (TIM13_CH1), IN2=PA7 (TIM14_CH1) */
static void Motor1_Set(int16_t duty_percent)
{
    if (duty_percent >  PWM_DUTY_MAX) duty_percent =  PWM_DUTY_MAX;
    if (duty_percent < -PWM_DUTY_MAX) duty_percent = -PWM_DUTY_MAX;

    uint32_t cmp = (uint32_t)((PWM_PERIOD + 1U) * (uint32_t)abs(duty_percent) / 100U);

    if (duty_percent > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, cmp); /* IN1 = PWM */
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U);  /* IN2 = 0   */
    }
    else if (duty_percent < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0U);  /* IN1 = 0   */
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, cmp); /* IN2 = PWM */
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0U);  /* coast */
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U);
    }
}

/* Motor2: IN1=PB4 (TIM3_CH1), IN2=PB5 (TIM3_CH2) */
static void Motor2_Set(int16_t duty_percent)
{
    if (duty_percent >  PWM_DUTY_MAX) duty_percent =  PWM_DUTY_MAX;
    if (duty_percent < -PWM_DUTY_MAX) duty_percent = -PWM_DUTY_MAX;

    uint32_t cmp = (uint32_t)((PWM_PERIOD + 1U) * (uint32_t)abs(duty_percent) / 100U);

    if (duty_percent > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, cmp);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    }
    else if (duty_percent < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, cmp);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0U);
    }
}

/* ============================================================
   Encoder Functions（x4 resolution）
   ============================================================ */

/* Encoder1: PC9=A, PC8=B のソフト4逓倍カウント */
static int32_t Encoder1_Get(void)
{
    return s_enc1_count;
}

/* Encoder2: TIM4 エンコーダモード（16bit を符号拡張） */
static int32_t Encoder2_Get(void)
{
    return (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

static void Encoder1_Reset(void)
{
    s_enc1_count = 0;
}

static void Encoder2_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

/**
 * @brief Encoder1 の A/B いずれかのエッジで呼ばれ、グレイコードの遷移から +1/-1 を加える。
 *        状態 = (A<<1)|B。00->10->11->01->00 を正転(+1)とする。
 *        不正な遷移（取りこぼし）は 0 として無視する。
 */
void MotorEncoderTest_EXTI_Callback(uint16_t gpio_pin)
{
    static const int8_t k_table[16] = {
         0, -1, +1,  0,
        +1,  0,  0, -1,
        -1,  0,  0, +1,
         0, +1, -1,  0
    };

    if (gpio_pin != ENC1_A_Pin && gpio_pin != ENC1_B_Pin) {
        return;
    }

    uint8_t now = (uint8_t)(((HAL_GPIO_ReadPin(ENC1_A_GPIO_Port, ENC1_A_Pin) == GPIO_PIN_SET) << 1)
                          |  (HAL_GPIO_ReadPin(ENC1_B_GPIO_Port, ENC1_B_Pin) == GPIO_PIN_SET));
    s_enc1_count += k_table[(s_enc1_prev << 2) | now];
    s_enc1_prev = now;
}

/* ============================================================
   TIM13 / TIM14 PWM 初期化（Motor1）
   PA6 = TIM13_CH1 (AF9) -> IN1, PA7 = TIM14_CH1 (AF9) -> IN2
   Prescaler/Period は Motor2 (TIM3) と同じ 300Hz
   ============================================================ */
static void MX_TIM13_14_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM13_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin       = MOTOR1_IN1_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
    HAL_GPIO_Init(MOTOR1_IN1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = MOTOR1_IN2_Pin;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;
    HAL_GPIO_Init(MOTOR1_IN2_GPIO_Port, &GPIO_InitStruct);

    htim13.Instance               = TIM13;
    htim13.Init.Prescaler         = PWM_PRESCALER;
    htim13.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim13.Init.Period            = PWM_PERIOD;
    htim13.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim13) != HAL_OK) Error_Handler();

    htim14.Instance               = TIM14;
    htim14.Init                   = htim13.Init;
    if (HAL_TIM_PWM_Init(&htim14) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}
