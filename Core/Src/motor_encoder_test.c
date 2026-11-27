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
#include "emergency_stop.h"
#include "ultrasonic.h"
#include "position_controller.h"
#include "speed_controller.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

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
#define PWM_DUTY_MAX      100       /* Motor_Set が出せる duty の上限 [%]（ハード上の上限。各テストでさらに絞る） */
#define UART_TIMEOUT    100U        /* UART transmit timeout [ms] */
#define ESTOP_TEST_DUTY    20       /* Duty cycle for the emergency stop test [%] */
#define ENCODER_TEST_DUTY  50       /* Duty cycle for the 2-motor encoder test (menu 4) [%] */

/* 位置制御テスト（Test 5, Issue #31）のパラメータ */
#define MM_PER_COUNT_1     0.039f   /* TODO(#31): 仮の値（2048PPR x4逓倍・出力軸側の計算値）。実機で測った値に置き換える（Encoder1） */
#define MM_PER_COUNT_2     0.039f   /* TODO(#31): 同上（Encoder2）。左右別々に測る */
#define POS_MOTOR1_SIGN   -1        /* Motor1 の出力の向き。+1 の duty で逆に進んだため -1（正の duty で前進するなら +1） */
#define POS_MOTOR2_SIGN   -1        /* 同上（Motor2） */
#define POS_KP             0.20f    /* 比例ゲイン [duty % / mm]（仮。偏差 100mm で duty 20%、250mm 以上で上限 50%） */
#define POS_DEADBAND_MM    5.0f     /* 残り偏差がこれ以下で止める [mm] */
#define POS_DUTY_MIN       20.0f    /* 走行中に出す最小 duty [%]（仮）。これ未満だと動き続けられず、目標の手前で止まる */
#define POS_DUTY_MAX       50.0f    /* 位置制御テストで出す duty の上限 [%] */
#define POS_KICK_DUTY      30.0f    /* 始動時に出す強めの duty [%]（静止摩擦に勝つため。仮） */
#define POS_KICK_MS        150U     /* 始動の強めの duty を出す時間 [ms]（仮） */
#define POS_TARGET_MAX_MM  2000     /* 入力できる目標距離の上限 [mm] */
#define POS_LOOP_MS        50U      /* 制御周期 [ms] */

/* 速度制御テスト（Test 6, Issue #33）のパラメータ（MM_PER_COUNT_1/2 と POS_MOTOR1/2_SIGN は上と共通） */
#define SPD_TARGET_MAX_MMPS 500     /* 入力できる目標速度の上限 [mm/s]（最大 0.5 m/s） */
#define SPD_KP              0.05f   /* 比例ゲイン [duty % / (mm/s)]（仮） */
#define SPD_KI              0.30f   /* 積分ゲイン [duty % / (mm/s·s)]（仮） */
#define SPD_INTEG_MAX       90.0f   /* 積分項の上限 [duty %]（仮）。500 mm/s で走り続けるのに必要な duty（約 70〜80 %）を積分項が出せる大きさ */
#define SPD_DUTY_MAX        90.0f   /* 速度制御テストで出す duty の上限 [%]。500 mm/s は無負荷で約 70 %、荷重で約 80 % 必要（設計書）。100 %まで使い切らず余裕を残す */
#define SPD_ACCEL_MMPS2     300.0f  /* 目標速度の加速の制限 [mm/s^2]（0.3 m/s^2） */
#define SPD_LOOP_MS         10U     /* 制御周期 [ms]（設計書は 1 ms。テストでは 10 ms） */
#define SPD_HIST_NUM        6U      /* 速度を求めるために保存するカウントの件数（窓 = (件数-1) x 周期 = 50 ms） */
#define SPD_KICK_DUTY       30.0f   /* 始動時に出す強めの duty [%]（位置制御テストと同じ） */
#define SPD_KICK_MS         150U    /* 始動の強めの duty を出す時間 [ms] */
#define SPD_BAND_RATIO      0.10f   /* 目標速度に「達した」とみなす範囲（目標の ±10 %） */
#define SPD_HOLD_MS         4000U   /* 目標速度に達してから止めるまでの時間 [ms] */
#define SPD_PRINT_MS        100U    /* 表示の間隔 [ms] */

static char g_tx_buf[256];


/* ============================================================
   Function Prototypes
   ============================================================ */
static void MotorEncoderTest_Init(void);
static void Test_Motors_Encoders(void);
static void Test_PositionControl(void);
static void Test_SpeedControl(void);
static void Test_Encoder_Monitor(void);
static void Test_Buzzer(void);
static void Test_Lcd(void);
static void Test_Ultrasonic(void);
static void Test_SwitchDetect(void);
static void Test_EmergencyStop(void);
static void Print_Menu(void);
static void Serial_Print(const char *str);
static void Serial_Println(const char *str);
static uint8_t Serial_ReceiveChar(char *c, uint32_t timeout_ms);
static uint8_t Serial_WaitEnter(void);
static uint8_t Serial_IsEnter(char c);
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
                case '1': Test_SwitchDetect();      break;
                case '2': Test_EmergencyStop();     break;
                case '3': Test_Ultrasonic();        break;
                case '4': Test_Motors_Encoders();   break;
                case '5': Test_PositionControl();   break;
                case '6': Test_SpeedControl();      break;
                case '7': Test_Encoder_Monitor();   break;
                case '8': Test_Buzzer();            break;
                case '9': Test_Lcd();               break;
                default:
                    Serial_Println("[!] Invalid input. Enter 1, 2, 3, 4, 5, 6, 7, 8 or 9.");
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
    Serial_Println(" 1 : Switch ON/OFF detection (PB1, no motors)");
    Serial_Println(" 2 : Emergency stop (both motors run; press the stop button)");
    Serial_Println(" 3 : Ultrasonic (4 sensors, distance in mm)");
    Serial_Println(" 4 : Motor1 + Motor2 + Encoder1 + Encoder2 (all at once)");
    Serial_Println(" 5 : Position control (enter target distance, Motor1 + Motor2)");
    Serial_Println(" 6 : Speed control (enter target speed, Motor1 + Motor2)");
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
   Test 1: スイッチ（モータ電源の緊急停止ボタン）の ON/OFF の検出テスト（モータは動かさない）
   PB1（ESTOP_SENSE）がモータ 12V の分圧を読む（HIGH = 電源 ON、LOW = OFF）。ボタンを押している間だけ OFF になる。
   生のレベル（raw）と、100ms の間に HIGH だった割合、確定後の状態（EmergencyStop の判定）、ON/OFF の変化の回数を表示する。
   確定の条件: raw が 5 回連続で LOW なら OFF（約 50ms）、2 回連続で HIGH なら ON（約 20ms）。
   ============================================================ */
static void Test_SwitchDetect(void)
{
    Serial_Println("\r\n[TEST 1] Switch ON/OFF detection (PB1: HIGH = motor power ON, LOW = OFF)");
    Serial_Println("Press and release the emergency stop button. Motors stay OFF. 'q' to stop, 'r' to reset the counts.");
    Serial_Println("raw: PB1 level.  high: share of HIGH in the last 100 ms (1 ms sampling).  state: confirmed by EmergencyStop.");

    Motor1_Set(0);      /* 念のためモータは止めておく */
    Motor2_Set(0);

    uint8_t  prev     = EmergencyStop_PowerOn();
    uint32_t n_on     = 0U;                 /* OFF → ON と確定した回数 */
    uint32_t n_off    = 0U;                 /* ON → OFF と確定した回数 */
    uint32_t t_edge   = HAL_GetTick();      /* 状態が変わった時刻 */
    uint32_t t_print  = HAL_GetTick();
    uint32_t samples  = 0U;
    uint32_t highs    = 0U;

    snprintf(g_tx_buf, sizeof(g_tx_buf), "  initial state: %s\r\n", prev ? "ON" : "OFF");
    Serial_Print(g_tx_buf);

    while (1)
    {
        char c = 0;
        if (Serial_ReceiveChar(&c, 1))          /* 受信待ち 1ms を、読み取り間隔の待ちとして使う */
        {
            if (c == 'q' || c == 'Q') break;
            if (c == 'r' || c == 'R') { n_on = 0U; n_off = 0U; }
        }

        samples++;
        if (HAL_GPIO_ReadPin(ESTOP_SENSE_GPIO_Port, ESTOP_SENSE_Pin) == GPIO_PIN_SET) highs++;

        uint8_t now = EmergencyStop_PowerOn();
        if (now != prev)
        {
            uint32_t t = HAL_GetTick();
            if (now) n_on++; else n_off++;
            snprintf(g_tx_buf, sizeof(g_tx_buf), "\r  %s (after %lu ms in the previous state)                    \r\n",
                     now ? ">>> ON  (button released)" : "<<< OFF (button pressed)", (unsigned long)(t - t_edge));
            Serial_Print(g_tx_buf);
            t_edge = t;
            prev   = now;
        }

        if (HAL_GetTick() - t_print >= 100U)
        {
            t_print = HAL_GetTick();
            snprintf(g_tx_buf, sizeof(g_tx_buf), "\r  raw = %d   high = %3lu%%   state = %-3s   ON count = %lu   OFF count = %lu   ",
                     (int)HAL_GPIO_ReadPin(ESTOP_SENSE_GPIO_Port, ESTOP_SENSE_Pin),
                     (unsigned long)(samples ? (highs * 100U / samples) : 0U),
                     now ? "ON" : "OFF", (unsigned long)n_on, (unsigned long)n_off);
            Serial_Print(g_tx_buf);
            samples = 0U;
            highs   = 0U;
        }
    }

    snprintf(g_tx_buf, sizeof(g_tx_buf), "\r\n  ON count = %lu, OFF count = %lu\r\n", (unsigned long)n_on, (unsigned long)n_off);
    Serial_Print(g_tx_buf);
    Serial_Println("[TEST 1] Done. Check: button not pressed -> ON (high near 100%); pressed -> OFF (high near 0%).");
}

/* ============================================================
   Test 2: 緊急停止のテスト（Motor1 と Motor2 を同時に回して、緊急停止ボタンで止める）
   緊急停止ボタン = モータ電源の押しボタン（ノーマルクローズ形・自己保持しない）。押している間だけ 12V が切れ、離すと戻る。
   PB1（ESTOP_SENSE）がモータ 12V の分圧を読む（HIGH = 電源 ON、LOW = OFF）。
   ボタンを押すと 12V が切れてモータが止まる。同時に EmergencyStop（SysTick の 10ms 周期の読み取り）が
   PWM の出力を 0 にしてラッチするので、ボタンを離して 12V が戻ってもモータは勝手に回らない。'r' で解除して再開する。
   このテストの間だけ緊急停止を有効にする（テスト 4/5/6 には組み込まない）。
   ============================================================ */
static void Test_EmergencyStop(void)
{
    Serial_Println("\r\n[TEST 2] Emergency stop (push button cuts the motor 12V; sensed on PB1: HIGH = power ON, LOW = OFF)");
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "Motor1 and Motor2 ramp up to %d%% duty and keep running. Press the emergency stop button to stop.\r\n",
             ESTOP_TEST_DUTY);
    Serial_Print(g_tx_buf);
    Serial_Println("After the stop, releasing the button (12V returns) must NOT restart the motors. 'r' = clear and restart, 'q' = quit.");
    Serial_Println("Do not press the button (motor power ON). Press Enter to start, 'q' to cancel...");

    if (!Serial_WaitEnter()) { Serial_Println("Cancelled."); return; }

    Motor1_Set(0);
    Motor2_Set(0);
    EmergencyStop_Clear();
    EmergencyStop_Enable(1U);       /* このテストの間だけ、緊急停止（モータ出力の 0 化とラッチ）を有効にする */

    uint8_t  prev    = EmergencyStop_PowerOn();
    uint32_t t_print = HAL_GetTick();
    uint32_t t_step  = HAL_GetTick();
    int      d       = 0;               /* 今のデューティ[%] */

    while (1)
    {
        char c = 0;
        if (Serial_ReceiveChar(&c, 5))
        {
            if (c == 'q' || c == 'Q') break;
            if (c == 'r' || c == 'R')
            {
                if (EmergencyStop_PowerOn())
                {
                    EmergencyStop_Clear();
                    d = 0;
                    t_step = HAL_GetTick();
                    Serial_Print("\r  latch cleared. restarting\r\n");
                }
                else
                {
                    Serial_Print("\r  release the emergency stop button first\r\n");
                }
            }
        }

        uint8_t now = EmergencyStop_PowerOn();
        if (now != prev)
        {
            prev = now;
            snprintf(g_tx_buf, sizeof(g_tx_buf), "\r  %s                                              \r\n",
                     now ? ">>> button released (motor power ON)" : "<<< button pressed (motor power OFF = emergency stop)");
            Serial_Print(g_tx_buf);
        }

        /* モータの駆動: 電源 ON で、ラッチされていないときだけ、300ms ごとに 5% ずつ ESTOP_TEST_DUTY まで上げる */
        if (now && !EmergencyStop_IsActive())
        {
            if (d < ESTOP_TEST_DUTY && HAL_GetTick() - t_step >= 300U)
            {
                t_step = HAL_GetTick();
                d += 5;
            }
            Motor1_Set((int16_t)d);
            Motor2_Set((int16_t)d);
        }

        if (HAL_GetTick() - t_print >= 100U)
        {
            t_print = HAL_GetTick();
            snprintf(g_tx_buf, sizeof(g_tx_buf),
                     "\r  raw = %d  power = %-3s  motors = %-8s  duty = %2d%%  OFF count = %lu  latched = %d   ",
                     (int)HAL_GPIO_ReadPin(ESTOP_SENSE_GPIO_Port, ESTOP_SENSE_Pin), now ? "ON" : "OFF",
                     EmergencyStop_IsActive() ? "STOPPED" : (now && d > 0 ? "RUNNING" : "IDLE"),
                     EmergencyStop_IsActive() ? 0 : d,
                     (unsigned long)EmergencyStop_OffCount(), (int)EmergencyStop_IsActive());
            Serial_Print(g_tx_buf);
        }
    }

    Motor1_Set(0);
    Motor2_Set(0);
    EmergencyStop_Enable(0U);       /* 無効に戻し、ラッチも解除する（4/5/6 のテストには影響させない） */
    EmergencyStop_Clear();

    snprintf(g_tx_buf, sizeof(g_tx_buf), "\r\n  OFF count = %lu\r\n", (unsigned long)EmergencyStop_OffCount());
    Serial_Print(g_tx_buf);
    Serial_Println("[TEST 2] Done. Check: button pressed -> motors stop, STOPPED and latched = 1; released -> motors stay stopped until 'r'.");
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
        if (Serial_IsEnter(c) && digits > 0) { *value = v; return 1U; }
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
    uint32_t t_start = HAL_GetTick();

    while (!(done[0] && done[1]))
    {
        dist[0] = fabsf((float)Encoder1_Get() * MM_PER_COUNT_1);      /* 進んだ距離は常にプラス（マイナスならプラスに補正） */
        dist[1] = fabsf((float)Encoder2_Get() * MM_PER_COUNT_2);

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
            if (HAL_GetTick() - t_start < POS_KICK_MS && out < POS_KICK_DUTY)
                out = POS_KICK_DUTY;                                /* 始動時だけ強めに出して、静止摩擦に勝つ */
            if (out > POS_DUTY_MAX)               out = POS_DUTY_MAX;
            duty[i] = out;
        }

        Motor1_Set((int16_t)(POS_MOTOR1_SIGN * (int)(duty[0] + 0.5f)));
        Motor2_Set((int16_t)(POS_MOTOR2_SIGN * (int)(duty[1] + 0.5f)));

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
    dist[0] = fabsf((float)Encoder1_Get() * MM_PER_COUNT_1);
    dist[1] = fabsf((float)Encoder2_Get() * MM_PER_COUNT_2);
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "\r\n  Final  L = %ld mm (error %+ld)   R = %ld mm (error %+ld)\r\n",
             (long)dist[0], (long)dist[0] - (long)target,
             (long)dist[1], (long)dist[1] - (long)target);
    Serial_Print(g_tx_buf);
    Serial_Println(stopped ? "[TEST 5] Stopped before reaching the target."
                           : "[TEST 5] Done. Check that the error is within +/-50 mm.");
}

/* ============================================================
   Test 6: 速度制御テスト（Issue #33 / 親 #5）
   目標速度 [mm/s] を入力し、Motor1 + Motor2 をエンコーダで測った速度で PI 制御して、その速度で走らせる。
   目標速度に達して SPD_HOLD_MS（4 秒）たったら止める。途中で 'q' を押しても止める。
   制御器（PI）は speed_controller.c にあり、ここでは偏差を渡して出力を受け取るだけ。
   ============================================================ */
static void Test_SpeedControl(void)
{
    Serial_Println("\r\n[TEST 6] Speed control (Motor1 + Motor2, Encoder1 + Encoder2)");
    snprintf(g_tx_buf, sizeof(g_tx_buf),
             "Enter target speed [mm/s] (1-%d) then Enter, 'q' to cancel.\r\n"
             "Stops %d s after reaching the target speed (within +/-%d%%), or when 'q' is pressed.\r\n",
             SPD_TARGET_MAX_MMPS, (int)(SPD_HOLD_MS / 1000U), (int)(SPD_BAND_RATIO * 100.0f));
    Serial_Print(g_tx_buf);
    Serial_Print("Target [mm/s] > ");

    uint32_t target = 0U;
    while (1)
    {
        if (!Serial_ReadNumber(&target)) { Serial_Println("\r\nCancelled."); return; }
        if (target >= 1U && target <= (uint32_t)SPD_TARGET_MAX_MMPS) break;
        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "\r\n[!] Out of range. Enter 1-%d.\r\nTarget [mm/s] > ", SPD_TARGET_MAX_MMPS);
        Serial_Print(g_tx_buf);
    }
    snprintf(g_tx_buf, sizeof(g_tx_buf), "\r\nTarget = %lu mm/s. Press Enter to start, 'q' to cancel...\r\n",
             (unsigned long)target);
    Serial_Print(g_tx_buf);
    if (!Serial_WaitEnter()) { Serial_Println("Cancelled."); return; }

    /* 左右それぞれに制御器を持つ（今は同じゲイン） */
    SpeedCtrl ctrl[2];
    SpeedCtrl_Init(&ctrl[0], SPD_KP, SPD_KI, SPD_INTEG_MAX);
    SpeedCtrl_Init(&ctrl[1], SPD_KP, SPD_KI, SPD_INTEG_MAX);

    Encoder1_Reset();
    Encoder2_Reset();

    const float mm_per_count[2] = { MM_PER_COUNT_1, MM_PER_COUNT_2 };
    const int   motor_sign[2]   = { POS_MOTOR1_SIGN, POS_MOTOR2_SIGN };

    /* 速度は、SPD_WINDOW_MS 前のカウントとの差から求める（1 カウントの数え誤差を小さくするため、窓を長めにとる） */
    int32_t  cnt_hist[SPD_HIST_NUM][2];
    uint32_t tick_hist[SPD_HIST_NUM];
    uint32_t hist_n = 0U;                        /* 保存した件数（SPD_HIST_NUM で頭打ち） */

    float    speed[2]   = { 0.0f, 0.0f };        /* 現在速度 [mm/s] */
    float    duty[2]    = { 0.0f, 0.0f };
    float    ref        = 0.0f;                  /* 加速を制限した目標速度 [mm/s] */
    uint8_t  reached    = 0U;
    uint32_t t_reached  = 0U;
    float    sum_speed[2] = { 0.0f, 0.0f };      /* 到達後の速度の合計（平均を出すため） */
    uint32_t sum_n      = 0U;
    uint8_t  stopped    = 0U;                    /* 1: 'q' で止めた */

    uint32_t t_start    = HAL_GetTick();
    uint32_t t_prev     = t_start;
    uint32_t t_next     = t_start;
    uint32_t t_print    = t_start;

    while (1)
    {
        /* 周期を待つ。待っている間に 'q' を確認する */
        t_next += SPD_LOOP_MS;
        while ((int32_t)(HAL_GetTick() - t_next) < 0)
        {
            if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE))
            {
                char key = (char)(huart2.Instance->DR & 0xFFU);
                if (key == 'q' || key == 'Q') { stopped = 1U; }
            }
        }
        if (stopped) break;

        uint32_t now = HAL_GetTick();
        float    dt  = (float)(now - t_prev) / 1000.0f;                 /* 実際の経過時間 [s] */
        t_prev = now;

        /* 現在速度を求める */
        int32_t cnt[2] = { Encoder1_Get(), Encoder2_Get() };
        for (uint32_t i = SPD_HIST_NUM - 1U; i > 0U; i--)
        {
            cnt_hist[i][0] = cnt_hist[i - 1U][0];
            cnt_hist[i][1] = cnt_hist[i - 1U][1];
            tick_hist[i]   = tick_hist[i - 1U];
        }
        cnt_hist[0][0] = cnt[0];
        cnt_hist[0][1] = cnt[1];
        tick_hist[0]   = now;
        if (hist_n < SPD_HIST_NUM) hist_n++;

        if (hist_n >= SPD_HIST_NUM)
        {
            float window_s = (float)(tick_hist[0] - tick_hist[SPD_HIST_NUM - 1U]) / 1000.0f;
            for (int i = 0; i < 2; i++)
            {
                float d = (float)(cnt_hist[0][i] - cnt_hist[SPD_HIST_NUM - 1U][i]) * mm_per_count[i];
                speed[i] = fabsf(d) / window_s;                         /* 速度は常にプラス（マイナスならプラスに補正） */
            }
        }

        /* 目標速度は、加速のみ SPD_ACCEL_MMPS2 に制限する */
        if (ref < (float)target)
        {
            ref += SPD_ACCEL_MMPS2 * dt;
            if (ref > (float)target) ref = (float)target;
        }

        /* 到達の判定：左右ともに目標速度の ±SPD_BAND_RATIO 以内になったら到達とし、そこから SPD_HOLD_MS 数える */
        float band = (float)target * SPD_BAND_RATIO;
        if (!reached && hist_n >= SPD_HIST_NUM
            && fabsf(speed[0] - (float)target) <= band && fabsf(speed[1] - (float)target) <= band)
        {
            reached   = 1U;
            t_reached = now;
            Serial_Println("\r\n  -- Reached the target speed. Holding... --");
        }
        if (reached)
        {
            sum_speed[0] += speed[0];
            sum_speed[1] += speed[1];
            sum_n++;
            if (now - t_reached >= SPD_HOLD_MS) break;
        }

        /* 速度 PI 制御：偏差 → 操作量 [%] */
        for (int i = 0; i < 2; i++)
        {
            float error;
            float out;

            if (now - t_start < SPD_KICK_MS)
            {
                out = SPD_KICK_DUTY;                                    /* 始動時だけ強めに出して、静止摩擦に勝つ */
                SpeedCtrl_Reset(&ctrl[i]);
            }
            else
            {
                error = ref - speed[i];
                out   = SpeedCtrl_Update(&ctrl[i], error, dt);
            }
            if (out < 0.0f)                out = 0.0f;                  /* 後退はしない */
            if (out > SPD_DUTY_MAX)        out = SPD_DUTY_MAX;
            duty[i] = out;
        }
        Motor1_Set((int16_t)(motor_sign[0] * (int)(duty[0] + 0.5f)));
        Motor2_Set((int16_t)(motor_sign[1] * (int)(duty[1] + 0.5f)));

        if (now - t_print >= SPD_PRINT_MS)
        {
            t_print = now;
            snprintf(g_tx_buf, sizeof(g_tx_buf),
                     "\r  ref %3ld  L %4ld (%2d%%)  R %4ld (%2d%%) mm/s   ",
                     (long)ref, (long)speed[0], (int)(duty[0] + 0.5f), (long)speed[1], (int)(duty[1] + 0.5f));
            Serial_Print(g_tx_buf);
        }
    }

    Motor1_Set(0);
    Motor2_Set(0);

    if (stopped) Serial_Println("\r\n  Stopped by user ('q').");
    else         Serial_Println("\r\n  Stopped (hold time elapsed).");

    if (sum_n > 0U)
    {
        float avg[2] = { sum_speed[0] / (float)sum_n, sum_speed[1] / (float)sum_n };
        snprintf(g_tx_buf, sizeof(g_tx_buf),
                 "  Average after reaching:  L = %ld mm/s (%+ld%%)   R = %ld mm/s (%+ld%%)   target %lu mm/s\r\n",
                 (long)avg[0], (long)((avg[0] - (float)target) * 100.0f / (float)target),
                 (long)avg[1], (long)((avg[1] - (float)target) * 100.0f / (float)target),
                 (unsigned long)target);
        Serial_Print(g_tx_buf);
    }
    else
    {
        Serial_Println("  The target speed was not reached.");
    }
    Serial_Println("[TEST 6] Done. Check that the speed error is within +/-10%.");
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

/* 直前に受け取った改行文字（13 or 10）。0 = 直前は改行ではない。CRLF / LFCR を 1 回の Enter にするために使う */
static char s_last_eol = 0;

/* c が Enter なら 1。CR+LF（または LF+CR）の 2 文字目は対の文字なので 0（読み捨て）。
   CR だけ・LF だけの端末は、1 文字ごとに Enter として扱う */
static uint8_t Serial_IsEnter(char c)
{
    if (c == 13 || c == 10)
    {
        if (s_last_eol != 0 && s_last_eol != c)
        {
            s_last_eol = 0;     /* 対の文字（CRLF の LF など）。Enter としては数えない */
            return 0U;
        }
        s_last_eol = c;
        return 1U;
    }
    s_last_eol = 0;
    return 0U;
}

/* 開始待ち: Enter（CR または LF）で 1、'q' / 'Q' で 0 を返す。それ以外の入力は無視して待ち続ける */
static uint8_t Serial_WaitEnter(void)
{
    char c = 0;
    while (1)
    {
        if (!Serial_ReceiveChar(&c, HAL_MAX_DELAY)) continue;
        if (Serial_IsEnter(c)) return 1U;
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
