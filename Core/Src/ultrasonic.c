/**
  ******************************************************************************
  * @file    ultrasonic.c
  * @brief   超音波計測ドライバ（HC-SR04 x4）
  *
  * ■ 超音波センサ(HC-SR04)が距離を測るしくみ
  *   1. マイコンが Trig ピンを 10us だけ HIGH にする（「測って」の合図）
  *   2. センサが超音波を出す
  *   3. 反射して戻ってくるまでの間、Echo ピンが HIGH になる
  *   4. Echo が HIGH だった時間（パルス幅）がわかれば距離が出る
  *        距離[mm] = パルス幅[us] x 0.343 / 2
  *      0.343 は音速（1us で 0.343mm 進む）。往復の時間なので 2 で割る。
  *      例: パルス幅 1000us -> 1000 x 0.343 / 2 = 171.5mm
  *
  * ■ このコードの設計方針
  *   - Trig を出したら待たずに戻る（非同期）
  *       待っている間にモータ制御など他の処理が止まってしまうため。
  *   - Echo は割り込み(EXTI)で測る
  *       立ち上がり・立ち下がりの瞬間を逃さないため。
  *   - 4個を 2個ずつの組で順番に鳴らす
  *       全部同時に鳴らすと音が混ざるため。隣り合うセンサは同じ組にしない。
  *         組0 = センサ0・2 / 組1 = センサ1・3
  *       60ms ごとに交互に鳴らすので、各センサは 120ms ごとに更新される。
  *   - 1us で数えるタイマ TIM5 を「ストップウォッチ」として使う
  *       Echo が立ち上がった時刻と立ち下がった時刻の差がパルス幅になる。
  *
  * ■ 使い方（流れ）
  *   起動時 : Ultrasonic_Init() -> Ultrasonic_CheckConnection() で接続確認
  *   周期処理: 60ms ごとに Ultrasonic_Trigger(0), Trigger(1), Trigger(0)...
  *   割り込み: Echo ピンが変化するたびに Ultrasonic_EXTI_Callback() が呼ばれ、
  *             距離が自動で保存される
  *   読み出し: Ultrasonic_GetResult() で最新の距離を受け取る
  *
  * ピンは docs/pin_assignment.md のとおり（センサ i = 回路図 U(i+1) と仮定）。
  * GPIO・EXTI・NVIC は CubeMX 生成コード（.ioc）で初期化されている前提。
  ******************************************************************************
  */
#include "ultrasonic.h"
#include "main.h"

extern TIM_HandleTypeDef htim5;   /* main.c で CubeMX が生成 */

#define US_SOUND_MM_PER_US   0.343f   /* 音速 [mm/us] */
#define US_TRIG_PULSE_US     10U      /* Trig パルスの幅 [us]（HC-SR04 の仕様） */
#define US_CHECK_WAIT_MS     60U      /* 接続確認で Echo を待つ時間 [ms] */

/* センサ1個分のピン情報 */
typedef struct {
  GPIO_TypeDef *trig_port;   /* Trig のポート */
  uint16_t      trig_pin;    /* Trig のピン（出力） */
  uint16_t      echo_pin;    /* Echo のピン（EXTI 割り込み入力。ポートは全部 GPIOC） */
} UsPin;

/* センサ番号 -> ピンの表。添字がそのままセンサ番号。
 * 基板上の U1〜U4 の並びがセンサ0〜3（左から）と違う場合はここを並べ替える。 */
static const UsPin s_pin[US_SENSOR_NUM] = {
  { GPIOC, GPIO_PIN_10, GPIO_PIN_0 },  /* センサ0 = U1 */
  { GPIOC, GPIO_PIN_11, GPIO_PIN_1 },  /* センサ1 = U2 */
  { GPIOC, GPIO_PIN_12, GPIO_PIN_2 },  /* センサ2 = U3 */
  { GPIOC, GPIO_PIN_4,  GPIO_PIN_3 },  /* センサ3 = U4 */
};

/* 組ごとに「どのセンサを鳴らすか」を、1ビット=1センサで表したもの。
 *   bit0=センサ0, bit1=センサ1, bit2=センサ2, bit3=センサ3
 *   組0: 0x05 = 0b0101 -> センサ0・2
 *   組1: 0x0A = 0b1010 -> センサ1・3 */
static const uint8_t s_pair_mask[US_PAIR_NUM] = { 0x05U, 0x0AU };

/* 以下の変数は割り込みの中でも書き換わるので volatile を付ける。
 * （付けないとコンパイラが値を使い回し、割り込みによる更新に気づけないことがある） */
static volatile float    s_dist_mm[US_SENSOR_NUM];                 /* 最新の距離 [mm] */
static volatile int      s_no_resp[US_SENSOR_NUM] = { 1, 1, 1, 1 };/* 1=応答なし（最初は全部 応答なし） */
static volatile uint8_t  s_waiting;                 /* Trig を出して Echo 待ちのセンサ（ビット） */
static volatile uint8_t  s_rise_valid;              /* Echo の立ち上がりを捕まえ済みのセンサ（ビット） */
static volatile uint32_t s_rise_us[US_SENSOR_NUM];  /* Echo が立ち上がった時刻（TIM5 の値） */

/**
  * @brief  TIM5（1us ごとに +1 するストップウォッチ）のカウントを開始する。
  *         main() の最初に1回だけ呼ぶ。
  *         分周（84MHz -> 1MHz）と上限（32bit 最大）は CubeMX が MX_TIM5_Init で
  *         設定済み（Prescaler=83, Period=4294967295）なので、ここでは開始するだけ。
  *         クロック設定を変えたときは、Prescaler も合わせて変えること。
  */
void Ultrasonic_Init(void)
{
  HAL_TIM_Base_Start(&htim5);
}

/**
  * @brief  mask で指定したセンサの Trig を、まとめて HIGH または LOW にする。
  * @param  mask   bit i = 1 のセンサ i を操作する
  * @param  state  GPIO_PIN_SET (HIGH) / GPIO_PIN_RESET (LOW)
  */
static void set_trig(uint8_t mask, GPIO_PinState state)
{
  for (int i = 0; i < US_SENSOR_NUM; i++) {
    if (mask & (1U << i)) {
      HAL_GPIO_WritePin(s_pin[i].trig_port, s_pin[i].trig_pin, state);
    }
  }
}

/**
  * @brief  mask で指定したセンサの Trig に、同時に 10us のパルスを出す。
  *         Echo は待たずに戻る（Echo は割り込みが測る）。
  * @param  mask  bit i = 1 のセンサ i を鳴らす
  */
static void fire(uint8_t mask)
{
  s_waiting    |= mask;                  /* 「Echo 待ち」に登録 */
  s_rise_valid &= (uint8_t)~mask;        /* 前回の立ち上がり記録を捨てる */

  set_trig(mask, GPIO_PIN_SET);          /* Trig を HIGH に */
  uint32_t t0 = TIM5->CNT;               /* 10us 待つ（TIM5 は 1us ごとに +1） */
  while ((TIM5->CNT - t0) < US_TRIG_PULSE_US) { }
  set_trig(mask, GPIO_PIN_RESET);        /* Trig を LOW に */
}

/**
  * @brief  指定した組のセンサを鳴らす。60ms ごとに呼ぶ。
  * @param  pair_id  0: センサ0・2 / 1: センサ1・3
  * @retval 0: 開始成功 / -1: 組番号が範囲外
  */
int Ultrasonic_Trigger(int pair_id)
{
  int i;

  if (pair_id < 0 || pair_id >= US_PAIR_NUM) {
    return -1;
  }

  /* 前の組を鳴らしたのに Echo が来なかったセンサは「応答なし」にする。
   * 距離に 0 を入れるのは、0 が有効範囲(20〜4000mm)の外なので
   * 後の距離データ処理で自動的に無効になるため。
   * 割り込みと同時に書き換えないよう、ここだけ割り込みを止める。 */
  __disable_irq();
  for (i = 0; i < US_SENSOR_NUM; i++) {
    if (s_waiting & (1U << i)) {
      s_dist_mm[i] = 0.0f;
      s_no_resp[i] = 1;
    }
  }
  s_waiting = 0U;
  __enable_irq();

  fire(s_pair_mask[pair_id]);   /* 今回の組を鳴らす */
  return 0;
}

/**
  * @brief  Echo のパルス幅から距離を計算して保存する。
  *         通常は Ultrasonic_EXTI_Callback の中から呼ばれる。
  * @param  sensor_id  センサ番号 0〜3
  * @param  pulse_us   Echo のパルス幅 [us]
  * @retval 0: 保存成功 / -1: センサ番号が範囲外
  */
int Ultrasonic_OnEcho(int sensor_id, float pulse_us)
{
  if (sensor_id < 0 || sensor_id >= US_SENSOR_NUM) {
    return -1;
  }
  /* 距離 = パルス幅 x 0.343 / 2。範囲の判定はここではせず、測った値をそのまま保存 */
  s_dist_mm[sensor_id] = pulse_us * US_SOUND_MM_PER_US / 2.0f;
  s_no_resp[sensor_id] = 0;     /* 応答があったので フラグを下ろす */
  return 0;
}

/**
  * @brief  保存してある最新の距離と応答なしフラグを、引数の配列にコピーして返す。
  * @param  dist_mm  距離 [mm] の格納先（添字 = センサ番号）
  * @param  no_resp  応答なしフラグの格納先（1 = 応答なし）
  * @retval 0 (常に成功)
  */
int Ultrasonic_GetResult(float dist_mm[US_SENSOR_NUM], int no_resp[US_SENSOR_NUM])
{
  int i;
  /* コピーの途中で割り込みが値を書き換えないよう、一瞬だけ割り込みを止める */
  __disable_irq();
  for (i = 0; i < US_SENSOR_NUM; i++) {
    dist_mm[i] = s_dist_mm[i];
    no_resp[i] = s_no_resp[i];
  }
  __enable_irq();
  return 0;
}

/**
  * @brief  Echo ピンの電圧が変わるたびに呼ばれる（EXTI 割り込み）。
  *         main.c の HAL_GPIO_EXTI_Callback から呼ぶ。
  *         HIGH になった瞬間の時刻を覚え、LOW になった瞬間に差を取ってパルス幅を求める。
  * @param  gpio_pin  変化があったピン（GPIO_PIN_0 など）
  */
void Ultrasonic_EXTI_Callback(uint16_t gpio_pin)
{
  int i;
  for (i = 0; i < US_SENSOR_NUM; i++) {
    if (s_pin[i].echo_pin != gpio_pin) {
      continue;                                 /* このセンサのピンではない */
    }
    uint8_t bit = (uint8_t)(1U << i);
    if (!(s_waiting & bit)) {
      return;                                   /* Trig していないのに変化 = ノイズなので無視 */
    }
    if (HAL_GPIO_ReadPin(GPIOC, gpio_pin) == GPIO_PIN_SET) {
      /* ピンが HIGH = Echo の立ち上がり。今の時刻を覚える */
      s_rise_us[i]  = TIM5->CNT;
      s_rise_valid |= bit;
    } else if (s_rise_valid & bit) {
      /* ピンが LOW = Echo の立ち下がり。（今の時刻 - 立ち上がり時刻）がパルス幅。
       * 符号なし32bit の引き算なので、カウンタが一周していても正しい値になる */
      uint32_t width = TIM5->CNT - s_rise_us[i];
      s_rise_valid &= (uint8_t)~bit;
      s_waiting    &= (uint8_t)~bit;            /* Echo が来たので 待ち状態を解除 */
      Ultrasonic_OnEcho(i, (float)width);       /* 距離を計算して保存 */
    }
    return;
  }
}

/**
  * @brief  接続確認。センサを1個ずつ鳴らし、距離が返ってくるかを調べる。
  *         周期計測(Ultrasonic_Trigger)を始める前に1回呼ぶ。約 4 x 60ms かかる。
  *         1個ずつ鳴らすのは、音の混信で配線ミスを見逃さないため。
  * @param  ok_mask  結果。bit i = 1 ならセンサ i が正常。
  *                  例: 0x0B(1011) -> センサ0,1,3 は正常、センサ2 は応答なし。NULL 可
  * @retval 0: 4個とも正常 / -1: 応答のないセンサあり
  */
int Ultrasonic_CheckConnection(uint8_t *ok_mask)
{
  uint8_t mask = 0U;
  int i;

  for (i = 0; i < US_SENSOR_NUM; i++) {
    s_no_resp[i] = 1;                           /* 前回の結果を消してから測る */
    s_dist_mm[i] = 0.0f;
    fire((uint8_t)(1U << i));                   /* センサ i だけ鳴らす */
    HAL_Delay(US_CHECK_WAIT_MS);                /* Echo が戻るのを待つ */

    /* 正常 = Echo が来て（待ち状態が解除済み）、距離が 20〜4000mm の範囲内 */
    if (!(s_waiting & (1U << i)) && s_no_resp[i] == 0 &&
        s_dist_mm[i] >= US_RANGE_MIN_MM && s_dist_mm[i] <= US_RANGE_MAX_MM) {
      mask |= (uint8_t)(1U << i);
    }
    __disable_irq();
    s_waiting &= (uint8_t)~(1U << i);           /* 来なかった場合の待ち状態を片付ける */
    __enable_irq();
  }

  if (ok_mask != NULL) {
    *ok_mask = mask;
  }
  return (mask == 0x0FU) ? 0 : -1;
}
