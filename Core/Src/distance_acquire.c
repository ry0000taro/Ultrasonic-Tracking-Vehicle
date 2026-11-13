/**
  ******************************************************************************
  * @file    distance_acquire.c
  * @brief   距離データの周期取得（ノンブロッキング）
  ******************************************************************************
  */
#include "distance_acquire.h"
#include "main.h"          /* HAL_GetTick */

static uint32_t s_last_ms;                 /* 前回の周期処理の時刻 */
static int      s_pair_id;                 /* 次に発火する組 */
static int      s_triggered;               /* 1: 発火済みで、結果の読み出し待ち */
static int      s_valid;                   /* 1: 最新データあり */

static float    s_dist[US_SENSOR_NUM];
static int      s_no_resp[US_SENSOR_NUM];
static UsResult s_res;

void DistanceAcq_Init(void)
{
    Ultrasonic_Init();
    s_pair_id   = 0;
    s_triggered = 0;
    s_valid     = 0;
    s_last_ms   = HAL_GetTick() - DISTANCE_ACQ_PERIOD_MS;   /* 最初の呼び出しですぐ発火する */
}

int DistanceAcq_Task(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_last_ms) < DISTANCE_ACQ_PERIOD_MS) {
        return 0;
    }
    s_last_ms = now;

    int updated = 0;

    /* 1) 前回の発火（60ms 前）の結果を読み、距離データ処理にかける */
    if (s_triggered) {
        float dist[US_SENSOR_NUM];
        int   no_resp[US_SENSOR_NUM];
        UsResult res;

        Ultrasonic_GetResult(dist, no_resp);
        Distance_Process(dist, &res);   /* 位置を推定できなくても他の値は更新される */

        for (int i = 0; i < US_SENSOR_NUM; i++) {
            s_dist[i]    = dist[i];
            s_no_resp[i] = no_resp[i];
        }
        s_res   = res;
        s_valid = 1;
        updated = 1;
    }

    /* 2) 次の組を発火する（応答は Echo 割り込みで保存される） */
    Ultrasonic_Trigger(s_pair_id);
    s_pair_id   = (s_pair_id + 1) % US_PAIR_NUM;
    s_triggered = 1;

    return updated;
}

int DistanceAcq_Get(float dist_mm[US_SENSOR_NUM], int no_resp[US_SENSOR_NUM], UsResult *res)
{
    if (!s_valid) {
        return -1;
    }
    if (dist_mm) {
        for (int i = 0; i < US_SENSOR_NUM; i++) dist_mm[i] = s_dist[i];
    }
    if (no_resp) {
        for (int i = 0; i < US_SENSOR_NUM; i++) no_resp[i] = s_no_resp[i];
    }
    if (res) {
        *res = s_res;
    }
    return 0;
}
