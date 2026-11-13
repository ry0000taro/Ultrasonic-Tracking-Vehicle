/**
  ******************************************************************************
  * @file    distance_process.c
  * @brief   距離データ処理（制御層）。仕様は distance_process.h を参照
  *
  * 位置(X,Y)は、各センサの距離 di と dᵢ = √((X-xᵢ)² + (Y-yᵢ)²) の残差の二乗和を
  * ガウス・ニュートン法で最小化して求める（パラメータ2個なので 2x2 を直接解く）。
  ******************************************************************************
  */
#include "distance_process.h"
#include <math.h>
#include <stddef.h>

#define DP_MIN_RHO_MM     1.0f    /* センサ〜推定点の距離がこれ未満なら微分できない */
#define DP_MIN_DET        1e-6f   /* JtJ の行列式の下限（これ未満は縮退として失敗） */
#define DP_MIN_Y_MM       1.0f    /* Y>0 の側に置くための下限 */

static const float s_sx[DP_SENSOR_NUM] = DP_SENSOR_X_MM;
static const float s_sy[DP_SENSOR_NUM] = DP_SENSOR_Y_MM;

/* 前回の推定値（次回の初期値）。初回は (0, 追従距離) */
static float s_prev_x = 0.0f;
static float s_prev_y = DP_INIT_FOLLOW_MM;

static void reset_prev(void)
{
    s_prev_x = 0.0f;
    s_prev_y = DP_INIT_FOLLOW_MM;
}

/* 有効値 n 個(idx[])から (X,Y) を推定。0:成功 / -1:失敗 */
static int estimate(const float d[DP_SENSOR_NUM], const int idx[], int n,
                    float *px, float *py)
{
    float x = s_prev_x;
    float y = s_prev_y;

    for (int iter = 0; iter < DP_GN_MAX_ITER; iter++) {
        float a = 0.0f, b = 0.0f, c = 0.0f;   /* JtJ = [a b; b c] */
        float g0 = 0.0f, g1 = 0.0f;           /* Jt r */

        for (int k = 0; k < n; k++) {
            int i = idx[k];
            float dx = x - s_sx[i];
            float dy = y - s_sy[i];
            float rho = sqrtf(dx * dx + dy * dy);
            if (rho < DP_MIN_RHO_MM) {
                /* 推定点がセンサ上に重なり微分できない。Y側へずらして続ける */
                dx = 0.0f;
                dy = DP_MIN_RHO_MM;
                rho = DP_MIN_RHO_MM;
            }
            float jx = dx / rho;
            float jy = dy / rho;
            float r = d[i] - rho;
            a  += jx * jx;
            b  += jx * jy;
            c  += jy * jy;
            g0 += jx * r;
            g1 += jy * r;
        }

        float det = a * c - b * b;
        if (det < DP_MIN_DET) {
            return -1;                         /* 縮退（センサが一直線上など） */
        }
        float sx = ( c * g0 - b * g1) / det;
        float sy = (-b * g0 + a * g1) / det;

        x += sx;
        y += sy;
        if (y < DP_MIN_Y_MM) {                 /* 必ず Y>0 の側に置く（鏡像解の除外） */
            y = (y < 0.0f) ? -y : 0.0f;
            if (y < DP_MIN_Y_MM) {
                y = DP_MIN_Y_MM;
            }
        }
        if (!isfinite(x) || !isfinite(y)) {
            return -1;
        }
        if (fabsf(sx) < DP_GN_EPS_MM && fabsf(sy) < DP_GN_EPS_MM) {
            *px = x;
            *py = y;
            return 0;
        }
    }
    return -1;                                 /* 収束しない */
}

int Distance_Process(const float dist_mm[DP_SENSOR_NUM], UsResult *out)
{
    if (dist_mm == NULL || out == NULL) {
        return -2;
    }

    int idx[DP_SENSOR_NUM];
    int n = 0;
    int detect = 0;

    for (int i = 0; i < DP_SENSOR_NUM; i++) {
        float d = dist_mm[i];
        if (d >= DP_RANGE_MIN_MM && d <= DP_RANGE_MAX_MM) {   /* NaN は偽になり無効 */
            out->dist_mm[i] = d;
            idx[n++] = i;
            if (d <= DP_DETECT_MM) {
                detect = 1;
            }
        } else {
            out->dist_mm[i] = -1.0f;
        }
    }
    out->detect = detect;

    out->x_mm = 0.0f;
    out->y_mm = 0.0f;
    out->estimated = 0;

    if (n < 2) {
        return -1;                             /* 前回値は残す */
    }

    float x, y;
    if (estimate(out->dist_mm, idx, n, &x, &y) != 0) {
        reset_prev();                          /* 発散した初期値は捨てて初回値に戻す */
        return -1;
    }

    s_prev_x = x;
    s_prev_y = y;
    out->x_mm = x;
    out->y_mm = y;
    out->estimated = 1;
    return 0;
}
