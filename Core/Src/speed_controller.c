/**
  ******************************************************************************
  * @file    speed_controller.c
  * @brief   速度制御器（Issue #33 / 親 #5）。速度PI制御。
  ******************************************************************************
  */
#include "speed_controller.h"

void SpeedCtrl_Init(SpeedCtrl *c, float kp, float ki, float integ_max)
{
    c->kp        = kp;
    c->ki        = ki;
    c->integ_max = integ_max;
    SpeedCtrl_Reset(c);
}

void SpeedCtrl_Reset(SpeedCtrl *c)
{
    c->integ = 0.0f;
}

float SpeedCtrl_Update(SpeedCtrl *c, float error, float dt)
{
    c->integ += error * dt;

    /* 積分項（ki × integ）が ±integ_max を超えないように、積分値を止める（アンチワインドアップ） */
    if (c->ki > 0.0f)
    {
        float limit = c->integ_max / c->ki;
        if (c->integ >  limit) c->integ =  limit;
        if (c->integ < -limit) c->integ = -limit;
    }

    return c->kp * error + c->ki * c->integ;
}
