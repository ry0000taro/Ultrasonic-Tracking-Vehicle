/**
  ******************************************************************************
  * @file    position_controller.c
  * @brief   位置制御器（Issue #31 / 親 #3）。今は P 制御のみ。
  ******************************************************************************
  */
#include "position_controller.h"

void PosCtrl_Init(PosCtrl *c, float kp)
{
    c->kp = kp;
    PosCtrl_Reset(c);
}

void PosCtrl_Reset(PosCtrl *c)
{
    (void)c;    /* P 制御は内部状態を持たない */
}

float PosCtrl_Update(PosCtrl *c, float error)
{
    return c->kp * error;
}
