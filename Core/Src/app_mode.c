/**
  ******************************************************************************
  * @file    app_mode.c
  * @brief   起動モードの選択と振り分け
  ******************************************************************************
  */
#include "app_mode.h"
#include "motor_encoder_test.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;

static void Print_ModeMenu(void)
{
    printf("\r\n================================================\r\n");
    printf("   Ultrasonic Tracking Vehicle  -  Mode select\r\n");
    printf("================================================\r\n");
    printf(" 1 : Test             (hardware checks: motor/encoder/US/LCD/buzzer)\r\n");
    printf(" 2 : SpeedControl     (not implemented yet)\r\n");
    printf(" 3 : PositionControl  (not implemented yet)\r\n");
    printf(" 4 : Combined         (not implemented yet)\r\n");
    printf("------------------------------------------------\r\n");
    printf(" Enter number > ");
}

AppMode AppMode_Select(void)
{
    Print_ModeMenu();

    while (1) {
        uint8_t c = 0;
        if (HAL_UART_Receive(&huart2, &c, 1, HAL_MAX_DELAY) != HAL_OK) {
            continue;
        }
        printf("%c\r\n", c);

        switch (c) {
        case '1': return APP_MODE_TEST;
        case '2': return APP_MODE_SPEED;
        case '3': return APP_MODE_POSITION;
        case '4': return APP_MODE_COMBINED;
        default:
            printf("[!] Invalid input. Enter 1-4.\r\n Enter number > ");
            break;
        }
    }
}

static void Run_NotImplemented(const char *name)
{
    printf("[%s] not implemented yet. Returning to mode select.\r\n", name);
}

void AppMode_Run(AppMode mode)
{
    switch (mode) {
    case APP_MODE_TEST:
        MotorEncoderTest_Run();   /* 戻らない */
        break;
    case APP_MODE_SPEED:
        Run_NotImplemented("SpeedControl");
        break;
    case APP_MODE_POSITION:
        Run_NotImplemented("PositionControl");
        break;
    case APP_MODE_COMBINED:
        Run_NotImplemented("Combined");
        break;
    default:
        break;
    }
}
