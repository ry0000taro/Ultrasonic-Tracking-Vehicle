#ifndef PINMAP_H
#define PINMAP_H

#include "stm32f4xx_hal.h"

/*
 * NUCLEO-F446RE ピン割り当て
 * 出典: Notion「マイコンピン対応表(NUCLEO-F446RE)」
 * 基板設計資料と電気系設計資料で重複していたピンは、電気系設計資料側(超音波・液晶)を
 * 優先して確定した。基板設計資料側の A/D1~4・タクトスイッチS1,S2,S4・LED D1~D4 は
 * 表示器をLCDに変更済み(docs/module/07_display_notification.md)のため未使用。
 */

/* 超音波センサ (HC-SR04) U1~U4 : Trig=出力, Echo=入力(5V->3.3V分圧の中点) */
#define US1_TRIG_GPIO_Port      GPIOC
#define US1_TRIG_Pin            GPIO_PIN_10
#define US1_ECHO_GPIO_Port      GPIOC
#define US1_ECHO_Pin            GPIO_PIN_0

#define US2_TRIG_GPIO_Port      GPIOC
#define US2_TRIG_Pin            GPIO_PIN_11
#define US2_ECHO_GPIO_Port      GPIOC
#define US2_ECHO_Pin            GPIO_PIN_1

#define US3_TRIG_GPIO_Port      GPIOC
#define US3_TRIG_Pin            GPIO_PIN_12
#define US3_ECHO_GPIO_Port      GPIOC
#define US3_ECHO_Pin            GPIO_PIN_2

#define US4_TRIG_GPIO_Port      GPIOC
#define US4_TRIG_Pin            GPIO_PIN_4
#define US4_ECHO_GPIO_Port      GPIOC
#define US4_ECHO_Pin            GPIO_PIN_3

/* 液晶 (ILI9341/MSP2807, SPI) */
#define LCD_CS_GPIO_Port        GPIOB
#define LCD_CS_Pin              GPIO_PIN_12
#define LCD_SCK_GPIO_Port       GPIOB
#define LCD_SCK_Pin             GPIO_PIN_13
#define LCD_SDI_GPIO_Port       GPIOB   /* マイコン→液晶(MOSI相当) */
#define LCD_SDI_Pin             GPIO_PIN_15
#define LCD_DC_GPIO_Port        GPIOA
#define LCD_DC_Pin              GPIO_PIN_9
#define LCD_RESET_GPIO_Port     GPIOA
#define LCD_RESET_Pin           GPIO_PIN_8

/* ブザー (UDB-05LFPN、トランジスタ2SC1815L-Y経由) */
#define BUZZER_GPIO_Port        GPIOA
#define BUZZER_Pin              GPIO_PIN_0

/* モータードライバ */
#define MOTOR1_IN1_GPIO_Port    GPIOA
#define MOTOR1_IN1_Pin          GPIO_PIN_6   /* JP1-2 */
#define MOTOR1_IN2_GPIO_Port    GPIOA
#define MOTOR1_IN2_Pin          GPIO_PIN_7   /* JP1-3 */

#define MOTOR2_IN1_GPIO_Port    GPIOB
#define MOTOR2_IN1_Pin          GPIO_PIN_5   /* JP2-2 */
#define MOTOR2_IN2_GPIO_Port    GPIOB
#define MOTOR2_IN2_Pin          GPIO_PIN_4   /* JP2-3 */

/* エンコーダ (5V -> 3.3V 変換モジュール経由) */
#define ENC1_A_GPIO_Port        GPIOC
#define ENC1_A_Pin              GPIO_PIN_9   /* EA5 */
#define ENC1_B_GPIO_Port        GPIOC
#define ENC1_B_Pin              GPIO_PIN_8   /* EA3 */

#define ENC2_A_GPIO_Port        GPIOB
#define ENC2_A_Pin              GPIO_PIN_7   /* EB5 */
#define ENC2_B_GPIO_Port        GPIOB
#define ENC2_B_Pin              GPIO_PIN_6   /* EB3 */

/* I2C1 (予備コネクタ J3) */
#define I2C1_SCL_GPIO_Port      GPIOB
#define I2C1_SCL_Pin            GPIO_PIN_8
#define I2C1_SDA_GPIO_Port      GPIOB
#define I2C1_SDA_Pin            GPIO_PIN_9

/* 開始/停止スイッチ (IF-05、実装方式は未確定。タクトスイッチS3を仮に割当) */
#define SW_START_STOP_GPIO_Port GPIOC
#define SW_START_STOP_Pin       GPIO_PIN_13

#endif /* PINMAP_H */
