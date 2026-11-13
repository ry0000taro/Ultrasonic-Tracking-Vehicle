/**
  ******************************************************************************
  * @file    lcd.h
  * @brief   液晶 MSP2807（2.8インチ 240x320 SPI）の簡易ドライバ（Issue #18）
  *
  * 配線（電気系設計の回路図で確認、2026-11-13）:
  *   CS=PB12(GPIO)  RESET=PA8(GPIO)  DC/RS=PA9(GPIO)
  *   SCK=PB13(SPI2_SCK)  SDI=PB15(SPI2_MOSI)  ※液晶は受信のみ（MISO 不要）
  *
  * 駆動ICは ILI9341 を仮定している（MSP2807 は一般に ILI9341。ドキュメント上は未確認）。
  * SPI2 は CubeMX 生成の hspi2（Mode0、8bit、MSB first、約5.25MHz）を使う。
  ******************************************************************************
  */
#ifndef LCD_H
#define LCD_H

#include "main.h"

#define LCD_WIDTH   240U
#define LCD_HEIGHT  320U

/* RGB565 の色（R5 G6 B5） */
#define LCD_RGB(r, g, b)  ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define LCD_BLACK   0x0000U
#define LCD_WHITE   0xFFFFU

/* PA8/PA9/PB12 を GPIO 出力にして、液晶を初期化する。
 * 注意: PA8/PA9 を GPIO に戻すため、TE-01 テスト用の Encoder1(TIM1, PA8/PA9) はリセットまで動かなくなる。 */
void Lcd_Init(void);

/* 画面全体を1色で塗りつぶす */
void Lcd_FillScreen(uint16_t color);

/* 確認用の適当な画像（カラーバー + グラデーション + 顔）を表示する */
void Lcd_DrawTestImage(void);

#endif /* LCD_H */
