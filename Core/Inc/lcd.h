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

/* 横向き（MADCTL=0x28）: 幅 320 x 高さ 240。縦向きに戻すなら 240 x 320 にして MADCTL を 0x48 にする */
#define LCD_WIDTH   320U
#define LCD_HEIGHT  240U

/* RGB565 の色（R5 G6 B5） */
#define LCD_RGB(r, g, b)  ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define LCD_BLACK   0x0000U
#define LCD_WHITE   0xFFFFU

/* PA8/PA9/PB12 を GPIO 出力にして、液晶を初期化する。
 * 注意: PA8/PA9 を GPIO に戻すため、TE-01 テスト用の Encoder1(TIM1, PA8/PA9) はリセットまで動かなくなる。 */
void Lcd_Init(void);

/* 画面全体を1色で塗りつぶす */
void Lcd_FillScreen(uint16_t color);

/* 状態表示（停止中 / 追従中 / エラー） */
typedef enum {
  LCD_STATE_STOP = 0,    /* 停止中: 赤の背景 + 白い四角 + "STOP" */
  LCD_STATE_FOLLOW,      /* 追従中: 緑の背景 + 白い右向き三角 + "FOLLOW" */
  LCD_STATE_ERROR        /* エラー: 黄の背景 + 黒い警告三角 + "ERROR" */
} LcdState;

/* 状態を画面全体に表示する。日本語フォントがないため、文字は英字（STOP / FOLLOW / ERROR）。 */
void Lcd_DrawState(LcdState s);

#endif /* LCD_H */
