/**
  ******************************************************************************
  * @file    lcd_ili9341.h
  * @brief   液晶 MSP2807 (ILI9341, 240x320) の最小ドライバ（動作確認用）
  *
  * SPI2 (SCK=PB13, MOSI=PB15) 送信専用、CS=PB12, DC=PB10, RESET=PB14。
  * 色は RGB565。画面の向きとバイト順は実機で確認する（LCD-04）。
  ******************************************************************************
  */
#ifndef LCD_ILI9341_H
#define LCD_ILI9341_H

#include "main.h"

#define LCD_WIDTH   240U
#define LCD_HEIGHT  320U

#define LCD_BLACK   0x0000U
#define LCD_WHITE   0xFFFFU
#define LCD_RED     0xF800U
#define LCD_GREEN   0x07E0U
#define LCD_BLUE    0x001FU
#define LCD_YELLOW  0xFFE0U

/* ハードウェアリセット後に初期化コマンドを送り、表示を ON にする */
void Lcd_Init(void);

/* 指定の矩形を単色で塗りつぶす（x, y は左上、w, h は大きさ） */
void Lcd_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/* 画面全体を単色で塗りつぶす */
void Lcd_FillScreen(uint16_t color);

/* 動作確認: 赤→緑→青→白→黒で全画面を塗り、最後に色の帯を描く（約3秒） */
void Lcd_Test(void);

#endif /* LCD_ILI9341_H */
