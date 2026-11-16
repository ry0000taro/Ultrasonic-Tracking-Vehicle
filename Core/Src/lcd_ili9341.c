/**
  ******************************************************************************
  * @file    lcd_ili9341.c
  * @brief   液晶 MSP2807 (ILI9341) の最小ドライバ（動作確認用）
  ******************************************************************************
  */
#include "lcd_ili9341.h"

extern SPI_HandleTypeDef hspi2;

/* TODO(#18 レビュー指摘・見送り): このドライバは動作確認用の最小実装。本番の表示処理（LCD-05/06）を作るときに直すこと。
 *   - HAL_SPI_Transmit の戻り値を見ていない。送信エラー/タイムアウトでも CS を戻して処理を続けるため、
 *     初期化が途中で失敗しても気づけない。戻り値を確認し、失敗を上位に返す。
 *   - 塗りつぶしは 256 バイトずつ約 600 回に分けて送るので遅く、メインループを止める（全画面で約 0.12 秒）。
 *     バッファを大きくするか DMA を使う。 */
#define LCD_SPI_TIMEOUT  100U
#define LCD_CHUNK_PIXELS 128U       /* 塗りつぶし時に 1 回で送る画素数 */

/* 画面の向きとバイト順: 0x48 = 縦向き, BGR（MSP2807 で一般的な値。実機で確認する） */
#define LCD_MADCTL_VALUE 0x48U

static void lcd_select(void)   { HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); }
static void lcd_deselect(void) { HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); }

static void lcd_write_cmd(uint8_t cmd)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);   /* DC=0: コマンド */
    lcd_select();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, LCD_SPI_TIMEOUT);
    lcd_deselect();
}

static void lcd_write_data(const uint8_t *data, uint16_t len)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);     /* DC=1: データ */
    lcd_select();
    HAL_SPI_Transmit(&hspi2, (uint8_t *)data, len, LCD_SPI_TIMEOUT);
    lcd_deselect();
}

/* コマンド + パラメータ列 */
static void lcd_cmd_args(uint8_t cmd, const uint8_t *args, uint16_t len)
{
    lcd_write_cmd(cmd);
    if (len > 0U) lcd_write_data(args, len);
}

#define CMD(c, ...) do { static const uint8_t a[] = { __VA_ARGS__ }; lcd_cmd_args((c), a, sizeof(a)); } while (0)

void Lcd_Init(void)
{
    /* ハードウェアリセット */
    lcd_deselect();
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(120);

    lcd_write_cmd(0x01);            /* ソフトウェアリセット */
    HAL_Delay(150);

    /* ILI9341 の標準的な初期化列 */
    CMD(0xCF, 0x00, 0xC1, 0x30);
    CMD(0xED, 0x64, 0x03, 0x12, 0x81);
    CMD(0xE8, 0x85, 0x00, 0x78);
    CMD(0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02);
    CMD(0xF7, 0x20);
    CMD(0xEA, 0x00, 0x00);
    CMD(0xC0, 0x23);                /* 電源制御 1 */
    CMD(0xC1, 0x10);                /* 電源制御 2 */
    CMD(0xC5, 0x3E, 0x28);          /* VCOM 制御 1 */
    CMD(0xC7, 0x86);                /* VCOM 制御 2 */
    CMD(0x36, LCD_MADCTL_VALUE);    /* メモリアクセス制御（向き・RGB/BGR） */
    CMD(0x3A, 0x55);                /* 16bit/pixel (RGB565) */
    CMD(0xB1, 0x00, 0x18);          /* フレームレート */
    CMD(0xB6, 0x08, 0x82, 0x27);    /* ディスプレイ機能制御 */
    CMD(0xF2, 0x00);
    CMD(0x26, 0x01);                /* ガンマカーブ選択 */
    CMD(0xE0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
              0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00);
    CMD(0xE1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
              0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F);

    lcd_write_cmd(0x11);            /* スリープ解除 */
    HAL_Delay(120);
    lcd_write_cmd(0x29);            /* 表示 ON */
    HAL_Delay(20);
}

/* 描画範囲を設定し、続けて RAM 書き込みを開始する */
static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t col[4] = { (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1 };
    uint8_t row[4] = { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1 };
    lcd_cmd_args(0x2A, col, 4);     /* 列アドレス */
    lcd_cmd_args(0x2B, row, 4);     /* 行アドレス */
    lcd_write_cmd(0x2C);            /* メモリ書き込み */
}

void Lcd_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (w == 0U || h == 0U || x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if (x + w > LCD_WIDTH)  w = LCD_WIDTH  - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;

    uint8_t buf[LCD_CHUNK_PIXELS * 2U];
    for (uint16_t i = 0; i < LCD_CHUNK_PIXELS; i++)
    {
        buf[2U * i]      = (uint8_t)(color >> 8);
        buf[2U * i + 1U] = (uint8_t)color;
    }

    lcd_set_window(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));

    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    lcd_select();
    uint32_t remaining = (uint32_t)w * h;
    while (remaining > 0U)
    {
        uint32_t n = (remaining > LCD_CHUNK_PIXELS) ? LCD_CHUNK_PIXELS : remaining;
        HAL_SPI_Transmit(&hspi2, buf, (uint16_t)(n * 2U), LCD_SPI_TIMEOUT);
        remaining -= n;
    }
    lcd_deselect();
}

void Lcd_FillScreen(uint16_t color)
{
    Lcd_FillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
}

void Lcd_Test(void)
{
    static const uint16_t colors[] = { LCD_RED, LCD_GREEN, LCD_BLUE, LCD_WHITE, LCD_BLACK };

    Lcd_Init();
    for (unsigned i = 0; i < sizeof(colors) / sizeof(colors[0]); i++)
    {
        Lcd_FillScreen(colors[i]);
        HAL_Delay(500);
    }

    /* 向きと色の確認用: 上から 赤・緑・青・黄 の帯。左上(0,0)に白い小さな四角 */
    static const uint16_t bands[] = { LCD_RED, LCD_GREEN, LCD_BLUE, LCD_YELLOW };
    for (unsigned i = 0; i < 4U; i++)
    {
        Lcd_FillRect(0, (uint16_t)(i * (LCD_HEIGHT / 4U)), LCD_WIDTH, LCD_HEIGHT / 4U, bands[i]);
    }
    Lcd_FillRect(0, 0, 20, 20, LCD_WHITE);
}
