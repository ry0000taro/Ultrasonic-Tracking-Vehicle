/**
  ******************************************************************************
  * @file    lcd.c
  * @brief   液晶 MSP2807（ILI9341 想定）の簡易ドライバ（Issue #18）。ピンは lcd.h を参照。
  ******************************************************************************
  */
#include "lcd.h"

extern SPI_HandleTypeDef hspi2;   /* main.c で CubeMX が生成 */

#define LCD_SPI_TIMEOUT  100U

/* ILI9341 のコマンド */
#define CMD_SWRESET  0x01
#define CMD_SLPOUT   0x11
#define CMD_DISPON   0x29
#define CMD_CASET    0x2A
#define CMD_PASET    0x2B
#define CMD_RAMWR    0x2C
#define CMD_MADCTL   0x36
#define CMD_COLMOD   0x3A

/* ---- 低レベル: 1バイト送信 ---- */
static void lcd_cs(int level)    { HAL_GPIO_WritePin(LCD_CS_GPIO_Port,    LCD_CS_Pin,    level ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void lcd_dc(int level)    { HAL_GPIO_WritePin(LCD_DC_GPIO_Port,    LCD_DC_Pin,    level ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void lcd_reset(int level) { HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET); }

static void lcd_write_cmd(uint8_t cmd)
{
  lcd_dc(0);
  lcd_cs(0);
  HAL_SPI_Transmit(&hspi2, &cmd, 1, LCD_SPI_TIMEOUT);
  lcd_cs(1);
}

static void lcd_write_data(const uint8_t *data, uint16_t len)
{
  lcd_dc(1);
  lcd_cs(0);
  HAL_SPI_Transmit(&hspi2, (uint8_t *)data, len, LCD_SPI_TIMEOUT);
  lcd_cs(1);
}

/* コマンド + データ引数 */
static void lcd_cmd_args(uint8_t cmd, const uint8_t *args, uint16_t n)
{
  lcd_write_cmd(cmd);
  if (n > 0U) {
    lcd_write_data(args, n);
  }
}

/* 描画範囲を設定して RAM 書き込みを開始する（このあと DC=1 でピクセルを送る） */
static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint8_t col[4] = { (uint8_t)(x0 >> 8), (uint8_t)x0, (uint8_t)(x1 >> 8), (uint8_t)x1 };
  uint8_t row[4] = { (uint8_t)(y0 >> 8), (uint8_t)y0, (uint8_t)(y1 >> 8), (uint8_t)y1 };
  lcd_cmd_args(CMD_CASET, col, 4);
  lcd_cmd_args(CMD_PASET, row, 4);
  lcd_write_cmd(CMD_RAMWR);
}

/* ---- 公開 API ---- */
void Lcd_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* RESET(PA8) / DC(PA9) / CS(PB12) を GPIO 出力にする（テスト用の TIM1 設定を上書きする） */
  lcd_cs(1);
  lcd_dc(0);
  lcd_reset(1);
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin = LCD_RESET_Pin | LCD_DC_Pin;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = LCD_CS_Pin;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &g);

  /* ハードウェアリセット */
  lcd_reset(0);
  HAL_Delay(20);
  lcd_reset(1);
  HAL_Delay(120);

  lcd_write_cmd(CMD_SWRESET);
  HAL_Delay(120);

  /* ILI9341 の標準的な初期化列（Adafruit ライブラリなどで広く使われているもの） */
  { const uint8_t d[] = {0x00, 0xC1, 0x30};             lcd_cmd_args(0xCF, d, sizeof d); }
  { const uint8_t d[] = {0x64, 0x03, 0x12, 0x81};       lcd_cmd_args(0xED, d, sizeof d); }
  { const uint8_t d[] = {0x85, 0x00, 0x78};             lcd_cmd_args(0xE8, d, sizeof d); }
  { const uint8_t d[] = {0x39, 0x2C, 0x00, 0x34, 0x02}; lcd_cmd_args(0xCB, d, sizeof d); }
  { const uint8_t d[] = {0x20};                         lcd_cmd_args(0xF7, d, sizeof d); }
  { const uint8_t d[] = {0x00, 0x00};                   lcd_cmd_args(0xEA, d, sizeof d); }
  { const uint8_t d[] = {0x23};                         lcd_cmd_args(0xC0, d, sizeof d); }
  { const uint8_t d[] = {0x10};                         lcd_cmd_args(0xC1, d, sizeof d); }
  { const uint8_t d[] = {0x3E, 0x28};                   lcd_cmd_args(0xC5, d, sizeof d); }
  { const uint8_t d[] = {0x86};                         lcd_cmd_args(0xC7, d, sizeof d); }
  { const uint8_t d[] = {0x48};                         lcd_cmd_args(CMD_MADCTL, d, sizeof d); }  /* 縦向き, BGR */
  { const uint8_t d[] = {0x55};                         lcd_cmd_args(CMD_COLMOD, d, sizeof d); }  /* 16bit/pixel */
  { const uint8_t d[] = {0x00, 0x18};                   lcd_cmd_args(0xB1, d, sizeof d); }
  { const uint8_t d[] = {0x08, 0x82, 0x27};             lcd_cmd_args(0xB6, d, sizeof d); }
  { const uint8_t d[] = {0x00};                         lcd_cmd_args(0xF2, d, sizeof d); }
  { const uint8_t d[] = {0x01};                         lcd_cmd_args(0x26, d, sizeof d); }
  { const uint8_t d[] = {0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00};
                                                        lcd_cmd_args(0xE0, d, sizeof d); }
  { const uint8_t d[] = {0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F};
                                                        lcd_cmd_args(0xE1, d, sizeof d); }

  lcd_write_cmd(CMD_SLPOUT);
  HAL_Delay(120);
  lcd_write_cmd(CMD_DISPON);
  HAL_Delay(20);
}

void Lcd_FillScreen(uint16_t color)
{
  uint8_t row[LCD_WIDTH * 2U];
  for (uint16_t x = 0; x < LCD_WIDTH; x++) {
    row[2U * x]      = (uint8_t)(color >> 8);
    row[2U * x + 1U] = (uint8_t)color;
  }
  lcd_set_window(0, 0, LCD_WIDTH - 1U, LCD_HEIGHT - 1U);
  lcd_dc(1);
  lcd_cs(0);
  for (uint16_t y = 0; y < LCD_HEIGHT; y++) {
    HAL_SPI_Transmit(&hspi2, row, sizeof row, LCD_SPI_TIMEOUT);
  }
  lcd_cs(1);
}

/* (x, y) のピクセル色を返す。上から: カラーバー / 青→水色のグラデーション(顔つき) / 白黒の縞 */
static uint16_t test_pixel(int x, int y)
{
  /* 上段 0..59: 8色のカラーバー（白 黄 水 緑 紫 赤 青 黒） */
  if (y < 60) {
    static const uint16_t bars[8] = {
      LCD_RGB(255,255,255), LCD_RGB(255,255,0), LCD_RGB(0,255,255), LCD_RGB(0,255,0),
      LCD_RGB(255,0,255),   LCD_RGB(255,0,0),   LCD_RGB(0,0,255),   LCD_RGB(0,0,0)
    };
    return bars[(x * 8) / (int)LCD_WIDTH];
  }

  /* 下段 280..319: 白黒の縞（8px ごと）。表示の向き・欠けの確認用 */
  if (y >= 280) {
    return (((x / 8) + ((y - 280) / 8)) & 1) ? LCD_WHITE : LCD_BLACK;
  }

  /* 中段: 縦のグラデーション背景 + 顔（黄色い円、目、口） */
  const int cx = 120, cy = 165, r = 70;
  int dx = x - cx, dy = y - cy;
  int d2 = dx * dx + dy * dy;

  /* 目（黒い円2つ） */
  int ex = (dx < 0 ? -dx : dx) - 26;       /* 左右の目の中心 x = cx ± 26 */
  int ey = dy + 22;                        /* 目の中心 y = cy - 22 */
  if (ex * ex + ey * ey <= 10 * 10) {
    return LCD_BLACK;
  }
  /* 口（下半分の細い円環） */
  int my = dy - 8;                         /* 口の中心 y = cy + 8 */
  int m2 = dx * dx + my * my;
  if (my > 0 && m2 >= 38 * 38 && m2 <= 46 * 46) {
    return LCD_BLACK;
  }
  /* 顔（黄色）と輪郭（赤） */
  if (d2 <= (r - 3) * (r - 3)) {
    return LCD_RGB(255, 220, 0);
  }
  if (d2 <= r * r) {
    return LCD_RGB(220, 0, 0);
  }
  /* 背景: 上(青)から下(水色)へ */
  int t = (y - 60) * 255 / 220;            /* 0..255 */
  return LCD_RGB(0, t / 2, 128 + t / 2);
}

void Lcd_DrawTestImage(void)
{
  uint8_t row[LCD_WIDTH * 2U];

  lcd_set_window(0, 0, LCD_WIDTH - 1U, LCD_HEIGHT - 1U);
  lcd_dc(1);
  lcd_cs(0);
  for (int y = 0; y < (int)LCD_HEIGHT; y++) {
    for (int x = 0; x < (int)LCD_WIDTH; x++) {
      uint16_t c = test_pixel(x, y);
      row[2 * x]     = (uint8_t)(c >> 8);
      row[2 * x + 1] = (uint8_t)c;
    }
    HAL_SPI_Transmit(&hspi2, row, sizeof row, LCD_SPI_TIMEOUT);
  }
  lcd_cs(1);
}
