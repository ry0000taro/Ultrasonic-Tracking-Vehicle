/**
  ******************************************************************************
  * @file    lcd.c
  * @brief   液晶 MSP2807（ILI9341 想定）の簡易ドライバ（Issue #18）。ピンは lcd.h を参照。
  ******************************************************************************
  */
#include "lcd.h"
#include <string.h>

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

/* ---- 状態表示（停止中 / 追従中 / エラー）----
 * 日本語フォントを持っていないため、文字は 5x7 の英字（STOP / FOLLOW / ERROR）を拡大して描く。
 * 色 + 図形 + 英字で状態を区別する。
 *   STOP   : 赤の背景 + 白い四角     （停止中）
 *   FOLLOW : 緑の背景 + 白い右向き三角（追従中）
 *   ERROR  : 黄の背景 + 黒い三角と「!」（エラー）
 */
#define TEXT_SCALE   6          /* 5x7 を 6 倍 → 1 文字 30x42 px、字間 6 px */
#define TEXT_TOP     200        /* 文字の上端 y */
#define TEXT_CELL    (6 * TEXT_SCALE)

/* 5x7 フォント（列ごとのビット。bit0 が上）。使う文字だけ持つ */
static const uint8_t *glyph(char ch)
{
  static const uint8_t E[5] = {0x7F,0x49,0x49,0x49,0x41};
  static const uint8_t F[5] = {0x7F,0x09,0x09,0x09,0x01};
  static const uint8_t L[5] = {0x7F,0x40,0x40,0x40,0x40};
  static const uint8_t O[5] = {0x3E,0x41,0x41,0x41,0x3E};
  static const uint8_t P[5] = {0x7F,0x09,0x09,0x09,0x06};
  static const uint8_t R[5] = {0x7F,0x09,0x19,0x29,0x46};
  static const uint8_t S[5] = {0x46,0x49,0x49,0x49,0x31};
  static const uint8_t T[5] = {0x01,0x01,0x7F,0x01,0x01};
  static const uint8_t W[5] = {0x3F,0x40,0x38,0x40,0x3F};
  switch (ch) {
    case 'E': return E;  case 'F': return F;  case 'L': return L;
    case 'O': return O;  case 'P': return P;  case 'R': return R;
    case 'S': return S;  case 'T': return T;  case 'W': return W;
    default:  return NULL;
  }
}

/* (x, y) が文字 text の「点灯」ピクセルなら 1 */
static int text_on(const char *text, int x, int y)
{
  int n = (int)strlen(text);
  int width = n * TEXT_CELL - TEXT_SCALE;
  int x0 = ((int)LCD_WIDTH - width) / 2;
  if (y < TEXT_TOP || y >= TEXT_TOP + 7 * TEXT_SCALE || x < x0 || x >= x0 + width) {
    return 0;
  }
  int i   = (x - x0) / TEXT_CELL;
  int col = ((x - x0) % TEXT_CELL) / TEXT_SCALE;      /* 0..5（5 は字間） */
  int row = (y - TEXT_TOP) / TEXT_SCALE;              /* 0..6 */
  const uint8_t *g = glyph(text[i]);
  if (g == NULL || col > 4) {
    return 0;
  }
  return (g[col] >> row) & 1;
}

static uint16_t state_pixel(LcdState s, int x, int y)
{
  const int cx = 120, cy = 110;
  int dx = x - cx, dy = y - cy;
  int adx = dx < 0 ? -dx : dx;
  int ady = dy < 0 ? -dy : dy;

  switch (s) {
  case LCD_STATE_STOP:
    if (text_on("STOP", x, y))    return LCD_WHITE;
    if (adx <= 40 && ady <= 40)   return LCD_WHITE;               /* 白い四角 */
    return LCD_RGB(200, 0, 0);

  case LCD_STATE_FOLLOW:
    if (text_on("FOLLOW", x, y))  return LCD_WHITE;
    if (x >= 80 && x <= 160) {                                    /* 白い右向き三角 */
      int h = (160 - x) * 50 / 80;
      if (ady <= h)               return LCD_WHITE;
    }
    return LCD_RGB(0, 150, 0);

  case LCD_STATE_ERROR:
  default: {
    const uint16_t yellow = LCD_RGB(255, 200, 0);
    if (text_on("ERROR", x, y))   return LCD_BLACK;
    if (y >= 60 && y <= 160 && adx <= (y - 60) * 55 / 100) {      /* 黒い三角 */
      if (adx <= 6 && y >= 95 && y <= 135)   return yellow;       /* 「!」の棒 */
      if (adx <= 7 && y >= 142 && y <= 152)  return yellow;       /* 「!」の点 */
      return LCD_BLACK;
    }
    return yellow;
  }
  }
}

void Lcd_DrawState(LcdState s)
{
  uint8_t row[LCD_WIDTH * 2U];

  lcd_set_window(0, 0, LCD_WIDTH - 1U, LCD_HEIGHT - 1U);
  lcd_dc(1);
  lcd_cs(0);
  for (int y = 0; y < (int)LCD_HEIGHT; y++) {
    for (int x = 0; x < (int)LCD_WIDTH; x++) {
      uint16_t c = state_pixel(s, x, y);
      row[2 * x]     = (uint8_t)(c >> 8);
      row[2 * x + 1] = (uint8_t)c;
    }
    HAL_SPI_Transmit(&hspi2, row, sizeof row, LCD_SPI_TIMEOUT);
  }
  lcd_cs(1);
}
