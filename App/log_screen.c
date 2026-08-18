#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "log_screen.h"
#include "log_phrases.h"
#include "digit_font.h"
#include "lcd_fsmc.h"

#define ROW_H            26     /* 每行高度(px): 短语位图20px高 + 上下留边 */
#define HEARTBEAT_MS     2000   /* 心跳/事件绘制节流间隔 */
#define MAX_PHRASE_W     144    /* 34条短语里最长的"传感器跟随开始/完成"=140px, 留余量 */
#define MAX_PHRASE_H     20
#define EVQ_CAP          8      /* 一次性事件队列深度, 事件很稀疏, 够用 */

static uint16_t g_line_buf[MAX_PHRASE_W * MAX_PHRASE_H];   /* 短语位图临时展开缓冲(复用) */
static uint16_t g_digit_buf[DIGIT_CELL_W * DIGIT_CELL_H];  /* 单个数字位图临时展开缓冲(复用) */

static uint8_t  g_rows;         /* 屏幕能放几行, lcd_height()/ROW_H, init时算 */
static uint8_t  g_row = 0;      /* 下一行要写在第几行 */
static uint32_t g_last_draw = 0;

static log_phrase_id_t g_state = LOGPH_ST_BOOT;

static struct { log_phrase_id_t id; uint8_t is_err; } g_evq[EVQ_CAP];
static uint8_t g_evq_head = 0, g_evq_tail = 0, g_evq_n = 0;

/* ---- 位图展开+搬到屏幕 ---- */
static void draw_phrase(uint16_t x, uint16_t y, log_phrase_id_t id, uint16_t fg)
{
   const log_phrase_dsc_t *d = &log_phrase_dsc[id];
   uint32_t rowBytes = ((uint32_t)d->w + 7u) / 8u;
   uint32_t px, py;

   for (py = 0; py < d->h; py++) {
      for (px = 0; px < d->w; px++) {
         uint8_t byte = log_phrase_bitmap[d->offset + py * rowBytes + (px >> 3)];
         int bit = (byte >> (7 - (px & 7))) & 1;
         g_line_buf[py * d->w + px] = bit ? fg : LCD_COLOR_BLACK;
      }
   }
   lcd_blit(x, y, (uint16_t)(x + d->w - 1), (uint16_t)(y + d->h - 1), g_line_buf);
}

static void draw_digit(uint16_t x, uint16_t y, char ch, uint16_t fg)
{
   int idx = (ch >= '0' && ch <= '9') ? (ch - '0') : 10;   /* 10 = ':' , 排在数字后面 */
   const uint8_t *glyph = &digit_bitmap[(uint32_t)idx * DIGIT_CELL_ROWBYTES * DIGIT_CELL_H];
   uint32_t px, py;

   for (py = 0; py < DIGIT_CELL_H; py++) {
      for (px = 0; px < DIGIT_CELL_W; px++) {
         uint8_t byte = glyph[py * DIGIT_CELL_ROWBYTES + (px >> 3)];
         int bit = (byte >> (7 - (px & 7))) & 1;
         g_digit_buf[py * DIGIT_CELL_W + px] = bit ? fg : LCD_COLOR_BLACK;
      }
   }
   lcd_blit(x, y, (uint16_t)(x + DIGIT_CELL_W - 1), (uint16_t)(y + DIGIT_CELL_H - 1), g_digit_buf);
}

/* 时间戳用开机以来经过时间(HH:MM:SS), 项目没接RTC, 跟uptime同一个时钟源 */
static void draw_timestamp(uint16_t x, uint16_t y, uint16_t fg)
{
   uint32_t s = HAL_GetTick() / 1000;
   char ts[9];
   int i;

   snprintf(ts, sizeof(ts), "%02lu:%02lu:%02lu",
            (unsigned long)((s / 3600) % 100), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
   for (i = 0; i < 8; i++) {
      draw_digit((uint16_t)(x + i * DIGIT_CELL_W), y, ts[i], fg);
   }
}

/* LOGPH_ST_NOLINK/LOGPH_ST_SHUTDOWN是"既非正常也非报警"的第三态提示(未连接EtherCAT/
 * 已下电待安全断电), 按用户要求不带时间戳(这两种状态下"经过了多久"没有意义)、用橙色
 * 跟绿(正常)/红(报警)区分开, 复用同一套"2秒心跳"节流机制, 只是画法特殊处理。*/
static int is_third_state(log_phrase_id_t id)
{
   return id == LOGPH_ST_NOLINK || id == LOGPH_ST_SHUTDOWN;
}

static void draw_row(uint8_t row, log_phrase_id_t id, int is_err)
{
   uint16_t y0 = (uint16_t)(row * ROW_H);
   int third = is_third_state(id);
   uint16_t fg = third ? LCD_COLOR_ORANGE : (is_err ? LCD_COLOR_RED : LCD_COLOR_GREEN);
   uint16_t phrase_x = 4;

   lcd_fill_rect(0, y0, (uint16_t)(lcd_width() - 1), (uint16_t)(y0 + ROW_H - 1), LCD_COLOR_BLACK);
   if (!third) {
      draw_timestamp(4, (uint16_t)(y0 + 4), fg);
      phrase_x = 4 + 8 * DIGIT_CELL_W + 8;
   }
   draw_phrase(phrase_x, (uint16_t)(y0 + 3), id, fg);
}

/* ============================================================ */

void log_screen_init(void)
{
   lcd_init();
   g_rows = (uint8_t)(lcd_height() / ROW_H);
   g_row = 0;
   g_last_draw = HAL_GetTick();
   g_state = LOGPH_ST_BOOT;
   g_evq_head = g_evq_tail = g_evq_n = 0;
}

void log_screen_set_state(log_phrase_id_t state_id)
{
   g_state = state_id;
}

void log_screen_event(log_phrase_id_t id, int is_err)
{
   if (g_evq_n >= EVQ_CAP) return;   /* 队满就丢, 事件很稀疏理论上不会发生 */
   g_evq[g_evq_tail].id = id;
   g_evq[g_evq_tail].is_err = (uint8_t)is_err;
   g_evq_tail = (uint8_t)((g_evq_tail + 1) % EVQ_CAP);
   g_evq_n++;
}

void log_screen_service(void)
{
   uint32_t now = HAL_GetTick();

   if ((uint32_t)(now - g_last_draw) < HEARTBEAT_MS) return;   /* 没到点, 几乎零开销 */
   g_last_draw = now;

   if (g_row >= g_rows) {   /* 铺满一屏, 整体清空重新从顶部开始 */
      lcd_clear(LCD_COLOR_BLACK);
      g_row = 0;
   }

   if (g_evq_n > 0) {   /* 有排队事件优先画, 画掉一条 */
      log_phrase_id_t id = g_evq[g_evq_head].id;
      int is_err = g_evq[g_evq_head].is_err;
      g_evq_head = (uint8_t)((g_evq_head + 1) % EVQ_CAP);
      g_evq_n--;
      draw_row(g_row, id, is_err);
   } else {   /* 没有排队事件, 播报当前心跳状态 */
      draw_row(g_row, g_state, g_state == LOGPH_ST_FAULT);
   }
   g_row++;
}
