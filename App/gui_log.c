#include <string.h>
#include "stm32f4xx_hal.h"
#include "gui_log.h"

static gui_log_entry_t g_buf[GUI_LOG_CAP];
static volatile uint32_t g_seq = 0;   /* 下一条要写的全局序号(单调递增) */

/* 关键字分类, 照抄webui-win/server.py的classify()规则 */
static uint8_t classify(const char *s)
{
   if (strstr(s, "\xe2\x98\x85") || strstr(s, "完成") || strstr(s, "OK") ||
       strstr(s, "到位") || strstr(s, "使能"))
      return GUI_LOG_OK;
   if (strstr(s, "报警") || strstr(s, "失败") || strstr(s, "ERROR") ||
       strstr(s, "错误") || strstr(s, "!!"))
      return GUI_LOG_ERR;
   if (strstr(s, "警告") || strstr(s, "WARN") || strstr(s, "停止"))
      return GUI_LOG_WARN;
   if (strstr(s, "mm") || strstr(s, "RPM") || strstr(s, "WKC") ||
       strstr(s, "从站") || strstr(s, "Modbus"))
      return GUI_LOG_DATA;
   return GUI_LOG_INFO;
}

void gui_log_push(const char *text)
{
   gui_log_entry_t *e = &g_buf[g_seq % GUI_LOG_CAP];

   e->t_ms = HAL_GetTick();
   e->cls  = classify(text);

   /* 去掉常见行尾\r\n再存, 屏幕上自己换行不需要这些 */
   {
      size_t n = strlen(text);
      while (n > 0 && (text[n - 1] == '\r' || text[n - 1] == '\n')) n--;
      if (n >= GUI_LOG_MSG_LEN) n = GUI_LOG_MSG_LEN - 1;
      memcpy(e->msg, text, n);
      e->msg[n] = '\0';
   }

   g_seq++;
}

uint32_t gui_log_seq(void) { return g_seq; }

const gui_log_entry_t *gui_log_at(uint32_t seq) { return &g_buf[seq % GUI_LOG_CAP]; }
