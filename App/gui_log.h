/*
 * 日志环形缓冲 —— uart_log()每次格式化完字符串后顺手拷一份进来(纯内存操作,
 * 不阻塞), 供屏幕的"实时日志"页面轮询读取渲染。仿 03-传感器调试/webui调试
 * 面板/webui-win/server.py 的 classify() 关键字分类规则(info/ok/warn/err/data)。
 */
#ifndef _gui_log_h_
#define _gui_log_h_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GUI_LOG_CAP      60    /* 环形缓冲条数 */
#define GUI_LOG_MSG_LEN  80    /* 单条最长字节数(含结尾\0), 超长截断 */

enum {
   GUI_LOG_INFO = 0,
   GUI_LOG_OK,
   GUI_LOG_WARN,
   GUI_LOG_ERR,
   GUI_LOG_DATA,
};

typedef struct {
   uint32_t t_ms;                  /* HAL_GetTick()时刻, 开机以来毫秒数 */
   uint8_t  cls;                   /* GUI_LOG_XXX */
   char     msg[GUI_LOG_MSG_LEN];
} gui_log_entry_t;

/* 格式化好的一行文本存进环形缓冲(内部按关键字分类), uart_log()里调。 */
void gui_log_push(const char *text);

/* 已写入总条数(单调递增, 不是环形缓冲当前条数)。日志页记住上次看到的序号,
 * 每次跟这个数比较, 只渲染新增的那些条(seq到gui_log_seq()-1)。 */
uint32_t gui_log_seq(void);

/* 按全局序号取一条(超出环形缓冲已被覆盖的范围由调用方自己控制不要取)。 */
const gui_log_entry_t *gui_log_at(uint32_t seq);

#ifdef __cplusplus
}
#endif

#endif
