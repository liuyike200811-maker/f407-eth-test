/*
 * 屏显日志终端 —— 纯展示屏, 不做触摸/控制, 只显示日志。
 * ================================================================
 * 黑底+绿字(正常)+红字(异常), 按行滚动展示, 铺满一屏后整体清空重新
 * 从顶部开始(不做逐行平滑滚动/不做控件框架, 每次只重画一行)。
 *
 * 两类文案(见 log_phrases.h 的 LOGPH_ST_ 系列和 LOGPH_EV_ 系列枚举):
 *   - "心跳类"(ST_*): 表示当前持续状态(启动中/上升中/运行中/下降中/归零中/
 *     待机中/故障中), 用 log_screen_set_state() 切换, 每约2秒自动重复播报
 *     一次——这是"证明程序没死"的关键, 只要屏幕还在动就说明没卡死。
 *   - "事件类"(EV_*): 一次性状态切换/异常标记(模式开始/完成/标定/故障等),
 *     用 log_screen_event() 排队, 排到即画, 画完不重复。
 *
 * ⚠ 时序安全: log_screen_service()内部做了2秒节流(没到点直接返回, 几个CPU
 * 周期), 真正的画一行操作(≈1~2ms总线时间, 已用FSMC总线时序算过)只在到点
 * 时才发生, 且每次最多画一行——因为有这层节流+每次工作量有界, 这次可以
 * 放心地把log_screen_service()也塞进run_rehab_mode/run_sensor_mode这些
 * 运动闭环循环里(为了能报"上升中/运行中/下降中"), 不会像之前LVGL那次一样
 * 在使能后制造无节流的大块阻塞——请求方只管调, 什么时候真正画由这个函数
 * 自己控制, 调用方不用关心节流逻辑。
 */
#ifndef _log_screen_h_
#define _log_screen_h_

#include "log_phrases.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化屏幕(内部调lcd_init()), 上电后调一次。 */
void log_screen_init(void);

/* 切换当前持续状态(只传LOGPH_ST_*), 之后每约2秒自动重复播报, 直到再次切换。 */
void log_screen_set_state(log_phrase_id_t state_id);

/* 排队一条一次性事件(只传LOGPH_EV_*), is_err非0画红字, 否则画绿字。 */
void log_screen_event(log_phrase_id_t id, int is_err);

/* 每个循环周期都可以调(内部2秒节流, 没到点几乎零开销)。 */
void log_screen_service(void);

#ifdef __cplusplus
}
#endif

#endif
