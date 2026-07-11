/*
 * SOEM OSAL 实现 —— STM32F407 裸机版
 * 参照 osal/rtk/osal.c 改写:
 *  - 时间基准: Cortex-M4 的 DWT->CYCCNT 周期计数器 (168MHz), 换算出 64位微秒
 *  - 线程/互斥: 裸机单线程主循环, 全部做成空操作
 *  - malloc/free: 直接用 newlib 堆 (注意 Makefile 里 _sbrk 的堆要够大)
 */
#include "osal.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>

/* ---------------- 64位微秒时间基准 (DWT CYCCNT) ---------------- */
static int      s_inited   = 0;
static uint32_t s_cyc_per_us = 168;   /* SystemCoreClock/1e6, 168MHz */
static uint32_t s_last_cyc = 0;
static uint64_t s_acc_cyc  = 0;

static void osal_time_init(void)
{
   CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 使能跟踪 */
   DWT->CYCCNT = 0;
   DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             /* 开周期计数器 */
   s_cyc_per_us = SystemCoreClock / 1000000u;
   if (s_cyc_per_us == 0) s_cyc_per_us = 168;
   s_last_cyc = DWT->CYCCNT;
   s_acc_cyc  = 0;
   s_inited   = 1;
}

/* 返回单调递增的 64位微秒. 必须被足够频繁调用(<25s一次)以正确跨越CYCCNT溢出,
 * EtherCAT 主循环高频调用, 满足要求. 单线程使用, 不做加锁. */
static uint64_t osal_us64(void)
{
   uint32_t now;
   if (!s_inited) osal_time_init();
   now = DWT->CYCCNT;
   s_acc_cyc += (uint32_t)(now - s_last_cyc);   /* 无符号相减自动处理一次溢出 */
   s_last_cyc = now;
   return s_acc_cyc / s_cyc_per_us;
}

static void osal_ts_from_us(uint64_t us, ec_timet *ts)
{
   ts->tv_sec  = (time_t)(us / 1000000ULL);
   ts->tv_nsec = (long)((us % 1000000ULL) * 1000ULL);
}

static uint64_t osal_us_from_ts(const ec_timet *ts)
{
   return (uint64_t)ts->tv_sec * 1000000ULL + (uint64_t)ts->tv_nsec / 1000ULL;
}

/* ---------------- osal.h 要求的接口 ---------------- */
void osal_get_monotonic_time(ec_timet *ts)
{
   osal_ts_from_us(osal_us64(), ts);
}

ec_timet osal_current_time(void)
{
   /* 裸机无实时钟, 用单调时间代替. DC 用的是相对时间, 最简版本够用 */
   ec_timet ts;
   osal_ts_from_us(osal_us64(), &ts);
   return ts;
}

void osal_time_diff(ec_timet *start, ec_timet *end, ec_timet *diff)
{
   uint64_t d = osal_us_from_ts(end) - osal_us_from_ts(start);
   osal_ts_from_us(d, diff);
}

void osal_timer_start(osal_timert *self, uint32 timeout_usec)
{
   osal_ts_from_us(osal_us64() + (uint64_t)timeout_usec, &self->stop_time);
}

boolean osal_timer_is_expired(osal_timert *self)
{
   return (osal_us64() >= osal_us_from_ts(&self->stop_time)) ? TRUE : FALSE;
}

int osal_usleep(uint32 usec)
{
   uint64_t deadline = osal_us64() + (uint64_t)usec;
   while (osal_us64() < deadline) { /* 忙等 */ }
   return 0;
}

int osal_monotonic_sleep(ec_timet *ts)
{
   uint64_t deadline = osal_us_from_ts(ts);
   while (osal_us64() < deadline) { /* 忙等到绝对时刻 */ }
   return 0;
}

void *osal_malloc(size_t size) { return malloc(size); }
void  osal_free(void *ptr)     { free(ptr); }

/* 裸机单线程: 不真正创建线程. EtherCAT 用 ecx_ 系列API在主循环里同步跑,
 * 只有 EC_MAX_MAPT>1 时才会调用这里, 我们设了 EC_MAX_MAPT=1, 不会走到. */
int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
   (void)thandle; (void)stacksize; (void)func; (void)param;
   return 0;   /* 返回失败, SOEM 会退回单线程路径 */
}
int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
   (void)thandle; (void)stacksize; (void)func; (void)param;
   return 0;
}

/* 裸机单线程: 互斥量空操作 */
void *osal_mutex_create(void)          { return (void *)1; }  /* 非NULL即可 */
void  osal_mutex_destroy(void *mutex)  { (void)mutex; }
void  osal_mutex_lock(void *mutex)     { (void)mutex; }
void  osal_mutex_unlock(void *mutex)   { (void)mutex; }
