/*
 * SOEM OSAL 平台定义 —— STM32F407 裸机版
 * 参照 osal/rtk/osal_defs.h 改写:
 *  - ec_timet 用 struct timespec (SOEM 的 ec_dc.c 会访问 .tv_sec/.tv_nsec, 必须是它)
 *  - 裸机单线程: 线程句柄/互斥量都用占位类型, osal.c 里做成空操作
 */
#ifndef _osal_defs_
#define _osal_defs_

#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>   /* newlib 提供 struct timespec */

/* 需要调试打印时定义 EC_DEBUG */
#ifdef EC_DEBUG
#include <stdio.h>
#define EC_PRINT printf
#else
#define EC_PRINT(...) do { } while (0)
#endif

#ifndef OSAL_PACKED
#define OSAL_PACKED_BEGIN
#define OSAL_PACKED __attribute__((__packed__))
#define OSAL_PACKED_END
#endif

#define ec_timet            struct timespec

/* 裸机: 无 RTOS 线程/互斥, 用占位类型, 相关 osal_* 函数做成空实现 */
#define OSAL_THREAD_HANDLE  void *
#define OSAL_THREAD_FUNC    void
#define OSAL_THREAD_FUNC_RT void
#define osal_mutext         void *

#ifdef __cplusplus
}
#endif

#endif
