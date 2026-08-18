/*
 * 预渲染日志短语位图(1bpp) —— 自动生成, 不要手改。
 * 生成方式: Node.js + node-canvas, 用系统"Microsoft YaHei"字体把下面这些固定
 * 短语渲染成图再转成1位位图(前景/背景, 颜色在draw时按状态决定绿/红), 20px字高。
 * 覆盖范围只有这份枚举里列出的固定短语——以后新增文案要重新跑生成脚本
 * (脚本+生成结果留档在 06-STM32屏显/, 不是这次固件仓库里)。
 */
#ifndef _log_phrases_h_
#define _log_phrases_h_
#include <stdint.h>

typedef enum {
   LOGPH_ST_BOOT,
   LOGPH_ST_RISE,
   LOGPH_ST_RUN,
   LOGPH_ST_FALL,
   LOGPH_ST_HOMING,
   LOGPH_ST_IDLE,
   LOGPH_ST_FAULT,
   LOGPH_EV_SCAN0,
   LOGPH_EV_SCAN1,
   LOGPH_EV_SCAN2,
   LOGPH_EV_SCAN3,
   LOGPH_EV_BUS_OK,
   LOGPH_EV_DRV_OK,
   LOGPH_EV_M0_START,
   LOGPH_EV_M0_DONE,
   LOGPH_EV_M1_START,
   LOGPH_EV_M1_DONE,
   LOGPH_EV_M2_START,
   LOGPH_EV_M2_DONE,
   LOGPH_EV_M3_START,
   LOGPH_EV_M3_DONE,
   LOGPH_EV_M4_START,
   LOGPH_EV_M4_DONE,
   LOGPH_EV_HM_START,
   LOGPH_EV_HM_DONE,
   LOGPH_EV_SF_START,
   LOGPH_EV_SF_DONE,
   LOGPH_EV_CAL_ING,
   LOGPH_EV_CAL_OK,
   LOGPH_EV_CAL_FAIL,
   LOGPH_EV_SLV1,
   LOGPH_EV_SLV2,
   LOGPH_EV_SLV3,
   LOGPH_EV_ESTOP,
   LOGPH_EV_SIGLOST,
   LOGPH_EV_HM_TO,
   LOGPH_ST_NOLINK,     /* 未连接: EtherCAT扫不到网卡/从站、以及下电退出后的终态心跳共用, 无时间戳+橙色 */
   LOGPH_COUNT
} log_phrase_id_t;

typedef struct { log_phrase_id_t id; uint8_t w; uint8_t h; uint16_t offset; } log_phrase_dsc_t;

extern const uint8_t log_phrase_bitmap[];
extern const log_phrase_dsc_t log_phrase_dsc[LOGPH_COUNT];

#endif
