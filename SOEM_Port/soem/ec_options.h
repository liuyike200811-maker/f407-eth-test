/*
 * SOEM 构建配置 —— STM32F407 裸机移植版
 * 原本由 CMake 从 ec_options.h.in 生成, 这里手工填成具体数值。
 * 针对 192KB RAM 做了收缩: EC_MAXSLAVE 从 200 降到 16, EC_MAX_MAPT=1(单线程,不开映射线程)。
 */
#ifndef _ec_options_
#define _ec_options_

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Max sizes ---- */
#define EC_BUFSIZE            1518   /* 标准以太网帧缓冲字节数 */
#define EC_MAXBUF             8      /* 每通道帧缓冲数(3从站够用, 调小省RAM) */
#define EC_MAXEEPBITMAP       128
#define EC_MAXEEPBUF          (EC_MAXEEPBITMAP * 32)
#define EC_LOGGROUPOFFSET     16
#define EC_MAXELIST           64
#define EC_MAXNAME            40
#define EC_MAXSLAVE           16     /* ⚠ 从站上限: 3个伺服够用, 调小以省RAM */
#define EC_MAXGROUP           2
#define EC_MAXIOSEGMENTS      64
#define EC_MAXMBX             1486
#define EC_MBXPOOLSIZE        32
#define EC_MAXEEPDO           0x200
#define EC_MAXSM              8
#define EC_MAXFMMU            4
#define EC_MAXLEN_ADAPTERNAME 128
#define EC_MAX_MAPT           1      /* ⚠ 裸机单线程: 映射不开子线程 */
#define EC_MAXODLIST          1024
#define EC_MAXOELIST          256
#define EC_SOE_MAXNAME        60
#define EC_SOE_MAXMAPPING     64

/* ---- Timeouts (us) 与重试, 用 SOEM 官方默认值 ---- */
#define EC_TIMEOUTRET         2000
#define EC_TIMEOUTRET3        (EC_TIMEOUTRET * 3)
#define EC_TIMEOUTSAFE        20000
#define EC_TIMEOUTEEP         20000
#define EC_TIMEOUTTXM         20000
#define EC_TIMEOUTRXM         700000
#define EC_TIMEOUTSTATE       2000000
#define EC_DEFAULTRETRIES     3

/* ---- EtherCAT 主站源 MAC(仅用于冗余路由标识, 与真实网卡MAC无关) ---- */
#define EC_PRIMARY_MAC_ARRAY   { 0x0101, 0x0101, 0x0101 }
#define EC_SECONDARY_MAC_ARRAY { 0x0404, 0x0404, 0x0404 }

#ifdef __cplusplus
}
#endif

#endif
