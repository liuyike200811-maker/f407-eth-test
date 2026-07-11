/*
 * SOEM 网卡驱动头 —— STM32F407 裸机版
 * 参照 oshw/rtk/nicdrv.h, 去掉 rt-kernel 的 mtx_t*, 改成 void*(空互斥)。
 * 缓冲/栈结构与官方完全一致, 保证与 SOEM 核心(ethercatbase.c)对得上。
 */
#ifndef _nicdrvh_
#define _nicdrvh_

#ifdef __cplusplus
extern "C" {
#endif

/** Tx/Rx 栈指针结构 */
typedef struct
{
   int *sock;
   ec_bufT (*txbuf)[EC_MAXBUF];
   int (*txbuflength)[EC_MAXBUF];
   ec_bufT *tempbuf;
   ec_bufT (*rxbuf)[EC_MAXBUF];
   int (*rxbufstat)[EC_MAXBUF];
   int (*rxsa)[EC_MAXBUF];
   uint64 rxcnt;
} ec_stackT;

/** 冗余端口缓冲结构(本移植不用冗余, 保留以兼容结构) */
typedef struct
{
   ec_stackT stack;
   int sockhandle;
   ec_bufT rxbuf[EC_MAXBUF];
   int rxbufstat[EC_MAXBUF];
   int rxsa[EC_MAXBUF];
   ec_bufT tempinbuf;
} ecx_redportt;

/** 端口实例: 缓冲/变量/互斥 */
typedef struct
{
   ec_stackT stack;
   int sockhandle;
   ec_bufT rxbuf[EC_MAXBUF];
   int rxbufstat[EC_MAXBUF];
   int rxsa[EC_MAXBUF];
   ec_bufT tempinbuf;
   int tempinbufs;
   ec_bufT txbuf[EC_MAXBUF];
   int txbuflength[EC_MAXBUF];
   ec_bufT txbuf2;
   int txbuflength2;
   uint8 lastidx;
   int redstate;
   ecx_redportt *redport;
   void *getindex_mutex;   /* 裸机: 空互斥 */
   void *tx_mutex;
   void *rx_mutex;
} ecx_portt;

extern const uint16 priMAC[3];
extern const uint16 secMAC[3];

void ec_setupheader(void *p);
int  ecx_setupnic(ecx_portt *port, const char *ifname, int secondary);
int  ecx_closenic(ecx_portt *port);
void ecx_setbufstat(ecx_portt *port, uint8 idx, int bufstat);
uint8 ecx_getindex(ecx_portt *port);
int  ecx_outframe(ecx_portt *port, uint8 idx, int sock);
int  ecx_outframe_red(ecx_portt *port, uint8 idx);
int  ecx_waitinframe(ecx_portt *port, uint8 idx, int timeout);
int  ecx_srconfirm(ecx_portt *port, uint8 idx, int timeout);

#ifdef __cplusplus
}
#endif

#endif
