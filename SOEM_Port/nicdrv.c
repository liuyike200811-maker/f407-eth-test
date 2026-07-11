/*
 * SOEM 网卡驱动 —— STM32F407 裸机版
 * 缓冲索引/状态机(EC_BUF_EMPTY/ALLOC/TX/RCVD/COMPLETE)与官方 linux/rtk 版完全一致,
 * 只把两处平台相关换掉:
 *   1) 收发: send/recv  ->  oshw_mac_send / oshw_mac_recv (见 stm32_eth.c)
 *   2) 加锁: 裸机单线程主循环, 所有 mutex 去掉
 * 不使用冗余(redundant)口。
 */
#include "soem/soem.h"
#include "nicdrv.h"
#include "oshw.h"
#include "osal.h"
#include <string.h>

/* 冗余状态枚举(官方在 linux/nicdrv.c 里是局部定义, 这里补上) */
enum { ECT_RED_NONE, ECT_RED_DOUBLE };

/* EtherCAT 主站源 MAC(仅路由标识, 与真实网卡无关), 取自 ec_options.h */
const uint16 priMAC[3] = EC_PRIMARY_MAC_ARRAY;
const uint16 secMAC[3] = EC_SECONDARY_MAC_ARRAY;

static void ecx_clear_rxbufstat(int *rxbufstat)
{
   int i;
   for (i = 0; i < EC_MAXBUF; i++)
      rxbufstat[i] = EC_BUF_EMPTY;
}

int ecx_setupnic(ecx_portt *port, const char *ifname, int secondary)
{
   int i;
   (void)ifname;

   if (secondary)
      return 0;   /* 本移植不支持冗余口 */

   if (oshw_mac_init((const uint8_t *)priMAC) != 0)
      return 0;   /* 网卡初始化失败 */

   port->getindex_mutex = osal_mutex_create();
   port->tx_mutex       = osal_mutex_create();
   port->rx_mutex       = osal_mutex_create();
   port->sockhandle     = -1;
   port->lastidx        = 0;
   port->redstate       = ECT_RED_NONE;
   port->stack.sock        = &(port->sockhandle);
   port->stack.txbuf       = &(port->txbuf);
   port->stack.txbuflength = &(port->txbuflength);
   port->stack.tempbuf     = &(port->tempinbuf);
   port->stack.rxbuf       = &(port->rxbuf);
   port->stack.rxbufstat   = &(port->rxbufstat);
   port->stack.rxsa        = &(port->rxsa);
   ecx_clear_rxbufstat(&(port->rxbufstat[0]));

   /* 预置各 tx 缓冲的以太网头, 之后不用重复填 */
   for (i = 0; i < EC_MAXBUF; i++)
   {
      ec_setupheader(&(port->txbuf[i]));
      port->rxbufstat[i] = EC_BUF_EMPTY;
   }
   ec_setupheader(&(port->txbuf2));

   return 1;
}

int ecx_closenic(ecx_portt *port)
{
   (void)port;
   /* HAL_ETH 常驻, 无需关闭; 如需可在此停 DMA */
   return 0;
}

void ec_setupheader(void *p)
{
   ec_etherheadert *bp = p;
   bp->da0 = oshw_htons(0xffff);
   bp->da1 = oshw_htons(0xffff);
   bp->da2 = oshw_htons(0xffff);
   bp->sa0 = oshw_htons(priMAC[0]);
   bp->sa1 = oshw_htons(priMAC[1]);
   bp->sa2 = oshw_htons(priMAC[2]);
   bp->etype = oshw_htons(ETH_P_ECAT);
}

uint8 ecx_getindex(ecx_portt *port)
{
   uint8 idx, cnt;

   idx = port->lastidx + 1;
   if (idx >= EC_MAXBUF) idx = 0;
   cnt = 0;
   while ((port->rxbufstat[idx] != EC_BUF_EMPTY) && (cnt < EC_MAXBUF))
   {
      idx++; cnt++;
      if (idx >= EC_MAXBUF) idx = 0;
   }
   port->rxbufstat[idx] = EC_BUF_ALLOC;
   port->lastidx = idx;
   return idx;
}

void ecx_setbufstat(ecx_portt *port, uint8 idx, int bufstat)
{
   port->rxbufstat[idx] = bufstat;
}

int ecx_outframe(ecx_portt *port, uint8 idx, int stacknumber)
{
   int lp, rval;
   ec_stackT *stack = &(port->stack);
   (void)stacknumber;

   lp = (*stack->txbuflength)[idx];
   (*stack->rxbufstat)[idx] = EC_BUF_TX;
   rval = oshw_mac_send((*stack->txbuf)[idx], lp);
   if (rval == -1)
      (*stack->rxbufstat)[idx] = EC_BUF_EMPTY;
   return rval;
}

int ecx_outframe_red(ecx_portt *port, uint8 idx)
{
   ec_etherheadert *ehp;
   ehp = (ec_etherheadert *)&(port->txbuf[idx]);
   ehp->sa1 = oshw_htons(priMAC[1]);   /* 源MAC中字设为主 */
   return ecx_outframe(port, idx, 0);  /* 非冗余: 只发主口 */
}

/* 非阻塞收一帧到 tempbuf, 返回是否收到 */
static int ecx_recvpkt(ecx_portt *port, int stacknumber)
{
   int bytesrx;
   ec_stackT *stack = &(port->stack);
   (void)stacknumber;

   bytesrx = oshw_mac_recv((*stack->tempbuf), sizeof(port->tempinbuf));
   port->tempinbufs = bytesrx;
   return (bytesrx > 0);
}

int ecx_inframe(ecx_portt *port, uint8 idx, int stacknumber)
{
   uint16 l;
   int rval;
   uint8 idxf;
   ec_etherheadert *ehp;
   ec_comt *ecp;
   ec_stackT *stack = &(port->stack);
   ec_bufT *rxbuf;
   (void)stacknumber;

   rval = EC_NOFRAME;
   rxbuf = &(*stack->rxbuf)[idx];
   if ((idx < EC_MAXBUF) && ((*stack->rxbufstat)[idx] == EC_BUF_RCVD))
   {
      l = (*rxbuf)[0] + ((uint16)((*rxbuf)[1] & 0x0f) << 8);
      rval = ((*rxbuf)[l] + ((uint16)(*rxbuf)[l + 1] << 8));
      (*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
   }
   else if (ecx_recvpkt(port, stacknumber))
   {
      rval = EC_OTHERFRAME;
      ehp = (ec_etherheadert *)(stack->tempbuf);
      if (ehp->etype == oshw_htons(ETH_P_ECAT))
      {
         stack->rxcnt++;
         ecp = (ec_comt *)(&(*stack->tempbuf)[ETH_HEADERSIZE]);
         l = etohs(ecp->elength) & 0x0fff;
         idxf = ecp->index;
         if (idxf == idx)
         {
            memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE],
                   (*stack->txbuflength)[idx] - ETH_HEADERSIZE);
            rval = ((*rxbuf)[l] + ((uint16)((*rxbuf)[l + 1]) << 8));
            (*stack->rxbufstat)[idx] = EC_BUF_COMPLETE;
            (*stack->rxsa)[idx] = oshw_ntohs(ehp->sa1);
         }
         else if (idxf < EC_MAXBUF && (*stack->rxbufstat)[idxf] == EC_BUF_TX)
         {
            rxbuf = &(*stack->rxbuf)[idxf];
            memcpy(rxbuf, &(*stack->tempbuf)[ETH_HEADERSIZE],
                   (*stack->txbuflength)[idxf] - ETH_HEADERSIZE);
            (*stack->rxbufstat)[idxf] = EC_BUF_RCVD;
            (*stack->rxsa)[idxf] = oshw_ntohs(ehp->sa1);
         }
      }
   }
   return rval;
}

static int ecx_waitinframe_red(ecx_portt *port, uint8 idx, osal_timert *timer)
{
   int wkc = EC_NOFRAME;
   do
   {
      wkc = ecx_inframe(port, idx, 0);
   } while ((wkc <= EC_NOFRAME) && !osal_timer_is_expired(timer));
   return wkc;
}

int ecx_waitinframe(ecx_portt *port, uint8 idx, int timeout)
{
   osal_timert timer;
   osal_timer_start(&timer, timeout);
   return ecx_waitinframe_red(port, idx, &timer);
}

int ecx_srconfirm(ecx_portt *port, uint8 idx, int timeout)
{
   int wkc = EC_NOFRAME;
   osal_timert timer1, timer2;

   osal_timer_start(&timer1, timeout);
   do
   {
      ecx_outframe_red(port, idx);
      if (timeout < EC_TIMEOUTRET) osal_timer_start(&timer2, timeout);
      else                         osal_timer_start(&timer2, EC_TIMEOUTRET);
      wkc = ecx_waitinframe_red(port, idx, &timer2);
   } while ((wkc <= EC_NOFRAME) && !osal_timer_is_expired(&timer1));

   return wkc;
}
