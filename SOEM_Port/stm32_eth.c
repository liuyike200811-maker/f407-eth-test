/*
 * STM32F407 以太网裸帧收发层 (给 SOEM 的 oshw_mac_init/send/recv)
 *
 * 本文件直接照搬工程里 LWIP/Target/ethernetif.c 的初始化时序 —— 那套在本板子上
 * 已用 ping 验证通过 —— 只做三处改动以适配 EtherCAT:
 *   1) 把 LwIP 的 pbuf 收发换成普通字节缓冲(拷贝进出)
 *   2) TX 关掉 IP 校验和卸载(EtherCAT 不是 IP 帧, 开了会破坏帧), 只保留硬件 CRC/PAD
 *   3) 打开 MAC 混杂模式, 确保收得到环回的 EtherCAT 帧
 *
 * ⚠ 前置条件: 必须把工程里的 LWIP 整个从编译中移除(见交付说明),
 *   否则 heth / DMARxDscrTab / HAL_ETH_MspInit / RxAllocateCallback 等会与
 *   ethernetif.c 重复定义, 链接冲突。
 *
 * ⚠ 本文件是整套移植里最需要在硬件上验证的一块。
 */
#include "stm32f4xx_hal.h"
#include "stm32_eth.h"
#include "lan8742.h"
#include <string.h>

/* ---------------- 句柄 / 描述符 / 缓冲 ---------------- */
ETH_HandleTypeDef  heth;
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT];
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT];
static ETH_TxPacketConfig TxConfig;

/* PHY 驱动对象 (LAN8742 寄存器兼容 LAN8720) */
static lan8742_Object_t LAN8742;

int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit(void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);

static lan8742_IOCtx_t LAN8742_IOCtx = {
   ETH_PHY_IO_Init, ETH_PHY_IO_DeInit,
   ETH_PHY_IO_WriteReg, ETH_PHY_IO_ReadReg, ETH_PHY_IO_GetTick
};

/* ---------------- RX 缓冲池 (替代 LwIP pbuf) ---------------- */
#define RX_POOL_CNT 8
static uint8_t  rx_pool[RX_POOL_CNT][ETH_RX_BUF_SIZE] __attribute__((aligned(4)));
static uint8_t  rx_used[RX_POOL_CNT];
static uint8_t *g_rx_first;   /* 本次收到的帧缓冲 */
static uint32_t g_rx_len;     /* 本次收到的帧长度 */

static uint8_t *rx_pool_alloc(void)
{
   for (int i = 0; i < RX_POOL_CNT; i++)
      if (!rx_used[i]) { rx_used[i] = 1; return rx_pool[i]; }
   return NULL;
}
static void rx_pool_free(uint8_t *p)
{
   for (int i = 0; i < RX_POOL_CNT; i++)
      if (rx_pool[i] == p) { rx_used[i] = 0; return; }
}

/* ---------------- HAL 回调: 给/收 RX 缓冲 ---------------- */
void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
   *buff = rx_pool_alloc();   /* 没有空闲则 NULL, HAL 会丢弃该帧 */
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
   (void)pEnd;
   /* EtherCAT 帧 < 1518 < ETH_RX_BUF_SIZE, 单缓冲即可 */
   *pStart   = buff;
   g_rx_first = buff;
   g_rx_len   = Length;
}

/* ---------------- PHY MDIO IO (与 ethernetif.c 一致) ---------------- */
int32_t ETH_PHY_IO_Init(void)   { HAL_ETH_SetMDIOClockRange(&heth); return 0; }
int32_t ETH_PHY_IO_DeInit(void) { return 0; }
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{ return (HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) == HAL_OK) ? 0 : -1; }
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{ return (HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) == HAL_OK) ? 0 : -1; }
int32_t ETH_PHY_IO_GetTick(void) { return HAL_GetTick(); }

/* ---------------- ETH MSP: RMII 引脚 + 时钟 (照搬 ethernetif.c) ---------------- */
void HAL_ETH_MspInit(ETH_HandleTypeDef *ethHandle)
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   if (ethHandle->Instance == ETH)
   {
      __HAL_RCC_ETH_CLK_ENABLE();
      __HAL_RCC_GPIOC_CLK_ENABLE();
      __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_GPIOG_CLK_ENABLE();
      /* PC1=MDC PC4=RXD0 PC5=RXD1 */
      GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
      /* PA1=REF_CLK PA2=MDIO PA7=CRS_DV */
      GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      /* PG11=TX_EN PG13=TXD0 PG14=TXD1 */
      GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
      HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
   }
}

/* ==================================================================
 *  oshw_mac_* : SOEM 网卡三接口
 * ================================================================== */
int oshw_mac_init(const uint8_t *mac_address)
{
   static uint8_t mac[6];
   int32_t phystate;
   uint32_t speed = ETH_SPEED_100M, duplex = ETH_FULLDUPLEX_MODE;
   ETH_MACConfigTypeDef       MACConf = {0};
   ETH_MACFilterConfigTypeDef Filter  = {0};
   uint32_t t0;

   memcpy(mac, mac_address, 6);

   heth.Instance          = ETH;
   heth.Init.MACAddr      = mac;
   heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
   heth.Init.TxDesc       = DMATxDscrTab;
   heth.Init.RxDesc       = DMARxDscrTab;
   heth.Init.RxBuffLen    = ETH_RX_BUF_SIZE;
   if (HAL_ETH_Init(&heth) != HAL_OK)
      return -1;

   /* TX 配置: EtherCAT 不是 IP 帧 —— 只开硬件 CRC/PAD, 关掉 IP 校验和卸载 */
   memset(&TxConfig, 0, sizeof(TxConfig));
   TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CRCPAD;
   TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

   /* PHY 初始化 + 协商 */
   LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
   if (LAN8742_Init(&LAN8742) != LAN8742_STATUS_OK)
      return -1;

   /* 等链路 up (最多 ~5s), 读协商结果 */
   t0 = HAL_GetTick();
   do {
      phystate = LAN8742_GetLinkState(&LAN8742);
      if (phystate > LAN8742_STATUS_LINK_DOWN) break;
   } while ((HAL_GetTick() - t0) < 5000);

   switch (phystate) {
   case LAN8742_STATUS_100MBITS_FULLDUPLEX: speed=ETH_SPEED_100M; duplex=ETH_FULLDUPLEX_MODE; break;
   case LAN8742_STATUS_100MBITS_HALFDUPLEX: speed=ETH_SPEED_100M; duplex=ETH_HALFDUPLEX_MODE; break;
   case LAN8742_STATUS_10MBITS_FULLDUPLEX:  speed=ETH_SPEED_10M;  duplex=ETH_FULLDUPLEX_MODE; break;
   case LAN8742_STATUS_10MBITS_HALFDUPLEX:  speed=ETH_SPEED_10M;  duplex=ETH_HALFDUPLEX_MODE; break;
   default: /* 没协商上, EtherCAT 默认按 100M 全双工 */
      speed = ETH_SPEED_100M; duplex = ETH_FULLDUPLEX_MODE; break;
   }
   HAL_ETH_GetMACConfig(&heth, &MACConf);
   MACConf.DuplexMode = duplex;
   MACConf.Speed      = speed;
   HAL_ETH_SetMACConfig(&heth, &MACConf);

   /* 混杂模式: 收下所有帧(EtherCAT 回环帧目的MAC是广播, 保险起见全收) */
   HAL_ETH_GetMACFilterConfig(&heth, &Filter);
   Filter.PromiscuousMode = ENABLE;
   HAL_ETH_SetMACFilterConfig(&heth, &Filter);

   HAL_ETH_Start(&heth);
   return 0;
}

/* 发一帧(阻塞). 返回发送字节数, 失败 -1 */
int oshw_mac_send(const void *payload, size_t tot_len)
{
   static uint8_t txbuf[1536] __attribute__((aligned(4)));   /* ETH 最大帧 */
   ETH_BufferTypeDef txseg;

   if (tot_len > sizeof(txbuf)) return -1;
   memcpy(txbuf, payload, tot_len);

   memset(&txseg, 0, sizeof(txseg));
   txseg.buffer = txbuf;
   txseg.len    = (uint32_t)tot_len;
   txseg.next   = NULL;

   TxConfig.Length   = (uint32_t)tot_len;
   TxConfig.TxBuffer = &txseg;
   TxConfig.pData    = NULL;

   if (HAL_ETH_Transmit(&heth, &TxConfig, 20) != HAL_OK)
      return -1;
   return (int)tot_len;
}

/* 非阻塞收一帧. 有帧则拷进 buffer 返回长度, 没有返回 0 */
int oshw_mac_recv(void *buffer, size_t buffer_length)
{
   void *pdata = NULL;
   uint32_t n;

   g_rx_first = NULL;
   g_rx_len   = 0;
   if (HAL_ETH_ReadData(&heth, &pdata) != HAL_OK || g_rx_first == NULL)
      return 0;

   n = g_rx_len;
   if (n > buffer_length) n = buffer_length;
   memcpy(buffer, g_rx_first, n);
   rx_pool_free(g_rx_first);   /* 拷完释放, 供下次接收复用 */
   return (int)n;
}
