/*
 * Modbus RTU 从站实现 —— USART3 (PB10=TX, PB11=RX), 115200 8N1, 站号1
 * 见 modbus_slave.h 头注释。寄存器级裸写, 风格对齐 uart_log.c。
 */
#include "stm32f4xx_hal.h"      /* GPIO/RCC/NVIC + USART3 寄存器定义 */
#include "modbus_slave.h"
#include <string.h>

/* ===== 契约寄存器区(主上下文读写) ===== */
volatile uint8_t  modbus_coils[MODBUS_NUM_COILS];
volatile uint16_t modbus_hreg[MODBUS_NUM_HREGS];
volatile uint32_t modbus_stat_frames = 0;
volatile uint32_t modbus_stat_valid  = 0;

/* ===== 收帧缓冲(ISR 写, 主上下文读) ===== */
#define MB_BUFSZ 256
static volatile uint8_t  rx_buf[MB_BUFSZ];
static volatile uint16_t rx_len = 0;
static volatile uint8_t  frame_buf[MB_BUFSZ];
static volatile uint16_t frame_len = 0;
static volatile uint8_t  frame_ready = 0;   /* 1=frame_buf 有整帧待主上下文处理 */

/* ================= 初始化 ================= */
void modbus_init(void)
{
   GPIO_InitTypeDef g = {0};

   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_USART3_CLK_ENABLE();

   /* PB10 = USART3_TX, PB11 = USART3_RX, 复用推挽; RX 上拉防悬空误触发 */
   g.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
   g.Mode      = GPIO_MODE_AF_PP;
   g.Pull      = GPIO_PULLUP;
   g.Speed     = GPIO_SPEED_FREQ_HIGH;
   g.Alternate = GPIO_AF7_USART3;
   HAL_GPIO_Init(GPIOB, &g);

   /* USART3 挂 APB1; BRR = PCLK1 / 波特率 */
   USART3->BRR = HAL_RCC_GetPCLK1Freq() / 115200U;
   USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE
               | USART_CR1_RXNEIE | USART_CR1_IDLEIE;   /* 收非空 + 行空闲 中断 */

   (void)USART3->SR; (void)USART3->DR;   /* 清可能的初始 IDLE/RXNE 标志 */

   HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);   /* 低于关键中断, ISR 极短 */
   HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* ================= 中断: 收字节 + IDLE 判帧尾 ================= */
void modbus_usart3_isr(void)
{
   uint32_t sr = USART3->SR;

   if (sr & USART_SR_RXNE) {
      uint8_t b = (uint8_t)(USART3->DR & 0xFF);   /* 读 DR 清 RXNE */
      if (rx_len < MB_BUFSZ) rx_buf[rx_len++] = b;
      /* 溢出则丢弃, 等下一个 IDLE 重置 */
   }
   if (sr & USART_SR_IDLE) {
      (void)USART3->DR;                 /* 读 SR(上面已读)+读 DR 清 IDLE */
      if (rx_len > 0) {
         modbus_stat_frames++;          /* 一个帧边界: RX 上确实有字节到了 */
         if (!frame_ready) {            /* 上一帧已被主上下文取走才收新帧 */
            for (uint16_t i = 0; i < rx_len; i++) frame_buf[i] = rx_buf[i];
            frame_len   = rx_len;
            frame_ready = 1;
         }
         rx_len = 0;
      }
   }
}

/* ================= 工具 ================= */
static uint16_t mb_crc16(const uint8_t *buf, uint16_t len)
{
   uint16_t crc = 0xFFFF;
   for (uint16_t i = 0; i < len; i++) {
      crc ^= buf[i];
      for (int j = 0; j < 8; j++)
         crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
   }
   return crc;
}

#define RD16(p)  ((uint16_t)(((uint16_t)(p)[0] << 8) | (p)[1]))

static void mb_send(uint8_t *resp, uint16_t n)
{
   uint16_t crc = mb_crc16(resp, n);
   resp[n]     = (uint8_t)(crc & 0xFF);   /* CRC 低字节在前 */
   resp[n + 1] = (uint8_t)(crc >> 8);
   n += 2;
   for (uint16_t i = 0; i < n; i++) {
      while (!(USART3->SR & USART_SR_TXE)) { }
      USART3->DR = resp[i];
   }
   while (!(USART3->SR & USART_SR_TC)) { }   /* 等最后一字节移位完再释放 */
}

/* ================= 回环自测(联调用, 平时保持关闭) =================
 * 打开下面那行 #define, 重新编译烧录后, 板子每约1秒往 USART3 发一小串测试字节。
 * 操作: 把【模块 DB9 的 Pin2 和 Pin3 短接】(HMI 那头先拔掉), 发出去的字节会
 *       经 MAX3232 绕回 PB11, 于是日志里的 "Modbus 收到帧=X" 会每秒 +1 左右:
 *    · X 开始涨  -> STM32 USART3 + MAX3232 + 模块DB9 整段都是好的, 病根在 HMI 那侧
 *    · X 仍为 0  -> 病根在板子这侧(模块坏 / DB9脚位不对 / 固件TX没真发出去)
 * 测完务必把下面这行重新注释掉, 再烧回正常固件(否则会一直自发自收干扰联调)。 */
/* #define MB_LOOPBACK_TEST 1 */

void modbus_loopback_selftest_tx(void)
{
#ifdef MB_LOOPBACK_TEST
   /* 0xAA 不是站号1, 环回收到后只会让"收到帧"+1, 不会被当成请求去应答 */
   static const uint8_t pat[4] = { 0xAA, 0x55, 0xAA, 0x55 };
   for (uint16_t i = 0; i < sizeof(pat); i++) {
      while (!(USART3->SR & USART_SR_TXE)) { }
      USART3->DR = pat[i];
   }
   while (!(USART3->SR & USART_SR_TC)) { }   /* 等发完, 让线空闲后触发 IDLE 判帧 */
#endif
}

static void mb_exception(uint8_t fc, uint8_t code)
{
   uint8_t r[4];
   r[0] = MODBUS_SLAVE_ID;
   r[1] = fc | 0x80;
   r[2] = code;              /* 01=非法功能 02=非法地址 */
   mb_send(r, 3);
}

/* ================= 主上下文: 解析一帧并应答 ================= */
void modbus_poll(void)
{
   uint8_t  f[MB_BUFSZ];
   uint8_t  resp[MB_BUFSZ];
   uint16_t len;

   if (!frame_ready) return;

   len = frame_len;
   for (uint16_t i = 0; i < len; i++) f[i] = frame_buf[i];   /* 快照, 尽早处理 */

   /* 最短帧: 站号+功能+2字节CRC */
   if (len < 4)                       goto done;
   if (f[0] != MODBUS_SLAVE_ID)       goto done;   /* 非本站(含广播0) 一律不理 */
   if (mb_crc16(f, len - 2) != (uint16_t)(f[len - 2] | (f[len - 1] << 8)))
      goto done;                                   /* CRC 错, 丢弃 */

   modbus_stat_valid++;                             /* 站号+CRC 都对: 给本站的合法请求 */
   uint8_t  fc = f[1];
   uint16_t start, qty, addr, val;

   switch (fc) {
   case 0x01:   /* 读线圈 */
   case 0x02:   /* 读离散量输入(本机无独立离散量, 复用线圈只读视图) */
      start = RD16(&f[2]); qty = RD16(&f[4]);
      if (qty < 1 || qty > 2000 || start + qty > MODBUS_NUM_COILS) { mb_exception(fc, 0x02); goto done; }
      {
         uint8_t bc = (uint8_t)((qty + 7) / 8);
         resp[0] = MODBUS_SLAVE_ID; resp[1] = fc; resp[2] = bc;
         for (uint8_t i = 0; i < bc; i++) resp[3 + i] = 0;
         for (uint16_t i = 0; i < qty; i++)
            if (modbus_coils[start + i]) resp[3 + (i >> 3)] |= (uint8_t)(1 << (i & 7));
         mb_send(resp, 3 + bc);
      }
      break;

   case 0x03:   /* 读保持寄存器 */
      start = RD16(&f[2]); qty = RD16(&f[4]);
      if (qty < 1 || qty > 125 || start + qty > MODBUS_NUM_HREGS) { mb_exception(fc, 0x02); goto done; }
      {
         uint8_t bc = (uint8_t)(qty * 2);
         resp[0] = MODBUS_SLAVE_ID; resp[1] = fc; resp[2] = bc;
         for (uint16_t i = 0; i < qty; i++) {
            uint16_t v = modbus_hreg[start + i];
            resp[3 + i * 2]     = (uint8_t)(v >> 8);
            resp[3 + i * 2 + 1] = (uint8_t)(v & 0xFF);
         }
         mb_send(resp, 3 + bc);
      }
      break;

   case 0x05:   /* 写单个线圈 (0xFF00=ON, 0x0000=OFF) */
      addr = RD16(&f[2]); val = RD16(&f[4]);
      if (addr >= MODBUS_NUM_COILS) { mb_exception(fc, 0x02); goto done; }
      modbus_coils[addr] = (val == 0xFF00) ? 1 : 0;
      memcpy(resp, f, 6); mb_send(resp, 6);        /* 原样回显 */
      break;

   case 0x06:   /* 写单个保持寄存器 */
      addr = RD16(&f[2]); val = RD16(&f[4]);
      if (addr >= MODBUS_NUM_HREGS) { mb_exception(fc, 0x02); goto done; }
      modbus_hreg[addr] = val;
      memcpy(resp, f, 6); mb_send(resp, 6);
      break;

   case 0x0F:   /* 写多个线圈 */
      start = RD16(&f[2]); qty = RD16(&f[4]);
      if (qty < 1 || start + qty > MODBUS_NUM_COILS) { mb_exception(fc, 0x02); goto done; }
      for (uint16_t i = 0; i < qty; i++)
         modbus_coils[start + i] = (uint8_t)((f[7 + (i >> 3)] >> (i & 7)) & 1);
      memcpy(resp, f, 6); mb_send(resp, 6);        /* 回 站号+功能+起址+数量 */
      break;

   case 0x10:   /* 写多个保持寄存器 */
      start = RD16(&f[2]); qty = RD16(&f[4]);
      if (qty < 1 || start + qty > MODBUS_NUM_HREGS) { mb_exception(fc, 0x02); goto done; }
      for (uint16_t i = 0; i < qty; i++)
         modbus_hreg[start + i] = (uint16_t)((f[7 + i * 2] << 8) | f[7 + i * 2 + 1]);
      memcpy(resp, f, 6); mb_send(resp, 6);
      break;

   default:
      mb_exception(fc, 0x01);   /* 非法功能码 */
      break;
   }

done:
   frame_ready = 0;   /* 末尾清, 释放 ISR 接收下一帧(不能提前清, 否则解析中被覆盖) */
}
