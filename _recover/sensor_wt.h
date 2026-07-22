/*
 * 维特 WT901 传感器接入实现 —— USART6 (PC7=RX), 115200 8N1
 * 见 sensor_wt.h 头注释。寄存器级裸写, 风格对齐 modbus_slave.c / uart_log.c。
 *
 * 收帧策略(与 Modbus 从站一致): RXNE 逐字节攒进 rx_buf, USART 行空闲(IDLE)
 * 判定一帧结束 → 拷进 frame_buf 交给主上下文。传感器 ~20Hz(每帧间隔~50ms),
 * 一帧 54B@115200 只需~4.7ms 发完, 帧间大间隙足以让 IDLE 可靠触发。
 */
#include "stm32f4xx_hal.h"      /* GPIO/RCC/NVIC + USART6 寄存器定义 */
#include "sensor_wt.h"

volatile uint32_t sensor_stat_frames = 0;
volatile uint32_t sensor_stat_valid  = 0;

/* ===== 收帧缓冲(ISR 写, 主上下文读) ===== */
#define SW_BUFSZ 128
static volatile uint8_t  rx_buf[SW_BUFSZ];
static volatile uint16_t rx_len = 0;
static volatile uint8_t  frame_buf[SW_BUFSZ];
static volatile uint16_t frame_len = 0;
static volatile uint8_t  frame_ready = 0;   /* 1=frame_buf 有整帧待主上下文解析 */

#define SW_FRAME_LEN  54       /* 12(ID) + 40(载荷) + 2(0D 0A) */

/* ================= 初始化 ================= */
void sensor_init(void)
{
   GPIO_InitTypeDef g = {0};

   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_USART6_CLK_ENABLE();

   /* PC7 = USART6_RX(上拉防悬空), PC6 = USART6_TX; 均复用推挽。
    * TX 用于每周期回传"本轮收没收到传感器"的 0/1 状态字节给电脑。 */
   g.Pin       = GPIO_PIN_7 | GPIO_PIN_6;
   g.Mode      = GPIO_MODE_AF_PP;
   g.Pull      = GPIO_PULLUP;
   g.Speed     = GPIO_SPEED_FREQ_HIGH;
   g.Alternate = GPIO_AF8_USART6;
   HAL_GPIO_Init(GPIOC, &g);

   /* USART6 挂 APB2; BRR = PCLK2 / 波特率 */
   USART6->BRR = HAL_RCC_GetPCLK2Freq() / 115200U;
   USART6->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE
               | USART_CR1_RXNEIE | USART_CR1_IDLEIE;   /* 使能USART + 收 + 发 + 收非空/行空闲中断 */

   (void)USART6->SR; (void)USART6->DR;   /* 清可能的初始 IDLE/RXNE 标志 */

   HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);   /* 与 Modbus 同级, ISR 极短 */
   HAL_NVIC_EnableIRQ(USART6_IRQn);
}

/* ================= 中断: 收字节 + IDLE 判帧尾 ================= */
void sensor_usart6_isr(void)
{
   uint32_t sr = USART6->SR;

   if (sr & USART_SR_RXNE) {
      uint8_t b = (uint8_t)(USART6->DR & 0xFF);   /* 读 DR 清 RXNE */
      if (rx_len < SW_BUFSZ) rx_buf[rx_len++] = b;
      /* 溢出则丢弃, 等下一个 IDLE 重置 */
   }
   if (sr & USART_SR_IDLE) {
      (void)USART6->DR;                 /* 读 SR(上面已读)+读 DR 清 IDLE */
      if (rx_len > 0) {
         sensor_stat_frames++;          /* 一个帧边界 */
         if (!frame_ready) {            /* 上一帧已被主上下文取走才收新帧 */
            for (uint16_t i = 0; i < rx_len; i++) frame_buf[i] = rx_buf[i];
            frame_len   = rx_len;
            frame_ready = 1;
         }
         rx_len = 0;
      }
   }
}

/* ================= 主上下文: 取最新有效帧 ================= */
int sensor_get(float *roll_deg, float *pitch_deg)
{
   if (!frame_ready) return 0;

   uint16_t n = frame_len;
   /* 拷一份局部, 尽快放行 ISR 收下一帧 */
   uint8_t f[SW_FRAME_LEN];
   int ok = (n == SW_FRAME_LEN);
   if (ok) for (uint16_t i = 0; i < SW_FRAME_LEN; i++) f[i] = frame_buf[i];
   frame_ready = 0;
   if (!ok) return 0;                            /* 长度不符(粘帧/断帧), 丢弃 */

   if (f[0] != 'W' || f[1] != 'T') return 0;     /* 设备ID前缀校验 */
   if (f[52] != 0x0D || f[53] != 0x0A) return 0; /* 帧尾校验 */

   const uint8_t *pl = &f[12];                   /* 40字节载荷 */
   uint16_t ver = (uint16_t)pl[38] | ((uint16_t)pl[39] << 8);
   if (ver != SENSOR_FW_VERSION) return 0;       /* 版本号不对 = 字节错位, 丢弃 */

   int16_t r = (int16_t)((uint16_t)pl[22] | ((uint16_t)pl[23] << 8));
   int16_t p = (int16_t)((uint16_t)pl[24] | ((uint16_t)pl[25] << 8));
   *roll_deg  = (float)r * 180.0f / 32768.0f;
   *pitch_deg = (float)p * 180.0f / 32768.0f;
   sensor_stat_valid++;
   return 1;
}

/* ================= 主上下文: 往 PC6(TX) 发一个状态字节 ================= */
/* 阻塞等 TXE。以 ≤1 字节/4ms 的节奏调用时, 上一字节早已发完(单字节@115200≈87us),
 * TXE 恒为置位, 实际不阻塞, 不会撑破控制周期。 */
void sensor_tx_byte(uint8_t b)
{
   while (!(USART6->SR & USART_SR_TXE)) { }
   USART6->DR = b;
}
