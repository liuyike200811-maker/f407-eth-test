/*
 * 维特 WT901 传感器接入实现 —— USART6 (PC7=RX), 115200 8N1
 * 见 sensor_wt.h 头注释。寄存器级裸写, 风格对齐 modbus_slave.c / uart_log.c。
 *
 * ============================ 安全重写说明 ============================
 * 本模块只负责把"一手串口字节流"变成【可信的姿态角】, 是驱动动平台的第一道闸门。
 * 原实现只校验 帧头/帧尾/版本号, 且每帧都在两个数据源间重新选择 —— 一旦选源翻转
 * 或载荷里出现单 bit 翻转(此私有帧无 checksum), 就会产出一个突变的大角度, 直接把
 * 平台猛推一下。重写引入三层防线:
 *
 *   收帧(ISR)           RXNE 攒字节 + IDLE 判帧尾  (未改, 稳定可靠)
 *      │
 *   ① 结构校验          长度=54 / 帧头"WT" / 帧尾0D0A / 版本13032 / 计数器去重
 *      │
 *   ② 源锁定 + 帧内校验  解锁后头几帧观测哪个源"活着"→锁死; 之后只用锁定源。
 *                       角度越界(|θ|>90°)丢; 加速度源额外要求 |a|∈[0.5,2.0]g
 *                       (排除快速甩动时线性加速度把 atan2 骗出大倾角)。
 *      │
 *   ③ 帧间尖峰过滤       无 checksum → 用时间一致性兜底: 相邻帧跳变 > 门限的
 *                       "孤立离群帧"先挂起不采信; 下一帧若与之一致才确认(真快速
 *                       运动只延迟一帧≈20ms), 否则丢弃(bit 翻转垃圾被隔离)。
 *
 * 只有全部通过, sensor_get() 才返回 1 并交出角度; 否则出参不动, 上层据此冻结保持。
 * ====================================================================
 */
#include "stm32f4xx_hal.h"
#include "sensor_wt.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- 校验/过滤门限(台架微调) ---- */
#define SW_ANGLE_LSB       (180.0f / 32768.0f)  /* 角度字段 raw→度 */
#define SW_ACC_LSB         (16.0f  / 32768.0f)  /* 加速度 raw→g   */
#define SW_MAX_ABS_DEG     90.0f    /* 角度范围校验: 超此值必为错位/垃圾 */
#define SW_ACC_G_MIN       0.5f     /* 加速度合理幅值下限(g): 低于此说明数据可疑 */
#define SW_ACC_G_MAX       2.0f     /* 上限: 高于此说明有大线性加速度(甩动), 倾角不可信 */
#define SW_JUMP_DEG        5.0f     /* 单帧跳变门限(度): 超此的孤立帧先挂起待确认(@50Hz, 20ms/帧, 等效240°/s) */
#define SW_SRC_OBS_FRAMES  15       /* 锁源观测帧数(≈0.3s @50Hz) */
#define SW_SRC_ALIVE_DEG   1.0f     /* 观测期角度字段幅值超此即认为"角度源活着" */

/* 数据源枚举 */
#define SRC_UNLOCKED  0
#define SRC_ANGLE     1
#define SRC_ACCEL     2

volatile uint32_t sensor_stat_frames = 0;
volatile uint32_t sensor_stat_valid  = 0;
volatile char     sensor_fail_reason = '0';

/* ---- 解析器状态(主上下文读写, 非 ISR) ---- */
static int      s_src = SRC_UNLOCKED;   /* 锁定的数据源 */
static int      s_obs_n = 0;            /* 已观测帧数 */
static float    s_obs_ang_max = 0.0f;   /* 观测期角度字段最大幅值 */
static float    s_last_r = 0.0f, s_last_p = 0.0f;
static int      s_have_last = 0;        /* 已有上一帧有效值(跳变基准) */
static float    s_pend_r = 0.0f, s_pend_p = 0.0f;
static int      s_have_pend = 0;        /* 有一个待确认的大跳变候选 */
static uint32_t s_last_counter = 0;
static int      s_have_counter = 0;

int sensor_using_accel(void) { return s_src == SRC_ACCEL; }

void sensor_reset(void)
{
   s_src = SRC_UNLOCKED;
   s_obs_n = 0;
   s_obs_ang_max = 0.0f;
   s_have_last = 0;
   s_have_pend = 0;
   s_have_counter = 0;
   sensor_fail_reason = '0';
}

/* ===== 收帧缓冲(ISR 写, 主上下文读) ===== */
#define SW_BUFSZ 128
static volatile uint8_t  rx_buf[SW_BUFSZ];
static volatile uint16_t rx_len = 0;
static volatile uint8_t  frame_buf[SW_BUFSZ];
static volatile uint16_t frame_len = 0;
static volatile uint8_t  frame_ready = 0;   /* 1=frame_buf 有整帧待主上下文解析 */

/* 调试: 无条件保留"最近一个帧边界"的原始字节(不受 frame_ready 门控影响),
 * 供降级空转/独立测试直接 hexdump 一手数据, 判断链路通没通、字节布局对不对。 */
static volatile uint8_t  dbg_buf[SW_BUFSZ];
static volatile uint16_t dbg_len = 0;

#define SW_FRAME_LEN  54       /* 12(ID) + 40(载荷) + 2(0D 0A) */

/* ================= 初始化 ================= */
void sensor_init(void)
{
   GPIO_InitTypeDef g = {0};

   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_USART6_CLK_ENABLE();

   /* PC7 = USART6_RX(上拉防悬空), PC6 = USART6_TX; 均复用推挽。
    * TX 用于每周期回传"本轮收没收到传感器"的状态字节给电脑。 */
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
         /* 调试快照: 每个帧边界都无条件更新, 即使主上下文没来取(frame_ready=1) */
         dbg_len = rx_len;
         for (uint16_t i = 0; i < rx_len; i++) dbg_buf[i] = rx_buf[i];
         if (!frame_ready) {            /* 上一帧已被主上下文取走才收新帧 */
            for (uint16_t i = 0; i < rx_len; i++) frame_buf[i] = rx_buf[i];
            frame_len   = rx_len;
            frame_ready = 1;
         }
         rx_len = 0;
      }
   }
}

/* 从 40 字节载荷解出角度字段候选(度)。 */
static void decode_angle_src(const uint8_t *pl, float *r, float *p)
{
   int16_t raw_r = (int16_t)((uint16_t)pl[22] | ((uint16_t)pl[23] << 8));
   int16_t raw_p = (int16_t)((uint16_t)pl[24] | ((uint16_t)pl[25] << 8));
   *r = (float)raw_r * SW_ANGLE_LSB;
   *p = (float)raw_p * SW_ANGLE_LSB;
}

/* 从加速度解出倾角(度), 并回报加速度幅值(g)供合理性门控。
 * roll  = atan2(ay, az)         绕 X → α(内翻外翻)
 * pitch = atan2(-ax, |ayz|)     绕 Y → β(跖屈背伸)  —— 与备忘录轴向一致, 不交换。*/
static float decode_accel_src(const uint8_t *pl, float *r, float *p)
{
   int16_t iax = (int16_t)((uint16_t)pl[8]  | ((uint16_t)pl[9]  << 8));
   int16_t iay = (int16_t)((uint16_t)pl[10] | ((uint16_t)pl[11] << 8));
   int16_t iaz = (int16_t)((uint16_t)pl[12] | ((uint16_t)pl[13] << 8));
   float ax = (float)iax * SW_ACC_LSB;
   float ay = (float)iay * SW_ACC_LSB;
   float az = (float)iaz * SW_ACC_LSB;
   float nyz = sqrtf(ay * ay + az * az);
   if (nyz < 1e-6f) nyz = 1e-6f;
   *r = atan2f(ay, az)   * 180.0f / (float)M_PI;
   *p = atan2f(-ax, nyz) * 180.0f / (float)M_PI;
   return sqrtf(ax * ax + ay * ay + az * az);   /* |a| in g */
}

/* ================= 主上下文: 取最新【通过全部校验】的帧 ================= */
int sensor_get(float *roll_deg, float *pitch_deg)
{
   if (!frame_ready) { sensor_fail_reason = '0'; return 0; }

   /* ---- 拷一份局部, 尽快放行 ISR 收下一帧 ---- */
   uint16_t n = frame_len;
   uint8_t  f[SW_FRAME_LEN];
   int ok = (n == SW_FRAME_LEN);
   if (ok) for (uint16_t i = 0; i < SW_FRAME_LEN; i++) f[i] = frame_buf[i];
   frame_ready = 0;

   /* ---- ① 结构校验 ---- */
   if (!ok)                                    { sensor_fail_reason = 'L'; return 0; }
   if (f[0] != 'W' || f[1] != 'T')             { sensor_fail_reason = 'H'; return 0; }
   if (f[52] != 0x0D || f[53] != 0x0A)         { sensor_fail_reason = 'T'; return 0; }

   const uint8_t *pl = &f[12];   /* 40 字节载荷 */
   uint16_t ver = (uint16_t)pl[38] | ((uint16_t)pl[39] << 8);
   if (ver != SENSOR_FW_VERSION)               { sensor_fail_reason = 'V'; return 0; }

   /* 计数器去重: 与上一帧完全相同=重发/卡帧, 不当作新数据(避免喂"假新鲜") */
   uint32_t cnt = (uint32_t)pl[4] | ((uint32_t)pl[5] << 8)
                | ((uint32_t)pl[6] << 16) | ((uint32_t)pl[7] << 24);
   if (s_have_counter && cnt == s_last_counter) { sensor_fail_reason = 'D'; return 0; }

   /* ---- ② 源锁定 + 帧内校验 ---- */
   float ar_ang, ap_ang, ar_acc, ap_acc, amag;
   decode_angle_src(pl, &ar_ang, &ap_ang);
   amag = decode_accel_src(pl, &ar_acc, &ap_acc);

   if (s_src == SRC_UNLOCKED) {
      /* 观测期: 累计角度字段幅值, 判它是否"活着"。观测期不产出有效数据(平台此时
       * 也在标定保持不动), 返回 0。够帧数即锁死数据源。 */
      float m = fabsf(ar_ang) + fabsf(ap_ang);
      if (m > s_obs_ang_max) s_obs_ang_max = m;
      s_last_counter = cnt; s_have_counter = 1;
      if (++s_obs_n >= SW_SRC_OBS_FRAMES) {
         s_src = (s_obs_ang_max > SW_SRC_ALIVE_DEG) ? SRC_ANGLE : SRC_ACCEL;
         s_have_last = 0;   /* 锁定后第一帧用作跳变基准, 不做跳变判定 */
      }
      sensor_fail_reason = 'U';
      return 0;
   }
   s_last_counter = cnt; s_have_counter = 1;

   float cr, cp;
   if (s_src == SRC_ACCEL) {
      /* 加速度源: 幅值须落在静态合理区间, 否则(甩动/垃圾)本帧不可信 */
      if (amag < SW_ACC_G_MIN || amag > SW_ACC_G_MAX) { sensor_fail_reason = 'A'; return 0; }
      cr = ar_acc; cp = ap_acc;
   } else {
      cr = ar_ang; cp = ap_ang;
   }

   /* 角度范围校验 */
   if (fabsf(cr) > SW_MAX_ABS_DEG || fabsf(cp) > SW_MAX_ABS_DEG) {
      sensor_fail_reason = 'R'; return 0;
   }

   /* ---- ③ 帧间尖峰过滤(无 checksum 的兜底: 时间一致性) ---- */
   if (!s_have_last) {
      /* 锁定后第一帧: 直接作为基准, 不判跳变 */
      s_last_r = cr; s_last_p = cp; s_have_last = 1; s_have_pend = 0;
   } else {
      float dr = fabsf(cr - s_last_r);
      float dp = fabsf(cp - s_last_p);
      if (dr <= SW_JUMP_DEG && dp <= SW_JUMP_DEG) {
         /* 平稳变化: 接受, 清挂起 */
         s_last_r = cr; s_last_p = cp; s_have_pend = 0;
      } else {
         /* 大跳变: 若与上一次挂起的候选一致 → 是真实快速运动, 采信;
          * 否则视为孤立离群(bit 翻转垃圾), 挂起本帧等下一帧确认, 本帧不产出。 */
         if (s_have_pend &&
             fabsf(cr - s_pend_r) <= SW_JUMP_DEG &&
             fabsf(cp - s_pend_p) <= SW_JUMP_DEG) {
            s_last_r = cr; s_last_p = cp; s_have_pend = 0;
         } else {
            s_pend_r = cr; s_pend_p = cp; s_have_pend = 1;
            sensor_fail_reason = 'J';
            return 0;
         }
      }
   }

   *roll_deg  = s_last_r;
   *pitch_deg = s_last_p;
   sensor_stat_valid++;
   sensor_fail_reason = '0';
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

/* ================= 调试: 取最近一个帧边界的原始字节 =================
 * 把最近收到(不管有没有过校验)的一帧原始字节拷进 out(最多 max 字节),
 * 返回实际字节数。用于降级空转/独立测试直接 hexdump 一手数据。
 * 非原子(ISR 可能正在写), 偶发一帧撕裂可接受 —— 仅诊断用。 */
uint16_t sensor_debug_last_frame(uint8_t *out, uint16_t max)
{
   uint16_t n = dbg_len;
   if (n > max) n = max;
   for (uint16_t i = 0; i < n; i++) out[i] = dbg_buf[i];
   return n;
}
