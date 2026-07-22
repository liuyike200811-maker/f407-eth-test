/*
 * 维特 WT901 传感器接入实现 —— USART6 (PC7=RX), 115200 8N1
 * 见 sensor_wt.h 头注释。寄存器级裸写, 风格对齐 modbus_slave.c / uart_log.c。
 *
 * ===================== 姿态解算: x-io Fusion 6轴 AHRS =====================
 * 【为什么换成融合】实测(6轴/50Hz)结论:
 *   - 传感器自带"欧拉角字段"(载荷pl22-25)是死的, 恒为哨兵值 -1 → 不可用。
 *   - 加速度(pl8-13)与陀螺(pl14-19)均活、随运动正确响应 → 原料齐全。
 * 原实现只能退回"纯加速度 atan2"解倾角, 它对平台运动时的线加速度敏感(甩一下就被
 * 骗出假倾角)、双轴解算在大角度还有耦合 —— 这正是"非目标自由度抖动"的根因。
 *
 * 本实现改用 Sebastian Madgwick 的 x-io Fusion 6轴IMU算法(加速度+陀螺, 无磁力计):
 *   - 陀螺积分给出【平滑、跟手、不受线加速度污染】的姿态变化;
 *   - 加速度作为【重力绝对基准】持续纠正陀螺漂移;
 *   - 内置【加速度剔除】: 当 |a| 明显偏离 1g(说明有大线加速度/甩动)时, 本步只信陀螺、
 *     不用加速度纠偏 —— 从源头挡住"运动时线加速度污染倾角", 根治抖动。
 *   - 初始化【渐变高增益】: 复位后头几秒用高增益快速收敛到真实姿态, 之后落到稳态增益。
 *
 * 数据链路(未改, 稳定可靠):
 *   收帧(ISR)  RXNE 攒字节 + IDLE 判帧尾
 *      │
 *   ① 结构校验  长度=54 / 帧头"WT" / 帧尾0D0A / 版本13032 / 计数器去重
 *      │
 *   ② 输入合理性 陀螺任一轴超量程(bit翻转垃圾)整帧丢; 加速度幅值荒谬整帧丢
 *      │
 *   ③ 6轴融合    fusion_update() 更新四元数 → 取欧拉角(roll绕X, pitch绕Y)
 *      │
 *   ④ 收敛预热 + 范围兜底  复位后头 SW_FUSION_WARMUP 帧不产出(等融合收敛); |角|越界丢
 *      │
 *   ⑤ 固定低通(本模块最后一步)  见下方"抖动来源"说明
 *
 * 只有全部通过, sensor_get() 才返回 1 并交出角度; 否则出参不动, 上层据此冻结保持。
 *
 * 【为什么融合之后还要再加一道固定低通】实测发现融合角对细微动作(哪怕生理性手部
 * 震颤/传感器本身噪声)反应很灵敏 —— 这本是融合算法的优点(不像旧算法靠"加速度幅值
 * 门控+跳变拒绝"误伤性地滤掉部分小动作), 但副作用是下游 ecat_motion.c 的死区(0.3°)、
 * One-Euro 滤波都是按"动得快不快"(角速度)判断要不要放松滤波的, 而抖动/震颤的角速度
 * 其实也不小(只是幅度小、来回快), 容易被误判成"用户在动"而放行。固定低通不看速度、
 * 只看【频率】: 抖动通常是高频(约4~12Hz)小幅来回, 真实康复动作是低频(<2~3Hz)单向
 * 移动, 截止频率卡在两者之间就能专门压高频、放低频, 正好补上速度自适应滤波这个盲区。
 * 放在本模块【最后一步】(融合之后、范围校验之后), 让 sensor_get() 交出去的就是"已经
 * 干净"的角度, 下游 ecat_motion.c 的整条处理链(零偏/增益/死区/One-Euro/限速)不用改。
 * ========================================================================
 */
#include "stm32f4xx_hal.h"
#include "sensor_wt.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- 量纲/校验门限(台架微调) ---- */
#define SW_ACC_LSB         (16.0f   / 32768.0f)  /* 加速度 raw→g   (量程±16g, 已实测: 水平az≈2048=1g) */
#define SW_GYRO_LSB        (2000.0f / 32768.0f)  /* 陀螺   raw→°/s (量程±2000°/s, WitMotion标准) */
#define SW_MAX_ABS_DEG     120.0f   /* 融合角范围兜底: 超此必为异常, 丢帧冻结 */
#define SW_GYRO_SANITY_DPS 1200.0f  /* 陀螺任一轴超此(°/s)判 bit 翻转垃圾, 整帧丢(远高于真实踝/手运动) */
#define SW_ACC_ABSURD_G    4.0f     /* 加速度幅值超此(g)判荒谬, 整帧丢 */

/* ---- x-io Fusion 参数 ---- */
#define FUSION_GAIN          0.5f   /* 稳态反馈增益(加速度纠偏权重), x-io默认0.5 */
#define FUSION_INIT_GAIN     10.0f  /* 初始化高增益: 复位后快速收敛到真实姿态 */
#define FUSION_INIT_PERIOD_S 3.0f   /* 初始化时长(s): 增益由 INIT_GAIN 线性降到 GAIN */
#define FUSION_ACC_REJECT_LO 0.75f  /* 加速度剔除下限(g): |a|出[LO,HI]则本步纯陀螺(不用加速度纠偏) */
#define FUSION_ACC_REJECT_HI 1.25f  /* 上限(g): 挡住甩动/平台运动的线加速度污染 */
#define SW_FUSION_WARMUP     25      /* 复位后预热帧数(≈0.5s @50Hz): 让融合收敛, 此间不产出有效数据 */
#define SW_FRAME_DT_S        0.02f   /* 帧间隔标称(50Hz); 实际按计数器增量推算, 见 sensor_get */

/* ---- 固定低通(融合之后的最后一步, 频率选择性压噪, 见上方说明) ---- */
#define SW_LPF_CUTOFF_HZ     2.0f    /* 截止频率(Hz): 卡在"正常康复动作"(<2~3Hz)与"抖动/震颤"(4~12Hz)之间, 台架可调 */

volatile uint32_t sensor_stat_frames = 0;
volatile uint32_t sensor_stat_valid  = 0;
volatile char     sensor_fail_reason = '0';

/* ---- 解析器状态(主上下文读写, 非 ISR) ---- */
static int      s_warm_n = 0;           /* 已预热帧数 */
static uint32_t s_last_counter = 0;
static int      s_have_counter = 0;

/* ---- 融合四元数状态(w,x,y,z) + 渐变增益 ---- */
static float    q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float    s_ramp_gain = FUSION_INIT_GAIN;

/* ---- 固定低通状态(融合之后的最后一步) ---- */
static float    s_lpf_r = 0.0f, s_lpf_p = 0.0f;
static int      s_lpf_init = 0;   /* 0=下一个有效帧直接作为起点(不引入"从0爬升"的假过渡) */

/* 保留接口: 现全程为融合(加速度+陀螺), 恒返回 1 供上层日志文案沿用。 */
int sensor_using_accel(void) { return 1; }

static void fusion_reset(void)
{
   q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
   s_ramp_gain = FUSION_INIT_GAIN;
}

void sensor_reset(void)
{
   s_warm_n = 0;
   s_have_counter = 0;
   fusion_reset();
   s_lpf_init = 0;   /* 低通下一帧重新取起点, 避免带着上一次会话的残留值 */
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

/* 从载荷解出加速度(g)与陀螺(°/s)。字节布局经实测确认:
 *   加速度 pl[8..13]  (X/Y/Z int16 小端), ×16/32768 = g
 *   陀螺   pl[14..19] (X/Y/Z int16 小端), ×2000/32768 = °/s   */
static void decode_imu(const uint8_t *pl,
                       float *ax, float *ay, float *az,
                       float *gx, float *gy, float *gz)
{
   int16_t iax = (int16_t)((uint16_t)pl[8]  | ((uint16_t)pl[9]  << 8));
   int16_t iay = (int16_t)((uint16_t)pl[10] | ((uint16_t)pl[11] << 8));
   int16_t iaz = (int16_t)((uint16_t)pl[12] | ((uint16_t)pl[13] << 8));
   int16_t igx = (int16_t)((uint16_t)pl[14] | ((uint16_t)pl[15] << 8));
   int16_t igy = (int16_t)((uint16_t)pl[16] | ((uint16_t)pl[17] << 8));
   int16_t igz = (int16_t)((uint16_t)pl[18] | ((uint16_t)pl[19] << 8));
   *ax = (float)iax * SW_ACC_LSB;  *ay = (float)iay * SW_ACC_LSB;  *az = (float)iaz * SW_ACC_LSB;
   *gx = (float)igx * SW_GYRO_LSB; *gy = (float)igy * SW_GYRO_LSB; *gz = (float)igz * SW_GYRO_LSB;
}

/* ================= x-io Fusion 6轴一步更新 =================
 * gyro 单位 °/s, accel 单位 g, dt 单位 s。更新全局四元数 q0..q3。
 * 算法即 x-io Fusion 的 FusionAhrsUpdateNoMagnetometer 核心:
 *   halfGravity = 由四元数估计的重力方向(半量);
 *   halfFeedback = accel归一化 × halfGravity  (叉积, 指向纠偏方向);
 *   半角速度 = 0.5*陀螺(rad/s) + 增益*halfFeedback;
 *   q += (q ⊗ (0,半角速度)) * dt; 归一化。
 * 加速度剔除: |a|偏离1g过多则本步 halfFeedback=0(纯陀螺积分), 挡住线加速度污染。 */
static void fusion_update(float gx, float gy, float gz,
                          float ax, float ay, float az, float dt)
{
   const float DEG2RAD = (float)M_PI / 180.0f;
   float wx = gx * DEG2RAD, wy = gy * DEG2RAD, wz = gz * DEG2RAD;

   /* --- 加速度纠偏(半反馈) --- */
   float fx = 0.0f, fy = 0.0f, fz = 0.0f;
   float amag = sqrtf(ax * ax + ay * ay + az * az);
   if (amag > FUSION_ACC_REJECT_LO && amag < FUSION_ACC_REJECT_HI) {
      float inv = 1.0f / amag;
      float axn = ax * inv, ayn = ay * inv, azn = az * inv;   /* 归一化重力测量 */
      /* 由当前四元数估计的重力方向(半量, 即旋转矩阵第三列/2) */
      float hgx = q1 * q3 - q0 * q2;
      float hgy = q0 * q1 + q2 * q3;
      float hgz = q0 * q0 - 0.5f + q3 * q3;
      /* 误差 = 测量重力 × 估计重力 (叉积) */
      fx = ayn * hgz - azn * hgy;
      fy = azn * hgx - axn * hgz;
      fz = axn * hgy - ayn * hgx;
   }
   /* 渐变增益: 初始化期高增益快速收敛, 之后落到稳态 GAIN */
   float gain = FUSION_GAIN;
   if (s_ramp_gain > FUSION_GAIN) {
      gain = s_ramp_gain;
      s_ramp_gain -= (FUSION_INIT_GAIN - FUSION_GAIN) * dt / FUSION_INIT_PERIOD_S;
      if (s_ramp_gain < FUSION_GAIN) s_ramp_gain = FUSION_GAIN;
   }
   /* 半角速度 = 0.5*陀螺 + 增益*半反馈 */
   float hwx = 0.5f * wx + gain * fx;
   float hwy = 0.5f * wy + gain * fy;
   float hwz = 0.5f * wz + gain * fz;
   /* 四元数积分: q += (q ⊗ (0, 半角速度)) * dt */
   float dq0 = (-q1 * hwx - q2 * hwy - q3 * hwz) * dt;
   float dq1 = ( q0 * hwx + q2 * hwz - q3 * hwy) * dt;
   float dq2 = ( q0 * hwy - q1 * hwz + q3 * hwx) * dt;
   float dq3 = ( q0 * hwz + q1 * hwy - q2 * hwx) * dt;
   q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;
   /* 归一化 */
   float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
   if (n > 1e-9f) { float inv = 1.0f / n; q0 *= inv; q1 *= inv; q2 *= inv; q3 *= inv; }
}

/* 四元数 → 欧拉角(度): roll 绕 X(内翻外翻α), pitch 绕 Y(跖屈背伸β)。
 * 采用标准 aerospace ZYX 提取, 已核对与旧 atan2 加速度解算【符号一致】,
 * 故 ecat_motion.c 的 SENSOR_SIGN_ALPHA/BETA 无需改动(仍可现场取反核对)。 */
static void fusion_euler(float *roll_deg, float *pitch_deg)
{
   const float RAD2DEG = 180.0f / (float)M_PI;
   float sinr = 2.0f * (q0 * q1 + q2 * q3);
   float cosr = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
   *roll_deg = atan2f(sinr, cosr) * RAD2DEG;
   float sinp = 2.0f * (q0 * q2 - q3 * q1);
   if (sinp >  1.0f) sinp =  1.0f;
   if (sinp < -1.0f) sinp = -1.0f;
   *pitch_deg = asinf(sinp) * RAD2DEG;
}

/* 固定截止频率一阶低通(非自适应, 与 ecat_motion.c 里 One-Euro 的"按速度调滤波强度"
 * 不同 —— 这里只按【频率】走, 专门压 SW_LPF_CUTOFF_HZ 以上的高频抖动/震颤, 见顶注。
 * 首帧(s_lpf_init=0)直接以当前值为起点, 不引入"从0爬升"的假过渡。 */
static void lpf_step(float cr, float cp, float dt, float *out_r, float *out_p)
{
   if (!s_lpf_init) {
      s_lpf_r = cr; s_lpf_p = cp; s_lpf_init = 1;
   } else {
      float tau   = 1.0f / (2.0f * (float)M_PI * SW_LPF_CUTOFF_HZ);
      float alpha = dt / (tau + dt);
      s_lpf_r += alpha * (cr - s_lpf_r);
      s_lpf_p += alpha * (cp - s_lpf_p);
   }
   *out_r = s_lpf_r; *out_p = s_lpf_p;
}

/* ================= 主上下文: 取最新【通过全部校验+融合+低通】的姿态角 ================= */
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

   /* 帧间 dt: 按计数器增量推算(丢帧时自动拉长), 异常则退回标称 20ms */
   float dt = SW_FRAME_DT_S;
   if (s_have_counter) {
      uint32_t d = cnt - s_last_counter;
      if (d >= 1 && d <= 5) dt = (float)d * SW_FRAME_DT_S;   /* 正常/轻微丢帧 */
   }
   s_last_counter = cnt; s_have_counter = 1;

   /* ---- ② 输入合理性(bit 翻转垃圾挡在融合之外) ---- */
   float ax, ay, az, gx, gy, gz;
   decode_imu(pl, &ax, &ay, &az, &gx, &gy, &gz);
   if (fabsf(gx) > SW_GYRO_SANITY_DPS ||
       fabsf(gy) > SW_GYRO_SANITY_DPS ||
       fabsf(gz) > SW_GYRO_SANITY_DPS)          { sensor_fail_reason = 'J'; return 0; }
   float amag = sqrtf(ax * ax + ay * ay + az * az);
   if (amag > SW_ACC_ABSURD_G)                  { sensor_fail_reason = 'A'; return 0; }

   /* ---- ③ 6轴融合更新 ---- */
   fusion_update(gx, gy, gz, ax, ay, az, dt);

   /* ---- ④ 收敛预热: 复位后头几帧只喂融合、不产出(平台此时也在标定保持不动) ---- */
   if (s_warm_n < SW_FUSION_WARMUP) {
      s_warm_n++;
      sensor_fail_reason = 'U';
      return 0;
   }

   float cr, cp;
   fusion_euler(&cr, &cp);

   /* 范围兜底(在低通之前, 用未滤波的原始融合角判, 避免异常值被悄悄磨平后放过) */
   if (fabsf(cr) > SW_MAX_ABS_DEG || fabsf(cp) > SW_MAX_ABS_DEG) {
      sensor_fail_reason = 'R'; return 0;
   }

   /* ---- ⑤ 固定低通(本模块最后一步, 见顶注"为什么融合之后还要再加一道固定低通") ---- */
   lpf_step(cr, cp, dt, roll_deg, pitch_deg);

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
