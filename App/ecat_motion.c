/*
 * EtherCAT 三轴康复运动控制 —— STM32 板上 SOEM (CSV 模式, 命令驱动)
 * ================================================================
 * 架构: 开机→扫从站→配CSV的PDO→进OP→使能→[待机]→听串口命令→跑模式→回待机
 *
 * 控制: CSV(速度模式, 6060=9)。位置反馈用 6063(CSV下6064恒为0, 见记忆),
 *       每轴 前馈速度 + 位置P闭环(带编码器方向自动锁定), 防漂移/防接反跑飞。
 *
 * 6个模式(全部CSV, 均为 上升→运动→下降 三段, 归零除外):
 *   0 综合波浪   1 跖屈/背伸   2 内翻/外翻   3 环形   4 8字      5 归零(堵转检测)
 *   1~4 走 3-RPS 正解(FK): 踝角(α内翻外翻, β跖屈背伸)→三电缸长度→速度。
 *
 * 串口命令 (USART1/CH340 115200 或 USB Slave/Micro-USB虚拟串口, 二选一或都行, 行末\r或\n):
 *   ?          查看状态/帮助
 *   h          归零(收回到机械限位)
 *   g<n>       运行模式 n (0波浪 1跖背 2内外 3环形 4八字)
 *   x          急停(停止当前运动, 平滑降速回待机)
 *   q          下电退出(脱力→退INIT, 之后可安全断电/拔网线)
 *   a<度> b<度>  设 α / β 幅度(度)     f<厘赫> 设频率(25=0.25Hz)
 *   w<mm>      设波浪幅度              r<mm> 设上升高度   c<次> 设运动循环数
 *
 * ⚠ 未做 DC 同步, 信捷 DS5C1 在 CSV 下自由运行(free-run)。实测单轴最大行程
 *   约159mm(见探测标定), 上升高度+运动幅度务必留余量; 建议每次运动前先 h 归零。
 */
#include "soem/soem.h"
#include "ecat_motion.h"
#include "uart_log.h"
#include "usbd_cdc_if.h"
#include "modbus_slave.h"
#include "osal.h"
#include "stm32f4xx_hal.h"   /* 心跳灯(GPIOF/PF9)直接寄存器访问用 */
#include <string.h>
#include <math.h>

volatile int g_ec_slavecount = 0;
volatile int g_ec_phase = 0;
volatile int g_ec_fault = 0;
volatile unsigned int g_reset_cause = 0;

static ecx_contextt ctx;
static uint8 IOmap[256];

/* ---- 机械/编码器常量 ---- */
#define LEAD_MM        3.0          /* 丝杠导程 mm/圈 */
#define PULSE_PER_REV  131072.0     /* 编码器 脉冲/圈 (17位) */
#define MM_PER_PULSE   (LEAD_MM / PULSE_PER_REV)
#define CSV_MODE       9
#define DT             0.004        /* 通信周期 4ms (与PC端轨迹常量一致) */
#define CYC_US         4000

/* ---- 闭环控制常量(与PC端 test_xinje_csv_wave_v2 对齐) ---- */
#define KP_POS         8.0          /* 位置环增益 1/s */
#define VMAX           (600.0/60.0*PULSE_PER_REV)   /* 速度上限 ≈600RPM */
#define AMAX           (VMAX/0.15)  /* 加速度上限, ~0.15s达满速 */
#define LPF_TAU        0.15         /* 纠正量低通时间常数 s */
#define DEADBAND       300.0        /* 位置误差死区 pulse */
#define SIGN_LOCK_PULSE 20000.0     /* 上升位移累计超此值即锁定反馈方向 */
#define RAMP_FRAMES    150          /* 上升/下降加减速帧数 */

/* ---- 归零(堵转检测, 与PC端 retract_all 对齐) ---- */
#define RETRACT_VEL    (-163840)    /* 收缩速度 pulse/s (≈-75RPM) */
#define STALL_THR      8000         /* |实际速度|低于此值算堵转 pulse/s */
#define STALL_FRAMES   50           /* 连续多少帧堵转判定到位(≈200ms) */
#define HOME_MAX_FRAMES 7500        /* 归零超时(≈30s) */

/* ---- 可由串口命令修改的运动参数(带安全默认值) ---- */
static int    g_aa_deg   = 15;      /* α 幅度(度) */
static int    g_ba_deg   = 15;      /* β 幅度(度) */
static int    g_freq_cHz = 25;      /* 频率(厘赫兹) 25=0.25Hz */
static int    g_wave_mm  = 30;      /* 波浪幅度 mm (mode0) */
static int    g_rise_mm  = 60;      /* 上升高度 mm */
static int    g_rise_rpm = 150;     /* 上升速度 RPM */
static int    g_cycles   = 9;       /* 运动循环数 */

#define WAVE_PEAK_RPM  471.0        /* 波浪峰值速度(mode0) */
#define FK_Z0          380.0        /* FK 归位参考工作高度 mm */

/* ---- 闭环状态(每轴) ---- */
static int32_t start_pos[EC_MAXSLAVE];   /* 运动起点编码器零点(6063) */
static double  cmd_pos[EC_MAXSLAVE];     /* 期望位置=前馈速度积分 */
static double  v_last[EC_MAXSLAVE];      /* 上帧命令速度, 限加速度用 */
static double  corr_filt[EC_MAXSLAVE];   /* 低通后的纠正量 */
static int     fb_sign[EC_MAXSLAVE];     /* 反馈方向: 0待锁 ±1已锁 */
static const double PHASE[4] = {0.0, 0.0, 2.0*M_PI/3.0, 4.0*M_PI/3.0};  /* 三轴波浪相位(索引1~3) */

/* ---- 命令状态 ---- */
static volatile int g_run_request = -1;  /* -1无; 0~4运行对应模式 */
static volatile int g_do_home     = 0;
static volatile int g_quit        = 0;
static volatile int g_abort       = 0;   /* 运动中急停标志 */

/* HMI(Modbus 4x0010/4x0018)反馈用: 运行状态字 与 当前运行模式 */
static volatile int g_status   = 0;      /* 0启动中 1待机 2运行 3归零 4故障 5已下电 */
static volatile int g_cur_mode = 99;     /* 0~4=正在跑的模式; 99=待机/无模式 */

static volatile int g_wkc = 0;

/* ---- PDO 字节访问 (RxPDO: CW+mode+60FF; TxPDO: SW+mode+6063+606C) ---- */
static inline void write_pdo(int sl, uint16_t cw, int32_t vel)
{
   uint8 *o = ctx.slavelist[sl].outputs;
   if (!o) return;
   *(uint16_t *)(o + 0) = cw;
   *(int8_t  *)(o + 2) = CSV_MODE;
   *(int32_t *)(o + 3) = vel;
}
static inline uint16_t read_sw(int sl)
{ uint8 *in = ctx.slavelist[sl].inputs; return in ? *(uint16_t *)(in + 0) : 0; }
static inline int32_t read_pos63(int sl)   /* 6063 实际位置(CSV下唯一有效) @offset3 */
{ uint8 *in = ctx.slavelist[sl].inputs; return in ? *(int32_t *)(in + 3) : 0; }
static inline int32_t read_vel(int sl)     /* 606C 实际速度 @offset7 */
{ uint8 *in = ctx.slavelist[sl].inputs; return in ? *(int32_t *)(in + 7) : 0; }

static int any_fault(void)
{
   for (int sl = 1; sl <= ctx.slavecount; sl++)
      if (read_sw(sl) & 0x0008) return sl;
   return 0;
}

/* 心跳灯: LED0=PF9 低电平点亮. 每约500ms翻转一次(4ms节拍 * 125次).
   只要这个在闪, 就说明 cycle() 在正常跑, 没卡死/没硬件异常复位. */
static inline void heartbeat(void)
{
   static int n = 0;
   if (++n >= 125) { n = 0; GPIOF->ODR ^= GPIO_PIN_9; }
}

/* 一个通信周期: 发+收过程数据, 固定4ms节拍。
 * 节拍对齐用绝对时刻(next_deadline每次只加4ms, 不受本轮实际耗时影响),
 * 不能用osal_usleep(相对延时) —— 那样真实周期会变成"4ms+本轮EtherCAT收发耗时",
 * 且逐帧抖动, 而vel_closed()里的位置积分(cmd_pos += v_ff*DT)是按严格4ms算的,
 * 周期一旦漂移/抖动, 闭环用来对比的"期望位置"就会跟伺服真实走的对不上,
 * P环持续误纠正, 表现为运动发飘/发抖(对齐PC端clock_nanoseep(TIMER_ABSTIME)的做法)。 */
static void cycle(void)
{
   static ec_timet next_deadline;
   static int inited = 0;

   ecx_send_processdata(&ctx);
   g_wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
   heartbeat();

   if (!inited) { osal_get_monotonic_time(&next_deadline); inited = 1; }

   next_deadline.tv_nsec += (long)CYC_US * 1000L;
   while (next_deadline.tv_nsec >= 1000000000L) {
      next_deadline.tv_sec++;
      next_deadline.tv_nsec -= 1000000000L;
   }
   osal_monotonic_sleep(&next_deadline);
}

/* ================= 串口命令解析 ================= */
/* 累积一行(\r或\n结束)后分派。行首字母=命令, 其余为可选整数参数。 */
static void handle_cmd(char *line)
{
   char c = line[0];
   int  arg = 0, has = 0;
   for (char *p = line + 1; *p; p++) {
      if (*p >= '0' && *p <= '9') { arg = arg * 10 + (*p - '0'); has = 1; }
   }
   switch (c) {
      case '?':
         uart_log("命令: ? 帮助 | h归零 | g<0-4>运行模式 | x急停 | q下电退出\r\n"
                  "      a<度>α b<度>β f<厘赫>频率 w<mm>波幅 r<mm>升高 c<次>循环\r\n"
                  "参数: mode默认由g指定  α=%d° β=%d° 频率=%.2fHz 波幅=%dmm 升高=%dmm 循环=%d\r\n"
                  "模式: 0波浪 1跖屈背伸 2内翻外翻 3环形 4八字 (5=归零用h)\r\n",
                  g_aa_deg, g_ba_deg, g_freq_cHz / 100.0, g_wave_mm, g_rise_mm, g_cycles);
         break;
      case 'h': g_do_home = 1; break;
      case 'x': g_abort = 1; break;
      case 'q': g_quit = 1; break;
      case 'g':
         if (has && arg >= 0 && arg <= 4) g_run_request = arg;
         else uart_log("g 需要 0~4 的模式号\r\n");
         break;
      case 'a': if (has) { g_aa_deg = arg; uart_log("α幅度=%d°\r\n", arg); } break;
      case 'b': if (has) { g_ba_deg = arg; uart_log("β幅度=%d°\r\n", arg); } break;
      case 'f': if (has) { g_freq_cHz = arg; uart_log("频率=%.2fHz\r\n", arg / 100.0); } break;
      case 'w': if (has) { g_wave_mm = arg; uart_log("波幅=%dmm\r\n", arg); } break;
      case 'r': if (has) { g_rise_mm = arg; uart_log("升高=%dmm\r\n", arg); } break;
      case 'c': if (has) { g_cycles = arg; uart_log("循环=%d\r\n", arg); } break;
      default: break;   /* 空行/未知命令忽略 */
   }
}

/* 把一个字节喂进行缓冲区, 攒够一行(\r或\n)就解析。USART1/USB两路共用同一份逻辑。 */
static void feed_cmd_byte(char ch)
{
   static char buf[32];
   static int  len = 0;
   if (ch == '\r' || ch == '\n') {
      if (len > 0) { buf[len] = '\0'; handle_cmd(buf); len = 0; }
   } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = ch;
   } else {
      len = 0;   /* 溢出丢弃 */
   }
}

/* ================= Modbus (HMI) 对接 ================= */
static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* 把 HMI 经 Modbus 写来的设定/命令落到本地全局。
 * 设定区: 只在 HMI 改动过(值变化)时才落地, 越界截断(兑现契约"越界自动截断"),
 *         这样串口/USB 命令改的参数不会被每周期原样覆盖。
 * 控制区: 一次性触发线圈, 受理后清0; 模式/归零仅待机受理, 急停/下电/复位随时受理。*/
static void modbus_sync(void)
{
   static uint16_t shadow[7];
   static int seeded = 0;
   if (!seeded) {   /* 用固件默认值播种设定区, 让 HMI 首次读到的是真实默认, 而非0 */
      modbus_hreg[0] = g_aa_deg;   modbus_hreg[1] = g_ba_deg;   modbus_hreg[2] = g_freq_cHz;
      modbus_hreg[3] = g_wave_mm;  modbus_hreg[4] = g_rise_mm;  modbus_hreg[5] = g_cycles;
      modbus_hreg[6] = g_rise_rpm;
      for (int i = 0; i < 7; i++) shadow[i] = modbus_hreg[i];
      seeded = 1;
   }
   if (modbus_hreg[0] != shadow[0]) { shadow[0] = modbus_hreg[0]; g_aa_deg   = clampi(modbus_hreg[0], 0, 30); }
   if (modbus_hreg[1] != shadow[1]) { shadow[1] = modbus_hreg[1]; g_ba_deg   = clampi(modbus_hreg[1], 0, 30); }
   if (modbus_hreg[2] != shadow[2]) { shadow[2] = modbus_hreg[2]; g_freq_cHz = clampi(modbus_hreg[2], 5, 80); }
   if (modbus_hreg[3] != shadow[3]) { shadow[3] = modbus_hreg[3]; g_wave_mm  = clampi(modbus_hreg[3], 0, 40); }
   if (modbus_hreg[4] != shadow[4]) { shadow[4] = modbus_hreg[4]; g_rise_mm  = clampi(modbus_hreg[4], 0, 80); }
   if (modbus_hreg[5] != shadow[5]) { shadow[5] = modbus_hreg[5]; g_cycles   = clampi(modbus_hreg[5], 1, 99); }
   if (modbus_hreg[6] != shadow[6]) { shadow[6] = modbus_hreg[6]; g_rise_rpm = clampi(modbus_hreg[6], 50, 200); }

   /* 急停/下电/故障复位: 任何时候都响应 */
   if (modbus_coils[6]) { modbus_coils[6] = 0; g_abort = 1; }
   if (modbus_coils[7]) { modbus_coils[7] = 0; g_quit  = 1; }
   if (modbus_coils[8]) { modbus_coils[8] = 0; g_ec_fault = 0; }  /* 清故障显示 */

   /* 模式0~4 / 归零: 仅待机受理; 运行中收到则清掉不执行(防中途乱切, 见契约) */
   if (g_status == 1) {
      int req = -1;
      for (int m = 0; m < 5; m++)
         if (modbus_coils[m]) { if (req < 0) req = m; modbus_coils[m] = 0; }  /* 多个同ON以先到为准 */
      if (modbus_coils[5]) { modbus_coils[5] = 0; g_do_home = 1; }
      else if (req >= 0)   { g_run_request = req; }
   } else {
      for (int m = 0; m <= 5; m++) modbus_coils[m] = 0;
   }
}

/* 把本地状态回写 Modbus 反馈寄存器(4x0010~4x0018 = 下标16~24)。每周期一次。 */
static void modbus_write_feedback(void)
{
   static uint16_t hb = 0;
   int32_t p1 = (ctx.slavecount >= 1) ? (read_pos63(1) - start_pos[1]) : 0;
   int32_t p2 = (ctx.slavecount >= 2) ? (read_pos63(2) - start_pos[2]) : 0;
   int32_t p3 = (ctx.slavecount >= 3) ? (read_pos63(3) - start_pos[3]) : 0;

   modbus_hreg[16] = (uint16_t)g_status;                    /* 运行状态字 */
   modbus_hreg[17] = (uint16_t)g_ec_fault;                  /* 故障从站号 */
   modbus_hreg[18] = (uint16_t)g_ec_slavecount;             /* 在线从站数 */
   modbus_hreg[19] = (uint16_t)g_wkc;                       /* WKC */
   modbus_hreg[20] = (uint16_t)(int16_t)(p1 * MM_PER_PULSE * 10.0);  /* 轴1位置 mm×10 有符号 */
   modbus_hreg[21] = (uint16_t)(int16_t)(p2 * MM_PER_PULSE * 10.0);  /* 轴2 */
   modbus_hreg[22] = (uint16_t)(int16_t)(p3 * MM_PER_PULSE * 10.0);  /* 轴3 */
   modbus_hreg[23] = ++hb;                                  /* 心跳(溢出自然归零) */
   modbus_hreg[24] = (uint16_t)g_cur_mode;                  /* 当前运行模式 */
}

/* 非阻塞轮询命令通道: USART1(CH340)、USB Slave(虚拟串口)、Modbus/HMI(USART3) 三路并收。
   每个通信周期调一次 —— 运动中也在调, 故急停/HMI命令运动中同样即时响应。 */
static void poll_cmd(void)
{
   int ch;
   while ((ch = uart_rx_getc()) >= 0)  feed_cmd_byte((char)ch);
   while ((ch = usb_cdc_getc()) >= 0)  feed_cmd_byte((char)ch);
   modbus_poll();            /* 收/解析/应答 Modbus 帧 */
   modbus_sync();            /* HMI 命令/参数 → 本地全局 */
   modbus_write_feedback();  /* 本地状态 → 反馈寄存器 */
}

/* ================= 前馈 + 位置P闭环 ================= */
static int32_t limit_out(int sl, double v)
{
   if (v >  VMAX) v =  VMAX;
   if (v < -VMAX) v = -VMAX;
   double dvmax = AMAX * DT;
   double dv = v - v_last[sl];
   if (dv >  dvmax) v = v_last[sl] + dvmax;
   if (dv < -dvmax) v = v_last[sl] - dvmax;
   v_last[sl] = v;
   return (int32_t)v;
}

/* v_ff: 前馈速度(pulse/s)。返回本帧应下发的速度指令。
 * 反馈方向未锁前走纯前馈(=开环, 安全), 用上升自然运动锁定极性保证负反馈。 */
static int32_t vel_closed(int sl, double v_ff)
{
   cmd_pos[sl] += v_ff * DT;
   double raw = (double)(read_pos63(sl) - start_pos[sl]);

   if (fb_sign[sl] == 0) {
      if (fabs(raw) > SIGN_LOCK_PULSE && fabs(cmd_pos[sl]) > SIGN_LOCK_PULSE) {
         fb_sign[sl] = ((cmd_pos[sl] > 0) == (raw > 0)) ? 1 : -1;
         cmd_pos[sl] = fb_sign[sl] * raw;   /* 对齐, 消除接入瞬间突跳 */
      }
      return limit_out(sl, v_ff);
   }

   double actual = fb_sign[sl] * raw;
   double err = cmd_pos[sl] - actual;
   double e = (err < DEADBAND && err > -DEADBAND) ? 0.0 : err;
   double corr_raw = KP_POS * e;
   double alpha = (LPF_TAU > 0.0) ? DT / (LPF_TAU + DT) : 1.0;
   corr_filt[sl] += alpha * (corr_raw - corr_filt[sl]);
   return limit_out(sl, v_ff + corr_filt[sl]);
}

/* ================= 3-RPS 正向运动学 ================= */
/* R_base=200 r_mov=140 120°均布; alpha内翻外翻(绕X), beta跖屈背伸(绕Y) */
static void fk_3rps(double alpha, double beta, double z_eff, double l_out[3])
{
   static const double A[3][2] = {
      { 200.0,   0.0        },
      {-100.0,  173.2050808 },
      {-100.0, -173.2050808 }
   };
   static const double B[3][2] = {
      { 140.0,   0.0        },
      { -70.0,  121.2435565 },
      { -70.0, -121.2435565 }
   };
   double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
   double R[3][3] = {
      { cb,   sa * sb,  ca * sb },
      { 0.0,  ca,      -sa      },
      {-sb,   sa * cb,  ca * cb }
   };
   for (int i = 0; i < 3; i++) {
      double bx = R[0][0] * B[i][0] + R[0][1] * B[i][1];
      double by = R[1][0] * B[i][0] + R[1][1] * B[i][1];
      double bz = z_eff + R[2][0] * B[i][0] + R[2][1] * B[i][1];
      double dx = bx - A[i][0];
      double dy = by - A[i][1];
      l_out[i] = sqrt(dx * dx + dy * dy + bz * bz);
   }
}

/* 角度轨迹: 按模式返回当前相位 t 的 alpha,beta(弧度) */
static void get_angles(int mode, double t, double aa_r, double ba_r,
                       double *alpha, double *beta)
{
   switch (mode) {
      case 1: *alpha = 0;             *beta = ba_r * sin(t);         break;  /* 跖屈/背伸 */
      case 2: *alpha = aa_r * sin(t); *beta = 0;                     break;  /* 内翻/外翻 */
      case 3: *alpha = aa_r * sin(t); *beta = ba_r * cos(t);         break;  /* 环形 */
      case 4: *alpha = aa_r * sin(t); *beta = ba_r * sin(2.0 * t);   break;  /* 8字 */
      default:*alpha = 0;             *beta = 0;                     break;
   }
}

/* ================= 运动阶段公共件 ================= */
/* 运动前初始化闭环零点(以当前位置为起点) */
static void motion_reset(void)
{
   for (int sl = 1; sl <= ctx.slavecount; sl++) {
      start_pos[sl] = read_pos63(sl);
      cmd_pos[sl]   = 0.0;
      v_last[sl]    = 0.0;
      corr_filt[sl] = 0.0;
      fb_sign[sl]   = 0;
   }
}

/* 急停/报警时平滑降速回0(闭环, 保持使能)。返回后回待机。 */
static void ramp_to_zero(void)
{
   for (int i = 0; i < RAMP_FRAMES; i++) {
      for (int sl = 1; sl <= ctx.slavecount; sl++)
         write_pdo(sl, 0x000F, vel_closed(sl, 0.0));
      cycle();
   }
   for (int i = 0; i < 50; i++) {
      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0);
      cycle();
   }
}

/* 三轴同步做一段"前馈速度=vff(每轴相同)"的运动. 返回: 0正常 -1报警 -2急停 */
/* raise_vel: 匀速目标(pulse/s, 正=上升 负=下降); 内部自动加/匀/减速三段 */
static int move_ramp(double raise_vel, int cruise_frames)
{
   int i;
   /* 加速 */
   for (i = 0; i < RAMP_FRAMES; i++) {
      double vff = raise_vel * i / RAMP_FRAMES;
      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, vff));
      cycle(); poll_cmd();
      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
      if (g_abort) return -2;
   }
   /* 匀速 */
   for (i = 0; i < cruise_frames; i++) {
      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, raise_vel));
      cycle(); poll_cmd();
      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
      if (g_abort) return -2;
   }
   /* 减速 */
   for (i = 0; i < RAMP_FRAMES; i++) {
      double vff = raise_vel * (RAMP_FRAMES - i) / RAMP_FRAMES;
      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, vff));
      cycle(); poll_cmd();
      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
      if (g_abort) return -2;
   }
   return 0;
}

/* ================= 康复模式 (上升→运动→下降) ================= */
static int run_rehab_mode(int mode)
{
   int i, sl, rc;
   g_abort = 0;
   g_ec_fault = 0;
   g_status = 2; g_cur_mode = mode;   /* HMI 反馈: 运行中 + 当前模式 */
   motion_reset();

   int32_t raise_vel  = (int32_t)((double)g_rise_rpm / 60.0 * PULSE_PER_REV);
   int32_t rise_pulse = (int32_t)((double)g_rise_mm * PULSE_PER_REV / LEAD_MM);
   int cruise_frames  = (int)((double)rise_pulse / ((double)raise_vel * DT));
   if (cruise_frames < 1) cruise_frames = 1;

   uart_log(">>> 模式%d 开始: 上升%dmm@%dRPM → 运动 → 下降 <<<\r\n", mode, g_rise_mm, g_rise_rpm);

   /* 阶段1: 上升 */
   rc = move_ramp((double)raise_vel, cruise_frames);
   if (rc < 0) goto stopmsg;
   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); }
   uart_log("  上升完成, 方向锁定: 轴1=%+d 轴2=%+d 轴3=%+d\r\n", fb_sign[1], fb_sign[2], fb_sign[3]);

   /* 阶段2: 运动 */
   if (mode == 0) {
      /* 综合波浪(电缸空间三轴相位差) */
      int32_t wave_amp = (int32_t)((double)g_wave_mm * PULSE_PER_REV / LEAD_MM);
      int wave_period = (int)((double)wave_amp * 2.0 * M_PI / (WAVE_PEAK_RPM / 60.0 * PULSE_PER_REV) / DT);
      if (wave_period < 100) wave_period = 100;
      double wave_v_amp = (double)wave_amp * 2.0 * M_PI / wave_period / DT;
      int total = g_cycles * wave_period;
      uart_log("  阶段2 综合波浪: 幅%dmm 周期%.1fs × %d\r\n", g_wave_mm, wave_period * DT, g_cycles);
      for (i = 0; i < total; i++) {
         double t = 2.0 * M_PI * i / wave_period;
         for (sl = 1; sl <= ctx.slavecount; sl++) {
            double vff = wave_v_amp * cos(t + PHASE[sl]);
            write_pdo(sl, 0x000F, vel_closed(sl, vff));
         }
         cycle(); poll_cmd();
         if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
         if (g_abort) { rc = -2; goto stopmsg; }
         if (i % 500 == 0)
            uart_log("  [%ds] 轴1=%.1fmm 轴2=%.1fmm 轴3=%.1fmm\r\n", (int)(i * DT),
                     (read_pos63(1) - start_pos[1]) * MM_PER_PULSE,
                     (read_pos63(2) - start_pos[2]) * MM_PER_PULSE,
                     (read_pos63(3) - start_pos[3]) * MM_PER_PULSE);
      }
   } else {
      /* FK 角度空间(1跖背 2内外 3环形 4八字) */
      double aa_r = g_aa_deg * M_PI / 180.0;
      double ba_r = g_ba_deg * M_PI / 180.0;
      if (aa_r > M_PI / 6) aa_r = M_PI / 6;    /* 限幅±30° */
      if (ba_r > M_PI / 6) ba_r = M_PI / 6;
      double freq = g_freq_cHz / 100.0;
      if (freq < 0.05) freq = 0.05;
      if (freq > 0.8)  freq = 0.8;
      double z_eff = FK_Z0 + (double)g_rise_mm;
      int period_frames = (int)(1.0 / (freq * DT));
      if (period_frames < 50) period_frames = 50;
      int total = g_cycles * period_frames;
      double omega = 2.0 * M_PI * freq;
      uart_log("  阶段2 FK模式%d: α%d° β%d° %.2fHz 周期%.1fs × %d\r\n",
               mode, g_aa_deg, g_ba_deg, freq, period_frames * DT, g_cycles);
      for (i = 0; i < total; i++) {
         double t0 = omega * i * DT, t1 = omega * (i + 1) * DT;
         double a0, b0, a1, b1, lc[3], ln[3];
         get_angles(mode, t0, aa_r, ba_r, &a0, &b0);
         get_angles(mode, t1, aa_r, ba_r, &a1, &b1);
         fk_3rps(a0, b0, z_eff, lc);
         fk_3rps(a1, b1, z_eff, ln);
         for (sl = 1; sl <= ctx.slavecount; sl++) {
            double v_mms = (ln[sl - 1] - lc[sl - 1]) / DT;   /* mm/s */
            double vff = v_mms / MM_PER_PULSE;               /* pulse/s */
            write_pdo(sl, 0x000F, vel_closed(sl, vff));
         }
         cycle(); poll_cmd();
         if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
         if (g_abort) { rc = -2; goto stopmsg; }
         if (i % 500 == 0) {
            double a_now, b_now;
            get_angles(mode, t0, aa_r, ba_r, &a_now, &b_now);
            uart_log("  [%ds] α=%.1f° β=%.1f°\r\n", (int)(i * DT),
                     a_now * 180.0 / M_PI, b_now * 180.0 / M_PI);
         }
      }
   }

   /* 阶段3: 下降回原点 */
   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); }
   rc = move_ramp(-(double)raise_vel, cruise_frames);
   if (rc < 0) goto stopmsg;
   /* 末端主动闭环回精确原点 */
   for (i = 0; i < 150; i++) {
      for (sl = 1; sl <= ctx.slavecount; sl++) { cmd_pos[sl] = 0.0; write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); }
      cycle(); poll_cmd();
      if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
      if (g_abort) { rc = -2; goto stopmsg; }
   }

   uart_log(">>> 模式%d 完成, 残余误差: 轴1=%.2fmm 轴2=%.2fmm 轴3=%.2fmm <<<\r\n", mode,
            (read_pos63(1) - start_pos[1]) * MM_PER_PULSE,
            (read_pos63(2) - start_pos[2]) * MM_PER_PULSE,
            (read_pos63(3) - start_pos[3]) * MM_PER_PULSE);
   return 0;

stopmsg:
   if (rc == -2) uart_log("!! 急停, 平滑降速回待机\r\n");
   else          uart_log("!! 从站%d 报警, 平滑降速回待机\r\n", g_ec_fault);
   ramp_to_zero();
   return rc;
}

/* ================= 归零(堵转检测收回) ================= */
static void run_homing(void)
{
   int i, sl;
   int stall_cnt[EC_MAXSLAVE] = {0}, stopped[EC_MAXSLAVE] = {0};
   int32_t vel_cmd[EC_MAXSLAVE];
   g_abort = 0; g_ec_fault = 0;
   g_status = 3; g_cur_mode = 99;   /* HMI 反馈: 归零中 */

   uart_log(">>> 归零: 三轴收缩@%dpulse/s, 堵转自停 <<<\r\n", RETRACT_VEL);
   for (sl = 1; sl <= ctx.slavecount; sl++) vel_cmd[sl] = RETRACT_VEL;

   /* 缓慢加速到收缩速度 */
   for (i = 0; i < 100; i++) {
      int32_t v = (int32_t)((int64_t)RETRACT_VEL * i / 100);
      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, v);
      cycle(); poll_cmd();
      if (g_abort) { ramp_to_zero(); uart_log("!! 归零急停\r\n"); return; }
   }

   for (i = 0; i < HOME_MAX_FRAMES; i++) {
      for (sl = 1; sl <= ctx.slavecount; sl++) {
         if (stopped[sl]) continue;
         int32_t av = read_vel(sl); if (av < 0) av = -av;
         if (av < STALL_THR) {
            if (++stall_cnt[sl] >= STALL_FRAMES) {
               stopped[sl] = 1; vel_cmd[sl] = 0;
               uart_log("  ★ 轴%d 到限位, 位置=%ld\r\n", sl, (long)read_pos63(sl));
            }
         } else stall_cnt[sl] = 0;
      }
      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_cmd[sl]);
      cycle(); poll_cmd();
      if (g_abort) { ramp_to_zero(); uart_log("!! 归零急停\r\n"); return; }

      int all = 1;
      for (sl = 1; sl <= ctx.slavecount; sl++) if (!stopped[sl]) all = 0;
      if (all) { uart_log(">>> 归零完成, 所有轴到位 <<<\r\n"); return; }
   }
   uart_log(">>> 归零超时(部分轴未检测到限位) <<<\r\n");
   ramp_to_zero();
}

/* ================= EtherCAT 启动(扫描→PDO→OP→使能) ================= */
static int setup_pdo_csv(int slave)
{
   uint8 u8; uint16 u16; uint32 u32; int wkc;

   u8 = 0;
   wkc = ecx_SDOwrite(&ctx, slave, 0x1C12, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
   if (wkc <= 0) { uart_log("  [错误] 从站%d SDO写(1C12)失败\r\n", slave); return -1; }
   u8 = 0; ecx_SDOwrite(&ctx, slave, 0x1600, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
   u32 = 0x60400010; ecx_SDOwrite(&ctx, slave, 0x1600, 0x01, FALSE, 4, &u32, EC_TIMEOUTRXM);
   u32 = 0x60600008; ecx_SDOwrite(&ctx, slave, 0x1600, 0x02, FALSE, 4, &u32, EC_TIMEOUTRXM);
   u32 = 0x60FF0020; ecx_SDOwrite(&ctx, slave, 0x1600, 0x03, FALSE, 4, &u32, EC_TIMEOUTRXM);
   u8 = 3; ecx_SDOwrite(&ctx, slave, 0x1600, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
   u16 = 0x1600; ecx_SDOwrite(&ctx, slave, 0x1C12, 0x01, FALSE, 2, &u16, EC_TIMEOUTRXM);
   u8 = 1;       ecx_SDOwrite(&ctx, slave, 0x1C12, 0x00, FALSE, 1, &u8,  EC_TIMEOUTRXM);

   u8 = 0; ecx_SDOwrite(&ctx, slave, 0x1C13, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
   u8 = 0; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
   u32 = 0x60410010; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x01, FALSE, 4, &u32, EC_TIMEOUTRXM);
   u32 = 0x60610008; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x02, FALSE, 4, &u32, EC_TIMEOUTRXM);
   u32 = 0x60630020; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x03, FALSE, 4, &u32, EC_TIMEOUTRXM);
   u32 = 0x606C0020; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x04, FALSE, 4, &u32, EC_TIMEOUTRXM);
   u8 = 4; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
   u16 = 0x1A00; ecx_SDOwrite(&ctx, slave, 0x1C13, 0x01, FALSE, 2, &u16, EC_TIMEOUTRXM);
   u8 = 1;       ecx_SDOwrite(&ctx, slave, 0x1C13, 0x00, FALSE, 1, &u8,  EC_TIMEOUTRXM);
   return 0;
}

/* 平滑下电: 脱力 → 退OP→SAFE-OP→INIT, 关看门狗, 之后可安全断电/拔网线 */
static void graceful_shutdown(void)
{
   int i, sl;
   g_status = 5;   /* HMI 反馈: 已下电 */
   uart_log("下电: 脱力...\r\n");
   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0006, 0); cycle(); }
   ctx.slavelist[0].state = EC_STATE_SAFE_OP;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   ctx.slavelist[0].state = EC_STATE_INIT;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE * 4);
   ecx_close(&ctx);
   uart_log(">>> 已退到 INIT, 可安全断电/拔网线 <<<\r\n");
}

/* EtherCAT 起不来(网卡失败/没扫到从站)时的降级空转: 不返回, 仍每~4ms服务一次
 * Modbus/HMI 与串口命令, 让上位机能连上板子看到状态(从站数0、状态=故障), 心跳灯照闪。
 * 也正是"没接伺服时单测 Modbus"依赖的路径 —— 否则扫不到从站会直接退出, Modbus 一声不吭。*/
static void modbus_idle_loop(void)
{
   g_status = 4;      /* 故障/未就绪(EtherCAT 未建立) */
   g_cur_mode = 99;
   uart_log(">>> EtherCAT 未就绪, 进入 Modbus 降级空转(仍可被 HMI/电脑 连接单测) <<<\r\n");
   for (;;) {
      poll_cmd();          /* modbus_poll/sync/feedback + USART1/USB 命令 */
      heartbeat();         /* 心跳灯照闪, 表明没死 */
      osal_usleep(CYC_US); /* 无 EtherCAT, 相对延时凑 ~4ms 节拍即可 */
   }
}

void ecat_motion_run(void)
{
   int sl, i;

   uart_log_init();
   modbus_init();   /* USART3 上的 Modbus RTU 从站(HMI 用), 与串口命令并行 */
   uart_log("\r\n\r\n===== STM32 SOEM 康复运动 (CSV, 命令驱动) =====\r\n");

   uart_log("上次复位原因:");
   if (g_reset_cause & RCC_CSR_PORRSTF)  uart_log(" 上电复位(POR/PDR)");
   if (g_reset_cause & RCC_CSR_PADRSTF)  uart_log(" 外部NRST引脚复位(硬件拉低, 疑似DTR)");
   if (g_reset_cause & RCC_CSR_SFTRSTF)  uart_log(" 软件复位");
   if (g_reset_cause & RCC_CSR_IWDGRSTF) uart_log(" 独立看门狗复位");
   if (g_reset_cause & RCC_CSR_WWDGRSTF) uart_log(" 窗口看门狗复位");
   if (g_reset_cause & RCC_CSR_LPWRRSTF) uart_log(" 低功耗复位");
   uart_log("  (CSR=0x%08lX)\r\n", (unsigned long)g_reset_cause);

   g_ec_phase = 1;
   if (!ecx_init(&ctx, "stm32eth")) {
      uart_log("[错误] 网卡初始化失败\r\n"); g_ec_phase = -1; modbus_idle_loop();
   }
   uart_log("网卡就绪, 扫描总线...\r\n");

   g_ec_phase = 2;
   if (ecx_config_init(&ctx) <= 0) {
      uart_log("[错误] 没扫到从站! 检查网线/伺服上电\r\n"); g_ec_phase = -2; ecx_close(&ctx); modbus_idle_loop();
   }
   g_ec_slavecount = ctx.slavecount;
   uart_log(">>> 扫到 %d 个从站 <<<\r\n", ctx.slavecount);
   for (sl = 1; sl <= ctx.slavecount; sl++) uart_log("    从站%d: %s\r\n", sl, ctx.slavelist[sl].name);
   if (ctx.slavecount < 1) { g_ec_phase = -2; ecx_close(&ctx); modbus_idle_loop(); }

   /* PRE-OP: 配 CSV 的 PDO */
   g_ec_phase = 3;
   ctx.slavelist[0].state = EC_STATE_PRE_OP;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
   ecx_readstate(&ctx);   /* 同步各从站本地状态, 否则SDO写静默失败 */
   for (sl = 1; sl <= ctx.slavecount; sl++)
      if (setup_pdo_csv(sl) < 0) { g_ec_phase = -3; ecx_close(&ctx); return; }

   ecx_config_map_group(&ctx, IOmap, 0);

   /* SAFE-OP → OP */
   g_ec_phase = 4;
   ctx.slavelist[0].state = EC_STATE_SAFE_OP;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0000, 0); cycle(); }

   ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
   ecx_writestate(&ctx, 0);
   for (i = 0; i < 200; i++) {
      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0000, 0);
      if (i % 40 == 0) ecx_writestate(&ctx, 0);
      cycle();
   }
   ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
   uart_log("OP 建立, WKC=%d (正常≈%d)\r\n", g_wkc, ctx.slavecount * 3);

   /* CiA402 使能: 0x06 → 0x07 → 0x0F */
   g_ec_phase = 5;
   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0006, 0); cycle(); }
   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0007, 0); cycle(); }
   for (i = 0; i < 300; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0); cycle(); }
   for (sl = 1; sl <= ctx.slavecount; sl++)
      uart_log("    从站%d 状态字=0x%04X %s\r\n", sl, read_sw(sl),
               (read_sw(sl) & 0x0008) ? "[报警!]" : (((read_sw(sl) & 0x006F) == 0x0027) ? "[已使能]" : ""));

   /* ===== 待机: 听命令 ===== */
   g_ec_phase = 99;
   for (sl = 1; sl <= ctx.slavecount; sl++) start_pos[sl] = read_pos63(sl);  /* 反馈位置以此刻为0基准, 避免开机显示绝对编码值 */
   uart_log("\r\n>>> 进入待机, 伺服使能保持零速. 发 ? 查看命令. 建议先 h 归零 <<<\r\n");
   while (!g_quit) {
      g_status = g_ec_fault ? 4 : 1;   /* HMI 反馈: 故障未清则4, 否则待机1 */
      g_cur_mode = 99;
      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0);
      cycle();
      poll_cmd();

      if (g_do_home)          { g_do_home = 0; g_run_request = -1; run_homing(); uart_log(">>> 回待机 <<<\r\n"); }
      else if (g_run_request >= 0) { int m = g_run_request; g_run_request = -1; g_do_home = 0; run_rehab_mode(m); uart_log(">>> 回待机 <<<\r\n"); }
   }

   /* 收到 q: 平滑下电退出 */
   graceful_shutdown();
   while (1) { osal_usleep(100000); }
}
