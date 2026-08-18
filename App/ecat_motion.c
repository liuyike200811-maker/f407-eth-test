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
#include "sensor_wt.h"
#include "log_screen.h"
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
/* 堵转判断(位置法, 比读瞬时速度稳, 与 Linux retract_all 对齐):
 * 维护去抖窗口, 窗口内累计位移 > STALL_WIN_EPS 说明还在往里走→重置; 连续
 * STALL_FRAMES 帧(75×4ms=300ms)位置几乎不动才判到位。避免中途机械紧点让速度
 * 瞬间掉到阈值下被误判成"到位"而提前停(杆没收到底); 且"本就在限位"也能正确判到。*/
#define STALL_WIN_EPS  1500         /* 一个窗口内累计位移小于此值(pulse)=没在动 */
#define STALL_FRAMES   75           /* 连续多少帧几乎不动才判到位(≈300ms) */
#define HOME_MAX_FRAMES 7500        /* 归零超时(≈30s) */

/* ---- 测试专用: 传感器独立测试(不接EtherCAT/电缸, 仅读传感器→micro-USB打印) ----
 * 1=启用: ecat_motion_run() 跳过EtherCAT扫描和所有电缸相关代码, 只读传感器打印。
 * 0=默认: 正常产线行为, 不受影响。排查完记得改回0重新编译烧录, 不要带着1发布。 */
#define SENSOR_STANDALONE_TEST 0

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

/* ---- 传感器实时跟随模式(WT901 姿态→踝角→电缸) ---- */
/* 平滑在【控制周期250Hz】上跑, 不再只在收到帧(~50Hz)时跳一步 —— 否则前馈速度
 * =Δ杆长/4ms 只在收帧那一帧非零, 变成50Hz尖峰串, 被限加速削平后平台几乎不动。
 *
 * ★ 喂入端安全闸(配合 sensor_wt.c 解析层的锁源/范围/尖峰三层校验):
 *   - 增益 1:1 : 原 GAIN=3.0 把手轻微一动放大成大幅倾斜, 是"极不安全"的根因;
 *     康复设备不应放大动作, 平台跟随幅度 ≤ 手部实际倾角(勿设 >1)。
 *   - 角速度硬限幅: 平台目标角每秒最多转 SENSOR_MAX_RATE_DPS 度, 独立于伺服加速度
 *     限幅的角度层物理保护 —— 任何异常输入都不会让平台猛倾。设在"正常跟随峰值之上"
 *     (正常踝运动峰速≈30°/s), 平时不介入, 只封异常; 异常最多持续到下一帧(≈20ms)即被
 *     校验层纠正, 故单次越界幅度极小(≤2~3°)。
 *
 * ★ 抗抖: One-Euro 自适应滤波(静止→截止降到 MINCUT 重滤波消抖; 运动→按角速度抬高截止
 *   轻滤波保跟手), 一举破解定点 EMA"调稳则滞后/调灵则抖"的死结。台架调法:
 *     还抖  → 降 MINCUT(如 0.6) 或 增大 DEADBAND;   跟手发滞后 → 增大 BETA(如 12~20)。 */
#define SENSOR_GAIN          1.0    /* 角度增益: 手倾角 ×1 = 平台目标角。⚠不要 >1 */
#define SENSOR_MAX_DEG       30.0   /* 跟随角度包络(度): 限幅, 防逆解出超程杆长(与协议文档一致) */
#define SENSOR_MAX_RATE_DPS  40.0   /* 平台目标角最大变化率(度/秒): 安全硬限幅(设在正常峰值之上) */
#define SENSOR_DEADBAND_DEG  0.3    /* 期望角死区(度): 静止时彻底冻结, 消除微抖 */
#define SENSOR_EURO_MINCUT   1.0    /* One-Euro 最小截止频率(Hz): 越低静止越稳, 略增延迟 */
#define SENSOR_EURO_BETA     8.0    /* One-Euro 速度系数: 越大运动时越跟手(延迟越小) */
#define SENSOR_EURO_DCUT     1.0    /* One-Euro 导数低通截止(Hz): 抑制速度估计噪声, 常取 1.0 */
#define SENSOR_WATCHDOG_FRAMES 75   /* 信号丢失>300ms: 冻结保持当前姿态(短暂丢帧不猛动) */
#define SENSOR_NEUTRAL_FRAMES  500  /* 信号丢失>2s: 判链路真断, 缓慢(限速)归平到水平最安全姿态 */

/* ---- 传感器→平台 方向符号(台架标定核对; 见 fk_3rps 的世界系约定) ----
 * 世界系(实测物理布局): +X=1号(正对), +Y偏2号那一侧(3轴120°均布, 不精确对准任一缸),
 * +Z向上, 右手系。传感器 +X 对准1号、水平放置标定。
 * 平台【复现】手的姿态(非镜像): 手向+Y(2号)侧抬→平台+Y(2号)侧抬; 手前端(+X/1号侧)
 * 下沉→平台+X侧下沉。核对: 进跟随后单轴慢慢倾手, 平台若反向, 把对应符号取反即可(仅一处)。
 * (旧版 α 靠"镜像几何×负号"两错抵消而恰好对, β 却是反的; 几何修正后两轴统一取 +1 才对。)*/
#define SENSOR_SIGN_ALPHA   (+1.0)  /* Roll→α: 反了改 -1.0 */
#define SENSOR_SIGN_BETA    (+1.0)  /* Pitch→β: 反了改 -1.0 */

/* 标定: 采集足够多"有效帧"求零偏, 并检查离散度(没放稳/数据乱则拒绝, 避免带病零点) */
#define SENSOR_CAL_MIN_SAMPLES 50   /* 标定所需最少有效帧(≈1s @50Hz) */
#define SENSOR_CAL_MAX_FRAMES  1000 /* 标定最长等待(≈4s), 含解析层锁源观测时间 */
#define SENSOR_CAL_MAX_STDDEV  3.0  /* 标定期角度标准差上限(度): 超了说明没放稳/数据乱 */

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
static volatile int g_do_sensor   = 0;   /* 进入传感器实时跟随模式 */
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

/* mode(0~4) → 屏显日志短语id, 給log_screen_event()用 */
static log_phrase_id_t mode_start_phrase(int mode)
{
   switch (mode) {
      case 0:  return LOGPH_EV_M0_START;
      case 1:  return LOGPH_EV_M1_START;
      case 2:  return LOGPH_EV_M2_START;
      case 3:  return LOGPH_EV_M3_START;
      default: return LOGPH_EV_M4_START;
   }
}
static log_phrase_id_t mode_done_phrase(int mode)
{
   switch (mode) {
      case 0:  return LOGPH_EV_M0_DONE;
      case 1:  return LOGPH_EV_M1_DONE;
      case 2:  return LOGPH_EV_M2_DONE;
      case 3:  return LOGPH_EV_M3_DONE;
      default: return LOGPH_EV_M4_DONE;
   }
}
/* 从站号(1~3) → 报警短语id; EC_MAXSLAVE虽然是16, 但实际只装了3个伺服 */
static log_phrase_id_t slave_fault_phrase(int slave)
{
   switch (slave) {
      case 1:  return LOGPH_EV_SLV1;
      case 2:  return LOGPH_EV_SLV2;
      default: return LOGPH_EV_SLV3;
   }
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
         uart_log("命令: ? 帮助 | h归零 | g<0-4>运行模式 | s传感器跟随 | x急停 | q下电退出\r\n"
                  "      a<度>α b<度>β f<厘赫>频率 w<mm>波幅 r<mm>升高 c<次>循环\r\n"
                  "参数: mode默认由g指定  α=%d° β=%d° 频率=%.2fHz 波幅=%dmm 升高=%dmm 循环=%d\r\n"
                  "模式: 0波浪 1跖屈背伸 2内翻外翻 3环形 4八字 (5=归零用h) s=传感器实时跟随\r\n",
                  g_aa_deg, g_ba_deg, g_freq_cHz / 100.0, g_wave_mm, g_rise_mm, g_cycles);
         break;
      case 'h': g_do_home = 1; break;
      case 's': g_do_sensor = 1; break;
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

   /* 模式0~4 / 归零 / 传感器跟随: 仅待机受理; 运行中收到则清掉不执行(防中途乱切, 见契约) */
   if (g_status == 1) {
      int req = -1;
      for (int m = 0; m < 5; m++)
         if (modbus_coils[m]) { if (req < 0) req = m; modbus_coils[m] = 0; }  /* 多个同ON以先到为准 */
      int sensor_req = modbus_coils[9]; modbus_coils[9] = 0;                  /* 0x0009 传感器跟随 */
      if (modbus_coils[5])   { modbus_coils[5] = 0; g_do_home = 1; }          /* 归零优先 */
      else if (sensor_req)   { g_do_sensor = 1; }
      else if (req >= 0)     { g_run_request = req; }
   } else {
      for (int m = 0; m <= 5; m++) modbus_coils[m] = 0;
      modbus_coils[9] = 0;
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

/* ================= 数字孪生遥测帧 (micro-USB / USB CDC) ================= */
/* 每控制周期往 USB CDC(micro-USB) 发一帧二进制遥测, 供电脑转发给局域网数字孪生。
 * 格式沿用旧 Linux 版 UDP 帧, 仅在最前面加 2 字节帧头 0xAA 0x55 便于电脑在
 * 字节流里定位(同一路 CDC 还混着 uart_log 的可读文本, 电脑遇 0xAA55 按固定长读一帧):
 *
 *   偏移  长度  字段
 *   0     2    帧头 0xAA 0x55
 *   2     4    seq     (uint32 小端, 每帧+1, 电脑据此判丢帧)
 *   6     1    phase   (uint8, = 当前模式 g_cur_mode: 0~4 康复模式 / 6 传感器跟随 / 99 待机)
 *   7     12   pos[3]  (float 小端 ×3, 各电缸相对 home 的伸出量, mm, 真实编码器 6063)
 *   19    12   vel[3]  (float 小端 ×3, 各电缸实际速度, RPM, 606C)
 *   31    1    xor     (前面 29 个载荷字节[偏移2~30]的逐字节异或, 供电脑校验/排除误同步)
 *   共 32 字节。载荷 29 字节(偏移2~30)的内部布局与旧 webui server.py 完全一致(不改内部结构),
 *   帧头 0xAA55 与尾部 xor 是外包装: 电脑扫到 0xAA55 → 读定长 32B → 核对 xor, 过了才是真帧。
 *
 * CDC_Transmit_FS 非阻塞: USB 忙/没插线直接丢弃本帧(尽力而为, seq 让电脑能发现空洞),
 * 不会阻塞 4ms 控制节拍。 */
static void twin_emit(void)
{
   static uint32_t seq = 0;
   uint8_t buf[32];
   float pos[3], vel[3];

   for (int sl = 1; sl <= 3; sl++) {
      int32_t dp = (ctx.slavecount >= sl) ? (read_pos63(sl) - start_pos[sl]) : 0;
      int32_t rv = (ctx.slavecount >= sl) ? read_vel(sl) : 0;
      pos[sl - 1] = (float)((double)dp * MM_PER_PULSE);         /* mm, 从 home 起 */
      vel[sl - 1] = (float)((double)rv / PULSE_PER_REV * 60.0); /* pulse/s → RPM */
   }

   seq++;
   buf[0] = 0xAA; buf[1] = 0x55;
   memcpy(buf + 2, &seq, 4);
   buf[6] = (uint8_t)g_cur_mode;
   memcpy(buf + 7,  pos, 12);
   memcpy(buf + 19, vel, 12);
   uint8_t xr = 0;
   for (int i = 2; i < 31; i++) xr ^= buf[i];   /* 异或校验: 载荷 29 字节 */
   buf[31] = xr;
   CDC_Transmit_FS(buf, sizeof buf);
}

/* 每控制周期往 USART6 TX(PC6) 回一个状态字节:
 * '1'=有效帧, 否则发 sensor_fail_reason ('L'长度/'H'头/'T'尾/'V'版本/'0'无帧)。 */
static void sensor_status_tx(void)
{
   static uint32_t last_valid = 0;
   uint32_t v = sensor_stat_valid;
   sensor_tx_byte((v != last_valid) ? '1' : (uint8_t)sensor_fail_reason);
   last_valid = v;
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
   sensor_status_tx();       /* → PC6: 传感器接收状态 0/1 */
   twin_emit();              /* → micro-USB: 数字孪生遥测帧 */
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

/* ================= 3-RPS 运动学: 位姿(α,β,z) → 三杆长 ================= */
/* 命名提醒: 并联机构里"位姿→杆长"严格是【逆解 IK】(简单/闭式方向);
 *   "杆长→位姿"才是正解 FK(难, 多解, 本工程未做也用不到)。
 *   函数名 fk_ 系历史沿用, 勿被"fk"字样误导成正解。
 *
 * ★ 世界坐标(实测物理布局, 俯视/看向 +Z, 右手系):  +X 精确指向 1号;  +Y 偏【2号】
 *   那一侧(3轴120°均布, 不精确对准任一缸); +Z 竖直向上。故三作动器真实角位置:
 *   1号=0°,  2号=+120°(在 +Y 半平面),  3号=-120°(在 -Y 半平面)。
 *   α=Roll(绕X, 内翻外翻), β=Pitch(绕Y, 跖屈背伸)。
 *   A[i]/B[i] 必须是 slave(i+1) 的真实位置(l_out[i] 直接发给 slave i+1)。
 * R_base=200 r_mov=140, 120°均布。 */
static void fk_3rps(double alpha, double beta, double z_eff, double l_out[3])
{
   static const double A[3][2] = {          /* 基座锚点(定平台), 单位 mm */
      { 200.0,   0.0        },              /* 1号 @  0°  (+X) */
      {-100.0,  173.2050808 },              /* 2号 @+120° (+Y 半平面, 即 +Y 偏2号侧) */
      {-100.0, -173.2050808 }               /* 3号 @-120° (-Y 半平面) */
   };
   static const double B[3][2] = {          /* 动平台锚点(随位姿旋转), 单位 mm */
      { 140.0,   0.0        },              /* 1号 @  0° */
      { -70.0,  121.2435565 },              /* 2号 @+120° */
      { -70.0, -121.2435565 }               /* 3号 @-120° */
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
      log_screen_service();
      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
      if (g_abort) return -2;
   }
   /* 匀速 */
   for (i = 0; i < cruise_frames; i++) {
      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, raise_vel));
      cycle(); poll_cmd();
      log_screen_service();
      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
      if (g_abort) return -2;
   }
   /* 减速 */
   for (i = 0; i < RAMP_FRAMES; i++) {
      double vff = raise_vel * (RAMP_FRAMES - i) / RAMP_FRAMES;
      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, vff));
      cycle(); poll_cmd();
      log_screen_service();
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
   log_screen_event(mode_start_phrase(mode), 0);

   /* 阶段1: 上升 */
   log_screen_set_state(LOGPH_ST_RISE);
   rc = move_ramp((double)raise_vel, cruise_frames);
   if (rc < 0) goto stopmsg;
   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); log_screen_service(); }
   uart_log("  上升完成, 方向锁定: 轴1=%+d 轴2=%+d 轴3=%+d\r\n", fb_sign[1], fb_sign[2], fb_sign[3]);

   /* 阶段2: 运动 */
   log_screen_set_state(LOGPH_ST_RUN);
   if (mode == 0) {
      /* 综合波浪(电缸空间三轴相位差) */
      int32_t wave_amp = (int32_t)((double)g_wave_mm * PULSE_PER_REV / LEAD_MM);
      int wave_period = (int)((double)wave_amp * 2.0 * M_PI / (WAVE_PEAK_RPM / 60.0 * PULSE_PER_REV) / DT);
      if (wave_period < 100) wave_period = 100;
      double wave_v_amp = (double)wave_amp * 2.0 * M_PI / wave_period / DT;
      int total = g_cycles * wave_period;
      uart_log("  阶段2 综合波浪: 幅%dmm 周期%d帧 × %d\r\n", g_wave_mm, wave_period, g_cycles);
      for (i = 0; i < total; i++) {
         double t = 2.0 * M_PI * i / wave_period;
         for (sl = 1; sl <= ctx.slavecount; sl++) {
            double vff = wave_v_amp * cos(t + PHASE[sl]);
            write_pdo(sl, 0x000F, vel_closed(sl, vff));
         }
         cycle(); poll_cmd();
         log_screen_service();   /* 内部2秒节流+每次最多画一行, 跟uart_log那种无节流阻塞是两回事, 可以放这里 */
         if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
         if (g_abort) { rc = -2; goto stopmsg; }
         /* 运动中绝不打印: uart_log 是阻塞式串口发送(每字符忙等~87us, 一行~4ms),
            会把 4ms 控制周期撑到十几ms → free-run 伺服顿挫。统计留到运动结束汇报。 */
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
      uart_log("  阶段2 FK模式%d: α%d° β%d° 频%d厘赫 周期%d帧 × %d\r\n",
               mode, g_aa_deg, g_ba_deg, g_freq_cHz, period_frames, g_cycles);
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
         log_screen_service();
         if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
         if (g_abort) { rc = -2; goto stopmsg; }
         /* 运动中绝不打印(同上, 避免 uart_log 阻塞撑破 4ms 节拍) */
      }
   }

   /* 阶段3: 下降回原点 */
   log_screen_set_state(LOGPH_ST_FALL);
   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); log_screen_service(); }
   rc = move_ramp(-(double)raise_vel, cruise_frames);
   if (rc < 0) goto stopmsg;
   /* 末端主动闭环回精确原点 */
   for (i = 0; i < 150; i++) {
      for (sl = 1; sl <= ctx.slavecount; sl++) { cmd_pos[sl] = 0.0; write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); }
      cycle(); poll_cmd();
      log_screen_service();
      if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
      if (g_abort) { rc = -2; goto stopmsg; }
   }

   uart_log(">>> 模式%d 完成, 残余误差: 轴1=%.2fmm 轴2=%.2fmm 轴3=%.2fmm <<<\r\n", mode,
            (read_pos63(1) - start_pos[1]) * MM_PER_PULSE,
            (read_pos63(2) - start_pos[2]) * MM_PER_PULSE,
            (read_pos63(3) - start_pos[3]) * MM_PER_PULSE);
   log_screen_event(mode_done_phrase(mode), 0);
   log_screen_set_state(LOGPH_ST_IDLE);
   return 0;

stopmsg:
   if (rc == -2) {
      uart_log("!! 急停, 平滑降速回待机\r\n");
      log_screen_event(LOGPH_EV_ESTOP, 1);
      log_screen_set_state(LOGPH_ST_IDLE);
   } else {
      uart_log("!! 从站%d 报警, 平滑降速回待机\r\n", g_ec_fault);
      log_screen_event(slave_fault_phrase(g_ec_fault), 1);
      log_screen_set_state(LOGPH_ST_FAULT);
   }
   ramp_to_zero();
   return rc;
}

/* ================= One-Euro 自适应滤波(抗抖核心) =================
 * 手持跟随的抖动/延迟本质是同一权衡: 定点低通调稳则滞后, 调灵则抖。One-Euro 让截止
 * 频率随信号角速度自适应 —— 静止时截止降到 MINCUT(重滤波, 抖动被压掉), 运动时按角速度
 * 抬高截止(轻滤波, 跟手不滞后)。这里在【控制周期 250Hz, dt=DT】上跑, 输入是"传感器
 * 设定角(帧间 ZOH 保持)", 输出直接作为平台目标角, 并在最后叠一道角速度硬限幅做安全兜底。*/
static double one_euro_alpha(double fc, double dt)
{
   double tau = 1.0 / (2.0 * M_PI * fc);
   return 1.0 / (1.0 + tau / dt);
}
typedef struct { double xf, dxf, xprev; int init; } euro_t;
/* 单步滤波 + 角速度硬限幅。x=设定角(rad), 返回滤波并限幅后的平台目标角(rad)。 */
static double euro_step(euro_t *s, double x, double dt, double max_step)
{
   if (!s->init) { s->xf = x; s->dxf = 0.0; s->xprev = x; s->init = 1; return s->xf; }
   double dxdt = (x - s->xprev) / dt;
   s->xprev = x;
   double ad = one_euro_alpha(SENSOR_EURO_DCUT, dt);
   s->dxf += ad * (dxdt - s->dxf);                          /* 平滑后的角速度估计 */
   double cutoff = SENSOR_EURO_MINCUT + SENSOR_EURO_BETA * fabs(s->dxf);
   double a = one_euro_alpha(cutoff, dt);
   double d = a * (x - s->xf);                              /* 本周期滤波步进 */
   if (d >  max_step) d =  max_step;                        /* 角速度硬限幅(安全兜底) */
   if (d < -max_step) d = -max_step;
   s->xf += d;
   return s->xf;
}

/* ================= 传感器实时跟随 (上升→标定→跟随→归平下降) =================
 * 数据流(每一步都是安全闸): 解析层(sensor_wt)已交出【6轴融合(加速度+陀螺, 含加速度剔除)】后的
 * 原始角 → 减零偏 → ×符号×增益(1:1) → 角度包络限幅 → 死区 → One-Euro自适应滤波 +
 * 角速度硬限幅得平台目标角 → fk_3rps 逆算三电缸目标长 → 相邻帧差分得前馈速度 → vel_closed。
 *
 * 三档信号丢失保护:
 *   ≤300ms  正常帧间保持(不动)
 *   >300ms  冻结当前姿态(短暂丢帧, 不猛动)
 *   >2s     判链路真断 → 目标缓慢(仍受角速度限幅)归平到水平, 患者脚回到最安全位。
 * x 急停随时退出。 */
static int run_sensor_mode(void)
{
   int i, sl, rc = 0;
   double ta = 0.0, tb = 0.0;          /* 平台实际输出目标角(rad); 退出归平复用 */
   double want_a = 0.0, want_b = 0.0;  /* 传感器设定角(rad, 经增益/限幅/死区), 帧间保持 */
   double l_prev[3];                   /* 上一帧 FK 目标杆长(mm), 用于差分求速度 */
   float  r, p;

   g_abort = 0; g_ec_fault = 0;
   g_status = 2; g_cur_mode = 6;   /* HMI 反馈: 运行中 + 模式6(传感器跟随) */
   motion_reset();
   sensor_reset();                 /* 清解析层跳变历史 + 解锁数据源, 保证标定/跟随同源 */

   int32_t raise_vel  = (int32_t)((double)g_rise_rpm / 60.0 * PULSE_PER_REV);
   int32_t rise_pulse = (int32_t)((double)g_rise_mm * PULSE_PER_REV / LEAD_MM);
   int cruise_frames  = (int)((double)rise_pulse / ((double)raise_vel * DT));
   if (cruise_frames < 1) cruise_frames = 1;
   double z_eff  = FK_Z0 + (double)g_rise_mm;
   double ang_lim = SENSOR_MAX_DEG * M_PI / 180.0;
   double max_step = SENSOR_MAX_RATE_DPS * M_PI / 180.0 * DT;   /* 每控制周期目标角最大步进(rad) */
   fk_3rps(0.0, 0.0, z_eff, l_prev);   /* 基线=标定姿态(零倾斜), 提前初始化防 goto 跳过 */

   uart_log(">>> 传感器跟随: 上升%dmm → 零偏标定 → 实时跟随(发 x 退出) <<<\r\n", g_rise_mm);
   log_screen_event(LOGPH_EV_SF_START, 0);

   /* 阶段1: 上升到工作高度(与其它模式一致) */
   log_screen_set_state(LOGPH_ST_RISE);
   rc = move_ramp((double)raise_vel, cruise_frames);
   if (rc < 0) goto stopmsg;
   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); log_screen_service(); }
   uart_log("  上升完成, 方向锁定: 轴1=%+d 轴2=%+d 轴3=%+d\r\n", fb_sign[1], fb_sign[2], fb_sign[3]);

   /* 阶段2: 零偏标定 —— 平台不动, 采集足够多【有效帧】求均值+离散度
    * (解析层前几帧在锁源观测, 会返回0, 所以按"有效帧数"而非"控制帧数"计, 且给足超时) */
   uart_log("  标定中(请把脚踝放到中位并保持不动)...\r\n");
   log_screen_event(LOGPH_EV_CAL_ING, 0);
   double sum_r = 0.0, sum_p = 0.0, sum_r2 = 0.0, sum_p2 = 0.0; int ncal = 0;
   for (i = 0; i < SENSOR_CAL_MAX_FRAMES && ncal < SENSOR_CAL_MIN_SAMPLES; i++) {
      if (sensor_get(&r, &p)) {
         sum_r += r; sum_p += p;
         sum_r2 += (double)r * r; sum_p2 += (double)p * p; ncal++;
      }
      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0));
      cycle(); poll_cmd();
      log_screen_service();
      if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
      if (g_abort)     { rc = -2; goto stopmsg; }
   }
   if (ncal < SENSOR_CAL_MIN_SAMPLES) {   /* 有效帧不够 → 链路没通/数据全被校验丢弃, 安全退出 */
      uart_log("!! 标定失败: 仅%d/%d 有效帧, 检查USART6(PC7)/转发/波特率/传感器. 退出.\r\n",
               ncal, SENSOR_CAL_MIN_SAMPLES);
      log_screen_event(LOGPH_EV_CAL_FAIL, 1);
      rc = -2; goto stopmsg;
   }
   double roll0 = sum_r / ncal, pitch0 = sum_p / ncal;
   double var_r = sum_r2 / ncal - roll0 * roll0;   if (var_r < 0) var_r = 0;
   double var_p = sum_p2 / ncal - pitch0 * pitch0; if (var_p < 0) var_p = 0;
   double sd_r = sqrt(var_r), sd_p = sqrt(var_p);
   if (sd_r > SENSOR_CAL_MAX_STDDEV || sd_p > SENSOR_CAL_MAX_STDDEV) {   /* 没放稳/数据跳→零点不可信 */
      uart_log("!! 标定失败: 数据不稳(σR=%.2f σP=%.2f°>%.1f), 请放稳后重试. 退出.\r\n",
               sd_r, sd_p, SENSOR_CAL_MAX_STDDEV);
      log_screen_event(LOGPH_EV_CAL_FAIL, 1);
      rc = -2; goto stopmsg;
   }
   uart_log("  标定完成(%d帧): Roll0=%.2f° Pitch0=%.2f° σ(%.2f,%.2f) 源=6轴融合. 开始跟随.\r\n",
            ncal, roll0, pitch0, sd_r, sd_p);
   log_screen_event(LOGPH_EV_CAL_OK, 0);
   log_screen_set_state(LOGPH_ST_RUN);

   /* 阶段3: 实时跟随
    * want_a/want_b 只在收到有效帧时刷新(经符号/增益/限幅/死区), 帧间 ZOH 保持;
    * ta/tb = One-Euro(want) 每控制周期更新: 静止重滤波消抖、运动轻滤波跟手, 末端再叠
    * 角速度硬限幅 max_step —— 无论 want 怎么变, 平台角每周期最多挪 max_step, 不会猛倾。 */
   const double dead = SENSOR_DEADBAND_DEG * M_PI / 180.0;
   euro_t ea = {0}, eb = {0};
   int lost = 0;
   for (;;) {
      if (sensor_get(&r, &p)) {
         lost = 0;
         double ar = SENSOR_SIGN_ALPHA * SENSOR_GAIN * ((double)r - roll0)  * M_PI / 180.0;  /* Roll → α(内翻外翻) */
         double br = SENSOR_SIGN_BETA  * SENSOR_GAIN * ((double)p - pitch0) * M_PI / 180.0;  /* Pitch → β(跖屈背伸) */
         if (ar >  ang_lim) ar =  ang_lim;                  /* 角度包络限幅(防逆解超程杆长) */
         if (ar < -ang_lim) ar = -ang_lim;
         if (br >  ang_lim) br =  ang_lim;
         if (br < -ang_lim) br = -ang_lim;
         if (fabs(ar - want_a) > dead) want_a = ar;         /* 死区: 静止时冻结, 消除微抖 */
         if (fabs(br - want_b) > dead) want_b = br;
      } else {
         /* 无新帧: 分档处理。>2s 判链路真断, 设定缓慢归平到水平(最安全); 否则冻结保持。 */
         if (lost < SENSOR_NEUTRAL_FRAMES) lost++;
         if (lost >= SENSOR_NEUTRAL_FRAMES) { want_a = 0.0; want_b = 0.0; }
         else if (lost == SENSOR_WATCHDOG_FRAMES) {
            uart_log("  [看门狗] 信号丢失>300ms, 冻结保持姿态\r\n");
            log_screen_event(LOGPH_EV_SIGLOST, 1);
         }
      }

      /* One-Euro 自适应滤波(内含角速度硬限幅) → 平台目标角 */
      ta = euro_step(&ea, want_a, DT, max_step);
      tb = euro_step(&eb, want_b, DT, max_step);

      double l_now[3];
      fk_3rps(ta, tb, z_eff, l_now);
      for (sl = 1; sl <= ctx.slavecount; sl++) {
         double v_mms = (l_now[sl - 1] - l_prev[sl - 1]) / DT;   /* mm/s */
         double vff   = v_mms / MM_PER_PULSE;                    /* pulse/s */
         write_pdo(sl, 0x000F, vel_closed(sl, vff));
         l_prev[sl - 1] = l_now[sl - 1];
      }
      cycle(); poll_cmd();
      log_screen_service();
      if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
      if (g_abort)     { rc = -2; goto stopmsg; }   /* x = 退出跟随 */
   }

stopmsg:
   if (rc == -1) {   /* 从站报警: 不再主动移动, 就地降速回待机 */
      uart_log("!! 从站%d 报警, 平滑降速回待机\r\n", g_ec_fault);
      log_screen_event(slave_fault_phrase(g_ec_fault), 1);
      log_screen_set_state(LOGPH_ST_FAULT);
      ramp_to_zero();
      return rc;
   }
   /* rc==-2(用户 x 退出 / 标定失败): 先把平台归平, 再下降回原点 */
   uart_log("!! 退出跟随: 归平 → 下降回原点\r\n");
   log_screen_set_state(LOGPH_ST_FALL);
   g_abort = 0;                       /* 清急停, 让后续下降动作能执行 */
   for (i = 0; i < 400; i++) {        /* 目标角指数衰减到 0, 平滑归平 */
      ta *= 0.98; tb *= 0.98;
      double l_now[3];
      fk_3rps(ta, tb, z_eff, l_now);
      for (sl = 1; sl <= ctx.slavecount; sl++) {
         double vff = (l_now[sl - 1] - l_prev[sl - 1]) / DT / MM_PER_PULSE;
         write_pdo(sl, 0x000F, vel_closed(sl, vff));
         l_prev[sl - 1] = l_now[sl - 1];
      }
      cycle(); poll_cmd();
      log_screen_service();
      if (any_fault()) { g_ec_fault = any_fault(); ramp_to_zero(); return -1; }
   }
   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); log_screen_service(); }
   move_ramp(-(double)raise_vel, cruise_frames);
   for (i = 0; i < 150; i++) {        /* 末端主动闭环回精确原点 */
      for (sl = 1; sl <= ctx.slavecount; sl++) { cmd_pos[sl] = 0.0; write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); }
      cycle(); poll_cmd();
      log_screen_service();
   }
   uart_log(">>> 跟随结束, 残余误差: 轴1=%.2fmm 轴2=%.2fmm 轴3=%.2fmm <<<\r\n",
            (read_pos63(1) - start_pos[1]) * MM_PER_PULSE,
            (read_pos63(2) - start_pos[2]) * MM_PER_PULSE,
            (read_pos63(3) - start_pos[3]) * MM_PER_PULSE);
   log_screen_event(LOGPH_EV_SF_DONE, 0);
   log_screen_set_state(LOGPH_ST_IDLE);
   return rc;
}

/* ================= 归零(堵转检测收回) ================= */
static void run_homing(void)
{
   int i, sl;
   int stall_cnt[EC_MAXSLAVE] = {0}, stopped[EC_MAXSLAVE] = {0};
   int32_t vel_cmd[EC_MAXSLAVE];
   int32_t pos_win[EC_MAXSLAVE];    /* 每轴去抖窗口起点位置(位置法堵转检测用) */
   g_abort = 0; g_ec_fault = 0;
   g_status = 3; g_cur_mode = 99;   /* HMI 反馈: 归零中 */

   uart_log(">>> 归零: 三轴收缩@%dpulse/s, 堵转自停 <<<\r\n", RETRACT_VEL);
   log_screen_event(LOGPH_EV_HM_START, 0);
   log_screen_set_state(LOGPH_ST_HOMING);
   for (sl = 1; sl <= ctx.slavecount; sl++) vel_cmd[sl] = RETRACT_VEL;

   /* 缓慢加速到收缩速度 */
   for (i = 0; i < 100; i++) {
      int32_t v = (int32_t)((int64_t)RETRACT_VEL * i / 100);
      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, v);
      cycle(); poll_cmd();
      log_screen_service();
      if (g_abort) {
         ramp_to_zero(); uart_log("!! 归零急停\r\n");
         log_screen_event(LOGPH_EV_ESTOP, 1); log_screen_set_state(LOGPH_ST_IDLE);
         return;
      }
   }

   for (sl = 1; sl <= ctx.slavecount; sl++) pos_win[sl] = read_pos63(sl);   /* 检测起点 */

   for (i = 0; i < HOME_MAX_FRAMES; i++) {
      for (sl = 1; sl <= ctx.slavecount; sl++) {
         if (stopped[sl]) continue;
         /* 位置法: 看窗口里位置还动不动, 不看瞬时速度。累计位移够大→还在收缩,
            窗口前移并重置; 位置几乎不动且连续够久→判定顶到机械限位。 */
         int32_t pos = read_pos63(sl);
         int32_t moved = pos - pos_win[sl]; if (moved < 0) moved = -moved;
         if (moved > STALL_WIN_EPS) {
            pos_win[sl] = pos; stall_cnt[sl] = 0;
         } else {
            if (++stall_cnt[sl] >= STALL_FRAMES) {
               stopped[sl] = 1; vel_cmd[sl] = 0;
               uart_log("  ★ 轴%d 到限位, 位置=%ld\r\n", sl, (long)pos);
            }
         }
      }
      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_cmd[sl]);
      cycle(); poll_cmd();
      log_screen_service();
      if (g_abort) {
         ramp_to_zero(); uart_log("!! 归零急停\r\n");
         log_screen_event(LOGPH_EV_ESTOP, 1); log_screen_set_state(LOGPH_ST_IDLE);
         return;
      }

      int all = 1;
      for (sl = 1; sl <= ctx.slavecount; sl++) if (!stopped[sl]) all = 0;
      if (all) {
         uart_log(">>> 归零完成, 所有轴到位 <<<\r\n");
         log_screen_event(LOGPH_EV_HM_DONE, 0); log_screen_set_state(LOGPH_ST_IDLE);
         return;
      }
   }
   uart_log(">>> 归零超时(部分轴未检测到限位) <<<\r\n");
   log_screen_event(LOGPH_EV_HM_TO, 1);
   log_screen_set_state(LOGPH_ST_IDLE);
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
   uart_log(">>> 本路已兼作传感器一手数据监视: 不接电缸时即在此打印 Roll/Pitch 与原始帧 <<<\r\n");
   sensor_reset();      /* 复位融合四元数+预热, 让 sensor_get 从头收敛 */
   int hb = 0;
   /* ★ 峰值保持诊断(现读的是6轴融合后的姿态角): 每1秒窗口内记录|·|最大的 融合角(R/P),
    * 窗口末打印后清零。倾斜保持时看融合角峰能否达到你倾的角度; 静止看是否稳定≈零偏。*/
   float pkR = 0.0f, pkP = 0.0f;          /* 融合角峰值(带符号, 取|·|最大者) */
   float lastR = 0.0f, lastP = 0.0f;      /* 最近一帧融合角(看静止时稳不稳) */
   for (;;) {
      /* ★ 关键: 每周期排空并解析传感器帧(现内部跑6轴融合)。原实现从不调 sensor_get(),
       * 帧永远堆在缓冲里没人取, sensor_fail_reason 也停在'0' —— 曾是"一直显示0"的根因。*/
      float r, p;
      if (sensor_get(&r, &p)) {
         lastR = r; lastP = p;
         if (fabsf(r) > fabsf(pkR)) pkR = r;
         if (fabsf(p) > fabsf(pkP)) pkP = p;
      }
      poll_cmd();          /* modbus_poll/sync/feedback + USART1/USB 命令 */
      heartbeat();         /* 心跳灯照闪, 表明没死 */
      osal_usleep(CYC_US); /* 无 EtherCAT, 相对延时凑 ~4ms 节拍即可 */
      if (++hb >= 250) {   /* 每约1秒播报一次融合角当前值+峰值+统计, 然后清零峰值 */
         hb = 0;
         modbus_loopback_selftest_tx();   /* 回环自测: 平时空操作(见 modbus_slave.c 的 MB_LOOPBACK_TEST) */
         uart_log("[融合角] 当前 R=%+6.1f P=%+6.1f | 本秒峰 R=%+6.1f P=%+6.1f\r\n",
                  lastR, lastP, pkR, pkP);
         pkR = pkP = 0.0f;   /* 清零, 开始下一窗口 */
         uart_log("[传感器统计] 帧边界=%lu 有效=%lu 最近失败='%c'\r\n",
                  (unsigned long)sensor_stat_frames, (unsigned long)sensor_stat_valid,
                  sensor_fail_reason);
      }
   }
}

#if SENSOR_STANDALONE_TEST
/* 独立测试循环: 不返回。只做"读传感器→打印", 不碰EtherCAT/电缸任何东西,
 * 电缸不接线/不上电都能跑。Modbus/串口命令通道保留(poll_cmd), 方便同时用
 * HMI或串口终端看心跳、发?查看(从站数会显示0, 属正常, 忽略即可)。 */
static void sensor_standalone_test(void)
{
   uart_log("\r\n>>> 传感器独立测试模式(未接EtherCAT/电缸, 仅读传感器→打印) <<<\r\n");
   sensor_reset();
   uint32_t hb = 0;
   for (;;) {
      float r, p;
      if (sensor_get(&r, &p)) {
         uart_log("[传感器] Roll=%+7.2f Pitch=%+7.2f 源=6轴融合\r\n", r, p);
      }
      poll_cmd();
      heartbeat();
      osal_usleep(CYC_US);
      if (++hb >= 250) {   /* 约1秒一次统计播报 */
         hb = 0;
         uart_log("[统计] 帧边界=%lu 有效=%lu 最近失败原因='%c'\r\n",
                  (unsigned long)sensor_stat_frames, (unsigned long)sensor_stat_valid,
                  sensor_fail_reason);
      }
   }
}
#endif

void ecat_motion_run(void)
{
   int sl, i;

   uart_log_init();
   modbus_init();   /* USART3 上的 Modbus RTU 从站(HMI 用), 与串口命令并行 */
   sensor_init();   /* USART6 上的 WT901 姿态传感器接入(传感器跟随模式用) */
   uart_log("\r\n\r\n===== STM32 SOEM 康复运动 (CSV, 命令驱动) =====\r\n");

   uart_log("上次复位原因:");
   if (g_reset_cause & RCC_CSR_PORRSTF)  uart_log(" 上电复位(POR/PDR)");
   if (g_reset_cause & RCC_CSR_PADRSTF)  uart_log(" 外部NRST引脚复位(硬件拉低, 疑似DTR)");
   if (g_reset_cause & RCC_CSR_SFTRSTF)  uart_log(" 软件复位");
   if (g_reset_cause & RCC_CSR_IWDGRSTF) uart_log(" 独立看门狗复位");
   if (g_reset_cause & RCC_CSR_WWDGRSTF) uart_log(" 窗口看门狗复位");
   if (g_reset_cause & RCC_CSR_LPWRRSTF) uart_log(" 低功耗复位");
   uart_log("  (CSR=0x%08lX)\r\n", (unsigned long)g_reset_cause);

#if SENSOR_STANDALONE_TEST
   sensor_standalone_test();   /* 不返回: 跳过下面所有EtherCAT初始化/电缸相关代码 */
#endif

   log_screen_set_state(LOGPH_ST_BOOT);
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
   log_screen_event(ctx.slavecount >= 3 ? LOGPH_EV_SCAN3 :
                     ctx.slavecount == 2 ? LOGPH_EV_SCAN2 :
                     ctx.slavecount == 1 ? LOGPH_EV_SCAN1 : LOGPH_EV_SCAN0, 0);
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
   log_screen_event(LOGPH_EV_BUS_OK, 0);
   uart_log("OP 建立, WKC=%d (正常≈%d)\r\n", g_wkc, ctx.slavecount * 3);

   /* CiA402 使能: 0x06 → 0x07 → 0x0F */
   g_ec_phase = 5;
   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0006, 0); cycle(); }
   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0007, 0); cycle(); }
   for (i = 0; i < 300; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0); cycle(); }
   for (sl = 1; sl <= ctx.slavecount; sl++)
      uart_log("    从站%d 状态字=0x%04X %s\r\n", sl, read_sw(sl),
               (read_sw(sl) & 0x0008) ? "[报警!]" : (((read_sw(sl) & 0x006F) == 0x0027) ? "[已使能]" : ""));
   log_screen_event(LOGPH_EV_DRV_OK, 0);

   /* ===== 待机: 听命令 ===== */
   g_ec_phase = 99;
   log_screen_set_state(LOGPH_ST_IDLE);
   for (sl = 1; sl <= ctx.slavecount; sl++) start_pos[sl] = read_pos63(sl);  /* 反馈位置以此刻为0基准, 避免开机显示绝对编码值 */
   uart_log("\r\n>>> 进入待机, 伺服使能保持零速. 发 ? 查看命令. 建议先 h 归零 <<<\r\n");
   int hb = 0;
   while (!g_quit) {
      g_status = g_ec_fault ? 4 : 1;   /* HMI 反馈: 故障未清则4, 否则待机1 */
      g_cur_mode = 99;
      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0);
      { float _r, _p; sensor_get(&_r, &_p); }   /* 待机也驱动校验计数,供 Bridge 诊断 */
      cycle();
      poll_cmd();
      log_screen_service();   /* 屏显日志(内部2秒节流), 待机中每2秒心跳一次证明没死机 */

      if (++hb >= 250) {   /* 每约1秒播报, 终端实时监看 + Modbus 联调(看HMI请求到没到) */
         hb = 0;
         modbus_loopback_selftest_tx();   /* 回环自测: 平时空操作(见 modbus_slave.c 的 MB_LOOPBACK_TEST) */
         uart_log("[待机] WKC=%d 从站=%d | Modbus 收到帧=%lu 有效=%lu\r\n",
                  g_wkc, ctx.slavecount,
                  (unsigned long)modbus_stat_frames, (unsigned long)modbus_stat_valid);
      }

      if (g_do_home)          { g_do_home = 0; g_do_sensor = 0; g_run_request = -1; run_homing(); uart_log(">>> 回待机 <<<\r\n"); }
      else if (g_do_sensor)   { g_do_sensor = 0; g_run_request = -1; run_sensor_mode(); uart_log(">>> 回待机 <<<\r\n"); }
      else if (g_run_request >= 0) { int m = g_run_request; g_run_request = -1; g_do_home = 0; run_rehab_mode(m); uart_log(">>> 回待机 <<<\r\n"); }
   }

   /* 收到 q: 平滑下电退出 */
   graceful_shutdown();
   while (1) { osal_usleep(100000); }
}
