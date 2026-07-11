/*
 * 最简 EtherCAT 运动控制 —— STM32 板上 SOEM
 *   流程: 扫从站 → 配 CSV 的 PDO → 进OP → CiA402使能 → 伸出/缩回 × 5 → 保持
 *   控制: CSV(速度模式,6060=9), 位置反馈用 6063(CSV下6064恒为0), 简单P速度环
 *   已刻意砍掉: 模式切换/FK/DC-PI同步/E-101等防护/诊断打印 —— 先跑通再说
 *
 *   ⚠ 未做 DC 同步(SYNC0)。信捷 DS5C1 在 CSV 下通常可自由运行(free-run);
 *     若上电后从站报 E-810/E-812 之类时钟类报警, 需要再补 DC(ecx_configdc+dcsync0)。
 */
#include "soem/soem.h"
#include "ecat_motion.h"
#include "uart_log.h"
#include "osal.h"
#include <string.h>

volatile int g_ec_slavecount = 0;
volatile int g_ec_phase = 0;
volatile int g_ec_fault = 0;

static ecx_contextt ctx;
static uint8 IOmap[256];

/* ---- 运动参数(最简, 想改就改这几个) ---- */
#define LEAD_MM       3.0        /* 丝杠导程 mm/圈 */
#define PULSE_PER_REV 131072.0   /* 编码器 脉冲/圈 */
#define ROD_MM        80.0       /* 每次伸出的杆长 mm (实测三轴最大行程均值159.65mm, 这里约留一半余量) */
#define ROD_PULSE     ((int32_t)(ROD_MM * PULSE_PER_REV / LEAD_MM))
#define CYCLES        5          /* 往复次数 */
#define CYC_US        2000       /* 通信周期 2ms */
#define VMAX          568000     /* 速度上限 pulse/s (≈13mm/s) */
#define KP            3          /* 位置->速度 比例增益 */
#define DEADBAND      500        /* 到位死区 pulse */
#define MAX_CYC_PHASE 8000       /* 单个伸/缩阶段最多跑多少周期(超时保护, 行程变大相应放宽) */
#define START_DELAY_S 5          /* 使能伺服前的倒计时(秒), 留时间让人离开机构 */

/* ---- 一次性标定: 探测三轴各自的最大可伸出量 ----
 * 改成1编译烧录, 单独跑一次探测, 记下日志里每轴的最大行程和三轴平均值,
 * 测完改回0, 用测出来的数据设置正式的 ROD_MM。
 * 没有硬件限位开关, 只能靠软件判断"顶到头了": 慢慢推, 若一段时间
 * (STALL_WINDOW_CYC个周期)位置几乎不再前进就判定该轴到极限、立刻停该轴,
 * 其余轴继续探测直到各自都停下, 避免某条腿先卡死、其余两条还在硬顶导致
 * 并联平台被扭。ABS_MAX_MM是保险丝, 万一失速判断失效也不会无限往前冲。
 * 第一次探测(0~150mm, 慢速)已经整段跑通、没有任何异常/报警, 说明这一段
 * 机械上是空的, 之后同样的行程可以用正常速度快速通过, 只在没验证过的
 * 未知地带(>150mm)才继续用慢速摸。 */
#define PROBE_MAX_STROKE  0      /* 已测完: 三轴均值159.65mm(159.70/159.54/159.70), 关掉探测模式 */
#define PROBE_FAST_MM     150.0        /* 已验证空段的终点: 用正常速度快速通过 */
#define PROBE_FAST_PULSE  ((int32_t)(PROBE_FAST_MM * PULSE_PER_REV / LEAD_MM))
#define PROBE_VMAX_FAST   VMAX         /* 已知安全段用正常速度(≈9mm/s) */
#define PROBE_VMAX_SLOW   (VMAX / 8)   /* 未知地带用慢速(≈1.1mm/s), 减小顶到硬止点的冲击 */
#define STALL_WINDOW_CYC  250          /* 判定失速的采样窗口(≈0.5s @2ms周期) */
#define STALL_EPS_PULSE   2000         /* 窗口内位置变化小于此值判定为失速 */
#define ABS_MAX_MM        250.0        /* 软件硬顶, 没实测行程前留够余量的保险丝, 到了强制停
                                           (第一次探测150mm三轴全都撞了硬顶, 没测到真失速, 说明
                                           实际行程≥150mm, 调大到250再探一次) */
#define ABS_MAX_PULSE     ((int32_t)(ABS_MAX_MM * PULSE_PER_REV / LEAD_MM))
#define PROBE_TIMEOUT_CYC 200000       /* 探测总超时(≈400s), 兜底防止死循环 */

#define CSV_MODE 9

static int32_t start_pos[EC_MAXSLAVE];

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
static inline int32_t read_pos63(int sl)
{ uint8 *in = ctx.slavelist[sl].inputs; return in ? *(int32_t *)(in + 3) : 0; }

static int any_fault(void)
{
   for (int sl = 1; sl <= ctx.slavecount; sl++)
      if (read_sw(sl) & 0x0008) return sl;
   return 0;
}

static volatile int g_wkc = 0;   /* 上一周期工作计数, 正常应 = 3*从站数附近 */

/* 一个通信周期: 发+收过程数据, 固定节拍 */
static void cycle(void)
{
   ecx_send_processdata(&ctx);
   g_wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
   osal_usleep(CYC_US);
}

/* CSV 模式 PDO 映射 */
static int setup_pdo_csv(int slave)
{
   uint8 u8; uint16 u16; uint32 u32; int wkc;

   /* --- RxPDO 0x1600: CW(6040) + Mode(6060) + TargetVel(60FF) --- */
   u8 = 0;
   wkc = ecx_SDOwrite(&ctx, slave, 0x1C12, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
   if (wkc <= 0) {
      uart_log("    [错误] 从站%d 首次SDO写(1C12)失败 wkc=%d, 邮箱/PRE-OP可能没建立\r\n", slave, wkc);
      return -1;
   }
   u8 = 0; ecx_SDOwrite(&ctx, slave, 0x1600, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
   u32 = 0x60400010; ecx_SDOwrite(&ctx, slave, 0x1600, 0x01, FALSE, 4, &u32, EC_TIMEOUTRXM);
   u32 = 0x60600008; ecx_SDOwrite(&ctx, slave, 0x1600, 0x02, FALSE, 4, &u32, EC_TIMEOUTRXM);
   u32 = 0x60FF0020; ecx_SDOwrite(&ctx, slave, 0x1600, 0x03, FALSE, 4, &u32, EC_TIMEOUTRXM);
   u8 = 3; ecx_SDOwrite(&ctx, slave, 0x1600, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
   u16 = 0x1600; ecx_SDOwrite(&ctx, slave, 0x1C12, 0x01, FALSE, 2, &u16, EC_TIMEOUTRXM);
   u8 = 1;       ecx_SDOwrite(&ctx, slave, 0x1C12, 0x00, FALSE, 1, &u8,  EC_TIMEOUTRXM);

   /* --- TxPDO 0x1A00: SW(6041) + Mode(6061) + Pos(6063) + Vel(606C) --- */
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

#if !PROBE_MAX_STROKE
/* 让三轴一起走到相对起点 +target_pulse 的位置(CSV速度环), 到位或超时返回 */
static void move_to(int32_t target_pulse)
{
   int i;
   for (i = 0; i < MAX_CYC_PHASE; i++)
   {
      int done = 1;
      for (int sl = 1; sl <= ctx.slavecount; sl++)
      {
         int32_t actual = read_pos63(sl) - start_pos[sl];
         int32_t err = target_pulse - actual;
         int32_t vel = (int32_t)KP * err;
         if (vel >  VMAX) vel =  VMAX;
         if (vel < -VMAX) vel = -VMAX;
         if (err < DEADBAND && err > -DEADBAND) vel = 0;
         else done = 0;
         write_pdo(sl, 0x000F, vel);
      }
      cycle();
      if (any_fault()) { g_ec_fault = any_fault(); return; }
      if (done) return;
   }
   uart_log("    [超时] %d个周期未到位, 从站1目标=%ld 实际=%ld\r\n",
            MAX_CYC_PHASE, (long)target_pulse, (long)(read_pos63(1) - start_pos[1]));
}
#endif

#if PROBE_MAX_STROKE
/* 慢速推到各轴自己的机械极限(失速判定), 记录每轴最大行程, 再慢速退回零位 */
static void probe_max_stroke(void)
{
   int32_t win_start_pos[EC_MAXSLAVE] = {0};
   int32_t max_pulse[EC_MAXSLAVE] = {0};
   int stalled[EC_MAXSLAVE] = {0};
   int win_cnt = 0;
   int cyc;

   uart_log(">>> 开始探测三轴最大行程 (0~%.0fmm已验证段用%.2fmm/s快速通过, "
            "之后未知段降到%.2fmm/s, 软件硬顶=%.0fmm) <<<\r\n",
            PROBE_FAST_MM, (double)PROBE_VMAX_FAST * LEAD_MM / PULSE_PER_REV,
            (double)PROBE_VMAX_SLOW * LEAD_MM / PULSE_PER_REV, ABS_MAX_MM);

   for (int sl = 1; sl <= ctx.slavecount; sl++) win_start_pos[sl] = read_pos63(sl) - start_pos[sl];

   for (cyc = 0; cyc < PROBE_TIMEOUT_CYC; cyc++)
   {
      int all_stalled = 1;
      for (int sl = 1; sl <= ctx.slavecount; sl++)
      {
         int32_t cur = read_pos63(sl) - start_pos[sl];
         if (!stalled[sl])
         {
            all_stalled = 0;
            if (cur >= ABS_MAX_PULSE) {
               stalled[sl] = 1;
               max_pulse[sl] = cur;
               uart_log("    从站%d 到达软件硬顶 %.0fmm, 停止该轴\r\n", sl, ABS_MAX_MM);
            } else {
               int32_t v = (cur < PROBE_FAST_PULSE) ? PROBE_VMAX_FAST : PROBE_VMAX_SLOW;
               write_pdo(sl, 0x000F, v);
            }
         }
         if (stalled[sl]) write_pdo(sl, 0x000F, 0);
      }
      cycle();
      if (any_fault()) {
         g_ec_fault = any_fault();
         uart_log("[中止] 探测中从站%d 报警, 停止探测\r\n", g_ec_fault);
         break;
      }

      if (++win_cnt >= STALL_WINDOW_CYC)
      {
         for (int sl = 1; sl <= ctx.slavecount; sl++)
         {
            if (!stalled[sl])
            {
               int32_t cur = read_pos63(sl) - start_pos[sl];
               if ((cur - win_start_pos[sl]) < STALL_EPS_PULSE)
               {
                  stalled[sl] = 1;
                  max_pulse[sl] = cur;
                  uart_log("    从站%d 失速, 判定最大行程=%.2fmm\r\n",
                           sl, (double)cur * LEAD_MM / PULSE_PER_REV);
               }
               win_start_pos[sl] = cur;
            }
         }
         uart_log("    [进度] 从站1=%.1fmm 从站2=%.1fmm 从站3=%.1fmm\r\n",
                  (double)(read_pos63(1) - start_pos[1]) * LEAD_MM / PULSE_PER_REV,
                  (double)(read_pos63(2) - start_pos[2]) * LEAD_MM / PULSE_PER_REV,
                  (double)(read_pos63(3) - start_pos[3]) * LEAD_MM / PULSE_PER_REV);
         win_cnt = 0;
      }

      if (all_stalled || g_ec_fault) break;
   }

   /* 兜底: 超时结束但还有轴没标记停止, 按当前位置收尾 */
   for (int sl = 1; sl <= ctx.slavecount; sl++)
      if (!stalled[sl]) max_pulse[sl] = read_pos63(sl) - start_pos[sl];

   {
      double avg = 0;
      for (int sl = 1; sl <= ctx.slavecount; sl++) {
         double mm = (double)max_pulse[sl] * LEAD_MM / PULSE_PER_REV;
         avg += mm;
         uart_log("    从站%d 最大行程 = %.2f mm\r\n", sl, mm);
      }
      avg /= ctx.slavecount;
      uart_log(">>> 三轴平均最大行程 = %.2f mm <<<\r\n", avg);
   }

   if (!g_ec_fault)
   {
      uart_log("退回零位...\r\n");
      for (cyc = 0; cyc < PROBE_TIMEOUT_CYC; cyc++)
      {
         int done = 1;
         for (int sl = 1; sl <= ctx.slavecount; sl++)
         {
            int32_t cur = read_pos63(sl) - start_pos[sl];
            /* 退回是往已经走过的空段退, 没有撞障碍风险, 全程用正常速度快退 */
            if (cur > DEADBAND) { write_pdo(sl, 0x000F, -PROBE_VMAX_FAST); done = 0; }
            else                { write_pdo(sl, 0x000F, 0); }
         }
         cycle();
         if (any_fault()) { g_ec_fault = any_fault(); break; }
         if (done) break;
      }
      uart_log(g_ec_fault ? "退回零位中断(报警)\r\n" : "已退回零位\r\n");
   }
}
#endif

void ecat_motion_run(void)
{
   int sl, i;

   uart_log_init();
   uart_log("\r\n\r\n===== STM32 SOEM 启动 =====\r\n");

   g_ec_phase = 1;
   /* ifname 参数在 STM32 移植里被忽略, 传占位字符串即可 */
   if (!ecx_init(&ctx, "stm32eth")) {
      uart_log("[错误] 网卡初始化失败 (ecx_init)\r\n");
      g_ec_phase = -1; return;
   }
   uart_log("网卡就绪, 开始扫描总线...\r\n");

   g_ec_phase = 2;
   if (ecx_config_init(&ctx) <= 0) {
      uart_log("[错误] 没扫到从站! 检查网线/伺服上电\r\n");
      g_ec_phase = -2; ecx_close(&ctx); return;
   }
   g_ec_slavecount = ctx.slavecount;      /* ===== 里程碑A: 这里应为 3 ===== */
   uart_log(">>> 里程碑A: 扫到 %d 个从站 <<<\r\n", ctx.slavecount);
   for (sl = 1; sl <= ctx.slavecount; sl++)
      uart_log("    从站%d: %s\r\n", sl, ctx.slavelist[sl].name);
   if (ctx.slavecount < 1) { g_ec_phase = -2; ecx_close(&ctx); return; }

   /* PRE-OP: 配 CSV 的 PDO */
   g_ec_phase = 3;
   ctx.slavelist[0].state = EC_STATE_PRE_OP;
   ecx_writestate(&ctx, 0);
   {
      int st = ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
      uart_log("PRE-OP 切换结果: 实际状态=0x%02X (期望0x%02X)\r\n", st, EC_STATE_PRE_OP);
   }
   /* ecx_statecheck(ctx,0,...)是广播检查,只更新slavelist[0].state,
      不会更新各从站自己的.state(仍停在config_init时的INIT)。
      ecx_mbxsend内部要靠slavelist[slave].state>=PRE_OP才会真正发SDO邮箱写,
      必须在这里用ecx_readstate同步一次各从站的本地状态缓存,否则SDO写会静默返回wkc=0。 */
   ecx_readstate(&ctx);
   for (sl = 1; sl <= ctx.slavecount; sl++) {
      uart_log("  配置从站%d PDO...\r\n", sl);
      if (setup_pdo_csv(sl) < 0) {
         g_ec_phase = -3;
         uart_log("[错误] 从站%d PDO配置失败, 停止 (phase=-3)\r\n", sl);
         ecx_close(&ctx);
         return;
      }
   }
   uart_log("所有从站PDO配置完成, 建立IOmap...\r\n");

   ecx_config_map_group(&ctx, IOmap, 0);

   /* SAFE-OP → 先发几帧 → OP */
   g_ec_phase = 4;
   uart_log("PDO映射完成, 进 SAFE-OP...\r\n");
   ctx.slavelist[0].state = EC_STATE_SAFE_OP;
   ecx_writestate(&ctx, 0);
   {
      int st = ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
      uart_log("SAFE-OP 切换结果: 实际状态=0x%02X (期望0x%02X)\r\n", st, EC_STATE_SAFE_OP);
   }
   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0000, 0); cycle(); }

   uart_log("进 OP...\r\n");
   ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
   ecx_writestate(&ctx, 0);
   for (i = 0; i < 200; i++) {
      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0000, 0);
      if (i % 40 == 0) ecx_writestate(&ctx, 0);
      cycle();
   }
   {
      int st = ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
      uart_log("OP 切换结果: 实际状态=0x%02X (期望0x%02X)\r\n", st, EC_STATE_OPERATIONAL);
   }
   uart_log("OP 建立, WKC=%d (正常≈%d)\r\n", g_wkc, ctx.slavecount * 3);

   /* 使能前倒计时: 留时间让人离开机构, 期间保持零指令、正常周期通讯 */
   uart_log("使能伺服前倒计时 %d 秒, 请远离机构...\r\n", START_DELAY_S);
   for (int s = START_DELAY_S; s > 0; s--) {
      uart_log("  %d...\r\n", s);
      for (int k = 0; k < (1000000 / CYC_US); k++) {
         for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0000, 0);
         cycle();
      }
   }

   /* CiA402 使能: 0x06 → 0x07 → 0x0F */
   g_ec_phase = 5;
   uart_log("使能伺服 (0x06→0x07→0x0F)...\r\n");
   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0006, 0); cycle(); }
   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0007, 0); cycle(); }
   for (i = 0; i < 300; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0); cycle(); }

   for (sl = 1; sl <= ctx.slavecount; sl++)
      uart_log("    从站%d 状态字=0x%04X %s\r\n", sl, read_sw(sl),
               (read_sw(sl) & 0x0008) ? "[报警!]" : (((read_sw(sl) & 0x006F) == 0x0027) ? "[已使能]" : ""));
   if (any_fault()) {
      g_ec_fault = any_fault(); g_ec_phase = -5;
      uart_log("[错误] 使能后从站%d 报警, 停止\r\n", g_ec_fault);
   }

   /* 记录使能后的位置作为零点 */
   for (sl = 1; sl <= ctx.slavecount; sl++) start_pos[sl] = read_pos63(sl);

#if PROBE_MAX_STROKE
   g_ec_phase = 10;
   probe_max_stroke();
#else
   /* ===== 里程碑B: 伸出/缩回 × 5 ===== */
   uart_log(">>> 里程碑B: 开始伸缩 %d 次 (每次 %d mm) <<<\r\n", CYCLES, (int)ROD_MM);
   for (int c = 0; c < CYCLES && !g_ec_fault; c++)
   {
      g_ec_phase = 10 + c;
      uart_log("  第%d次: 伸出...\r\n", c + 1);
      move_to(ROD_PULSE);   /* 伸出 */
      for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0); cycle(); }
      uart_log("  第%d次: 缩回...\r\n", c + 1);
      move_to(0);           /* 缩回 */
      for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0); cycle(); }
   }
#endif

   if (g_ec_fault) uart_log("[结束] 运动中从站%d 报警\r\n", g_ec_fault);
   else            uart_log(">>> 完成! 准备下电 <<<\r\n");

   /* 结束: CiA402 Shutdown(CW=0x06)脱力, 不再输出转矩/保持位置 */
   g_ec_phase = 99;
   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0006, 0); cycle(); }
   for (sl = 1; sl <= ctx.slavecount; sl++)
      uart_log("    从站%d 已下电, 状态字=0x%04X\r\n", sl, read_sw(sl));

   /* 退到 SAFE-OP 再退到 INIT: 过程数据看门狗(SM2/3 watchdog)只在SAFE-OP/OP下生效,
      退回INIT后从站不再监视周期帧, 之后拔网线不会触发从站看门狗报警。
      逐级退(OP→SAFE-OP→INIT)是标准做法, 比直接跳INIT更稳妥。 */
   uart_log("退出OP, 关闭过程数据看门狗监视...\r\n");
   ctx.slavelist[0].state = EC_STATE_SAFE_OP;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   ctx.slavelist[0].state = EC_STATE_INIT;
   ecx_writestate(&ctx, 0);
   {
      int st = ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE * 4);
      uart_log(">>> 已退到 INIT (0x%02X), 不再需要周期通讯, 现在可以安全拔网线 <<<\r\n", st);
   }
   ecx_close(&ctx);

   while (1) { osal_usleep(100000); }   /* 彻底空转, 不再收发任何EtherCAT帧 */
}
