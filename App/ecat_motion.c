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
#define ROD_MM        10.0       /* 每次伸出的杆长 mm */
#define ROD_PULSE     ((int32_t)(ROD_MM * PULSE_PER_REV / LEAD_MM))
#define CYCLES        5          /* 往复次数 */
#define CYC_US        2000       /* 通信周期 2ms */
#define VMAX          200000     /* 速度上限 pulse/s (≈4.5mm/s) */
#define KP            3          /* 位置->速度 比例增益 */
#define DEADBAND      500        /* 到位死区 pulse */
#define MAX_CYC_PHASE 5000       /* 单个伸/缩阶段最多跑多少周期(超时保护) */

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
   {
      ec_timet t0, t1, dt;
      u8 = 0;
      osal_get_monotonic_time(&t0);
      wkc = ecx_SDOwrite(&ctx, slave, 0x1C12, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
      osal_get_monotonic_time(&t1);
      osal_time_diff(&t0, &t1, &dt);
      uart_log("    首次SDO写(1C12) 耗时=%ld.%06ld秒 wkc=%d\r\n",
               (long)dt.tv_sec, (long)dt.tv_nsec / 1000, wkc);
      if (wkc <= 0) {
         uart_log("    [错误] 从站%d 首次SDO写(1C12)失败 wkc=%d, 邮箱/PRE-OP可能没建立\r\n", slave, wkc);
         return -1;
      }
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
      if (done) {
         uart_log("    到位, 用时%d个周期, 从站1实际位置=%ld\r\n", i, (long)(read_pos63(1) - start_pos[1]));
         return;
      }
   }
   uart_log("    [超时] %d个周期未到位, 从站1目标=%ld 实际=%ld\r\n",
            MAX_CYC_PHASE, (long)target_pulse, (long)(read_pos63(1) - start_pos[1]));
}

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

   /* 邮箱配置诊断: mbx_wo/ro 是CoE走的物理地址, 若为0说明SII读取或邮箱配置有问题 */
   for (sl = 1; sl <= ctx.slavecount; sl++)
      uart_log("    从站%d 邮箱: configadr=0x%04X wo=0x%04X l=%d ro=0x%04X rl=%d proto=0x%04X CoE=%d\r\n",
               sl, ctx.slavelist[sl].configadr,
               ctx.slavelist[sl].mbx_wo, ctx.slavelist[sl].mbx_l,
               ctx.slavelist[sl].mbx_ro, ctx.slavelist[sl].mbx_rl,
               ctx.slavelist[sl].mbx_proto, ctx.slavelist[sl].CoEdetails);

   /* 硬件寄存器回读诊断: 直接读ESC上SM0/SM1的真实寄存器内容(而不是主站本地缓存),
      核对 config_init 那次配置SM的写(返回值未检查!)是否真的生效, 尤其Activate位 */
   for (sl = 1; sl <= ctx.slavecount; sl++)
   {
      uint8_t smraw[16] = {0};
      int rwkc = ecx_FPRD(&ctx.port, ctx.slavelist[sl].configadr, ECT_REG_SM0, sizeof(smraw), smraw, EC_TIMEOUTRET3);
      uart_log("    从站%d SM寄存器回读 wkc=%d: SM0[addr=%02X%02X len=%02X%02X ctrl=%02X stat=%02X act=%02X pdi=%02X] "
               "SM1[addr=%02X%02X len=%02X%02X ctrl=%02X stat=%02X act=%02X pdi=%02X]\r\n",
               sl, rwkc,
               smraw[1], smraw[0], smraw[3], smraw[2], smraw[4], smraw[5], smraw[6], smraw[7],
               smraw[9], smraw[8], smraw[11], smraw[10], smraw[12], smraw[13], smraw[14], smraw[15]);
   }

   /* 原始64字节FPWR测试: 绕开SDO/mbxsend整套逻辑, 直接测"写64字节到0x1000"这个
      具体操作本身是否被ESC接受, 把"邮箱协议逻辑"和"底层收发通不通"这两个变量分开 */
   {
      uint8_t pattern[64];
      int pwkc;
      for (int k = 0; k < 64; k++) pattern[k] = (uint8_t)k;
      pwkc = ecx_FPWR(&ctx.port, ctx.slavelist[1].configadr, ctx.slavelist[1].mbx_wo, 64, pattern, EC_TIMEOUTRET3);
      uart_log("    [诊断] 原始64字节FPWR到从站1 0x1000: wkc=%d\r\n", pwkc);
   }

   /* PRE-OP: 配 CSV 的 PDO */
   g_ec_phase = 3;
   ctx.slavelist[0].state = EC_STATE_PRE_OP;
   ecx_writestate(&ctx, 0);
   {
      int st = ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
      uart_log("PRE-OP 切换结果: 实际状态=0x%02X (期望0x%02X)\r\n", st, EC_STATE_PRE_OP);
   }
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

   if (g_ec_fault) uart_log("[结束] 运动中从站%d 报警\r\n", g_ec_fault);
   else            uart_log(">>> 完成! 伸缩 %d 次结束, 保持零速 <<<\r\n", CYCLES);

   /* 结束: 保持零速, 停在这里 */
   g_ec_phase = 99;
   while (1) {
      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0);
      cycle();
   }
}
