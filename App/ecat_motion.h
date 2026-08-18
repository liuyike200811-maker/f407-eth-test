/*
 * 最简 EtherCAT 运动: 扫从站 → CSV 使能 → 伸出/缩回 × 5
 */
#ifndef _ecat_motion_h_
#define _ecat_motion_h_

#ifdef __cplusplus
extern "C" {
#endif

/* 调试可见的全局状态(用调试器或 LED 观察) */
extern volatile int g_ec_slavecount;  /* 扫到的从站数 (里程碑A: 应为3) */
extern volatile int g_ec_phase;       /* 当前阶段编号 */
extern volatile int g_ec_fault;       /* !=0 表示某从站报警 */

/* 屏显GUI(gui.c)用: 运行状态字(0启动中1待机2运行3归零4故障5已下电) + EtherCAT WKC */
extern volatile int g_status;
extern volatile int g_wkc;

/* 上一次复位原因(RCC->CSR原始值), main()里HAL_Init前保存, 进ecat_motion_run打印后即可丢弃 */
extern volatile unsigned int g_reset_cause;

/* 上电后调用一次, 内部自带完整流程, 不返回(跑完停在保持) */
void ecat_motion_run(void);

#ifdef __cplusplus
}
#endif

#endif
