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

/* 上电后调用一次, 内部自带完整流程, 不返回(跑完停在保持) */
void ecat_motion_run(void);

#ifdef __cplusplus
}
#endif

#endif
