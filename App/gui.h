/*
 * 屏显GUI —— 开机自检页/主页/实时日志页/系统诊断页, 按用户确认的设计稿
 * (06-STM32屏显/code_artifact.html)实现。不做控制, 纯展示+日志查看。
 */
#ifndef _gui_h_
#define _gui_h_

#ifdef __cplusplus
extern "C" {
#endif

/* 建开机页并显示, 在lv_init()+lv_port_disp_init()+lv_port_indev_init()
 * 之后、ecat_motion_run()真正开始扫从站之前调一次。 */
void gui_init(void);

/* 开机自检三行状态, 对应ecat_motion.c里g_ec_phase的真实阶段变化时调用 */
void gui_boot_mcu_ok(void);
void gui_boot_bus_ok(int slave_count);
void gui_boot_drv_enable(void);

/* "系统就绪"收尾, 然后切到主页(boot页自动释放) */
void gui_boot_ready_and_show_home(void);

/* EtherCAT连接状态, 待机循环里每次刷新时调, 驱动主页状态点+系统诊断页 */
void gui_set_ec_status(int connected, int wkc, int slave_count);

/* 待机循环里每次调用: 跑LVGL的定时器/动画/触摸处理 + 周期性刷新日志页/
 * 运行时间。⚠ 只能在待机循环里调, 不能塞进run_rehab_mode/run_sensor_mode
 * 这些运动闭环循环里(会拖慢4ms节拍, 参考代码导读里的教训)。 */
void gui_service(void);

#ifdef __cplusplus
}
#endif

#endif
