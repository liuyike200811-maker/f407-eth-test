/*
 * LVGL显示驱动移植层 —— 把lcd_fsmc.c的FSMC总线驱动接到LVGL的flush回调。
 */
#ifndef _lv_port_disp_h_
#define _lv_port_disp_h_

#ifdef __cplusplus
extern "C" {
#endif

/* 注册LVGL显示驱动, 在lv_init()之后调一次。 */
void lv_port_disp_init(void);

#ifdef __cplusplus
}
#endif

#endif
