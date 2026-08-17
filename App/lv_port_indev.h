/*
 * LVGL输入设备移植层 —— 把touch_xpt2046.c接到LVGL的触摸输入。
 */
#ifndef _lv_port_indev_h_
#define _lv_port_indev_h_

#ifdef __cplusplus
extern "C" {
#endif

/* 注册LVGL触摸输入设备, 在lv_init()之后调一次。 */
void lv_port_indev_init(void);

#ifdef __cplusplus
}
#endif

#endif
