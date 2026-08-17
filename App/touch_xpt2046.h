/*
 * 电阻触摸屏驱动 —— XPT2046, 软件模拟SPI(不占硬件SPI外设)
 * 引脚对照《普中STM32-F407-战神开发板资料》官方touch.c驱动源码核对:
 *   T_PEN(触笔中断,低有效)=PB1   T_MISO(DOUT)=PB2   T_MOSI(TDIN)=PF11
 *   T_SCK(TCLK)=PB0   T_CS(TCS)=PC13
 */
#ifndef _touch_xpt2046_h_
#define _touch_xpt2046_h_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void touch_init(void);

/* 取一次触摸点(已转换成屏幕像素坐标, 按lcd_fsmc当前竖屏320×480系)。
 * 返回1=有效触摸(笔尖按下且读数合理), 0=没按/读数异常(*x,*y不动)。
 *
 * ⚠ 台架待定项: X_RAW_MIN/MAX、Y_RAW_MIN/MAX(本文件顶部#define)是XPT2046
 * 常见经验值, 不是这块屏实测标定过的——上电后点屏幕四角, 如果坐标跟手指
 * 对不上(比如偏移、左右/上下相反), 需要按实测调这四个值, 或者加一步九点
 * 校准流程(这次先不做, 用经验值起步)。 */
int touch_get_point(uint16_t *x, uint16_t *y);

#ifdef __cplusplus
}
#endif

#endif
