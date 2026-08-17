/*
 * 板载3.5寸TFT彩屏驱动 —— FSMC并口(Bank1 NE4, 16位数据总线, A6作RS命令/数据选择)
 * ================================================================
 * 硬件对照《普中STM32-F407-战神开发板资料》官方原理图与驱动源码
 * (2--开发板原理图/彩屏原理图, 4--实验程序/1--基础实验/30-FSMC-TFTLCD显示实验、33-触摸屏实验):
 *   数据总线: PD14/15=D0/D1, PD0/1=D2/D3, PE7~15=D4~D12, PD8/9/10=D13/D14/D15
 *   控制线:   PD4=NOE(读) PD5=NWE(写) PF12=A6(命令/数据选择) PG12=NE4(片选)
 *   背光:     PB15(简单GPIO推挽输出, 高=亮)
 *
 * 面板驱动IC: 3.5寸屏在这块板子上只有4种可能(对照官方资料彩屏原理图目录):
 *   HX8357D / ILI9481 / ILI9488 / ST7793, 分辨率均为480×320。
 *   本驱动先实现 ILI9488(市面最常见), 用 LCD_PANEL_XXXX 宏切换 —— 如果点亮后
 *   颜色/花屏不对, 换成 LCD_PANEL_ILI9481 / LCD_PANEL_HX8357D / LCD_PANEL_ST7793
 *   其中之一重新编译烧录即可, 四选一总能对上(其余三款面板初始化时序待确认
 *   具体型号后再从官方驱动源码port进来, 目前只有ILI9488分支)。
 *
 * 坐标系: 上电默认横屏(对齐官方驱动 TFTLCD_DIR=1 默认值), 480(宽)×320(高)。
 */
#ifndef _lcd_fsmc_h_
#define _lcd_fsmc_h_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 面板型号选择(四选一, 见上方注释) ---- */
#define LCD_PANEL_ILI9488

/* ---- RGB565 常用颜色(与官方驱动调色板一致, 方便对照效果) ---- */
#define LCD_COLOR_WHITE      0xFFFF
#define LCD_COLOR_BLACK      0x0000
#define LCD_COLOR_RED        0xF800
#define LCD_COLOR_GREEN      0x07E0
#define LCD_COLOR_BLUE       0x001F
#define LCD_COLOR_YELLOW     0xFFE0
#define LCD_COLOR_CYAN       0x7FFF
#define LCD_COLOR_GRAY       0x8430

/* 初始化FSMC总线+GPIO+面板上电时序, 上电后调一次。内部含背光开启。 */
void lcd_init(void);

/* 面板当前分辨率(横屏480×320) */
uint16_t lcd_width(void);
uint16_t lcd_height(void);

/* 整屏填充单色 */
void lcd_clear(uint16_t color);

/* 矩形区域填充单色, (x0,y0)~(x1,y1)含边界, 坐标越界由调用方保证 */
void lcd_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/* 单点画点 */
void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

/* 背光开关 */
void lcd_backlight(int on);

#ifdef __cplusplus
}
#endif

#endif
