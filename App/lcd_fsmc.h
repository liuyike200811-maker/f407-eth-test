/*
 * 板载3.5寸TFT彩屏驱动 —— FSMC并口(Bank1 NE4, 16位数据总线, A6作RS命令/数据选择)
 * ================================================================
 * 硬件对照《普中STM32-F407-战神开发板资料》官方原理图与驱动源码
 * (2--开发板原理图/彩屏原理图, 4--实验程序/1--基础实验/30-FSMC-TFTLCD显示实验、33-触摸屏实验):
 *   数据总线: PD14/15=D0/D1, PD0/1=D2/D3, PE7~15=D4~D12, PD8/9/10=D13/D14/D15
 *   控制线:   PD4=NOE(读) PD5=NWE(写) PF12=A6(命令/数据选择) PG12=NE4(片选)
 *   背光:     PB15(简单GPIO推挽输出, 高=亮)
 *
 * 面板驱动IC: 用户核对实物确认为 HX8357DN(3.5寸, 480×320)。ILI9481/ILI9488/
 *   ST7793 是同一路FSMC总线上另外三种可能型号(未用), 初始化时序未实现。
 *
 * 坐标系: 竖屏, 320(宽)×480(高) —— 对齐实测物理板子的竖放朝向(2026-08 台架
 *   核对: 横屏初始化会导致内容整体转90°, 已改用官方驱动 dir=0 的竖屏寄存器值)。
 */
#ifndef _lcd_fsmc_h_
#define _lcd_fsmc_h_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 面板型号(已核对实物确认, 见上方注释) ---- */
#define LCD_PANEL_HX8357DN

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

/* 把一块像素数组(RGB565, 按行优先连续排列, 长度=(x1-x0+1)*(y1-y0+1))写进
 * (x0,y0)~(x1,y1)窗口。供LVGL显示驱动的flush回调用, 逐像素调lcd_wr_color。 */
void lcd_blit(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint16_t *pixels);

/* 背光开关 */
void lcd_backlight(int on);

#ifdef __cplusplus
}
#endif

#endif
