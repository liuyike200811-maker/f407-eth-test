/*
 * 板载3.5寸TFT彩屏驱动 —— FSMC并口(Bank1 NE4, 16位数据总线, A6作RS命令/数据选择)
 * ================================================================
 * 硬件对照《普中STM32-F407-战神开发板资料》官方原理图与驱动源码
 * (2--开发板原理图/彩屏原理图, 4--实验程序/1--基础实验/30-FSMC-TFTLCD显示实验、33-触摸屏实验):
 *   数据总线: PD14/15=D0/D1, PD0/1=D2/D3, PE7~15=D4~D12, PD8/9/10=D13/D14/D15
 *   控制线:   PD4=NOE(读) PD5=NWE(写) PF12=A6(命令/数据选择) PG12=NE4(片选)
 *   背光:     PB15(简单GPIO推挽输出, 高=亮)
 *
 * 面板驱动IC: HX8357DN(用户核对实物确认), 分辨率320×480竖屏(实测物理板子
 *   是竖放的, 2026-08台架核对过——横屏初始化会导致内容整体转90°)。
 *
 * 这一版屏幕定位改成纯日志展示屏(不做触摸/复杂GUI), 只用到:
 *   lcd_init() / lcd_fill_rect() / lcd_blit()，没有draw_pixel等用不到的接口。
 */
#ifndef _lcd_fsmc_h_
#define _lcd_fsmc_h_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- RGB565 常用颜色 ---- */
#define LCD_COLOR_BLACK      0x0000
#define LCD_COLOR_GREEN      0x3E06   /* 终端绿, 不是纯饱和绿, 护眼一点 */
#define LCD_COLOR_RED        0xF8A5   /* 稍微柔和的红, 跟绿在同一亮度量级 */
#define LCD_COLOR_ORANGE     0xFD80   /* 琥珀橙, 用于"未连接/已下电"这类既非正常(绿)也非
                                          报警(红)的第三态提示, 一眼跟前两者区分开 */

/* 初始化FSMC总线+GPIO+面板上电时序, 上电后调一次。内部含背光开启+清黑屏。 */
void lcd_init(void);

uint16_t lcd_width(void);
uint16_t lcd_height(void);

/* 整屏/矩形区域填充单色 */
void lcd_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void lcd_clear(uint16_t color);

/* 把一块像素数组(RGB565, 按行优先连续排列)写进(x0,y0)~(x1,y1)窗口。 */
void lcd_blit(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint16_t *pixels);

#ifdef __cplusplus
}
#endif

#endif
