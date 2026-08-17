/*
 * LVGL显示驱动移植层实现。
 * 局部刷新: 缓冲区只开10行(320×10×2byte≈6.25KB), 不做整屏framebuffer(那要
 * 320×480×2byte=300KB, 主RAM+CCMRAM全占了都不够)。LVGL算完一块之后调
 * flush_cb, 这里直接同步搬到FSMC总线上, 搬完调lv_disp_flush_ready()。
 *
 * 缓冲区跟LVGL内存池一样放CCMRAM(见lv_conf.h的LV_ATTRIBUTE_LARGE_RAM_ARRAY),
 * 主RAM放不下。
 */
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lcd_fsmc.h"

#define DISP_BUF_LINES 10

static lv_disp_draw_buf_t draw_buf;
static LV_ATTRIBUTE_LARGE_RAM_ARRAY lv_color_t buf1[320 * DISP_BUF_LINES];
static lv_disp_drv_t disp_drv;

static void disp_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
   lcd_blit((uint16_t)area->x1, (uint16_t)area->y1,
            (uint16_t)area->x2, (uint16_t)area->y2,
            (const uint16_t *)color_p);
   lv_disp_flush_ready(drv);
}

void lv_port_disp_init(void)
{
   lv_disp_draw_buf_init(&draw_buf, buf1, NULL, 320 * DISP_BUF_LINES);

   lv_disp_drv_init(&disp_drv);
   disp_drv.hor_res  = lcd_width();
   disp_drv.ver_res  = lcd_height();
   disp_drv.flush_cb = disp_flush_cb;
   disp_drv.draw_buf = &draw_buf;
   lv_disp_drv_register(&disp_drv);
}
