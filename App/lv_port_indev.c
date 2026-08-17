/*
 * LVGL输入设备移植层实现。
 */
#include "lvgl.h"
#include "lv_port_indev.h"
#include "touch_xpt2046.h"

static lv_indev_drv_t indev_drv;

static void touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
   uint16_t x, y;

   if (touch_get_point(&x, &y)) {
      data->state = LV_INDEV_STATE_PR;
      data->point.x = x;
      data->point.y = y;
   } else {
      data->state = LV_INDEV_STATE_REL;
   }
}

void lv_port_indev_init(void)
{
   touch_init();

   lv_indev_drv_init(&indev_drv);
   indev_drv.type    = LV_INDEV_TYPE_POINTER;
   indev_drv.read_cb  = touch_read_cb;
   lv_indev_drv_register(&indev_drv);
}
