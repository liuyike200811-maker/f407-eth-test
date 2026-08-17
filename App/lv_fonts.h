/*
 * 中文字库子集(LVGL格式) —— 用lv_font_conv从Noto Sans CJK SC提取生成,
 * 字符集取自已确认的UI设计稿(06-STM32屏显/code_artifact.html 和
 * 屏显原型.html)里实际出现的全部汉字+ASCII可见字符, 共426个字形。
 *
 * 只覆盖这426个字——以后新增界面文案如果用到没收录的汉字, 需要重新跑一次
 * lv_font_conv生成(char集合从UI稿提取, 不是每次手动列)。
 *
 * lv_font_cn_14: 正文/数据(对应设计稿里10~15px那档)
 * lv_font_cn_20: 标题/品牌字(对应设计稿里17~22px那档)
 * 字形位图数据是const数组(LV_ATTRIBUTE_LARGE_CONST), 存Flash不占RAM。
 */
#ifndef _lv_fonts_h_
#define _lv_fonts_h_

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

LV_FONT_DECLARE(lv_font_cn_14)
LV_FONT_DECLARE(lv_font_cn_20)

#ifdef __cplusplus
}
#endif

#endif
