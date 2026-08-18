/*
 * 屏显GUI实现。屏幕对象"进哪页建哪页, 离开就删"(lv_scr_load_anim的auto_del),
 * 不常驻四个屏幕对象——嵌入式RAM紧张, 这是跟网页demo版本(四层DOM常驻靠
 * opacity切换)的关键实现差异, 视觉效果等价。
 *
 * 主页是常驻的"枢纽"(建一次不删), 日志页/系统诊断页每次进入重建、离开时
 * 通过auto_del自动释放, 对应的缓存指针在离开时置NULL。
 */
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "lvgl.h"
#include "gui.h"
#include "gui_log.h"
#include "lv_fonts.h"

/* ---- 配色(照抄已确认设计稿 code_artifact.html 的CSS变量) ---- */
#define C_BG        lv_color_hex(0x090a0c)
#define C_SURFACE   lv_color_hex(0x14171c)
#define C_SURFACE_H lv_color_hex(0x1f242d)
#define C_BORDER    lv_color_hex(0x232933)
#define C_INK       lv_color_hex(0xf1f5f9)
#define C_INK_SUB   lv_color_hex(0x8d9aab)
#define C_INK_DIM   lv_color_hex(0x4d5766)
#define C_ORANGE    lv_color_hex(0xff5722)
#define C_EMERALD   lv_color_hex(0x10b981)
#define C_AMBER     lv_color_hex(0xf59e0b)
#define C_ROSE      lv_color_hex(0xef4444)
#define C_BLUE      lv_color_hex(0x3b82f6)

/* ---- 屏幕/常驻控件缓存指针 ---- */
static lv_obj_t *scr_boot;
static lv_obj_t *scr_home;
static lv_obj_t *scr_log;
static lv_obj_t *scr_sysinfo;

static lv_obj_t *boot_chk_mcu, *boot_chk_bus, *boot_chk_drv;
static lv_obj_t *boot_phase_label, *boot_pct_label, *boot_bar;

static lv_obj_t *home_uptime_label;
static lv_obj_t *home_status_dot, *home_status_label;

static lv_obj_t *log_body;
static uint32_t  log_last_seq = 0;
static int       log_paused = 0;

static lv_obj_t *sysinfo_conn_label, *sysinfo_wkc_label;

static int g_ec_connected = 0;
static int g_ec_wkc = 0;
static int g_ec_slaves = 0;

static lv_timer_t *service_timer;

/* ============================================================
 * 小工具
 * ============================================================ */
static lv_obj_t *mk_screen(void)
{
   lv_obj_t *s = lv_obj_create(NULL);
   lv_obj_set_style_bg_color(s, C_BG, 0);
   lv_obj_set_style_border_width(s, 0, 0);
   lv_obj_set_style_pad_all(s, 0, 0);
   lv_obj_set_style_text_color(s, C_INK, 0);
   lv_obj_set_style_text_font(s, &lv_font_cn_14, 0);
   lv_obj_clear_flag(s, LV_OBJ_FLAG_SCROLLABLE);
   return s;
}

static lv_obj_t *mk_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color, const char *txt)
{
   lv_obj_t *l = lv_label_create(parent);
   lv_obj_set_style_text_font(l, font, 0);
   lv_obj_set_style_text_color(l, color, 0);
   lv_label_set_text(l, txt);
   return l;
}

/* 日志/系统诊断图标: 用几何线条/矩形直接画, 不需要位图素材 */
static lv_obj_t *mk_icon_box(lv_obj_t *parent, lv_color_t accent)
{
   lv_obj_t *box = lv_obj_create(parent);
   lv_obj_set_size(box, 42, 42);
   lv_obj_set_style_bg_color(box, lv_color_white(), 0);
   lv_obj_set_style_bg_opa(box, LV_OPA_10, 0);
   lv_obj_set_style_border_color(box, C_BORDER, 0);
   lv_obj_set_style_border_width(box, 1, 0);
   lv_obj_set_style_radius(box, 10, 0);
   lv_obj_set_style_pad_all(box, 0, 0);
   lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_clear_flag(box, LV_OBJ_FLAG_CLICKABLE);
   (void)accent;
   return box;
}

static void icon_log_draw(lv_obj_t *box, lv_color_t color)
{
   static lv_point_t p1[2] = {{6, 12}, {36, 12}};
   static lv_point_t p2[2] = {{6, 21}, {36, 21}};
   static lv_point_t p3[2] = {{6, 30}, {26, 30}};
   static lv_style_t st;
   lv_style_init(&st);
   lv_style_set_line_width(&st, 2);
   lv_style_set_line_color(&st, color);
   lv_style_set_line_rounded(&st, true);

   lv_obj_t *l1 = lv_line_create(box); lv_line_set_points(l1, p1, 2); lv_obj_add_style(l1, &st, 0);
   lv_obj_t *l2 = lv_line_create(box); lv_line_set_points(l2, p2, 2); lv_obj_add_style(l2, &st, 0);
   lv_obj_t *l3 = lv_line_create(box); lv_line_set_points(l3, p3, 2); lv_obj_add_style(l3, &st, 0);
}

static void icon_sysinfo_draw(lv_obj_t *box, lv_color_t color)
{
   lv_obj_t *outer = lv_obj_create(box);
   lv_obj_set_size(outer, 26, 26);
   lv_obj_align(outer, LV_ALIGN_CENTER, 0, 0);
   lv_obj_set_style_bg_opa(outer, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_color(outer, color, 0);
   lv_obj_set_style_border_width(outer, 2, 0);
   lv_obj_set_style_radius(outer, 3, 0);
   lv_obj_set_style_pad_all(outer, 0, 0);
   lv_obj_clear_flag(outer, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_clear_flag(outer, LV_OBJ_FLAG_CLICKABLE);

   lv_obj_t *inner = lv_obj_create(outer);
   lv_obj_set_size(inner, 10, 10);
   lv_obj_center(inner);
   lv_obj_set_style_bg_color(inner, color, 0);
   lv_obj_set_style_bg_opa(inner, LV_OPA_COVER, 0);
   lv_obj_set_style_border_width(inner, 0, 0);
   lv_obj_set_style_radius(inner, 1, 0);
   lv_obj_clear_flag(inner, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_clear_flag(inner, LV_OBJ_FLAG_CLICKABLE);
}

/* ============================================================
 * 开机自检页
 * ============================================================ */
static lv_obj_t *mk_boot_row(lv_obj_t *parent, const char *label_txt)
{
   lv_obj_t *row = lv_obj_create(parent);
   lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
   lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(row, 0, 0);
   lv_obj_set_style_pad_all(row, 0, 0);
   lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
   lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

   mk_label(row, &lv_font_cn_14, C_INK_DIM, label_txt);
   lv_obj_t *status = mk_label(row, &lv_font_cn_14, C_INK_DIM, "PENDING");
   return status;
}

static void build_boot_screen(void)
{
   scr_boot = mk_screen();

   lv_obj_t *root = lv_obj_create(scr_boot);
   lv_obj_set_size(root, 320, 480);
   lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(root, 0, 0);
   lv_obj_set_style_pad_all(root, 20, 0);
   lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(root, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_flex_align(root, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

   /* 顶栏: 名称 + 芯片标签 */
   lv_obj_t *hdr = lv_obj_create(root);
   lv_obj_set_size(hdr, LV_PCT(100), LV_SIZE_CONTENT);
   lv_obj_set_style_bg_opa(hdr, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, 0);
   lv_obj_set_style_border_color(hdr, C_BORDER, 0);
   lv_obj_set_style_border_width(hdr, 1, 0);
   lv_obj_set_style_pad_bottom(hdr, 10, 0);
   lv_obj_set_style_pad_all(hdr, 0, 0);
   lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
   lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
   mk_label(hdr, &lv_font_cn_14, C_INK, "ANKLE-OS V2.4");
   lv_obj_t *tag = mk_label(hdr, &lv_font_cn_14, C_ORANGE, "STM32F407ZG");
   lv_obj_set_style_bg_color(tag, C_SURFACE, 0);
   lv_obj_set_style_bg_opa(tag, LV_OPA_COVER, 0);
   lv_obj_set_style_border_color(tag, C_BORDER, 0);
   lv_obj_set_style_border_width(tag, 1, 0);
   lv_obj_set_style_pad_hor(tag, 6, 0);
   lv_obj_set_style_pad_ver(tag, 2, 0);
   lv_obj_set_style_radius(tag, 4, 0);

   /* 中段: 品牌名 + 自检清单 */
   lv_obj_t *mid = lv_obj_create(root);
   lv_obj_set_size(mid, LV_PCT(100), LV_SIZE_CONTENT);
   lv_obj_set_style_bg_opa(mid, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(mid, 0, 0);
   lv_obj_set_style_pad_all(mid, 0, 0);
   lv_obj_clear_flag(mid, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(mid, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_style_pad_row(mid, 16, 0);
   lv_obj_align(mid, LV_ALIGN_CENTER, 0, 0);

   mk_label(mid, &lv_font_cn_20, lv_color_white(), "踝康复平台");

   lv_obj_t *chk = lv_obj_create(mid);
   lv_obj_set_size(chk, 280, LV_SIZE_CONTENT);
   lv_obj_set_style_bg_color(chk, C_SURFACE, 0);
   lv_obj_set_style_bg_opa(chk, LV_OPA_COVER, 0);
   lv_obj_set_style_border_color(chk, C_BORDER, 0);
   lv_obj_set_style_border_width(chk, 1, 0);
   lv_obj_set_style_radius(chk, 8, 0);
   lv_obj_set_style_pad_all(chk, 12, 0);
   lv_obj_clear_flag(chk, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(chk, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_style_pad_row(chk, 8, 0);

   boot_chk_mcu = mk_boot_row(chk, "[MCU] STM32F407");
   boot_chk_bus = mk_boot_row(chk, "[BUS] EtherCAT Master");
   boot_chk_drv = mk_boot_row(chk, "[DRV] Servo Enable");

   /* 底部: 阶段文字 + 进度条 */
   lv_obj_t *bot = lv_obj_create(root);
   lv_obj_set_size(bot, LV_PCT(100), LV_SIZE_CONTENT);
   lv_obj_set_style_bg_opa(bot, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(bot, 0, 0);
   lv_obj_set_style_pad_all(bot, 0, 0);
   lv_obj_clear_flag(bot, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(bot, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_style_pad_row(bot, 6, 0);

   lv_obj_t *row = lv_obj_create(bot);
   lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
   lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(row, 0, 0);
   lv_obj_set_style_pad_all(row, 0, 0);
   lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
   lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
   boot_phase_label = mk_label(row, &lv_font_cn_14, C_INK_SUB, "硬件初始化...");
   boot_pct_label   = mk_label(row, &lv_font_cn_14, C_INK_SUB, "0%");

   boot_bar = lv_bar_create(bot);
   lv_obj_set_size(boot_bar, LV_PCT(100), 4);
   lv_obj_set_style_bg_color(boot_bar, C_SURFACE, LV_PART_MAIN);
   lv_obj_set_style_bg_opa(boot_bar, LV_OPA_COVER, LV_PART_MAIN);
   lv_obj_set_style_border_color(boot_bar, C_BORDER, LV_PART_MAIN);
   lv_obj_set_style_border_width(boot_bar, 1, LV_PART_MAIN);
   lv_obj_set_style_bg_color(boot_bar, C_ORANGE, LV_PART_INDICATOR);
   lv_obj_set_style_bg_opa(boot_bar, LV_OPA_COVER, LV_PART_INDICATOR);
   lv_bar_set_range(boot_bar, 0, 100);
   lv_bar_set_value(boot_bar, 0, LV_ANIM_OFF);

   lv_scr_load(scr_boot);
}

/* ============================================================
 * 主页
 * ============================================================ */
static void tile_log_cb(lv_event_t *e);
static void tile_sysinfo_cb(lv_event_t *e);

static lv_obj_t *mk_tile(lv_obj_t *parent, lv_color_t accent, const char *name, int is_log)
{
   lv_obj_t *tile = lv_btn_create(parent);
   lv_obj_set_size(tile, 130, 140);
   lv_obj_set_style_bg_color(tile, C_SURFACE, 0);
   lv_obj_set_style_bg_color(tile, C_SURFACE_H, LV_STATE_PRESSED);
   lv_obj_set_style_border_color(tile, C_BORDER, 0);
   lv_obj_set_style_border_width(tile, 1, 0);
   lv_obj_set_style_radius(tile, 12, 0);
   lv_obj_set_style_pad_all(tile, 14, 0);
   lv_obj_set_style_shadow_width(tile, 0, 0);
   lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

   lv_obj_t *box = mk_icon_box(tile, accent);
   if (is_log) icon_log_draw(box, accent); else icon_sysinfo_draw(box, accent);

   mk_label(tile, &lv_font_cn_14, lv_color_white(), name);

   lv_obj_add_event_cb(tile, is_log ? tile_log_cb : tile_sysinfo_cb, LV_EVENT_CLICKED, NULL);
   return tile;
}

static void build_home_screen(void)
{
   scr_home = mk_screen();

   /* 状态栏 */
   lv_obj_t *sb = lv_obj_create(scr_home);
   lv_obj_set_size(sb, 320, LV_SIZE_CONTENT);
   lv_obj_set_style_bg_color(sb, C_SURFACE, 0);
   lv_obj_set_style_bg_opa(sb, LV_OPA_COVER, 0);
   lv_obj_set_style_border_side(sb, LV_BORDER_SIDE_BOTTOM, 0);
   lv_obj_set_style_border_color(sb, C_BORDER, 0);
   lv_obj_set_style_border_width(sb, 1, 0);
   lv_obj_set_style_pad_hor(sb, 14, 0);
   lv_obj_set_style_pad_ver(sb, 12, 0);
   lv_obj_clear_flag(sb, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(sb, LV_FLEX_FLOW_ROW);
   lv_obj_set_flex_align(sb, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

   home_uptime_label = mk_label(sb, &lv_font_cn_14, C_ORANGE, "00:00:00");
   lv_obj_set_style_bg_color(home_uptime_label, C_ORANGE, 0);
   lv_obj_set_style_bg_opa(home_uptime_label, LV_OPA_20, 0);
   lv_obj_set_style_pad_hor(home_uptime_label, 8, 0);
   lv_obj_set_style_pad_ver(home_uptime_label, 4, 0);
   lv_obj_set_style_radius(home_uptime_label, 5, 0);

   lv_obj_t *statgrp = lv_obj_create(sb);
   lv_obj_set_size(statgrp, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
   lv_obj_set_style_bg_opa(statgrp, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(statgrp, 0, 0);
   lv_obj_set_style_pad_all(statgrp, 0, 0);
   lv_obj_clear_flag(statgrp, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(statgrp, LV_FLEX_FLOW_ROW);
   lv_obj_set_flex_align(statgrp, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
   lv_obj_set_style_pad_column(statgrp, 6, 0);

   home_status_dot = lv_obj_create(statgrp);
   lv_obj_set_size(home_status_dot, 7, 7);
   lv_obj_set_style_radius(home_status_dot, LV_RADIUS_CIRCLE, 0);
   lv_obj_set_style_bg_color(home_status_dot, C_EMERALD, 0);
   lv_obj_set_style_bg_opa(home_status_dot, LV_OPA_COVER, 0);
   lv_obj_set_style_border_width(home_status_dot, 0, 0);
   lv_obj_clear_flag(home_status_dot, LV_OBJ_FLAG_SCROLLABLE);

   home_status_label = mk_label(statgrp, &lv_font_cn_14, C_INK, "Connected");

   /* 内容区 */
   lv_obj_t *body = lv_obj_create(scr_home);
   lv_obj_set_size(body, 320, 480 - 50);
   lv_obj_align(body, LV_ALIGN_TOP_MID, 0, 50);
   lv_obj_set_style_bg_opa(body, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(body, 0, 0);
   lv_obj_set_style_pad_all(body, 14, 0);
   lv_obj_clear_flag(body, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(body, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_style_pad_row(body, 14, 0);

   lv_obj_t *banner = lv_obj_create(body);
   lv_obj_set_size(banner, LV_PCT(100), LV_SIZE_CONTENT);
   lv_obj_set_style_bg_color(banner, C_SURFACE, 0);
   lv_obj_set_style_bg_opa(banner, LV_OPA_COVER, 0);
   lv_obj_set_style_border_color(banner, C_BORDER, 0);
   lv_obj_set_style_border_width(banner, 1, 0);
   lv_obj_set_style_border_side(banner, LV_BORDER_SIDE_TOP, 0);
   lv_obj_set_style_radius(banner, 8, 0);
   lv_obj_set_style_pad_all(banner, 14, 0);
   lv_obj_clear_flag(banner, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(banner, LV_FLEX_FLOW_ROW);
   lv_obj_set_flex_align(banner, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
   mk_label(banner, &lv_font_cn_14, lv_color_white(), "控制终端总览");
   lv_obj_t *badge = mk_label(banner, &lv_font_cn_14, C_EMERALD, "伺服使能");
   lv_obj_set_style_bg_color(badge, C_EMERALD, 0);
   lv_obj_set_style_bg_opa(badge, LV_OPA_20, 0);
   lv_obj_set_style_pad_hor(badge, 8, 0);
   lv_obj_set_style_pad_ver(badge, 3, 0);
   lv_obj_set_style_radius(badge, 4, 0);

   lv_obj_t *tiles = lv_obj_create(body);
   lv_obj_set_size(tiles, LV_PCT(100), LV_SIZE_CONTENT);
   lv_obj_set_style_bg_opa(tiles, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(tiles, 0, 0);
   lv_obj_set_style_pad_all(tiles, 0, 0);
   lv_obj_clear_flag(tiles, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(tiles, LV_FLEX_FLOW_ROW_WRAP);
   lv_obj_set_style_pad_column(tiles, 12, 0);
   lv_obj_set_style_pad_row(tiles, 12, 0);

   mk_tile(tiles, C_ORANGE, "实时日志", 1);
   mk_tile(tiles, C_BLUE,   "系统诊断", 0);

   lv_scr_load_anim(scr_home, LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, true);
}

/* ============================================================
 * 实时日志页
 * ============================================================ */
static lv_color_t cls_color(uint8_t cls)
{
   switch (cls) {
      case GUI_LOG_OK:   return C_EMERALD;
      case GUI_LOG_WARN: return C_AMBER;
      case GUI_LOG_ERR:  return C_ROSE;
      case GUI_LOG_DATA: return C_BLUE;
      default:           return C_INK_SUB;
   }
}

static void log_back_cb(lv_event_t *e)
{
   (void)e;
   log_body = NULL;
   lv_scr_load_anim(scr_home, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, true);
}

static void log_clear_cb(lv_event_t *e)
{
   (void)e;
   lv_obj_clean(log_body);
   log_last_seq = gui_log_seq();
}

static void log_pause_cb(lv_event_t *e)
{
   lv_obj_t *btn = lv_event_get_target(e);
   log_paused = !log_paused;
   lv_label_set_text(lv_obj_get_child(btn, 0), log_paused ? "已暂停" : "暂停");
   if (log_paused) lv_obj_set_style_text_color(lv_obj_get_child(btn, 0), C_AMBER, 0);
   else            lv_obj_set_style_text_color(lv_obj_get_child(btn, 0), C_INK_SUB, 0);
}

static lv_obj_t *mk_hdr_btn(lv_obj_t *parent, const char *txt)
{
   lv_obj_t *btn = lv_btn_create(parent);
   lv_obj_set_style_bg_color(btn, C_SURFACE, 0);
   lv_obj_set_style_bg_color(btn, C_SURFACE_H, LV_STATE_PRESSED);
   lv_obj_set_style_border_color(btn, C_BORDER, 0);
   lv_obj_set_style_border_width(btn, 1, 0);
   lv_obj_set_style_radius(btn, 6, 0);
   lv_obj_set_style_shadow_width(btn, 0, 0);
   lv_obj_set_style_pad_hor(btn, 10, 0);
   lv_obj_set_style_pad_ver(btn, 5, 0);
   lv_obj_set_size(btn, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
   mk_label(btn, &lv_font_cn_14, C_INK_SUB, txt);
   return btn;
}

static void build_log_screen(void)
{
   scr_log = mk_screen();

   lv_obj_t *hdr = lv_obj_create(scr_log);
   lv_obj_set_size(hdr, 320, LV_SIZE_CONTENT);
   lv_obj_set_style_bg_color(hdr, C_SURFACE, 0);
   lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
   lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, 0);
   lv_obj_set_style_border_color(hdr, C_BORDER, 0);
   lv_obj_set_style_border_width(hdr, 1, 0);
   lv_obj_set_style_pad_all(hdr, 10, 0);
   lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
   lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

   lv_obj_t *back = mk_hdr_btn(hdr, "\xe2\x80\xb9 返回");   /* U+2039 ‹ */
   lv_obj_add_event_cb(back, log_back_cb, LV_EVENT_CLICKED, NULL);

   mk_label(hdr, &lv_font_cn_14, lv_color_white(), "实时终端日志");

   lv_obj_t *actions = lv_obj_create(hdr);
   lv_obj_set_size(actions, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
   lv_obj_set_style_bg_opa(actions, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(actions, 0, 0);
   lv_obj_set_style_pad_all(actions, 0, 0);
   lv_obj_clear_flag(actions, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(actions, LV_FLEX_FLOW_ROW);
   lv_obj_set_style_pad_column(actions, 6, 0);

   lv_obj_t *clear_btn = mk_hdr_btn(actions, "清屏");
   lv_obj_add_event_cb(clear_btn, log_clear_cb, LV_EVENT_CLICKED, NULL);
   lv_obj_t *pause_btn = mk_hdr_btn(actions, log_paused ? "已暂停" : "暂停");
   lv_obj_add_event_cb(pause_btn, log_pause_cb, LV_EVENT_CLICKED, NULL);

   log_body = lv_obj_create(scr_log);
   lv_obj_set_size(log_body, 320, 480 - 50);
   lv_obj_align(log_body, LV_ALIGN_TOP_MID, 0, 50);
   lv_obj_set_style_bg_color(log_body, lv_color_hex(0x050608), 0);
   lv_obj_set_style_bg_opa(log_body, LV_OPA_COVER, 0);
   lv_obj_set_style_border_width(log_body, 0, 0);
   lv_obj_set_style_pad_all(log_body, 10, 0);
   lv_obj_set_flex_flow(log_body, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_style_pad_row(log_body, 3, 0);

   /* 进页面先把已有日志(最多塞满一屏)补出来, 而不是从这次进入以后才开始记 */
   {
      uint32_t seq = gui_log_seq();
      uint32_t start = (seq > GUI_LOG_CAP) ? (seq - GUI_LOG_CAP) : 0;
      log_last_seq = start;
   }

   lv_scr_load_anim(scr_log, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false);
}

static void log_pump(void)
{
   uint32_t seq = gui_log_seq();
   if (!log_body) { log_last_seq = seq; return; }

   for (; log_last_seq < seq; log_last_seq++) {
      const gui_log_entry_t *e = gui_log_at(log_last_seq);
      char line[16 + GUI_LOG_MSG_LEN];
      uint32_t s = e->t_ms / 1000;
      snprintf(line, sizeof(line), "[%02lu:%02lu:%02lu] %s",
               (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60), e->msg);

      lv_obj_t *row = lv_label_create(log_body);
      lv_obj_set_width(row, LV_PCT(100));
      lv_label_set_long_mode(row, LV_LABEL_LONG_WRAP);
      lv_obj_set_style_text_font(row, &lv_font_cn_14, 0);
      lv_obj_set_style_text_color(row, cls_color(e->cls), 0);
      lv_label_set_text(row, line);

      while (lv_obj_get_child_cnt(log_body) > GUI_LOG_CAP)
         lv_obj_del(lv_obj_get_child(log_body, 0));
   }
   if (!log_paused) lv_obj_scroll_to_y(log_body, LV_COORD_MAX, LV_ANIM_OFF);
}

/* ============================================================
 * 系统诊断页
 * ============================================================ */
static lv_obj_t *mk_kv_row(lv_obj_t *parent, const char *k, const char *v, lv_color_t vc)
{
   lv_obj_t *row = lv_obj_create(parent);
   lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
   lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(row, 0, 0);
   lv_obj_set_style_pad_all(row, 0, 0);
   lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
   lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
   mk_label(row, &lv_font_cn_14, C_INK_SUB, k);
   return mk_label(row, &lv_font_cn_14, vc, v);
}

static lv_obj_t *mk_sys_card(lv_obj_t *parent, const char *title)
{
   lv_obj_t *card = lv_obj_create(parent);
   lv_obj_set_size(card, LV_PCT(100), LV_SIZE_CONTENT);
   lv_obj_set_style_bg_color(card, C_SURFACE, 0);
   lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
   lv_obj_set_style_border_color(card, C_BORDER, 0);
   lv_obj_set_style_border_width(card, 1, 0);
   lv_obj_set_style_radius(card, 8, 0);
   lv_obj_set_style_pad_all(card, 12, 0);
   lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_style_pad_row(card, 8, 0);

   lv_obj_t *t = mk_label(card, &lv_font_cn_14, C_ORANGE, title);
   lv_obj_set_style_border_side(t, LV_BORDER_SIDE_BOTTOM, 0);
   lv_obj_set_style_border_color(t, C_BORDER, 0);
   lv_obj_set_style_border_width(t, 1, 0);
   lv_obj_set_style_pad_bottom(t, 6, 0);
   lv_obj_set_width(t, LV_PCT(100));
   return card;
}

static void sysinfo_back_cb(lv_event_t *e)
{
   (void)e;
   sysinfo_conn_label = NULL;
   sysinfo_wkc_label  = NULL;
   lv_scr_load_anim(scr_home, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, true);
}

static void build_sysinfo_screen(void)
{
   scr_sysinfo = mk_screen();

   lv_obj_t *hdr = lv_obj_create(scr_sysinfo);
   lv_obj_set_size(hdr, 320, LV_SIZE_CONTENT);
   lv_obj_set_style_bg_color(hdr, C_SURFACE, 0);
   lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
   lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, 0);
   lv_obj_set_style_border_color(hdr, C_BORDER, 0);
   lv_obj_set_style_border_width(hdr, 1, 0);
   lv_obj_set_style_pad_all(hdr, 10, 0);
   lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);
   lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
   lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
   lv_obj_t *back = mk_hdr_btn(hdr, "\xe2\x80\xb9 返回");
   lv_obj_add_event_cb(back, sysinfo_back_cb, LV_EVENT_CLICKED, NULL);
   mk_label(hdr, &lv_font_cn_14, lv_color_white(), "系统硬件诊断");
   lv_obj_t *spacer = lv_obj_create(hdr);   /* 占位, 让标题保持居中 */
   lv_obj_set_size(spacer, 1, 1);
   lv_obj_set_style_bg_opa(spacer, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(spacer, 0, 0);

   lv_obj_t *body = lv_obj_create(scr_sysinfo);
   lv_obj_set_size(body, 320, 480 - 50);
   lv_obj_align(body, LV_ALIGN_TOP_MID, 0, 50);
   lv_obj_set_style_bg_opa(body, LV_OPA_TRANSP, 0);
   lv_obj_set_style_border_width(body, 0, 0);
   lv_obj_set_style_pad_all(body, 12, 0);
   lv_obj_set_flex_flow(body, LV_FLEX_FLOW_COLUMN);
   lv_obj_set_style_pad_row(body, 12, 0);

   lv_obj_t *mcu = mk_sys_card(body, "主控微处理器 (MCU)");
   mk_kv_row(mcu, "芯片型号", "STM32F407ZGT6", C_INK);
   mk_kv_row(mcu, "内核架构", "Cortex-M4 @ 168MHz", C_INK);
   mk_kv_row(mcu, "SRAM / Flash", "192KB / 1024KB", C_INK);
   mk_kv_row(mcu, "显示接口", "FSMC Bank1 NE4", C_INK);

   lv_obj_t *ec = mk_sys_card(body, "EtherCAT 总线网络");
   sysinfo_conn_label = mk_kv_row(ec, "通信状态", g_ec_connected ? "Connected" : "Unconnected",
                                   g_ec_connected ? C_EMERALD : C_ROSE);
   mk_kv_row(ec, "同步周期", "4.00 ms", C_INK);
   {
      char buf[16];
      snprintf(buf, sizeof(buf), "%d / %d", g_ec_wkc, g_ec_slaves * 3);
      sysinfo_wkc_label = mk_kv_row(ec, "工作计数 (WKC)", buf, C_INK);
   }
   {
      char buf[8];
      snprintf(buf, sizeof(buf), "%d 轴", g_ec_slaves);
      mk_kv_row(ec, "伺服轴数", buf, C_INK);
   }

   lv_scr_load_anim(scr_sysinfo, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, false);
}

/* ============================================================
 * 主页图块点击回调(放这里是因为要引用build_xxx_screen)
 * ============================================================ */
static void tile_log_cb(lv_event_t *e)      { (void)e; build_log_screen(); }
static void tile_sysinfo_cb(lv_event_t *e)  { (void)e; build_sysinfo_screen(); }

/* ============================================================
 * 周期服务(待机循环里调)
 * ============================================================ */
static void service_timer_cb(lv_timer_t *t)
{
   (void)t;
   uint32_t s = HAL_GetTick() / 1000;

   if (home_uptime_label) {
      char buf[12];
      snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu",
               (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
      lv_label_set_text(home_uptime_label, buf);
   }
   if (home_status_dot) {
      lv_obj_set_style_bg_color(home_status_dot, g_ec_connected ? C_EMERALD : C_ROSE, 0);
      lv_label_set_text(home_status_label, g_ec_connected ? "Connected" : "Unconnected");
   }
   if (sysinfo_conn_label) {
      lv_label_set_text(sysinfo_conn_label, g_ec_connected ? "Connected" : "Unconnected");
      lv_obj_set_style_text_color(sysinfo_conn_label, g_ec_connected ? C_EMERALD : C_ROSE, 0);
   }
   if (sysinfo_wkc_label) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%d / %d", g_ec_wkc, g_ec_slaves * 3);
      lv_label_set_text(sysinfo_wkc_label, buf);
   }

   log_pump();
}

/* ============================================================
 * 公开API
 * ============================================================ */
void gui_init(void)
{
   build_boot_screen();
   service_timer = lv_timer_create(service_timer_cb, 400, NULL);
}

void gui_boot_mcu_ok(void)
{
   if (!scr_boot) return;   /* 保护: 开机页已经切走(理论上不该再调到这几个函数) */
   lv_label_set_text(boot_chk_mcu, "OK");
   lv_obj_set_style_text_color(boot_chk_mcu, C_EMERALD, 0);
   lv_label_set_text(boot_phase_label, "内核系统初始化...");
   lv_bar_set_value(boot_bar, 25, LV_ANIM_ON);
   lv_label_set_text(boot_pct_label, "25%");
   lv_timer_handler();
}

void gui_boot_bus_ok(int slave_count)
{
   char buf[16];
   if (!scr_boot) return;
   snprintf(buf, sizeof(buf), "OK (%d/%d)", slave_count, slave_count);
   lv_label_set_text(boot_chk_bus, buf);
   lv_obj_set_style_text_color(boot_chk_bus, C_EMERALD, 0);
   lv_label_set_text(boot_phase_label, "扫描 EtherCAT 节点...");
   lv_bar_set_value(boot_bar, 65, LV_ANIM_ON);
   lv_label_set_text(boot_pct_label, "65%");
   lv_timer_handler();
}

void gui_boot_drv_enable(void)
{
   if (!scr_boot) return;
   lv_label_set_text(boot_chk_drv, "ENABLE");
   lv_obj_set_style_text_color(boot_chk_drv, C_EMERALD, 0);
   lv_label_set_text(boot_phase_label, "配置 PDO & 使能伺服...");
   lv_bar_set_value(boot_bar, 90, LV_ANIM_ON);
   lv_label_set_text(boot_pct_label, "90%");
   lv_timer_handler();
}

void gui_boot_ready_and_show_home(void)
{
   if (!scr_boot) return;
   lv_label_set_text(boot_phase_label, "系统就绪");
   lv_bar_set_value(boot_bar, 100, LV_ANIM_ON);
   lv_label_set_text(boot_pct_label, "100%");
   lv_timer_handler();

   build_home_screen();
   scr_boot = NULL;   /* auto_del已经把开机页对象释放了, 清指针防止后面误用 */
}

void gui_set_ec_status(int connected, int wkc, int slave_count)
{
   g_ec_connected = connected;
   g_ec_wkc = wkc;
   g_ec_slaves = slave_count;
}

void gui_service(void)
{
   lv_timer_handler();
}
