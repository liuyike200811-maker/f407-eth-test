/*
 * 时间戳用等宽数字字模(0-9和冒号) —— 自动生成, 不要手改(生成方式同log_phrases.h)。
 * 每字10×18px, 1bpp, 按"0123456789:"顺序排列, 第N个字符位图从
 * digit_bitmap[N * DIGIT_CELL_ROWBYTES * DIGIT_CELL_H]开始。
 */
#ifndef _digit_font_h_
#define _digit_font_h_
#include <stdint.h>

#define DIGIT_CELL_W 10
#define DIGIT_CELL_H 18
#define DIGIT_CELL_ROWBYTES  ((DIGIT_CELL_W + 7) / 8)

extern const uint8_t digit_bitmap[];

#endif
