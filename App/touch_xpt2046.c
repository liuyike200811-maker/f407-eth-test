/*
 * XPT2046触摸驱动实现 —— 软件SPI, 寄存器级GPIO(与项目其余驱动风格一致)。
 * 命令字节0xD0/0x90是XPT2046手册里的标准差分12位读X/Y命令, 不是猜的。
 * 校准范围(X/Y_RAW_MIN/MAX)和方向翻转开关是经验起始值, 台架实测后再调
 * (见touch_xpt2046.h顶部注释)。
 */
#include "stm32f4xx_hal.h"
#include "touch_xpt2046.h"
#include "lcd_fsmc.h"

/* ---- 台架待定项: 原始ADC范围与方向, 实测后改这几个 #define ---- */
#define X_RAW_MIN   300
#define X_RAW_MAX   3900
#define Y_RAW_MIN   300
#define Y_RAW_MAX   3900
#define TOUCH_SWAP_XY   0   /* 触摸X/Y跟屏幕X/Y对调 → 改成1 */
#define TOUCH_INVERT_X  0   /* 横向坐标反了 → 改成1 */
#define TOUCH_INVERT_Y  0   /* 纵向坐标反了 → 改成1 */

#define CMD_READ_X  0xD0
#define CMD_READ_Y  0x90

#define TCLK_SET(v)  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  (v) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define TDIN_SET(v)  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, (v) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define TCS_SET(v)   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (v) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define DOUT_GET()   HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)
#define PEN_GET()    HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)

static void spi_delay(void)
{
   volatile int i;
   for (i = 0; i < 6; i++) { __NOP(); }
}

void touch_init(void)
{
   GPIO_InitTypeDef g = {0};

   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOF_CLK_ENABLE();

   /* SCK/MOSI/CS: 普通推挽输出 */
   g.Mode  = GPIO_MODE_OUTPUT_PP;
   g.Pull  = GPIO_NOPULL;
   g.Speed = GPIO_SPEED_FREQ_LOW;
   g.Pin   = GPIO_PIN_0;               /* PB0 = T_SCK */
   HAL_GPIO_Init(GPIOB, &g);
   g.Pin   = GPIO_PIN_11;              /* PF11 = T_MOSI */
   HAL_GPIO_Init(GPIOF, &g);
   g.Pin   = GPIO_PIN_13;              /* PC13 = T_CS */
   HAL_GPIO_Init(GPIOC, &g);

   /* MISO: 输入 */
   g.Mode  = GPIO_MODE_INPUT;
   g.Pull  = GPIO_NOPULL;
   g.Pin   = GPIO_PIN_2;               /* PB2 = T_MISO */
   HAL_GPIO_Init(GPIOB, &g);

   /* PEN: 输入上拉(触笔中断, 低有效, 空闲时靠上拉拉高) */
   g.Mode  = GPIO_MODE_INPUT;
   g.Pull  = GPIO_PULLUP;
   g.Pin   = GPIO_PIN_1;               /* PB1 = T_PEN */
   HAL_GPIO_Init(GPIOB, &g);

   TCS_SET(1);
   TCLK_SET(0);
}

/* 软件SPI: 发一个命令字节, 读回12位ADC结果(高位在前, 丢弃末尾4个无效位) */
static uint16_t xpt_read_cmd(uint8_t cmd)
{
   uint16_t result = 0;
   int i;

   TCS_SET(0);

   /* 发命令字节, MSB先出 */
   for (i = 7; i >= 0; i--) {
      TDIN_SET((cmd >> i) & 0x01);
      spi_delay();
      TCLK_SET(1);
      spi_delay();
      TCLK_SET(0);
   }
   spi_delay();

   /* 读12位结果, MSB先进 */
   for (i = 0; i < 12; i++) {
      TCLK_SET(1);
      spi_delay();
      TCLK_SET(0);
      result <<= 1;
      if (DOUT_GET() == GPIO_PIN_SET) result |= 0x01;
      spi_delay();
   }

   TCS_SET(1);
   return result;
}

/* 同一通道采样N次, 掐头(丢第一次)后取平均, 抑制读数噪声 */
static uint16_t xpt_read_avg(uint8_t cmd)
{
   uint16_t s1, s2, s3;
   xpt_read_cmd(cmd);           /* 掐头: 第一次读数常不稳定, 丢掉 */
   s1 = xpt_read_cmd(cmd);
   s2 = xpt_read_cmd(cmd);
   s3 = xpt_read_cmd(cmd);
   return (uint16_t)(((uint32_t)s1 + s2 + s3) / 3);
}

int touch_get_point(uint16_t *x, uint16_t *y)
{
   uint16_t rawx, rawy;
   int32_t px, py;

   if (PEN_GET() == GPIO_PIN_SET) return 0;   /* 没按下 */

   rawx = xpt_read_avg(CMD_READ_X);
   rawy = xpt_read_avg(CMD_READ_Y);

   if (PEN_GET() == GPIO_PIN_SET) return 0;   /* 采样过程中抬笔了, 这次不算 */
   if (rawx < X_RAW_MIN || rawx > X_RAW_MAX) return 0;
   if (rawy < Y_RAW_MIN || rawy > Y_RAW_MAX) return 0;

#if TOUCH_SWAP_XY
   { uint16_t t = rawx; rawx = rawy; rawy = t; }
#endif

   px = (int32_t)(rawx - X_RAW_MIN) * lcd_width()  / (X_RAW_MAX - X_RAW_MIN);
   py = (int32_t)(rawy - Y_RAW_MIN) * lcd_height() / (Y_RAW_MAX - Y_RAW_MIN);

#if TOUCH_INVERT_X
   px = lcd_width() - 1 - px;
#endif
#if TOUCH_INVERT_Y
   py = lcd_height() - 1 - py;
#endif

   if (px < 0) px = 0;
   if (px >= lcd_width())  px = lcd_width()  - 1;
   if (py < 0) py = 0;
   if (py >= lcd_height()) py = lcd_height() - 1;

   *x = (uint16_t)px;
   *y = (uint16_t)py;
   return 1;
}
