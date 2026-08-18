/*
 * 板载3.5寸TFT彩屏驱动实现 —— 寄存器级FSMC Bank1 NE4, 不依赖HAL_SRAM/LVGL。
 * (与本项目其余外设驱动风格一致: uart_log.c/modbus_slave.c 都是寄存器级直接访问)
 *
 * 面板初始化时序照抄《普中STM32-F407-战神开发板资料》官方tftlcd.c驱动源码里
 * TFTLCD_HX8357DN 分支(命令码/参数逐条核对过); FSMC总线时序未沿用官方数值
 * (官方 FSMC_AddressHoldTime=0x15 超出ADDHLD寄存器位宽[3:0]最大0xF, 是靠溢出
 * 歪打正着能用, 不适合照抄), 改用下方注释里推导的、在位宽范围内留足裕量的
 * 保守时序。方向直接用竖屏值(0x36=0x4C), 320×480——横屏(0x2C)已经在台架上
 * 验证过是错的(内容整体转90°)。
 */
#include "stm32f4xx_hal.h"
#include "lcd_fsmc.h"

/* ---- FSMC Bank1 NE4 地址映射: A6作命令/数据选择线 ----
 * 16位总线下MCU内部地址右移1位对齐外部地址线, A6对应字节地址0x80,
 * 即"寄存器地址"(命令口)在总线基址, "数据口"在基址+0x80。 */
#define LCD_BANK4_BASE   0x6C000000UL
typedef struct {
   __IO uint16_t CMD;
   __IO uint16_t DATA;
} LCD_RegDef;
#define LCD_REG   ((LCD_RegDef *)(LCD_BANK4_BASE | 0x7EUL))

static uint16_t g_width  = 320;
static uint16_t g_height = 480;

static inline void lcd_wr_cmd(uint16_t cmd)  { LCD_REG->CMD = cmd; }
static inline void lcd_wr_data(uint16_t d)   { LCD_REG->DATA = d; }
static inline void lcd_wr_cmd_data(uint16_t cmd, uint16_t d) { lcd_wr_cmd(cmd); lcd_wr_data(d); }

/* HX8357DN 内部按8位/次传输RGB565的两个字节 */
static inline void lcd_wr_color(uint16_t color)
{
   LCD_REG->DATA = color >> 8;
   LCD_REG->DATA = color & 0xFF;
}

static void lcd_gpio_init(void)
{
   GPIO_InitTypeDef g = {0};

   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();
   __HAL_RCC_GPIOE_CLK_ENABLE();
   __HAL_RCC_GPIOF_CLK_ENABLE();
   __HAL_RCC_GPIOG_CLK_ENABLE();

   g.Mode      = GPIO_MODE_AF_PP;
   g.Pull      = GPIO_PULLUP;
   g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
   g.Alternate = GPIO_AF12_FSMC;

   g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5
         | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
   HAL_GPIO_Init(GPIOD, &g);

   g.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
         | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
   HAL_GPIO_Init(GPIOE, &g);

   g.Pin = GPIO_PIN_12;   /* A6 */
   HAL_GPIO_Init(GPIOF, &g);

   g.Pin = GPIO_PIN_12;   /* NE4 */
   HAL_GPIO_Init(GPIOG, &g);

   g.Pin       = GPIO_PIN_15;   /* 背光: 普通推挽输出, 不走FSMC复用 */
   g.Mode      = GPIO_MODE_OUTPUT_PP;
   g.Pull      = GPIO_NOPULL;
   g.Speed     = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &g);
}

/* ---- FSMC Bank1 NE4 总线时序 ----
 * HCLK=168MHz, 1cycle≈5.95ns。读: ADDSET=15(89ns) DATAST=96(571ns); 写:
 * ADDSET=8(48ns) ADDHLD=8(48ns) DATAST=10(60ns), 三段共≈156ns/次写。 */
static void lcd_fsmc_bus_init(void)
{
   __HAL_RCC_FSMC_CLK_ENABLE();

   /* BCR4: MBKEN=1(bit0) MWID=01(16位,bit4) WREN=1(bit12) EXTMOD=1(读写分开定时,bit14) */
   FSMC_Bank1->BTCR[6] = (1u << 0) | (1u << 4) | (1u << 12) | (1u << 14);
   /* BTR4(读时序): ADDSET=0xF DATAST=0x60 ACCMOD=A(0) */
   FSMC_Bank1->BTCR[7] = (0xFu << 0) | (0x60u << 8);
   /* BWTR4(写时序): ADDSET=8 ADDHLD=8 DATAST=10 ACCMOD=A(0) */
   FSMC_Bank1E->BWTR[6] = (8u << 0) | (8u << 4) | (10u << 8);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
   lcd_wr_cmd(0x2A);
   lcd_wr_data(x0 >> 8); lcd_wr_data(x0 & 0xFF);
   lcd_wr_data(x1 >> 8); lcd_wr_data(x1 & 0xFF);

   lcd_wr_cmd(0x2B);
   lcd_wr_data(y0 >> 8); lcd_wr_data(y0 & 0xFF);
   lcd_wr_data(y1 >> 8); lcd_wr_data(y1 & 0xFF);

   lcd_wr_cmd(0x2C);   /* 进入GRAM写 */
}

/* HX8357DN初始化时序: 逐条照抄官方tftlcd.c的TFTLCD_HX8357DN分支
 * (Gamma/电源/接口参数), 0x36=0x4C为竖屏地址模式(官方dir=0默认值)。 */
static void lcd_panel_init(void)
{
   lcd_wr_cmd_data(0xE9, 0x20);

   lcd_wr_cmd(0x11);              /* Exit Sleep */
   HAL_Delay(10);

   lcd_wr_cmd_data(0x3A, 0x55);   /* 16bit colors */

   lcd_wr_cmd(0xD1);
   lcd_wr_data(0x00); lcd_wr_data(0x65); lcd_wr_data(0x1F);   /* VCOM电压/灰阶电压 */

   lcd_wr_cmd(0xD0);
   lcd_wr_data(0x07); lcd_wr_data(0x07); lcd_wr_data(0x80);

   lcd_wr_cmd_data(0x36, 0x4C);   /* Set_address_mode: 竖屏 */
   g_width = 320; g_height = 480;

   lcd_wr_cmd(0xC1);
   lcd_wr_data(0x10); lcd_wr_data(0x10); lcd_wr_data(0x02); lcd_wr_data(0x02);

   lcd_wr_cmd(0xC0);              /* Set Default Gamma */
   lcd_wr_data(0x00); lcd_wr_data(0x35); lcd_wr_data(0x00);
   lcd_wr_data(0x00); lcd_wr_data(0x01); lcd_wr_data(0x02);

   lcd_wr_cmd_data(0xC4, 0x03);
   lcd_wr_cmd_data(0xC5, 0x01);   /* Set frame rate */

   lcd_wr_cmd(0xD2);              /* power setting */
   lcd_wr_data(0x01); lcd_wr_data(0x22);

   lcd_wr_cmd_data(0xE7, 0x38);

   lcd_wr_cmd(0xF3);
   lcd_wr_data(0x08); lcd_wr_data(0x12); lcd_wr_data(0x12); lcd_wr_data(0x08);

   lcd_wr_cmd(0xC8);              /* Set Gamma */
   lcd_wr_data(0x01); lcd_wr_data(0x52); lcd_wr_data(0x37); lcd_wr_data(0x10);
   lcd_wr_data(0x0D); lcd_wr_data(0x01); lcd_wr_data(0x04); lcd_wr_data(0x51);
   lcd_wr_data(0x77); lcd_wr_data(0x01); lcd_wr_data(0x01); lcd_wr_data(0x0D);
   lcd_wr_data(0x08); lcd_wr_data(0x80); lcd_wr_data(0x00);

   lcd_wr_cmd(0x29);              /* Display ON */
}

static void lcd_backlight(int on)
{
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void lcd_init(void)
{
   lcd_gpio_init();
   lcd_fsmc_bus_init();
   HAL_Delay(50);
   lcd_panel_init();
   lcd_backlight(1);
   lcd_clear(LCD_COLOR_BLACK);
}

uint16_t lcd_width(void)  { return g_width; }
uint16_t lcd_height(void) { return g_height; }

void lcd_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
   uint32_t n, total;

   if (x0 > x1 || y0 > y1) return;
   lcd_set_window(x0, y0, x1, y1);
   total = (uint32_t)(x1 - x0 + 1) * (uint32_t)(y1 - y0 + 1);
   for (n = 0; n < total; n++) lcd_wr_color(color);
}

void lcd_clear(uint16_t color)
{
   lcd_fill_rect(0, 0, g_width - 1, g_height - 1, color);
}

void lcd_blit(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint16_t *pixels)
{
   uint32_t n, total;

   if (x0 > x1 || y0 > y1) return;
   lcd_set_window(x0, y0, x1, y1);
   total = (uint32_t)(x1 - x0 + 1) * (uint32_t)(y1 - y0 + 1);
   for (n = 0; n < total; n++) lcd_wr_color(pixels[n]);
}
