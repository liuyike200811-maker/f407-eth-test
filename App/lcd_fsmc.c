/*
 * 板载3.5寸TFT彩屏驱动实现 —— 寄存器级FSMC Bank1 NE4, 不依赖HAL_SRAM模块
 * (与本项目其余外设驱动风格一致: uart_log.c/modbus_slave.c 都是寄存器级直接访问)
 *
 * 面板初始化时序照抄《普中STM32-F407-战神开发板资料》官方tftlcd.c驱动源码里
 * TFTLCD_HX8357DN 分支(命令码/参数逐条核对过, 型号已由用户核对实物确认), 只是把
 * StdPeriph 库调用换成本项目惯用的 HAL 寄存器级写法; FSMC总线时序未沿用官方数值
 * (官方 FSMC_AddressHoldTime=0x15 超出ADDHLD寄存器位宽[3:0]最大0xF, 实际是靠溢出
 * 溢出到ADDSET位歪打正着能用, 不适合照抄), 改用下方注释里推导的、在位宽范围内留
 * 足裕量的保守时序。
 *
 * ILI9488分支(此前未确认型号前的试探性实现)仍保留在文件里但已不生效
 * (LCD_PANEL_HX8357DN 已在 lcd_fsmc.h 里选定), 留作四种候选面板之一的参考。
 */
#include "stm32f4xx_hal.h"
#include "lcd_fsmc.h"

/* ---- FSMC Bank1 NE4 地址映射: A6作命令/数据选择线 ----
 * 16位总线下MCU内部地址右移1位对齐外部地址线, A6对应字节地址0x80,
 * 即"寄存器地址"(命令口)在总线基址, "数据口"在基址+0x80(与官方驱动
 * TFTLCD_BASE=(0x6C000000|0x7E)完全一致, 0x7E/2*... 二者是同一份推导)。 */
#define LCD_BANK4_BASE   0x6C000000UL
typedef struct {
   __IO uint16_t CMD;
   __IO uint16_t DATA;
} LCD_RegDef;
#define LCD_REG   ((LCD_RegDef *)(LCD_BANK4_BASE | 0x7EUL))

static uint16_t g_width  = 480;
static uint16_t g_height = 320;

/* ---- 底层总线读写 ---- */
static inline void lcd_wr_cmd(uint16_t cmd)  { LCD_REG->CMD = cmd; }
static inline void lcd_wr_data(uint16_t d)   { LCD_REG->DATA = d; }
static inline void lcd_wr_cmd_data(uint16_t cmd, uint16_t d) { lcd_wr_cmd(cmd); lcd_wr_data(d); }

#if defined(LCD_PANEL_HX8357DN) || defined(LCD_PANEL_ILI9488)
/* HX8357DN/ILI9488 都是内部按8位/次传输RGB565的两个字节(与官方驱动LCD_WriteData_Color一致) */
static inline void lcd_wr_color(uint16_t color)
{
   LCD_REG->DATA = color >> 8;
   LCD_REG->DATA = color & 0xFF;
}
#else
static inline void lcd_wr_color(uint16_t color) { LCD_REG->DATA = color; }
#endif

/* ---- FSMC总线 GPIO 初始化 ----
 * 数据总线16位: PD14/15=D0/D1 PD0/1=D2/D3 PE7~15=D4~D12 PD8/9/10=D13/D14/D15
 * 控制线: PD4=NOE PD5=NWE PF12=A6(RS) PG12=NE4(CS); 背光PB15普通GPIO输出 */
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

   /* 背光: 普通推挽输出, 不走FSMC复用 */
   g.Pin       = GPIO_PIN_15;
   g.Mode      = GPIO_MODE_OUTPUT_PP;
   g.Pull      = GPIO_NOPULL;
   g.Speed     = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &g);
}

/* ---- FSMC Bank1 NE4 总线时序 ----
 * HCLK=168MHz, 1cycle≈5.95ns。读: ADDSET=15(89ns) DATAST=96(571ns, 留足裕量给
 * 面板寄存器读操作); 写: ADDSET=8(48ns) ADDHLD=8(48ns) DATAST=10(60ns), 三段共
 * ≈156ns/次写, 已大于面板datasheet要求的最短写周期。均在字段位宽内(ADDSET/ADDHLD
 * 4位, 最大0xF=15; DATAST 8位, 最大255), 不像官方驱动那样靠溢出凑数。 */
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

/* ---- 窗口/GRAM 操作 ---- */
static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
#if defined(LCD_PANEL_HX8357DN) || defined(LCD_PANEL_ILI9488)
   /* HX8357DN与ILI9488的列/行地址设置+进GRAM写命令码一致(0x2A/0x2B/0x2C) */
   lcd_wr_cmd(0x2A);
   lcd_wr_data(x0 >> 8); lcd_wr_data(x0 & 0xFF);
   lcd_wr_data(x1 >> 8); lcd_wr_data(x1 & 0xFF);

   lcd_wr_cmd(0x2B);
   lcd_wr_data(y0 >> 8); lcd_wr_data(y0 & 0xFF);
   lcd_wr_data(y1 >> 8); lcd_wr_data(y1 & 0xFF);

   lcd_wr_cmd(0x2C);   /* 进入GRAM写 */
#endif
}

#if defined(LCD_PANEL_HX8357DN)
/* HX8357DN初始化时序: 逐条照抄官方tftlcd.c的TFTLCD_HX8357DN分支(Gamma/电源/接口参数)。
 * 官方驱动init里先写0x36=0x4c(其内部默认方向), 随后又在LCD_Display_Dir(1)里改写
 * 0x36=0x2c+宽高对调成横屏480×320 —— 这里直接一步到位写成横屏最终态, 效果等价。 */
static void lcd_panel_init_hx8357dn(void)
{
   lcd_wr_cmd_data(0xE9, 0x20);

   lcd_wr_cmd(0x11);              /* Exit Sleep */
   HAL_Delay(10);

   lcd_wr_cmd_data(0x3A, 0x55);   /* 16bit colors */

   lcd_wr_cmd(0xD1);
   lcd_wr_data(0x00); lcd_wr_data(0x65); lcd_wr_data(0x1F);   /* VCOM电压/灰阶电压 */

   lcd_wr_cmd(0xD0);
   lcd_wr_data(0x07); lcd_wr_data(0x07); lcd_wr_data(0x80);

   lcd_wr_cmd_data(0x36, 0x2C);   /* Set_address_mode: 横屏(取代官方init里的0x4c,
                                      与官方LCD_Display_Dir(1)最终写入值一致) */
   g_width = 480; g_height = 320;

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
#endif

#if defined(LCD_PANEL_ILI9488)
/* ILI9488初始化时序(型号确认前的试探性实现, 现未启用, 留作参考): 逐条照抄官方
 * tftlcd.c的TFTLCD_ILI9488分支(Gamma/电源/接口参数) */
static void lcd_panel_init_ili9488(void)
{
   lcd_wr_cmd(0xE0);  /* P-Gamma */
   lcd_wr_data(0x00); lcd_wr_data(0x13); lcd_wr_data(0x18); lcd_wr_data(0x04);
   lcd_wr_data(0x0F); lcd_wr_data(0x06); lcd_wr_data(0x3A); lcd_wr_data(0x56);
   lcd_wr_data(0x4D); lcd_wr_data(0x03); lcd_wr_data(0x0A); lcd_wr_data(0x06);
   lcd_wr_data(0x30); lcd_wr_data(0x3E); lcd_wr_data(0x0F);

   lcd_wr_cmd(0xE1);  /* N-Gamma */
   lcd_wr_data(0x00); lcd_wr_data(0x13); lcd_wr_data(0x18); lcd_wr_data(0x01);
   lcd_wr_data(0x11); lcd_wr_data(0x06); lcd_wr_data(0x38); lcd_wr_data(0x34);
   lcd_wr_data(0x4D); lcd_wr_data(0x06); lcd_wr_data(0x0D); lcd_wr_data(0x0B);
   lcd_wr_data(0x31); lcd_wr_data(0x37); lcd_wr_data(0x0F);

   lcd_wr_cmd_data(0xC0, 0x18);           /* Power Control 1: Vreg1out */
   lcd_wr_data(0x17);                      /* Vreg2out */

   lcd_wr_cmd_data(0xC1, 0x41);           /* Power Control 2: VGH,VGL */

   lcd_wr_cmd(0xC5);                       /* Power Control 3 */
   lcd_wr_data(0x00); lcd_wr_data(0x1A); lcd_wr_data(0x80);   /* Vcom */

   lcd_wr_cmd_data(0x36, 0x28);           /* Memory Access: 横屏 */
   lcd_wr_cmd(0XB6);
   lcd_wr_data(0x00); lcd_wr_data(0x02); lcd_wr_data(0x3B);
   g_width = 480; g_height = 320;

   lcd_wr_cmd_data(0x3A, 0x55);           /* Interface Pixel Format: 16bit */
   lcd_wr_cmd_data(0xB0, 0x00);           /* Interface Mode Control */
   lcd_wr_cmd_data(0xB1, 0xA0);           /* Frame rate: 60Hz */
   lcd_wr_cmd_data(0xB4, 0x02);           /* Display Inversion: 2-dot */

   lcd_wr_cmd(0xB6);
   lcd_wr_data(0x00); lcd_wr_data(0x22); lcd_wr_data(0x3B);   /* RGB/MCU Interface */

   lcd_wr_cmd_data(0xE9, 0x00);           /* disable 24bit data input */

   lcd_wr_cmd(0xF7);                       /* Adjust Control */
   lcd_wr_data(0xA9); lcd_wr_data(0x51); lcd_wr_data(0x2C); lcd_wr_data(0x82);

   lcd_wr_cmd(0x11);                       /* Sleep out */
   HAL_Delay(120);
   lcd_wr_cmd(0x29);                       /* Display ON */
   lcd_wr_cmd(0x2C);
}
#endif

void lcd_init(void)
{
   lcd_gpio_init();
   lcd_fsmc_bus_init();
   HAL_Delay(50);

#if defined(LCD_PANEL_HX8357DN)
   lcd_panel_init_hx8357dn();
#elif defined(LCD_PANEL_ILI9488)
   lcd_panel_init_ili9488();
#endif

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

void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
   lcd_set_window(x, y, x, y);
   lcd_wr_color(color);
}

void lcd_backlight(int on)
{
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
