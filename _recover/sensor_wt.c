1	/*
2	 * 维特 WT901 传感器接入实现 —— USART6 (PC7=RX), 115200 8N1
3	 * 见 sensor_wt.h 头注释。寄存器级裸写, 风格对齐 modbus_slave.c / uart_log.c。
4	 *
5	 * 收帧策略(与 Modbus 从站一致): RXNE 逐字节攒进 rx_buf, USART 行空闲(IDLE)
6	 * 判定一帧结束 → 拷进 frame_buf 交给主上下文。传感器 ~20Hz(每帧间隔~50ms),
7	 * 一帧 54B@115200 只需~4.7ms 发完, 帧间大间隙足以让 IDLE 可靠触发。
8	 */
9	#include "stm32f4xx_hal.h"      /* GPIO/RCC/NVIC + USART6 寄存器定义 */
10	#include "sensor_wt.h"
11	
12	volatile uint32_t sensor_stat_frames = 0;
13	volatile uint32_t sensor_stat_valid  = 0;
14	
15	/* ===== 收帧缓冲(ISR 写, 主上下文读) ===== */
16	#define SW_BUFSZ 128
17	static volatile uint8_t  rx_buf[SW_BUFSZ];
18	static volatile uint16_t rx_len = 0;
19	static volatile uint8_t  frame_buf[SW_BUFSZ];
20	static volatile uint16_t frame_len = 0;
21	static volatile uint8_t  frame_ready = 0;   /* 1=frame_buf 有整帧待主上下文解析 */
22	
23	#define SW_FRAME_LEN  54       /* 12(ID) + 40(载荷) + 2(0D 0A) */
24	
25	/* ================= 初始化 ================= */
26	void sensor_init(void)
27	{
28	   GPIO_InitTypeDef g = {0};
29	
30	   __HAL_RCC_GPIOC_CLK_ENABLE();
31	   __HAL_RCC_USART6_CLK_ENABLE();
32	
33	   /* PC7 = USART6_RX(上拉防悬空), PC6 = USART6_TX; 均复用推挽。
34	    * TX 用于每周期回传"本轮收没收到传感器"的 0/1 状态字节给电脑。 */
35	   g.Pin       = GPIO_PIN_7 | GPIO_PIN_6;
36	   g.Mode      = GPIO_MODE_AF_PP;
37	   g.Pull      = GPIO_PULLUP;
38	   g.Speed     = GPIO_SPEED_FREQ_HIGH;
39	   g.Alternate = GPIO_AF8_USART6;
40	   HAL_GPIO_Init(GPIOC, &g);
41	
42	   /* USART6 挂 APB2; BRR = PCLK2 / 波特率 */
43	   USART6->BRR = HAL_RCC_GetPCLK2Freq() / 115200U;
44	   USART6->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE
45	               | USART_CR1_RXNEIE | USART_CR1_IDLEIE;   /* 使能USART + 收 + 发 + 收非空/行空闲中断 */
46	
47	   (void)USART6->SR; (void)USART6->DR;   /* 清可能的初始 IDLE/RXNE 标志 */
48	
49	   HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);   /* 与 Modbus 同级, ISR 极短 */
50	   HAL_NVIC_EnableIRQ(USART6_IRQn);
51	}
52	
53	/* ================= 中断: 收字节 + IDLE 判帧尾 ================= */
54	void sensor_usart6_isr(void)
55	{
56	   uint32_t sr = USART6->SR;
57	
58	   if (sr & USART_SR_RXNE) {
59	      uint8_t b = (uint8_t)(USART6->DR & 0xFF);   /* 读 DR 清 RXNE */
60	      if (rx_len < SW_BUFSZ) rx_buf[rx_len++] = b;
61	      /* 溢出则丢弃, 等下一个 IDLE 重置 */
62	   }
63	   if (sr & USART_SR_IDLE) {
64	      (void)USART6->DR;                 /* 读 SR(上面已读)+读 DR 清 IDLE */
65	      if (rx_len > 0) {
66	         sensor_stat_frames++;          /* 一个帧边界 */
67	         if (!frame_ready) {            /* 上一帧已被主上下文取走才收新帧 */
68	            for (uint16_t i = 0; i < rx_len; i++) frame_buf[i] = rx_buf[i];
69	            frame_len   = rx_len;
70	            frame_ready = 1;
71	         }
72	         rx_len = 0;
73	      }
74	   }
75	}
76	
77	/* ================= 主上下文: 取最新有效帧 ================= */
78	int sensor_get(float *roll_deg, float *pitch_deg)
79	{
80	   if (!frame_ready) return 0;
81	
82	   uint16_t n = frame_len;
83	   /* 拷一份局部, 尽快放行 ISR 收下一帧 */
84	   uint8_t f[SW_FRAME_LEN];
85	   int ok = (n == SW_FRAME_LEN);
86	   if (ok) for (uint16_t i = 0; i < SW_FRAME_LEN; i++) f[i] = frame_buf[i];
87	   frame_ready = 0;
88	   if (!ok) return 0;                            /* 长度不符(粘帧/断帧), 丢弃 */
89	
90	   if (f[0] != 'W' || f[1] != 'T') return 0;     /* 设备ID前缀校验 */
91	   if (f[52] != 0x0D || f[53] != 0x0A) return 0; /* 帧尾校验 */
92	
93	   const uint8_t *pl = &f[12];                   /* 40字节载荷 */
94	   uint16_t ver = (uint16_t)pl[38] | ((uint16_t)pl[39] << 8);
95	   if (ver != SENSOR_FW_VERSION) return 0;       /* 版本号不对 = 字节错位, 丢弃 */
96	
97	   int16_t r = (int16_t)((uint16_t)pl[22] | ((uint16_t)pl[23] << 8));
98	   int16_t p = (int16_t)((uint16_t)pl[24] | ((uint16_t)pl[25] << 8));
99	   *roll_deg  = (float)r * 180.0f / 32768.0f;
100	   *pitch_deg = (float)p * 180.0f / 32768.0f;
101	   sensor_stat_valid++;
102	   return 1;
103	}
104	
105	/* ================= 主上下文: 往 PC6(TX) 发一个状态字节 ================= */
106	/* 阻塞等 TXE。以 ≤1 字节/4ms 的节奏调用时, 上一字节早已发完(单字节@115200≈87us),
107	 * TXE 恒为置位, 实际不阻塞, 不会撑破控制周期。 */
108	void sensor_tx_byte(uint8_t b)
109	{
110	   while (!(USART6->SR & USART_SR_TXE)) { }
111	   USART6->DR = b;
112	}
113	