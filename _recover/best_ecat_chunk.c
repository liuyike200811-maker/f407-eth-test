1	/*
2	 * EtherCAT 三轴康复运动控制 —— STM32 板上 SOEM (CSV 模式, 命令驱动)
3	 * ================================================================
4	 * 架构: 开机→扫从站→配CSV的PDO→进OP→使能→[待机]→听串口命令→跑模式→回待机
5	 *
6	 * 控制: CSV(速度模式, 6060=9)。位置反馈用 6063(CSV下6064恒为0, 见记忆),
7	 *       每轴 前馈速度 + 位置P闭环(带编码器方向自动锁定), 防漂移/防接反跑飞。
8	 *
9	 * 6个模式(全部CSV, 均为 上升→运动→下降 三段, 归零除外):
10	 *   0 综合波浪   1 跖屈/背伸   2 内翻/外翻   3 环形   4 8字      5 归零(堵转检测)
11	 *   1~4 走 3-RPS 正解(FK): 踝角(α内翻外翻, β跖屈背伸)→三电缸长度→速度。
12	 *
13	 * 串口命令 (USART1/CH340 115200 或 USB Slave/Micro-USB虚拟串口, 二选一或都行, 行末\r或\n):
14	 *   ?          查看状态/帮助
15	 *   h          归零(收回到机械限位)
16	 *   g<n>       运行模式 n (0波浪 1跖背 2内外 3环形 4八字)
17	 *   x          急停(停止当前运动, 平滑降速回待机)
18	 *   q          下电退出(脱力→退INIT, 之后可安全断电/拔网线)
19	 *   a<度> b<度>  设 α / β 幅度(度)     f<厘赫> 设频率(25=0.25Hz)
20	 *   w<mm>      设波浪幅度              r<mm> 设上升高度   c<次> 设运动循环数
21	 *
22	 * ⚠ 未做 DC 同步, 信捷 DS5C1 在 CSV 下自由运行(free-run)。实测单轴最大行程
23	 *   约159mm(见探测标定), 上升高度+运动幅度务必留余量; 建议每次运动前先 h 归零。
24	 */
25	#include "soem/soem.h"
26	#include "ecat_motion.h"
27	#include "uart_log.h"
28	#include "usbd_cdc_if.h"
29	#include "modbus_slave.h"
30	#include "sensor_wt.h"
31	#include "osal.h"
32	#include "stm32f4xx_hal.h"   /* 心跳灯(GPIOF/PF9)直接寄存器访问用 */
33	#include <string.h>
34	#include <math.h>
35	
36	volatile int g_ec_slavecount = 0;
37	volatile int g_ec_phase = 0;
38	volatile int g_ec_fault = 0;
39	volatile unsigned int g_reset_cause = 0;
40	
41	static ecx_contextt ctx;
42	static uint8 IOmap[256];
43	
44	/* ---- 机械/编码器常量 ---- */
45	#define LEAD_MM        3.0          /* 丝杠导程 mm/圈 */
46	#define PULSE_PER_REV  131072.0     /* 编码器 脉冲/圈 (17位) */
47	#define MM_PER_PULSE   (LEAD_MM / PULSE_PER_REV)
48	#define CSV_MODE       9
49	#define DT             0.004        /* 通信周期 4ms (与PC端轨迹常量一致) */
50	#define CYC_US         4000
51	
52	/* ---- 闭环控制常量(与PC端 test_xinje_csv_wave_v2 对齐) ---- */
53	#define KP_POS         8.0          /* 位置环增益 1/s */
54	#define VMAX           (600.0/60.0*PULSE_PER_REV)   /* 速度上限 ≈600RPM */
55	#define AMAX           (VMAX/0.15)  /* 加速度上限, ~0.15s达满速 */
56	#define LPF_TAU        0.15         /* 纠正量低通时间常数 s */
57	#define DEADBAND       300.0        /* 位置误差死区 pulse */
58	#define SIGN_LOCK_PULSE 20000.0     /* 上升位移累计超此值即锁定反馈方向 */
59	#define RAMP_FRAMES    150          /* 上升/下降加减速帧数 */
60	
61	/* ---- 归零(堵转检测, 与PC端 retract_all 对齐) ---- */
62	#define RETRACT_VEL    (-163840)    /* 收缩速度 pulse/s (≈-75RPM) */
63	/* 堵转判断(位置法, 比读瞬时速度稳, 与 Linux retract_all 对齐):
64	 * 维护去抖窗口, 窗口内累计位移 > STALL_WIN_EPS 说明还在往里走→重置; 连续
65	 * STALL_FRAMES 帧(75×4ms=300ms)位置几乎不动才判到位。避免中途机械紧点让速度
66	 * 瞬间掉到阈值下被误判成"到位"而提前停(杆没收到底); 且"本就在限位"也能正确判到。*/
67	#define STALL_WIN_EPS  1500         /* 一个窗口内累计位移小于此值(pulse)=没在动 */
68	#define STALL_FRAMES   75           /* 连续多少帧几乎不动才判到位(≈300ms) */
69	#define HOME_MAX_FRAMES 7500        /* 归零超时(≈30s) */
70	
71	/* ---- 可由串口命令修改的运动参数(带安全默认值) ---- */
72	static int    g_aa_deg   = 15;      /* α 幅度(度) */
73	static int    g_ba_deg   = 15;      /* β 幅度(度) */
74	static int    g_freq_cHz = 25;      /* 频率(厘赫兹) 25=0.25Hz */
75	static int    g_wave_mm  = 30;      /* 波浪幅度 mm (mode0) */
76	static int    g_rise_mm  = 60;      /* 上升高度 mm */
77	static int    g_rise_rpm = 150;     /* 上升速度 RPM */
78	static int    g_cycles   = 9;       /* 运动循环数 */
79	
80	#define WAVE_PEAK_RPM  471.0        /* 波浪峰值速度(mode0) */
81	#define FK_Z0          380.0        /* FK 归位参考工作高度 mm */
82	
83	/* ---- 传感器实时跟随模式(WT901 姿态→踝角→电缸) ---- */
84	/* 平滑在【控制周期250Hz】上跑, 不再只在收到帧(~20Hz)时跳一步 —— 否则前馈速度
85	 * =Δ杆长/4ms 只在收帧那一帧非零, 变成20Hz尖峰串, 被限加速削平后平台几乎不动。
86	 * SENSOR_FOLLOW_TAU: 跟随平滑时间常数(s)。越小越跟手/灵敏(幅度足), 越大越稳/滞后。
87	 *   经验: 0.05~0.12。要通过手部跟随带宽又能磨平50ms帧台阶, 取略大于帧间隔(0.05s)。*/
88	#define SENSOR_FOLLOW_TAU    0.08   /* 跟随平滑时间常数(s), 台架微调此值定手感 */
89	#define SENSOR_DEADBAND_DEG  0.3    /* 角度死区(度): 变化小于此值不更新目标, 抑制静止抖动 */
90	#define SENSOR_GAIN          3.0    /* 角度增益: 手倾角 × 此倍数 = 平台目标角(再经 MAX 限幅封顶) */
91	/* +X(+β, 朝1轴)侧专用增益: 只放大"往+X推"这一侧, 中位(水平)仍=0, 不产生静态偏置。
92	 * 根因: 上升阶段三缸都伸→fb_sign锁在"伸"方向; 跟随时 +β 要求 1轴【反向缩回】,
93	 * 需跨过换向死区/回程间隙, 故 +X 侧手感明显偏弱。-X(-β)是"继续伸"故正常。
94	 * 对策: 给 +β 侧更大目标角 → 更大前馈速度, 一把冲过换向死区。
95	 * ★台架试调此值: 太小仍不动, 太大则+X过冲。base=3.0, 起步给 5.0(≈1.7×)往上加。*/
96	#define SENSOR_GAIN_BETA_XP  5.0    /* +X(+β)侧增益, >= SENSOR_GAIN */
97	#define SENSOR_MAX_DEG       30.0   /* 跟随角度包络(度): 限幅, 防逆解出超程杆长 */
98	#define SENSOR_CAL_FRAMES    250    /* 零偏标定采样帧数(≈1s) */
99	#define SENSOR_WATCHDOG_FRAMES 75   /* 信号丢失看门狗(≈300ms 无新帧→冻结保持) */
100	
101	/* ---- 闭环状态(每轴) ---- */
102	static int32_t start_pos[EC_MAXSLAVE];   /* 运动起点编码器零点(6063) */
103	static double  cmd_pos[EC_MAXSLAVE];     /* 期望位置=前馈速度积分 */
104	static double  v_last[EC_MAXSLAVE];      /* 上帧命令速度, 限加速度用 */
105	static double  corr_filt[EC_MAXSLAVE];   /* 低通后的纠正量 */
106	static int     fb_sign[EC_MAXSLAVE];     /* 反馈方向: 0待锁 ±1已锁 */
107	static const double PHASE[4] = {0.0, 0.0, 2.0*M_PI/3.0, 4.0*M_PI/3.0};  /* 三轴波浪相位(索引1~3) */
108	
109	/* ---- 命令状态 ---- */
110	static volatile int g_run_request = -1;  /* -1无; 0~4运行对应模式 */
111	static volatile int g_do_home     = 0;
112	static volatile int g_do_sensor   = 0;   /* 进入传感器实时跟随模式 */
113	static volatile int g_quit        = 0;
114	static volatile int g_abort       = 0;   /* 运动中急停标志 */
115	
116	/* HMI(Modbus 4x0010/4x0018)反馈用: 运行状态字 与 当前运行模式 */
117	static volatile int g_status   = 0;      /* 0启动中 1待机 2运行 3归零 4故障 5已下电 */
118	static volatile int g_cur_mode = 99;     /* 0~4=正在跑的模式; 99=待机/无模式 */
119	
120	static volatile int g_wkc = 0;
121	
122	/* ---- PDO 字节访问 (RxPDO: CW+mode+60FF; TxPDO: SW+mode+6063+606C) ---- */
123	static inline void write_pdo(int sl, uint16_t cw, int32_t vel)
124	{
125	   uint8 *o = ctx.slavelist[sl].outputs;
126	   if (!o) return;
127	   *(uint16_t *)(o + 0) = cw;
128	   *(int8_t  *)(o + 2) = CSV_MODE;
129	   *(int32_t *)(o + 3) = vel;
130	}
131	static inline uint16_t read_sw(int sl)
132	{ uint8 *in = ctx.slavelist[sl].inputs; return in ? *(uint16_t *)(in + 0) : 0; }
133	static inline int32_t read_pos63(int sl)   /* 6063 实际位置(CSV下唯一有效) @offset3 */
134	{ uint8 *in = ctx.slavelist[sl].inputs; return in ? *(int32_t *)(in + 3) : 0; }
135	static inline int32_t read_vel(int sl)     /* 606C 实际速度 @offset7 */
136	{ uint8 *in = ctx.slavelist[sl].inputs; return in ? *(int32_t *)(in + 7) : 0; }
137	
138	static int any_fault(void)
139	{
140	   for (int sl = 1; sl <= ctx.slavecount; sl++)
141	      if (read_sw(sl) & 0x0008) return sl;
142	   return 0;
143	}
144	
145	/* 心跳灯: LED0=PF9 低电平点亮. 每约500ms翻转一次(4ms节拍 * 125次).
146	   只要这个在闪, 就说明 cycle() 在正常跑, 没卡死/没硬件异常复位. */
147	static inline void heartbeat(void)
148	{
149	   static int n = 0;
150	   if (++n >= 125) { n = 0; GPIOF->ODR ^= GPIO_PIN_9; }
151	}
152	
153	/* 一个通信周期: 发+收过程数据, 固定4ms节拍。
154	 * 节拍对齐用绝对时刻(next_deadline每次只加4ms, 不受本轮实际耗时影响),
155	 * 不能用osal_usleep(相对延时) —— 那样真实周期会变成"4ms+本轮EtherCAT收发耗时",
156	 * 且逐帧抖动, 而vel_closed()里的位置积分(cmd_pos += v_ff*DT)是按严格4ms算的,
157	 * 周期一旦漂移/抖动, 闭环用来对比的"期望位置"就会跟伺服真实走的对不上,
158	 * P环持续误纠正, 表现为运动发飘/发抖(对齐PC端clock_nanoseep(TIMER_ABSTIME)的做法)。 */
159	static void cycle(void)
160	{
161	   static ec_timet next_deadline;
162	   static int inited = 0;
163	
164	   ecx_send_processdata(&ctx);
165	   g_wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
166	   heartbeat();
167	
168	   if (!inited) { osal_get_monotonic_time(&next_deadline); inited = 1; }
169	
170	   next_deadline.tv_nsec += (long)CYC_US * 1000L;
171	   while (next_deadline.tv_nsec >= 1000000000L) {
172	      next_deadline.tv_sec++;
173	      next_deadline.tv_nsec -= 1000000000L;
174	   }
175	   osal_monotonic_sleep(&next_deadline);
176	}
177	
178	/* ================= 串口命令解析 ================= */
179	/* 累积一行(\r或\n结束)后分派。行首字母=命令, 其余为可选整数参数。 */
180	static void handle_cmd(char *line)
181	{
182	   char c = line[0];
183	   int  arg = 0, has = 0;
184	   for (char *p = line + 1; *p; p++) {
185	      if (*p >= '0' && *p <= '9') { arg = arg * 10 + (*p - '0'); has = 1; }
186	   }
187	   switch (c) {
188	      case '?':
189	         uart_log("命令: ? 帮助 | h归零 | g<0-4>运行模式 | s传感器跟随 | x急停 | q下电退出\r\n"
190	                  "      a<度>α b<度>β f<厘赫>频率 w<mm>波幅 r<mm>升高 c<次>循环\r\n"
191	                  "参数: mode默认由g指定  α=%d° β=%d° 频率=%.2fHz 波幅=%dmm 升高=%dmm 循环=%d\r\n"
192	                  "模式: 0波浪 1跖屈背伸 2内翻外翻 3环形 4八字 (5=归零用h) s=传感器实时跟随\r\n",
193	                  g_aa_deg, g_ba_deg, g_freq_cHz / 100.0, g_wave_mm, g_rise_mm, g_cycles);
194	         break;
195	      case 'h': g_do_home = 1; break;
196	      case 's': g_do_sensor = 1; break;
197	      case 'x': g_abort = 1; break;
198	      case 'q': g_quit = 1; break;
199	      case 'g':
200	         if (has && arg >= 0 && arg <= 4) g_run_request = arg;
201	         else uart_log("g 需要 0~4 的模式号\r\n");
202	         break;
203	      case 'a': if (has) { g_aa_deg = arg; uart_log("α幅度=%d°\r\n", arg); } break;
204	      case 'b': if (has) { g_ba_deg = arg; uart_log("β幅度=%d°\r\n", arg); } break;
205	      case 'f': if (has) { g_freq_cHz = arg; uart_log("频率=%.2fHz\r\n", arg / 100.0); } break;
206	      case 'w': if (has) { g_wave_mm = arg; uart_log("波幅=%dmm\r\n", arg); } break;
207	      case 'r': if (has) { g_rise_mm = arg; uart_log("升高=%dmm\r\n", arg); } break;
208	      case 'c': if (has) { g_cycles = arg; uart_log("循环=%d\r\n", arg); } break;
209	      default: break;   /* 空行/未知命令忽略 */
210	   }
211	}
212	
213	/* 把一个字节喂进行缓冲区, 攒够一行(\r或\n)就解析。USART1/USB两路共用同一份逻辑。 */
214	static void feed_cmd_byte(char ch)
215	{
216	   static char buf[32];
217	   static int  len = 0;
218	   if (ch == '\r' || ch == '\n') {
219	      if (len > 0) { buf[len] = '\0'; handle_cmd(buf); len = 0; }
220	   } else if (len < (int)sizeof(buf) - 1) {
221	      buf[len++] = ch;
222	   } else {
223	      len = 0;   /* 溢出丢弃 */
224	   }
225	}
226	
227	/* ================= Modbus (HMI) 对接 ================= */
228	static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
229	
230	/* 把 HMI 经 Modbus 写来的设定/命令落到本地全局。
231	 * 设定区: 只在 HMI 改动过(值变化)时才落地, 越界截断(兑现契约"越界自动截断"),
232	 *         这样串口/USB 命令改的参数不会被每周期原样覆盖。
233	 * 控制区: 一次性触发线圈, 受理后清0; 模式/归零仅待机受理, 急停/下电/复位随时受理。*/
234	static void modbus_sync(void)
235	{
236	   static uint16_t shadow[7];
237	   static int seeded = 0;
238	   if (!seeded) {   /* 用固件默认值播种设定区, 让 HMI 首次读到的是真实默认, 而非0 */
239	      modbus_hreg[0] = g_aa_deg;   modbus_hreg[1] = g_ba_deg;   modbus_hreg[2] = g_freq_cHz;
240	      modbus_hreg[3] = g_wave_mm;  modbus_hreg[4] = g_rise_mm;  modbus_hreg[5] = g_cycles;
241	      modbus_hreg[6] = g_rise_rpm;
242	      for (int i = 0; i < 7; i++) shadow[i] = modbus_hreg[i];
243	      seeded = 1;
244	   }
245	   if (modbus_hreg[0] != shadow[0]) { shadow[0] = modbus_hreg[0]; g_aa_deg   = clampi(modbus_hreg[0], 0, 30); }
246	   if (modbus_hreg[1] != shadow[1]) { shadow[1] = modbus_hreg[1]; g_ba_deg   = clampi(modbus_hreg[1], 0, 30); }
247	   if (modbus_hreg[2] != shadow[2]) { shadow[2] = modbus_hreg[2]; g_freq_cHz = clampi(modbus_hreg[2], 5, 80); }
248	   if (modbus_hreg[3] != shadow[3]) { shadow[3] = modbus_hreg[3]; g_wave_mm  = clampi(modbus_hreg[3], 0, 40); }
249	   if (modbus_hreg[4] != shadow[4]) { shadow[4] = modbus_hreg[4]; g_rise_mm  = clampi(modbus_hreg[4], 0, 80); }
250	   if (modbus_hreg[5] != shadow[5]) { shadow[5] = modbus_hreg[5]; g_cycles   = clampi(modbus_hreg[5], 1, 99); }
251	   if (modbus_hreg[6] != shadow[6]) { shadow[6] = modbus_hreg[6]; g_rise_rpm = clampi(modbus_hreg[6], 50, 200); }
252	
253	   /* 急停/下电/故障复位: 任何时候都响应 */
254	   if (modbus_coils[6]) { modbus_coils[6] = 0; g_abort = 1; }
255	   if (modbus_coils[7]) { modbus_coils[7] = 0; g_quit  = 1; }
256	   if (modbus_coils[8]) { modbus_coils[8] = 0; g_ec_fault = 0; }  /* 清故障显示 */
257	
258	   /* 模式0~4 / 归零 / 传感器跟随: 仅待机受理; 运行中收到则清掉不执行(防中途乱切, 见契约) */
259	   if (g_status == 1) {
260	      int req = -1;
261	      for (int m = 0; m < 5; m++)
262	         if (modbus_coils[m]) { if (req < 0) req = m; modbus_coils[m] = 0; }  /* 多个同ON以先到为准 */
263	      int sensor_req = modbus_coils[9]; modbus_coils[9] = 0;                  /* 0x0009 传感器跟随 */
264	      if (modbus_coils[5])   { modbus_coils[5] = 0; g_do_home = 1; }          /* 归零优先 */
265	      else if (sensor_req)   { g_do_sensor = 1; }
266	      else if (req >= 0)     { g_run_request = req; }
267	   } else {
268	      for (int m = 0; m <= 5; m++) modbus_coils[m] = 0;
269	      modbus_coils[9] = 0;
270	   }
271	}
272	
273	/* 把本地状态回写 Modbus 反馈寄存器(4x0010~4x0018 = 下标16~24)。每周期一次。 */
274	static void modbus_write_feedback(void)
275	{
276	   static uint16_t hb = 0;
277	   int32_t p1 = (ctx.slavecount >= 1) ? (read_pos63(1) - start_pos[1]) : 0;
278	   int32_t p2 = (ctx.slavecount >= 2) ? (read_pos63(2) - start_pos[2]) : 0;
279	   int32_t p3 = (ctx.slavecount >= 3) ? (read_pos63(3) - start_pos[3]) : 0;
280	
281	   modbus_hreg[16] = (uint16_t)g_status;                    /* 运行状态字 */
282	   modbus_hreg[17] = (uint16_t)g_ec_fault;                  /* 故障从站号 */
283	   modbus_hreg[18] = (uint16_t)g_ec_slavecount;             /* 在线从站数 */
284	   modbus_hreg[19] = (uint16_t)g_wkc;                       /* WKC */
285	   modbus_hreg[20] = (uint16_t)(int16_t)(p1 * MM_PER_PULSE * 10.0);  /* 轴1位置 mm×10 有符号 */
286	   modbus_hreg[21] = (uint16_t)(int16_t)(p2 * MM_PER_PULSE * 10.0);  /* 轴2 */
287	   modbus_hreg[22] = (uint16_t)(int16_t)(p3 * MM_PER_PULSE * 10.0);  /* 轴3 */
288	   modbus_hreg[23] = ++hb;                                  /* 心跳(溢出自然归零) */
289	   modbus_hreg[24] = (uint16_t)g_cur_mode;                  /* 当前运行模式 */
290	}
291	
292	/* ================= 数字孪生遥测帧 (micro-USB / USB CDC) ================= */
293	/* 每控制周期往 USB CDC(micro-USB) 发一帧二进制遥测, 供电脑转发给局域网数字孪生。
294	 * 格式沿用旧 Linux 版 UDP 帧, 仅在最前面加 2 字节帧头 0xAA 0x55 便于电脑在
295	 * 字节流里定位(同一路 CDC 还混着 uart_log 的可读文本, 电脑遇 0xAA55 按固定长读一帧):
296	 *
297	 *   偏移  长度  字段
298	 *   0     2    帧头 0xAA 0x55
299	 *   2     4    seq     (uint32 小端, 每帧+1, 电脑据此判丢帧)
300	 *   6     1    phase   (uint8, = 当前模式 g_cur_mode: 0~4 康复模式 / 6 传感器跟随 / 99 待机)
301	 *   7     12   pos[3]  (float 小端 ×3, 各电缸相对 home 的伸出量, mm, 真实编码器 6063)
302	 *   19    12   vel[3]  (float 小端 ×3, 各电缸实际速度, RPM, 606C)
303	 *   31    1    xor     (前面 29 个载荷字节[偏移2~30]的逐字节异或, 供电脑校验/排除误同步)
304	 *   共 32 字节。载荷 29 字节(偏移2~30)的内部布局与旧 webui server.py 完全一致(不改内部结构),
305	 *   帧头 0xAA55 与尾部 xor 是外包装: 电脑扫到 0xAA55 → 读定长 32B → 核对 xor, 过了才是真帧。
306	 *
307	 * CDC_Transmit_FS 非阻塞: USB 忙/没插线直接丢弃本帧(尽力而为, seq 让电脑能发现空洞),
308	 * 不会阻塞 4ms 控制节拍。 */
309	static void twin_emit(void)
310	{
311	   static uint32_t seq = 0;
312	   uint8_t buf[32];
313	   float pos[3], vel[3];
314	
315	   for (int sl = 1; sl <= 3; sl++) {
316	      int32_t dp = (ctx.slavecount >= sl) ? (read_pos63(sl) - start_pos[sl]) : 0;
317	      int32_t rv = (ctx.slavecount >= sl) ? read_vel(sl) : 0;
318	      pos[sl - 1] = (float)((double)dp * MM_PER_PULSE);         /* mm, 从 home 起 */
319	      vel[sl - 1] = (float)((double)rv / PULSE_PER_REV * 60.0); /* pulse/s → RPM */
320	   }
321	
322	   seq++;
323	   buf[0] = 0xAA; buf[1] = 0x55;
324	   memcpy(buf + 2, &seq, 4);
325	   buf[6] = (uint8_t)g_cur_mode;
326	   memcpy(buf + 7,  pos, 12);
327	   memcpy(buf + 19, vel, 12);
328	   uint8_t xr = 0;
329	   for (int i = 2; i < 31; i++) xr ^= buf[i];   /* 异或校验: 载荷 29 字节 */
330	   buf[31] = xr;
331	   CDC_Transmit_FS(buf, sizeof buf);
332	}
333	
334	/* 每控制周期往 USART6 TX(PC6) 回一个状态字节:
335	 * '1'=有效帧, 否则发 sensor_fail_reason ('L'长度/'H'头/'T'尾/'V'版本/'0'无帧)。 */
336	static void sensor_status_tx(void)
337	{
338	   static uint32_t last_valid = 0;
339	   uint32_t v = sensor_stat_valid;
340	   sensor_tx_byte((v != last_valid) ? '1' : (uint8_t)sensor_fail_reason);
341	   last_valid = v;
342	}
343	
344	/* 非阻塞轮询命令通道: USART1(CH340)、USB Slave(虚拟串口)、Modbus/HMI(USART3) 三路并收。
345	   每个通信周期调一次 —— 运动中也在调, 故急停/HMI命令运动中同样即时响应。 */
346	static void poll_cmd(void)
347	{
348	   int ch;
349	   while ((ch = uart_rx_getc()) >= 0)  feed_cmd_byte((char)ch);
350	   while ((ch = usb_cdc_getc()) >= 0)  feed_cmd_byte((char)ch);
351	   modbus_poll();            /* 收/解析/应答 Modbus 帧 */
352	   modbus_sync();            /* HMI 命令/参数 → 本地全局 */
353	   modbus_write_feedback();  /* 本地状态 → 反馈寄存器 */
354	   sensor_status_tx();       /* → PC6: 传感器接收状态 0/1 */
355	   twin_emit();              /* → micro-USB: 数字孪生遥测帧 */
356	}
357	
358	/* ================= 前馈 + 位置P闭环 ================= */
359	static int32_t limit_out(int sl, double v)
360	{
361	   if (v >  VMAX) v =  VMAX;
362	   if (v < -VMAX) v = -VMAX;
363	   double dvmax = AMAX * DT;
364	   double dv = v - v_last[sl];
365	   if (dv >  dvmax) v = v_last[sl] + dvmax;
366	   if (dv < -dvmax) v = v_last[sl] - dvmax;
367	   v_last[sl] = v;
368	   return (int32_t)v;
369	}
370	
371	/* v_ff: 前馈速度(pulse/s)。返回本帧应下发的速度指令。
372	 * 反馈方向未锁前走纯前馈(=开环, 安全), 用上升自然运动锁定极性保证负反馈。 */
373	static int32_t vel_closed(int sl, double v_ff)
374	{
375	   cmd_pos[sl] += v_ff * DT;
376	   double raw = (double)(read_pos63(sl) - start_pos[sl]);
377	
378	   if (fb_sign[sl] == 0) {
379	      if (fabs(raw) > SIGN_LOCK_PULSE && fabs(cmd_pos[sl]) > SIGN_LOCK_PULSE) {
380	         fb_sign[sl] = ((cmd_pos[sl] > 0) == (raw > 0)) ? 1 : -1;
381	         cmd_pos[sl] = fb_sign[sl] * raw;   /* 对齐, 消除接入瞬间突跳 */
382	      }
383	      return limit_out(sl, v_ff);
384	   }
385	
386	   double actual = fb_sign[sl] * raw;
387	   double err = cmd_pos[sl] - actual;
388	   double e = (err < DEADBAND && err > -DEADBAND) ? 0.0 : err;
389	   double corr_raw = KP_POS * e;
390	   double alpha = (LPF_TAU > 0.0) ? DT / (LPF_TAU + DT) : 1.0;
391	   corr_filt[sl] += alpha * (corr_raw - corr_filt[sl]);
392	   return limit_out(sl, v_ff + corr_filt[sl]);
393	}
394	
395	/* ================= 3-RPS 运动学: 位姿(α,β,z) → 三杆长 ================= */
396	/* 命名提醒: 并联机构里"位姿→杆长"严格是【逆解 IK】(简单/闭式方向);
397	 *   "杆长→位姿"才是正解 FK(难, 多解, 本工程未做也用不到)。
398	 *   函数名 fk_ 系历史沿用, 勿被"fk"字样误导成正解。
399	 * R_base=200 r_mov=140 120°均布; alpha内翻外翻(绕X), beta跖屈背伸(绕Y) */
400	static void fk_3rps(double alpha, double beta, double z_eff, double l_out[3])
401	{
402	   static const double A[3][2] = {
403	      { 200.0,   0.0        },
404	      {-100.0,  173.2050808 },
405	      {-100.0, -173.2050808 }
406	   };
407	   static const double B[3][2] = {
408	      { 140.0,   0.0        },
409	      { -70.0,  121.2435565 },
410	      { -70.0, -121.2435565 }
411	   };
412	   double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
413	   double R[3][3] = {
414	      { cb,   sa * sb,  ca * sb },
415	      { 0.0,  ca,      -sa      },
416	      {-sb,   sa * cb,  ca * cb }
417	   };
418	   for (int i = 0; i < 3; i++) {
419	      double bx = R[0][0] * B[i][0] + R[0][1] * B[i][1];
420	      double by = R[1][0] * B[i][0] + R[1][1] * B[i][1];
421	      double bz = z_eff + R[2][0] * B[i][0] + R[2][1] * B[i][1];
422	      double dx = bx - A[i][0];
423	      double dy = by - A[i][1];
424	      l_out[i] = sqrt(dx * dx + dy * dy + bz * bz);
425	   }
426	}
427	
428	/* 角度轨迹: 按模式返回当前相位 t 的 alpha,beta(弧度) */
429	static void get_angles(int mode, double t, double aa_r, double ba_r,
430	                       double *alpha, double *beta)
431	{
432	   switch (mode) {
433	      case 1: *alpha = 0;             *beta = ba_r * sin(t);         break;  /* 跖屈/背伸 */
434	      case 2: *alpha = aa_r * sin(t); *beta = 0;                     break;  /* 内翻/外翻 */
435	      case 3: *alpha = aa_r * sin(t); *beta = ba_r * cos(t);         break;  /* 环形 */
436	      case 4: *alpha = aa_r * sin(t); *beta = ba_r * sin(2.0 * t);   break;  /* 8字 */
437	      default:*alpha = 0;             *beta = 0;                     break;
438	   }
439	}
440	
441	/* ================= 运动阶段公共件 ================= */
442	/* 运动前初始化闭环零点(以当前位置为起点) */
443	static void motion_reset(void)
444	{
445	   for (int sl = 1; sl <= ctx.slavecount; sl++) {
446	      start_pos[sl] = read_pos63(sl);
447	      cmd_pos[sl]   = 0.0;
448	      v_last[sl]    = 0.0;
449	      corr_filt[sl] = 0.0;
450	      fb_sign[sl]   = 0;
451	   }
452	}
453	
454	/* 急停/报警时平滑降速回0(闭环, 保持使能)。返回后回待机。 */
455	static void ramp_to_zero(void)
456	{
457	   for (int i = 0; i < RAMP_FRAMES; i++) {
458	      for (int sl = 1; sl <= ctx.slavecount; sl++)
459	         write_pdo(sl, 0x000F, vel_closed(sl, 0.0));
460	      cycle();
461	   }
462	   for (int i = 0; i < 50; i++) {
463	      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0);
464	      cycle();
465	   }
466	}
467	
468	/* 三轴同步做一段"前馈速度=vff(每轴相同)"的运动. 返回: 0正常 -1报警 -2急停 */
469	/* raise_vel: 匀速目标(pulse/s, 正=上升 负=下降); 内部自动加/匀/减速三段 */
470	static int move_ramp(double raise_vel, int cruise_frames)
471	{
472	   int i;
473	   /* 加速 */
474	   for (i = 0; i < RAMP_FRAMES; i++) {
475	      double vff = raise_vel * i / RAMP_FRAMES;
476	      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, vff));
477	      cycle(); poll_cmd();
478	      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
479	      if (g_abort) return -2;
480	   }
481	   /* 匀速 */
482	   for (i = 0; i < cruise_frames; i++) {
483	      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, raise_vel));
484	      cycle(); poll_cmd();
485	      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
486	      if (g_abort) return -2;
487	   }
488	   /* 减速 */
489	   for (i = 0; i < RAMP_FRAMES; i++) {
490	      double vff = raise_vel * (RAMP_FRAMES - i) / RAMP_FRAMES;
491	      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, vff));
492	      cycle(); poll_cmd();
493	      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
494	      if (g_abort) return -2;
495	   }
496	   return 0;
497	}
498	
499	/* ================= 康复模式 (上升→运动→下降) ================= */
500	static int run_rehab_mode(int mode)
501	{
502	   int i, sl, rc;
503	   g_abort = 0;
504	   g_ec_fault = 0;
505	   g_status = 2; g_cur_mode = mode;   /* HMI 反馈: 运行中 + 当前模式 */
506	   motion_reset();
507	
508	   int32_t raise_vel  = (int32_t)((double)g_rise_rpm / 60.0 * PULSE_PER_REV);
509	   int32_t rise_pulse = (int32_t)((double)g_rise_mm * PULSE_PER_REV / LEAD_MM);
510	   int cruise_frames  = (int)((double)rise_pulse / ((double)raise_vel * DT));
511	   if (cruise_frames < 1) cruise_frames = 1;
512	
513	   uart_log(">>> 模式%d 开始: 上升%dmm@%dRPM → 运动 → 下降 <<<\r\n", mode, g_rise_mm, g_rise_rpm);
514	
515	   /* 阶段1: 上升 */
516	   rc = move_ramp((double)raise_vel, cruise_frames);
517	   if (rc < 0) goto stopmsg;
518	   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); }
519	   uart_log("  上升完成, 方向锁定: 轴1=%+d 轴2=%+d 轴3=%+d\r\n", fb_sign[1], fb_sign[2], fb_sign[3]);
520	
521	   /* 阶段2: 运动 */
522	   if (mode == 0) {
523	      /* 综合波浪(电缸空间三轴相位差) */
524	      int32_t wave_amp = (int32_t)((double)g_wave_mm * PULSE_PER_REV / LEAD_MM);
525	      int wave_period = (int)((double)wave_amp * 2.0 * M_PI / (WAVE_PEAK_RPM / 60.0 * PULSE_PER_REV) / DT);
526	      if (wave_period < 100) wave_period = 100;
527	      double wave_v_amp = (double)wave_amp * 2.0 * M_PI / wave_period / DT;
528	      int total = g_cycles * wave_period;
529	      uart_log("  阶段2 综合波浪: 幅%dmm 周期%d帧 × %d\r\n", g_wave_mm, wave_period, g_cycles);
530	      for (i = 0; i < total; i++) {
531	         double t = 2.0 * M_PI * i / wave_period;
532	         for (sl = 1; sl <= ctx.slavecount; sl++) {
533	            double vff = wave_v_amp * cos(t + PHASE[sl]);
534	            write_pdo(sl, 0x000F, vel_closed(sl, vff));
535	         }
536	         cycle(); poll_cmd();
537	         if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
538	         if (g_abort) { rc = -2; goto stopmsg; }
539	         /* 运动中绝不打印: uart_log 是阻塞式串口发送(每字符忙等~87us, 一行~4ms),
540	            会把 4ms 控制周期撑到十几ms → free-run 伺服顿挫。统计留到运动结束汇报。 */
541	      }
542	   } else {
543	      /* FK 角度空间(1跖背 2内外 3环形 4八字) */
544	      double aa_r = g_aa_deg * M_PI / 180.0;
545	      double ba_r = g_ba_deg * M_PI / 180.0;
546	      if (aa_r > M_PI / 6) aa_r = M_PI / 6;    /* 限幅±30° */
547	      if (ba_r > M_PI / 6) ba_r = M_PI / 6;
548	      double freq = g_freq_cHz / 100.0;
549	      if (freq < 0.05) freq = 0.05;
550	      if (freq > 0.8)  freq = 0.8;
551	      double z_eff = FK_Z0 + (double)g_rise_mm;
552	      int period_frames = (int)(1.0 / (freq * DT));
553	      if (period_frames < 50) period_frames = 50;
554	      int total = g_cycles * period_frames;
555	      double omega = 2.0 * M_PI * freq;
556	      uart_log("  阶段2 FK模式%d: α%d° β%d° 频%d厘赫 周期%d帧 × %d\r\n",
557	               mode, g_aa_deg, g_ba_deg, g_freq_cHz, period_frames, g_cycles);
558	      for (i = 0; i < total; i++) {
559	         double t0 = omega * i * DT, t1 = omega * (i + 1) * DT;
560	         double a0, b0, a1, b1, lc[3], ln[3];
561	         get_angles(mode, t0, aa_r, ba_r, &a0, &b0);
562	         get_angles(mode, t1, aa_r, ba_r, &a1, &b1);
563	         fk_3rps(a0, b0, z_eff, lc);
564	         fk_3rps(a1, b1, z_eff, ln);
565	         for (sl = 1; sl <= ctx.slavecount; sl++) {
566	            double v_mms = (ln[sl - 1] - lc[sl - 1]) / DT;   /* mm/s */
567	            double vff = v_mms / MM_PER_PULSE;               /* pulse/s */
568	            write_pdo(sl, 0x000F, vel_closed(sl, vff));
569	         }
570	         cycle(); poll_cmd();
571	         if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
572	         if (g_abort) { rc = -2; goto stopmsg; }
573	         /* 运动中绝不打印(同上, 避免 uart_log 阻塞撑破 4ms 节拍) */
574	      }
575	   }
576	
577	   /* 阶段3: 下降回原点 */
578	   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); }
579	   rc = move_ramp(-(double)raise_vel, cruise_frames);
580	   if (rc < 0) goto stopmsg;
581	   /* 末端主动闭环回精确原点 */
582	   for (i = 0; i < 150; i++) {
583	      for (sl = 1; sl <= ctx.slavecount; sl++) { cmd_pos[sl] = 0.0; write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); }
584	      cycle(); poll_cmd();
585	      if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
586	      if (g_abort) { rc = -2; goto stopmsg; }
587	   }
588	
589	   uart_log(">>> 模式%d 完成, 残余误差: 轴1=%.2fmm 轴2=%.2fmm 轴3=%.2fmm <<<\r\n", mode,
590	            (read_pos63(1) - start_pos[1]) * MM_PER_PULSE,
591	            (read_pos63(2) - start_pos[2]) * MM_PER_PULSE,
592	            (read_pos63(3) - start_pos[3]) * MM_PER_PULSE);
593	   return 0;
594	
595	stopmsg:
596	   if (rc == -2) uart_log("!! 急停, 平滑降速回待机\r\n");
597	   else          uart_log("!! 从站%d 报警, 平滑降速回待机\r\n", g_ec_fault);
598	   ramp_to_zero();
599	   return rc;
600	}
601	
602	/* ================= 传感器实时跟随 (上升→标定→跟随→归平下降) =================
603	 * WT901 姿态 → 减零偏 → EMA → 死区 → 限幅 → fk_3rps 逆算三电缸目标长 →
604	 * 相邻帧长度差分得前馈速度 → vel_closed 闭环输出。x 急停退出, 信号丢失自动冻结。 */
605	static int run_sensor_mode(void)
606	{
607	   int i, sl, rc = 0;
608	   double ta = 0.0, tb = 0.0;         /* 控制周期平滑后目标角(rad, 相对零偏); 退出归平复用 */
609	   double ar_raw = 0.0, br_raw = 0.0; /* 最新传感器目标角(rad), 仅收到帧时刷新, 帧间保持 */
610	   double l_prev[3];                  /* 上一帧 FK 目标杆长(mm), 用于差分求速度 */
611	   float  r, p;
612	
613	   g_abort = 0; g_ec_fault = 0;
614	   g_status = 2; g_cur_mode = 6;   /* HMI 反馈: 运行中 + 模式6(传感器跟随) */
615	   motion_reset();
616	
617	   int32_t raise_vel  = (int32_t)((double)g_rise_rpm / 60.0 * PULSE_PER_REV);
618	   int32_t rise_pulse = (int32_t)((double)g_rise_mm * PULSE_PER_REV / LEAD_MM);
619	   int cruise_frames  = (int)((double)rise_pulse / ((double)raise_vel * DT));
620	   if (cruise_frames < 1) cruise_frames = 1;
621	   double z_eff  = FK_Z0 + (double)g_rise_mm;
622	   double ang_lim = SENSOR_MAX_DEG * M_PI / 180.0;
623	   fk_3rps(0.0, 0.0, z_eff, l_prev);   /* 基线=标定姿态(零倾斜), 提前初始化防 goto 跳过 */
624	
625	   uart_log(">>> 传感器跟随: 上升%dmm → 零偏标定 → 实时跟随(发 x 退出) <<<\r\n", g_rise_mm);
626	
627	   /* 阶段1: 上升到工作高度(与其它模式一致) */
628	   rc = move_ramp((double)raise_vel, cruise_frames);
629	   if (rc < 0) goto stopmsg;
630	   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); }
631	   uart_log("  上升完成, 方向锁定: 轴1=%+d 轴2=%+d 轴3=%+d\r\n", fb_sign[1], fb_sign[2], fb_sign[3]);
632	
633	   /* 阶段2: 零偏标定 —— 平台保持不动, 采集~1s传感器角度求均值(用户此时把踝置于中位) */
634	   uart_log("  标定中(请把脚踝放到中位并保持)...\r\n");
635	   double sum_r = 0.0, sum_p = 0.0; int ncal = 0;
636	   for (i = 0; i < SENSOR_CAL_FRAMES; i++) {
637	      if (sensor_get(&r, &p)) { sum_r += r; sum_p += p; ncal++; }
638	      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0));
639	      cycle(); poll_cmd();
640	      if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
641	      if (g_abort)     { rc = -2; goto stopmsg; }
642	   }
643	   if (ncal < 5) {   /* 标定期几乎没收到帧 → 传感器/转发链路没通, 安全退出 */
644	      uart_log("!! 标定失败: 仅收到%d帧, 检查USART6接线(PC7)/电脑转发/波特率. 退出.\r\n", ncal);
645	      rc = -2; goto stopmsg;
646	   }
647	   double roll0 = sum_r / ncal, pitch0 = sum_p / ncal;
648	   uart_log("  标定完成(%d帧): Roll0=%.2f° Pitch0=%.2f°. 开始跟随.\r\n", ncal, roll0, pitch0);
649	
650	   /* 阶段3: 实时跟随
651	    * 关键: 目标角 ta/tb 在【每个控制周期(4ms)】朝最新传感器读数 ar_raw/br_raw 平滑逼近,
652	    * 而不是只在收到帧(~20Hz)那一帧突跳。这样 (l_now-l_prev)/DT 得到的前馈速度是
653	    * 连续的(不再是被限加速削平的20Hz尖峰), 平台幅度/跟手都恢复正常。
654	    * ar_raw/br_raw 只在收到帧时刷新, 帧间保持 → 信号丢失即自然冻结在最后目标。 */
655	   const double dead    = SENSOR_DEADBAND_DEG * M_PI / 180.0;
656	   const double a_ctrl  = DT / (SENSOR_FOLLOW_TAU + DT);   /* 控制周期平滑系数 */
657	   int lost = 0;
658	   for (;;) {
659	      if (sensor_get(&r, &p)) {
660	         lost = 0;
661	         double ar = -SENSOR_GAIN * ((double)r - roll0)  * M_PI / 180.0;   /* Roll → α(内翻外翻); 负号消除镜像反向, ×增益放大幅度 */
662	         /* Pitch→ β(跖屈背伸)。br>0 = +β = 往+X(1轴)。此侧要1轴反向缩回, 跨换向死区手感弱,
663	          * 故对 +β 侧单独用更大增益补偿; -β(-X)侧保持基准增益。中位处 db=0 → br=0, 无静态偏置。 */
664	         double db  = (double)p - pitch0;
665	         double gb  = (db < 0.0) ? SENSOR_GAIN_BETA_XP : SENSOR_GAIN;   /* db<0 → -GAIN*db>0 → +β → +X, 加大 */
666	         double br  = -gb * db * M_PI / 180.0;
667	         if (ar >  ang_lim) ar =  ang_lim;                  /* 限幅(防逆解超程杆长) */
668	         if (ar < -ang_lim) ar = -ang_lim;
669	         if (br >  ang_lim) br =  ang_lim;
670	         if (br < -ang_lim) br = -ang_lim;
671	         if (ar - ar_raw > dead || ar_raw - ar > dead) ar_raw = ar;   /* 死区: 抑制静止抖动 */
672	         if (br - br_raw > dead || br_raw - br > dead) br_raw = br;
673	      } else {
674	         /* 无新帧: 保持 ar_raw/br_raw 不变; 看门狗超时 → 目标不再变→平台冻结在安全位 */
675	         if (lost < SENSOR_WATCHDOG_FRAMES) lost++;
676	      }
677	
678	      /* 控制周期平滑: 目标角每帧挪一点, 使前馈速度连续 */
679	      ta += a_ctrl * (ar_raw - ta);
680	      tb += a_ctrl * (br_raw - tb);
681	
682	      double l_now[3];
683	      fk_3rps(ta, tb, z_eff, l_now);
684	      for (sl = 1; sl <= ctx.slavecount; sl++) {
685	         double v_mms = (l_now[sl - 1] - l_prev[sl - 1]) / DT;   /* mm/s */
686	         double vff   = v_mms / MM_PER_PULSE;                    /* pulse/s */
687	         write_pdo(sl, 0x000F, vel_closed(sl, vff));
688	         l_prev[sl - 1] = l_now[sl - 1];
689	      }
690	      cycle(); poll_cmd();
691	      if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
692	      if (g_abort)     { rc = -2; goto stopmsg; }   /* x = 退出跟随 */
693	   }
694	
695	stopmsg:
696	   if (rc == -1) {   /* 从站报警: 不再主动移动, 就地降速回待机 */
697	      uart_log("!! 从站%d 报警, 平滑降速回待机\r\n", g_ec_fault);
698	      ramp_to_zero();
699	      return rc;
700	   }
701	   /* rc==-2(用户 x 退出 / 标定失败): 先把平台归平, 再下降回原点 */
702	   uart_log("!! 退出跟随: 归平 → 下降回原点\r\n");
703	   g_abort = 0;                       /* 清急停, 让后续下降动作能执行 */
704	   for (i = 0; i < 400; i++) {        /* 目标角指数衰减到 0, 平滑归平 */
705	      ta *= 0.98; tb *= 0.98;
706	      double l_now[3];
707	      fk_3rps(ta, tb, z_eff, l_now);
708	      for (sl = 1; sl <= ctx.slavecount; sl++) {
709	         double vff = (l_now[sl - 1] - l_prev[sl - 1]) / DT / MM_PER_PULSE;
710	         write_pdo(sl, 0x000F, vel_closed(sl, vff));
711	         l_prev[sl - 1] = l_now[sl - 1];
712	      }
713	      cycle(); poll_cmd();
714	      if (any_fault()) { g_ec_fault = any_fault(); ramp_to_zero(); return -1; }
715	   }
716	   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); }
717	   move_ramp(-(double)raise_vel, cruise_frames);
718	   for (i = 0; i < 150; i++) {        /* 末端主动闭环回精确原点 */
719	      for (sl = 1; sl <= ctx.slavecount; sl++) { cmd_pos[sl] = 0.0; write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); }
720	      cycle(); poll_cmd();
721	   }
722	   uart_log(">>> 跟随结束, 残余误差: 轴1=%.2fmm 轴2=%.2fmm 轴3=%.2fmm <<<\r\n",
723	            (read_pos63(1) - start_pos[1]) * MM_PER_PULSE,
724	            (read_pos63(2) - start_pos[2]) * MM_PER_PULSE,
725	            (read_pos63(3) - start_pos[3]) * MM_PER_PULSE);
726	   return rc;
727	}
728	
729	/* ================= 归零(堵转检测收回) ================= */
730	static void run_homing(void)
731	{
732	   int i, sl;
733	   int stall_cnt[EC_MAXSLAVE] = {0}, stopped[EC_MAXSLAVE] = {0};
734	   int32_t vel_cmd[EC_MAXSLAVE];
735	   int32_t pos_win[EC_MAXSLAVE];    /* 每轴去抖窗口起点位置(位置法堵转检测用) */
736	   g_abort = 0; g_ec_fault = 0;
737	   g_status = 3; g_cur_mode = 99;   /* HMI 反馈: 归零中 */
738	
739	   uart_log(">>> 归零: 三轴收缩@%dpulse/s, 堵转自停 <<<\r\n", RETRACT_VEL);
740	   for (sl = 1; sl <= ctx.slavecount; sl++) vel_cmd[sl] = RETRACT_VEL;
741	
742	   /* 缓慢加速到收缩速度 */
743	   for (i = 0; i < 100; i++) {
744	      int32_t v = (int32_t)((int64_t)RETRACT_VEL * i / 100);
745	      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, v);
746	      cycle(); poll_cmd();
747	      if (g_abort) { ramp_to_zero(); uart_log("!! 归零急停\r\n"); return; }
748	   }
749	
750	   for (sl = 1; sl <= ctx.slavecount; sl++) pos_win[sl] = read_pos63(sl);   /* 检测起点 */
751	
752	   for (i = 0; i < HOME_MAX_FRAMES; i++) {
753	      for (sl = 1; sl <= ctx.slavecount; sl++) {
754	         if (stopped[sl]) continue;
755	         /* 位置法: 看窗口里位置还动不动, 不看瞬时速度。累计位移够大→还在收缩,
756	            窗口前移并重置; 位置几乎不动且连续够久→判定顶到机械限位。 */
757	         int32_t pos = read_pos63(sl);
758	         int32_t moved = pos - pos_win[sl]; if (moved < 0) moved = -moved;
759	         if (moved > STALL_WIN_EPS) {
760	            pos_win[sl] = pos; stall_cnt[sl] = 0;
761	         } else {
762	            if (++stall_cnt[sl] >= STALL_FRAMES) {
763	               stopped[sl] = 1; vel_cmd[sl] = 0;
764	               uart_log("  ★ 轴%d 到限位, 位置=%ld\r\n", sl, (long)pos);
765	            }
766	         }
767	      }
768	      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_cmd[sl]);
769	      cycle(); poll_cmd();
770	      if (g_abort) { ramp_to_zero(); uart_log("!! 归零急停\r\n"); return; }
771	
772	      int all = 1;
773	      for (sl = 1; sl <= ctx.slavecount; sl++) if (!stopped[sl]) all = 0;
774	      if (all) { uart_log(">>> 归零完成, 所有轴到位 <<<\r\n"); return; }
775	   }
776	   uart_log(">>> 归零超时(部分轴未检测到限位) <<<\r\n");
777	   ramp_to_zero();
778	}
779	
780	/* ================= EtherCAT 启动(扫描→PDO→OP→使能) ================= */
781	static int setup_pdo_csv(int slave)
782	{
783	   uint8 u8; uint16 u16; uint32 u32; int wkc;
784	
785	   u8 = 0;
786	   wkc = ecx_SDOwrite(&ctx, slave, 0x1C12, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
787	   if (wkc <= 0) { uart_log("  [错误] 从站%d SDO写(1C12)失败\r\n", slave); return -1; }
788	   u8 = 0; ecx_SDOwrite(&ctx, slave, 0x1600, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
789	   u32 = 0x60400010; ecx_SDOwrite(&ctx, slave, 0x1600, 0x01, FALSE, 4, &u32, EC_TIMEOUTRXM);
790	   u32 = 0x60600008; ecx_SDOwrite(&ctx, slave, 0x1600, 0x02, FALSE, 4, &u32, EC_TIMEOUTRXM);
791	   u32 = 0x60FF0020; ecx_SDOwrite(&ctx, slave, 0x1600, 0x03, FALSE, 4, &u32, EC_TIMEOUTRXM);
792	   u8 = 3; ecx_SDOwrite(&ctx, slave, 0x1600, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
793	   u16 = 0x1600; ecx_SDOwrite(&ctx, slave, 0x1C12, 0x01, FALSE, 2, &u16, EC_TIMEOUTRXM);
794	   u8 = 1;       ecx_SDOwrite(&ctx, slave, 0x1C12, 0x00, FALSE, 1, &u8,  EC_TIMEOUTRXM);
795	
796	   u8 = 0; ecx_SDOwrite(&ctx, slave, 0x1C13, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
797	   u8 = 0; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
798	   u32 = 0x60410010; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x01, FALSE, 4, &u32, EC_TIMEOUTRXM);
799	   u32 = 0x60610008; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x02, FALSE, 4, &u32, EC_TIMEOUTRXM);
800	   u32 = 0x60630020; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x03, FALSE, 4, &u32, EC_TIMEOUTRXM);
801	   u32 = 0x606C0020; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x04, FALSE, 4, &u32, EC_TIMEOUTRXM);
802	   u8 = 4; ecx_SDOwrite(&ctx, slave, 0x1A00, 0x00, FALSE, 1, &u8, EC_TIMEOUTRXM);
803	   u16 = 0x1A00; ecx_SDOwrite(&ctx, slave, 0x1C13, 0x01, FALSE, 2, &u16, EC_TIMEOUTRXM);
804	   u8 = 1;       ecx_SDOwrite(&ctx, slave, 0x1C13, 0x00, FALSE, 1, &u8,  EC_TIMEOUTRXM);
805	   return 0;
806	}
807	
808	/* 平滑下电: 脱力 → 退OP→SAFE-OP→INIT, 关看门狗, 之后可安全断电/拔网线 */
809	static void graceful_shutdown(void)
810	{
811	   int i, sl;
812	   g_status = 5;   /* HMI 反馈: 已下电 */
813	   uart_log("下电: 脱力...\r\n");
814	   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0006, 0); cycle(); }
815	   ctx.slavelist[0].state = EC_STATE_SAFE_OP;
816	   ecx_writestate(&ctx, 0);
817	   ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
818	   ctx.slavelist[0].state = EC_STATE_INIT;
819	   ecx_writestate(&ctx, 0);
820	   ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE * 4);
821	   ecx_close(&ctx);
822	   uart_log(">>> 已退到 INIT, 可安全断电/拔网线 <<<\r\n");
823	}
824	
825	/* EtherCAT 起不来(网卡失败/没扫到从站)时的降级空转: 不返回, 仍每~4ms服务一次
826	 * Modbus/HMI 与串口命令, 让上位机能连上板子看到状态(从站数0、状态=故障), 心跳灯照闪。
827	 * 也正是"没接伺服时单测 Modbus"依赖的路径 —— 否则扫不到从站会直接退出, Modbus 一声不吭。*/
828	static void modbus_idle_loop(void)
829	{
830	   g_status = 4;      /* 故障/未就绪(EtherCAT 未建立) */
831	   g_cur_mode = 99;
832	   uart_log(">>> EtherCAT 未就绪, 进入 Modbus 降级空转(仍可被 HMI/电脑 连接单测) <<<\r\n");
833	   int hb = 0;
834	   for (;;) {
835	      poll_cmd();          /* modbus_poll/sync/feedback + USART1/USB 命令 */
836	      heartbeat();         /* 心跳灯照闪, 表明没死 */
837	      osal_usleep(CYC_US); /* 无 EtherCAT, 相对延时凑 ~4ms 节拍即可 */
838	      if (++hb >= 250) {   /* 每约1秒播报, 便于终端实时监看 Modbus 是否收到 HMI 请求 */
839	         hb = 0;
840	         modbus_loopback_selftest_tx();   /* 回环自测: 平时空操作(见 modbus_slave.c 的 MB_LOOPBACK_TEST) */
841	         uart_log("[降级空转] Modbus 收到帧=%lu 有效=%lu\r\n",
842	                  (unsigned long)modbus_stat_frames, (unsigned long)modbus_stat_valid);
843	      }
844	   }
845	}
846	
847	void ecat_motion_run(void)
848	{
849	   int sl, i;
850	
851	   uart_log_init();
852	   modbus_init();   /* USART3 上的 Modbus RTU 从站(HMI 用), 与串口命令并行 */
853	   sensor_init();   /* USART6 上的 WT901 姿态传感器接入(传感器跟随模式用) */
854	   uart_log("\r\n\r\n===== STM32 SOEM 康复运动 (CSV, 命令驱动) =====\r\n");
855	
856	   uart_log("上次复位原因:");
857	   if (g_reset_cause & RCC_CSR_PORRSTF)  uart_log(" 上电复位(POR/PDR)");
858	   if (g_reset_cause & RCC_CSR_PADRSTF)  uart_log(" 外部NRST引脚复位(硬件拉低, 疑似DTR)");
859	   if (g_reset_cause & RCC_CSR_SFTRSTF)  uart_log(" 软件复位");
860	   if (g_reset_cause & RCC_CSR_IWDGRSTF) uart_log(" 独立看门狗复位");
861	   if (g_reset_cause & RCC_CSR_WWDGRSTF) uart_log(" 窗口看门狗复位");
862	   if (g_reset_cause & RCC_CSR_LPWRRSTF) uart_log(" 低功耗复位");
863	   uart_log("  (CSR=0x%08lX)\r\n", (unsigned long)g_reset_cause);
864	
865	   g_ec_phase = 1;
866	   if (!ecx_init(&ctx, "stm32eth")) {
867	      uart_log("[错误] 网卡初始化失败\r\n"); g_ec_phase = -1; modbus_idle_loop();
868	   }
869	   uart_log("网卡就绪, 扫描总线...\r\n");
870	
871	   g_ec_phase = 2;
872	   if (ecx_config_init(&ctx) <= 0) {
873	      uart_log("[错误] 没扫到从站! 检查网线/伺服上电\r\n"); g_ec_phase = -2; ecx_close(&ctx); modbus_idle_loop();
874	   }
875	   g_ec_slavecount = ctx.slavecount;
876	   uart_log(">>> 扫到 %d 个从站 <<<\r\n", ctx.slavecount);
877	   for (sl = 1; sl <= ctx.slavecount; sl++) uart_log("    从站%d: %s\r\n", sl, ctx.slavelist[sl].name);
878	   if (ctx.slavecount < 1) { g_ec_phase = -2; ecx_close(&ctx); modbus_idle_loop(); }
879	
880	   /* PRE-OP: 配 CSV 的 PDO */
881	   g_ec_phase = 3;
882	   ctx.slavelist[0].state = EC_STATE_PRE_OP;
883	   ecx_writestate(&ctx, 0);
884	   ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
885	   ecx_readstate(&ctx);   /* 同步各从站本地状态, 否则SDO写静默失败 */
886	   for (sl = 1; sl <= ctx.slavecount; sl++)
887	      if (setup_pdo_csv(sl) < 0) { g_ec_phase = -3; ecx_close(&ctx); return; }
888	
889	   ecx_config_map_group(&ctx, IOmap, 0);
890	
891	   /* SAFE-OP → OP */
892	   g_ec_phase = 4;
893	   ctx.slavelist[0].state = EC_STATE_SAFE_OP;
894	   ecx_writestate(&ctx, 0);
895	   ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
896	   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0000, 0); cycle(); }
897	
898	   ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
899	   ecx_writestate(&ctx, 0);
900	   for (i = 0; i < 200; i++) {
901	      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0000, 0);
902	      if (i % 40 == 0) ecx_writestate(&ctx, 0);
903	      cycle();
904	   }
905	   ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
906	   uart_log("OP 建立, WKC=%d (正常≈%d)\r\n", g_wkc, ctx.slavecount * 3);
907	
908	   /* CiA402 使能: 0x06 → 0x07 → 0x0F */
909	   g_ec_phase = 5;
910	   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0006, 0); cycle(); }
911	   for (i = 0; i < 200; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x0007, 0); cycle(); }
912	   for (i = 0; i < 300; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0); cycle(); }
913	   for (sl = 1; sl <= ctx.slavecount; sl++)
914	      uart_log("    从站%d 状态字=0x%04X %s\r\n", sl, read_sw(sl),
915	               (read_sw(sl) & 0x0008) ? "[报警!]" : (((read_sw(sl) & 0x006F) == 0x0027) ? "[已使能]" : ""));
916	
917	   /* ===== 待机: 听命令 ===== */
918	   g_ec_phase = 99;
919	   for (sl = 1; sl <= ctx.slavecount; sl++) start_pos[sl] = read_pos63(sl);  /* 反馈位置以此刻为0基准, 避免开机显示绝对编码值 */
920	   uart_log("\r\n>>> 进入待机, 伺服使能保持零速. 发 ? 查看命令. 建议先 h 归零 <<<\r\n");
921	   int hb = 0;
922	   while (!g_quit) {
923	      g_status = g_ec_fault ? 4 : 1;   /* HMI 反馈: 故障未清则4, 否则待机1 */
924	      g_cur_mode = 99;
925	      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, 0);
926	      { float _r, _p; sensor_get(&_r, &_p); }   /* 待机也驱动校验计数,供 Bridge 诊断 */
927	      cycle();
928	      poll_cmd();
929	
930	      if (++hb >= 250) {   /* 每约1秒播报, 终端实时监看 + Modbus 联调(看HMI请求到没到) */
931	         hb = 0;
932	         modbus_loopback_selftest_tx();   /* 回环自测: 平时空操作(见 modbus_slave.c 的 MB_LOOPBACK_TEST) */
933	         uart_log("[待机] WKC=%d 从站=%d | Modbus 收到帧=%lu 有效=%lu\r\n",
934	                  g_wkc, ctx.slavecount,
935	                  (unsigned long)modbus_stat_frames, (unsigned long)modbus_stat_valid);
936	      }
937	
938	      if (g_do_home)          { g_do_home = 0; g_do_sensor = 0; g_run_request = -1; run_homing(); uart_log(">>> 回待机 <<<\r\n"); }
939	      else if (g_do_sensor)   { g_do_sensor = 0; g_run_request = -1; run_sensor_mode(); uart_log(">>> 回待机 <<<\r\n"); }
940	      else if (g_run_request >= 0) { int m = g_run_request; g_run_request = -1; g_do_home = 0; run_rehab_mode(m); uart_log(">>> 回待机 <<<\r\n"); }
941	   }
942	
943	   /* 收到 q: 平滑下电退出 */
944	   graceful_shutdown();
945	   while (1) { osal_usleep(100000); }
946	}
947	