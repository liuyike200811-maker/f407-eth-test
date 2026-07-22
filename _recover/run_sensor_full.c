459	/* raise_vel: 匀速目标(pulse/s, 正=上升 负=下降); 内部自动加/匀/减速三段 */
460	static int move_ramp(double raise_vel, int cruise_frames)
461	{
462	   int i;
463	   /* 加速 */
464	   for (i = 0; i < RAMP_FRAMES; i++) {
465	      double vff = raise_vel * i / RAMP_FRAMES;
466	      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, vff));
467	      cycle(); poll_cmd();
468	      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
469	      if (g_abort) return -2;
470	   }
471	   /* 匀速 */
472	   for (i = 0; i < cruise_frames; i++) {
473	      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, raise_vel));
474	      cycle(); poll_cmd();
475	      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
476	      if (g_abort) return -2;
477	   }
478	   /* 减速 */
479	   for (i = 0; i < RAMP_FRAMES; i++) {
480	      double vff = raise_vel * (RAMP_FRAMES - i) / RAMP_FRAMES;
481	      for (int sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, vff));
482	      cycle(); poll_cmd();
483	      if (any_fault()) { g_ec_fault = any_fault(); return -1; }
484	      if (g_abort) return -2;
485	   }
486	   return 0;
487	}
488	
489	/* ================= 康复模式 (上升→运动→下降) ================= */
490	static int run_rehab_mode(int mode)
491	{
492	   int i, sl, rc;
493	   g_abort = 0;
494	   g_ec_fault = 0;
495	   g_status = 2; g_cur_mode = mode;   /* HMI 反馈: 运行中 + 当前模式 */
496	   motion_reset();
497	
498	   int32_t raise_vel  = (int32_t)((double)g_rise_rpm / 60.0 * PULSE_PER_REV);
499	   int32_t rise_pulse = (int32_t)((double)g_rise_mm * PULSE_PER_REV / LEAD_MM);
500	   int cruise_frames  = (int)((double)rise_pulse / ((double)raise_vel * DT));
501	   if (cruise_frames < 1) cruise_frames = 1;
502	
503	   uart_log(">>> 模式%d 开始: 上升%dmm@%dRPM → 运动 → 下降 <<<\r\n", mode, g_rise_mm, g_rise_rpm);
504	
505	   /* 阶段1: 上升 */
506	   rc = move_ramp((double)raise_vel, cruise_frames);
507	   if (rc < 0) goto stopmsg;
508	   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); }
509	   uart_log("  上升完成, 方向锁定: 轴1=%+d 轴2=%+d 轴3=%+d\r\n", fb_sign[1], fb_sign[2], fb_sign[3]);
510	
511	   /* 阶段2: 运动 */
512	   if (mode == 0) {
513	      /* 综合波浪(电缸空间三轴相位差) */
514	      int32_t wave_amp = (int32_t)((double)g_wave_mm * PULSE_PER_REV / LEAD_MM);
515	      int wave_period = (int)((double)wave_amp * 2.0 * M_PI / (WAVE_PEAK_RPM / 60.0 * PULSE_PER_REV) / DT);
516	      if (wave_period < 100) wave_period = 100;
517	      double wave_v_amp = (double)wave_amp * 2.0 * M_PI / wave_period / DT;
518	      int total = g_cycles * wave_period;
519	      uart_log("  阶段2 综合波浪: 幅%dmm 周期%d帧 × %d\r\n", g_wave_mm, wave_period, g_cycles);
520	      for (i = 0; i < total; i++) {
521	         double t = 2.0 * M_PI * i / wave_period;
522	         for (sl = 1; sl <= ctx.slavecount; sl++) {
523	            double vff = wave_v_amp * cos(t + PHASE[sl]);
524	            write_pdo(sl, 0x000F, vel_closed(sl, vff));
525	         }
526	         cycle(); poll_cmd();
527	         if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
528	         if (g_abort) { rc = -2; goto stopmsg; }
529	         /* 运动中绝不打印: uart_log 是阻塞式串口发送(每字符忙等~87us, 一行~4ms),
530	            会把 4ms 控制周期撑到十几ms → free-run 伺服顿挫。统计留到运动结束汇报。 */
531	      }
532	   } else {
533	      /* FK 角度空间(1跖背 2内外 3环形 4八字) */
534	      double aa_r = g_aa_deg * M_PI / 180.0;
535	      double ba_r = g_ba_deg * M_PI / 180.0;
536	      if (aa_r > M_PI / 6) aa_r = M_PI / 6;    /* 限幅±30° */
537	      if (ba_r > M_PI / 6) ba_r = M_PI / 6;
538	      double freq = g_freq_cHz / 100.0;
539	      if (freq < 0.05) freq = 0.05;
540	      if (freq > 0.8)  freq = 0.8;
541	      double z_eff = FK_Z0 + (double)g_rise_mm;
542	      int period_frames = (int)(1.0 / (freq * DT));
543	      if (period_frames < 50) period_frames = 50;
544	      int total = g_cycles * period_frames;
545	      double omega = 2.0 * M_PI * freq;
546	      uart_log("  阶段2 FK模式%d: α%d° β%d° 频%d厘赫 周期%d帧 × %d\r\n",
547	               mode, g_aa_deg, g_ba_deg, g_freq_cHz, period_frames, g_cycles);
548	      for (i = 0; i < total; i++) {
549	         double t0 = omega * i * DT, t1 = omega * (i + 1) * DT;
550	         double a0, b0, a1, b1, lc[3], ln[3];
551	         get_angles(mode, t0, aa_r, ba_r, &a0, &b0);
552	         get_angles(mode, t1, aa_r, ba_r, &a1, &b1);
553	         fk_3rps(a0, b0, z_eff, lc);
554	         fk_3rps(a1, b1, z_eff, ln);
555	         for (sl = 1; sl <= ctx.slavecount; sl++) {
556	            double v_mms = (ln[sl - 1] - lc[sl - 1]) / DT;   /* mm/s */
557	            double vff = v_mms / MM_PER_PULSE;               /* pulse/s */
558	            write_pdo(sl, 0x000F, vel_closed(sl, vff));
559	         }
560	         cycle(); poll_cmd();
561	         if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
562	         if (g_abort) { rc = -2; goto stopmsg; }
563	         /* 运动中绝不打印(同上, 避免 uart_log 阻塞撑破 4ms 节拍) */
564	      }
565	   }
566	
567	   /* 阶段3: 下降回原点 */
568	   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); }
569	   rc = move_ramp(-(double)raise_vel, cruise_frames);
570	   if (rc < 0) goto stopmsg;
571	   /* 末端主动闭环回精确原点 */
572	   for (i = 0; i < 150; i++) {
573	      for (sl = 1; sl <= ctx.slavecount; sl++) { cmd_pos[sl] = 0.0; write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); }
574	      cycle(); poll_cmd();
575	      if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
576	      if (g_abort) { rc = -2; goto stopmsg; }
577	   }
578	
579	   uart_log(">>> 模式%d 完成, 残余误差: 轴1=%.2fmm 轴2=%.2fmm 轴3=%.2fmm <<<\r\n", mode,
580	            (read_pos63(1) - start_pos[1]) * MM_PER_PULSE,
581	            (read_pos63(2) - start_pos[2]) * MM_PER_PULSE,
582	            (read_pos63(3) - start_pos[3]) * MM_PER_PULSE);
583	   return 0;
584	
585	stopmsg:
586	   if (rc == -2) uart_log("!! 急停, 平滑降速回待机\r\n");
587	   else          uart_log("!! 从站%d 报警, 平滑降速回待机\r\n", g_ec_fault);
588	   ramp_to_zero();
589	   return rc;
590	}
591	
592	/* ================= 传感器实时跟随 (上升→标定→跟随→归平下降) =================
593	 * WT901 姿态 → 减零偏 → EMA → 死区 → 限幅 → fk_3rps 逆算三电缸目标长 →
594	 * 相邻帧长度差分得前馈速度 → vel_closed 闭环输出。x 急停退出, 信号丢失自动冻结。 */
595	static int run_sensor_mode(void)
596	{
597	   int i, sl, rc = 0;
598	   double ta = 0.0, tb = 0.0;   /* 死区后目标角(rad, 相对零偏) */
599	   double fa = 0.0, fb = 0.0;   /* EMA 平滑后角(rad) */
600	   double l_prev[3];            /* 上一帧 FK 目标杆长(mm), 用于差分求速度 */
601	   float  r, p;
602	
603	   g_abort = 0; g_ec_fault = 0;
604	   g_status = 2; g_cur_mode = 6;   /* HMI 反馈: 运行中 + 模式6(传感器跟随) */
605	   motion_reset();
606	
607	   int32_t raise_vel  = (int32_t)((double)g_rise_rpm / 60.0 * PULSE_PER_REV);
608	   int32_t rise_pulse = (int32_t)((double)g_rise_mm * PULSE_PER_REV / LEAD_MM);
609	   int cruise_frames  = (int)((double)rise_pulse / ((double)raise_vel * DT));
610	   if (cruise_frames < 1) cruise_frames = 1;
611	   double z_eff  = FK_Z0 + (double)g_rise_mm;
612	   double ang_lim = SENSOR_MAX_DEG * M_PI / 180.0;
613	   fk_3rps(0.0, 0.0, z_eff, l_prev);   /* 基线=标定姿态(零倾斜), 提前初始化防 goto 跳过 */
614	
615	   uart_log(">>> 传感器跟随: 上升%dmm → 零偏标定 → 实时跟随(发 x 退出) <<<\r\n", g_rise_mm);
616	
617	   /* 阶段1: 上升到工作高度(与其它模式一致) */
618	   rc = move_ramp((double)raise_vel, cruise_frames);
619	   if (rc < 0) goto stopmsg;
620	   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); }
621	   uart_log("  上升完成, 方向锁定: 轴1=%+d 轴2=%+d 轴3=%+d\r\n", fb_sign[1], fb_sign[2], fb_sign[3]);
622	
623	   /* 阶段2: 零偏标定 —— 平台保持不动, 采集~1s传感器角度求均值(用户此时把踝置于中位) */
624	   uart_log("  标定中(请把脚踝放到中位并保持)...\r\n");
625	   double sum_r = 0.0, sum_p = 0.0; int ncal = 0;
626	   for (i = 0; i < SENSOR_CAL_FRAMES; i++) {
627	      if (sensor_get(&r, &p)) { sum_r += r; sum_p += p; ncal++; }
628	      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0));
629	      cycle(); poll_cmd();
630	      if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
631	      if (g_abort)     { rc = -2; goto stopmsg; }
632	   }
633	   if (ncal < 5) {   /* 标定期几乎没收到帧 → 传感器/转发链路没通, 安全退出 */
634	      uart_log("!! 标定失败: 仅收到%d帧, 检查USART6接线(PC7)/电脑转发/波特率. 退出.\r\n", ncal);
635	      rc = -2; goto stopmsg;
636	   }
637	   double roll0 = sum_r / ncal, pitch0 = sum_p / ncal;
638	   uart_log("  标定完成(%d帧): Roll0=%.2f° Pitch0=%.2f°. 开始跟随.\r\n", ncal, roll0, pitch0);
639	
640	   /* 阶段3: 实时跟随 */
641	   int lost = 0;
642	   for (;;) {
643	      if (sensor_get(&r, &p)) {
644	         lost = 0;
645	         double ar = ((double)r - roll0)  * M_PI / 180.0;   /* Roll → α(内翻外翻) */
646	         double br = ((double)p - pitch0) * M_PI / 180.0;   /* Pitch→ β(跖屈背伸) */
647	         fa += SENSOR_EMA_ALPHA * (ar - fa);                /* EMA 平滑 */
648	         fb += SENSOR_EMA_ALPHA * (br - fb);
649	         double dead = SENSOR_DEADBAND_DEG * M_PI / 180.0;  /* 死区: 变化够大才更新目标 */
650	         if (fa - ta > dead || ta - fa > dead) ta = fa;
651	         if (fb - tb > dead || tb - fb > dead) tb = fb;
652	         if (ta >  ang_lim) ta =  ang_lim;                  /* 限幅 */
653	         if (ta < -ang_lim) ta = -ang_lim;
654	         if (tb >  ang_lim) tb =  ang_lim;
655	         if (tb < -ang_lim) tb = -ang_lim;
656	      } else {
657	         /* 无新帧: 看门狗超时则保持目标不变(FK差分=0→速度自然归0, 平台冻结在安全位) */
658	         if (lost < SENSOR_WATCHDOG_FRAMES) lost++;
659	      }
660	
661	      double l_now[3];
662	      fk_3rps(ta, tb, z_eff, l_now);
663	      for (sl = 1; sl <= ctx.slavecount; sl++) {
664	         double v_mms = (l_now[sl - 1] - l_prev[sl - 1]) / DT;   /* mm/s */
665	         double vff   = v_mms / MM_PER_PULSE;                    /* pulse/s */
666	         write_pdo(sl, 0x000F, vel_closed(sl, vff));
667	         l_prev[sl - 1] = l_now[sl - 1];
668	      }
669	      cycle(); poll_cmd();
670	      if (any_fault()) { g_ec_fault = any_fault(); rc = -1; goto stopmsg; }
671	      if (g_abort)     { rc = -2; goto stopmsg; }   /* x = 退出跟随 */
672	   }
673	
674	stopmsg:
675	   if (rc == -1) {   /* 从站报警: 不再主动移动, 就地降速回待机 */
676	      uart_log("!! 从站%d 报警, 平滑降速回待机\r\n", g_ec_fault);
677	      ramp_to_zero();
678	      return rc;
679	   }
680	   /* rc==-2(用户 x 退出 / 标定失败): 先把平台归平, 再下降回原点 */
681	   uart_log("!! 退出跟随: 归平 → 下降回原点\r\n");
682	   g_abort = 0;                       /* 清急停, 让后续下降动作能执行 */
683	   for (i = 0; i < 400; i++) {        /* 目标角指数衰减到 0, 平滑归平 */
684	      ta *= 0.98; tb *= 0.98;
685	      double l_now[3];
686	      fk_3rps(ta, tb, z_eff, l_now);
687	      for (sl = 1; sl <= ctx.slavecount; sl++) {
688	         double vff = (l_now[sl - 1] - l_prev[sl - 1]) / DT / MM_PER_PULSE;
689	         write_pdo(sl, 0x000F, vel_closed(sl, vff));
690	         l_prev[sl - 1] = l_now[sl - 1];
691	      }
692	      cycle(); poll_cmd();
693	      if (any_fault()) { g_ec_fault = any_fault(); ramp_to_zero(); return -1; }
694	   }
695	   for (i = 0; i < 100; i++) { for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); cycle(); poll_cmd(); }
696	   move_ramp(-(double)raise_vel, cruise_frames);
697	   for (i = 0; i < 150; i++) {        /* 末端主动闭环回精确原点 */
698	      for (sl = 1; sl <= ctx.slavecount; sl++) { cmd_pos[sl] = 0.0; write_pdo(sl, 0x000F, vel_closed(sl, 0.0)); }
699	      cycle(); poll_cmd();
700	   }
701	   uart_log(">>> 跟随结束, 残余误差: 轴1=%.2fmm 轴2=%.2fmm 轴3=%.2fmm <<<\r\n",
702	            (read_pos63(1) - start_pos[1]) * MM_PER_PULSE,
703	            (read_pos63(2) - start_pos[2]) * MM_PER_PULSE,
704	            (read_pos63(3) - start_pos[3]) * MM_PER_PULSE);
705	   return rc;
706	}
707	
708	/* ================= 归零(堵转检测收回) ================= */
709	static void run_homing(void)
710	{
711	   int i, sl;
712	   int stall_cnt[EC_MAXSLAVE] = {0}, stopped[EC_MAXSLAVE] = {0};
713	   int32_t vel_cmd[EC_MAXSLAVE];
714	   int32_t pos_win[EC_MAXSLAVE];    /* 每轴去抖窗口起点位置(位置法堵转检测用) */
715	   g_abort = 0; g_ec_fault = 0;
716	   g_status = 3; g_cur_mode = 99;   /* HMI 反馈: 归零中 */
717	
718	   uart_log(">>> 归零: 三轴收缩@%dpulse/s, 堵转自停 <<<\r\n", RETRACT_VEL);
719	   for (sl = 1; sl <= ctx.slavecount; sl++) vel_cmd[sl] = RETRACT_VEL;
720	
721	   /* 缓慢加速到收缩速度 */
722	   for (i = 0; i < 100; i++) {
723	      int32_t v = (int32_t)((int64_t)RETRACT_VEL * i / 100);
724	      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, v);
725	      cycle(); poll_cmd();
726	      if (g_abort) { ramp_to_zero(); uart_log("!! 归零急停\r\n"); return; }
727	   }
728	
729	   for (sl = 1; sl <= ctx.slavecount; sl++) pos_win[sl] = read_pos63(sl);   /* 检测起点 */
730	
731	   for (i = 0; i < HOME_MAX_FRAMES; i++) {
732	      for (sl = 1; sl <= ctx.slavecount; sl++) {
733	         if (stopped[sl]) continue;
734	         /* 位置法: 看窗口里位置还动不动, 不看瞬时速度。累计位移够大→还在收缩,
735	            窗口前移并重置; 位置几乎不动且连续够久→判定顶到机械限位。 */
736	         int32_t pos = read_pos63(sl);
737	         int32_t moved = pos - pos_win[sl]; if (moved < 0) moved = -moved;
738	         if (moved > STALL_WIN_EPS) {
739	            pos_win[sl] = pos; stall_cnt[sl] = 0;
740	         } else {
741	            if (++stall_cnt[sl] >= STALL_FRAMES) {
742	               stopped[sl] = 1; vel_cmd[sl] = 0;
743	               uart_log("  ★ 轴%d 到限位, 位置=%ld\r\n", sl, (long)pos);
744	            }
745	         }
746	      }
747	      for (sl = 1; sl <= ctx.slavecount; sl++) write_pdo(sl, 0x000F, vel_cmd[sl]);
748	      cycle(); poll_cmd();
749	      if (g_abort) { ramp_to_zero(); uart_log("!! 归零急停\r\n"); return; }
750	
751	      int all = 1;
752	      for (sl = 1; sl <= ctx.slavecount; sl++) if (!stopped[sl]) all = 0;
753	      if (all) { uart_log(">>> 归零完成, 所有轴到位 <<<\r\n"); return; }
754	   }
755	   uart_log(">>> 归零超时(部分轴未检测到限位) <<<\r\n");
756	   ramp_to_zero();
757	}
758	
759	/* ================= EtherCAT 启动(扫描→PDO→OP→使能) ================= */