#ifndef TWIN_STATE_H
#define TWIN_STATE_H

/* VT1保留原有0..7，追加原因码；UDP目标格式和既有Modbus地址不变。 */
enum twin_detail {
   VT_OFF, VT_WAITING, VT_MOVING, VT_REACHED, VT_LINK_LOST,
   VT_NEEDS_REFERENCE, VT_REJECTED, VT_FAULT, VT_NOT_READY, VT_STOPPING
};
enum reference_reason {
   REF_UNKNOWN, REF_HOMED, REF_REHOMING, REF_HOME_INTERRUPTED,
   REF_DRIVE_FAULT, REF_BUS_LOST, REF_SHUTDOWN
};
typedef struct {
   int valid;
   int established;  /* 曾有机械参考：失效时也不偷偷切换反馈坐标原点。 */
   enum reference_reason reason;
} twin_reference_t;

static inline void twin_reference_invalidate(twin_reference_t *r,
                                             enum reference_reason reason)
{
   r->valid = 0;
   r->reason = reason;
}
static inline void twin_reference_establish(twin_reference_t *r)
{
   r->valid = 1;
   r->established = 1;
   r->reason = REF_HOMED;
}

/* 不从“总线又正常了”推导参考自动恢复；恢复必须有新的参考建立事件。 */
static inline int twin_reference_observe(twin_reference_t *r, int bus_ready, int drive_fault)
{
   if (!r->valid) return 0;
   if (!bus_ready) {
      twin_reference_invalidate(r, REF_BUS_LOST);
      return 1;
   }
   if (drive_fault) {
      twin_reference_invalidate(r, REF_DRIVE_FAULT);
      return 1;
   }
   return 0;
}

static inline int twin_entry_detail(int ready, int reference_valid)
{
   if (!ready) return VT_NOT_READY;
   return reference_valid ? VT_OFF : VT_NEEDS_REFERENCE;
}

static inline int twin_start_allowed(int status, int abort_requested, int quit_requested)
{
   return status == 1 && !abort_requested && !quit_requested;
}

/* HMI寄存器25只显示六种简洁状态；详细原因仍由VT1/VR1交给电脑。 */
static inline int twin_hmi_progress(int detail)
{
   switch (detail) {
      case VT_OFF: return 0;       /* 空白，不在其他模式下堆叠诊断。 */
      case VT_WAITING: return 1;
      case VT_MOVING: return 2;
      case VT_REACHED: return 3;
      case VT_STOPPING: return 5;
      default: return 4;           /* 未就绪/异常：具体原因见电脑。 */
   }
}
#endif
