/* Exercise the same policy functions included by the ARM firmware; no hardware. */
#include "../App/twin_state.h"
#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
#if defined(_WIN32)
__declspec(dllexport)
#endif
int twin_state_contract_test(void)
{
   twin_reference_t ref;
   ref.valid = 0;
   ref.established = 0;
   ref.reason = REF_UNKNOWN;
   CHECK(twin_entry_detail(1, ref.valid) == VT_NEEDS_REFERENCE);
   CHECK(twin_entry_detail(0, ref.valid) == VT_NOT_READY);
   CHECK(twin_start_allowed(1, 0, 0));
   CHECK(!twin_start_allowed(1, 1, 0)); /* Stop and start in the same poll. */
   CHECK(!twin_start_allowed(1, 0, 1)); /* Shutdown and start in the same poll. */
   CHECK(!twin_start_allowed(7, 0, 0)); /* No competing mode while running. */
   CHECK(twin_blockers(1,99,0,0,0,3,9,0,0,1,0)==0);
   CHECK(twin_blockers(7,7,0,0,0,3,9,0,0,1,0)==0);
   CHECK(twin_blockers(1,99,0,0,0,2,6,0,0,0,0)==(TB_AXES|TB_WKC|TB_REFERENCE));
   CHECK(twin_blockers(2,0,1,1,1,3,9,99,5,1,5)==
      (TB_BUSY|TB_STOP|TB_QUIT|TB_ESTOP|TB_FAULT|TB_ALARM|TB_POS1|TB_POS3));
   CHECK(!twin_reference_observe(&ref, 1, 0));
   CHECK(!ref.valid);                  /* Healthy feedback alone cannot home. */
   twin_reference_establish(&ref);
   CHECK(ref.valid && ref.established && ref.reason == REF_HOMED);
   CHECK(twin_entry_detail(1, ref.valid) == VT_OFF);
   CHECK(!twin_reference_observe(&ref, 1, 0));
   CHECK(ref.valid);                   /* Healthy modes/ordinary stops reuse it. */
   CHECK(twin_reference_observe(&ref, 1, 1));
   CHECK(!ref.valid && ref.established && ref.reason == REF_DRIVE_FAULT);
   CHECK(!twin_reference_observe(&ref, 1, 0) && !ref.valid);
   twin_reference_establish(&ref);
   CHECK(twin_reference_observe(&ref, 0, 0));
   CHECK(!ref.valid && ref.established && ref.reason == REF_BUS_LOST);
   CHECK(!twin_reference_observe(&ref, 1, 0) && !ref.valid);
   twin_reference_invalidate(&ref, REF_REHOMING);
   CHECK(!ref.valid && ref.established);
   twin_reference_invalidate(&ref, REF_HOME_INTERRUPTED);
   CHECK(!ref.valid && ref.reason == REF_HOME_INTERRUPTED);
   twin_reference_establish(&ref);
   twin_reference_invalidate(&ref, REF_SHUTDOWN);
   CHECK(!ref.valid && ref.established && ref.reason == REF_SHUTDOWN);

   CHECK(twin_hmi_progress(VT_OFF) == 0);
   CHECK(twin_hmi_progress(VT_WAITING) == 1);
   CHECK(twin_hmi_progress(VT_MOVING) == 2);
   CHECK(twin_hmi_progress(VT_REACHED) == 3);
   CHECK(twin_hmi_progress(VT_STOPPING) == 5);
   for (int detail = VT_LINK_LOST; detail <= VT_NOT_READY; ++detail)
      CHECK(twin_hmi_progress(detail) == 4);
   return 0;
}
