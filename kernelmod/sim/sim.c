#include "kernel_mab.h"        // pulls in sim.h under -DSIM (stdio, types, stubs)
#include "data/xalanx250.h"

// The recorded trace carries cycles + load-head counts per interval. LoadHead
// mode uses them directly. IPC mode has no instruction-retired counter in the
// trace, so we synthesize one below (see feed loop) - flip MAB_SCORE_MODE in
// kernel_mab.h and both the score path and this feed switch with it.
#if MAB_SCORE_MODE == MAB_SCORE_IPC
#define SCORE_MODE_NAME "IPC (synthetic instr = cycles - load_head)"
#else
#define SCORE_MODE_NAME "LoadHead"
#endif

// Trace indexed by arm: 0 = boot-default, 1 = aggressive.
static const uint64_t *arm_cycles[AVAILABLE_ARMS] = { cycles_bootdefault, cycles_allenable };
static const uint64_t *arm_ldhead[AVAILABLE_ARMS] = { ldhead_bootdefault, ldhead_allenable };

// Number of recorded intervals.
#define TRACE_LEN ((int)(sizeof(cycles_bootdefault) / sizeof(cycles_bootdefault[0])))

struct core_state_s corestate[MAX_NUM_CORES];
int sys_first_core = 0;

__u64 ktime_get_ns(void)      { return 0; }
int   msr_set_dirty(int core) { (void)core; return 0; }

int main(void)
{
	int runs[AVAILABLE_ARMS] = {0};

	printf("** MAB sim - scoring mode: %s\n", SCORE_MODE_NAME);

	// Disable cores 1..3 so mab_module_score() aggregates core 0 alone;
	// leaving them enabled averages the score over 4 cores and quarters it.
	for (int c = 1; c < CORES_PER_COMPUTE_MODULE; c++)
		corestate[c].core_disabled = 1;

	mab_setup_default_arms();
	mab_modules[0].initialized = MAB_INIT_NOT_STARTED;

	for (int t = 0; t < TRACE_LEN; t++) {
		int arm = mab_modules[0].active_arm;
		uint64_t cyc = arm_cycles[arm][t];
		uint64_t ldh = arm_ldhead[arm][t];

#if MAB_SCORE_MODE == MAB_SCORE_IPC
		// No instruction trace was captured; model instructions retired as
		// the cycles not stalled at the load head (IPC_peak = 1).
		uint64_t event_a = (ldh < cyc) ? (cyc - ldh) : 0;
#else
		uint64_t event_a = ldh;
#endif

		// pmu_update() stand-in: raw keeps accumulating, old = previous raw.
		// SCORE_EVENT_A / SCORE_EVENT_B track whatever the current mode reads.
		corestate[0].pmu_old[SCORE_EVENT_B] = corestate[0].pmu_raw[SCORE_EVENT_B];
		corestate[0].pmu_old[SCORE_EVENT_A] = corestate[0].pmu_raw[SCORE_EVENT_A];
		corestate[0].pmu_raw[SCORE_EVENT_B] += cyc;
		corestate[0].pmu_raw[SCORE_EVENT_A] += event_a;

		if (mab_modules[0].initialized != MAB_INIT_DONE) {
			mab_init_module_step(0);
		} else {
			mab_tuning(0);
			runs[mab_modules[0].active_arm]++;
		}
	}

	printf("\n** Benchmark done (%s)\n", SCORE_MODE_NAME);
	for (int i = 0; i < AVAILABLE_ARMS; i++)
		printf("  arm %d: %d runs\n", i, runs[i]);

	return 0;
}
