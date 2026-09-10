#include <stdlib.h>            // atoi

#include "kernel_mab.h"        // pulls in sim.h under -DSIM (stdio, types, stubs)

//#define GCC_250_1C
//#define GCC_1000_1C
#define XALANC_250_1C
//#define XALANC_1000_1C
//#define OMNET_250_1C
//#define OMNET_1000_1C

#include "data/1c_xalanc250.h"
#include "data/1c_xalanc1000.h"
#include "data/1c_gcc250.h"
#include "data/1c_gcc1000.h"
#include "data/1c_omnet250.h"
#include "data/1c_omnet1000.h"



#if MAB_SCORE_MODE == MAB_SCORE_IPC
#define SCORE_MODE_NAME "IPC"
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

int main(int argc, char *argv[])
{
	int runs[AVAILABLE_ARMS] = {0};
	int sim_steps = TRACE_LEN;

	// Optional argv[1]: number of trace intervals to simulate.
	// Defaults to the whole trace; must stay within its bounds.
	if (argc > 1) {
		sim_steps = atoi(argv[1]);
		if (sim_steps <= 0 || sim_steps > TRACE_LEN) {
			fprintf(stderr,
				"usage: %s [steps]  (steps 1..%d, default %d)\n",
				argv[0], TRACE_LEN, TRACE_LEN);
			return 1;
		}
	}

	printf("** MAB sim - scoring mode: %s\n", SCORE_MODE_NAME);
	printf("** Trace: %d interval(s) of %d\n", sim_steps, TRACE_LEN);

	for (int c = 1; c < CORES_PER_COMPUTE_MODULE; c++)
		corestate[c].core_disabled = 1;

	mab_setup_default_arms();
	mab_modules[0].initialized = MAB_INIT_NOT_STARTED;

	for (int t = 0; t < sim_steps; t++) {
		int arm = mab_modules[0].active_arm;
		uint64_t cyc = arm_cycles[arm][t];
		uint64_t ldh = arm_ldhead[arm][t];

#if MAB_SCORE_MODE == MAB_SCORE_IPC
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

	printf("\n** Benchmark done (%s, %d steps)\n", SCORE_MODE_NAME,
	       sim_steps);
	for (int i = 0; i < AVAILABLE_ARMS; i++)
		printf("  arm %d: %d runs\n", i, runs[i]);

	return 0;
}
