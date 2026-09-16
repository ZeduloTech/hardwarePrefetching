#define _GNU_SOURCE

#include <linux/timekeeping.h>

//#include "../include/atom_msr.h"
#include "kernel_common.h"

int sys_first_core = 0; //set by core_range API
int sys_active_cores = 0; //set by core_range API

struct core_state_s corestate[MAX_NUM_CORES];
int ddr_bw_target;

static struct msr_shadow_s msr_shadow[MAX_NUM_CORES][NR_OF_MSR];

// Write one prefetch MSR unless the new value is identical to the last value
// written for this core/MSR (tracked by the shadow). Returns 1 if a wrmsr was
// actually performed, 0 if it was skipped as unchanged.
// Example call:
//   msr_shadow_write(core_id, MSR_1321_INDEX, 0x1321);
static int msr_shadow_write(int core_id, int msr_index, unsigned int msr_addr)
{
	u64 value;

	value = corestate[core_id].pf_msr[msr_index].v;

	if (msr_shadow[core_id][msr_index].valid &&
	    msr_shadow[core_id][msr_index].value == value) {
		corestate[core_id].msr_skipped++;
		return 0;
	}

	__wrmsr(msr_addr, (u32)value, (u32)(value >> 32));

	// Record the new value in the shadow
	msr_shadow[core_id][msr_index].value = value;
	msr_shadow[core_id][msr_index].valid = 1;

	corestate[core_id].msr_written++;

	return 1;
}

//update the pmu field in the corestate struct for the given core.
// IMPORTANT: This has to be the core with core_id that calls this function or incorrect state will be updated

int pmu_update(int core_id)
{
	if (core_id < 0 || core_id >= MAX_NUM_CORES || corestate[core_id].core_disabled)
		return -EINVAL;

	for (int i = 0; i < PMU_COUNTERS; i++)
		corestate[core_id].pmu_old[i] = corestate[core_id].pmu_raw[i];

	// Update PMU counters using the defined indices from kernel_common.h
	corestate[core_id].pmu_raw[PERF_MEM_UOPS_RETIRED_ALL_LOADS] = native_read_pmc(0);
	corestate[core_id].pmu_raw[PERF_MEM_LOAD_UOPS_RETIRED_L2_HIT] = native_read_pmc(1);
	corestate[core_id].pmu_raw[PERF_MEM_LOAD_UOPS_RETIRED_L3_HIT] =	native_read_pmc(2);
	corestate[core_id].pmu_raw[PERF_MEM_LOAD_UOPS_RETIRED_DRAM_HIT] = native_read_pmc(3);
	corestate[core_id].pmu_raw[PERF_XQ_PROMOTION_ALL] = native_read_pmc(4);
	corestate[core_id].pmu_raw[PERF_CPU_CLK_UNHALTED_THREAD] = native_read_pmc(5);
	corestate[core_id].pmu_raw[PERF_INST_RETIRED_ANY_P] = native_read_pmc(6);
	corestate[core_id].pmu_raw[PERF_MEM_LOAD_UOPS_RETIRED_L2_MISS] = native_read_pmc(7);

	return 0;
}

//returns the msr dirty indicator, 0 if not dirty, 1 if dirty
int inline is_msr_dirty(int core_id)
{
	return corestate[core_id].pf_msr_dirty;
}

//sets the msr dirty indicator for the given core
int inline msr_set_dirty(int core_id)
{
	corestate[core_id].pf_msr_dirty = 1;

	return 0;
}


//Loads MSR values into the corestate msr field
// IMPORTANT: This has to be the core with core_id that calls this function or incorrect state will be updated
int msr_load(int core_id)
{
	corestate[core_id].pf_msr[MSR_1320_INDEX].v = __rdmsr(0x1320);
	corestate[core_id].pf_msr[MSR_1321_INDEX].v = __rdmsr(0x1321);
	corestate[core_id].pf_msr[MSR_1322_INDEX].v = __rdmsr(0x1322);
	corestate[core_id].pf_msr[MSR_1323_INDEX].v = __rdmsr(0x1323);
	corestate[core_id].pf_msr[MSR_1324_INDEX].v = __rdmsr(0x1324);
	corestate[core_id].pf_msr[MSR_1325_INDEX].v = __rdmsr(0x1325);
	corestate[core_id].pf_msr[MSR_1326_INDEX].v = __rdmsr(0x1326);
	corestate[core_id].pf_msr[MSR_1327_INDEX].v = __rdmsr(0x1327);
	corestate[core_id].pf_msr[MSR_1A4_INDEX].v = __rdmsr(0x1a4);

	// Sync the shadow to the values actually in hardware so that a later
	// msr_update() with the same value is correctly skipped.
	for (int i = 0; i < NR_OF_MSR; i++) {
		msr_shadow[core_id][i].value = corestate[core_id].pf_msr[i].v;
		msr_shadow[core_id][i].valid = 1;
	}

	return 0;
}

//Update MSR values from corestate msr field if the msr dirty bit has been set for this core
// IMPORTANT: This has to be the core with core_id that calls this function or incorrect state will be updated
int msr_update(int core_id)
{
	corestate[core_id].pf_msr_dirty = 0; //reset msr state

	// Each prefetch MSR is written only if its value changed since the last
	// write
	msr_shadow_write(core_id, MSR_1320_INDEX, 0x1320);
	msr_shadow_write(core_id, MSR_1321_INDEX, 0x1321);
	msr_shadow_write(core_id, MSR_1322_INDEX, 0x1322);
	msr_shadow_write(core_id, MSR_1323_INDEX, 0x1323);
	msr_shadow_write(core_id, MSR_1324_INDEX, 0x1324);
	msr_shadow_write(core_id, MSR_1325_INDEX, 0x1325);
	msr_shadow_write(core_id, MSR_1326_INDEX, 0x1326);
	msr_shadow_write(core_id, MSR_1327_INDEX, 0x1327);
	msr_shadow_write(core_id, MSR_1A4_INDEX, 0x1a4);

	return 0;
}

// Set value in MSR table
int msr_set_l2xq(int core_id, int value)
{
	corestate[core_id].pf_msr[MSR_1320_INDEX].msr1320.L2_STREAM_AMP_XQ_THRESHOLD = value;

	return 0;
}

int msr_get_l2xq(int core_id)
{
	return corestate[core_id].pf_msr[MSR_1320_INDEX].msr1320.L2_STREAM_AMP_XQ_THRESHOLD;
}

// Set value in MSR table
int msr_set_l3xq(int core_id, int value)
{
	corestate[core_id].pf_msr[MSR_1320_INDEX].msr1320.LLC_STREAM_XQ_THRESHOLD = value;

	return 0;
}

int msr_get_l3xq(int core_id)
{
	return corestate[core_id].pf_msr[MSR_1320_INDEX].msr1320.LLC_STREAM_XQ_THRESHOLD;
}
