#ifndef __KERNEL_MAB_H__
#define __KERNEL_MAB_H__

#ifdef SIM
#include "sim.h"
#else
#include <linux/types.h>
#include <linux/atomic.h>

#include "kernel_common.h"
#include "kernel_primitive.h"
#endif

// Compile-time bounds for number of active arms in kernel MAB.
#define MIN_ARMS (2)
#define MAX_ARMS (10)

#define AVAILABLE_ARMS (2)

// Active-arm scoring metric. Change MAB_SCORE_MODE to pick one; it applies to
// both the kernel module and the -DSIM offline sim (both include this header).
#define MAB_SCORE_IPC      (0)  // score: (instructions        >> SHIFT_A) / (cycles >> SHIFT_B)
#define MAB_SCORE_LOADHEAD (1)  // score: ((cycles - loadhead) >> SHIFT_A) / (cycles >> SHIFT_C)

#ifndef MAB_SCORE_MODE
#define MAB_SCORE_MODE MAB_SCORE_LOADHEAD
#endif

// Right-shifts applied before dividing; SHIFT_B sets the score scale.
// Change freely: SHIFT_B=10 -> ~*1000, 12 -> ~*4096, 14 -> ~*16384.
#define SHIFT_A (0)
#define SHIFT_B (10)
#define SHIFT_C (12)

#define SCORE_EVENT_B PERF_CPU_CLK_UNHALTED_THREAD

#if MAB_SCORE_MODE == MAB_SCORE_IPC
#define SCORE_EVENT_A PERF_INST_RETIRED_ANY_P
#elif MAB_SCORE_MODE == MAB_SCORE_LOADHEAD
#define SCORE_EVENT_A PERF_LD_HEAD_ANY_AT_RET
#endif

// Number of consecutive idle intervals before core MAB state is reset.
#define IDLE_THRESHOLD (5)

// Number of compute modules derived from core count and module size.
#define MAX_MODULES (MAX_NUM_CORES / CORES_PER_COMPUTE_MODULE)

// Initialization state values for mab_core.initialized.
#define MAB_INIT_NOT_STARTED (-1)
#define MAB_INIT_IN_PROGRESS (0)
#define MAB_INIT_DONE        (1)

// MAB arm state for reward calculation.
struct mab_arm_s {
	__u32 score;
	__u32 last_score;
};

// One arm maps to one MSR configuration.
struct mab_arm_config_s {
	union msr_u pf_msr[NR_OF_MSR];
};

// Per-core MAB state
struct mab_core_s {
	struct mab_arm_s arms[MAX_ARMS];
	__u64 time_old;  // timestamp of last tick, used to compute time delta
	int active_arm;
	int initialized; // MAB initialization state
	int idle_counter;
} __attribute__((__aligned__(64)));

// Per-module MAB state: one entry per compute module.
// Arm selection and init scanning are driven at module scope by the module leader.
struct mab_module_s {
	int active_arm;
	int initialized;
	int idle_counter;
	int in_transition;
	int enabled_cores;
	atomic_t cores_done __attribute__((aligned(64)));
	__u64 arm_selection_count[MAX_ARMS] __attribute__((aligned(64)));
	__u64 switch_count;
} __attribute__((__aligned__(64)));



// Declare the global MAB state arrays for cores, modules, and arm configurations.
extern struct mab_core_s mab_cores[MAX_NUM_CORES];
extern struct mab_module_s mab_modules[MAX_MODULES];
extern struct mab_arm_config_s mab_arm_configs[MAX_ARMS];

// Returns 1 if core is active, 0 if idle, -1 on error or first call.
// Updates mab_cores[core_id].time_old on each call.
int mab_core_is_active(int core_id);

// Executes one initialization step for a non-initialized core.
// Returns 0 on success, -1 on error.
int mab_init_core_step(int core_id);

// Main MAB algorithm for initialized cores: scoring and arm selection.
// Returns 0 on success, -EINVAL on bad core_id.
int mab_tuning(int core_id);

// Populates mab_arm_configs with the two default MSR configurations.
// arm 0 = boot default, arm 1 = aggressive.
// Must be called once before mab_init_core_step is used.
void mab_setup_default_arms(void);

// Returns the first non-disabled core in a module regardless of idle state.
// Used for module-level idle counter tracking.
// Returns -1 if all cores in the module are disabled.
int mab_first_enabled_core(int module_idx);

// Returns the index of the first active, non-disabled core in a module.
// Returns -1 if all cores in the module are idle or disabled.
int mab_first_active_core(int module_idx);

// Placeholder declaration for module-level init step (implemented in Task C).
int mab_init_module_step(int core_id);

// Dumps per-module MAB statistics (arm selection counts and switch count) to
// dmesg and resets the counters. Called when a module first enters the idle
// state (all 4 cores idle).
void mab_dump_module_stats(int module_idx);

// Applies one arm's MSR configuration to the selected core's pf_msr table and sets dirty.
int mab_apply_msr_config(int core_id, int arm_id);

// Synchronously broadcasts arm MSR configuration to all cores in the module.
void mab_broadcast_module_arm(int module_idx, int new_arm);

#endif // __KERNEL_MAB_H__