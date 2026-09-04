#ifdef SIM
#include "sim.h"
#else
#include <linux/ktime.h>
#include <linux/printk.h>
#endif

#include "kernel_mab.h"
#include "kernel_primitive.h"

#ifdef SIM
// sim has no api_tuning(); aggressiveness is a compile-time constant.
#define aggr (5)
#else
// Runtime aggressiveness parameter set by userspace via api_tuning().
extern int aggr;
#endif

// define persistent kernel MAB state.
// Runtime logic wiring is intentionally deferred to later tasks.
struct mab_core mab_cores[MAX_NUM_CORES];
struct mab_module mab_modules[MAX_MODULES];
struct mab_arm_config mab_arm_configs[MAX_ARMS];

// Returns the first non-disabled core in a module, regardless of idle state.
// Used to assign responsibility for module-level idle tracking.
// Returns -1 if all cores in the module are disabled.
int mab_first_enabled_core(int module_idx)
{
	int i;
	int module_start;

	module_start = sys_first_core + module_idx * CORES_PER_COMPUTE_MODULE;

	for (i = module_start; i < module_start + CORES_PER_COMPUTE_MODULE; i
		++) {
		if (!corestate[i].core_disabled)
			return i;
	}

	return -1;
}

// Returns the index of the first active, non-disabled core in a module.
// Returns -1 if all cores in the module are idle or disabled.
int mab_first_active_core(int module_idx)
{
	int i;
	int module_start;

	module_start = sys_first_core + module_idx * CORES_PER_COMPUTE_MODULE;

	for (i = module_start; i < module_start + CORES_PER_COMPUTE_MODULE; i
		++) {
		if (corestate[i].core_disabled || mab_cores[i].idle_counter >
			0) {
			// pr_info("MAB first_active: skipping core %d
			// (disabled=%d idle=%d)\n",
			// 	 i, corestate[i].core_disabled, mab_cores[i].
			// idle_counter);
			continue;
		}
		return i;
	}

	return -1;
}

// Compute the average score for one arm across all active non-idle cores in a
// module.
// Returns 0 if no active cores contribute — safe for arm comparisons.
static __u32 mab_module_score(int module_idx, int arm_id)
{
	int i;
	int module_start;
	__u64 total_score = 0;
	int cores_count = 0;

	// Determine the core range for this module.
	module_start = sys_first_core + module_idx * CORES_PER_COMPUTE_MODULE;

	// check the aggregate score for this arm across all active non-idle
	// cores in the module.
	for (i = module_start; i < module_start + CORES_PER_COMPUTE_MODULE; i++) {

		if (corestate[i].core_disabled)
			continue;

		if (mab_cores[i].idle_counter > 0)
			continue;

		total_score += mab_cores[i].arms[arm_id].score;
		cores_count++;
	}

	pr_info("MAB module_score mod %d arm %d: total=%llu count=%d avg=%u\n",
		module_idx, arm_id, total_score, cores_count,
		cores_count ? (unsigned)(total_score / cores_count) : 0);

	// If no active cores contribute to the score, return 0 to avoid
	// division by zero.
	if (cores_count == 0) {
		return 0;
	}

	return (__u32)(total_score / cores_count);
}

// Apply one arm's MSR configuration to the selected core.
// Returns 0 on success, -EINVAL if arm_id is out of bounds.
static int mab_apply_msr_config(int core_id, int arm_id)
{
	int i;

	if (arm_id < 0 || arm_id >= MAX_ARMS) {
		pr_err("MAB: invalid arm_id %d in mab_apply_msr_config\n",
		       arm_id);
		return -EINVAL;
	}

	for (i = 0; i < NR_OF_MSR; i++)
		corestate[core_id].pf_msr[i] = mab_arm_configs[arm_id].pf_msr[i];

	msr_set_dirty(core_id);

	return 0;
}

// Compute the active arm score for the given core (higher = better).
// IPC mode:      (instructions        >> SHIFT_A) / (cycles >> SHIFT_B)
// LoadHead mode: ((cycles - load_head) >> SHIFT_A) / (cycles >> SHIFT_B)
// Returns 0 when there is no usable signal (b_scaled 0, or load_head >= cycles).
static __u32 mab_compute_score(int core_id)
{
	__u64 event_a;		// IPC mode: instructions | LoadHead mode: load-head cycles
	__u64 event_b;		// cycles (both modes)
	__u64 b_scaled;		// cycles >> SHIFT_B (denominator)

	event_a = corestate[core_id].pmu_raw[SCORE_EVENT_A] -
		corestate[core_id].pmu_old[SCORE_EVENT_A];
	event_b = corestate[core_id].pmu_raw[SCORE_EVENT_B] -
		corestate[core_id].pmu_old[SCORE_EVENT_B];

	b_scaled = event_b >> SHIFT_B;			// cycles
	if (b_scaled == 0)
		return 0;

	// Kernel_mab.h defines MAB_SCORE_MODE to select the scoring mode at compile time. 
#if MAB_SCORE_MODE == MAB_SCORE_IPC
	{
		__u64 a_scaled = event_a >> SHIFT_A;		// instructions

		pr_info("MAB score core %d [IPC]: instr=%llu cyc=%llu a_sc=%llu b_sc=%llu score=%u\n",
			core_id, event_a, event_b, a_scaled, b_scaled,
			(__u32)(a_scaled / b_scaled));

		return (__u32)(a_scaled / b_scaled);
	}
#elif MAB_SCORE_MODE == MAB_SCORE_LOADHEAD
	{
		__u64 a_b_scaled;

		if (event_a >= event_b)				// stalled on loads the whole interval
			return 0;
		a_b_scaled = (event_b - event_a) >> SHIFT_A;	// cycles not stalled on loads

		pr_info("MAB score core %d [LDHEAD]: ldhead=%llu cyc=%llu a_b_sc=%llu b_sc=%llu score=%u\n",
			core_id, event_a, event_b, a_b_scaled, b_scaled,
			(__u32)(a_b_scaled / b_scaled));

		return (__u32)(a_b_scaled / b_scaled);
	}
#else
#error "MAB_SCORE_MODE must be MAB_SCORE_IPC or MAB_SCORE_LOADHEAD"
#endif
}

// Returns 1 if core is active, 0 if idle, -1 on error or first call.
// Computes time delta from mab_cores[core_id].time_old and updates it.
int mab_core_is_active(int core_id)
{
	__u64 core_cycles;
	__u64 cycles_per_ms;
	__u64 time_now;
	__u64 time_delta_ms;
	time_now = ktime_get_ns();

	// First call: seed the timestamp and skip this tick.
	if (mab_cores[core_id].time_old == 0) {
		mab_cores[core_id].time_old = time_now;
		return -1;
	}

	time_delta_ms = (time_now - mab_cores[core_id].time_old) / 1000000;
	mab_cores[core_id].time_old = time_now;

	if (time_delta_ms == 0)
		return -1;

	core_cycles = corestate[core_id].pmu_raw[PERF_CPU_CLK_UNHALTED_THREAD] -
		corestate[core_id].pmu_old[PERF_CPU_CLK_UNHALTED_THREAD];
	cycles_per_ms = core_cycles / time_delta_ms;

	if (cycles_per_ms < IDLE_CYCLES_THRESHOLD)
		return 0;

	return 1;
}

// to test the mab after initialization
void mab_setup_default_arms(void)
{
	// arm 0: boot default prefetcher config
	mab_arm_configs[0].pf_msr[MSR_1A4_INDEX].v = 0x0000000000000004ULL;
	mab_arm_configs[0].pf_msr[MSR_1320_INDEX].v = 0x10883fea070906c4ULL;
	mab_arm_configs[0].pf_msr[MSR_1321_INDEX].v = 0x0000251134140001ULL;
	mab_arm_configs[0].pf_msr[MSR_1322_INDEX].v = 0x2807ffff4cd0046cULL;
	mab_arm_configs[0].pf_msr[MSR_1323_INDEX].v = 0x0001f9c0c0000000ULL;
	mab_arm_configs[0].pf_msr[MSR_1324_INDEX].v = 0x0600000000000000ULL;
	mab_arm_configs[0].pf_msr[MSR_1327_INDEX].v = 0x0000000005920014ULL;

	// arm 1: aggressive prefetcher config
	mab_arm_configs[1].pf_msr[MSR_1A4_INDEX].v = 0x00ULL;
	mab_arm_configs[1].pf_msr[MSR_1320_INDEX].v = 0x108837ea070906c4ULL;
	mab_arm_configs[1].pf_msr[MSR_1321_INDEX].v = 0x241134140001ULL;
	mab_arm_configs[1].pf_msr[MSR_1322_INDEX].v = 0x2807ffff4cd0046cULL;
	mab_arm_configs[1].pf_msr[MSR_1323_INDEX].v = 0x1f9cc00000000ULL;
	mab_arm_configs[1].pf_msr[MSR_1324_INDEX].v = 0x600000000000000ULL;
	mab_arm_configs[1].pf_msr[MSR_1327_INDEX].v = 0x5920014ULL;

	pr_info("MAB: default arm configs loaded (arm0=boot_default, arm1=aggressive)\n");
}



// Main MAB algorithm for initialized cores.
// All cores update their own per-core arm scores each tick.
// Only the module leader aggregates scores and makes arm selection for the
// whole module.
// Returns 0 on success.
// Main MAB algorithm for initialized cores.
// All cores update their own per-core arm scores each tick.
// Only the module leader aggregates scores and makes arm selection for the
// whole module.
// Returns 0 on success.
int mab_tuning(int core_id)
{
	__u32 score;
	int i;
	int best_arm;
	int module_idx;
	struct mab_module* mod;
	__u32 scores[AVAILABLE_ARMS];
	int passive_arm;

	module_idx = module_id(core_id);
	mod = &mab_modules[module_idx];
	int active_arm = mod->active_arm;

	// Determine passive arm (assuming 2 arms for now)
	// This is only used for logging purposes
	passive_arm = (active_arm == 0) ? 1 : 0;

	// Compute the active arm score for this core and update its per-core state.
	score = mab_compute_score(core_id);
	mab_cores[core_id].arms[active_arm].last_score =
		mab_cores[core_id].arms[active_arm].score;
	mab_cores[core_id].arms[active_arm].score = score;

	// Print in the old detailed style (before applying the boost)
	pr_info("MAB tuning core %d: active=ARM%d score=%u (raw, aggr NOT added) | "
		"passive=ARM%d last=%u + aggr=%u = %u\n",
		core_id,
		active_arm, mab_cores[core_id].arms[active_arm].score,
		passive_arm,
		mab_cores[core_id].arms[passive_arm].score,
		mod->arm_aggressiveness[passive_arm],
		mab_cores[core_id].arms[passive_arm].score +
		mod->arm_aggressiveness[passive_arm]);

	// Add aggressiveness to passive arms (all except active)
	for (i = 0; i < AVAILABLE_ARMS; i++) {
		if (i != active_arm) {
			mab_cores[core_id].arms[i].score += mod->arm_aggressiveness[i];
		}
	}

	// Non-leaders return here since arm selection is at module scope
	if (core_id != mab_first_active_core(module_idx))
		return 0;

	// Calculate aggregate scores and find best arm
	best_arm = active_arm;
	for (i = 0; i < AVAILABLE_ARMS; i++) {
		scores[i] = mab_module_score(module_idx, i);
		if (scores[i] > scores[best_arm])
			best_arm = i;
	}

	// Track how long active arm has been running (for penalty logic)
	mod->arm_consecutive_runs[active_arm]++;

	pr_info("MAB mod %d DECISION: active=%d score=%u best=%d score=%u\n",
		module_idx, active_arm, scores[active_arm],
		best_arm, scores[best_arm]);

	// If passive arm wins
	if (best_arm != active_arm) {
		pr_info("MAB mod %d: challenger ARM%d won, swapping immediately\n",
			module_idx, best_arm);

		// Apply penalty based on how long active arm ran
		if (mod->arm_consecutive_runs[active_arm] == 1) {
			// Ran only 1 interval -> penalize
			__u32 old_aggr = mod->arm_aggressiveness[active_arm];

			int penalty = aggr / AGGR_REDUCTION_FACTOR;
			// if (penalty < 1)
			// 	penalty = 1;

			if (mod->arm_aggressiveness[active_arm] > penalty)
				mod->arm_aggressiveness[active_arm] -= penalty;
			else
				mod->arm_aggressiveness[active_arm] = MIN_AGGRESSIVENESS;

			pr_info("MAB mod %d PENALISED: ARM%d ran 1 interval -> aggr %u -> %u (penalty=%d)\n",
				module_idx, active_arm, old_aggr,
				mod->arm_aggressiveness[active_arm], penalty);
		} else {
			// Ran 2+ intervals -> reset to default
			__u32 old_aggr = mod->arm_aggressiveness[active_arm];
			mod->arm_aggressiveness[active_arm] = aggr;

			pr_info("MAB mod %d RESET: ARM%d ran %u intervals -> aggr %u -> %d\n",
				module_idx, active_arm,
				mod->arm_consecutive_runs[active_arm],
				old_aggr, mod->arm_aggressiveness[active_arm]);
		}

		// Reset all counters (preparing for swap)
		for (i = 0; i < AVAILABLE_ARMS; i++) {
			mod->arm_consecutive_wins[i] = 0;
			mod->arm_consecutive_runs[i] = 0;
		}

		// Perform swap
		mod->active_arm = best_arm;
		mab_apply_msr_config(core_id, best_arm);

		// Track new active arm
		mod->arm_consecutive_wins[best_arm] = 1;

		pr_info("MAB mod %d SWAP: %d -> %d (active=ARM%d aggr=%d, passive=ARM%d aggr=%d)\n",
			module_idx, active_arm, best_arm,
			best_arm, mod->arm_aggressiveness[best_arm],
			active_arm, mod->arm_aggressiveness[active_arm]);

	} else {
		// Active arm keeps winning this interval
		mod->arm_consecutive_wins[active_arm]++;

		// Only trigger RESTORE if aggr is not already default
		if (mod->arm_consecutive_wins[active_arm] >= 2 &&
		    mod->arm_aggressiveness[active_arm] != aggr) {

			__u32 old_aggr = mod->arm_aggressiveness[active_arm];
			mod->arm_aggressiveness[active_arm] = aggr;

			pr_info("MAB mod %d RESTORE: ARM%d won 2x consecutively -> aggr %u -> %d\n",
				module_idx, active_arm, old_aggr,
				mod->arm_aggressiveness[active_arm]);

			// Reset wins counter to prevent repeated checks
			mod->arm_consecutive_wins[active_arm] = 0;
		}

		// Reset win counter for all passive arms
		for (i = 0; i < AVAILABLE_ARMS; i++) {
			if (i != active_arm && mod->arm_consecutive_wins[i] > 0)
				mod->arm_consecutive_wins[i] = 0;
		}

		pr_info("MAB mod %d KEEP: ARM%d active wins\n",
			module_idx, active_arm);
	}

	return 0;
}

// Perform one non-blocking initialization step for a compute module.
// Scans one arm per tick, scores it via module aggregate, selects best when
// all arms are scored.
// Caller must ensure core_id is the first active core in the module (see
// mab_first_active_core).
int mab_init_module_step(int core_id)
{
	int i;
	int best_arm;
	int module_idx;
	struct mab_module* mod;
	__u32 score;

	module_idx = module_id(core_id);
	mod = &mab_modules[module_idx];
	int module_start = sys_first_core + module_idx * CORES_PER_COMPUTE_MODULE;

	// First apply arm 0 to all cores in the module and begin scanning.
	if (mod->initialized == MAB_INIT_NOT_STARTED) {

		mab_apply_msr_config(core_id, 0);

		mod->active_arm = 0;
		mod->initialized = MAB_INIT_IN_PROGRESS;

		return 0;
	}

	// Score the arm that ran last tick: compute fresh PMU scores for each
	// active core in the module, store them in their per-core arms[],
	// then aggregate via mab_module_score.
	for (i = module_start; i < module_start + CORES_PER_COMPUTE_MODULE; i++) {

		if (corestate[i].core_disabled)
			continue;
		if (mab_cores[i].idle_counter > 0)
			continue;

		score = mab_compute_score(i);
		mab_cores[i].arms[mod->active_arm].last_score =
			mab_cores[i].arms[mod->active_arm].score;
		mab_cores[i].arms[mod->active_arm].score = score;
	}

	score = mab_module_score(module_idx, mod->active_arm);
	mab_cores[core_id].arms[mod->active_arm].score = score;

	pr_info("MAB init_step mod %d leader %d state %d active_arm %d score %u\n",
		module_idx, core_id, mod->initialized, mod->active_arm, score);

	// Advance to next arm if more remain to be tested.
	if (mod->active_arm + 1 < AVAILABLE_ARMS) {
		mod->active_arm++;

		mab_apply_msr_config(core_id, mod->active_arm);

		return 0;
	}

	// All arms scored: find the best using scores stored in the module
	// leader's mab_core.
	best_arm = 0;
	for (i = 1; i < AVAILABLE_ARMS; i++) {
		if (mab_cores[core_id].arms[i].score > mab_cores[core_id].arms[best_arm].score)
			best_arm = i;
	}

	pr_info("MAB module %d init done: best arm %d\n", module_idx, best_arm);

	// Initialize all cores' arm scores with module-level scores
	for (i = module_start; i < module_start + CORES_PER_COMPUTE_MODULE; i++) {
		if (corestate[i].core_disabled || mab_cores[i].idle_counter > 0)
			continue;

		int arm;
		for (arm = 0; arm < AVAILABLE_ARMS; arm++) {
			__u32 arm_module_score = mab_module_score(module_idx, arm);
			mab_cores[i].arms[arm].score = arm_module_score;
			mab_cores[i].arms[arm].last_score = arm_module_score;
		}
	}

	// Initialize per-arm aggressiveness and consecutive run counters
	for (i = 0; i < AVAILABLE_ARMS; i++) {
		mod->arm_aggressiveness[i] = aggr;
		mod->arm_consecutive_runs[i] = 0;
		mod->arm_consecutive_wins[i] = 0;
	}

	mod->active_arm = best_arm;
	mab_apply_msr_config(core_id, best_arm);

	mod->initialized = MAB_INIT_DONE;

	return 0;
}
