#ifdef SIM
#include "sim.h"
#else
#include <linux/ktime.h>
#include <linux/printk.h>
#endif

#include "kernel_mab.h"
#include "kernel_primitive.h"

// Symmetric rotation policy tunable (score units). No aggressiveness boost, no
// default-arm anchor, and no forced probing: an arm is only switched away from
// when the other arm's stored score leads by ROTATE_HYST or more.
#define ROTATE_HYST   (16)  /* switch arms when the other leads by >= this */

// Define persistent kernel MAB state.
// FIX: use the _s-suffixed struct tags declared in kernel_mab.h.
struct mab_core_s mab_cores[MAX_NUM_CORES];
struct mab_module_s mab_modules[MAX_MODULES];
struct mab_arm_config_s mab_arm_configs[MAX_ARMS];

// Returns the first non-disabled core in a module, regardless of idle state.
// Used to assign responsibility for module-level idle tracking.
// Returns -1 if all cores in the module are disabled.
int mab_first_enabled_core(int module_idx)
{
	int i;
	int module_start;

	module_start = sys_first_core + module_idx * CORES_PER_COMPUTE_MODULE;

	for (i = module_start; i < module_start + CORES_PER_COMPUTE_MODULE; i++) {
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

	for (i = module_start; i < module_start + CORES_PER_COMPUTE_MODULE; i++) {
		if (corestate[i].core_disabled || mab_cores[i].idle_counter > 0) {
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
// FIX: removed `static` to match the non-static declaration in kernel_mab.h.
int mab_apply_msr_config(int core_id, int arm_id)
{
	int i;

	if (arm_id < 0 || arm_id >= MAX_ARMS) {
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

	event_a = corestate[core_id].pmu_raw[SCORE_EVENT_A] -
		corestate[core_id].pmu_old[SCORE_EVENT_A];
	event_b = corestate[core_id].pmu_raw[SCORE_EVENT_B] -
		corestate[core_id].pmu_old[SCORE_EVENT_B];

	// Kernel_mab.h defines MAB_SCORE_MODE to select the scoring mode at compile time.
#if MAB_SCORE_MODE == MAB_SCORE_IPC
	{
		__u64 a_scaled = event_a >> SHIFT_A;		// instructions
		__u64 b_scaled = event_b >> SHIFT_B;	// cycles (denominator)

		if (b_scaled == 0)
			return 0;

		return (__u32)(a_scaled / b_scaled);
	}
#elif MAB_SCORE_MODE == MAB_SCORE_LOADHEAD
	{
		__u64 a_b_scaled;
		__u64 b_scaled = event_b >> SHIFT_C;	// cycles (denominator)

		if (b_scaled == 0)
			return 0;

		if (event_a >= event_b)				// stalled on loads the whole interval
			return 0;
		a_b_scaled = (event_b - event_a);	// cycles not stalled on loads

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

	// arm 1: All enabled prefetcher config
	mab_arm_configs[1].pf_msr[MSR_1A4_INDEX].v = 0x00ULL;
	mab_arm_configs[1].pf_msr[MSR_1320_INDEX].v = 0x108837ea070906c4ULL;
	mab_arm_configs[1].pf_msr[MSR_1321_INDEX].v = 0x241134140001ULL;
	mab_arm_configs[1].pf_msr[MSR_1322_INDEX].v = 0x2807ffff4cd0046cULL;
	mab_arm_configs[1].pf_msr[MSR_1323_INDEX].v = 0x1f9cc00000000ULL;
	mab_arm_configs[1].pf_msr[MSR_1324_INDEX].v = 0x600000000000000ULL;
	mab_arm_configs[1].pf_msr[MSR_1327_INDEX].v = 0x5920014ULL;
}

// Main MAB algorithm for initialized modules.
// Active cores record scores; active and idle cores report completion.
// The last core to finish in the module aggregates scores and may update MSRs.
// Returns 0 on success.
int mab_tuning(int core_id)
{
	int i;
	int module_core;
	int best_arm;
	int module_idx;
	struct mab_module_s *mod;   // FIX: use _s tag
	int active_arm;
	int enabled_cores = 0;
	__u32 score = 0;
	__u32 scores[AVAILABLE_ARMS];

	module_idx = module_id(core_id);
	mod = &mab_modules[module_idx];
	active_arm = mod->active_arm;

	// Idle cores still report completion, but do not contribute a new score.
	if (mab_cores[core_id].idle_counter == 0) {
		score = mab_compute_score(core_id);
		mab_cores[core_id].arms[active_arm].last_score =
			mab_cores[core_id].arms[active_arm].score;
		mab_cores[core_id].arms[active_arm].score = score;
	}

	// Count enabled cores in this module for the completion target.
	// FIX: removed `enabled_cores = mod->enabled_cores;` (now cached in mod
	// after first count; re-count only if not yet set).
	if (mod->enabled_cores == 0) {
		for (module_core = sys_first_core + module_idx * CORES_PER_COMPUTE_MODULE;
		     module_core < sys_first_core +
				(module_idx + 1) * CORES_PER_COMPUTE_MODULE;
		     module_core++) {
			if (!corestate[module_core].core_disabled)
				mod->enabled_cores++;
		}
	}
	enabled_cores = mod->enabled_cores;

	// Atomically announce completion for this core.
	if (atomic_inc_return(&mod->cores_done) < enabled_cores) {
		if (mab_cores[core_id].idle_counter > 0)
			pr_info("Core %d -> idle -> reported\n", core_id);
		else
			pr_info("Core %d -> score %u -> reported\n", core_id, score);
		return 0;
	}

	// Last core arrived: aggregate module scores and select the arm.
	pr_info("Core %d -> all module cores reported -> aggregate\n", core_id);

	for (i = 0; i < AVAILABLE_ARMS; i++)
		scores[i] = mab_module_score(module_idx, i);

	best_arm = active_arm;

	if (scores[1 - active_arm] - scores[active_arm] >= ROTATE_HYST)
		best_arm = 1 - active_arm;

	pr_info("MAB decision [mod %d by core %d]: arm0=%u arm1=%u -> chosen=%d (active=%d)\n",
		module_idx, core_id, scores[0], scores[1], best_arm, active_arm);

	// Count this aggregation round as one selection of the chosen arm.
	if (best_arm >= 0 && best_arm < AVAILABLE_ARMS)
		mod->arm_selection_count[best_arm]++;

	if (best_arm != active_arm) {
		mod->active_arm = best_arm;
		mod->switch_count++;
		mab_apply_msr_config(core_id, best_arm);
		pr_info("MAB switch [mod %d by core %d]: switched ARM %d -> %d, MSRs updated\n",
			module_idx, core_id, active_arm, best_arm);
	}

	// FIX: reset completion counter for next aggregation round.
	atomic_set(&mod->cores_done, 0);

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
	struct mab_module_s *mod;   // FIX: use _s tag
	__u32 score;
	int module_start;

	module_idx = module_id(core_id);
	mod = &mab_modules[module_idx];
	module_start = sys_first_core + module_idx * CORES_PER_COMPUTE_MODULE;

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

	mod->active_arm = best_arm;
	if (best_arm >= 0 && best_arm < AVAILABLE_ARMS)
		mod->arm_selection_count[best_arm]++;
	mab_apply_msr_config(core_id, best_arm);

	mod->initialized = MAB_INIT_DONE;

	return 0;
}


// Dump per-module MAB statistics to dmesg and reset the counters.
// Called when a module first enters the idle state (all 4 cores idle), so the
// per-tick path stays quiet and the statistics reflect one full active period.
void mab_dump_module_stats(int module_idx)
{
	int i;

	if (module_idx < 0 || module_idx >= MAX_MODULES)
		return;


        pr_info("MAB STATS mod %d: switch_count=%llu active_arm=%d arm_selection_count[active_arm]=%llu\n",
		module_idx, mab_modules[module_idx].switch_count, mab_modules[module_idx].active_arm,
		mab_modules[module_idx].arm_selection_count[mab_modules[module_idx].active_arm]);

	for (i = 0; i < AVAILABLE_ARMS; i++) {
		pr_info("MAB STATS mod %d: arm%d selected=%llu times\n",
			module_idx, i, mab_modules[module_idx].arm_selection_count[i]);
	}

	// Reset the per-episode statistics so the next active period is clean.
	for (i = 0; i < AVAILABLE_ARMS; i++)
		mab_modules[module_idx].arm_selection_count[i] = 0;
	mab_modules[module_idx].switch_count = 0;
}