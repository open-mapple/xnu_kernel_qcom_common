/*
 * SM7150 CPU frequency: request a fixed OSM performance level (boot-arg
 * sweetcpufreq=1: the top level; sweetcpufreqidx=N: LUT row N instead).
 * Without it the cores stay at whatever ABL left; nothing else in this
 * kernel scales CPU clocks.
 *
 * OSM blocks (sdmmagpie.dtsi qcom,clk-cpu-osm-sdmmagpie; Linux clk-cpu-osm.c):
 * L3 0x18321000, silver (6 x A55) 0x18323000, gold (2 x A76) 0x18325800.
 * LUT row i at 0x110 + 32 * i: bits 31:30 source (0 = 300 MHz boot clock),
 * bits 7:0 L-value (rate = 19.2 MHz * L); the table ends at the first repeat.
 * Request: DCVS_PERF_STATE_DESIRED(n) = 0x920 + 4 * n, per core when
 * CORE_DCVS_CTRL (0xbc) bit 0 is set, else core 0 only. OSM firmware does
 * the PLL and CPR voltage changes; the LMh block still throttles on heat.
 * Each domain's cycle counter (0x9c0 + 4 * n) is sampled over 100 ms
 * before and after, so the log shows what the clock really did, and a
 * monitor thread repeats one summary line every 10 s (the one-shot lines
 * scroll away under launchd's console output; also shows throttling).
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
/* Mach MIG declaration is not included by BSD's exported thread.h. */
extern kern_return_t thread_terminate(thread_t);
void sweet_cpufreq_init(void);

#define OSM_ENABLE              0x000u
#define OSM_FREQ_LUT            0x110u
#define OSM_LUT_ROW             32u
#define OSM_LUT_ROWS            40u
#define OSM_CORE_DCVS_CTRL      0x0bcu
#define OSM_PERF_STATE(n)       (0x920u + 4u * (n))
#define OSM_CYCLES(n)           (0x9c0u + 4u * (n))
#define OSM_XO_KHZ              19200u
#define OSM_BOOT_KHZ            300000u

static const struct {
	const char *name;
	uint32_t pa;
	unsigned cores;
} osm_domains[] = {
	{ "l3", 0x18321000u, 1 },
	{ "silver", 0x18323000u, 6 },
	{ "gold", 0x18325800u, 2 },
};

#define OSM_DOMAINS     (sizeof(osm_domains) / sizeof(osm_domains[0]))

/* What init found, for the monitor thread. */
static struct {
	vm_offset_t base;
	unsigned khz[OSM_LUT_ROWS], levels;
} osm_state[3];
static int osm_sleep_chan;

static uint32_t
osm_rd(vm_offset_t base, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static void
osm_wr(vm_offset_t base, uint32_t off, uint32_t val)
{
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)(base + off) = val;
	__asm__ volatile("dsb osh" ::: "memory");
}

static unsigned
osm_khz(uint32_t row)
{
	return (row >> 30) ? OSM_XO_KHZ * (row & 0xffu) : OSM_BOOT_KHZ;
}

/* Cycle counter delta over 100 ms, as MHz (assumes one count per cycle). */
static unsigned
osm_measured_mhz(vm_offset_t base)
{
	uint32_t a = osm_rd(base, OSM_CYCLES(0));
	delay(100000);
	uint32_t b = osm_rd(base, OSM_CYCLES(0));
	return (unsigned)((uint32_t)(b - a) / 100000u);
}

static void
osm_sleep_ms(unsigned ms)
{
	uint64_t deadline;
	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	(void)assert_wait_deadline(&osm_sleep_chan, THREAD_UNINT, deadline);
	thread_block(THREAD_CONTINUE_NULL);
}

/* Every 10 s: requested level and cycle-counter MHz (100 ms window, sleeping) per domain. */
static void
osm_monitor(void *arg, wait_result_t result)
{
	(void)arg;
	(void)result;
	for (;;) {
		osm_sleep_ms(10000);
		char line[160];
		int n = snprintf(line, sizeof(line), "SWEET CPUFREQ:");
		for (unsigned d = 0; d < OSM_DOMAINS && n > 0 && (size_t)n < sizeof(line); ++d) {
			vm_offset_t base = osm_state[d].base;
			if (!base) {
				continue;
			}
			uint32_t st = osm_rd(base, OSM_PERF_STATE(0));
			uint32_t c0 = osm_rd(base, OSM_CYCLES(0));
			uint64_t t0 = mach_absolute_time();
			osm_sleep_ms(100);
			uint32_t c1 = osm_rd(base, OSM_CYCLES(0));
			uint64_t ns;
			absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
			unsigned mhz = ns ? (unsigned)((uint64_t)(uint32_t)(c1 - c0) * 1000u / ns) : 0u;
			n += snprintf(line + n, sizeof(line) - (size_t)n, " %s %u/%u req %u meas %u MHz;",
			    osm_domains[d].name, st, osm_state[d].levels,
			    st < osm_state[d].levels ? osm_state[d].khz[st] / 1000u : 0u, mhz);
		}
		printf("%s\n", line);
	}
}

void
sweet_cpufreq_init(void)
{
	int mode = 0, pick = -1;
	(void)PE_parse_boot_argn("sweetcpufreq", &mode, sizeof(mode));
	(void)PE_parse_boot_argn("sweetcpufreqidx", &pick, sizeof(pick));
	for (unsigned d = 0; d < OSM_DOMAINS; ++d) {
		vm_offset_t base = ml_io_map(osm_domains[d].pa, 0x1000);
		if (!base) {
			printf("SWEET CPUFREQ: %s: mapping failed\n", osm_domains[d].name);
			continue;
		}
		unsigned *khz = osm_state[d].khz, levels = 0;
		for (; levels < OSM_LUT_ROWS; ++levels) {
			khz[levels] = osm_khz(osm_rd(base, OSM_FREQ_LUT + OSM_LUT_ROW * levels));
			if (levels && khz[levels] == khz[levels - 1]) {
				break;
			}
		}
		osm_state[d].base = base;
		osm_state[d].levels = levels;
		uint32_t enable = osm_rd(base, OSM_ENABLE), ctrl = osm_rd(base, OSM_CORE_DCVS_CTRL);
		uint32_t now = osm_rd(base, OSM_PERF_STATE(0));
		bool per_core = ctrl & 1u;
		printf("SWEET CPUFREQ: %s enable=%#x dcvs=%#x levels=%u state=%u (%u MHz) range %u-%u MHz measured %u MHz\n",
		    osm_domains[d].name, enable, ctrl, levels, now,
		    now < levels ? khz[now] / 1000u : 0u, khz[0] / 1000u,
		    levels ? khz[levels - 1] / 1000u : 0u, osm_measured_mhz(base));
		if (mode != 1 || !(enable & 1u) || levels == 0) {
			continue;
		}
		unsigned idx = (pick >= 0 && (unsigned)pick < levels) ? (unsigned)pick : levels - 1u;
		for (unsigned c = 0; c < (per_core ? osm_domains[d].cores : 1u); ++c) {
			osm_wr(base, OSM_PERF_STATE(c), idx);
		}
		delay(1000);
		printf("SWEET CPUFREQ: %s -> level %u (%u MHz)%s, now state=%u measured %u MHz\n",
		    osm_domains[d].name, idx, khz[idx] / 1000u, per_core ? " per-core" : "",
		    osm_rd(base, OSM_PERF_STATE(0)), osm_measured_mhz(base));
	}
	thread_t thread;
	if (kernel_thread_start(osm_monitor, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
