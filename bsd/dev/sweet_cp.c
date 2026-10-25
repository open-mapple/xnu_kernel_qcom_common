/*
 * SM7150 Adreno 618, docs/GPU_A618_PLAN.md stage 4: CP first light.
 *
 * Starts the command processor the way mainline's a6xx hw_init does for the
 * qcom,sm7150 a618 entry (Linux 6.18 a6xx_gpu.c, a6xx_catalog.c,
 * soc/qcom/ubwc_config.c), minus what is not needed to prove the CP runs: no
 * clock gating, no CP_PROTECT, RBBM_INT_0_MASK left at 0, and no GPU writes
 * to DRAM (the ring has NO_UPDATE and every test packet writes a CP register).
 * Register offsets and values come from mainline; the code is ours.
 * sweetcp=N stops after step 4.N:
 *  4.0 reads only; then the SQE (without dword 0), a 16 KiB ring and a
 *      tripwire page go into GPU-GMU-MEM past stage 3's table (IOVA = PA
 *      through bank 0's identity map)
 *  4.1 the hardware init list, each register read back
 *  4.2 SQE and ring registers; the CP is still stopped
 *  4.3 CP_SQE_CNTL = 1 on an empty ring: the GPU's first fetch through the SMMU
 *  4.4 CP_ME_INIT
 *  4.5 PKT4 markers to CP_SCRATCH_REG(7), then 2.5 laps of the ring
 *  4.6 one marker every 10 s for 30 s; the SQE stays running
 * Everything polls. When a step fails the CP state is dumped and, once the SQE
 * runs, it is halted (CP_SQE_CNTL = 0); nothing else is undone.
 * Stage 5 (sweet_zap.c) uses the ring and the scratch (tripwire) page through
 * the sweet_cp_* wrappers at the end.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include "sweet_gpu_regs.h"
#include "sweet_gpu_internal.h"

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);

#define CP_REGS_SIZE            0x40000u        /* the GPU block up to the SMMU at 0x5040000 */

/* a6xx registers missing from sweet_gpu_regs.h; dword offsets from GPU_PA (a6xx.xml) */
#define REG_A6XX_RBBM_VBIF_CLIENT_QOS_CNTL      0x00010u
#define REG_A6XX_RBBM_GBIF_HALT                 0x00016u
#define REG_A6XX_RBBM_GBIF_HALT_ACK             0x00017u
#define REG_A6XX_RBBM_INTERFACE_HANG_INT_CNTL   0x0001fu
#define REG_A6XX_RBBM_INT_0_MASK                0x00038u
#define REG_A6XX_RBBM_INT_0_STATUS              0x00201u
#define REG_A6XX_RBBM_PERFCTR_CNTL              0x00500u
#define REG_A6XX_RBBM_PERFCTR_GPU_BUSY_MASKED   0x0050bu
#define REG_A6XX_RBBM_NC_MODE_CNTL              0x00534u
#define REG_A6XX_CP_RB_BASE                     0x00800u        /* 64-bit */
#define REG_A6XX_CP_RB_CNTL                     0x00802u
#define REG_A6XX_CP_RB_RPTR                     0x00806u
#define REG_A6XX_CP_RB_WPTR                     0x00807u
#define REG_A6XX_CP_SQE_CNTL                    0x00808u
#define REG_A6XX_CP_HW_FAULT                    0x00821u
#define REG_A6XX_CP_INTERRUPT_STATUS            0x00823u
#define REG_A6XX_CP_PROTECT_STATUS              0x00824u
#define REG_A6XX_CP_STATUS_1                    0x00825u
#define REG_A6XX_CP_SQE_INSTR_BASE              0x00830u        /* 64-bit */
#define REG_A6XX_CP_ADDR_MODE_CNTL              0x00842u
#define REG_A6XX_CP_PROTECT_CNTL                0x0084fu
#define REG_A6XX_CP_PROTECT_REG0                0x00850u        /* 32 of them */
#define REG_A6XX_CP_SCRATCH_REG0                0x00883u        /* 8 of them */
#define REG_A6XX_CP_ROQ_THRESHOLDS_1            0x008c1u
#define REG_A6XX_CP_ROQ_THRESHOLDS_2            0x008c2u
#define REG_A6XX_CP_MEM_POOL_SIZE               0x008c3u
#define REG_A6XX_CP_PERFCTR_CP_SEL0             0x008d0u
#define REG_A6XX_CP_IB1_BASE                    0x00928u        /* 64-bit */
#define REG_A6XX_CP_IB1_REM_SIZE                0x0092au
#define REG_A6XX_CP_ROQ_AVAIL_RB                0x00948u
#define REG_A6XX_CP_AHB_CNTL                    0x0098du
#define REG_A6XX_VSC_ADDR_MODE_CNTL             0x00c01u
#define REG_A6XX_UCHE_ADDR_MODE_CNTL            0x00e00u
#define REG_A6XX_UCHE_MODE_CNTL                 0x00e01u
#define REG_A6XX_UCHE_WRITE_RANGE_MAX           0x00e05u        /* 64-bit */
#define REG_A6XX_UCHE_WRITE_THRU_BASE           0x00e07u        /* 64-bit */
#define REG_A6XX_UCHE_TRAP_BASE                 0x00e09u        /* 64-bit */
#define REG_A6XX_UCHE_GMEM_RANGE_MIN            0x00e0bu        /* 64-bit */
#define REG_A6XX_UCHE_GMEM_RANGE_MAX            0x00e0du        /* 64-bit */
#define REG_A6XX_UCHE_CACHE_WAYS                0x00e17u
#define REG_A6XX_UCHE_FILTER_CNTL               0x00e18u
#define REG_A6XX_UCHE_CLIENT_PF                 0x00e19u
#define REG_A6XX_GBIF_HALT                      0x03c45u
#define REG_A6XX_GBIF_HALT_ACK                  0x03c46u
#define REG_A6XX_GRAS_ADDR_MODE_CNTL            0x08601u
#define REG_A6XX_RB_ADDR_MODE_CNTL              0x08e05u
#define REG_A6XX_RB_NC_MODE_CNTL                0x08e08u
#define REG_A6XX_VPC_ADDR_MODE_CNTL             0x09601u
#define REG_A6XX_PC_DBG_ECO_CNTL                0x09e00u
#define REG_A6XX_PC_ADDR_MODE_CNTL              0x09e01u
#define REG_A6XX_VFD_ADDR_MODE_CNTL             0x0a601u
#define REG_A6XX_SP_ADDR_MODE_CNTL              0x0ae01u
#define REG_A6XX_SP_NC_MODE_CNTL                0x0ae02u
#define REG_A6XX_TPL1_ADDR_MODE_CNTL            0x0b601u
#define REG_A6XX_TPL1_NC_MODE_CNTL              0x0b604u
#define REG_A6XX_HLSQ_ADDR_MODE_CNTL            0x0be05u
#define REG_A6XX_RBBM_SECVID_TRUST_CNTL         0x0f400u        /* read only here: secvid-set-once */
#define REG_A6XX_RBBM_SECVID_TSB_TRUSTED_BASE   0x0f800u        /* 64-bit */
#define REG_A6XX_RBBM_SECVID_TSB_TRUSTED_SIZE   0x0f802u
#define REG_A6XX_RBBM_SECVID_TSB_CNTL           0x0f803u
#define REG_A6XX_RBBM_SECVID_TSB_ADDR_MODE_CNTL 0x0f810u

/* Memory past stage 3's table, and the ring */
#define CP_SQE_AREA             0x8000u         /* a630_sqe.fw is 32136 bytes */
#define CP_RING_BYTES           0x4000u
#define CP_RING_DWORDS          (CP_RING_BYTES / 4u)
#define CP_RB_CNTL_VALUE        0x0800020bu     /* BUFSZ 11 (16 KiB / 8), BLKSZ 2 (32 B / 8) << 8, NO_UPDATE */
#define CP_TRIP_BYTES           0x1000u
#define CP_TRIP_WORD            0x5ee7deadu
#define CP_IDMAP_START          0x80000000ull   /* stage 3's identity map */
#define CP_IDMAP_END            0x200000000ull
#define CP_SQE_MIN_VERSION      0x190u          /* a6xx_ucode_check_version for a630_sqe.fw */
#define CP_TIMEOUT_US           1000000u
#define CP_MARKER_REG           (REG_A6XX_CP_SCRATCH_REG0 + 7u)
#define CP_WRAP_BATCHES         80u
#define CP_WRAP_MARKERS         64u
#define PM4_CP_ME_INIT          0x48u
#define RBBM_STATUS_BUSY        (~1u)           /* bit 0 (CX master AHB) is our own read */

/* Mainline's a6xx hw_init list for a618, in its order. must: the CP needs it, a mismatch stops 4.1. */
static const struct {
	const char *name;
	uint32_t reg, val;
	bool must;
} cp_init_list[] = {
	{ "GBIF_HALT", REG_A6XX_GBIF_HALT, 0, true },
	/* Reads 0xdeafbead before and after on G4a (as do 0x17 and 0x534); mainline does not check it either */
	{ "RBBM_GBIF_HALT", REG_A6XX_RBBM_GBIF_HALT, 0, false },
	{ "RBBM_SECVID_TSB_CNTL", REG_A6XX_RBBM_SECVID_TSB_CNTL, 0, true },
	{ "SECVID_TSB_TRUSTED_BASE_LO", REG_A6XX_RBBM_SECVID_TSB_TRUSTED_BASE, 0, true },
	{ "SECVID_TSB_TRUSTED_BASE_HI", REG_A6XX_RBBM_SECVID_TSB_TRUSTED_BASE + 1u, 0, true },
	{ "SECVID_TSB_TRUSTED_SIZE", REG_A6XX_RBBM_SECVID_TSB_TRUSTED_SIZE, 0, true },
	{ "CP_ADDR_MODE_CNTL", REG_A6XX_CP_ADDR_MODE_CNTL, 1, true },
	{ "VSC_ADDR_MODE_CNTL", REG_A6XX_VSC_ADDR_MODE_CNTL, 1, true },
	{ "GRAS_ADDR_MODE_CNTL", REG_A6XX_GRAS_ADDR_MODE_CNTL, 1, true },
	{ "RB_ADDR_MODE_CNTL", REG_A6XX_RB_ADDR_MODE_CNTL, 1, true },
	{ "PC_ADDR_MODE_CNTL", REG_A6XX_PC_ADDR_MODE_CNTL, 1, true },
	{ "HLSQ_ADDR_MODE_CNTL", REG_A6XX_HLSQ_ADDR_MODE_CNTL, 1, true },
	{ "VFD_ADDR_MODE_CNTL", REG_A6XX_VFD_ADDR_MODE_CNTL, 1, true },
	{ "VPC_ADDR_MODE_CNTL", REG_A6XX_VPC_ADDR_MODE_CNTL, 1, true },
	{ "UCHE_ADDR_MODE_CNTL", REG_A6XX_UCHE_ADDR_MODE_CNTL, 1, true },
	{ "SP_ADDR_MODE_CNTL", REG_A6XX_SP_ADDR_MODE_CNTL, 1, true },
	{ "TPL1_ADDR_MODE_CNTL", REG_A6XX_TPL1_ADDR_MODE_CNTL, 1, true },
	{ "SECVID_TSB_ADDR_MODE_CNTL", REG_A6XX_RBBM_SECVID_TSB_ADDR_MODE_CNTL, 1, true },
	/* HWCG (a615_hwcg) skipped: clock gating is power, not function (stage 9) */
	{ "RBBM_VBIF_CLIENT_QOS_CNTL", REG_A6XX_RBBM_VBIF_CLIENT_QOS_CNTL, 0x3u, false },
	{ "RBBM_PERFCTR_GPU_BUSY_MASKED", REG_A6XX_RBBM_PERFCTR_GPU_BUSY_MASKED, 0xffffffffu, false },
	{ "UCHE_WRITE_RANGE_MAX_LO", REG_A6XX_UCHE_WRITE_RANGE_MAX, 0xffffffc0u, false },
	{ "UCHE_WRITE_RANGE_MAX_HI", REG_A6XX_UCHE_WRITE_RANGE_MAX + 1u, 0x0001ffffu, false },
	{ "UCHE_TRAP_BASE_LO", REG_A6XX_UCHE_TRAP_BASE, 0xfffff000u, false },
	{ "UCHE_TRAP_BASE_HI", REG_A6XX_UCHE_TRAP_BASE + 1u, 0x0001ffffu, false },
	{ "UCHE_WRITE_THRU_BASE_LO", REG_A6XX_UCHE_WRITE_THRU_BASE, 0xfffff000u, false },
	{ "UCHE_WRITE_THRU_BASE_HI", REG_A6XX_UCHE_WRITE_THRU_BASE + 1u, 0x0001ffffu, false },
	{ "UCHE_GMEM_RANGE_MIN_LO", REG_A6XX_UCHE_GMEM_RANGE_MIN, 0x00100000u, false },
	{ "UCHE_GMEM_RANGE_MIN_HI", REG_A6XX_UCHE_GMEM_RANGE_MIN + 1u, 0, false },
	{ "UCHE_GMEM_RANGE_MAX_LO", REG_A6XX_UCHE_GMEM_RANGE_MAX, 0x0017ffffu, false },
	{ "UCHE_GMEM_RANGE_MAX_HI", REG_A6XX_UCHE_GMEM_RANGE_MAX + 1u, 0, false },
	{ "UCHE_FILTER_CNTL", REG_A6XX_UCHE_FILTER_CNTL, 0x804u, false },
	{ "UCHE_CACHE_WAYS", REG_A6XX_UCHE_CACHE_WAYS, 0x4u, false },
	{ "CP_ROQ_THRESHOLDS_2", REG_A6XX_CP_ROQ_THRESHOLDS_2, 0x010000c0u, false },
	{ "CP_ROQ_THRESHOLDS_1", REG_A6XX_CP_ROQ_THRESHOLDS_1, 0x8040362cu, false },
	{ "CP_MEM_POOL_SIZE", REG_A6XX_CP_MEM_POOL_SIZE, 128u, false },
	{ "PC_DBG_ECO_CNTL", REG_A6XX_PC_DBG_ECO_CNTL, 0x00180000u, false },
	{ "CP_AHB_CNTL", REG_A6XX_CP_AHB_CNTL, 0x1u, false },
	{ "RBBM_PERFCTR_CNTL", REG_A6XX_RBBM_PERFCTR_CNTL, 0x1u, false },
	{ "CP_PERFCTR_CP_SEL0", REG_A6XX_CP_PERFCTR_CP_SEL0, 0, false },     /* PERF_CP_ALWAYS_COUNT */
	/* UBWC (sm7150: UBWC 2.0, highest bank bit 14): hbb_lo 1, everything else 0 */
	{ "RB_NC_MODE_CNTL", REG_A6XX_RB_NC_MODE_CNTL, 0x2u, false },
	{ "TPL1_NC_MODE_CNTL", REG_A6XX_TPL1_NC_MODE_CNTL, 0x2u, false },
	{ "SP_NC_MODE_CNTL", REG_A6XX_SP_NC_MODE_CNTL, 0x2u, false },
	{ "UCHE_MODE_CNTL", REG_A6XX_UCHE_MODE_CNTL, 0x00200000u, false },
	{ "RBBM_NC_MODE_CNTL", REG_A6XX_RBBM_NC_MODE_CNTL, 0, false },
	{ "RBBM_INTERFACE_HANG_INT_CNTL", REG_A6XX_RBBM_INTERFACE_HANG_INT_CNTL, 0x401fffffu, false },
	{ "UCHE_CLIENT_PF", REG_A6XX_UCHE_CLIENT_PF, 0x81u, false },
	/* GMU busy counters, CP_PROTECT and RBBM_INT_0_MASK skipped (stages 6 and 9) */
};

/* Read in 4.0 besides the list: what 4.2-4.3 write, and what stays untouched */
static const struct {
	const char *name;
	uint32_t reg;
} cp_survey_list[] = {
	{ "RBBM_SECVID_TRUST_CNTL", REG_A6XX_RBBM_SECVID_TRUST_CNTL },
	{ "RBBM_CLOCK_CNTL", REG_A6XX_RBBM_CLOCK_CNTL },
	{ "RBBM_INT_0_MASK", REG_A6XX_RBBM_INT_0_MASK },
	{ "GBIF_HALT_ACK", REG_A6XX_GBIF_HALT_ACK },
	{ "RBBM_GBIF_HALT_ACK", REG_A6XX_RBBM_GBIF_HALT_ACK },
	{ "CP_PROTECT_CNTL", REG_A6XX_CP_PROTECT_CNTL },
	{ "CP_SQE_INSTR_BASE_LO", REG_A6XX_CP_SQE_INSTR_BASE },
	{ "CP_SQE_INSTR_BASE_HI", REG_A6XX_CP_SQE_INSTR_BASE + 1u },
	{ "CP_RB_BASE_LO", REG_A6XX_CP_RB_BASE },
	{ "CP_RB_BASE_HI", REG_A6XX_CP_RB_BASE + 1u },
	{ "CP_RB_CNTL", REG_A6XX_CP_RB_CNTL },
	{ "CP_SQE_CNTL", REG_A6XX_CP_SQE_CNTL },
};

static vm_offset_t cp_gpu, cp_gx_gdscr;
static int cp_cb = -1;
static volatile uint32_t *cp_sqe, *cp_ring, *cp_trip;
static uint64_t cp_sqe_pa, cp_ring_pa, cp_trip_pa;
static uint32_t cp_wptr;                        /* next ring dword; what CP_RB_WPTR gets */
static uint32_t cp_base_hw, cp_base_prot;       /* CP_HW_FAULT and CP_PROTECT_STATUS in 4.0 */
static bool cp_sqe_on;

static inline uint32_t
cp_rd(uint32_t reg)
{
	return gpu_rd(cp_gpu, reg * 4u);
}

static inline void
cp_wr(uint32_t reg, uint32_t val)
{
	gpu_wr(cp_gpu, reg * 4u, val);
}

/* PM4 headers: mainline adreno_gpu.h PM4_PARITY, PKT4 and PKT7 */
static uint32_t
pm4_parity(uint32_t v)
{
	return (0x9669u >> (0xfu & (v ^ (v >> 4) ^ (v >> 8) ^ (v >> 12) ^ (v >> 16) ^ (v >> 20) ^ (v >> 24) ^
	       (v >> 28)))) & 1u;
}

static uint32_t
pkt4(uint32_t reg, uint32_t cnt)
{
	return 0x40000000u | cnt | (pm4_parity(cnt) << 7) | ((reg & 0x3ffffu) << 8) | (pm4_parity(reg) << 27);
}

static uint32_t
pkt7(uint32_t op, uint32_t cnt)
{
	return 0x70000000u | cnt | (pm4_parity(cnt) << 15) | ((op & 0x7fu) << 16) | (pm4_parity(op) << 23);
}

static uint32_t
le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline bool
cp_idle(void)
{
	return (cp_rd(REG_A6XX_RBBM_STATUS) & RBBM_STATUS_BUSY) == 0;
}

/* The CP dump: GPU, CP, SMMU bank, GMU and GX GDSC state */
static void
cp_dump(const char *when)
{
	printf("SWEET CP: %s: RBBM_STATUS 0x%08x STATUS1 0x%08x STATUS2 0x%08x STATUS3 0x%08x INT_0_STATUS 0x%08x\n",
	    when, cp_rd(REG_A6XX_RBBM_STATUS), cp_rd(REG_A6XX_RBBM_STATUS1), cp_rd(REG_A6XX_RBBM_STATUS2),
	    cp_rd(REG_A6XX_RBBM_STATUS3), cp_rd(REG_A6XX_RBBM_INT_0_STATUS));
	printf("SWEET CP:   RPTR %u WPTR %u SQE_CNTL 0x%x HW_FAULT 0x%08x INTERRUPT_STATUS 0x%08x PROTECT_STATUS 0x%08x "
	    "STATUS_1 0x%08x\n", cp_rd(REG_A6XX_CP_RB_RPTR), cp_rd(REG_A6XX_CP_RB_WPTR), cp_rd(REG_A6XX_CP_SQE_CNTL),
	    cp_rd(REG_A6XX_CP_HW_FAULT), cp_rd(REG_A6XX_CP_INTERRUPT_STATUS), cp_rd(REG_A6XX_CP_PROTECT_STATUS),
	    cp_rd(REG_A6XX_CP_STATUS_1));
	printf("SWEET CP:   IB1_BASE 0x%08x%08x IB1_REM_SIZE 0x%x ROQ_AVAIL_RB 0x%08x ALWAYS_ON 0x%08x%08x\n",
	    cp_rd(REG_A6XX_CP_IB1_BASE + 1u), cp_rd(REG_A6XX_CP_IB1_BASE), cp_rd(REG_A6XX_CP_IB1_REM_SIZE),
	    cp_rd(REG_A6XX_CP_ROQ_AVAIL_RB), cp_rd(REG_A6XX_CP_ALWAYS_ON_COUNTER + 1u),
	    cp_rd(REG_A6XX_CP_ALWAYS_ON_COUNTER));
	printf("SWEET CP:   SCRATCH 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x; gx_gdscr 0x%08x\n",
	    cp_rd(REG_A6XX_CP_SCRATCH_REG0), cp_rd(REG_A6XX_CP_SCRATCH_REG0 + 1u), cp_rd(REG_A6XX_CP_SCRATCH_REG0 + 2u),
	    cp_rd(REG_A6XX_CP_SCRATCH_REG0 + 3u), cp_rd(REG_A6XX_CP_SCRATCH_REG0 + 4u),
	    cp_rd(REG_A6XX_CP_SCRATCH_REG0 + 5u), cp_rd(REG_A6XX_CP_SCRATCH_REG0 + 6u),
	    cp_rd(REG_A6XX_CP_SCRATCH_REG0 + 7u), gpu_rd(cp_gx_gdscr, 0));
	(void)sweet_smmu_faults(cp_cb, when);
	sweet_gmu_status(when);
}

/* No SMMU fault on the GPU's bank, and CP_HW_FAULT / CP_PROTECT_STATUS as they were in 4.0 */
static bool
cp_clean(const char *when)
{
	bool ok = sweet_smmu_faults(cp_cb, when) == 0;
	uint32_t hw = cp_rd(REG_A6XX_CP_HW_FAULT), prot = cp_rd(REG_A6XX_CP_PROTECT_STATUS);
	if (hw != cp_base_hw || prot != cp_base_prot) {
		printf("SWEET CP: %s: CP_HW_FAULT 0x%08x (was 0x%08x) CP_PROTECT_STATUS 0x%08x (was 0x%08x)\n", when, hw,
		    cp_base_hw, prot, cp_base_prot);
		ok = false;
	}
	return ok;
}

static void
cp_emit(uint32_t w)
{
	cp_ring[cp_wptr] = w;
	cp_wptr = (cp_wptr + 1u) % CP_RING_DWORDS;
}

/* The ring is write-combined: drain it before the doorbell. */
static void
cp_kick(void)
{
	__asm__ volatile("dsb sy" ::: "memory");
	cp_wr(REG_A6XX_CP_RB_WPTR, cp_wptr);
}

/* Poll every 10 us until RPTR == WPTR and the GPU is idle; microseconds taken, or -1. */
static int
cp_wait_idle(unsigned timeout_us)
{
	for (unsigned t = 0; t <= timeout_us; t += 10u) {
		if (cp_rd(REG_A6XX_CP_RB_RPTR) == cp_wptr && cp_idle()) {
			return (int)t;
		}
		delay(10);
	}
	return -1;
}

/* One PKT4 to CP_SCRATCH_REG(7); true when the CP has run it. */
static bool
cp_marker(uint32_t val, const char *when)
{
	cp_emit(pkt4(CP_MARKER_REG, 1));
	cp_emit(val);
	cp_kick();
	int us = cp_wait_idle(CP_TIMEOUT_US);
	uint32_t got = cp_rd(CP_MARKER_REG);
	if (us < 0 || got != val) {
		printf("SWEET CP: %s: marker 0x%08x: SCRATCH_REG7 0x%08x, RPTR %u WPTR %u, %s\n", when, val, got,
		    cp_rd(REG_A6XX_CP_RB_RPTR), cp_wptr, us < 0 ? "not idle after 1 s" : "idle");
		return false;
	}
	return true;
}

static bool
cp_alloc(uint32_t len, volatile uint32_t **va, uint64_t *pa, const char *name)
{
	vm_offset_t v = sweet_gmu_spare(len, pa);
	if (!v || *pa < CP_IDMAP_START || *pa + len > CP_IDMAP_END) {
		printf("SWEET CP: 4.0: no room for the %s in GPU-GMU-MEM (or outside the identity map)\n", name);
		return false;
	}
	*va = (volatile uint32_t *)v;
	printf("SWEET CP:   %-8s pa/iova %#llx, %#x bytes\n", name, *pa, len);
	return true;
}

/* 4.0: reads only, then the SQE, ring and tripwire page in our own memory */
static bool
cp_survey(const uint8_t *sqe, uint32_t len)
{
	printf("SWEET CP: 4.0 survey (reads only) and memory\n");
	for (unsigned i = 0; i < sizeof(cp_init_list) / sizeof(cp_init_list[0]); ++i) {
		printf("SWEET CP:   %-30s %#07x = 0x%08x (4.1 writes 0x%08x)\n", cp_init_list[i].name, cp_init_list[i].reg,
		    cp_rd(cp_init_list[i].reg), cp_init_list[i].val);
	}
	for (unsigned i = 0; i < sizeof(cp_survey_list) / sizeof(cp_survey_list[0]); ++i) {
		printf("SWEET CP:   %-30s %#07x = 0x%08x\n", cp_survey_list[i].name, cp_survey_list[i].reg,
		    cp_rd(cp_survey_list[i].reg));
	}
	for (unsigned i = 0; i < 32u; i += 8u) {
		printf("SWEET CP:   CP_PROTECT_REG%-2u 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n", i,
		    cp_rd(REG_A6XX_CP_PROTECT_REG0 + i), cp_rd(REG_A6XX_CP_PROTECT_REG0 + i + 1u),
		    cp_rd(REG_A6XX_CP_PROTECT_REG0 + i + 2u), cp_rd(REG_A6XX_CP_PROTECT_REG0 + i + 3u),
		    cp_rd(REG_A6XX_CP_PROTECT_REG0 + i + 4u), cp_rd(REG_A6XX_CP_PROTECT_REG0 + i + 5u),
		    cp_rd(REG_A6XX_CP_PROTECT_REG0 + i + 6u), cp_rd(REG_A6XX_CP_PROTECT_REG0 + i + 7u));
	}
	cp_dump("4.0 as found");
	cp_base_hw = cp_rd(REG_A6XX_CP_HW_FAULT);
	cp_base_prot = cp_rd(REG_A6XX_CP_PROTECT_STATUS);

	if (!sqe || len < 8u || (len & 3u) || len - 4u > CP_SQE_AREA) {
		printf("SWEET CP: 4.0 stopped: GPU-SQE missing or %u bytes (mkboot.sh GPUFW=1)\n", len);
		return false;
	}
	uint32_t w0 = le32(sqe), w1 = le32(sqe + 4), ver = w1 & 0xfffu;
	printf("SWEET CP:   GPU-SQE %u bytes, dword 0 0x%08x, version word 0x%08x: 0x%03x, %s 0x%03x\n", len, w0, w1, ver,
	    ver >= CP_SQE_MIN_VERSION ? "at least" : "BELOW (logged only)", CP_SQE_MIN_VERSION);
	if (w0 != 0) {
		printf("SWEET CP: 4.0 stopped: dword 0 is not 0, not the a630_sqe.fw layout\n");
		return false;
	}
	if (!cp_alloc(CP_SQE_AREA, &cp_sqe, &cp_sqe_pa, "SQE") ||
	    !cp_alloc(CP_RING_BYTES, &cp_ring, &cp_ring_pa, "ring") ||
	    !cp_alloc(CP_TRIP_BYTES, &cp_trip, &cp_trip_pa, "tripwire")) {
		return false;
	}
	uint32_t n = len / 4u - 1u;
	for (uint32_t i = 0; i < CP_SQE_AREA / 4u; ++i) {
		cp_sqe[i] = i < n ? le32(sqe + 4u + i * 4u) : 0;
	}
	for (uint32_t i = 0; i < CP_RING_DWORDS; ++i) {
		cp_ring[i] = 0;
	}
	for (uint32_t i = 0; i < CP_TRIP_BYTES / 4u; ++i) {
		cp_trip[i] = CP_TRIP_WORD;
	}
	__asm__ volatile("dsb sy" ::: "memory");
	unsigned bad = 0;
	for (uint32_t i = 0; i < CP_SQE_AREA / 4u; ++i) {
		bad += cp_sqe[i] != (i < n ? le32(sqe + 4u + i * 4u) : 0);
	}
	for (uint32_t i = 0; i < CP_RING_DWORDS; ++i) {
		bad += cp_ring[i] != 0;
	}
	for (uint32_t i = 0; i < CP_TRIP_BYTES / 4u; ++i) {
		bad += cp_trip[i] != CP_TRIP_WORD;
	}
	printf("SWEET CP:   SQE copied (%u words, first 0x%08x), ring zeroed, tripwire 0x%08x: %u words differ\n", n,
	    cp_sqe[0], CP_TRIP_WORD, bad);
	return bad == 0 && sweet_smmu_faults(cp_cb, "4.0") == 0;
}

/* 4.1: the init list; a "must" row that reads back differently stops it */
static bool
cp_init_regs(void)
{
	printf("SWEET CP: 4.1 hardware init list (mainline a618 values)\n");
	bool ok = true;
	unsigned differ = 0;
	for (unsigned i = 0; i < sizeof(cp_init_list) / sizeof(cp_init_list[0]); ++i) {
		if (i == 0) {
			gpu_pause("4.1 GBIF un-halt, SECVID TSB, 64-bit address modes (GPU block 0x5000000)");
		} else if (cp_init_list[i].reg == REG_A6XX_RBBM_VBIF_CLIENT_QOS_CNTL) {
			gpu_pause("4.1 the rest: QoS, UCHE, CP queues, perf counters, UBWC, hang detect");
		}
		cp_wr(cp_init_list[i].reg, cp_init_list[i].val);
		uint32_t got = cp_rd(cp_init_list[i].reg);
		bool same = got == cp_init_list[i].val;
		printf("SWEET CP:   %-30s %#07x <- 0x%08x: reads 0x%08x%s\n", cp_init_list[i].name, cp_init_list[i].reg,
		    cp_init_list[i].val, got, same ? "" : (cp_init_list[i].must ? " DIFFERS (needed: stop)" : " differs"));
		differ += !same;
		if (!same && cp_init_list[i].must) {
			ok = false;
			break;
		}
	}
	printf("SWEET CP:   GBIF_HALT_ACK 0x%08x RBBM_GBIF_HALT_ACK 0x%08x; %u rows read back differently\n",
	    cp_rd(REG_A6XX_GBIF_HALT_ACK), cp_rd(REG_A6XX_RBBM_GBIF_HALT_ACK), differ);
	cp_dump("after 4.1");
	return cp_clean("after 4.1") && ok;
}

/* 4.2: SQE and ring registers, the CP still stopped */
static bool
cp_ring_regs(void)
{
	printf("SWEET CP: 4.2 SQE and ring registers; the CP stays stopped\n");
	gpu_pause("4.2 CP_SQE_INSTR_BASE, CP_RB_BASE, CP_RB_CNTL, CP_RB_WPTR = 0");
	const struct {
		const char *name;
		uint32_t reg, val;
	} w[] = {
		{ "CP_SQE_INSTR_BASE_LO", REG_A6XX_CP_SQE_INSTR_BASE, (uint32_t)cp_sqe_pa },
		{ "CP_SQE_INSTR_BASE_HI", REG_A6XX_CP_SQE_INSTR_BASE + 1u, (uint32_t)(cp_sqe_pa >> 32) },
		{ "CP_RB_BASE_LO", REG_A6XX_CP_RB_BASE, (uint32_t)cp_ring_pa },
		{ "CP_RB_BASE_HI", REG_A6XX_CP_RB_BASE + 1u, (uint32_t)(cp_ring_pa >> 32) },
		{ "CP_RB_CNTL", REG_A6XX_CP_RB_CNTL, CP_RB_CNTL_VALUE },
		{ "CP_RB_WPTR", REG_A6XX_CP_RB_WPTR, 0 },
	};
	bool ok = true;
	for (unsigned i = 0; i < sizeof(w) / sizeof(w[0]); ++i) {
		cp_wr(w[i].reg, w[i].val);
		uint32_t got = cp_rd(w[i].reg);
		printf("SWEET CP:   %-30s %#07x <- 0x%08x: reads 0x%08x%s\n", w[i].name, w[i].reg, w[i].val, got,
		    got == w[i].val ? "" : " DIFFERS");
		ok = ok && got == w[i].val;
	}
	cp_wptr = 0;
	uint32_t rptr = cp_rd(REG_A6XX_CP_RB_RPTR);
	printf("SWEET CP:   CP_RB_RPTR %u (want 0)\n", rptr);
	cp_dump("after 4.2");
	return cp_clean("after 4.2") && ok && rptr == 0;
}

/* 4.3: start the SQE on an empty ring */
static bool
cp_start(void)
{
	printf("SWEET CP: 4.3 SQE start on an empty ring\n");
	gpu_pause("4.3 CP_SQE_CNTL = 1: the SQE fetches its microcode through SMMU bank 0 (first GPU fetch)");
	cp_wr(REG_A6XX_CP_SQE_CNTL, 1);
	cp_sqe_on = true;
	delay(1000);
	cp_dump("4.3 after 1 ms");
	gpu_sleep_ms(99);
	cp_dump("4.3 after 100 ms");
	gpu_sleep_ms(900);
	cp_dump("4.3 after 1 s");
	uint32_t rptr = cp_rd(REG_A6XX_CP_RB_RPTR);
	bool idle = cp_idle();
	printf("SWEET CP:   RPTR %u (want 0), GPU %s\n", rptr, idle ? "idle" : "BUSY");
	return cp_clean("4.3") && rptr == 0 && idle;
}

/* 4.4: CP_ME_INIT, mainline's a6xx payload */
static bool
cp_me_init(void)
{
	static const uint32_t payload[8] = { 0x2fu, 0x3u, 0x20000000u, 0, 0, 0, 0, 0 };
	printf("SWEET CP: 4.4 CP_ME_INIT: header 0x%08x, then 8 dwords\n", pkt7(PM4_CP_ME_INIT, 8));
	gpu_pause("4.4 CP_ME_INIT in ring dwords 0-8, then CP_RB_WPTR = 9");
	cp_emit(pkt7(PM4_CP_ME_INIT, 8));
	for (unsigned i = 0; i < 8u; ++i) {
		cp_emit(payload[i]);
	}
	cp_kick();
	int us = cp_wait_idle(CP_TIMEOUT_US);
	uint32_t rptr = cp_rd(REG_A6XX_CP_RB_RPTR);
	if (us >= 0) {
		printf("SWEET CP:   ME_INIT done: RPTR %u, idle after %d us\n", rptr, us);
	} else {
		printf("SWEET CP:   ME_INIT NOT done after 1 s: RPTR %u WPTR %u\n", rptr, cp_wptr);
	}
	cp_dump("after 4.4");
	return cp_clean("4.4") && us >= 0 && rptr == 9u;
}

/* 4.5: markers, then 2.5 laps of the ring; ME_INIT's 9 dwords make some packets straddle the end */
static bool
cp_markers(void)
{
	printf("SWEET CP: 4.5 PKT4 markers to CP_SCRATCH_REG(7): header 0x%08x\n", pkt4(CP_MARKER_REG, 1));
	unsigned dirty = 0;
	for (uint32_t i = cp_wptr; i < CP_RING_DWORDS; ++i) {
		dirty += cp_ring[i] != 0;
	}
	printf("SWEET CP:   ring past ME_INIT: %u non-zero words (want 0)\n", dirty);
	gpu_pause("4.5 one marker, then 80 batches of 64 markers (2.5 ring laps)");
	bool ok = dirty == 0 && cp_marker(0x5ee70001u, "4.5 first marker");
	if (ok) {
		printf("SWEET CP:   first marker landed: SCRATCH_REG7 0x%08x RPTR %u\n", cp_rd(CP_MARKER_REG),
		    cp_rd(REG_A6XX_CP_RB_RPTR));
	}
	unsigned laps = 0;
	for (uint32_t b = 0; ok && b < CP_WRAP_BATCHES; ++b) {
		uint32_t last = 0, before = cp_wptr;
		for (uint32_t m = 0; m < CP_WRAP_MARKERS; ++m) {
			last = 0x5ee71000u + b * CP_WRAP_MARKERS + m;
			cp_emit(pkt4(CP_MARKER_REG, 1));
			cp_emit(last);
		}
		laps += cp_wptr < before;
		cp_kick();
		int us = cp_wait_idle(CP_TIMEOUT_US);
		uint32_t got = cp_rd(CP_MARKER_REG);
		if (us < 0 || got != last) {
			printf("SWEET CP:   batch %u: SCRATCH_REG7 0x%08x (want 0x%08x), RPTR %u WPTR %u, %s\n", b, got, last,
			    cp_rd(REG_A6XX_CP_RB_RPTR), cp_wptr, us < 0 ? "not idle after 1 s" : "idle");
			ok = false;
		} else if ((b + 1u) % 16u == 0) {
			printf("SWEET CP:   batch %u ok: WPTR %u, %u wraps, %d us\n", b, cp_wptr, laps, us);
		}
	}
	unsigned trip = 0;
	for (uint32_t i = 0; i < CP_TRIP_BYTES / 4u; ++i) {
		trip += cp_trip[i] != CP_TRIP_WORD;
	}
	printf("SWEET CP:   %u wraps; tripwire page: %u words changed (want 0)\n", laps, trip);
	cp_dump("after 4.5");
	return cp_clean("4.5") && ok && trip == 0;
}

/* 4.6: hold for 30 s, a marker every 10 s */
static bool
cp_hold(void)
{
	printf("SWEET CP: 4.6 hold: a marker every 10 s for 30 s\n");
	bool ok = true;
	for (unsigned s = 10; s <= 30u; s += 10u) {
		gpu_sleep_ms(10000);
		char when[24];
		snprintf(when, sizeof(when), "4.6 hold %u s", s);
		bool m = cp_marker(0x5ee72000u + s / 10u, when);
		cp_dump(when);
		ok = cp_clean(when) && m && ok;
	}
	return ok;
}

static void
cp_halt(void)
{
	if (cp_sqe_on) {
		gpu_pause("halt the SQE (CP_SQE_CNTL = 0)");
		cp_wr(REG_A6XX_CP_SQE_CNTL, 0);
		cp_sqe_on = false;
		printf("SWEET CP:   CP_SQE_CNTL now 0x%x\n", cp_rd(REG_A6XX_CP_SQE_CNTL));
	}
}

bool
sweet_cp_run(int stop, int cb, const uint8_t *sqe, uint32_t sqe_len, vm_offset_t gx_gdscr)
{
	printf("SWEET CP: stage 4 (sweetgpu=6 sweetcp=%d): CP first light on SMMU bank %d\n", stop, cb);
	cp_cb = cb;
	cp_gx_gdscr = gx_gdscr;
	cp_gpu = ml_io_map(GPU_PA, CP_REGS_SIZE);
	if (!cp_gpu) {
		printf("SWEET CP: stage 4 mapping failed\n");
		return false;
	}
	int step = 0;
	bool ok = cp_survey(sqe, sqe_len);
	if (ok && stop >= 1) {
		step = 1;
		ok = cp_init_regs();
	}
	if (ok && stop >= 2) {
		step = 2;
		ok = cp_ring_regs();
	}
	if (ok && stop >= 3) {
		step = 3;
		ok = cp_start();
	}
	if (ok && stop >= 4) {
		step = 4;
		ok = cp_me_init();
	}
	if (ok && stop >= 5) {
		step = 5;
		ok = cp_markers();
	}
	if (ok && stop >= 6) {
		step = 6;
		ok = cp_hold();
	}
	if (!ok) {
		cp_dump("at the failure");
		cp_halt();
	}
	printf("SWEET CP: stage 4 %s at 4.%d (sweetcp=%d); %s\n", ok ? "done" : "STOPPED by a failure", step, stop,
	    cp_sqe_on ? "the SQE stays running" : "the SQE is stopped");
	return ok;
}

/* For stages 5 and 6, after sweet_cp_run passed 4.4 or later */
uint32_t
sweet_cp_rd(uint32_t reg)
{
	return cp_rd(reg);
}

void
sweet_cp_wr(uint32_t reg, uint32_t val)
{
	cp_wr(reg, val);
}

uint32_t
sweet_cp_pkt4(uint32_t reg, uint32_t cnt)
{
	return pkt4(reg, cnt);
}

uint32_t
sweet_cp_pkt7(uint32_t op, uint32_t cnt)
{
	return pkt7(op, cnt);
}

/* Ring dwords free to emit; RPTR from the register, one slot kept empty */
uint32_t
sweet_cp_ring_free(void)
{
	uint32_t rptr = cp_rd(REG_A6XX_CP_RB_RPTR) % CP_RING_DWORDS;
	return (rptr + CP_RING_DWORDS - cp_wptr - 1u) % CP_RING_DWORDS;
}

void
sweet_cp_emit(uint32_t w)
{
	cp_emit(w);
}

void
sweet_cp_kick(void)
{
	cp_kick();
}

int
sweet_cp_wait_idle(unsigned timeout_us)
{
	return cp_wait_idle(timeout_us);
}

bool
sweet_cp_marker(uint32_t val, const char *when)
{
	return cp_marker(val, when);
}

void
sweet_cp_dump(const char *when)
{
	cp_dump(when);
}

bool
sweet_cp_clean(const char *when)
{
	return cp_clean(when);
}

void
sweet_cp_halt(void)
{
	cp_halt();
}

volatile uint32_t *
sweet_cp_scratch(uint64_t *pa)
{
	*pa = cp_trip_pa;
	return cp_trip;
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
