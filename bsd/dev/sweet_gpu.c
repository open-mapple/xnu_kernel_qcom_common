/*
 * SM7150 Adreno 618 bring-up (docs/GPU_A618_PLAN.md), stages 0, 1, 2A, 2B and 3;
 * stages 4 and 5 start from here.
 *
 * Boot-arg sweetgpu=N (N >= 1) starts a thread sweetgpudelay seconds after
 * boot (default 40, so the home screen is up first). Stage 0 (N >= 1) only
 * reads; no GPU, GMU or gpucc register is touched. It
 *  - finds the firmware blobs mkpayload.py --blob put in /chosen/memory-map
 *    (GPU-SQE, GPU-GMU, GPU-ZAP-MDT, GPU-ZAP-B02 = <phys, byte length>),
 *    checks their headers and logs an FNV-1a hash to compare with the one
 *    tools/sunfish/mkboot.sh prints;
 *  - dumps the cmd-db ARC and BCM entries (sweet_rpmh.c) that the GPU rail
 *    votes and the GMU PDC sequence will take their addresses from;
 *  - reads the GPU speed bin from QFPROM (sdmmagpie-gpu.dtsi:
 *    qcom,gpu-speed-bin = <0x41a0 0x1fe00000 21> on qfprom 0x780000;
 *    bin = fmax / 4.8 MHz, 0 selects the default 825 MHz table).
 * Stage 1 (N >= 2) powers the CX side the way the downstream sunfish kernel
 * does before it boots the GMU (gcc-sdmmagpie.c, gpucc-sdmmagpie.c,
 * gdsc-regulator.c with sdmmagpie-gdsc.dtsi, and the kgsl GMU clock list):
 *  - CX GDSC on first: the GMU's vddcx-supply, powered before its clocks
 *    (upstream a6xx_gmu_resume: pm_runtime_get_sync, then the clocks).
 *    gpu_memnoc_gfx and gpu_snoc_dvm_gfx stay at CLK_OFF while it is
 *    collapsed (seen on G1). gpucc is reachable already: gpu_cfg_ahb is
 *    critical and on;
 *  - GCC: GPLL0 and GPLL0-div votes for gpucc, then the ddrss_gpu_axi,
 *    gpu_memnoc_gfx and gpu_snoc_dvm_gfx branches;
 *  - gpucc: GMU RCG to 200 MHz (GPLL0 div / 1.5), then the ahb, cxo,
 *    cx_gmu, cx_snoc_dvm and cx_apb branches;
 *  - reads CX-side GMU registers, the GMU always-on counter, the first ITCM
 *    words and cx_misc.
 * In stage 1 GX stays off: nothing in the 0x5000000 GPU block is touched, and
 * the gx_* clocks, the GX GDSC and the GPU PLLs are only read.
 * Stage 2A (N >= 3) powers GX from the host, without the GMU firmware, at the
 * lowest corner (180 MHz), in the order the downstream drivers would:
 *  - RPMh: mx.lvl, then gfx.lvl, to the first level >= LOW_SVS (vlvl 64),
 *    which gdsc-regulator.c votes on the GX GDSC's parent before enabling
 *    it; 180 MHz needs MIN_SVS (gpucc vdd_gx), and gfx.lvl has no level
 *    below 64. mx goes first and stays >= gfx (rpmhpd/GMU rule);
 *  - PLL0 (Fabia, MX rail, clk-alpha-pll.c): the gpu_cc_pll0_config values
 *    at 360 MHz with the even post-divider at 2, out of reset, RUN, lock;
 *  - gfx3d RCG root forced on (force-enable-root-clk on the GX GDSC,
 *    FORCE_ENABLE_RCG). With GX clamped its root stayed off on G2 (0097);
 *    clk_rcg2_set_force_enable only warns then, and gdsc_enable goes on;
 *  - GX GDSC (gdsc-regulator.c, toggle_logic with sw-reset, domain-addr and
 *    reset-aon-logic): BCR pulse, GMEM reset pulse, unclamp, un-collapse,
 *    poll PWR_ON. On timeout it collapses and clamps again;
 *  - with GX unclamped (on G2b, 0098, the GDSC came on but the root still
 *    read off): the gfx3d root again, then the gx_cxo, gx_gmu, gx_gfx3d and
 *    cx_gfx3d branches;
 *  - the GCC frequency counter (debugcc-sdmmagpie.c, clk-debug.c) through
 *    the gpucc debug mux: cx_gmu (200 MHz, checks the method), gx_cxo,
 *    gx_gmu, gx_gfx3d, cx_gfx3d. It shows which clocks run whatever the
 *    status bits say (gx_gfx3d is BRANCH_HALT_SKIP downstream);
 *  - gfx3d to crc_div (PLL0 even = 180 MHz) divider 1 with an UPDATE even
 *    if the root reads off, as clk_rcg2_set_rate does after its force
 *    enable; then cx_gmu, gx_gfx3d and cx_gfx3d are measured again;
 *  - SPTP head switch through the GMU (a6xx_sptprac_enable);
 *  - only if the gfx3d update took, gx_gfx3d runs near 180 MHz (by the
 *    counter when cx_gmu reads near 200 MHz, else by the root status bit)
 *    and the GMU sees the GX head switch and its clock on: RBBM_STATUS,
 *    RBBM_CLOCK_CNTL and the CP always-on counter, the first GX reads.
 * 2A showed that gx_gfx3d never runs when the host drives GX. Stage 2B
 * (sweetgpu=4, instead of 2A) lets the GMU firmware do it, as kgsl does
 * (sweet_gmu.c, sweet_smmu.c). It needs speed bin 146 (its perf table) and
 * the GPU-GMU and GPU-GMU-MEM blobs. sweetgmu=N (default 0) stops after step
 * 2B.N; sweetgmursc=1 adds mainline's RSCC start handshake before the CM3
 * starts:
 *  2B.0 survey of the GPU SMMU, GMU, RSCC and PDC; nothing written
 *  2B.1 GMU memory (GPU-GMU-MEM, uncached) and an SMMU bank for its streams
 *  2B.2 RSCC and PDC microcode
 *  2B.3 firmware into the TCMs, boot registers, CM3 start
 *  2B.4 GX rail through the GMU (OOB boot_slumber), SPTPRAC, then the GX
 *       GDSC, gfx3d RCG and the clock counter, read only
 *  2B.5 HFI queues and messages
 *  2B.6 OOB_GPU and a DCVS vote, the clock counter again, the first GX reads
 *       if gx_gfx3d runs, then a minute of GMU status every 10 s
 * Stage 3 (sweetgpu=5, after all of 2B, so sweetgmu=6) gives the GPU its own
 * SMMU bank: bank 0 (kgsl's) for stream 0 with a 36-bit input range and one
 * level-1 table in GPU-GMU-MEM past the GMU's part, mapping DRAM
 * 0x80000000..0x1ffffffff 1:1 in 1 GiB blocks. ATOS is off on this SMMU and
 * the GPU sends nothing yet, so the test is the readback and fault-free
 * banks for 30 s; stage 4's first CP fetch is the first walk.
 * Stage 4 (sweetgpu=6, after stage 3) is CP first light in sweet_cp.c:
 * sweetcp=N (0-6) stops after step 4.N, from reads only (0) through the init
 * list, the ring registers, the SQE start, CP_ME_INIT and register-write
 * markers to a 30 s hold with the SQE running (6).
 * Stage 5 (sweetgpu=7, after stage 4 with sweetcp >= 5) is the zap shader in
 * sweet_zap.c: sweetzap=N (0-6) stops after step 5.N, from read-only SCM
 * queries (0) through TrustZone loading the zap (3), CP_SET_SECURE_MODE 0 (4)
 * and the GPU's first writes to DRAM (5) to a 30 s hold (6).
 * Stage 6 (sweetgpu=8, after stage 5 with sweetzap >= 5) is the kernel GPU
 * service in sweet_gsvc.c: sweetsvc=N (0-7) stops after step 6.N, from reads
 * only (0) through CP_PROTECT, fences, IBs and the GPU VA window, the
 * self-test, the hang detector and DCVS to "service ready" and a 60 s hold (7).
 * GMU and cx_misc offsets come from the generated sweet_gpu_regs.h; GCC and
 * gpucc offsets from the downstream drivers above. Everything stays on after
 * the last stage that runs.
 * Before every access that could hang the bus it logs and sleeps
 * SWEET_GPU_PAUSE_MS, so the persistent log (sweetlog=1 ufslogsecs=1) ends
 * at the last good step, and every step logs its result: one stuck core does
 * not stop iOS, so a "next:" line with no result after it marks the hang.
 * sweetgpuhang=1 then spins with interrupts off on purpose, to check that
 * tools/sweet-log-pull.sh recovers such a log.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/sched_prim.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <pexpert/device_tree.h>
#include "sweet_gpu_regs.h"
#include "sweet_gpu_internal.h"

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
extern int sweet_rpmh_cmddb_dump(void);
extern int sweet_rpmh_arc_vote(const char *res, unsigned vlvl, unsigned *hlvl);
void sweet_gpu_init(void);

#define SWEET_GPU_DELAY_S       40u
#define SWEET_GPU_PAUSE_MS      2000u
#define GPU_BLOB_MAX            0x100000u
#define GPU_GMU_TCM_SIZE        0x8000u         /* 16 KiB ITCM code, then 16 KiB DTCM data */
#define GPU_GMU_MEM_SIZE        0x100000u
#define GPU_BIN_2B              146u            /* the perf table sweet_gmu.c sends */
#define QFPROM_PA               0x780000u
#define QFPROM_GPU_SPEED_BIN    0x41a0u
#define GPU_SPEED_BIN_MASK      0x1fe00000u
#define GPU_SPEED_BIN_SHIFT     21

/* Stage 1. GCC (gcc-sdmmagpie.c) */
#define GCC_PA                  0x00100000u
#define GCC_SIZE                0x00080000u
#define GCC_DDRSS_GPU_AXI_CBCR  0x4452cu
#define GCC_APCS_CLOCK_VOTE     0x52004u
#define GCC_VOTE_GPU_GPLL0      (1u << 15)
#define GCC_VOTE_GPU_GPLL0_DIV  (1u << 16)
#define GCC_GPU_CFG_AHB_CBCR    0x71004u
#define GCC_GPU_MEMNOC_GFX_CBCR 0x7100cu
#define GCC_GPU_SNOC_DVM_CBCR   0x71018u
#define GCC_GPU_MISC            0x71028u
/* gpucc (gpucc-sdmmagpie.c, sdmmagpie-gdsc.dtsi) */
#define GPUCC_PA                0x05090000u
#define GPUCC_SIZE              0x9000u
#define GPUCC_PLL0              0x0000u         /* Fabia: +0 MODE, +4 L */
#define GPUCC_PLL1              0x0100u
#define GPUCC_GX_GDSCR          0x100cu
#define GPUCC_CX_GDSCR          0x106cu
#define GPUCC_AHB_CBCR          0x1078u
#define GPUCC_CX_APB_CBCR       0x1088u
#define GPUCC_CX_SNOC_DVM_CBCR  0x108cu
#define GPUCC_CX_GMU_CBCR       0x1098u
#define GPUCC_CXO_CBCR          0x109cu
#define GPUCC_GMU_RCG           0x1120u         /* +0 CMD, +4 CFG */
#define GPUCC_GX_DOMAIN         0x1508u         /* bit 0: GX clamp_io */
#define GPUCC_CX_GDS_HW_CTRL    0x1540u         /* hw-ctrl-addr: CX GDSC status */
/* cx_misc (sdmmagpie-gpu.dtsi); GPU_PA and GMU_PA are in sweet_gpu_internal.h */
#define CX_MISC_PA              0x0509e000u
#define CX_MISC_SIZE            0x1000u

#define CBCR_CLK_OFF            (1u << 31)
#define CBCR_CLK_ENABLE         (1u << 0)
#define CBCR_GMU_WAKE_SLEEP     (0xffu << 4)    /* gpucc probe: cx_gmu wake [11:8], sleep [7:4] = 0xf */
#define GDSC_PWR_ON             (1u << 31)
#define GDSC_CLK_DIS_WAIT_SHIFT 12
#define GDSC_CLK_DIS_WAIT_MASK  (0xfu << GDSC_CLK_DIS_WAIT_SHIFT)
#define GDSC_CX_CLK_DIS_WAIT    8u              /* qcom,clk-dis-wait-val */
#define GDSC_SW_OVERRIDE        (1u << 2)
#define GDSC_HW_CONTROL         (1u << 1)
#define GDSC_SW_COLLAPSE        (1u << 0)
#define GDSC_TIMEOUT_US         500u            /* qcom,gds-timeout */
#define RCG_CMD_UPDATE          (1u << 0)
#define RCG_CFG_MASK            (0x1fu | (7u << 8) | (3u << 12) | (1u << 20))
#define RCG_CFG_GMU_200MHZ      ((6u << 8) | 2u) /* gpll0_out_main_div (300 MHz), divider 1.5 */

/* Stage 2A. gpucc GX side (gpucc-sdmmagpie.c, sdmmagpie-gdsc.dtsi) */
#define GPUCC_GX_BCR            0x1008u         /* GX GDSC sw-reset: BLK_ARES bit 0 */
#define GPUCC_GX_GFX3D_RCG      0x101cu         /* +0 CMD, +4 CFG; hid_width 5 */
#define GPUCC_GX_GFX3D_CBCR     0x1054u         /* BRANCH_HALT_SKIP */
#define GPUCC_GX_CXO_CBCR       0x1060u
#define GPUCC_GX_GMU_CBCR       0x1064u
#define GPUCC_CX_GFX3D_CBCR     0x10a4u         /* BRANCH_HALT_DELAY */
/* Fabia PLL registers from the PLL base (clk-alpha-pll.c) */
#define PLL_MODE                0x00u
#define PLL_L                   0x04u
#define PLL_CAL_L               0x08u
#define PLL_USER_CTL            0x0cu
#define PLL_USER_CTL_U          0x10u
#define PLL_CONFIG_CTL          0x14u
#define PLL_CONFIG_CTL_U        0x18u
#define PLL_TEST_CTL_U          0x20u
#define PLL_OPMODE              0x2cu
#define PLL_FRAC                0x38u
#define PLL_MODE_OUTCTRL        (1u << 0)
#define PLL_MODE_RESET_N        (1u << 2)
#define PLL_MODE_FSM_ENA        (1u << 20)
#define PLL_MODE_UPDATE_BYPASS  (1u << 23)      /* HW_UPDATE_LOGIC_BYPASS */
#define PLL_MODE_LOCK_DET       (1u << 31)
#define PLL_OPMODE_STANDBY      0u
#define PLL_OPMODE_RUN          1u
#define PLL_OUT_ALL             7u              /* main, even, odd */
/* gpu_cc_pll0_config: 19.2 MHz * (0x12 + 0xc000 / 2^16) = 360 MHz */
#define PLL0_L                  0x12u
#define PLL0_FRAC               0xc000u
#define PLL0_CAL_L              0x3fu
#define PLL0_CONFIG_CTL         0x20485699u
#define PLL0_CONFIG_CTL_U       0x00002067u
#define PLL0_USER_CTL           (1u | (1u << 8)) /* main output; even post-divider 2 (table value 1) */
#define PLL0_USER_CTL_U         0x00004805u
#define PLL0_TEST_CTL_U         0x40000000u
#define RCG_CMD_ROOT_EN         (1u << 1)
#define RCG_CMD_ROOT_OFF        (1u << 31)
#define RCG_CFG_GFX3D_180MHZ    ((1u << 8) | 1u) /* crc_div (PLL0 even, 180 MHz), divider 1 */
#define GX_DOMAIN_CLAMP_IO      (1u << 0)
#define GX_DOMAIN_GMEM_RESET    (1u << 4)
#define BCR_BLK_ARES            (1u << 0)
#define SPTPRAC_POWER_ON        0x778000u       /* a6xx_sptprac_enable */
#define SPTPRAC_ON_MASK         0x38u
#define SPTPRAC_ON_WANT         0x28u           /* SPTP GDSC on, SP clock on, GMU up */
#define RPMH_VLVL_LOW_SVS       64u
#define GPU_REGS_SIZE           0x10000u
/* Clock measurement: gpucc debug mux -> GCC debug mux -> counter gated by XO/4 ticks */
#define GCC_DEBUG_POST_DIV      0x62000u        /* [3:0] = divider - 1 */
#define GCC_DEBUG_CBCR          0x62004u
#define GCC_DEBUG_SEL           0x62008u        /* [9:0] */
#define GCC_DEBUG_CTL           0x62024u
#define GCC_DEBUG_STATUS        0x62028u
#define GCC_XO_DIV4_CBCR        0x43008u
#define GCC_DEBUG_SEL_GPUCC     0x144u
#define GCC_DEBUG_DIV           4u
#define GPUCC_DEBUG_SEL         0x1568u         /* [7:0] */
#define GPUCC_DEBUG_POST_DIV    0x10fcu         /* [1:0] = divider - 1 */
#define GPUCC_DEBUG_CBCR        0x1100u
#define GPUCC_DEBUG_DIV         2u
#define MEAS_CNT_EN             (1u << 20)
#define MEAS_DONE               (1u << 25)
#define MEAS_CNT_MASK           0x1ffffffu
#define MEAS_TICKS_SHORT        0x1000u         /* about 1 ms of XO/4 */
#define MEAS_TICKS_FULL         0x10000u        /* about 14 ms */
#define MEAS_XO_DIV4_HZ         4800000u
#define MEAS_FAILED             0xffffffffu

enum { BLOB_SQE, BLOB_GMU, BLOB_ZAP_MDT, BLOB_ZAP_B02, BLOB_GMU_MEM, BLOB_COUNT };

static const char *const gpu_blob_names[BLOB_COUNT] = {
	"GPU-SQE", "GPU-GMU", "GPU-ZAP-MDT", "GPU-ZAP-B02", "GPU-GMU-MEM",
};

static struct {
	uint64_t pa;
	uint32_t len;
	const uint8_t *va;
} gpu_blobs[BLOB_COUNT];

static int gpu_level, gpu_hang, gpu_gmu_stop, gpu_gmu_rsc, gpu_cp_stop, gpu_zap_stop, gpu_svc_stop;
static vm_offset_t gpu_meas_gcc, gpu_meas_cc, gpu_l1_va;       /* for stage 6 */
static uint64_t gpu_l1_pa;
static uint32_t gpu_delay_s = SWEET_GPU_DELAY_S;
static int gpu_sleep_chan;

void
gpu_sleep_ms(unsigned ms)
{
	uint64_t deadline;
	clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
	(void)assert_wait_deadline(&gpu_sleep_chan, THREAD_UNINT, deadline);
	thread_block(THREAD_CONTINUE_NULL);
}

/* Log what comes next, then give the persistent log time to reach the disk. */
void
gpu_pause(const char *next)
{
	printf("SWEET GPU: next: %s (in %u ms)\n", next, SWEET_GPU_PAUSE_MS);
	gpu_sleep_ms(SWEET_GPU_PAUSE_MS);
}

/* Blob bytes are in RAM; read words without assuming alignment. */
static uint32_t
blob32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static uint32_t
fnv1a(const uint8_t *p, uint32_t len)
{
	uint32_t h = 2166136261u;
	for (uint32_t i = 0; i < len; ++i) {
		h = (h ^ p[i]) * 16777619u;
	}
	return h;
}

static unsigned
gpu_find_blobs(void)
{
	DTEntry chosen, mmap;
	void const *prop;
	unsigned int size;
	uint64_t dram_base = 0, dram_size = 0;

	if (SecureDTLookupEntry(NULL, "/chosen", &chosen) == kSuccess &&
	    SecureDTGetProperty(chosen, "dram-base", &prop, &size) == kSuccess && size == 8) {
		dram_base = *(uint64_t const *)prop;
		if (SecureDTGetProperty(chosen, "dram-size", &prop, &size) == kSuccess && size == 8) {
			dram_size = *(uint64_t const *)prop;
		}
	}
	if (SecureDTLookupEntry(NULL, "/chosen/memory-map", &mmap) != kSuccess) {
		printf("SWEET GPU: no /chosen/memory-map\n");
		return 0;
	}
	unsigned found = 0;
	for (unsigned i = 0; i < BLOB_COUNT; ++i) {
		DTMemoryMapRange const *r;
		if (SecureDTGetProperty(mmap, gpu_blob_names[i], (void const **)&r, &size) != kSuccess) {
			printf("SWEET GPU: %s: not in the boot payload (mkboot.sh GPUFW=1)\n", gpu_blob_names[i]);
			continue;
		}
		/* mkpayload places blobs page-aligned inside kernel-managed RAM. */
		if (size != sizeof(*r) || (r->paddr & PAGE_MASK) || r->length == 0 ||
		    r->length > GPU_BLOB_MAX || r->paddr < dram_base ||
		    r->paddr + r->length > dram_base + dram_size) {
			printf("SWEET GPU: %s: bad range size=%u pa=%#lx len=%#lx (dram %#llx+%#llx)\n",
			    gpu_blob_names[i], size, size == sizeof(*r) ? r->paddr : 0,
			    size == sizeof(*r) ? r->length : 0, dram_base, dram_size);
			continue;
		}
		gpu_blobs[i].pa = r->paddr;
		gpu_blobs[i].len = (uint32_t)r->length;
		vm_offset_t va = ml_static_ptovirt(r->paddr);
		const uint8_t *p = (const uint8_t *)va;
		gpu_blobs[i].va = p;
		printf("SWEET GPU: %s pa=%#llx len=%u fnv1a=%08x words %08x %08x %08x %08x\n",
		    gpu_blob_names[i], gpu_blobs[i].pa, gpu_blobs[i].len, fnv1a(p, gpu_blobs[i].len),
		    blob32(p), gpu_blobs[i].len >= 8 ? blob32(p + 4) : 0,
		    gpu_blobs[i].len >= 12 ? blob32(p + 8) : 0, gpu_blobs[i].len >= 16 ? blob32(p + 12) : 0);
		++found;
	}
	return found;
}

/* Header checks only; the firmware is not loaded anywhere yet. */
static void
gpu_check_blobs(void)
{
	const uint8_t *p;
	uint32_t len;

	if ((p = gpu_blobs[BLOB_SQE].va)) {
		len = gpu_blobs[BLOB_SQE].len;
		/* Linux a6xx copies from dword 1 on; dword 0 is not microcode. */
		bool ok = len >= 16 && !(len & 3u) && blob32(p) == 0;
		printf("SWEET GPU: SQE %s: %u dwords, version word %#x\n",
		    ok ? "ok" : "BAD", len / 4u, len >= 8 ? blob32(p + 4) : 0);
	}
	if ((p = gpu_blobs[BLOB_GMU].va)) {
		len = gpu_blobs[BLOB_GMU].len;
		/*
		 * Legacy (a618) image, no header: 16 KiB of CM3 code (vector table
		 * first) for the ITCM, then 16 KiB of DTCM data, which follows the
		 * ITCM in the host window. One 32 KiB write fills both.
		 */
		uint32_t sp = len >= 8 ? blob32(p) : 0, reset = len >= 8 ? blob32(p + 4) : 0;
		bool ok = len == GPU_GMU_TCM_SIZE && (reset & 1u) && reset < len / 2u;
		printf("SWEET GPU: GMU %s: %u bytes, initial sp %#x, reset vector %#x\n",
		    ok ? "ok" : "BAD", len, sp, reset);
	}
	uint32_t load_filesz = 0;
	if ((p = gpu_blobs[BLOB_ZAP_MDT].va)) {
		len = gpu_blobs[BLOB_ZAP_MDT].len;
		uint32_t phoff = len >= 0x34 ? blob32(p + 0x1c) : 0;
		unsigned phnum = len >= 0x34 ? (p[0x2c] | p[0x2d] << 8) : 0;
		bool ok = len >= 0x34 && blob32(p) == 0x464c457fu && p[4] == 1 && phnum && phnum <= 8 &&
		    phoff <= len && phnum * 32u <= len - phoff;
		printf("SWEET GPU: ZAP-MDT %s: ELF32 phoff %#x phnum %u\n", ok ? "ok" : "BAD", phoff, phnum);
		for (unsigned i = 0; ok && i < phnum; ++i) {
			const uint8_t *ph = p + phoff + 32u * i;
			printf("SWEET GPU:   ph%u type %#x off %#x pa %#x filesz %#x memsz %#x flags %#x\n", i,
			    blob32(ph), blob32(ph + 4), blob32(ph + 12), blob32(ph + 16), blob32(ph + 20), blob32(ph + 24));
			if (blob32(ph) == 1) {
				load_filesz = blob32(ph + 16);
			}
		}
	}
	if (gpu_blobs[BLOB_ZAP_B02].va) {
		printf("SWEET GPU: ZAP-B02 %s: %u bytes, MDT LOAD filesz %#x\n",
		    gpu_blobs[BLOB_ZAP_B02].len == load_filesz ? "ok" : "BAD",
		    gpu_blobs[BLOB_ZAP_B02].len, load_filesz);
	}
	if ((p = gpu_blobs[BLOB_GMU_MEM].va)) {
		/* Zeros from mkboot.sh; stage 2B maps it for the GMU. */
		len = gpu_blobs[BLOB_GMU_MEM].len;
		uint32_t nonzero = 0;
		for (uint32_t i = 0; i < len; ++i) {
			nonzero += p[i] != 0;
		}
		printf("SWEET GPU: GMU-MEM %s: %u bytes, %u non-zero\n", len == GPU_GMU_MEM_SIZE && !nonzero ? "ok" : "BAD",
		    len, nonzero);
	}
}

/* The bin, or -1 if QFPROM could not be read. */
static int
gpu_speed_bin(void)
{
	gpu_pause("QFPROM speed-bin read at 0x7841a0");
	vm_offset_t q = ml_io_map(QFPROM_PA + QFPROM_GPU_SPEED_BIN, 4);
	if (!q) {
		printf("SWEET GPU: QFPROM mapping failed\n");
		return -1;
	}
	uint32_t raw = gpu_rd(q, 0);
	unsigned bin = (raw & GPU_SPEED_BIN_MASK) >> GPU_SPEED_BIN_SHIFT;
	printf("SWEET GPU: speed bin raw=%#x bin=%u -> %s fmax %u MHz\n", raw, bin,
	    bin ? "table" : "default table,", bin ? bin * 48u / 10u : 825u);
	return (int)bin;
}

static void
gpu_cmddb(void)
{
	gpu_pause("cmd-db ARC/BCM dump");
	for (unsigned tries = 0; tries < 10; ++tries) {
		int error = sweet_rpmh_cmddb_dump();
		if (error != EBUSY) {
			if (error) {
				printf("SWEET GPU: cmd-db dump failed errno=%d\n", error);
			}
			return;
		}
		gpu_sleep_ms(500);
	}
	printf("SWEET GPU: cmd-db busy (RPMh vote in progress); skipped\n");
}

struct gpu_branch {
	const char *name;
	uint32_t cbcr;
	bool halt;      /* BRANCH_HALT / BRANCH_VOTED: CLK_OFF must clear. Else BRANCH_HALT_DELAY. */
};

static const struct gpu_branch gpu_gcc_branches[] = {
	{ "gcc_ddrss_gpu_axi", GCC_DDRSS_GPU_AXI_CBCR, true },
	{ "gcc_gpu_memnoc_gfx", GCC_GPU_MEMNOC_GFX_CBCR, true },
	{ "gcc_gpu_snoc_dvm_gfx", GCC_GPU_SNOC_DVM_CBCR, true },
};

static const struct gpu_branch gpu_cc_branches[] = {
	{ "gpu_cc_ahb", GPUCC_AHB_CBCR, false },
	{ "gpu_cc_cxo", GPUCC_CXO_CBCR, true },
	{ "gpu_cc_cx_gmu", GPUCC_CX_GMU_CBCR, true },
	{ "gpu_cc_cx_snoc_dvm", GPUCC_CX_SNOC_DVM_CBCR, false },
	{ "gpu_cc_cx_apb", GPUCC_CX_APB_CBCR, false },
};

static bool
gpu_branch_on(vm_offset_t base, const struct gpu_branch *b)
{
	uint32_t before = gpu_rd(base, b->cbcr);
	if (!(before & CBCR_CLK_ENABLE)) {
		gpu_wr(base, b->cbcr, before | CBCR_CLK_ENABLE);
	}
	bool ok = true;
	if (b->halt) {
		ok = gpu_poll(base, b->cbcr, CBCR_CLK_OFF, 0, 200);
	} else {
		delay(10);
	}
	printf("SWEET GPU:   %-20s cbcr 0x%08x -> 0x%08x %s\n", b->name, before, gpu_rd(base, b->cbcr),
	    !ok ? "STILL OFF" : b->halt ? "on" : "enabled (no halt status)");
	return ok;
}

/* gpucc is reachable only while gcc_gpu_cfg_ahb runs (cc == 0: GCC only). */
static void
gpu_log_clocks(vm_offset_t gcc, vm_offset_t cc, const char *when)
{
	printf("SWEET GPU: GCC %s: gpu_cfg_ahb 0x%08x ddrss_gpu_axi 0x%08x memnoc_gfx 0x%08x snoc_dvm_gfx 0x%08x "
	    "vote 0x%08x gpu_misc 0x%08x\n", when, gpu_rd(gcc, GCC_GPU_CFG_AHB_CBCR),
	    gpu_rd(gcc, GCC_DDRSS_GPU_AXI_CBCR), gpu_rd(gcc, GCC_GPU_MEMNOC_GFX_CBCR),
	    gpu_rd(gcc, GCC_GPU_SNOC_DVM_CBCR), gpu_rd(gcc, GCC_APCS_CLOCK_VOTE), gpu_rd(gcc, GCC_GPU_MISC));
	if (!cc) {
		return;
	}
	printf("SWEET GPU: gpucc %s: cx_gdscr 0x%08x hw_ctrl 0x%08x gx_gdscr 0x%08x gx_domain 0x%08x "
	    "gmu_rcg 0x%08x/0x%08x\n", when, gpu_rd(cc, GPUCC_CX_GDSCR), gpu_rd(cc, GPUCC_CX_GDS_HW_CTRL),
	    gpu_rd(cc, GPUCC_GX_GDSCR), gpu_rd(cc, GPUCC_GX_DOMAIN), gpu_rd(cc, GPUCC_GMU_RCG),
	    gpu_rd(cc, GPUCC_GMU_RCG + 4u));
	printf("SWEET GPU: gpucc %s: ahb 0x%08x cxo 0x%08x cx_gmu 0x%08x cx_snoc_dvm 0x%08x cx_apb 0x%08x "
	    "pll0 0x%08x L %#x pll1 0x%08x L %#x\n", when, gpu_rd(cc, GPUCC_AHB_CBCR), gpu_rd(cc, GPUCC_CXO_CBCR),
	    gpu_rd(cc, GPUCC_CX_GMU_CBCR), gpu_rd(cc, GPUCC_CX_SNOC_DVM_CBCR), gpu_rd(cc, GPUCC_CX_APB_CBCR),
	    gpu_rd(cc, GPUCC_PLL0), gpu_rd(cc, GPUCC_PLL0 + 4u), gpu_rd(cc, GPUCC_PLL1), gpu_rd(cc, GPUCC_PLL1 + 4u));
}

static bool
gpu_gcc_on(vm_offset_t gcc)
{
	uint32_t vote = gpu_rd(gcc, GCC_APCS_CLOCK_VOTE);
	gpu_wr(gcc, GCC_APCS_CLOCK_VOTE, vote | GCC_VOTE_GPU_GPLL0 | GCC_VOTE_GPU_GPLL0_DIV);
	delay(10);      /* BRANCH_HALT_DELAY */
	printf("SWEET GPU:   gcc vote 0x%08x -> 0x%08x (gpu_gpll0, gpu_gpll0_div)\n", vote,
	    gpu_rd(gcc, GCC_APCS_CLOCK_VOTE));
	bool ok = true;
	for (unsigned i = 0; i < sizeof(gpu_gcc_branches) / sizeof(gpu_gcc_branches[0]); ++i) {
		ok = gpu_branch_on(gcc, &gpu_gcc_branches[i]) && ok;
	}
	return ok;
}

/* gdsc-regulator.c: probe (software sequencing, clk-dis-wait) then enable. */
static bool
gpu_cx_gdsc_on(vm_offset_t cc)
{
	uint32_t v = gpu_rd(cc, GPUCC_CX_GDSCR), hw = gpu_rd(cc, GPUCC_CX_GDS_HW_CTRL);
	if ((hw & GDSC_PWR_ON) && !(v & GDSC_SW_COLLAPSE)) {
		printf("SWEET GPU:   CX GDSC already on: gdscr 0x%08x hw_ctrl 0x%08x\n", v, hw);
		return true;
	}
	uint32_t cfg = (v & ~(GDSC_HW_CONTROL | GDSC_SW_OVERRIDE | GDSC_CLK_DIS_WAIT_MASK)) |
	    (GDSC_CX_CLK_DIS_WAIT << GDSC_CLK_DIS_WAIT_SHIFT);
	gpu_wr(cc, GPUCC_CX_GDSCR, cfg);
	gpu_wr(cc, GPUCC_CX_GDSCR, cfg & ~GDSC_SW_COLLAPSE);
	delay(1);       /* 8 XO cycles before the first status read */
	bool ok = gpu_poll(cc, GPUCC_CX_GDS_HW_CTRL, GDSC_PWR_ON, GDSC_PWR_ON, 2u * GDSC_TIMEOUT_US);
	printf("SWEET GPU:   CX GDSC gdscr 0x%08x -> 0x%08x, hw_ctrl 0x%08x -> 0x%08x: %s\n", v,
	    gpu_rd(cc, GPUCC_CX_GDSCR), hw, gpu_rd(cc, GPUCC_CX_GDS_HW_CTRL), ok ? "on" : "TIMEOUT");
	return ok;
}

static bool
gpu_gmu_rcg_200mhz(vm_offset_t cc)
{
	uint32_t cmd = gpu_rd(cc, GPUCC_GMU_RCG), cfg = gpu_rd(cc, GPUCC_GMU_RCG + 4u);
	if ((cfg & RCG_CFG_MASK) == RCG_CFG_GMU_200MHZ) {
		printf("SWEET GPU:   GMU RCG already 200 MHz: cmd 0x%08x cfg 0x%08x\n", cmd, cfg);
		return true;
	}
	gpu_wr(cc, GPUCC_GMU_RCG + 4u, (cfg & ~RCG_CFG_MASK) | RCG_CFG_GMU_200MHZ);
	gpu_wr(cc, GPUCC_GMU_RCG, cmd | RCG_CMD_UPDATE);
	bool ok = gpu_poll(cc, GPUCC_GMU_RCG, RCG_CMD_UPDATE, 0, 500);
	printf("SWEET GPU:   GMU RCG cmd 0x%08x -> 0x%08x, cfg 0x%08x -> 0x%08x: %s\n", cmd,
	    gpu_rd(cc, GPUCC_GMU_RCG), cfg, gpu_rd(cc, GPUCC_GMU_RCG + 4u), ok ? "200 MHz" : "UPDATE TIMEOUT");
	return ok;
}

static bool
gpu_cc_on(vm_offset_t cc)
{
	gpu_wr(cc, GPUCC_CX_GMU_CBCR, gpu_rd(cc, GPUCC_CX_GMU_CBCR) | CBCR_GMU_WAKE_SLEEP);
	bool ok = true;
	for (unsigned i = 0; i < sizeof(gpu_cc_branches) / sizeof(gpu_cc_branches[0]); ++i) {
		ok = gpu_branch_on(cc, &gpu_cc_branches[i]) && ok;
	}
	return ok;
}

static uint64_t
gmu_alwayson(vm_offset_t gmu)
{
	uint32_t hi = gmu_rd(gmu, REG_A6XX_GMU_ALWAYS_ON_COUNTER_H), lo, hi2;
	for (;;) {
		lo = gmu_rd(gmu, REG_A6XX_GMU_ALWAYS_ON_COUNTER_L);
		hi2 = gmu_rd(gmu, REG_A6XX_GMU_ALWAYS_ON_COUNTER_H);
		if (hi2 == hi) {
			return (uint64_t)hi << 32 | lo;
		}
		hi = hi2;
	}
}

static void
gpu_gmu_reads(vm_offset_t gmu, vm_offset_t cc)
{
	static const struct {
		const char *name;
		uint32_t reg;
	} regs[] = {
		{ "CM3_SYSRESET", REG_A6XX_GMU_CM3_SYSRESET },
		{ "CM3_BOOT_CONFIG", REG_A6XX_GMU_CM3_BOOT_CONFIG },
		{ "CM3_FW_BUSY", REG_A6XX_GMU_CM3_FW_BUSY },
		{ "CM3_FW_INIT_RESULT", REG_A6XX_GMU_CM3_FW_INIT_RESULT },
		{ "HFI_CTRL_STATUS", REG_A6XX_GMU_HFI_CTRL_STATUS },
		{ "HFI_VERSION_INFO", REG_A6XX_GMU_HFI_VERSION_INFO },
		{ "SPTPRAC_PWR_CLK_STATUS", REG_A6XX_GMU_SPTPRAC_PWR_CLK_STATUS },
		{ "CX_GMU_RPMH_POWER_STATE", REG_A6XX_GPU_GMU_CX_GMU_RPMH_POWER_STATE },
		{ "AO_GPU_CX_BUSY_STATUS", REG_A6XX_GPU_GMU_AO_GPU_CX_BUSY_STATUS },
		{ "AO_AHB_FENCE_CTRL", REG_A6XX_GMU_AO_AHB_FENCE_CTRL },
		{ "AHB_FENCE_STATUS", REG_A6XX_GMU_AHB_FENCE_STATUS },
	};
	for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
		printf("SWEET GPU:   GMU %-24s [0x%05x] = 0x%08x\n", regs[i].name, regs[i].reg, gmu_rd(gmu, regs[i].reg));
	}
	uint32_t sp = gmu_rd(gmu, REG_A6XX_GMU_SPTPRAC_PWR_CLK_STATUS);
	printf("SWEET GPU:   GX side: HM GDSC %s, HM clock %s, gx_gdscr 0x%08x\n",
	    (sp & A6XX_GMU_SPTPRAC_PWR_CLK_STATUS_GX_HM_GDSC_POWER_OFF__MASK) ? "off" : "ON",
	    (sp & A6XX_GMU_SPTPRAC_PWR_CLK_STATUS_GX_HM_CLK_OFF__MASK) ? "off" : "ON", gpu_rd(cc, GPUCC_GX_GDSCR));
	/* XO-clocked: about 19200 ticks per millisecond if it runs. */
	uint64_t t0 = gmu_alwayson(gmu);
	delay(1000);
	uint64_t t1 = gmu_alwayson(gmu);
	printf("SWEET GPU:   GMU always-on counter %llu -> %llu (+%llu in 1 ms)\n", t0, t1, t1 - t0);
}

/* True with *gccp, *ccp and *gmup set when the CX side is up for stage 2A. */
static bool
gpu_stage1(vm_offset_t *gccp, vm_offset_t *ccp, vm_offset_t *gmup)
{
	printf("SWEET GPU: stage 1 (sweetgpu=%d): CX clocks and GDSC, then CX-side reads; GX untouched\n", gpu_level);
	vm_offset_t gcc = ml_io_map(GCC_PA, GCC_SIZE), cc = ml_io_map(GPUCC_PA, GPUCC_SIZE);
	vm_offset_t gmu = ml_io_map(GMU_PA, GMU_SIZE), cxm = ml_io_map(CX_MISC_PA, CX_MISC_SIZE);
	if (!gcc || !cc || !gmu || !cxm) {
		printf("SWEET GPU: stage 1 mapping failed\n");
		return false;
	}
	bool ahb = !(gpu_rd(gcc, GCC_GPU_CFG_AHB_CBCR) & CBCR_CLK_OFF);
	gpu_log_clocks(gcc, ahb ? cc : 0, "before");
	if (!ahb) {
		static const struct gpu_branch cfg_ahb = { "gcc_gpu_cfg_ahb", GCC_GPU_CFG_AHB_CBCR, true };
		gpu_pause("gcc_gpu_cfg_ahb on (it is off, so gpucc is unreachable)");
		if (!gpu_branch_on(gcc, &cfg_ahb)) {
			printf("SWEET GPU: stage 1 stopped: gcc_gpu_cfg_ahb\n");
			return false;
		}
		gpu_log_clocks(gcc, cc, "with gpu_cfg_ahb");
	}
	gpu_pause("CX GDSC on (gpucc 0x509106c)");
	if (!gpu_cx_gdsc_on(cc)) {
		printf("SWEET GPU: stage 1 stopped: CX GDSC\n");
		return false;
	}
	gpu_log_clocks(gcc, cc, "with CX GDSC");
	gpu_pause("GCC GPU branches and GPLL0 votes");
	if (!gpu_gcc_on(gcc)) {
		printf("SWEET GPU: stage 1 stopped: GCC branches\n");
		return false;
	}
	gpu_pause("GMU RCG to 200 MHz (gpucc 0x5091120)");
	if (!gpu_gmu_rcg_200mhz(cc)) {
		printf("SWEET GPU: stage 1 stopped: GMU RCG\n");
		return false;
	}
	gpu_pause("gpucc CX branches: ahb, cxo, cx_gmu, cx_snoc_dvm, cx_apb");
	bool cx = gpu_cc_on(cc);
	gpu_log_clocks(gcc, cc, "after");
	if (!cx) {
		printf("SWEET GPU: stage 1 stopped: gpucc branches\n");
		return false;
	}
	gpu_pause("GMU register reads (CX side of 0x506a000)");
	gpu_gmu_reads(gmu, cc);
	gpu_pause("GMU ITCM read (0x506d000)");
	printf("SWEET GPU:   ITCM %08x %08x %08x %08x\n", gmu_rd(gmu, REG_A6XX_GMU_CM3_ITCM_START),
	    gmu_rd(gmu, REG_A6XX_GMU_CM3_ITCM_START + 1u), gmu_rd(gmu, REG_A6XX_GMU_CM3_ITCM_START + 2u),
	    gmu_rd(gmu, REG_A6XX_GMU_CM3_ITCM_START + 3u));
	gpu_pause("cx_misc read (0x509e000)");
	printf("SWEET GPU:   cx_misc SYSTEM_CACHE_CNTL_0 0x%08x _1 0x%08x\n",
	    gpu_rd(cxm, REG_A6XX_SYSTEM_CACHE_CNTL_0 * 4u), gpu_rd(cxm, REG_A6XX_SYSTEM_CACHE_CNTL_1 * 4u));
	printf("SWEET GPU: stage 1 done; CX clocks and GDSC stay on\n");
	*gccp = gcc;
	*ccp = cc;
	*gmup = gmu;
	return true;
}

/* The cmd-db dump or a touch-rail vote may hold the RPMh lock briefly. */
static bool
gpu_rail_vote(const char *res, unsigned vlvl)
{
	for (unsigned tries = 0; tries < 10; ++tries) {
		unsigned hlvl = 0;
		int error = sweet_rpmh_arc_vote(res, vlvl, &hlvl);
		if (error != EBUSY) {
			printf("SWEET GPU:   %s vlvl >= %u: %s (hlvl %u, errno %d)\n", res, vlvl,
			    error ? "FAILED" : "voted", hlvl, error);
			return !error;
		}
		gpu_sleep_ms(500);
	}
	printf("SWEET GPU:   %s: RPMh busy; not voted\n", res);
	return false;
}

/*
 * The GX GDSC, its BCR and domain register, the gfx3d RCG and PLL0 are on the
 * CX side (Linux touches them with GX off). The gx_* branch registers are read
 * only with the GX GDSC on (gx_on).
 */
static void
gpu_log_gx(vm_offset_t cc, vm_offset_t gmu, const char *when, bool gx_on)
{
	printf("SWEET GPU: GX %s: gx_gdscr 0x%08x gx_domain 0x%08x gx_bcr 0x%08x gfx3d_rcg 0x%08x/0x%08x "
	    "sptprac 0x%08x\n", when, gpu_rd(cc, GPUCC_GX_GDSCR), gpu_rd(cc, GPUCC_GX_DOMAIN),
	    gpu_rd(cc, GPUCC_GX_BCR), gpu_rd(cc, GPUCC_GX_GFX3D_RCG), gpu_rd(cc, GPUCC_GX_GFX3D_RCG + 4u),
	    gmu_rd(gmu, REG_A6XX_GMU_SPTPRAC_PWR_CLK_STATUS));
	printf("SWEET GPU: GX %s: pll0 mode 0x%08x L %#x cal_l %#x frac %#x user 0x%08x/0x%08x "
	    "config 0x%08x/0x%08x opmode %#x\n", when, gpu_rd(cc, GPUCC_PLL0 + PLL_MODE),
	    gpu_rd(cc, GPUCC_PLL0 + PLL_L), gpu_rd(cc, GPUCC_PLL0 + PLL_CAL_L), gpu_rd(cc, GPUCC_PLL0 + PLL_FRAC),
	    gpu_rd(cc, GPUCC_PLL0 + PLL_USER_CTL), gpu_rd(cc, GPUCC_PLL0 + PLL_USER_CTL_U),
	    gpu_rd(cc, GPUCC_PLL0 + PLL_CONFIG_CTL), gpu_rd(cc, GPUCC_PLL0 + PLL_CONFIG_CTL_U),
	    gpu_rd(cc, GPUCC_PLL0 + PLL_OPMODE));
	if (!gx_on) {
		return;
	}
	printf("SWEET GPU: GX %s: gx_cxo 0x%08x gx_gmu 0x%08x gx_gfx3d 0x%08x cx_gfx3d 0x%08x\n", when,
	    gpu_rd(cc, GPUCC_GX_CXO_CBCR), gpu_rd(cc, GPUCC_GX_GMU_CBCR), gpu_rd(cc, GPUCC_GX_GFX3D_CBCR),
	    gpu_rd(cc, GPUCC_CX_GFX3D_CBCR));
}

/* clk_fabia_pll_configure, then clk_fabia_pll_enable; refuses a PLL that something else runs. */
static bool
gpu_pll0_360mhz(vm_offset_t cc)
{
	const vm_offset_t p = cc + GPUCC_PLL0;
	uint32_t mode = gpu_rd(p, PLL_MODE), opmode = gpu_rd(p, PLL_OPMODE);
	if ((mode & (PLL_MODE_OUTCTRL | PLL_MODE_FSM_ENA)) || opmode == PLL_OPMODE_RUN) {
		printf("SWEET GPU:   PLL0 is in use (mode 0x%08x opmode %#x); not reprogrammed\n", mode, opmode);
		return false;
	}
	gpu_wr(p, PLL_L, PLL0_L);
	gpu_wr(p, PLL_CAL_L, PLL0_CAL_L);
	gpu_wr(p, PLL_FRAC, PLL0_FRAC);
	gpu_wr(p, PLL_CONFIG_CTL, PLL0_CONFIG_CTL);
	gpu_wr(p, PLL_CONFIG_CTL_U, PLL0_CONFIG_CTL_U);
	gpu_wr(p, PLL_USER_CTL, PLL0_USER_CTL);
	gpu_wr(p, PLL_USER_CTL_U, PLL0_USER_CTL_U);
	gpu_wr(p, PLL_TEST_CTL_U, PLL0_TEST_CTL_U);
	gpu_wr(p, PLL_MODE, gpu_rd(p, PLL_MODE) | PLL_MODE_UPDATE_BYPASS);
	gpu_wr(p, PLL_OPMODE, PLL_OPMODE_STANDBY);
	/* Calibration runs as the PLL leaves reset (CAL_L sets its frequency). */
	gpu_wr(p, PLL_MODE, gpu_rd(p, PLL_MODE) | PLL_MODE_RESET_N);
	gpu_wr(p, PLL_OPMODE, PLL_OPMODE_RUN);
	bool ok = gpu_poll(p, PLL_MODE, PLL_MODE_LOCK_DET, PLL_MODE_LOCK_DET, 200);
	if (ok) {
		gpu_wr(p, PLL_USER_CTL, gpu_rd(p, PLL_USER_CTL) | PLL_OUT_ALL);
		gpu_wr(p, PLL_MODE, gpu_rd(p, PLL_MODE) | PLL_MODE_OUTCTRL);
	} else {
		gpu_wr(p, PLL_OPMODE, PLL_OPMODE_STANDBY);
	}
	printf("SWEET GPU:   PLL0 mode 0x%08x -> 0x%08x opmode %#x user 0x%08x: %s\n", mode, gpu_rd(p, PLL_MODE),
	    gpu_rd(p, PLL_OPMODE), gpu_rd(p, PLL_USER_CTL), ok ? "locked at 360 MHz, outputs on" : "NO LOCK, back to standby");
	return ok;
}

/* clk_rcg2_set_force_enable: a root that stays off is a warning there, not an error. */
static bool
gpu_gfx3d_root_on(vm_offset_t cc, const char *when)
{
	uint32_t cmd = gpu_rd(cc, GPUCC_GX_GFX3D_RCG);
	gpu_wr(cc, GPUCC_GX_GFX3D_RCG, cmd | RCG_CMD_ROOT_EN);
	bool on = gpu_poll(cc, GPUCC_GX_GFX3D_RCG, RCG_CMD_ROOT_OFF, 0, 500);
	printf("SWEET GPU:   gfx3d RCG root %s: cmd 0x%08x -> 0x%08x, cfg 0x%08x: %s\n", when, cmd,
	    gpu_rd(cc, GPUCC_GX_GFX3D_RCG), gpu_rd(cc, GPUCC_GX_GFX3D_RCG + 4u), on ? "on" : "still off");
	return on;
}

/*
 * __clk_rcg2_set_rate: CFG, then UPDATE. Downstream does this inside the
 * force enable whether or not the root came on; the root stays forced on.
 */
static bool
gpu_gfx3d_rcg_180mhz(vm_offset_t cc)
{
	uint32_t cmd = gpu_rd(cc, GPUCC_GX_GFX3D_RCG), cfg = gpu_rd(cc, GPUCC_GX_GFX3D_RCG + 4u);
	gpu_wr(cc, GPUCC_GX_GFX3D_RCG + 4u, (cfg & ~RCG_CFG_MASK) | RCG_CFG_GFX3D_180MHZ);
	gpu_wr(cc, GPUCC_GX_GFX3D_RCG, cmd | RCG_CMD_UPDATE);
	bool ok = gpu_poll(cc, GPUCC_GX_GFX3D_RCG, RCG_CMD_UPDATE, 0, 500);
	printf("SWEET GPU:   gfx3d RCG cmd 0x%08x -> 0x%08x, cfg 0x%08x -> 0x%08x: %s\n", cmd,
	    gpu_rd(cc, GPUCC_GX_GFX3D_RCG), cfg, gpu_rd(cc, GPUCC_GX_GFX3D_RCG + 4u),
	    ok ? "update took" : "UPDATE TIMEOUT");
	return ok;
}

/* gdsc_enable for gpu_gx_gdsc; on timeout undo it the way gdsc_disable would. */
static bool
gpu_gx_gdsc_on(vm_offset_t cc)
{
	uint32_t v = gpu_rd(cc, GPUCC_GX_GDSCR), dom = gpu_rd(cc, GPUCC_GX_DOMAIN);
	if (v & GDSC_HW_CONTROL) {
		printf("SWEET GPU:   GX GDSC is under HW control (0x%08x); not touched\n", v);
		return false;
	}
	if ((v & GDSC_PWR_ON) && !(v & GDSC_SW_COLLAPSE)) {
		printf("SWEET GPU:   GX GDSC already on: gdscr 0x%08x domain 0x%08x\n", v, dom);
		return true;
	}
	uint32_t bcr = gpu_rd(cc, GPUCC_GX_BCR);
	gpu_wr(cc, GPUCC_GX_BCR, bcr | BCR_BLK_ARES);
	delay(1);
	gpu_wr(cc, GPUCC_GX_BCR, bcr & ~BCR_BLK_ARES);
	gpu_wr(cc, GPUCC_GX_DOMAIN, dom | GX_DOMAIN_GMEM_RESET);
	delay(1);
	gpu_wr(cc, GPUCC_GX_DOMAIN, dom & ~GX_DOMAIN_GMEM_RESET);
	gpu_wr(cc, GPUCC_GX_DOMAIN, dom & ~(GX_DOMAIN_GMEM_RESET | GX_DOMAIN_CLAMP_IO));
	gpu_wr(cc, GPUCC_GX_GDSCR, v & ~GDSC_SW_COLLAPSE);
	delay(1);       /* 8 XO cycles before the first status read */
	bool ok = gpu_poll(cc, GPUCC_GX_GDSCR, GDSC_PWR_ON, GDSC_PWR_ON, GDSC_TIMEOUT_US);
	uint32_t after = gpu_rd(cc, GPUCC_GX_GDSCR);
	if (ok) {
		delay(2);       /* clocks re-enable, staggered memory power-up */
	} else {
		gpu_wr(cc, GPUCC_GX_GDSCR, gpu_rd(cc, GPUCC_GX_GDSCR) | GDSC_SW_COLLAPSE);
		delay(1);
		gpu_wr(cc, GPUCC_GX_DOMAIN, gpu_rd(cc, GPUCC_GX_DOMAIN) | GX_DOMAIN_CLAMP_IO);
	}
	printf("SWEET GPU:   GX GDSC gdscr 0x%08x -> 0x%08x, domain 0x%08x -> 0x%08x, bcr 0x%08x: %s\n", v, after,
	    dom, gpu_rd(cc, GPUCC_GX_DOMAIN), gpu_rd(cc, GPUCC_GX_BCR),
	    ok ? "on" : "TIMEOUT; collapsed and clamped again");
	return ok;
}

static const struct gpu_branch gpu_gx_branches[] = {
	{ "gpu_cc_gx_cxo", GPUCC_GX_CXO_CBCR, true },
	{ "gpu_cc_gx_gmu", GPUCC_GX_GMU_CBCR, true },
	{ "gpu_cc_gx_gfx3d", GPUCC_GX_GFX3D_CBCR, false },
	{ "gpu_cc_cx_gfx3d", GPUCC_CX_GFX3D_CBCR, false },
};

/* run_measurement in clk-debug.c, with timeouts. */
static bool
gpu_meas_run(vm_offset_t gcc, uint32_t ticks, uint32_t *count)
{
	gpu_wr(gcc, GCC_DEBUG_CTL, ticks);
	if (!gpu_poll(gcc, GCC_DEBUG_STATUS, MEAS_DONE, 0, 1000)) {
		return false;
	}
	gpu_wr(gcc, GCC_DEBUG_CTL, MEAS_CNT_EN | ticks);
	bool ok = gpu_poll(gcc, GCC_DEBUG_STATUS, MEAS_DONE, MEAS_DONE, 50000);
	*count = gpu_rd(gcc, GCC_DEBUG_STATUS) & MEAS_CNT_MASK;
	gpu_wr(gcc, GCC_DEBUG_CTL, ticks);
	return ok;
}

/*
 * clk_debug_mux_set_parent + clk_debug_mux_measure_rate for one gpucc debug
 * mux input: kHz, 0 if the clock does not run, MEAS_FAILED if the counter
 * never finishes.
 */
static uint32_t
gpu_measure_khz(vm_offset_t gcc, vm_offset_t cc, uint32_t sel)
{
	gpu_wr(cc, GPUCC_DEBUG_SEL, (gpu_rd(cc, GPUCC_DEBUG_SEL) & ~0xffu) | sel);
	gpu_wr(cc, GPUCC_DEBUG_POST_DIV, (gpu_rd(cc, GPUCC_DEBUG_POST_DIV) & ~3u) | (GPUCC_DEBUG_DIV - 1u));
	gpu_wr(cc, GPUCC_DEBUG_CBCR, gpu_rd(cc, GPUCC_DEBUG_CBCR) | CBCR_CLK_ENABLE);
	gpu_wr(gcc, GCC_DEBUG_SEL, (gpu_rd(gcc, GCC_DEBUG_SEL) & ~0x3ffu) | GCC_DEBUG_SEL_GPUCC);
	gpu_wr(gcc, GCC_DEBUG_POST_DIV, (gpu_rd(gcc, GCC_DEBUG_POST_DIV) & ~0xfu) | (GCC_DEBUG_DIV - 1u));
	gpu_wr(gcc, GCC_DEBUG_CBCR, gpu_rd(gcc, GCC_DEBUG_CBCR) | CBCR_CLK_ENABLE);
	gpu_wr(gcc, GCC_XO_DIV4_CBCR, gpu_rd(gcc, GCC_XO_DIV4_CBCR) | CBCR_CLK_ENABLE);
	/* A stopped clock does not reset the counter: equal short and full counts mean off. */
	uint32_t s = 0, f = 0;
	bool ok = gpu_meas_run(gcc, MEAS_TICKS_SHORT, &s) && gpu_meas_run(gcc, MEAS_TICKS_FULL, &f);
	gpu_wr(gcc, GCC_XO_DIV4_CBCR, gpu_rd(gcc, GCC_XO_DIV4_CBCR) & ~CBCR_CLK_ENABLE);
	gpu_wr(gcc, GCC_DEBUG_CBCR, gpu_rd(gcc, GCC_DEBUG_CBCR) & ~CBCR_CLK_ENABLE);
	gpu_wr(cc, GPUCC_DEBUG_CBCR, gpu_rd(cc, GPUCC_DEBUG_CBCR) & ~CBCR_CLK_ENABLE);
	if (!ok) {
		return MEAS_FAILED;
	}
	if (f == s) {
		return 0;
	}
	uint64_t hz = ((uint64_t)f * 10u + 15u) * MEAS_XO_DIV4_HZ / (MEAS_TICKS_FULL * 10u + 35u);
	return (uint32_t)(hz * GCC_DEBUG_DIV * GPUCC_DEBUG_DIV / 1000u);
}

/* Stage 6.6: gx_gfx3d now, in kHz (0 stopped, MEAS_FAILED if not measurable) */
uint32_t
sweet_gpu_gfx3d_khz(void)
{
	if (!gpu_meas_gcc || !gpu_meas_cc) {
		return MEAS_FAILED;
	}
	return gpu_measure_khz(gpu_meas_gcc, gpu_meas_cc, 0x0c);
}

struct gpu_meas {
	const char *name;
	uint32_t sel;           /* gpucc debug mux input */
};

/* cx_gmu first: stage 1 set it to 200 MHz, so it shows whether the counter can be trusted. */
static const struct gpu_meas gpu_meas_list[] = {
	{ "cx_gmu", 0x19 }, { "gx_gfx3d", 0x0c }, { "cx_gfx3d", 0x1a }, { "gx_cxo", 0x0f }, { "gx_gmu", 0x10 },
};
#define MEAS_ALL                5u
#define MEAS_GFX3D              3u              /* cx_gmu, gx_gfx3d, cx_gfx3d */

/* The first n clocks of gpu_meas_list into khz[]. */
static void
gpu_measure(vm_offset_t gcc, vm_offset_t cc, unsigned n, const char *when, uint32_t *khz)
{
	for (unsigned i = 0; i < n; ++i) {
		khz[i] = gpu_measure_khz(gcc, cc, gpu_meas_list[i].sel);
		if (khz[i] == MEAS_FAILED) {
			printf("SWEET GPU:   measured %s %s: counter did not finish\n", when, gpu_meas_list[i].name);
		} else {
			printf("SWEET GPU:   measured %s %-9s %u kHz%s\n", when, gpu_meas_list[i].name, khz[i],
			    khz[i] ? "" : " (not running)");
		}
	}
}

/* a6xx_sptprac_enable; upstream polls 100 us. */
static bool
gpu_sptprac_on(vm_offset_t gmu)
{
	uint32_t before = gmu_rd(gmu, REG_A6XX_GMU_SPTPRAC_PWR_CLK_STATUS);
	gmu_wr(gmu, REG_A6XX_GMU_GX_SPTPRAC_POWER_CONTROL, SPTPRAC_POWER_ON);
	bool ok = false;
	for (unsigned i = 0; i < 500 && !ok; ++i) {
		ok = (gmu_rd(gmu, REG_A6XX_GMU_SPTPRAC_PWR_CLK_STATUS) & SPTPRAC_ON_MASK) == SPTPRAC_ON_WANT;
		if (!ok) {
			delay(1);
		}
	}
	printf("SWEET GPU:   SPTPRAC status 0x%08x -> 0x%08x: %s\n", before,
	    gmu_rd(gmu, REG_A6XX_GMU_SPTPRAC_PWR_CLK_STATUS), ok ? "on" : "NOT ON");
	return ok;
}

static uint64_t
gpu_cp_alwayson(vm_offset_t gpu)
{
	const uint32_t lo_off = REG_A6XX_CP_ALWAYS_ON_COUNTER * 4u, hi_off = lo_off + 4u;
	uint32_t hi = gpu_rd(gpu, hi_off), lo, hi2;
	for (unsigned i = 0; i < 4; ++i) {
		lo = gpu_rd(gpu, lo_off);
		hi2 = gpu_rd(gpu, hi_off);
		if (hi2 == hi) {
			return (uint64_t)hi << 32 | lo;
		}
		hi = hi2;
	}
	return (uint64_t)hi << 32 | gpu_rd(gpu, lo_off);
}

static void
gpu_stage2a(vm_offset_t gcc, vm_offset_t cc, vm_offset_t gmu)
{
	printf("SWEET GPU: stage 2A (sweetgpu=%d): host-driven GX power at 180 MHz, then the first GX reads\n",
	    gpu_level);
	vm_offset_t gpu = ml_io_map(GPU_PA, GPU_REGS_SIZE);
	if (!gpu) {
		printf("SWEET GPU: stage 2A mapping failed\n");
		return;
	}
	gpu_log_gx(cc, gmu, "before", false);
	gpu_pause("RPMh ARC vote mx.lvl >= LOW_SVS (first GPU rail vote)");
	if (!gpu_rail_vote("mx.lvl", RPMH_VLVL_LOW_SVS)) {
		printf("SWEET GPU: stage 2A stopped: mx.lvl vote\n");
		return;
	}
	gpu_pause("RPMh ARC vote gfx.lvl >= LOW_SVS (GX rail up)");
	if (!gpu_rail_vote("gfx.lvl", RPMH_VLVL_LOW_SVS)) {
		printf("SWEET GPU: stage 2A stopped: gfx.lvl vote\n");
		return;
	}
	gpu_pause("PLL0 configure and lock at 360 MHz (gpucc 0x5090000)");
	if (!gpu_pll0_360mhz(cc)) {
		printf("SWEET GPU: stage 2A stopped: PLL0\n");
		return;
	}
	gpu_pause("gfx3d RCG root forced on, as gdsc_enable does first (gpucc 0x509101c)");
	bool root = gpu_gfx3d_root_on(cc, "before GDSC");
	if (!root) {
		printf("SWEET GPU:   going on: downstream gdsc_enable only warns here (GX is still clamped)\n");
	}
	gpu_pause("GX GDSC on: BCR and GMEM reset pulses, unclamp, un-collapse (gpucc 0x509100c)");
	if (!gpu_gx_gdsc_on(cc)) {
		gpu_log_gx(cc, gmu, "after GDSC", false);
		printf("SWEET GPU: stage 2A stopped: GX GDSC (gfx.lvl vote may not reach the rail)\n");
		return;
	}
	gpu_log_gx(cc, gmu, "with GX GDSC", true);
	if (!root) {
		gpu_pause("gfx3d RCG root again, GX unclamped");
		root = gpu_gfx3d_root_on(cc, "with GX GDSC");
	}
	gpu_pause("GX branches: gx_cxo, gx_gmu, gx_gfx3d, cx_gfx3d");
	bool clocks = true;
	for (unsigned i = 0; i < sizeof(gpu_gx_branches) / sizeof(gpu_gx_branches[0]); ++i) {
		clocks = gpu_branch_on(cc, &gpu_gx_branches[i]) && clocks;
	}
	uint32_t khz[MEAS_ALL];
	gpu_pause("clock counter (GCC 0x162000): cx_gmu, gx_gfx3d, cx_gfx3d, gx_cxo, gx_gmu");
	gpu_measure(gcc, cc, MEAS_ALL, "with branches", khz);
	if (!clocks) {
		gpu_log_gx(cc, gmu, "after branches", true);
		printf("SWEET GPU: stage 2A stopped: GX branches\n");
		return;
	}
	gpu_pause(root ? "gfx3d RCG to 180 MHz from PLL0 even" :
	    "gfx3d RCG to 180 MHz from PLL0 even, UPDATE although the root reads off");
	bool update = gpu_gfx3d_rcg_180mhz(cc);
	root = !(gpu_rd(cc, GPUCC_GX_GFX3D_RCG) & RCG_CMD_ROOT_OFF);
	gpu_pause("clock counter again: cx_gmu, gx_gfx3d, cx_gfx3d");
	gpu_measure(gcc, cc, MEAS_GFX3D, "after update", khz);
	/* MEAS_FAILED is above both ranges. */
	bool counter = khz[0] >= 150000u && khz[0] <= 250000u;
	bool runs = counter ? khz[1] >= 100000u && khz[1] <= 250000u : root;
	printf("SWEET GPU:   gfx3d: update %s, root reads %s, counter %s, so gx_gfx3d %s\n",
	    update ? "took" : "TIMED OUT", root ? "on" : "off",
	    counter ? "trusted (cx_gmu near 200 MHz)" : "NOT trusted (falling back to the root bit)",
	    runs ? "runs near 180 MHz" : "does not run near 180 MHz");
	gpu_pause("SPTP head switch on (GMU_GX_SPTPRAC_POWER_CONTROL)");
	bool sptp = gpu_sptprac_on(gmu);
	gpu_log_gx(cc, gmu, "after", true);
	if (!update || !runs) {
		printf("SWEET GPU: stage 2A stopped: %s; no GX reads without the core clock\n",
		    update ? "gx_gfx3d does not run near 180 MHz" : "gfx3d RCG update timed out");
		return;
	}
	uint32_t sp = gmu_rd(gmu, REG_A6XX_GMU_SPTPRAC_PWR_CLK_STATUS);
	if (sp & (A6XX_GMU_SPTPRAC_PWR_CLK_STATUS_GX_HM_GDSC_POWER_OFF__MASK |
	    A6XX_GMU_SPTPRAC_PWR_CLK_STATUS_GX_HM_CLK_OFF__MASK)) {
		printf("SWEET GPU: stage 2A stopped: the GMU sees GX %s (sptprac 0x%08x); no GX reads\n",
		    (sp & A6XX_GMU_SPTPRAC_PWR_CLK_STATUS_GX_HM_GDSC_POWER_OFF__MASK) ? "power off" : "clock off", sp);
		return;
	}
	gpu_pause("first GX read: RBBM_STATUS (0x5000840)");
	printf("SWEET GPU:   RBBM_STATUS 0x%08x\n", gpu_rd(gpu, REG_A6XX_RBBM_STATUS * 4u));
	printf("SWEET GPU:   RBBM_STATUS1 0x%08x STATUS2 0x%08x STATUS3 0x%08x CLOCK_CNTL 0x%08x\n",
	    gpu_rd(gpu, REG_A6XX_RBBM_STATUS1 * 4u), gpu_rd(gpu, REG_A6XX_RBBM_STATUS2 * 4u),
	    gpu_rd(gpu, REG_A6XX_RBBM_STATUS3 * 4u), gpu_rd(gpu, REG_A6XX_RBBM_CLOCK_CNTL * 4u));
	gpu_pause("CP always-on counter (0x5002600) twice, 1 ms apart");
	uint64_t c0 = gpu_cp_alwayson(gpu), g0 = gmu_alwayson(gmu);
	delay(1000);
	uint64_t c1 = gpu_cp_alwayson(gpu), g1 = gmu_alwayson(gmu);
	printf("SWEET GPU:   CP always-on counter %llu -> %llu (+%llu); GMU counter +%llu in the same 1 ms\n",
	    c0, c1, c1 - c0, g1 - g0);
	uint32_t rbbm = gpu_rd(gpu, REG_A6XX_RBBM_STATUS * 4u);
	printf("SWEET GPU: stage 2A %s: GX reads work, RBBM_STATUS 0x%08x (%s), CP counter %s, SPTP %s; GX stays on\n",
	    c1 > c0 && !(rbbm & A6XX_RBBM_STATUS_GPU_BUSY_IGN_AHB__MASK) ? "PASSED" : "done with findings", rbbm,
	    (rbbm & A6XX_RBBM_STATUS_GPU_BUSY_IGN_AHB__MASK) ? "busy" : "idle", c1 > c0 ? "runs" : "STOPPED",
	    sptp ? "on" : "NOT on");
}

/*
 * GX as the GMU left it, read only: GDSC, gfx3d RCG, SPTPRAC, then the clock
 * counter (the gx_* clocks only with the GX GDSC on). True if the GDSC is on
 * and gx_gfx3d runs at any rate.
 */
static bool
gpu_gx_check(vm_offset_t gcc, vm_offset_t cc, vm_offset_t gmu, const char *when)
{
	bool gdsc = (gpu_rd(cc, GPUCC_GX_GDSCR) & GDSC_PWR_ON) != 0;
	gpu_log_gx(cc, gmu, when, gdsc);
	uint32_t khz[MEAS_ALL] = { 0 };
	gpu_pause(gdsc ? "clock counter (GCC 0x162000): cx_gmu, gx_gfx3d, cx_gfx3d, gx_cxo, gx_gmu" :
	    "clock counter (GCC 0x162000): cx_gmu only, GX GDSC off");
	gpu_measure(gcc, cc, gdsc ? MEAS_ALL : 1u, when, khz);
	bool counter = khz[0] >= 150000u && khz[0] <= 250000u;
	bool runs = gdsc && khz[1] != MEAS_FAILED && khz[1] != 0;
	printf("SWEET GPU: %s: GX GDSC %s, gx_gfx3d %s (%u kHz), counter %s\n", when, gdsc ? "on" : "OFF",
	    runs ? "runs" : "does not run", runs ? khz[1] : 0, counter ? "trusted (cx_gmu near 200 MHz)" : "NOT trusted");
	return runs;
}

/* Not in sweet_gpu_regs.h; dword offsets from GPU_PA (a6xx.xml) */
#define REG_A6XX_RBBM_INT_0_STATUS      0x00201u
#define REG_A6XX_GBIF_HALT              0x03c45u

/* With GX on and gx_gfx3d running: the first GX reads. GBIF_HALT is only read. */
static void
gpu_gx_reads(vm_offset_t gpu, vm_offset_t gmu)
{
	gpu_pause("first GX reads: RBBM_STATUS (0x5000840), RBBM_INT_0_STATUS, GBIF_HALT");
	uint32_t rbbm = gpu_rd(gpu, REG_A6XX_RBBM_STATUS * 4u);
	printf("SWEET GPU:   RBBM_STATUS 0x%08x RBBM_INT_0_STATUS 0x%08x GBIF_HALT 0x%08x\n", rbbm,
	    gpu_rd(gpu, REG_A6XX_RBBM_INT_0_STATUS * 4u), gpu_rd(gpu, REG_A6XX_GBIF_HALT * 4u));
	gpu_pause("CP always-on counter (0x5002600) twice, 1 ms apart");
	uint64_t c0 = gpu_cp_alwayson(gpu), g0 = gmu_alwayson(gmu);
	delay(1000);
	uint64_t c1 = gpu_cp_alwayson(gpu), g1 = gmu_alwayson(gmu);
	printf("SWEET GPU:   CP always-on counter %llu -> %llu (+%llu); GMU counter +%llu in the same 1 ms\n",
	    c0, c1, c1 - c0, g1 - g0);
	printf("SWEET GPU: GX reads %s: RBBM_STATUS 0x%08x (%s), CP counter %s\n", c1 > c0 ? "work" : "with findings",
	    rbbm, (rbbm & A6XX_RBBM_STATUS_GPU_BUSY_IGN_AHB__MASK) ? "busy" : "idle", c1 > c0 ? "runs" : "STOPPED");
}

/*
 * Stage 2B (sweetgpu=4): the GMU firmware powers GX. sweet_gmu.c does the
 * GMU steps 2B.0 to 2B.6; sweetgmu=N stops after 2B.N. GX is checked here
 * with stage 2A's clock counter. True if every step that ran passed.
 */
static bool
gpu_stage2b(vm_offset_t gcc, vm_offset_t cc, vm_offset_t gmu, int bin)
{
	printf("SWEET GPU: stage 2B (sweetgpu=%d sweetgmu=%d%s): the GMU firmware powers GX\n", gpu_level,
	    gpu_gmu_stop, gpu_gmu_rsc == 1 ? " sweetgmursc=1" : "");
	if (bin != (int)GPU_BIN_2B) {
		printf("SWEET GPU: stage 2B stopped: speed bin %d; the GMU perf table is for bin %u\n", bin, GPU_BIN_2B);
		return false;
	}
	if (!gpu_blobs[BLOB_GMU].va || !gpu_blobs[BLOB_GMU_MEM].va) {
		printf("SWEET GPU: stage 2B stopped: GPU-GMU or GPU-GMU-MEM missing (mkboot.sh GPUFW=1)\n");
		return false;
	}
	vm_offset_t gpu = ml_io_map(GPU_PA, GPU_REGS_SIZE);
	if (!gpu) {
		printf("SWEET GPU: stage 2B mapping failed\n");
		return false;
	}
	gpu_log_gx(cc, gmu, "as found", false);
	int step = 0;
	bool ok = sweet_gmu_survey(gmu, gpu_blobs[BLOB_GMU_MEM].pa, gpu_blobs[BLOB_GMU_MEM].len);
	if (ok && gpu_gmu_stop >= 1) {
		step = 1;
		ok = sweet_gmu_mem();
	}
	if (ok && gpu_gmu_stop >= 2) {
		step = 2;
		ok = sweet_gmu_rpmh();
	}
	if (ok && gpu_gmu_stop >= 3) {
		step = 3;
		ok = sweet_gmu_boot(gpu_blobs[BLOB_GMU].va, gpu_blobs[BLOB_GMU].len, gpu_gmu_rsc == 1);
	}
	if (ok && gpu_gmu_stop >= 4) {
		step = 4;
		ok = sweet_gmu_gx_on();
		ok = gpu_gx_check(gcc, cc, gmu, "after GMU rail") && ok;
	}
	if (ok && gpu_gmu_stop >= 5) {
		step = 5;
		ok = sweet_gmu_hfi();
	}
	if (ok && gpu_gmu_stop >= 6) {
		step = 6;
		ok = sweet_gmu_oob_gpu();
		bool runs = gpu_gx_check(gcc, cc, gmu, "after OOB_GPU");
		if (ok && runs) {
			gpu_gx_reads(gpu, gmu);
		}
		ok = ok && runs;
		for (unsigned s = 10; s <= 60; s += 10) {
			gpu_sleep_ms(10000);
			char when[24];
			snprintf(when, sizeof(when), "monitor %u s", s);
			sweet_gmu_status(when);
			printf("SWEET GPU: %s: gx_gdscr 0x%08x\n", when, gpu_rd(cc, GPUCC_GX_GDSCR));
		}
	}
	printf("SWEET GPU: stage 2B %s at 2B.%d (sweetgmu=%d); what is on stays on\n",
	    ok ? "done" : "STOPPED by a failure", step, gpu_gmu_stop);
	return ok;
}

/* Stage 3: the GPU's SMMU bank, stream 0, DRAM mapped 1:1 (see the top of this file) */
#define GPU_SMMU_SID            0u
#define GPU_SMMU_BANK           0
#define GPU_SMMU_IA_BITS        36u
#define GPU_IDMAP_START         0x80000000ull
#define GPU_IDMAP_END           0x200000000ull
#define GPU_SMMU_PT_SIZE        0x1000u         /* level 1 only: 1 GiB blocks */
#define A6XX_RBBM_STATUS3_SMMU_STALLED  (1u << 24)

static bool
gpu_stage3(vm_offset_t gpu, int *cbp)
{
	static const uint32_t sids[] = { GPU_SMMU_SID };
	printf("SWEET GPU: stage 3 (sweetgpu=%d): SMMU bank %d for GPU stream %u, %#llx..%#llx 1:1\n", gpu_level,
	    GPU_SMMU_BANK, GPU_SMMU_SID, GPU_IDMAP_START, GPU_IDMAP_END - 1u);
	uint64_t pa = 0;
	vm_offset_t va = sweet_gmu_spare(GPU_SMMU_PT_SIZE, &pa);
	if (!va) {
		printf("SWEET GPU: stage 3 stopped: no room past the GMU's part of GPU-GMU-MEM\n");
		return false;
	}
	struct sweet_smmu_pt pt;
	sweet_smmu_pt_init(&pt, va, pa, GPU_SMMU_PT_SIZE);
	if (!sweet_smmu_map_1g(&pt, GPU_IDMAP_START, GPU_IDMAP_START, GPU_IDMAP_END - GPU_IDMAP_START)) {
		return false;
	}
	gpu_pause("SMMU bank 0 and a stream match for GPU stream 0 (0x5040000)");
	int cb = sweet_smmu_attach(sids, 1, pa, GPU_SMMU_IA_BITS, GPU_SMMU_BANK);
	if (cb < 0) {
		return false;
	}
	*cbp = cb;
	gpu_l1_va = va;
	gpu_l1_pa = pa;
	bool ok = sweet_smmu_faults(cb, "stage 3 after attach") == 0;
	uint32_t st3 = gpu_rd(gpu, REG_A6XX_RBBM_STATUS3 * 4u);
	printf("SWEET GPU:   RBBM_STATUS 0x%08x RBBM_STATUS3 0x%08x (%s)\n", gpu_rd(gpu, REG_A6XX_RBBM_STATUS * 4u), st3,
	    (st3 & A6XX_RBBM_STATUS3_SMMU_STALLED) ? "STALLED on an SMMU fault" : "not stalled");
	ok = ok && !(st3 & A6XX_RBBM_STATUS3_SMMU_STALLED);
	for (unsigned s = 10; s <= 30; s += 10) {
		gpu_sleep_ms(10000);
		char when[32];
		snprintf(when, sizeof(when), "stage 3 monitor %u s", s);
		ok = sweet_smmu_faults(cb, when) == 0 && ok;
		sweet_gmu_status(when);
	}
	printf("SWEET GPU: stage 3 %s: bank %d %s; what is on stays on\n", ok ? "done" : "STOPPED by a failure", cb,
	    ok ? "set up, no fault" : "has a fault");
	return ok;
}

static void
gpu_thread(void *arg, wait_result_t result)
{
	(void)arg;
	(void)result;
	uint64_t now_ns;
	absolutetime_to_nanoseconds(mach_absolute_time(), &now_ns);
	uint64_t now_ms = now_ns / NSEC_PER_MSEC, at_ms = (uint64_t)gpu_delay_s * 1000u;
	if (now_ms < at_ms) {
		gpu_sleep_ms((unsigned)(at_ms - now_ms));
	}
	printf("SWEET GPU: stage 0 (sweetgpu=%d): firmware blobs, cmd-db, speed bin; no GPU registers\n",
	    gpu_level);
	if (gpu_level > 8) {
		printf("SWEET GPU: stages above 6 are not built yet; running stages 0 to 6\n");
	}
	unsigned found = gpu_find_blobs();
	gpu_check_blobs();
	printf("SWEET GPU: %u of %u blobs present\n", found, (unsigned)BLOB_COUNT);
	gpu_cmddb();
	int bin = gpu_speed_bin();
	printf("SWEET GPU: stage 0 done\n");
	vm_offset_t gcc = 0, cc = 0, gmu = 0;
	if (gpu_level >= 2 && gpu_stage1(&gcc, &cc, &gmu)) {
		if (gpu_level == 3) {
			gpu_stage2a(gcc, cc, gmu);
		} else if (gpu_level >= 4 && gpu_stage2b(gcc, cc, gmu, bin) && gpu_level >= 5) {
			vm_offset_t gpu = ml_io_map(GPU_PA, GPU_REGS_SIZE);
			gpu_meas_gcc = gcc;
			gpu_meas_cc = cc;
			if (gpu_gmu_stop != 6) {
				printf("SWEET GPU: stage 3 needs all of 2B (sweetgmu=6); not run\n");
			} else if (gpu) {
				int cb = -1;
				if (gpu_stage3(gpu, &cb) && gpu_level >= 6 &&
				    sweet_cp_run(gpu_cp_stop, cb, gpu_blobs[BLOB_SQE].va, gpu_blobs[BLOB_SQE].len,
				    cc + GPUCC_GX_GDSCR) && gpu_level >= 7) {
					if (gpu_cp_stop < 5) {
						printf("SWEET GPU: stage 5 needs stage 4 through 4.5 (sweetcp=5 or 6); not run\n");
					} else if (sweet_zap_run(gpu_zap_stop, gpu_blobs[BLOB_ZAP_MDT].va,
					    gpu_blobs[BLOB_ZAP_MDT].len, gpu_blobs[BLOB_ZAP_B02].va,
					    gpu_blobs[BLOB_ZAP_B02].len) && gpu_level >= 8) {
						if (gpu_zap_stop < 5) {
							printf("SWEET GPU: stage 6 needs stage 5 through 5.5 (sweetzap=5 or 6); not run\n");
						} else {
							(void)sweet_gsvc_run(gpu_svc_stop, cb, gpu_l1_va, gpu_l1_pa);
						}
					}
				}
			}
		}
	}
	if (gpu_hang == 1) {
		gpu_pause("deliberate hang (sweetgpuhang=1): interrupts off, spinning; pull the log from recovery");
		(void)ml_set_interrupts_enabled(FALSE);
		for (;;) {
			__asm__ volatile("wfe");
		}
	}
}

void
sweet_gpu_init(void)
{
	(void)PE_parse_boot_argn("sweetgpu", &gpu_level, sizeof(gpu_level));
	if (gpu_level <= 0) {
		return;
	}
	(void)PE_parse_boot_argn("sweetgpuhang", &gpu_hang, sizeof(gpu_hang));
	(void)PE_parse_boot_argn("sweetgpudelay", &gpu_delay_s, sizeof(gpu_delay_s));
	if (gpu_delay_s > 600) {
		gpu_delay_s = SWEET_GPU_DELAY_S;
	}
	(void)PE_parse_boot_argn("sweetgmu", &gpu_gmu_stop, sizeof(gpu_gmu_stop));
	if (gpu_gmu_stop < 0 || gpu_gmu_stop > 6) {
		gpu_gmu_stop = 0;
	}
	(void)PE_parse_boot_argn("sweetgmursc", &gpu_gmu_rsc, sizeof(gpu_gmu_rsc));
	(void)PE_parse_boot_argn("sweetcp", &gpu_cp_stop, sizeof(gpu_cp_stop));
	if (gpu_cp_stop < 0 || gpu_cp_stop > 6) {
		gpu_cp_stop = 0;
	}
	(void)PE_parse_boot_argn("sweetzap", &gpu_zap_stop, sizeof(gpu_zap_stop));
	if (gpu_zap_stop < 0 || gpu_zap_stop > 6) {
		gpu_zap_stop = 0;
	}
	(void)PE_parse_boot_argn("sweetsvc", &gpu_svc_stop, sizeof(gpu_svc_stop));
	if (gpu_svc_stop < 0 || gpu_svc_stop > 7) {
		gpu_svc_stop = 0;
	}
	thread_t thread;
	if (kernel_thread_start(gpu_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
		printf("SWEET GPU: probe thread starts %u s after boot\n", gpu_delay_s);
	}
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
