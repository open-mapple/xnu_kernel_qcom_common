/*
 * Shared by the SM7150 Adreno 618 bring-up files (docs/GPU_A618_PLAN.md):
 * sweet_gpu.c (stages 0 to 2, the 2B flow), sweet_gmu.c (GMU firmware,
 * RSCC/PDC, HFI), sweet_smmu.c (the GPU SMMU), sweet_cp.c (stage 4, the CP)
 * and sweet_zap.c (stage 5, the zap shader).
 */
#ifndef _SWEET_GPU_INTERNAL_H_
#define _SWEET_GPU_INTERNAL_H_
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/systm.h>
#include <kern/clock.h>

/* GMU (sdmmagpie-gpu.dtsi); sweet_gpu_regs.h offsets are dwords from GPU_PA */
#define GPU_PA                  0x05000000u
#define GMU_PA                  0x0506a000u
#define GMU_SIZE                0x31000u        /* covers RSCC (0x508d000) and GMU_AO (0x508ec00) */

/* sweet_gpu.c: log what comes next, then let the persistent log reach the disk. */
void gpu_pause(const char *next);
void gpu_sleep_ms(unsigned ms);

static inline uint32_t
gpu_rd(vm_offset_t base, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static inline void
gpu_wr(vm_offset_t base, uint32_t off, uint32_t val)
{
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)(base + off) = val;
	__asm__ volatile("dsb osh" ::: "memory");
}

/* Poll every 1 us until (reg & mask) == want; false after us tries. */
static inline bool
gpu_poll(vm_offset_t base, uint32_t off, uint32_t mask, uint32_t want, unsigned us)
{
	for (unsigned i = 0; i < us; ++i) {
		if ((gpu_rd(base, off) & mask) == want) {
			return true;
		}
		delay(1);
	}
	return (gpu_rd(base, off) & mask) == want;
}

/* reg is a dword offset from GPU_PA, as in sweet_gpu_regs.h. */
static inline uint32_t
gmu_rd(vm_offset_t gmu, uint32_t reg)
{
	uint32_t off = reg * 4u - (GMU_PA - GPU_PA);
	if (reg * 4u < GMU_PA - GPU_PA || off >= GMU_SIZE) {
		printf("SWEET GPU: register %#x is outside the GMU block\n", reg);
		return 0xffffffffu;
	}
	return gpu_rd(gmu, off);
}

static inline void
gmu_wr(vm_offset_t gmu, uint32_t reg, uint32_t val)
{
	uint32_t off = reg * 4u - (GMU_PA - GPU_PA);
	if (reg * 4u < GMU_PA - GPU_PA || off >= GMU_SIZE) {
		printf("SWEET GPU: register %#x is outside the GMU block; not written\n", reg);
		return;
	}
	gpu_wr(gmu, off, val);
}

/* sweet_smmu.c: the GPU SMMU (qcom,smmu-v2 at 0x5040000) */
struct sweet_smmu_pt {
	vm_offset_t va;         /* uncached mapping of the table area */
	uint64_t pa;
	uint32_t size, used;    /* bytes; 4 KiB tables from the start, level 1 first */
};
bool sweet_smmu_survey(void);
void sweet_smmu_pt_init(struct sweet_smmu_pt *pt, vm_offset_t va, uint64_t pa, uint32_t size);
bool sweet_smmu_map(struct sweet_smmu_pt *pt, uint32_t iova, uint64_t pa, uint32_t len);
bool sweet_smmu_map_1g(struct sweet_smmu_pt *pt, uint64_t iova, uint64_t pa, uint64_t len);
int sweet_smmu_attach(const uint32_t *sids, unsigned nsid, uint64_t ttbr, unsigned ia_bits, int want);
uint32_t sweet_smmu_faults(int cb, const char *when);

/* sweet_gmu.c: stage 2B steps, in order; each needs the previous one */
bool sweet_gmu_survey(vm_offset_t gmu, uint64_t mem_pa, uint32_t mem_len);     /* 2B.0 */
bool sweet_gmu_mem(void);                                                       /* 2B.1 */
bool sweet_gmu_rpmh(void);                                                      /* 2B.2 */
bool sweet_gmu_boot(const uint8_t *fw, uint32_t len, bool rsc_start);         /* 2B.3 */
bool sweet_gmu_gx_on(void);                                                     /* 2B.4 */
bool sweet_gmu_hfi(void);                                                       /* 2B.5 */
bool sweet_gmu_oob_gpu(void);                                                   /* 2B.6 */
void sweet_gmu_status(const char *when);
vm_offset_t sweet_gmu_spare(uint32_t len, uint64_t *pa);       /* GPU-GMU-MEM past the GMU's part, after 2B.1 */

/* sweet_cp.c: stage 4, CP first light; stop = sweetcp (0-6), cb = stage 3's bank, gx_gdscr = mapped GX GDSCR */
bool sweet_cp_run(int stop, int cb, const uint8_t *sqe, uint32_t sqe_len, vm_offset_t gx_gdscr);
/* The ring and the scratch page for stage 5, once sweet_cp_run passed 4.4 or later */
uint32_t sweet_cp_rd(uint32_t reg);
uint32_t sweet_cp_pkt7(uint32_t op, uint32_t cnt);
void sweet_cp_emit(uint32_t w);
void sweet_cp_kick(void);                                       /* dsb sy, then CP_RB_WPTR */
int sweet_cp_wait_idle(unsigned timeout_us);                    /* us taken, or -1 */
bool sweet_cp_marker(uint32_t val, const char *when);          /* PKT4 to CP_SCRATCH_REG(7), kick, wait */
void sweet_cp_dump(const char *when);
bool sweet_cp_clean(const char *when);                          /* no SMMU, CP_HW_FAULT or PROTECT change */
void sweet_cp_halt(void);                                       /* CP_SQE_CNTL = 0 if the SQE runs */
volatile uint32_t *sweet_cp_scratch(uint64_t *pa);              /* 4 KiB, uncached, IOVA = PA */

/* sweet_zap.c: stage 5, the zap shader; stop = sweetzap (0-6) */
bool sweet_zap_run(int stop, const uint8_t *mdt, uint32_t mdt_len, const uint8_t *b02, uint32_t b02_len);

/* sweet_rpmh.c: cmd-db address of an ARC (slave 3) or BCM (slave 5) resource, and an ARC's levels */
int sweet_rpmh_cmddb_query(const char *res, unsigned slave, uint32_t *addr, unsigned *vlvl,
    unsigned nmax, unsigned *n);

#endif /* ARM64_BOARD_CONFIG_SWEET */
#endif /* _SWEET_GPU_INTERNAL_H_ */
