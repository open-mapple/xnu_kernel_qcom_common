/*
 * SM7150 GPU SMMU (qcom,smmu-v2 "kgsl_smmu" at 0x5040000) for the Adreno 618
 * bring-up, docs/GPU_A618_PLAN.md stage 2B.0, 2B.1 and later stage 3.
 *
 * Register layout from the ARM SMMUv2 architecture spec; the global space is
 * 2^(NUMPAGENDXB + 1) pages, then the context banks, one page each (kgsl's
 * qcom,gpu-offset puts bank 0 at +0x8000). The SMMU needs the CX GDSC and
 * gcc_gpu_memnoc_gfx, both on after stage 1.
 *  - sweet_smmu_survey reads the ID and global registers, every stream-match
 *    group and every context bank; nothing is written.
 *  - sweet_smmu_attach programs one free bank (never bank 0, the GPU's) as
 *    AArch64 stage 1 with stage 2 bypass: 4 KiB granule, 32-bit input range
 *    (walks start at level 1), non-cacheable walks, faults terminate. Then a
 *    free stream-match group points the given streams at it. Every write is
 *    read back; a mismatch undoes the stream match.
 *  - sweet_smmu_map fills the tables in caller memory that is mapped
 *    uncached, so the SMMU's non-cacheable walks see every store.
 * Unmatched streams keep bypassing; sCR0 is written only if the SMMU is off
 * (CLIENTPD), and then only to turn it on with unmatched streams bypassing.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include "sweet_gpu_internal.h"

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);

#define SMMU_PA                 0x05040000u
#define SMMU_SIZE               0x10000u
/* Global register space 0 */
#define SMMU_sCR0               0x000u
#define SMMU_sACR               0x010u
#define SMMU_IDR0               0x020u
#define SMMU_IDR1               0x024u
#define SMMU_IDR2               0x028u
#define SMMU_IDR7               0x03cu
#define SMMU_sGFAR              0x040u          /* 64-bit */
#define SMMU_sGFSR              0x048u
#define SMMU_sGFSYNR0           0x050u
#define SMMU_TLBIALLNSNH        0x068u
#define SMMU_sTLBGSYNC          0x070u
#define SMMU_sTLBGSTATUS        0x074u
#define SMMU_SMR(n)             (0x800u + 4u * (n))
#define SMMU_S2CR(n)            (0xc00u + 4u * (n))
/* Global register space 1, one page up */
#define SMMU_CBAR(n)            (0x000u + 4u * (n))
#define SMMU_CBA2R(n)           (0x800u + 4u * (n))
/* Context bank */
#define CB_SCTLR                0x000u
#define CB_TCR2                 0x010u
#define CB_TTBR0                0x020u          /* 64-bit */
#define CB_TCR                  0x030u
#define CB_CONTEXTIDR           0x034u
#define CB_MAIR0                0x038u
#define CB_FSR                  0x058u
#define CB_FAR                  0x060u          /* 64-bit */
#define CB_FSYNR0               0x068u
#define CB_TLBIALL              0x618u
#define CB_TLBSYNC              0x7f0u
#define CB_TLBSTATUS            0x7f4u

#define sCR0_CLIENTPD           (1u << 0)
#define sCR0_USFCFG             (1u << 10)
#define IDR0_S1TS               (1u << 30)
#define IDR1_PAGESIZE           (1u << 31)
#define IDR2_PTFS_4K            (1u << 12)
#define TLBSTATUS_ACTIVE        (1u << 0)
#define SMR_VALID               (1u << 31)
#define SMR_ID_MASK             0x7fffu
#define S2CR_TYPE(v)            (((v) >> 16) & 3u)      /* 0 translate, 1 bypass, 2 fault */
#define S2CR_CBNDX(v)           ((v) & 0xffu)
#define S2CR_CHECKED            ((3u << 16) | 0xffu)
#define CBAR_S1_BYPASS_S2       (1u << 16)
#define CBAR_BPSHCFG_NSH        (3u << 8)
#define CBAR_MEMATTR_WB         (0xfu << 12)
#define CBAR_CHECKED            ((3u << 16) | (0xfu << 12) | (3u << 8))
#define CBA2R_VA64              (1u << 0)
#define TCR2_SEP_UPSTREAM       (7u << 15)
#define TCR2_AS                 (1u << 4)
#define CB_TCR_T0SZ_32             32u
#define CB_TCR_SH0_OUTER           (2u << 12)
#define CB_TCR_EPD1                (1u << 23)      /* IRGN0 = ORGN0 = 0: non-cacheable walks; TG0 = 0: 4 KiB */
#define MAIR0_VALUE             0xf404ff44u     /* attr 0 Normal NC, 1 WB, 2 Device nGnRE, 3 outer WB */
#define SCTLR_M                 (1u << 0)
#define SCTLR_TRE               (1u << 1)
#define SCTLR_AFE               (1u << 2)
#define SCTLR_CFRE              (1u << 5)
#define SCTLR_ASIDPNE           (1u << 12)
#define SCTLR_VALUE             (SCTLR_M | SCTLR_TRE | SCTLR_AFE | SCTLR_CFRE | SCTLR_ASIDPNE)
/* Long-descriptor entries, 4 KiB granule */
#define PTE_TABLE               3u
#define PTE_PAGE                3u
#define PTE_AP_UNPRIV           (1u << 6)       /* read/write at any privilege; AP[2] = 0 */
#define PTE_SH_OUTER            (2u << 8)
#define PTE_AF                  (1u << 10)
#define PTE_NG                  (1u << 11)
#define PTE_PAGE_RW             (PTE_PAGE | PTE_AP_UNPRIV | PTE_SH_OUTER | PTE_AF | PTE_NG)    /* attr 0 */
#define PTE_ADDR_MASK           0x0000fffffffff000ull
#define PT_SIZE                 0x1000u

static vm_offset_t smmu;
static uint32_t smmu_pgsize, smmu_cb_base;
static unsigned smmu_nsmr, smmu_ncb, smmu_oas;
static bool smmu_ok;

static uint64_t
smmu_rd64(vm_offset_t base, uint32_t off)
{
	uint64_t v = *(volatile uint64_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static void
smmu_wr64(vm_offset_t base, uint32_t off, uint64_t val)
{
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint64_t *)(base + off) = val;
	__asm__ volatile("dsb osh" ::: "memory");
}

static vm_offset_t
smmu_cb(unsigned n)
{
	return smmu + smmu_cb_base + n * smmu_pgsize;
}

static unsigned
smmu_oas_bits(unsigned oas)
{
	static const unsigned bits[] = { 32, 36, 40, 42, 44, 48 };
	return oas < 6 ? bits[oas] : 48;
}

bool
sweet_smmu_survey(void)
{
	if (!smmu && !(smmu = ml_io_map(SMMU_PA, SMMU_SIZE))) {
		printf("SWEET SMMU: mapping failed\n");
		return false;
	}
	uint32_t idr0 = gpu_rd(smmu, SMMU_IDR0), idr1 = gpu_rd(smmu, SMMU_IDR1), idr2 = gpu_rd(smmu, SMMU_IDR2);
	uint32_t scr0 = gpu_rd(smmu, SMMU_sCR0);
	printf("SWEET SMMU: IDR0 0x%08x IDR1 0x%08x IDR2 0x%08x IDR7 0x%08x sCR0 0x%08x sACR 0x%08x sGFSR 0x%08x\n",
	    idr0, idr1, idr2, gpu_rd(smmu, SMMU_IDR7), scr0, gpu_rd(smmu, SMMU_sACR), gpu_rd(smmu, SMMU_sGFSR));
	smmu_nsmr = idr0 & 0xffu;
	smmu_ncb = idr1 & 0xffu;
	smmu_pgsize = (idr1 & IDR1_PAGESIZE) ? 0x10000u : 0x1000u;
	smmu_cb_base = (1u << (((idr1 >> 28) & 7u) + 1u)) * smmu_pgsize;
	smmu_oas = (idr2 >> 4) & 0xfu;
	printf("SWEET SMMU: %u stream-match groups, %u SID bits, %u context banks of %#x from +%#x, OAS %u bits, "
	    "stage 1 %s, 4K tables %s, %s\n", smmu_nsmr, (idr0 >> 9) & 0xfu, smmu_ncb, smmu_pgsize, smmu_cb_base,
	    smmu_oas_bits(smmu_oas), (idr0 & IDR0_S1TS) ? "yes" : "NO", (idr2 & IDR2_PTFS_4K) ? "yes" : "NO",
	    (scr0 & sCR0_CLIENTPD) ? "OFF (CLIENTPD: all streams bypass)" : "on");
	if (!smmu_nsmr || smmu_nsmr > 128 || !smmu_ncb || smmu_cb_base + smmu_ncb * smmu_pgsize > SMMU_SIZE) {
		printf("SWEET SMMU: unexpected geometry; not used\n");
		return false;
	}
	for (unsigned i = 0; i < smmu_nsmr; ++i) {
		uint32_t smr = gpu_rd(smmu, SMMU_SMR(i)), s2cr = gpu_rd(smmu, SMMU_S2CR(i));
		printf("SWEET SMMU:   SMR%-3u 0x%08x %s id %#x mask %#x  S2CR 0x%08x (%s, bank %u)\n", i, smr,
		    (smr & SMR_VALID) ? "valid" : "free ", smr & SMR_ID_MASK, (smr >> 16) & SMR_ID_MASK, s2cr,
		    S2CR_TYPE(s2cr) == 0 ? "translate" : S2CR_TYPE(s2cr) == 1 ? "bypass" : "fault", S2CR_CBNDX(s2cr));
	}
	vm_offset_t gr1 = smmu + smmu_pgsize;
	for (unsigned n = 0; n < smmu_ncb; ++n) {
		vm_offset_t cb = smmu_cb(n);
		printf("SWEET SMMU:   CB%u CBAR 0x%08x CBA2R 0x%08x SCTLR 0x%08x TCR 0x%08x TCR2 0x%08x TTBR0 0x%016llx "
		    "MAIR0 0x%08x FSR 0x%08x\n", n, gpu_rd(gr1, SMMU_CBAR(n)), gpu_rd(gr1, SMMU_CBA2R(n)),
		    gpu_rd(cb, CB_SCTLR), gpu_rd(cb, CB_TCR), gpu_rd(cb, CB_TCR2), smmu_rd64(cb, CB_TTBR0),
		    gpu_rd(cb, CB_MAIR0), gpu_rd(cb, CB_FSR));
	}
	smmu_ok = (idr0 & IDR0_S1TS) && (idr2 & IDR2_PTFS_4K);
	return true;
}

void
sweet_smmu_pt_init(struct sweet_smmu_pt *pt, vm_offset_t va, uint64_t pa, uint32_t size)
{
	pt->va = va;
	pt->pa = pa;
	pt->size = size;
	pt->used = PT_SIZE;     /* the level-1 table */
	for (uint32_t i = 0; i < PT_SIZE; i += 8) {
		*(volatile uint64_t *)(va + i) = 0;
	}
}

/* The table a descriptor points to, made (zeroed) if the slot is empty; 0 if out of room or not ours. */
static vm_offset_t
smmu_pt_next(struct sweet_smmu_pt *pt, vm_offset_t slot)
{
	uint64_t d = *(volatile uint64_t *)slot;
	if ((d & 3u) == PTE_TABLE) {
		uint64_t tpa = d & PTE_ADDR_MASK;
		return tpa >= pt->pa && tpa - pt->pa < pt->used ? pt->va + (vm_offset_t)(tpa - pt->pa) : 0;
	}
	if ((d & 1u) || pt->used + PT_SIZE > pt->size) {
		return 0;
	}
	vm_offset_t t = pt->va + pt->used;
	for (uint32_t i = 0; i < PT_SIZE; i += 8) {
		*(volatile uint64_t *)(t + i) = 0;
	}
	*(volatile uint64_t *)slot = (pt->pa + pt->used) | PTE_TABLE;
	pt->used += PT_SIZE;
	return t;
}

/* 4 KiB pages iova..iova+len -> pa..; refuses a page that is already mapped. */
bool
sweet_smmu_map(struct sweet_smmu_pt *pt, uint32_t iova, uint64_t pa, uint32_t len)
{
	if ((iova | pa | len) & (PT_SIZE - 1u) || (uint64_t)iova + len > (1ull << CB_TCR_T0SZ_32)) {
		printf("SWEET SMMU: map %#x -> %#llx len %#x: not page-aligned or above 4 GiB\n", iova, pa, len);
		return false;
	}
	for (uint32_t off = 0; off < len; off += PT_SIZE) {
		uint32_t va = iova + off;
		vm_offset_t l2 = smmu_pt_next(pt, pt->va + 8u * ((va >> 30) & 3u));
		vm_offset_t l3 = l2 ? smmu_pt_next(pt, l2 + 8u * ((va >> 21) & 0x1ffu)) : 0;
		if (!l3) {
			printf("SWEET SMMU: map %#x: no room for another table (%#x of %#x used)\n", va, pt->used, pt->size);
			return false;
		}
		vm_offset_t slot = l3 + 8u * ((va >> 12) & 0x1ffu);
		if (*(volatile uint64_t *)slot & 1u) {
			printf("SWEET SMMU: map %#x: already mapped\n", va);
			return false;
		}
		*(volatile uint64_t *)slot = (pa + off) | PTE_PAGE_RW;
	}
	__asm__ volatile("dsb sy" ::: "memory");
	printf("SWEET SMMU: mapped %#010x..%#010x -> %#llx (%#x of %#x table bytes used)\n", iova, iova + len - 1u,
	    pa, pt->used, pt->size);
	return true;
}

static bool
smmu_tlb_sync(vm_offset_t base, uint32_t sync, uint32_t status)
{
	gpu_wr(base, sync, 0);
	return gpu_poll(base, status, TLBSTATUS_ACTIVE, 0, 1000);
}

static bool
smmu_check(const char *what, uint32_t got, uint32_t want, uint32_t mask)
{
	if ((got & mask) == (want & mask)) {
		return true;
	}
	printf("SWEET SMMU:   %s reads 0x%08x, wrote 0x%08x (mask 0x%08x): MISMATCH\n", what, got, want, mask);
	return false;
}

/* A bank no valid stream-match group translates through and with translation off; not bank 0. */
static int
smmu_free_bank(void)
{
	for (unsigned n = smmu_ncb - 1u; n >= 1u; --n) {
		bool used = (gpu_rd(smmu_cb(n), CB_SCTLR) & SCTLR_M) != 0;
		for (unsigned i = 0; i < smmu_nsmr && !used; ++i) {
			uint32_t s2cr = gpu_rd(smmu, SMMU_S2CR(i));
			used = (gpu_rd(smmu, SMMU_SMR(i)) & SMR_VALID) && S2CR_TYPE(s2cr) == 0 && S2CR_CBNDX(s2cr) == n;
		}
		if (!used) {
			return (int)n;
		}
	}
	return -1;
}

/*
 * Translate streams sid..(sid | mask) through a free context bank whose
 * level-1 table is at ttbr. Returns the bank, or -1 with the SMMU as it was.
 */
int
sweet_smmu_attach(uint32_t sid, uint32_t mask, uint64_t ttbr)
{
	if (!smmu_ok) {
		printf("SWEET SMMU: not surveyed or no stage 1 with 4K tables; not attached\n");
		return -1;
	}
	unsigned pabits = smmu_oas_bits(smmu_oas);
	if ((ttbr & (PT_SIZE - 1u)) || (pabits < 64 && (ttbr >> pabits))) {
		printf("SWEET SMMU: tables at %#llx are unaligned or above the %u-bit output range\n", ttbr, pabits);
		return -1;
	}
	int free_smr = -1;
	for (unsigned i = 0; i < smmu_nsmr; ++i) {
		uint32_t smr = gpu_rd(smmu, SMMU_SMR(i));
		uint32_t id = smr & SMR_ID_MASK, m = (smr >> 16) & SMR_ID_MASK;
		if (!(smr & SMR_VALID)) {
			free_smr = free_smr < 0 ? (int)i : free_smr;
		} else if (((sid ^ id) & ~(m | mask) & SMR_ID_MASK) == 0) {
			printf("SWEET SMMU: SMR%u (0x%08x) already matches stream %#x/%#x; not attached\n", i, smr, sid, mask);
			return -1;
		}
	}
	int bank = smmu_free_bank();
	if (free_smr < 0 || bank < 0) {
		printf("SWEET SMMU: no free %s; not attached\n", free_smr < 0 ? "stream-match group" : "context bank");
		return -1;
	}
	unsigned n = (unsigned)bank, g = (unsigned)free_smr;
	vm_offset_t cb = smmu_cb(n), gr1 = smmu + smmu_pgsize;
	printf("SWEET SMMU: streams %#x/%#x -> SMR%u, bank %u, tables at %#llx\n", sid, mask, g, n, ttbr);

	const uint32_t cbar = CBAR_S1_BYPASS_S2 | CBAR_BPSHCFG_NSH | CBAR_MEMATTR_WB;
	const uint32_t tcr2 = TCR2_SEP_UPSTREAM | TCR2_AS | (smmu_oas < 5 ? smmu_oas : 5u);
	const uint32_t tcr = CB_TCR_EPD1 | CB_TCR_SH0_OUTER | CB_TCR_T0SZ_32;
	const uint64_t ttbr0 = ttbr | (uint64_t)n << 48;        /* ASID = bank */
	gpu_wr(cb, CB_SCTLR, 0);
	gpu_wr(gr1, SMMU_CBA2R(n), CBA2R_VA64);
	gpu_wr(gr1, SMMU_CBAR(n), cbar);
	gpu_wr(cb, CB_TCR2, tcr2);
	gpu_wr(cb, CB_TCR, tcr);
	gpu_wr(cb, CB_MAIR0, MAIR0_VALUE);
	smmu_wr64(cb, CB_TTBR0, ttbr0);
	gpu_wr(cb, CB_CONTEXTIDR, n);
	gpu_wr(cb, CB_TLBIALL, 0);
	bool ok = smmu_tlb_sync(cb, CB_TLBSYNC, CB_TLBSTATUS);
	if (!ok) {
		printf("SWEET SMMU:   bank TLB sync timed out\n");
	}
	gpu_wr(cb, CB_FSR, gpu_rd(cb, CB_FSR));         /* write-one-to-clear stale faults */
	gpu_wr(cb, CB_SCTLR, SCTLR_VALUE);
	ok = smmu_check("CBA2R", gpu_rd(gr1, SMMU_CBA2R(n)), CBA2R_VA64, CBA2R_VA64) && ok;
	ok = smmu_check("CBAR", gpu_rd(gr1, SMMU_CBAR(n)), cbar, CBAR_CHECKED) && ok;
	ok = smmu_check("TCR2", gpu_rd(cb, CB_TCR2), tcr2, 0xffffffffu) && ok;
	ok = smmu_check("TCR", gpu_rd(cb, CB_TCR), tcr, 0xffffffffu) && ok;
	ok = smmu_check("MAIR0", gpu_rd(cb, CB_MAIR0), MAIR0_VALUE, 0xffffffffu) && ok;
	uint64_t t = smmu_rd64(cb, CB_TTBR0);
	ok = smmu_check("TTBR0 lo", (uint32_t)t, (uint32_t)ttbr0, 0xffffffffu) && ok;
	ok = smmu_check("TTBR0 hi", (uint32_t)(t >> 32), (uint32_t)(ttbr0 >> 32), 0xffffffffu) && ok;
	ok = smmu_check("SCTLR", gpu_rd(cb, CB_SCTLR), SCTLR_VALUE, SCTLR_VALUE | (1u << 6) | (1u << 7)) && ok;
	if (!ok) {
		gpu_wr(cb, CB_SCTLR, 0);
		printf("SWEET SMMU: bank %u did not take; translation off again, no stream points at it\n", n);
		return -1;
	}

	uint32_t s2cr_old = gpu_rd(smmu, SMMU_S2CR(g));
	const uint32_t s2cr = n, smr = SMR_VALID | (mask & SMR_ID_MASK) << 16 | (sid & SMR_ID_MASK);
	gpu_wr(smmu, SMMU_S2CR(g), s2cr);
	gpu_wr(smmu, SMMU_SMR(g), smr);
	uint32_t scr0 = gpu_rd(smmu, SMMU_sCR0);
	ok = smmu_check("S2CR", gpu_rd(smmu, SMMU_S2CR(g)), s2cr, S2CR_CHECKED) &&
	    smmu_check("SMR", gpu_rd(smmu, SMMU_SMR(g)), smr, 0xffffffffu);
	if (ok && (scr0 & sCR0_CLIENTPD)) {
		uint32_t on = scr0 & ~(sCR0_CLIENTPD | sCR0_USFCFG);
		gpu_wr(smmu, SMMU_sCR0, on);
		ok = smmu_check("sCR0", gpu_rd(smmu, SMMU_sCR0), on, sCR0_CLIENTPD | sCR0_USFCFG);
		printf("SWEET SMMU:   sCR0 0x%08x -> 0x%08x (SMMU on, unmatched streams bypass)\n", scr0,
		    gpu_rd(smmu, SMMU_sCR0));
	}
	gpu_wr(smmu, SMMU_TLBIALLNSNH, 0);
	if (!smmu_tlb_sync(smmu, SMMU_sTLBGSYNC, SMMU_sTLBGSTATUS)) {
		printf("SWEET SMMU:   global TLB sync timed out\n");
		ok = false;
	}
	if (!ok) {
		gpu_wr(smmu, SMMU_SMR(g), 0);
		gpu_wr(smmu, SMMU_S2CR(g), s2cr_old);
		gpu_wr(cb, CB_SCTLR, 0);
		printf("SWEET SMMU: attach undone: SMR%u 0x%08x S2CR 0x%08x, bank %u off\n", g,
		    gpu_rd(smmu, SMMU_SMR(g)), gpu_rd(smmu, SMMU_S2CR(g)), n);
		return -1;
	}
	printf("SWEET SMMU: bank %u: SCTLR 0x%08x TCR 0x%08x TCR2 0x%08x TTBR0 0x%016llx; SMR%u 0x%08x S2CR 0x%08x "
	    "sCR0 0x%08x\n", n, gpu_rd(cb, CB_SCTLR), gpu_rd(cb, CB_TCR), gpu_rd(cb, CB_TCR2), smmu_rd64(cb, CB_TTBR0),
	    g, gpu_rd(smmu, SMMU_SMR(g)), gpu_rd(smmu, SMMU_S2CR(g)), gpu_rd(smmu, SMMU_sCR0));
	return bank;
}

/* Logs the global and (cb >= 0) the bank's fault registers; returns sGFSR | FSR, 0 if clean. */
uint32_t
sweet_smmu_faults(int cb, const char *when)
{
	if (!smmu) {
		return 0;
	}
	uint32_t gfsr = gpu_rd(smmu, SMMU_sGFSR), fsr = 0;
	if (cb >= 0 && (unsigned)cb < smmu_ncb) {
		vm_offset_t b = smmu_cb((unsigned)cb);
		fsr = gpu_rd(b, CB_FSR);
		printf("SWEET SMMU: %s: sGFSR 0x%08x sGFSYNR0 0x%08x sGFAR %#llx; bank %d FSR 0x%08x FAR %#llx FSYNR0 0x%08x\n",
		    when, gfsr, gpu_rd(smmu, SMMU_sGFSYNR0), smmu_rd64(smmu, SMMU_sGFAR), cb, fsr, smmu_rd64(b, CB_FAR),
		    gpu_rd(b, CB_FSYNR0));
	} else {
		printf("SWEET SMMU: %s: sGFSR 0x%08x sGFSYNR0 0x%08x sGFAR %#llx\n", when, gfsr,
		    gpu_rd(smmu, SMMU_sGFSYNR0), smmu_rd64(smmu, SMMU_sGFAR));
	}
	return gfsr | fsr;
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
