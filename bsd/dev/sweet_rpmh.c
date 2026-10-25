/*
 * SWEET PM6150L L11C bring-up; see report.md for DT and source evidence.
 * Sole XNU apps-DRV owner, called from thread context with deep idle disabled.
 * No sleep/wake programming, SPMI writes, GIC handler, or timeout recovery.
 * GPU stage 2A (sweet_gpu.c) adds one-command ARC level votes for gfx.lvl
 * and mx.lvl through the same ACTIVE TCS path and the same safety rules.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
int sweet_rpmh_vreg_enable(const char *res, unsigned mv);
int sweet_rpmh_arc_vote(const char *res, unsigned vlvl, unsigned *hlvl);
int sweet_rpmh_cmddb_dump(void);
int sweet_rpmh_cmddb_query(const char *res, unsigned slave, uint32_t *addr, unsigned *vlvl, unsigned nmax, unsigned *n);

/* Live downstream DT has two positional reg entries, no reg-names. */
#define RH_RSC_PA       0x18220000u
#define RH_TCS_PA       0x18220d00u
#define RH_TCS_SIZE     0x3000u
#define RH_DICT_PA      0x0c3f000cu
#define RH_DRV          2u
#define RH_ACTIVE_N     2u       /* DT order: ACTIVE 2, SLEEP 3, WAKE 3 */
#define RH_NORMAL_N     8u       /* hidden CONTROL TCS is not in this bank */
#define RH_TCS_STRIDE   0x2a0u
#define RH_CMD_STRIDE   0x14u
#define RH_IRQ_EN       0x00u
#define RH_IRQ_STATUS   0x04u
#define RH_IRQ_CLEAR    0x08u
#define RH_WAIT         0x10u
#define RH_CONTROL      0x14u
#define RH_STATUS       0x18u
#define RH_ENABLE       0x1cu
#define RH_MSGID        0x30u
#define RH_ADDR         0x34u
#define RH_DATA         0x38u
#define RH_CMD_STATUS   0x3cu
#define RH_AMC_EN       (1u << 16)
#define RH_TRIGGER      (1u << 24)
#define RH_ISSUED       (1u << 8)
#define RH_COMPLETE     (1u << 16)
#define RH_WRITE_ACK    (8u | (1u << 8) | (1u << 16))
#define RH_DB_MAGIC     0x0c0330dbu
#define RH_DB_HEADER    144u
#define RH_DB_ENTRY     24u
#define RH_DB_MAX       0x100000u /* policy bound, not a DT size */
#define RH_ARC          3u
#define RH_VRM          4u
#define RH_BCM          5u
#define RH_ARC_LEVELS   16u      /* Linux rpmhpd RPMH_ARC_MAX_LEVELS */
#define RH_ARC_VLVL_MAX 64u      /* RPMh LOW_SVS: stage 2A votes only the lowest active corner */
#define RH_CMDS_MAX     3u
#define RH_TIMEOUT_US   100000u

struct rh_cmd {
	uint32_t addr, data;
};

static vm_offset_t rh_rsc, rh_tcs, rh_dict, rh_db;
static uint32_t rh_db_size;
static unsigned rh_ncpt;
static unsigned rh_lock;
static bool rh_poisoned;

static uint32_t
rh_read(vm_offset_t base, uint32_t off)
{
	uint32_t v = *(volatile uint32_t *)(base + off);
	__asm__ volatile("dmb oshld" ::: "memory");
	return v;
}

static void
rh_write(vm_offset_t base, uint32_t off, uint32_t value)
{
	__asm__ volatile("dmb oshst" ::: "memory");
	*(volatile uint32_t *)(base + off) = value;
	__asm__ volatile("dsb osh" ::: "memory");
}

static bool
rh_sync(uint32_t off, uint32_t value)
{
	rh_write(rh_tcs, off, value);
	for (unsigned i = 0; i < 1000; ++i) {
		if (rh_read(rh_tcs, off) == value) {
			return true;
		}
		delay(1);
	}
	printf("SWEET RPMH: readback timeout off=%#x want=%#x got=%#x\n",
	    off, value, rh_read(rh_tcs, off));
	return false;
}

/* DB is little endian; avoid packed structs and unaligned Device loads. */
static uint16_t
rh_db16(uint32_t off)
{
	uint32_t v = rh_read(rh_db, off & ~3u);
	return (uint16_t)(v >> (8u * (off & 3u)));
}

static bool
rh_range(uint32_t off, uint32_t size)
{
	return off <= rh_db_size && size <= rh_db_size - off;
}

static int
rh_map(void)
{
	if (!rh_rsc) {
		rh_rsc = ml_io_map(RH_RSC_PA, 0x100);
	}
	if (!rh_tcs) {
		rh_tcs = ml_io_map(RH_TCS_PA, RH_TCS_SIZE);
	}
	if (!rh_dict) {
		rh_dict = ml_io_map(RH_DICT_PA, 8);
	}
	if (!rh_rsc || !rh_tcs || !rh_dict) {
		printf("SWEET RPMH: RSC/locator mapping failed\n");
		return ENOMEM;
	}
	uint32_t id = rh_read(rh_rsc, 0);
	uint32_t solver = rh_read(rh_rsc, 4);
	uint32_t config = rh_read(rh_rsc, 0x0c);
	unsigned major = (id >> 16) & 0xffu;
	unsigned count = (config >> (6u * RH_DRV)) & 0x3fu;
	rh_ncpt = (config >> 27) & 0x1fu;
	printf("SWEET RPMH: rsc=%#x tcs=%#x DRV=%u id=%#x solver=%#x config=%#x TCS=%u ncpt=%u\n",
	    RH_RSC_PA, RH_TCS_PA, RH_DRV, id, solver, config, count, rh_ncpt);
	/* This client implements the pre-v3 layout only; do not guess v3 offsets. */
	if ((major != 1 && major != 2) || config == 0xffffffffu ||
	    solver == 0xffffffffu || count < RH_NORMAL_N ||
	    count > RH_TCS_SIZE / RH_TCS_STRIDE || rh_ncpt < 3 || rh_ncpt > 16) {
		printf("SWEET RPMH: unsupported RSC geometry/revision\n");
		return ENODEV;
	}
	if (!rh_db) {
		uint32_t pa = rh_read(rh_dict, 0);
		uint32_t size = rh_read(rh_dict, 4);
		printf("SWEET RPMH: cmd-db locator=%#x pa=%#x size=%#x\n", RH_DICT_PA, pa, size);
		/* Firmware supplies the reserved RAM address; never scan physical RAM. */
		if (pa < 0x80000000u || (pa & 3u) || (size & 3u) ||
		    size < RH_DB_HEADER || size > RH_DB_MAX ||
		    (uint64_t)pa + size > 0x100000000ULL) {
			printf("SWEET RPMH: invalid cmd-db locator\n");
			return EINVAL;
		}
		rh_db = ml_io_map(pa, size);
		if (!rh_db) {
			return ENOMEM;
		}
		rh_db_size = size;
	}
	uint32_t db_version = rh_read(rh_db, 0);
	uint32_t magic = rh_read(rh_db, 4);
	uint32_t flags = rh_read(rh_db, 140);
	printf("SWEET RPMH: cmd-db version=%#x magic=%#x checksum=%#x flags=%#x\n",
	    db_version, magic, rh_read(rh_db, 136), flags);
	/* Linux only checks magic; version 1 is an intentionally stricter policy. */
	if (magic != RH_DB_MAGIC || db_version != 1 || (flags & 1u)) {
		printf("SWEET RPMH: unsupported/standalone cmd-db\n");
		return EINVAL;
	}
	return 0;
}

/* want is RH_VRM, RH_ARC or RH_BCM; aux_off/aux_len (optional) locate the entry's aux data. */
static int
rh_lookup(const char *res, unsigned want, uint32_t *address, uint32_t *aux_off, unsigned *aux_len)
{
	uint32_t key[2] = {0, 0};
	for (unsigned i = 0; i < 8 && res[i]; ++i) {
		key[i / 4] |= (uint32_t)(uint8_t)res[i] << (8u * (i % 4));
	}
	unsigned matches = 0;
	for (unsigned i = 0; i < 8; ++i) {
		uint32_t h = 8u + 16u * i;
		unsigned slave = rh_db16(h);
		if (!slave) {
			break;
		}
		uint32_t entries = RH_DB_HEADER + rh_db16(h + 2);
		uint32_t aux = RH_DB_HEADER + rh_db16(h + 4);
		unsigned count = rh_db16(h + 6);
		if (slave < 3 || slave > 5 || (entries & 3u) ||
		    !rh_range(entries, count * RH_DB_ENTRY) || !rh_range(aux, 0)) {
			printf("SWEET RPMH: malformed cmd-db slave header %u\n", i);
			return EINVAL;
		}
		for (unsigned j = 0; j < count; ++j) {
			uint32_t e = entries + RH_DB_ENTRY * j;
			if (rh_read(rh_db, e) != key[0] || rh_read(rh_db, e + 4) != key[1]) {
				continue;
			}
			uint32_t addr = rh_read(rh_db, e + 16);
			uint32_t off = aux + rh_db16(e + 22);
			unsigned len = rh_db16(e + 20);
			/* BCM addresses are 4 apart (CN0 0x500a8); ARC and VRM 16. */
			if (slave != want || (addr >> 16) != want || (addr & (want == RH_BCM ? 3u : 15u)) ||
			    !rh_range(off, len)) {
				printf("SWEET RPMH: invalid %s entry for %s\n",
				    want == RH_ARC ? "ARC" : want == RH_BCM ? "BCM" : "VRM", res);
				return EINVAL;
			}
			*address = addr;
			if (aux_off) {
				*aux_off = off;
				*aux_len = len;
			}
			++matches;
			printf("SWEET RPMH: %s addr=%#x slave=%u aux-version=%#x priority=%#x:%#x\n",
			    res, addr, slave, rh_db16(h + 8),
			    rh_read(rh_db, e + 8), rh_read(rh_db, e + 12));
		}
	}
	if (matches != 1) {
		printf("SWEET RPMH: %s lookup matches=%u; refusing\n", res, matches);
		return ENODEV;
	}
	return 0;
}

static bool
rh_free(unsigned t)
{
	uint32_t b = t * RH_TCS_STRIDE;
	/* Idle alone is insufficient: a sleep/wake payload can be armed and idle. */
	return rh_read(rh_tcs, b + RH_STATUS) != 0 &&
	    rh_read(rh_tcs, b + RH_STATUS) != 0xffffffffu &&
	    rh_read(rh_tcs, b + RH_CONTROL) == 0 &&
	    rh_read(rh_tcs, b + RH_ENABLE) == 0 &&
	    rh_read(rh_tcs, b + RH_WAIT) == 0 &&
	    !(rh_read(rh_tcs, RH_IRQ_STATUS) & (1u << t));
}

/* One ACTIVE request of n commands to one resource (cmds[0].addr is its base). */
static int
rh_send(const struct rh_cmd *cmds, unsigned n, const char *what)
{
	if (n == 0 || n > RH_CMDS_MAX || n > rh_ncpt) {
		return EINVAL;
	}
	uint32_t address = cmds[0].addr & ~15u, mask = (1u << n) - 1u;
	int chosen = -1;
	for (unsigned t = 0; t < RH_ACTIVE_N; ++t) {
		uint32_t b = t * RH_TCS_STRIDE;
		uint32_t enabled = rh_read(rh_tcs, b + RH_ENABLE);
		printf("SWEET RPMH: ACTIVE TCS%u idle=%#x control=%#x enable=%#x wait=%#x\n",
		    t, rh_read(rh_tcs, b + RH_STATUS), rh_read(rh_tcs, b + RH_CONTROL),
		    enabled, rh_read(rh_tcs, b + RH_WAIT));
		if (enabled & ~((1u << rh_ncpt) - 1u)) {
			return EIO;
		}
		for (unsigned c = 0; c < rh_ncpt; ++c) {
			if ((enabled & (1u << c)) &&
			    (rh_read(rh_tcs, b + RH_ADDR + c * RH_CMD_STRIDE) & ~15u) == address) {
				printf("SWEET RPMH: conflicting inherited %s request in TCS%u\n", what, t);
				return EBUSY;
			}
		}
		if (chosen < 0 && rh_free(t)) {
			chosen = (int)t;
		}
	}
	if (chosen < 0) {
		printf("SWEET RPMH: no empty idle ACTIVE TCS; leaving inherited state intact\n");
		return EBUSY;
	}
	unsigned t = (unsigned)chosen;
	uint32_t b = t * RH_TCS_STRIDE, bit = 1u << t;
	uint32_t irq_en = rh_read(rh_tcs, RH_IRQ_EN);
	if (irq_en == 0xffffffffu || !rh_free(t)) {
		return EBUSY;
	}
	/* Only this TCS's source is masked. Other sources/CPU IRQs remain intact. */
	if (!rh_sync(RH_IRQ_EN, irq_en & ~bit)) {
		rh_poisoned = true;
		return EIO;
	}
	if (!rh_free(t)) {
		/* No payload was touched. Do not acknowledge somebody else's IRQ. */
		if (!rh_sync(RH_IRQ_EN, (rh_read(rh_tcs, RH_IRQ_EN) & ~bit) | (irq_en & bit))) {
			rh_poisoned = true;
		}
		return EBUSY;
	}
	/* From here an uncertain write/timeout quarantines our slot for this boot. */
	rh_poisoned = true;
	printf("SWEET RPMH: TCS%u ACTIVE %s, %u command(s), irq-enable=%#x\n", t, what, n, irq_en);
	for (unsigned c = 0; c < n; ++c) {
		uint32_t cmd = b + c * RH_CMD_STRIDE;
		printf("SWEET RPMH:   cmd%u addr=%#x data=%#x\n", c, cmds[c].addr, cmds[c].data);
		if (!rh_sync(cmd + RH_MSGID, RH_WRITE_ACK) ||
		    !rh_sync(cmd + RH_ADDR, cmds[c].addr) ||
		    !rh_sync(cmd + RH_DATA, cmds[c].data)) {
			return EIO;
		}
	}
	if (!rh_sync(b + RH_WAIT, mask) || !rh_sync(b + RH_ENABLE, mask) ||
	    !rh_sync(b + RH_CONTROL, 0) || !rh_sync(b + RH_CONTROL, RH_AMC_EN)) {
		return EIO;
	}
	rh_write(rh_tcs, b + RH_CONTROL, RH_AMC_EN | RH_TRIGGER);
	uint32_t irq = 0, idle = 0, status[RH_CMDS_MAX] = {0, 0, 0};
	bool done = false, progress = false;
	for (unsigned us = 0; us < RH_TIMEOUT_US; ++us) {
		irq = rh_read(rh_tcs, RH_IRQ_STATUS);
		idle = rh_read(rh_tcs, b + RH_STATUS);
		bool complete = true;
		for (unsigned c = 0; c < n; ++c) {
			status[c] = rh_read(rh_tcs, b + RH_CMD_STATUS + c * RH_CMD_STRIDE);
			complete &= status[c] != 0xffffffffu &&
			    (status[c] & (RH_ISSUED | RH_COMPLETE)) == (RH_ISSUED | RH_COMPLETE);
		}
		/* A fresh IRQ or observed execution transition excludes stale CMD_STATUS. */
		progress |= idle == 0 || !complete;
		if (idle && idle != 0xffffffffu && irq != 0xffffffffu &&
		    ((irq & bit) || progress) && complete) {
			done = true;
			break;
		}
		delay(1);
	}
	printf("SWEET RPMH: TCS%u done=%d irq=%#x idle=%#x control=%#x cmd-status=%#x/%#x/%#x\n",
	    t, done, irq, idle, rh_read(rh_tcs, b + RH_CONTROL), status[0], status[1], status[2]);
	if (!done) {
		printf("SWEET RPMH: timeout; TCS quarantined, IRQ remains masked; no reset/retry\n");
		return ETIMEDOUT;
	}
	/* Proven completion: clear trigger before mode, then free only our commands. */
	if (!rh_sync(b + RH_CONTROL, RH_AMC_EN) || !rh_sync(b + RH_CONTROL, 0) ||
	    !rh_sync(b + RH_ENABLE, 0) || !rh_sync(b + RH_WAIT, 0)) {
		return EIO;
	}
	rh_write(rh_tcs, RH_IRQ_CLEAR, bit); /* W1C: never clear other TCS bits */
	if ((rh_read(rh_tcs, RH_IRQ_STATUS) & bit) ||
	    !rh_sync(RH_IRQ_EN, (rh_read(rh_tcs, RH_IRQ_EN) & ~bit) | (irq_en & bit))) {
		return EIO;
	}
	rh_poisoned = false;
	printf("SWEET RPMH: %s ACTIVE vote complete (rail voltage requires hardware verification)\n", what);
	return 0;
}

int
sweet_rpmh_vreg_enable(const char *res, unsigned mv)
{
	/* Board-specific helper: do not apply PMIC5-LDO policy to arbitrary resources. */
	if (!res || strcmp(res, "ldoc11") || mv < 2950 || mv > 3400) {
		printf("SWEET RPMH: expected ldoc11 and 2950..3400 mV\n");
		return EINVAL;
	}
	if (__atomic_exchange_n(&rh_lock, 1, __ATOMIC_ACQUIRE)) {
		return EBUSY;
	}
	int error = EIO;
	uint32_t address = 0;
	if (rh_poisoned) {
		printf("SWEET RPMH: previous uncertain transaction; refusing further access\n");
		goto out;
	}
	error = rh_map();
	if (!error) {
		error = rh_lookup(res, RH_VRM, &address, NULL, NULL);
	}
	if (!error) {
		/* HPM (raw 7) is supported by this PMIC5 LDO; enable comes last. */
		const struct rh_cmd cmds[3] = {
			{ address + 0, mv }, { address + 8, 7 }, { address + 4, 1 },
		};
		error = rh_send(cmds, 3, "L11C");
	}
out:
	if (error) {
		printf("SWEET RPMH: enable failed errno=%d\n", error);
	}
	__atomic_store_n(&rh_lock, 0, __ATOMIC_RELEASE);
	return error;
}

/*
 * ARC level for a requested vlvl, as Linux rpmhpd maps a corner: the first
 * hlvl whose vlvl is >= the request. The aux table must rise strictly.
 */
static int
rh_arc_level(const char *res, uint32_t off, unsigned len, unsigned vlvl, unsigned *hlvl)
{
	unsigned prev = 0;
	for (unsigned i = 0; !(off & 1u) && i < len / 2u && i < RH_ARC_LEVELS; ++i) {
		unsigned v = rh_db16(off + 2u * i);
		if (i && v <= prev) {
			break;
		}
		if (v >= vlvl) {
			printf("SWEET RPMH: %s vlvl %u -> hlvl %u (vlvl %u)\n", res, vlvl, i, v);
			*hlvl = i;
			return 0;
		}
		prev = v;
	}
	printf("SWEET RPMH: %s has no level >= vlvl %u; refusing\n", res, vlvl);
	return EINVAL;
}

int
sweet_rpmh_arc_vote(const char *res, unsigned vlvl, unsigned *hlvl)
{
	/* GPU rails only, at the lowest active corner: a wrong vote cannot over-volt. */
	if (!res || (strcmp(res, "gfx.lvl") && strcmp(res, "mx.lvl")) || vlvl == 0 ||
	    vlvl > RH_ARC_VLVL_MAX) {
		printf("SWEET RPMH: expected gfx.lvl or mx.lvl and vlvl 1..%u\n", RH_ARC_VLVL_MAX);
		return EINVAL;
	}
	if (__atomic_exchange_n(&rh_lock, 1, __ATOMIC_ACQUIRE)) {
		return EBUSY;
	}
	int error = EIO;
	uint32_t address = 0, aux = 0;
	unsigned len = 0, level = 0;
	if (rh_poisoned) {
		printf("SWEET RPMH: previous uncertain transaction; refusing further access\n");
		goto out;
	}
	error = rh_map();
	if (!error) {
		error = rh_lookup(res, RH_ARC, &address, &aux, &len);
	}
	if (!error) {
		error = rh_arc_level(res, aux, len, vlvl, &level);
	}
	if (!error) {
		const struct rh_cmd cmd = { address, level };
		error = rh_send(&cmd, 1, res);
	}
	if (!error && hlvl) {
		*hlvl = level;
	}
out:
	if (error) {
		printf("SWEET RPMH: %s vote failed errno=%d\n", res, error);
	}
	__atomic_store_n(&rh_lock, 0, __ATOMIC_RELEASE);
	return error;
}

/*
 * cmd-db lookup for the GMU tables (sweet_gmu.c): the address of an ARC
 * (slave 3) or BCM (slave 5) resource, and for an ARC with vlvl set its
 * level table, vlvl[hlvl], up to the first level that does not rise.
 * Read-only: no TCS is touched.
 */
int
sweet_rpmh_cmddb_query(const char *res, unsigned slave, uint32_t *addr, unsigned *vlvl, unsigned nmax, unsigned *n)
{
	if (!res || !addr || (slave != RH_ARC && slave != RH_BCM) || (vlvl && (slave != RH_ARC || !n))) {
		return EINVAL;
	}
	if (__atomic_exchange_n(&rh_lock, 1, __ATOMIC_ACQUIRE)) {
		return EBUSY;
	}
	uint32_t address = 0, aux = 0;
	unsigned len = 0, count = 0;
	int error = rh_map();
	if (!error) {
		error = rh_lookup(res, slave, &address, &aux, &len);
	}
	for (unsigned i = 0; !error && vlvl && !(aux & 1u) && i < len / 2u && i < RH_ARC_LEVELS && i < nmax; ++i) {
		unsigned v = rh_db16(aux + 2u * i);
		if (i && v <= vlvl[i - 1u]) {
			break;
		}
		vlvl[i] = v;
		count = i + 1u;
	}
	if (!error && vlvl && !count) {
		printf("SWEET RPMH: %s has no level table\n", res);
		error = EINVAL;
	}
	if (!error) {
		*addr = address;
		if (n) {
			*n = count;
		}
	}
	__atomic_store_n(&rh_lock, 0, __ATOMIC_RELEASE);
	return error;
}

/*
 * Read-only cmd-db listing for the GPU port (sweet_gpu.c): every ARC
 * resource with its level table (aux = u16 vlvl per hlvl, as rpmhpd reads
 * it), every BCM with its aux {le32 unit, le16 width, u8 vcd}, and the VRM
 * names on summary lines. Touches only the cmd-db RAM and the RSC id
 * registers rh_map() always reads; no TCS is written.
 */
static void
rh_entry_id(uint32_t e, char id[9])
{
	uint32_t w[2] = { rh_read(rh_db, e), rh_read(rh_db, e + 4) };
	for (unsigned i = 0; i < 8; ++i) {
		char c = (char)(w[i / 4] >> (8u * (i % 4)));
		id[i] = (c >= 0x20 && c < 0x7f) ? c : (c ? '?' : '\0');
	}
	id[8] = '\0';
}

static void
rh_dump_entry(unsigned slave, uint32_t e, uint32_t aux)
{
	char id[9];
	rh_entry_id(e, id);
	uint32_t addr = rh_read(rh_db, e + 16);
	uint32_t len = rh_db16(e + 20), off = aux + rh_db16(e + 22);
	if (len && (!rh_range(off, len) || (off & 1u))) {
		printf("SWEET RPMH:   %-8s addr=%#x aux len %u at %#x out of range\n", id, addr, len, off);
		return;
	}
	if (slave == RH_ARC) {
		char line[128];
		int n = snprintf(line, sizeof(line), "SWEET RPMH:   ARC %-8s addr=%#x levels:", id, addr);
		for (unsigned i = 0; i < len / 2u && i < RH_ARC_LEVELS && n > 0 && (size_t)n < sizeof(line); ++i) {
			unsigned vlvl = rh_db16(off + 2u * i);
			if (i && !vlvl) {
				break;
			}
			n += snprintf(line + n, sizeof(line) - (size_t)n, " %u", vlvl);
		}
		printf("%s\n", line);
	} else if (slave == RH_BCM) {
		if (len < 8) {
			printf("SWEET RPMH:   BCM %-8s addr=%#x aux len %u\n", id, addr, len);
			return;
		}
		uint32_t unit = rh_db16(off) | (uint32_t)rh_db16(off + 2) << 16;
		printf("SWEET RPMH:   BCM %-8s addr=%#x unit=%u width=%u vcd=%u\n",
		    id, addr, unit, rh_db16(off + 4), rh_db16(off + 6) & 0xffu);
	}
}

int
sweet_rpmh_cmddb_dump(void)
{
	if (__atomic_exchange_n(&rh_lock, 1, __ATOMIC_ACQUIRE)) {
		return EBUSY;
	}
	int error = rh_map();
	for (unsigned i = 0; !error && i < 8; ++i) {
		uint32_t h = 8u + 16u * i;
		unsigned slave = rh_db16(h);
		if (!slave) {
			break;
		}
		uint32_t entries = RH_DB_HEADER + rh_db16(h + 2);
		uint32_t aux = RH_DB_HEADER + rh_db16(h + 4);
		unsigned count = rh_db16(h + 6);
		if (slave < RH_ARC || slave > RH_BCM || (entries & 3u) ||
		    !rh_range(entries, count * RH_DB_ENTRY) || !rh_range(aux, 0)) {
			printf("SWEET RPMH: malformed cmd-db slave header %u\n", i);
			error = EINVAL;
			break;
		}
		printf("SWEET RPMH: cmd-db slave %u (%s) version=%#x entries=%u\n", slave,
		    slave == RH_ARC ? "ARC" : slave == RH_VRM ? "VRM" : "BCM", rh_db16(h + 8), count);
		if (slave != RH_VRM) {
			for (unsigned j = 0; j < count; ++j) {
				rh_dump_entry(slave, entries + RH_DB_ENTRY * j, aux);
			}
			continue;
		}
		/* VRM: names only, six to a line (each up to " ldoa10@0x3f000"). */
		char line[160];
		int n = 0;
		for (unsigned j = 0; j < count; ++j) {
			uint32_t e = entries + RH_DB_ENTRY * j;
			char id[9];
			rh_entry_id(e, id);
			if (j % 6 == 0) {
				n = snprintf(line, sizeof(line), "SWEET RPMH:   VRM");
			}
			if (n > 0 && (size_t)n < sizeof(line)) {
				n += snprintf(line + n, sizeof(line) - (size_t)n, " %s@%#x", id, rh_read(rh_db, e + 16));
			}
			if (j % 6 == 5 || j + 1 == count) {
				printf("%s\n", line);
			}
		}
	}
	__atomic_store_n(&rh_lock, 0, __ATOMIC_RELEASE);
	return error;
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
