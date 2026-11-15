/*
 * The user interface of the SM7150 Adreno 618 service (bsd/dev/sweet_gsvc.c): an IOKit service
 * and user client that speak the msm DRM commands, so Mesa's freedreno can sit on a libdrm
 * stand-in. Opt-in: nothing is published without the boot-arg sweetgpuabi=1.
 *
 * Selectors are 0x40 + DRM_MSM_x (sweet_msm.h), plus 0x70 (ABI info) and 0x72 (close a BO). A BO
 * is kernel memory from sweet_gsvc_bo_new at a fixed GPU address; the client maps it with
 * IOConnectMapMemory64 (memory type = handle) and reads the address with GEM_INFO. Addresses
 * never move, so a submit carries no relocations. One ring, one address space, polled fences.
 */
#include <pexpert/arm64/board_config.h>

#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <mach/mach_time.h>
#include "sweet_msm.h"

extern "C" {
#define SWEET_GSVC_READY 1u
#define SWEET_GSVC_DEAD  2u
struct sweet_gsvc_ib {
	uint32_t handle, offset, size_dw;
};
struct sweet_gsvc_info {
	uint32_t chip_id, gmem_size, gmem_base;
	uint32_t cur_khz, max_khz, last_retired, flags;
	uint64_t fence_pa, window_base, window_size;
};
/* bsd/dev/sweet_gsvc.c */
int sweet_gsvc_bo_new(uint32_t size, uint32_t flags, uint32_t *handle, uint64_t *gpu_va, volatile uint32_t **cpu);
int sweet_gsvc_bo_free(uint32_t handle);
int sweet_gsvc_bo_sync(uint32_t handle);
int sweet_gsvc_submit(const struct sweet_gsvc_ib *ibs, unsigned n, uint32_t *seq);
int sweet_gsvc_wait(uint32_t seq, uint32_t timeout_ms);
void sweet_gsvc_info(struct sweet_gsvc_info *info);
}

#define SG_MAX_BO       2048u   /* the table of sweet_gsvc.c with sweetgpuvm=1 (64 without it) */
#define SG_MAX_IBS      16u
#define SG_MAX_BOS      1024u   /* entries of a submit's BO table */
#define SG_GPU_ID       618u
#define SG_WAIT_MAX_MS  5000u

class SweetGPU : public IOService {
	OSDeclareDefaultStructors(SweetGPU);

public:
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(SweetGPU, IOService);

bool
SweetGPU::start(IOService *provider)
{
	uint32_t on = 0;
	if (!PE_parse_boot_argn("sweetgpuabi", &on, sizeof(on)) || on != 1) {
		return false;
	}
	if (!IOService::start(provider)) {
		return false;
	}
	setProperty(kIOUserClientClassKey, "SweetGPUClient");
	registerService();
	IOLog("SWEET GPU: user interface published (sweetgpuabi=1, abi %u)\n", SWEET_GPU_ABI);
	return true;
}

class SweetGPUClient : public IOUserClient {
	OSDeclareDefaultStructors(SweetGPUClient);

public:
	bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	void free(void) APPLE_KEXT_OVERRIDE;
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target,
	    void *reference) APPLE_KEXT_OVERRIDE;

private:
	struct Bo {
		bool used, retired;
		volatile uint32_t *cpu;
		uint32_t size, flags;
		uint64_t gva;
	};

	task_t fTask;
	IOLock *fLock;
	Bo fBo[SG_MAX_BO + 1];
	uint32_t fQueues;
	uint32_t fLastFence;
	bool fClosed;

	bool owns(uint32_t handle);
	bool copyIn(uint64_t uaddr, void *dst, size_t len);
	IOReturn getParam(IOExternalMethodArguments *args);
	IOReturn gemNew(IOExternalMethodArguments *args);
	IOReturn gemInfo(IOExternalMethodArguments *args);
	IOReturn gemPrep(IOExternalMethodArguments *args);
	IOReturn gemClose(IOExternalMethodArguments *args);
	IOReturn submit(IOExternalMethodArguments *args);
	IOReturn waitFence(IOExternalMethodArguments *args);
	IOReturn queueNew(IOExternalMethodArguments *args);
};

OSDefineMetaClassAndStructors(SweetGPUClient, IOUserClient);

static IOReturn
sg_ret(int e)
{
	switch (e) {
	case 0:         return kIOReturnSuccess;
	case ENOMEM:    return kIOReturnNoMemory;
	case EINVAL:    return kIOReturnBadArgument;
	case ENXIO:     return kIOReturnNotReady;
	case EBUSY:     return kIOReturnBusy;
	case ETIMEDOUT: return kIOReturnTimeout;
	default:        return kIOReturnIOError;
	}
}

static bool
sg_usable(void)
{
	struct sweet_gsvc_info info;
	sweet_gsvc_info(&info);
	return (info.flags & SWEET_GSVC_READY) && !(info.flags & SWEET_GSVC_DEAD);
}

/* The structure input of a call, exactly the size the command takes. */
template <typename T>
static bool
sg_in(IOExternalMethodArguments *args, T *out)
{
	if (args->structureInput == NULL || args->structureInputSize != sizeof(T)) {
		return false;
	}
	memcpy(out, args->structureInput, sizeof(T));
	return true;
}

template <typename T>
static bool
sg_out(IOExternalMethodArguments *args, const T *in)
{
	if (args->structureOutput == NULL || args->structureOutputSize != sizeof(T)) {
		return false;
	}
	memcpy(args->structureOutput, in, sizeof(T));
	return true;
}

bool
SweetGPUClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	fLock = IOLockAlloc();
	return fLock != NULL;
}

IOReturn
SweetGPUClient::clientClose(void)
{
	if (fLock == NULL) {
		return kIOReturnSuccess;
	}
	IOLockLock(fLock);
	bool first = !fClosed;
	fClosed = true;
	uint32_t last = fLastFence;
	IOLockUnlock(fLock);
	if (first) {
		/* the GPU may still read what is freed here */
		if (last) {
			(void)sweet_gsvc_wait(last, SG_WAIT_MAX_MS);
		}
		for (uint32_t h = 1; h <= SG_MAX_BO; h++) {
			if (fBo[h].used) {
				(void)sweet_gsvc_bo_free(h);
				fBo[h].used = false;
			}
		}
	}
	terminate();
	return kIOReturnSuccess;
}

void
SweetGPUClient::free(void)
{
	if (fLock != NULL) {
		IOLockFree(fLock);
		fLock = NULL;
	}
	IOUserClient::free();
}

bool
SweetGPUClient::owns(uint32_t handle)
{
	return handle >= 1 && handle <= SG_MAX_BO && fBo[handle].used && !fBo[handle].retired;
}

/* Reads LEN bytes of the calling task's memory at UADDR. */
bool
SweetGPUClient::copyIn(uint64_t uaddr, void *dst, size_t len)
{
	if (len == 0) {
		return true;
	}
	IOMemoryDescriptor *md = IOMemoryDescriptor::withAddressRange(uaddr, len, kIODirectionOut, fTask);
	if (md == NULL) {
		return false;
	}
	bool ok = md->prepare() == kIOReturnSuccess;
	if (ok) {
		ok = md->readBytes(0, dst, len) == len;
		md->complete();
	}
	md->release();
	return ok;
}

IOReturn
SweetGPUClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory)
{
	IOLockLock(fLock);
	bool ok = owns(type);
	Bo bo = ok ? fBo[type] : Bo();
	IOLockUnlock(fLock);
	if (!ok) {
		return kIOReturnBadArgument;
	}
	/* cacheable, like the kernel's own alias: CPU_PREP / CPU_FINI do the cache work */
	IOMemoryDescriptor *md = IOMemoryDescriptor::withAddressRange((mach_vm_address_t)bo.cpu,
	    round_page(bo.size), kIODirectionInOut, kernel_task);
	if (md == NULL) {
		return kIOReturnNoMemory;
	}
	*options = 0;
	*memory = md;
	return kIOReturnSuccess;
}

IOReturn
SweetGPUClient::getParam(IOExternalMethodArguments *args)
{
	struct drm_msm_param p;
	if (!sg_in(args, &p)) {
		return kIOReturnBadArgument;
	}
	struct sweet_gsvc_info info;
	sweet_gsvc_info(&info);
	uint64_t v;
	switch (p.param) {
	case MSM_PARAM_GPU_ID:     v = SG_GPU_ID; break;
	case MSM_PARAM_GMEM_SIZE:  v = info.gmem_size; break;
	case MSM_PARAM_CHIP_ID:    v = info.chip_id; break;
	case MSM_PARAM_MAX_FREQ:   v = (uint64_t)info.max_khz * 1000u; break;
	case MSM_PARAM_TIMESTAMP:  v = mach_absolute_time(); break;      /* the 19.2 MHz architected counter */
	case MSM_PARAM_GMEM_BASE:  v = info.gmem_base; break;
	case MSM_PARAM_PRIORITIES: v = 1; break;                         /* one ring */
	case MSM_PARAM_PP_PGTABLE: v = 0; break;                         /* one address space */
	/* not VA_START / VA_SIZE: userspace then takes the addresses GEM_INFO returns */
	default:                   return kIOReturnUnsupported;
	}
	p.value = v;
	return sg_out(args, &p) ? kIOReturnSuccess : kIOReturnBadArgument;
}

IOReturn
SweetGPUClient::gemNew(IOExternalMethodArguments *args)
{
	struct drm_msm_gem_new n;
	if (!sg_in(args, &n) || n.size == 0 || n.size > 0xffffffffull) {
		return kIOReturnBadArgument;
	}
	uint32_t h = 0;
	uint64_t gva = 0;
	volatile uint32_t *cpu = NULL;
	if (!sg_usable()) {
		return kIOReturnNotReady;
	}
	int e = sweet_gsvc_bo_new((uint32_t)n.size, 0, &h, &gva, &cpu);
	if (e != 0 && h == 0) {
		return sg_ret(e);
	}
	IOLockLock(fLock);
	fBo[h] = (Bo){ true, false, cpu, (uint32_t)n.size, n.flags, gva };
	IOLockUnlock(fLock);
	n.handle = h;
	return sg_out(args, &n) ? sg_ret(e) : kIOReturnBadArgument;
}

IOReturn
SweetGPUClient::gemInfo(IOExternalMethodArguments *args)
{
	struct drm_msm_gem_info i;
	if (!sg_in(args, &i)) {
		return kIOReturnBadArgument;
	}
	IOLockLock(fLock);
	bool ok = owns(i.handle);
	Bo bo = ok ? fBo[i.handle] : Bo();
	IOLockUnlock(fLock);
	if (!ok) {
		return kIOReturnBadArgument;
	}
	switch (i.info) {
	case MSM_INFO_GET_OFFSET: i.value = i.handle; break;   /* there is no mmap offset: map memory type = handle */
	case MSM_INFO_GET_IOVA:   i.value = bo.gva; break;
	case MSM_INFO_GET_FLAGS:  i.value = bo.flags; break;
	case MSM_INFO_SET_NAME:   break;                       /* debug names are dropped */
	default:                  return kIOReturnUnsupported;
	}
	return sg_out(args, &i) ? kIOReturnSuccess : kIOReturnBadArgument;
}

/* CPU_PREP: whatever the direction, clean and invalidate the BO to the point of coherency. */
IOReturn
SweetGPUClient::gemPrep(IOExternalMethodArguments *args)
{
	struct drm_msm_gem_cpu_prep p;
	if (!sg_in(args, &p)) {
		return kIOReturnBadArgument;
	}
	IOLockLock(fLock);
	bool ok = owns(p.handle);
	IOLockUnlock(fLock);
	return ok ? sg_ret(sweet_gsvc_bo_sync(p.handle)) : kIOReturnBadArgument;
}

/*
 * GEM_CLOSE only retires the handle: the memory may still be mapped in the client, and freeing it under a
 * live mapping would hand those pages to someone else. It is released when the client closes, so a client
 * has SG_MAX_BO buffers for its whole life (the service's own table size).
 */
IOReturn
SweetGPUClient::gemClose(IOExternalMethodArguments *args)
{
	if (args->scalarInputCount != 1) {
		return kIOReturnBadArgument;
	}
	uint32_t h = (uint32_t)args->scalarInput[0];
	IOLockLock(fLock);
	bool ok = owns(h);
	if (ok) {
		fBo[h].retired = true;
	}
	IOLockUnlock(fLock);
	return ok ? kIOReturnSuccess : kIOReturnBadArgument;
}

IOReturn
SweetGPUClient::submit(IOExternalMethodArguments *args)
{
	struct drm_msm_gem_submit s;
	if (!sg_in(args, &s)) {
		return kIOReturnBadArgument;
	}
	if (s.nr_cmds == 0 || s.nr_cmds > SG_MAX_IBS || s.nr_bos == 0 || s.nr_bos > SG_MAX_BOS ||
	    s.nr_in_syncobjs != 0 || s.nr_out_syncobjs != 0) {
		return kIOReturnBadArgument;
	}
	if (!sg_usable()) {
		return kIOReturnNotReady;
	}
	struct drm_msm_gem_submit_cmd cmds[SG_MAX_IBS];
	struct sweet_gsvc_ib ibs[SG_MAX_IBS];
	size_t bos_size = s.nr_bos * sizeof(struct drm_msm_gem_submit_bo);
	struct drm_msm_gem_submit_bo *bos = (struct drm_msm_gem_submit_bo *)IOMalloc(bos_size);
	if (bos == NULL) {
		return kIOReturnNoMemory;
	}
	if (!copyIn(s.cmds, cmds, s.nr_cmds * sizeof(cmds[0])) || !copyIn(s.bos, bos, bos_size)) {
		IOFree(bos, bos_size);
		return kIOReturnBadArgument;
	}
	IOLockLock(fLock);
	for (uint32_t i = 0; i < s.nr_cmds; i++) {
		/* addresses are fixed, so there is nothing to relocate; one kind of command buffer */
		if (cmds[i].type != MSM_SUBMIT_CMD_BUF || cmds[i].nr_relocs != 0 || cmds[i].submit_idx >= s.nr_bos ||
		    (cmds[i].size & 3u) || cmds[i].size == 0) {
			IOLockUnlock(fLock);
			IOFree(bos, bos_size);
			return kIOReturnBadArgument;
		}
		uint32_t h = bos[cmds[i].submit_idx].handle;
		if (!owns(h)) {
			IOLockUnlock(fLock);
			IOFree(bos, bos_size);
			return kIOReturnBadArgument;
		}
		ibs[i] = (struct sweet_gsvc_ib){ h, cmds[i].submit_offset, cmds[i].size / 4u };
	}
	IOLockUnlock(fLock);
	IOFree(bos, bos_size);
	for (uint32_t i = 0; i < s.nr_cmds; i++) {
		(void)sweet_gsvc_bo_sync(ibs[i].handle);       /* what the CPU wrote reaches DRAM before the CP reads it */
	}
	uint32_t seq = 0;
	int e = sweet_gsvc_submit(ibs, s.nr_cmds, &seq);
	if (e != 0) {
		return sg_ret(e);
	}
	IOLockLock(fLock);
	fLastFence = seq;
	IOLockUnlock(fLock);
	s.fence = seq;
	return sg_out(args, &s) ? kIOReturnSuccess : kIOReturnBadArgument;
}

/* timeout is an absolute time on the uptime clock, like msm's CLOCK_MONOTONIC */
IOReturn
SweetGPUClient::waitFence(IOExternalMethodArguments *args)
{
	struct drm_msm_wait_fence w;
	if (!sg_in(args, &w)) {
		return kIOReturnBadArgument;
	}
	uint64_t now = 0;
	absolutetime_to_nanoseconds(mach_absolute_time(), &now);
	uint64_t at = (uint64_t)w.timeout.tv_sec * 1000000000ull + (uint64_t)w.timeout.tv_nsec;
	uint64_t ms = at > now ? (at - now) / 1000000ull : 0;
	if (ms > SG_WAIT_MAX_MS) {
		ms = SG_WAIT_MAX_MS;
	}
	return sg_ret(sweet_gsvc_wait(w.fence, (uint32_t)ms));
}

/* One ring: any priority and flags are accepted and every queue is the same; ids only count */
IOReturn
SweetGPUClient::queueNew(IOExternalMethodArguments *args)
{
	struct drm_msm_submitqueue q;
	if (!sg_in(args, &q)) {
		return kIOReturnBadArgument;
	}
	IOLockLock(fLock);
	q.id = ++fQueues;
	IOLockUnlock(fLock);
	return sg_out(args, &q) ? kIOReturnSuccess : kIOReturnBadArgument;
}

static uint64_t sweet_gpu_logged[2];    /* selectors already logged, by the low 7 bits */

IOReturn
SweetGPUClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target, __unused void *reference)
{
	uint64_t *seen = &sweet_gpu_logged[(selector >> 6) & 1u];
	if (!(*seen & (1ull << (selector & 63u)))) {
		*seen |= 1ull << (selector & 63u);
		IOLog("SWEET GPU: selector %#x\n", selector);
	}
	switch (selector) {
	case SWEET_MSM_BASE + DRM_MSM_GET_PARAM:
		return getParam(args);
	case SWEET_MSM_BASE + DRM_MSM_GEM_NEW:
		return gemNew(args);
	case SWEET_MSM_BASE + DRM_MSM_GEM_INFO:
		return gemInfo(args);
	case SWEET_MSM_BASE + DRM_MSM_GEM_CPU_PREP:
		return gemPrep(args);
	case SWEET_MSM_BASE + DRM_MSM_GEM_CPU_FINI:
		return args->structureInputSize == sizeof(struct drm_msm_gem_cpu_fini) ? kIOReturnSuccess : kIOReturnBadArgument;
	case SWEET_MSM_BASE + DRM_MSM_GEM_SUBMIT:
		return submit(args);
	case SWEET_MSM_BASE + DRM_MSM_WAIT_FENCE:
		return waitFence(args);
	case SWEET_MSM_BASE + DRM_MSM_SUBMITQUEUE_NEW:
		return queueNew(args);
	case SWEET_MSM_BASE + DRM_MSM_SUBMITQUEUE_CLOSE:
		return kIOReturnSuccess;
	case SWEET_GPU_SEL_INFO: {
		struct sweet_gsvc_info info;
		struct sweet_gpu_abi_info out;
		sweet_gsvc_info(&info);
		out = (struct sweet_gpu_abi_info){ SWEET_GPU_ABI, info.chip_id, info.gmem_size, info.gmem_base,
			                           info.window_base, info.window_size };
		return sg_out(args, &out) ? kIOReturnSuccess : kIOReturnBadArgument;
	}
	case SWEET_GPU_SEL_GEM_CLOSE:
		return gemClose(args);
	default:
		return kIOReturnUnsupported;
	}
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
