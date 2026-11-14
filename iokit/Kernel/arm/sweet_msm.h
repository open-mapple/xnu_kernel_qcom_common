/*
 * The part of the Linux msm DRM user interface that SweetGPUClient.cpp serves, with the
 * kernel types spelled as stdint. Structure layouts and numbers follow include/uapi/drm/msm_drm.h
 * (Mesa's copy of it, fetched 2026-11-14); a Mesa built from another revision of that header
 * has to match it.
 *
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef _SWEET_MSM_H_
#define _SWEET_MSM_H_

#include <stdint.h>

/* external-method selector = SWEET_MSM_BASE + DRM_MSM_x, so a libdrm stand-in forwards drmIoctl one to one */
#define SWEET_MSM_BASE            0x40u
#define DRM_MSM_GET_PARAM         0x00
#define DRM_MSM_GEM_NEW           0x02
#define DRM_MSM_GEM_INFO          0x03
#define DRM_MSM_GEM_CPU_PREP      0x04
#define DRM_MSM_GEM_CPU_FINI      0x05
#define DRM_MSM_GEM_SUBMIT        0x06
#define DRM_MSM_WAIT_FENCE        0x07
#define DRM_MSM_SUBMITQUEUE_NEW   0x0A
#define DRM_MSM_SUBMITQUEUE_CLOSE 0x0B
#define DRM_MSM_SUBMITQUEUE_QUERY 0x0C

/* selectors that are not msm's */
#define SWEET_GPU_SEL_INFO        0x70u
#define SWEET_GPU_SEL_GEM_CLOSE   0x72u
#define SWEET_GPU_ABI             1u

#define MSM_PARAM_GPU_ID          0x01
#define MSM_PARAM_GMEM_SIZE       0x02
#define MSM_PARAM_CHIP_ID         0x03
#define MSM_PARAM_MAX_FREQ        0x04
#define MSM_PARAM_TIMESTAMP       0x05
#define MSM_PARAM_GMEM_BASE       0x06
#define MSM_PARAM_PRIORITIES      0x07
#define MSM_PARAM_PP_PGTABLE      0x08
#define MSM_PARAM_VA_START        0x0e
#define MSM_PARAM_VA_SIZE         0x0f

struct drm_msm_timespec {
	int64_t tv_sec;
	int64_t tv_nsec;
};

struct drm_msm_param {
	uint32_t pipe;
	uint32_t param;
	uint64_t value;
	uint32_t len;
	uint32_t pad;
};

#define MSM_BO_CACHE_MASK         0x000f0000
struct drm_msm_gem_new {
	uint64_t size;
	uint32_t flags;
	uint32_t handle;
};

#define MSM_INFO_GET_OFFSET       0x00
#define MSM_INFO_GET_IOVA         0x01
#define MSM_INFO_SET_NAME         0x02
#define MSM_INFO_GET_NAME         0x03
#define MSM_INFO_GET_FLAGS        0x05
struct drm_msm_gem_info {
	uint32_t handle;
	uint32_t info;
	uint64_t value;
	uint32_t len;
	uint32_t pad;
};

#define MSM_PREP_READ             0x01
#define MSM_PREP_WRITE            0x02
struct drm_msm_gem_cpu_prep {
	uint32_t handle;
	uint32_t op;
	struct drm_msm_timespec timeout;
};

struct drm_msm_gem_cpu_fini {
	uint32_t handle;
};

#define MSM_SUBMIT_CMD_BUF        0x0001
struct drm_msm_gem_submit_cmd {
	uint32_t type;
	uint32_t submit_idx;
	uint32_t submit_offset;
	uint32_t size;
	uint32_t pad;
	uint32_t nr_relocs;
	uint64_t relocs;
};

struct drm_msm_gem_submit_bo {
	uint32_t flags;
	uint32_t handle;
	uint64_t presumed;
};

struct drm_msm_gem_submit {
	uint32_t flags;
	uint32_t fence;
	uint32_t nr_bos;
	uint32_t nr_cmds;
	uint64_t bos;
	uint64_t cmds;
	int32_t  fence_fd;
	uint32_t queueid;
	uint64_t in_syncobjs;
	uint64_t out_syncobjs;
	uint32_t nr_in_syncobjs;
	uint32_t nr_out_syncobjs;
	uint32_t syncobj_stride;
	uint32_t pad;
};

struct drm_msm_wait_fence {
	uint32_t fence;
	uint32_t flags;
	struct drm_msm_timespec timeout;
	uint32_t queueid;
};

struct drm_msm_submitqueue {
	uint32_t flags;
	uint32_t prio;
	uint32_t id;
};

/* SWEET_GPU_SEL_INFO output */
struct sweet_gpu_abi_info {
	uint32_t abi, chip_id, gmem_size, gmem_base;
	uint64_t va_base, va_size;
};

_Static_assert(sizeof(struct drm_msm_param) == 24, "drm_msm_param");
_Static_assert(sizeof(struct drm_msm_gem_new) == 16, "drm_msm_gem_new");
_Static_assert(sizeof(struct drm_msm_gem_info) == 24, "drm_msm_gem_info");
_Static_assert(sizeof(struct drm_msm_gem_submit) == 72, "drm_msm_gem_submit");
_Static_assert(sizeof(struct drm_msm_gem_submit_cmd) == 32, "drm_msm_gem_submit_cmd");
_Static_assert(sizeof(struct drm_msm_gem_submit_bo) == 16, "drm_msm_gem_submit_bo");
_Static_assert(sizeof(struct drm_msm_wait_fence) == 32, "drm_msm_wait_fence");

#endif /* _SWEET_MSM_H_ */
