/*
 * Copyright (c) 2019 Apple Inc. All rights reserved.
 *
 * @APPLE_OSREFERENCE_LICENSE_HEADER_START@
 *
 * This file contains Original Code and/or Modifications of Original Code
 * as defined in and that are subject to the Apple Public Source License
 * Version 2.0 (the 'License'). You may not use this file except in
 * compliance with the License. The rights granted to you under the License
 * may not be used to create, or enable the creation or redistribution of,
 * unlawful or unlicensed copies of an Apple operating system, or to
 * circumvent, violate, or enable the circumvention or violation of, any
 * terms of an Apple operating system software license agreement.
 *
 * Please obtain a copy of the License at
 * http://www.opensource.apple.com/apsl/ and read it before using this file.
 *
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 * Please see the License for the specific language governing rights and
 * limitations under the License.
 *
 * @APPLE_OSREFERENCE_LICENSE_HEADER_END@
 */

#ifndef _ARM64_PAC_ASM_H_
#define _ARM64_PAC_ASM_H_

#ifndef __ASSEMBLER__
#error "This header should only be used in .s files"
#endif

#include <pexpert/arm64/board_config.h>
#include <arm64/proc_reg.h>
#if HAS_PARAVIRTUALIZED_PAC
#include <arm64/hv_hvc.h>
#include "smccc_asm.h"
#endif
#include "assym.s"

#if defined(HAS_APPLE_PAC)

#if HAS_NATIVE_PAC
/*
 * FUXI arm64e (patch 0115). fuxi_native_pac (start.s), u64 words: the ten key
 * halves in APIA, APIB, APDA, APDB, APGA Lo/Hi order, then the status words
 * that osfmk/arm64/fuxi_pac.c prints (mirrored in fuxi_pac_logic.h FP_NATIVE_*).
 */
#define NATIVE_PAC_KEY_WORDS    10
#define NATIVE_PAC_READY        10      /* 1: key set generated */
#define NATIVE_PAC_MODE         11      /* 1 native: the first CPU read its keys back; 2 hyp: it did not (keys kept as found) */
#define NATIVE_PAC_CPUS         12      /* passes through PAC_INIT_KEY_STATE, warm resets included */
#define NATIVE_PAC_BAD          13      /* passes whose key readback differed */
#define NATIVE_PAC_BAD_MPIDR    14
#define NATIVE_PAC_GA_REF       15      /* PACGA of the first pass (0: no generic auth) */
#define NATIVE_PAC_GA_BAD       16      /* passes with another PACGA: parked, never returned */
#define NATIVE_PAC_GA_MPIDR     17
#define NATIVE_PAC_WORDS        24      /* three 64-byte lines */
#define NATIVE_PAC_MODIFIER     0x5eedfacecafe1234
#endif /* HAS_NATIVE_PAC */

/* BEGIN IGNORE CODESTYLE */

/**
 * REPROGRAM_JOP_KEYS
 *
 * Loads a userspace process's JOP key (task->jop_pid) into the CPU, and
 * updates current_cpu_datap()->jop_key accordingly.  This reprogramming process
 * is skipped whenever the "new" JOP key has already been loaded into the CPU.
 *
 *   skip_label - branch to this label if new_jop_key is already loaded into CPU
 *   new_jop_key - process's jop_pid
 *   cpudatap - current cpu_data_t *
 *   tmp - scratch register
 */
.macro REPROGRAM_JOP_KEYS	skip_label, new_jop_key, cpudatap, tmp
	ldr		\tmp, [\cpudatap, CPU_JOP_KEY]
	cmp		\new_jop_key, \tmp
	b.eq	\skip_label
	SET_JOP_KEY_REGISTERS	\new_jop_key, \tmp
	str		\new_jop_key, [\cpudatap, CPU_JOP_KEY]
.endmacro

/**
 * REPROGRAM_ROP_KEYS
 *
 * Loads a userspace process's ROP key (task->rop_pid) into the CPU, and
 * updates current_cpu_datap()->rop_key accordingly.  This reprogramming process
 * is skipped whenever the "new" ROP key has already been loaded into the CPU.
 *
 *   skip_label - branch to this label if new_rop_key is already loaded into CPU
 *   new_rop_key - process's rop_pid
 *   cpudatap - current cpu_data_t *
 *   tmp - scratch register
 */
.macro REPROGRAM_ROP_KEYS	skip_label, new_rop_key, cpudatap, tmp
	ldr		\tmp, [\cpudatap, CPU_ROP_KEY]
	cmp		\new_rop_key, \tmp
	b.eq	\skip_label
	SET_ROP_KEY_REGISTERS	\new_rop_key, \tmp
	str		\new_rop_key, [\cpudatap, CPU_ROP_KEY]
.endmacro

/**
 * SET_JOP_KEY_REGISTERS
 *
 * Unconditionally loads a userspace process's JOP key (task->jop_pid) into the
 * CPU.  The caller is responsible for updating current_cpu_datap()->jop_key as
 * needed.
 *
 *   new_jop_key - process's jop_pid
 *   tmp - scratch register
 */
.macro SET_JOP_KEY_REGISTERS	new_jop_key, tmp
#if HAS_PARAVIRTUALIZED_PAC
	SAVE_SMCCC_CLOBBERED_REGISTERS
	/*
	 * We're deliberately calling PAC_SET_EL0_DIVERSIFIER here, even though the
	 * EL0 diversifier affects both A (JOP) and B (ROP) keys.  We don't want
	 * SET_JOP_KEY_REGISTERS to have an impact on the EL1 A key state, since
	 * these are the keys the kernel uses to sign pointers on the heap.
	 *
	 * Using new_jop_key as the EL0 diversifer has the same net effect of giving
	 * userspace its own set of JOP keys, but doesn't affect EL1 A key state.
	 */
	MOV64	x0, VMAPPLE_PAC_SET_EL0_DIVERSIFIER
	mov		x1, \new_jop_key
	hvc		#0
	LOAD_SMCCC_CLOBBERED_REGISTERS
#endif /* HAS_PARAVIRTUALIZED_PAC */
.endmacro

/**
 * SET_ROP_KEY_REGISTERS
 *
 * Unconditionally loads a userspace process's ROP key (task->rop_pid) into the
 * CPU.  The caller is responsible for updating current_cpu_datap()->rop_key as
 * needed.
 *
 *   new_rop_key - process's rop_pid
 *   tmp - scratch register
 */
.macro SET_ROP_KEY_REGISTERS	new_rop_key, tmp
#if HAS_PARAVIRTUALIZED_PAC
	SAVE_SMCCC_CLOBBERED_REGISTERS
	MOV64	x0, VMAPPLE_PAC_SET_B_KEYS
	mov		x1, \new_rop_key
	hvc		#0
	LOAD_SMCCC_CLOBBERED_REGISTERS
#endif /* HAS_PARAVIRTUALIZED_PAC */
.endmacro

/**
 * PAC_INIT_KEY_STATE
 *
 * Sets the initial PAC key state, but does not enable the keys.
 *
 *   tmp - scratch register
 *   tmp2 - scratch register
 */
#if HAS_NATIVE_PAC
.macro NATIVE_PAC_CHECK key, off
	mrs		x2, \key
	ldr		x3, [x0, #(\off)]
	eor		x2, x2, x3
	orr		x7, x7, x2
.endmacro

.macro NATIVE_PAC_INC word, tmp
	ldr		\tmp, [x0, #((\word) * 8)]
	add		\tmp, \tmp, #1
	str		\tmp, [x0, #((\word) * 8)]
.endmacro

.macro NATIVE_PAC_FLUSH
	dc		civac, x0
	add		x2, x0, #64
	dc		civac, x2
	add		x2, x0, #128
	dc		civac, x2
	dsb		sy
.endmacro

.macro NATIVE_PAC_STAGE stage
#if defined(FUXI)
	adrp	x16, EXT(fuxi_early_status)@page
	add		x16, x16, EXT(fuxi_early_status)@pageoff
	mov		x17, #(\stage)
	str		x17, [x16]
	dc		civac, x16
	dsb		sy
#endif
.endmacro
#endif /* HAS_NATIVE_PAC */

.macro PAC_INIT_KEY_STATE	tmp, tmp2
#if HAS_NATIVE_PAC
	/*
	 * FUXI arm64e (patch 0115). Key generation and loading follow UniXNU's
	 * native-pac-keys.patch (BSD-2-Clause-Patent, UniXNU contributors).
	 * ARMv8.3 keys are shared by EL0 and EL1: the kernel and every process use
	 * one key set per boot. The first pass derives it from CNTPCT_EL0 with
	 * splitmix64; every later pass (secondaries, warm resets) loads the stored
	 * set. Each pass reads the keys back (mismatch: counted; on the first pass,
	 * mode 2 keeps whatever keys the hardware has) and computes PACGA: a CPU
	 * whose PACGA differs from the first pass would fail every pointer signed
	 * elsewhere, so it records itself and parks (fuxi_early_status 0x35).
	 * CPUs come up one at a time, so plain loads/stores suffice. The MMU may be
	 * off: stores go straight to memory, dc civac drops stale cached lines.
	 * fuxi_early_status 0x30 while the keys are written, 0x31 after.
	 * Uses x0-x7 (and x16/x17 for the stage word): common_start scratch only.
	 */
	adrp	x0, EXT(fuxi_native_pac)@page
	add		x0, x0, EXT(fuxi_native_pac)@pageoff
	ldr		x1, [x0, #(NATIVE_PAC_READY * 8)]
	cbnz	x1, Lnative_pac_load\@
	mrs		x1, CNTPCT_EL0
	MOV64	x3, 0x9e3779b97f4a7c15
	MOV64	x4, 0xbf58476d1ce4e5b9
	MOV64	x5, 0x94d049bb133111eb
	mov		x2, #0
Lnative_pac_gen\@:
	add		x1, x1, x3
	eor		x6, x1, x1, lsr #30
	mul		x6, x6, x4
	eor		x6, x6, x6, lsr #27
	mul		x6, x6, x5
	eor		x6, x6, x6, lsr #31
	str		x6, [x0, x2, lsl #3]
	add		x2, x2, #1
	cmp		x2, #(NATIVE_PAC_KEY_WORDS)
	b.lo	Lnative_pac_gen\@
	mov		x2, #1
	str		x2, [x0, #(NATIVE_PAC_READY * 8)]
	str		x2, [x0, #(NATIVE_PAC_MODE * 8)]
	NATIVE_PAC_FLUSH
Lnative_pac_load\@:
	NATIVE_PAC_STAGE 0x30
	ldp		x2, x3, [x0, #0]
	msr		APIAKeyLo_EL1, x2
	msr		APIAKeyHi_EL1, x3
	ldp		x2, x3, [x0, #16]
	msr		APIBKeyLo_EL1, x2
	msr		APIBKeyHi_EL1, x3
	ldp		x2, x3, [x0, #32]
	msr		APDAKeyLo_EL1, x2
	msr		APDAKeyHi_EL1, x3
	ldp		x2, x3, [x0, #48]
	msr		APDBKeyLo_EL1, x2
	msr		APDBKeyHi_EL1, x3
	ldp		x2, x3, [x0, #64]
	msr		APGAKeyLo_EL1, x2
	msr		APGAKeyHi_EL1, x3
	isb		sy
	mov		x7, #0
	NATIVE_PAC_CHECK APIAKeyLo_EL1, 0
	NATIVE_PAC_CHECK APIAKeyHi_EL1, 8
	NATIVE_PAC_CHECK APIBKeyLo_EL1, 16
	NATIVE_PAC_CHECK APIBKeyHi_EL1, 24
	NATIVE_PAC_CHECK APDAKeyLo_EL1, 32
	NATIVE_PAC_CHECK APDAKeyHi_EL1, 40
	NATIVE_PAC_CHECK APDBKeyLo_EL1, 48
	NATIVE_PAC_CHECK APDBKeyHi_EL1, 56
	NATIVE_PAC_CHECK APGAKeyLo_EL1, 64
	NATIVE_PAC_CHECK APGAKeyHi_EL1, 72
	cbz		x7, Lnative_pac_ga\@
	NATIVE_PAC_INC NATIVE_PAC_BAD, x2
	mrs		x2, MPIDR_EL1
	str		x2, [x0, #(NATIVE_PAC_BAD_MPIDR * 8)]
	ldr		x2, [x0, #(NATIVE_PAC_CPUS * 8)]
	cbnz	x2, Lnative_pac_ga\@
	mov		x2, #2
	str		x2, [x0, #(NATIVE_PAC_MODE * 8)]
Lnative_pac_ga\@:
	/* PACGA only when ID_AA64ISAR1 GPA/GPI or ID_AA64ISAR2 GPA3 say it exists. */
	mrs		x2, ID_AA64ISAR1_EL1
	ubfx	x3, x2, #24, #8
	mrs		x2, S3_0_C0_C6_2
	ubfx	x2, x2, #8, #4
	orr		x3, x3, x2
	cbz		x3, Lnative_pac_done\@
	MOV64	x4, NATIVE_PAC_MODIFIER
	pacga	x6, x0, x4
	ldr		x2, [x0, #(NATIVE_PAC_CPUS * 8)]
	cbnz	x2, Lnative_pac_cmp\@
	str		x6, [x0, #(NATIVE_PAC_GA_REF * 8)]
	b		Lnative_pac_done\@
Lnative_pac_cmp\@:
	ldr		x3, [x0, #(NATIVE_PAC_GA_REF * 8)]
	cmp		x3, x6
	b.eq	Lnative_pac_done\@
	NATIVE_PAC_INC NATIVE_PAC_GA_BAD, x2
	mrs		x2, MPIDR_EL1
	str		x2, [x0, #(NATIVE_PAC_GA_MPIDR * 8)]
	NATIVE_PAC_FLUSH
	NATIVE_PAC_STAGE 0x35
Lnative_pac_park\@:
	wfe
	b		Lnative_pac_park\@
Lnative_pac_done\@:
	NATIVE_PAC_INC NATIVE_PAC_CPUS, x2
	NATIVE_PAC_FLUSH
	NATIVE_PAC_STAGE 0x31
#endif /* HAS_NATIVE_PAC */
#if HAS_PARAVIRTUALIZED_PAC
#if HIBERNATION
	#error PAC_INIT_KEY_STATE is not implemented for HAS_PARAVIRTUALIZED_PAC && HIBERNATION
#endif
	/*
	 * This call clobbers x0-x3.  However we only initialize PAC at a point in
	 * common_start where x0-x3 are safe to clobber, and where we don't yet have
	 * a working stack to stash the existing values anyway.
	 */
	mov		x0, #VMAPPLE_PAC_SET_INITIAL_STATE
	hvc		#0
#endif /* HAS_PARAVIRTUALIZED_PAC */
.endmacro

/* END IGNORE CODESTYLE */

#endif /* defined(HAS_APPLE_PAC) */

#endif /* _ARM64_PAC_ASM_H_ */

/* vim: set ts=4 ft=asm: */
