/*
 * Copyright (c) 2016-2018, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: SFTL version: 5.0.48 20180925 for kernel
 */
	.arch armv7-a
	.fpu softvfp
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 2
	.eabi_attribute 30, 4
	.eabi_attribute 18, 4
	.file	"rk_sftl.c"
	.global	__aeabi_uidiv
	.text
	.align	2
	.type	l2p_addr_tran, %function
l2p_addr_tran:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r1
	ldr	r3, .L3
	mov	sl, r2
	ldr	r7, [r0, #4]
	ldrh	r5, [r3, #8]
	ubfx	r8, r7, #10, #16
	ldrh	r6, [r3, #10]
	mov	r7, r7, asl #22
	ldrh	r3, [r3, #14]
	mov	r0, r8
	mov	r7, r7, lsr #22
	cmp	r3, #4
	moveq	r5, r5, lsr #1
	moveq	r6, r6, asl #1
	mov	r1, r5
	uxtheq	r6, r6
	bl	__aeabi_uidiv
	uxth	r0, r0
	mls	r5, r5, r0, r8
	mla	r6, r6, r5, r7
	str	r6, [r4, #0]
	str	r0, [sl, #0]
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L4:
	.align	2
.L3:
	.word	.LANCHOR0
	.fnend
	.size	l2p_addr_tran, .-l2p_addr_tran
	.align	2
	.global	Ftl_log2
	.type	Ftl_log2, %function
Ftl_log2:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r2, #1
	mov	r3, #0
	b	.L6
.L7:
	add	r3, r3, #1
	mov	r2, r2, asl #1
	uxth	r3, r3
.L6:
	cmp	r2, r0
	bls	.L7
	sub	r3, r3, #1
	uxth	r0, r3
	bx	lr
	.fnend
	.size	Ftl_log2, .-Ftl_log2
	.align	2
	.global	FtlPrintInfo
	.type	FtlPrintInfo, %function
FtlPrintInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	FtlPrintInfo, .-FtlPrintInfo
	.align	2
	.global	FtlSysBlkNumInit
	.type	FtlSysBlkNumInit, %function
FtlSysBlkNumInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L12
	cmp	r0, #23
	movls	r0, #24
	ldrh	r2, [r3, #28]
	ldrh	r1, [r3, #38]
	str	r0, [r3, #24]
	mul	r2, r2, r0
	rsb	r0, r0, r1
	ldr	r1, [r3, #44]
	strh	r0, [r3, #36]	@ movhi
	mov	r0, #0
	str	r2, [r3, #32]
	rsb	r2, r2, r1
	str	r2, [r3, #40]
	bx	lr
.L13:
	.align	2
.L12:
	.word	.LANCHOR0
	.fnend
	.size	FtlSysBlkNumInit, .-FtlSysBlkNumInit
	.global	__aeabi_idiv
	.align	2
	.global	FtlConstantsInit
	.type	FtlConstantsInit, %function
FtlConstantsInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r0
	ldrh	r1, [sl, #14]
	ldrh	r2, [r0, #0]
	cmp	r1, #4
	ldr	r3, .L21
	ldrh	r5, [r0, #2]
	ldreqh	r1, [sl, #6]
	ldrh	r0, [r0, #4]
	strh	r2, [r3, #48]	@ movhi
	moveq	r1, r1, lsr #1
	strh	r5, [r3, #50]	@ movhi
	streqh	r1, [r3, #38]	@ movhi
	moveq	r1, #8
	strh	r0, [r3, #52]	@ movhi
	streqh	r1, [r3, #54]	@ movhi
	mov	r3, #0
.L16:
	ldr	r4, .L21
	add	r1, r4, #56
	strb	r3, [r3, r1]
	add	r3, r3, #1
	cmp	r3, #32
	bne	.L16
	ldrh	r7, [r4, #38]
	cmp	r2, #1
	mul	r5, r5, r0
	mov	r3, #5
	mul	r0, r0, r7
	strh	r3, [r4, #88]	@ movhi
	mov	r8, #640
	streqh	r2, [r4, #88]	@ movhi
	mov	r3, #0
	strh	r8, [r4, #92]	@ movhi
	uxth	r5, r5
	strh	r3, [r4, #90]	@ movhi
	strh	r5, [r4, #28]	@ movhi
	uxth	r0, r0
	strh	r0, [r4, #94]	@ movhi
	bl	Ftl_log2
	ldrh	r9, [sl, #12]
	ldrh	fp, [r4, #54]
	mul	r3, r9, r5
	strh	r9, [r4, #98]	@ movhi
	strh	r9, [r4, #100]	@ movhi
	strh	r3, [r4, #102]	@ movhi
	strh	r0, [r4, #96]	@ movhi
	mov	r0, fp
	bl	Ftl_log2
	mov	r3, fp, asl #9
	mul	r1, r9, fp
	uxth	r3, r3
	strh	r3, [r4, #106]	@ movhi
	mov	r3, r3, lsr #8
	strh	r3, [r4, #108]	@ movhi
	ldrh	r3, [sl, #20]
	strh	r3, [r4, #110]	@ movhi
	mul	r3, r7, r5
	mov	r7, r7, asl #6
	str	r3, [r4, #44]
	mul	r3, fp, r3
	mul	r3, r9, r3
	mov	r3, r3, asr #11
	str	r3, [r4, #112]
	mov	r6, r0
	strh	r0, [r4, #104]	@ movhi
	mov	r0, #5120
	mov	r8, r8, asr r6
	bl	__aeabi_idiv
	add	r6, r6, #9
	mov	r6, r7, asr r6
	mov	r1, r5
	add	r8, r8, #2
	uxth	r6, r6
	uxth	r0, r0
	strh	r0, [r4, #116]	@ movhi
	cmp	r0, #4
	ldr	r4, .L21
	ldrls	r3, .L21
	movls	r2, #4
	strh	r6, [r4, #120]	@ movhi
	strh	r8, [r4, #118]	@ movhi
	strlsh	r2, [r3, #116]	@ movhi
	mul	r3, r5, r6
	ldrh	r0, [r4, #116]
	add	r6, r6, #8
	str	r3, [r4, #124]
	bl	__aeabi_uidiv
	cmp	r5, #1
	uxtah	r6, r6, r0
	str	r6, [r4, #24]
	addeq	r6, r6, #4
	streq	r6, [r4, #24]
	ldrh	r0, [r4, #24]
	bl	FtlSysBlkNumInit
	ldr	r3, [r4, #24]
	ldrh	r2, [r4, #98]
	mov	r0, #0
	str	r0, [r4, #136]
	str	r3, [r4, #128]
	ldr	r3, [r4, #40]
	mov	r3, r3, asl #2
	mul	r3, r2, r3
	ldrh	r2, [r4, #104]
	add	r2, r2, #9
	mov	r3, r3, lsr r2
	add	r3, r3, #2
	strh	r3, [r4, #132]	@ movhi
	mov	r3, #32
	strh	r3, [r4, #134]	@ movhi
	ldrh	r3, [r4, #116]
	add	r3, r3, #3
	strh	r3, [r4, #116]	@ movhi
	ldr	r3, [r4, #124]
	add	r3, r3, #3
	str	r3, [r4, #124]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L22:
	.align	2
.L21:
	.word	.LANCHOR0
	.fnend
	.size	FtlConstantsInit, .-FtlConstantsInit
	.align	2
	.global	IsBlkInVendorPart
	.type	IsBlkInVendorPart, %function
IsBlkInVendorPart:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L31
	ldrh	r2, [r3, #140]
	cmp	r2, #0
	ldrneh	r1, [r3, #116]
	ldrne	r2, [r3, #144]
	movne	r3, #0
	bne	.L25
	b	.L30
.L27:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L29
	add	r3, r3, #1
	uxth	r3, r3
.L25:
	cmp	r3, r1
	bne	.L27
	mov	r0, #0
	bx	lr
.L29:
	mov	r0, #1
	bx	lr
.L30:
	mov	r0, r2
	bx	lr
.L32:
	.align	2
.L31:
	.word	.LANCHOR0
	.fnend
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.align	2
	.global	FtlCacheWriteBack
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r0, #0
	bx	lr
	.fnend
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
	.align	2
	.global	sftl_get_density
	.type	sftl_get_density, %function
sftl_get_density:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L35
	ldr	r0, [r3, #136]
	bx	lr
.L36:
	.align	2
.L35:
	.word	.LANCHOR0
	.fnend
	.size	sftl_get_density, .-sftl_get_density
	.align	2
	.global	FtlBbmMapBadBlock
	.type	FtlBbmMapBadBlock, %function
FtlBbmMapBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, lr}
	.save {r0, r1, r4, r5, r6, lr}
	mov	r5, r0
	ldr	r4, .L38
	ldrh	r6, [r4, #94]
	mov	r1, r6
	bl	__aeabi_uidiv
	uxth	r2, r0
	mls	r3, r6, r2, r5
	add	r1, r4, r2, asl #2
	mov	r6, #1
	ldr	r0, [r1, #176]
	uxth	r3, r3
	and	r1, r3, #31
	mov	ip, r3, lsr #5
	ldr	lr, [r0, ip, asl #2]
	orr	r1, lr, r6, asl r1
	str	r1, [r0, ip, asl #2]
	str	r1, [sp, #0]
	mov	r1, r5
	ldr	r0, .L38+4
	bl	printk
	ldrh	r3, [r4, #154]
	mov	r0, #0
	add	r3, r3, r6
	strh	r3, [r4, #154]	@ movhi
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L39:
	.align	2
.L38:
	.word	.LANCHOR0
	.word	.LC0
	.fnend
	.size	FtlBbmMapBadBlock, .-FtlBbmMapBadBlock
	.global	__aeabi_uidivmod
	.align	2
	.global	FtlBbmIsBadBlock
	.type	FtlBbmIsBadBlock, %function
FtlBbmIsBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r7, r0
	ldr	r5, .L41
	ldrh	r6, [r5, #94]
	mov	r1, r6
	bl	__aeabi_uidivmod
	mov	r0, r7
	uxth	r4, r1
	mov	r1, r6
	bl	__aeabi_uidiv
	mov	r2, r4, lsr #5
	and	r4, r4, #31
	uxth	r0, r0
	add	r5, r5, r0, asl #2
	ldr	r3, [r5, #176]
	ldr	r0, [r3, r2, asl #2]
	mov	r0, r0, lsr r4
	and	r0, r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L42:
	.align	2
.L41:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbmIsBadBlock, .-FtlBbmIsBadBlock
	.align	2
	.global	FtlBbtInfoPrint
	.type	FtlBbtInfoPrint, %function
FtlBbtInfoPrint:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	FtlBbtInfoPrint, .-FtlBbtInfoPrint
	.align	2
	.global	FtlBbtCalcTotleCnt
	.type	FtlBbtCalcTotleCnt, %function
FtlBbtCalcTotleCnt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, #0
	mov	r5, r4
	ldr	r6, .L48
	b	.L45
.L47:
	mov	r0, r5
	add	r5, r5, #1
	bl	FtlBbmIsBadBlock
	uxth	r5, r5
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
.L45:
	ldrh	r3, [r6, #94]
	ldrh	r2, [r6, #50]
	mul	r3, r2, r3
	cmp	r5, r3
	blt	.L47
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L49:
	.align	2
.L48:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbtCalcTotleCnt, .-FtlBbtCalcTotleCnt
	.align	2
	.global	V2P_block
	.type	V2P_block, %function
V2P_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r7, r1
	ldr	r5, .L51
	mov	r6, r0
	ldrh	r4, [r5, #52]
	mov	r1, r4
	bl	__aeabi_uidiv
	ldrh	r5, [r5, #94]
	mul	r7, r4, r7
	mov	r1, r4
	mla	r5, r5, r0, r7
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r5, r1
	uxth	r0, r1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L52:
	.align	2
.L51:
	.word	.LANCHOR0
	.fnend
	.size	V2P_block, .-V2P_block
	.align	2
	.global	P2V_plane
	.type	P2V_plane, %function
P2V_plane:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L54
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldrh	r1, [r3, #94]
	ldrh	r4, [r3, #52]
	bl	__aeabi_uidiv
	mov	r1, r4
	mul	r5, r0, r4
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r5, r1
	uxth	r0, r1
	ldmfd	sp!, {r4, r5, r6, pc}
.L55:
	.align	2
.L54:
	.word	.LANCHOR0
	.fnend
	.size	P2V_plane, .-P2V_plane
	.align	2
	.global	P2V_block_in_plane
	.type	P2V_block_in_plane, %function
P2V_block_in_plane:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, .L57
	ldrh	r1, [r4, #94]
	bl	__aeabi_uidivmod
	uxth	r0, r1
	ldrh	r1, [r4, #52]
	bl	__aeabi_uidiv
	uxth	r0, r0
	ldmfd	sp!, {r4, pc}
.L58:
	.align	2
.L57:
	.word	.LANCHOR0
	.fnend
	.size	P2V_block_in_plane, .-P2V_block_in_plane
	.align	2
	.global	ftl_cmp_data_ver
	.type	ftl_cmp_data_ver, %function
ftl_cmp_data_ver:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r0, r1
	bls	.L60
	rsb	r0, r1, r0
	cmp	r0, #-2147483648
	movhi	r0, #0
	movls	r0, #1
	bx	lr
.L60:
	rsb	r0, r0, r1
	cmp	r0, #-2147483648
	movls	r0, #0
	movhi	r0, #1
	bx	lr
	.fnend
	.size	ftl_cmp_data_ver, .-ftl_cmp_data_ver
	.align	2
	.global	FtlFreeSysBlkQueueEmpty
	.type	FtlFreeSysBlkQueueEmpty, %function
FtlFreeSysBlkQueueEmpty:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L63
	ldrh	r0, [r3, #214]
	rsbs	r0, r0, #1
	movcc	r0, #0
	bx	lr
.L64:
	.align	2
.L63:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueEmpty, .-FtlFreeSysBlkQueueEmpty
	.align	2
	.global	FtlFreeSysBlkQueueFull
	.type	FtlFreeSysBlkQueueFull, %function
FtlFreeSysBlkQueueFull:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L66
	ldrh	r0, [r3, #214]
	sub	r3, r0, #1024
	rsbs	r0, r3, #0
	adc	r0, r0, r3
	bx	lr
.L67:
	.align	2
.L66:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.align	2
	.global	FtlFreeSysBLkSort
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L72
	movw	r2, #2292
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldrh	r4, [r3, r2]
	ldrh	r2, [r3, #214]
	and	r4, r4, #31
	cmp	r2, #0
	ldrneh	r1, [r3, #210]
	movne	r0, #0
	ldrneh	r2, [r3, #212]
	bne	.L70
	ldmfd	sp!, {r4, r5, pc}
.L71:
	add	ip, r3, r1, asl #1
	add	r1, r1, #1
	add	r0, r0, #1
	ldrh	r5, [ip, #216]
	add	ip, r3, r2, asl #1
	add	r2, r2, #1
	mov	r1, r1, asl #22
	uxth	r0, r0
	mov	r2, r2, asl #22
	mov	r1, r1, lsr #22
	strh	r5, [ip, #216]	@ movhi
	mov	r2, r2, lsr #22
.L70:
	cmp	r0, r4
	ldr	ip, .L72
	blt	.L71
	strh	r1, [ip, #210]	@ movhi
	strh	r2, [ip, #212]	@ movhi
	ldmfd	sp!, {r4, r5, pc}
.L73:
	.align	2
.L72:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	2
	.global	IsInFreeQueue
	.type	IsInFreeQueue, %function
IsInFreeQueue:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L80
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrh	r1, [r2, #214]
	cmp	r1, #1024
	ldrneh	ip, [r2, #210]
	movne	r3, #0
	bne	.L76
	b	.L78
.L77:
	add	r4, r3, ip
	mov	r4, r4, asl #22
	add	r4, r2, r4, lsr #21
	ldrh	r4, [r4, #216]
	cmp	r4, r0
	beq	.L79
	add	r3, r3, #1
.L76:
	cmp	r3, r1
	bcc	.L77
.L78:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L79:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L81:
	.align	2
.L80:
	.word	.LANCHOR0
	.fnend
	.size	IsInFreeQueue, .-IsInFreeQueue
	.align	2
	.global	insert_data_list
	.type	insert_data_list, %function
insert_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r1, .L97
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldrh	r3, [r1, #36]
	cmp	r3, r0
	bls	.L83
	mov	ip, #6
	ldr	r4, [r1, #2312]
	mul	ip, ip, r0
	mvn	r6, #0
	add	r2, r4, ip
	strh	r6, [r2, #2]	@ movhi
	strh	r6, [r4, ip]	@ movhi
	ldr	r3, [r1, #2316]
	cmp	r3, #0
	beq	.L96
.L84:
	ldr	r7, [r1, #2320]
	mov	r5, r0, asl #1
	ldrh	r1, [r2, #4]
	ldr	sl, .L97+4
	ldrh	r8, [r7, r5]
	cmp	r1, #0
	mulne	r6, r1, r8
	ldr	r8, .L97
	ldr	fp, [r8, #2312]
	ldrh	r9, [r8, #36]
	rsb	r1, fp, r3
	ldr	r8, [r8, #2324]
	mov	r1, r1, asr #1
	add	r5, r8, r5
	str	r5, [sp, #12]
	mov	r5, #0
	mul	r1, sl, r1
	str	r9, [sp, #8]
	str	r8, [sp, #4]
	uxth	r1, r1
.L91:
	add	r5, r5, #1
	ldr	sl, [sp, #8]
	uxth	r5, r5
	cmp	r5, sl
	bhi	.L83
	cmp	r0, r1
	beq	.L83
	mov	r9, r1, asl #1
	ldrh	r8, [r3, #4]
	ldrh	sl, [r7, r9]
	cmp	r8, #0
	mvneq	r8, #0
	mulne	r8, r8, sl
	cmp	r8, r6
	bne	.L87
	ldr	r8, [sp, #4]
	ldrh	sl, [r8, r9]
	ldr	r9, [sp, #12]
	ldrh	r8, [r9, #0]
	cmp	sl, r8
	bcc	.L89
	b	.L88
.L87:
	bhi	.L88
.L89:
	ldrh	r8, [r3, #0]
	movw	sl, #65535
	cmp	r8, sl
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	ldreq	r3, .L97
	streq	r2, [r3, #2328]
	beq	.L83
.L90:
	mov	r1, #6
	mla	r3, r1, r8, fp
	mov	r1, r8
	b	.L91
.L88:
	strh	r1, [r4, ip]	@ movhi
	ldrh	r1, [r3, #2]
	strh	r1, [r2, #2]	@ movhi
	ldr	r1, .L97
	ldr	ip, [r1, #2316]
	cmp	r3, ip
	bne	.L92
	strh	r0, [r3, #2]	@ movhi
.L96:
	str	r2, [r1, #2316]
	b	.L83
.L92:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #2312]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L83:
	mov	r0, #0
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L98:
	.align	2
.L97:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	insert_data_list, .-insert_data_list
	.align	2
	.global	INSERT_DATA_LIST
	.type	INSERT_DATA_LIST, %function
INSERT_DATA_LIST:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	insert_data_list
	movw	r1, #2332
	ldr	r3, .L101
	ldrh	r2, [r3, r1]
	add	r2, r2, #1
	uxth	r2, r2
	strh	r2, [r3, r1]	@ movhi
	ldrh	r3, [r3, #36]
	cmp	r3, r2
	ldmcsfd	sp!, {r3, pc}
	ldr	r0, .L101+4
	mov	r2, #214
	ldr	r1, .L101+8
	ldmfd	sp!, {r3, lr}
	b	printk
.L102:
	.align	2
.L101:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1
	.fnend
	.size	INSERT_DATA_LIST, .-INSERT_DATA_LIST
	.align	2
	.global	insert_free_list
	.type	insert_free_list, %function
insert_free_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	movw	r6, #65535
	cmp	r0, r6
	beq	.L104
	ldr	r1, .L111
	mov	r5, #6
	mul	ip, r5, r0
	mvn	r3, #0
	ldr	r4, [r1, #2312]
	add	r2, r4, ip
	strh	r3, [r2, #2]	@ movhi
	strh	r3, [r4, ip]	@ movhi
	ldr	r3, [r1, #2336]
	cmp	r3, #0
	beq	.L110
.L105:
	ldr	sl, [r1, #2312]
	mov	r8, r0, asl #1
	ldr	r7, [r1, #2324]
	rsb	r1, sl, r3
	ldr	r9, .L111+4
	mov	r1, r1, asr #1
	ldrh	r8, [r7, r8]
	mul	r1, r9, r1
	mov	r9, r5
	uxth	r1, r1
.L108:
	mov	r5, r1, asl #1
	ldrh	r5, [r7, r5]
	cmp	r5, r8
	bcs	.L106
	ldrh	r5, [r3, #0]
	cmp	r5, r6
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	beq	.L104
.L107:
	mla	r3, r9, r5, sl
	mov	r1, r5
	b	.L108
.L106:
	ldrh	r5, [r3, #2]
	strh	r5, [r2, #2]	@ movhi
	strh	r1, [r4, ip]	@ movhi
	ldr	r1, .L111
	ldr	ip, [r1, #2336]
	cmp	r3, ip
	bne	.L109
	strh	r0, [r3, #2]	@ movhi
.L110:
	str	r2, [r1, #2336]
	b	.L104
.L109:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #2312]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L104:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L112:
	.align	2
.L111:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	insert_free_list, .-insert_free_list
	.align	2
	.global	INSERT_FREE_LIST
	.type	INSERT_FREE_LIST, %function
INSERT_FREE_LIST:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	insert_free_list
	movw	r1, #2340
	ldr	r3, .L115
	ldrh	r2, [r3, r1]
	add	r2, r2, #1
	uxth	r2, r2
	strh	r2, [r3, r1]	@ movhi
	ldrh	r3, [r3, #36]
	cmp	r3, r2
	ldmcsfd	sp!, {r3, pc}
	ldr	r0, .L115+4
	mov	r2, #207
	ldr	r1, .L115+8
	ldmfd	sp!, {r3, lr}
	b	printk
.L116:
	.align	2
.L115:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+17
	.fnend
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.align	2
	.global	List_remove_node
	.type	List_remove_node, %function
List_remove_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r5, #6
	ldr	r3, .L123
	mov	r7, r0
	mul	r5, r5, r1
	ldr	r6, [r3, #2312]
	movw	r3, #65535
	add	r4, r6, r5
	ldrh	r2, [r4, #2]
	cmp	r2, r3
	bne	.L118
	ldr	r3, [r0, #0]
	cmp	r4, r3
	beq	.L118
	ldr	r0, .L123+4
	mov	r2, #372
	ldr	r1, .L123+8
	bl	printk
.L118:
	ldr	r3, [r7, #0]
	movw	r2, #65535
	cmp	r4, r3
	ldrh	r3, [r6, r5]
	bne	.L119
	cmp	r3, r2
	ldrne	r2, .L123
	movne	r1, #6
	moveq	r3, #0
	streq	r3, [r7, #0]
	ldrne	r2, [r2, #2312]
	mlane	r3, r1, r3, r2
	mvnne	r2, #0
	strne	r3, [r7, #0]
	strneh	r2, [r3, #2]	@ movhi
	b	.L121
.L119:
	cmp	r3, r2
	ldrh	r2, [r4, #2]
	bne	.L122
	cmp	r2, r3
	beq	.L121
	mov	r1, #6
	ldr	r3, .L123
	mul	r2, r1, r2
	mvn	r1, #0
	ldr	r3, [r3, #2312]
	strh	r1, [r3, r2]	@ movhi
	b	.L121
.L122:
	ldr	r0, .L123
	mov	r1, #6
	ldr	ip, [r0, #2312]
	mla	r3, r1, r3, ip
	strh	r2, [r3, #2]	@ movhi
	ldrh	ip, [r4, #2]
	ldrh	r2, [r6, r5]
	ldr	r3, [r0, #2312]
	mul	r1, r1, ip
	strh	r2, [r3, r1]	@ movhi
.L121:
	mvn	r3, #0
	mov	r0, #0
	strh	r3, [r6, r5]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L124:
	.align	2
.L123:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+34
	.fnend
	.size	List_remove_node, .-List_remove_node
	.align	2
	.global	List_pop_index_node
	.type	List_pop_index_node, %function
List_pop_index_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r0, #0]
	cmp	r3, #0
	beq	.L130
	ldr	r2, .L131
	movw	lr, #65535
	mov	ip, #6
	ldr	r4, [r2, #2312]
	b	.L127
.L129:
	mla	r3, ip, r2, r4
	sub	r1, r1, #1
	uxth	r1, r1
.L127:
	cmp	r1, #0
	beq	.L128
	ldrh	r2, [r3, #0]
	cmp	r2, lr
	bne	.L129
.L128:
	rsb	r4, r4, r3
	ldr	r3, .L131+4
	mov	r4, r4, asr #1
	mul	r4, r3, r4
	uxth	r4, r4
	mov	r1, r4
	bl	List_remove_node
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L130:
	movw	r0, #65535
	ldmfd	sp!, {r4, pc}
.L132:
	.align	2
.L131:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	List_pop_index_node, .-List_pop_index_node
	.align	2
	.global	List_pop_head_node
	.type	List_pop_head_node, %function
List_pop_head_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r1, #0
	b	List_pop_index_node
	.fnend
	.size	List_pop_head_node, .-List_pop_head_node
	.align	2
	.global	List_get_gc_head_node
	.type	List_get_gc_head_node, %function
List_get_gc_head_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L140
	ldr	r3, [r2, #2316]
	cmp	r3, #0
	beq	.L139
	ldr	r2, [r2, #2312]
	movw	r1, #65535
	mov	ip, #6
	b	.L136
.L138:
	mla	r3, ip, r3, r2
	sub	r0, r0, #1
	uxth	r0, r0
.L136:
	cmp	r0, #0
	beq	.L137
	ldrh	r3, [r3, #0]
	cmp	r3, r1
	bne	.L138
	mov	r0, r1
	bx	lr
.L137:
	rsb	r3, r2, r3
	ldr	r0, .L140+4
	mov	r3, r3, asr #1
	mul	r0, r0, r3
	uxth	r0, r0
	bx	lr
.L139:
	movw	r0, #65535
	bx	lr
.L141:
	.align	2
.L140:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	List_get_gc_head_node, .-List_get_gc_head_node
	.align	2
	.global	List_update_data_list
	.type	List_update_data_list, %function
List_update_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	movw	r2, #2344
	ldr	r3, .L151
	mov	r4, r0
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L143
	movw	r2, #2392
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L143
	movw	r2, #2440
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L143
	mov	r7, #6
	ldr	r8, [r3, #2312]
	mul	r7, r7, r0
	ldr	r2, [r3, #2316]
	add	r5, r8, r7
	cmp	r5, r2
	beq	.L143
	ldr	r2, [r3, #2320]
	mov	r3, r0, asl #1
	movw	r6, #65535
	ldrh	sl, [r2, r3]
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	mulne	sl, r3, sl
	ldrh	r3, [r5, #2]
	mvneq	sl, #0
	cmp	r3, r6
	bne	.L145
	ldrh	r3, [r8, r7]
	cmp	r3, r6
	bne	.L145
	ldr	r0, .L151+4
	movw	r2, #463
	ldr	r1, .L151+8
	bl	printk
.L145:
	ldrh	r2, [r5, #2]
	cmp	r2, r6
	bne	.L146
	ldrh	r3, [r8, r7]
	cmp	r3, r6
	beq	.L143
.L146:
	ldr	r6, .L151
	mov	r1, #6
	ldr	r3, [r6, #2312]
	mla	r2, r1, r2, r3
	ldr	r1, .L151+12
	rsb	r3, r3, r2
	mov	r3, r3, asr #1
	mul	r3, r1, r3
	ldr	r1, [r6, #2320]
	uxth	r3, r3
	mov	r3, r3, asl #1
	ldrh	r1, [r1, r3]
	ldrh	r3, [r2, #4]
	cmp	r3, #0
	mulne	r3, r3, r1
	mvneq	r3, #0
	cmp	sl, r3
	bcs	.L143
	ldr	r0, .L151+16
	mov	r1, r4
	movw	r5, #2332
	bl	List_remove_node
	ldrh	r3, [r6, r5]
	cmp	r3, #0
	bne	.L148
	ldr	r0, .L151+4
	movw	r2, #474
	ldr	r1, .L151+8
	bl	printk
.L148:
	ldr	r3, .L151
	mov	r0, r4
	ldrh	r2, [r3, r5]
	sub	r2, r2, #1
	strh	r2, [r3, r5]	@ movhi
	bl	INSERT_DATA_LIST
.L143:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L152:
	.align	2
.L151:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+51
	.word	-1431655765
	.word	.LANCHOR0+2316
	.fnend
	.size	List_update_data_list, .-List_update_data_list
	.align	2
	.global	select_l2p_ram_region
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L165
	movw	r1, #65535
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	ldrh	r3, [r2, #134]
	ldr	r0, [r2, #2488]
	mov	r2, #0
	mov	r4, r2
	b	.L154
.L156:
	add	r2, r2, #12
	add	ip, r0, r2
	ldrh	ip, [ip, #-12]
	cmp	ip, r1
	beq	.L155
	add	r4, r4, #1
	uxth	r4, r4
.L154:
	cmp	r4, r3
	bne	.L156
	mov	r1, #0
	mov	r4, r3
	mov	r5, #-2147483648
	mov	r2, r1
	b	.L157
.L159:
	add	ip, r0, r1
	ldr	ip, [ip, #4]
	cmp	ip, #0
	blt	.L158
	cmp	ip, r5
	movcc	r5, ip
	movcc	r4, r2
.L158:
	add	r2, r2, #1
	add	r1, r1, #12
	uxth	r2, r2
.L157:
	cmp	r2, r3
	bne	.L159
	cmp	r4, r3
	bcc	.L155
	ldr	r1, .L165
	movw	r2, #2492
	mov	r4, r3
	mvn	ip, #0
	ldrh	r6, [r1, r2]
	mov	r1, #0
	mov	r2, r1
	b	.L160
.L162:
	add	r5, r0, r1
	ldr	r5, [r5, #4]
	cmp	r5, ip
	bcs	.L161
	ldrh	r7, [r0, r1]
	cmp	r7, r6
	movne	ip, r5
	movne	r4, r2
.L161:
	add	r2, r2, #1
	add	r1, r1, #12
	uxth	r2, r2
.L160:
	cmp	r2, r3
	bne	.L162
	cmp	r4, r2
	bcc	.L155
	ldr	r0, .L165+4
	movw	r2, #787
	ldr	r1, .L165+8
	bl	printk
.L155:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L166:
	.align	2
.L165:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+73
	.fnend
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L173
	movw	r2, #2494
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldrh	r1, [r3, r2]
	cmp	r1, #4
	add	ip, r1, #1
	mov	r1, r2
	strh	ip, [r3, r2]	@ movhi
	bhi	.L168
	cmp	r0, #0
	ldmeqfd	sp!, {r4, r5, pc}
.L168:
	mov	r2, #0
	ldrh	r4, [r3, #36]
	strh	r2, [r3, r1]	@ movhi
	movw	ip, #65535
	str	r2, [r3, #2496]
	ldr	r1, [r3, #2320]
	mov	r3, r2
	ldr	r2, .L173
	b	.L170
.L172:
	ldrh	r0, [r1], #2
	add	r3, r3, #1
	cmp	r0, ip
	uxth	r3, r3
	ldrne	r5, [r2, #2496]
	addne	r0, r0, r5
	strne	r0, [r2, #2496]
.L170:
	cmp	r3, r4
	bne	.L172
	ldmfd	sp!, {r4, r5, pc}
.L174:
	.align	2
.L173:
	.word	.LANCHOR0
	.fnend
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.align	2
	.global	ftl_set_blk_mode
	.type	ftl_set_blk_mode, %function
ftl_set_blk_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L179
	mov	r2, r0, lsr #5
	cmp	r1, #0
	and	r0, r0, #31
	uxth	r2, r2
	mov	ip, #1
	ldr	r3, [r3, #2500]
	ldr	r1, [r3, r2, asl #2]
	orrne	r0, r1, ip, asl r0
	biceq	r0, r1, ip, asl r0
	str	r0, [r3, r2, asl #2]
	bx	lr
.L180:
	.align	2
.L179:
	.word	.LANCHOR0
	.fnend
	.size	ftl_set_blk_mode, .-ftl_set_blk_mode
	.align	2
	.global	ftl_get_blk_mode
	.type	ftl_get_blk_mode, %function
ftl_get_blk_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L182
	mov	r2, r0, lsr #5
	and	r0, r0, #31
	ldr	r3, [r3, #2500]
	ldr	r3, [r3, r2, asl #2]
	mov	r0, r3, lsr r0
	and	r0, r0, #1
	bx	lr
.L183:
	.align	2
.L182:
	.word	.LANCHOR0
	.fnend
	.size	ftl_get_blk_mode, .-ftl_get_blk_mode
	.align	2
	.global	ftl_sb_update_avl_pages
	.type	ftl_sb_update_avl_pages, %function
ftl_sb_update_avl_pages:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	r3, #0
	strh	r3, [r0, #4]	@ movhi
	ldr	r3, .L191
	movw	ip, #65535
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldrh	r3, [r3, #28]
	b	.L185
.L187:
	add	r4, r0, r2, asl #1
	add	r2, r2, #1
	ldrh	r4, [r4, #16]
	uxth	r2, r2
	cmp	r4, ip
	ldrneh	r4, [r0, #4]
	addne	r4, r4, #1
	strneh	r4, [r0, #4]	@ movhi
.L185:
	cmp	r2, r3
	bcc	.L187
	ldr	r2, .L191
	mov	ip, r0
	movw	r4, #65535
	mvn	r1, r1
	ldrh	r5, [r2, #98]
	mov	r2, #0
	b	.L188
.L190:
	ldrh	r6, [ip, #16]
	add	r2, r2, #1
	add	ip, ip, #2
	cmp	r6, r4
	uxth	r2, r2
	ldrneh	r6, [r0, #4]
	addne	r6, r5, r6
	addne	r6, r6, r1
	strneh	r6, [r0, #4]	@ movhi
.L188:
	cmp	r2, r3
	bne	.L190
	ldmfd	sp!, {r4, r5, r6, pc}
.L192:
	.align	2
.L191:
	.word	.LANCHOR0
	.fnend
	.size	ftl_sb_update_avl_pages, .-ftl_sb_update_avl_pages
	.align	2
	.global	FtlSlcSuperblockCheck
	.type	FtlSlcSuperblockCheck, %function
FtlSlcSuperblockCheck:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldrh	r3, [r0, #4]
	cmp	r3, #0
	bxeq	lr
	ldrh	r2, [r0, #0]
	movw	r3, #65535
	cmp	r2, r3
	bxeq	lr
	ldrb	r2, [r0, #6]	@ zero_extendqisi2
	ldr	r1, .L198
	add	r2, r0, r2, asl #1
	ldrh	ip, [r1, #28]
	mov	r1, #0
	ldrh	r2, [r2, #16]
	b	.L195
.L197:
	ldrb	r2, [r0, #6]	@ zero_extendqisi2
	add	r2, r2, #1
	uxtb	r2, r2
	strb	r2, [r0, #6]
	cmp	r2, ip
	streqb	r1, [r0, #6]
	ldreqh	r2, [r0, #2]
	addeq	r2, r2, #1
	streqh	r2, [r0, #2]	@ movhi
	ldrb	r2, [r0, #6]	@ zero_extendqisi2
	add	r2, r0, r2, asl #1
	ldrh	r2, [r2, #16]
.L195:
	cmp	r2, r3
	beq	.L197
	bx	lr
.L199:
	.align	2
.L198:
	.word	.LANCHOR0
	.fnend
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
	.align	2
	.global	make_superblock
	.type	make_superblock, %function
make_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r0
	ldr	r3, .L205
	ldrh	r2, [r0, #0]
	ldrh	r3, [r3, #36]
	cmp	r2, r3
	bcc	.L201
	ldr	r0, .L205+4
	movw	r2, #2110
	ldr	r1, .L205+8
	bl	printk
.L201:
	mov	r5, #0
	ldr	r6, .L205
	strh	r5, [r4, #4]	@ movhi
	mvn	r7, #0
	strb	r5, [r4, #7]
	b	.L202
.L204:
	add	r3, r6, r5
	ldrh	r1, [r4, #0]
	add	r8, r5, #8
	add	r5, r5, #1
	ldrb	r0, [r3, #56]	@ zero_extendqisi2
	bl	V2P_block
	mov	r8, r8, asl #1
	uxth	r5, r5
	strh	r7, [r4, r8]	@ movhi
	mov	sl, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	streqh	sl, [r4, r8]	@ movhi
	ldreqb	r3, [r4, #7]	@ zero_extendqisi2
	addeq	r3, r3, #1
	streqb	r3, [r4, #7]
.L202:
	ldrh	r3, [r6, #28]
	cmp	r3, r5
	bhi	.L204
	ldr	r2, .L205
	mov	r0, #0
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	strb	r0, [r4, #9]
	ldrh	r2, [r2, #98]
	mul	r3, r2, r3
	strh	r3, [r4, #4]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L206:
	.align	2
.L205:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+95
	.fnend
	.size	make_superblock, .-make_superblock
	.align	2
	.global	update_multiplier_value
	.type	update_multiplier_value, %function
update_multiplier_value:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, #0
	mov	r7, r0
	mov	r5, r4
	ldr	r6, .L212
	b	.L208
.L210:
	add	r3, r6, r5
	mov	r1, r7
	add	r5, r5, #1
	ldrb	r0, [r3, #56]	@ zero_extendqisi2
	bl	V2P_block
	uxth	r5, r5
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldreqh	r3, [r6, #98]
	addeq	r4, r4, r3
	uxtheq	r4, r4
.L208:
	ldrh	r3, [r6, #28]
	cmp	r3, r5
	bhi	.L210
	cmp	r4, #0
	beq	.L211
	mov	r1, r4
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r4, r0
.L211:
	ldr	r3, .L212
	mov	r2, #6
	mov	r0, #0
	ldr	r3, [r3, #2312]
	mla	r7, r2, r7, r3
	strh	r4, [r7, #4]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L213:
	.align	2
.L212:
	.word	.LANCHOR0
	.fnend
	.size	update_multiplier_value, .-update_multiplier_value
	.align	2
	.global	GetFreeBlockMinEraseCount
	.type	GetFreeBlockMinEraseCount, %function
GetFreeBlockMinEraseCount:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L217
	ldr	r0, [r3, #2336]
	cmp	r0, #0
	bxeq	lr
	ldr	r2, [r3, #2312]
	rsb	r0, r2, r0
	ldr	r2, .L217+4
	mov	r0, r0, asr #1
	mul	r0, r2, r0
	ldr	r2, [r3, #2324]
	uxth	r0, r0
	mov	r3, r0, asl #1
	ldrh	r0, [r2, r3]
	bx	lr
.L218:
	.align	2
.L217:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	GetFreeBlockMinEraseCount, .-GetFreeBlockMinEraseCount
	.align	2
	.global	GetFreeBlockMaxEraseCount
	.type	GetFreeBlockMaxEraseCount, %function
GetFreeBlockMaxEraseCount:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L226
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldr	r3, [r2, #2336]
	cmp	r3, #0
	beq	.L225
	movw	r1, #2340
	mov	ip, #7
	ldrh	r1, [r2, r1]
	mov	r5, #6
	movw	r4, #65535
	mul	r1, ip, r1
	mov	r1, r1, asr #3
	cmp	r0, r1
	uxthgt	r0, r1
	ldr	r1, [r2, #2312]
	ldr	r2, .L226+4
	rsb	r3, r1, r3
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	mov	r2, #0
	uxth	r3, r3
	b	.L222
.L224:
	mul	ip, r5, r3
	ldrh	ip, [r1, ip]
	cmp	ip, r4
	beq	.L223
	add	r2, r2, #1
	mov	r3, ip
	uxth	r2, r2
.L222:
	cmp	r2, r0
	bne	.L224
.L223:
	ldr	r2, .L226
	mov	r3, r3, asl #1
	ldr	r2, [r2, #2324]
	ldrh	r0, [r2, r3]
	ldmfd	sp!, {r4, r5, pc}
.L225:
	mov	r0, r3
	ldmfd	sp!, {r4, r5, pc}
.L227:
	.align	2
.L226:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	GetFreeBlockMaxEraseCount, .-GetFreeBlockMaxEraseCount
	.align	2
	.global	FtlPrintInfo2buf
	.type	FtlPrintInfo2buf, %function
FtlPrintInfo2buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r1, #64
	ldr	r2, .L238
	mov	r9, r0
	bl	snprintf
	ldr	r5, .L238+4
	mov	r1, #64
	ldr	r2, .L238+8
	movw	r7, #2340
	movw	r6, #2344
	ldr	r3, [r5, #112]
	add	r4, r9, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+12
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+16
	ldr	r3, [r5, #2504]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+20
	ldr	r3, [r5, #2496]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+24
	ldr	r3, [r5, #2508]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+28
	ldr	r3, [r5, #2512]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+32
	ldr	r3, [r5, #2516]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+36
	ldr	r3, [r5, #2520]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+40
	ldr	r3, [r5, #2524]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+44
	ldr	r3, [r5, #2528]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	ldr	r3, [r5, #2532]
	mov	r1, #64
	ldr	r2, .L238+48
	mov	r3, r3, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	ldr	r3, [r5, #2536]
	mov	r1, #64
	ldr	r2, .L238+52
	mov	r3, r3, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+56
	ldr	r3, [r5, #2540]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+60
	ldr	r3, [r5, #2544]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+64
	ldrh	r3, [r5, #154]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+68
	ldrh	r3, [r5, r7]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+72
	ldr	r3, [r5, #2548]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+76
	ldr	r3, [r5, #2552]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+80
	ldr	r3, [r5, #2556]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+84
	ldr	r3, [r5, #2560]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+88
	ldr	r3, [r5, #2564]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+92
	ldr	r3, [r5, #2568]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2294
	mov	r1, #64
	ldr	r2, .L238+96
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2292
	mov	r1, #64
	ldr	r2, .L238+100
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+104
	ldr	r3, [r5, #136]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+108
	ldr	r3, [r5, #128]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+112
	ldr	r3, [r5, #24]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+116
	ldrh	r3, [r5, #214]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+120
	ldrh	r3, [r5, #36]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2572
	mov	r1, #64
	ldr	r2, .L238+124
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+128
	ldr	r3, [r5, #40]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r3, #2576
	mov	r1, #64
	ldr	r2, .L238+132
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+136
	ldrh	r3, [r5, #148]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2346
	mov	r1, #64
	ldr	r2, .L238+140
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+144
	ldrb	r3, [r5, #2350]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	ldrh	r3, [r5, r6]
	mov	r1, #64
	ldr	r2, .L238+148
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+152
	ldrb	r3, [r5, #2352]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2348
	mov	r1, #64
	ldr	r2, .L238+156
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	ldrh	r3, [r5, r6]
	ldr	ip, [r5, #2320]
	mov	r1, #64
	ldr	r2, .L238+160
	add	r6, r6, #48
	mov	r3, r3, asl #1
	ldrh	r3, [ip, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2394
	mov	r1, #64
	ldr	r2, .L238+164
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+168
	ldrb	r3, [r5, #2398]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	ldrh	r3, [r5, r6]
	mov	r1, #64
	ldr	r2, .L238+172
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+176
	ldrb	r3, [r5, #2400]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2396
	mov	r1, #64
	ldr	r2, .L238+180
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	ldrh	r3, [r5, r6]
	ldr	ip, [r5, #2320]
	mov	r1, #64
	ldr	r2, .L238+184
	add	r6, r6, #196
	mov	r3, r3, asl #1
	ldrh	r3, [ip, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2442
	mov	r1, #64
	ldr	r2, .L238+188
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+192
	ldrb	r3, [r5, #2446]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2440
	mov	r1, #64
	ldr	r2, .L238+196
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+200
	ldrb	r3, [r5, #2448]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2444
	mov	r1, #64
	ldr	r2, .L238+204
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #2590
	mov	r1, #64
	ldr	r2, .L238+208
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+212
	ldrb	r3, [r5, #2594]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+216
	ldrh	r3, [r5, r6]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+220
	ldrb	r3, [r5, #2596]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r3, #2592
	mov	r1, #64
	ldr	r2, .L238+224
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	ldr	r3, [r5, #2720]
	mov	r1, #64
	ldr	r2, .L238+228
	str	r3, [sp, #0]
	ldr	r3, [r5, #2712]
	str	r3, [sp, #4]
	ldr	r3, [r5, #2716]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+232
	ldr	r3, [r5, #2708]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+236
	ldr	r3, [r5, #2732]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #3148
	mov	r1, #64
	ldr	r2, .L238+240
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #3150
	mov	r1, #64
	ldr	r2, .L238+244
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	mov	r1, #64
	ldr	r2, .L238+248
	ldr	r3, [r5, #3152]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	movw	r3, #3156
	mov	r1, #64
	ldr	r2, .L238+252
	ldrh	r3, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	bl	GetFreeBlockMinEraseCount
	mov	r1, #64
	ldr	r2, .L238+256
	mov	r3, r0
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r0, [r5, r7]
	bl	GetFreeBlockMaxEraseCount
	mov	r1, #64
	ldr	r2, .L238+260
	mov	r3, r0
	mov	r0, r4
	bl	snprintf
	ldr	r3, .L238+264
	ldr	r3, [r3, #0]
	cmp	r3, #1
	add	r4, r4, r0
	bne	.L234
.L229:
	ldrh	r3, [r5, r6]
	movw	r2, #65535
	cmp	r3, r2
	beq	.L231
	ldr	ip, [r5, #2320]
	mov	r3, r3, asl #1
	mov	r0, r4
	mov	r1, #64
	ldr	r2, .L238+268
	ldrh	r3, [ip, r3]
	bl	snprintf
	add	r4, r4, r0
.L231:
	mov	r0, #0
	ldr	r5, .L238+4
	bl	List_get_gc_head_node
	mov	r6, #0
	movw	r8, #65535
	mov	sl, #6
	uxth	r0, r0
.L233:
	cmp	r0, r8
	beq	.L232
	str	r0, [sp, #0]
	mov	r3, r0, asl #1
	ldr	r2, [r5, #2320]
	mov	r1, #64
	mul	r7, sl, r0
	mov	r0, r4
	ldrh	r2, [r2, r3]
	str	r2, [sp, #4]
	ldr	r2, [r5, #2312]
	add	r2, r2, r7
	ldrh	r2, [r2, #4]
	str	r2, [sp, #8]
	ldr	r2, [r5, #2324]
	ldrh	r3, [r2, r3]
	ldr	r2, .L238+272
	str	r3, [sp, #12]
	mov	r3, r6
	bl	snprintf
	add	r6, r6, #1
	ldr	r3, [r5, #2312]
	cmp	r6, #16
	add	r4, r4, r0
	ldrh	r0, [r3, r7]
	bne	.L233
.L232:
	ldr	r6, .L238+4
	mov	r5, #0
	movw	r8, #65535
	mov	sl, #6
	ldr	r2, [r6, #2336]
	ldr	r3, [r6, #2312]
	rsb	r3, r3, r2
	ldr	r2, .L238+276
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r3, r3
.L235:
	cmp	r3, r8
	beq	.L234
	mul	r7, sl, r3
	str	r3, [sp, #0]
	ldr	r2, [r6, #2312]
	mov	r3, r3, asl #1
	mov	r0, r4
	mov	r1, #64
	add	r2, r2, r7
	ldrh	r2, [r2, #4]
	str	r2, [sp, #4]
	ldr	r2, [r6, #2324]
	ldrh	r3, [r2, r3]
	ldr	r2, .L238+280
	str	r3, [sp, #8]
	mov	r3, r5
	bl	snprintf
	add	r5, r5, #1
	ldr	r3, [r6, #2312]
	cmp	r5, #4
	ldrh	r3, [r3, r7]
	add	r4, r4, r0
	bne	.L235
.L234:
	rsb	r0, r9, r4
	add	sp, sp, #16
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L239:
	.align	2
.L238:
	.word	.LC2
	.word	.LANCHOR0
	.word	.LC3
	.word	.LC4
	.word	.LC5
	.word	.LC6
	.word	.LC7
	.word	.LC8
	.word	.LC9
	.word	.LC10
	.word	.LC11
	.word	.LC12
	.word	.LC13
	.word	.LC14
	.word	.LC15
	.word	.LC16
	.word	.LC17
	.word	.LC18
	.word	.LC19
	.word	.LC20
	.word	.LC21
	.word	.LC22
	.word	.LC23
	.word	.LC24
	.word	.LC25
	.word	.LC26
	.word	.LC27
	.word	.LC28
	.word	.LC29
	.word	.LC30
	.word	.LC31
	.word	.LC32
	.word	.LC33
	.word	.LC34
	.word	.LC35
	.word	.LC36
	.word	.LC37
	.word	.LC38
	.word	.LC39
	.word	.LC40
	.word	.LC41
	.word	.LC42
	.word	.LC43
	.word	.LC44
	.word	.LC45
	.word	.LC46
	.word	.LC47
	.word	.LC48
	.word	.LC49
	.word	.LC50
	.word	.LC51
	.word	.LC52
	.word	.LC53
	.word	.LC54
	.word	.LC55
	.word	.LC56
	.word	.LC57
	.word	.LC58
	.word	.LC59
	.word	.LC60
	.word	.LC61
	.word	.LC62
	.word	.LC63
	.word	.LC64
	.word	.LC65
	.word	.LC66
	.word	.LANCHOR2
	.word	.LC67
	.word	.LC68
	.word	-1431655765
	.word	.LC69
	.fnend
	.size	FtlPrintInfo2buf, .-FtlPrintInfo2buf
	.align	2
	.global	rknand_proc_ftlread
	.type	rknand_proc_ftlread, %function
rknand_proc_ftlread:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #2048
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r1
	blt	.L242
	mov	r0, r1
	ldr	r2, .L243
	mov	r1, #64
	ldr	r3, .L243+4
	bl	snprintf
	add	r5, r4, r0
	mov	r0, r5
	bl	FtlPrintInfo2buf
	add	r0, r5, r0
	rsb	r0, r4, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L242:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L244:
	.align	2
.L243:
	.word	.LC70
	.word	.LC71
	.fnend
	.size	rknand_proc_ftlread, .-rknand_proc_ftlread
	.align	2
	.global	GetSwlReplaceBlock
	.type	GetSwlReplaceBlock, %function
GetSwlReplaceBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L268
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r1, [r3, #2556]
	ldr	r2, [r3, #2568]
	cmp	r1, r2
	bcs	.L246
	mov	r2, #0
	ldrh	r1, [r3, #36]
	str	r2, [r3, #2548]
	ldr	r0, [r3, #2324]
	b	.L247
.L248:
	ldrh	lr, [r0], #2
	add	r2, r2, #1
	ldr	ip, [r3, #2548]
	add	ip, lr, ip
	str	ip, [r3, #2548]
.L247:
	cmp	r2, r1
	ldr	r4, .L268
	bcc	.L248
	ldr	r5, [r4, #2548]
	mov	r0, r5
	bl	__aeabi_uidiv
	ldrh	r1, [r4, #88]
	str	r0, [r4, #2556]
	ldr	r0, [r4, #2552]
	rsb	r0, r0, r5
	bl	__aeabi_uidiv
	str	r0, [r4, #2548]
	b	.L249
.L246:
	ldr	r2, [r3, #2564]
	cmp	r1, r2
	addhi	r2, r2, #1
	strhi	r2, [r3, #2564]
	movhi	r2, #0
	bhi	.L250
	b	.L249
.L251:
	ldr	r0, [r3, #2324]
	mov	r1, r2, asl #1
	add	r2, r2, #1
	ldrh	ip, [r0, r1]
	add	ip, ip, #1
	strh	ip, [r0, r1]	@ movhi
.L250:
	ldrh	r1, [r3, #36]
	cmp	r2, r1
	bcc	.L251
.L249:
	ldr	r3, .L268
	ldr	r6, [r3, #2568]
	ldr	r5, [r3, #2556]
	add	r2, r6, #256
	cmp	r2, r5
	mov	r2, r3
	bls	.L252
	ldr	r3, [r3, #2564]
	add	r1, r6, #768
	cmp	r1, r3
	bhi	.L267
.L252:
	movw	r3, #2340
	ldr	r4, .L268
	ldrh	r0, [r2, r3]
	add	r0, r0, r0, asl #1
	ubfx	r0, r0, #2, #16
	bl	GetFreeBlockMaxEraseCount
	add	r3, r6, #64
	cmp	r0, r3
	mov	r8, r0
	bcc	.L267
	ldr	r3, [r4, #2316]
	cmp	r3, #0
	beq	.L267
	movw	r7, #65535
	ldrh	sl, [r4, #36]
	ldr	r0, [r4, #2312]
	mov	r2, #0
	ldr	r1, [r4, #2324]
	mov	fp, r7
	mov	r4, r7
	b	.L254
.L257:
	add	r2, r2, #1
	uxth	r2, r2
	cmp	r2, sl
	bhi	.L267
	ldrh	ip, [r3, #4]
	cmp	ip, #0
	beq	.L255
	rsb	r3, r0, r3
	ldr	ip, .L268+4
	mov	r3, r3, asr #1
	mul	r3, ip, r3
	uxth	r3, r3
	mov	ip, r3, asl #1
	ldrh	ip, [r1, ip]
	cmp	ip, r6
	bls	.L264
	cmp	ip, r7
	movcc	r7, ip
	movcc	r4, r3
.L255:
	mov	ip, #6
	mla	r3, ip, r9, r0
.L254:
	ldrh	r9, [r3, #0]
	cmp	r9, fp
	bne	.L257
	b	.L256
.L264:
	mov	r4, r3
.L256:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L253
	mov	r9, r4, asl #1
	ldrh	sl, [r1, r9]
	cmp	sl, r6
	bls	.L258
	bl	GetFreeBlockMinEraseCount
	cmp	r0, r6
	ldrhi	r3, .L268
	strhi	r7, [r3, #2568]
.L258:
	cmp	sl, r5
	bcs	.L267
	add	r3, sl, #128
	cmp	r8, r3
	ble	.L267
	add	r3, sl, #256
	ldr	r6, .L268
	cmp	r3, r5
	bcc	.L259
	ldr	r3, [r6, #2564]
	add	sl, sl, #768
	cmp	sl, r3
	bcs	.L267
.L259:
	ldr	r3, [r6, #2320]
	mov	r1, r4
	ldr	r0, .L268+8
	mov	r2, r5
	ldrh	r3, [r3, r9]
	str	r3, [sp, #0]
	ldr	r3, [r6, #2324]
	ldrh	r3, [r3, r9]
	stmib	sp, {r3, r8}
	ldr	r3, [r6, #2564]
	bl	printk
	mov	r3, #1
	str	r3, [r6, #3160]
	b	.L253
.L267:
	movw	r4, #65535
.L253:
	mov	r0, r4
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L269:
	.align	2
.L268:
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LC72
	.fnend
	.size	GetSwlReplaceBlock, .-GetSwlReplaceBlock
	.align	2
	.global	free_data_superblock
	.type	free_data_superblock, %function
free_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movw	r2, #65535
	cmp	r0, r2
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	beq	.L271
	ldr	r2, .L272
	mov	r3, r0, asl #1
	mov	r1, #0
	ldr	r2, [r2, #2320]
	strh	r1, [r2, r3]	@ movhi
	bl	INSERT_FREE_LIST
.L271:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L273:
	.align	2
.L272:
	.word	.LANCHOR0
	.fnend
	.size	free_data_superblock, .-free_data_superblock
	.align	2
	.global	get_new_active_ppa
	.type	get_new_active_ppa, %function
get_new_active_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movw	r3, #65535
	ldrh	r2, [r0, #0]
	mov	r4, r0
	cmp	r2, r3
	bne	.L275
	ldr	r0, .L285
	movw	r2, #2737
	ldr	r1, .L285+4
	bl	printk
.L275:
	ldr	r3, .L285+8
	ldrh	r2, [r4, #2]
	ldrh	r3, [r3, #98]
	cmp	r2, r3
	bne	.L276
	ldr	r0, .L285
	movw	r2, #2738
	ldr	r1, .L285+4
	bl	printk
.L276:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L277
	ldr	r0, .L285
	movw	r2, #2739
	ldr	r1, .L285+4
	bl	printk
.L277:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	mov	r2, #0
	ldr	r1, .L285+8
	strb	r2, [r4, #10]
	add	r3, r4, r3, asl #1
	ldrh	r0, [r1, #28]
	movw	r1, #65535
	ldrh	r3, [r3, #16]
	b	.L278
.L280:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	add	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r4, #6]
	cmp	r3, r0
	streqb	r2, [r4, #6]
	ldreqh	r3, [r4, #2]
	addeq	r3, r3, #1
	streqh	r3, [r4, #2]	@ movhi
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	add	r3, r4, r3, asl #1
	ldrh	r3, [r3, #16]
.L278:
	cmp	r3, r1
	beq	.L280
	ldrh	r5, [r4, #2]
	movw	r1, #65535
	ldrh	r2, [r4, #4]
	orr	r5, r5, r3, asl #10
	ldr	r3, .L285+8
	sub	r2, r2, #1
	ldrh	r0, [r3, #28]
	uxth	r2, r2
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	strh	r2, [r4, #4]	@ movhi
.L282:
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, r0
	ldreqh	r3, [r4, #2]
	addeq	r3, r3, #1
	streqh	r3, [r4, #2]	@ movhi
	moveq	r3, #0
	add	ip, r4, r3, asl #1
	ldrh	ip, [ip, #16]
	cmp	ip, r1
	beq	.L282
	strb	r3, [r4, #6]
	ldr	r3, .L285+8
	ldrh	r1, [r4, #2]
	ldrh	r3, [r3, #98]
	cmp	r1, r3
	bne	.L283
	cmp	r2, #0
	beq	.L283
	ldr	r0, .L285
	movw	r2, #2760
	ldr	r1, .L285+4
	bl	printk
.L283:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, pc}
.L286:
	.align	2
.L285:
	.word	.LC1
	.word	.LANCHOR1+111
	.word	.LANCHOR0
	.fnend
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	2
	.global	FtlGcBufInit
	.type	FtlGcBufInit, %function
FtlGcBufInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L295
	mov	r1, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r7, #12
	str	r1, [r3, #3164]
	mov	r6, #1
	mov	r5, #20
	b	.L288
.L289:
	mul	r2, r7, r1
	ldr	ip, [r3, #3168]
	add	r0, ip, r2
	str	r6, [r0, #8]
	ldrh	r0, [r3, #106]
	mul	r0, r0, r1
	add	r4, r0, #3
	cmp	r0, #0
	movlt	r0, r4
	ldr	r4, [r3, #3172]
	bic	r0, r0, #3
	add	r0, r4, r0
	str	r0, [ip, r2]
	ldrh	r0, [r3, #108]
	ldr	r4, [r3, #3168]
	mul	r0, r0, r1
	add	ip, r4, r2
	add	r8, r0, #3
	cmp	r0, #0
	movlt	r0, r8
	ldr	r8, [r3, #3176]
	bic	r0, r0, #3
	add	r0, r8, r0
	str	r0, [ip, #4]
	ldr	r0, [r3, #3180]
	ldr	r2, [r4, r2]
	mla	r0, r5, r1, r0
	add	r1, r1, #1
	uxth	r1, r1
	str	r2, [r0, #8]
	ldr	r2, [ip, #4]
	str	r2, [r0, #12]
.L288:
	ldrh	r2, [r3, #28]
	cmp	r1, r2
	bcc	.L289
	b	.L294
.L291:
	mul	r1, r5, r2
	ldr	ip, [r3, #3168]
	add	r0, ip, r1
	str	r4, [r0, #8]
	ldrh	r0, [r3, #106]
	mul	r0, r0, r2
	add	r6, r0, #3
	cmp	r0, #0
	movlt	r0, r6
	ldr	r6, [r3, #3172]
	bic	r0, r0, #3
	add	r0, r6, r0
	str	r0, [ip, r1]
	ldr	r0, [r3, #3168]
	add	r0, r0, r1
	ldrh	r1, [r3, #108]
	mul	r1, r1, r2
	add	r2, r2, #1
	uxth	r2, r2
	add	ip, r1, #3
	cmp	r1, #0
	movlt	r1, ip
	ldr	ip, [r3, #3176]
	bic	r1, r1, #3
	add	r1, ip, r1
	str	r1, [r0, #4]
	b	.L293
.L294:
	ldr	r3, .L295
	mov	r5, #12
	mov	r4, #0
.L293:
	ldr	r1, [r3, #3184]
	cmp	r2, r1
	bcc	.L291
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L296:
	.align	2
.L295:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcBufInit, .-FtlGcBufInit
	.align	2
	.global	FtlGcBufFree
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L304
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r5, #20
	ldr	r6, [r3, #3184]
	mov	fp, #12
	ldr	ip, [r3, #3168]
	mov	r3, #0
	mov	r4, r3
	b	.L298
.L301:
	mul	sl, fp, r2
	add	r8, ip, sl
	ldr	r9, [ip, sl]
	ldr	sl, [r7, #8]
	cmp	r9, sl
	streq	r4, [r8, #8]
	beq	.L300
.L299:
	add	r2, r2, #1
	uxth	r2, r2
.L303:
	cmp	r2, r6
	bcc	.L301
.L300:
	add	r3, r3, #1
	uxth	r3, r3
.L298:
	cmp	r3, r1
	ldmcsfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
	mla	r7, r5, r3, r0
	mov	r2, #0
	b	.L303
.L305:
	.align	2
.L304:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcBufFree, .-FtlGcBufFree
	.align	2
	.global	FtlGcBufAlloc
	.type	FtlGcBufAlloc, %function
FtlGcBufAlloc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L313
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	mov	r6, #12
	ldr	r8, [r3, #3184]
	mov	r5, #1
	ldr	r7, [r3, #3168]
	mov	r4, #20
	mov	r3, #0
	b	.L307
.L310:
	mla	ip, r6, r2, r7
	ldr	sl, [ip, #8]
	cmp	sl, #0
	bne	.L308
	mla	r2, r4, r3, r0
	ldr	sl, [ip, #0]
	str	r5, [ip, #8]
	str	sl, [r2, #8]
	ldr	ip, [ip, #4]
	str	ip, [r2, #12]
	b	.L309
.L308:
	add	r2, r2, #1
	uxth	r2, r2
	b	.L311
.L312:
	mov	r2, #0
.L311:
	cmp	r2, r8
	bcc	.L310
.L309:
	add	r3, r3, #1
	uxth	r3, r3
.L307:
	cmp	r3, r1
	bcc	.L312
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L314:
	.align	2
.L313:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcBufAlloc, .-FtlGcBufAlloc
	.align	2
	.global	IsBlkInGcList
	.type	IsBlkInGcList, %function
IsBlkInGcList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L320
	movw	r2, #3188
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #3192]
	mov	r3, #0
	b	.L316
.L318:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L319
	add	r3, r3, #1
	uxth	r3, r3
.L316:
	cmp	r3, r1
	bne	.L318
	mov	r0, #0
	bx	lr
.L319:
	mov	r0, #1
	bx	lr
.L321:
	.align	2
.L320:
	.word	.LANCHOR0
	.fnend
	.size	IsBlkInGcList, .-IsBlkInGcList
	.align	2
	.global	FtlGcUpdatePage
	.type	FtlGcUpdatePage, %function
FtlGcUpdatePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ubfx	r0, r0, #10, #16
	mov	r5, r1
	mov	r6, r2
	bl	P2V_block_in_plane
	ldr	r3, .L326
	movw	r2, #3188
	ldrh	ip, [r3, r2]
	ldr	r2, [r3, #3192]
	mov	r3, #0
	mov	r1, r2
	b	.L323
.L325:
	ldrh	r7, [r1], #2
	cmp	r7, r0
	beq	.L324
	add	r3, r3, #1
	uxth	r3, r3
.L323:
	cmp	r3, ip
	bne	.L325
	mov	r3, r3, asl #1
	strh	r0, [r2, r3]	@ movhi
	movw	r3, #3188
	ldr	r2, .L326
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
.L324:
	ldr	r3, .L326
	mov	r2, #3200
	mov	r0, #12
	ldrh	r1, [r3, r2]
	mul	r1, r0, r1
	ldr	r0, [r3, #3196]
	add	ip, r0, r1
	stmib	ip, {r5, r6}
	str	r4, [r0, r1]
	ldrh	r1, [r3, r2]
	add	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L327:
	.align	2
.L326:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	2
	.global	FtlGcRefreshBlock
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	mov	r1, r4
	ldr	r0, .L331
	bl	printk
	ldr	r3, .L331+4
	movw	ip, #3202
	ldrh	r5, [r3, ip]
	cmp	r5, r4
	beq	.L329
	movw	r2, #3204
	ldrh	r0, [r3, r2]
	cmp	r0, r4
	beq	.L329
	movw	r1, #65535
	cmp	r5, r1
	streqh	r4, [r3, ip]	@ movhi
	beq	.L329
	cmp	r0, r1
	streqh	r4, [r3, r2]	@ movhi
.L329:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L332:
	.align	2
.L331:
	.word	.LC73
	.word	.LANCHOR0
	.fnend
	.size	FtlGcRefreshBlock, .-FtlGcRefreshBlock
	.align	2
	.global	FtlGcMarkBadPhyBlk
	.type	FtlGcMarkBadPhyBlk, %function
FtlGcMarkBadPhyBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	bl	P2V_block_in_plane
	ldr	r6, .L337
	movw	r5, #3206
	mov	r2, r4
	ldrh	r1, [r6, r5]
	mov	r7, r0
	ldr	r0, .L337+4
	bl	printk
	mov	r0, r7
	bl	FtlGcRefreshBlock
	ldrh	r1, [r6, r5]
	ldr	r2, .L337+8
	mov	r3, #0
	b	.L334
.L336:
	ldrh	r0, [r2, #2]!
	cmp	r0, r4
	beq	.L335
	add	r3, r3, #1
	uxth	r3, r3
.L334:
	cmp	r3, r1
	bne	.L336
	cmp	r3, #15
	bhi	.L335
	ldr	r2, .L337
	movw	r1, #3208
	add	r0, r2, r3, asl #1
	add	r3, r3, #1
	strh	r4, [r0, r1]	@ movhi
	movw	r1, #3206
	strh	r3, [r2, r1]	@ movhi
.L335:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L338:
	.align	2
.L337:
	.word	.LANCHOR0
	.word	.LC74
	.word	.LANCHOR0+3206
	.fnend
	.size	FtlGcMarkBadPhyBlk, .-FtlGcMarkBadPhyBlk
	.align	2
	.global	FtlGcReFreshBadBlk
	.type	FtlGcReFreshBadBlk, %function
FtlGcReFreshBadBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movw	r2, #3206
	ldr	r3, .L342
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L340
	movw	r1, #3202
	ldrh	r0, [r3, r1]
	movw	r1, #65535
	cmp	r0, r1
	bne	.L340
	movw	r4, #3242
	ldr	r5, .L342
	ldrh	r1, [r3, r4]
	cmp	r1, r2
	movcs	r2, #0
	strcsh	r2, [r3, r4]	@ movhi
	ldrh	r2, [r5, r4]
	movw	r3, #3208
	add	r2, r5, r2, asl #1
	ldrh	r0, [r2, r3]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r5, r4]
	add	r3, r3, #1
	strh	r3, [r5, r4]	@ movhi
.L340:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L343:
	.align	2
.L342:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
	.align	2
	.global	FlashReadPages
	.type	FlashReadPages, %function
FlashReadPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	mov	r8, r1
	ldr	r6, .L354
	mov	r4, r0
	mov	r7, #0
	ldrh	sl, [r6, #12]
	b	.L345
.L353:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L346
	ldr	r3, [r4, #12]
	cmp	r3, #0
	bne	.L347
.L346:
	ldr	r0, .L354+4
	mov	r2, #52
	ldr	r1, .L354+8
	bl	printk
.L347:
	add	r1, sp, #4
	mov	r2, sp
	mov	r0, r4
	bl	l2p_addr_tran
	ldr	r5, [r4, #8]
	ldr	r3, [r4, #12]
	tst	r5, #63
	ldr	ip, [r6, #3260]
	ldrb	r0, [sp, #0]	@ zero_extendqisi2
	ldrne	r5, [r6, #3244]
	ldr	r1, [sp, #4]
	mov	r2, r5
	blx	ip
	str	r0, [r4, #0]
	ldrh	r3, [r6, #14]
	cmp	r3, #4
	bne	.L349
	ldr	r3, [r4, #12]
	add	r2, r5, #2048
	ldr	r1, [sp, #4]
	ldr	ip, [r6, #3260]
	add	r3, r3, #8
	ldrb	r0, [sp, #0]	@ zero_extendqisi2
	add	r1, sl, r1
	blx	ip
	cmn	r0, #1
	beq	.L350
	ldr	r3, [r4, #12]
	ldr	r2, [r3, #12]
	cmn	r2, #1
	bne	.L351
	ldr	r2, [r3, #8]
	cmn	r2, #1
	bne	.L351
	ldr	r3, [r3, #0]
	cmn	r3, #1
	beq	.L351
.L350:
	mvn	r3, #0
	str	r3, [r4, #0]
.L351:
	ldr	r3, [r4, #0]
	sub	r2, r0, #256
	rsbs	r0, r2, #0
	adc	r0, r0, r2
	cmn	r3, #1
	moveq	r0, #0
	cmp	r0, #0
	movne	r3, #256
	strne	r3, [r4, #0]
.L349:
	ldr	r3, [r6, #3244]
	cmp	r5, r3
	bne	.L352
	ldr	r0, [r4, #8]
	cmp	r5, r0
	beq	.L352
	ldrh	r2, [r6, #54]
	mov	r1, r5
	mov	r2, r2, asl #9
	bl	memcpy
.L352:
	add	r7, r7, #1
	add	r4, r4, #20
.L345:
	cmp	r7, r8
	bne	.L353
	mov	r0, #0
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L355:
	.align	2
.L354:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+130
	.fnend
	.size	FlashReadPages, .-FlashReadPages
	.align	2
	.global	FtlGetLastWrittenPage
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L367
	cmp	r1, #1
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	.pad #92
	sub	sp, sp, #92
	ldreqh	r4, [r3, #100]
	add	r7, sp, #4
	ldrneh	r4, [r3, #98]
	mov	r5, r1
	ldr	r3, [r3, #3264]
	mov	sl, r0, asl #10
	sub	r4, r4, #1
	mov	r0, r7
	mov	r1, #1
	mov	r2, r5
	uxth	r4, r4
	str	r3, [sp, #12]
	add	r3, sp, #24
	str	r3, [sp, #16]
	sxth	r3, r4
	orr	r3, r3, sl
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #24]
	cmn	r3, #1
	bne	.L360
	b	.L366
.L363:
	add	r6, r6, r3
	mov	r0, r7
	mov	r1, #1
	mov	r2, r5
	add	r6, r6, r6, lsr #31
	ubfx	r6, r6, #1, #16
	sxth	r3, r6
	orr	r3, r3, sl
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #24]
	cmn	r3, #1
	bne	.L361
	ldr	r3, [sp, #28]
	cmn	r3, #1
	bne	.L361
	ldr	r3, [sp, #4]
	cmn	r3, #1
	subne	r4, r6, #1
	uxthne	r4, r4
	bne	.L365
.L361:
	add	r6, r6, #1
	uxth	r8, r6
	b	.L365
.L366:
	mov	r8, #0
.L365:
	sxth	r6, r8
	sxth	r3, r4
	cmp	r6, r3
	ble	.L363
.L360:
	sxth	r0, r4
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L368:
	.align	2
.L367:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.align	2
	.global	FtlLoadFactoryBbt
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r6, #0
	ldr	r4, .L376
	add	r7, r4, #160
	ldr	r3, [r4, #3288]
	ldr	r8, [r4, #3292]
	str	r3, [r4, #3276]
	str	r8, [r4, #3280]
	b	.L370
.L375:
	ldrh	r5, [r4, #94]
	mvn	r3, #0
	movw	sl, #61664
	strh	r3, [r7], #2	@ movhi
	add	r5, r5, r3
	uxth	r5, r5
	b	.L371
.L374:
	mla	r3, r3, r6, r5
	mov	r1, #1
	ldr	r0, .L376+4
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [r4, #3272]
	bl	FlashReadPages
	ldr	r3, [r4, #3268]
	cmn	r3, #1
	beq	.L372
	ldrh	r3, [r8, #0]
	cmp	r3, sl
	streqh	r5, [r7, #-2]	@ movhi
	beq	.L373
.L372:
	sub	r5, r5, #1
	uxth	r5, r5
.L371:
	ldrh	r3, [r4, #94]
	sub	r2, r3, #16
	cmp	r5, r2
	bgt	.L374
.L373:
	add	r6, r6, #1
.L370:
	ldrh	r3, [r4, #50]
	cmp	r6, r3
	bcc	.L375
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L377:
	.align	2
.L376:
	.word	.LANCHOR0
	.word	.LANCHOR0+3268
	.fnend
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	2
	.global	FlashProgPages
	.type	FlashProgPages, %function
FlashProgPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #52
	sub	sp, sp, #52
	ldr	r6, .L398
	mov	r9, r1
	str	r2, [sp, #8]
	mov	r5, r0
	str	r3, [sp, #12]
	mov	r8, #0
	ldrh	fp, [r6, #12]
	mov	sl, r0
	mov	r1, fp, asl #3
	str	r1, [sp, #4]
	b	.L379
.L389:
	ldr	r3, [r5, #8]
	cmp	r3, #0
	beq	.L380
	ldr	r3, [r5, #12]
	cmp	r3, #0
	bne	.L381
.L380:
	ldr	r0, .L398+4
	mov	r2, #92
	ldr	r1, .L398+8
	bl	printk
.L381:
	mov	r0, r5
	add	r1, sp, #24
	add	r2, sp, #20
	bl	l2p_addr_tran
	ldr	r3, [sp, #20]
	cmp	r3, #0
	bne	.L382
	ldr	r2, [sp, #24]
	ldr	r1, [sp, #4]
	cmp	r2, r1
	bcs	.L382
	b	.L397
.L384:
	ldr	r1, .L398+8
	add	r5, r5, #1
	ldr	r2, [r4, #-16]
	str	r6, [r4, #-20]
	ldr	r0, .L398+12
	bl	printk
	ldr	r0, .L398+16
	ldr	r1, [r4, #-12]
	mov	r2, #4
	mov	r3, #16
	bl	rknand_print_hex
	mov	r2, #4
	ldr	r0, .L398+20
	mov	r3, r2
	ldr	r1, [r4, #-8]
	bl	rknand_print_hex
	b	.L383
.L397:
	mov	r4, sl
	mov	r5, r3
	mvn	r6, #0
.L383:
	cmp	r5, r9
	add	r4, r4, #20
	bne	.L384
	bl	dump_stack
	b	.L385
.L382:
	ldr	r4, [r5, #8]
	tst	r4, #63
	beq	.L386
	ldr	r7, [r6, #3244]
	cmp	r4, r7
	beq	.L386
	ldrh	r2, [r6, #54]
	mov	r1, r4
	mov	r0, r7
	mov	r4, r7
	mov	r2, r2, asl #9
	bl	memcpy
.L386:
	ldr	r3, [r5, #12]
	mov	r2, r4
	ldr	ip, [r6, #3256]
	ldrb	r0, [sp, #20]	@ zero_extendqisi2
	ldr	r1, [sp, #24]
	blx	ip
	cmp	r0, #0
	str	r0, [r5, #0]
	mvnne	r3, #0
	strne	r3, [r5, #0]
	ldrh	r3, [r6, #14]
	cmp	r3, #4
	bne	.L388
	ldr	r3, [r5, #12]
	add	r2, r4, #2048
	ldr	r1, [sp, #24]
	add	r3, r3, #8
	ldr	ip, [r6, #3256]
	ldrb	r0, [sp, #20]	@ zero_extendqisi2
	add	r1, fp, r1
	blx	ip
	cmp	r0, #0
	mvnne	r3, #0
	strne	r3, [r5, #0]
.L388:
	add	r8, r8, #1
	add	r5, r5, #20
.L379:
	cmp	r8, r9
	bne	.L389
	ldr	r3, [sp, #12]
	mov	r4, sl
	cmp	r3, #0
	beq	.L385
	mov	r6, #0
	ldr	r5, .L398
	mov	r7, r6
	b	.L390
.L394:
	ldr	r3, [r5, #3244]
	add	r0, sp, #28
	mov	r1, #1
	ldr	r2, [sp, #8]
	str	r7, [r3, #0]
	ldr	r3, [r5, #3296]
	str	r7, [r3, #0]
	ldr	r3, [r4, #4]
	str	r3, [sp, #32]
	ldr	r3, [r5, #3244]
	str	r3, [sp, #36]
	ldr	r3, [r5, #3296]
	str	r3, [sp, #40]
	bl	FlashReadPages
	ldr	r8, [sp, #28]
	cmn	r8, #1
	bne	.L391
	ldr	r0, .L398+24
	ldr	r1, [r4, #4]
	bl	printk
	str	r8, [r4, #0]
.L391:
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L392
	ldr	r2, [r3, #0]
	ldr	r3, [r5, #3296]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L392
	ldr	r0, .L398+28
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #0]
.L392:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L393
	ldr	r2, [r3, #0]
	ldr	r3, [r5, #3244]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L393
	ldr	r0, .L398+32
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #0]
.L393:
	add	r6, r6, #1
	add	r4, r4, #20
.L390:
	cmp	r6, r9
	bne	.L394
.L385:
	mov	r0, #0
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L399:
	.align	2
.L398:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+145
	.word	.LC75
	.word	.LC76
	.word	.LC77
	.word	.LC78
	.word	.LC79
	.word	.LC80
	.fnend
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.global	FlashEraseBlocks
	.type	FlashEraseBlocks, %function
FlashEraseBlocks:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r5, r2
	ldr	r7, .L411
	mov	r4, r0
	mov	r6, r0
	mov	r8, #0
	ldrh	sl, [r7, #12]
	mov	fp, r7
	mov	r9, sl, asl #3
	b	.L401
.L408:
	mov	r0, r6
	add	r1, sp, #4
	mov	r2, sp
	bl	l2p_addr_tran
	ldr	r0, [sp, #0]
	cmp	r0, #0
	bne	.L402
	ldr	r3, [sp, #4]
	cmp	r3, r9
	bcs	.L402
	b	.L410
.L404:
	ldr	r1, .L411+4
	add	r6, r6, #1
	ldr	r2, [sp, #4]
	str	r7, [r4, #-20]
	ldr	r0, .L411+8
	bl	printk
	ldr	r0, .L411+12
	ldr	r1, [r4, #-12]
	mov	r2, #4
	mov	r3, #16
	bl	rknand_print_hex
	mov	r2, #4
	ldr	r0, .L411+16
	mov	r3, r2
	ldr	r1, [r4, #-8]
	bl	rknand_print_hex
	b	.L403
.L410:
	mov	r6, r0
	mvn	r7, #0
.L403:
	cmp	r6, r5
	add	r4, r4, #20
	bne	.L404
	bl	dump_stack
	b	.L405
.L402:
	ldr	r3, [r7, #3252]
	uxtb	r0, r0
	ldr	r1, [sp, #4]
	blx	r3
	cmp	r0, #0
	str	r0, [r6, #0]
	mvnne	r3, #0
	strne	r3, [r6, #0]
	ldrh	r3, [fp, #14]
	cmp	r3, #4
	bne	.L407
	ldr	r1, [sp, #4]
	ldr	r3, [r7, #3252]
	ldrb	r0, [sp, #0]	@ zero_extendqisi2
	add	r1, sl, r1
	blx	r3
	cmp	r0, #0
	mvnne	r3, #0
	strne	r3, [r6, #0]
.L407:
	add	r8, r8, #1
	add	r6, r6, #20
.L401:
	cmp	r8, r5
	bne	.L408
.L405:
	mov	r0, #0
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L412:
	.align	2
.L411:
	.word	.LANCHOR0
	.word	.LANCHOR1+160
	.word	.LC75
	.word	.LC76
	.word	.LC77
	.fnend
	.size	FlashEraseBlocks, .-FlashEraseBlocks
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r4, .L418
	ldrh	r3, [r4, #214]
	cmp	r3, #0
	movweq	r5, #65535
	beq	.L414
	ldrh	r2, [r4, #210]
	sub	r3, r3, #1
	ldr	r0, [r4, #3300]
	add	r1, r4, r2, asl #1
	add	r2, r2, #1
	bic	r2, r2, #64512
	ldrh	r5, [r1, #216]
	mov	r1, #1
	strh	r3, [r4, #214]	@ movhi
	strh	r2, [r4, #210]	@ movhi
	mov	r2, r1
	mov	r3, r5, asl #10
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r3, [r4, #2560]
	add	r3, r3, #1
	str	r3, [r4, #2560]
.L414:
	sub	r2, r5, #1
	movw	r3, #65533
	uxth	r2, r2
	cmp	r2, r3
	bls	.L415
	ldr	r3, .L418
	mov	r1, r5
	ldr	r0, .L418+4
	ldrh	r2, [r3, #214]
	bl	printk
.L416:
	b	.L416
.L415:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, pc}
.L419:
	.align	2
.L418:
	.word	.LANCHOR0
	.word	.LC81
	.fnend
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	2
	.global	ftl_map_blk_alloc_new_blk
	.type	ftl_map_blk_alloc_new_blk, %function
ftl_map_blk_alloc_new_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ldrh	r2, [r0, #10]
	mov	r5, #0
	ldr	r3, [r0, #12]
	b	.L421
.L426:
	mov	r7, r3
	add	r3, r3, #2
	ldrh	r6, [r7, #0]
	cmp	r6, #0
	bne	.L422
	bl	FtlFreeSysBlkQueueOut
	movw	r3, #65533
	sub	r2, r0, #1
	mov	r1, r0
	strh	r0, [r7, #0]	@ movhi
	uxth	r2, r2
	cmp	r2, r3
	bls	.L423
	ldr	r3, .L428
	ldr	r0, .L428+4
	ldrh	r2, [r3, #214]
	bl	printk
.L424:
	b	.L424
.L423:
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	add	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	strh	r5, [r4, #0]	@ movhi
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	b	.L425
.L422:
	add	r5, r5, #1
	uxth	r5, r5
.L421:
	cmp	r5, r2
	bne	.L426
.L425:
	ldrh	r3, [r4, #10]
	cmp	r3, r5
	bhi	.L427
	ldr	r0, .L428+8
	movw	r2, #578
	ldr	r1, .L428+12
	bl	printk
.L427:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L429:
	.align	2
.L428:
	.word	.LANCHOR0
	.word	.LC82
	.word	.LC1
	.word	.LANCHOR1+177
	.fnend
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	2
	.type	FtlFreeSysBlkQueueIn.part.7, %function
FtlFreeSysBlkQueueIn.part.7:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	beq	.L431
	bl	P2V_block_in_plane
	ldr	r4, .L432
	mov	r1, #1
	mov	r3, r5, asl #10
	mov	r2, r1
	mov	r6, r0
	ldr	r0, [r4, #3300]
	mov	r6, r6, asl #1
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r3, [r4, #2324]
	ldrh	r2, [r3, r6]
	add	r2, r2, #1
	strh	r2, [r3, r6]	@ movhi
	ldr	r3, [r4, #2560]
	add	r3, r3, #1
	str	r3, [r4, #2560]
.L431:
	ldr	r3, .L432
	ldrh	r2, [r3, #214]
	add	r2, r2, #1
	strh	r2, [r3, #214]	@ movhi
	ldrh	r2, [r3, #212]
	add	r1, r3, r2, asl #1
	add	r2, r2, #1
	bic	r2, r2, #64512
	strh	r5, [r1, #216]	@ movhi
	strh	r2, [r3, #212]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L433:
	.align	2
.L432:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueIn.part.7, .-FtlFreeSysBlkQueueIn.part.7
	.align	2
	.global	FtlFreeSysBlkQueueIn
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	sub	r2, r0, #1
	movw	r3, #65533
	uxth	r2, r2
	cmp	r2, r3
	bxhi	lr
	ldr	r3, .L436
	ldrh	r3, [r3, #214]
	cmp	r3, #1024
	bxeq	lr
	b	FtlFreeSysBlkQueueIn.part.7
.L437:
	.align	2
.L436:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.align	2
	.global	FtlLowFormatEraseBlock
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, #0
	ldr	r9, .L466
	.pad #20
	sub	sp, sp, #20
	mov	sl, r0
	mov	r7, r1
	mov	r4, r8
	mov	r6, r8
	str	r0, [r9, #3304]
	mov	fp, #20
	b	.L439
.L443:
	mul	r3, fp, r8
	ldr	r2, [r9, #3300]
	mov	r1, #0
	str	r1, [r2, r3]
	add	r3, r9, r8
	mov	r1, sl
	ldrb	r0, [r3, #56]	@ zero_extendqisi2
	bl	V2P_block
	cmp	r7, #0
	mov	r5, r0
	beq	.L440
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L441
.L440:
	mov	r0, r5
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addne	r6, r6, #1
	uxthne	r6, r6
	bne	.L441
	ldr	r3, [r9, #3300]
	mov	r5, r5, asl #10
	ldr	r2, [r9, #3308]
	mla	r3, fp, r4, r3
	str	r2, [r3, #8]
	ldrh	r2, [r9, #108]
	str	r5, [r3, #4]
	mul	r2, r2, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [r9, #3312]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L441:
	add	r8, r8, #1
	uxth	r8, r8
.L439:
	ldrh	r3, [r9, #28]
	ldr	r5, .L466
	cmp	r3, r8
	bhi	.L443
	cmp	r4, #0
	beq	.L445
	ldr	r0, [r5, #3300]
	mov	r1, #0
	mov	r2, r4
	mov	r8, #0
	bl	FlashEraseBlocks
	mov	r9, r8
.L447:
	ldr	r3, [r5, #3300]
	add	r2, r3, r8
	ldr	r3, [r3, r8]
	cmn	r3, #1
	bne	.L446
	ldr	r0, [r2, #4]
	add	r6, r6, #1
	ubfx	r0, r0, #10, #16
	uxth	r6, r6
	bl	FtlBbmMapBadBlock
.L446:
	add	r9, r9, #1
	add	r8, r8, #20
	uxth	r9, r9
	cmp	r9, r4
	bne	.L447
.L448:
	cmp	r7, #0
	ldr	r4, .L466
	mov	r8, #0
	streq	r7, [sp, #4]
	ldrne	r3, .L466
	moveq	r9, #1
	moveq	r1, #6
	streq	r1, [sp, #8]
	ldrneh	r9, [r3, #100]
	movne	r3, #1
	strne	r3, [sp, #4]
	movne	r2, r9, lsr #2
	strne	r2, [sp, #8]
.L456:
	mov	fp, #0
	mov	r5, fp
	b	.L449
.L452:
	mov	r1, #20
	ldr	r2, [r4, #3300]
	mul	r3, r1, fp
	mov	r1, #0
	str	r1, [r2, r3]
	add	r3, r4, fp
	mov	r1, sl
	ldrb	r0, [r3, #56]	@ zero_extendqisi2
	bl	V2P_block
	cmp	r7, #0
	str	r0, [sp, #12]
	beq	.L450
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L451
.L450:
	ldr	r0, [sp, #12]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L451
	ldr	r3, [r4, #3300]
	mov	r2, #20
	ldr	r1, [sp, #12]
	mla	r3, r2, r5, r3
	add	r2, r8, r1, asl #10
	str	r2, [r3, #4]
	ldr	r2, [r4, #3316]
	str	r2, [r3, #8]
	ldrh	r2, [r4, #108]
	mul	r2, r2, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [r4, #3308]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L451:
	add	fp, fp, #1
	uxth	fp, fp
.L449:
	ldrh	r3, [r4, #28]
	cmp	r3, fp
	bhi	.L452
	cmp	r5, #0
	beq	.L445
	ldr	r2, .L466
	mov	r1, r5
	mov	r3, #1
	mov	fp, #0
	ldr	r0, [r2, #3300]
	ldr	r2, [sp, #4]
	bl	FlashProgPages
	mov	ip, r7
	mov	r7, r5
	mov	r5, fp
.L455:
	ldr	r2, [r4, #3300]
	add	r1, r2, fp
	ldr	r2, [r2, fp]
	cmp	r2, #0
	beq	.L454
	ldr	r0, [r1, #4]
	add	r6, r6, #1
	str	ip, [sp, #0]
	ubfx	r0, r0, #10, #16
	uxth	r6, r6
	bl	FtlBbmMapBadBlock
	ldr	ip, [sp, #0]
.L454:
	add	r5, r5, #1
	add	fp, fp, #20
	uxth	r5, r5
	cmp	r5, r7
	bne	.L455
	ldr	r3, [sp, #8]
	mov	r5, r7
	mov	r7, ip
	add	r8, r8, r3
	uxth	r8, r8
	cmp	r8, r9
	bcc	.L456
	ldr	r9, .L466
	mov	r4, #0
	mov	r8, r4
.L458:
	cmp	r7, #0
	beq	.L457
	ldr	r3, [r9, #3300]
	add	r2, r3, r4
	ldr	r3, [r3, r4]
	cmp	r3, #0
	bne	.L457
	ldr	r0, [r2, #4]
	mov	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L457:
	add	r8, r8, #1
	add	r4, r4, #20
	uxth	r8, r8
	cmp	r8, r5
	bne	.L458
	cmp	sl, #63
	movhi	sl, #0
	movls	sl, #1
	cmp	r7, #0
	moveq	r7, sl
	orrne	r7, sl, #1
	cmp	r7, #0
	beq	.L445
	ldr	r3, .L466
	mov	r2, r8
	ldr	r1, [sp, #4]
	ldr	r0, [r3, #3300]
	bl	FlashEraseBlocks
.L445:
	mov	r0, r6
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L467:
	.align	2
.L466:
	.word	.LANCHOR0
	.fnend
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	2
	.global	ftl_memset
	.type	ftl_memset, %function
ftl_memset:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r2, #0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	beq	.L469
	bl	memset
.L469:
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
	.fnend
	.size	ftl_memset, .-ftl_memset
	.align	2
	.global	FlashGetBadBlockList
	.type	FlashGetBadBlockList, %function
FlashGetBadBlockList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r2, #256
	ldr	r5, .L475
	mov	r4, r0
	mov	r6, r1
	mov	r1, #255
	bl	ftl_memset
	mov	r0, r4
	ldr	r3, [r5, #3248]
	mov	r1, r6
	blx	r3
	uxth	r0, r0
	cmp	r0, #50
	bls	.L471
	mov	r0, r4
	mov	r1, #255
	mov	r2, #256
	bl	ftl_memset
	mov	r0, #0
.L471:
	ldrh	r3, [r5, #14]
	cmp	r3, #4
	moveq	r3, #0
	ldmnefd	sp!, {r4, r5, r6, pc}
	b	.L473
.L474:
	add	r3, r3, #1
	ldrh	r2, [r4, #0]
	uxth	r3, r3
	mov	r2, r2, lsr #1
	strh	r2, [r4], #2	@ movhi
.L473:
	cmp	r3, r0
	bne	.L474
	ldmfd	sp!, {r4, r5, r6, pc}
.L476:
	.align	2
.L475:
	.word	.LANCHOR0
	.fnend
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
	.align	2
	.global	FtlGcPageVarInit
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	movw	r2, #3188
	ldr	r4, .L478
	mov	r3, #0
	mov	r1, #255
	strh	r3, [r4, r2]	@ movhi
	mov	r2, #3200
	strh	r3, [r4, r2]	@ movhi
	ldrh	r2, [r4, #102]
	ldr	r0, [r4, #3192]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r4, #102]
	mov	r2, #12
	ldr	r0, [r4, #3196]
	mov	r1, #255
	mul	r2, r2, r3
	bl	ftl_memset
	ldmfd	sp!, {r4, lr}
	b	FtlGcBufInit
.L479:
	.align	2
.L478:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	2
	.global	SupperBlkListInit
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r2, #6
	ldr	r4, .L491
	mov	r1, #0
	mov	r7, #0
	mov	r8, r7
	mov	r5, r7
	ldrh	r3, [r4, #38]
	mov	fp, r4
	ldr	r0, [r4, #2312]
	mov	r9, r4
	mul	r2, r2, r3
	bl	ftl_memset
	movw	r3, #2332
	strh	r7, [r4, r3]	@ movhi
	add	r3, r3, #8
	str	r7, [r4, #2336]
	str	r7, [r4, #2316]
	str	r7, [r4, #2328]
	strh	r7, [r4, r3]	@ movhi
	b	.L481
.L483:
	add	r3, r4, sl
	mov	r1, r5
	add	sl, sl, #1
	ldrb	r0, [r3, #56]	@ zero_extendqisi2
	bl	V2P_block
	uxth	sl, sl
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldreqh	r3, [r4, #98]
	addeq	r6, r6, r3
	uxtheq	r6, r6
	b	.L488
.L490:
	mov	r6, #0
	mov	sl, r6
.L488:
	ldrh	r3, [r4, #28]
	cmp	r3, sl
	bhi	.L483
	cmp	r6, #0
	beq	.L484
	mov	r1, r6
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r6, r0
	b	.L485
.L484:
	ldr	r2, [fp, #2320]
	mov	r3, r5, asl #1
	mvn	r1, #0
	strh	r1, [r2, r3]	@ movhi
.L485:
	ldr	r3, [r9, #2312]
	mov	r2, #6
	mla	r3, r2, r5, r3
	strh	r6, [r3, #4]	@ movhi
	movw	r3, #2344
	ldrh	r3, [r9, r3]
	cmp	r3, r5
	beq	.L486
	movw	r3, #2392
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L486
	movw	r3, #2440
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L486
	ldr	r2, [r4, #2320]
	mov	r3, r5, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	bne	.L487
	add	r7, r7, #1
	mov	r0, r5
	uxth	r7, r7
	bl	INSERT_FREE_LIST
	b	.L486
.L487:
	add	r8, r8, #1
	mov	r0, r5
	uxth	r8, r8
	bl	INSERT_DATA_LIST
.L486:
	add	r5, r5, #1
	uxth	r5, r5
.L481:
	ldrh	r3, [r4, #36]
	ldr	r2, .L491
	cmp	r5, r3
	bcc	.L490
	movw	r1, #2332
	strh	r8, [r2, r1]	@ movhi
	add	r1, r1, #8
	strh	r7, [r2, r1]	@ movhi
	add	r7, r8, r7
	cmp	r7, r3
	ble	.L489
	ldr	r0, .L491+4
	movw	r2, #2170
	ldr	r1, .L491+8
	bl	printk
.L489:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L492:
	.align	2
.L491:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+203
	.fnend
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	2
	.global	FtlL2PDataInit
	.type	FtlL2PDataInit, %function
FtlL2PDataInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r1, #0
	ldr	r4, .L496
	mov	r5, #12
	ldr	r2, [r4, #124]
	ldr	r0, [r4, #3320]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r4, #106]
	ldrh	r2, [r4, #134]
	mov	r1, #255
	ldr	r0, [r4, #3324]
	mul	r2, r2, r3
	bl	ftl_memset
	mov	r2, #0
	mov	ip, r2
	mvn	r0, #0
	b	.L494
.L495:
	mul	r3, r5, r2
	ldr	r1, [r4, #2488]
	add	r6, r1, r3
	str	ip, [r6, #4]
	strh	r0, [r1, r3]	@ movhi
	ldr	r1, [r4, #2488]
	ldr	r6, [r4, #3324]
	add	r3, r1, r3
	ldrh	r1, [r4, #106]
	mul	r1, r2, r1
	add	r2, r2, #1
	uxth	r2, r2
	bic	r1, r1, #3
	add	r1, r6, r1
	str	r1, [r3, #8]
.L494:
	ldrh	r1, [r4, #134]
	ldr	r3, .L496
	cmp	r1, r2
	bhi	.L495
	movw	r1, #3330
	mvn	r2, #0
	strh	r2, [r3, r1]	@ movhi
	mov	r1, #3328
	strh	r2, [r3, r1]	@ movhi
	movw	r2, #3338
	ldr	r1, [r3, #124]
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #3332
	ldr	r1, .L496+4
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #36
	ldrh	r1, [r3, r2]
	movw	r2, #3336
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #3334
	ldrh	r1, [r3, #132]
	strh	r1, [r3, r2]	@ movhi
	ldr	r2, [r3, #3372]
	str	r2, [r3, #3340]
	ldr	r2, [r3, #3376]
	str	r2, [r3, #3344]
	ldr	r2, [r3, #3320]
	str	r2, [r3, #3348]
	ldr	r2, [r3, #3380]
	str	r2, [r3, #3352]
	ldmfd	sp!, {r4, r5, r6, pc}
.L497:
	.align	2
.L496:
	.word	.LANCHOR0
	.word	-3902
	.fnend
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	2
	.global	ftl_free_no_use_map_blk
	.type	ftl_free_no_use_map_blk, %function
ftl_free_no_use_map_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r1, #0
	ldrh	r2, [r0, #10]
	mov	r4, r0
	ldr	r5, [r0, #20]
	ldr	r6, [r0, #12]
	ldr	r7, [r0, #24]
	mov	r2, r2, asl #1
	mov	r0, r5
	bl	ftl_memset
	mov	r3, #0
	b	.L499
.L503:
	ldr	r0, [r7, r3, asl #2]
	mov	r2, #0
	ubfx	r0, r0, #10, #16
	b	.L500
.L502:
	mov	r1, r2, asl #1
	ldrh	ip, [r6, r1]
	cmp	ip, r0
	bne	.L501
	cmp	r0, #0
	ldrneh	ip, [r5, r1]
	addne	ip, ip, #1
	strneh	ip, [r5, r1]	@ movhi
.L501:
	add	r2, r2, #1
	uxth	r2, r2
.L500:
	ldrh	r1, [r4, #10]
	cmp	r1, r2
	bhi	.L502
	add	r3, r3, #1
	uxth	r3, r3
.L499:
	ldrh	r2, [r4, #6]
	cmp	r2, r3
	bhi	.L503
	ldr	r3, .L509
	mov	r8, #0
	mov	r7, r8
	ldrh	r2, [r3, #100]
	ldrh	r3, [r4, #0]
	mov	r3, r3, asl #1
	strh	r2, [r5, r3]	@ movhi
	ldrh	fp, [r5, #0]
	b	.L504
.L507:
	mov	sl, r7, asl #1
	ldrh	r9, [r5, sl]
	cmp	fp, r9
	bls	.L505
	ldrh	r3, [r6, sl]
	cmp	r3, #0
	movne	r8, r7
	movne	fp, r9
.L505:
	cmp	r9, #0
	bne	.L506
	ldrh	r0, [r6, sl]
	cmp	r0, #0
	beq	.L506
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	strh	r9, [r6, sl]	@ movhi
	ldrh	r3, [r4, #8]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L506:
	add	r7, r7, #1
	uxth	r7, r7
.L504:
	ldrh	r3, [r4, #10]
	cmp	r3, r7
	bhi	.L507
	mov	r0, r8
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L510:
	.align	2
.L509:
	.word	.LANCHOR0
	.fnend
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.align	2
	.global	FtlFreeSysBlkQueueInit
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L512
	mov	r2, #2048
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, #0
	strh	r0, [r3, #208]	@ movhi
	mov	r1, r4
	strh	r4, [r3, #210]	@ movhi
	add	r0, r3, #216
	strh	r4, [r3, #212]	@ movhi
	strh	r4, [r3, #214]	@ movhi
	bl	ftl_memset
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L513:
	.align	2
.L512:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueInit, .-FtlFreeSysBlkQueueInit
	.align	2
	.global	FtlBbtMemInit
	.type	FtlBbtMemInit, %function
FtlBbtMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r0, .L515
	mvn	r3, #0
	mov	r1, #255
	mov	r2, #16
	strh	r3, [r0, #148]	@ movhi
	mov	r3, #0
	strh	r3, [r0, #154]	@ movhi
	add	r0, r0, #160
	b	ftl_memset
.L516:
	.align	2
.L515:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	2
	.global	FtlLoadBbt
	.type	FtlLoadBbt, %function
FtlLoadBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movw	r7, #61649
	ldr	r5, .L536
	ldr	r3, [r5, #3288]
	ldr	r4, [r5, #3292]
	str	r3, [r5, #3276]
	str	r4, [r5, #3280]
	bl	FtlBbtMemInit
	ldrh	r6, [r5, #94]
	sub	r6, r6, #1
	uxth	r6, r6
	b	.L518
.L522:
	mov	r1, #1
	mov	r3, r6, asl #10
	ldr	r0, .L536+4
	mov	r2, r1
	str	r3, [r5, #3272]
	bl	FlashReadPages
	ldr	r3, [r5, #3268]
	cmn	r3, #1
	bne	.L519
	ldr	r3, [r5, #3272]
	mov	r1, #1
	ldr	r0, .L536+4
	mov	r2, r1
	add	r3, r3, #1
	str	r3, [r5, #3272]
	bl	FlashReadPages
.L519:
	ldr	r2, [r5, #3268]
	ldr	r3, .L536
	cmn	r2, #1
	beq	.L520
	ldrh	r2, [r4, #0]
	cmp	r2, r7
	bne	.L520
	ldr	r2, [r4, #4]
	strh	r6, [r3, #148]	@ movhi
	str	r2, [r3, #156]
	ldrh	r2, [r4, #8]
	strh	r2, [r3, #152]	@ movhi
	b	.L521
.L520:
	sub	r6, r6, #1
	uxth	r6, r6
.L518:
	ldrh	r3, [r5, #94]
	sub	r3, r3, #16
	cmp	r6, r3
	bgt	.L522
.L521:
	ldr	r5, .L536
	movw	r3, #65535
	ldrh	r2, [r5, #148]
	cmp	r2, r3
	beq	.L533
	ldrh	r2, [r5, #152]
	cmp	r2, r3
	beq	.L524
	mov	r1, #1
	mov	r2, r2, asl #10
	ldr	r0, .L536+4
	str	r2, [r5, #3272]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r3, [r5, #3268]
	cmn	r3, #1
	beq	.L524
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L524
	ldr	r3, [r4, #4]
	ldr	r2, [r5, #156]
	cmp	r3, r2
	strhi	r3, [r5, #156]
	ldrhih	r2, [r5, #152]
	ldrhih	r3, [r4, #8]
	strhih	r2, [r5, #148]	@ movhi
	strhih	r3, [r5, #152]	@ movhi
.L524:
	ldr	r5, .L536
	mov	r1, #1
	mov	r6, #0
	movw	r8, #61649
	ldrh	r0, [r5, #148]
	bl	FtlGetLastWrittenPage
	add	r3, r0, #1
	mov	r7, r0
	strh	r3, [r5, #150]	@ movhi
	b	.L525
.L528:
	ldrh	r2, [r5, #148]
	mov	r1, #1
	ldr	r0, .L536+4
	orr	r3, r3, r2, asl #10
	str	r3, [r5, #3272]
	ldr	r3, [r5, #3288]
	mov	r2, r1
	str	r3, [r5, #3276]
	bl	FlashReadPages
	ldr	r3, [r5, #3268]
	cmn	r3, #1
	beq	.L526
	ldrh	r3, [r4, #0]
	cmp	r3, r8
	beq	.L527
.L526:
	sub	r6, r6, #1
	uxth	r6, r6
.L525:
	add	r3, r6, r7
	sxth	r3, r3
	cmp	r3, #0
	bge	.L528
	b	.L535
.L527:
	ldrh	r2, [r4, #10]
	ldr	r3, .L536
	ldrh	r0, [r4, #12]
	strh	r2, [r3, #154]	@ movhi
	movw	r2, #65535
	cmp	r0, r2
	beq	.L530
	ldr	r2, [r3, #24]
	cmp	r0, r2
	beq	.L530
	ldrh	r3, [r3, #38]
	mov	r3, r3, lsr #2
	cmp	r2, r3
	bcs	.L530
	cmp	r0, r3
	bcs	.L530
	bl	FtlSysBlkNumInit
.L530:
	ldr	r6, .L536+8
	mov	r4, #0
	movw	r7, #3384
	sub	r5, r6, #172
	b	.L531
.L532:
	ldrh	r2, [r5, r7]
	ldr	r1, [r5, #3276]
	ldr	r0, [r6, #4]!
	mov	r2, r2, asl #2
	mla	r1, r4, r2, r1
	bl	memcpy
	add	r4, r4, #1
.L531:
	ldrh	r3, [r5, #50]
	cmp	r4, r3
	bcc	.L532
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L533:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L535:
	ldr	r1, .L536+12
	mov	r2, #253
	ldr	r0, .L536+16
	bl	printk
	b	.L527
.L537:
	.align	2
.L536:
	.word	.LANCHOR0
	.word	.LANCHOR0+3268
	.word	.LANCHOR0+172
	.word	.LANCHOR1+221
	.word	.LC1
	.fnend
	.size	FtlLoadBbt, .-FtlLoadBbt
	.align	2
	.global	FtlBbt2Bitmap
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	movw	r3, #3384
	ldr	r6, .L543
	mov	r7, r0
	mov	r5, r1
	mov	r0, r1
	mov	r1, #0
	mov	r4, #0
	ldrh	r2, [r6, r3]
	movw	sl, #65535
	mov	r8, #1
	mov	r2, r2, asl #2
	bl	ftl_memset
.L541:
	ldrh	r3, [r7, r4]
	cmp	r3, sl
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
	ldrh	r2, [r6, #94]
	cmp	r2, r3
	bhi	.L540
	ldr	r0, .L543+4
	mov	r2, #74
	ldr	r1, .L543+8
	bl	printk
.L540:
	ldrh	r2, [r7, r4]
	add	r4, r4, #2
	cmp	r4, #1024
	mov	r3, r2, lsr #5
	and	r2, r2, #31
	ldr	r1, [r5, r3, asl #2]
	orr	r2, r1, r8, asl r2
	str	r2, [r5, r3, asl #2]
	bne	.L541
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L544:
	.align	2
.L543:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+232
	.fnend
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
	.align	2
	.global	FtlVariablesInit
	.type	FtlVariablesInit, %function
FtlVariablesInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movw	r2, #3386
	ldr	r4, .L546
	mvn	r3, #0
	mov	r5, #0
	mov	r1, r5
	strh	r3, [r4, r2]	@ movhi
	ldrh	r2, [r4, #116]
	ldr	r0, [r4, #144]
	str	r3, [r4, #3396]
	mov	r2, r2, asl #1
	str	r5, [r4, #3388]
	str	r5, [r4, #3392]
	strh	r5, [r4, #140]	@ movhi
	bl	ftl_memset
	ldrh	r2, [r4, #38]
	mov	r1, r5
	ldr	r0, [r4, #2324]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r4, #38]
	mov	r1, r5
	ldr	r0, [r4, #3400]
	mov	r2, r2, asl #1
	bl	ftl_memset
	mov	r1, r5
	mov	r2, #48
	ldr	r0, .L546+4
	bl	ftl_memset
	mov	r1, r5
	mov	r2, #512
	ldr	r0, .L546+8
	bl	ftl_memset
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, pc}
.L547:
	.align	2
.L546:
	.word	.LANCHOR0
	.word	.LANCHOR0+2264
	.word	.LANCHOR0+2636
	.fnend
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	2
	.global	FtlMemInit
	.type	FtlMemInit, %function
FtlMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movw	r2, #3202
	ldr	r4, .L578
	movw	r3, #65535
	mov	r6, #0
	mov	r5, #12
	mov	r7, #20
	str	r3, [r4, #3408]
	mvn	r3, #0
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #2
	ldrh	r0, [r4, #102]
	strh	r3, [r4, r2]	@ movhi
	movw	r3, #3148
	mov	r2, #32
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	mov	r2, #128
	mov	r0, r0, asl #1
	str	r6, [r4, #2540]
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #6
	str	r6, [r4, #2544]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #50
	str	r6, [r4, #2528]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #36
	str	r6, [r4, #2516]
	strh	r6, [r4, r3]	@ movhi
	str	r6, [r4, #2512]
	str	r6, [r4, #2520]
	str	r6, [r4, #2524]
	str	r6, [r4, #2508]
	str	r6, [r4, #2548]
	str	r6, [r4, #2552]
	str	r6, [r4, #2560]
	str	r6, [r4, #2564]
	str	r6, [r4, #2568]
	str	r6, [r4, #3404]
	str	r6, [r4, #3160]
	str	r6, [r4, #3412]
	str	r6, [r4, #3152]
	str	r6, [r4, #3304]
	bl	ftl_malloc
	str	r0, [r4, #3192]
	ldrh	r0, [r4, #102]
	mul	r0, r5, r0
	bl	ftl_malloc
	ldrh	r3, [r4, #28]
	mul	r7, r7, r3
	mov	r8, r7, asl #2
	str	r0, [r4, #3196]
	mov	r0, r8
	bl	ftl_malloc
	str	r0, [r4, #3416]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3420]
	mov	r0, r8
	bl	ftl_malloc
	str	r0, [r4, #3424]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3300]
	mov	r0, r7
	bl	ftl_malloc
	ldrh	r7, [r4, #106]
	ldrh	r3, [r4, #28]
	mov	r3, r3, asl #1
	add	r3, r3, #1
	str	r3, [r4, #3184]
	str	r0, [r4, #3180]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3288]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3428]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3432]
	ldr	r0, [r4, #3184]
	mul	r0, r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3172]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3316]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3308]
	ldr	r0, [r4, #3184]
	mul	r0, r5, r0
	bl	ftl_malloc
	str	r0, [r4, #3168]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3244]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3264]
	ldrh	r0, [r4, #54]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r3, [r4, #108]
	ldrh	r7, [r4, #28]
	mul	r7, r7, r3
	str	r0, [r4, #3296]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3292]
	mov	r0, r7, asl #2
	bl	ftl_malloc
	ldrh	r3, [r4, #108]
	movw	r7, #3436
	str	r0, [r4, #3312]
	ldr	r0, [r4, #3184]
	mul	r0, r0, r3
	bl	ftl_malloc
	str	r0, [r4, #3176]
	ldrh	r0, [r4, #38]
	mov	r0, r0, asl #1
	uxth	r0, r0
	strh	r0, [r4, r7]	@ movhi
	bl	ftl_malloc
	str	r0, [r4, #3400]
	ldrh	r0, [r4, r7]
	add	r0, r0, #544
	add	r0, r0, #3
	mov	r0, r0, lsr #9
	strh	r0, [r4, r7]	@ movhi
	mov	r0, r0, asl #9
	bl	ftl_malloc
	ldrh	r7, [r4, #38]
	mov	r7, r7, asl #1
	str	r0, [r4, #3440]
	add	r0, r0, #32
	str	r0, [r4, #2324]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3444]
	mov	r0, r7
	bl	ftl_malloc
	ldr	r7, [r4, #124]
	mov	r7, r7, asl #1
	str	r0, [r4, #2320]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3372]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #3320]
	ldrh	r0, [r4, #38]
	mov	r0, r0, lsr #3
	add	r0, r0, #4
	bl	ftl_malloc
	str	r0, [r4, #2500]
	ldrh	r0, [r4, #116]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #144]
	ldrh	r0, [r4, #116]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #3448]
	ldrh	r0, [r4, #116]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #3452]
	ldrh	r0, [r4, #118]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r2, [r4, #118]
	mov	r1, r6
	mov	r2, r2, asl #2
	str	r0, [r4, #3456]
	bl	ftl_memset
	ldrh	r0, [r4, #132]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #3380]
	ldr	r0, [r4, #124]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #3376]
	ldrh	r0, [r4, #134]
	mul	r0, r5, r0
	bl	ftl_malloc
	ldrh	r3, [r4, #134]
	movw	r5, #3384
	str	r0, [r4, #2488]
	ldrh	r0, [r4, #106]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r4, #38]
	str	r0, [r4, #3324]
	mov	r0, #6
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r4, #94]
	ldrh	r2, [r4, #50]
	add	r3, r3, #31
	mov	r3, r3, lsr #5
	strh	r3, [r4, r5]	@ movhi
	mul	r3, r2, r3
	str	r0, [r4, #2312]
	mov	r0, r3, asl #2
	bl	ftl_malloc
	ldrh	r1, [r4, r5]
	ldrh	ip, [r4, #50]
	mov	r3, #1
	mov	r1, r1, asl #2
	mov	r2, r1
	str	r0, [r4, #176]
	add	r0, r4, #176
	b	.L549
.L550:
	ldr	r5, [r4, #176]
	add	r3, r3, #1
	add	r5, r5, r2
	add	r2, r2, r1
	str	r5, [r0, #4]!
.L549:
	cmp	r3, ip
	bcc	.L550
	ldr	r0, .L578+4
	mov	r2, #0
	mov	r1, r2
	add	r0, r0, r3, asl #2
	b	.L551
.L552:
	add	ip, r0, r2
	add	r3, r3, #1
	add	r2, r2, #4
	str	r1, [ip, #28]
.L551:
	cmp	r3, #7
	bls	.L552
	ldr	r3, .L578
	ldr	r2, [r3, #3372]
	cmp	r2, #0
	beq	.L577
.L553:
	ldr	r2, [r3, #3320]
	cmp	r2, #0
	beq	.L577
.L555:
	ldr	r2, [r3, #3380]
	cmp	r2, #0
	beq	.L577
.L556:
	ldr	r2, [r3, #3376]
	cmp	r2, #0
	beq	.L577
.L557:
	ldr	r2, [r3, #2488]
	cmp	r2, #0
	beq	.L577
.L558:
	ldr	r2, [r3, #3324]
	cmp	r2, #0
	beq	.L577
.L559:
	ldr	r2, [r3, #2312]
	cmp	r2, #0
	beq	.L577
.L560:
	ldr	r2, [r3, #176]
	cmp	r2, #0
	beq	.L577
.L561:
	ldr	r3, [r3, #2320]
	cmp	r3, #0
	beq	.L577
.L562:
	ldr	r3, .L578
	ldr	r2, [r3, #3192]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3196]
	cmp	r2, #0
	beq	.L577
.L564:
	ldr	r2, [r3, #3416]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3424]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3300]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3180]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3420]
	cmp	r2, #0
	beq	.L577
.L566:
	ldr	r2, [r3, #3288]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3428]
	cmp	r2, #0
	beq	.L577
	ldr	r3, [r3, #3432]
	cmp	r3, #0
	beq	.L577
	ldr	r3, .L578
	ldr	r2, [r3, #3172]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3316]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3308]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3168]
	cmp	r2, #0
	beq	.L577
.L568:
	ldr	r2, [r3, #3292]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3312]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3176]
	cmp	r2, #0
	beq	.L577
.L570:
	ldr	r2, [r3, #2324]
	cmp	r2, #0
	beq	.L577
	ldr	r2, [r3, #3400]
	cmp	r2, #0
	beq	.L577
.L572:
	ldr	r3, [r3, #144]
	cmp	r3, #0
	beq	.L577
.L573:
	ldr	r3, .L578
	ldr	r2, [r3, #3448]
	cmp	r2, #0
	beq	.L577
.L574:
	ldr	r2, [r3, #3452]
	cmp	r2, #0
	beq	.L577
.L575:
	ldr	r3, [r3, #3456]
	cmp	r3, #0
	bne	.L576
.L577:
	ldr	r0, .L578+8
	ldr	r1, .L578+12
	bl	printk
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L576:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L579:
	.align	2
.L578:
	.word	.LANCHOR0
	.word	.LANCHOR0+148
	.word	.LC83
	.word	.LANCHOR1+246
	.fnend
	.size	FtlMemInit, .-FtlMemInit
	.align	2
	.global	FlashTestBlk
	.type	FlashTestBlk, %function
FlashTestBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #11
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	mov	r4, r0
	.pad #92
	sub	sp, sp, #92
	movls	r4, #0
	bls	.L581
.L582:
	ldr	r5, .L585
	add	r0, sp, #24
	mov	r1, #165
	mov	r2, #32
	str	r0, [sp, #16]
	mov	r4, r4, asl #10
	ldr	r3, [r5, #3264]
	str	r3, [sp, #12]
	bl	ftl_memset
	mov	r1, #90
	mov	r2, #8
	ldr	r0, [r5, #3264]
	bl	ftl_memset
	mov	r1, #1
	mov	r2, r1
	add	r0, sp, #4
	str	r4, [sp, #8]
	bl	FlashEraseBlocks
	mov	r1, #1
	add	r0, sp, #4
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldr	r4, [sp, #4]
	add	r0, sp, #4
	mov	r1, #0
	cmp	r4, #0
	mov	r2, #1
	mvnne	r4, #0
	bl	FlashEraseBlocks
.L581:
	mov	r0, r4
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, pc}
.L586:
	.align	2
.L585:
	.word	.LANCHOR0
	.fnend
	.size	FlashTestBlk, .-FlashTestBlk
	.align	2
	.global	FtlScanSysBlk
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r3, #3368
	ldr	r4, .L647
	mov	r5, #0
	.pad #20
	sub	sp, sp, #20
	mov	r1, r5
	ldr	r2, [r4, #124]
	mov	sl, r4
	strh	r5, [r4, r3]	@ movhi
	strh	r5, [r4, #140]	@ movhi
	mov	r2, r2, asl #2
	ldr	r0, [r4, #3376]
	bl	ftl_memset
	ldr	r2, [r4, #124]
	mov	r1, r5
	ldr	r0, [r4, #3372]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r4, #116]
	mov	r1, r5
	ldr	r0, [r4, #3452]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r4, #116]
	mov	r1, r5
	ldr	r0, [r4, #144]
	mov	r2, r2, asl #1
	bl	ftl_memset
	add	r0, r4, #2576
	mov	r1, #255
	mov	r2, #12
	bl	ftl_memset
	ldrh	r3, [r4, #36]
	str	r3, [sp, #8]
	b	.L588
.L590:
	add	r3, r4, r5
	ldr	r1, [sp, #8]
	ldrb	r0, [r3, #56]	@ zero_extendqisi2
	bl	V2P_block
	mov	r7, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L589
	ldr	r3, [r4, #3416]
	mov	r7, r7, asl #10
	ldr	r2, [r4, #3172]
	mla	r3, r6, r8, r3
	str	r2, [r3, #8]
	ldrh	r2, [r4, #108]
	str	r7, [r3, #4]
	mul	r2, r2, r8
	add	r8, r8, #1
	uxth	r8, r8
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [r4, #3176]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L589:
	add	r5, r5, #1
	uxth	r5, r5
	b	.L624
.L638:
	mov	r8, #0
	mov	r6, #20
	mov	r5, r8
.L624:
	ldrh	r3, [r4, #28]
	cmp	r3, r5
	bhi	.L590
	cmp	r8, #0
	beq	.L591
	ldr	r0, [sl, #3416]
	mov	r1, r8
	mov	r2, #1
	mov	fp, r8
	bl	FlashReadPages
	mov	r6, #0
	str	r6, [sp, #12]
.L623:
	ldr	r3, [r4, #3416]
	add	r2, r3, r6
	ldr	r3, [r3, r6]
	ldr	r7, [r2, #4]
	cmn	r3, #1
	ldr	r5, [r2, #12]
	ubfx	r7, r7, #10, #16
	bne	.L592
	mov	r8, #16
.L594:
	ldr	r0, [r4, #3416]
	mov	r1, #1
	mov	r2, r1
	add	r0, r0, r6
	ldr	r3, [r0, #4]
	add	r3, r3, #1
	str	r3, [r0, #4]
	bl	FlashReadPages
	ldrh	r3, [r5, #0]
	movw	ip, #65535
	cmp	r3, ip
	ldreq	r3, [sl, #3416]
	mvneq	r2, #0
	streq	r2, [r3, r6]
	beq	.L592
.L593:
	ldr	r3, [r4, #3416]
	ldr	r3, [r3, r6]
	cmn	r3, #1
	bne	.L592
	sub	r8, r8, #1
	uxth	r8, r8
	cmp	r8, #0
	bne	.L594
.L592:
	ldr	r3, [r4, #3416]
	ldr	r3, [r3, r6]
	cmn	r3, #1
	beq	.L595
	ldr	r2, [r4, #2540]
	ldr	r3, [r5, #4]
	cmn	r2, #1
	beq	.L596
	cmp	r2, r3
	bhi	.L597
.L596:
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [sl, #2540]
.L597:
	ldrh	r3, [r5, #0]
	movw	r2, #61604
	cmp	r3, r2
	beq	.L600
	bhi	.L603
	movw	r2, #61574
	cmp	r3, r2
	bne	.L598
	b	.L645
.L603:
	movw	r2, #61634
	cmp	r3, r2
	beq	.L601
	movw	r1, #65535
	cmp	r3, r1
	bne	.L598
	b	.L646
.L601:
	movw	r3, #3368
	ldrh	r2, [r4, r3]
	ldr	r3, [r4, #124]
	cmp	r2, r3
	bls	.L604
	ldr	r0, .L647+4
	movw	r2, #1222
	ldr	r1, .L647+8
	bl	printk
.L604:
	ldr	r0, [r4, #124]
	movw	r1, #3368
	ldrh	r1, [r4, r1]
	mov	r9, r7
	ldr	ip, [r4, #3376]
	uxth	r2, r0
	sub	r3, r2, #1
	rsb	r2, r1, r2
	sub	r2, r2, #1
	uxth	r3, r3
	sxth	r2, r2
	str	r2, [sp, #4]
	b	.L605
.L611:
	mov	r7, r2, asl #2
	ldr	r8, [r5, #4]
	str	r7, [sp, #0]
	ldr	r7, [ip, r2, asl #2]
	cmp	r8, r7
	bls	.L606
	ldr	ip, [ip, #0]
	mov	r7, r9
	cmp	ip, #0
	bne	.L607
	cmp	r1, r0
	addne	r1, r1, #1
	movwne	r0, #3368
	strneh	r1, [r4, r0]	@ movhi
.L607:
	mov	r0, #0
	sxth	r9, r3
	str	r3, [sp, #4]
	b	.L608
.L609:
	ldr	r8, [r4, #3376]
	add	ip, r1, #1
	add	r0, r0, #1
	ldr	r3, [r8, ip, asl #2]
	mov	ip, ip, asl #1
	uxth	r0, r0
	str	r3, [r8, r1, asl #2]
	mov	r1, r1, asl #1
	ldr	r8, [r4, #3372]
	ldrh	ip, [r8, ip]
	strh	ip, [r8, r1]	@ movhi
.L608:
	sxth	r1, r0
	cmp	r1, r9
	bne	.L609
	ldr	r1, [sl, #3376]
	mov	r2, r2, asl #1
	ldr	r0, [r5, #4]
	ldr	ip, [sp, #0]
	ldr	r3, [sp, #4]
	str	r0, [r1, ip]
	ldr	r1, [sl, #3372]
	strh	r7, [r1, r2]	@ movhi
	b	.L610
.L606:
	sub	r3, r3, #1
	uxth	r3, r3
.L605:
	ldr	r7, [sp, #4]
	sxth	r2, r3
	cmp	r2, r7
	bgt	.L611
	mov	r7, r9
.L610:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L598
	movw	r2, #3368
	ldr	r0, [r4, #124]
	ldrh	r1, [r4, r2]
	rsb	r0, r1, r0
	sub	r0, r0, #1
	sxth	r0, r0
	cmp	r3, r0
	bgt	.L598
	add	r1, r1, #1
	strh	r1, [r4, r2]	@ movhi
	ldr	r2, [r4, #3376]
	ldr	r1, [r5, #4]
	str	r1, [r2, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r4, #3372]
	b	.L642
.L645:
	ldrh	r2, [r4, #140]
	ldrh	r3, [r4, #116]
	cmp	r2, r3
	bls	.L612
	ldr	r0, .L647+4
	movw	r2, #1263
	ldr	r1, .L647+8
	bl	printk
.L612:
	ldrh	r0, [r4, #116]
	mov	r9, r7
	ldrh	r1, [r4, #140]
	sub	r2, r0, #1
	ldr	ip, [r4, #3452]
	uxth	r3, r2
	rsb	r2, r1, r2
	str	r2, [sp, #0]
	b	.L613
.L619:
	mov	r7, r2, asl #2
	ldr	r8, [r5, #4]
	str	r7, [sp, #4]
	ldr	r7, [ip, r2, asl #2]
	cmp	r8, r7
	bls	.L614
	ldr	ip, [ip, #0]
	mov	r7, r9
	cmp	ip, #0
	bne	.L615
	cmp	r1, r0
	addne	r1, r1, #1
	strneh	r1, [r4, #140]	@ movhi
.L615:
	mov	r0, #0
	sxth	r9, r3
	str	r3, [sp, #0]
	b	.L616
.L617:
	ldr	r8, [r4, #3452]
	add	ip, r1, #1
	add	r0, r0, #1
	ldr	r3, [r8, ip, asl #2]
	mov	ip, ip, asl #1
	uxth	r0, r0
	str	r3, [r8, r1, asl #2]
	mov	r1, r1, asl #1
	ldr	r8, [r4, #144]
	ldrh	ip, [r8, ip]
	strh	ip, [r8, r1]	@ movhi
.L616:
	sxth	r1, r0
	cmp	r1, r9
	bne	.L617
	ldr	r1, [sl, #3452]
	mov	r2, r2, asl #1
	ldr	r0, [r5, #4]
	ldr	ip, [sp, #4]
	ldr	r3, [sp, #0]
	str	r0, [r1, ip]
	ldr	r1, [sl, #144]
	strh	r7, [r1, r2]	@ movhi
	b	.L618
.L614:
	sub	r3, r3, #1
	uxth	r3, r3
.L613:
	ldr	r7, [sp, #0]
	sxth	r2, r3
	cmp	r2, r7
	bgt	.L619
	mov	r7, r9
.L618:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L598
	ldrh	r1, [r4, #116]
	ldrh	r2, [r4, #140]
	sub	r1, r1, #1
	rsb	r1, r2, r1
	sxth	r1, r1
	cmp	r3, r1
	bgt	.L598
	add	r2, r2, #1
	ldr	r1, [r5, #4]
	strh	r2, [r4, #140]	@ movhi
	ldr	r2, [r4, #3452]
	str	r1, [r2, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r4, #144]
.L642:
	strh	r7, [r2, r3]	@ movhi
	b	.L598
.L600:
	mov	r3, #2576
	movw	ip, #65535
	ldrh	r2, [r4, r3]
	cmp	r2, ip
	streqh	r7, [r4, r3]	@ movhi
	beq	.L644
	movw	r3, #2580
	movw	r1, #65535
	ldrh	r0, [r4, r3]
	cmp	r0, r1
	beq	.L621
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L621:
	ldr	r3, [r5, #4]
	ldr	r2, [r4, #2584]
	cmp	r2, r3
	movw	r3, #2580
	strcsh	r7, [r4, r3]	@ movhi
	bcs	.L598
	mov	r2, #2576
	ldrh	r1, [r4, r2]
	strh	r7, [r4, r2]	@ movhi
	strh	r1, [r4, r3]	@ movhi
.L644:
	ldr	r3, [r5, #4]
	str	r3, [r4, #2584]
	b	.L598
.L646:
	mov	r0, r7
	mov	r1, #0
	b	.L643
.L595:
	mov	r0, r7
	mov	r1, #1
.L643:
	bl	FtlFreeSysBlkQueueIn
.L598:
	ldr	r7, [sp, #12]
	add	r6, r6, #20
	add	r3, r7, #1
	uxth	r3, r3
	str	r3, [sp, #12]
	cmp	r3, fp
	bne	.L623
.L591:
	ldr	ip, [sp, #8]
	add	r3, ip, #1
	uxth	r3, r3
	str	r3, [sp, #8]
.L588:
	ldrh	r3, [r4, #38]
	ldr	r7, [sp, #8]
	ldr	r2, .L647
	cmp	r3, r7
	bhi	.L638
	ldr	r1, [r2, #3372]
	ldrh	r3, [r1, #0]
	cmp	r3, #0
	bne	.L625
	movw	r0, #3368
	ldrh	r0, [r2, r0]
	cmp	r0, #0
	ldrne	r0, [r2, #124]
	bne	.L626
	b	.L625
.L630:
	mov	r2, r2, asl #1
	ldrh	r2, [r1, r2]
	cmp	r2, #0
	beq	.L627
	ldr	r2, .L647
	sxth	r6, r3
	mov	r5, #0
	b	.L628
.L629:
	ldr	ip, [r2, #3372]
	mov	r0, r1, asl #1
	rsb	r4, r6, r1
	add	r3, r3, #1
	ldrh	r8, [ip, r0]
	mov	r7, r4, asl #1
	uxth	r3, r3
	strh	r8, [ip, r7]	@ movhi
	ldr	ip, [r2, #3376]
	ldr	r1, [ip, r1, asl #2]
	str	r1, [ip, r4, asl #2]
	ldr	r1, [r2, #3372]
	strh	r5, [r1, r0]	@ movhi
.L628:
	ldr	r0, [r2, #124]
	sxth	r1, r3
	cmp	r1, r0
	bcc	.L629
	b	.L625
.L627:
	add	r3, r3, #1
	uxth	r3, r3
.L626:
	sxth	r2, r3
	cmp	r2, r0
	bcc	.L630
.L625:
	ldr	r2, .L647
	ldr	r1, [r2, #144]
	ldrh	r3, [r1, #0]
	cmp	r3, #0
	bne	.L631
	ldrh	r0, [r2, #140]
	cmp	r0, #0
	ldrneh	r2, [r2, #116]
	bne	.L632
	b	.L631
.L636:
	mov	ip, r0, asl #1
	ldrh	ip, [r1, ip]
	cmp	ip, #0
	beq	.L633
	mov	r1, r3
	mov	r6, #0
	ldr	r3, .L647
	b	.L634
.L635:
	ldr	r4, [r3, #144]
	mov	ip, r2, asl #1
	rsb	r5, r0, r2
	add	r1, r1, #1
	ldrh	r8, [r4, ip]
	mov	r7, r5, asl #1
	uxth	r1, r1
	strh	r8, [r4, r7]	@ movhi
	ldr	r4, [r3, #3452]
	ldr	r2, [r4, r2, asl #2]
	str	r2, [r4, r5, asl #2]
	ldr	r2, [r3, #144]
	strh	r6, [r2, ip]	@ movhi
.L634:
	ldrh	ip, [r3, #116]
	sxth	r2, r1
	cmp	r2, ip
	blt	.L635
	b	.L631
.L633:
	add	r3, r3, #1
	uxth	r3, r3
.L632:
	sxth	r0, r3
	cmp	r0, r2
	blt	.L636
.L631:
	ldr	r3, .L647
	movw	r2, #3368
	ldrh	r2, [r3, r2]
	ldr	r3, [r3, #124]
	cmp	r2, r3
	bls	.L637
	ldr	r0, .L647+4
	movw	r2, #1388
	ldr	r1, .L647+8
	bl	printk
.L637:
	mov	r0, #0
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L648:
	.align	2
.L647:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+257
	.fnend
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	2
	.global	Ftl_write_map_blk_to_last_page
	.type	Ftl_write_map_blk_to_last_page, %function
Ftl_write_map_blk_to_last_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	r2, #65535
	ldrh	r3, [r0, #0]
	mov	r4, r0
	ldr	r5, [r0, #12]
	cmp	r3, r2
	ldr	r7, [r0, #24]
	bne	.L650
	ldrh	r3, [r0, #8]
	cmp	r3, #0
	beq	.L651
	ldr	r0, .L656
	movw	r2, #641
	ldr	r1, .L656+4
	bl	printk
.L651:
	ldrh	r3, [r4, #8]
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	mov	r3, #0
	strh	r0, [r5, #0]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	strh	r3, [r4, #0]	@ movhi
	ldr	r3, [r4, #28]
	add	r3, r3, #1
	str	r3, [r4, #28]
	b	.L652
.L650:
	mov	r3, r3, asl #1
	ldr	r2, [r0, #28]
	mov	r1, #255
	ldrh	r6, [r5, r3]
	ldr	r5, .L656+8
	ldrh	r3, [r0, #2]
	orr	r3, r3, r6, asl #10
	str	r3, [r5, #3272]
	ldr	r3, [r5, #3288]
	str	r3, [r5, #3276]
	ldr	r3, [r5, #3292]
	str	r3, [r5, #3280]
	str	r2, [r3, #4]
	ldr	r2, .L656+12
	strh	r2, [r3, #8]	@ movhi
	ldrh	r2, [r0, #4]
	strh	r6, [r3, #2]	@ movhi
	strh	r2, [r3, #0]	@ movhi
	ldrh	r2, [r5, #100]
	ldr	r0, [r5, #3288]
	mov	r2, r2, asl #3
	bl	ftl_memset
	mov	r2, #0
	mov	r3, r2
	b	.L653
.L655:
	ldr	r1, [r7, r3, asl #2]
	cmp	r6, r1, lsr #10
	bne	.L654
	add	r2, r2, #1
	ldr	r1, [r5, #3288]
	uxth	r2, r2
	str	r3, [r1, r2, asl #3]
	ldr	r1, [r5, #3288]
	ldr	r0, [r7, r3, asl #2]
	add	r1, r1, r2, asl #3
	str	r0, [r1, #4]
.L654:
	add	r3, r3, #1
	uxth	r3, r3
.L653:
	ldrh	r1, [r4, #6]
	cmp	r1, r3
	bhi	.L655
	mov	r1, #1
	mov	r3, #0
	ldr	r0, .L656+16
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	add	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
.L652:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L657:
	.align	2
.L656:
	.word	.LC1
	.word	.LANCHOR1+271
	.word	.LANCHOR0
	.word	-1291
	.word	.LANCHOR0+3268
	.fnend
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.align	2
	.global	FtlMapWritePage
	.type	FtlMapWritePage, %function
FtlMapWritePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r4, r0
	ldr	r5, .L676
	mov	r7, r1
	mov	r8, r2
	mov	r6, #0
	movw	r9, #65535
.L674:
	ldr	r3, [r5, #2524]
	add	r3, r3, #1
	str	r3, [r5, #2524]
	ldrh	r3, [r5, #100]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	cmp	r2, r3
	bge	.L660
	ldrh	r3, [r4, #0]
	cmp	r3, r9
	bne	.L661
.L660:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L661:
	ldrh	r3, [r4, #0]
	ldr	r2, [r4, #12]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	bne	.L662
	ldr	r0, .L676+4
	movw	r2, #699
	ldr	r1, .L676+8
	bl	printk
.L662:
	ldrh	r2, [r4, #0]
	ldrh	r3, [r4, #10]
	cmp	r2, r3
	bcc	.L663
	ldr	r0, .L676+4
	mov	r2, #700
	ldr	r1, .L676+8
	bl	printk
.L663:
	ldrh	r3, [r4, #0]
	mov	r1, #0
	ldr	r2, [r4, #12]
	ldr	r0, [r5, #3292]
	mov	r3, r3, asl #1
	ldrh	sl, [r2, r3]
	mov	r2, #16
	ldrh	r3, [r4, #2]
	str	r8, [r5, #3276]
	orr	r3, r3, sl, asl #10
	str	r0, [r5, #3280]
	str	r3, [r5, #3272]
	bl	ftl_memset
	ldr	r3, [r5, #3280]
	mov	r1, #1
	ldr	r2, [r4, #28]
	ldr	r0, .L676+12
	strh	r7, [r3, #8]	@ movhi
	str	r2, [r3, #4]
	ldrh	r2, [r4, #4]
	strh	sl, [r3, #2]	@ movhi
	strh	r2, [r3, #0]	@ movhi
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r2, [r4, #2]
	add	r2, r2, #1
	uxth	r2, r2
	strh	r2, [r4, #2]	@ movhi
	ldr	r3, [r5, #3268]
	cmn	r3, #1
	bne	.L664
	ldr	r0, .L676+16
	add	r6, r6, #1
	ldr	r1, [r5, #3272]
	bl	printk
	ldrh	r3, [r4, #2]
	uxth	r6, r6
	cmp	r3, #2
	ldrlsh	r3, [r5, #100]
	subls	r3, r3, #1
	strlsh	r3, [r4, #2]	@ movhi
	cmp	r6, #3
	bls	.L666
	ldr	r3, .L676
	mov	r2, r6
	ldr	r0, .L676+20
	ldr	r1, [r3, #3272]
	bl	printk
.L667:
	b	.L667
.L666:
	ldr	r3, [r4, #32]
	cmp	r3, #0
	beq	.L674
.L675:
	b	.L675
.L664:
	cmp	r2, #1
	beq	.L670
	cmp	r3, #256
	beq	.L670
	ldr	r0, [r4, #36]
	cmp	r0, #0
	beq	.L671
.L670:
	mov	r3, #0
	str	r3, [r4, #36]
	b	.L674
.L671:
	ldr	r3, .L676
	ldr	r2, [r3, #3272]
	ldr	r3, [r4, #24]
	str	r2, [r3, r7, asl #2]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L677:
	.align	2
.L676:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+302
	.word	.LANCHOR0+3268
	.word	.LC84
	.word	.LC85
	.fnend
	.size	FtlMapWritePage, .-FtlMapWritePage
	.align	2
	.global	ftl_map_blk_gc
	.type	ftl_map_blk_gc, %function
ftl_map_blk_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r0
	ldr	r5, [r0, #12]
	ldr	sl, [r0, #24]
	bl	ftl_free_no_use_map_blk
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #8]
	sub	r3, r3, #5
	cmp	r2, r3
	blt	.L679
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r8, [r5, r0]
	cmp	r8, #0
	beq	.L679
	ldr	r3, [r4, #32]
	cmp	r3, #0
	bne	.L679
	mov	r2, #1
	str	r2, [r4, #32]
	strh	r3, [r5, r0]	@ movhi
	ldrh	r3, [r4, #8]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, .L689
	ldrh	r3, [r3, #100]
	cmp	r2, r3
	bcc	.L680
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L680:
	ldr	r6, .L689
	mov	r5, #0
	mov	r9, r6
	b	.L681
.L687:
	ldr	r3, [sl, r5, asl #2]
	add	fp, sl, r5, asl #2
	cmp	r8, r3, lsr #10
	bne	.L682
	ldr	r3, [r6, #3428]
	mov	r1, #1
	ldr	r7, [r6, #3292]
	mov	r2, r1
	ldr	r0, .L689+4
	str	r3, [r6, #3276]
	str	r7, [r6, #3280]
	ldr	r3, [sl, r5, asl #2]
	str	r3, [r6, #3272]
	bl	FlashReadPages
	ldrh	r3, [r7, #8]
	cmp	r3, r5
	beq	.L683
	ldr	r0, .L689+8
	movw	r2, #611
	ldr	r1, .L689+12
	bl	printk
.L683:
	ldr	r3, [r9, #3268]
	cmn	r3, #1
	beq	.L684
	ldrh	r3, [r7, #8]
	cmp	r3, r5
	bne	.L684
	ldrh	r2, [r7, #0]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	beq	.L685
.L684:
	mov	r3, #0
	str	r3, [fp, #0]
.L686:
	b	.L686
.L685:
	mov	r0, r4
	mov	r1, r5
	ldr	r2, [r6, #3276]
	bl	FtlMapWritePage
.L682:
	add	r5, r5, #1
	uxth	r5, r5
.L681:
	ldrh	r3, [r4, #6]
	cmp	r3, r5
	bhi	.L687
	mov	r0, r8
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	mov	r3, #0
	str	r3, [r4, #32]
.L679:
	ldr	r3, .L689
	ldrh	r2, [r4, #2]
	ldrh	r3, [r3, #100]
	cmp	r2, r3
	bcc	.L688
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L688:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L690:
	.align	2
.L689:
	.word	.LANCHOR0
	.word	.LANCHOR0+3268
	.word	.LC1
	.word	.LANCHOR1+318
	.fnend
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	2
	.global	flush_l2p_region
	.type	flush_l2p_region, %function
flush_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, #12
	ldr	r5, .L692
	mul	r4, r4, r0
	add	r0, r5, #3328
	ldr	r2, [r5, #2488]
	add	r3, r2, r4
	ldrh	r1, [r2, r4]
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
	ldr	r3, [r5, #2488]
	mov	r0, #0
	add	r4, r3, r4
	ldr	r3, [r4, #4]
	bic	r3, r3, #-2147483648
	str	r3, [r4, #4]
	ldmfd	sp!, {r3, r4, r5, pc}
.L693:
	.align	2
.L692:
	.word	.LANCHOR0
	.fnend
	.size	flush_l2p_region, .-flush_l2p_region
	.align	2
	.global	FtlMapBlkWriteDump_data
	.type	FtlMapBlkWriteDump_data, %function
FtlMapBlkWriteDump_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	ldr	r2, [r0, #36]
	ldrh	r6, [r0, #6]
	cmp	r2, #0
	ldr	r3, [r0, #24]
	ldmeqfd	sp!, {r4, r5, r6, pc}
	ldr	r5, .L698
	sub	r6, r6, #1
	mov	r2, #0
	str	r2, [r0, #36]
	uxth	r6, r6
	ldr	r0, [r5, #3428]
	ldr	r2, [r5, #3292]
	str	r0, [r5, #3276]
	str	r2, [r5, #3280]
	ldr	r3, [r3, r6, asl #2]
	cmp	r3, #0
	str	r3, [r5, #3272]
	beq	.L696
	mov	r1, #1
	ldr	r0, .L698+4
	mov	r2, r1
	bl	FlashReadPages
	b	.L697
.L696:
	mov	r1, #255
	ldrh	r2, [r5, #106]
	bl	ftl_memset
.L697:
	mov	r0, r4
	mov	r1, r6
	ldr	r2, [r5, #3276]
	ldmfd	sp!, {r4, r5, r6, lr}
	b	FtlMapWritePage
.L699:
	.align	2
.L698:
	.word	.LANCHOR0
	.word	.LANCHOR0+3268
	.fnend
	.size	FtlMapBlkWriteDump_data, .-FtlMapBlkWriteDump_data
	.align	2
	.global	load_l2p_region
	.type	load_l2p_region, %function
load_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r5, r0
	ldr	r7, .L707
	mov	r6, r1
	ldrh	r3, [r7, #132]
	cmp	r3, r0
	bcs	.L701
	ldr	r0, .L707+4
	movw	r2, #485
	ldr	r1, .L707+8
	bl	printk
.L701:
	ldr	r3, [r7, #3380]
	ldr	r4, .L707
	ldr	r8, [r3, r5, asl #2]
	mov	r3, #12
	cmp	r8, #0
	bne	.L702
	mul	r6, r3, r6
	ldr	r3, [r4, #2488]
	mov	r1, #255
	ldrh	r2, [r4, #106]
	add	r3, r3, r6
	ldr	r0, [r3, #8]
	bl	ftl_memset
	ldr	r3, [r4, #2488]
	strh	r5, [r3, r6]	@ movhi
	ldr	r3, [r4, #2488]
	add	r6, r3, r6
	str	r8, [r6, #4]
	b	.L703
.L702:
	mul	r6, r3, r6
	ldr	r3, [r4, #2488]
	mov	r1, #1
	ldr	r0, .L707+12
	mov	r2, r1
	str	r8, [r4, #3272]
	add	r3, r3, r6
	ldr	r3, [r3, #8]
	str	r3, [r4, #3276]
	ldr	r3, [r4, #3292]
	str	r3, [r4, #3280]
	bl	FlashReadPages
	ldr	r7, [r4, #3280]
	ldrh	r3, [r7, #8]
	cmp	r3, r5
	beq	.L704
	mov	r1, r5
	mov	r2, r8
	ldr	r0, .L707+16
	bl	printk
	mov	r2, #4
	ldr	r1, [r4, #3280]
	mov	r3, r2
	ldr	r0, .L707+20
	bl	rknand_print_hex
	ldr	r0, .L707+24
	ldr	r1, [r4, #3380]
	mov	r2, #4
	ldrh	r3, [r4, #132]
	bl	rknand_print_hex
	b	.L705
.L704:
	ldr	r3, [r4, #3268]
	cmp	r3, #256
	bne	.L705
	mov	r1, r5
	mov	r2, r8
	ldr	r0, .L707+28
	bl	printk
	ldr	r3, [r4, #2488]
	add	r0, r4, #3328
	mov	r1, r5
	add	r3, r3, r6
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
.L705:
	ldrh	r3, [r7, #8]
	cmp	r3, r5
	beq	.L706
	ldr	r0, .L707+4
	mov	r2, #508
	ldr	r1, .L707+8
	bl	printk
.L706:
	ldr	r3, .L707
	mov	r1, #0
	ldr	r3, [r3, #2488]
	add	r2, r3, r6
	str	r1, [r2, #4]
	strh	r5, [r3, r6]	@ movhi
.L703:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L708:
	.align	2
.L707:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+333
	.word	.LANCHOR0+3268
	.word	.LC86
	.word	.LC87
	.word	.LC88
	.word	.LC89
	.fnend
	.size	load_l2p_region, .-load_l2p_region
	.align	2
	.global	log2phys
	.type	log2phys, %function
log2phys:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r0
	ldr	sl, .L723
	mov	r6, r1
	mov	r8, r2
	ldrh	r7, [sl, #104]
	ldr	r3, [sl, #2504]
	add	r7, r7, #7
	mov	r5, r0, lsr r7
	cmp	r0, r3
	uxth	r5, r5
	bcc	.L710
	ldr	r0, .L723+4
	movw	r2, #811
	ldr	r1, .L723+8
	bl	printk
.L710:
	ldr	r2, [sl, #2504]
	ldr	r3, .L723
	cmp	r4, r2
	ldrcch	r1, [r3, #134]
	ldrcc	r2, [r3, #2488]
	movcc	r3, #0
	movcc	sl, r3
	bcc	.L712
.L711:
	cmp	r8, #0
	mvn	r0, #0
	streq	r0, [r6, #0]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L718:
	add	r3, r3, #12
	add	r0, r2, r3
	ldrh	r0, [r0, #-12]
	cmp	r0, r5
	bne	.L714
.L715:
	mvn	r3, #0
	cmp	r8, #0
	bic	r4, r4, r3, asl r7
	mov	r2, #12
	ldr	r3, .L723
	uxth	r4, r4
	bne	.L716
	ldr	r3, [r3, #2488]
	mla	r2, r2, sl, r3
	ldr	r3, [r2, #8]
	ldr	r3, [r3, r4, asl #2]
	str	r3, [r6, #0]
	b	.L717
.L716:
	mul	r2, r2, sl
	ldr	r1, [r3, #2488]
	ldr	r0, [r6, #0]
	add	r1, r1, r2
	ldr	r1, [r1, #8]
	str	r0, [r1, r4, asl #2]
	ldr	r1, [r3, #2488]
	add	r2, r1, r2
	ldr	r1, [r2, #4]
	orr	r1, r1, #-2147483648
	str	r1, [r2, #4]
	movw	r2, #2492
	strh	r5, [r3, r2]	@ movhi
.L717:
	ldr	r3, .L723
	mov	r2, #12
	ldr	r3, [r3, #2488]
	mla	sl, r2, sl, r3
	ldr	r3, [sl, #4]
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [sl, #4]
	movne	r0, #0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
	b	.L722
.L714:
	add	sl, sl, #1
	uxth	sl, sl
.L712:
	cmp	sl, r1
	bne	.L718
	bl	select_l2p_ram_region
	mov	r3, #12
	ldr	r2, .L723
	ldr	r2, [r2, #2488]
	mul	r3, r3, r0
	mov	sl, r0
	add	r1, r2, r3
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L719
	ldr	r3, [r1, #4]
	cmp	r3, #0
	bge	.L719
	bl	flush_l2p_region
.L719:
	mov	r0, r5
	mov	r1, sl
	bl	load_l2p_region
	b	.L715
.L722:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L724:
	.align	2
.L723:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+349
	.fnend
	.size	log2phys, .-log2phys
	.align	2
	.type	FtlReadRefresh.part.9, %function
FtlReadRefresh.part.9:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #92
	sub	sp, sp, #92
	ldr	r4, .L730
	mov	r5, #2048
.L728:
	ldr	r0, [r4, #2720]
	ldr	r3, [r4, #2504]
	cmp	r0, r3
	bcs	.L726
	mov	r2, #0
	mov	r1, sp
	bl	log2phys
	ldr	r2, [sp, #0]
	ldr	r3, [r4, #2720]
	cmn	r2, #1
	add	r3, r3, #1
	str	r3, [r4, #2720]
	beq	.L727
	str	r3, [sp, #20]
	add	r0, sp, #88
	ldr	r3, .L730
	mov	r1, #1
	str	r2, [sp, #8]
	mov	r2, #0
	str	r2, [r0, #-84]!
	ldr	r3, [r3, #3264]
	str	r3, [sp, #12]
	add	r3, sp, #24
	str	r3, [sp, #16]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmp	r3, #256
	bne	.L726
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L726
.L727:
	subs	r5, r5, #1
	bne	.L728
.L726:
	mvn	r0, #0
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, pc}
.L731:
	.align	2
.L730:
	.word	.LANCHOR0
	.fnend
	.size	FtlReadRefresh.part.9, .-FtlReadRefresh.part.9
	.align	2
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L738
	ldr	r2, [r3, #2716]
	cmp	r2, #0
	mov	r2, r3
	beq	.L733
	ldr	r1, [r3, #2720]
	ldr	r2, [r3, #2504]
	cmp	r1, r2
	bcs	.L734
	b	FtlReadRefresh.part.9
.L734:
	mov	r2, #0
	str	r2, [r3, #2716]
	str	r2, [r3, #2720]
	ldr	r2, [r3, #2508]
	str	r2, [r3, #2712]
	b	.L735
.L733:
	ldr	r1, [r3, #2712]
	ldr	r0, [r3, #2564]
	ldr	r3, [r3, #2508]
	add	ip, r3, #1048576
	cmp	r1, ip
	bhi	.L736
	mov	r0, r0, lsr #10
	mov	ip, #33554432
	add	r1, r1, ip, asr r0
	cmp	r1, r3
	bcc	.L736
	ldrb	r1, [r2, #2292]	@ zero_extendqisi2
	cmp	r1, #0
	bne	.L735
.L736:
	mov	r1, #1
	str	r3, [r2, #2712]
	str	r1, [r2, #2716]
	mov	r1, #0
	str	r1, [r2, #2720]
.L735:
	mov	r0, #0
	bx	lr
.L739:
	.align	2
.L738:
	.word	.LANCHOR0
	.fnend
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	2
	.global	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L767
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #44
	sub	sp, sp, #44
	ldrh	r5, [r3, #4]
	movw	r3, #65535
	mov	r4, r0
	str	r1, [sp, #12]
	cmp	r5, r3
	beq	.L762
	cmp	r5, #0
	bne	.L741
	b	.L742
.L762:
	mov	r5, #0
.L741:
	ldr	r3, .L767+4
	ldr	r9, [sp, #12]
	ldrh	r3, [r3, #98]
	cmp	r9, r3
	bne	.L743
.L742:
	bl	FtlGcPageVarInit
.L743:
	mov	fp, #0
.L744:
	ldrh	r3, [r4, #0]
	movw	r1, #65535
	mov	r0, #0
	strb	r0, [r4, #8]
	cmp	r3, r1
	beq	.L745
.L746:
	mov	sl, r4
.L763:
	ldr	r3, .L767+4
	mov	r2, sl
	ldrh	r8, [r3, #28]
	ldr	r7, [r3, #3416]
	ldr	r6, [r3, #3172]
	ldrh	lr, [r3, #106]
	ldr	ip, [r3, #3176]
	ldrh	r3, [r3, #108]
	str	r3, [sp, #4]
	mov	r3, #0
	mov	r4, r3
	b	.L747
.L749:
	ldrh	r0, [r2, #16]
	movw	r9, #65535
	cmp	r0, r9
	beq	.L748
	mov	r9, #20
	orr	r0, r5, r0, asl #10
	mla	r1, r9, r4, r7
	str	r0, [r1, #4]
	mul	r0, lr, r4
	add	r9, r0, #3
	cmp	r0, #0
	movlt	r0, r9
	ldr	r9, [sp, #4]
	bic	r0, r0, #3
	add	r0, r6, r0
	str	r0, [r1, #8]
	mul	r0, r9, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	r9, r0, #3
	cmp	r0, #0
	movlt	r0, r9
	bic	r0, r0, #3
	add	r0, ip, r0
	str	r0, [r1, #12]
.L748:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L747:
	cmp	r3, r8
	bne	.L749
	ldr	r6, .L767+4
	mov	r1, r4
	mov	r2, #0
	mov	r8, #0
	mov	r7, r5
	ldr	r0, [r6, #3416]
	bl	FlashReadPages
	str	r8, [sp, #4]
	b	.L750
.L758:
	ldr	r1, [r6, #3416]
	add	r2, r1, r8
	ldr	r9, [r2, #4]
	str	r9, [sp, #8]
	ldr	r9, [r1, r8]
	cmp	r9, #0
	bne	.L751
	ldr	r5, [r2, #12]
	movw	r0, #65535
	ldrh	r2, [r5, #0]
	cmp	r2, r0
	bne	.L752
	ldrh	r1, [sl, #0]
	mov	r4, sl
	ldr	r2, [sp, #8]
	ldr	r0, .L767+8
	str	r3, [sp, #0]
	bl	printk
	ldr	r3, [sp, #0]
	ldrh	r1, [sl, #0]
	ldr	r2, [r3, #2320]
	mov	r3, r1, asl #1
	strh	r9, [r2, r3]	@ movhi
	b	.L766
.L752:
	mov	r2, r9
	ldr	r0, [r5, #8]
	add	r1, sp, #16
	bl	log2phys
	ldr	r3, [sp, #16]
	ldr	r2, [r5, #12]
	cmp	r3, r2
	bne	.L754
	str	r3, [sp, #24]
	mov	r1, #1
	ldr	r3, [r6, #3308]
	mov	r2, r9
	add	r0, sp, #20
	str	r3, [sp, #28]
	ldr	r3, [r6, #3312]
	str	r3, [sp, #32]
	bl	FlashReadPages
	ldrh	r0, [r6, #54]
	mov	r3, r9
	ldr	r1, [r6, #3416]
	ldr	r2, [sp, #28]
	mov	r0, r0, asl #7
	add	r1, r1, r8
	b	.L755
.L757:
	ldr	ip, [r1, #8]
	ldr	lr, [ip, r9]
	add	r9, r9, #4
	add	ip, r2, r9
	ldr	ip, [ip, #-4]
	cmp	lr, ip
	beq	.L756
	ldrh	r1, [sl, #0]
	mov	r4, sl
	ldr	r2, [sp, #24]
	ldr	r0, .L767+12
	bl	printk
	ldr	r2, .L767+4
	ldrh	r3, [sl, #0]
	mov	r1, #0	@ movhi
	ldr	r2, [r2, #2320]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	b	.L766
.L756:
	add	r3, r3, #1
.L755:
	cmp	r3, r0
	bne	.L757
.L754:
	ldr	r0, [r5, #12]
	add	r8, r8, #20
	ldr	r1, [sp, #8]
	ldr	r2, [r5, #8]
	bl	FtlGcUpdatePage
	ldr	r9, [sp, #4]
	add	r3, r9, #1
	uxth	r3, r3
	str	r3, [sp, #4]
	b	.L750
.L751:
	ldrh	r1, [sl, #0]
	mov	r4, sl
	ldr	r2, [r3, #2320]
	mov	r0, #0	@ movhi
	mov	r3, r1, asl #1
	strh	r0, [r2, r3]	@ movhi
.L766:
	ldrh	r0, [r4, #0]
	mov	r5, #0
	bl	INSERT_FREE_LIST
	mvn	r3, #0
	strh	r3, [r4, #0]	@ movhi
	bl	FtlGcPageVarInit
	b	.L744
.L750:
	ldr	r3, [sp, #4]
	cmp	r3, r4
	ldr	r3, .L767+4
	bne	.L758
	ldr	r9, [sp, #12]
	add	fp, fp, #1
	add	r5, r7, #1
	cmp	fp, r9
	uxth	r5, r5
	bcc	.L759
	ldr	r2, .L767
	movw	r0, #65535
	ldrh	r1, [r2, #4]
	cmp	r1, r0
	beq	.L759
	add	r1, r1, fp
	strh	r1, [r2, #4]	@ movhi
	ldrh	r2, [r3, #98]
	cmp	r2, r5
	bhi	.L760
.L759:
	ldrh	r3, [r3, #98]
	cmp	r3, r5
	bhi	.L763
	mov	r4, sl
.L745:
	ldr	r3, .L767
	mvn	r2, #0
	strh	r5, [r4, #2]	@ movhi
	mov	r0, r4
	mov	r1, r5
	strh	r2, [r3, #4]	@ movhi
	mov	r2, #0
	strb	r2, [r4, #6]
	bl	ftl_sb_update_avl_pages
.L760:
	mvn	r0, #0
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L768:
	.align	2
.L767:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC90
	.word	.LC91
	.fnend
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	2
	.global	ftl_scan_all_data
	.type	ftl_scan_all_data, %function
ftl_scan_all_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	mov	r1, #0
	ldr	r4, .L775
	.pad #36
	sub	sp, sp, #36
	ldr	r0, .L775+4
	mov	r5, #0
	bl	printk
	mov	r6, r4
	b	.L770
.L774:
	mov	r0, r5
	add	r1, sp, #28
	mov	r2, #0
	bl	log2phys
	movs	r3, r5, asl #21
	bne	.L771
	ldr	r0, .L775+8
	mov	r1, r5
	ldr	r2, [sp, #28]
	bl	printk
.L771:
	ldr	r3, [sp, #28]
	cmn	r3, #1
	beq	.L772
	str	r3, [r4, #3272]
	mov	r2, #0
	ldr	r3, [r4, #3288]
	mov	r1, #1
	ldr	r7, [r4, #3292]
	ldr	r0, .L775+12
	str	r3, [r4, #3276]
	str	r5, [r4, #3284]
	str	r7, [r4, #3280]
	str	r2, [r4, #3268]
	bl	FlashReadPages
	ldr	r3, [r4, #3268]
	cmn	r3, #1
	cmpne	r3, #256
	beq	.L773
	ldr	r3, [r7, #8]
	cmp	r3, r5
	beq	.L772
.L773:
	ldr	r3, [r6, #3280]
	ldr	r2, [r6, #3276]
	ldr	r0, .L775+16
	ldr	r1, [r3, #4]
	str	r1, [sp, #0]
	ldr	r1, [r3, #8]
	str	r1, [sp, #4]
	ldr	r1, [r3, #12]
	str	r1, [sp, #8]
	ldr	r1, [r2, #0]
	str	r1, [sp, #12]
	mov	r1, r5
	ldr	r2, [r2, #4]
	str	r2, [sp, #16]
	ldr	r2, [r6, #3272]
	ldr	r3, [r3, #0]
	bl	printk
.L772:
	add	r5, r5, #1
.L770:
	ldr	r3, [r4, #2504]
	cmp	r5, r3
	bcc	.L774
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L776:
	.align	2
.L775:
	.word	.LANCHOR0
	.word	.LC92
	.word	.LC93
	.word	.LANCHOR0+3268
	.word	.LC94
	.fnend
	.size	ftl_scan_all_data, .-ftl_scan_all_data
	.align	2
	.global	FtlReUsePrevPpa
	.type	FtlReUsePrevPpa, %function
FtlReUsePrevPpa:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	mov	r8, r0
	ubfx	r0, r1, #10, #16
	str	r1, [sp, #4]
	bl	P2V_block_in_plane
	ldr	r2, .L784
	ldr	r3, [r2, #2320]
	mov	r4, r0, asl #1
	mov	r5, r0
	ldrh	r1, [r3, r4]
	cmp	r1, #0
	addne	r1, r1, #1
	strneh	r1, [r3, r4]	@ movhi
	bne	.L779
	ldr	r3, [r2, #2336]
	cmp	r3, #0
	beq	.L779
	movw	r0, #2340
	mov	ip, #6
	ldrh	lr, [r2, r0]
	ldr	r2, [r2, #2312]
	ldr	r0, .L784+4
	rsb	r3, r2, r3
	mov	r3, r3, asr #1
	mul	r3, r0, r3
	movw	r0, #65535
	uxth	r3, r3
	b	.L780
.L783:
	cmp	r3, r5
	bne	.L781
	ldr	r6, .L784
	mov	r1, r5
	movw	r7, #2340
	add	r0, r6, #2336
	bl	List_remove_node
	ldrh	r3, [r6, r7]
	cmp	r3, #0
	bne	.L782
	ldr	r0, .L784+8
	movw	r2, #1699
	ldr	r1, .L784+12
	bl	printk
.L782:
	ldrh	r3, [r6, r7]
	mov	r0, r5
	sub	r3, r3, #1
	strh	r3, [r6, r7]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r6, #2320]
	ldrh	r2, [r3, r4]
	add	r2, r2, #1
	strh	r2, [r3, r4]	@ movhi
	b	.L779
.L781:
	mul	r3, ip, r3
	ldrh	r3, [r2, r3]
	cmp	r3, r0
	beq	.L779
	add	r1, r1, #1
	uxth	r1, r1
.L780:
	cmp	r1, lr
	bne	.L783
.L779:
	mov	r0, r8
	add	r1, sp, #4
	mov	r2, #1
	bl	log2phys
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L785:
	.align	2
.L784:
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LC1
	.word	.LANCHOR1+358
	.fnend
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	2
	.global	ftl_check_vpc
	.type	ftl_check_vpc, %function
ftl_check_vpc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	mov	r4, #0
	ldr	r1, .L800
	ldr	r0, .L800+4
	bl	printk
	ldr	r0, .L800+8
	mov	r1, #0
	mov	r2, #8192
	bl	ftl_memset
	ldr	r5, .L800+12
	b	.L787
.L789:
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L788
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r0, r5, r0, asl #1
	add	r0, r0, #3456
	ldrh	r3, [r0, #4]
	add	r3, r3, #1
	strh	r3, [r0, #4]	@ movhi
.L788:
	add	r4, r4, #1
.L787:
	ldr	r3, [r5, #2504]
	cmp	r4, r3
	bcc	.L789
	mov	r4, #0
	ldr	r5, .L800+12
	movw	r8, #65535
	b	.L790
.L793:
	mov	r7, r4, asl #1
	ldr	r3, [r5, #2320]
	add	r6, r5, r7
	add	r6, r6, #3456
	ldrh	r2, [r3, r7]
	ldrh	r3, [r6, #4]
	add	r6, r6, #4
	cmp	r2, r3
	beq	.L791
	ldr	r0, .L800+16
	mov	r1, r4
	bl	printk
	ldr	r3, [r5, #2320]
	ldrh	r3, [r3, r7]
	cmp	r3, r8
	beq	.L791
	ldrh	r2, [r6, #0]
	cmp	r2, r3
	bls	.L791
.L799:
	b	.L799
.L791:
	add	r4, r4, #1
	uxth	r4, r4
.L790:
	ldrh	r2, [r5, #36]
	ldr	r3, .L800+12
	cmp	r2, r4
	bhi	.L793
	ldr	r2, [r3, #2336]
	cmp	r2, #0
	beq	.L786
	ldr	r4, [r3, #2312]
	movw	r1, #2340
	ldrh	r7, [r3, r1]
	mov	r1, #0
	rsb	r2, r4, r2
	ldr	r4, .L800+20
	mov	r6, r1
	mov	r5, r3
	mov	r2, r2, asr #1
	movw	r8, #65535
	mul	r4, r4, r2
	uxth	r4, r4
	b	.L795
.L798:
	ldr	r2, [r5, #2320]
	mov	r3, r4, asl #1
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	beq	.L796
	add	r3, r5, r3
	mov	r1, r4
	add	r3, r3, #3456
	ldr	r0, .L800+24
	add	r3, r3, #4
	ldrh	r3, [r3, #0]
	bl	printk
	mov	r1, #1
.L796:
	mov	r2, #6
	ldr	r3, [r5, #2312]
	mul	r4, r2, r4
	ldrh	r4, [r3, r4]
	cmp	r4, r8
	beq	.L797
	add	r6, r6, #1
	uxth	r6, r6
.L795:
	cmp	r6, r7
	bne	.L798
.L797:
	cmp	r1, #0
	beq	.L786
	ldr	r0, .L800+28
	movw	r2, #2342
	ldr	r1, .L800
	bl	printk
.L786:
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L801:
	.align	2
.L800:
	.word	.LANCHOR1+374
	.word	.LC95
	.word	.LANCHOR0+3460
	.word	.LANCHOR0
	.word	.LC96
	.word	-1431655765
	.word	.LC97
	.word	.LC1
	.fnend
	.size	ftl_check_vpc, .-ftl_check_vpc
	.align	2
	.global	FtlMapTblRecovery
	.type	FtlMapTblRecovery, %function
FtlMapTblRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r3, [r0, #16]
	mov	r4, r0
	ldrh	sl, [r0, #6]
	mov	r1, #0
	ldr	r8, [r0, #24]
	mov	fp, #0
	ldr	r5, .L822
	str	r3, [sp, #8]
	mov	r2, sl, asl #2
	ldrh	r3, [r0, #8]
	ldr	r9, [r0, #12]
	mov	r0, r8
	str	r3, [sp, #0]
	bl	ftl_memset
	ldr	r3, [r5, #3288]
	ldr	r6, [r5, #3292]
	str	fp, [r4, #32]
	str	r3, [r5, #3276]
	mvn	r3, #0
	str	r6, [r5, #3280]
	strh	r3, [r4, #0]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #1
	str	r3, [r4, #36]
	ldr	r3, [sp, #0]
	str	fp, [r4, #28]
	sub	r3, r3, #1
	str	r3, [sp, #4]
	b	.L803
.L816:
	ldr	r1, [sp, #4]
	cmp	r3, r1
	mov	r3, r3, asl #1
	bne	.L804
	ldrh	r0, [r9, r3]
	mov	r1, #1
	add	r2, r9, r3
	str	r2, [sp, #0]
	bl	FtlGetLastWrittenPage
	ldmib	sp, {r1, r2}
	strh	fp, [r4, #0]	@ movhi
	mov	r7, #0
	ldr	r5, .L822
	uxth	r0, r0
	add	r3, r0, #1
	strh	r3, [r4, #2]	@ movhi
	ldr	r3, [r2, r1, asl #2]
	sxth	r9, r0
	add	r9, r9, #1
	str	r3, [r4, #28]
	b	.L805
.L807:
	ldr	r1, [sp, #0]
	ldr	r0, .L822+4
	ldrh	r2, [r1, #0]
	mov	r1, #1
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r5, #3272]
	bl	FlashReadPages
	ldr	r3, [r5, #3268]
	cmn	r3, #1
	beq	.L806
	ldrh	r3, [r6, #8]
	cmp	r3, sl
	bcs	.L806
	ldrh	r2, [r4, #4]
	ldrh	r1, [r6, #0]
	cmp	r1, r2
	ldreq	r2, [r5, #3272]
	streq	r2, [r8, r3, asl #2]
.L806:
	add	r7, r7, #1
	uxth	r7, r7
.L805:
	sxth	r3, r7
	cmp	r3, r9
	blt	.L807
	b	.L808
.L804:
	ldr	r2, [r5, #3288]
	mov	r1, #1
	ldr	r0, .L822+4
	str	r2, [r5, #3276]
	add	r2, r9, r3
	str	r2, [sp, #12]
	ldrh	r2, [r9, r3]
	ldrh	r3, [r5, #100]
	sub	r3, r3, #1
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r5, #3272]
	bl	FlashReadPages
	ldr	r3, [r5, #3268]
	cmn	r3, #1
	beq	.L819
	ldrh	r2, [r6, #0]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	bne	.L819
	ldrh	r2, [r6, #8]
	movw	r3, #64245
	cmp	r2, r3
	bne	.L819
	b	.L820
.L812:
	ldr	r1, [r5, #3288]
	mov	r0, r2, asl #3
	add	r3, r3, #1
	ldr	r2, [r1, r2, asl #3]
	uxth	r3, r3
	uxth	r2, r2
	cmp	r2, sl
	addcc	r1, r1, r0
	ldrcc	r1, [r1, #4]
	strcc	r1, [r8, r2, asl #2]
	b	.L810
.L820:
	mov	r3, #0
.L810:
	ldrh	r1, [r5, #100]
	sxth	r2, r3
	sub	r1, r1, #1
	cmp	r2, r1
	blt	.L812
	b	.L813
.L815:
	ldr	r1, [sp, #12]
	ldr	r0, .L822+4
	ldrh	r2, [r1, #0]
	mov	r1, #1
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r5, #3272]
	bl	FlashReadPages
	ldr	r3, [r5, #3268]
	cmn	r3, #1
	beq	.L814
	ldrh	r3, [r6, #8]
	cmp	r3, sl
	bcs	.L814
	ldrh	r2, [r4, #4]
	ldrh	r1, [r6, #0]
	cmp	r1, r2
	ldreq	r2, [r5, #3272]
	streq	r2, [r8, r3, asl #2]
.L814:
	add	r7, r7, #1
	uxth	r7, r7
	b	.L821
.L819:
	mov	r7, #0
.L821:
	ldrh	r2, [r5, #100]
	sxth	r3, r7
	cmp	r3, r2
	blt	.L815
.L813:
	add	fp, fp, #1
	uxth	fp, fp
.L803:
	ldr	r2, [sp, #0]
	sxth	r3, fp
	cmp	r3, r2
	blt	.L816
.L808:
	mov	r0, r4
	bl	ftl_free_no_use_map_blk
	ldr	r3, .L822
	ldrh	r2, [r4, #2]
	ldrh	r3, [r3, #100]
	cmp	r2, r3
	bne	.L817
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L817:
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, #0
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L823:
	.align	2
.L822:
	.word	.LANCHOR0
	.word	.LANCHOR0+3268
	.fnend
	.size	FtlMapTblRecovery, .-FtlMapTblRecovery
	.align	2
	.global	FtlLoadMapInfo
	.type	FtlLoadMapInfo, %function
FtlLoadMapInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	FtlL2PDataInit
	ldr	r0, .L825
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L826:
	.align	2
.L825:
	.word	.LANCHOR0+3328
	.fnend
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.align	2
	.global	FtlLoadVonderInfo
	.type	FtlLoadVonderInfo, %function
FtlLoadVonderInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L828
	movw	r1, #3486
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L828+4
	ldrh	r0, [r2, #116]
	strh	r0, [r3, r1]	@ movhi
	movw	r1, #3480
	ldr	r0, .L828+8
	strh	r0, [r3, r1]	@ movhi
	add	r1, r1, #4
	ldrh	r0, [r2, #140]
	strh	r0, [r3, r1]	@ movhi
	movw	r1, #3482
	ldrh	r0, [r2, #118]
	strh	r0, [r3, r1]	@ movhi
	ldr	r1, [r2, #144]
	ldr	r0, .L828+12
	str	r1, [r3, #3488]
	ldr	r1, [r2, #3452]
	str	r1, [r3, #3492]
	ldr	r1, [r2, #3448]
	ldr	r2, [r2, #3456]
	str	r1, [r3, #3496]
	str	r2, [r3, #3500]
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L829:
	.align	2
.L828:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	-3962
	.word	.LANCHOR3+3476
	.fnend
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.align	2
	.global	FtlLoadSysInfo
	.type	FtlLoadSysInfo, %function
FtlLoadSysInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r1, #0
	ldr	r4, .L850
	ldr	r3, [r4, #3288]
	ldrh	r2, [r4, #36]
	ldr	r0, [r4, #2320]
	str	r3, [r4, #3276]
	ldr	r3, [r4, #3292]
	mov	r2, r2, asl #1
	str	r3, [r4, #3280]
	bl	ftl_memset
	mov	r3, #2576
	ldrh	r0, [r4, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L846
	mov	r1, #1
	mov	r5, #0
	bl	FtlGetLastWrittenPage
	movw	r3, #2578
	ldr	r8, .L850+4
	movw	r7, #61604
	add	r2, r0, #1
	mov	r6, r0
	strh	r2, [r4, r3]	@ movhi
	b	.L832
.L835:
	mov	r2, #2576
	mov	r1, #1
	ldrh	r2, [r4, r2]
	ldr	r0, .L850+8
	orr	r3, r3, r2, asl #10
	str	r3, [r4, #3272]
	ldr	r3, [r4, #3288]
	mov	r2, r1
	str	r3, [r4, #3276]
	bl	FlashReadPages
	ldr	r3, [r4, #3268]
	cmn	r3, #1
	beq	.L833
	ldr	r3, [r4, #3288]
	ldr	r3, [r3, #0]
	cmp	r3, r8
	bne	.L833
	ldr	r3, [r4, #3292]
	ldrh	r3, [r3, #0]
	cmp	r3, r7
	beq	.L834
.L833:
	sub	r5, r5, #1
	uxth	r5, r5
.L832:
	add	r3, r5, r6
	sxth	r3, r3
	cmp	r3, #0
	bge	.L835
	b	.L849
.L834:
	ldr	r5, .L850
	ldrh	r2, [r5, #36]
	ldrh	r3, [r5, #106]
	add	r2, r2, #24
	cmp	r3, r2, asl #1
	bcs	.L837
	ldr	r0, .L850+12
	movw	r2, #1450
	ldr	r1, .L850+16
	bl	printk
.L837:
	ldr	r6, [r5, #3276]
	mov	r2, #48
	ldr	r0, .L850+20
	ldr	r4, .L850
	mov	r1, r6
	bl	memcpy
	ldrh	r2, [r5, #36]
	add	r1, r6, #48
	ldr	r0, [r5, #2320]
	mov	r2, r2, asl #1
	bl	memcpy
	ldrh	r1, [r5, #36]
	ldr	r3, [r5, #3276]
	ldr	r0, [r5, #2500]
	mov	r2, r1, lsr #3
	add	r1, r1, #24
	add	r2, r2, #4
	mov	r1, r1, lsr #1
	add	r1, r3, r1, asl #2
	bl	memcpy
	ldr	r2, [r5, #2264]
	ldr	r3, .L850+4
	cmp	r2, r3
	bne	.L846
	mov	r3, #2272
	ldrb	r2, [r4, #2274]	@ zero_extendqisi2
	ldrh	r5, [r4, r3]
	movw	r3, #2582
	strh	r5, [r4, r3]	@ movhi
	ldrh	r3, [r4, #50]
	cmp	r2, r3
	bne	.L846
	ldr	r3, .L850+24
	ldrh	r2, [r4, #54]
	ldr	r6, [r4, #40]
	str	r5, [r3, #3516]
	ldrh	r3, [r4, #98]
	ldrh	r0, [r4, #154]
	ldrh	r1, [r4, #28]
	mul	r3, r5, r3
	rsb	r0, r0, r6
	rsb	r0, r5, r0
	str	r3, [r4, #2504]
	mul	r3, r2, r3
	str	r3, [r4, #136]
	bl	__aeabi_uidiv
	cmp	r5, r6
	movw	r3, #2572
	strh	r0, [r4, r3]	@ movhi
	bls	.L838
	ldr	r0, .L850+12
	mov	r2, #1472
	ldr	r1, .L850+16
	bl	printk
.L838:
	ldr	r3, .L850
	movw	r2, #2278
	movw	r0, #2346
	movw	r5, #65535
	ldrh	r1, [r3, r2]
	add	r2, r2, #66
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #2280
	ldrh	r2, [r3, r2]
	mov	ip, r2, lsr #6
	and	r2, r2, #63
	strb	r2, [r3, #2350]
	ldrb	r2, [r3, #2275]	@ zero_extendqisi2
	strh	ip, [r3, r0]	@ movhi
	mvn	r0, #0
	movw	ip, #2590
	strb	r2, [r3, #2352]
	movw	r2, #2588
	strh	r0, [r3, r2]	@ movhi
	movw	r0, #2282
	mov	r2, #0
	strh	r2, [r3, ip]	@ movhi
	ldrh	ip, [r3, r0]
	add	r0, r0, #110
	strb	r2, [r3, #2594]
	strb	r2, [r3, #2596]
	strh	ip, [r3, r0]	@ movhi
	movw	r0, #2284
	ldrh	r0, [r3, r0]
	movw	ip, #2394
	str	r2, [r3, #2528]
	str	r2, [r3, #2516]
	mov	r4, r0, lsr #6
	and	r0, r0, #63
	strb	r0, [r3, #2398]
	ldrb	r0, [r3, #2276]	@ zero_extendqisi2
	strh	r4, [r3, ip]	@ movhi
	str	r2, [r3, #2508]
	strb	r0, [r3, #2400]
	movw	r0, #2286
	ldrh	ip, [r3, r0]
	add	r0, r0, #154
	str	r2, [r3, #2524]
	str	r2, [r3, #2552]
	strh	ip, [r3, r0]	@ movhi
	mov	r0, #2288
	ldrh	r0, [r3, r0]
	movw	ip, #2442
	str	r2, [r3, #2564]
	mov	r4, r0, lsr #6
	and	r0, r0, #63
	strb	r0, [r3, #2446]
	ldrb	r0, [r3, #2277]	@ zero_extendqisi2
	strh	r4, [r3, ip]	@ movhi
	strb	r0, [r3, #2448]
	ldr	r0, [r3, #2296]
	str	r0, [r3, #2548]
	str	r2, [r3, #2520]
	ldr	r2, [r3, #2304]
	ldr	r0, [r3, #2540]
	cmp	r2, r0
	strhi	r2, [r3, #2540]
	ldr	r2, [r3, #2300]
	ldr	r3, [r3, #2544]
	cmp	r2, r3
	ldrhi	r3, .L850
	strhi	r2, [r3, #2544]
	cmp	r1, r5
	beq	.L841
	ldr	r0, .L850+28
	bl	make_superblock
.L841:
	ldr	r4, .L850
	movw	r3, #2392
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L842
	ldr	r0, .L850+32
	bl	make_superblock
.L842:
	movw	r3, #2440
	ldrh	r3, [r4, r3]
	movw	r4, #65535
	cmp	r3, r4
	beq	.L843
	ldr	r0, .L850+36
	bl	make_superblock
.L843:
	ldr	r2, .L850
	movw	r3, #2588
	ldrh	r3, [r2, r3]
	cmp	r3, r4
	beq	.L847
	ldr	r0, .L850+40
	bl	make_superblock
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L846:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L847:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L849:
	ldr	r1, .L850+16
	movw	r2, #1448
	ldr	r0, .L850+12
	bl	printk
	b	.L834
.L851:
	.align	2
.L850:
	.word	.LANCHOR0
	.word	1179929683
	.word	.LANCHOR0+3268
	.word	.LC1
	.word	.LANCHOR1+388
	.word	.LANCHOR0+2264
	.word	.LANCHOR3
	.word	.LANCHOR0+2344
	.word	.LANCHOR0+2392
	.word	.LANCHOR0+2440
	.word	.LANCHOR0+2588
	.fnend
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r7, #0
	ldr	r4, .L864
	mov	r1, #255
	movw	r8, #65535
	movw	r6, #2578
	ldr	r3, [r4, #3288]
	mov	sl, r4
	ldr	r5, [r4, #3292]
	str	r3, [r4, #3276]
	mov	r3, #2576
	ldrh	r3, [r4, r3]
	str	r5, [r4, #3280]
	str	r7, [r5, #12]
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, .L864+4
	strh	r3, [r5, #0]	@ movhi
	ldr	r3, [r4, #2584]
	str	r7, [r5, #8]
	str	r3, [r5, #4]
	ldr	r3, .L864+8
	str	r3, [r4, #2264]
	ldr	r3, .L864+12
	str	r3, [r4, #2268]
	movw	r3, #2582
	ldrh	r2, [r4, r3]
	mov	r3, #2272
	strh	r2, [r4, r3]	@ movhi
	ldrh	r3, [r4, #50]
	strb	r3, [r4, #2274]
	movw	r3, #2344
	ldrh	r2, [r4, r3]
	movw	r3, #2278
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #68
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #2350]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	movw	r3, #2280
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #2352]	@ zero_extendqisi2
	strb	r3, [r4, #2275]
	movw	r3, #2392
	ldrh	r2, [r4, r3]
	movw	r3, #2282
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #112
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #2398]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	movw	r3, #2284
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #2400]	@ zero_extendqisi2
	strb	r3, [r4, #2276]
	movw	r3, #2440
	ldrh	r2, [r4, r3]
	movw	r3, #2286
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #156
	ldr	r0, [r4, #3276]
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #2446]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	mov	r3, #2288
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #2448]	@ zero_extendqisi2
	ldrh	r2, [r4, #106]
	strb	r3, [r4, #2277]
	ldr	r3, [r4, #2548]
	str	r3, [r4, #2296]
	ldr	r3, [r4, #2540]
	str	r3, [r4, #2304]
	ldr	r3, [r4, #2544]
	str	r3, [r4, #2300]
	bl	ftl_memset
	ldr	r1, .L864+16
	mov	r2, #48
	ldr	r0, [r4, #3276]
	bl	memcpy
	ldrh	r2, [r4, #36]
	ldr	r0, [r4, #3276]
	ldr	r1, [r4, #2320]
	mov	r2, r2, asl #1
	add	r0, r0, #48
	bl	memcpy
	ldrh	r2, [r4, #36]
	ldr	r3, [r4, #3276]
	add	r0, r2, #24
	ldr	r1, [r4, #2500]
	mov	r2, r2, lsr #3
	mov	r0, r0, lsr #1
	add	r2, r2, #4
	add	r0, r3, r0, asl #2
	bl	memcpy
	mov	r0, r7
	bl	FtlUpdateVaildLpn
.L863:
	ldr	r3, [r4, #3288]
	mov	r9, #2576
	ldrh	r2, [r4, r9]
	str	r3, [r4, #3276]
	ldr	r3, [r4, #3292]
	str	r3, [r4, #3280]
	ldrh	r3, [r4, r6]
	orr	r1, r3, r2, asl #10
	str	r1, [r4, #3272]
	ldrh	r1, [r4, #100]
	sub	r1, r1, #1
	cmp	r3, r1
	blt	.L854
	movw	r3, #2580
	ldrh	r8, [r4, r3]
	strh	r2, [r4, r3]	@ movhi
	mov	r3, #0
	strh	r3, [r4, r6]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, [r4, #2540]
	add	r2, r3, #1
	str	r2, [r4, #2540]
	str	r3, [r4, #2584]
	mov	r2, r0, asl #10
	strh	r0, [r4, r9]	@ movhi
	str	r2, [r4, #3272]
	str	r3, [r5, #4]
	strh	r0, [r5, #2]	@ movhi
.L854:
	mov	r1, #1
	ldr	r0, .L864+20
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #3268]
	add	r3, r3, #1
	cmn	r2, #1
	uxth	r3, r3
	strh	r3, [r4, r6]	@ movhi
	bne	.L855
	cmp	r3, #1
	bne	.L856
	ldr	r0, .L864+24
	add	r2, r2, #1136
	ldr	r1, .L864+28
	bl	printk
.L856:
	movw	r3, #2578
	add	r7, r7, #1
	ldrh	r3, [sl, r3]
	uxth	r7, r7
	cmp	r3, #1
	ldreqh	r3, [r4, #100]
	subeq	r3, r3, #1
	streqh	r3, [r4, r6]	@ movhi
	cmp	r7, #3
	bls	.L863
	ldr	r3, .L864
	mov	r2, r7
	ldr	r0, .L864+32
	ldr	r1, [r3, #3272]
	bl	printk
.L859:
	b	.L859
.L855:
	cmp	r3, #1
	beq	.L863
	cmp	r2, #256
	beq	.L863
	movw	r3, #65535
	cmp	r8, r3
	beq	.L860
	mov	r0, r8
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L860:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L865:
	.align	2
.L864:
	.word	.LANCHOR0
	.word	-3932
	.word	1179929683
	.word	1342177352
	.word	.LANCHOR0+2264
	.word	.LANCHOR0+3268
	.word	.LC1
	.word	.LANCHOR1+403
	.word	.LC98
	.fnend
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	2
	.global	FtlVendorPartRead
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r9, r2
	ldr	r5, .L877
	add	r2, r1, r0
	.pad #100
	sub	sp, sp, #100
	mov	r8, r0
	mov	r6, r1
	ldrh	r3, [r5, #92]
	ldrh	r7, [r5, #104]
	cmp	r2, r3
	mvnhi	fp, #0
	bhi	.L867
	mov	r7, r0, lsr r7
	mov	fp, #0
	mov	r3, r7, asl #2
	str	r3, [sp, #0]
	b	.L868
.L874:
	ldrh	r4, [r5, #54]
	mov	r0, r8
	ldr	r3, [r5, #3456]
	ldr	r2, [sp, #0]
	mov	r1, r4
	ldr	sl, [r3, r2]
	bl	__aeabi_uidivmod
	uxth	r1, r1
	str	r1, [sp, #4]
	rsb	r4, r1, r4
	uxth	r4, r4
	cmp	r4, r6
	uxthhi	r4, r6
	cmp	sl, #0
	beq	.L870
	ldr	r2, .L877
	mov	r1, #1
	add	r0, sp, #12
	str	sl, [sp, #16]
	ldr	r3, [r2, #3432]
	mov	r2, r1
	str	r3, [sp, #20]
	add	r3, sp, #32
	str	r3, [sp, #24]
	bl	FlashReadPages
	ldr	r3, [sp, #12]
	cmn	r3, #1
	moveq	fp, r3
	ldr	r3, [r5, #3268]
	cmp	r3, #256
	bne	.L872
	mov	r1, r7
	mov	r2, sl
	ldr	r0, .L877+4
	bl	printk
	ldr	r0, .L877+8
	mov	r1, r7
	ldr	r2, [r5, #3432]
	bl	FtlMapWritePage
.L872:
	ldr	r3, [sp, #4]
	mov	r0, r9
	ldr	r1, [r5, #3432]
	mov	r2, r4, asl #9
	add	r1, r1, r3, asl #9
	bl	memcpy
	b	.L873
.L870:
	mov	r0, r9
	mov	r1, sl
	mov	r2, r4, asl #9
	bl	ftl_memset
.L873:
	ldr	r3, [sp, #0]
	add	r7, r7, #1
	rsb	r6, r4, r6
	add	r8, r8, r4
	add	r9, r9, r4, asl #9
	add	r3, r3, #4
	str	r3, [sp, #0]
.L868:
	cmp	r6, #0
	bne	.L874
.L867:
	mov	r0, fp
	add	sp, sp, #100
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L878:
	.align	2
.L877:
	.word	.LANCHOR0
	.word	.LC99
	.word	.LANCHOR3+3476
	.fnend
	.size	FtlVendorPartRead, .-FtlVendorPartRead
	.align	2
	.global	Ftl_load_ext_data
	.type	Ftl_load_ext_data, %function
Ftl_load_ext_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r0, #0
	ldr	r5, .L882
	mov	r1, #1
	ldr	r2, .L882+4
	bl	FtlVendorPartRead
	ldr	r4, .L882+8
	ldr	r3, [r5, #2636]
	cmp	r3, r4
	beq	.L880
	ldr	r0, .L882+4
	mov	r1, #0
	mov	r2, #512
	bl	ftl_memset
	str	r4, [r5, #2636]
.L880:
	ldr	r2, [r5, #2636]
	ldr	r3, .L882
	cmp	r2, r4
	bne	.L881
	ldr	r2, [r3, #2724]
	str	r2, [r3, #2532]
	ldr	r2, [r3, #2728]
	str	r2, [r3, #2536]
	ldr	r2, [r3, #2644]
	str	r2, [r3, #2528]
	ldr	r2, [r3, #2648]
	str	r2, [r3, #2516]
	ldr	r2, [r3, #2652]
	str	r2, [r3, #2508]
	ldr	r2, [r3, #2656]
	str	r2, [r3, #2524]
	ldr	r2, [r3, #2664]
	str	r2, [r3, #2552]
	ldr	r2, [r3, #2668]
	str	r2, [r3, #2560]
	ldr	r2, [r3, #2672]
	str	r2, [r3, #2512]
	ldr	r2, [r3, #2676]
	str	r2, [r3, #2520]
	ldr	r2, [r3, #2680]
	str	r2, [r3, #2564]
	ldr	r2, [r3, #2684]
	str	r2, [r3, #2568]
.L881:
	ldr	r4, .L882
	ldrh	r2, [r4, #88]
	ldr	r3, [r4, #2552]
	ldr	r0, [r4, #2548]
	ldrh	r1, [r4, #36]
	mla	r0, r0, r2, r3
	bl	__aeabi_uidiv
	str	r0, [r4, #2556]
	ldmfd	sp!, {r3, r4, r5, pc}
.L883:
	.align	2
.L882:
	.word	.LANCHOR0
	.word	.LANCHOR0+2636
	.word	1179929683
	.fnend
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
	.align	2
	.global	FtlLoadEctTbl
	.type	FtlLoadEctTbl, %function
FtlLoadEctTbl:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movw	r5, #3436
	ldr	r4, .L886
	mov	r0, #64
	ldr	r2, [r4, #3440]
	ldrh	r1, [r4, r5]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #3440]
	ldr	r2, [r3, #0]
	ldr	r3, .L886+4
	cmp	r2, r3
	beq	.L885
	ldr	r1, .L886+8
	ldr	r0, .L886+12
	bl	printk
	ldrh	r2, [r4, r5]
	ldr	r0, [r4, #3440]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L885:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L887:
	.align	2
.L886:
	.word	.LANCHOR0
	.word	1112818501
	.word	.LC100
	.word	.LC70
	.fnend
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	2
	.global	FtlVendorPartWrite
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #108
	sub	sp, sp, #108
	ldr	r6, .L898
	mov	r7, r0
	str	r2, [sp, #8]
	add	r2, r1, r0
	mov	r4, r1
	ldrh	r3, [r6, #92]
	ldrh	r8, [r6, #104]
	cmp	r2, r3
	mvnhi	r9, #0
	bhi	.L889
	mov	r8, r0, lsr r8
	mov	r9, #0
	mov	sl, r6
	mov	r3, r8, asl #2
	str	r3, [sp, #4]
	b	.L890
.L895:
	ldr	r2, [sp, #4]
	mov	r0, r7
	ldr	r3, [r6, #3456]
	ldrh	fp, [r6, #54]
	ldr	r3, [r3, r2]
	mov	r1, fp
	str	r3, [sp, #0]
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #0]
	uxth	r1, r1
	str	r1, [sp, #12]
	rsb	r5, r1, fp
	uxth	r5, r5
	cmp	r5, r4
	uxthhi	r5, r4
	cmp	r3, #0
	beq	.L892
	cmp	r5, fp
	beq	.L892
	str	r3, [sp, #24]
	mov	r1, #1
	ldr	r3, [sl, #3432]
	add	r0, sp, #20
	mov	r2, r1
	str	r3, [sp, #28]
	add	r3, sp, #40
	str	r3, [sp, #32]
	bl	FlashReadPages
	b	.L893
.L892:
	ldr	r0, [sl, #3432]
	mov	r1, #0
	ldrh	r2, [sl, #106]
	bl	ftl_memset
.L893:
	ldr	r3, [sp, #12]
	mov	fp, r5, asl #9
	ldr	r0, [r6, #3432]
	rsb	r4, r5, r4
	ldr	r1, [sp, #8]
	mov	r2, fp
	add	r0, r0, r3, asl #9
	add	r7, r7, r5
	bl	memcpy
	mov	r1, r8
	ldr	r0, .L898+4
	add	r8, r8, #1
	ldr	r2, [r6, #3432]
	bl	FtlMapWritePage
	ldr	r3, [sp, #8]
	add	r3, r3, fp
	str	r3, [sp, #8]
	ldr	r3, [sp, #4]
	add	r3, r3, #4
	str	r3, [sp, #4]
	cmn	r0, #1
	moveq	r9, r0
.L890:
	cmp	r4, #0
	bne	.L895
.L889:
	mov	r0, r9
	add	sp, sp, #108
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L899:
	.align	2
.L898:
	.word	.LANCHOR0
	.word	.LANCHOR3+3476
	.fnend
	.size	FtlVendorPartWrite, .-FtlVendorPartWrite
	.align	2
	.global	Ftl_save_ext_data
	.type	Ftl_save_ext_data, %function
Ftl_save_ext_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L902
	ldr	r2, .L902+4
	ldr	r1, [r3, #2636]
	cmp	r1, r2
	bxne	lr
	ldr	r2, .L902+8
	mov	r0, #0
	mov	r1, #1
	str	r2, [r3, #2640]
	ldr	r2, [r3, #2532]
	str	r2, [r3, #2724]
	ldr	r2, [r3, #2536]
	str	r2, [r3, #2728]
	ldr	r2, [r3, #2528]
	str	r2, [r3, #2644]
	ldr	r2, [r3, #2516]
	str	r2, [r3, #2648]
	ldr	r2, [r3, #2508]
	str	r2, [r3, #2652]
	ldr	r2, [r3, #2524]
	str	r2, [r3, #2656]
	ldr	r2, [r3, #2552]
	str	r2, [r3, #2664]
	ldr	r2, [r3, #2560]
	str	r2, [r3, #2668]
	ldr	r2, [r3, #2512]
	str	r2, [r3, #2672]
	ldr	r2, [r3, #2520]
	str	r2, [r3, #2676]
	ldr	r2, [r3, #2564]
	str	r2, [r3, #2680]
	ldr	r2, [r3, #2568]
	str	r2, [r3, #2684]
	ldr	r2, .L902+12
	b	FtlVendorPartWrite
.L903:
	.align	2
.L902:
	.word	.LANCHOR0
	.word	1179929683
	.word	1342177352
	.word	.LANCHOR0+2636
	.fnend
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.align	2
	.global	FtlEctTblFlush
	.type	FtlEctTblFlush, %function
FtlEctTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L909
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	mov	r3, #3520
	ldrh	r1, [r2, r3]
	cmp	r1, #31
	addls	r1, r1, #1
	strlsh	r1, [r2, r3]	@ movhi
	movhi	r1, #32
	movls	r1, #1
	cmp	r0, #0
	ldr	r3, .L909+4
	bne	.L906
	ldr	r2, [r3, #3440]
	ldr	r0, [r2, #20]
	ldr	r2, [r2, #16]
	add	r1, r1, r0
	cmp	r2, r1
	bcc	.L907
.L906:
	ldr	r2, [r3, #3440]
	mov	r0, #64
	ldr	r1, [r2, #16]
	str	r1, [r2, #20]
	ldr	r1, .L909+8
	str	r1, [r2, #0]
	movw	r1, #3436
	ldr	r2, [r3, #3440]
	ldrh	r1, [r3, r1]
	mov	r3, r1, asl #9
	str	r3, [r2, #12]
	ldr	r3, [r2, #8]
	add	r3, r3, #1
	str	r3, [r2, #8]
	mov	r3, #0
	str	r3, [r2, #4]
	bl	FtlVendorPartWrite
	bl	Ftl_save_ext_data
.L907:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L910:
	.align	2
.L909:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	1112818501
	.fnend
	.size	FtlEctTblFlush, .-FtlEctTblFlush
	.align	2
	.global	FtlBbmTblFlush
	.type	FtlBbmTblFlush, %function
FtlBbmTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	mov	r1, #0
	ldr	r4, .L923
	mov	r5, #0
	movw	r8, #3384
	add	r7, r4, #172
	ldr	r0, [r4, #3288]
	mov	r6, r4
	ldr	r3, [r4, #3292]
	ldrh	r2, [r4, #106]
	str	r0, [r4, #3276]
	str	r3, [r4, #3280]
	bl	ftl_memset
	b	.L912
.L913:
	ldrh	r2, [r6, r8]
	ldr	r3, [r6, #3276]
	ldr	r1, [r7, #4]!
	mul	r0, r2, r5
	mov	r2, r2, asl #2
	add	r5, r5, #1
	add	r0, r3, r0, asl #2
	bl	memcpy
.L912:
	ldrh	r3, [r6, #50]
	ldr	r4, .L923
	cmp	r5, r3
	blt	.L913
	ldr	r5, [r4, #3280]
	mov	r1, #255
	mov	r2, #16
	mov	r6, #0
	mov	r7, r6
	mov	r8, r6
	mov	r0, r5
	mov	sl, r6
	bl	ftl_memset
	ldr	r3, .L923+4
	strh	r3, [r5, #0]	@ movhi
	ldr	r3, [r4, #156]
	str	r3, [r5, #4]
	ldrh	r3, [r4, #148]
	strh	r3, [r5, #2]	@ movhi
	ldrh	r3, [r4, #152]
	strh	r3, [r5, #8]	@ movhi
	ldrh	r3, [r4, #154]
	strh	r3, [r5, #10]	@ movhi
	ldr	r3, [r4, #24]
	strh	r3, [r5, #12]	@ movhi
	b	.L922
.L919:
	mov	r7, #1
.L922:
	ldr	r3, [r4, #3288]
	ldrh	r1, [r4, #148]
	ldrh	r2, [r4, #150]
	str	r3, [r4, #3276]
	ldr	r3, [r4, #3292]
	str	sl, [r4, #3268]
	str	r3, [r4, #3280]
	orr	r3, r2, r1, asl #10
	ldrh	r0, [r5, #10]
	str	r3, [r4, #3272]
	ldrh	r3, [r4, #152]
	str	r0, [sp, #0]
	ldr	r0, .L923+8
	bl	printk
	ldrh	r3, [r4, #100]
	ldrh	r2, [r4, #150]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L915
	ldr	r3, [r4, #156]
	mov	r1, #1
	ldrh	r2, [r4, #148]
	add	r3, r3, #1
	ldr	r0, [r4, #3300]
	str	r3, [r4, #156]
	str	r3, [r5, #4]
	ldrh	r3, [r4, #152]
	strh	r2, [r5, #8]	@ movhi
	strh	r2, [r4, #152]	@ movhi
	mov	r2, r1
	strh	r3, [r4, #148]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r4, #3272]
	str	r3, [r0, #4]
	strh	r8, [r4, #150]	@ movhi
	bl	FlashEraseBlocks
.L915:
	mov	r1, #1
	ldr	r0, .L923+12
	mov	r3, r1
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #150]
	add	r3, r3, #1
	strh	r3, [r4, #150]	@ movhi
	ldr	r3, [r4, #3268]
	cmn	r3, #1
	bne	.L916
	add	r6, r6, #1
	ldr	r0, .L923+16
	ldr	r1, [r4, #3272]
	uxth	r6, r6
	bl	printk
	cmp	r6, #3
	bls	.L922
	ldr	r3, .L923
	mov	r2, r6
	ldr	r0, .L923+20
	ldr	r1, [r3, #3272]
	bl	printk
.L918:
	b	.L918
.L916:
	cmp	r7, #0
	beq	.L919
	mov	r0, #0
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L924:
	.align	2
.L923:
	.word	.LANCHOR0
	.word	-3887
	.word	.LC101
	.word	.LANCHOR0+3268
	.word	.LC102
	.word	.LC103
	.fnend
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.align	2
	.global	FtlGcFreeBadSuperBlk
	.type	FtlGcFreeBadSuperBlk, %function
FtlGcFreeBadSuperBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	movw	r3, #3206
	ldr	r4, .L935
	mov	r9, r0
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	movne	r6, #0
	movne	sl, r4
	bne	.L926
	b	.L927
.L933:
	add	r3, r4, r6
	mov	r1, r9
	mov	r5, #0
	movw	r7, #3206
	ldrb	r0, [r3, #56]	@ zero_extendqisi2
	bl	V2P_block
	mov	r8, r0
	b	.L928
.L932:
	add	r3, r4, r5, asl #1
	add	r3, r3, #3200
	add	r3, r3, #8
	ldrh	r3, [r3, #0]
	cmp	r3, r8
	bne	.L929
	mov	r1, r8
	ldr	r0, .L935+4
	bl	printk
	mov	r0, r8
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	r1, [r4, r7]
	mov	r3, r5
	b	.L930
.L931:
	add	r0, r3, #1
	add	r3, r4, r3, asl #1
	add	r3, r3, #3200
	add	r2, r4, r0, asl #1
	add	r2, r2, #3200
	add	r2, r2, #8
	ldrh	r2, [r2, #0]
	strh	r2, [r3, #8]	@ movhi
	uxth	r3, r0
.L930:
	cmp	r3, r1
	bcc	.L931
	sub	r1, r1, #1
	strh	r1, [sl, r7]	@ movhi
.L929:
	add	r5, r5, #1
	uxth	r5, r5
.L928:
	ldrh	r3, [r4, r7]
	cmp	r3, r5
	bhi	.L932
	add	r6, r6, #1
	uxth	r6, r6
.L926:
	ldrh	r3, [r4, #28]
	cmp	r3, r6
	bhi	.L933
	bl	FtlGcReFreshBadBlk
.L927:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L936:
	.align	2
.L935:
	.word	.LANCHOR0
	.word	.LC104
	.fnend
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	2
	.type	update_vpc_list.part.10, %function
update_vpc_list.part.10:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldr	r4, .L940
	mov	r1, r6
	ldr	r0, .L940+4
	movw	r5, #2332
	bl	List_remove_node
	ldrh	r3, [r4, r5]
	cmp	r3, #0
	bne	.L938
	ldr	r0, .L940+8
	movw	r2, #2774
	ldr	r1, .L940+12
	bl	printk
.L938:
	ldrh	r3, [r4, r5]
	mov	r0, r6
	sub	r3, r3, #1
	strh	r3, [r4, r5]	@ movhi
	bl	free_data_superblock
	mov	r0, r6
	bl	FtlGcFreeBadSuperBlk
	movw	r3, #2340
	ldrh	r2, [r4, r3]
	ldrh	r3, [r4, r5]
	add	r2, r2, r3
	ldrh	r3, [r4, #36]
	cmp	r2, r3
	ble	.L939
	ldr	r0, .L940+8
	movw	r2, #2777
	ldr	r1, .L940+12
	bl	printk
.L939:
	mov	r0, #1
	ldmfd	sp!, {r4, r5, r6, pc}
.L941:
	.align	2
.L940:
	.word	.LANCHOR0
	.word	.LANCHOR0+2316
	.word	.LC1
	.word	.LANCHOR1+418
	.fnend
	.size	update_vpc_list.part.10, .-update_vpc_list.part.10
	.align	2
	.global	update_vpc_list
	.type	update_vpc_list, %function
update_vpc_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L950
	mov	r1, r0, asl #1
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	mov	r3, r0
	ldr	ip, [r2, #2320]
	ldrh	r1, [ip, r1]
	cmp	r1, #0
	bne	.L943
	movw	r1, #2588
	ldrh	r0, [r2, r1]
	cmp	r0, r3
	mvneq	r0, #0
	streqh	r0, [r2, r1]	@ movhi
	beq	.L945
	movw	r1, #2344
	ldrh	r1, [r2, r1]
	cmp	r1, r3
	beq	.L946
	movw	r1, #2392
	ldrh	r1, [r2, r1]
	cmp	r1, r3
	beq	.L946
	movw	r1, #2440
	ldrh	r2, [r2, r1]
	cmp	r2, r3
	beq	.L946
.L945:
	mov	r0, r3
	ldmfd	sp!, {r3, lr}
	b	update_vpc_list.part.10
.L943:
	bl	List_update_data_list
.L946:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L951:
	.align	2
.L950:
	.word	.LANCHOR0
	.fnend
	.size	update_vpc_list, .-update_vpc_list
	.align	2
	.global	decrement_vpc_count
	.type	decrement_vpc_count, %function
decrement_vpc_count:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	r3, #65535
	cmp	r0, r3
	mov	r7, r0
	beq	.L953
	ldr	r6, .L959
	mov	r5, r0, asl #1
	ldr	r3, [r6, #2320]
	ldrh	r4, [r3, r5]
	cmp	r4, #0
	subne	r4, r4, #1
	strneh	r4, [r3, r5]	@ movhi
	bne	.L953
	ldr	r0, .L959+4
	mov	r1, r7
	mov	r2, r4
	bl	printk
	ldr	r3, [r6, #2320]
	ldrh	r5, [r3, r5]
	cmp	r5, #0
	bne	.L955
	ldr	r0, .L959+8
	movw	r2, #2792
	ldr	r1, .L959+12
	mov	r4, r5
	bl	printk
	b	.L955
.L953:
	ldr	r5, .L959
	movw	r6, #3386
	movw	r3, #65535
	ldrh	r0, [r5, r6]
	cmp	r0, r3
	streqh	r7, [r5, r6]	@ movhi
	beq	.L958
	cmp	r0, r7
	beq	.L958
	bl	update_vpc_list
	ldr	r2, [r5, #2316]
	ldr	r3, [r5, #2312]
	ldr	r1, [r5, #2320]
	rsb	r3, r3, r2
	ldr	r2, .L959+16
	strh	r7, [r5, r6]	@ movhi
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r3, r3
	mov	r2, r3, asl #1
	ldrh	r2, [r1, r2]
	adds	r4, r0, #0
	movne	r4, #1
	cmp	r2, #0
	bne	.L955
	cmp	r7, r3
	beq	.L955
	ldr	r1, .L959+12
	movw	r2, #2808
	ldr	r0, .L959+8
	bl	printk
	b	.L955
.L958:
	mov	r4, #0
.L955:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L960:
	.align	2
.L959:
	.word	.LANCHOR0
	.word	.LC105
	.word	.LC1
	.word	.LANCHOR1+434
	.word	-1431655765
	.fnend
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	2
	.global	FtlSuperblockPowerLostFix
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #24
	sub	sp, sp, #24
	ldr	r6, .L966
	mvn	r3, #0
	str	r3, [sp, #20]
	mov	r8, #0
	movw	r2, #61589
	mov	r4, r0
	ldr	r5, [r6, #3292]
	mov	r7, #7
	ldr	r3, [r6, #3288]
	str	r5, [sp, #16]
	str	r3, [sp, #12]
	mvn	r3, #2
	str	r3, [r5, #8]
	mvn	r3, #1
	str	r3, [r5, #12]
	ldrh	r3, [r0, #0]
	strh	r8, [r5, #0]	@ movhi
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, [r6, #3288]
	str	r2, [r3, #0]
	ldr	r2, .L966+4
	ldr	r3, [r6, #3288]
	str	r2, [r3, #4]
	b	.L962
.L965:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L963
	mov	r0, r4
	bl	get_new_active_ppa
	cmn	r0, #1
	str	r0, [sp, #8]
	beq	.L963
	ldr	r3, [r6, #2544]
	mov	r2, #0
	add	r0, sp, #4
	mov	r1, #1
	str	r3, [r5, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, r8
	str	r3, [r6, #2544]
	mov	r3, r2
	bl	FlashProgPages
	ldrh	r0, [r4, #0]
	bl	decrement_vpc_count
.L962:
	subs	r7, r7, #1
	bne	.L965
.L963:
	ldr	r3, .L966
	ldrh	r2, [r4, #0]
	ldrh	r0, [r4, #4]
	ldr	r1, [r3, #2320]
	mov	r2, r2, asl #1
	ldrh	ip, [r1, r2]
	rsb	r0, r0, ip
	strh	r0, [r1, r2]	@ movhi
	ldrh	r3, [r3, #98]
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
	add	sp, sp, #24
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L967:
	.align	2
.L966:
	.word	.LANCHOR0
	.word	305419896
	.fnend
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	2
	.global	FtlRecoverySuperblock
	.type	FtlRecoverySuperblock, %function
FtlRecoverySuperblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r3, #65535
	ldrh	r2, [r0, #0]
	.pad #60
	sub	sp, sp, #60
	mov	r4, r0
	cmp	r2, r3
	beq	.L969
	ldr	r2, .L1038
	ldrb	r5, [r0, #6]	@ zero_extendqisi2
	ldrh	r7, [r0, #2]
	ldrh	r2, [r2, #98]
	str	r5, [sp, #16]
	mov	r5, #0
	cmp	r2, r7
	streqh	r5, [r0, #4]	@ movhi
	moveq	r3, r5	@ movhi
	ldrneh	r0, [r0, #16]
	bne	.L971
	b	.L1036
.L972:
	add	r5, r5, #1
	uxth	r5, r5
	add	r2, r4, r5, asl #1
	ldrh	r0, [r2, #16]
.L971:
	cmp	r0, r3
	beq	.L972
	mov	r1, #1
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r9, r0
	beq	.L973
	ldr	r3, .L1038
	mov	r5, #0
	str	r4, [sp, #32]
	mov	r2, r4
	ldrh	sl, [r3, #28]
	ldr	r8, [r3, #3416]
	ldr	r6, [r3, #3172]
	ldrh	lr, [r3, #106]
	ldr	ip, [r3, #3176]
	ldrh	r3, [r3, #108]
	str	r3, [sp, #12]
	mov	r3, r5
	b	.L974
.L973:
	cmp	r7, #0
	beq	.L975
	ldr	r0, .L1038+4
	movw	r2, #1766
	ldr	r1, .L1038+8
	bl	printk
.L975:
	ldr	fp, [sp, #16]
	cmp	fp, r5
	cmpne	fp, #0
	beq	.L976
	ldr	r0, .L1038+4
	movw	r2, #1767
	ldr	r1, .L1038+8
	bl	printk
.L976:
	mov	r3, #0
	strh	r3, [r4, #2]	@ movhi
	strb	r3, [r4, #6]
	b	.L969
.L978:
	ldrh	r0, [r2, #16]
	movw	r1, #65535
	cmp	r0, r1
	beq	.L977
	mov	fp, #20
	orr	r0, r9, r0, asl #10
	mla	r1, fp, r5, r8
	str	r0, [r1, #4]
	mul	r0, lr, r5
	add	fp, r0, #3
	cmp	r0, #0
	movlt	r0, fp
	ldr	fp, [sp, #12]
	bic	r0, r0, #3
	add	r0, r6, r0
	str	r0, [r1, #8]
	mul	r0, fp, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	fp, r0, #3
	cmp	r0, #0
	movlt	r0, fp
	bic	r0, r0, #3
	add	r0, ip, r0
	str	r0, [r1, #12]
.L977:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L974:
	cmp	r3, sl
	bne	.L978
	ldr	r6, .L1038
	mov	r2, #0
	mov	r1, r5
	mov	fp, r7
	mov	r8, #0
	ldr	r0, [r6, #3416]
	bl	FlashReadPages
	ldr	ip, [r6, #2544]
	movw	r2, #65535
	str	r2, [sp, #8]
	uxth	r2, r9
	ldr	sl, [r6, #3416]
	str	ip, [sp, #20]
	mov	r3, r6
	mov	r7, r2
	b	.L979
.L984:
	ldr	r2, [sl, #0]
	cmp	r2, #0
	strne	r7, [sp, #8]
	bne	.L982
	ldr	r2, [sl, #12]
	ldr	r6, [r2, #4]
	cmn	r6, #1
	beq	.L981
	ldr	r1, [r3, #2544]
	mov	r0, r6
	str	r2, [sp, #4]
	str	r3, [sp, #0]
	bl	ftl_cmp_data_ver
	ldr	r3, [sp, #0]
	ldr	r2, [sp, #4]
	cmp	r0, #0
	addne	r6, r6, #1
	strne	r6, [r3, #2544]
.L981:
	ldr	r2, [r2, #0]
	cmn	r2, #1
	bne	.L982
	ldr	r3, .L1038
	mov	r2, #20
	mov	r7, fp
	uxth	fp, r9
	ldr	r3, [r3, #3416]
	mla	r8, r2, r8, r3
	ldr	r0, [r8, #4]
	b	.L1033
.L982:
	add	r8, r8, #1
	add	sl, sl, #20
	uxth	r8, r8
.L979:
	cmp	r8, r5
	bne	.L984
	ldr	r3, .L1038
	mov	r7, fp
	add	fp, r9, #1
	ldr	r3, [r3, #3416]
	uxth	fp, fp
	ldr	r0, [r3, #4]
.L1033:
	ubfx	r0, r0, #10, #16
	bl	P2V_plane
	ldr	r3, .L1038
	ldr	r5, [sp, #16]
	ldrh	r3, [r3, #98]
	cmp	r3, fp
	streqh	fp, [r4, #2]	@ movhi
	moveq	r3, #0
	streqb	r3, [r4, #6]
	streqh	r3, [r4, #4]	@ movhi
	str	r0, [sp, #12]
	ldr	r3, [sp, #12]
	cmp	r3, r5
	cmpeq	fp, r7
	moveq	r0, r4
	moveq	r1, fp
	moveq	r2, r3
	beq	.L1037
	ldr	r2, [sp, #8]
	movw	r3, #65535
	ldr	ip, [sp, #20]
	cmp	r2, r3
	sub	r6, ip, #1
	bne	.L987
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L988
.L987:
	ldr	r3, .L1038
	uxth	r9, r9
	ldr	sl, .L1038
	ldr	r2, [r3, #3396]
	cmn	r2, #1
	streq	r6, [r3, #3396]
	add	r3, r7, #7
	cmp	r9, r3
	movw	r3, #65535
	subgt	r8, r9, #7
	movle	r8, r7
	uxthgt	r8, r8
	b	.L1031
.L993:
	ldrh	r0, [r1, #16]
	cmp	r0, r3
	beq	.L992
	ldr	ip, [sp, #8]
	mov	r4, #20
	orr	r0, r8, r0, asl #10
	mla	lr, r4, r5, ip
	add	r5, r5, #1
	uxth	r5, r5
	str	r0, [lr, #4]
.L992:
	add	r2, r2, #1
	add	r1, r1, #2
	uxth	r2, r2
.L998:
	ldr	r0, [sp, #20]
	cmp	r2, r0
	bne	.L993
	ldr	r0, [sl, #3416]
	mov	r1, r5
	mov	r2, #0
	str	r3, [sp, #0]
	ldr	r4, [sp, #24]
	bl	FlashReadPages
	ldr	r2, [sl, #3416]
	mov	r1, #0
	ldr	r3, [sp, #0]
	b	.L994
.L996:
	ldr	r0, [r2, #0]
	cmp	r0, #0
	bne	.L988
	ldr	r0, [r2, #12]
	ldrh	ip, [r0, #0]
	cmp	ip, r3
	beq	.L995
	ldr	r0, [r0, #4]
	cmn	r0, #1
	strne	r0, [sl, #3396]
.L995:
	add	r1, r1, #1
	add	r2, r2, #20
	uxth	r1, r1
.L994:
	cmp	r1, r5
	bne	.L996
	add	r8, r8, #1
	uxth	r8, r8
.L1031:
	cmp	r8, r9
	bhi	.L997
	ldr	r1, [sl, #3416]
	mov	r5, #0
	ldrh	ip, [sl, #28]
	mov	r2, r5
	str	r4, [sp, #24]
	str	r1, [sp, #8]
	mov	r1, r4
	str	ip, [sp, #20]
	b	.L998
.L997:
	ldr	r3, .L1038
	mvn	r2, #0
	str	r2, [r3, #3396]
.L988:
	ldr	r2, .L1038+12
	movw	r3, #3522
	mov	r9, r7
	mov	r1, #1
	str	fp, [sp, #24]
	strh	r1, [r2, r3]	@ movhi
	str	r7, [sp, #28]
.L999:
	ldr	r3, .L1038
	mov	r7, #0
	mov	r2, r4
	mov	r0, #20
	ldrh	lr, [r3, #28]
	ldr	ip, [r3, #3416]
	mov	r3, r7
	b	.L1000
.L1002:
	ldrh	r1, [r2, #16]
	movw	r5, #65535
	add	r3, r3, #1
	add	r2, r2, #2
	cmp	r1, r5
	uxth	r3, r3
	orrne	r1, r9, r1, asl #10
	mlane	r5, r0, r7, ip
	addne	r7, r7, #1
	uxthne	r7, r7
	strne	r1, [r5, #4]
.L1000:
	cmp	r3, lr
	bne	.L1002
	ldr	r5, .L1038
	mov	r1, r7
	mov	r2, #0
	ldr	r0, [r5, #3416]
	bl	FlashReadPages
	mov	r3, #20
	mul	r3, r3, r7
	str	r4, [sp, #8]
	mov	r7, #0
	str	r3, [sp, #36]
	b	.L1003
.L1025:
	ldr	r8, [r5, #3416]
	add	r8, r8, r7
	ldr	r4, [r8, #4]
	ubfx	r0, r4, #10, #16
	str	r4, [sp, #52]
	bl	P2V_plane
	ldr	ip, [sp, #28]
	cmp	r9, ip
	bcc	.L1004
	ldr	fp, [sp, #16]
	cmp	r0, fp
	movcs	r3, #0
	movcc	r3, #1
	cmp	r9, ip
	movne	r3, #0
	cmp	r3, #0
	bne	.L1004
	ldr	ip, [sp, #12]
	ldr	r3, [sp, #24]
	cmp	r0, ip
	cmpeq	r9, r3
	beq	.L1005
	ldr	r3, [r8, #0]
	cmn	r3, #1
	beq	.L1006
	ldr	r8, [r8, #12]
	movw	r4, #61589
	ldrh	r3, [r8, #0]
	cmp	r3, r4
	ldrne	ip, [sp, #8]
	ldrneh	r0, [ip, #0]
	bne	.L1034
	ldr	r6, [r8, #4]
	cmn	r6, #1
	beq	.L1008
	ldr	r1, [r5, #2544]
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	addne	r3, r6, #1
	strne	r3, [r5, #2544]
.L1008:
	ldrh	r3, [r8, #0]
	movw	r0, #61589
	cmp	r3, r0
	beq	.L1009
	ldr	r0, .L1038+4
	movw	r2, #1918
	ldr	r1, .L1038+8
	bl	printk
.L1009:
	ldr	r4, [r8, #8]
	add	r1, sp, #48
	ldr	r3, [r8, #12]
	mov	r2, #0
	mov	r0, r4
	str	r3, [sp, #44]
	bl	log2phys
	ldr	r1, [r5, #3396]
	cmn	r1, #1
	beq	.L1010
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1010
	ldr	r3, [sp, #44]
	cmn	r3, #1
	beq	.L1011
	ldr	r0, [r5, #3416]
	mov	r1, #1
	mov	r2, #0
	add	r0, r0, r7
	ldr	sl, [r0, #12]
	str	r3, [r0, #4]
	bl	FlashReadPages
	ldr	r3, [sl, #4]
	str	r3, [sp, #20]
	ldr	r3, [r5, #3416]
	add	fp, r3, r7
	ldr	r3, [r3, r7]
	cmn	r3, #1
	bne	.L1012
	b	.L1013
.L1011:
	ldr	r3, [sp, #52]
	ldr	r2, [sp, #48]
	cmp	r2, r3
	bne	.L1004
	mov	r0, r4
	add	r1, sp, #44
	mov	r2, #1
	bl	log2phys
	b	.L1004
.L1012:
	ldr	r8, [sl, #8]
	cmp	r8, r4
	bne	.L1013
	ldr	r0, [r5, #3396]
	ldr	r1, [sp, #20]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1013
	ldr	r3, [sp, #48]
	ldr	r2, [sp, #52]
	cmp	r3, r2
	beq	.L1017
.L1014:
	ldr	r2, [sp, #44]
	cmp	r3, r2
	beq	.L1013
	cmn	r3, #1
	streq	r3, [fp, #0]
	beq	.L1016
	str	r3, [fp, #4]
	mov	r0, fp
	mov	r1, #1
	mov	r2, #0
	ldr	sl, [fp, #12]
	bl	FlashReadPages
.L1016:
	ldr	r3, [r5, #3416]
	ldr	r3, [r3, r7]
	cmn	r3, #1
	beq	.L1017
	ldr	r4, [sl, #4]
	ldr	r0, [r5, #3396]
	mov	r1, r4
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1017
	ldr	r0, [sp, #20]
	mov	r1, r4
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1013
.L1017:
	mov	r0, r8
	ldr	r1, [sp, #44]
	bl	FtlReUsePrevPpa
.L1013:
	mvn	r3, #0
	str	r3, [sp, #44]
	b	.L1019
.L1010:
	ldr	r3, [sp, #52]
	ldr	r2, [sp, #48]
	cmp	r2, r3
	beq	.L1019
	mov	r0, r4
	add	r1, sp, #52
	mov	r2, #1
	bl	log2phys
	ldr	fp, [sp, #48]
	cmn	fp, #1
	beq	.L1019
	ldr	r3, [sp, #44]
	cmp	fp, r3
	beq	.L1019
	ubfx	r0, fp, #10, #16
	ldr	sl, .L1038
	bl	P2V_block_in_plane
	movw	r3, #2344
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L1020
	movw	r3, #2392
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L1020
	movw	r3, #2440
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	bne	.L1019
.L1020:
	ldr	r0, [sl, #3416]
	mov	r1, #1
	mov	r2, #0
	str	fp, [r0, #4]
	ldr	r8, [r0, #12]
	bl	FlashReadPages
	ldr	r3, [sl, #3416]
	ldr	r1, [r8, #4]
	ldr	r3, [r3, #0]
	cmn	r3, #1
	beq	.L1019
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	bne	.L1019
	mov	r0, r4
	add	r1, sp, #48
	mov	r2, #1
	bl	log2phys
.L1019:
	ldr	r0, [sp, #44]
	cmn	r0, #1
	beq	.L1004
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r5, #36]
	cmp	r3, r0
	mov	r4, r0
	bhi	.L1021
	ldr	r0, .L1038+4
	movw	r2, #2019
	ldr	r1, .L1038+8
	bl	printk
.L1021:
	ldr	r3, .L1038
	ldr	r2, [r3, #2320]
	mov	r3, r4, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1022
	mov	r0, r4
.L1034:
	bl	decrement_vpc_count
	b	.L1004
.L1022:
	mov	r1, r4
	ldr	r0, .L1038+16
	bl	printk
	b	.L1004
.L1006:
	ldr	r3, .L1038+12
	ldr	r2, [r3, #3524]
	cmp	r2, #31
	addls	r1, r3, r2, asl #2
	addls	r2, r2, #1
	strls	r2, [r3, #3524]
	strls	r4, [r1, #3528]
	ldr	r4, [sp, #8]
	ldrh	r0, [r4, #0]
	bl	decrement_vpc_count
	ldr	r3, [r5, #3396]
	cmn	r3, #1
	beq	.L1035
.L1024:
	cmp	r3, r6
	bls	.L1004
.L1035:
	str	r6, [r5, #3396]
.L1004:
	add	r7, r7, #20
.L1003:
	ldr	fp, [sp, #36]
	ldr	r3, .L1038
	cmp	r7, fp
	bne	.L1025
	add	r9, r9, #1
	ldrh	r2, [r3, #98]
	ldr	r4, [sp, #8]
	uxth	r9, r9
	cmp	r9, r2
	bne	.L999
	mov	r2, #0
	ldrh	r1, [r3, #28]
	strh	r2, [r4, #4]	@ movhi
	mov	r3, r2
	strh	r9, [r4, #2]	@ movhi
	movw	r2, #65535
	ldr	r0, [sp, #32]
	b	.L1026
.L1028:
	add	r0, r0, #2
	ldrh	ip, [r0, #14]
	cmp	ip, r2
	beq	.L1027
.L1036:
	strb	r3, [r4, #6]
	b	.L969
.L1027:
	add	r3, r3, #1
	uxth	r3, r3
.L1026:
	cmp	r3, r1
	bne	.L1028
	b	.L969
.L1005:
	ldr	r4, [sp, #8]
	ldr	fp, [sp, #24]
	ldr	ip, [sp, #12]
	mov	r0, r4
	mov	r1, fp
	strh	fp, [r4, #2]	@ movhi
	mov	r2, ip
	strb	ip, [r4, #6]
.L1037:
	bl	ftl_sb_update_avl_pages
.L969:
	mov	r0, #0
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1039:
	.align	2
.L1038:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+454
	.word	.LANCHOR3
	.word	.LC106
	.fnend
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
	.align	2
	.global	allocate_data_superblock
	.type	allocate_data_superblock, %function
allocate_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r4, .L1073
	mov	r5, r0
	movw	fp, #2340
	mov	r6, r4
.L1070:
	movw	r3, #2332
	ldrh	r2, [r4, fp]
	ldrh	r3, [r4, r3]
	add	r2, r2, r3
	ldrh	r3, [r4, #36]
	cmp	r2, r3
	ble	.L1042
	ldr	r0, .L1073+4
	movw	r2, #2612
	ldr	r1, .L1073+8
	bl	printk
.L1042:
	ldr	r3, .L1073+12
	cmp	r5, r3
	movne	r1, #0
	bne	.L1043
	ldrh	r3, [r4, fp]
	ldr	r1, [r4, #3160]
	mov	r2, r3, lsr #1
	mul	r1, r1, r3
	add	r2, r2, #1
	add	r1, r2, r1, lsr #2
	uxth	r1, r1
	cmp	r1, #0
	subne	r1, r1, #1
	uxthne	r1, r1
.L1043:
	ldr	r0, .L1073+16
	movw	r8, #2340
	bl	List_pop_index_node
	ldrh	r3, [r4, fp]
	cmp	r3, #0
	uxth	r7, r0
	bne	.L1044
	ldr	r0, .L1073+4
	movw	r2, #2621
	ldr	r1, .L1073+8
	bl	printk
.L1044:
	ldrh	r3, [r6, r8]
	sub	r3, r3, #1
	strh	r3, [r6, r8]	@ movhi
	ldrh	r3, [r6, #36]
	cmp	r3, r7
	bls	.L1070
	ldr	r3, [r4, #2320]
	mov	r8, r7, asl #1
	ldrh	r3, [r3, r8]
	cmp	r3, #0
	bne	.L1070
	strh	r7, [r5, #0]	@ movhi
	mov	r0, r5
	bl	make_superblock
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	movw	sl, #2332
	cmp	r3, #0
	bne	.L1046
	ldr	r3, [r4, #2320]
	mvn	r2, #0
	mov	r0, r7
	strh	r2, [r3, r8]	@ movhi
	bl	INSERT_DATA_LIST
	ldrh	r3, [r4, sl]
	ldrh	r2, [r4, fp]
	add	r2, r2, r3
	ldrh	r3, [r4, #36]
	cmp	r2, r3
	ble	.L1070
	ldr	r1, .L1073+8
	movw	r2, #2635
	ldr	r0, .L1073+4
	bl	printk
	b	.L1070
.L1046:
	ldrh	r3, [r4, sl]
	ldrh	r2, [r4, fp]
	add	r2, r2, r3
	ldrh	r3, [r4, #36]
	cmp	r2, r3
	ble	.L1047
	ldr	r0, .L1073+4
	movw	r2, #2638
	ldr	r1, .L1073+8
	bl	printk
.L1047:
	ldrh	r3, [r4, #28]
	mov	sl, #0
	ldr	ip, [r4, #3300]
	mov	r1, r5
	str	r5, [sp, #16]
	mov	r2, sl
	str	r3, [sp, #20]
	mov	r0, sl
	mov	r3, ip
	stmib	sp, {r5, ip}
	b	.L1048
.L1050:
	str	r0, [r3, #8]
	movw	r5, #65535
	str	r0, [r3, #12]
	ldrh	lr, [r1, #16]
	cmp	lr, r5
	beq	.L1049
	ldr	r5, [sp, #8]
	mov	ip, #20
	mov	lr, lr, asl #10
	mla	r9, ip, sl, r5
	add	sl, sl, #1
	uxth	sl, sl
	str	lr, [r9, #4]
.L1049:
	add	r2, r2, #1
	add	r3, r3, #20
	add	r1, r1, #2
	uxth	r2, r2
.L1048:
	ldr	r9, [sp, #20]
	cmp	r2, r9
	bne	.L1050
	cmp	sl, #0
	ldr	r5, [sp, #4]
	bne	.L1051
	ldr	r0, .L1073+4
	movw	r2, #2649
	ldr	r1, .L1073+8
	bl	printk
.L1051:
	movw	r3, #2588
	ldrh	r3, [r4, r3]
	cmp	r3, r7
	bne	.L1052
	ldr	r0, .L1073+4
	movw	r2, #2651
	ldr	r1, .L1073+8
	bl	printk
.L1052:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	cmp	r3, #0
	ldr	r3, [r6, #2324]
	ldrh	r2, [r3, r8]
	bne	.L1053
	cmp	r2, #0
	mov	r0, r7
	ldrneh	r1, [r4, #88]
	moveq	r2, #2
	addne	r2, r2, r1
	strh	r2, [r3, r8]	@ movhi
	ldr	r3, [r4, #2548]
	mov	r1, #0
	add	r3, r3, #1
	str	r3, [r4, #2548]
	b	.L1072
.L1053:
	add	r2, r2, #1
	strh	r2, [r3, r8]	@ movhi
	ldr	r3, [r6, #2552]
	mov	r0, r7
	mov	r1, #1
	add	r3, r3, #1
	str	r3, [r6, #2552]
.L1072:
	bl	ftl_set_blk_mode
	ldr	r3, [r4, #2324]
	ldr	r2, [r4, #2564]
	mov	r9, #0
	ldr	r0, [r6, #2548]
	ldrh	r3, [r3, r8]
	ldrh	r1, [r6, #36]
	cmp	r3, r2
	ldrh	r2, [r6, #88]
	strhi	r3, [r4, #2564]
	ldr	r3, [r6, #2552]
	mla	r0, r0, r2, r3
	bl	__aeabi_uidiv
	ldr	r3, [r6, #3440]
	ldr	r2, [r3, #16]
	add	r2, r2, #1
	str	r2, [r3, #16]
	ldr	r3, [r6, #3300]
	str	r0, [r6, #2556]
	b	.L1058
.L1059:
	add	r9, r9, #1
	ldr	r2, [r3, #-16]
	uxth	r9, r9
	bic	r2, r2, #1020
	bic	r2, r2, #3
	str	r2, [r3, #-16]
.L1058:
	cmp	r9, sl
	add	r3, r3, #20
	bne	.L1059
	ldrb	r1, [r5, #8]	@ zero_extendqisi2
	mov	r2, r9
	ldr	r0, [r4, #3300]
	mov	sl, #0
	bl	FlashEraseBlocks
	mov	r2, #20
	mul	r2, r2, r9
	mov	ip, r7
	str	r8, [sp, #20]
	mov	r7, r5
	mov	r5, sl
	mov	r8, r2
	b	.L1060
.L1062:
	ldr	r0, [r4, #3300]
	add	r1, r0, sl
	ldr	r9, [r0, sl]
	cmn	r9, #1
	bne	.L1061
	ldr	r0, [r1, #4]
	add	r5, r5, #1
	str	ip, [sp, #12]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	ldr	r3, [sp, #16]
	strh	r9, [r3, #16]	@ movhi
	ldrb	r1, [r7, #7]	@ zero_extendqisi2
	sub	r1, r1, #1
	strb	r1, [r7, #7]
	ldr	ip, [sp, #12]
.L1061:
	ldr	r9, [sp, #16]
	add	sl, sl, #20
	add	r9, r9, #2
	str	r9, [sp, #16]
.L1060:
	cmp	sl, r8
	ldr	r9, .L1073
	bne	.L1062
	mov	r3, r5
	cmp	r3, #0
	mov	r5, r7
	ldr	r8, [sp, #20]
	mov	r7, ip
	beq	.L1063
	mov	r0, ip
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L1063:
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, [r6, #2320]
	mvneq	r2, #0
	streqh	r2, [r3, r8]	@ movhi
	beq	.L1070
.L1064:
	ldrh	r2, [r9, #98]
	strh	r7, [r5, #0]	@ movhi
	mul	r3, r2, r3
	mov	r2, #0
	strh	r2, [r5, #2]	@ movhi
	strb	r2, [r5, #6]
	ldr	r2, [r9, #2540]
	ldr	r1, [r9, #2320]
	uxth	r3, r3
	strh	r3, [r5, #4]	@ movhi
	str	r2, [r5, #12]
	add	r2, r2, #1
	str	r2, [r9, #2540]
	ldrh	r2, [r5, #0]
	mov	r2, r2, asl #1
	strh	r3, [r1, r2]	@ movhi
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	beq	.L1065
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1066
.L1065:
	ldr	r0, .L1073+4
	mov	r2, #2704
	ldr	r1, .L1073+8
	bl	printk
.L1066:
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1074:
	.align	2
.L1073:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+476
	.word	.LANCHOR0+2440
	.word	.LANCHOR0+2336
	.fnend
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlWriteDump_data
	.type	FtlWriteDump_data, %function
FtlWriteDump_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	movw	r3, #2348
	ldr	r5, .L1084
	.pad #28
	sub	sp, sp, #28
	ldrh	r3, [r5, r3]
	mov	r2, r5
	cmp	r3, #0
	beq	.L1076
	ldrb	r1, [r5, #2352]	@ zero_extendqisi2
	cmp	r1, #0
	bne	.L1076
	ldrb	r1, [r5, #2351]	@ zero_extendqisi2
	ldrh	r0, [r5, #98]
	mul	r1, r0, r1
	cmp	r3, r1
	beq	.L1076
	ldrb	r8, [r5, #2354]	@ zero_extendqisi2
	ldr	r7, [r5, #2504]
	cmp	r8, #0
	ldrh	r6, [r5, #28]
	bne	.L1075
	sub	r7, r7, #1
	mov	r1, sp
	mov	r2, r8
	mov	r0, r7
	bl	log2phys
	ldr	r3, [sp, #0]
	ldr	r4, [r5, #3292]
	ldr	r0, [r5, #3288]
	cmn	r3, #1
	str	r3, [sp, #8]
	str	r7, [sp, #20]
	str	r0, [sp, #12]
	str	r4, [sp, #16]
	str	r8, [r4, #4]
	beq	.L1078
	add	r0, sp, #4
	mov	r1, #1
	mov	r2, r8
	bl	FlashReadPages
	b	.L1079
.L1078:
	mov	r1, #255
	ldrh	r2, [r5, #106]
	bl	ftl_memset
.L1079:
	ldr	r3, .L1084+4
	mov	r6, r6, asl #2
	ldr	r5, .L1084
	movw	sl, #2348
	movw	r8, #2344
	strh	r3, [r4, #0]	@ movhi
	b	.L1080
.L1083:
	ldrh	r3, [r5, sl]
	cmp	r3, #0
	beq	.L1081
	ldr	r3, [sp, #8]
	sub	r6, r6, #1
	str	r7, [r4, #8]
	ldr	r0, .L1084+8
	str	r3, [r4, #12]
	ldrh	r3, [r5, r8]
	strh	r3, [r4, #2]	@ movhi
	bl	get_new_active_ppa
	ldr	r3, [r5, #2544]
	mov	r2, #0
	mov	r1, #1
	str	r0, [sp, #8]
	add	r0, sp, #4
	str	r3, [r4, #4]
	add	r3, r3, #1
	cmn	r3, #1
	str	r3, [r5, #2544]
	moveq	r3, #0
	streq	r3, [r5, #2544]
	mov	r3, r2
	bl	FlashProgPages
	ldrh	r0, [r5, r8]
	bl	decrement_vpc_count
.L1080:
	cmp	r6, #0
	bne	.L1083
.L1081:
	ldr	r3, .L1084
	mov	r2, #1
	strb	r2, [r3, #2354]
	b	.L1075
.L1076:
	mov	r3, #0
	strb	r3, [r2, #2354]
.L1075:
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L1085:
	.align	2
.L1084:
	.word	.LANCHOR0
	.word	-3947
	.word	.LANCHOR0+2344
	.fnend
	.size	FtlWriteDump_data, .-FtlWriteDump_data
	.align	2
	.global	l2p_flush
	.type	l2p_flush, %function
l2p_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	bl	FtlWriteDump_data
	mov	r4, #0
	ldr	r5, .L1090
	mov	r6, #12
	b	.L1087
.L1089:
	ldr	r3, [r5, #2488]
	mla	r3, r6, r4, r3
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L1088
	mov	r0, r4
	bl	flush_l2p_region
.L1088:
	add	r4, r4, #1
	uxth	r4, r4
.L1087:
	ldrh	r3, [r5, #134]
	cmp	r3, r4
	bhi	.L1089
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1091:
	.align	2
.L1090:
	.word	.LANCHOR0
	.fnend
	.size	l2p_flush, .-l2p_flush
	.align	2
	.global	FtlSysFlush
	.type	FtlSysFlush, %function
FtlSysFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	l2p_flush
	mov	r0, #1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
	.fnend
	.size	FtlSysFlush, .-FtlSysFlush
	.align	2
	.global	sftl_deinit
	.type	sftl_deinit, %function
sftl_deinit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L1095
	ldr	r3, [r3, #0]
	cmp	r3, #1
	bne	.L1094
	bl	FtlSysFlush
.L1094:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1096:
	.align	2
.L1095:
	.word	.LANCHOR2
	.fnend
	.size	sftl_deinit, .-sftl_deinit
	.align	2
	.global	sftl_discard
	.type	sftl_discard, %function
sftl_discard:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1106
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, lr}
	mov	r4, r1
	ldr	r2, [r3, #136]
	add	r1, r1, r0
	mov	r7, r0
	cmp	r1, r2
	mvnhi	r0, #0
	bhi	.L1098
	cmp	r4, #31
	movls	r0, #0
	bls	.L1098
	ldrh	r6, [r3, #54]
	mov	r1, r6
	bl	__aeabi_uidiv
	mls	r7, r0, r6, r7
	mov	r5, r0
	uxth	r7, r7
	cmp	r7, #0
	beq	.L1099
	rsb	r6, r7, r6
	add	r5, r0, #1
	cmp	r6, r4
	movcs	r6, r4
	uxth	r6, r6
	rsb	r4, r6, r4
.L1099:
	mvn	r3, #0
	ldr	r6, .L1106
	str	r3, [sp, #4]
	ldr	r7, .L1106+4
	b	.L1100
.L1102:
	mov	r0, r5
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp, #0]
	cmn	r3, #1
	beq	.L1101
	ldr	r3, [r7, #3656]
	add	r1, sp, #4
	mov	r2, #1
	mov	r0, r5
	add	r3, r3, #1
	str	r3, [r7, #3656]
	ldr	r3, [r6, #2512]
	add	r3, r3, #1
	str	r3, [r6, #2512]
	bl	log2phys
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L1101:
	ldrh	r3, [r6, #54]
	add	r5, r5, #1
	rsb	r4, r3, r4
.L1100:
	ldrh	r3, [r6, #54]
	cmp	r4, r3
	bcs	.L1102
	ldr	r3, .L1106+4
	mov	r4, #0
	ldr	r2, [r3, #3656]
	cmp	r2, #32
	bls	.L1105
	str	r4, [r3, #3656]
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1105:
	mov	r0, r4
.L1098:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, pc}
.L1107:
	.align	2
.L1106:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.fnend
	.size	sftl_discard, .-sftl_discard
	.align	2
	.global	FtlGcFreeTempBlock
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1128
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	movw	r2, #2440
	ldrh	r4, [r3, r2]
	movw	r2, #65535
	ldrh	r1, [r3, #98]
	cmp	r4, r2
	beq	.L1109
	cmp	r0, #0
	beq	.L1110
	ldr	r0, .L1128+4
	ldrh	ip, [r0, #4]
	cmp	ip, r2
	movne	r1, #2
	bne	.L1110
	mov	r2, #0
	strh	r2, [r0, #4]	@ movhi
	movw	r2, #2340
	ldrh	r3, [r3, r2]
	cmp	r3, #17
	movhi	r1, #2
.L1110:
	ldr	r0, .L1128+8
	bl	FtlGcScanTempBlk
	cmn	r0, #1
	str	r0, [sp, #4]
	beq	.L1111
	ldr	r3, .L1128
	mov	r4, r4, asl #1
	ldr	r3, [r3, #2324]
	ldrh	r2, [r3, r4]
	cmp	r2, #4
	bls	.L1112
	sub	r2, r2, #5
	mov	r0, #1
	strh	r2, [r3, r4]	@ movhi
	bl	FtlEctTblFlush
.L1112:
	ldr	r3, .L1128
	ldr	r2, [r3, #3392]
	cmp	r2, #0
	bne	.L1113
	ldr	r2, [r3, #2732]
	ldr	r0, [sp, #4]
	add	r2, r2, #1
	str	r2, [r3, #2732]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L1113:
	ldr	r3, .L1128
	mov	r2, #0
	str	r2, [r3, #3392]
	b	.L1124
.L1111:
	ldr	r3, .L1128+4
	ldrh	r2, [r3, #4]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1124
.L1109:
	ldr	r4, .L1128
	movw	r3, #2440
	mov	r5, #0
	ldrh	r2, [r4, r3]
	movw	r3, #65535
	str	r5, [r4, #3392]
	cmp	r2, r3
	beq	.L1114
	mov	r3, #3200
	ldrh	r1, [r4, #98]
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #2447]	@ zero_extendqisi2
	mul	r3, r1, r3
	cmp	r2, r3
	beq	.L1115
	ldr	r0, .L1128+12
	mov	r2, #162
	ldr	r1, .L1128+16
	bl	printk
.L1115:
	ldrb	r1, [r4, #2447]	@ zero_extendqisi2
	movw	sl, #2440
	ldrh	r0, [r4, #98]
	mov	r6, #0
	ldrh	r3, [r4, sl]
	ldr	r2, [r4, #2320]
	mul	r1, r0, r1
	mov	r3, r3, asl #1
	ldr	r5, .L1128
	strh	r1, [r2, r3]	@ movhi
	mov	r3, #3200
	ldrh	r2, [r4, r3]
	ldr	r3, [r4, #2528]
	add	r3, r2, r3
	str	r3, [r4, #2528]
	b	.L1116
.L1120:
	mov	r7, #12
	ldr	r8, [r5, #3196]
	mul	r7, r7, r6
	ldr	r3, [r5, #2504]
	add	r4, r8, r7
	ldr	r2, [r4, #8]
	cmp	r2, r3
	bcc	.L1117
	ldr	r0, .L1128+12
	mov	r2, #168
	ldr	r1, .L1128+16
	bl	printk
.L1117:
	ldr	r0, [r4, #8]
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	ldr	r3, [r8, r7]
	cmp	r0, r3
	bne	.L1118
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r1, r4, #4
	mov	r2, #1
	mov	r7, r0
	ldr	r0, [r4, #8]
	bl	log2phys
	mov	r0, r7
	b	.L1127
.L1118:
	ldr	r3, [r4, #4]
	cmp	r0, r3
	beq	.L1119
	ldrh	r0, [r5, sl]
.L1127:
	bl	decrement_vpc_count
.L1119:
	add	r6, r6, #1
	uxth	r6, r6
.L1116:
	mov	r3, #3200
	ldr	r4, .L1128
	ldrh	r3, [r5, r3]
	cmp	r3, r6
	bhi	.L1120
	movw	r0, #65535
	movw	r5, #2440
	bl	decrement_vpc_count
	ldrh	r0, [r4, r5]
	ldr	r2, [r4, #2320]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1121
	bl	INSERT_DATA_LIST
	b	.L1122
.L1121:
	bl	INSERT_FREE_LIST
.L1122:
	ldr	r4, .L1128
	mov	r3, #3200
	mvn	r6, #0
	strh	r6, [r4, r5]	@ movhi
	mov	r5, #0
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3188
	strh	r5, [r4, r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	movw	r3, #2588
	strh	r6, [r4, r3]	@ movhi
	movw	r3, #2340
	ldrh	r2, [r4, r3]
	add	r3, r3, #232
	ldrh	r3, [r4, r3]
	add	r3, r3, r3, asl #1
	cmp	r2, r3, lsr #2
	movwgt	r3, #3148
	movgt	r2, #20
	strgth	r2, [r4, r3]	@ movhi
	b	.L1114
.L1124:
	mov	r5, #1
.L1114:
	mov	r0, r5
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L1129:
	.align	2
.L1128:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2440
	.word	.LC1
	.word	.LANCHOR1+501
	.fnend
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	2
	.global	Ftl_gc_temp_data_write_back
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r2, #0
	ldr	r4, .L1136
	mov	r3, r2
	mov	r5, #0
	mov	r6, #20
	ldr	r0, [r4, #3420]
	ldr	r1, [r4, #3164]
	bl	FlashProgPages
	b	.L1131
.L1134:
	mul	r3, r6, r5
	ldr	r1, [r4, #3420]
	add	r2, r1, r3
	ldr	r1, [r1, r3]
	cmn	r1, #1
	bne	.L1132
	movw	r2, #2440
	ldr	ip, [r7, #2320]
	ldrh	r0, [r7, r2]
	mov	lr, #0
	mov	r0, r0, asl #1
	strh	lr, [ip, r0]	@ movhi
	strh	r1, [r7, r2]	@ movhi
	ldr	r2, [r7, #2732]
	add	r2, r2, #1
	str	r2, [r7, #2732]
	ldr	r2, [r7, #3420]
	add	r3, r2, r3
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	bl	FtlGcPageVarInit
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1132:
	ldr	r3, [r2, #12]
	add	r5, r5, #1
	ldr	r1, [r2, #4]
	uxth	r5, r5
	ldr	r0, [r3, #12]
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
.L1131:
	ldr	r1, [r4, #3164]
	ldr	r7, .L1136
	cmp	r5, r1
	bcc	.L1134
	ldr	r0, [r7, #3420]
	bl	FtlGcBufFree
	movw	r3, #2444
	ldrh	r3, [r7, r3]
	mov	r0, #0
	str	r0, [r7, #3164]
	cmp	r3, r0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1137:
	.align	2
.L1136:
	.word	.LANCHOR0
	.fnend
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	2
	.global	FtlGcPageRecovery
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, .L1140
	ldr	r0, .L1140+4
	ldrh	r1, [r4, #98]
	bl	FtlGcScanTempBlk
	movw	r3, #2442
	ldrh	r2, [r4, r3]
	ldrh	r3, [r4, #98]
	cmp	r2, r3
	ldmccfd	sp!, {r4, pc}
	add	r0, r4, #3328
	bl	FtlMapBlkWriteDump_data
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	mov	r3, #0
	str	r3, [r4, #3392]
	ldmfd	sp!, {r4, pc}
.L1141:
	.align	2
.L1140:
	.word	.LANCHOR0
	.word	.LANCHOR0+2440
	.fnend
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.align	2
	.global	FtlPowerLostRecovery
	.type	FtlPowerLostRecovery, %function
FtlPowerLostRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, #0
	ldr	r3, .L1143
	ldr	r5, .L1143+4
	str	r4, [r3, #3524]
	mov	r0, r5
	bl	FtlRecoverySuperblock
	mov	r0, r5
	add	r5, r5, #48
	bl	FtlSlcSuperblockCheck
	mov	r0, r5
	bl	FtlRecoverySuperblock
	mov	r0, r5
	bl	FtlSlcSuperblockCheck
	bl	FtlGcPageRecovery
	movw	r0, #65535
	bl	decrement_vpc_count
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L1144:
	.align	2
.L1143:
	.word	.LANCHOR3
	.word	.LANCHOR0+2344
	.fnend
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	2
	.global	Ftl_get_new_temp_ppa
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movw	r3, #2440
	ldr	r5, .L1148
	ldrh	r2, [r5, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1146
	movw	r3, #2444
	ldrh	r3, [r5, r3]
	cmp	r3, #0
	bne	.L1147
.L1146:
	mov	r0, #0
	mov	r4, #0
	bl	FtlGcFreeTempBlock
	ldr	r0, .L1148+4
	strb	r4, [r5, #2448]
	bl	allocate_data_superblock
	movw	r3, #3188
	strh	r4, [r5, r3]	@ movhi
	mov	r3, #3200
	strh	r4, [r5, r3]	@ movhi
	bl	l2p_flush
	mov	r0, r4
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L1147:
	ldr	r0, .L1148+4
	ldmfd	sp!, {r3, r4, r5, lr}
	b	get_new_active_ppa
.L1149:
	.align	2
.L1148:
	.word	.LANCHOR0
	.word	.LANCHOR0+2440
	.fnend
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.type	rk_ftl_garbage_collect.part.11, %function
rk_ftl_garbage_collect.part.11:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r3, #3204
	ldr	r4, .L1231
	.pad #36
	sub	sp, sp, #36
	str	r0, [sp, #12]
	movw	r0, #65535
	ldrh	r1, [r4, r3]
	cmp	r1, r0
	beq	.L1151
	movw	r2, #3202
	ldrh	ip, [r4, r2]
	cmp	ip, r0
	streqh	r1, [r4, r2]	@ movhi
	mvneq	r2, #0
	streqh	r2, [r4, r3]	@ movhi
.L1151:
	bl	FtlReadRefresh
	ldr	r1, [r4, #3152]
	ldr	r7, [sp, #12]
	movw	r3, #2588
	ldrh	r2, [r4, r3]
	add	r1, r1, #1
	ldr	r5, .L1231
	add	r1, r1, r7, asl #7
	str	r1, [r4, #3152]
	movw	r4, #65535
	cmp	r2, r4
	bne	.L1152
	movw	r3, #2440
	ldrh	r3, [r5, r3]
	cmp	r3, r2
	movne	r4, r2
	bne	.L1152
	movw	r2, #3202
	ldrh	r6, [r5, r2]
	cmp	r6, r3
	movne	r4, r3
	bne	.L1152
	movw	r8, #2340
	ldrh	r3, [r5, r8]
	cmp	r3, #23
	movhi	r3, #1024
	movls	r3, #5120
	cmp	r1, r3
	movls	r4, r6
	bls	.L1152
	ldr	r2, .L1231+4
	movw	r3, #3660
	mov	r7, #0
	str	r7, [r5, #3152]
	strh	r7, [r2, r3]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, r6
	mov	r4, r0
	mov	sl, r0
	bne	.L1154
	movw	r6, #3150
	ldrh	r2, [r5, r8]
	ldrh	r3, [r5, r6]
	cmp	r2, r3
	movcs	r3, #80
	bcs	.L1227
	mov	r0, #64
	bl	List_get_gc_head_node
	uxth	r0, r0
	cmp	r0, r4
	beq	.L1158
	ldr	r3, [r5, #2320]
	mov	r0, r0, asl #1
	ldrh	r3, [r3, r0]
	cmp	r3, #7
	movhi	r3, #64
	bhi	.L1227
	mov	r0, r7
	bl	List_get_gc_head_node
	mov	r3, #128
	strh	r3, [r5, r6]	@ movhi
	uxth	r4, r0
	cmp	r4, sl
	beq	.L1158
	b	.L1154
.L1227:
	strh	r3, [r5, r6]	@ movhi
	b	.L1158
.L1154:
	ldr	r1, .L1231
	movw	r3, #2340
	mov	r0, r4, asl #1
	ldrh	r2, [r1, r3]
	ldr	ip, [r1, #2324]
	ldr	r3, [r1, #2320]
	ldrh	r3, [r3, r0]
	ldrh	r0, [ip, r0]
	str	r0, [sp, #0]
	movw	r0, #3148
	ldrh	r1, [r1, r0]
	ldr	r0, .L1231+8
	str	r1, [sp, #4]
	mov	r1, r4
	bl	printk
.L1158:
	bl	FtlGcReFreshBadBlk
.L1152:
	movw	ip, #65535
	rsb	r1, ip, r4
	rsbs	r0, r1, #0
	ldr	r7, [sp, #12]
	adc	r0, r0, r1
	ldr	r3, .L1231
	cmp	r7, #0
	movne	r1, #0
	andeq	r1, r0, #1
	cmp	r1, #0
	beq	.L1159
	movw	r2, #2340
	ldrh	r1, [r3, r2]
	cmp	r1, #24
	movhi	r6, #1
	bhi	.L1160
	ldrh	r6, [r3, #98]
	cmp	r1, #16
	movhi	r6, r6, lsr #5
	bhi	.L1160
	cmp	r1, #12
	movhi	r6, r6, lsr #4
	bhi	.L1160
	cmp	r1, #8
	movhi	r6, r6, lsr #2
.L1160:
	movw	ip, #3148
	ldr	r2, .L1231
	ldrh	r3, [r3, ip]
	cmp	r3, r1
	mov	r1, ip
	bcs	.L1164
	movw	r3, #2440
	movw	r0, #65535
	ldrh	r3, [r2, r3]
	cmp	r3, r0
	bne	.L1165
	movw	r0, #3202
	ldrh	r0, [r2, r0]
	cmp	r0, r3
	bne	.L1165
	ldr	r1, .L1231+4
	movw	r3, #3660
	ldrh	r0, [r1, r3]
	cmp	r0, #0
	bne	.L1166
	ldr	r3, [r2, #2504]
	ldr	r1, [r2, #2496]
	add	r3, r3, r3, asl #1
	cmp	r1, r3, lsr #2
	movcs	r3, #18
	strcsh	r3, [r2, ip]	@ movhi
	bcs	.L1168
.L1166:
	ldr	r3, .L1231
	movw	r2, #2572
	movw	r1, #3148
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L1168:
	ldr	r3, .L1231
	mov	r2, #0
	str	r2, [r3, #3160]
	b	.L1169
.L1165:
	ldr	r3, .L1231
	movw	r2, #2572
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L1164:
	ldr	r2, .L1231
	movw	r3, #3206
	movw	r4, #65535
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	addne	r6, r6, #32
	uxthne	r6, r6
	b	.L1170
.L1159:
	movw	r2, #2440
	ldrh	r2, [r3, r2]
	cmp	r2, ip
	bne	.L1228
	movw	ip, #3202
	ldrh	ip, [r3, ip]
	cmp	ip, r2
	movne	r0, #0
	andeq	r0, r0, #1
	cmp	r0, #0
	beq	.L1228
	movw	r0, #2588
	ldrh	r0, [r3, r0]
	cmp	r0, r2
	movne	r4, r2
	bne	.L1228
	str	r1, [r3, #3160]
	movw	r2, #3148
	movw	r1, #2340
	ldrh	r0, [r3, r1]
	ldrh	r1, [r3, r2]
	cmp	r0, r1
	bls	.L1171
	ldr	r0, .L1231+4
	movw	r1, #3660
	ldrh	r1, [r0, r1]
	cmp	r1, #0
	bne	.L1172
	ldr	r1, [r3, #2504]
	ldr	r0, [r3, #2496]
	add	r1, r1, r1, asl #1
	cmp	r0, r1, lsr #2
	movcs	r1, #18
	strcsh	r1, [r3, r2]	@ movhi
	bcs	.L1174
.L1172:
	ldr	r3, .L1231
	movw	r2, #2572
	movw	r1, #3148
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L1174:
	bl	FtlReadRefresh
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, .L1231
	ldr	r3, [r3, #2320]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r3, [r3, r0]
	cmp	r3, #4
	bhi	.L1230
.L1171:
	ldr	r7, .L1231+4
	movw	r6, #3660
	ldrh	r0, [r7, r6]
	cmp	r0, #0
	bne	.L1224
	ldr	r4, .L1231
	movw	r3, #2572
	ldrh	r5, [r4, r3]
	add	r3, r3, #576
	add	r2, r5, r5, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r4, r3]	@ movhi
	bl	List_get_gc_head_node
	ldrh	r2, [r4, #100]
	ldrh	r1, [r4, #28]
	ldr	r3, [r4, #2320]
	mul	r2, r1, r2
	add	r2, r2, r2, lsr #31
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r3, [r3, r0]
	cmp	r3, r2, asr #1
	ble	.L1175
	movw	r2, #2340
	sub	r5, r5, #1
	ldrh	r2, [r4, r2]
	cmp	r2, r5
	blt	.L1175
	bl	FtlReadRefresh
	ldrh	r0, [r7, r6]
	b	.L1169
.L1175:
	cmp	r3, #0
	bne	.L1224
	movw	r0, #65535
	bl	decrement_vpc_count
	ldr	r2, .L1231
	movw	r3, #2340
	ldrh	r0, [r2, r3]
	add	r0, r0, #1
	b	.L1169
.L1224:
	movw	r4, #65535
.L1228:
	mov	r6, #1
.L1170:
	ldr	r3, .L1231
	movw	r2, #2588
	movw	r0, #65535
	ldrh	r1, [r3, r2]
	cmp	r1, r0
	bne	.L1176
	cmp	r4, r1
	strneh	r4, [r3, r2]	@ movhi
	bne	.L1178
	movw	r2, #3202
	ldrh	r1, [r3, r2]
	cmp	r1, r4
	beq	.L1178
	ldr	r0, [r3, #2320]
	mov	r1, r1, asl #1
	ldrh	r1, [r0, r1]
	cmp	r1, #0
	mvneq	r1, #0
	streqh	r1, [r3, r2]	@ movhi
	ldr	r3, .L1231
	movw	r1, #2588
	ldrh	r0, [r3, r2]
	strh	r0, [r3, r1]	@ movhi
	mvn	r1, #0
	strh	r1, [r3, r2]	@ movhi
.L1178:
	ldr	r8, .L1231
	movw	r7, #2588
	mov	r3, #0
	ldrh	r0, [r8, r7]
	strb	r3, [r8, #2596]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1176
	bl	IsBlkInGcList
	ldr	r5, .L1231
	cmp	r0, #0
	mvnne	r3, #0
	strneh	r3, [r8, r7]	@ movhi
	ldrh	r2, [r8, r7]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1176
	ldr	r0, .L1231+12
	bl	make_superblock
	movw	r1, #2590
	ldr	r2, .L1231+4
	mov	r3, #0
	strh	r3, [r5, r1]	@ movhi
	add	r1, r1, #1072
	strb	r3, [r5, #2594]
	strh	r3, [r2, r1]	@ movhi
	movw	r3, #2588
	ldrh	r3, [r5, r3]
	ldr	r1, [r5, #2320]
	mov	r3, r3, asl #1
	ldrh	r1, [r1, r3]
	mov	r3, #3664
	strh	r1, [r2, r3]	@ movhi
.L1176:
	ldr	r3, .L1231
	movw	r1, #2588
	movw	r0, #2344
	ldrh	r2, [r3, r1]
	ldrh	r0, [r3, r0]
	cmp	r0, r2
	beq	.L1181
	movw	r0, #2392
	ldrh	r0, [r3, r0]
	cmp	r0, r2
	bne	.L1211
.L1181:
	mvn	r2, #0
	strh	r2, [r3, r1]	@ movhi
.L1211:
	str	r4, [sp, #16]
.L1226:
	ldr	r4, .L1231
	movw	sl, #2588
	movw	r0, #65535
	ldrh	r3, [r4, sl]
	cmp	r3, r0
	bne	.L1183
	movw	r5, #3156
	mov	r3, #0
	str	r3, [r4, #3160]
.L1225:
	ldrh	r8, [r4, r5]
	mov	r0, r8
	bl	List_get_gc_head_node
	movw	r2, #65535
	uxth	r7, r0
	strh	r7, [r4, sl]	@ movhi
	cmp	r7, r2
	bne	.L1185
	ldr	r2, .L1231
	movw	r3, #3156
	mov	r1, #0
	mov	r0, #8
	strh	r1, [r2, r3]	@ movhi
	b	.L1169
.L1185:
	mov	r0, r7
	add	r8, r8, #1
	bl	IsBlkInGcList
	cmp	r0, #0
	strneh	r8, [r4, r5]	@ movhi
	bne	.L1225
	ldrh	r0, [r4, #28]
	mov	r7, r7, asl #1
	ldrh	r3, [r4, #98]
	uxth	r8, r8
	ldr	r2, [r4, #2320]
	strh	r8, [r4, r5]	@ movhi
	mul	r3, r0, r3
	ldrh	r1, [r2, r7]
	add	r0, r3, r3, lsr #31
	cmp	r1, r0, asr #1
	bgt	.L1188
	cmp	r8, #48
	bls	.L1189
	cmp	r1, #8
	bls	.L1189
	movw	r1, #3188
	ldrh	r1, [r4, r1]
	cmp	r1, #35
	bhi	.L1189
.L1188:
	mov	r1, #0
	strh	r1, [r4, r5]	@ movhi
.L1189:
	ldrh	r1, [r2, r7]
	movw	r0, #65535
	ldr	r7, [sp, #16]
	ldr	r2, .L1231
	cmp	r1, r3
	cmpge	r7, r0
	movw	r0, #2588
	movne	r3, #0
	moveq	r3, #1
	bne	.L1190
	mvn	r3, #0
	strh	r3, [r2, r0]	@ movhi
	movw	r3, #3156
	mov	r1, #0
	strh	r1, [r2, r3]	@ movhi
	add	r3, r3, #504
.L1229:
	ldr	r2, .L1231+4
	ldrh	r0, [r2, r3]
	b	.L1169
.L1190:
	cmp	r1, #0
	bne	.L1191
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r3, [r4, r5]
	add	r3, r3, #1
	strh	r3, [r4, r5]	@ movhi
	b	.L1225
.L1191:
	strb	r3, [r2, #2596]
	movw	r3, #2344
	ldrh	r1, [r2, r0]
	ldrh	r3, [r2, r3]
	cmp	r1, r3
	bne	.L1192
	ldr	r0, .L1231+16
	movw	r2, #698
	ldr	r1, .L1231+20
	bl	printk
.L1192:
	ldr	r4, .L1231
	movw	r5, #2588
	movw	r3, #2392
	ldrh	r2, [r4, r5]
	ldrh	r3, [r4, r3]
	cmp	r2, r3
	bne	.L1193
	ldr	r0, .L1231+16
	movw	r2, #699
	ldr	r1, .L1231+20
	bl	printk
.L1193:
	movw	r3, #2440
	ldrh	r2, [r4, r5]
	ldrh	r3, [r4, r3]
	cmp	r2, r3
	bne	.L1194
	ldr	r0, .L1231+16
	mov	r2, #700
	ldr	r1, .L1231+20
	bl	printk
.L1194:
	ldr	r0, .L1231+12
	bl	make_superblock
	ldr	r2, .L1231+4
	ldr	r4, .L1231
	movw	r0, #3662
	mov	r3, #0
	strh	r3, [r2, r0]	@ movhi
	movw	r0, #2588
	ldrh	r0, [r4, r0]
	ldr	ip, [r4, #2320]
	mov	r0, r0, asl #1
	ldrh	ip, [ip, r0]
	mov	r0, #3664
	strb	r3, [r4, #2594]
	strh	ip, [r2, r0]	@ movhi
	movw	r2, #2590
	strh	r3, [r4, r2]	@ movhi
.L1183:
	ldr	r7, [sp, #12]
	mov	r2, #1
	ldr	r3, .L1231
	cmp	r7, #0
	str	r2, [r3, #3404]
	ldrh	fp, [r3, #98]
	beq	.L1195
	movw	r2, #2588
	ldrh	r1, [r3, #28]
	ldrh	r0, [r3, r2]
	ldr	r2, [r3, #2320]
	mul	r1, r1, fp
	mov	r3, r0, asl #1
	ldrh	r2, [r2, r3]
	rsb	r3, r2, r1
	add	r2, r3, #3
	cmp	r3, #0
	movlt	r3, r2
	add	r6, r6, r3, asr #2
	uxth	r6, r6
.L1195:
	ldr	r2, .L1231
	movw	r3, #2590
	mov	r9, #0
	ldr	r4, .L1231
	str	fp, [sp, #20]
	ldrh	r3, [r2, r3]
	add	r2, r3, r6
	cmp	r2, fp
	rsbgt	r6, r3, fp
	uxthgt	r6, r6
	mov	sl, r6
	b	.L1197
.L1199:
	ldrh	r1, [r2, #2]!
	movw	r7, #65535
	add	r3, r3, #1
	cmp	r1, r7
	uxth	r3, r3
	orrne	r1, ip, r1, asl #10
	mlane	r7, r0, r6, lr
	addne	r6, r6, #1
	uxthne	r6, r6
	strne	r1, [r7, #4]
.L1206:
	cmp	r3, r5
	bne	.L1199
	ldr	r0, [r4, #3180]
	mov	r1, r6
	ldrb	r2, [r4, #2596]	@ zero_extendqisi2
	mov	r5, #0
	bl	FlashReadPages
	mov	fp, r5
	b	.L1200
.L1204:
	ldr	r3, [r4, #3180]
	add	r2, r3, r5
	ldr	r3, [r3, r5]
	ldr	r7, [r2, #12]
	cmn	r3, #1
	beq	.L1201
	ldrh	r3, [r7, #0]
	movw	r0, #61589
	cmp	r3, r0
	bne	.L1201
	ldr	r8, [r7, #8]
	cmn	r8, #1
	bne	.L1202
	ldr	r0, .L1231+16
	movw	r2, #734
	ldr	r1, .L1231+20
	bl	printk
.L1202:
	add	r1, sp, #28
	mov	r2, #0
	mov	r0, r8
	bl	log2phys
	ldr	r2, [r4, #3180]
	ldr	r1, [sp, #28]
	add	r2, r2, r5
	ldr	r3, [r2, #4]
	cmp	r1, r3
	bne	.L1201
	ldr	r1, .L1231+4
	movw	r3, #3662
	mov	r8, #20
	ldr	r2, [r2, #16]
	ldrh	r0, [r1, r3]
	add	r0, r0, #1
	strh	r0, [r1, r3]	@ movhi
	ldr	r1, [r4, #3164]
	ldr	r3, [r4, #3420]
	mla	r3, r8, r1, r3
	str	r2, [r3, #16]
	str	r3, [sp, #8]
	bl	Ftl_get_new_temp_ppa
	ldr	r3, [sp, #8]
	mov	r1, #1
	ldr	r2, [r4, #3164]
	str	r0, [r3, #4]
	ldr	r3, [r4, #3420]
	mla	r8, r8, r2, r3
	ldr	r3, [r4, #3180]
	add	r3, r3, r5
	ldr	r2, [r3, #8]
	str	r2, [r8, #8]
	ldr	r3, [r3, #12]
	str	r3, [r8, #12]
	ldr	r3, [sp, #28]
	str	r3, [r7, #12]
	movw	r3, #2440
	ldrh	r3, [r4, r3]
	strh	r3, [r7, #2]	@ movhi
	ldr	r3, [r4, #2544]
	ldr	r0, [r4, #3180]
	str	r3, [r7, #4]
	add	r0, r0, r5
	ldr	r3, [r4, #3164]
	add	r3, r3, #1
	str	r3, [r4, #3164]
	bl	FtlGcBufAlloc
	ldrb	r3, [r4, #2447]	@ zero_extendqisi2
	ldr	r2, [r4, #3164]
	cmp	r2, r3
	beq	.L1203
	movw	r3, #2444
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L1201
.L1203:
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	beq	.L1201
	ldr	r3, .L1231
	mov	r2, #0
	str	r2, [r3, #3404]
.L1230:
	movw	r3, #3660
	b	.L1229
.L1201:
	add	fp, fp, #1
	add	r5, r5, #20
	uxth	fp, fp
.L1200:
	cmp	fp, r6
	bne	.L1204
	add	r9, r9, #1
.L1197:
	uxth	r3, r9
	ldr	r7, .L1231
	cmp	r3, sl
	movw	r3, #2590
	bcs	.L1205
	ldrh	ip, [r4, r3]
	mov	r6, #0
	ldrh	r5, [r4, #28]
	mov	r3, r6
	ldr	lr, [r4, #3180]
	add	ip, ip, r9
	ldr	r2, .L1231+24
	mov	r0, #20
	b	.L1206
.L1205:
	ldrh	r2, [r7, r3]
	ldr	fp, [sp, #20]
	add	r6, sl, r2
	uxth	r6, r6
	strh	r6, [r7, r3]	@ movhi
	cmp	r6, fp
	bcc	.L1207
	ldr	r3, [r7, #3164]
	ldr	r5, .L1231+4
	cmp	r3, #0
	beq	.L1208
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	movne	r3, #0
	strne	r3, [r7, #3404]
	movwne	r3, #3660
	ldrneh	r0, [r5, r3]
	bne	.L1169
.L1208:
	movw	r3, #3662
	ldrh	ip, [r5, r3]
	cmp	ip, #0
	bne	.L1209
	ldr	r3, .L1231
	movw	r2, #2588
	ldrh	r1, [r3, r2]
	ldr	r0, [r3, #2320]
	mov	r1, r1, asl #1
	ldrh	r4, [r0, r1]
	cmp	r4, #0
	beq	.L1209
	strh	ip, [r0, r1]	@ movhi
	ldrh	r0, [r3, r2]
	bl	update_vpc_list
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1209:
	ldr	r2, .L1231
	movw	r3, #2588
	mvn	r1, #0
	strh	r1, [r2, r3]	@ movhi
.L1207:
	ldr	r3, .L1231
	mov	r2, #0
	str	r2, [r3, #3404]
	movw	r2, #2340
	ldrh	r2, [r3, r2]
	cmp	r2, #2
	ldrlsh	r6, [r3, #98]
	bls	.L1226
.L1210:
	ldr	r1, .L1231+4
	movw	r3, #3660
	ldrh	r0, [r1, r3]
	cmp	r0, #0
	addeq	r0, r2, #1
.L1169:
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1232:
	.align	2
.L1231:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC107
	.word	.LANCHOR0+2588
	.word	.LC1
	.word	.LANCHOR1+520
	.word	.LANCHOR0+2602
	.fnend
	.size	rk_ftl_garbage_collect.part.11, .-rk_ftl_garbage_collect.part.11
	.align	2
	.global	rk_ftl_garbage_collect
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1239
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	ldr	r0, [r3, #3404]
	cmp	r0, #0
	movne	r0, #0
	ldmnefd	sp!, {r4, pc}
	movw	r2, #2332
	ldrh	r2, [r3, r2]
	cmp	r2, #47
	ldmlsfd	sp!, {r4, pc}
	ldr	r2, .L1239+4
	ldrh	r1, [r2, #4]
	movw	r2, #65535
	cmp	r1, r2
	beq	.L1235
	movw	r1, #2440
	ldrh	r3, [r3, r1]
	cmp	r3, r2
	beq	.L1235
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	bne	.L1238
.L1235:
	mov	r0, r4
	ldmfd	sp!, {r4, lr}
	b	rk_ftl_garbage_collect.part.11
.L1238:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L1240:
	.align	2
.L1239:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	rk_ftl_garbage_collect, .-rk_ftl_garbage_collect
	.align	2
	.global	sftl_gc
	.type	sftl_gc, %function
sftl_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r0, #1
	mov	r1, r0
	b	rk_ftl_garbage_collect
	.fnend
	.size	sftl_gc, .-sftl_gc
	.align	2
	.global	allocate_new_data_superblock
	.type	allocate_new_data_superblock, %function
allocate_new_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldr	r6, .L1249
	ldrh	r4, [r0, #0]
	ldrh	r3, [r6, #36]
	cmp	r3, r4
	bcs	.L1243
	ldr	r0, .L1249+4
	movw	r2, #2711
	ldr	r1, .L1249+8
	bl	printk
.L1243:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L1244
	ldr	r2, [r6, #2320]
	mov	r3, r4, asl #1
	mov	r0, r4
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1245
	bl	INSERT_DATA_LIST
	b	.L1244
.L1245:
	bl	INSERT_FREE_LIST
.L1244:
	ldr	r6, .L1249
	mov	r3, #1
	strb	r3, [r5, #8]
	movw	r3, #3386
	ldrh	r0, [r6, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1246
	cmp	r4, r0
	bne	.L1247
	ldr	r2, [r6, #2320]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1248
.L1247:
	bl	update_vpc_list
.L1248:
	movw	r3, #3386
	mvn	r2, #0
	strh	r2, [r6, r3]	@ movhi
.L1246:
	mov	r0, r5
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	r0, #0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1250:
	.align	2
.L1249:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+543
	.fnend
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	2
	.global	FtlProgPages
	.type	FtlProgPages, %function
FtlProgPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r3
	mov	r2, #0
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	mov	r5, r0
	mov	sl, r1
	mov	r7, #0
	bl	FlashProgPages
	ldr	r6, .L1264
	b	.L1252
.L1255:
	ldr	r0, [r5, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r4, #0]
	cmp	r3, r0
	bne	.L1253
	ldr	r2, [r6, #2320]
	mov	r3, r3, asl #1
	ldrh	r1, [r4, #4]
	ldrh	r0, [r2, r3]
	rsb	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	ldrh	r3, [r6, #98]
	strb	r8, [r4, #6]
	strh	r8, [r4, #4]	@ movhi
	strh	r3, [r4, #2]	@ movhi
.L1253:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L1254
	mov	r0, r4
	bl	allocate_new_data_superblock
.L1254:
	ldr	r3, [r6, #2732]
	add	r3, r3, #1
	str	r3, [r6, #2732]
	ldr	r0, [r5, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlGcMarkBadPhyBlk
	mov	r0, r4
	bl	get_new_active_ppa
	mov	r1, #1
	mov	r2, #0
	str	r0, [r5, #4]
	str	r0, [sp, #4]
	mov	r0, r5
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	bl	FlashProgPages
	b	.L1262
.L1263:
	mov	r8, #0
.L1262:
	ldr	r3, [r5, #0]
	cmn	r3, #1
	beq	.L1255
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	ldrh	r3, [r6, #28]
	cmp	r2, r3
	bcc	.L1256
	ldr	r0, .L1264+4
	movw	r2, #958
	ldr	r1, .L1264+8
	bl	printk
.L1256:
	ldr	r3, [r5, #4]
	add	r1, sp, #4
	ldr	r0, [r5, #16]
	mov	r2, #1
	str	r3, [sp, #4]
	bl	log2phys
	ldr	r3, [r5, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L1257
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r6, #2320]
	mov	r3, r0, asl #1
	mov	r8, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L1258
	ldr	r0, .L1264+12
	mov	r1, r8
	bl	printk
.L1258:
	mov	r0, r8
	bl	decrement_vpc_count
.L1257:
	add	r7, r7, #1
	add	r5, r5, #20
.L1252:
	cmp	r7, sl
	bne	.L1263
	ldr	r3, .L1264
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	ldrh	r3, [r3, #28]
	cmp	r2, r3
	bcc	.L1251
	ldr	r0, .L1264+4
	movw	r2, #973
	ldr	r1, .L1264+8
	bl	printk
.L1251:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L1265:
	.align	2
.L1264:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+572
	.word	.LC108
	.fnend
	.size	FtlProgPages, .-FtlProgPages
	.align	2
	.global	FtlVpcCheckAndModify
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	mov	r4, #0
	ldr	r5, .L1274
	ldr	r1, .L1274+4
	ldr	r0, .L1274+8
	bl	printk
	ldrh	r2, [r5, #38]
	ldr	r0, [r5, #3444]
	mov	r1, #0
	mov	r2, r2, asl #1
	bl	ftl_memset
	b	.L1267
.L1269:
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L1268
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, [r5, #3444]
	mov	r0, r0, asl #1
	ldrh	r2, [r3, r0]
	add	r2, r2, #1
	strh	r2, [r3, r0]	@ movhi
.L1268:
	add	r4, r4, #1
.L1267:
	ldr	r3, [r5, #2504]
	cmp	r4, r3
	bcc	.L1269
	mov	r5, #0
	ldr	r4, .L1274
	movw	r7, #65535
	movw	r8, #2344
	b	.L1270
.L1273:
	ldr	r3, [r4, #2320]
	mov	r6, r5, asl #1
	ldrh	r2, [r3, r6]
	ldr	r3, [r4, #3444]
	ldrh	r3, [r3, r6]
	cmp	r2, r3
	beq	.L1271
	cmp	r2, r7
	beq	.L1271
	ldr	r0, .L1274+12
	mov	r1, r5
	bl	printk
	ldrh	r3, [r4, r8]
	cmp	r3, r5
	beq	.L1271
	movw	r3, #2440
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L1271
	movw	r3, #2392
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L1271
	ldr	r3, [r4, #2320]
	ldrh	r2, [r3, r6]
	cmp	r2, #0
	ldr	r2, [r4, #3444]
	ldrh	r2, [r2, r6]
	strh	r2, [r3, r6]	@ movhi
	beq	.L1271
.L1272:
	mov	r0, r5
	bl	update_vpc_list
.L1271:
	add	r5, r5, #1
	uxth	r5, r5
.L1270:
	ldrh	r3, [r4, #36]
	cmp	r3, r5
	bhi	.L1273
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L1275:
	.align	2
.L1274:
	.word	.LANCHOR0
	.word	.LANCHOR1+585
	.word	.LC95
	.word	.LC109
	.fnend
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.align	2
	.global	FtlSysBlkInit
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movw	r3, #3522
	ldr	r2, .L1288
	mov	r1, #0
	ldr	r4, .L1288+4
	strh	r1, [r2, r3]	@ movhi
	ldrh	r0, [r4, #32]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	mov	r3, #2576
	ldrh	r2, [r4, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1284
	bl	FtlLoadSysInfo
	subs	r5, r0, #0
	bne	.L1284
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	mov	r0, #1
	bl	FtlUpdateVaildLpn
	ldrh	r2, [r4, #134]
	ldr	r1, [r4, #2488]
	mov	r3, r5
	b	.L1278
.L1280:
	add	r0, r1, r5
	add	r5, r5, #12
	ldr	r0, [r0, #4]
	cmp	r0, #0
	blt	.L1279
	add	r3, r3, #1
.L1278:
	cmp	r3, r2
	blt	.L1280
.L1279:
	ldr	r4, .L1288+4
	movw	r1, #2292
	cmp	r3, r2
	ldrh	r0, [r4, r1]
	add	r0, r0, #1
	strh	r0, [r4, r1]	@ movhi
	blt	.L1281
	ldr	r2, .L1288
	movw	r3, #3522
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1282
.L1281:
	ldr	r0, .L1288+8
	bl	FtlSuperblockPowerLostFix
	ldr	r0, .L1288+12
	bl	FtlSuperblockPowerLostFix
	movw	r3, #2344
	ldrh	r3, [r4, r3]
	movw	r2, #2348
	ldr	r1, [r4, #2320]
	ldrh	r0, [r4, r2]
	mov	r3, r3, asl #1
	ldrh	ip, [r1, r3]
	rsb	r0, r0, ip
	strh	r0, [r1, r3]	@ movhi
	ldrh	r1, [r4, #98]
	movw	r3, #2346
	ldr	r0, [r4, #2320]
	strh	r1, [r4, r3]	@ movhi
	mov	r3, #0
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #44
	strb	r3, [r4, #2350]
	ldrh	r1, [r4, r2]
	add	r2, r2, #4
	ldrh	ip, [r4, r2]
	mov	r1, r1, asl #1
	ldrh	lr, [r0, r1]
	rsb	ip, ip, lr
	strh	ip, [r0, r1]	@ movhi
	strh	r3, [r4, r2]	@ movhi
	movw	r1, #2394
	ldrh	r0, [r4, #98]
	strb	r3, [r4, #2398]
	movw	r3, #2294
	ldrh	r2, [r4, r3]
	strh	r0, [r4, r1]	@ movhi
	add	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L1282:
	ldr	r4, .L1288+4
	bl	FtlVpcCheckAndModify
	movw	r3, #2344
	ldrh	r0, [r4, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1286
	movw	r3, #2348
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L1286
	movw	r2, #2396
	ldrh	r5, [r4, r2]
	cmp	r5, #0
	bne	.L1287
	bl	FtlGcRefreshBlock
	movw	r3, #2392
	ldrh	r0, [r4, r3]
	bl	FtlGcRefreshBlock
	bl	FtlVpcTblFlush
	ldr	r0, .L1288+8
	bl	allocate_new_data_superblock
	ldr	r0, .L1288+12
	bl	allocate_new_data_superblock
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, pc}
.L1284:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1286:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1287:
	mov	r0, r3
	ldmfd	sp!, {r3, r4, r5, pc}
.L1289:
	.align	2
.L1288:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	.LANCHOR0+2344
	.word	.LANCHOR0+2392
	.fnend
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.global	sftl_init
	.type	sftl_init, %function
sftl_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.L1291:
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mvn	r3, #0
	ldr	r4, .L1295
	ldr	r5, .L1295+4
	ldr	r1, .L1295+8
	str	r3, [r4, #0]
	ldr	r0, .L1295+12
	bl	printk
	mov	r0, r5
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldrh	r0, [r5, #32]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	bne	.L1292
	bl	FtlSysBlkInit
	cmp	r0, #0
	bne	.L1292
	mov	r3, #1
	str	r3, [r4, #0]
	movw	r3, #2340
	ldrh	r3, [r5, r3]
	cmp	r3, #15
	bhi	.L1292
	movw	r4, #8129
.L1293:
	mov	r0, #0
	mov	r1, #1
	bl	rk_ftl_garbage_collect
	subs	r4, r4, #1
	bne	.L1293
.L1292:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1296:
	.align	2
.L1295:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC71
	.word	.LC70
	.fnend
	.size	sftl_init, .-sftl_init
	.align	2
	.global	FtlMakeBbt
	.type	FtlMakeBbt, %function
FtlMakeBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	bl	FtlBbtMemInit
	mov	r5, #0
	ldr	r6, .L1315
	bl	FtlLoadFactoryBbt
	add	r8, r6, #12
	sub	r4, r6, #148
	b	.L1298
.L1304:
	ldrh	r3, [r8], #2
	movw	r2, #65535
	ldr	r0, [r4, #3288]
	ldr	sl, [r4, #3292]
	cmp	r3, r2
	str	r0, [r4, #3276]
	str	sl, [r4, #3280]
	beq	.L1299
	ldrh	r7, [r4, #94]
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L1315+4
	mla	r7, r7, r5, r3
	mov	r3, r7, asl #10
	str	r3, [r4, #3272]
	bl	FlashReadPages
	ldrh	r2, [r4, #94]
	ldr	r0, [r6, #28]
	add	r2, r2, #7
	ldr	r1, [r4, #3276]
	mov	r2, r2, lsr #3
	bl	memcpy
	b	.L1300
.L1299:
	mov	r1, r5
	ldr	fp, .L1315+8
	bl	FlashGetBadBlockList
	ldr	r0, [r4, #3276]
	ldr	r1, [r6, #28]
	bl	FtlBbt2Bitmap
	ldrh	r9, [r4, #94]
	sub	r9, r9, #1
	uxth	r9, r9
	b	.L1313
.L1302:
	sub	r9, r9, #1
	uxth	r9, r9
.L1313:
	ldrh	r0, [r4, #94]
	mla	r0, r0, r5, r9
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L1302
	mov	r1, #0
	mov	r2, #16
	strh	r9, [r8, #-2]	@ movhi
	ldr	r0, [r4, #3292]
	bl	ftl_memset
	mov	r1, #0
	mov	r2, #4096
	ldr	r0, [r4, #3288]
	bl	ftl_memset
	strh	fp, [sl, #0]	@ movhi
	mov	r3, #0
	str	r3, [sl, #4]
	ldrh	r3, [r8, #-2]
	ldrh	r7, [r4, #94]
	strh	r3, [sl, #2]	@ movhi
	ldrh	r3, [r8, #-2]
	ldr	r1, [r6, #28]
	ldr	r0, [r4, #3276]
	mla	r7, r7, r5, r3
	mov	r3, r7, asl #10
	str	r3, [r4, #3272]
	movw	r3, #3384
	ldrh	r2, [r4, r3]
	mov	r2, r2, asl #2
	bl	memcpy
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L1315+4
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r3, r1
	ldr	r0, .L1315+4
	mov	r2, r1
	bl	FlashProgPages
	ldr	r3, [r4, #3268]
	cmn	r3, #1
	bne	.L1300
	uxth	r0, r7
	bl	FtlBbmMapBadBlock
	b	.L1313
.L1300:
	uxth	r0, r7
	add	r5, r5, #1
	bl	FtlBbmMapBadBlock
	add	r6, r6, #4
.L1298:
	ldrh	r3, [r4, #50]
	cmp	r5, r3
	bcc	.L1304
	mov	r4, #0
	ldr	r6, .L1315+12
	b	.L1305
.L1306:
	mov	r0, r4
	add	r4, r4, #1
	bl	FtlBbmMapBadBlock
	uxth	r4, r4
.L1305:
	ldrh	r3, [r6, #110]
	ldr	r5, .L1315+12
	cmp	r3, r4
	bhi	.L1306
	ldrh	r4, [r5, #160]
	movw	r6, #65535
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L1307
.L1312:
	mov	r0, r4
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L1308
	mov	r0, r4
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1309
	mov	r0, r4
	bl	FtlBbmMapBadBlock
	b	.L1308
.L1309:
	ldrh	r3, [r5, #148]
	cmp	r3, r6
	streqh	r4, [r5, #148]	@ movhi
.L1310:
	ldrne	r3, .L1315+12
	strneh	r4, [r3, #152]	@ movhi
	bne	.L1311
.L1308:
	sub	r4, r4, #1
	uxth	r4, r4
.L1307:
	ldrh	r3, [r5, #160]
	sub	r3, r3, #48
	cmp	r4, r3
	bgt	.L1312
.L1311:
	ldr	r4, .L1315+12
	mov	r5, #0
	mov	r1, #1
	mov	r2, #2
	ldr	r0, [r4, #3300]
	ldrh	r3, [r4, #148]
	str	r5, [r4, #156]
	strh	r5, [r4, #150]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r0, #4]
	ldrh	r3, [r4, #152]
	mov	r3, r3, asl #10
	str	r3, [r0, #24]
	bl	FlashEraseBlocks
	ldrh	r0, [r4, #148]
	bl	FtlBbmMapBadBlock
	ldrh	r0, [r4, #152]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [r4, #156]
	ldrh	r2, [r4, #152]
	add	r3, r3, #1
	str	r3, [r4, #156]
	ldrh	r3, [r4, #148]
	strh	r5, [r4, #150]	@ movhi
	strh	r2, [r4, #148]	@ movhi
	strh	r3, [r4, #152]	@ movhi
	bl	FtlBbmTblFlush
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1316:
	.align	2
.L1315:
	.word	.LANCHOR0+148
	.word	.LANCHOR0+3268
	.word	-3872
	.word	.LANCHOR0
	.fnend
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	2
	.global	ftl_low_format
	.type	ftl_low_format, %function
ftl_low_format:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1336
	mov	r2, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	str	r2, [r3, #2540]
	str	r2, [r3, #2544]
	ldrh	r0, [r3, #32]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	beq	.L1318
	bl	FtlMakeBbt
.L1318:
	mov	r3, #0
	ldr	r2, .L1336
	ldr	r1, .L1336+4
	b	.L1319
.L1320:
	ldr	r0, [r2, #3316]
	mvn	ip, r3
	orr	ip, r3, ip, asl #16
	str	ip, [r0, r3, asl #2]
	ldr	r0, [r2, #3308]
	str	r1, [r0, r3, asl #2]
	add	r3, r3, #1
	uxth	r3, r3
.L1319:
	ldrh	r0, [r2, #54]
	ldr	r7, .L1336
	cmp	r3, r0, asl #7
	blt	.L1320
	ldrh	r6, [r7, #36]
	mov	r5, #0
	b	.L1321
.L1322:
	mov	r0, r6
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r5, r5, r0
	uxth	r5, r5
.L1321:
	ldrh	r3, [r7, #38]
	ldr	r4, .L1336
	cmp	r3, r6
	bhi	.L1322
	ldrh	r1, [r4, #28]
	sub	r3, r5, #3
	cmp	r3, r1, asl #1
	bge	.L1323
.L1327:
	mov	r5, #0
	ldr	r7, .L1336
	mov	r6, r5
	b	.L1324
.L1323:
	mov	r0, r5
	bl	__aeabi_uidiv
	ldr	r3, [r4, #128]
	add	r0, r0, r3
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldrh	r0, [r4, #32]
	bl	FtlFreeSysBlkQueueInit
	ldrh	r5, [r4, #36]
	b	.L1325
.L1326:
	mov	r0, r5
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r5, r5, #1
	uxth	r5, r5
.L1325:
	ldrh	r3, [r4, #38]
	cmp	r3, r5
	bhi	.L1326
	b	.L1327
.L1328:
	mov	r0, r6
	mov	r1, #0
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r5, r5, r0
	uxth	r5, r5
.L1324:
	ldrh	r3, [r7, #36]
	ldr	r4, .L1336
	cmp	r3, r6
	bhi	.L1328
	ldrh	r6, [r4, #28]
	movw	sl, #2572
	ldr	r9, [r4, #40]
	ldrh	r3, [r4, #38]
	mov	r1, r6
	mov	r0, r9
	str	r3, [r4, #3304]
	bl	__aeabi_uidiv
	ubfx	r8, r0, #5, #16
	mov	r7, r0
	add	r3, r8, #36
	strh	r3, [r4, sl]	@ movhi
	mov	r3, #24
	str	r0, [r4, #2504]
	mul	r3, r3, r6
	cmp	r5, r3
	ble	.L1329
	rsb	r0, r5, r9
	mov	r1, r6
	bl	__aeabi_uidiv
	str	r0, [r4, #2504]
	mov	r0, r0, lsr #5
	add	r0, r0, #24
	strh	r0, [r4, sl]	@ movhi
.L1329:
	ldr	r3, .L1336
	ldrh	r2, [r3, #90]
	cmp	r2, #0
	beq	.L1330
	movw	r1, #2572
	ldrh	r0, [r3, r1]
	add	r0, r0, r2, lsr #1
	strh	r0, [r3, r1]	@ movhi
	mul	r0, r6, r2
	cmp	r0, r5
	strgt	r7, [r3, #2504]
	addgt	r2, r2, #32
	addgt	r8, r8, r2
	strgth	r8, [r3, r1]	@ movhi
.L1330:
	ldr	r4, .L1336
	movw	r3, #2572
	mvn	r5, #0
	ldr	r2, [r4, #2504]
	ldrh	r3, [r4, r3]
	rsb	r3, r3, r2
	mul	r6, r6, r3
	ldr	r3, .L1336+8
	str	r6, [r3, #3516]
	ldrh	r3, [r4, #98]
	mul	r6, r3, r6
	ldrh	r3, [r4, #54]
	str	r6, [r4, #2504]
	mul	r6, r3, r6
	str	r6, [r4, #136]
	bl	FtlBbmTblFlush
	ldrh	r2, [r4, #38]
	mov	r1, #0
	ldr	r0, [r4, #2320]
	movw	r6, #2344
	mov	r2, r2, asl #1
	bl	ftl_memset
	movw	r2, #2588
	strh	r5, [r4, r2]	@ movhi
	movw	r1, #2590
	ldrh	r2, [r4, #36]
	mov	r3, #0
	strh	r3, [r4, r1]	@ movhi
	movw	r1, #2346
	strh	r3, [r4, r1]	@ movhi
	mov	r1, #255
	ldr	r0, [r4, #2500]
	mov	r2, r2, lsr #3
	str	r3, [r4, #2496]
	strb	r3, [r4, #2594]
	strb	r3, [r4, #2596]
	strb	r3, [r4, #2350]
	strh	r3, [r4, r6]	@ movhi
	mov	r3, #1
	strb	r3, [r4, #2352]
	bl	ftl_memset
.L1331:
	ldr	r0, .L1336+12
	bl	make_superblock
	ldrb	r3, [r4, #2351]	@ zero_extendqisi2
	ldr	r7, .L1336
	cmp	r3, #0
	movw	r3, #2344
	bne	.L1332
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #2320]
	mov	r3, r3, asl #1
	strh	r5, [r2, r3]	@ movhi
	ldrh	r3, [r4, r6]
	add	r3, r3, #1
	strh	r3, [r4, r6]	@ movhi
	b	.L1331
.L1332:
	ldr	r2, [r7, #2540]
	movw	r5, #2392
	ldr	r1, [r7, #2320]
	mvn	r6, #0
	str	r2, [r7, #2356]
	add	r2, r2, #1
	str	r2, [r7, #2540]
	movw	r2, #2348
	ldrh	r0, [r7, r2]
	ldrh	r2, [r7, r3]
	mov	r2, r2, asl #1
	strh	r0, [r1, r2]	@ movhi
	movw	r0, #2394
	ldrh	r3, [r7, r3]
	mov	r2, #0
	strh	r2, [r7, r0]	@ movhi
	add	r3, r3, #1
	strb	r2, [r7, #2398]
	strh	r3, [r7, r5]	@ movhi
	mov	r3, #1
	strb	r3, [r7, #2400]
.L1333:
	ldr	r0, .L1336+16
	bl	make_superblock
	ldrb	r3, [r7, #2399]	@ zero_extendqisi2
	ldr	r4, .L1336
	cmp	r3, #0
	bne	.L1334
	ldrh	r3, [r7, r5]
	ldr	r2, [r7, #2320]
	mov	r3, r3, asl #1
	strh	r6, [r2, r3]	@ movhi
	ldrh	r3, [r7, r5]
	add	r3, r3, #1
	strh	r3, [r7, r5]	@ movhi
	b	.L1333
.L1334:
	ldr	r3, [r4, #2540]
	mvn	r5, #0
	ldr	r2, [r4, #2320]
	str	r3, [r4, #2404]
	add	r3, r3, #1
	str	r3, [r4, #2540]
	movw	r3, #2396
	ldrh	r1, [r4, r3]
	movw	r3, #2392
	ldrh	r3, [r4, r3]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	movw	r3, #2440
	strh	r5, [r4, r3]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	mov	r3, #2576
	mov	r2, #0
	strh	r0, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r5, [r4, r3]	@ movhi
	ldr	r3, .L1336+8
	ldr	r2, [r3, #3516]
	movw	r3, #2582
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, [r4, #2540]
	str	r3, [r4, #2584]
	add	r3, r3, #1
	str	r3, [r4, #2540]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cmp	r0, #0
	mov	r0, #0
	ldreq	r3, .L1336+20
	moveq	r2, #1
	streq	r2, [r3, #0]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1337:
	.align	2
.L1336:
	.word	.LANCHOR0
	.word	168778952
	.word	.LANCHOR3
	.word	.LANCHOR0+2344
	.word	.LANCHOR0+2392
	.word	.LANCHOR2
	.fnend
	.size	ftl_low_format, .-ftl_low_format
	.align	2
	.global	FtlWrite
	.type	FtlWrite, %function
FtlWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 72
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	cmp	r0, #16
	.pad #76
	sub	sp, sp, #76
	mov	r9, r1
	str	r2, [sp, #16]
	str	r3, [sp, #8]
	bne	.L1339
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r3
	bl	FtlVendorPartWrite
	b	.L1340
.L1339:
	ldr	r4, .L1386
	ldr	r0, [sp, #16]
	ldr	r3, [r4, #136]
	add	r7, r0, r1
	cmp	r7, r3
	mvnhi	r0, #0
	bhi	.L1340
	ldr	r5, .L1386+4
	ldr	r0, [r5, #0]
	cmn	r0, #1
	beq	.L1340
	ldr	r3, .L1386+8
	mov	r2, #2048
	ldrh	r6, [r4, #54]
	mov	r0, r1
	str	r2, [r3, #3668]
	mov	r1, r6
	bl	__aeabi_uidiv
	mov	r1, r6
	str	r0, [sp, #24]
	sub	r0, r7, #1
	bl	__aeabi_uidiv
	ldr	r3, [r4, #2532]
	ldr	r1, [sp, #24]
	ldr	r2, [sp, #16]
	add	r3, r2, r3
	str	r3, [r4, #2532]
	ldr	r3, [r4, #2516]
	rsb	r1, r1, r0
	str	r0, [sp, #28]
	add	fp, r1, #1
	str	r1, [sp, #32]
	add	r3, fp, r3
	str	r3, [r4, #2516]
	ldr	r3, [r5, #8]
	cmp	r3, #0
	ldreq	r5, .L1386+12
	beq	.L1341
	movw	r3, #2348
	ldrh	r5, [r4, r3]
	ldr	r3, .L1386+12
	cmp	r5, #0
	add	r2, r3, #48
	movne	r5, r3
	moveq	r5, r2
.L1341:
	mov	r7, #0
	ldr	r6, [sp, #24]
	str	r7, [sp, #20]
	ldr	r4, .L1386
	b	.L1342
.L1368:
	ldrb	r2, [r5, #6]	@ zero_extendqisi2
	ldrh	r3, [r4, #28]
	cmp	r2, r3
	bcc	.L1343
	ldr	r0, .L1386+16
	movw	r2, #1014
	ldr	r1, .L1386+20
	bl	printk
.L1343:
	ldrh	r2, [r5, #4]
	cmp	r2, #0
	bne	.L1344
	ldr	r3, .L1386+12
	cmp	r5, r3
	bne	.L1345
	movw	r3, #2396
	ldrh	r8, [r4, r3]
	cmp	r8, #0
	bne	.L1346
	add	r0, r5, #48
	bl	allocate_new_data_superblock
	ldr	r3, .L1386+4
	str	r8, [r3, #8]
.L1346:
	ldr	r0, .L1386+12
	bl	allocate_new_data_superblock
	ldr	r3, .L1386+4
	ldr	r2, [r3, #8]
	ldr	r3, .L1386+24
	cmp	r2, #0
	movne	r5, r3
	b	.L1347
.L1345:
	ldr	r1, .L1386+4
	str	r2, [r1, #8]
	movw	r2, #2348
	ldrh	r2, [r4, r2]
	cmp	r2, #0
	movne	r5, r3
	bne	.L1347
	mov	r0, r5
	bl	allocate_new_data_superblock
.L1347:
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	bne	.L1344
	mov	r0, r5
	bl	allocate_new_data_superblock
.L1344:
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	ldrh	r2, [r5, #4]
	mov	r3, r3, asl #2
	cmp	r3, r2
	movcs	r3, r2
	ldrb	r2, [r5, #6]	@ zero_extendqisi2
	cmp	r3, fp
	movcs	r3, fp
	str	r3, [sp, #12]
	ldrh	r3, [r4, #28]
	cmp	r2, r3
	bcc	.L1348
	ldr	r0, .L1386+16
	movw	r2, #1047
	ldr	r1, .L1386+20
	bl	printk
.L1348:
	mov	r3, #0
	str	fp, [sp, #44]
	str	r3, [sp, #4]
	mov	sl, r5
	b	.L1349
.L1366:
	ldrh	r3, [sl, #4]
	cmp	r3, #0
	beq	.L1377
	add	r1, sp, #48
	mov	r2, #0
	mov	r0, r6
	mov	r5, #20
	bl	log2phys
	mov	r0, sl
	bl	get_new_active_ppa
	ldr	r3, [sp, #4]
	ldrh	r2, [r4, #108]
	mul	r5, r5, r3
	ldr	r3, [r4, #3424]
	add	r3, r3, r5
	str	r6, [r3, #16]
	str	r0, [r3, #4]
	ldr	r0, [sp, #4]
	mul	r1, r2, r0
	bic	r1, r1, #3
	str	r1, [sp, #36]
	ldr	r0, [sp, #36]
	ldr	r1, [r4, #3312]
	add	r8, r1, r0
	str	r8, [r3, #12]
	str	r1, [sp, #40]
	mov	r1, #0
	mov	r0, r8
	bl	ftl_memset
	ldr	r1, [sp, #24]
	ldr	r2, [sp, #28]
	rsb	r0, r1, r6
	rsbs	fp, r0, #0
	rsb	ip, r2, r6
	adc	fp, fp, r0
	rsbs	r3, ip, #0
	adc	r3, r3, ip
	orrs	r0, r3, fp
	beq	.L1351
	cmp	fp, #0
	beq	.L1352
	ldrh	r7, [r4, #54]
	mov	r0, r9
	mov	r1, r7
	bl	__aeabi_uidivmod
	rsb	r7, r1, r7
	str	r1, [sp, #20]
	ldr	r1, [sp, #16]
	cmp	r7, r1
	movcs	r7, r1
	b	.L1353
.L1352:
	cmp	r3, #0
	beq	.L1353
	ldr	r2, [sp, #16]
	ldrh	r3, [r4, #54]
	add	r7, r2, r9
	str	fp, [sp, #20]
	mls	r7, r3, r6, r7
	uxth	r7, r7
.L1353:
	ldrh	r3, [r4, #54]
	cmp	r7, r3
	ldr	r3, [r4, #3424]
	bne	.L1354
	cmp	fp, #0
	add	r5, r3, r5
	ldrne	r3, [sp, #8]
	muleq	r3, r7, r6
	beq	.L1383
	b	.L1384
.L1354:
	cmp	fp, #0
	add	r3, r3, r5
	ldrne	r2, [r4, #3316]
	ldreq	r2, [r4, #3308]
	str	r2, [r3, #8]
	ldr	r3, [sp, #48]
	cmn	r3, #1
	beq	.L1359
	str	r3, [sp, #56]
	add	r0, sp, #52
	ldr	r3, [r4, #3424]
	mov	r1, #1
	str	r6, [sp, #68]
	add	r3, r3, r5
	ldr	r2, [r3, #8]
	ldr	r3, [r3, #12]
	str	r2, [sp, #60]
	mov	r2, #0
	str	r3, [sp, #64]
	bl	FlashReadPages
	ldr	r3, [sp, #52]
	cmn	r3, #1
	ldreq	r3, [r4, #2708]
	addeq	r3, r3, #1
	streq	r3, [r4, #2708]
	beq	.L1361
	ldr	r3, [r8, #8]
	cmp	r3, r6
	beq	.L1362
	ldr	r3, [r4, #2708]
	mov	r2, r6
	ldr	r0, .L1386+28
	add	r3, r3, #1
	str	r3, [r4, #2708]
	ldr	r1, [r8, #8]
	bl	printk
.L1362:
	ldr	r3, [r8, #8]
	cmp	r3, r6
	beq	.L1361
	ldr	r1, .L1386+20
	movw	r2, #1100
	ldr	r0, .L1386+16
	bl	printk
	b	.L1361
.L1359:
	ldr	r3, [r4, #3424]
	mov	r1, #0
	ldrh	r2, [r4, #106]
	add	r3, r3, r5
	ldr	r0, [r3, #8]
	bl	ftl_memset
.L1361:
	cmp	fp, #0
	ldr	r3, [r4, #3424]
	ldreqh	r1, [r4, #54]
	add	r5, r3, r5
	ldrne	r1, [sp, #20]
	ldreq	r2, [sp, #8]
	muleq	r1, r1, r6
	ldrne	r0, [r5, #8]
	ldreq	r0, [r5, #8]
	addne	r0, r0, r1, asl #9
	ldrne	r1, [sp, #8]
	rsbeq	r1, r9, r1
	addeq	r1, r2, r1, asl #9
	mov	r2, r7, asl #9
	bl	memcpy
	b	.L1356
.L1351:
	ldr	r3, [r4, #3424]
	add	r5, r3, r5
	ldrh	r3, [r4, #54]
	mul	r3, r3, r6
.L1383:
	rsb	r3, r9, r3
	ldr	r0, [sp, #8]
	add	r3, r0, r3, asl #9
.L1384:
	str	r3, [r5, #8]
.L1356:
	ldrb	r2, [sl, #6]	@ zero_extendqisi2
	ldrh	r3, [r4, #28]
	cmp	r2, r3
	bcc	.L1364
	ldr	r0, .L1386+16
	movw	r2, #1115
	ldr	r1, .L1386+20
	bl	printk
.L1364:
	ldr	r3, .L1386+32
	ldr	r0, .L1386
	ldr	r2, [sp, #40]
	ldr	r1, [sp, #36]
	strh	r3, [r2, r1]	@ movhi
	ldr	r3, [r0, #2544]
	str	r6, [r8, #8]
	add	r6, r6, #1
	str	r3, [r8, #4]
	add	r3, r3, #1
	cmn	r3, #1
	str	r3, [r0, #2544]
	moveq	r3, #0
	streq	r3, [r4, #2544]
	ldr	r3, [sp, #48]
	str	r3, [r8, #12]
	ldrh	r3, [sl, #0]
	strh	r3, [r8, #2]	@ movhi
	ldr	r3, [sp, #4]
	add	r3, r3, #1
	str	r3, [sp, #4]
.L1349:
	ldr	r3, [sp, #4]
	ldr	r0, [sp, #12]
	cmp	r3, r0
	bne	.L1366
	ldr	fp, [sp, #44]
	mov	r5, sl
	b	.L1350
.L1377:
	ldr	r3, [sp, #4]
	mov	r5, sl
	ldr	fp, [sp, #44]
	str	r3, [sp, #12]
.L1350:
	ldr	r0, [r4, #3424]
	mov	r2, #0
	ldr	r1, [sp, #12]
	mov	r3, r5
	bl	FtlProgPages
	ldr	r0, [sp, #12]
	cmp	fp, r0
	bcs	.L1367
	ldr	r0, .L1386+16
	movw	r2, #1124
	ldr	r1, .L1386+20
	bl	printk
.L1367:
	ldr	r1, [sp, #12]
	rsb	fp, r1, fp
.L1342:
	cmp	fp, #0
	ldr	r8, .L1386
	bne	.L1368
	mov	r0, fp
	ldr	r1, [sp, #32]
	bl	rk_ftl_garbage_collect
	movw	r3, #2340
	ldrh	r3, [r8, r3]
	cmp	r3, #15
	movhi	r0, fp
	bhi	.L1340
	movw	r5, #2588
	mov	r4, r8
.L1380:
	ldrh	r3, [r8, r5]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1369
	movw	r2, #3202
	ldrh	r2, [r8, r2]
	cmp	r2, r3
	bne	.L1369
	mov	r0, #0
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L1369:
	movw	r2, #3150
	mov	r3, #128
	mov	r0, #1
	strh	r3, [r4, r2]	@ movhi
	movw	r2, #3148
	mov	r1, r0
	strh	r3, [r4, r2]	@ movhi
	bl	rk_ftl_garbage_collect
	mov	r0, #0
	mov	r1, #1
	bl	rk_ftl_garbage_collect
	movw	r3, #2340
	ldrh	r3, [r4, r3]
	cmp	r3, #8
	bls	.L1380
	mov	r0, #0
.L1340:
	add	sp, sp, #76
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1387:
	.align	2
.L1386:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR3
	.word	.LANCHOR0+2344
	.word	.LC1
	.word	.LANCHOR1+606
	.word	.LANCHOR0+2392
	.word	.LC110
	.word	-3947
	.fnend
	.size	FtlWrite, .-FtlWrite
	.align	2
	.global	sftl_write
	.type	sftl_write, %function
sftl_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	ip, r1
	mov	r3, r2
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	mov	r1, r4
	mov	r0, #0
	mov	r2, ip
	ldmfd	sp!, {r4, lr}
	b	FtlWrite
	.fnend
	.size	sftl_write, .-sftl_write
	.align	2
	.global	FtlRead
	.type	FtlRead, %function
FtlRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	cmp	r0, #16
	.pad #60
	sub	sp, sp, #60
	mov	r4, r1
	mov	sl, r3
	str	r2, [sp, #24]
	bne	.L1390
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r3
	bl	FtlVendorPartRead
	str	r0, [sp, #4]
	b	.L1391
.L1390:
	ldr	r3, [sp, #24]
	ldr	r5, .L1421
	add	r3, r3, r1
	str	r3, [sp, #12]
	ldr	ip, [sp, #12]
	ldr	r3, [r5, #136]
	cmp	ip, r3
	mvnhi	ip, #0
	strhi	ip, [sp, #4]
	bhi	.L1391
	ldr	r3, .L1421+4
	ldr	r3, [r3, #0]
	cmn	r3, #1
	streq	r3, [sp, #4]
	beq	.L1391
	ldrh	r6, [r5, #54]
	mov	r0, r1
	mov	r8, #0
	mov	r1, r6
	bl	__aeabi_uidiv
	ldr	r3, [sp, #12]
	mov	r1, r6
	mov	r6, r8
	str	r0, [sp, #16]
	sub	r0, r3, #1
	bl	__aeabi_uidiv
	ldr	ip, [sp, #16]
	ldr	r7, [sp, #16]
	rsb	r3, ip, #1
	ldr	ip, [sp, #24]
	str	r8, [sp, #32]
	str	r8, [sp, #28]
	str	r8, [sp, #4]
	add	r3, r3, r0
	str	r3, [sp, #8]
	ldr	r3, [r5, #2536]
	str	r0, [sp, #20]
	add	r3, ip, r3
	ldr	ip, [sp, #8]
	str	r3, [r5, #2536]
	ldr	r3, [r5, #2508]
	add	r3, ip, r3
	str	r3, [r5, #2508]
	b	.L1417
.L1409:
	mov	r0, r7
	add	r1, sp, #52
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp, #52]
	cmn	r3, #1
	bne	.L1418
	b	.L1420
.L1396:
	mla	r0, r0, r7, r9
	cmp	r0, r4
	bcc	.L1395
	ldr	r3, [sp, #12]
	cmp	r0, r3
	bcs	.L1395
	rsb	r0, r4, r0
	mov	r1, #0
	mov	r2, #512
	add	r0, sl, r0, asl #9
	bl	ftl_memset
.L1395:
	add	r9, r9, #1
	b	.L1393
.L1420:
	mov	r9, #0
.L1393:
	ldrh	r0, [r5, #54]
	cmp	r9, r0
	bcc	.L1396
	b	.L1397
.L1418:
	ldr	r2, [r5, #3416]
	mov	r9, #20
	ldr	ip, [sp, #16]
	ldrh	fp, [r5, #54]
	mla	r9, r9, r6, r2
	cmp	r7, ip
	str	r3, [r9, #4]
	bne	.L1398
	ldr	r3, [r5, #3316]
	mov	r0, r4
	mov	r1, fp
	str	r3, [r9, #8]
	bl	__aeabi_uidivmod
	ldr	ip, [sp, #24]
	rsb	r3, r1, fp
	str	r1, [sp, #28]
	cmp	r3, ip
	movcc	ip, r3
	str	ip, [sp, #32]
	cmp	ip, fp
	streq	sl, [r9, #8]
	b	.L1399
.L1398:
	ldr	r3, [sp, #20]
	cmp	r7, r3
	bne	.L1400
	ldr	r3, [r5, #3308]
	ldr	ip, [sp, #12]
	str	r3, [r9, #8]
	mul	r3, fp, r7
	rsb	r8, r3, ip
	cmp	r8, fp
	rsbeq	r3, r4, r3
	addeq	r3, sl, r3, asl #9
	streq	r3, [r9, #8]
	b	.L1399
.L1400:
	mul	fp, fp, r7
	rsb	fp, r4, fp
	add	fp, sl, fp, asl #9
	str	fp, [r9, #8]
.L1399:
	ldrh	r3, [r5, #108]
	ldr	r2, [r5, #3312]
	str	r7, [r9, #16]
	mul	r3, r3, r6
	add	r6, r6, #1
	bic	r3, r3, #3
	add	r3, r2, r3
	str	r3, [r9, #12]
.L1397:
	ldr	r3, [sp, #8]
	add	r7, r7, #1
	subs	r3, r3, #1
	str	r3, [sp, #8]
	beq	.L1401
	ldrh	r3, [r5, #28]
	cmp	r6, r3, asl #2
	bne	.L1417
.L1401:
	cmp	r6, #0
	beq	.L1417
	ldr	r0, [r5, #3416]
	mov	r1, r6
	mov	r2, #0
	mov	fp, #0
	bl	FlashReadPages
	ldr	ip, [sp, #28]
	ldr	r3, [sp, #32]
	mov	r9, r7
	mov	ip, ip, asl #9
	str	ip, [sp, #36]
	mov	r3, r3, asl #9
	mov	ip, r8, asl #9
	str	r3, [sp, #40]
	str	ip, [sp, #44]
.L1408:
	mov	r1, #20
	ldr	r3, [r5, #3416]
	mul	r7, r1, fp
	ldr	ip, [sp, #16]
	add	r3, r3, r7
	ldr	r2, [r3, #16]
	cmp	r2, ip
	bne	.L1403
	ldr	r1, [r3, #8]
	ldr	r3, [r5, #3316]
	cmp	r1, r3
	bne	.L1404
	ldr	r3, [sp, #36]
	mov	r0, sl
	ldr	r2, [sp, #40]
	add	r1, r1, r3
	b	.L1419
.L1403:
	ldr	ip, [sp, #20]
	cmp	r2, ip
	bne	.L1404
	ldr	r1, [r3, #8]
	ldr	r3, [r5, #3308]
	cmp	r1, r3
	bne	.L1404
	ldrh	r0, [r5, #54]
	ldr	r2, [sp, #44]
	mul	r0, r0, ip
	rsb	r0, r4, r0
	add	r0, sl, r0, asl #9
.L1419:
	bl	memcpy
.L1404:
	ldr	r2, [r5, #3416]
	add	r3, r2, r7
	ldr	r1, [r3, #12]
	ldr	r0, [r3, #16]
	ldr	r1, [r1, #8]
	cmp	r0, r1
	ldrne	r1, [r5, #2708]
	addne	r1, r1, #1
	strne	r1, [r5, #2708]
	ldr	r1, [r2, r7]
	cmn	r1, #1
	streq	r1, [sp, #4]
	ldreq	ip, .L1421
	ldreq	r0, [ip, #2708]
	addeq	r0, r0, #1
	streq	r0, [ip, #2708]
	ldr	r2, [r2, r7]
	cmp	r2, #256
	bne	.L1407
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L1407:
	add	fp, fp, #1
	cmp	fp, r6
	bne	.L1408
	mov	r7, r9
	mov	r6, #0
.L1417:
	ldr	r3, [sp, #8]
	cmp	r3, #0
	bne	.L1409
	ldr	r3, .L1421
	movw	r2, #3206
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	bne	.L1410
	movw	r2, #2340
	ldrh	r3, [r3, r2]
	cmp	r3, #31
	bhi	.L1391
.L1410:
	mov	r0, #0
	mov	r1, #1
	bl	rk_ftl_garbage_collect
.L1391:
	ldr	r0, [sp, #4]
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1422:
	.align	2
.L1421:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlRead, .-FtlRead
	.align	2
	.global	sftl_read
	.type	sftl_read, %function
sftl_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	ip, r1
	mov	r3, r2
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	mov	r1, r4
	mov	r0, #0
	mov	r2, ip
	ldmfd	sp!, {r4, lr}
	b	FtlRead
	.fnend
	.size	sftl_read, .-sftl_read
	.align	2
	.global	ftl_memcpy
	.type	ftl_memcpy, %function
ftl_memcpy:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	memcpy
	.fnend
	.size	ftl_memcpy, .-ftl_memcpy
	.align	2
	.global	ftl_memcmp
	.type	ftl_memcmp, %function
ftl_memcmp:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	memcmp
	.fnend
	.size	ftl_memcmp, .-ftl_memcmp
	.global	g_nand_ops
	.global	g_nand_phy_info
	.global	gc_ink_free_return_value
	.global	check_vpc_table
	.global	FtlUpdateVaildLpnCount
	.global	g_ect_tbl_power_up_flush
	.global	power_up_flag
	.global	gFtlInitStatus
	.global	DeviceCapacity
	.global	g_power_lost_recovery_flag
	.global	c_mlc_erase_count_value
	.global	g_recovery_ppa_tbl
	.global	g_recovery_page_min_ver
	.global	g_recovery_page_num
	.global	sftl_nand_check_spare_buf
	.global	sftl_temp_buf
	.global	sftl_nand_check_buf
	.global	g_cur_erase_blk
	.global	g_gc_skip_write_count
	.global	g_gc_head_data_block_count
	.global	g_gc_head_data_block
	.global	g_ftl_nand_free_count
	.global	g_in_swl_replace
	.global	g_in_gc_progress
	.global	g_max_erase_count
	.global	g_totle_sys_slc_erase_count
	.global	g_totle_slc_erase_count
	.global	g_min_erase_count
	.global	g_totle_avg_erase_count
	.global	g_totle_mlc_erase_count
	.global	g_totle_l2p_write_count
	.global	g_totle_cache_write_count
	.global	g_tmp_data_superblock_id
	.global	g_totle_read_page_count
	.global	g_totle_discard_page_count
	.global	g_totle_read_sector
	.global	g_totle_write_sector
	.global	g_totle_write_page_count
	.global	g_totle_gc_page_count
	.global	g_gc_blk_index
	.global	g_gc_merge_free_blk_threshold
	.global	g_gc_free_blk_threshold
	.global	g_gc_bad_block_temp_tbl
	.global	g_gc_bad_block_gc_index
	.global	g_gc_bad_block_temp_num
	.global	g_gc_next_blk_1
	.global	g_gc_next_blk
	.global	g_gc_cur_blk_max_valid_pages
	.global	g_gc_cur_blk_valid_pages
	.global	g_gc_page_offset
	.global	g_gc_blk_num
	.global	p_gc_blk_tbl
	.global	p_gc_page_info
	.global	g_sys_ext_data
	.global	g_sys_save_data
	.global	gp_last_act_superblock
	.global	g_gc_superblock
	.global	g_gc_temp_superblock
	.global	g_buffer_superblock
	.global	g_active_superblock
	.global	g_num_data_superblocks
	.global	g_num_free_superblocks
	.global	p_data_block_list_tail
	.global	p_data_block_list_head
	.global	p_free_data_block_list_head
	.global	p_data_block_list_table
	.global	g_l2p_last_update_region_id
	.global	p_l2p_map_buf
	.global	p_l2p_ram_map
	.global	g_totle_vendor_block
	.global	p_vendor_region_ppn_table
	.global	p_vendor_block_ver_table
	.global	p_vendor_block_valid_page_count
	.global	p_vendor_block_table
	.global	g_totle_map_block
	.global	p_map_region_ppn_table
	.global	p_map_block_ver_table
	.global	p_map_block_valid_page_count
	.global	p_map_block_table
	.global	p_blk_mode_table
	.global	p_valid_page_count_check_table
	.global	p_valid_page_count_table
	.global	g_totle_swl_count
	.global	p_swl_mul_table
	.global	p_erase_count_table
	.global	g_ect_tbl_info_size
	.global	gp_ect_tbl_info
	.global	g_gc_num_req
	.global	c_gc_page_buf_num
	.global	gp_gc_page_buf_info
	.global	p_gc_data_buf
	.global	p_gc_spare_buf
	.global	p_io_spare_buf
	.global	p_io_data_buf_1
	.global	p_io_data_buf_0
	.global	p_sys_spare_buf
	.global	p_vendor_data_buf
	.global	p_sys_data_buf_1
	.global	p_sys_data_buf
	.global	p_plane_order_table
	.global	req_gc_dst
	.global	req_gc
	.global	req_erase
	.global	req_prgm
	.global	req_read
	.global	req_sys
	.global	gVendorBlkInfo
	.global	gL2pMapInfo
	.global	gSysFreeQueue
	.global	gSysInfo
	.global	gBbtInfo
	.global	g_MaxLbn
	.global	g_VaildLpn
	.global	g_MaxLpn
	.global	g_MaxLbaSector
	.global	g_GlobalDataVersion
	.global	g_GlobalSysVersion
	.global	ftl_gc_temp_power_lost_recovery_flag
	.global	c_ftl_nand_max_data_blks
	.global	c_ftl_nand_data_op_blks_per_plane
	.global	c_ftl_nand_data_blks_per_plane
	.global	c_ftl_nand_max_sys_blks
	.global	c_ftl_nand_init_sys_blks_per_plane
	.global	c_ftl_nand_sys_blks_per_plane
	.global	c_ftl_vendor_part_size
	.global	c_ftl_nand_max_vendor_blks
	.global	c_ftl_nand_max_map_blks
	.global	c_ftl_nand_map_blks_per_plane
	.global	c_ftl_nand_vendor_region_num
	.global	c_ftl_nand_l2pmap_ram_region_num
	.global	c_ftl_nand_map_region_num
	.global	c_ftl_nand_totle_phy_blks
	.global	c_ftl_nand_reserved_blks
	.global	c_ftl_nand_byte_pre_oob
	.global	c_ftl_nand_byte_pre_page
	.global	c_ftl_nand_sec_pre_page_shift
	.global	c_ftl_nand_sec_pre_page
	.global	c_ftl_nand_page_pre_super_blk
	.global	c_ftl_nand_page_pre_slc_blk
	.global	c_ftl_nand_page_pre_blk
	.global	c_ftl_nand_bbm_buf_size
	.global	c_ftl_nand_ext_blk_pre_plane
	.global	c_ftl_nand_blk_pre_plane
	.global	c_ftl_nand_planes_num
	.global	c_ftl_nand_blks_per_die_shift
	.global	c_ftl_nand_blks_per_die
	.global	c_ftl_nand_planes_per_die
	.global	c_ftl_nand_die_num
	.global	c_ftl_nand_type
	.section	.rodata
	.set	.LANCHOR1,. + 0
	.type	__func__.3643, %object
	.size	__func__.3643, 17
__func__.3643:
	.ascii	"INSERT_DATA_LIST\000"
	.type	__func__.3638, %object
	.size	__func__.3638, 17
__func__.3638:
	.ascii	"INSERT_FREE_LIST\000"
	.type	__func__.3674, %object
	.size	__func__.3674, 17
__func__.3674:
	.ascii	"List_remove_node\000"
	.type	__func__.3706, %object
	.size	__func__.3706, 22
__func__.3706:
	.ascii	"List_update_data_list\000"
	.type	__func__.3813, %object
	.size	__func__.3813, 22
__func__.3813:
	.ascii	"select_l2p_ram_region\000"
	.type	__func__.4136, %object
	.size	__func__.4136, 16
__func__.4136:
	.ascii	"make_superblock\000"
	.type	__func__.4312, %object
	.size	__func__.4312, 19
__func__.4312:
	.ascii	"get_new_active_ppa\000"
	.type	__func__.4557, %object
	.size	__func__.4557, 15
__func__.4557:
	.ascii	"FlashReadPages\000"
	.type	__func__.4575, %object
	.size	__func__.4575, 15
__func__.4575:
	.ascii	"FlashProgPages\000"
	.type	__func__.4598, %object
	.size	__func__.4598, 17
__func__.4598:
	.ascii	"FlashEraseBlocks\000"
	.type	__func__.3748, %object
	.size	__func__.3748, 26
__func__.3748:
	.ascii	"ftl_map_blk_alloc_new_blk\000"
	.type	__func__.4157, %object
	.size	__func__.4157, 18
__func__.4157:
	.ascii	"SupperBlkListInit\000"
	.type	__func__.3523, %object
	.size	__func__.3523, 11
__func__.3523:
	.ascii	"FtlLoadBbt\000"
	.type	__func__.3480, %object
	.size	__func__.3480, 14
__func__.3480:
	.ascii	"FtlBbt2Bitmap\000"
	.type	__func__.3264, %object
	.size	__func__.3264, 11
__func__.3264:
	.ascii	"FtlMemInit\000"
	.type	__func__.3925, %object
	.size	__func__.3925, 14
__func__.3925:
	.ascii	"FtlScanSysBlk\000"
	.type	__func__.3774, %object
	.size	__func__.3774, 31
__func__.3774:
	.ascii	"Ftl_write_map_blk_to_last_page\000"
	.type	__func__.3788, %object
	.size	__func__.3788, 16
__func__.3788:
	.ascii	"FtlMapWritePage\000"
	.type	__func__.3759, %object
	.size	__func__.3759, 15
__func__.3759:
	.ascii	"ftl_map_blk_gc\000"
	.type	__func__.3715, %object
	.size	__func__.3715, 16
__func__.3715:
	.ascii	"load_l2p_region\000"
	.type	__func__.3830, %object
	.size	__func__.3830, 9
__func__.3830:
	.ascii	"log2phys\000"
	.type	__func__.4045, %object
	.size	__func__.4045, 16
__func__.4045:
	.ascii	"FtlReUsePrevPpa\000"
	.type	__func__.4198, %object
	.size	__func__.4198, 14
__func__.4198:
	.ascii	"ftl_check_vpc\000"
	.type	__func__.3982, %object
	.size	__func__.3982, 15
__func__.3982:
	.ascii	"FtlLoadSysInfo\000"
	.type	__func__.3903, %object
	.size	__func__.3903, 15
__func__.3903:
	.ascii	"FtlVpcTblFlush\000"
	.type	__func__.4325, %object
	.size	__func__.4325, 16
__func__.4325:
	.ascii	"update_vpc_list\000"
	.type	__func__.4332, %object
	.size	__func__.4332, 20
__func__.4332:
	.ascii	"decrement_vpc_count\000"
	.type	__func__.4079, %object
	.size	__func__.4079, 22
__func__.4079:
	.ascii	"FtlRecoverySuperblock\000"
	.type	__func__.4284, %object
	.size	__func__.4284, 25
__func__.4284:
	.ascii	"allocate_data_superblock\000"
	.type	__func__.4402, %object
	.size	__func__.4402, 19
__func__.4402:
	.ascii	"FtlGcFreeTempBlock\000"
	.type	__func__.4513, %object
	.size	__func__.4513, 23
__func__.4513:
	.ascii	"rk_ftl_garbage_collect\000"
	.type	__func__.4305, %object
	.size	__func__.4305, 29
__func__.4305:
	.ascii	"allocate_new_data_superblock\000"
	.type	__func__.3388, %object
	.size	__func__.3388, 13
__func__.3388:
	.ascii	"FtlProgPages\000"
	.type	__func__.4182, %object
	.size	__func__.4182, 21
__func__.4182:
	.ascii	"FtlVpcCheckAndModify\000"
	.type	__func__.3416, %object
	.size	__func__.3416, 9
__func__.3416:
	.ascii	"FtlWrite\000"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.ascii	"phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\012"
	.ascii	"\000"
.LC1:
	.ascii	"\012!!!!! error @ func:%s - line:%d\012\000"
.LC2:
	.ascii	"FLASH INFO:\012\000"
.LC3:
	.ascii	"Device Capacity: %d MB\012\000"
.LC4:
	.ascii	"FTL INFO:\012\000"
.LC5:
	.ascii	"g_MaxLpn = 0x%x\012\000"
.LC6:
	.ascii	"g_VaildLpn = 0x%x\012\000"
.LC7:
	.ascii	"read_page_count = 0x%x\012\000"
.LC8:
	.ascii	"discard_page_count = 0x%x\012\000"
.LC9:
	.ascii	"write_page_count = 0x%x\012\000"
.LC10:
	.ascii	"cache_write_count = 0x%x\012\000"
.LC11:
	.ascii	"l2p_write_count = 0x%x\012\000"
.LC12:
	.ascii	"gc_page_count = 0x%x\012\000"
.LC13:
	.ascii	"totle_write = %d MB\012\000"
.LC14:
	.ascii	"totle_read = %d MB\012\000"
.LC15:
	.ascii	"GSV = 0x%x\012\000"
.LC16:
	.ascii	"GDV = 0x%x\012\000"
.LC17:
	.ascii	"bad blk num = %d\012\000"
.LC18:
	.ascii	"free_superblocks = 0x%x\012\000"
.LC19:
	.ascii	"mlc_EC = 0x%x\012\000"
.LC20:
	.ascii	"slc_EC = 0x%x\012\000"
.LC21:
	.ascii	"avg_EC = 0x%x\012\000"
.LC22:
	.ascii	"sys_EC = 0x%x\012\000"
.LC23:
	.ascii	"max_EC = 0x%x\012\000"
.LC24:
	.ascii	"min_EC = 0x%x\012\000"
.LC25:
	.ascii	"PLT = 0x%x\012\000"
.LC26:
	.ascii	"POT = 0x%x\012\000"
.LC27:
	.ascii	"MaxSector = 0x%x\012\000"
.LC28:
	.ascii	"init_sys_blks_pp = 0x%x\012\000"
.LC29:
	.ascii	"sys_blks_pp = 0x%x\012\000"
.LC30:
	.ascii	"free sysblock = 0x%x\012\000"
.LC31:
	.ascii	"data_blks_pp = 0x%x\012\000"
.LC32:
	.ascii	"data_op_blks_pp = 0x%x\012\000"
.LC33:
	.ascii	"max_data_blks = 0x%x\012\000"
.LC34:
	.ascii	"Sys.id = 0x%x\012\000"
.LC35:
	.ascii	"Bbt.id = 0x%x\012\000"
.LC36:
	.ascii	"ACT.page = 0x%x\012\000"
.LC37:
	.ascii	"ACT.plane = 0x%x\012\000"
.LC38:
	.ascii	"ACT.id = 0x%x\012\000"
.LC39:
	.ascii	"ACT.mode = 0x%x\012\000"
.LC40:
	.ascii	"ACT.a_pages = 0x%x\012\000"
.LC41:
	.ascii	"ACT VPC = 0x%x\012\000"
.LC42:
	.ascii	"BUF.page = 0x%x\012\000"
.LC43:
	.ascii	"BUF.plane = 0x%x\012\000"
.LC44:
	.ascii	"BUF.id = 0x%x\012\000"
.LC45:
	.ascii	"BUF.mode = 0x%x\012\000"
.LC46:
	.ascii	"BUF.a_pages = 0x%x\012\000"
.LC47:
	.ascii	"BUF VPC = 0x%x\012\000"
.LC48:
	.ascii	"TMP.page = 0x%x\012\000"
.LC49:
	.ascii	"TMP.plane = 0x%x\012\000"
.LC50:
	.ascii	"TMP.id = 0x%x\012\000"
.LC51:
	.ascii	"TMP.mode = 0x%x\012\000"
.LC52:
	.ascii	"TMP.a_pages = 0x%x\012\000"
.LC53:
	.ascii	"GC.page = 0x%x\012\000"
.LC54:
	.ascii	"GC.plane = 0x%x\012\000"
.LC55:
	.ascii	"GC.id = 0x%x\012\000"
.LC56:
	.ascii	"GC.mode = 0x%x\012\000"
.LC57:
	.ascii	"GC.a_pages = 0x%x\012\000"
.LC58:
	.ascii	"WR_CHK = %x %x %x\012\000"
.LC59:
	.ascii	"Read Err Cnt = 0x%x\012\000"
.LC60:
	.ascii	"Prog Err Cnt = 0x%x\012\000"
.LC61:
	.ascii	"gc_free_blk_th= 0x%x\012\000"
.LC62:
	.ascii	"gc_merge_free_blk_th= 0x%x\012\000"
.LC63:
	.ascii	"gc_skip_write_count= 0x%x\012\000"
.LC64:
	.ascii	"gc_blk_index= 0x%x\012\000"
.LC65:
	.ascii	"free min EC= 0x%x\012\000"
.LC66:
	.ascii	"free max EC= 0x%x\012\000"
.LC67:
	.ascii	"GC__SB VPC = 0x%x\012\000"
.LC68:
	.ascii	"%d. [0x%x]=0x%x 0x%x  0x%x\012\000"
.LC69:
	.ascii	"free %d. [0x%x] 0x%x  0x%x\012\000"
.LC70:
	.ascii	"%s\012\000"
.LC71:
	.ascii	"SFTL version: 5.0.48 20180922\000"
.LC72:
	.ascii	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x"
	.ascii	"\012\000"
.LC73:
	.ascii	"FtlGcRefreshBlock  0x%x\012\000"
.LC74:
	.ascii	"FtlGcMarkBadPhyBlk %d 0x%x\012\000"
.LC75:
	.ascii	"%s: addr: %x is in id block!!!!!!!!!!\012\000"
.LC76:
	.ascii	"not free: w: d:\000"
.LC77:
	.ascii	"not free: w: s:\000"
.LC78:
	.ascii	"prog read error: = %x\012\000"
.LC79:
	.ascii	"prog read s error: = %x %x %x\012\000"
.LC80:
	.ascii	"prog read d error: = %x %x %x\012\000"
.LC81:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d, error\012"
	.ascii	"\000"
.LC82:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d\012\000"
.LC83:
	.ascii	"%s error allocating memory. return -1\012\000"
.LC84:
	.ascii	"FtlMapWritePage error = %x \012\000"
.LC85:
	.ascii	"FtlMapWritePage error = %x error count = %d\012\000"
.LC86:
	.ascii	"region_id = %x phyAddr = %x\012\000"
.LC87:
	.ascii	"spare:\000"
.LC88:
	.ascii	"map_ppn:\000"
.LC89:
	.ascii	"load_l2p_region refresh = %x phyAddr = %x\012\000"
.LC90:
	.ascii	"FtlGcScanTempBlk Error ID %x %x!!!!!!!spor\012\000"
.LC91:
	.ascii	"FtlGcScanTempBlk Error ID %x %x!!!!!!! \012\000"
.LC92:
	.ascii	"ftl_scan_all_data = %x\012\000"
.LC93:
	.ascii	"scan lpa = %x ppa= %x\012\000"
.LC94:
	.ascii	"lba = %x,addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC95:
	.ascii	"...%s enter...\012\000"
.LC96:
	.ascii	"FtlCheckVpc2 %x = %x  %x\012\000"
.LC97:
	.ascii	"free blk vpc error %x = %x  %x\012\000"
.LC98:
	.ascii	"FtlVpcTblFlush error = %x error count = %d\012\000"
.LC99:
	.ascii	"FtlVendorPartRead refresh = %x phyAddr = %x\012\000"
.LC100:
	.ascii	"no ect\000"
.LC101:
	.ascii	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\012\000"
.LC102:
	.ascii	"FtlBbmTblFlush error:%x\012\000"
.LC103:
	.ascii	"FtlBbmTblFlush error = %x error count = %d\012\000"
.LC104:
	.ascii	"FtlGcFreeBadSuperBlk 0x%x\012\000"
.LC105:
	.ascii	"decrement_vpc_count %x = %d\012\000"
.LC106:
	.ascii	"spuer block %x vpn is 0\012 \000"
.LC107:
	.ascii	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\012\000"
.LC108:
	.ascii	"Ftlwrite decrement_vpc_count %x = %d\012\000"
.LC109:
	.ascii	"FtlCheckVpc %x = %x  %x\012\000"
.LC110:
	.ascii	"FtlWrite: lpa error:%x %x\012\000"
	.data
	.align	2
	.set	.LANCHOR2,. + 0
	.type	gFtlInitStatus, %object
	.size	gFtlInitStatus, 4
gFtlInitStatus:
	.word	-1
	.type	ftl_gc_temp_block_bops_scan_page_addr, %object
	.size	ftl_gc_temp_block_bops_scan_page_addr, 2
ftl_gc_temp_block_bops_scan_page_addr:
	.short	-1
	.space	2
	.type	power_up_flag, %object
	.size	power_up_flag, 4
power_up_flag:
	.word	1
	.bss
	.align	2
	.set	.LANCHOR0,. + 0
	.set	.LANCHOR3,. + 16368
	.type	g_nand_phy_info, %object
	.size	g_nand_phy_info, 24
g_nand_phy_info:
	.space	24
	.type	c_ftl_nand_sys_blks_per_plane, %object
	.size	c_ftl_nand_sys_blks_per_plane, 4
c_ftl_nand_sys_blks_per_plane:
	.space	4
	.type	c_ftl_nand_planes_num, %object
	.size	c_ftl_nand_planes_num, 2
c_ftl_nand_planes_num:
	.space	2
	.space	2
	.type	c_ftl_nand_max_sys_blks, %object
	.size	c_ftl_nand_max_sys_blks, 4
c_ftl_nand_max_sys_blks:
	.space	4
	.type	c_ftl_nand_data_blks_per_plane, %object
	.size	c_ftl_nand_data_blks_per_plane, 2
c_ftl_nand_data_blks_per_plane:
	.space	2
	.type	c_ftl_nand_blk_pre_plane, %object
	.size	c_ftl_nand_blk_pre_plane, 2
c_ftl_nand_blk_pre_plane:
	.space	2
	.type	c_ftl_nand_max_data_blks, %object
	.size	c_ftl_nand_max_data_blks, 4
c_ftl_nand_max_data_blks:
	.space	4
	.type	c_ftl_nand_totle_phy_blks, %object
	.size	c_ftl_nand_totle_phy_blks, 4
c_ftl_nand_totle_phy_blks:
	.space	4
	.type	c_ftl_nand_type, %object
	.size	c_ftl_nand_type, 2
c_ftl_nand_type:
	.space	2
	.type	c_ftl_nand_die_num, %object
	.size	c_ftl_nand_die_num, 2
c_ftl_nand_die_num:
	.space	2
	.type	c_ftl_nand_planes_per_die, %object
	.size	c_ftl_nand_planes_per_die, 2
c_ftl_nand_planes_per_die:
	.space	2
	.type	c_ftl_nand_sec_pre_page, %object
	.size	c_ftl_nand_sec_pre_page, 2
c_ftl_nand_sec_pre_page:
	.space	2
	.type	p_plane_order_table, %object
	.size	p_plane_order_table, 32
p_plane_order_table:
	.space	32
	.type	c_mlc_erase_count_value, %object
	.size	c_mlc_erase_count_value, 2
c_mlc_erase_count_value:
	.space	2
	.type	c_ftl_nand_ext_blk_pre_plane, %object
	.size	c_ftl_nand_ext_blk_pre_plane, 2
c_ftl_nand_ext_blk_pre_plane:
	.space	2
	.type	c_ftl_vendor_part_size, %object
	.size	c_ftl_vendor_part_size, 2
c_ftl_vendor_part_size:
	.space	2
	.type	c_ftl_nand_blks_per_die, %object
	.size	c_ftl_nand_blks_per_die, 2
c_ftl_nand_blks_per_die:
	.space	2
	.type	c_ftl_nand_blks_per_die_shift, %object
	.size	c_ftl_nand_blks_per_die_shift, 2
c_ftl_nand_blks_per_die_shift:
	.space	2
	.type	c_ftl_nand_page_pre_blk, %object
	.size	c_ftl_nand_page_pre_blk, 2
c_ftl_nand_page_pre_blk:
	.space	2
	.type	c_ftl_nand_page_pre_slc_blk, %object
	.size	c_ftl_nand_page_pre_slc_blk, 2
c_ftl_nand_page_pre_slc_blk:
	.space	2
	.type	c_ftl_nand_page_pre_super_blk, %object
	.size	c_ftl_nand_page_pre_super_blk, 2
c_ftl_nand_page_pre_super_blk:
	.space	2
	.type	c_ftl_nand_sec_pre_page_shift, %object
	.size	c_ftl_nand_sec_pre_page_shift, 2
c_ftl_nand_sec_pre_page_shift:
	.space	2
	.type	c_ftl_nand_byte_pre_page, %object
	.size	c_ftl_nand_byte_pre_page, 2
c_ftl_nand_byte_pre_page:
	.space	2
	.type	c_ftl_nand_byte_pre_oob, %object
	.size	c_ftl_nand_byte_pre_oob, 2
c_ftl_nand_byte_pre_oob:
	.space	2
	.type	c_ftl_nand_reserved_blks, %object
	.size	c_ftl_nand_reserved_blks, 2
c_ftl_nand_reserved_blks:
	.space	2
	.type	DeviceCapacity, %object
	.size	DeviceCapacity, 4
DeviceCapacity:
	.space	4
	.type	c_ftl_nand_max_vendor_blks, %object
	.size	c_ftl_nand_max_vendor_blks, 2
c_ftl_nand_max_vendor_blks:
	.space	2
	.type	c_ftl_nand_vendor_region_num, %object
	.size	c_ftl_nand_vendor_region_num, 2
c_ftl_nand_vendor_region_num:
	.space	2
	.type	c_ftl_nand_map_blks_per_plane, %object
	.size	c_ftl_nand_map_blks_per_plane, 2
c_ftl_nand_map_blks_per_plane:
	.space	2
	.space	2
	.type	c_ftl_nand_max_map_blks, %object
	.size	c_ftl_nand_max_map_blks, 4
c_ftl_nand_max_map_blks:
	.space	4
	.type	c_ftl_nand_init_sys_blks_per_plane, %object
	.size	c_ftl_nand_init_sys_blks_per_plane, 4
c_ftl_nand_init_sys_blks_per_plane:
	.space	4
	.type	c_ftl_nand_map_region_num, %object
	.size	c_ftl_nand_map_region_num, 2
c_ftl_nand_map_region_num:
	.space	2
	.type	c_ftl_nand_l2pmap_ram_region_num, %object
	.size	c_ftl_nand_l2pmap_ram_region_num, 2
c_ftl_nand_l2pmap_ram_region_num:
	.space	2
	.type	g_MaxLbaSector, %object
	.size	g_MaxLbaSector, 4
g_MaxLbaSector:
	.space	4
	.type	g_totle_vendor_block, %object
	.size	g_totle_vendor_block, 2
g_totle_vendor_block:
	.space	2
	.space	2
	.type	p_vendor_block_table, %object
	.size	p_vendor_block_table, 4
p_vendor_block_table:
	.space	4
	.type	gBbtInfo, %object
	.size	gBbtInfo, 60
gBbtInfo:
	.space	60
	.type	gSysFreeQueue, %object
	.size	gSysFreeQueue, 2056
gSysFreeQueue:
	.space	2056
	.type	g_sys_save_data, %object
	.size	g_sys_save_data, 48
g_sys_save_data:
	.space	48
	.type	p_data_block_list_table, %object
	.size	p_data_block_list_table, 4
p_data_block_list_table:
	.space	4
	.type	p_data_block_list_head, %object
	.size	p_data_block_list_head, 4
p_data_block_list_head:
	.space	4
	.type	p_valid_page_count_table, %object
	.size	p_valid_page_count_table, 4
p_valid_page_count_table:
	.space	4
	.type	p_erase_count_table, %object
	.size	p_erase_count_table, 4
p_erase_count_table:
	.space	4
	.type	p_data_block_list_tail, %object
	.size	p_data_block_list_tail, 4
p_data_block_list_tail:
	.space	4
	.type	g_num_data_superblocks, %object
	.size	g_num_data_superblocks, 2
g_num_data_superblocks:
	.space	2
	.space	2
	.type	p_free_data_block_list_head, %object
	.size	p_free_data_block_list_head, 4
p_free_data_block_list_head:
	.space	4
	.type	g_num_free_superblocks, %object
	.size	g_num_free_superblocks, 2
g_num_free_superblocks:
	.space	2
	.space	2
	.type	g_active_superblock, %object
	.size	g_active_superblock, 48
g_active_superblock:
	.space	48
	.type	g_buffer_superblock, %object
	.size	g_buffer_superblock, 48
g_buffer_superblock:
	.space	48
	.type	g_gc_temp_superblock, %object
	.size	g_gc_temp_superblock, 48
g_gc_temp_superblock:
	.space	48
	.type	p_l2p_ram_map, %object
	.size	p_l2p_ram_map, 4
p_l2p_ram_map:
	.space	4
	.type	g_l2p_last_update_region_id, %object
	.size	g_l2p_last_update_region_id, 2
g_l2p_last_update_region_id:
	.space	2
	.type	FtlUpdateVaildLpnCount, %object
	.size	FtlUpdateVaildLpnCount, 2
FtlUpdateVaildLpnCount:
	.space	2
	.type	g_VaildLpn, %object
	.size	g_VaildLpn, 4
g_VaildLpn:
	.space	4
	.type	p_blk_mode_table, %object
	.size	p_blk_mode_table, 4
p_blk_mode_table:
	.space	4
	.type	g_MaxLpn, %object
	.size	g_MaxLpn, 4
g_MaxLpn:
	.space	4
	.type	g_totle_read_page_count, %object
	.size	g_totle_read_page_count, 4
g_totle_read_page_count:
	.space	4
	.type	g_totle_discard_page_count, %object
	.size	g_totle_discard_page_count, 4
g_totle_discard_page_count:
	.space	4
	.type	g_totle_write_page_count, %object
	.size	g_totle_write_page_count, 4
g_totle_write_page_count:
	.space	4
	.type	g_totle_cache_write_count, %object
	.size	g_totle_cache_write_count, 4
g_totle_cache_write_count:
	.space	4
	.type	g_totle_l2p_write_count, %object
	.size	g_totle_l2p_write_count, 4
g_totle_l2p_write_count:
	.space	4
	.type	g_totle_gc_page_count, %object
	.size	g_totle_gc_page_count, 4
g_totle_gc_page_count:
	.space	4
	.type	g_totle_write_sector, %object
	.size	g_totle_write_sector, 4
g_totle_write_sector:
	.space	4
	.type	g_totle_read_sector, %object
	.size	g_totle_read_sector, 4
g_totle_read_sector:
	.space	4
	.type	g_GlobalSysVersion, %object
	.size	g_GlobalSysVersion, 4
g_GlobalSysVersion:
	.space	4
	.type	g_GlobalDataVersion, %object
	.size	g_GlobalDataVersion, 4
g_GlobalDataVersion:
	.space	4
	.type	g_totle_mlc_erase_count, %object
	.size	g_totle_mlc_erase_count, 4
g_totle_mlc_erase_count:
	.space	4
	.type	g_totle_slc_erase_count, %object
	.size	g_totle_slc_erase_count, 4
g_totle_slc_erase_count:
	.space	4
	.type	g_totle_avg_erase_count, %object
	.size	g_totle_avg_erase_count, 4
g_totle_avg_erase_count:
	.space	4
	.type	g_totle_sys_slc_erase_count, %object
	.size	g_totle_sys_slc_erase_count, 4
g_totle_sys_slc_erase_count:
	.space	4
	.type	g_max_erase_count, %object
	.size	g_max_erase_count, 4
g_max_erase_count:
	.space	4
	.type	g_min_erase_count, %object
	.size	g_min_erase_count, 4
g_min_erase_count:
	.space	4
	.type	c_ftl_nand_data_op_blks_per_plane, %object
	.size	c_ftl_nand_data_op_blks_per_plane, 2
c_ftl_nand_data_op_blks_per_plane:
	.space	2
	.space	2
	.type	gSysInfo, %object
	.size	gSysInfo, 12
gSysInfo:
	.space	12
	.type	g_gc_superblock, %object
	.size	g_gc_superblock, 48
g_gc_superblock:
	.space	48
	.type	g_sys_ext_data, %object
	.size	g_sys_ext_data, 512
g_sys_ext_data:
	.space	512
	.type	g_gc_free_blk_threshold, %object
	.size	g_gc_free_blk_threshold, 2
g_gc_free_blk_threshold:
	.space	2
	.type	g_gc_merge_free_blk_threshold, %object
	.size	g_gc_merge_free_blk_threshold, 2
g_gc_merge_free_blk_threshold:
	.space	2
	.type	g_gc_skip_write_count, %object
	.size	g_gc_skip_write_count, 4
g_gc_skip_write_count:
	.space	4
	.type	g_gc_blk_index, %object
	.size	g_gc_blk_index, 2
g_gc_blk_index:
	.space	2
	.space	2
	.type	g_in_swl_replace, %object
	.size	g_in_swl_replace, 4
g_in_swl_replace:
	.space	4
	.type	g_gc_num_req, %object
	.size	g_gc_num_req, 4
g_gc_num_req:
	.space	4
	.type	gp_gc_page_buf_info, %object
	.size	gp_gc_page_buf_info, 4
gp_gc_page_buf_info:
	.space	4
	.type	p_gc_data_buf, %object
	.size	p_gc_data_buf, 4
p_gc_data_buf:
	.space	4
	.type	p_gc_spare_buf, %object
	.size	p_gc_spare_buf, 4
p_gc_spare_buf:
	.space	4
	.type	req_gc, %object
	.size	req_gc, 4
req_gc:
	.space	4
	.type	c_gc_page_buf_num, %object
	.size	c_gc_page_buf_num, 4
c_gc_page_buf_num:
	.space	4
	.type	g_gc_blk_num, %object
	.size	g_gc_blk_num, 2
g_gc_blk_num:
	.space	2
	.space	2
	.type	p_gc_blk_tbl, %object
	.size	p_gc_blk_tbl, 4
p_gc_blk_tbl:
	.space	4
	.type	p_gc_page_info, %object
	.size	p_gc_page_info, 4
p_gc_page_info:
	.space	4
	.type	g_gc_page_offset, %object
	.size	g_gc_page_offset, 2
g_gc_page_offset:
	.space	2
	.type	g_gc_next_blk, %object
	.size	g_gc_next_blk, 2
g_gc_next_blk:
	.space	2
	.type	g_gc_next_blk_1, %object
	.size	g_gc_next_blk_1, 2
g_gc_next_blk_1:
	.space	2
	.type	g_gc_bad_block_temp_num, %object
	.size	g_gc_bad_block_temp_num, 2
g_gc_bad_block_temp_num:
	.space	2
	.type	g_gc_bad_block_temp_tbl, %object
	.size	g_gc_bad_block_temp_tbl, 34
g_gc_bad_block_temp_tbl:
	.space	34
	.type	g_gc_bad_block_gc_index, %object
	.size	g_gc_bad_block_gc_index, 2
g_gc_bad_block_gc_index:
	.space	2
	.type	sftl_nand_check_buf, %object
	.size	sftl_nand_check_buf, 4
sftl_nand_check_buf:
	.space	4
	.type	g_nand_ops, %object
	.size	g_nand_ops, 16
g_nand_ops:
	.space	16
	.type	sftl_temp_buf, %object
	.size	sftl_temp_buf, 4
sftl_temp_buf:
	.space	4
	.type	req_sys, %object
	.size	req_sys, 20
req_sys:
	.space	20
	.type	p_sys_data_buf, %object
	.size	p_sys_data_buf, 4
p_sys_data_buf:
	.space	4
	.type	p_sys_spare_buf, %object
	.size	p_sys_spare_buf, 4
p_sys_spare_buf:
	.space	4
	.type	sftl_nand_check_spare_buf, %object
	.size	sftl_nand_check_spare_buf, 4
sftl_nand_check_spare_buf:
	.space	4
	.type	req_erase, %object
	.size	req_erase, 4
req_erase:
	.space	4
	.type	g_cur_erase_blk, %object
	.size	g_cur_erase_blk, 4
g_cur_erase_blk:
	.space	4
	.type	p_io_data_buf_1, %object
	.size	p_io_data_buf_1, 4
p_io_data_buf_1:
	.space	4
	.type	p_io_spare_buf, %object
	.size	p_io_spare_buf, 4
p_io_spare_buf:
	.space	4
	.type	p_io_data_buf_0, %object
	.size	p_io_data_buf_0, 4
p_io_data_buf_0:
	.space	4
	.type	p_map_block_valid_page_count, %object
	.size	p_map_block_valid_page_count, 4
p_map_block_valid_page_count:
	.space	4
	.type	p_l2p_map_buf, %object
	.size	p_l2p_map_buf, 4
p_l2p_map_buf:
	.space	4
	.type	gL2pMapInfo, %object
	.size	gL2pMapInfo, 40
gL2pMapInfo:
	.space	40
	.type	g_totle_map_block, %object
	.size	g_totle_map_block, 2
g_totle_map_block:
	.space	2
	.space	2
	.type	p_map_block_table, %object
	.size	p_map_block_table, 4
p_map_block_table:
	.space	4
	.type	p_map_block_ver_table, %object
	.size	p_map_block_ver_table, 4
p_map_block_ver_table:
	.space	4
	.type	p_map_region_ppn_table, %object
	.size	p_map_region_ppn_table, 4
p_map_region_ppn_table:
	.space	4
	.type	c_ftl_nand_bbm_buf_size, %object
	.size	c_ftl_nand_bbm_buf_size, 2
c_ftl_nand_bbm_buf_size:
	.space	2
	.type	g_tmp_data_superblock_id, %object
	.size	g_tmp_data_superblock_id, 2
g_tmp_data_superblock_id:
	.space	2
	.type	g_totle_swl_count, %object
	.size	g_totle_swl_count, 4
g_totle_swl_count:
	.space	4
	.type	ftl_gc_temp_power_lost_recovery_flag, %object
	.size	ftl_gc_temp_power_lost_recovery_flag, 4
ftl_gc_temp_power_lost_recovery_flag:
	.space	4
	.type	g_recovery_page_min_ver, %object
	.size	g_recovery_page_min_ver, 4
g_recovery_page_min_ver:
	.space	4
	.type	p_swl_mul_table, %object
	.size	p_swl_mul_table, 4
p_swl_mul_table:
	.space	4
	.type	g_in_gc_progress, %object
	.size	g_in_gc_progress, 4
g_in_gc_progress:
	.space	4
	.type	g_gc_head_data_block, %object
	.size	g_gc_head_data_block, 4
g_gc_head_data_block:
	.space	4
	.type	g_gc_head_data_block_count, %object
	.size	g_gc_head_data_block_count, 4
g_gc_head_data_block_count:
	.space	4
	.type	req_read, %object
	.size	req_read, 4
req_read:
	.space	4
	.type	req_gc_dst, %object
	.size	req_gc_dst, 4
req_gc_dst:
	.space	4
	.type	req_prgm, %object
	.size	req_prgm, 4
req_prgm:
	.space	4
	.type	p_sys_data_buf_1, %object
	.size	p_sys_data_buf_1, 4
p_sys_data_buf_1:
	.space	4
	.type	p_vendor_data_buf, %object
	.size	p_vendor_data_buf, 4
p_vendor_data_buf:
	.space	4
	.type	g_ect_tbl_info_size, %object
	.size	g_ect_tbl_info_size, 2
g_ect_tbl_info_size:
	.space	2
	.space	2
	.type	gp_ect_tbl_info, %object
	.size	gp_ect_tbl_info, 4
gp_ect_tbl_info:
	.space	4
	.type	p_valid_page_count_check_table, %object
	.size	p_valid_page_count_check_table, 4
p_valid_page_count_check_table:
	.space	4
	.type	p_vendor_block_valid_page_count, %object
	.size	p_vendor_block_valid_page_count, 4
p_vendor_block_valid_page_count:
	.space	4
	.type	p_vendor_block_ver_table, %object
	.size	p_vendor_block_ver_table, 4
p_vendor_block_ver_table:
	.space	4
	.type	p_vendor_region_ppn_table, %object
	.size	p_vendor_region_ppn_table, 4
p_vendor_region_ppn_table:
	.space	4
	.type	check_vpc_table, %object
	.size	check_vpc_table, 16384
check_vpc_table:
	.space	16384
	.type	gVendorBlkInfo, %object
	.size	gVendorBlkInfo, 40
gVendorBlkInfo:
	.space	40
	.type	g_MaxLbn, %object
	.size	g_MaxLbn, 4
g_MaxLbn:
	.space	4
	.type	g_ect_tbl_power_up_flush, %object
	.size	g_ect_tbl_power_up_flush, 2
g_ect_tbl_power_up_flush:
	.space	2
	.type	g_power_lost_recovery_flag, %object
	.size	g_power_lost_recovery_flag, 2
g_power_lost_recovery_flag:
	.space	2
	.type	g_recovery_page_num, %object
	.size	g_recovery_page_num, 4
g_recovery_page_num:
	.space	4
	.type	g_recovery_ppa_tbl, %object
	.size	g_recovery_ppa_tbl, 128
g_recovery_ppa_tbl:
	.space	128
	.type	gc_discard_updated, %object
	.size	gc_discard_updated, 4
gc_discard_updated:
	.space	4
	.type	gc_ink_free_return_value, %object
	.size	gc_ink_free_return_value, 2
gc_ink_free_return_value:
	.space	2
	.type	g_gc_cur_blk_valid_pages, %object
	.size	g_gc_cur_blk_valid_pages, 2
g_gc_cur_blk_valid_pages:
	.space	2
	.type	g_gc_cur_blk_max_valid_pages, %object
	.size	g_gc_cur_blk_max_valid_pages, 2
g_gc_cur_blk_max_valid_pages:
	.space	2
	.space	2
	.type	g_ftl_nand_free_count, %object
	.size	g_ftl_nand_free_count, 4
g_ftl_nand_free_count:
	.space	4
	.type	gp_last_act_superblock, %object
	.size	gp_last_act_superblock, 4
gp_last_act_superblock:
	.space	4
