/*
 * Copyright (c) 2016-2017, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2018-04-12
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
	.word	g_nand_phy_info
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
	ldrh	r2, [r3, #4]
	ldrh	r1, [r3, #14]
	str	r0, [r3, #0]
	mul	r2, r2, r0
	rsb	r0, r0, r1
	ldr	r1, [r3, #20]
	strh	r0, [r3, #12]	@ movhi
	mov	r0, #0
	str	r2, [r3, #8]
	rsb	r2, r2, r1
	str	r2, [r3, #16]
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
	strh	r2, [r3, #24]	@ movhi
	moveq	r1, r1, lsr #1
	strh	r5, [r3, #26]	@ movhi
	streqh	r1, [r3, #14]	@ movhi
	moveq	r1, #8
	strh	r0, [r3, #28]	@ movhi
	streqh	r1, [r3, #30]	@ movhi
	mov	r3, #0
.L16:
	ldr	r4, .L21
	add	r1, r4, #32
	strb	r3, [r3, r1]
	add	r3, r3, #1
	cmp	r3, #32
	bne	.L16
	ldrh	r7, [r4, #14]
	cmp	r2, #1
	mul	r5, r5, r0
	mov	r3, #5
	mul	r0, r0, r7
	strh	r3, [r4, #64]	@ movhi
	mov	r8, #640
	streqh	r2, [r4, #64]	@ movhi
	mov	r3, #0
	strh	r8, [r4, #68]	@ movhi
	uxth	r5, r5
	strh	r3, [r4, #66]	@ movhi
	strh	r5, [r4, #4]	@ movhi
	uxth	r0, r0
	strh	r0, [r4, #70]	@ movhi
	bl	Ftl_log2
	ldrh	r9, [sl, #12]
	ldrh	fp, [r4, #30]
	mul	r3, r9, r5
	strh	r9, [r4, #74]	@ movhi
	strh	r9, [r4, #76]	@ movhi
	strh	r3, [r4, #78]	@ movhi
	strh	r0, [r4, #72]	@ movhi
	mov	r0, fp
	bl	Ftl_log2
	mov	r3, fp, asl #9
	mul	r1, r9, fp
	uxth	r3, r3
	strh	r3, [r4, #82]	@ movhi
	mov	r3, r3, lsr #8
	strh	r3, [r4, #84]	@ movhi
	ldrh	r3, [sl, #20]
	strh	r3, [r4, #86]	@ movhi
	mul	r3, r7, r5
	mov	r7, r7, asl #6
	str	r3, [r4, #20]
	mul	r3, fp, r3
	mul	r3, r9, r3
	mov	r3, r3, asr #11
	str	r3, [r4, #88]
	mov	r6, r0
	strh	r0, [r4, #80]	@ movhi
	mov	r0, #5120
	mov	r8, r8, asr r6
	bl	__aeabi_idiv
	add	r6, r6, #9
	mov	r6, r7, asr r6
	mov	r1, r5
	add	r8, r8, #2
	uxth	r6, r6
	uxth	r0, r0
	strh	r0, [r4, #92]	@ movhi
	cmp	r0, #4
	ldr	r4, .L21
	ldrls	r3, .L21
	movls	r2, #4
	strh	r6, [r4, #96]	@ movhi
	strh	r8, [r4, #94]	@ movhi
	strlsh	r2, [r3, #92]	@ movhi
	mul	r3, r5, r6
	ldrh	r0, [r4, #92]
	add	r6, r6, #8
	str	r3, [r4, #100]
	bl	__aeabi_uidiv
	cmp	r5, #1
	uxtah	r6, r6, r0
	str	r6, [r4, #0]
	addeq	r6, r6, #4
	streq	r6, [r4, #0]
	ldrh	r0, [r4, #0]
	bl	FtlSysBlkNumInit
	ldr	r3, [r4, #0]
	ldrh	r2, [r4, #74]
	mov	r0, #0
	str	r0, [r4, #112]
	str	r3, [r4, #104]
	ldr	r3, [r4, #16]
	mov	r3, r3, asl #2
	mul	r3, r2, r3
	ldrh	r2, [r4, #80]
	add	r2, r2, #9
	mov	r3, r3, lsr r2
	add	r3, r3, #2
	strh	r3, [r4, #108]	@ movhi
	mov	r3, #32
	strh	r3, [r4, #110]	@ movhi
	ldrh	r3, [r4, #92]
	add	r3, r3, #3
	strh	r3, [r4, #92]	@ movhi
	ldr	r3, [r4, #100]
	add	r3, r3, #3
	str	r3, [r4, #100]
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
	ldrh	r2, [r3, #116]
	cmp	r2, #0
	ldrneh	r1, [r3, #92]
	ldrne	r2, [r3, #120]
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
	.global	sftl_get_density
	.type	sftl_get_density, %function
sftl_get_density:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L34
	ldr	r0, [r3, #112]
	bx	lr
.L35:
	.align	2
.L34:
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
	ldr	r4, .L37
	ldrh	r6, [r4, #70]
	mov	r1, r6
	bl	__aeabi_uidiv
	uxth	r2, r0
	mls	r3, r6, r2, r5
	add	r1, r4, r2, asl #2
	mov	r6, #1
	ldr	r0, [r1, #152]
	uxth	r3, r3
	and	r1, r3, #31
	mov	ip, r3, lsr #5
	ldr	lr, [r0, ip, asl #2]
	orr	r1, lr, r6, asl r1
	str	r1, [r0, ip, asl #2]
	str	r1, [sp, #0]
	mov	r1, r5
	ldr	r0, .L37+4
	bl	printk
	ldrh	r3, [r4, #130]
	mov	r0, #0
	add	r3, r3, r6
	strh	r3, [r4, #130]	@ movhi
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L38:
	.align	2
.L37:
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
	ldr	r5, .L40
	ldrh	r6, [r5, #70]
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
	ldr	r3, [r5, #152]
	ldr	r0, [r3, r2, asl #2]
	mov	r0, r0, lsr r4
	and	r0, r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L41:
	.align	2
.L40:
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
	ldr	r6, .L47
	b	.L44
.L46:
	mov	r0, r5
	add	r5, r5, #1
	bl	FtlBbmIsBadBlock
	uxth	r5, r5
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
.L44:
	ldrh	r3, [r6, #70]
	ldrh	r2, [r6, #26]
	mul	r3, r2, r3
	cmp	r5, r3
	blt	.L46
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L48:
	.align	2
.L47:
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
	ldr	r5, .L50
	mov	r6, r0
	ldrh	r4, [r5, #28]
	mov	r1, r4
	bl	__aeabi_uidiv
	ldrh	r5, [r5, #70]
	mul	r7, r4, r7
	mov	r1, r4
	mla	r5, r5, r0, r7
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r5, r1
	uxth	r0, r1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L51:
	.align	2
.L50:
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
	ldr	r3, .L53
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldrh	r1, [r3, #70]
	ldrh	r4, [r3, #28]
	bl	__aeabi_uidiv
	mov	r1, r4
	mul	r5, r0, r4
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r5, r1
	uxth	r0, r1
	ldmfd	sp!, {r4, r5, r6, pc}
.L54:
	.align	2
.L53:
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
	ldr	r4, .L56
	ldrh	r1, [r4, #70]
	bl	__aeabi_uidivmod
	uxth	r0, r1
	ldrh	r1, [r4, #28]
	bl	__aeabi_uidiv
	uxth	r0, r0
	ldmfd	sp!, {r4, pc}
.L57:
	.align	2
.L56:
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
	bls	.L59
	rsb	r0, r1, r0
	cmp	r0, #-2147483648
	movhi	r0, #0
	movls	r0, #1
	bx	lr
.L59:
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
	ldr	r3, .L62
	ldrh	r0, [r3, #190]
	rsbs	r0, r0, #1
	movcc	r0, #0
	bx	lr
.L63:
	.align	2
.L62:
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
	ldr	r3, .L65
	ldrh	r0, [r3, #190]
	sub	r3, r0, #1024
	rsbs	r0, r3, #0
	adc	r0, r0, r3
	bx	lr
.L66:
	.align	2
.L65:
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
	ldr	r1, .L70
	movw	r3, #2268
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	mov	r0, #0
	ldrh	r4, [r1, r3]
	ldrh	r2, [r1, #186]
	and	r4, r4, #31
	ldrh	r3, [r1, #188]
	b	.L68
.L69:
	add	ip, r1, r2, asl #1
	add	r2, r2, #1
	add	r0, r0, #1
	ldrh	r5, [ip, #192]
	add	ip, r1, r3, asl #1
	add	r3, r3, #1
	mov	r2, r2, asl #22
	uxth	r0, r0
	mov	r3, r3, asl #22
	mov	r2, r2, lsr #22
	strh	r5, [ip, #192]	@ movhi
	mov	r3, r3, lsr #22
.L68:
	cmp	r0, r4
	ldr	ip, .L70
	blt	.L69
	strh	r2, [ip, #186]	@ movhi
	strh	r3, [ip, #188]	@ movhi
	ldmfd	sp!, {r4, r5, pc}
.L71:
	.align	2
.L70:
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
	ldr	r2, .L78
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrh	r1, [r2, #190]
	cmp	r1, #1024
	ldrneh	ip, [r2, #186]
	movne	r3, #0
	bne	.L74
	b	.L76
.L75:
	add	r4, r3, ip
	mov	r4, r4, asl #22
	add	r4, r2, r4, lsr #21
	ldrh	r4, [r4, #192]
	cmp	r4, r0
	beq	.L77
	add	r3, r3, #1
.L74:
	cmp	r3, r1
	bcc	.L75
.L76:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L77:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L79:
	.align	2
.L78:
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
	ldr	r1, .L95
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldrh	r3, [r1, #12]
	cmp	r3, r0
	bls	.L81
	mov	ip, #6
	ldr	r4, [r1, #2288]
	mul	ip, ip, r0
	mvn	r6, #0
	add	r2, r4, ip
	strh	r6, [r2, #2]	@ movhi
	strh	r6, [r4, ip]	@ movhi
	ldr	r3, [r1, #2292]
	cmp	r3, #0
	beq	.L94
.L82:
	ldr	r7, [r1, #2296]
	mov	r5, r0, asl #1
	ldrh	r1, [r2, #4]
	ldr	sl, .L95+4
	ldrh	r8, [r7, r5]
	cmp	r1, #0
	mulne	r6, r1, r8
	ldr	r8, .L95
	ldr	fp, [r8, #2288]
	ldrh	r9, [r8, #12]
	rsb	r1, fp, r3
	ldr	r8, [r8, #2300]
	mov	r1, r1, asr #1
	add	r5, r8, r5
	str	r5, [sp, #12]
	mov	r5, #0
	mul	r1, sl, r1
	str	r9, [sp, #8]
	str	r8, [sp, #4]
	uxth	r1, r1
.L89:
	add	r5, r5, #1
	ldr	sl, [sp, #8]
	uxth	r5, r5
	cmp	r5, sl
	bhi	.L81
	cmp	r0, r1
	beq	.L81
	mov	r9, r1, asl #1
	ldrh	r8, [r3, #4]
	ldrh	sl, [r7, r9]
	cmp	r8, #0
	mvneq	r8, #0
	mulne	r8, r8, sl
	cmp	r8, r6
	bne	.L85
	ldr	r8, [sp, #4]
	ldrh	sl, [r8, r9]
	ldr	r9, [sp, #12]
	ldrh	r8, [r9, #0]
	cmp	sl, r8
	bcc	.L87
	b	.L86
.L85:
	bhi	.L86
.L87:
	ldrh	r8, [r3, #0]
	movw	sl, #65535
	cmp	r8, sl
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	ldreq	r3, .L95
	streq	r2, [r3, #2304]
	beq	.L81
.L88:
	mov	r1, #6
	mla	r3, r1, r8, fp
	mov	r1, r8
	b	.L89
.L86:
	strh	r1, [r4, ip]	@ movhi
	ldrh	r1, [r3, #2]
	strh	r1, [r2, #2]	@ movhi
	ldr	r1, .L95
	ldr	ip, [r1, #2292]
	cmp	r3, ip
	bne	.L90
	strh	r0, [r3, #2]	@ movhi
.L94:
	str	r2, [r1, #2292]
	b	.L81
.L90:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #2288]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L81:
	mov	r0, #0
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L96:
	.align	2
.L95:
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
	movw	r1, #2308
	ldr	r3, .L99
	ldrh	r2, [r3, r1]
	add	r2, r2, #1
	uxth	r2, r2
	strh	r2, [r3, r1]	@ movhi
	ldrh	r3, [r3, #12]
	cmp	r3, r2
	ldmcsfd	sp!, {r3, pc}
	ldr	r0, .L99+4
	mov	r2, #219
	ldr	r1, .L99+8
	ldmfd	sp!, {r3, lr}
	b	printk
.L100:
	.align	2
.L99:
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
	beq	.L102
	ldr	r1, .L109
	mov	r5, #6
	mul	ip, r5, r0
	mvn	r3, #0
	ldr	r4, [r1, #2288]
	add	r2, r4, ip
	strh	r3, [r2, #2]	@ movhi
	strh	r3, [r4, ip]	@ movhi
	ldr	r3, [r1, #2312]
	cmp	r3, #0
	beq	.L108
.L103:
	ldr	sl, [r1, #2288]
	mov	r8, r0, asl #1
	ldr	r7, [r1, #2300]
	rsb	r1, sl, r3
	ldr	r9, .L109+4
	mov	r1, r1, asr #1
	ldrh	r8, [r7, r8]
	mul	r1, r9, r1
	mov	r9, r5
	uxth	r1, r1
.L106:
	mov	r5, r1, asl #1
	ldrh	r5, [r7, r5]
	cmp	r5, r8
	bcs	.L104
	ldrh	r5, [r3, #0]
	cmp	r5, r6
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	beq	.L102
.L105:
	mla	r3, r9, r5, sl
	mov	r1, r5
	b	.L106
.L104:
	ldrh	r5, [r3, #2]
	strh	r5, [r2, #2]	@ movhi
	strh	r1, [r4, ip]	@ movhi
	ldr	r1, .L109
	ldr	ip, [r1, #2312]
	cmp	r3, ip
	bne	.L107
	strh	r0, [r3, #2]	@ movhi
.L108:
	str	r2, [r1, #2312]
	b	.L102
.L107:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #2288]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L102:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L110:
	.align	2
.L109:
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
	movw	r1, #2316
	ldr	r3, .L113
	ldrh	r2, [r3, r1]
	add	r2, r2, #1
	uxth	r2, r2
	strh	r2, [r3, r1]	@ movhi
	ldrh	r3, [r3, #12]
	cmp	r3, r2
	ldmcsfd	sp!, {r3, pc}
	ldr	r0, .L113+4
	mov	r2, #212
	ldr	r1, .L113+8
	ldmfd	sp!, {r3, lr}
	b	printk
.L114:
	.align	2
.L113:
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
	ldr	r3, .L121
	mov	r7, r0
	mul	r5, r5, r1
	ldr	r6, [r3, #2288]
	movw	r3, #65535
	add	r4, r6, r5
	ldrh	r2, [r4, #2]
	cmp	r2, r3
	bne	.L116
	ldr	r3, [r0, #0]
	cmp	r4, r3
	beq	.L116
	ldr	r0, .L121+4
	movw	r2, #377
	ldr	r1, .L121+8
	bl	printk
.L116:
	ldr	r3, [r7, #0]
	movw	r2, #65535
	cmp	r4, r3
	ldrh	r3, [r6, r5]
	bne	.L117
	cmp	r3, r2
	ldrne	r2, .L121
	movne	r1, #6
	moveq	r3, #0
	streq	r3, [r7, #0]
	ldrne	r2, [r2, #2288]
	mlane	r3, r1, r3, r2
	mvnne	r2, #0
	strne	r3, [r7, #0]
	strneh	r2, [r3, #2]	@ movhi
	b	.L119
.L117:
	cmp	r3, r2
	ldrh	r2, [r4, #2]
	bne	.L120
	cmp	r2, r3
	beq	.L119
	mov	r1, #6
	ldr	r3, .L121
	mul	r2, r1, r2
	mvn	r1, #0
	ldr	r3, [r3, #2288]
	strh	r1, [r3, r2]	@ movhi
	b	.L119
.L120:
	ldr	r0, .L121
	mov	r1, #6
	ldr	ip, [r0, #2288]
	mla	r3, r1, r3, ip
	strh	r2, [r3, #2]	@ movhi
	ldrh	ip, [r4, #2]
	ldrh	r2, [r6, r5]
	ldr	r3, [r0, #2288]
	mul	r1, r1, ip
	strh	r2, [r3, r1]	@ movhi
.L119:
	mvn	r3, #0
	mov	r0, #0
	strh	r3, [r6, r5]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L122:
	.align	2
.L121:
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
	beq	.L128
	ldr	r2, .L129
	movw	lr, #65535
	mov	ip, #6
	ldr	r4, [r2, #2288]
	b	.L125
.L127:
	mla	r3, ip, r2, r4
	sub	r1, r1, #1
	uxth	r1, r1
.L125:
	cmp	r1, #0
	beq	.L126
	ldrh	r2, [r3, #0]
	cmp	r2, lr
	bne	.L127
.L126:
	rsb	r4, r4, r3
	ldr	r3, .L129+4
	mov	r4, r4, asr #1
	mul	r4, r3, r4
	uxth	r4, r4
	mov	r1, r4
	bl	List_remove_node
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L128:
	movw	r0, #65535
	ldmfd	sp!, {r4, pc}
.L130:
	.align	2
.L129:
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
	ldr	r2, .L138
	ldr	r3, [r2, #2292]
	cmp	r3, #0
	beq	.L137
	ldr	r2, [r2, #2288]
	movw	r1, #65535
	mov	ip, #6
	b	.L134
.L136:
	mla	r3, ip, r3, r2
	sub	r0, r0, #1
	uxth	r0, r0
.L134:
	cmp	r0, #0
	beq	.L135
	ldrh	r3, [r3, #0]
	cmp	r3, r1
	bne	.L136
	mov	r0, r1
	bx	lr
.L135:
	rsb	r3, r2, r3
	ldr	r0, .L138+4
	mov	r3, r3, asr #1
	mul	r0, r0, r3
	uxth	r0, r0
	bx	lr
.L137:
	movw	r0, #65535
	bx	lr
.L139:
	.align	2
.L138:
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
	mov	r2, #2320
	ldr	r3, .L149
	mov	r4, r0
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L141
	mov	r2, #2368
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L141
	mov	r2, #2416
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L141
	mov	r7, #6
	ldr	r8, [r3, #2288]
	mul	r7, r7, r0
	ldr	r2, [r3, #2292]
	add	r5, r8, r7
	cmp	r5, r2
	beq	.L141
	ldr	r2, [r3, #2296]
	mov	r3, r0, asl #1
	movw	r6, #65535
	ldrh	sl, [r2, r3]
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	mulne	sl, r3, sl
	ldrh	r3, [r5, #2]
	mvneq	sl, #0
	cmp	r3, r6
	bne	.L143
	ldrh	r3, [r8, r7]
	cmp	r3, r6
	bne	.L143
	ldr	r0, .L149+4
	mov	r2, #468
	ldr	r1, .L149+8
	bl	printk
.L143:
	ldrh	r2, [r5, #2]
	cmp	r2, r6
	bne	.L144
	ldrh	r3, [r8, r7]
	cmp	r3, r6
	beq	.L141
.L144:
	ldr	r6, .L149
	mov	r1, #6
	ldr	r3, [r6, #2288]
	mla	r2, r1, r2, r3
	ldr	r1, .L149+12
	rsb	r3, r3, r2
	mov	r3, r3, asr #1
	mul	r3, r1, r3
	ldr	r1, [r6, #2296]
	uxth	r3, r3
	mov	r3, r3, asl #1
	ldrh	r1, [r1, r3]
	ldrh	r3, [r2, #4]
	cmp	r3, #0
	mulne	r3, r3, r1
	mvneq	r3, #0
	cmp	sl, r3
	bcs	.L141
	ldr	r0, .L149+16
	mov	r1, r4
	movw	r5, #2308
	bl	List_remove_node
	ldrh	r3, [r6, r5]
	cmp	r3, #0
	bne	.L146
	ldr	r0, .L149+4
	movw	r2, #479
	ldr	r1, .L149+8
	bl	printk
.L146:
	ldr	r3, .L149
	mov	r0, r4
	ldrh	r2, [r3, r5]
	sub	r2, r2, #1
	strh	r2, [r3, r5]	@ movhi
	bl	INSERT_DATA_LIST
.L141:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L150:
	.align	2
.L149:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+51
	.word	-1431655765
	.word	.LANCHOR0+2292
	.fnend
	.size	List_update_data_list, .-List_update_data_list
	.align	2
	.global	select_l2p_ram_region
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L163
	movw	r1, #65535
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	ldrh	r3, [r2, #110]
	ldr	r0, [r2, #2464]
	mov	r2, #0
	mov	r4, r2
	b	.L152
.L154:
	add	r2, r2, #12
	add	ip, r0, r2
	ldrh	ip, [ip, #-12]
	cmp	ip, r1
	beq	.L153
	add	r4, r4, #1
	uxth	r4, r4
.L152:
	cmp	r4, r3
	bne	.L154
	mov	r1, #0
	mov	r4, r3
	mov	r5, #-2147483648
	mov	r2, r1
	b	.L155
.L157:
	add	ip, r0, r1
	ldr	ip, [ip, #4]
	cmp	ip, #0
	blt	.L156
	cmp	ip, r5
	movcc	r5, ip
	movcc	r4, r2
.L156:
	add	r2, r2, #1
	add	r1, r1, #12
	uxth	r2, r2
.L155:
	cmp	r2, r3
	bne	.L157
	cmp	r4, r3
	bcc	.L153
	ldr	r1, .L163
	movw	r2, #2468
	mov	r4, r3
	mvn	ip, #0
	ldrh	r6, [r1, r2]
	mov	r1, #0
	mov	r2, r1
	b	.L158
.L160:
	add	r5, r0, r1
	ldr	r5, [r5, #4]
	cmp	r5, ip
	bcs	.L159
	ldrh	r7, [r0, r1]
	cmp	r7, r6
	movne	ip, r5
	movne	r4, r2
.L159:
	add	r2, r2, #1
	add	r1, r1, #12
	uxth	r2, r2
.L158:
	cmp	r2, r3
	bne	.L160
	cmp	r4, r2
	bcc	.L153
	ldr	r0, .L163+4
	movw	r2, #782
	ldr	r1, .L163+8
	bl	printk
.L153:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L164:
	.align	2
.L163:
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
	ldr	r3, .L171
	movw	r2, #2470
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldrh	r1, [r3, r2]
	cmp	r1, #4
	add	ip, r1, #1
	mov	r1, r2
	strh	ip, [r3, r2]	@ movhi
	bhi	.L166
	cmp	r0, #0
	ldmeqfd	sp!, {r4, r5, pc}
.L166:
	mov	r2, #0
	ldrh	r4, [r3, #12]
	strh	r2, [r3, r1]	@ movhi
	movw	ip, #65535
	str	r2, [r3, #2472]
	ldr	r1, [r3, #2296]
	mov	r3, r2
	ldr	r2, .L171
	b	.L168
.L170:
	ldrh	r0, [r1], #2
	add	r3, r3, #1
	cmp	r0, ip
	uxth	r3, r3
	ldrne	r5, [r2, #2472]
	addne	r0, r0, r5
	strne	r0, [r2, #2472]
.L168:
	cmp	r3, r4
	bne	.L170
	ldmfd	sp!, {r4, r5, pc}
.L172:
	.align	2
.L171:
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
	ldr	r3, .L177
	mov	r2, r0, lsr #5
	cmp	r1, #0
	and	r0, r0, #31
	uxth	r2, r2
	mov	ip, #1
	ldr	r3, [r3, #2476]
	ldr	r1, [r3, r2, asl #2]
	orrne	r0, r1, ip, asl r0
	biceq	r0, r1, ip, asl r0
	str	r0, [r3, r2, asl #2]
	bx	lr
.L178:
	.align	2
.L177:
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
	ldr	r3, .L180
	mov	r2, r0, lsr #5
	and	r0, r0, #31
	ldr	r3, [r3, #2476]
	ldr	r3, [r3, r2, asl #2]
	mov	r0, r3, lsr r0
	and	r0, r0, #1
	bx	lr
.L181:
	.align	2
.L180:
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
	ldr	r3, .L189
	movw	ip, #65535
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldrh	r3, [r3, #4]
	b	.L183
.L185:
	add	r4, r0, r2, asl #1
	add	r2, r2, #1
	ldrh	r4, [r4, #16]
	uxth	r2, r2
	cmp	r4, ip
	ldrneh	r4, [r0, #4]
	addne	r4, r4, #1
	strneh	r4, [r0, #4]	@ movhi
.L183:
	cmp	r2, r3
	bcc	.L185
	ldr	r2, .L189
	mov	ip, r0
	movw	r4, #65535
	mvn	r1, r1
	ldrh	r5, [r2, #74]
	mov	r2, #0
	b	.L186
.L188:
	ldrh	r6, [ip, #16]
	add	r2, r2, #1
	add	ip, ip, #2
	cmp	r6, r4
	uxth	r2, r2
	ldrneh	r6, [r0, #4]
	addne	r6, r5, r6
	addne	r6, r6, r1
	strneh	r6, [r0, #4]	@ movhi
.L186:
	cmp	r2, r3
	bne	.L188
	ldmfd	sp!, {r4, r5, r6, pc}
.L190:
	.align	2
.L189:
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
	ldr	r1, .L196
	add	r2, r0, r2, asl #1
	ldrh	ip, [r1, #4]
	mov	r1, #0
	ldrh	r2, [r2, #16]
	b	.L193
.L195:
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
.L193:
	cmp	r2, r3
	beq	.L195
	bx	lr
.L197:
	.align	2
.L196:
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
	ldr	r3, .L203
	ldrh	r2, [r0, #0]
	ldrh	r3, [r3, #12]
	cmp	r2, r3
	bcc	.L199
	ldr	r0, .L203+4
	movw	r2, #2095
	ldr	r1, .L203+8
	bl	printk
.L199:
	mov	r5, #0
	ldr	r6, .L203
	strh	r5, [r4, #4]	@ movhi
	mvn	r7, #0
	strb	r5, [r4, #7]
	b	.L200
.L202:
	add	r3, r6, r5
	ldrh	r1, [r4, #0]
	add	r8, r5, #8
	add	r5, r5, #1
	ldrb	r0, [r3, #32]	@ zero_extendqisi2
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
.L200:
	ldrh	r3, [r6, #4]
	cmp	r3, r5
	bhi	.L202
	ldr	r2, .L203
	mov	r0, #0
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	ldrh	r2, [r2, #74]
	mul	r3, r2, r3
	strh	r3, [r4, #4]	@ movhi
	mov	r3, #1
	strb	r3, [r4, #9]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L204:
	.align	2
.L203:
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
	ldr	r6, .L210
	b	.L206
.L208:
	add	r3, r6, r5
	mov	r1, r7
	add	r5, r5, #1
	ldrb	r0, [r3, #32]	@ zero_extendqisi2
	bl	V2P_block
	uxth	r5, r5
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldreqh	r3, [r6, #74]
	addeq	r4, r4, r3
	uxtheq	r4, r4
.L206:
	ldrh	r3, [r6, #4]
	cmp	r3, r5
	bhi	.L208
	cmp	r4, #0
	beq	.L209
	mov	r1, r4
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r4, r0
.L209:
	ldr	r3, .L210
	mov	r2, #6
	mov	r0, #0
	ldr	r3, [r3, #2288]
	mla	r7, r2, r7, r3
	strh	r4, [r7, #4]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L211:
	.align	2
.L210:
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
	ldr	r3, .L215
	ldr	r0, [r3, #2312]
	cmp	r0, #0
	bxeq	lr
	ldr	r2, [r3, #2288]
	rsb	r0, r2, r0
	ldr	r2, .L215+4
	mov	r0, r0, asr #1
	mul	r0, r2, r0
	ldr	r2, [r3, #2300]
	uxth	r0, r0
	mov	r3, r0, asl #1
	ldrh	r0, [r2, r3]
	bx	lr
.L216:
	.align	2
.L215:
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
	ldr	r2, .L224
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldr	r3, [r2, #2312]
	cmp	r3, #0
	beq	.L223
	movw	r1, #2316
	mov	ip, #7
	ldrh	r1, [r2, r1]
	mov	r5, #6
	movw	r4, #65535
	mul	r1, ip, r1
	mov	r1, r1, asr #3
	cmp	r0, r1
	uxthgt	r0, r1
	ldr	r1, [r2, #2288]
	ldr	r2, .L224+4
	rsb	r3, r1, r3
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	mov	r2, #0
	uxth	r3, r3
	b	.L220
.L222:
	mul	ip, r5, r3
	ldrh	ip, [r1, ip]
	cmp	ip, r4
	beq	.L221
	add	r2, r2, #1
	mov	r3, ip
	uxth	r2, r2
.L220:
	cmp	r2, r0
	bne	.L222
.L221:
	ldr	r2, .L224
	mov	r3, r3, asl #1
	ldr	r2, [r2, #2300]
	ldrh	r0, [r2, r3]
	ldmfd	sp!, {r4, r5, pc}
.L223:
	mov	r0, r3
	ldmfd	sp!, {r4, r5, pc}
.L225:
	.align	2
.L224:
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
	mov	r8, r0
	ldr	r4, .L236
	add	r5, r8, #12
	ldr	r1, .L236+4
	movw	r7, #2316
	bl	strcpy
	mov	r0, r5
	ldr	r2, [r4, #88]
	mov	r6, #2320
	ldr	r1, .L236+8
	bl	sprintf
	ldr	r1, .L236+12
	add	r5, r5, r0
	mov	r0, r5
	add	r5, r5, #10
	bl	strcpy
	ldr	r1, .L236+16
	mov	r0, r5
	ldr	r2, [r4, #2480]
	bl	sprintf
	ldr	r1, .L236+20
	ldr	r2, [r4, #2472]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+24
	ldr	r2, [r4, #2484]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+28
	ldr	r2, [r4, #2488]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+32
	ldr	r2, [r4, #2492]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+36
	ldr	r2, [r4, #2496]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+40
	ldr	r2, [r4, #2500]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+44
	ldr	r2, [r4, #2504]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r4, #2508]
	ldr	r1, .L236+48
	mov	r2, r2, lsr #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r4, #2512]
	ldr	r1, .L236+52
	mov	r2, r2, lsr #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+56
	ldr	r2, [r4, #2516]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+60
	ldr	r2, [r4, #2520]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+64
	ldrh	r2, [r4, #130]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+68
	ldrh	r2, [r4, r7]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+72
	ldr	r2, [r4, #2524]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+76
	ldr	r2, [r4, #2528]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+80
	ldr	r2, [r4, #2532]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+84
	ldr	r2, [r4, #2536]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+88
	ldr	r2, [r4, #2540]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+92
	ldr	r2, [r4, #2544]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2270
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+96
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2268
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+100
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+104
	ldr	r2, [r4, #112]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+108
	ldr	r2, [r4, #104]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+112
	ldr	r2, [r4, #0]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+116
	ldrh	r2, [r4, #190]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+120
	ldrh	r2, [r4, #12]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2548
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+124
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+128
	ldr	r2, [r4, #16]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2552
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+132
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+136
	ldrh	r2, [r4, #124]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2322
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+140
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+144
	ldrb	r2, [r4, #2326]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, r6]
	ldr	r1, .L236+148
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+152
	ldrb	r2, [r4, #2328]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2324
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+156
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #2296]
	mov	r6, #2368
	ldr	r1, .L236+160
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2370
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+164
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+168
	ldrb	r2, [r4, #2374]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, r6]
	ldr	r1, .L236+172
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+176
	ldrb	r2, [r4, #2376]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2372
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+180
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #2296]
	add	r6, r6, #196
	ldr	r1, .L236+184
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2418
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+188
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+192
	ldrb	r2, [r4, #2422]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	mov	r3, #2416
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+196
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+200
	ldrb	r2, [r4, #2424]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2420
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+204
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2566
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+208
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+212
	ldrb	r2, [r4, #2570]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+216
	ldrh	r2, [r4, r6]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+220
	ldrb	r2, [r4, #2572]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2568
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+224
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, [r4, #2688]
	ldr	r1, .L236+228
	str	r3, [sp, #0]
	ldr	r3, [r4, #2696]
	ldr	r2, [r4, #2692]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+232
	ldr	r2, [r4, #2684]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+236
	ldr	r2, [r4, #2708]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #3124
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+240
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #3126
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+244
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L236+248
	ldr	r2, [r4, #3128]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #3132
	ldrh	r2, [r4, r3]
	ldr	r1, .L236+252
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	bl	GetFreeBlockMinEraseCount
	ldr	r1, .L236+256
	mov	r2, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	ldrh	r0, [r4, r7]
	bl	GetFreeBlockMaxEraseCount
	ldr	r1, .L236+260
	mov	r2, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L236+264
	ldr	r3, [r3, #0]
	cmp	r3, #1
	add	r5, r5, r0
	bne	.L232
.L227:
	ldrh	r3, [r4, r6]
	movw	r2, #65535
	cmp	r3, r2
	beq	.L229
	ldr	r2, [r4, #2296]
	mov	r3, r3, asl #1
	mov	r0, r5
	ldr	r1, .L236+268
	ldrh	r2, [r2, r3]
	bl	sprintf
	add	r5, r5, r0
.L229:
	mov	r0, #0
	ldr	r4, .L236
	bl	List_get_gc_head_node
	mov	r6, #0
	movw	sl, #65535
	mov	r9, #6
	uxth	r3, r0
.L231:
	cmp	r3, sl
	beq	.L230
	ldr	r1, [r4, #2296]
	mov	r2, r3, asl #1
	mul	r7, r9, r3
	mov	r0, r5
	ldrh	r1, [r1, r2]
	str	r1, [sp, #0]
	ldr	r1, [r4, #2288]
	add	r1, r1, r7
	ldrh	r1, [r1, #4]
	str	r1, [sp, #4]
	ldr	r1, [r4, #2300]
	ldrh	r2, [r1, r2]
	ldr	r1, .L236+272
	str	r2, [sp, #8]
	mov	r2, r6
	bl	sprintf
	add	r6, r6, #1
	ldr	r3, [r4, #2288]
	cmp	r6, #16
	ldrh	r3, [r3, r7]
	add	r5, r5, r0
	bne	.L231
.L230:
	ldr	r6, .L236
	mov	r4, #0
	movw	sl, #65535
	mov	r9, #6
	ldr	r2, [r6, #2312]
	ldr	r3, [r6, #2288]
	rsb	r3, r3, r2
	ldr	r2, .L236+276
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r3, r3
.L233:
	cmp	r3, sl
	beq	.L232
	mul	r7, r9, r3
	ldr	r2, [r6, #2288]
	mov	r0, r5
	add	r2, r2, r7
	ldrh	r2, [r2, #4]
	str	r2, [sp, #0]
	mov	r2, r3, asl #1
	ldr	r1, [r6, #2300]
	ldrh	r2, [r1, r2]
	ldr	r1, .L236+280
	str	r2, [sp, #4]
	mov	r2, r4
	bl	sprintf
	add	r4, r4, #1
	ldr	r3, [r6, #2288]
	cmp	r4, #4
	ldrh	r3, [r3, r7]
	add	r5, r5, r0
	bne	.L233
.L232:
	rsb	r0, r8, r5
	add	sp, sp, #16
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L237:
	.align	2
.L236:
	.word	.LANCHOR0
	.word	.LC2
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
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	ldr	r1, .L239
	ldr	r2, .L239+4
	bl	sprintf
	add	r5, r4, r0
	mov	r0, r5
	bl	FtlPrintInfo2buf
	add	r0, r5, r0
	rsb	r0, r4, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L240:
	.align	2
.L239:
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
	ldr	r3, .L264
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r1, [r3, #2532]
	ldr	r2, [r3, #2544]
	cmp	r1, r2
	bcs	.L242
	mov	r2, #0
	ldrh	r1, [r3, #12]
	str	r2, [r3, #2524]
	ldr	r0, [r3, #2300]
	b	.L243
.L244:
	ldrh	lr, [r0], #2
	add	r2, r2, #1
	ldr	ip, [r3, #2524]
	add	ip, lr, ip
	str	ip, [r3, #2524]
.L243:
	cmp	r2, r1
	ldr	r4, .L264
	bcc	.L244
	ldr	r5, [r4, #2524]
	mov	r0, r5
	bl	__aeabi_uidiv
	ldrh	r1, [r4, #64]
	str	r0, [r4, #2532]
	ldr	r0, [r4, #2528]
	rsb	r0, r0, r5
	bl	__aeabi_uidiv
	str	r0, [r4, #2524]
	b	.L245
.L242:
	ldr	r2, [r3, #2540]
	cmp	r1, r2
	addhi	r2, r2, #1
	strhi	r2, [r3, #2540]
	movhi	r2, #0
	bhi	.L246
	b	.L245
.L247:
	ldr	r0, [r3, #2300]
	mov	r1, r2, asl #1
	add	r2, r2, #1
	ldrh	ip, [r0, r1]
	add	ip, ip, #1
	strh	ip, [r0, r1]	@ movhi
.L246:
	ldrh	r1, [r3, #12]
	cmp	r2, r1
	bcc	.L247
.L245:
	ldr	r3, .L264
	ldr	r6, [r3, #2544]
	ldr	r5, [r3, #2532]
	add	r2, r6, #256
	cmp	r2, r5
	mov	r2, r3
	bls	.L248
	ldr	r3, [r3, #2540]
	add	r1, r6, #768
	cmp	r1, r3
	bhi	.L263
.L248:
	movw	r3, #2316
	ldr	r4, .L264
	ldrh	r0, [r2, r3]
	add	r0, r0, r0, asl #1
	ubfx	r0, r0, #2, #16
	bl	GetFreeBlockMaxEraseCount
	add	r3, r6, #64
	cmp	r0, r3
	mov	r8, r0
	bcc	.L263
	ldr	r3, [r4, #2292]
	cmp	r3, #0
	beq	.L263
	movw	r7, #65535
	ldrh	sl, [r4, #12]
	ldr	r0, [r4, #2288]
	mov	r2, #0
	ldr	r1, [r4, #2300]
	mov	fp, r7
	mov	r4, r7
	b	.L250
.L253:
	add	r2, r2, #1
	uxth	r2, r2
	cmp	r2, sl
	bhi	.L263
	ldrh	ip, [r3, #4]
	cmp	ip, #0
	beq	.L251
	rsb	r3, r0, r3
	ldr	ip, .L264+4
	mov	r3, r3, asr #1
	mul	r3, ip, r3
	uxth	r3, r3
	mov	ip, r3, asl #1
	ldrh	ip, [r1, ip]
	cmp	ip, r6
	bls	.L260
	cmp	ip, r7
	movcc	r7, ip
	movcc	r4, r3
.L251:
	mov	ip, #6
	mla	r3, ip, r9, r0
.L250:
	ldrh	r9, [r3, #0]
	cmp	r9, fp
	bne	.L253
	b	.L252
.L260:
	mov	r4, r3
.L252:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L249
	mov	r9, r4, asl #1
	ldrh	sl, [r1, r9]
	cmp	sl, r6
	bls	.L254
	bl	GetFreeBlockMinEraseCount
	cmp	r0, r6
	ldrhi	r3, .L264
	strhi	r7, [r3, #2544]
.L254:
	cmp	sl, r5
	bcs	.L263
	add	r3, sl, #128
	cmp	r8, r3
	ble	.L263
	add	r3, sl, #256
	ldr	r6, .L264
	cmp	r3, r5
	bcc	.L255
	ldr	r3, [r6, #2540]
	add	sl, sl, #768
	cmp	sl, r3
	bcs	.L263
.L255:
	ldr	r3, [r6, #2296]
	mov	r1, r4
	ldr	r0, .L264+8
	mov	r2, r5
	ldrh	r3, [r3, r9]
	str	r3, [sp, #0]
	ldr	r3, [r6, #2300]
	ldrh	r3, [r3, r9]
	stmib	sp, {r3, r8}
	ldr	r3, [r6, #2540]
	bl	printk
	mov	r3, #1
	str	r3, [r6, #3136]
	b	.L249
.L263:
	movw	r4, #65535
.L249:
	mov	r0, r4
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L265:
	.align	2
.L264:
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
	beq	.L267
	ldr	r2, .L268
	mov	r3, r0, asl #1
	mov	r1, #0
	ldr	r2, [r2, #2296]
	strh	r1, [r2, r3]	@ movhi
	bl	INSERT_FREE_LIST
.L267:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L269:
	.align	2
.L268:
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
	bne	.L271
	ldr	r0, .L281
	mov	r2, #2704
	ldr	r1, .L281+4
	bl	printk
.L271:
	ldr	r3, .L281+8
	ldrh	r2, [r4, #2]
	ldrh	r3, [r3, #74]
	cmp	r2, r3
	bne	.L272
	ldr	r0, .L281
	movw	r2, #2705
	ldr	r1, .L281+4
	bl	printk
.L272:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L273
	ldr	r0, .L281
	movw	r2, #2706
	ldr	r1, .L281+4
	bl	printk
.L273:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	mov	r2, #0
	ldr	r1, .L281+8
	strb	r2, [r4, #10]
	add	r3, r4, r3, asl #1
	ldrh	r0, [r1, #4]
	movw	r1, #65535
	ldrh	r3, [r3, #16]
	b	.L274
.L276:
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
.L274:
	cmp	r3, r1
	beq	.L276
	ldrh	r5, [r4, #2]
	movw	r1, #65535
	ldrh	r2, [r4, #4]
	orr	r5, r5, r3, asl #10
	ldr	r3, .L281+8
	sub	r2, r2, #1
	ldrh	r0, [r3, #4]
	uxth	r2, r2
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	strh	r2, [r4, #4]	@ movhi
.L278:
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
	beq	.L278
	strb	r3, [r4, #6]
	ldr	r3, .L281+8
	ldrh	r1, [r4, #2]
	ldrh	r3, [r3, #74]
	cmp	r1, r3
	bne	.L279
	cmp	r2, #0
	beq	.L279
	ldr	r0, .L281
	movw	r2, #2727
	ldr	r1, .L281+4
	bl	printk
.L279:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, pc}
.L282:
	.align	2
.L281:
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
	ldr	r3, .L291
	mov	r1, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r7, #12
	str	r1, [r3, #3140]
	mov	r6, #1
	mov	r5, #20
	b	.L284
.L285:
	mul	r2, r7, r1
	ldr	ip, [r3, #3144]
	add	r0, ip, r2
	str	r6, [r0, #8]
	ldrh	r0, [r3, #82]
	mul	r0, r0, r1
	add	r4, r0, #3
	cmp	r0, #0
	movlt	r0, r4
	ldr	r4, [r3, #3148]
	bic	r0, r0, #3
	add	r0, r4, r0
	str	r0, [ip, r2]
	ldrh	r0, [r3, #84]
	ldr	r4, [r3, #3144]
	mul	r0, r0, r1
	add	ip, r4, r2
	add	r8, r0, #3
	cmp	r0, #0
	movlt	r0, r8
	ldr	r8, [r3, #3152]
	bic	r0, r0, #3
	add	r0, r8, r0
	str	r0, [ip, #4]
	ldr	r0, [r3, #3156]
	ldr	r2, [r4, r2]
	mla	r0, r5, r1, r0
	add	r1, r1, #1
	uxth	r1, r1
	str	r2, [r0, #8]
	ldr	r2, [ip, #4]
	str	r2, [r0, #12]
.L284:
	ldrh	r2, [r3, #4]
	cmp	r1, r2
	bcc	.L285
	b	.L290
.L287:
	mul	r1, r5, r2
	ldr	ip, [r3, #3144]
	add	r0, ip, r1
	str	r4, [r0, #8]
	ldrh	r0, [r3, #82]
	mul	r0, r0, r2
	add	r6, r0, #3
	cmp	r0, #0
	movlt	r0, r6
	ldr	r6, [r3, #3148]
	bic	r0, r0, #3
	add	r0, r6, r0
	str	r0, [ip, r1]
	ldr	r0, [r3, #3144]
	add	r0, r0, r1
	ldrh	r1, [r3, #84]
	mul	r1, r1, r2
	add	r2, r2, #1
	uxth	r2, r2
	add	ip, r1, #3
	cmp	r1, #0
	movlt	r1, ip
	ldr	ip, [r3, #3152]
	bic	r1, r1, #3
	add	r1, ip, r1
	str	r1, [r0, #4]
	b	.L289
.L290:
	ldr	r3, .L291
	mov	r5, #12
	mov	r4, #0
.L289:
	ldr	r1, [r3, #3160]
	cmp	r2, r1
	bcc	.L287
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L292:
	.align	2
.L291:
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
	ldr	r3, .L300
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r5, #20
	ldr	r6, [r3, #3160]
	mov	fp, #12
	ldr	ip, [r3, #3144]
	mov	r3, #0
	mov	r4, r3
	b	.L294
.L297:
	mul	sl, fp, r2
	add	r8, ip, sl
	ldr	r9, [ip, sl]
	ldr	sl, [r7, #8]
	cmp	r9, sl
	streq	r4, [r8, #8]
	beq	.L296
.L295:
	add	r2, r2, #1
	uxth	r2, r2
.L299:
	cmp	r2, r6
	bcc	.L297
.L296:
	add	r3, r3, #1
	uxth	r3, r3
.L294:
	cmp	r3, r1
	ldmcsfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
	mla	r7, r5, r3, r0
	mov	r2, #0
	b	.L299
.L301:
	.align	2
.L300:
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
	ldr	r3, .L309
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	mov	r6, #12
	ldr	r8, [r3, #3160]
	mov	r5, #1
	ldr	r7, [r3, #3144]
	mov	r4, #20
	mov	r3, #0
	b	.L303
.L306:
	mla	ip, r6, r2, r7
	ldr	sl, [ip, #8]
	cmp	sl, #0
	bne	.L304
	mla	r2, r4, r3, r0
	ldr	sl, [ip, #0]
	str	r5, [ip, #8]
	str	sl, [r2, #8]
	ldr	ip, [ip, #4]
	str	ip, [r2, #12]
	b	.L305
.L304:
	add	r2, r2, #1
	uxth	r2, r2
	b	.L307
.L308:
	mov	r2, #0
.L307:
	cmp	r2, r8
	bcc	.L306
.L305:
	add	r3, r3, #1
	uxth	r3, r3
.L303:
	cmp	r3, r1
	bcc	.L308
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L310:
	.align	2
.L309:
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
	ldr	r3, .L316
	movw	r2, #3164
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #3168]
	mov	r3, #0
	b	.L312
.L314:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L315
	add	r3, r3, #1
	uxth	r3, r3
.L312:
	cmp	r3, r1
	bne	.L314
	mov	r0, #0
	bx	lr
.L315:
	mov	r0, #1
	bx	lr
.L317:
	.align	2
.L316:
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
	ldr	r3, .L322
	movw	r2, #3164
	ldrh	ip, [r3, r2]
	ldr	r2, [r3, #3168]
	mov	r3, #0
	mov	r1, r2
	b	.L319
.L321:
	ldrh	r7, [r1], #2
	cmp	r7, r0
	beq	.L320
	add	r3, r3, #1
	uxth	r3, r3
.L319:
	cmp	r3, ip
	bne	.L321
	mov	r3, r3, asl #1
	strh	r0, [r2, r3]	@ movhi
	movw	r3, #3164
	ldr	r2, .L322
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
.L320:
	ldr	r3, .L322
	movw	r2, #3176
	mov	r0, #12
	ldrh	r1, [r3, r2]
	mul	r1, r0, r1
	ldr	r0, [r3, #3172]
	add	ip, r0, r1
	stmib	ip, {r5, r6}
	str	r4, [r0, r1]
	ldrh	r1, [r3, r2]
	add	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L323:
	.align	2
.L322:
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
	ldr	r0, .L327
	bl	printk
	ldr	r3, .L327+4
	movw	ip, #3178
	ldrh	r5, [r3, ip]
	cmp	r5, r4
	beq	.L325
	movw	r2, #3180
	ldrh	r0, [r3, r2]
	cmp	r0, r4
	beq	.L325
	movw	r1, #65535
	cmp	r5, r1
	streqh	r4, [r3, ip]	@ movhi
	beq	.L325
	cmp	r0, r1
	streqh	r4, [r3, r2]	@ movhi
.L325:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L328:
	.align	2
.L327:
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
	ldr	r6, .L333
	movw	r5, #3182
	mov	r2, r4
	ldrh	r1, [r6, r5]
	mov	r7, r0
	ldr	r0, .L333+4
	bl	printk
	mov	r0, r7
	bl	FtlGcRefreshBlock
	ldrh	r1, [r6, r5]
	ldr	r2, .L333+8
	mov	r3, #0
	b	.L330
.L332:
	ldrh	r0, [r2, #2]!
	cmp	r0, r4
	beq	.L331
	add	r3, r3, #1
	uxth	r3, r3
.L330:
	cmp	r3, r1
	bne	.L332
	cmp	r3, #15
	bhi	.L331
	ldr	r2, .L333
	add	r1, r2, r3, asl #1
	add	r3, r3, #1
	add	r1, r1, #3184
	strh	r4, [r1, #0]	@ movhi
	movw	r1, #3182
	strh	r3, [r2, r1]	@ movhi
.L331:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L334:
	.align	2
.L333:
	.word	.LANCHOR0
	.word	.LC74
	.word	.LANCHOR0+3182
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
	movw	r2, #3182
	ldr	r3, .L338
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L336
	movw	r1, #3178
	ldrh	r0, [r3, r1]
	movw	r1, #65535
	cmp	r0, r1
	bne	.L336
	movw	r4, #3218
	ldr	r5, .L338
	ldrh	r1, [r3, r4]
	cmp	r1, r2
	movcs	r2, #0
	strcsh	r2, [r3, r4]	@ movhi
	ldrh	r3, [r5, r4]
	add	r3, r5, r3, asl #1
	add	r3, r3, #3184
	ldrh	r0, [r3, #0]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r5, r4]
	add	r3, r3, #1
	strh	r3, [r5, r4]	@ movhi
.L336:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L339:
	.align	2
.L338:
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
	ldr	r3, .L347
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	mov	r7, r1
	ldrh	r8, [r3, #12]
	mov	r4, r0
	mov	r5, #0
	ldr	r6, .L347+4
	b	.L341
.L346:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L342
	ldr	r3, [r4, #12]
	cmp	r3, #0
	bne	.L343
.L342:
	ldr	r0, .L347+8
	mov	r2, #51
	ldr	r1, .L347+12
	bl	printk
.L343:
	add	r1, sp, #4
	mov	r2, sp
	mov	r0, r4
	bl	l2p_addr_tran
	ldr	r3, [r4, #12]
	ldr	ip, [r6, #12]
	ldrb	r0, [sp, #0]	@ zero_extendqisi2
	ldr	r1, [sp, #4]
	ldr	r2, [r4, #8]
	blx	ip
	ldr	r3, .L347
	str	r0, [r4, #0]
	ldrh	r3, [r3, #14]
	cmp	r3, #4
	bne	.L344
	ldr	r2, [r4, #8]
	ldr	r3, [r4, #12]
	ldr	r1, [sp, #4]
	add	r2, r2, #2048
	ldr	ip, [r6, #12]
	add	r3, r3, #8
	ldrb	r0, [sp, #0]	@ zero_extendqisi2
	add	r1, r8, r1
	blx	ip
	cmn	r0, #1
	beq	.L345
	ldr	r3, [r4, #12]
	ldr	r2, [r3, #12]
	cmn	r2, #1
	bne	.L344
	ldr	r2, [r3, #8]
	cmn	r2, #1
	bne	.L344
	ldr	r3, [r3, #0]
	cmn	r3, #1
	beq	.L344
.L345:
	mvn	r3, #0
	str	r3, [r4, #0]
.L344:
	add	r5, r5, #1
	add	r4, r4, #20
.L341:
	cmp	r5, r7
	bne	.L346
	mov	r0, #0
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L348:
	.align	2
.L347:
	.word	g_nand_phy_info
	.word	g_nand_ops
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
	ldr	r3, .L360
	cmp	r1, #1
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	.pad #92
	sub	sp, sp, #92
	ldreqh	r4, [r3, #76]
	add	r7, sp, #4
	ldrneh	r4, [r3, #74]
	mov	r5, r1
	ldr	r3, .L360+4
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
	bne	.L353
	b	.L359
.L356:
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
	bne	.L354
	ldr	r3, [sp, #28]
	cmn	r3, #1
	subeq	r4, r6, #1
	uxtheq	r4, r4
	beq	.L358
.L354:
	add	r6, r6, #1
	uxth	r8, r6
	b	.L358
.L359:
	mov	r8, #0
.L358:
	sxth	r6, r8
	sxth	r3, r4
	cmp	r6, r3
	ble	.L356
.L353:
	sxth	r0, r4
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L361:
	.align	2
.L360:
	.word	.LANCHOR0
	.word	.LANCHOR0+3220
	.fnend
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.align	2
	.global	FtlLoadFactoryBbt
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r5, #0
	ldr	r8, .L369
	ldr	r7, .L369+4
	ldr	r3, [r8, #-848]
	sub	r6, r7, #136
	ldr	sl, [r8, #-844]
	str	r3, [r8, #-860]
	str	sl, [r8, #-856]
	b	.L363
.L368:
	ldrh	r4, [r6, #70]
	mvn	r3, #0
	movw	r9, #61664
	strh	r3, [r7], #2	@ movhi
	add	r4, r4, r3
	uxth	r4, r4
	b	.L364
.L367:
	mla	r3, r3, r5, r4
	mov	r1, #1
	ldr	r0, .L369+8
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [r8, #-864]
	bl	FlashReadPages
	ldr	r3, [r8, #-868]
	cmn	r3, #1
	beq	.L365
	ldrh	r3, [sl, #0]
	cmp	r3, r9
	streqh	r4, [r7, #-2]	@ movhi
	beq	.L366
.L365:
	sub	r4, r4, #1
	uxth	r4, r4
.L364:
	ldrh	r3, [r6, #70]
	sub	r2, r3, #16
	cmp	r4, r2
	bgt	.L367
.L366:
	add	r5, r5, #1
.L363:
	ldrh	r3, [r6, #26]
	cmp	r5, r3
	bcc	.L368
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L370:
	.align	2
.L369:
	.word	.LANCHOR3
	.word	.LANCHOR0+136
	.word	.LANCHOR3-868
	.fnend
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	2
	.global	FlashProgPages
	.type	FlashProgPages, %function
FlashProgPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	fp, r3
	ldr	r7, .L384
	.pad #44
	sub	sp, sp, #44
	ldr	r3, .L384+4
	mov	sl, r1
	str	r2, [sp, #4]
	mov	r4, r0
	mov	r5, r0
	mov	r6, #0
	ldrh	r9, [r3, #12]
	mov	r8, r7
	b	.L372
.L377:
	ldr	r3, [r5, #8]
	cmp	r3, #0
	beq	.L373
	ldr	r3, [r5, #12]
	cmp	r3, #0
	bne	.L374
.L373:
	ldr	r0, .L384+8
	mov	r2, #81
	ldr	r1, .L384+12
	bl	printk
.L374:
	add	r1, sp, #16
	add	r2, sp, #12
	mov	r0, r5
	bl	l2p_addr_tran
	ldr	r3, [r5, #12]
	ldr	ip, [r7, #8]
	ldrb	r0, [sp, #12]	@ zero_extendqisi2
	ldr	r1, [sp, #16]
	ldr	r2, [r5, #8]
	blx	ip
	cmp	r0, #0
	str	r0, [r5, #0]
	mvnne	r3, #0
	strne	r3, [r5, #0]
	ldr	r3, .L384+4
	ldrh	r3, [r3, #14]
	cmp	r3, #4
	bne	.L376
	ldr	r2, [r5, #8]
	ldr	r3, [r5, #12]
	ldr	r1, [sp, #16]
	add	r2, r2, #2048
	add	r3, r3, #8
	ldr	ip, [r8, #8]
	ldrb	r0, [sp, #12]	@ zero_extendqisi2
	add	r1, r9, r1
	blx	ip
	cmp	r0, #0
	mvnne	r3, #0
	strne	r3, [r5, #0]
.L376:
	add	r6, r6, #1
	add	r5, r5, #20
.L372:
	cmp	r6, sl
	bne	.L377
	cmp	fp, #0
	beq	.L378
	ldr	r5, .L384+16
	mov	r6, #0
	sub	r7, r5, #840
	add	r8, r7, #4096
	b	.L379
.L383:
	mov	r3, #0
	str	r3, [r5, #-840]
	str	r3, [r5, #3256]
	add	r0, sp, #20
	ldr	r3, [r4, #4]
	mov	r1, #1
	ldr	r2, [sp, #4]
	str	r7, [sp, #28]
	str	r3, [sp, #24]
	str	r8, [sp, #32]
	bl	FlashReadPages
	ldr	r9, [sp, #20]
	cmn	r9, #1
	bne	.L380
	ldr	r0, .L384+20
	ldr	r1, [r4, #4]
	bl	printk
	str	r9, [r4, #0]
.L380:
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L381
	ldr	r2, [r3, #0]
	ldr	r3, [r5, #3256]
	cmp	r2, r3
	beq	.L381
	ldr	r0, .L384+24
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #0]
.L381:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L382
	ldr	r2, [r3, #0]
	ldr	r3, [r5, #-840]
	cmp	r2, r3
	beq	.L382
	ldr	r0, .L384+28
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #0]
.L382:
	add	r6, r6, #1
	add	r4, r4, #20
.L379:
	cmp	r6, sl
	bne	.L383
.L378:
	mov	r0, #0
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L385:
	.align	2
.L384:
	.word	g_nand_ops
	.word	g_nand_phy_info
	.word	.LC1
	.word	.LANCHOR1+145
	.word	.LANCHOR3
	.word	.LC75
	.word	.LC76
	.word	.LC77
	.fnend
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.global	FlashEraseBlocks
	.type	FlashEraseBlocks, %function
FlashEraseBlocks:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	mov	sl, r2
	ldr	r3, .L391
	mov	r4, r0
	ldr	r6, .L391+4
	mov	r5, #0
	ldrh	r8, [r3, #12]
	mov	r7, r6
	b	.L387
.L390:
	add	r1, sp, #4
	mov	r2, sp
	mov	r0, r4
	bl	l2p_addr_tran
	ldr	r3, [r6, #4]
	ldrb	r0, [sp, #0]	@ zero_extendqisi2
	ldr	r1, [sp, #4]
	blx	r3
	cmp	r0, #0
	str	r0, [r4, #0]
	mvnne	r3, #0
	strne	r3, [r4, #0]
	ldr	r3, .L391
	ldrh	r3, [r3, #14]
	cmp	r3, #4
	bne	.L389
	ldr	r1, [sp, #4]
	ldr	r3, [r7, #4]
	ldrb	r0, [sp, #0]	@ zero_extendqisi2
	add	r1, r8, r1
	blx	r3
	cmp	r0, #0
	mvnne	r3, #0
	strne	r3, [r4, #0]
.L389:
	add	r5, r5, #1
	add	r4, r4, #20
.L387:
	cmp	r5, sl
	bne	.L390
	mov	r0, #0
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L392:
	.align	2
.L391:
	.word	g_nand_phy_info
	.word	g_nand_ops
	.fnend
	.size	FlashEraseBlocks, .-FlashEraseBlocks
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	ldr	r4, .L401
	ldr	r6, .L401+4
.L394:
	ldrh	r1, [r4, #190]
	cmp	r1, #0
	beq	.L395
	ldrh	r3, [r4, #186]
	sub	r1, r1, #1
	add	r2, r4, r3, asl #1
	add	r3, r3, #1
	bic	r3, r3, #64512
	ldrh	r5, [r2, #192]
	strh	r1, [r4, #190]	@ movhi
	strh	r3, [r4, #186]	@ movhi
	mov	r0, r5
	bl	P2V_block_in_plane
	mov	r1, #1
	mov	r3, r5, asl #10
	mov	r2, r1
	mov	r7, r0
	ldr	r0, [r6, #3768]
	mov	r7, r7, asl #1
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r3, [r4, #2300]
	cmp	r5, #0
	ldrh	r2, [r3, r7]
	add	r2, r2, #1
	strh	r2, [r3, r7]	@ movhi
	ldr	r3, [r4, #2536]
	add	r3, r3, #1
	str	r3, [r4, #2536]
	bne	.L399
	b	.L400
.L395:
	ldr	r0, .L401+8
	bl	printk
.L398:
	b	.L398
.L400:
	mov	r1, r5
	ldrh	r2, [r4, #190]
	ldr	r0, .L401+12
	bl	printk
	b	.L394
.L399:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L402:
	.align	2
.L401:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC78
	.word	.LC79
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
	b	.L404
.L409:
	mov	r7, r3
	add	r3, r3, #2
	ldrh	r6, [r7, #0]
	cmp	r6, #0
	bne	.L405
	bl	FtlFreeSysBlkQueueOut
	movw	r3, #65533
	sub	r2, r0, #1
	mov	r1, r0
	strh	r0, [r7, #0]	@ movhi
	uxth	r2, r2
	cmp	r2, r3
	bls	.L406
	ldr	r3, .L411
	ldr	r0, .L411+4
	ldrh	r2, [r3, #190]
	bl	printk
.L407:
	b	.L407
.L406:
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	add	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	strh	r5, [r4, #0]	@ movhi
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	b	.L408
.L405:
	add	r5, r5, #1
	uxth	r5, r5
.L404:
	cmp	r5, r2
	bne	.L409
.L408:
	ldrh	r3, [r4, #10]
	cmp	r3, r5
	bhi	.L410
	ldr	r0, .L411+8
	movw	r2, #579
	ldr	r1, .L411+12
	bl	printk
.L410:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L412:
	.align	2
.L411:
	.word	.LANCHOR0
	.word	.LC80
	.word	.LC1
	.word	.LANCHOR1+160
	.fnend
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	2
	.global	FtlFreeSysBlkQueueIn
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldr	r4, .L416
	ldrh	r3, [r4, #190]
	cmp	r3, #1024
	ldmeqfd	sp!, {r4, r5, r6, pc}
	cmp	r1, #0
	beq	.L415
	bl	P2V_block_in_plane
	ldr	r3, .L416+4
	mov	r1, #1
	mov	r2, r1
	mov	r6, r0
	ldr	r0, [r3, #3768]
	mov	r3, r5, asl #10
	mov	r6, r6, asl #1
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r3, [r4, #2300]
	ldrh	r2, [r3, r6]
	add	r2, r2, #1
	strh	r2, [r3, r6]	@ movhi
	ldr	r3, [r4, #2536]
	add	r3, r3, #1
	str	r3, [r4, #2536]
.L415:
	ldr	r3, .L416
	ldrh	r2, [r3, #190]
	add	r2, r2, #1
	strh	r2, [r3, #190]	@ movhi
	ldrh	r2, [r3, #188]
	add	r1, r3, r2, asl #1
	add	r2, r2, #1
	bic	r2, r2, #64512
	strh	r5, [r1, #192]	@ movhi
	strh	r2, [r3, #188]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L417:
	.align	2
.L416:
	.word	.LANCHOR0
	.word	.LANCHOR3
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
	mov	r7, #0
	ldr	r8, .L446
	.pad #20
	sub	sp, sp, #20
	mov	sl, r0
	mov	r6, r1
	mov	r4, r7
	mov	r5, r7
	str	r0, [r8, #3772]
	mov	fp, #20
	ldr	r9, .L446+4
	b	.L419
.L423:
	mul	r3, fp, r7
	ldr	r2, [r8, #3768]
	mov	r1, #0
	str	r1, [r2, r3]
	add	r3, r9, r7
	mov	r1, sl
	ldrb	r0, [r3, #32]	@ zero_extendqisi2
	bl	V2P_block
	cmp	r6, #0
	str	r0, [sp, #4]
	beq	.L420
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L421
.L420:
	ldr	r0, [sp, #4]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addne	r5, r5, #1
	uxthne	r5, r5
	bne	.L421
	ldr	r3, [r8, #3768]
	ldr	r1, [sp, #4]
	mla	r3, fp, r4, r3
	mov	r2, r1, asl #10
	str	r2, [r3, #4]
	ldr	r2, [r8, #3776]
	str	r2, [r3, #8]
	ldrh	r2, [r9, #84]
	mul	r2, r2, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [r8, #3780]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L421:
	add	r7, r7, #1
	uxth	r7, r7
.L419:
	ldrh	r3, [r9, #4]
	cmp	r3, r7
	bhi	.L423
	cmp	r4, #0
	beq	.L425
	ldr	r7, .L446
	mov	r1, #0
	mov	r2, r4
	mov	r8, #0
	mov	r9, r8
	ldr	r0, [r7, #3768]
	bl	FlashEraseBlocks
.L427:
	ldr	r3, [r7, #3768]
	add	r2, r3, r8
	ldr	r3, [r3, r8]
	cmn	r3, #1
	bne	.L426
	ldr	r0, [r2, #4]
	add	r5, r5, #1
	ubfx	r0, r0, #10, #16
	uxth	r5, r5
	bl	FtlBbmMapBadBlock
.L426:
	add	r9, r9, #1
	add	r8, r8, #20
	uxth	r9, r9
	cmp	r9, r4
	bne	.L427
.L428:
	cmp	r6, #0
	mov	r8, #0
	mov	r7, r6
	streq	r6, [sp, #4]
	ldrne	r3, .L446+4
	moveq	r9, #1
	moveq	r1, #6
	streq	r1, [sp, #8]
	ldrneh	r9, [r3, #76]
	movne	r3, #1
	strne	r3, [sp, #4]
	movne	r2, r9, lsr #2
	strne	r2, [sp, #8]
.L436:
	mov	r6, #0
	ldr	fp, .L446
	mov	r4, r6
	b	.L429
.L432:
	mov	r1, #20
	ldr	r2, [fp, #3768]
	mul	r3, r1, r6
	mov	r1, #0
	str	r1, [r2, r3]
	mov	r1, sl
	ldr	r2, .L446+4
	add	r3, r2, r6
	ldrb	r0, [r3, #32]	@ zero_extendqisi2
	bl	V2P_block
	cmp	r7, #0
	str	r0, [sp, #12]
	beq	.L430
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L431
.L430:
	ldr	r0, [sp, #12]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L431
	ldr	r3, [fp, #3768]
	mov	r1, #20
	mla	r3, r1, r4, r3
	ldr	r1, [sp, #12]
	add	r2, r8, r1, asl #10
	ldr	r1, .L446+4
	str	r2, [r3, #4]
	ldr	r2, [fp, #3784]
	str	r2, [r3, #8]
	ldrh	r2, [r1, #84]
	mul	r2, r2, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [fp, #3776]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L431:
	add	r6, r6, #1
	uxth	r6, r6
.L429:
	ldr	r2, .L446+4
	ldrh	r3, [r2, #4]
	cmp	r3, r6
	bhi	.L432
	cmp	r4, #0
	beq	.L425
	ldr	r6, .L446
	mov	r1, r4
	ldr	r2, [sp, #4]
	mov	r3, #1
	mov	fp, #0
	ldr	r0, [r6, #3768]
	bl	FlashProgPages
	mov	ip, r7
	mov	r7, r4
	mov	r4, fp
.L435:
	ldr	r2, [r6, #3768]
	add	r1, r2, fp
	ldr	r2, [r2, fp]
	cmp	r2, #0
	beq	.L434
	ldr	r0, [r1, #4]
	add	r5, r5, #1
	str	ip, [sp, #0]
	ubfx	r0, r0, #10, #16
	uxth	r5, r5
	bl	FtlBbmMapBadBlock
	ldr	ip, [sp, #0]
.L434:
	add	r4, r4, #1
	add	fp, fp, #20
	uxth	r4, r4
	cmp	r4, r7
	bne	.L435
	ldr	r3, [sp, #8]
	mov	r4, r7
	mov	r7, ip
	add	r8, r8, r3
	uxth	r8, r8
	cmp	r8, r9
	bcc	.L436
	ldr	r9, .L446
	mov	r7, #0
	mov	r6, ip
	mov	r8, r7
.L438:
	cmp	r6, #0
	beq	.L437
	ldr	r3, [r9, #3768]
	add	r2, r3, r7
	ldr	r3, [r3, r7]
	cmp	r3, #0
	bne	.L437
	ldr	r0, [r2, #4]
	mov	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L437:
	add	r8, r8, #1
	add	r7, r7, #20
	uxth	r8, r8
	cmp	r8, r4
	bne	.L438
	cmp	sl, #63
	movhi	sl, #0
	movls	sl, #1
	cmp	r6, #0
	moveq	r6, sl
	orrne	r6, sl, #1
	cmp	r6, #0
	beq	.L425
	ldr	r3, .L446
	mov	r2, r8
	ldr	r1, [sp, #4]
	ldr	r0, [r3, #3768]
	bl	FlashEraseBlocks
.L425:
	mov	r0, r5
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L447:
	.align	2
.L446:
	.word	.LANCHOR3
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
	beq	.L449
	bl	memset
.L449:
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
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r2, #256
	mov	r5, r1
	mov	r1, #255
	mov	r4, r0
	bl	ftl_memset
	ldr	r3, .L455
	mov	r0, r4
	mov	r1, r5
	ldr	r3, [r3, #0]
	blx	r3
	uxth	r0, r0
	cmp	r0, #50
	bls	.L451
	mov	r0, r4
	mov	r1, #255
	mov	r2, #256
	bl	ftl_memset
	mov	r0, #0
.L451:
	ldr	r3, .L455+4
	ldrh	r3, [r3, #14]
	cmp	r3, #4
	moveq	r3, #0
	ldmnefd	sp!, {r3, r4, r5, pc}
	b	.L453
.L454:
	add	r3, r3, #1
	ldrh	r2, [r4, #0]
	uxth	r3, r3
	mov	r2, r2, lsr #1
	strh	r2, [r4], #2	@ movhi
.L453:
	cmp	r3, r0
	bne	.L454
	ldmfd	sp!, {r3, r4, r5, pc}
.L456:
	.align	2
.L455:
	.word	g_nand_ops
	.word	g_nand_phy_info
	.fnend
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
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
	bls	.L458
.L459:
	ldr	r5, .L462
	add	r0, sp, #24
	mov	r1, #165
	mov	r2, #32
	str	r0, [sp, #16]
	mov	r4, r4, asl #10
	str	r5, [sp, #12]
	bl	ftl_memset
	mov	r1, #90
	mov	r2, #8
	mov	r0, r5
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
.L458:
	mov	r0, r4
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, pc}
.L463:
	.align	2
.L462:
	.word	.LANCHOR0+3220
	.fnend
	.size	FlashTestBlk, .-FlashTestBlk
	.align	2
	.global	FtlGcPageVarInit
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	movw	r2, #3164
	ldr	r4, .L465
	mov	r3, #0
	mov	r1, #255
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #12
	ldr	r0, [r4, #3168]
	strh	r3, [r4, r2]	@ movhi
	ldrh	r2, [r4, #78]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r4, #78]
	mov	r2, #12
	ldr	r0, [r4, #3172]
	mov	r1, #255
	mul	r2, r2, r3
	bl	ftl_memset
	ldmfd	sp!, {r4, lr}
	b	FtlGcBufInit
.L466:
	.align	2
.L465:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	2
	.global	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L488
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldrh	r4, [r3, #4]
	movw	r3, #65535
	mov	r5, r0
	str	r1, [sp, #12]
	cmp	r4, r3
	beq	.L484
	cmp	r4, #0
	bne	.L468
	b	.L469
.L484:
	mov	r4, #0
.L468:
	ldr	r3, .L488+4
	ldr	fp, [sp, #12]
	ldrh	r3, [r3, #74]
	cmp	fp, r3
	bne	.L470
.L469:
	bl	FtlGcPageVarInit
.L470:
	mov	r7, #0
	movw	r9, #65535
.L479:
	ldrh	r3, [r5, #0]
	mov	r1, #0
	strb	r1, [r5, #8]
	cmp	r3, r9
	beq	.L485
.L472:
	mov	sl, r5
.L486:
	ldr	r3, .L488+4
	ldr	r2, .L488+8
	ldrh	r8, [r3, #4]
	ldr	r6, [r2, #3788]
	ldr	lr, [r3, #3148]
	ldr	r2, [r3, #3152]
	ldrh	ip, [r3, #82]
	ldrh	r3, [r3, #84]
	str	r2, [sp, #16]
	mov	r2, sl
	str	r3, [sp, #20]
	mov	r3, #0
	mov	r5, r3
	b	.L473
.L475:
	ldrh	r0, [r2, #16]
	cmp	r0, r9
	beq	.L474
	mov	fp, #20
	orr	r0, r4, r0, asl #10
	mla	r1, fp, r5, r6
	str	r0, [r1, #4]
	mul	r0, ip, r5
	add	fp, r0, #3
	cmp	r0, #0
	movlt	r0, fp
	ldr	fp, [sp, #20]
	bic	r0, r0, #3
	add	r0, lr, r0
	str	r0, [r1, #8]
	mul	r0, fp, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	fp, r0, #3
	cmp	r0, #0
	movlt	r0, fp
	ldr	fp, [sp, #16]
	bic	r0, r0, #3
	add	r0, fp, r0
	str	r0, [r1, #12]
.L474:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L473:
	cmp	r3, r8
	bne	.L475
	ldr	r8, .L488+8
	mov	r1, r5
	mov	r2, #0
	mov	r6, #0
	mov	fp, r6
	ldr	r0, [r8, #3788]
	bl	FlashReadPages
	b	.L476
.L480:
	ldr	r2, .L488+8
	ldr	r1, [r2, #3788]
	add	r3, r1, r6
	ldr	r8, [r3, #4]
	stmib	sp, {r1, r3}
	ubfx	r0, r8, #10, #16
	bl	P2V_plane
	ldr	r1, [sp, #4]
	ldr	r3, [sp, #8]
	ldr	r1, [r1, r6]
	cmp	r1, #0
	mov	r2, r0
	bne	.L477
	ldr	r3, [r3, #12]
	add	r6, r6, #20
	ldrh	r1, [r3, #0]
	cmp	r1, r9
	ldreq	r3, .L488+8
	moveq	r1, #1
	moveq	r5, sl
	streq	r1, [r3, #3792]
	beq	.L471
.L478:
	add	fp, fp, #1
	ldr	r0, [r3, #12]
	mov	r1, r8
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
	uxth	fp, fp
	b	.L476
.L477:
	ldr	r2, .L488+4
	mov	fp, #0	@ movhi
	ldrh	r3, [sl, #0]
	mov	r5, sl
	mov	r4, #0
	ldr	r2, [r2, #2296]
	mov	r3, r3, asl #1
	strh	fp, [r2, r3]	@ movhi
	ldrh	r0, [sl, #0]
	bl	INSERT_FREE_LIST
	mvn	r3, #0
	strh	r3, [sl, #0]	@ movhi
	bl	FtlGcPageVarInit
	b	.L479
.L476:
	cmp	fp, r5
	bne	.L480
	ldr	r3, [sp, #12]
	add	r7, r7, #1
	add	r4, r4, #1
	cmp	r7, r3
	ldr	r3, .L488+4
	uxth	r4, r4
	bcc	.L481
	ldr	r2, .L488
	ldrh	r1, [r2, #4]
	cmp	r1, r9
	beq	.L481
	add	r1, r1, r7
	strh	r1, [r2, #4]	@ movhi
	ldrh	r2, [r3, #74]
	cmp	r2, r4
	bhi	.L482
.L481:
	ldrh	r3, [r3, #74]
	cmp	r3, r4
	bhi	.L486
	mov	r5, sl
.L485:
	mov	r2, #0
.L471:
	ldr	r3, .L488
	mvn	r1, #0
	strh	r4, [r5, #2]	@ movhi
	mov	r0, r5
	strb	r2, [r5, #6]
	strh	r1, [r3, #4]	@ movhi
	mov	r1, r4
	bl	ftl_sb_update_avl_pages
.L482:
	mvn	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L489:
	.align	2
.L488:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR3
	.fnend
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	2
	.global	SupperBlkListInit
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r2, #6
	ldr	r4, .L500
	mov	r1, #0
	mov	r7, #0
	mov	r8, r7
	mov	r5, r7
	ldrh	r3, [r4, #14]
	mov	r9, r4
	ldr	r0, [r4, #2288]
	mul	r2, r2, r3
	bl	ftl_memset
	movw	r3, #2308
	strh	r7, [r4, r3]	@ movhi
	add	r3, r3, #8
	str	r7, [r4, #2312]
	str	r7, [r4, #2292]
	str	r7, [r4, #2304]
	strh	r7, [r4, r3]	@ movhi
	b	.L491
.L493:
	add	r3, r4, sl
	mov	r1, r5
	add	sl, sl, #1
	ldrb	r0, [r3, #32]	@ zero_extendqisi2
	bl	V2P_block
	uxth	sl, sl
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldreqh	r3, [r4, #74]
	addeq	r6, r6, r3
	uxtheq	r6, r6
	b	.L497
.L499:
	mov	r6, #0
	mov	sl, r6
.L497:
	ldrh	r3, [r4, #4]
	cmp	r3, sl
	bhi	.L493
	cmp	r6, #0
	beq	.L494
	mov	r1, r6
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r6, r0
.L494:
	ldr	r3, [r9, #2288]
	mov	r2, #6
	mla	r3, r2, r5, r3
	strh	r6, [r3, #4]	@ movhi
	mov	r3, #2320
	ldrh	r3, [r9, r3]
	cmp	r3, r5
	beq	.L495
	mov	r3, #2368
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L495
	mov	r3, #2416
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L495
	ldr	r2, [r4, #2296]
	mov	r3, r5, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	bne	.L496
	add	r7, r7, #1
	mov	r0, r5
	uxth	r7, r7
	bl	INSERT_FREE_LIST
	b	.L495
.L496:
	add	r8, r8, #1
	mov	r0, r5
	uxth	r8, r8
	bl	INSERT_DATA_LIST
.L495:
	add	r5, r5, #1
	uxth	r5, r5
.L491:
	ldrh	r3, [r4, #12]
	ldr	r2, .L500
	cmp	r5, r3
	bcc	.L499
	movw	r1, #2308
	strh	r8, [r2, r1]	@ movhi
	add	r1, r1, #8
	strh	r7, [r2, r1]	@ movhi
	add	r7, r8, r7
	cmp	r7, r3
	ble	.L498
	ldr	r0, .L500+4
	movw	r2, #2152
	ldr	r1, .L500+8
	bl	printk
.L498:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L501:
	.align	2
.L500:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+186
	.fnend
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	2
	.global	FtlL2PDataInit
	.type	FtlL2PDataInit, %function
FtlL2PDataInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r1, #0
	ldr	r4, .L505
	mov	r6, #12
	ldr	r5, .L505+4
	ldr	r2, [r4, #100]
	ldr	r0, [r5, #3796]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r4, #82]
	ldrh	r2, [r4, #110]
	mov	r1, #255
	ldr	r0, [r5, #3800]
	mul	r2, r2, r3
	bl	ftl_memset
	mov	r3, #0
	mov	ip, r3
	mvn	r0, #0
	b	.L503
.L504:
	mul	r2, r6, r3
	ldr	r1, [r4, #2464]
	add	r7, r1, r2
	str	ip, [r7, #4]
	strh	r0, [r1, r2]	@ movhi
	ldr	r1, [r4, #2464]
	ldr	r7, [r5, #3800]
	add	r2, r1, r2
	ldrh	r1, [r4, #82]
	mul	r1, r3, r1
	add	r3, r3, #1
	uxth	r3, r3
	bic	r1, r1, #3
	add	r1, r7, r1
	str	r1, [r2, #8]
.L503:
	ldrh	r1, [r4, #110]
	ldr	r2, .L505
	cmp	r1, r3
	bhi	.L504
	ldr	r3, .L505+4
	movw	r0, #3806
	mvn	r1, #0
	strh	r1, [r3, r0]	@ movhi
	movw	r0, #3804
	strh	r1, [r3, r0]	@ movhi
	movw	r1, #3814
	ldr	r0, [r2, #100]
	strh	r0, [r3, r1]	@ movhi
	mov	r1, #3808
	ldr	r0, .L505+8
	strh	r0, [r3, r1]	@ movhi
	add	r1, r1, #36
	ldrh	r0, [r3, r1]
	movw	r1, #3812
	strh	r0, [r3, r1]	@ movhi
	ldrh	r1, [r2, #108]
	movw	r2, #3810
	strh	r1, [r3, r2]	@ movhi
	ldr	r2, [r3, #3848]
	str	r2, [r3, #3816]
	ldr	r2, [r3, #3852]
	str	r2, [r3, #3820]
	ldr	r2, [r3, #3796]
	str	r2, [r3, #3824]
	ldr	r2, [r3, #3856]
	str	r2, [r3, #3828]
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L506:
	.align	2
.L505:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	-3902
	.fnend
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	2
	.global	FtlScanSysBlk
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r3, #3844
	ldr	r5, .L566
	mov	r6, #0
	ldr	r4, .L566+4
	.pad #28
	sub	sp, sp, #28
	mov	r1, r6
	ldr	r2, [r5, #100]
	strh	r6, [r4, r3]	@ movhi
	mov	r9, r4
	strh	r6, [r5, #116]	@ movhi
	mov	r2, r2, asl #2
	ldr	r0, [r4, #3852]
	bl	ftl_memset
	ldr	r2, [r5, #100]
	mov	r1, r6
	ldr	r0, [r4, #3848]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r5, #92]
	mov	r1, r6
	ldr	r0, [r4, #3860]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r5, #92]
	ldr	r0, [r5, #120]
	mov	r1, r6
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldr	r0, .L566+8
	mov	r1, #255
	mov	r2, #12
	bl	ftl_memset
	ldrh	r5, [r5, #12]
	str	r5, [sp, #8]
	b	.L508
.L510:
	add	r3, r5, r6
	ldr	r1, [sp, #8]
	ldrb	r0, [r3, #32]	@ zero_extendqisi2
	bl	V2P_block
	mov	r8, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L509
	ldr	r3, [r4, #3788]
	mov	r8, r8, asl #10
	ldr	r2, [r5, #3148]
	mla	r3, r7, sl, r3
	str	r2, [r3, #8]
	ldrh	r2, [r5, #84]
	str	r8, [r3, #4]
	mul	r2, r2, sl
	add	sl, sl, #1
	uxth	sl, sl
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [r5, #3152]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L509:
	add	r6, r6, #1
	uxth	r6, r6
	b	.L544
.L558:
	mov	sl, #0
	mov	r7, #20
	mov	r6, sl
.L544:
	ldrh	r3, [r5, #4]
	cmp	r3, r6
	bhi	.L510
	cmp	sl, #0
	beq	.L511
	ldr	r0, [r4, #3788]
	mov	r1, sl
	mov	r2, #1
	mov	r6, #0
	bl	FlashReadPages
	str	r6, [sp, #12]
	str	sl, [sp, #20]
.L543:
	ldr	r3, [r4, #3788]
	add	r2, r3, r6
	ldr	r3, [r3, r6]
	ldr	r7, [r2, #4]
	cmn	r3, #1
	ldr	r5, [r2, #12]
	ubfx	r7, r7, #10, #16
	bne	.L512
	mov	r8, #16
.L514:
	ldr	r0, [r4, #3788]
	mov	r1, #1
	mov	r2, r1
	add	r0, r0, r6
	ldr	r3, [r0, #4]
	add	r3, r3, #1
	str	r3, [r0, #4]
	bl	FlashReadPages
	ldrh	r3, [r5, #0]
	movw	r0, #65535
	cmp	r3, r0
	ldreq	r3, [r9, #3788]
	mvneq	r2, #0
	streq	r2, [r3, r6]
	beq	.L512
.L513:
	ldr	r3, [r4, #3788]
	ldr	r3, [r3, r6]
	cmn	r3, #1
	bne	.L512
	sub	r8, r8, #1
	uxth	r8, r8
	cmp	r8, #0
	bne	.L514
.L512:
	ldr	r3, [r4, #3788]
	ldr	r3, [r3, r6]
	cmn	r3, #1
	beq	.L515
	ldr	r2, .L566
	ldr	r3, [r5, #4]
	ldr	r1, [r2, #2516]
	cmn	r1, #1
	beq	.L516
	cmp	r1, r3
	bhi	.L517
.L516:
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [r2, #2516]
.L517:
	ldrh	r3, [r5, #0]
	movw	r1, #61604
	cmp	r3, r1
	beq	.L520
	bhi	.L523
	movw	r2, #61574
	cmp	r3, r2
	bne	.L518
	b	.L564
.L523:
	movw	r2, #61634
	cmp	r3, r2
	beq	.L521
	movw	r2, #65535
	cmp	r3, r2
	bne	.L518
	b	.L565
.L521:
	ldr	r8, .L566
	movw	r3, #3844
	ldrh	r2, [r4, r3]
	ldr	r3, [r8, #100]
	cmp	r2, r3
	bls	.L524
	ldr	r0, .L566+12
	movw	r2, #1209
	ldr	r1, .L566+16
	bl	printk
.L524:
	ldr	r0, [r8, #100]
	movw	r2, #3844
	ldrh	r1, [r4, r2]
	mov	sl, r7
	ldr	ip, [r4, #3852]
	mov	r8, r6
	uxth	fp, r0
	sub	r3, fp, #1
	rsb	fp, r1, fp
	sub	fp, fp, #1
	uxth	r3, r3
	sxth	fp, fp
	b	.L525
.L531:
	mov	r6, r2, asl #2
	ldr	r7, [r5, #4]
	str	r6, [sp, #16]
	ldr	r6, [ip, r2, asl #2]
	cmp	r7, r6
	bls	.L526
	ldr	ip, [ip, #0]
	mov	r7, sl
	mov	r6, r8
	cmp	ip, #0
	bne	.L527
	cmp	r1, r0
	addne	r1, r1, #1
	movwne	r0, #3844
	strneh	r1, [r4, r0]	@ movhi
.L527:
	mov	r0, #0
	sxth	sl, r3
	b	.L528
.L529:
	ldr	r8, [r4, #3852]
	add	ip, r1, #1
	add	r0, r0, #1
	ldr	fp, [r8, ip, asl #2]
	mov	ip, ip, asl #1
	uxth	r0, r0
	str	fp, [r8, r1, asl #2]
	mov	r1, r1, asl #1
	ldr	r8, [r4, #3848]
	ldrh	ip, [r8, ip]
	strh	ip, [r8, r1]	@ movhi
.L528:
	sxth	r1, r0
	cmp	r1, sl
	bne	.L529
	ldr	r1, [r9, #3852]
	mov	r2, r2, asl #1
	ldr	r0, [r5, #4]
	ldr	ip, [sp, #16]
	str	r0, [r1, ip]
	ldr	r1, [r9, #3848]
	strh	r7, [r1, r2]	@ movhi
	b	.L530
.L526:
	sub	r3, r3, #1
	uxth	r3, r3
.L525:
	sxth	r2, r3
	cmp	r2, fp
	bgt	.L531
	mov	r7, sl
	mov	r6, r8
.L530:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L518
	ldr	r0, .L566
	movw	r2, #3844
	ldrh	r1, [r4, r2]
	ldr	r0, [r0, #100]
	rsb	r0, r1, r0
	sub	r0, r0, #1
	sxth	r0, r0
	cmp	r3, r0
	bgt	.L518
	add	r1, r1, #1
	strh	r1, [r4, r2]	@ movhi
	ldr	r2, [r4, #3852]
	ldr	r1, [r5, #4]
	str	r1, [r2, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r4, #3848]
	b	.L542
.L564:
	ldr	r8, .L566
	ldrh	r2, [r8, #116]
	ldrh	r3, [r8, #92]
	cmp	r2, r3
	bls	.L532
	ldr	r0, .L566+12
	movw	r2, #1250
	ldr	r1, .L566+16
	bl	printk
.L532:
	ldrh	r0, [r8, #92]
	mov	sl, r7
	ldrh	r1, [r8, #116]
	mov	r8, r6
	sub	fp, r0, #1
	ldr	ip, [r4, #3860]
	uxth	r3, fp
	rsb	fp, r1, fp
	b	.L533
.L539:
	mov	r6, r2, asl #2
	ldr	r7, [r5, #4]
	str	r6, [sp, #16]
	ldr	r6, [ip, r2, asl #2]
	cmp	r7, r6
	bls	.L534
	ldr	ip, [ip, #0]
	mov	r7, sl
	mov	r6, r8
	cmp	ip, #0
	bne	.L535
	cmp	r1, r0
	addne	r1, r1, #1
	ldrne	r0, .L566
	strneh	r1, [r0, #116]	@ movhi
.L535:
	mov	r0, #0
	sxth	fp, r3
	ldr	sl, .L566
	mov	r8, r3
	str	r2, [sp, #4]
	b	.L536
.L537:
	ldr	ip, [r4, #3860]
	add	r1, r3, #1
	add	r0, r0, #1
	ldr	r2, [ip, r1, asl #2]
	mov	r1, r1, asl #1
	uxth	r0, r0
	str	r2, [ip, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	ip, [sl, #120]
	ldrh	r1, [ip, r1]
	strh	r1, [ip, r3]	@ movhi
.L536:
	sxth	r3, r0
	cmp	r3, fp
	bne	.L537
	ldr	r1, [r9, #3860]
	mov	r3, r8
	ldr	r0, [r5, #4]
	ldr	ip, [sp, #16]
	ldr	r2, [sp, #4]
	str	r0, [r1, ip]
	ldr	r1, .L566
	mov	r2, r2, asl #1
	ldr	r1, [r1, #120]
	strh	r7, [r1, r2]	@ movhi
	b	.L538
.L534:
	sub	r3, r3, #1
	uxth	r3, r3
.L533:
	sxth	r2, r3
	cmp	r2, fp
	bgt	.L539
	mov	r7, sl
	mov	r6, r8
.L538:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L518
	ldr	r2, .L566
	ldrh	r0, [r2, #92]
	ldrh	r1, [r2, #116]
	sub	r0, r0, #1
	rsb	r0, r1, r0
	sxth	r0, r0
	cmp	r3, r0
	bgt	.L518
	add	r1, r1, #1
	ldr	r0, [r5, #4]
	strh	r1, [r2, #116]	@ movhi
	ldr	r1, [r4, #3860]
	str	r0, [r1, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r2, #120]
	b	.L542
.L520:
	movw	r1, #2552
	movw	r0, #65535
	ldrh	r2, [r2, r1]
	ldr	r3, .L566
	cmp	r2, r0
	beq	.L563
.L540:
	movw	r2, #2556
	ldrh	r0, [r3, r2]
	movw	r2, #65535
	cmp	r0, r2
	beq	.L541
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L541:
	ldr	r3, .L566
	ldr	r2, [r5, #4]
	ldr	r1, [r3, #2560]
	cmp	r1, r2
	movw	r2, #2556
	bcs	.L542
	movw	r1, #2552
	ldrh	r0, [r3, r1]
	strh	r0, [r3, r2]	@ movhi
.L563:
	ldr	r2, [r5, #4]
	strh	r7, [r3, r1]	@ movhi
	str	r2, [r3, #2560]
	b	.L518
.L542:
	strh	r7, [r3, r2]	@ movhi
	b	.L518
.L565:
	mov	r0, r7
	mov	r1, #0
	b	.L562
.L515:
	mov	r0, r7
	mov	r1, #1
.L562:
	bl	FtlFreeSysBlkQueueIn
.L518:
	ldr	ip, [sp, #12]
	add	r6, r6, #20
	add	r3, ip, #1
	ldr	ip, [sp, #20]
	uxth	r3, r3
	str	r3, [sp, #12]
	cmp	r3, ip
	bne	.L543
.L511:
	ldr	r6, [sp, #8]
	add	r3, r6, #1
	uxth	r3, r3
	str	r3, [sp, #8]
.L508:
	ldr	r5, .L566
	ldr	ip, [sp, #8]
	ldrh	r3, [r5, #14]
	cmp	r3, ip
	bhi	.L558
	ldr	r1, .L566+4
	ldr	r2, [r1, #3848]
	ldrh	r3, [r2, #0]
	cmp	r3, #0
	bne	.L545
	movw	r0, #3844
	ldrh	r1, [r1, r0]
	cmp	r1, #0
	ldrne	r0, [r5, #100]
	bne	.L546
	b	.L545
.L550:
	mov	r1, r1, asl #1
	ldrh	r1, [r2, r1]
	cmp	r1, #0
	beq	.L547
	ldr	r7, .L566
	sxth	r6, r3
	ldr	r1, .L566+4
	mov	r5, #0
	b	.L548
.L549:
	ldr	ip, [r1, #3848]
	mov	r0, r2, asl #1
	rsb	r4, r6, r2
	add	r3, r3, #1
	ldrh	sl, [ip, r0]
	mov	r8, r4, asl #1
	uxth	r3, r3
	strh	sl, [ip, r8]	@ movhi
	ldr	ip, [r1, #3852]
	ldr	r2, [ip, r2, asl #2]
	str	r2, [ip, r4, asl #2]
	ldr	r2, [r1, #3848]
	strh	r5, [r2, r0]	@ movhi
.L548:
	ldr	r0, [r7, #100]
	sxth	r2, r3
	cmp	r2, r0
	bcc	.L549
	b	.L545
.L547:
	add	r3, r3, #1
	uxth	r3, r3
.L546:
	sxth	r1, r3
	cmp	r1, r0
	bcc	.L550
.L545:
	ldr	r2, .L566
	ldr	r1, [r2, #120]
	ldrh	r3, [r1, #0]
	cmp	r3, #0
	bne	.L551
	ldrh	r0, [r2, #116]
	cmp	r0, #0
	ldrneh	r2, [r2, #92]
	bne	.L552
	b	.L551
.L556:
	mov	ip, r0, asl #1
	ldrh	ip, [r1, ip]
	cmp	ip, #0
	beq	.L553
	ldr	r1, .L566
	mov	r6, #0
	ldr	r7, .L566+4
	b	.L554
.L555:
	ldr	r4, [r1, #120]
	mov	ip, r2, asl #1
	rsb	r5, r0, r2
	add	r3, r3, #1
	ldrh	sl, [r4, ip]
	mov	r8, r5, asl #1
	uxth	r3, r3
	strh	sl, [r4, r8]	@ movhi
	ldr	r4, [r7, #3860]
	ldr	r2, [r4, r2, asl #2]
	str	r2, [r4, r5, asl #2]
	ldr	r2, [r1, #120]
	strh	r6, [r2, ip]	@ movhi
.L554:
	ldrh	ip, [r1, #92]
	sxth	r2, r3
	cmp	r2, ip
	blt	.L555
	b	.L551
.L553:
	add	r3, r3, #1
	uxth	r3, r3
.L552:
	sxth	r0, r3
	cmp	r0, r2
	blt	.L556
.L551:
	ldr	r2, .L566+4
	movw	r3, #3844
	ldrh	r2, [r2, r3]
	ldr	r3, .L566
	ldr	r3, [r3, #100]
	cmp	r2, r3
	bls	.L557
	ldr	r0, .L566+12
	movw	r2, #1375
	ldr	r1, .L566+16
	bl	printk
.L557:
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L567:
	.align	2
.L566:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR0+2552
	.word	.LC1
	.word	.LANCHOR1+204
	.fnend
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	2
	.global	ftl_free_no_use_map_blk
	.type	ftl_free_no_use_map_blk, %function
ftl_free_no_use_map_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r0
	ldrh	r2, [r0, #10]
	mov	r1, #0
	ldr	r5, [r0, #20]
	ldr	r6, [r0, #12]
	ldr	r7, [r0, #24]
	mov	r2, r2, asl #1
	mov	r0, r5
	bl	ftl_memset
	mov	r3, #0
	b	.L569
.L573:
	ldr	r0, [r7, r3, asl #2]
	mov	r2, #0
	ubfx	r0, r0, #10, #16
	b	.L570
.L572:
	mov	r1, r2, asl #1
	add	r2, r2, #1
	ldrh	ip, [r6, r1]
	uxth	r2, r2
	cmp	ip, r0
	ldreqh	ip, [r5, r1]
	addeq	ip, ip, #1
	streqh	ip, [r5, r1]	@ movhi
.L570:
	ldrh	r1, [r4, #10]
	cmp	r1, r2
	bhi	.L572
	add	r3, r3, #1
	uxth	r3, r3
.L569:
	ldrh	r2, [r4, #6]
	cmp	r2, r3
	bhi	.L573
	mov	sl, #0
	ldrh	fp, [r5, #0]
	mov	r7, sl
	ldr	r3, .L579
	b	.L574
.L578:
	ldrh	r2, [r4, #0]
	cmp	r2, r7
	bne	.L575
	ldrh	r1, [r4, #2]
	ldrh	r2, [r3, #76]
	cmp	r1, r2
	movcc	r1, r7, asl #1
	strcch	r2, [r5, r1]	@ movhi
.L575:
	mov	r9, r7, asl #1
	ldrh	r8, [r5, r9]
	cmp	fp, r8
	movhi	sl, r7
	movhi	fp, r8
	cmp	r8, #0
	bne	.L577
	ldrh	r0, [r6, r9]
	cmp	r0, #0
	beq	.L577
	mov	r1, #1
	str	r3, [sp, #4]
	bl	FtlFreeSysBlkQueueIn
	strh	r8, [r6, r9]	@ movhi
	ldr	r3, [sp, #4]
	ldrh	r2, [r4, #8]
	sub	r2, r2, #1
	strh	r2, [r4, #8]	@ movhi
.L577:
	add	r7, r7, #1
	uxth	r7, r7
.L574:
	ldrh	r2, [r4, #10]
	cmp	r2, r7
	bhi	.L578
	mov	r0, sl
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L580:
	.align	2
.L579:
	.word	.LANCHOR0
	.fnend
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
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
	bne	.L582
	ldrh	r3, [r0, #8]
	cmp	r3, #0
	beq	.L583
	ldr	r0, .L588
	movw	r2, #639
	ldr	r1, .L588+4
	bl	printk
.L583:
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
	b	.L584
.L582:
	mov	r3, r3, asl #1
	ldr	r2, [r0, #28]
	mov	r1, #255
	ldrh	r6, [r5, r3]
	ldr	r5, .L588+8
	ldrh	r3, [r0, #2]
	orr	r3, r3, r6, asl #10
	str	r3, [r5, #-864]
	ldr	r3, [r5, #-848]
	str	r3, [r5, #-860]
	ldr	r3, [r5, #-844]
	str	r3, [r5, #-856]
	str	r2, [r3, #4]
	ldr	r2, .L588+12
	strh	r2, [r3, #8]	@ movhi
	ldrh	r2, [r0, #4]
	strh	r6, [r3, #2]	@ movhi
	strh	r2, [r3, #0]	@ movhi
	ldr	r3, .L588+16
	ldr	r0, [r5, #-848]
	ldrh	r2, [r3, #76]
	mov	r2, r2, asl #3
	bl	ftl_memset
	mov	r2, #0
	mov	r3, r2
	b	.L585
.L587:
	ldr	r1, [r7, r3, asl #2]
	cmp	r6, r1, lsr #10
	bne	.L586
	add	r2, r2, #1
	ldr	r1, [r5, #-848]
	uxth	r2, r2
	str	r3, [r1, r2, asl #3]
	ldr	r1, [r5, #-848]
	ldr	r0, [r7, r3, asl #2]
	add	r1, r1, r2, asl #3
	str	r0, [r1, #4]
.L586:
	add	r3, r3, #1
	uxth	r3, r3
.L585:
	ldrh	r1, [r4, #6]
	cmp	r1, r3
	bhi	.L587
	mov	r1, #1
	mov	r3, #0
	ldr	r0, .L588+20
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	add	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
.L584:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L589:
	.align	2
.L588:
	.word	.LC1
	.word	.LANCHOR1+218
	.word	.LANCHOR3
	.word	-1291
	.word	.LANCHOR0
	.word	.LANCHOR3-868
	.fnend
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.align	2
	.global	FtlMapWritePage
	.type	FtlMapWritePage, %function
FtlMapWritePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r0
	ldr	r7, .L606
	mov	r9, r1
	ldr	r5, .L606+4
	mov	fp, r2
	mov	r6, #0
.L604:
	ldr	r3, [r7, #2500]
	add	r3, r3, #1
	str	r3, [r7, #2500]
	ldrh	r3, [r7, #76]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	cmp	r2, r3
	bge	.L592
	ldrh	r3, [r4, #0]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L593
.L592:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L593:
	ldrh	r3, [r4, #0]
	ldr	r2, [r4, #12]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	bne	.L594
	ldr	r0, .L606+8
	movw	r2, #697
	ldr	r1, .L606+12
	bl	printk
.L594:
	ldrh	r2, [r4, #0]
	ldrh	r3, [r4, #10]
	cmp	r2, r3
	bcc	.L595
	ldr	r0, .L606+8
	movw	r2, #698
	ldr	r1, .L606+12
	bl	printk
.L595:
	ldrh	r3, [r4, #0]
	mov	r1, #0
	ldr	r2, [r4, #12]
	ldr	r0, [r5, #-844]
	mov	r3, r3, asl #1
	ldr	r8, .L606+4
	ldrh	sl, [r2, r3]
	mov	r2, #16
	ldrh	r3, [r4, #2]
	str	fp, [r5, #-860]
	orr	r3, r3, sl, asl #10
	str	r0, [r5, #-856]
	str	r3, [r5, #-864]
	bl	ftl_memset
	ldr	r3, [r5, #-856]
	mov	r1, #1
	ldr	r2, [r4, #28]
	sub	r0, r8, #868
	strh	r9, [r3, #8]	@ movhi
	str	r2, [r3, #4]
	ldrh	r2, [r4, #4]
	strh	sl, [r3, #2]	@ movhi
	strh	r2, [r3, #0]	@ movhi
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r4, #2]	@ movhi
	ldr	r2, [r5, #-868]
	cmn	r2, #1
	bne	.L596
	ldr	r0, .L606+16
	add	r6, r6, #1
	ldr	r1, [r5, #-864]
	bl	printk
	ldrh	r3, [r4, #2]
	uxth	r6, r6
	cmp	r3, #2
	ldrlsh	r3, [r7, #76]
	subls	r3, r3, #1
	strlsh	r3, [r4, #2]	@ movhi
	cmp	r6, #3
	bls	.L598
	ldr	r3, .L606+4
	mov	r2, r6
	ldr	r0, .L606+20
	ldr	r1, [r3, #-864]
	bl	printk
.L599:
	b	.L599
.L598:
	ldr	r3, [r4, #32]
	cmp	r3, #0
	beq	.L604
.L605:
	b	.L605
.L596:
	cmp	r3, #1
	beq	.L604
	ldr	r2, [r8, #-864]
	mov	r0, #0
	ldr	r3, [r4, #24]
	str	r2, [r3, r9, asl #2]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L607:
	.align	2
.L606:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC1
	.word	.LANCHOR1+249
	.word	.LC81
	.word	.LC82
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
	ldr	r8, [r0, #24]
	bl	ftl_free_no_use_map_blk
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #8]
	sub	r3, r3, #4
	cmp	r2, r3
	blt	.L609
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r7, [r5, r0]
	cmp	r7, #0
	beq	.L609
	ldr	r3, [r4, #32]
	cmp	r3, #0
	bne	.L609
	mov	r2, #1
	str	r2, [r4, #32]
	strh	r3, [r5, r0]	@ movhi
	ldrh	r3, [r4, #8]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, .L617
	ldrh	r3, [r3, #76]
	cmp	r2, r3
	bcc	.L610
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L610:
	ldr	r6, .L617+4
	mov	r5, #0
	mov	r9, r6
	b	.L611
.L615:
	ldr	r3, [r8, r5, asl #2]
	mov	fp, r5, asl #2
	cmp	r7, r3, lsr #10
	bne	.L612
	ldr	r3, [r6, #3864]
	mov	r1, #1
	ldr	sl, [r6, #-844]
	mov	r2, r1
	ldr	r0, .L617+8
	str	r3, [r6, #-860]
	str	sl, [r6, #-856]
	ldr	r3, [r8, r5, asl #2]
	str	r3, [r6, #-864]
	bl	FlashReadPages
	ldrh	r3, [sl, #8]
	cmp	r3, r5
	beq	.L613
	ldr	r0, .L617+12
	movw	r2, #611
	ldr	r1, .L617+16
	bl	printk
.L613:
	ldr	r3, [r9, #-868]
	cmn	r3, #1
	moveq	r3, #0
	streq	r3, [r8, fp]
	beq	.L612
	mov	r0, r4
	mov	r1, r5
	ldr	r2, [r6, #-860]
	bl	FtlMapWritePage
.L612:
	add	r5, r5, #1
	uxth	r5, r5
.L611:
	ldrh	r3, [r4, #6]
	cmp	r3, r5
	bhi	.L615
	mov	r0, r7
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	mov	r3, #0
	str	r3, [r4, #32]
.L609:
	ldr	r3, .L617
	ldrh	r2, [r4, #2]
	ldrh	r3, [r3, #76]
	cmp	r2, r3
	bcc	.L616
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L616:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L618:
	.align	2
.L617:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR3-868
	.word	.LC1
	.word	.LANCHOR1+265
	.fnend
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	2
	.global	FtlMapTblRecovery
	.type	FtlMapTblRecovery, %function
FtlMapTblRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r3, [r0, #16]
	mov	r4, r0
	ldr	r7, [r0, #24]
	mov	r1, #0
	ldrh	r8, [r0, #6]
	mov	fp, #0
	str	r3, [sp, #16]
	ldrh	r3, [r0, #8]
	mov	sl, r7
	ldr	r9, [r0, #12]
	mov	r2, r8, asl #2
	mov	r0, r7
	str	r3, [sp, #8]
	bl	ftl_memset
	ldr	r3, .L639
	str	fp, [r4, #32]
	str	fp, [r4, #28]
	ldr	r2, [r3, #-848]
	ldr	r5, [r3, #-844]
	str	r2, [r3, #-860]
	str	r5, [r3, #-856]
	mvn	r3, #0
	strh	r3, [r4, #0]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #1
	str	r3, [r4, #36]
	ldr	r3, [sp, #8]
	sub	r3, r3, #1
	str	r3, [sp, #12]
	ldr	r3, .L639+4
	b	.L620
.L633:
	ldr	r0, [sp, #12]
	cmp	r2, r0
	mov	r2, r2, asl #1
	bne	.L621
	ldrh	r0, [r9, r2]
	mov	r1, #1
	add	r3, r9, r2
	str	r3, [sp, #8]
	bl	FtlGetLastWrittenPage
	ldr	r1, [sp, #12]
	ldr	r2, [sp, #16]
	mov	r7, sl
	strh	fp, [r4, #0]	@ movhi
	mov	sl, #0
	ldr	r6, .L639
	uxth	r0, r0
	add	r3, r0, #1
	strh	r3, [r4, #2]	@ movhi
	ldr	r3, [r2, r1, asl #2]
	sxth	r9, r0
	add	r9, r9, #1
	str	r3, [r4, #28]
	b	.L622
.L624:
	ldr	r0, [sp, #8]
	mov	r1, #1
	ldrh	r2, [r0, #0]
	ldr	r0, .L639+8
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r6, #-864]
	bl	FlashReadPages
	ldr	r3, [r6, #-868]
	cmn	r3, #1
	beq	.L623
	ldrh	r3, [r5, #8]
	cmp	r3, r8
	bcs	.L623
	ldrh	r2, [r4, #4]
	ldrh	r1, [r5, #0]
	cmp	r1, r2
	ldreq	r2, [r6, #-864]
	streq	r2, [r7, r3, asl #2]
.L623:
	add	sl, sl, #1
	uxth	sl, sl
.L622:
	sxth	r3, sl
	cmp	r3, r9
	blt	.L624
	b	.L625
.L621:
	ldr	r6, .L639
	sub	r0, r6, #868
	ldr	r1, [r6, #-848]
	str	r1, [r6, #-860]
	add	r1, r9, r2
	str	r1, [sp, #20]
	ldrh	r1, [r9, r2]
	ldrh	r2, [r3, #76]
	sub	r2, r2, #1
	orr	r2, r2, r1, asl #10
	mov	r1, #1
	str	r2, [r6, #-864]
	mov	r2, r1
	str	r3, [sp, #4]
	bl	FlashReadPages
	ldr	r2, [r6, #-868]
	ldr	r3, [sp, #4]
	cmn	r2, #1
	beq	.L636
	ldrh	r1, [r5, #0]
	ldrh	r2, [r4, #4]
	cmp	r1, r2
	bne	.L636
	ldrh	r1, [r5, #8]
	movw	r2, #64245
	cmp	r1, r2
	bne	.L636
	b	.L637
.L629:
	ldr	r0, [r6, #-848]
	mov	ip, r1, asl #3
	add	r2, r2, #1
	ldr	r1, [r0, r1, asl #3]
	uxth	r2, r2
	uxth	r1, r1
	cmp	r1, r8
	addcc	r0, r0, ip
	ldrcc	r0, [r0, #4]
	strcc	r0, [sl, r1, asl #2]
	b	.L627
.L637:
	mov	r2, #0
.L627:
	ldrh	r0, [r3, #76]
	sxth	r1, r2
	sub	r0, r0, #1
	cmp	r1, r0
	blt	.L629
	b	.L630
.L632:
	ldr	r0, [sp, #20]
	ldrh	r1, [r0, #0]
	ldr	r0, .L639+8
	orr	r2, r2, r1, asl #10
	mov	r1, #1
	str	r2, [r7, #-864]
	mov	r2, r1
	str	r3, [sp, #4]
	bl	FlashReadPages
	ldr	r2, [r7, #-868]
	ldr	r3, [sp, #4]
	cmn	r2, #1
	beq	.L631
	ldrh	r2, [r5, #8]
	cmp	r2, r8
	bcs	.L631
	ldrh	r1, [r4, #4]
	ldrh	r0, [r5, #0]
	cmp	r0, r1
	ldreq	r1, [r7, #-864]
	streq	r1, [sl, r2, asl #2]
.L631:
	add	r6, r6, #1
	uxth	r6, r6
	b	.L638
.L636:
	ldr	r7, .L639
	mov	r6, #0
.L638:
	ldrh	r1, [r3, #76]
	sxth	r2, r6
	cmp	r2, r1
	blt	.L632
.L630:
	add	fp, fp, #1
	uxth	fp, fp
.L620:
	ldr	r1, [sp, #8]
	sxth	r2, fp
	cmp	r2, r1
	blt	.L633
.L625:
	mov	r0, r4
	bl	ftl_free_no_use_map_blk
	ldr	r3, .L639+4
	ldrh	r2, [r4, #2]
	ldrh	r3, [r3, #76]
	cmp	r2, r3
	bne	.L634
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L634:
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L640:
	.align	2
.L639:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	.LANCHOR3-868
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
	ldr	r0, .L642
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L643:
	.align	2
.L642:
	.word	.LANCHOR3+3804
	.fnend
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.align	2
	.global	FtlLoadVonderInfo
	.type	FtlLoadVonderInfo, %function
FtlLoadVonderInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L645
	movw	r1, #3878
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L645+4
	ldrh	r0, [r2, #92]
	strh	r0, [r3, r1]	@ movhi
	mov	r1, #3872
	ldr	r0, .L645+8
	strh	r0, [r3, r1]	@ movhi
	add	r1, r1, #4
	ldrh	r0, [r2, #116]
	strh	r0, [r3, r1]	@ movhi
	movw	r1, #3874
	ldrh	r0, [r2, #94]
	ldr	r2, [r2, #120]
	strh	r0, [r3, r1]	@ movhi
	str	r2, [r3, #3880]
	ldr	r2, [r3, #3860]
	ldr	r0, .L645+12
	str	r2, [r3, #3884]
	ldr	r2, [r3, #3908]
	str	r2, [r3, #3888]
	ldr	r2, [r3, #3912]
	str	r2, [r3, #3892]
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L646:
	.align	2
.L645:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	-3962
	.word	.LANCHOR3+3868
	.fnend
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
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
	ldr	r5, .L648
	mul	r4, r4, r0
	ldr	r0, .L648+4
	ldr	r2, [r5, #2464]
	add	r3, r2, r4
	ldrh	r1, [r2, r4]
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
	ldr	r3, [r5, #2464]
	mov	r0, #0
	add	r4, r3, r4
	ldr	r3, [r4, #4]
	bic	r3, r3, #-2147483648
	str	r3, [r4, #4]
	ldmfd	sp!, {r3, r4, r5, pc}
.L649:
	.align	2
.L648:
	.word	.LANCHOR0
	.word	.LANCHOR3+3804
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
	ldr	r5, .L654
	sub	r6, r6, #1
	mov	r2, #0
	str	r2, [r0, #36]
	uxth	r6, r6
	ldr	r0, [r5, #3864]
	ldr	r2, [r5, #-844]
	str	r0, [r5, #-860]
	str	r2, [r5, #-856]
	ldr	r3, [r3, r6, asl #2]
	cmp	r3, #0
	str	r3, [r5, #-864]
	beq	.L652
	mov	r1, #1
	sub	r0, r5, #868
	mov	r2, r1
	bl	FlashReadPages
	b	.L653
.L652:
	ldr	r3, .L654+4
	mov	r1, #255
	ldrh	r2, [r3, #82]
	bl	ftl_memset
.L653:
	mov	r0, r4
	mov	r1, r6
	ldr	r2, [r5, #-860]
	ldmfd	sp!, {r4, r5, r6, lr}
	b	FtlMapWritePage
.L655:
	.align	2
.L654:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.fnend
	.size	FtlMapBlkWriteDump_data, .-FtlMapBlkWriteDump_data
	.align	2
	.global	load_l2p_region
	.type	load_l2p_region, %function
load_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r5, r0
	ldr	r3, .L662
	mov	r7, r1
	ldrh	r3, [r3, #108]
	cmp	r3, r0
	bcs	.L657
	ldr	r0, .L662+4
	movw	r2, #490
	ldr	r1, .L662+8
	bl	printk
.L657:
	ldr	r4, .L662+12
	mov	r8, #12
	ldr	r6, .L662
	ldr	r3, [r4, #3856]
	ldr	sl, [r3, r5, asl #2]
	cmp	sl, #0
	bne	.L658
	mul	r8, r8, r7
	ldr	r3, [r6, #2464]
	mov	r1, #255
	ldrh	r2, [r6, #82]
	add	r3, r3, r8
	ldr	r0, [r3, #8]
	bl	ftl_memset
	ldr	r3, [r6, #2464]
	strh	r5, [r3, r8]	@ movhi
	ldr	r3, [r6, #2464]
	add	r8, r3, r8
	str	sl, [r8, #4]
	b	.L659
.L658:
	mul	r8, r8, r7
	ldr	r3, [r6, #2464]
	mov	r1, #1
	sub	r0, r4, #868
	mov	r2, r1
	str	sl, [r4, #-864]
	add	r3, r3, r8
	ldr	r3, [r3, #8]
	str	r3, [r4, #-860]
	ldr	r3, [r4, #-844]
	str	r3, [r4, #-856]
	bl	FlashReadPages
	ldr	r7, [r4, #-856]
	ldrh	r3, [r7, #8]
	cmp	r3, r5
	beq	.L660
	mov	r1, r5
	mov	r2, sl
	ldr	r0, .L662+16
	bl	printk
	mov	r2, #4
	mov	r3, r2
	ldr	r0, .L662+20
	ldr	r1, [r4, #-856]
	bl	rknand_print_hex
	ldr	r0, .L662+24
	ldr	r1, [r4, #3856]
	mov	r2, #4
	ldrh	r3, [r6, #108]
	bl	rknand_print_hex
.L660:
	ldrh	r3, [r7, #8]
	cmp	r3, r5
	beq	.L661
	ldr	r0, .L662+4
	movw	r2, #511
	ldr	r1, .L662+8
	bl	printk
.L661:
	ldr	r3, .L662
	mov	r1, #0
	ldr	r3, [r3, #2464]
	add	r2, r3, r8
	str	r1, [r2, #4]
	strh	r5, [r3, r8]	@ movhi
.L659:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L663:
	.align	2
.L662:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+280
	.word	.LANCHOR3
	.word	.LC83
	.word	.LC84
	.word	.LC85
	.fnend
	.size	load_l2p_region, .-load_l2p_region
	.align	2
	.global	log2phys
	.type	log2phys, %function
log2phys:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mvn	r3, #0
	ldr	r7, .L674
	mov	r6, r1
	mov	r8, r2
	ldrh	r5, [r7, #80]
	add	r5, r5, #7
	mov	r4, r0, lsr r5
	bic	r5, r0, r3, asl r5
	ldr	r3, [r7, #2480]
	uxth	r4, r4
	cmp	r0, r3
	uxth	r5, r5
	bcc	.L665
	ldr	r0, .L674+4
	movw	r2, #806
	ldr	r1, .L674+8
	bl	printk
.L665:
	mov	r3, #0
	ldrh	r1, [r7, #110]
	ldr	r2, [r7, #2464]
	mov	r7, r3
	b	.L666
.L672:
	add	r3, r3, #12
	add	r0, r2, r3
	ldrh	r0, [r0, #-12]
	cmp	r0, r4
	bne	.L667
.L668:
	cmp	r8, #0
	ldr	r3, .L674
	mov	r2, #12
	bne	.L669
	ldr	r3, [r3, #2464]
	mla	r2, r2, r7, r3
	ldr	r3, [r2, #8]
	ldr	r3, [r3, r5, asl #2]
	str	r3, [r6, #0]
	b	.L670
.L669:
	mul	r2, r2, r7
	ldr	r1, [r3, #2464]
	ldr	r0, [r6, #0]
	add	r1, r1, r2
	ldr	r1, [r1, #8]
	str	r0, [r1, r5, asl #2]
	ldr	r1, [r3, #2464]
	add	r2, r1, r2
	ldr	r1, [r2, #4]
	orr	r1, r1, #-2147483648
	str	r1, [r2, #4]
	movw	r2, #2468
	strh	r4, [r3, r2]	@ movhi
.L670:
	ldr	r3, .L674
	mov	r2, #12
	mov	r0, #0
	ldr	r3, [r3, #2464]
	mla	r7, r2, r7, r3
	ldr	r3, [r7, #4]
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [r7, #4]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L667:
	add	r7, r7, #1
	uxth	r7, r7
.L666:
	cmp	r7, r1
	bne	.L672
	bl	select_l2p_ram_region
	mov	r3, #12
	ldr	r2, .L674
	ldr	r2, [r2, #2464]
	mul	r3, r3, r0
	mov	r7, r0
	add	r1, r2, r3
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L673
	ldr	r3, [r1, #4]
	cmp	r3, #0
	bge	.L673
	bl	flush_l2p_region
.L673:
	mov	r0, r4
	mov	r1, r7
	bl	load_l2p_region
	b	.L668
.L675:
	.align	2
.L674:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+296
	.fnend
	.size	log2phys, .-log2phys
	.align	2
	.type	FtlReadRefresh.part.8, %function
FtlReadRefresh.part.8:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #92
	sub	sp, sp, #92
	ldr	r4, .L681
	mov	r5, #2048
.L679:
	ldr	r0, [r4, #2696]
	ldr	r3, [r4, #2480]
	cmp	r0, r3
	bcs	.L677
	mov	r2, #0
	mov	r1, sp
	bl	log2phys
	ldr	r2, [sp, #0]
	ldr	r3, [r4, #2696]
	cmn	r2, #1
	add	r3, r3, #1
	str	r3, [r4, #2696]
	beq	.L678
	str	r3, [sp, #20]
	add	r0, sp, #88
	ldr	r3, .L681+4
	mov	r1, #1
	str	r2, [sp, #8]
	mov	r2, #0
	str	r2, [r0, #-84]!
	str	r3, [sp, #12]
	add	r3, sp, #24
	str	r3, [sp, #16]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmp	r3, #256
	bne	.L677
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L677
.L678:
	subs	r5, r5, #1
	bne	.L679
.L677:
	mvn	r0, #0
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, pc}
.L682:
	.align	2
.L681:
	.word	.LANCHOR0
	.word	.LANCHOR0+3220
	.fnend
	.size	FtlReadRefresh.part.8, .-FtlReadRefresh.part.8
	.align	2
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L689
	ldr	r2, [r3, #2692]
	cmp	r2, #0
	mov	r2, r3
	beq	.L684
	ldr	r1, [r3, #2696]
	ldr	r2, [r3, #2480]
	cmp	r1, r2
	bcs	.L685
	b	FtlReadRefresh.part.8
.L685:
	mov	r2, #0
	str	r2, [r3, #2692]
	str	r2, [r3, #2696]
	ldr	r2, [r3, #2484]
	str	r2, [r3, #2688]
	b	.L686
.L684:
	ldr	r1, [r3, #2688]
	ldr	r0, [r3, #2540]
	ldr	r3, [r3, #2484]
	add	ip, r3, #1048576
	cmp	r1, ip
	bhi	.L687
	mov	r0, r0, lsr #10
	mov	ip, #33554432
	add	r1, r1, ip, asr r0
	cmp	r1, r3
	bcc	.L687
	ldrb	r1, [r2, #2268]	@ zero_extendqisi2
	cmp	r1, #0
	bne	.L686
.L687:
	mov	r1, #1
	str	r3, [r2, #2688]
	str	r1, [r2, #2692]
	mov	r1, #0
	str	r1, [r2, #2696]
.L686:
	mov	r0, #0
	bx	lr
.L690:
	.align	2
.L689:
	.word	.LANCHOR0
	.fnend
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	2
	.global	ftl_scan_all_data
	.type	ftl_scan_all_data, %function
ftl_scan_all_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r1, #0
	ldr	r4, .L697
	.pad #32
	sub	sp, sp, #32
	ldr	r0, .L697+4
	mov	r5, #0
	bl	printk
	ldr	r7, .L697+8
	mov	r6, r4
	b	.L692
.L696:
	mov	r0, r5
	add	r1, sp, #28
	mov	r2, #0
	bl	log2phys
	movs	r3, r5, asl #21
	bne	.L693
	ldr	r0, .L697+12
	mov	r1, r5
	ldr	r2, [sp, #28]
	bl	printk
.L693:
	ldr	r3, [sp, #28]
	cmn	r3, #1
	beq	.L694
	str	r3, [r4, #-864]
	mov	r2, #0
	ldr	r3, [r4, #-848]
	mov	r1, #1
	ldr	r8, [r4, #-844]
	ldr	r0, .L697+16
	str	r3, [r4, #-860]
	str	r5, [r4, #-852]
	str	r8, [r4, #-856]
	str	r2, [r4, #-868]
	bl	FlashReadPages
	ldr	r3, [r4, #-868]
	cmn	r3, #1
	cmpne	r3, #256
	beq	.L695
	ldr	r3, [r8, #8]
	cmp	r3, r5
	beq	.L694
.L695:
	ldr	r3, [r6, #-856]
	ldr	r2, [r6, #-860]
	ldr	r0, .L697+20
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
	ldr	r2, [r6, #-864]
	ldr	r3, [r3, #0]
	bl	printk
.L694:
	add	r5, r5, #1
.L692:
	ldr	r3, [r7, #2480]
	cmp	r5, r3
	bcc	.L696
	add	sp, sp, #32
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L698:
	.align	2
.L697:
	.word	.LANCHOR3
	.word	.LC86
	.word	.LANCHOR0
	.word	.LC87
	.word	.LANCHOR3-868
	.word	.LC88
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
	ldr	r2, .L706
	ldr	r3, [r2, #2296]
	mov	r4, r0, asl #1
	mov	r5, r0
	ldrh	r1, [r3, r4]
	cmp	r1, #0
	addne	r1, r1, #1
	strneh	r1, [r3, r4]	@ movhi
	bne	.L701
	ldr	r3, [r2, #2312]
	cmp	r3, #0
	beq	.L701
	movw	r0, #2316
	mov	ip, #6
	ldrh	lr, [r2, r0]
	ldr	r2, [r2, #2288]
	ldr	r0, .L706+4
	rsb	r3, r2, r3
	mov	r3, r3, asr #1
	mul	r3, r0, r3
	movw	r0, #65535
	uxth	r3, r3
	b	.L702
.L705:
	cmp	r3, r5
	bne	.L703
	ldr	r6, .L706
	mov	r1, r5
	ldr	r0, .L706+8
	movw	r7, #2316
	bl	List_remove_node
	ldrh	r3, [r6, r7]
	cmp	r3, #0
	bne	.L704
	ldr	r0, .L706+12
	movw	r2, #1686
	ldr	r1, .L706+16
	bl	printk
.L704:
	ldrh	r3, [r6, r7]
	mov	r0, r5
	sub	r3, r3, #1
	strh	r3, [r6, r7]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r6, #2296]
	ldrh	r2, [r3, r4]
	add	r2, r2, #1
	strh	r2, [r3, r4]	@ movhi
	b	.L701
.L703:
	mul	r3, ip, r3
	ldrh	r3, [r2, r3]
	cmp	r3, r0
	beq	.L701
	add	r1, r1, #1
	uxth	r1, r1
.L702:
	cmp	r1, lr
	bne	.L705
.L701:
	mov	r0, r8
	add	r1, sp, #4
	mov	r2, #1
	bl	log2phys
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L707:
	.align	2
.L706:
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LANCHOR0+2312
	.word	.LC1
	.word	.LANCHOR1+305
	.fnend
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	2
	.global	ftl_check_vpc
	.type	ftl_check_vpc, %function
ftl_check_vpc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r4, #0
	ldr	r1, .L721
	ldr	r0, .L721+4
	bl	printk
	ldr	r0, .L721+8
	mov	r1, #0
	mov	r2, #8192
	bl	ftl_memset
	ldr	r6, .L721+12
	ldr	r5, .L721+16
	b	.L709
.L711:
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L710
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r0, r5, r0, asl #1
	add	r0, r0, #3904
	ldrh	r3, [r0, #12]
	add	r3, r3, #1
	strh	r3, [r0, #12]	@ movhi
.L710:
	add	r4, r4, #1
.L709:
	ldr	r3, [r6, #2480]
	cmp	r4, r3
	bcc	.L711
	mov	r4, #0
	ldr	r7, .L721+12
	mov	r5, r4
	ldr	sl, .L721+16
	movw	r9, #65535
	b	.L712
.L714:
	mov	r8, r5, asl #1
	ldr	r3, [r7, #2296]
	add	r6, sl, r8
	add	r6, r6, #3904
	ldrh	r2, [r3, r8]
	ldrh	r3, [r6, #12]
	add	r6, r6, #12
	cmp	r2, r3
	beq	.L713
	ldr	r0, .L721+20
	mov	r1, r5
	bl	printk
	ldr	r3, [r7, #2296]
	ldrh	r3, [r3, r8]
	cmp	r3, r9
	beq	.L713
	ldrh	r2, [r6, #0]
	cmp	r2, r3
	movhi	r4, #1
.L713:
	add	r5, r5, #1
	uxth	r5, r5
.L712:
	ldrh	r2, [r7, #12]
	ldr	r3, .L721+12
	cmp	r2, r5
	bhi	.L714
	ldr	r2, [r3, #2312]
	cmp	r2, #0
	beq	.L715
	ldr	r5, [r3, #2288]
	movw	r1, #2316
	ldrh	r8, [r3, r1]
	mov	r6, #0
	rsb	r2, r5, r2
	ldr	r5, .L721+24
	mov	r7, r3
	ldr	sl, .L721+16
	mov	r2, r2, asr #1
	movw	r9, #65535
	mul	r5, r5, r2
	uxth	r5, r5
	b	.L716
.L718:
	ldr	r2, [r7, #2296]
	mov	r3, r5, asl #1
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	beq	.L717
	add	r3, sl, r3
	ldr	r0, .L721+28
	add	r3, r3, #3904
	mov	r1, r5
	add	r3, r3, #12
	mov	r4, #1
	ldrh	r3, [r3, #0]
	bl	printk
.L717:
	mov	r2, #6
	ldr	r3, [r7, #2288]
	mul	r5, r2, r5
	ldrh	r5, [r3, r5]
	cmp	r5, r9
	beq	.L715
	add	r6, r6, #1
	uxth	r6, r6
.L716:
	cmp	r6, r8
	bne	.L718
.L715:
	cmp	r4, #0
	beq	.L708
	ldr	r0, .L721+32
	movw	r2, #2318
	ldr	r1, .L721
	bl	printk
.L708:
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, r9, sl, pc}
.L722:
	.align	2
.L721:
	.word	.LANCHOR1+321
	.word	.LC89
	.word	.LANCHOR3+3916
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC90
	.word	-1431655765
	.word	.LC91
	.word	.LC1
	.fnend
	.size	ftl_check_vpc, .-ftl_check_vpc
	.align	2
	.global	FtlFreeSysBlkQueueInit
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L724
	mov	r2, #2048
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, #0
	strh	r0, [r3, #184]	@ movhi
	mov	r1, r4
	strh	r4, [r3, #186]	@ movhi
	add	r0, r3, #192
	strh	r4, [r3, #188]	@ movhi
	strh	r4, [r3, #190]	@ movhi
	bl	ftl_memset
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L725:
	.align	2
.L724:
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
	ldr	r0, .L727
	mvn	r3, #0
	mov	r1, #255
	mov	r2, #16
	strh	r3, [r0, #124]	@ movhi
	mov	r3, #0
	strh	r3, [r0, #130]	@ movhi
	add	r0, r0, #136
	b	ftl_memset
.L728:
	.align	2
.L727:
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
	ldr	r6, .L748
	ldr	r8, .L748+4
	ldr	r3, [r6, #-848]
	mov	r7, r6
	ldr	r4, [r6, #-844]
	str	r3, [r6, #-860]
	str	r4, [r6, #-856]
	bl	FtlBbtMemInit
	ldrh	r5, [r8, #70]
	sub	r5, r5, #1
	uxth	r5, r5
	b	.L730
.L734:
	mov	r1, #1
	mov	r3, r5, asl #10
	ldr	r0, .L748+8
	mov	r2, r1
	str	r3, [r6, #-864]
	bl	FlashReadPages
	ldr	r3, [r6, #-868]
	cmn	r3, #1
	bne	.L731
	ldr	r3, [r6, #-864]
	mov	r1, #1
	ldr	r0, .L748+8
	mov	r2, r1
	add	r3, r3, #1
	str	r3, [r6, #-864]
	bl	FlashReadPages
.L731:
	ldr	r3, [r7, #-868]
	cmn	r3, #1
	beq	.L732
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L732
	ldr	r3, .L748+4
	ldr	r2, [r4, #4]
	strh	r5, [r3, #124]	@ movhi
	str	r2, [r3, #132]
	ldrh	r2, [r4, #8]
	strh	r2, [r3, #128]	@ movhi
	b	.L733
.L732:
	sub	r5, r5, #1
	uxth	r5, r5
.L730:
	ldrh	r3, [r8, #70]
	sub	r3, r3, #16
	cmp	r5, r3
	bgt	.L734
.L733:
	ldr	r5, .L748+4
	movw	r3, #65535
	ldrh	r2, [r5, #124]
	cmp	r2, r3
	beq	.L745
	ldrh	r2, [r5, #128]
	cmp	r2, r3
	beq	.L736
	ldr	r6, .L748
	mov	r1, #1
	mov	r2, r2, asl #10
	sub	r0, r6, #868
	str	r2, [r6, #-864]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r3, [r6, #-868]
	cmn	r3, #1
	beq	.L736
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L736
	ldr	r3, [r4, #4]
	ldr	r2, [r5, #132]
	cmp	r3, r2
	strhi	r3, [r5, #132]
	ldrhih	r2, [r5, #128]
	ldrhih	r3, [r4, #8]
	strhih	r2, [r5, #124]	@ movhi
	strhih	r3, [r5, #128]	@ movhi
.L736:
	ldr	r8, .L748+4
	mov	r1, #1
	mov	r6, #0
	ldr	r5, .L748
	ldrh	r0, [r8, #124]
	bl	FtlGetLastWrittenPage
	add	r3, r0, #1
	mov	r7, r0
	strh	r3, [r8, #126]	@ movhi
	b	.L737
.L740:
	ldrh	r2, [r8, #124]
	mov	r1, #1
	ldr	r0, .L748+8
	orr	r3, r3, r2, asl #10
	str	r3, [r5, #-864]
	ldr	r3, [r5, #-848]
	mov	r2, r1
	str	r3, [r5, #-860]
	bl	FlashReadPages
	ldr	r3, [r5, #-868]
	cmn	r3, #1
	beq	.L738
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	beq	.L739
.L738:
	sub	r6, r6, #1
	uxth	r6, r6
.L737:
	add	r3, r6, r7
	sxth	r3, r3
	cmp	r3, #0
	bge	.L740
	b	.L747
.L739:
	ldrh	r2, [r4, #10]
	ldr	r3, .L748+4
	ldrh	r0, [r4, #12]
	strh	r2, [r3, #130]	@ movhi
	movw	r2, #65535
	cmp	r0, r2
	beq	.L742
	ldr	r2, [r3, #0]
	cmp	r0, r2
	beq	.L742
	ldrh	r3, [r3, #14]
	mov	r3, r3, lsr #2
	cmp	r2, r3
	bcs	.L742
	cmp	r0, r3
	bcs	.L742
	bl	FtlSysBlkNumInit
.L742:
	ldr	r5, .L748+12
	mov	r4, #0
	ldr	r7, .L748+16
	movw	r6, #3932
	sub	r8, r5, #148
	b	.L743
.L744:
	ldr	r3, .L748
	ldrh	r2, [r7, r6]
	ldr	r0, [r5, #4]!
	ldr	r1, [r3, #-860]
	mov	r2, r2, asl #2
	mla	r1, r4, r2, r1
	bl	memcpy
	add	r4, r4, #1
.L743:
	ldrh	r3, [r8, #26]
	cmp	r4, r3
	bcc	.L744
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L745:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L747:
	ldr	r1, .L748+20
	mov	r2, #254
	ldr	r0, .L748+24
	bl	printk
	b	.L739
.L749:
	.align	2
.L748:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	.LANCHOR3-868
	.word	.LANCHOR0+148
	.word	.LANCHOR4
	.word	.LANCHOR1+335
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
	ldr	r2, .L755
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	movw	r3, #3932
	ldrh	r2, [r2, r3]
	mov	r6, r0
	mov	r5, r1
	mov	r0, r1
	mov	r1, #0
	ldr	r8, .L755+4
	mov	r2, r2, asl #2
	mov	r4, #0
	bl	ftl_memset
	movw	sl, #65535
	mov	r7, #1
.L753:
	ldrh	r3, [r6, r4]
	cmp	r3, sl
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
	ldrh	r2, [r8, #70]
	cmp	r2, r3
	bhi	.L752
	ldr	r0, .L755+8
	mov	r2, #74
	ldr	r1, .L755+12
	bl	printk
.L752:
	ldrh	r2, [r6, r4]
	add	r4, r4, #2
	cmp	r4, #1024
	mov	r3, r2, lsr #5
	and	r2, r2, #31
	ldr	r1, [r5, r3, asl #2]
	orr	r2, r1, r7, asl r2
	str	r2, [r5, r3, asl #2]
	bne	.L753
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L756:
	.align	2
.L755:
	.word	.LANCHOR4
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+346
	.fnend
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
	.align	2
	.global	FtlVariablesInit
	.type	FtlVariablesInit, %function
FtlVariablesInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movw	r2, #3940
	ldr	r6, .L758
	mvn	r3, #0
	ldr	r5, .L758+4
	mov	r4, #0
	mov	r1, r4
	strh	r3, [r6, r2]	@ movhi
	ldr	r2, .L758+8
	ldr	r0, [r5, #120]
	str	r3, [r6, #3948]
	str	r4, [r2, #3792]
	ldrh	r2, [r5, #92]
	str	r4, [r6, #3936]
	str	r4, [r6, #3944]
	mov	r2, r2, asl #1
	strh	r4, [r5, #116]	@ movhi
	bl	ftl_memset
	ldrh	r2, [r5, #14]
	mov	r1, r4
	ldr	r0, [r5, #2300]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r5, #14]
	mov	r1, r4
	ldr	r0, [r6, #3952]
	mov	r2, r2, asl #1
	bl	ftl_memset
	mov	r1, r4
	add	r0, r5, #2240
	mov	r2, #48
	bl	ftl_memset
	mov	r1, r4
	mov	r2, #512
	ldr	r0, .L758+12
	bl	ftl_memset
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L759:
	.align	2
.L758:
	.word	.LANCHOR4
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR0+2612
	.fnend
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	2
	.global	FtlMemInit
	.type	FtlMemInit, %function
FtlMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	movw	r2, #3178
	ldr	r4, .L790
	movw	r3, #65535
	ldr	r6, .L790+4
	mov	r8, #0
	ldr	r5, .L790+8
	mov	r7, #12
	ldrh	r0, [r4, #78]
	mov	sl, #20
	str	r3, [r6, #3960]
	mvn	r3, #0
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #2
	str	r8, [r5, #3772]
	mov	r0, r0, asl #1
	strh	r3, [r4, r2]	@ movhi
	movw	r3, #3124
	mov	r2, #32
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	mov	r2, #128
	str	r8, [r4, #2516]
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #6
	str	r8, [r4, #2520]
	strh	r8, [r4, r3]	@ movhi
	add	r3, r3, #50
	str	r8, [r4, #2504]
	strh	r8, [r4, r3]	@ movhi
	add	r3, r3, #36
	str	r8, [r4, #2492]
	strh	r8, [r4, r3]	@ movhi
	str	r8, [r4, #2488]
	str	r8, [r4, #2496]
	str	r8, [r4, #2500]
	str	r8, [r4, #2484]
	str	r8, [r4, #2524]
	str	r8, [r4, #2528]
	str	r8, [r4, #2536]
	str	r8, [r4, #2540]
	str	r8, [r4, #2544]
	str	r8, [r6, #3956]
	str	r8, [r4, #3136]
	str	r8, [r6, #3964]
	str	r8, [r4, #3128]
	bl	ftl_malloc
	str	r0, [r4, #3168]
	ldrh	r0, [r4, #78]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r4, #4]
	mul	sl, sl, r3
	mov	r9, sl, asl #2
	str	r0, [r4, #3172]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r5, #3788]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r6, #3968]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r6, #3972]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r5, #3768]
	mov	r0, sl
	bl	ftl_malloc
	ldrh	sl, [r4, #82]
	ldrh	r3, [r4, #4]
	mov	r3, r3, asl #1
	add	r3, r3, #1
	str	r3, [r4, #3160]
	str	r0, [r4, #3156]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r5, #-848]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r5, #3864]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r6, #3976]
	ldr	r0, [r4, #3160]
	mul	r0, r0, sl
	bl	ftl_malloc
	str	r0, [r4, #3148]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r5, #3784]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r5, #3776]
	ldr	r0, [r4, #3160]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r4, #84]
	ldrh	sl, [r4, #4]
	mul	sl, sl, r3
	str	r0, [r4, #3144]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r5, #-844]
	mov	r0, sl, asl #2
	bl	ftl_malloc
	ldrh	r3, [r4, #84]
	movw	sl, #3980
	str	r0, [r5, #3780]
	ldr	r0, [r4, #3160]
	mul	r0, r0, r3
	bl	ftl_malloc
	str	r0, [r4, #3152]
	ldrh	r0, [r4, #14]
	mov	r0, r0, asl #1
	uxth	r0, r0
	strh	r0, [r6, sl]	@ movhi
	bl	ftl_malloc
	str	r0, [r6, #3952]
	ldrh	r0, [r6, sl]
	add	r0, r0, #544
	add	r0, r0, #3
	mov	r0, r0, lsr #9
	strh	r0, [r6, sl]	@ movhi
	mov	r0, r0, asl #9
	bl	ftl_malloc
	ldrh	sl, [r4, #14]
	mov	sl, sl, asl #1
	str	r0, [r6, #3984]
	add	r0, r0, #32
	str	r0, [r4, #2300]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r6, #3988]
	mov	r0, sl
	bl	ftl_malloc
	ldr	sl, [r4, #100]
	mov	sl, sl, asl #1
	str	r0, [r4, #2296]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r5, #3848]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r5, #3796]
	ldrh	r0, [r4, #14]
	mov	r0, r0, lsr #3
	add	r0, r0, #4
	bl	ftl_malloc
	str	r0, [r4, #2476]
	ldrh	r0, [r4, #92]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #120]
	ldrh	r0, [r4, #92]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r5, #3908]
	ldrh	r0, [r4, #92]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r5, #3860]
	ldrh	r0, [r4, #94]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r2, [r4, #94]
	mov	r1, r8
	mov	r2, r2, asl #2
	str	r0, [r5, #3912]
	bl	ftl_memset
	ldrh	r0, [r4, #108]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r5, #3856]
	ldr	r0, [r4, #100]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r5, #3852]
	ldrh	r0, [r4, #110]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r4, #110]
	str	r0, [r4, #2464]
	ldrh	r0, [r4, #82]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r4, #14]
	str	r0, [r5, #3800]
	mov	r0, #6
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r4, #70]
	ldrh	r2, [r4, #26]
	movw	r5, #3932
	add	r3, r3, #31
	mov	r3, r3, lsr #5
	strh	r3, [r6, r5]	@ movhi
	mul	r3, r2, r3
	str	r0, [r4, #2288]
	mov	r0, r3, asl #2
	bl	ftl_malloc
	ldrh	r1, [r6, r5]
	ldrh	ip, [r4, #26]
	mov	r3, #1
	mov	r1, r1, asl #2
	mov	r2, r1
	str	r0, [r4, #152]
	add	r0, r4, #152
	b	.L761
.L762:
	ldr	r5, [r4, #152]
	add	r3, r3, #1
	add	r5, r5, r2
	add	r2, r2, r1
	str	r5, [r0, #4]!
.L761:
	cmp	r3, ip
	bcc	.L762
	ldr	r0, .L790+12
	mov	r2, #0
	mov	r1, r2
	add	r0, r0, r3, asl #2
	b	.L763
.L764:
	add	ip, r0, r2
	add	r3, r3, #1
	add	r2, r2, #4
	str	r1, [ip, #28]
.L763:
	cmp	r3, #7
	bls	.L764
	ldr	r3, .L790+8
	ldr	r2, [r3, #3848]
	cmp	r2, #0
	beq	.L789
.L765:
	ldr	r2, [r3, #3796]
	cmp	r2, #0
	beq	.L789
.L767:
	ldr	r2, [r3, #3856]
	cmp	r2, #0
	beq	.L789
.L768:
	ldr	r2, [r3, #3852]
	cmp	r2, #0
	beq	.L789
.L769:
	ldr	r2, .L790
	ldr	r1, [r2, #2464]
	cmp	r1, #0
	beq	.L789
.L770:
	ldr	r3, [r3, #3800]
	cmp	r3, #0
	beq	.L789
.L771:
	ldr	r3, [r2, #2288]
	cmp	r3, #0
	beq	.L789
.L772:
	ldr	r3, [r2, #152]
	cmp	r3, #0
	beq	.L789
.L773:
	ldr	r3, [r2, #2296]
	cmp	r3, #0
	beq	.L789
.L774:
	ldr	r1, .L790
	ldr	r3, [r1, #3168]
	cmp	r3, #0
	beq	.L789
	ldr	r3, [r1, #3172]
	cmp	r3, #0
	beq	.L789
.L776:
	ldr	r3, .L790+8
	ldr	r2, [r3, #3788]
	cmp	r2, #0
	beq	.L789
	ldr	r2, .L790+4
	ldr	r0, [r2, #3972]
	cmp	r0, #0
	beq	.L789
	ldr	r0, [r3, #3768]
	cmp	r0, #0
	beq	.L789
	ldr	r1, [r1, #3156]
	cmp	r1, #0
	beq	.L789
	ldr	r1, [r2, #3968]
	cmp	r1, #0
	beq	.L789
.L778:
	ldr	r1, [r3, #-848]
	cmp	r1, #0
	beq	.L789
	ldr	r3, [r3, #3864]
	cmp	r3, #0
	beq	.L789
	ldr	r3, [r2, #3976]
	cmp	r3, #0
	beq	.L789
	ldr	r3, .L790
	ldr	r2, [r3, #3148]
	cmp	r2, #0
	beq	.L789
	ldr	r2, .L790+8
	ldr	r1, [r2, #3784]
	cmp	r1, #0
	beq	.L789
	ldr	r1, [r2, #3776]
	cmp	r1, #0
	beq	.L789
	ldr	r1, [r3, #3144]
	cmp	r1, #0
	beq	.L789
.L780:
	ldr	r1, [r2, #-844]
	cmp	r1, #0
	beq	.L789
	ldr	r2, [r2, #3780]
	cmp	r2, #0
	beq	.L789
	ldr	r2, [r3, #3152]
	cmp	r2, #0
	beq	.L789
.L782:
	ldr	r2, [r3, #2300]
	cmp	r2, #0
	beq	.L789
	ldr	r2, .L790+4
	ldr	r2, [r2, #3952]
	cmp	r2, #0
	beq	.L789
.L784:
	ldr	r3, [r3, #120]
	cmp	r3, #0
	beq	.L789
.L785:
	ldr	r3, .L790+8
	ldr	r2, [r3, #3908]
	cmp	r2, #0
	beq	.L789
.L786:
	ldr	r2, [r3, #3860]
	cmp	r2, #0
	beq	.L789
.L787:
	ldr	r3, [r3, #3912]
	cmp	r3, #0
	bne	.L788
.L789:
	ldr	r0, .L790+16
	ldr	r1, .L790+20
	bl	printk
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L788:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L791:
	.align	2
.L790:
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR3
	.word	.LANCHOR0+124
	.word	.LC92
	.word	.LANCHOR1+360
	.fnend
	.size	FtlMemInit, .-FtlMemInit
	.align	2
	.global	FtlLoadSysInfo
	.type	FtlLoadSysInfo, %function
FtlLoadSysInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r1, #0
	ldr	r4, .L812
	ldr	r5, .L812+4
	ldr	r3, [r4, #-848]
	ldrh	r2, [r5, #12]
	ldr	r0, [r5, #2296]
	str	r3, [r4, #-860]
	ldr	r3, [r4, #-844]
	mov	r2, r2, asl #1
	str	r3, [r4, #-856]
	bl	ftl_memset
	movw	r3, #2552
	ldrh	r0, [r5, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L808
	mov	r1, #1
	mov	r6, #0
	bl	FtlGetLastWrittenPage
	movw	r3, #2554
	add	r2, r0, #1
	mov	r7, r0
	strh	r2, [r5, r3]	@ movhi
	b	.L794
.L797:
	movw	r2, #2552
	mov	r1, #1
	ldrh	r2, [r5, r2]
	ldr	r0, .L812+8
	orr	r3, r3, r2, asl #10
	str	r3, [r4, #-864]
	ldr	r3, [r4, #-848]
	mov	r2, r1
	str	r3, [r4, #-860]
	bl	FlashReadPages
	ldr	r3, [r4, #-868]
	cmn	r3, #1
	beq	.L795
	ldr	r3, [r4, #-848]
	ldr	r2, [r3, #0]
	ldr	r3, .L812+12
	cmp	r2, r3
	bne	.L795
	ldr	r3, [r4, #-844]
	ldrh	r2, [r3, #0]
	movw	r3, #61604
	cmp	r2, r3
	beq	.L796
.L795:
	sub	r6, r6, #1
	uxth	r6, r6
.L794:
	add	r3, r6, r7
	sxth	r3, r3
	cmp	r3, #0
	bge	.L797
	b	.L811
.L796:
	ldr	r5, .L812+4
	ldrh	r2, [r5, #12]
	ldrh	r3, [r5, #82]
	add	r2, r2, #24
	cmp	r3, r2, asl #1
	bcs	.L799
	ldr	r0, .L812+16
	movw	r2, #1437
	ldr	r1, .L812+20
	bl	printk
.L799:
	ldr	r6, .L812
	mov	r2, #48
	ldr	r4, .L812+4
	ldr	r7, [r6, #-860]
	add	r0, r4, #2240
	mov	r1, r7
	bl	memcpy
	ldrh	r2, [r5, #12]
	add	r1, r7, #48
	ldr	r0, [r5, #2296]
	mov	r2, r2, asl #1
	bl	memcpy
	ldrh	r1, [r5, #12]
	ldr	r3, [r6, #-860]
	ldr	r0, [r5, #2476]
	mov	r2, r1, lsr #3
	add	r1, r1, #24
	add	r2, r2, #4
	mov	r1, r1, lsr #1
	add	r1, r3, r1, asl #2
	bl	memcpy
	ldr	r2, [r5, #2240]
	ldr	r3, .L812+12
	cmp	r2, r3
	bne	.L808
	movw	r3, #2248
	ldrb	r2, [r4, #2250]	@ zero_extendqisi2
	ldrh	r5, [r4, r3]
	movw	r3, #2558
	strh	r5, [r4, r3]	@ movhi
	ldrh	r3, [r4, #26]
	cmp	r2, r3
	bne	.L808
	ldr	r3, .L812+24
	ldrh	r2, [r4, #30]
	ldr	r6, [r4, #16]
	str	r5, [r3, #3992]
	ldrh	r3, [r4, #74]
	ldrh	r0, [r4, #130]
	ldrh	r1, [r4, #4]
	mul	r3, r5, r3
	rsb	r0, r0, r6
	rsb	r0, r5, r0
	str	r3, [r4, #2480]
	mul	r3, r2, r3
	str	r3, [r4, #112]
	bl	__aeabi_uidiv
	cmp	r5, r6
	movw	r3, #2548
	strh	r0, [r4, r3]	@ movhi
	bls	.L800
	ldr	r0, .L812+16
	movw	r2, #1459
	ldr	r1, .L812+20
	bl	printk
.L800:
	ldr	r3, .L812+4
	movw	r2, #2254
	movw	r0, #2322
	movw	r5, #65535
	ldrh	r1, [r3, r2]
	mov	r2, #2320
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #2256
	ldrh	r2, [r3, r2]
	mov	ip, r2, lsr #6
	and	r2, r2, #63
	strb	r2, [r3, #2326]
	ldrb	r2, [r3, #2251]	@ zero_extendqisi2
	strh	ip, [r3, r0]	@ movhi
	mvn	r0, #0
	movw	ip, #2566
	strb	r2, [r3, #2328]
	movw	r2, #2564
	strh	r0, [r3, r2]	@ movhi
	movw	r0, #2258
	mov	r2, #0
	strh	r2, [r3, ip]	@ movhi
	ldrh	ip, [r3, r0]
	mov	r0, #2368
	strb	r2, [r3, #2570]
	strb	r2, [r3, #2572]
	strh	ip, [r3, r0]	@ movhi
	movw	r0, #2260
	ldrh	r0, [r3, r0]
	movw	ip, #2370
	str	r2, [r3, #2504]
	str	r2, [r3, #2492]
	mov	r4, r0, lsr #6
	and	r0, r0, #63
	strb	r0, [r3, #2374]
	ldrb	r0, [r3, #2252]	@ zero_extendqisi2
	strh	r4, [r3, ip]	@ movhi
	str	r2, [r3, #2484]
	strb	r0, [r3, #2376]
	movw	r0, #2262
	ldrh	ip, [r3, r0]
	mov	r0, #2416
	str	r2, [r3, #2500]
	str	r2, [r3, #2528]
	strh	ip, [r3, r0]	@ movhi
	movw	r0, #2264
	ldrh	r0, [r3, r0]
	movw	ip, #2418
	str	r2, [r3, #2540]
	mov	r4, r0, lsr #6
	and	r0, r0, #63
	strb	r0, [r3, #2422]
	ldrb	r0, [r3, #2253]	@ zero_extendqisi2
	strh	r4, [r3, ip]	@ movhi
	strb	r0, [r3, #2424]
	ldr	r0, [r3, #2272]
	str	r0, [r3, #2524]
	str	r2, [r3, #2496]
	ldr	r2, [r3, #2280]
	ldr	r0, [r3, #2516]
	cmp	r2, r0
	strhi	r2, [r3, #2516]
	ldr	r2, [r3, #2276]
	ldr	r3, [r3, #2520]
	cmp	r2, r3
	ldrhi	r3, .L812+4
	strhi	r2, [r3, #2520]
	cmp	r1, r5
	beq	.L803
	ldr	r0, .L812+28
	bl	make_superblock
.L803:
	ldr	r4, .L812+4
	mov	r3, #2368
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L804
	add	r0, r4, #2368
	bl	make_superblock
.L804:
	mov	r3, #2416
	ldrh	r3, [r4, r3]
	movw	r4, #65535
	cmp	r3, r4
	beq	.L805
	ldr	r0, .L812+32
	bl	make_superblock
.L805:
	ldr	r2, .L812+4
	movw	r3, #2564
	ldrh	r3, [r2, r3]
	cmp	r3, r4
	beq	.L809
	ldr	r0, .L812+36
	bl	make_superblock
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L808:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L809:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L811:
	ldr	r1, .L812+20
	movw	r2, #1435
	ldr	r0, .L812+16
	bl	printk
	b	.L796
.L813:
	.align	2
.L812:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	.LANCHOR3-868
	.word	1179929683
	.word	.LC1
	.word	.LANCHOR1+371
	.word	.LANCHOR4
	.word	.LANCHOR0+2320
	.word	.LANCHOR0+2416
	.word	.LANCHOR0+2564
	.fnend
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r7, #0
	ldr	r6, .L826
	mov	r1, #255
	ldr	r4, .L826+4
	movw	r9, #65535
	ldr	r3, [r6, #-848]
	ldr	r5, [r6, #-844]
	mov	fp, r4
	str	r3, [r6, #-860]
	movw	r3, #2552
	ldrh	r3, [r4, r3]
	str	r5, [r6, #-856]
	str	r7, [r5, #12]
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, .L826+8
	strh	r3, [r5, #0]	@ movhi
	ldr	r3, [r4, #2560]
	str	r7, [r5, #8]
	str	r3, [r5, #4]
	ldr	r3, .L826+12
	str	r3, [r4, #2240]
	ldr	r3, .L826+16
	str	r3, [r4, #2244]
	movw	r3, #2558
	ldrh	r2, [r4, r3]
	movw	r3, #2248
	strh	r2, [r4, r3]	@ movhi
	ldrh	r3, [r4, #26]
	strb	r3, [r4, #2250]
	mov	r3, #2320
	ldrh	r2, [r4, r3]
	movw	r3, #2254
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #68
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #2326]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	mov	r3, #2256
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #2328]	@ zero_extendqisi2
	strb	r3, [r4, #2251]
	mov	r3, #2368
	ldrh	r2, [r4, r3]
	movw	r3, #2258
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #112
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #2374]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	movw	r3, #2260
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #2376]	@ zero_extendqisi2
	strb	r3, [r4, #2252]
	mov	r3, #2416
	ldrh	r2, [r4, r3]
	movw	r3, #2262
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #156
	ldr	r0, [r6, #-860]
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #2422]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	movw	r3, #2264
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #2424]	@ zero_extendqisi2
	ldrh	r2, [r4, #82]
	strb	r3, [r4, #2253]
	ldr	r3, [r4, #2524]
	str	r3, [r4, #2272]
	ldr	r3, [r4, #2516]
	str	r3, [r4, #2280]
	ldr	r3, [r4, #2520]
	str	r3, [r4, #2276]
	bl	ftl_memset
	add	r1, r4, #2240
	mov	r2, #48
	ldr	r0, [r6, #-860]
	bl	memcpy
	ldrh	r2, [r4, #12]
	ldr	r0, [r6, #-860]
	ldr	r1, [r4, #2296]
	mov	r2, r2, asl #1
	add	r0, r0, #48
	bl	memcpy
	ldrh	r2, [r4, #12]
	ldr	r3, [r6, #-860]
	add	r0, r2, #24
	ldr	r1, [r4, #2476]
	mov	r2, r2, lsr #3
	mov	r0, r0, lsr #1
	add	r2, r2, #4
	add	r0, r3, r0, asl #2
	bl	memcpy
	mov	r0, r7
	bl	FtlUpdateVaildLpn
.L825:
	ldr	r3, [r6, #-848]
	movw	sl, #2552
	ldrh	r2, [r4, sl]
	movw	r8, #2554
	mov	r1, #1
	ldr	r0, .L826+20
	str	r3, [r6, #-860]
	ldr	r3, [r6, #-844]
	str	r3, [r6, #-856]
	ldrh	r3, [r4, r8]
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r6, #-864]
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #76]
	ldrh	r2, [r4, r8]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L816
	ldrh	r2, [r4, sl]
	movw	r3, #2556
	ldrh	r9, [r4, r3]
	strh	r2, [r4, r3]	@ movhi
	mov	r3, #0
	strh	r3, [r4, r8]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, [r4, #2516]
	mov	r1, #1
	add	r2, r3, #1
	str	r2, [r4, #2516]
	str	r3, [r4, #2560]
	mov	r2, r0, asl #10
	strh	r0, [r4, sl]	@ movhi
	str	r2, [r6, #-864]
	mov	r2, r1
	str	r3, [r5, #4]
	mov	r3, r1
	strh	r0, [r5, #2]	@ movhi
	ldr	r0, .L826+20
	bl	FlashProgPages
.L816:
	movw	r8, #2554
	ldr	r2, [r6, #-868]
	ldrh	r3, [r4, r8]
	cmn	r2, #1
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r4, r8]	@ movhi
	bne	.L817
	cmp	r3, #1
	bne	.L818
	ldr	r0, .L826+24
	movw	r2, #1122
	ldr	r1, .L826+28
	bl	printk
.L818:
	ldrh	r3, [fp, r8]
	add	r7, r7, #1
	cmp	r3, #1
	uxth	r7, r7
	ldreqh	r2, [r4, #76]
	movweq	r3, #2554
	subeq	r2, r2, #1
	streqh	r2, [r4, r3]	@ movhi
	cmp	r7, #3
	bls	.L825
	ldr	r3, .L826
	mov	r2, r7
	ldr	r0, .L826+32
	ldr	r1, [r3, #-864]
	bl	printk
.L821:
	b	.L821
.L817:
	cmp	r3, #1
	beq	.L825
	movw	r3, #65535
	cmp	r9, r3
	beq	.L822
	mov	r0, r9
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L822:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L827:
	.align	2
.L826:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	-3932
	.word	1179929683
	.word	1342177344
	.word	.LANCHOR3-868
	.word	.LC1
	.word	.LANCHOR1+386
	.word	.LC93
	.fnend
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	2
	.global	FtlVendorPartRead
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L838
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, r2
	ldrh	sl, [r3, #80]
	add	r2, r1, r0
	ldrh	r3, [r3, #68]
	.pad #100
	sub	sp, sp, #100
	mov	r6, r0
	mov	r5, r1
	cmp	r2, r3
	mvnhi	r7, #0
	bhi	.L829
	mov	sl, r0, lsr sl
	mov	r7, #0
	ldr	r9, .L838+4
	mov	sl, sl, asl #2
	b	.L830
.L835:
	ldr	r2, .L838+8
	mov	r0, r6
	ldr	r3, [r2, #3912]
	ldr	fp, [r3, sl]
	ldr	r3, .L838
	ldrh	r4, [r3, #30]
	mov	r1, r4
	bl	__aeabi_uidivmod
	uxth	r3, r1
	rsb	r4, r3, r4
	uxth	r4, r4
	cmp	r4, r5
	uxthhi	r4, r5
	cmp	fp, #0
	beq	.L832
	ldr	r2, [r9, #3976]
	mov	r1, #1
	add	r0, sp, #12
	str	r3, [sp, #4]
	str	fp, [sp, #16]
	str	r2, [sp, #20]
	add	r2, sp, #32
	str	r2, [sp, #24]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [sp, #12]
	ldr	r3, [sp, #4]
	mov	r0, r8
	cmn	r2, #1
	moveq	r7, r2
	ldr	r2, .L838+4
	ldr	r1, [r2, #3976]
	mov	r2, r4, asl #9
	add	r1, r1, r3, asl #9
	bl	memcpy
	b	.L834
.L832:
	mov	r0, r8
	mov	r1, fp
	mov	r2, r4, asl #9
	bl	ftl_memset
.L834:
	rsb	r5, r4, r5
	add	r6, r6, r4
	add	r8, r8, r4, asl #9
	add	sl, sl, #4
.L830:
	cmp	r5, #0
	bne	.L835
.L829:
	mov	r0, r7
	add	sp, sp, #100
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L839:
	.align	2
.L838:
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR3
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
	ldr	r5, .L843
	mov	r1, #1
	ldr	r2, .L843+4
	bl	FtlVendorPartRead
	ldr	r4, .L843+8
	ldr	r3, [r5, #2612]
	cmp	r3, r4
	beq	.L841
	ldr	r0, .L843+4
	mov	r1, #0
	mov	r2, #512
	bl	ftl_memset
	str	r4, [r5, #2612]
.L841:
	ldr	r2, [r5, #2612]
	ldr	r3, .L843
	cmp	r2, r4
	bne	.L842
	ldr	r2, [r3, #2700]
	str	r2, [r3, #2508]
	ldr	r2, [r3, #2704]
	str	r2, [r3, #2512]
	ldr	r2, [r3, #2620]
	str	r2, [r3, #2504]
	ldr	r2, [r3, #2624]
	str	r2, [r3, #2492]
	ldr	r2, [r3, #2628]
	str	r2, [r3, #2484]
	ldr	r2, [r3, #2632]
	str	r2, [r3, #2500]
	ldr	r2, [r3, #2640]
	str	r2, [r3, #2528]
	ldr	r2, [r3, #2644]
	str	r2, [r3, #2536]
	ldr	r2, [r3, #2648]
	str	r2, [r3, #2488]
	ldr	r2, [r3, #2652]
	str	r2, [r3, #2496]
	ldr	r2, [r3, #2656]
	str	r2, [r3, #2540]
	ldr	r2, [r3, #2660]
	str	r2, [r3, #2544]
.L842:
	ldr	r4, .L843
	ldrh	r2, [r4, #64]
	ldr	r3, [r4, #2528]
	ldr	r0, [r4, #2524]
	ldrh	r1, [r4, #12]
	mla	r0, r0, r2, r3
	bl	__aeabi_uidiv
	str	r0, [r4, #2532]
	ldmfd	sp!, {r3, r4, r5, pc}
.L844:
	.align	2
.L843:
	.word	.LANCHOR0
	.word	.LANCHOR0+2612
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
	movw	r5, #3980
	ldr	r4, .L847
	mov	r0, #64
	ldr	r2, [r4, #3984]
	ldrh	r1, [r4, r5]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #3984]
	ldr	r2, [r3, #0]
	ldr	r3, .L847+4
	cmp	r2, r3
	beq	.L846
	ldr	r1, .L847+8
	ldr	r0, .L847+12
	bl	printk
	ldrh	r2, [r4, r5]
	ldr	r0, [r4, #3984]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L846:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L848:
	.align	2
.L847:
	.word	.LANCHOR4
	.word	1112818501
	.word	.LC94
	.word	.LC70
	.fnend
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	2
	.global	FtlVendorPartWrite
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L859
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r9, r2
	ldrh	r2, [r3, #68]
	mov	r4, r1
	add	r1, r1, r0
	.pad #100
	sub	sp, sp, #100
	cmp	r1, r2
	mov	r6, r0
	ldrh	r8, [r3, #80]
	mvnhi	r7, #0
	bhi	.L850
	mov	r8, r0, lsr r8
	mov	r7, #0
	mov	sl, r7
	mov	fp, r8, asl #2
	b	.L851
.L856:
	ldr	r2, .L859+4
	mov	r0, r6
	ldr	r3, [r2, #3912]
	ldr	r2, .L859
	ldr	r3, [r3, fp]
	ldrh	r7, [r2, #30]
	str	r3, [sp, #0]
	mov	r1, r7
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #0]
	ldr	r2, .L859+8
	uxth	r1, r1
	str	r1, [sp, #4]
	rsb	r5, r1, r7
	uxth	r5, r5
	cmp	r5, r4
	uxthhi	r5, r4
	cmp	r3, #0
	beq	.L853
	cmp	r5, r7
	beq	.L853
	str	r3, [sp, #16]
	mov	r1, #1
	ldr	r3, [r2, #3976]
	add	r0, sp, #12
	mov	r2, r1
	str	r3, [sp, #20]
	add	r3, sp, #32
	str	r3, [sp, #24]
	bl	FlashReadPages
	b	.L854
.L853:
	ldr	r3, .L859
	mov	r1, #0
	ldr	r0, [r2, #3976]
	ldrh	r2, [r3, #82]
	bl	ftl_memset
.L854:
	ldr	r7, .L859+8
	mov	r3, r5, asl #9
	ldr	r2, [sp, #4]
	mov	r1, r9
	rsb	r4, r5, r4
	add	r6, r6, r5
	ldr	r0, [r7, #3976]
	add	fp, fp, #4
	str	r3, [sp, #0]
	add	r0, r0, r2, asl #9
	mov	r2, r3
	bl	memcpy
	mov	r1, r8
	ldr	r0, .L859+12
	add	r8, r8, #1
	ldr	r2, [r7, #3976]
	bl	FtlMapWritePage
	ldr	r3, [sp, #0]
	add	r9, r9, r3
	cmn	r0, #1
	moveq	sl, r0
.L851:
	cmp	r4, #0
	bne	.L856
	mov	r7, sl
.L850:
	mov	r0, r7
	add	sp, sp, #100
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L860:
	.align	2
.L859:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR4
	.word	.LANCHOR3+3868
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
	ldr	r3, .L863
	ldr	r2, .L863+4
	ldr	r1, [r3, #2612]
	cmp	r1, r2
	bxne	lr
	ldr	r2, .L863+8
	mov	r0, #0
	mov	r1, #1
	str	r2, [r3, #2616]
	ldr	r2, [r3, #2508]
	str	r2, [r3, #2700]
	ldr	r2, [r3, #2512]
	str	r2, [r3, #2704]
	ldr	r2, [r3, #2504]
	str	r2, [r3, #2620]
	ldr	r2, [r3, #2492]
	str	r2, [r3, #2624]
	ldr	r2, [r3, #2484]
	str	r2, [r3, #2628]
	ldr	r2, [r3, #2500]
	str	r2, [r3, #2632]
	ldr	r2, [r3, #2528]
	str	r2, [r3, #2640]
	ldr	r2, [r3, #2536]
	str	r2, [r3, #2644]
	ldr	r2, [r3, #2488]
	str	r2, [r3, #2648]
	ldr	r2, [r3, #2496]
	str	r2, [r3, #2652]
	ldr	r2, [r3, #2540]
	str	r2, [r3, #2656]
	ldr	r2, [r3, #2544]
	str	r2, [r3, #2660]
	ldr	r2, .L863+12
	b	FtlVendorPartWrite
.L864:
	.align	2
.L863:
	.word	.LANCHOR0
	.word	1179929683
	.word	1342177344
	.word	.LANCHOR0+2612
	.fnend
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.align	2
	.global	FtlEctTblFlush
	.type	FtlEctTblFlush, %function
FtlEctTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	movw	r2, #3996
	ldr	r3, .L870
	ldrh	r1, [r3, r2]
	cmp	r1, #31
	addls	r1, r1, #1
	strlsh	r1, [r3, r2]	@ movhi
	movhi	r1, #32
	movls	r1, #1
	cmp	r0, #0
	bne	.L867
	ldr	r2, [r3, #3984]
	ldr	r0, [r2, #20]
	ldr	r2, [r2, #16]
	add	r1, r1, r0
	cmp	r2, r1
	bcc	.L868
.L867:
	ldr	r2, [r3, #3984]
	mov	r0, #64
	ldr	r1, [r2, #16]
	str	r1, [r2, #20]
	ldr	r1, .L870+4
	str	r1, [r2, #0]
	movw	r1, #3980
	ldr	r2, [r3, #3984]
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
.L868:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L871:
	.align	2
.L870:
	.word	.LANCHOR4
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
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r1, #0
	ldr	r8, .L884
	mov	r6, #0
	ldr	sl, .L884+4
	ldr	r9, .L884+8
	ldr	r0, [r8, #-848]
	add	r7, sl, #148
	ldr	r3, [r8, #-844]
	ldrh	r2, [sl, #82]
	str	r0, [r8, #-860]
	str	r3, [r8, #-856]
	bl	ftl_memset
	b	.L873
.L874:
	movw	r3, #3932
	ldr	r1, [r7, #4]!
	ldrh	r2, [r9, r3]
	ldr	r3, [r8, #-860]
	mul	r0, r2, r6
	mov	r2, r2, asl #2
	add	r6, r6, #1
	add	r0, r3, r0, asl #2
	bl	memcpy
.L873:
	ldrh	r3, [sl, #26]
	ldr	r4, .L884+4
	cmp	r6, r3
	ldr	r5, .L884
	blt	.L874
	ldr	r6, [r5, #-856]
	mov	r1, #255
	mov	r2, #16
	mov	r7, #0
	mov	r8, r7
	mov	sl, r7
	mov	r0, r6
	mov	r9, r7
	bl	ftl_memset
	ldr	r3, .L884+12
	strh	r3, [r6, #0]	@ movhi
	ldr	r3, [r4, #132]
	str	r3, [r6, #4]
	ldrh	r3, [r4, #124]
	strh	r3, [r6, #2]	@ movhi
	ldrh	r3, [r4, #128]
	strh	r3, [r6, #8]	@ movhi
	ldrh	r3, [r4, #130]
	strh	r3, [r6, #10]	@ movhi
	ldr	r3, [r4, #0]
	strh	r3, [r6, #12]	@ movhi
	b	.L883
.L880:
	mov	r8, #1
.L883:
	ldr	r3, [r5, #-848]
	ldr	r4, .L884+4
	str	r9, [r5, #-868]
	str	r3, [r5, #-860]
	ldr	r3, [r5, #-844]
	ldrh	r1, [r4, #124]
	ldrh	r2, [r4, #126]
	str	r3, [r5, #-856]
	ldrh	r0, [r6, #10]
	orr	r3, r2, r1, asl #10
	str	r3, [r5, #-864]
	ldrh	r3, [r4, #128]
	str	r0, [sp, #0]
	ldr	r0, .L884+16
	bl	printk
	mov	r1, #1
	mov	r2, r1
	mov	r3, r1
	ldr	r0, .L884+20
	bl	FlashProgPages
	ldrh	r3, [r4, #76]
	ldrh	r2, [r4, #126]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L876
	ldr	r3, [r4, #132]
	mov	r1, #1
	ldrh	r2, [r4, #124]
	add	r3, r3, #1
	ldr	r0, [r5, #3768]
	str	r3, [r4, #132]
	str	r3, [r6, #4]
	ldrh	r3, [r4, #128]
	strh	r2, [r6, #8]	@ movhi
	strh	r2, [r4, #128]	@ movhi
	mov	r2, r1
	strh	r3, [r4, #124]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r5, #-864]
	str	r3, [r0, #4]
	strh	sl, [r4, #126]	@ movhi
	bl	FlashEraseBlocks
	mov	r1, #1
	ldr	r0, .L884+20
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
.L876:
	ldr	r3, .L884+4
	ldrh	r2, [r3, #126]
	add	r2, r2, #1
	strh	r2, [r3, #126]	@ movhi
	ldr	r3, [r5, #-868]
	cmn	r3, #1
	bne	.L877
	add	r7, r7, #1
	ldr	r0, .L884+24
	ldr	r1, [r5, #-864]
	uxth	r7, r7
	bl	printk
	cmp	r7, #3
	bls	.L883
	ldr	r3, .L884
	mov	r2, r7
	ldr	r0, .L884+28
	ldr	r1, [r3, #-864]
	bl	printk
.L879:
	b	.L879
.L877:
	cmp	r8, #0
	beq	.L880
	mov	r0, #0
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, r9, sl, pc}
.L885:
	.align	2
.L884:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	-3887
	.word	.LC95
	.word	.LANCHOR3-868
	.word	.LC96
	.word	.LC97
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
	movw	r3, #3182
	ldr	r4, .L896
	mov	r9, r0
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	movne	r6, #0
	movne	sl, r4
	bne	.L887
	b	.L888
.L894:
	add	r3, r4, r6
	mov	r1, r9
	mov	r5, #0
	movw	r7, #3182
	ldrb	r0, [r3, #32]	@ zero_extendqisi2
	bl	V2P_block
	mov	r8, r0
	b	.L889
.L893:
	add	r3, r4, r5, asl #1
	add	r3, r3, #3184
	ldrh	r3, [r3, #0]
	cmp	r3, r8
	bne	.L890
	mov	r1, r8
	ldr	r0, .L896+4
	bl	printk
	mov	r0, r8
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	r2, [r4, r7]
	mov	r3, r5
	b	.L891
.L892:
	add	r1, r3, #1
	add	r3, r4, r3, asl #1
	add	r3, r3, #3184
	add	r0, r4, r1, asl #1
	add	r0, r0, #3184
	ldrh	r0, [r0, #0]
	strh	r0, [r3, #0]	@ movhi
	uxth	r3, r1
.L891:
	cmp	r3, r2
	bcc	.L892
	sub	r2, r2, #1
	strh	r2, [sl, r7]	@ movhi
.L890:
	add	r5, r5, #1
	uxth	r5, r5
.L889:
	ldrh	r3, [r4, r7]
	cmp	r3, r5
	bhi	.L893
	add	r6, r6, #1
	uxth	r6, r6
.L887:
	ldrh	r3, [r4, #4]
	cmp	r3, r6
	bhi	.L894
	bl	FtlGcReFreshBadBlk
.L888:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L897:
	.align	2
.L896:
	.word	.LANCHOR0
	.word	.LC98
	.fnend
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	2
	.type	update_vpc_list.part.9, %function
update_vpc_list.part.9:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldr	r4, .L901
	mov	r1, r6
	ldr	r0, .L901+4
	movw	r5, #2308
	bl	List_remove_node
	ldrh	r3, [r4, r5]
	cmp	r3, #0
	bne	.L899
	ldr	r0, .L901+8
	movw	r2, #2741
	ldr	r1, .L901+12
	bl	printk
.L899:
	ldrh	r3, [r4, r5]
	mov	r0, r6
	sub	r3, r3, #1
	strh	r3, [r4, r5]	@ movhi
	bl	free_data_superblock
	mov	r0, r6
	bl	FtlGcFreeBadSuperBlk
	movw	r3, #2316
	ldrh	r2, [r4, r3]
	ldrh	r3, [r4, r5]
	add	r2, r2, r3
	ldrh	r3, [r4, #12]
	cmp	r2, r3
	ble	.L900
	ldr	r0, .L901+8
	movw	r2, #2744
	ldr	r1, .L901+12
	bl	printk
.L900:
	mov	r0, #1
	ldmfd	sp!, {r4, r5, r6, pc}
.L902:
	.align	2
.L901:
	.word	.LANCHOR0
	.word	.LANCHOR0+2292
	.word	.LC1
	.word	.LANCHOR1+401
	.fnend
	.size	update_vpc_list.part.9, .-update_vpc_list.part.9
	.align	2
	.global	update_vpc_list
	.type	update_vpc_list, %function
update_vpc_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L911
	mov	r1, r0, asl #1
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	mov	r3, r0
	ldr	ip, [r2, #2296]
	ldrh	r1, [ip, r1]
	cmp	r1, #0
	bne	.L904
	movw	r1, #2564
	ldrh	r0, [r2, r1]
	cmp	r0, r3
	mvneq	r0, #0
	streqh	r0, [r2, r1]	@ movhi
	beq	.L906
	mov	r1, #2320
	ldrh	r1, [r2, r1]
	cmp	r1, r3
	beq	.L907
	mov	r1, #2368
	ldrh	r1, [r2, r1]
	cmp	r1, r3
	beq	.L907
	mov	r1, #2416
	ldrh	r2, [r2, r1]
	cmp	r2, r3
	beq	.L907
.L906:
	mov	r0, r3
	ldmfd	sp!, {r3, lr}
	b	update_vpc_list.part.9
.L904:
	bl	List_update_data_list
.L907:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L912:
	.align	2
.L911:
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
	beq	.L914
	ldr	r6, .L920
	mov	r5, r0, asl #1
	ldr	r3, [r6, #2296]
	ldrh	r4, [r3, r5]
	cmp	r4, #0
	subne	r4, r4, #1
	strneh	r4, [r3, r5]	@ movhi
	bne	.L914
	ldr	r0, .L920+4
	mov	r1, r7
	mov	r2, r4
	bl	printk
	ldr	r3, [r6, #2296]
	ldrh	r5, [r3, r5]
	cmp	r5, #0
	bne	.L916
	ldr	r0, .L920+8
	movw	r2, #2759
	ldr	r1, .L920+12
	mov	r4, r5
	bl	printk
	b	.L916
.L914:
	ldr	r6, .L920+16
	movw	r5, #3940
	movw	r3, #65535
	ldrh	r0, [r6, r5]
	cmp	r0, r3
	streqh	r7, [r6, r5]	@ movhi
	beq	.L919
	cmp	r0, r7
	beq	.L919
	bl	update_vpc_list
	ldr	r3, .L920
	strh	r7, [r6, r5]	@ movhi
	ldr	r1, [r3, #2292]
	ldr	r2, [r3, #2288]
	rsb	r2, r2, r1
	ldr	r1, .L920+20
	mov	r2, r2, asr #1
	mul	r2, r1, r2
	ldr	r1, [r3, #2296]
	uxth	r2, r2
	mov	r3, r2, asl #1
	ldrh	r3, [r1, r3]
	adds	r4, r0, #0
	movne	r4, #1
	cmp	r3, #0
	bne	.L916
	cmp	r7, r2
	beq	.L916
	ldr	r1, .L920+12
	movw	r2, #2775
	ldr	r0, .L920+8
	bl	printk
	b	.L916
.L919:
	mov	r4, #0
.L916:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L921:
	.align	2
.L920:
	.word	.LANCHOR0
	.word	.LC99
	.word	.LC1
	.word	.LANCHOR1+417
	.word	.LANCHOR4
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
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	mov	r4, r0
	.pad #28
	sub	sp, sp, #28
	mov	r5, #7
	ldr	r7, .L927
	ldr	r6, .L927+4
	b	.L923
.L926:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L924
	mov	r0, r4
	bl	get_new_active_ppa
	cmn	r0, #1
	str	r0, [sp, #8]
	beq	.L924
	ldr	r3, [r7, #-848]
	mvn	r2, #0
	str	r2, [sp, #20]
	add	r0, sp, #4
	str	r3, [sp, #12]
	ldr	r3, [r7, #-844]
	str	r3, [sp, #16]
	str	r2, [r3, #8]
	str	r2, [r3, #12]
	ldrh	r2, [r4, #0]
	strh	r2, [r3, #2]	@ movhi
	mov	r2, #0
	strh	r2, [r3, #0]	@ movhi
	ldr	r1, [r6, #2520]
	cmn	r1, #2
	str	r1, [r3, #4]
	addne	r1, r1, #1
	moveq	r1, #0
	mov	r3, r2
	str	r1, [r6, #2520]
	mov	r1, #1
	bl	FlashProgPages
	ldrh	r0, [r4, #0]
	bl	decrement_vpc_count
.L923:
	subs	r5, r5, #1
	bne	.L926
.L924:
	ldr	r3, .L927+4
	ldrh	r2, [r4, #0]
	ldrh	r0, [r4, #4]
	ldr	r1, [r3, #2296]
	mov	r2, r2, asl #1
	ldrh	ip, [r1, r2]
	rsb	r0, r0, ip
	strh	r0, [r1, r2]	@ movhi
	ldrh	r3, [r3, #74]
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L928:
	.align	2
.L927:
	.word	.LANCHOR3
	.word	.LANCHOR0
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
	beq	.L930
	ldr	r2, .L1000
	mov	r5, #0
	ldrh	r7, [r0, #2]
	ldrb	fp, [r0, #6]	@ zero_extendqisi2
	ldrh	r2, [r2, #74]
	str	r7, [sp, #4]
	cmp	r2, r7
	str	fp, [sp, #20]
	streqh	r5, [r0, #4]	@ movhi
	moveq	r3, r5	@ movhi
	ldrneh	r0, [r0, #16]
	bne	.L932
	b	.L998
.L933:
	add	r5, r5, #1
	uxth	r5, r5
	add	r2, r4, r5, asl #1
	ldrh	r0, [r2, #16]
.L932:
	cmp	r0, r3
	beq	.L933
	mov	r1, #1
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	sl, r0
	beq	.L934
	ldr	r3, .L1000
	mov	r5, #0
	ldr	r2, .L1000+4
	str	r4, [sp, #32]
	ldrh	r9, [r3, #4]
	ldr	r8, [r2, #3788]
	mov	r2, r4
	ldr	r7, [r3, #3148]
	ldrh	r6, [r3, #82]
	ldr	lr, [r3, #3152]
	ldrh	ip, [r3, #84]
	mov	r3, r5
	b	.L935
.L934:
	ldr	r3, [sp, #4]
	cmp	r3, #0
	beq	.L936
	ldr	r0, .L1000+8
	movw	r2, #1753
	ldr	r1, .L1000+12
	bl	printk
.L936:
	ldr	r7, [sp, #20]
	cmp	r7, r5
	cmpne	r7, #0
	beq	.L937
	ldr	r0, .L1000+8
	movw	r2, #1754
	ldr	r1, .L1000+12
	bl	printk
.L937:
	mov	r3, #0
	strh	r3, [r4, #2]	@ movhi
	strb	r3, [r4, #6]
	b	.L930
.L939:
	ldrh	r0, [r2, #16]
	movw	fp, #65535
	cmp	r0, fp
	beq	.L938
	mov	fp, #20
	orr	r0, sl, r0, asl #10
	mla	r1, fp, r5, r8
	str	r0, [r1, #4]
	mul	r0, r6, r5
	add	fp, r0, #3
	cmp	r0, #0
	movlt	r0, fp
	bic	r0, r0, #3
	add	r0, r7, r0
	str	r0, [r1, #8]
	mul	r0, ip, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	fp, r0, #3
	cmp	r0, #0
	movlt	r0, fp
	bic	r0, r0, #3
	add	r0, lr, r0
	str	r0, [r1, #12]
.L938:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L935:
	cmp	r3, r9
	bne	.L939
	ldr	r6, .L1000+4
	mov	r2, #0
	ldr	r9, .L1000
	mov	r1, r5
	ldr	r0, [r6, #3788]
	bl	FlashReadPages
	ldr	r2, [r9, #2520]
	movw	r3, #65535
	str	r3, [sp, #8]
	uxth	r3, sl
	ldr	r7, [r6, #3788]
	str	r2, [sp, #16]
	mov	r6, #0
	mov	fp, r3
	b	.L940
.L945:
	ldr	r3, [r7, #0]
	cmp	r3, #0
	strne	fp, [sp, #8]
	bne	.L943
	ldr	r3, [r7, #12]
	ldr	r8, [r3, #4]
	cmn	r8, #1
	beq	.L942
	ldr	r1, [r9, #2520]
	mov	r0, r8
	str	r3, [sp, #0]
	bl	ftl_cmp_data_ver
	ldr	r3, [sp, #0]
	cmp	r0, #0
	addne	r8, r8, #1
	strne	r8, [r9, #2520]
.L942:
	ldr	r3, [r3, #0]
	cmn	r3, #1
	bne	.L943
	ldr	r3, .L1000+4
	mov	r2, #20
	uxth	fp, sl
	ldr	r3, [r3, #3788]
	mla	r6, r2, r6, r3
	ldr	r0, [r6, #4]
	b	.L995
.L943:
	add	r6, r6, #1
	add	r7, r7, #20
	uxth	r6, r6
.L940:
	cmp	r6, r5
	bne	.L945
	ldr	r3, .L1000+4
	add	fp, sl, #1
	uxth	fp, fp
	ldr	r3, [r3, #3788]
	ldr	r0, [r3, #4]
.L995:
	ubfx	r0, r0, #10, #16
	bl	P2V_plane
	ldr	r3, .L1000
	ldr	r1, [sp, #4]
	ldrh	r3, [r3, #74]
	cmp	r3, fp
	streqh	fp, [r4, #2]	@ movhi
	moveq	r3, #0
	streqb	r3, [r4, #6]
	streqh	r3, [r4, #4]	@ movhi
	ldr	r3, [sp, #20]
	str	r0, [sp, #12]
	ldr	r7, [sp, #12]
	cmp	r7, r3
	cmpeq	fp, r1
	beq	.L999
.L947:
	ldr	r7, [sp, #8]
	movw	r3, #65535
	ldr	r2, [sp, #16]
	cmp	r7, r3
	sub	r5, r2, #1
	bne	.L948
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L949
.L948:
	ldr	r3, .L1000+16
	uxth	sl, sl
	ldr	r7, [sp, #4]
	movw	r9, #65535
	mov	r8, r5
	ldr	r2, [r3, #3948]
	cmn	r2, #1
	streq	r5, [r3, #3948]
	add	r3, r7, #7
	cmp	sl, r3
	subgt	r7, sl, #7
	ldrle	r7, [sp, #4]
	uxthgt	r7, r7
	b	.L993
.L954:
	ldrh	r1, [r2, #16]
	add	r3, r3, #1
	add	r2, r2, #2
	cmp	r1, r9
	uxth	r3, r3
	orrne	r1, r7, r1, asl #10
	mlane	r6, r0, r5, ip
	addne	r5, r5, #1
	uxthne	r5, r5
	strne	r1, [r6, #4]
.L959:
	cmp	r3, lr
	bne	.L954
	ldr	r6, .L1000+4
	mov	r2, #0
	mov	r1, r5
	ldr	r0, [r6, #3788]
	bl	FlashReadPages
	ldr	r3, [r6, #3788]
	mov	r2, #0
	ldr	r0, .L1000+16
	b	.L955
.L957:
	ldr	r1, [r3, #0]
	cmp	r1, #0
	bne	.L994
	ldr	r1, [r3, #12]
	ldrh	ip, [r1, #0]
	cmp	ip, r9
	beq	.L956
	ldr	r1, [r1, #4]
	cmn	r1, #1
	strne	r1, [r0, #3948]
.L956:
	add	r2, r2, #1
	add	r3, r3, #20
	uxth	r2, r2
.L955:
	cmp	r2, r5
	bne	.L957
	add	r7, r7, #1
	uxth	r7, r7
.L993:
	cmp	r7, sl
	bhi	.L958
	ldr	r3, .L1000+4
	mov	r5, #0
	ldr	r1, .L1000
	mov	r2, r4
	mov	r0, #20
	ldr	ip, [r3, #3788]
	mov	r3, r5
	ldrh	lr, [r1, #4]
	b	.L959
.L958:
	ldr	r3, .L1000+16
	mvn	r2, #0
	mov	r5, r8
	str	r2, [r3, #3948]
	b	.L949
.L994:
	mov	r5, r8
.L949:
	ldr	r2, .L1000+16
	movw	r3, #3998
	mov	r1, #1
	ldr	r0, .L1000+20
	ldr	r6, .L1000
	mov	sl, r4
	strh	r1, [r2, r3]	@ movhi
	bl	FtlMapBlkWriteDump_data
	ldr	r9, [sp, #4]
	str	fp, [sp, #28]
.L960:
	ldr	r3, .L1000+4
	mov	r4, #0
	ldrh	lr, [r6, #4]
	mov	r2, sl
	mov	r0, #20
	ldr	ip, [r3, #3788]
	mov	r3, r4
	b	.L961
.L963:
	ldrh	r1, [r2, #16]
	movw	r7, #65535
	add	r3, r3, #1
	add	r2, r2, #2
	cmp	r1, r7
	uxth	r3, r3
	orrne	r1, r9, r1, asl #10
	mlane	r7, r0, r4, ip
	addne	r4, r4, #1
	uxthne	r4, r4
	strne	r1, [r7, #4]
.L961:
	cmp	r3, lr
	bne	.L963
	ldr	r7, .L1000+4
	mov	r1, r4
	mov	r2, #0
	ldr	r0, [r7, #3788]
	bl	FlashReadPages
	mov	r3, #20
	mul	r3, r3, r4
	mov	r4, #0
	str	r3, [sp, #36]
	b	.L964
.L987:
	ldr	fp, [r7, #3788]
	add	fp, fp, r4
	ldr	r8, [fp, #4]
	ubfx	r0, r8, #10, #16
	str	r8, [sp, #52]
	bl	P2V_plane
	ldr	r3, [sp, #4]
	cmp	r9, r3
	bcc	.L965
	ldr	r1, [sp, #20]
	mov	r2, r3
	cmp	r0, r1
	movcs	r3, #0
	movcc	r3, #1
	cmp	r9, r2
	movne	r3, #0
	cmp	r3, #0
	bne	.L965
	ldr	r3, [sp, #12]
	ldr	r1, [sp, #28]
	cmp	r0, r3
	cmpeq	r9, r1
	beq	.L966
	ldr	r3, [fp, #0]
	cmn	r3, #1
	beq	.L967
	ldr	r8, [fp, #12]
	movw	r3, #61589
	ldrh	r2, [r8, #0]
	cmp	r2, r3
	bne	.L975
.L968:
	ldr	r5, [r8, #4]
	cmn	r5, #1
	beq	.L969
	ldr	r1, [r6, #2520]
	mov	r0, r5
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	addne	r3, r5, #1
	strne	r3, [r6, #2520]
.L969:
	ldrh	r2, [r8, #0]
	movw	r3, #61589
	cmp	r2, r3
	beq	.L970
	ldr	r0, .L1000+8
	movw	r2, #1903
	ldr	r1, .L1000+12
	bl	printk
.L970:
	ldr	r3, [r8, #8]
	add	r1, sp, #48
	ldr	fp, .L1000+16
	mov	r2, #0
	str	r3, [sp, #8]
	ldr	r3, [r8, #12]
	ldr	r0, [sp, #8]
	str	r3, [sp, #44]
	bl	log2phys
	ldr	r1, [fp, #3948]
	cmn	r1, #1
	beq	.L971
	mov	r0, r5
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L971
	ldr	r3, [sp, #44]
	cmn	r3, #1
	beq	.L972
	ldr	r0, [r7, #3788]
	mov	r1, #1
	mov	r2, #0
	add	r0, r0, r4
	ldr	fp, [r0, #12]
	str	r3, [r0, #4]
	str	fp, [sp, #16]
	bl	FlashReadPages
	ldr	r3, [fp, #4]
	str	r3, [sp, #24]
	ldr	r3, [r7, #3788]
	add	fp, r3, r4
	ldr	r3, [r3, r4]
	cmn	r3, #1
	bne	.L973
	b	.L974
.L972:
	ldr	r3, [sp, #52]
	ldr	r2, [sp, #48]
	cmp	r2, r3
	bne	.L975
	ldr	r0, [sp, #8]
	add	r1, sp, #44
	mov	r2, #1
	bl	log2phys
.L975:
	ldrh	r0, [sl, #0]
	b	.L997
.L973:
	ldr	r3, [sp, #16]
	ldr	r8, [r3, #8]
	ldr	r3, [sp, #8]
	cmp	r8, r3
	bne	.L974
	ldr	r1, .L1000+16
	ldr	r0, [r1, #3948]
	ldr	r1, [sp, #24]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L974
	ldr	r3, [sp, #48]
	ldr	r2, [sp, #52]
	cmp	r3, r2
	beq	.L979
.L976:
	ldr	r2, [sp, #44]
	cmp	r3, r2
	beq	.L974
	cmn	r3, #1
	streq	r3, [fp, #0]
	beq	.L978
	ldr	r2, [fp, #12]
	mov	r0, fp
	str	r3, [fp, #4]
	mov	r1, #1
	str	r2, [sp, #16]
	mov	r2, #0
	bl	FlashReadPages
.L978:
	ldr	r3, [r7, #3788]
	ldr	r3, [r3, r4]
	cmn	r3, #1
	beq	.L979
	ldr	r3, [sp, #16]
	ldr	fp, [r3, #4]
	ldr	r3, .L1000+16
	mov	r1, fp
	ldr	r0, [r3, #3948]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L979
	ldr	r0, [sp, #24]
	mov	r1, fp
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L974
.L979:
	mov	r0, r8
	ldr	r1, [sp, #44]
	bl	FtlReUsePrevPpa
.L974:
	ldrh	r0, [sl, #0]
	mvn	r3, #0
	str	r3, [sp, #44]
	bl	decrement_vpc_count
	b	.L981
.L971:
	ldr	r3, [sp, #52]
	ldr	r2, [sp, #48]
	cmp	r2, r3
	beq	.L981
	ldr	r0, [sp, #8]
	add	r1, sp, #52
	mov	r2, #1
	bl	log2phys
	ldr	fp, [sp, #48]
	cmn	fp, #1
	beq	.L981
	ldr	r3, [sp, #44]
	cmp	fp, r3
	beq	.L981
	ubfx	r0, fp, #10, #16
	bl	P2V_block_in_plane
	mov	r3, #2320
	ldrh	r3, [r6, r3]
	cmp	r3, r0
	beq	.L982
	mov	r3, #2368
	ldrh	r3, [r6, r3]
	cmp	r3, r0
	beq	.L982
	mov	r3, #2416
	ldrh	r3, [r6, r3]
	cmp	r3, r0
	bne	.L981
.L982:
	ldr	r0, [r7, #3788]
	mov	r1, #1
	mov	r2, #0
	str	fp, [r0, #4]
	ldr	r8, [r0, #12]
	bl	FlashReadPages
	ldr	r3, [r7, #3788]
	ldr	r1, [r8, #4]
	ldr	r3, [r3, #0]
	cmn	r3, #1
	beq	.L981
	mov	r0, r5
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	bne	.L981
	ldr	r0, [sp, #8]
	add	r1, sp, #48
	mov	r2, #1
	bl	log2phys
.L981:
	ldr	r0, [sp, #44]
	cmn	r0, #1
	beq	.L965
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r6, #12]
	cmp	r3, r0
	mov	r8, r0
	bhi	.L983
	ldr	r0, .L1000+8
	movw	r2, #2004
	ldr	r1, .L1000+12
	bl	printk
.L983:
	ldr	fp, .L1000
	mov	r3, r8, asl #1
	ldr	r2, [fp, #2296]
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L984
	mov	r0, r8
.L997:
	bl	decrement_vpc_count
	b	.L965
.L984:
	mov	r1, r8
	ldr	r0, .L1000+24
	bl	printk
	b	.L965
.L967:
	ldr	r3, .L1000+16
	ldr	r2, [r3, #4000]
	cmp	r2, #31
	addls	r1, r3, r2, asl #2
	addls	r2, r2, #1
	strls	r2, [r3, #4000]
	strls	r8, [r1, #4004]
	ldrh	r0, [sl, #0]
	bl	decrement_vpc_count
	ldr	r3, .L1000+16
	ldr	r2, [r3, #3948]
	cmn	r2, #1
	beq	.L996
.L986:
	cmp	r2, r5
	bls	.L965
.L996:
	str	r5, [r3, #3948]
.L965:
	add	r4, r4, #20
.L964:
	ldr	r3, [sp, #36]
	cmp	r4, r3
	bne	.L987
	add	r9, r9, #1
	ldrh	r3, [r6, #74]
	uxth	r9, r9
	cmp	r9, r3
	bne	.L960
	ldr	r2, .L1000
	mov	r3, #0
	mov	r4, sl
	strh	r9, [sl, #2]	@ movhi
	strh	r3, [sl, #4]	@ movhi
	ldrh	r1, [r2, #4]
	movw	r2, #65535
	ldr	r0, [sp, #32]
	b	.L988
.L990:
	add	r0, r0, #2
	ldrh	ip, [r0, #14]
	cmp	ip, r2
	beq	.L989
.L998:
	strb	r3, [r4, #6]
	b	.L930
.L989:
	add	r3, r3, #1
	uxth	r3, r3
.L988:
	cmp	r3, r1
	bne	.L990
	b	.L930
.L966:
	ldr	fp, [sp, #28]
	mov	r4, sl
	ldr	r7, [sp, #12]
	strh	fp, [sl, #2]	@ movhi
	strb	r7, [sl, #6]
.L999:
	mov	r0, r4
	mov	r1, fp
	mov	r2, r7
	bl	ftl_sb_update_avl_pages
.L930:
	mov	r0, #0
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1001:
	.align	2
.L1000:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC1
	.word	.LANCHOR1+437
	.word	.LANCHOR4
	.word	.LANCHOR3+3804
	.word	.LC100
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
	ldr	r4, .L1035
	mov	r5, r0
	movw	r8, #2316
.L1032:
	movw	r3, #2308
	ldrh	r2, [r4, r8]
	ldrh	r3, [r4, r3]
	add	r2, r2, r3
	ldrh	r3, [r4, #12]
	cmp	r2, r3
	ble	.L1004
	ldr	r0, .L1035+4
	movw	r2, #2582
	ldr	r1, .L1035+8
	bl	printk
.L1004:
	ldr	r3, .L1035+12
	cmp	r5, r3
	movne	r1, #0
	bne	.L1005
	ldrh	r3, [r4, r8]
	ldr	r1, [r4, #3136]
	mov	r2, r3, lsr #1
	mul	r1, r1, r3
	add	r2, r2, #1
	add	r1, r2, r1, lsr #2
	uxth	r1, r1
	cmp	r1, #0
	subne	r1, r1, #1
	uxthne	r1, r1
.L1005:
	ldr	r0, .L1035+16
	movw	r7, #2316
	bl	List_pop_index_node
	ldrh	r3, [r4, r8]
	ldr	sl, .L1035
	cmp	r3, #0
	uxth	r6, r0
	bne	.L1006
	ldr	r0, .L1035+4
	movw	r2, #2591
	ldr	r1, .L1035+8
	bl	printk
.L1006:
	ldrh	r3, [sl, r7]
	mov	r0, r5
	sub	r3, r3, #1
	strh	r3, [sl, r7]	@ movhi
	strh	r6, [r5, #0]	@ movhi
	bl	make_superblock
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1007
	ldr	r2, [r4, #2296]
	mov	r3, r6, asl #1
	mvn	r1, #0
	mov	r0, r6
	strh	r1, [r2, r3]	@ movhi
	bl	INSERT_DATA_LIST
	movw	r3, #2308
	ldrh	r3, [r4, r3]
	ldrh	r2, [r4, r8]
	add	r2, r2, r3
	ldrh	r3, [r4, #12]
	cmp	r2, r3
	ble	.L1032
	ldr	r1, .L1035+8
	movw	r2, #2602
	ldr	r0, .L1035+4
	bl	printk
	b	.L1032
.L1007:
	movw	r3, #2308
	ldrh	r2, [r4, r8]
	ldrh	r3, [r4, r3]
	add	r2, r2, r3
	ldrh	r3, [r4, #12]
	cmp	r2, r3
	ble	.L1009
	ldr	r0, .L1035+4
	movw	r2, #2605
	ldr	r1, .L1035+8
	bl	printk
.L1009:
	ldr	r3, .L1035+20
	mov	r7, #0
	ldrh	sl, [r4, #4]
	mov	r9, r5
	mov	r1, r5
	mov	r2, r7
	ldr	ip, [r3, #3768]
	mov	r0, r7
	mov	r3, ip
	stmib	sp, {r5, ip}
	b	.L1010
.L1012:
	str	r0, [r3, #8]
	movw	r5, #65535
	str	r0, [r3, #12]
	ldrh	lr, [r1, #16]
	cmp	lr, r5
	beq	.L1011
	ldr	r5, [sp, #8]
	mov	ip, #20
	mov	lr, lr, asl #10
	mla	fp, ip, r7, r5
	add	r7, r7, #1
	uxth	r7, r7
	str	lr, [fp, #4]
.L1011:
	add	r2, r2, #1
	add	r3, r3, #20
	add	r1, r1, #2
	uxth	r2, r2
.L1010:
	cmp	r2, sl
	bne	.L1012
	cmp	r7, #0
	ldr	r5, [sp, #4]
	bne	.L1013
	ldr	r0, .L1035+4
	movw	r2, #2616
	ldr	r1, .L1035+8
	bl	printk
.L1013:
	movw	r3, #2564
	ldr	sl, .L1035
	ldrh	r3, [r4, r3]
	cmp	r3, r6
	bne	.L1014
	ldr	r0, .L1035+4
	movw	r2, #2618
	ldr	r1, .L1035+8
	bl	printk
.L1014:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	ldr	r2, [sl, #2300]
	cmp	r3, #0
	mov	r3, r6, asl #1
	ldrh	r1, [r2, r3]
	bne	.L1015
	cmp	r1, #0
	ldrneh	r0, [r4, #64]
	moveq	r1, #2
	addne	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, [r4, #2524]
	mov	r0, r6
	mov	r1, #0
	add	r3, r3, #1
	str	r3, [r4, #2524]
	b	.L1034
.L1015:
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, [sl, #2528]
	mov	r0, r6
	mov	r1, #1
	add	r3, r3, #1
	str	r3, [sl, #2528]
.L1034:
	bl	ftl_set_blk_mode
	ldr	r3, [r4, #2300]
	mov	fp, r6, asl #1
	ldr	sl, .L1035
	ldr	r2, [r4, #2540]
	ldrh	r3, [r3, fp]
	ldr	r0, [sl, #2524]
	cmp	r3, r2
	ldrh	r2, [sl, #64]
	strhi	r3, [r4, #2540]
	ldr	r3, [sl, #2528]
	ldrh	r1, [sl, #12]
	mla	r0, r0, r2, r3
	bl	__aeabi_uidiv
	ldr	r3, .L1035+24
	ldr	r3, [r3, #3984]
	ldr	r2, [r3, #16]
	add	r2, r2, #1
	str	r2, [r3, #16]
	ldr	r3, .L1035+20
	ldr	r3, [r3, #3768]
	str	r0, [sl, #2532]
	mov	sl, #0
	b	.L1020
.L1021:
	add	sl, sl, #1
	ldr	r2, [r3, #-16]
	uxth	sl, sl
	bic	r2, r2, #1020
	bic	r2, r2, #3
	str	r2, [r3, #-16]
.L1020:
	cmp	sl, r7
	add	r3, r3, #20
	bne	.L1021
	ldr	r7, .L1035+20
	mov	r2, sl
	ldrb	r1, [r5, #8]	@ zero_extendqisi2
	ldr	r0, [r7, #3768]
	bl	FlashEraseBlocks
	mov	r1, #20
	mul	r1, r1, sl
	mov	sl, #0
	mov	r2, r7
	mov	ip, fp
	mov	r7, sl
	mov	fp, r6
	mov	r6, r1
	b	.L1022
.L1024:
	ldr	r1, [r2, #3768]
	add	r0, r1, sl
	ldr	r1, [r1, sl]
	cmn	r1, #1
	bne	.L1023
	ldr	r0, [r0, #4]
	add	r7, r7, #1
	str	r1, [sp, #20]
	ubfx	r0, r0, #10, #16
	str	r2, [sp, #16]
	str	ip, [sp, #12]
	bl	FtlBbmMapBadBlock
	ldr	r1, [sp, #20]
	strh	r1, [r9, #16]	@ movhi
	ldrb	r1, [r5, #7]	@ zero_extendqisi2
	sub	r1, r1, #1
	strb	r1, [r5, #7]
	ldr	ip, [sp, #12]
	ldr	r2, [sp, #16]
.L1023:
	add	sl, sl, #20
	add	r9, r9, #2
.L1022:
	cmp	sl, r6
	bne	.L1024
	cmp	r7, #0
	mov	r6, fp
	mov	fp, ip
	beq	.L1025
	mov	r0, r6
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L1025:
	ldrb	r2, [r5, #7]	@ zero_extendqisi2
	ldr	r3, .L1035
	cmp	r2, #0
	bne	.L1026
	ldr	r3, [r4, #2296]
	mvn	r2, #0
	mov	r0, r6
	strh	r2, [r3, fp]	@ movhi
	bl	INSERT_DATA_LIST
	b	.L1032
.L1026:
	ldrh	r1, [r3, #74]
	strh	r6, [r5, #0]	@ movhi
	mul	r2, r1, r2
	mov	r1, #0
	strh	r1, [r5, #2]	@ movhi
	strb	r1, [r5, #6]
	ldr	r1, [r3, #2516]
	uxth	r2, r2
	strh	r2, [r5, #4]	@ movhi
	str	r1, [r5, #12]
	add	r1, r1, #1
	str	r1, [r3, #2516]
	ldrh	r0, [r5, #0]
	ldr	r1, [r3, #2296]
	mov	r3, r0, asl #1
	strh	r2, [r1, r3]	@ movhi
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	beq	.L1027
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1028
.L1027:
	ldr	r0, .L1035+4
	movw	r2, #2671
	ldr	r1, .L1035+8
	bl	printk
.L1028:
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1036:
	.align	2
.L1035:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+459
	.word	.LANCHOR0+2416
	.word	.LANCHOR0+2312
	.word	.LANCHOR3
	.word	.LANCHOR4
	.fnend
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlWriteDump_data
	.type	FtlWriteDump_data, %function
FtlWriteDump_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movw	r3, #2324
	ldr	r5, .L1046
	.pad #24
	sub	sp, sp, #24
	ldrh	r3, [r5, r3]
	mov	r2, r5
	cmp	r3, #0
	beq	.L1038
	ldrb	r1, [r5, #2328]	@ zero_extendqisi2
	cmp	r1, #0
	bne	.L1038
	ldrb	r1, [r5, #2327]	@ zero_extendqisi2
	ldrh	r0, [r5, #74]
	mul	r1, r0, r1
	cmp	r3, r1
	beq	.L1038
	ldrb	r8, [r5, #2330]	@ zero_extendqisi2
	ldr	r7, [r5, #2480]
	cmp	r8, #0
	ldrh	r6, [r5, #4]
	bne	.L1037
	sub	r7, r7, #1
	mov	r2, r8
	mov	r1, sp
	mov	r0, r7
	bl	log2phys
	ldr	r2, .L1046+4
	ldr	r3, [sp, #0]
	str	r7, [sp, #20]
	ldr	r4, [r2, #-844]
	cmn	r3, #1
	ldr	r0, [r2, #-848]
	str	r3, [sp, #8]
	str	r4, [sp, #16]
	str	r0, [sp, #12]
	str	r8, [r4, #4]
	beq	.L1040
	add	r0, sp, #4
	mov	r1, #1
	mov	r2, r8
	bl	FlashReadPages
	b	.L1041
.L1040:
	mov	r1, #255
	ldrh	r2, [r5, #82]
	bl	ftl_memset
.L1041:
	ldr	r3, .L1046+8
	mov	r6, r6, asl #2
	ldr	r5, .L1046
	movw	r8, #2324
	strh	r3, [r4, #0]	@ movhi
	b	.L1042
.L1045:
	ldrh	r3, [r5, r8]
	cmp	r3, #0
	beq	.L1043
	ldr	r3, [sp, #8]
	add	r0, r0, #2320
	str	r7, [r4, #8]
	sub	r6, r6, #1
	str	r3, [r4, #12]
	mov	r3, #2320
	ldrh	r3, [r5, r3]
	strh	r3, [r4, #2]	@ movhi
	bl	get_new_active_ppa
	ldr	r3, [r5, #2520]
	mov	r2, #0
	mov	r1, #1
	str	r0, [sp, #8]
	add	r0, sp, #4
	str	r3, [r4, #4]
	add	r3, r3, #1
	cmn	r3, #1
	str	r3, [r5, #2520]
	moveq	r3, #0
	streq	r3, [r5, #2520]
	mov	r3, r2
	bl	FlashProgPages
	mov	r3, #2320
	ldrh	r0, [r5, r3]
	bl	decrement_vpc_count
.L1042:
	cmp	r6, #0
	ldr	r0, .L1046
	bne	.L1045
.L1043:
	mov	r3, #1
	strb	r3, [r0, #2330]
	b	.L1037
.L1038:
	mov	r3, #0
	strb	r3, [r2, #2330]
.L1037:
	add	sp, sp, #24
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1047:
	.align	2
.L1046:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	-3947
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
	ldr	r5, .L1052
	mov	r6, #12
	b	.L1049
.L1051:
	ldr	r3, [r5, #2464]
	mla	r3, r6, r4, r3
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L1050
	mov	r0, r4
	bl	flush_l2p_region
.L1050:
	add	r4, r4, #1
	uxth	r4, r4
.L1049:
	ldrh	r3, [r5, #110]
	cmp	r3, r4
	bhi	.L1051
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1053:
	.align	2
.L1052:
	.word	.LANCHOR0
	.fnend
	.size	l2p_flush, .-l2p_flush
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
	ldr	r6, .L1061
	ldrh	r4, [r0, #0]
	ldrh	r3, [r6, #12]
	cmp	r3, r4
	bcs	.L1055
	ldr	r0, .L1061+4
	movw	r2, #2678
	ldr	r1, .L1061+8
	bl	printk
.L1055:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L1056
	ldr	r2, [r6, #2296]
	mov	r3, r4, asl #1
	mov	r0, r4
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1057
	bl	INSERT_DATA_LIST
	b	.L1056
.L1057:
	bl	INSERT_FREE_LIST
.L1056:
	ldr	r2, .L1061+12
	mov	r3, #1
	strb	r3, [r5, #8]
	movw	r3, #3940
	ldrh	r0, [r2, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1058
	cmp	r4, r0
	bne	.L1059
	ldr	r3, .L1061
	ldr	r2, [r3, #2296]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1060
.L1059:
	bl	update_vpc_list
.L1060:
	ldr	r2, .L1061+12
	movw	r3, #3940
	mvn	r1, #0
	strh	r1, [r2, r3]	@ movhi
.L1058:
	mov	r0, r5
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	r0, #0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1062:
	.align	2
.L1061:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+484
	.word	.LANCHOR4
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
	ldr	r6, .L1076
	b	.L1064
.L1067:
	ldr	r0, [r5, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r4, #0]
	cmp	r3, r0
	bne	.L1065
	ldr	r2, [r6, #2296]
	mov	r3, r3, asl #1
	ldrh	r1, [r4, #4]
	ldrh	r0, [r2, r3]
	rsb	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	ldrh	r3, [r6, #74]
	strb	r8, [r4, #6]
	strh	r8, [r4, #4]	@ movhi
	strh	r3, [r4, #2]	@ movhi
.L1065:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L1066
	mov	r0, r4
	bl	allocate_new_data_superblock
.L1066:
	ldr	r3, [r6, #2708]
	add	r3, r3, #1
	str	r3, [r6, #2708]
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
	b	.L1074
.L1075:
	mov	r8, #0
.L1074:
	ldr	r3, [r5, #0]
	cmn	r3, #1
	beq	.L1067
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	ldrh	r3, [r6, #4]
	cmp	r2, r3
	bcc	.L1068
	ldr	r0, .L1076+4
	movw	r2, #945
	ldr	r1, .L1076+8
	bl	printk
.L1068:
	ldr	r3, [r5, #4]
	add	r1, sp, #4
	ldr	r0, [r5, #16]
	mov	r2, #1
	str	r3, [sp, #4]
	bl	log2phys
	ldr	r3, [r5, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L1069
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r6, #2296]
	mov	r3, r0, asl #1
	mov	r8, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L1070
	ldr	r0, .L1076+12
	mov	r1, r8
	bl	printk
.L1070:
	mov	r0, r8
	bl	decrement_vpc_count
.L1069:
	add	r7, r7, #1
	add	r5, r5, #20
.L1064:
	cmp	r7, sl
	bne	.L1075
	ldr	r3, .L1076
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	ldrh	r3, [r3, #4]
	cmp	r2, r3
	bcc	.L1063
	ldr	r0, .L1076+4
	mov	r2, #960
	ldr	r1, .L1076+8
	bl	printk
.L1063:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L1077:
	.align	2
.L1076:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1+513
	.word	.LC101
	.fnend
	.size	FtlProgPages, .-FtlProgPages
	.align	2
	.global	FtlCacheWriteBack
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, .L1080
	ldr	r0, [r4, #3936]
	cmp	r0, #0
	beq	.L1079
	ldr	r3, .L1080+4
	mov	r1, #1
	mov	r2, #0
	ldr	r3, [r3, #-4052]
	bl	FtlProgPages
	mov	r3, #0
	str	r3, [r4, #3936]
.L1079:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L1081:
	.align	2
.L1080:
	.word	.LANCHOR4
	.word	.LANCHOR5
	.fnend
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
	.align	2
	.global	FtlGcFreeTempBlock
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1102
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	mov	r2, #2416
	ldrh	r5, [r3, r2]
	movw	r2, #65535
	ldrh	r1, [r3, #74]
	cmp	r5, r2
	beq	.L1083
	cmp	r0, #0
	beq	.L1084
	ldr	r0, .L1102+4
	ldrh	ip, [r0, #4]
	cmp	ip, r2
	movne	r1, #2
	bne	.L1084
	mov	r2, #0
	strh	r2, [r0, #4]	@ movhi
	movw	r2, #2316
	ldrh	r3, [r3, r2]
	cmp	r3, #17
	movhi	r1, #2
.L1084:
	ldr	r4, .L1102
	add	r0, r4, #2416
	bl	FtlGcScanTempBlk
	cmn	r0, #1
	str	r0, [sp, #4]
	beq	.L1085
	ldr	r3, [r4, #2300]
	mov	r5, r5, asl #1
	ldrh	r2, [r3, r5]
	cmp	r2, #4
	bls	.L1086
	sub	r2, r2, #5
	mov	r0, #1
	strh	r2, [r3, r5]	@ movhi
	bl	FtlEctTblFlush
.L1086:
	ldr	r3, .L1102+8
	ldr	r3, [r3, #3792]
	cmp	r3, #0
	bne	.L1087
	ldr	r3, .L1102
	ldr	r0, [sp, #4]
	ldr	r2, [r3, #2708]
	ubfx	r0, r0, #10, #16
	add	r2, r2, #1
	str	r2, [r3, #2708]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L1087:
	ldr	r3, .L1102+8
	mov	r2, #0
	str	r2, [r3, #3792]
	b	.L1098
.L1085:
	ldr	r3, .L1102+4
	ldrh	r2, [r3, #4]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1098
.L1083:
	ldr	r3, .L1102+8
	mov	r5, #0
	ldr	r4, .L1102
	str	r5, [r3, #3792]
	mov	r3, #2416
	ldrh	r2, [r4, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1088
	bl	FtlCacheWriteBack
	movw	r3, #3176
	ldrh	r2, [r4, r3]
	ldrh	r1, [r4, #74]
	ldrb	r3, [r4, #2423]	@ zero_extendqisi2
	mul	r3, r1, r3
	cmp	r2, r3
	beq	.L1089
	ldr	r0, .L1102+12
	mov	r2, #162
	ldr	r1, .L1102+16
	bl	printk
.L1089:
	ldr	r5, .L1102
	mov	r3, #2416
	movw	sl, #3176
	mov	r6, #0
	ldrb	r1, [r5, #2423]	@ zero_extendqisi2
	ldrh	r0, [r5, #74]
	ldrh	r3, [r5, r3]
	ldr	r2, [r5, #2296]
	mul	r1, r0, r1
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, [r5, #2504]
	ldrh	r2, [r5, sl]
	add	r3, r2, r3
	str	r3, [r5, #2504]
	b	.L1090
.L1094:
	mov	r7, #12
	ldr	r8, [r5, #3172]
	mul	r7, r7, r6
	ldr	r3, [r5, #2480]
	add	r4, r8, r7
	ldr	r2, [r4, #8]
	cmp	r2, r3
	bcc	.L1091
	ldr	r0, .L1102+12
	mov	r2, #168
	ldr	r1, .L1102+16
	bl	printk
.L1091:
	ldr	r0, [r4, #8]
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	ldr	r3, [r8, r7]
	cmp	r0, r3
	bne	.L1092
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r1, r4, #4
	mov	r2, #1
	mov	r7, r0
	ldr	r0, [r4, #8]
	bl	log2phys
	mov	r0, r7
	b	.L1101
.L1092:
	ldr	r3, [r4, #4]
	cmp	r0, r3
	beq	.L1093
	mov	r3, #2416
	ldrh	r0, [r5, r3]
.L1101:
	bl	decrement_vpc_count
.L1093:
	add	r6, r6, #1
	uxth	r6, r6
.L1090:
	ldrh	r3, [r5, sl]
	ldr	r4, .L1102
	cmp	r3, r6
	bhi	.L1094
	movw	r0, #65535
	mov	r5, #2416
	bl	decrement_vpc_count
	ldrh	r0, [r4, r5]
	ldr	r2, [r4, #2296]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1095
	bl	INSERT_DATA_LIST
	b	.L1096
.L1095:
	bl	INSERT_FREE_LIST
.L1096:
	ldr	r4, .L1102
	movw	r3, #3176
	mvn	r6, #0
	strh	r6, [r4, r5]	@ movhi
	mov	r5, #0
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3164
	strh	r5, [r4, r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	movw	r3, #2316
	ldrh	r2, [r4, r3]
	add	r3, r3, #232
	ldrh	r3, [r4, r3]
	add	r3, r3, r3, asl #1
	cmp	r2, r3, lsr #2
	movwgt	r3, #2564
	movgt	r2, #20
	strgth	r6, [r4, r3]	@ movhi
	addgt	r3, r3, #560
	strgth	r2, [r4, r3]	@ movhi
	b	.L1088
.L1098:
	mov	r5, #1
.L1088:
	mov	r0, r5
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L1103:
	.align	2
.L1102:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR3
	.word	.LC1
	.word	.LANCHOR1+526
	.fnend
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	2
	.global	Ftl_gc_temp_data_write_back
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r2, #0
	ldr	r4, .L1110
	mov	r3, r2
	ldr	r5, .L1110+4
	mov	r6, #0
	mov	r7, #20
	ldr	r0, [r4, #3968]
	ldr	r1, [r5, #3140]
	bl	FlashProgPages
	b	.L1105
.L1108:
	mul	r3, r7, r6
	ldr	r1, [r4, #3968]
	add	r2, r1, r3
	ldr	r1, [r1, r3]
	cmn	r1, #1
	bne	.L1106
	mov	r2, #2416
	ldr	lr, [r8, #2296]
	ldrh	ip, [r8, r2]
	mov	r4, #0
	mov	ip, ip, asl #1
	strh	r4, [lr, ip]	@ movhi
	strh	r1, [r8, r2]	@ movhi
	ldr	r2, [r8, #2708]
	add	r2, r2, #1
	str	r2, [r8, #2708]
	ldr	r2, [r0, #3968]
	add	r3, r2, r3
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	bl	FtlGcPageVarInit
	mov	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1106:
	ldr	r3, [r2, #12]
	add	r6, r6, #1
	ldr	r1, [r2, #4]
	uxth	r6, r6
	ldr	r0, [r3, #12]
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
.L1105:
	ldr	r1, [r5, #3140]
	ldr	r8, .L1110+4
	cmp	r6, r1
	ldr	r0, .L1110
	bcc	.L1108
	ldr	r0, [r0, #3968]
	bl	FtlGcBufFree
	movw	r3, #2420
	ldrh	r3, [r8, r3]
	mov	r0, #0
	str	r0, [r8, #3140]
	cmp	r3, r0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, pc}
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	mov	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1111:
	.align	2
.L1110:
	.word	.LANCHOR4
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
	ldr	r4, .L1114
	add	r0, r4, #2416
	ldrh	r1, [r4, #74]
	bl	FtlGcScanTempBlk
	movw	r3, #2418
	ldrh	r2, [r4, r3]
	ldrh	r3, [r4, #74]
	cmp	r2, r3
	ldmccfd	sp!, {r4, pc}
	ldr	r0, .L1114+4
	bl	FtlMapBlkWriteDump_data
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	ldr	r3, .L1114+8
	mov	r2, #0
	str	r2, [r3, #3792]
	ldmfd	sp!, {r4, pc}
.L1115:
	.align	2
.L1114:
	.word	.LANCHOR0
	.word	.LANCHOR3+3804
	.word	.LANCHOR3
	.fnend
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.align	2
	.global	FtlPowerLostRecovery
	.type	FtlPowerLostRecovery, %function
FtlPowerLostRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, #0
	ldr	r5, .L1117
	ldr	r3, .L1117+4
	add	r6, r5, #2320
	add	r5, r5, #2368
	str	r4, [r3, #4000]
	mov	r0, r6
	bl	FtlRecoverySuperblock
	mov	r0, r6
	bl	FtlSlcSuperblockCheck
	mov	r0, r5
	bl	FtlRecoverySuperblock
	mov	r0, r5
	bl	FtlSlcSuperblockCheck
	bl	FtlGcPageRecovery
	movw	r0, #65535
	bl	decrement_vpc_count
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1118:
	.align	2
.L1117:
	.word	.LANCHOR0
	.word	.LANCHOR4
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
	mov	r3, #2416
	ldr	r5, .L1122
	ldrh	r2, [r5, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1120
	movw	r3, #2420
	ldrh	r3, [r5, r3]
	cmp	r3, #0
	bne	.L1121
.L1120:
	bl	FtlCacheWriteBack
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	ldr	r0, .L1122+4
	mov	r4, #0
	strb	r4, [r5, #2424]
	bl	allocate_data_superblock
	movw	r3, #3164
	strh	r4, [r5, r3]	@ movhi
	add	r3, r3, #12
	strh	r4, [r5, r3]	@ movhi
	bl	l2p_flush
	mov	r0, r4
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L1121:
	ldr	r0, .L1122+4
	ldmfd	sp!, {r3, r4, r5, lr}
	b	get_new_active_ppa
.L1123:
	.align	2
.L1122:
	.word	.LANCHOR0
	.word	.LANCHOR0+2416
	.fnend
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.type	rk_ftl_garbage_collect.part.10, %function
rk_ftl_garbage_collect.part.10:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1206
	movw	r2, #3180
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r5, r0
	ldrh	r0, [r3, r2]
	movw	ip, #65535
	.pad #36
	sub	sp, sp, #36
	cmp	r0, ip
	beq	.L1125
	movw	r1, #3178
	ldrh	r4, [r3, r1]
	cmp	r4, ip
	streqh	r0, [r3, r1]	@ movhi
	mvneq	r1, #0
	streqh	r1, [r3, r2]	@ movhi
.L1125:
	movw	r2, #2564
	ldr	r1, [r3, #3128]
	ldrh	r2, [r3, r2]
	movw	r4, #65535
	add	r1, r1, #1
	ldr	r6, .L1206
	cmp	r2, r4
	add	r1, r1, r5, asl #7
	str	r1, [r3, #3128]
	bne	.L1126
	mov	r3, #2416
	ldrh	r3, [r6, r3]
	cmp	r3, r2
	movne	r4, r2
	bne	.L1126
	movw	r2, #3178
	ldrh	r7, [r6, r2]
	cmp	r7, r3
	movne	r4, r3
	bne	.L1126
	movw	sl, #2316
	ldrh	r3, [r6, sl]
	cmp	r3, #23
	movhi	r3, #1024
	movls	r3, #5120
	cmp	r1, r3
	movls	r4, r7
	bls	.L1126
	ldr	r3, .L1206+4
	mov	r8, #0
	ldr	r2, .L1206+8
	str	r8, [r6, #3128]
	strh	r8, [r2, r3]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, r7
	mov	r4, r0
	mov	r9, r0
	bne	.L1128
	movw	r7, #3126
	ldrh	r2, [r6, sl]
	ldrh	r3, [r6, r7]
	cmp	r2, r3
	movcs	r3, #80
	bcs	.L1202
	mov	r0, #64
	bl	List_get_gc_head_node
	uxth	r0, r0
	cmp	r0, r4
	beq	.L1132
	ldr	r3, [r6, #2296]
	mov	r0, r0, asl #1
	ldrh	r3, [r3, r0]
	cmp	r3, #7
	movhi	r3, #64
	bhi	.L1202
	mov	r0, r8
	bl	List_get_gc_head_node
	mov	r3, #128
	strh	r3, [r6, r7]	@ movhi
	uxth	r4, r0
	cmp	r4, r9
	beq	.L1132
	b	.L1128
.L1202:
	strh	r3, [r6, r7]	@ movhi
	b	.L1132
.L1128:
	ldr	r1, .L1206
	movw	r3, #2316
	mov	r0, r4, asl #1
	ldrh	r2, [r1, r3]
	ldr	ip, [r1, #2300]
	ldr	r3, [r1, #2296]
	ldrh	r3, [r3, r0]
	ldrh	r0, [ip, r0]
	str	r0, [sp, #0]
	movw	r0, #3124
	ldrh	r1, [r1, r0]
	ldr	r0, .L1206+12
	str	r1, [sp, #4]
	mov	r1, r4
	bl	printk
.L1132:
	bl	FtlGcReFreshBadBlk
.L1126:
	movw	r1, #65535
	rsb	r0, r1, r4
	rsbs	r2, r0, #0
	ldr	r7, .L1206
	adc	r2, r2, r0
	cmp	r5, #0
	movne	r5, #0
	andeq	r5, r2, #1
	cmp	r5, #0
	beq	.L1133
	movw	r3, #2316
	ldrh	r2, [r7, r3]
	cmp	r2, #24
	movhi	r6, #1
	bhi	.L1134
	ldrh	r6, [r7, #74]
	cmp	r2, #16
	movhi	r6, r6, lsr #5
	bhi	.L1134
	cmp	r2, #12
	movhi	r6, r6, lsr #4
	bhi	.L1134
	cmp	r2, #8
	movhi	r6, r6, lsr #2
.L1134:
	movw	r1, #3124
	ldr	r3, .L1206
	ldrh	r0, [r7, r1]
	cmp	r0, r2
	mov	r0, r1
	bcs	.L1138
	mov	r2, #2416
	movw	ip, #65535
	ldrh	r2, [r3, r2]
	cmp	r2, ip
	bne	.L1139
	movw	ip, #3178
	ldrh	ip, [r3, ip]
	cmp	ip, r2
	bne	.L1139
	ldr	r2, .L1206+4
	ldr	r0, .L1206+8
	ldrh	r0, [r0, r2]
	cmp	r0, #0
	bne	.L1140
	ldr	r2, [r3, #2480]
	ldr	ip, [r3, #2472]
	add	r2, r2, r2, asl #1
	cmp	ip, r2, lsr #2
	movcs	r2, #18
	bcs	.L1203
.L1140:
	ldr	r3, .L1206
	movw	r2, #2548
	movw	r1, #3124
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
.L1203:
	strh	r2, [r3, r1]	@ movhi
	mov	r2, #0
	ldr	r3, .L1206
	str	r2, [r3, #3136]
	b	.L1143
.L1139:
	ldr	r3, .L1206
	movw	r2, #2548
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r0]	@ movhi
.L1138:
	ldr	r2, .L1206
	movw	r3, #3182
	movw	r4, #65535
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	addne	r6, r6, #32
	uxthne	r6, r6
	b	.L1144
.L1133:
	mov	r3, #2416
	ldrh	r3, [r7, r3]
	cmp	r3, r1
	bne	.L1204
	movw	r1, #3178
	ldrh	r1, [r7, r1]
	cmp	r1, r3
	movne	r2, #0
	andeq	r2, r2, #1
	cmp	r2, #0
	beq	.L1204
	movw	r2, #2564
	ldrh	r4, [r7, r2]
	cmp	r4, r3
	movne	r4, r3
	bne	.L1204
	movw	r3, #2316
	str	r5, [r7, #3136]
	ldrh	r8, [r7, r3]
	add	r3, r3, #808
	ldr	r6, .L1206+8
	ldrh	r2, [r7, r3]
	ldr	r5, .L1206+4
	cmp	r2, r8
	bcs	.L1145
	ldrh	r2, [r6, r5]
	cmp	r2, #0
	bne	.L1146
	ldr	r2, [r7, #2480]
	ldr	r1, [r7, #2472]
	add	r2, r2, r2, asl #1
	cmp	r1, r2, lsr #2
	movcs	r2, #18
	strcsh	r2, [r7, r3]	@ movhi
	bcs	.L1148
.L1146:
	ldr	r3, .L1206
	movw	r2, #2548
	movw	r1, #3124
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L1148:
	bl	FtlReadRefresh
	b	.L1205
.L1145:
	ldrh	r0, [r6, r5]
	cmp	r0, #0
	bne	.L1204
	movw	r2, #2548
	ldrh	r4, [r7, r2]
	add	r2, r4, r4, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r7, r3]	@ movhi
	bl	List_get_gc_head_node
	ldrh	r2, [r7, #76]
	ldrh	r1, [r7, #4]
	ldr	r3, [r7, #2296]
	mul	r2, r1, r2
	add	r2, r2, r2, lsr #31
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r3, [r3, r0]
	cmp	r3, r2, asr #1
	ble	.L1149
	sub	r4, r4, #1
	cmp	r8, r4
	blt	.L1149
	bl	FtlReadRefresh
	ldrh	r0, [r6, r5]
	b	.L1143
.L1149:
	cmp	r3, #0
	movwne	r4, #65535
	bne	.L1204
	movw	r0, #65535
	bl	decrement_vpc_count
	ldr	r2, .L1206
	movw	r3, #2316
	ldrh	r0, [r2, r3]
	add	r0, r0, #1
	b	.L1143
.L1204:
	mov	r6, #1
.L1144:
	ldr	r3, .L1206
	movw	r2, #2564
	movw	r0, #65535
	ldrh	r1, [r3, r2]
	cmp	r1, r0
	bne	.L1150
	cmp	r4, r1
	strneh	r4, [r3, r2]	@ movhi
	bne	.L1152
	movw	r2, #3178
	ldrh	r1, [r3, r2]
	cmp	r1, r4
	beq	.L1152
	ldr	r0, [r3, #2296]
	mov	r1, r1, asl #1
	ldrh	r1, [r0, r1]
	cmp	r1, #0
	mvneq	r1, #0
	streqh	r1, [r3, r2]	@ movhi
	ldr	r3, .L1206
	movw	r1, #2564
	ldrh	r0, [r3, r2]
	strh	r0, [r3, r1]	@ movhi
	mvn	r1, #0
	strh	r1, [r3, r2]	@ movhi
.L1152:
	ldr	r8, .L1206
	movw	r7, #2564
	mov	r3, #0
	ldrh	r0, [r8, r7]
	strb	r3, [r8, #2572]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1150
	bl	IsBlkInGcList
	ldr	r5, .L1206
	cmp	r0, #0
	mvnne	r3, #0
	strneh	r3, [r8, r7]	@ movhi
	ldrh	r2, [r8, r7]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1150
	ldr	r0, .L1206+16
	bl	make_superblock
	movw	r1, #2566
	mov	r3, #0
	ldr	r2, .L1206+8
	strh	r3, [r5, r1]	@ movhi
	ldr	r1, .L1206+20
	strb	r3, [r5, #2570]
	strh	r3, [r2, r1]	@ movhi
	movw	r3, #2564
	ldrh	r3, [r5, r3]
	ldr	r1, [r5, #2296]
	mov	r3, r3, asl #1
	ldrh	r1, [r1, r3]
	ldr	r3, .L1206+24
	strh	r1, [r2, r3]	@ movhi
.L1150:
	ldr	r3, .L1206
	movw	r1, #2564
	mov	r0, #2320
	ldrh	r0, [r3, r0]
	ldrh	r2, [r3, r1]
	cmp	r0, r2
	mov	r0, r3
	beq	.L1155
	mov	ip, #2368
	ldrh	r3, [r3, ip]
	cmp	r3, r2
	movne	fp, r4
	bne	.L1201
.L1156:
.L1155:
	mvn	r3, #0
	strh	r3, [r0, r1]	@ movhi
.L1205:
	ldr	r3, .L1206+4
	ldr	r2, .L1206+8
	ldrh	r0, [r2, r3]
	b	.L1143
.L1201:
	ldr	r4, .L1206
	movw	sl, #2564
	movw	r1, #65535
	ldrh	r3, [r4, sl]
	cmp	r3, r1
	bne	.L1158
	movw	r5, #3132
	mov	r3, #0
	str	r3, [r4, #3136]
.L1200:
	ldrh	r8, [r4, r5]
	mov	r0, r8
	bl	List_get_gc_head_node
	movw	r2, #65535
	uxth	r7, r0
	strh	r7, [r4, sl]	@ movhi
	cmp	r7, r2
	bne	.L1160
	ldr	r2, .L1206
	movw	r3, #3132
	mov	r1, #0
	mov	r0, #8
	strh	r1, [r2, r3]	@ movhi
	b	.L1143
.L1160:
	mov	r0, r7
	add	r8, r8, #1
	bl	IsBlkInGcList
	cmp	r0, #0
	strneh	r8, [r4, r5]	@ movhi
	bne	.L1200
	ldrh	r1, [r4, #74]
	mov	r7, r7, asl #1
	ldrh	r0, [r4, #4]
	uxth	r8, r8
	ldr	r3, [r4, #2296]
	strh	r8, [r4, r5]	@ movhi
	mul	r0, r0, r1
	ldrh	r2, [r3, r7]
	add	r1, r0, r0, lsr #31
	cmp	r2, r1, asr #1
	bgt	.L1163
	cmp	r8, #48
	bls	.L1164
	cmp	r2, #8
	bls	.L1164
	movw	r2, #3164
	ldrh	r2, [r4, r2]
	cmp	r2, #35
	bhi	.L1164
.L1163:
	mov	r2, #0
	strh	r2, [r4, r5]	@ movhi
.L1164:
	ldrh	r2, [r3, r7]
	movw	r3, #65535
	movw	r1, #2564
	cmp	r2, r0
	cmpge	fp, r3
	ldr	r3, .L1206
	movne	r0, #0
	moveq	r0, #1
	bne	.L1165
	mvn	r2, #0
	strh	r2, [r3, r1]	@ movhi
	movw	r2, #3132
	mov	r1, #0
	strh	r1, [r3, r2]	@ movhi
	b	.L1205
.L1165:
	cmp	r2, #0
	bne	.L1166
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r3, [r4, r5]
	add	r3, r3, #1
	strh	r3, [r4, r5]	@ movhi
	b	.L1200
.L1166:
	ldrh	r2, [r3, r1]
	mov	r1, #2320
	strb	r0, [r3, #2572]
	ldrh	r3, [r3, r1]
	cmp	r2, r3
	bne	.L1167
	ldr	r0, .L1206+28
	movw	r2, #658
	ldr	r1, .L1206+32
	bl	printk
.L1167:
	ldr	r4, .L1206
	movw	r5, #2564
	mov	r3, #2368
	ldrh	r2, [r4, r5]
	ldrh	r3, [r4, r3]
	cmp	r2, r3
	bne	.L1168
	ldr	r0, .L1206+28
	movw	r2, #659
	ldr	r1, .L1206+32
	bl	printk
.L1168:
	mov	r3, #2416
	ldrh	r2, [r4, r5]
	ldrh	r3, [r4, r3]
	cmp	r2, r3
	bne	.L1169
	ldr	r0, .L1206+28
	mov	r2, #660
	ldr	r1, .L1206+32
	bl	printk
.L1169:
	ldr	r0, .L1206+16
	bl	make_superblock
	ldr	r2, .L1206+8
	ldr	r0, .L1206+20
	mov	r3, #0
	ldr	r4, .L1206
	strh	r3, [r2, r0]	@ movhi
	movw	r0, #2564
	ldrh	r0, [r4, r0]
	ldr	ip, [r4, #2296]
	mov	r0, r0, asl #1
	ldrh	ip, [ip, r0]
	ldr	r0, .L1206+24
	strb	r3, [r4, #2570]
	strh	ip, [r2, r0]	@ movhi
	movw	r2, #2566
	strh	r3, [r4, r2]	@ movhi
.L1158:
	bl	FtlReadRefresh
	ldr	r3, .L1206+36
	mov	r2, #1
	mov	r9, #0
	ldr	r4, .L1206
	str	r2, [r3, #3956]
	movw	r2, #2566
	ldr	r3, .L1206
	str	fp, [sp, #20]
	ldrh	r7, [r3, #74]
	ldrh	r3, [r3, r2]
	add	r2, r3, r6
	str	r7, [sp, #16]
	cmp	r2, r7
	rsbgt	r6, r3, r7
	uxthgt	r6, r6
	mov	sl, r6
	b	.L1171
.L1173:
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
.L1180:
	cmp	r3, r5
	bne	.L1173
	ldr	r0, [r4, #3156]
	mov	r1, r6
	ldrb	r2, [r4, #2572]	@ zero_extendqisi2
	mov	r5, #0
	bl	FlashReadPages
	mov	fp, r5
	b	.L1174
.L1178:
	ldr	r3, [r4, #3156]
	add	r2, r3, r5
	ldr	r3, [r3, r5]
	ldr	r7, [r2, #12]
	cmn	r3, #1
	beq	.L1175
	ldrh	r3, [r7, #0]
	movw	r1, #61589
	cmp	r3, r1
	bne	.L1175
	ldr	r8, [r7, #8]
	cmn	r8, #1
	bne	.L1176
	ldr	r0, .L1206+28
	mov	r2, #696
	ldr	r1, .L1206+32
	bl	printk
.L1176:
	add	r1, sp, #28
	mov	r2, #0
	mov	r0, r8
	bl	log2phys
	ldr	r1, [r4, #3156]
	ldr	r2, [sp, #28]
	add	r1, r1, r5
	ldr	r3, [r1, #4]
	cmp	r2, r3
	bne	.L1175
	ldr	r2, .L1206+8
	mov	r8, #20
	ldr	r3, .L1206+20
	ldr	r1, [r1, #16]
	ldrh	r0, [r2, r3]
	add	r0, r0, #1
	strh	r0, [r2, r3]	@ movhi
	ldr	r3, .L1206+36
	ldr	r0, [r4, #3140]
	ldr	r2, [r3, #3968]
	mla	r2, r8, r0, r2
	str	r1, [r2, #16]
	str	r2, [sp, #12]
	str	r3, [sp, #8]
	bl	Ftl_get_new_temp_ppa
	ldr	r2, [sp, #12]
	mov	r1, #1
	str	r0, [r2, #4]
	ldr	r3, [sp, #8]
	ldr	r2, [r4, #3140]
	ldr	r3, [r3, #3968]
	mla	r8, r8, r2, r3
	ldr	r3, [r4, #3156]
	add	r3, r3, r5
	ldr	r2, [r3, #8]
	str	r2, [r8, #8]
	ldr	r3, [r3, #12]
	str	r3, [r8, #12]
	ldr	r3, [sp, #28]
	str	r3, [r7, #12]
	mov	r3, #2416
	ldrh	r3, [r4, r3]
	strh	r3, [r7, #2]	@ movhi
	ldr	r3, [r4, #2520]
	ldr	r0, [r4, #3156]
	str	r3, [r7, #4]
	add	r0, r0, r5
	ldr	r3, [r4, #3140]
	add	r3, r3, #1
	str	r3, [r4, #3140]
	bl	FtlGcBufAlloc
	ldrb	r3, [r4, #2423]	@ zero_extendqisi2
	ldr	r2, [r4, #3140]
	cmp	r2, r3
	beq	.L1177
	movw	r3, #2420
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L1175
.L1177:
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	ldrne	r3, .L1206+36
	movne	r2, #0
	strne	r2, [r3, #3956]
	bne	.L1205
.L1175:
	add	fp, fp, #1
	add	r5, r5, #20
	uxth	fp, fp
.L1174:
	cmp	fp, r6
	bne	.L1178
	add	r9, r9, #1
.L1171:
	uxth	r3, r9
	ldr	r2, .L1206
	cmp	r3, sl
	movw	r3, #2566
	bcs	.L1179
	ldrh	ip, [r4, r3]
	mov	r6, #0
	ldrh	r5, [r4, #4]
	mov	r3, r6
	ldr	lr, [r4, #3156]
	add	ip, ip, r9
	ldr	r2, .L1206+40
	mov	r0, #20
	b	.L1180
.L1179:
	ldrh	r1, [r2, r3]
	ldr	fp, [sp, #20]
	add	r6, sl, r1
	uxth	r6, r6
	strh	r6, [r2, r3]	@ movhi
	ldr	r3, [sp, #16]
	cmp	r6, r3
	bcs	.L1181
	ldr	r3, .L1206+8
	ldr	r2, .L1206+20
	ldr	r1, .L1206+24
	ldrh	r2, [r3, r2]
	ldrh	r3, [r3, r1]
	cmp	r2, r3
	bne	.L1182
.L1181:
	ldr	r3, .L1206
	ldr	r5, .L1206+8
	ldr	r3, [r3, #3140]
	cmp	r3, #0
	beq	.L1183
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	beq	.L1183
	ldr	r3, .L1206+36
	mov	r2, #0
	str	r2, [r3, #3956]
	ldr	r3, .L1206+4
	ldrh	r0, [r5, r3]
	b	.L1143
.L1183:
	ldr	r3, .L1206+20
	ldrh	ip, [r5, r3]
	cmp	ip, #0
	bne	.L1184
	ldr	r3, .L1206
	movw	r2, #2564
	ldrh	r1, [r3, r2]
	ldr	r0, [r3, #2296]
	mov	r1, r1, asl #1
	ldrh	r4, [r0, r1]
	cmp	r4, #0
	beq	.L1184
	strh	ip, [r0, r1]	@ movhi
	ldrh	r0, [r3, r2]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1184:
	ldr	r2, .L1206
	movw	r3, #2564
	mvn	r1, #0
	strh	r1, [r2, r3]	@ movhi
.L1182:
	ldr	r2, .L1206
	movw	r3, #2316
	ldrh	r3, [r2, r3]
	cmp	r3, #2
	ldrlsh	r6, [r2, #74]
	bls	.L1201
.L1185:
	ldr	r2, .L1206+36
	mov	r1, #0
	str	r1, [r2, #3956]
	ldr	r2, .L1206+4
	ldr	r1, .L1206+8
	ldrh	r0, [r1, r2]
	cmp	r0, #0
	addeq	r0, r3, #1
.L1143:
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1207:
	.align	2
.L1206:
	.word	.LANCHOR0
	.word	-4048
	.word	.LANCHOR5
	.word	.LC102
	.word	.LANCHOR0+2564
	.word	-4046
	.word	-4044
	.word	.LC1
	.word	.LANCHOR1+545
	.word	.LANCHOR4
	.word	.LANCHOR0+2578
	.fnend
	.size	rk_ftl_garbage_collect.part.10, .-rk_ftl_garbage_collect.part.10
	.align	2
	.global	rk_ftl_garbage_collect
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1214
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	ldr	r0, [r3, #3956]
	cmp	r0, #0
	movne	r0, #0
	ldmnefd	sp!, {r4, pc}
	ldr	r2, .L1214+4
	movw	r3, #2308
	ldrh	r3, [r2, r3]
	cmp	r3, #47
	ldmlsfd	sp!, {r4, pc}
	ldr	r3, .L1214+8
	ldrh	r1, [r3, #4]
	movw	r3, #65535
	cmp	r1, r3
	beq	.L1210
	mov	r1, #2416
	ldrh	r2, [r2, r1]
	cmp	r2, r3
	beq	.L1210
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	bne	.L1213
.L1210:
	mov	r0, r4
	ldmfd	sp!, {r4, lr}
	b	rk_ftl_garbage_collect.part.10
.L1213:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L1215:
	.align	2
.L1214:
	.word	.LANCHOR4
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
	.global	FtlDiscard
	.type	FtlDiscard, %function
FtlDiscard:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, lr}
	add	r2, r1, r0
	ldr	r5, .L1226
	mov	r7, r0
	mov	r4, r1
	ldr	r3, [r5, #112]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L1218
	cmp	r1, #31
	movls	r0, #0
	bls	.L1218
	bl	FtlCacheWriteBack
	ldrh	r6, [r5, #30]
	mov	r0, r7
	mov	r1, r6
	bl	__aeabi_uidiv
	mls	r7, r0, r6, r7
	mov	r5, r0
	uxth	r7, r7
	cmp	r7, #0
	beq	.L1219
	rsb	r6, r7, r6
	add	r5, r0, #1
	cmp	r6, r4
	movcs	r6, r4
	uxth	r6, r6
	rsb	r4, r6, r4
.L1219:
	mvn	r3, #0
	ldr	r6, .L1226
	str	r3, [sp, #4]
	ldr	r7, .L1226+4
	b	.L1220
.L1222:
	mov	r0, r5
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp, #0]
	cmn	r3, #1
	beq	.L1221
	ldr	r3, [r7, #-4040]
	add	r1, sp, #4
	mov	r2, #1
	mov	r0, r5
	add	r3, r3, #1
	str	r3, [r7, #-4040]
	ldr	r3, [r6, #2488]
	add	r3, r3, #1
	str	r3, [r6, #2488]
	bl	log2phys
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L1221:
	ldrh	r3, [r6, #30]
	add	r5, r5, #1
	rsb	r4, r3, r4
.L1220:
	ldrh	r3, [r6, #30]
	cmp	r4, r3
	bcs	.L1222
	ldr	r3, .L1226+4
	mov	r4, #0
	ldr	r2, [r3, #-4040]
	cmp	r2, #32
	bls	.L1225
	str	r4, [r3, #-4040]
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1225:
	mov	r0, r4
.L1218:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, pc}
.L1227:
	.align	2
.L1226:
	.word	.LANCHOR0
	.word	.LANCHOR5
	.fnend
	.size	FtlDiscard, .-FtlDiscard
	.align	2
	.global	FtlSysFlush
	.type	FtlSysFlush, %function
FtlSysFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	FtlCacheWriteBack
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
	ldr	r3, .L1231
	ldr	r3, [r3, #0]
	cmp	r3, #1
	bne	.L1230
	bl	FtlSysFlush
.L1230:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1232:
	.align	2
.L1231:
	.word	.LANCHOR2
	.fnend
	.size	sftl_deinit, .-sftl_deinit
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
	ldr	r6, .L1240
	ldr	r1, .L1240+4
	ldr	r0, .L1240+8
	bl	printk
	ldr	r5, .L1240+12
	ldrh	r2, [r6, #14]
	mov	r1, #0
	ldr	r0, [r5, #3988]
	mov	r2, r2, asl #1
	bl	ftl_memset
	b	.L1234
.L1236:
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L1235
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, [r5, #3988]
	mov	r0, r0, asl #1
	ldrh	r2, [r3, r0]
	add	r2, r2, #1
	strh	r2, [r3, r0]	@ movhi
.L1235:
	add	r4, r4, #1
.L1234:
	ldr	r3, [r6, #2480]
	cmp	r4, r3
	bcc	.L1236
	mov	r4, #0
	ldr	r5, .L1240
	ldr	r7, .L1240+12
	movw	r8, #65535
	b	.L1237
.L1239:
	ldr	r3, [r5, #2296]
	mov	r6, r4, asl #1
	ldrh	r2, [r3, r6]
	ldr	r3, [r7, #3988]
	ldrh	r3, [r3, r6]
	cmp	r2, r3
	beq	.L1238
	cmp	r2, r8
	beq	.L1238
	ldr	r0, .L1240+16
	mov	r1, r4
	bl	printk
	mov	r3, #2320
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1238
	mov	r3, #2416
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1238
	mov	r3, #2368
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1238
	ldr	r3, [r7, #3988]
	mov	r0, r4
	ldrh	r2, [r3, r6]
	ldr	r3, [r5, #2296]
	strh	r2, [r3, r6]	@ movhi
	bl	update_vpc_list
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1238:
	add	r4, r4, #1
	uxth	r4, r4
.L1237:
	ldrh	r3, [r5, #12]
	cmp	r3, r4
	bhi	.L1239
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L1241:
	.align	2
.L1240:
	.word	.LANCHOR0
	.word	.LANCHOR1+568
	.word	.LC89
	.word	.LANCHOR4
	.word	.LC103
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
	movw	r3, #3998
	ldr	r2, .L1253
	mov	r1, #0
	ldr	r4, .L1253+4
	strh	r1, [r2, r3]	@ movhi
	ldrh	r0, [r4, #8]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	movw	r3, #2552
	ldrh	r2, [r4, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1251
	bl	FtlLoadSysInfo
	subs	r5, r0, #0
	bne	.L1251
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	mov	r0, #1
	bl	FtlUpdateVaildLpn
	ldrh	r2, [r4, #110]
	ldr	r1, [r4, #2464]
	mov	r3, r5
	b	.L1244
.L1246:
	add	r0, r1, r5
	add	r5, r5, #12
	ldr	r0, [r0, #4]
	cmp	r0, #0
	blt	.L1245
	add	r3, r3, #1
.L1244:
	cmp	r3, r2
	blt	.L1246
.L1245:
	ldr	r4, .L1253+4
	movw	r1, #2268
	cmp	r3, r2
	ldrh	r0, [r4, r1]
	add	r0, r0, #1
	strh	r0, [r4, r1]	@ movhi
	blt	.L1247
	ldr	r2, .L1253
	movw	r3, #3998
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1248
.L1247:
	ldr	r0, .L1253+8
	bl	FtlSuperblockPowerLostFix
	ldr	r0, .L1253+12
	bl	FtlSuperblockPowerLostFix
	mov	r3, #2320
	ldrh	r3, [r4, r3]
	movw	r2, #2324
	ldr	r1, [r4, #2296]
	ldrh	r0, [r4, r2]
	mov	r3, r3, asl #1
	ldrh	ip, [r1, r3]
	rsb	r0, r0, ip
	strh	r0, [r1, r3]	@ movhi
	ldrh	r1, [r4, #74]
	movw	r3, #2322
	ldr	r0, [r4, #2296]
	strh	r1, [r4, r3]	@ movhi
	mov	r3, #0
	strh	r3, [r4, r2]	@ movhi
	mov	r2, #2368
	ldrh	r1, [r4, r2]
	add	r2, r2, #4
	strb	r3, [r4, #2326]
	ldrh	ip, [r4, r2]
	mov	r1, r1, asl #1
	ldrh	lr, [r0, r1]
	rsb	ip, ip, lr
	strh	ip, [r0, r1]	@ movhi
	ldrh	r0, [r4, #74]
	movw	r1, #2370
	strh	r3, [r4, r2]	@ movhi
	strb	r3, [r4, #2374]
	strh	r0, [r4, r1]	@ movhi
	ldr	r0, .L1253+16
	bl	FtlMapBlkWriteDump_data
	ldr	r0, .L1253+20
	bl	FtlMapBlkWriteDump_data
	movw	r3, #2270
	ldrh	r2, [r4, r3]
	add	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L1248:
	ldr	r4, .L1253+4
	mov	r3, #2320
	ldrh	r0, [r4, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1249
	movw	r3, #2324
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L1249
	movw	r3, #2372
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L1249
	bl	FtlGcRefreshBlock
	mov	r3, #2368
	ldrh	r0, [r4, r3]
	bl	FtlGcRefreshBlock
	add	r0, r4, #2320
	bl	allocate_new_data_superblock
	add	r0, r4, #2368
	bl	allocate_new_data_superblock
.L1249:
	ldr	r2, .L1253+4
	movw	r3, #2268
	ldrh	r0, [r2, r3]
	ands	r4, r0, #31
	bne	.L1252
	bl	FtlVpcCheckAndModify
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L1251:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1252:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1254:
	.align	2
.L1253:
	.word	.LANCHOR4
	.word	.LANCHOR0
	.word	.LANCHOR0+2320
	.word	.LANCHOR0+2368
	.word	.LANCHOR3+3804
	.word	.LANCHOR3+3868
	.fnend
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.global	sftl_init
	.type	sftl_init, %function
sftl_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.L1256:
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mvn	r3, #0
	ldr	r4, .L1260
	ldr	r5, .L1260+4
	ldr	r1, .L1260+8
	str	r3, [r4, #0]
	ldr	r0, .L1260+12
	bl	printk
	ldr	r0, .L1260+16
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldrh	r0, [r5, #8]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	bne	.L1257
	bl	FtlSysBlkInit
	cmp	r0, #0
	bne	.L1257
	mov	r3, #1
	str	r3, [r4, #0]
	movw	r3, #2316
	ldrh	r3, [r5, r3]
	cmp	r3, #15
	bhi	.L1257
	movw	r4, #8129
.L1258:
	mov	r0, #0
	mov	r1, #1
	bl	rk_ftl_garbage_collect
	subs	r4, r4, #1
	bne	.L1258
.L1257:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1261:
	.align	2
.L1260:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC71
	.word	.LC70
	.word	g_nand_phy_info
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
	ldr	r7, .L1279
	bl	FtlLoadFactoryBbt
	ldr	r4, .L1279+4
	add	sl, r7, #12
	sub	r6, r7, #124
	b	.L1263
.L1269:
	ldrh	r3, [sl], #2
	movw	r2, #65535
	ldr	r0, [r4, #-848]
	ldr	r9, [r4, #-844]
	cmp	r3, r2
	str	r0, [r4, #-860]
	str	r9, [r4, #-856]
	beq	.L1264
	ldrh	r8, [r6, #70]
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L1279+8
	mla	r8, r8, r5, r3
	mov	r3, r8, asl #10
	str	r3, [r4, #-864]
	bl	FlashReadPages
	ldrh	r2, [r6, #70]
	ldr	r0, [r7, #28]
	add	r2, r2, #7
	ldr	r1, [r4, #-860]
	mov	r2, r2, lsr #3
	bl	memcpy
	b	.L1265
.L1264:
	mov	r1, r5
	bl	FlashGetBadBlockList
	ldr	r0, [r4, #-860]
	ldr	r1, [r7, #28]
	bl	FtlBbt2Bitmap
	ldrh	fp, [r6, #70]
.L1267:
	sub	fp, fp, #1
	uxth	fp, fp
.L1278:
	ldrh	r0, [r6, #70]
	mla	r0, r0, r5, fp
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L1267
	mov	r1, #0
	mov	r2, #16
	strh	fp, [sl, #-2]	@ movhi
	ldr	r0, [r4, #-844]
	bl	ftl_memset
	mov	r1, #0
	mov	r2, #4096
	ldr	r0, [r4, #-848]
	bl	ftl_memset
	ldr	r3, .L1279+12
	ldr	r2, .L1279+16
	strh	r3, [r9, #0]	@ movhi
	mov	r3, #0
	str	r3, [r9, #4]
	ldrh	r3, [sl, #-2]
	ldrh	r8, [r6, #70]
	strh	r3, [r9, #2]	@ movhi
	ldrh	r3, [sl, #-2]
	ldr	r1, [r7, #28]
	ldr	r0, [r4, #-860]
	mla	r8, r8, r5, r3
	mov	r3, r8, asl #10
	str	r3, [r4, #-864]
	movw	r3, #3932
	ldrh	r2, [r2, r3]
	mov	r2, r2, asl #2
	bl	memcpy
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L1279+8
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r3, r1
	ldr	r0, .L1279+8
	mov	r2, r1
	bl	FlashProgPages
	ldr	r3, [r4, #-868]
	cmn	r3, #1
	bne	.L1265
	uxth	r0, r8
	bl	FtlBbmMapBadBlock
	b	.L1278
.L1265:
	uxth	r0, r8
	add	r5, r5, #1
	bl	FtlBbmMapBadBlock
	add	r7, r7, #4
.L1263:
	ldrh	r3, [r6, #26]
	cmp	r5, r3
	bcc	.L1269
	mov	r4, #0
	ldr	r6, .L1279+20
	b	.L1270
.L1271:
	mov	r0, r4
	add	r4, r4, #1
	bl	FtlBbmMapBadBlock
	uxth	r4, r4
.L1270:
	ldrh	r3, [r6, #86]
	ldr	r5, .L1279+20
	cmp	r3, r4
	bhi	.L1271
	ldrh	r4, [r5, #136]
	movw	r6, #65535
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L1272
.L1277:
	mov	r0, r4
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L1273
	mov	r0, r4
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1274
	mov	r0, r4
	bl	FtlBbmMapBadBlock
	b	.L1273
.L1274:
	ldrh	r3, [r5, #124]
	cmp	r3, r6
	streqh	r4, [r5, #124]	@ movhi
.L1275:
	ldrne	r3, .L1279+20
	strneh	r4, [r3, #128]	@ movhi
	bne	.L1276
.L1273:
	sub	r4, r4, #1
	uxth	r4, r4
.L1272:
	ldrh	r3, [r5, #136]
	sub	r3, r3, #48
	cmp	r4, r3
	bgt	.L1277
.L1276:
	ldr	r4, .L1279+20
	mov	r5, #0
	ldr	r3, .L1279+4
	mov	r1, #1
	mov	r2, #2
	str	r5, [r4, #132]
	ldr	r0, [r3, #3768]
	ldrh	r3, [r4, #124]
	strh	r5, [r4, #126]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r0, #4]
	ldrh	r3, [r4, #128]
	mov	r3, r3, asl #10
	str	r3, [r0, #24]
	bl	FlashEraseBlocks
	ldrh	r0, [r4, #124]
	bl	FtlBbmMapBadBlock
	ldrh	r0, [r4, #128]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [r4, #132]
	ldrh	r2, [r4, #128]
	add	r3, r3, #1
	str	r3, [r4, #132]
	ldrh	r3, [r4, #124]
	strh	r5, [r4, #126]	@ movhi
	strh	r2, [r4, #124]	@ movhi
	strh	r3, [r4, #128]	@ movhi
	bl	FtlBbmTblFlush
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1280:
	.align	2
.L1279:
	.word	.LANCHOR0+124
	.word	.LANCHOR3
	.word	.LANCHOR3-868
	.word	-3872
	.word	.LANCHOR4
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
	ldr	r3, .L1300
	mov	r2, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	str	r2, [r3, #2516]
	str	r2, [r3, #2520]
	ldrh	r0, [r3, #8]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	beq	.L1282
	bl	FtlMakeBbt
.L1282:
	mov	r3, #0
	ldr	r0, .L1300
	ldr	r2, .L1300+4
	ldr	r1, .L1300+8
	b	.L1283
.L1284:
	ldr	ip, [r2, #3784]
	mvn	lr, r3
	orr	lr, r3, lr, asl #16
	str	lr, [ip, r3, asl #2]
	ldr	ip, [r2, #3776]
	str	r1, [ip, r3, asl #2]
	add	r3, r3, #1
	uxth	r3, r3
.L1283:
	ldrh	ip, [r0, #30]
	ldr	r7, .L1300
	cmp	r3, ip, asl #7
	blt	.L1284
	ldrh	r6, [r7, #12]
	mov	r5, #0
	b	.L1285
.L1286:
	mov	r0, r6
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r5, r5, r0
	uxth	r5, r5
.L1285:
	ldrh	r3, [r7, #14]
	ldr	r4, .L1300
	cmp	r3, r6
	bhi	.L1286
	ldrh	r1, [r4, #4]
	sub	r3, r5, #3
	cmp	r3, r1, asl #1
	bge	.L1287
.L1291:
	mov	r5, #0
	ldr	r7, .L1300
	mov	r6, r5
	b	.L1288
.L1287:
	mov	r0, r5
	bl	__aeabi_uidiv
	ldr	r3, [r4, #104]
	add	r0, r0, r3
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldrh	r0, [r4, #8]
	bl	FtlFreeSysBlkQueueInit
	ldrh	r5, [r4, #12]
	b	.L1289
.L1290:
	mov	r0, r5
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r5, r5, #1
	uxth	r5, r5
.L1289:
	ldrh	r3, [r4, #14]
	cmp	r3, r5
	bhi	.L1290
	b	.L1291
.L1292:
	mov	r0, r6
	mov	r1, #0
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r5, r5, r0
	uxth	r5, r5
.L1288:
	ldrh	r3, [r7, #12]
	ldr	r4, .L1300
	cmp	r3, r6
	bhi	.L1292
	ldr	r3, .L1300+4
	movw	sl, #2548
	ldrh	r2, [r4, #14]
	ldrh	r6, [r4, #4]
	ldr	r9, [r4, #16]
	str	r2, [r3, #3772]
	mov	r1, r6
	mov	r0, r9
	bl	__aeabi_uidiv
	ubfx	r8, r0, #5, #16
	mov	r7, r0
	add	r3, r8, #36
	strh	r3, [r4, sl]	@ movhi
	mov	r3, #24
	str	r0, [r4, #2480]
	mul	r3, r3, r6
	cmp	r5, r3
	ble	.L1293
	rsb	r0, r5, r9
	mov	r1, r6
	bl	__aeabi_uidiv
	str	r0, [r4, #2480]
	mov	r0, r0, lsr #5
	add	r0, r0, #24
	strh	r0, [r4, sl]	@ movhi
.L1293:
	ldr	r3, .L1300
	ldrh	r2, [r3, #66]
	cmp	r2, #0
	beq	.L1294
	movw	r1, #2548
	ldrh	r0, [r3, r1]
	add	r0, r0, r2, lsr #1
	strh	r0, [r3, r1]	@ movhi
	mul	r0, r6, r2
	cmp	r0, r5
	strgt	r7, [r3, #2480]
	addgt	r2, r2, #32
	addgt	r8, r8, r2
	strgth	r8, [r3, r1]	@ movhi
.L1294:
	ldr	r4, .L1300
	movw	r3, #2548
	mov	r5, #2320
	ldr	r2, [r4, #2480]
	ldrh	r3, [r4, r3]
	rsb	r3, r3, r2
	mul	r6, r6, r3
	ldr	r3, .L1300+12
	str	r6, [r3, #3992]
	ldrh	r3, [r4, #74]
	mul	r6, r3, r6
	ldrh	r3, [r4, #30]
	str	r6, [r4, #2480]
	mul	r6, r3, r6
	str	r6, [r4, #112]
	bl	FtlBbmTblFlush
	ldrh	r2, [r4, #14]
	mov	r1, #0
	ldr	r0, [r4, #2296]
	mov	r2, r2, asl #1
	bl	ftl_memset
	movw	r2, #2564
	mvn	r1, #0
	strh	r1, [r4, r2]	@ movhi
	ldrh	r2, [r4, #12]
	movw	r1, #2566
	mov	r3, #0
	strh	r3, [r4, r1]	@ movhi
	movw	r1, #2322
	ldr	r0, [r4, #2476]
	strh	r3, [r4, r1]	@ movhi
	mov	r2, r2, lsr #3
	mov	r1, #255
	str	r3, [r4, #2472]
	strb	r3, [r4, #2570]
	strb	r3, [r4, #2572]
	strb	r3, [r4, #2326]
	strh	r3, [r4, r5]	@ movhi
	mov	r3, #1
	strb	r3, [r4, #2328]
	bl	ftl_memset
.L1295:
	ldr	r6, .L1300
	add	r0, r6, #2320
	bl	make_superblock
	ldrb	r3, [r4, #2327]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1296
	ldrh	r3, [r4, r5]
	mvn	r1, #0
	ldr	r2, [r4, #2296]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldrh	r3, [r4, r5]
	add	r3, r3, #1
	strh	r3, [r4, r5]	@ movhi
	b	.L1295
.L1296:
	ldr	r3, [r6, #2516]
	mov	r5, #2368
	ldr	r1, [r6, #2296]
	str	r3, [r6, #2332]
	add	r3, r3, #1
	str	r3, [r6, #2516]
	movw	r3, #2324
	ldrh	r0, [r6, r3]
	mov	r3, #2320
	ldrh	r2, [r6, r3]
	mov	r2, r2, asl #1
	strh	r0, [r1, r2]	@ movhi
	movw	r0, #2370
	ldrh	r3, [r6, r3]
	mov	r2, #0
	strh	r2, [r6, r0]	@ movhi
	add	r3, r3, #1
	strb	r2, [r6, #2374]
	strh	r3, [r6, r5]	@ movhi
	mov	r3, #1
	strb	r3, [r6, #2376]
.L1297:
	ldr	r4, .L1300
	add	r0, r4, #2368
	bl	make_superblock
	ldrb	r3, [r6, #2375]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1298
	ldrh	r3, [r6, r5]
	mvn	r1, #0
	ldr	r2, [r6, #2296]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldrh	r3, [r6, r5]
	add	r3, r3, #1
	strh	r3, [r6, r5]	@ movhi
	b	.L1297
.L1298:
	ldr	r3, [r4, #2516]
	mvn	r5, #0
	ldr	r2, [r4, #2296]
	str	r3, [r4, #2380]
	add	r3, r3, #1
	str	r3, [r4, #2516]
	movw	r3, #2372
	ldrh	r1, [r4, r3]
	mov	r3, #2368
	ldrh	r3, [r4, r3]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	mov	r3, #2416
	strh	r5, [r4, r3]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	movw	r3, #2552
	mov	r2, #0
	strh	r0, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r5, [r4, r3]	@ movhi
	ldr	r3, .L1300+12
	ldr	r2, [r3, #3992]
	movw	r3, #2558
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, [r4, #2516]
	str	r3, [r4, #2560]
	add	r3, r3, #1
	str	r3, [r4, #2516]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cmp	r0, #0
	mov	r0, #0
	ldreq	r3, .L1300+16
	moveq	r2, #1
	streq	r2, [r3, #0]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1301:
	.align	2
.L1300:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	168778952
	.word	.LANCHOR4
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
	mov	r7, r1
	mov	r9, r3
	str	r2, [sp, #16]
	bne	.L1303
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r3
	bl	FtlVendorPartWrite
	b	.L1304
.L1303:
	ldr	r6, .L1356
	ldr	ip, [sp, #16]
	ldr	r3, [r6, #112]
	add	r8, ip, r1
	cmp	r8, r3
	mvnhi	r0, #0
	bhi	.L1304
	ldr	r3, .L1356+4
	ldr	r0, [r3, #0]
	cmn	r0, #1
	beq	.L1304
	ldr	r3, .L1356+8
	mov	r2, #2048
	ldrh	r5, [r6, #30]
	mov	r0, r1
	str	r2, [r3, #-4036]
	mov	r1, r5
	bl	__aeabi_uidiv
	mov	r1, r5
	str	r0, [sp, #12]
	sub	r0, r8, #1
	bl	__aeabi_uidiv
	ldr	ip, [sp, #12]
	ldr	r3, [r6, #2492]
	rsb	r4, ip, r0
	str	r0, [sp, #32]
	add	ip, r4, #1
	str	ip, [sp, #4]
	add	r3, ip, r3
	str	r3, [r6, #2492]
	ldr	r3, .L1356+12
	ldr	r8, [r3, #3936]
	cmp	r8, #0
	beq	.L1305
	ldr	r3, [r8, #16]
	ldr	ip, [sp, #12]
	cmp	ip, r3
	beq	.L1306
	bl	FtlCacheWriteBack
	b	.L1305
.L1306:
	ldr	r3, [r6, #2496]
	mov	r1, r5
	mov	r0, r7
	add	r3, r3, #1
	str	r3, [r6, #2496]
	bl	__aeabi_uidivmod
	ldr	ip, [sp, #16]
	ldr	r0, [r8, #8]
	rsb	r5, r1, r5
	add	r0, r0, r1, asl #9
	cmp	r5, ip
	movcs	r5, ip
	mov	r1, r9
	mov	r6, r5, asl #9
	mov	r2, r6
	bl	memcpy
	cmp	r4, #0
	moveq	r0, r4
	beq	.L1304
	ldr	ip, [sp, #16]
	add	r7, r7, r5
	add	r9, r9, r6
	rsb	ip, r5, ip
	str	ip, [sp, #16]
	ldr	ip, [sp, #12]
	add	ip, ip, #1
	str	ip, [sp, #12]
	bl	FtlCacheWriteBack
	str	r4, [sp, #4]
.L1305:
	ldr	r3, .L1356+4
	ldr	r3, [r3, #8]
	cmp	r3, #0
	ldreq	r4, .L1356+16
	beq	.L1307
	ldr	r4, .L1356
	movw	r3, #2324
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	addeq	r4, r4, #2368
	addne	r4, r4, #2320
.L1307:
	mov	fp, #0
	ldr	r6, [sp, #12]
	str	fp, [sp, #28]
	mov	sl, r7
	ldr	r5, .L1356
	b	.L1308
.L1337:
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	ldrh	r3, [r5, #4]
	cmp	r2, r3
	bcc	.L1309
	ldr	r0, .L1356+20
	movw	r2, #1023
	ldr	r1, .L1356+24
	bl	printk
.L1309:
	ldrh	r8, [r4, #4]
	cmp	r8, #0
	bne	.L1310
	bl	FtlCacheWriteBack
	ldr	r3, .L1356+16
	ldr	r7, .L1356+4
	cmp	r4, r3
	bne	.L1311
	movw	r3, #2372
	ldrh	r8, [r5, r3]
	cmp	r8, #0
	bne	.L1312
	add	r0, r4, #48
	bl	allocate_new_data_superblock
	str	r8, [r7, #8]
.L1312:
	ldr	r0, .L1356+16
	bl	allocate_new_data_superblock
	ldr	r3, .L1356+4
	ldr	r2, [r3, #8]
	ldr	r3, .L1356+28
	cmp	r2, #0
	movne	r4, r3
	b	.L1313
.L1311:
	movw	r2, #2324
	str	r8, [r7, #8]
	ldrh	r2, [r5, r2]
	cmp	r2, #0
	movne	r4, r3
	bne	.L1313
	mov	r0, r4
	bl	allocate_new_data_superblock
.L1313:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L1310
	mov	r0, r4
	bl	allocate_new_data_superblock
.L1310:
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	ldrh	r2, [r4, #4]
	ldr	ip, [sp, #4]
	mov	r3, r3, asl #2
	cmp	r3, r2
	movcs	r3, r2
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	cmp	r3, ip
	movcc	ip, r3
	ldrh	r3, [r5, #4]
	str	ip, [sp, #8]
	cmp	r2, r3
	bcc	.L1314
	ldr	r0, .L1356+20
	mov	r2, #1056
	ldr	r1, .L1356+24
	bl	printk
.L1314:
	mov	r3, #0
	b	.L1355
.L1333:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L1347
	add	r1, sp, #48
	mov	r2, #0
	mov	r0, r6
	mov	r7, #20
	bl	log2phys
	mov	r0, r4
	bl	get_new_active_ppa
	ldr	ip, [sp, #20]
	ldr	r3, .L1356+12
	ldrh	r2, [r5, #84]
	mul	r7, r7, ip
	ldr	r1, [r3, #3972]
	add	r1, r1, r7
	str	r6, [r1, #16]
	str	r0, [r1, #4]
	mul	r0, r2, ip
	bic	r0, r0, #3
	str	r0, [sp, #40]
	ldr	r0, .L1356+32
	ldr	ip, [sp, #40]
	ldr	r0, [r0, #3780]
	add	r8, r0, ip
	str	r8, [r1, #12]
	str	r0, [sp, #44]
	mov	r1, #0
	mov	r0, r8
	str	r3, [sp, #0]
	bl	ftl_memset
	ldr	ip, [sp, #12]
	rsb	lr, ip, r6
	rsbs	ip, lr, #0
	adc	ip, ip, lr
	str	ip, [sp, #24]
	ldr	ip, [sp, #32]
	ldr	r0, [sp, #24]
	rsb	r3, ip, r6
	rsbs	ip, r3, #0
	adc	ip, ip, r3
	str	ip, [sp, #36]
	orrs	ip, ip, r0
	ldr	r3, [sp, #0]
	beq	.L1317
	cmp	r0, #0
	beq	.L1318
	ldrh	fp, [r5, #30]
	mov	r0, sl
	mov	r1, fp
	bl	__aeabi_uidivmod
	ldr	ip, [sp, #16]
	rsb	fp, r1, fp
	str	r1, [sp, #28]
	cmp	fp, ip
	movcs	fp, ip
	b	.L1319
.L1318:
	ldr	r3, [sp, #36]
	cmp	r3, #0
	beq	.L1319
	ldr	ip, [sp, #16]
	ldrh	r3, [r5, #30]
	add	fp, ip, sl
	mls	fp, r3, r6, fp
	ldr	r3, [sp, #24]
	str	r3, [sp, #28]
	uxth	fp, fp
.L1319:
	ldrh	r3, [r5, #30]
	ldr	ip, [sp, #24]
	cmp	fp, r3
	ldr	r3, .L1356+12
	ldr	r3, [r3, #3972]
	bne	.L1320
	add	r7, r3, r7
	cmp	ip, #0
	strne	r9, [r7, #8]
	muleq	r3, fp, r6
	beq	.L1354
	b	.L1322
.L1320:
	ldr	r2, .L1356+32
	cmp	ip, #0
	add	r3, r3, r7
	ldrne	r2, [r2, #3784]
	ldreq	r2, [r2, #3776]
	str	r2, [r3, #8]
	ldr	r2, [sp, #48]
	ldr	r3, .L1356+12
	cmn	r2, #1
	beq	.L1325
	ldr	r3, [r3, #3972]
	add	r0, sp, #52
	str	r6, [sp, #68]
	mov	r1, #1
	add	r3, r3, r7
	str	r2, [sp, #56]
	ldr	r2, [r3, #8]
	ldr	r3, [r3, #12]
	str	r2, [sp, #60]
	mov	r2, #0
	str	r3, [sp, #64]
	bl	FlashReadPages
	ldr	r3, [sp, #52]
	cmn	r3, #1
	ldreq	r3, [r5, #2684]
	addeq	r3, r3, #1
	streq	r3, [r5, #2684]
	beq	.L1327
	ldr	r3, [r8, #8]
	cmp	r3, r6
	beq	.L1328
	ldr	r3, [r5, #2684]
	mov	r2, r6
	ldr	r0, .L1356+36
	add	r3, r3, #1
	str	r3, [r5, #2684]
	ldr	r1, [r8, #8]
	bl	printk
.L1328:
	ldr	r3, [r8, #8]
	cmp	r3, r6
	beq	.L1327
	ldr	r1, .L1356+24
	movw	r2, #1109
	ldr	r0, .L1356+20
	bl	printk
	b	.L1327
.L1325:
	ldr	r3, [r3, #3972]
	mov	r1, #0
	ldrh	r2, [r5, #82]
	add	r3, r3, r7
	ldr	r0, [r3, #8]
	bl	ftl_memset
.L1327:
	ldr	r3, [sp, #24]
	mov	r2, fp, asl #9
	cmp	r3, #0
	ldr	r3, .L1356+12
	ldreqh	r1, [r5, #30]
	movne	r1, r9
	ldr	r3, [r3, #3972]
	ldrne	ip, [sp, #28]
	muleq	r1, r1, r6
	add	r3, r3, r7
	ldrne	r0, [r3, #8]
	rsbeq	r1, sl, r1
	ldreq	r0, [r3, #8]
	addne	r0, r0, ip, asl #9
	addeq	r1, r9, r1, asl #9
	bl	memcpy
	ldr	r3, [sp, #36]
	cmp	r3, #0
	beq	.L1322
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L1322
	ldr	r3, .L1356+12
	ldr	r2, [r3, #3972]
	add	r7, r2, r7
	str	r7, [r3, #3936]
	ldr	r3, .L1356+8
	str	r4, [r3, #-4052]
	b	.L1322
.L1317:
	ldr	r3, [r3, #3972]
	add	r7, r3, r7
	ldrh	r3, [r5, #30]
	mul	r3, r3, r6
.L1354:
	rsb	r3, sl, r3
	add	r3, r9, r3, asl #9
	str	r3, [r7, #8]
.L1322:
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	ldrh	r3, [r5, #4]
	cmp	r2, r3
	bcc	.L1331
	ldr	r0, .L1356+20
	movw	r2, #1130
	ldr	r1, .L1356+24
	bl	printk
.L1331:
	ldr	r3, .L1356+40
	ldr	r1, .L1356
	ldr	r0, [sp, #44]
	ldr	ip, [sp, #40]
	strh	r3, [r0, ip]	@ movhi
	ldr	r3, [r1, #2520]
	str	r6, [r8, #8]
	add	r6, r6, #1
	str	r3, [r8, #4]
	add	r3, r3, #1
	cmn	r3, #1
	str	r3, [r1, #2520]
	moveq	r3, #0
	streq	r3, [r5, #2520]
	ldr	r3, [sp, #48]
	str	r3, [r8, #12]
	ldrh	r3, [r4, #0]
	strh	r3, [r8, #2]	@ movhi
	ldr	r3, [sp, #20]
	add	r3, r3, #1
.L1355:
	str	r3, [sp, #20]
	ldr	ip, [sp, #20]
	ldr	r0, [sp, #8]
	cmp	ip, r0
	bne	.L1333
	b	.L1316
.L1347:
	ldr	r3, [sp, #20]
	str	r3, [sp, #8]
.L1316:
	ldr	r3, .L1356+12
	ldr	r2, [r3, #3936]
	cmp	r2, #0
	beq	.L1334
	ldr	ip, [sp, #8]
	subs	ip, ip, #1
	str	ip, [sp, #8]
	beq	.L1335
	ldr	ip, [sp, #4]
	sub	ip, ip, #1
	str	ip, [sp, #4]
.L1334:
	ldr	r0, [r3, #3972]
	mov	r2, #0
	ldr	r1, [sp, #8]
	mov	r3, r4
	bl	FtlProgPages
	ldr	ip, [sp, #4]
	ldr	r0, [sp, #8]
	cmp	ip, r0
	bcs	.L1336
	ldr	r0, .L1356+20
	movw	r2, #1145
	ldr	r1, .L1356+24
	bl	printk
.L1336:
	ldr	ip, [sp, #4]
	ldr	r0, [sp, #8]
	rsb	ip, r0, ip
	str	ip, [sp, #4]
.L1308:
	ldr	ip, [sp, #4]
	cmp	ip, #0
	bne	.L1337
.L1335:
	ldr	ip, [sp, #32]
	mov	r0, #0
	ldr	r2, [sp, #12]
	ldr	r5, .L1356
	rsb	r1, r2, ip
	bl	rk_ftl_garbage_collect
	movw	r3, #2316
	ldrh	r3, [r5, r3]
	cmp	r3, #15
	bhi	.L1349
	movw	r6, #2564
	mov	r4, r5
.L1350:
	ldrh	r3, [r5, r6]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1338
	movw	r2, #3178
	ldrh	r2, [r5, r2]
	cmp	r2, r3
	bne	.L1338
	mov	r0, #0
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L1338:
	movw	r2, #3126
	mov	r3, #128
	mov	r0, #1
	strh	r3, [r4, r2]	@ movhi
	movw	r2, #3124
	mov	r1, r0
	strh	r3, [r4, r2]	@ movhi
	bl	rk_ftl_garbage_collect
	mov	r0, #0
	mov	r1, #1
	bl	rk_ftl_garbage_collect
	movw	r3, #2316
	ldrh	r3, [r4, r3]
	cmp	r3, #8
	bls	.L1350
.L1349:
	mov	r0, #0
.L1304:
	add	sp, sp, #76
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1357:
	.align	2
.L1356:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR5
	.word	.LANCHOR4
	.word	.LANCHOR0+2320
	.word	.LC1
	.word	.LANCHOR1+589
	.word	.LANCHOR0+2368
	.word	.LANCHOR3
	.word	.LC104
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
	mov	r8, r3
	str	r2, [sp, #28]
	bne	.L1360
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r3
	bl	FtlVendorPartRead
	mov	fp, r0
	b	.L1361
.L1360:
	ldr	r3, [sp, #28]
	ldr	r5, .L1391
	add	r3, r3, r1
	str	r3, [sp, #4]
	ldr	r1, [sp, #4]
	ldr	r3, [r5, #112]
	cmp	r1, r3
	mvnhi	fp, #0
	bhi	.L1361
	ldr	r3, .L1391+4
	ldr	fp, [r3, #0]
	cmn	fp, #1
	beq	.L1361
	bl	FtlCacheWriteBack
	ldrh	r6, [r5, #30]
	mov	r0, r4
	mov	r9, #0
	mov	r1, r6
	bl	__aeabi_uidiv
	ldr	r3, [sp, #4]
	mov	r1, r6
	mov	r6, r9
	str	r0, [sp, #8]
	sub	r0, r3, #1
	bl	__aeabi_uidiv
	ldr	r1, [sp, #8]
	str	r9, [sp, #24]
	rsb	r3, r1, #1
	str	r9, [sp, #20]
	mov	r7, r1
	str	r9, [sp, #32]
	add	r3, r3, r0
	str	r3, [sp, #0]
	ldr	r2, [sp, #0]
	ldr	r3, [r5, #2484]
	str	r0, [sp, #12]
	add	r3, r2, r3
	str	r3, [r5, #2484]
	b	.L1386
.L1378:
	mov	r2, #0
	mov	r0, r7
	add	r1, sp, #52
	bl	log2phys
	ldr	r2, [sp, #52]
	cmn	r2, #1
	bne	.L1387
	b	.L1390
.L1366:
	mla	r0, r0, r7, sl
	cmp	r0, r4
	bcc	.L1365
	ldr	r3, [sp, #4]
	cmp	r0, r3
	bcs	.L1365
	rsb	r0, r4, r0
	mov	r1, #0
	mov	r2, #512
	add	r0, r8, r0, asl #9
	bl	ftl_memset
.L1365:
	add	sl, sl, #1
	b	.L1363
.L1390:
	mov	sl, #0
.L1363:
	ldrh	r0, [r5, #30]
	cmp	sl, r0
	bcc	.L1366
	b	.L1367
.L1387:
	ldr	r3, .L1391+8
	mov	sl, #20
	ldr	r1, [r3, #3788]
	mla	sl, sl, r6, r1
	ldr	r1, [sp, #8]
	cmp	r7, r1
	str	r2, [sl, #4]
	bne	.L1368
	ldr	r3, [r3, #3784]
	mov	r0, r4
	ldrh	fp, [r5, #30]
	str	r3, [sl, #8]
	mov	r1, fp
	bl	__aeabi_uidivmod
	ldr	r2, [sp, #28]
	rsb	r3, r1, fp
	str	r1, [sp, #20]
	cmp	r3, r2
	movcc	r2, r3
	str	r2, [sp, #24]
	cmp	r2, fp
	streq	r8, [sl, #8]
	b	.L1369
.L1368:
	ldr	r1, [sp, #12]
	cmp	r7, r1
	ldrneh	r3, [r5, #30]
	mulne	r3, r3, r7
	bne	.L1388
	ldr	r3, [r3, #3776]
	ldrh	r2, [r5, #30]
	ldr	r1, [sp, #4]
	str	r3, [sl, #8]
	mul	r3, r2, r7
	rsb	r9, r3, r1
	cmp	r9, r2
	bne	.L1369
.L1388:
	rsb	r3, r4, r3
	add	r3, r8, r3, asl #9
	str	r3, [sl, #8]
.L1369:
	ldrh	r3, [r5, #84]
	ldr	r2, .L1391+8
	str	r7, [sl, #16]
	mul	r3, r3, r6
	ldr	r2, [r2, #3780]
	add	r6, r6, #1
	bic	r3, r3, #3
	add	r3, r2, r3
	str	r3, [sl, #12]
.L1367:
	ldr	r3, [sp, #0]
	add	r7, r7, #1
	subs	r3, r3, #1
	str	r3, [sp, #0]
	beq	.L1371
	ldrh	r3, [r5, #4]
	cmp	r6, r3, asl #2
	bne	.L1386
.L1371:
	cmp	r6, #0
	beq	.L1386
	ldr	sl, .L1391+8
	mov	r1, r6
	mov	r2, #0
	mov	fp, r7
	ldr	r0, [sl, #3788]
	bl	FlashReadPages
	ldr	r3, [sp, #20]
	mov	r3, r3, asl #9
	str	r3, [sp, #36]
	ldr	r3, [sp, #24]
	mov	r3, r3, asl #9
	str	r3, [sp, #40]
	mov	r3, r9, asl #9
	str	r3, [sp, #44]
	mov	r3, #0
	str	r3, [sp, #16]
.L1377:
	ldr	r3, [sp, #16]
	mov	r1, #20
	mul	r7, r1, r3
	ldr	r3, [sl, #3788]
	ldr	r1, [sp, #8]
	add	r3, r3, r7
	ldr	r2, [r3, #16]
	cmp	r2, r1
	bne	.L1373
	ldr	r1, [r3, #8]
	ldr	r3, [sl, #3784]
	cmp	r1, r3
	bne	.L1374
	ldr	r3, [sp, #36]
	mov	r0, r8
	ldr	r2, [sp, #40]
	add	r1, r1, r3
	b	.L1389
.L1373:
	ldr	r1, [sp, #12]
	cmp	r2, r1
	bne	.L1374
	ldr	r1, [r3, #8]
	ldr	r3, [sl, #3776]
	cmp	r1, r3
	bne	.L1374
	ldrh	r0, [r5, #30]
	ldr	r3, [sp, #12]
	ldr	r2, [sp, #44]
	mul	r0, r0, r3
	rsb	r0, r4, r0
	add	r0, r8, r0, asl #9
.L1389:
	bl	memcpy
.L1374:
	ldr	r3, [sl, #3788]
	add	r1, r3, r7
	ldr	r2, [r3, r7]
	cmn	r2, #1
	streq	r2, [sp, #32]
	ldreq	r0, [r5, #2684]
	addeq	r0, r0, #1
	streq	r0, [r5, #2684]
	ldr	r3, [r3, r7]
	cmp	r3, #256
	bne	.L1376
	ldr	r0, [r1, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L1376:
	ldr	r3, [sp, #16]
	add	r3, r3, #1
	str	r3, [sp, #16]
	cmp	r3, r6
	bne	.L1377
	mov	r7, fp
	mov	r6, #0
.L1386:
	ldr	r3, [sp, #0]
	cmp	r3, #0
	bne	.L1378
	ldr	r3, .L1391
	movw	r2, #3182
	ldr	fp, [sp, #32]
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	bne	.L1379
	movw	r2, #2316
	ldrh	r3, [r3, r2]
	cmp	r3, #31
	bhi	.L1361
.L1379:
	mov	r0, #0
	mov	r1, #1
	bl	rk_ftl_garbage_collect
.L1361:
	mov	r0, fp
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1392:
	.align	2
.L1391:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR3
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
	.global	ftl_temp_buf
	.global	gc_ink_free_return_value
	.global	check_vpc_table
	.global	FtlUpdateVaildLpnCount
	.global	g_ect_tbl_power_up_flush
	.global	power_up_flag
	.global	low_format_en
	.global	gFtlInitStatus
	.global	DeviceCapacity
	.global	g_power_lost_recovery_flag
	.global	c_mlc_erase_count_value
	.global	g_recovery_ppa_tbl
	.global	g_recovery_page_min_ver
	.global	g_recovery_page_num
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
	.global	g_req_cache
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
	.type	__func__.3702, %object
	.size	__func__.3702, 17
__func__.3702:
	.ascii	"INSERT_DATA_LIST\000"
	.type	__func__.3697, %object
	.size	__func__.3697, 17
__func__.3697:
	.ascii	"INSERT_FREE_LIST\000"
	.type	__func__.3733, %object
	.size	__func__.3733, 17
__func__.3733:
	.ascii	"List_remove_node\000"
	.type	__func__.3765, %object
	.size	__func__.3765, 22
__func__.3765:
	.ascii	"List_update_data_list\000"
	.type	__func__.3871, %object
	.size	__func__.3871, 22
__func__.3871:
	.ascii	"select_l2p_ram_region\000"
	.type	__func__.4194, %object
	.size	__func__.4194, 16
__func__.4194:
	.ascii	"make_superblock\000"
	.type	__func__.4369, %object
	.size	__func__.4369, 19
__func__.4369:
	.ascii	"get_new_active_ppa\000"
	.type	__func__.4609, %object
	.size	__func__.4609, 15
__func__.4609:
	.ascii	"FlashReadPages\000"
	.type	__func__.4626, %object
	.size	__func__.4626, 15
__func__.4626:
	.ascii	"FlashProgPages\000"
	.type	__func__.3807, %object
	.size	__func__.3807, 26
__func__.3807:
	.ascii	"ftl_map_blk_alloc_new_blk\000"
	.type	__func__.4215, %object
	.size	__func__.4215, 18
__func__.4215:
	.ascii	"SupperBlkListInit\000"
	.type	__func__.3983, %object
	.size	__func__.3983, 14
__func__.3983:
	.ascii	"FtlScanSysBlk\000"
	.type	__func__.3832, %object
	.size	__func__.3832, 31
__func__.3832:
	.ascii	"Ftl_write_map_blk_to_last_page\000"
	.type	__func__.3846, %object
	.size	__func__.3846, 16
__func__.3846:
	.ascii	"FtlMapWritePage\000"
	.type	__func__.3818, %object
	.size	__func__.3818, 15
__func__.3818:
	.ascii	"ftl_map_blk_gc\000"
	.type	__func__.3774, %object
	.size	__func__.3774, 16
__func__.3774:
	.ascii	"load_l2p_region\000"
	.type	__func__.3888, %object
	.size	__func__.3888, 9
__func__.3888:
	.ascii	"log2phys\000"
	.type	__func__.4103, %object
	.size	__func__.4103, 16
__func__.4103:
	.ascii	"FtlReUsePrevPpa\000"
	.type	__func__.4256, %object
	.size	__func__.4256, 14
__func__.4256:
	.ascii	"ftl_check_vpc\000"
	.type	__func__.3580, %object
	.size	__func__.3580, 11
__func__.3580:
	.ascii	"FtlLoadBbt\000"
	.type	__func__.3537, %object
	.size	__func__.3537, 14
__func__.3537:
	.ascii	"FtlBbt2Bitmap\000"
	.type	__func__.3320, %object
	.size	__func__.3320, 11
__func__.3320:
	.ascii	"FtlMemInit\000"
	.type	__func__.4040, %object
	.size	__func__.4040, 15
__func__.4040:
	.ascii	"FtlLoadSysInfo\000"
	.type	__func__.3961, %object
	.size	__func__.3961, 15
__func__.3961:
	.ascii	"FtlVpcTblFlush\000"
	.type	__func__.4382, %object
	.size	__func__.4382, 16
__func__.4382:
	.ascii	"update_vpc_list\000"
	.type	__func__.4389, %object
	.size	__func__.4389, 20
__func__.4389:
	.ascii	"decrement_vpc_count\000"
	.type	__func__.4137, %object
	.size	__func__.4137, 22
__func__.4137:
	.ascii	"FtlRecoverySuperblock\000"
	.type	__func__.4341, %object
	.size	__func__.4341, 25
__func__.4341:
	.ascii	"allocate_data_superblock\000"
	.type	__func__.4362, %object
	.size	__func__.4362, 29
__func__.4362:
	.ascii	"allocate_new_data_superblock\000"
	.type	__func__.3445, %object
	.size	__func__.3445, 13
__func__.3445:
	.ascii	"FtlProgPages\000"
	.type	__func__.4459, %object
	.size	__func__.4459, 19
__func__.4459:
	.ascii	"FtlGcFreeTempBlock\000"
	.type	__func__.4565, %object
	.size	__func__.4565, 23
__func__.4565:
	.ascii	"rk_ftl_garbage_collect\000"
	.type	__func__.4240, %object
	.size	__func__.4240, 21
__func__.4240:
	.ascii	"FtlVpcCheckAndModify\000"
	.type	__func__.3473, %object
	.size	__func__.3473, 9
__func__.3473:
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
	.ascii	"SFTL version: 5.0.43 20180413\000"
.LC72:
	.ascii	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x"
	.ascii	"\012\000"
.LC73:
	.ascii	"FtlGcRefreshBlock  0x%x\012\000"
.LC74:
	.ascii	"FtlGcMarkBadPhyBlk %d 0x%x\012\000"
.LC75:
	.ascii	"prog read error: = %x\012\000"
.LC76:
	.ascii	"prog read s error: = %x %x %x\012\000"
.LC77:
	.ascii	"prog read d error: = %x %x %x\012\000"
.LC78:
	.ascii	"FtlFreeSysBlkQueueOut free count = %d\012\000"
.LC79:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d, error\012"
	.ascii	"\000"
.LC80:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d\012\000"
.LC81:
	.ascii	"FtlMapWritePage error = %x \012\000"
.LC82:
	.ascii	"FtlMapWritePage error = %x error count = %d\012\000"
.LC83:
	.ascii	"region_id = %x phyAddr = %x\012\000"
.LC84:
	.ascii	"spare:\000"
.LC85:
	.ascii	"map_ppn:\000"
.LC86:
	.ascii	"ftl_scan_all_data = %x\012\000"
.LC87:
	.ascii	"scan lpa = %x ppa= %x\012\000"
.LC88:
	.ascii	"lba = %x,addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC89:
	.ascii	"...%s enter...\012\000"
.LC90:
	.ascii	"FtlCheckVpc2 %x = %x  %x\012\000"
.LC91:
	.ascii	"free blk vpc error %x = %x  %x\012\000"
.LC92:
	.ascii	"%s error allocating memory. return -1\012\000"
.LC93:
	.ascii	"FtlVpcTblFlush error = %x error count = %d\012\000"
.LC94:
	.ascii	"no ect\000"
.LC95:
	.ascii	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\012\000"
.LC96:
	.ascii	"FtlBbmTblFlush error:%x\012\000"
.LC97:
	.ascii	"FtlBbmTblFlush error = %x error count = %d\012\000"
.LC98:
	.ascii	"FtlGcFreeBadSuperBlk 0x%x\012\000"
.LC99:
	.ascii	"decrement_vpc_count %x = %d\012\000"
.LC100:
	.ascii	"spuer block %x vpn is 0\012 \000"
.LC101:
	.ascii	"Ftlwrite decrement_vpc_count %x = %d\012\000"
.LC102:
	.ascii	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\012\000"
.LC103:
	.ascii	"FtlCheckVpc %x = %x  %x\012\000"
.LC104:
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
	.set	.LANCHOR3,. + 8184
	.set	.LANCHOR4,. + 24552
	.set	.LANCHOR5,. + 32736
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
	.type	ftl_temp_buf, %object
	.size	ftl_temp_buf, 4096
ftl_temp_buf:
	.space	4096
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
	.type	check_buf, %object
	.size	check_buf, 4096
check_buf:
	.space	4096
	.type	check_spare_buf, %object
	.size	check_spare_buf, 512
check_spare_buf:
	.space	512
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
	.type	req_read, %object
	.size	req_read, 4
req_read:
	.space	4
	.type	ftl_gc_temp_power_lost_recovery_flag, %object
	.size	ftl_gc_temp_power_lost_recovery_flag, 4
ftl_gc_temp_power_lost_recovery_flag:
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
	.type	p_vendor_block_ver_table, %object
	.size	p_vendor_block_ver_table, 4
p_vendor_block_ver_table:
	.space	4
	.type	p_sys_data_buf_1, %object
	.size	p_sys_data_buf_1, 4
p_sys_data_buf_1:
	.space	4
	.type	gVendorBlkInfo, %object
	.size	gVendorBlkInfo, 40
gVendorBlkInfo:
	.space	40
	.type	p_vendor_block_valid_page_count, %object
	.size	p_vendor_block_valid_page_count, 4
p_vendor_block_valid_page_count:
	.space	4
	.type	p_vendor_region_ppn_table, %object
	.size	p_vendor_region_ppn_table, 4
p_vendor_region_ppn_table:
	.space	4
	.type	check_vpc_table, %object
	.size	check_vpc_table, 16384
check_vpc_table:
	.space	16384
	.type	c_ftl_nand_bbm_buf_size, %object
	.size	c_ftl_nand_bbm_buf_size, 2
c_ftl_nand_bbm_buf_size:
	.space	2
	.space	2
	.type	g_req_cache, %object
	.size	g_req_cache, 4
g_req_cache:
	.space	4
	.type	g_tmp_data_superblock_id, %object
	.size	g_tmp_data_superblock_id, 2
g_tmp_data_superblock_id:
	.space	2
	.space	2
	.type	g_totle_swl_count, %object
	.size	g_totle_swl_count, 4
g_totle_swl_count:
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
	.type	req_gc_dst, %object
	.size	req_gc_dst, 4
req_gc_dst:
	.space	4
	.type	req_prgm, %object
	.size	req_prgm, 4
req_prgm:
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
	.type	gp_last_act_superblock, %object
	.size	gp_last_act_superblock, 4
gp_last_act_superblock:
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
	.type	gc_discard_updated, %object
	.size	gc_discard_updated, 4
gc_discard_updated:
	.space	4
	.type	g_ftl_nand_free_count, %object
	.size	g_ftl_nand_free_count, 4
g_ftl_nand_free_count:
	.space	4
	.type	low_format_en, %object
	.size	low_format_en, 4
low_format_en:
	.space	4
