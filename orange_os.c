
/*
	;***************************************************************
	; 内存看上去是这样的：
	;              ┃                                    ┃
	;              ┃                 .                  ┃
	;              ┃                 .                  ┃
	;              ┃                 .                  ┃
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;              ┃■■■■■■Page  Tables■■■■■■┃
	;              ┃■■■■■(大小由LOADER决定)■■■■┃
	;    00101000h ┃■■■■■■■■■■■■■■■■■■┃ PAGE_TBL_BASE
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;    00100000h ┃■■■■Page Directory Table■■■■┃ PAGE_DIR_BASE  <- 1M
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃□□□□□□□□□□□□□□□□□□┃
	;       F0000h ┃□□□□□□□System ROM□□□□□□┃
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃□□□□□□□□□□□□□□□□□□┃
	;       E0000h ┃□□□□Expansion of system ROM □□┃
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃□□□□□□□□□□□□□□□□□□┃
	;       C0000h ┃□□□Reserved for ROM expansion□□┃
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃□□□□□□□□□□□□□□□□□□┃ B8000h ← gs
	;       A0000h ┃□□□Display adapter reserved□□□┃
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃□□□□□□□□□□□□□□□□□□┃
	;       9FC00h ┃□□extended BIOS data area (EBDA)□┃
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;       90000h ┃■■■■■■■LOADER.BIN■■■■■■┃ somewhere in LOADER ← esp
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;       70000h ┃■■■■■■■KERNEL.BIN■■■■■■┃
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;              ┃■■■■■■■■■■■■■■■■■■┃ 7C00h~7DFFh : BOOT SECTOR, overwritten by the kernel
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;              ┃■■■■■■■■■■■■■■■■■■┃
	;        1000h ┃■■■■■■■■KERNEL■■■■■■■┃ 1000h ← KERNEL 入口 (KRNL_ENT_PT_PHY_ADDR)
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃                                    ┃
	;         500h ┃              F  R  E  E            ┃
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃□□□□□□□□□□□□□□□□□□┃
	;         400h ┃□□□□ROM BIOS parameter area □□┃
	;              ┣━━━━━━━━━━━━━━━━━━┫
	;              ┃◇◇◇◇◇◇◇◇◇◇◇◇◇◇◇◇◇◇┃
	;           0h ┃◇◇◇◇◇◇Int  Vectors◇◇◇◇◇◇┃
	;              ┗━━━━━━━━━━━━━━━━━━┛ ← cs, ds, es, fs, ss
	;
	;
	;		┏━━━┓		┏━━━┓
	;		┃■■■┃ 我们使用 	┃□□□┃ 不能使用的内存
	;		┗━━━┛		┗━━━┛
	;		┏━━━┓		┏━━━┓
	;		┃      ┃ 未使用空间	┃◇◇◇┃ 可以覆盖的内存
	;		┗━━━┛		┗━━━┛
	;
	; 注：KERNEL 的位置实际上是很灵活的，可以通过同时改变 LOAD.INC 中的 KRNL_ENT_PT_PHY_ADDR 和 MAKEFILE 中参数 -Ttext 的值来改变。
	;     比如，如果把 KRNL_ENT_PT_PHY_ADDR 和 -Ttext 的值都改为 0x400400，则 KERNEL 就会被加载到内存 0x400000(4M) 处，入口在 0x400400。
	;
*/


LABEL_GOON_LOADING_FILE:
	push	ax			; ┓
	push	bx			; ┃
	mov	ah, 0Eh			; ┃ 每读一个扇区就在 "Booting  " 后面打一个点, 形成这样的效果:
	mov	al, '.'			; ┃
	mov	bl, 0Fh			; ┃ Booting ......
	int	10h			; ┃
	pop	bx			; ┃
	pop	ax			; ┛

	mov	cl, 1
	call	ReadSector
	pop	ax			; 取出此 Sector 在 FAT 中的序号
	call	GetFATEntry
	cmp	ax, 0FFFh
	jz	LABEL_FILE_LOADED
	push	ax			; 保存 Sector 在 FAT 中的序号
	mov	dx, RootDirSectors
	add	ax, dx
	add	ax, DeltaSectorNo
	add	bx, [BPB_BytsPerSec]
	jmp	LABEL_GOON_LOADING_FILE
LABEL_FILE_LOADED:

	mov	dh, 1			; "Ready."
	call	DispStr			; 显示字符串

; *****************************************************************************************************
	jmp	LOADER_SEG:LOADER_OFF	; 这一句正式跳转到已加载到内存中的 LOADER.BIN 的开始处
						; 开始执行 LOADER.BIN 的代码
						; Boot Sector 的使命到此结束


						LABEL_GOON_LOADING_FILE:
	push	ax			; ┓
	push	bx			; ┃
	mov	ah, 0Eh			; ┃ 每读一个扇区就在 "Loading  " 后面
	mov	al, '.'			; ┃ 打一个点, 形成这样的效果:
	mov	bl, 0Fh			; ┃ Loading ......
	int	10h			; ┃
	pop	bx			; ┃
	pop	ax			; ┛

	mov	cl, 1
	call	ReadSector
	pop	ax			; 取出此 Sector 在 FAT 中的序号
	call	GetFATEntry
	cmp	ax, 0FFFh
	jz	LABEL_FILE_LOADED
	push	ax			; 保存 Sector 在 FAT 中的序号
	mov	dx, RootDirSectors
	add	ax, dx
	add	ax, DeltaSectorNo
	add	bx, [BPB_BytsPerSec]
	jc	.1			; 如果 bx 重新变成 0，说明内核大于 64K
	jmp	.2
.1:
	push	ax			; es += 0x1000  ← es 指向下一个段
	mov	ax, es
	add	ax, 1000h
	mov	es, ax
	pop	ax
.2:
	jmp	LABEL_GOON_LOADING_FILE


	; 下面准备跳入保护模式 -------------------------------------------

; 加载 GDTR
	lgdt	[GdtPtr]

; 关中断
	cli

; 打开地址线A20
	in	al, 92h
	or	al, 00000010b
	out	92h, al

; 准备切换到保护模式
	mov	eax, cr0
	or	eax, 1
	mov	cr0, eax

; 真正进入保护模式
	jmp	dword SelectorFlatC:(LOADER_PHY_ADDR+LABEL_PM_START)


	LABEL_PM_START:
	mov	ax, SelectorVideo
	mov	gs, ax
	mov	ax, SelectorFlatRW
	mov	ds, ax
	mov	es, ax
	mov	fs, ax
	mov	ss, ax
	mov	esp, TopOfStack

	call	DispMemInfo
;;; 	call	DispReturn
;;; 	call	DispHDInfo	; int 13h 读出的硬盘 geometry 好像有点不对头，不知道为什么，干脆不管它了
	call	SetupPaging

	;mov	ah, 0Fh				; 0000: 黑底    1111: 白字
	;mov	al, 'P'
	;mov	[gs:((80 * 0 + 39) * 2)], ax	; 屏幕第 0 行, 第 39 列。

	call	InitKernel

	;jmp	$

	;; fill in BootParam[]
	mov	dword [BOOT_PARAM_ADDR], BOOT_PARAM_MAGIC ; Magic Number
	mov	eax, [dwMemSize]
	mov	[BOOT_PARAM_ADDR + 4], eax ; memory size
	mov	eax, KERNEL_FILE_SEG
	shl	eax, 4
	add	eax, KERNEL_FILE_OFF
	mov	[BOOT_PARAM_ADDR + 8], eax ; phy-addr of kernel.bin

	;***************************************************************
	jmp	SelectorFlatC:KRNL_ENT_PT_PHY_ADDR	; 正式进入内核 *


	SelectorFlatC		equ	LABEL_DESC_FLAT_C	- LABEL_GDT
	jmp	dword SelectorFlatC:(LOADER_PHY_ADDR+LABEL_PM_START)
	jmp	SelectorFlatC:KRNL_ENT_PT_PHY_ADDR	; 正式进入内核 *


	; InitKernel ---------------------------------------------------------------------------------
; 将 KERNEL.BIN 的内容经过整理对齐后放到新的位置
; --------------------------------------------------------------------------------------------
InitKernel:	; 遍历每一个 Program Header，根据 Program Header 中的信息来确定把什么放进内存，放到什么位置，以及放多少。
	xor	esi, esi
	mov	cx, word [KERNEL_FILE_PHY_ADDR + 2Ch]; ┓ ecx <- pELFHdr->e_phnum
	movzx	ecx, cx					; ┛
	mov	esi, [KERNEL_FILE_PHY_ADDR + 1Ch]	; esi <- pELFHdr->e_phoff
	add	esi, KERNEL_FILE_PHY_ADDR		; esi <- OffsetOfKernel + pELFHdr->e_phoff
.Begin:
	mov	eax, [esi + 0]
	cmp	eax, 0				; PT_NULL
	jz	.NoAction
	push	dword [esi + 010h]		; size	┓
	mov	eax, [esi + 04h]		;	┃
	add	eax, KERNEL_FILE_PHY_ADDR	;	┣ ::memcpy(	(void*)(pPHdr->p_vaddr),
	push	eax				; src	┃		uchCode + pPHdr->p_offset,
	push	dword [esi + 08h]		; dst	┃		pPHdr->p_filesz;
	call	MemCpy				;	┃
	add	esp, 12				;	┛
.NoAction:
	add	esi, 020h			; esi += pELFHdr->e_phentsize
	dec	ecx
	jnz	.Begin

	ret
; InitKernel ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


; 启动分页机制 --------------------------------------------------------------
SetupPaging:
	; 根据内存大小计算应初始化多少PDE以及多少页表
	xor	edx, edx
	mov	eax, [dwMemSize]
	mov	ebx, 400000h	; 400000h = 4M = 4096 * 1024, 一个页表对应的内存大小
	div	ebx
	mov	ecx, eax	; 此时 ecx 为页表的个数，也即 PDE 应该的个数
	test	edx, edx
	jz	.no_remainder
	inc	ecx		; 如果余数不为 0 就需增加一个页表
.no_remainder:
	push	ecx		; 暂存页表个数

	; 为简化处理, 所有线性地址对应相等的物理地址. 并且不考虑内存空洞.

	; 首先初始化页目录
	mov	ax, SelectorFlatRW
	mov	es, ax
	mov	edi, PAGE_DIR_BASE	; 此段首地址为 PAGE_DIR_BASE
	xor	eax, eax
	mov	eax, PAGE_TBL_BASE | PG_P  | PG_USU | PG_RWW
.1:
	stosd
	add	eax, 4096		; 为了简化, 所有页表在内存中是连续的.
	loop	.1

	; 再初始化所有页表
	pop	eax			; 页表个数
	mov	ebx, 1024		; 每个页表 1024 个 PTE
	mul	ebx
	mov	ecx, eax		; PTE个数 = 页表个数 * 1024
	mov	edi, PAGE_TBL_BASE	; 此段首地址为 PAGE_TBL_BASE
	xor	eax, eax
	mov	eax, PG_P  | PG_USU | PG_RWW
.2:
	stosd
	add	eax, 4096		; 每一页指向 4K 的空间
	loop	.2

	mov	eax, PAGE_DIR_BASE
	mov	cr3, eax
	mov	eax, cr0
	or	eax, 80000000h
	mov	cr0, eax
	jmp	short .3
.3:
	nop

	ret
; 分页机制启动完毕 ----------------------------------------------------------

