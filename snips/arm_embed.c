
void echo_hello(void)
{
    rt_kprintf("hello world!\n");
}
MSH_CMD_EXPORT(echo_hello,echo hello...);

struct acm32_spi_config
{
    SPI_TypeDef         *Instance;
    char                *bus_name;
    IRQn_Type           irq_type;
    enum_Enable_ID_t    enable_id;
#if defined(BSP_SPI1_RX_USING_DMA) || defined(BSP_SPI2_RX_USING_DMA)
    struct dma_config   *dma_rx;
#endif
#if defined(BSP_SPI1_TX_USING_DMA) || defined(BSP_SPI2_TX_USING_DMA)
    struct dma_config   *dma_tx;
#endif

    enum_GPIOx_t        cs_port;
    rt_uint32_t         cs_pin;
    rt_uint32_t         cs_alternate;

    enum_GPIOx_t        sck_port;
    rt_uint32_t         sck_pin;
    rt_uint32_t         sck_alternate;

    enum_GPIOx_t        mosi_port;
    rt_uint32_t         mosi_pin;
    rt_uint32_t         mosi_alternate;

    enum_GPIOx_t        miso_port;
    rt_uint32_t         miso_pin;
    rt_uint32_t         miso_alternate;

    enum_GPIOx_t        wp_port;
    rt_uint32_t         wp_pin;
    rt_uint32_t         wp_alternate;

    enum_GPIOx_t        hold_port;
    rt_uint32_t         hold_pin;
    rt_uint32_t         hold_alternate;
};


struct acm32_spi
{
    SPI_HandleTypeDef           handle;
    struct acm32_spi_config     *config;
    struct rt_spi_configuration *cfg;

#if defined(BSP_SPI1_TX_USING_DMA) || defined(BSP_SPI1_RX_USING_DMA) || defined(BSP_SPI2_TX_USING_DMA) || defined(BSP_SPI2_RX_USING_DMA)
    struct
    {
#if defined(BSP_SPI1_RX_USING_DMA) || defined(BSP_SPI2_RX_USING_DMA)
        DMA_HandleTypeDef       handle_rx;
#endif
#if defined(BSP_SPI1_TX_USING_DMA) || defined(BSP_SPI2_TX_USING_DMA)
        DMA_HandleTypeDef       handle_tx;
#endif
    } dma;

    rt_uint8_t                  spi_dma_flag;
#endif

    struct rt_spi_bus           spi_bus;
};


#define MBOX_ADDR 0xc00000

struct lwip_mbox {
  void* sem;
  void** q_mem;
  unsigned int head, tail;
  int size;
  int used;
};


        #define INIT_EXPORT(fn, level)                                  \
                                const char __rti_level_##fn[] = ".rti_fn." level;       \
                                const char __rti_##fn##_name[] = #fn;                   \
                                __declspec(allocate("rti_fn$f"))                        \
                                RT_USED const struct rt_init_desc __rt_init_msc_##fn =  \
                                {__rti_level_##fn, fn, __rti_##fn##_name};


struct mem_desc {
    rt_uint32_t vaddr_start;
    rt_uint32_t vaddr_end;
    rt_uint32_t paddr_start;
    rt_uint32_t sect_attr;   /* when page mapped */
    rt_uint32_t page_attr;   /* only sector mapped valid */
    rt_uint32_t mapped_mode;
#define     SECT_MAPPED  0
#define     PAGE_MAPPED  1
};

struct mem_desc
{
    rt_uint32_t vaddr_start;
    rt_uint32_t vaddr_end;
    rt_uint32_t paddr_start;
    rt_uint32_t attr;
};


#define DEVICE_MEM     (SHARED | SHAREDEVICE | RW_NCNBXN)


#define NORMAL_MEM     (SHARED | AP_RW | DOMAIN0 | MEMWBWA | DESC_SEC)


#define RT_SCHEDULE_IPI                 0

void rt_scheduler_ipi_handler(int vector, void *param)
{
    rt_schedule();
}

void rt_hw_ipi_handler_install(int ipi_vector, rt_isr_handler_t ipi_isr_handler)
{
    /* note: ipi_vector maybe different with irq_vector */
    rt_hw_interrupt_install(ipi_vector, ipi_isr_handler, 0, "IPI_HANDLER");
}


rt_hw_interrupt_disable:
    MRS     R0, CPSR
    ORR     R1, R0, #NOINT
    MSR     CPSR_c, R1
    BX      LR

/*
 * void rt_hw_interrupt_enable(rt_base_t level);
 */

rt_hw_cpu_id:
    mrc p15, 0, r0, c0, c0, 5
    ubfx r0, r0, #0, #12
    cmp r0, #0
    beq core0
    cmp r0, #1
    beq core1
    cmp r0, #256
    beq core2
    ldr r1,= #257
    cmp r0, r1
    beq core3
    b default


core0:
    mov r0, #0
    b return
core1:
    mov r0, #1
    b return
core2:
    mov r0, #2
    b return


struct rt_cpu *rt_cpu_index(int index)
{
    return &_cpus[index];
}

static struct rt_cpu _cpus[RT_CPUS_NR];


struct rt_cpu
{
    struct rt_thread *current_thread;

    rt_uint16_t irq_nest;
    rt_uint8_t  irq_switch_flag;

    rt_uint8_t current_priority;
    rt_list_t priority_table[RT_THREAD_PRIORITY_MAX];
#if RT_THREAD_PRIORITY_MAX > 32
    rt_uint32_t priority_group;
    rt_uint8_t ready_table[32];
#else
    rt_uint32_t priority_group;
#endif

    rt_tick_t tick;
};

typedef long                            rt_base_t;      /**< Nbit CPU related date type */
struct rt_cpu *rt_cpu_self(void)
{
    return &_cpus[rt_hw_cpu_id()];
}

typedef rt_err_t (*virtio_device_init_handler)(rt_ubase_t *mmio_base, rt_uint32_t irq);
/**
 * @brief This function will handle IPI interrupt and do a scheduling in system.
 *
 * @param vector is the number of IPI interrupt for system scheduling.
 *
 * @param param is not used, and can be set to RT_NULL.
 *
 * @note this function should be invoke or register as ISR in BSP.
 */
void rt_schedule(void)


#ifdef RT_USING_SMP


/**
 * @brief This function will perform scheduling once. It will select one thread
 *        with the highest priority, and switch to it immediately.
 */

struct rt_thread
{
    /* rt object */
    char        name[RT_NAME_MAX];                      /**< the name of thread */
    rt_uint8_t  type;                                   /**< type of object */
    rt_uint8_t  flags;                                  /**< thread's flags */

#ifdef RT_USING_MODULE
    void       *module_id;                              /**< id of application module */
#endif

    rt_list_t   list;                                   /**< the object list */
    rt_list_t   tlist;                                  /**< the thread list */

    /* stack point and entry */
    void       *sp;                                     /**< stack point */
    void       *entry;                                  /**< entry */
    void       *parameter;                              /**< parameter */
    void       *stack_addr;                             /**< stack address */
    rt_uint32_t stack_size;                             /**< stack size */

    /* error code */
    rt_err_t    error;                                  /**< error code */

    rt_uint8_t  stat;                                   /**< thread status */

#ifdef RT_USING_SMP
    rt_uint8_t  bind_cpu;                               /**< thread is bind to cpu */
    rt_uint8_t  oncpu;                                  /**< process on cpu */

    rt_uint16_t scheduler_lock_nest;                    /**< scheduler lock count */
    rt_uint16_t cpus_lock_nest;                         /**< cpus lock count */
    rt_uint16_t critical_lock_nest;                     /**< critical lock count */
#endif /*RT_USING_SMP*/

    /* priority */
    rt_uint8_t  current_priority;                       /**< current priority */
#if RT_THREAD_PRIORITY_MAX > 32
    rt_uint8_t  number;
    rt_uint8_t  high_mask;
#endif
    rt_uint32_t number_mask;

#if defined(RT_USING_EVENT)
    /* thread event */
    rt_uint32_t event_set;
    rt_uint8_t  event_info;
#endif

#if defined(RT_USING_SIGNALS)
    rt_sigset_t     sig_pending;                        /**< the pending signals */
    rt_sigset_t     sig_mask;                           /**< the mask bits of signal */

#ifndef RT_USING_SMP
    void            *sig_ret;                           /**< the return stack pointer from signal */
#endif
    rt_sighandler_t *sig_vectors;                       /**< vectors of signal handler */
    void            *si_list;                           /**< the signal infor list */
#endif

    rt_ubase_t  init_tick;                              /**< thread's initialized tick */
    rt_ubase_t  remaining_tick;                         /**< remaining tick */

#ifdef RT_USING_CPU_USAGE
    rt_uint64_t  duration_tick;                          /**< cpu usage tick */
#endif

    struct rt_timer thread_timer;                       /**< built-in thread timer */

    void (*cleanup)(struct rt_thread *tid);             /**< cleanup function when thread exit */

    /* light weight process if present */
#ifdef RT_USING_LWP
    void        *lwp;
#endif

    rt_ubase_t user_data;                             /**< private user data beyond this thread */
};


/**
 * @brief This function checks whether a scheduling is needed after an IRQ context switching. If yes,
 *        it will select one thread with the highest priority level, and then switch
 *        to it.
 */
void rt_scheduler_do_irq_switch(void *context)
{
    int cpu_id;
    rt_base_t level;
    struct rt_cpu* pcpu;
    struct rt_thread *to_thread;
    struct rt_thread *current_thread;

    level = rt_hw_interrupt_disable();

    cpu_id = rt_hw_cpu_id();
    pcpu   = rt_cpu_index(cpu_id);
    current_thread = pcpu->current_thread;

#ifdef RT_USING_SIGNALS
    if ((current_thread->stat & RT_THREAD_STAT_MASK) == RT_THREAD_SUSPEND)
    {
        /* if current_thread signal is in pending */

        if ((current_thread->stat & RT_THREAD_STAT_SIGNAL_MASK) & RT_THREAD_STAT_SIGNAL_PENDING)
        {
            rt_thread_resume(current_thread);
        }
    }
#endif /* RT_USING_SIGNALS */

    if (pcpu->irq_switch_flag == 0)
    {
        rt_hw_interrupt_enable(level);
        return;
    }

    if (current_thread->scheduler_lock_nest == 1 && pcpu->irq_nest == 0)
    {
        rt_ubase_t highest_ready_priority;

        /* clear irq switch flag */
        pcpu->irq_switch_flag = 0;

        if (rt_thread_ready_priority_group != 0 || pcpu->priority_group != 0)
        {
            to_thread = _scheduler_get_highest_priority_thread(&highest_ready_priority);
            current_thread->oncpu = RT_CPU_DETACHED;
            if ((current_thread->stat & RT_THREAD_STAT_MASK) == RT_THREAD_RUNNING)
            {
                if (current_thread->current_priority < highest_ready_priority)
                {
                    to_thread = current_thread;
                }
                else if (current_thread->current_priority == highest_ready_priority && (current_thread->stat & RT_THREAD_STAT_YIELD_MASK) == 0)
                {
                    to_thread = current_thread;
                }
                else
                {
                    rt_schedule_insert_thread(current_thread);
                }
                current_thread->stat &= ~RT_THREAD_STAT_YIELD_MASK;
            }
            to_thread->oncpu = cpu_id;
            if (to_thread != current_thread)
            {
                /* if the destination thread is not the same as current thread */

                pcpu->current_priority = (rt_uint8_t)highest_ready_priority;

                RT_OBJECT_HOOK_CALL(rt_scheduler_hook, (current_thread, to_thread));

                rt_schedule_remove_thread(to_thread);
                to_thread->stat = RT_THREAD_RUNNING | (to_thread->stat & ~RT_THREAD_STAT_MASK);

#ifdef RT_USING_OVERFLOW_CHECK
                _rt_scheduler_stack_check(to_thread);
#endif /* RT_USING_OVERFLOW_CHECK */
                RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("switch in interrupt\n"));

                current_thread->cpus_lock_nest--;
                current_thread->scheduler_lock_nest--;

                RT_OBJECT_HOOK_CALL(rt_scheduler_switch_hook, (current_thread));

                rt_hw_context_switch_interrupt(context, (rt_ubase_t)&current_thread->sp,
                        (rt_ubase_t)&to_thread->sp, to_thread);
            }
        }
    }
    rt_hw_interrupt_enable(level);
}

/**
 * @brief This function will insert a thread to the system ready queue. The state of
 *        thread will be set as READY and the thread will be removed from suspend queue.
 *
 * @param thread is the thread to be inserted.
 *
 * @note  Please do not invoke this function in user application.
 */

void rt_schedule_insert_thread(struct rt_thread *thread)
{
    int cpu_id;
    int bind_cpu;
    rt_uint32_t cpu_mask;
    register rt_base_t level;

    RT_ASSERT(thread != RT_NULL);

    /* disable interrupt */
    level = rt_hw_interrupt_disable();

    /* it should be RUNNING thread */
    if (thread->oncpu != RT_CPU_DETACHED)
    {
        thread->stat = RT_THREAD_RUNNING | (thread->stat & ~RT_THREAD_STAT_MASK);
        goto __exit;
    }

    /* READY thread, insert to ready queue */
    thread->stat = RT_THREAD_READY | (thread->stat & ~RT_THREAD_STAT_MASK);

    cpu_id   = rt_hw_cpu_id();
    bind_cpu = thread->bind_cpu ;

    /* insert thread to ready list */
    if (bind_cpu == RT_CPUS_NR)
    {
#if RT_THREAD_PRIORITY_MAX > 32
        rt_thread_ready_table[thread->number] |= thread->high_mask;
#endif /* RT_THREAD_PRIORITY_MAX > 32 */
        rt_thread_ready_priority_group |= thread->number_mask;

        rt_list_insert_before(&(rt_thread_priority_table[thread->current_priority]),
                              &(thread->tlist));
        cpu_mask = RT_CPU_MASK ^ (1 << cpu_id);
        rt_hw_ipi_send(RT_SCHEDULE_IPI, cpu_mask);
    }
    else
    {
        struct rt_cpu *pcpu = rt_cpu_index(bind_cpu);

#if RT_THREAD_PRIORITY_MAX > 32
        pcpu->ready_table[thread->number] |= thread->high_mask;
#endif /* RT_THREAD_PRIORITY_MAX > 32 */
        pcpu->priority_group |= thread->number_mask;

        rt_list_insert_before(&(rt_cpu_index(bind_cpu)->priority_table[thread->current_priority]),
                              &(thread->tlist));

        if (cpu_id != bind_cpu)
        {
            cpu_mask = 1 << bind_cpu;
            rt_hw_ipi_send(RT_SCHEDULE_IPI, cpu_mask);
        }
    }

    RT_DEBUG_LOG(RT_DEBUG_SCHEDULER, ("insert thread[%.*s], the priority: %d\n",
                                      RT_NAME_MAX, thread->name, thread->current_priority));

__exit:
    /* enable interrupt */
    rt_hw_interrupt_enable(level);
}

/**
 * @brief This function will unlock the thread scheduler.
 */
#ifdef RT_USING_SMP
void rt_exit_critical(void)
{
    register rt_base_t level;
    struct rt_thread *current_thread;

    /* disable interrupt */
    level = rt_hw_local_irq_disable();

    current_thread = rt_cpu_self()->current_thread;
    if (!current_thread)
    {
        rt_hw_local_irq_enable(level);
        return;
    }

    current_thread->scheduler_lock_nest --;

    current_thread->critical_lock_nest --;

    current_thread->cpus_lock_nest--;
    if (current_thread->cpus_lock_nest == 0)
    {
        current_thread->scheduler_lock_nest --;
        rt_hw_spin_unlock(&_cpus_lock);
    }

    if (current_thread->scheduler_lock_nest <= 0)
    {
        current_thread->scheduler_lock_nest = 0;
        /* enable interrupt */
        rt_hw_local_irq_enable(level);

        rt_schedule();
    }
    else
    {
        /* enable interrupt */
        rt_hw_local_irq_enable(level);
    }
}


/**
 * This function will initial board.
 */
void rt_hw_board_init()
{
    //rt_thread_idle_sethook(idle_hook);

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    SysTick_Config(SYSTEM_CLOCK_FREQ / RT_TICK_PER_SECOND);

    rt_components_board_init();
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
}



/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

RTM_EXPORT(rt_show_version);



		.org 0x200
		.global Reset_Handler,hard_fault_handler,svc_handler,pendsv_handler,systick,irq0,irq1,irq2,irq3,irq4,irq5,irq6,irq7,irq8,irq9,irq10,irq11,irq12,irq13,irq14,irq15,irq16,irq17,irq18,irq19,irq20,irq21,irq22,irq23,irq24,irq25,irq26,irq27,irq28,irq29,irq30,irq31


		.long


Reset_Handler:
		ldr r0,=hardware_init
		bx r0
		.thumb_func

hard_fault_handler:
		ldr r0,=HARD_FAULT_IRQHandler
		bx r0
		nop
		.thumb_func

svc_handler:
		ldr r0,=SVC_IRQHandler
		bx r0
		nop
		.thumb_func

pendsv_handler:
		ldr r0,=PENDSV_IRQHandler
		bx r0
		nop
		.thumb_func
systick:
		ldr r0,=SYSTICK_IRQHandler
		bx r0
		nop
		.thumb_func
irq0:
		mov r0,#4*0
		b isr
		.thumb_func


//====

hardware_init:
		ldr	r1, =__exidx_start
		ldr	r2, =__data_start__
		ldr	r3, =__data_end__

		sub	r3, r2
		ble	.L_loop1_done

	.L_loop1:
		sub	r3, #4
		ldr	r0, [r1,r3]
		str	r0, [r2,r3]
		bgt	.L_loop1

	.L_loop1_done:



	/*  Single BSS section scheme.
	 *
	 *  The BSS section is specified by following symbols
	 *    _sbss: start of the BSS section.
	 *    _ebss: end of the BSS section.
	 *
	 *  Both addresses must be aligned to 4 bytes boundary.
	 */
		ldr	r1, =__bss_start__
		ldr	r2, =__bss_end__

		mov	r0, #0

		sub	r2, r1
		ble	.L_loop3_done

	.L_loop3:
		sub	r2, #4
		str	r0, [r1, r2]
		bgt	.L_loop3
	.L_loop3_done:
		ldr	r0,=0x12345
		ldr	r3,=0x1111
		bl	main


		.globl delay
		.syntax unified


		.globl delay
		.syntax unified
delay:
		subs r0,#1
		bne delay
		nop
		bx lr


//======= orange os ======

;;; .hderr:
;;; 	mov	dword [_dwNrHead], 0FFFFh
;;; .hdok:
	;; 将硬盘引导扇区内容读入内存 0500h 处
	xor     ax, ax
	mov     es, ax
	mov     ax, 0201h       ; AH = 02
	                        ; AL = number of sectors to read (must be nonzero)
	mov     cx, 1           ; CH = low eight bits of cylinder number
	                        ; CL = sector number 1-63 (bits 0-5)
	                        ;      high two bits of cylinder (bits 6-7, hard disk only)
	mov     dx, 80h         ; DH = head number
	                        ; DL = drive number (bit 7 set for hard disk)
	mov     bx, 500h        ; ES:BX -> data buffer
	int     13h
	;; 硬盘操作完毕

	mov	dh, 2			; "Ready."
	call	DispStrRealMode		; 显示字符串


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

