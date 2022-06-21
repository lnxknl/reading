int frosted_init(void)
{
    extern void * _k__syscall__;
    int xip_mounted;
    /* ktimers must be enabled before systick */
    ktimer_init();

    kernel_task_init();

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    fpb_init();
#endif

    vfs_init();
    devnull_init(fno_search("/dev"));

    /* Set up system */


    hw_init();
    mpu_init();

    syscalls_init();

    memfs_init();
    xipfs_init();
    sysfs_init();
    fatfs_init();

    ltdc_init();
    fbcon_init( 480, 272);
    tty_console_init();

    vfs_mount(NULL, "/tmp", "memfs", 0, NULL);
    xip_mounted = vfs_mount((char *)init, "/bin", "xipfs", 0, NULL);
    vfs_mount(NULL, "/sys", "sysfs", 0, NULL);

    klog_init();


#ifdef UNIX
    socket_un_init();
#endif

    return xip_mounted;
}

/* Init function */
void ktimer_init(void)
{
    ktimer_list = heap_init();
}

static inline heap_##type *heap_init(void)                                              \
{                                                                                       \
    heap_##type *p = kcalloc(1, sizeof(heap_##type));                                    \
    return p;                                                                           \
} \

#define kcalloc(x,y) f_calloc(MEM_KERNEL,x,y)

void * f_calloc(int flags, size_t num, size_t size)
{
    void * ptr = f_malloc(flags, num * size);
    if (ptr)
        memset(ptr, 0, num * size);
    return ptr;
}

void * f_malloc(int flags, size_t size)
{
    struct f_malloc_block * blk = NULL, *last = NULL;
    void *ret = NULL;
    while((size % 4) != 0) {
        size++;
    }

    /* kernelspace calls: pid=0 (kernel, kthreads */
    if (this_task_getpid() == 0) {

        /* kernel receives a null (no blocking) */
        if (this_task() == NULL) {
            if (mutex_trylock(mlock) < 0)
                return NULL;
        } else {
            /* ktrheads can block. */
            mutex_lock(mlock);
        }
    }
    /* Userspace calls acquire mlock in the syscall handler. */


    /* Travel the linked list for first fit */
    blk = f_find_best_fit(flags, size, &last);
    if (blk)
    {
        dbg_malloc("Found best fit!\n");
        /*
         * if ((flags & MEM_USER) && blk->next && ((uint8_t *)blk + 24 + blk->size != (uint8_t *)blk->next))
         *      while(1);;
         */
        /* first fit found, now split it if it's much bigger than needed */
        if (2 * (size + sizeof(struct f_malloc_block)) < blk->size)
        {
            dbg_malloc("Splitting blocks, since requested size [%d] << best fit block size [%d]!\n", size, blk->size);
            split_block(blk, size);
        }
    } else {
        /* No first fit found: ask for new memory */
        blk = (struct f_malloc_block *)f_sbrk(flags, size + sizeof(struct f_malloc_block));  // can OS give us more memory?
        if ((long)blk == -1) {
            ret = NULL;
            goto out;
        }

        /* first call -> set entrypoint */
        if (malloc_entry[MEMPOOL(flags)] == NULL) {
            malloc_entry[MEMPOOL(flags)] = blk;
            blk->prev = NULL;
        }
        blk->magic = F_MALLOC_MAGIC;
        blk->size = size;
        blk->next = NULL; /* Newly allocated memory is always last in the linked list */
        /* Link this entry to the previous last entry */
        if (last)
        {
            // assert(last->next == NULL);
            last->next = blk;
        }
        blk->prev = last;
    }

    /* destination found, fill in  meta-data */
    blk->flags = F_IN_USE | flags;
    if (flags & MEM_USER)
        blk->pid = this_task_getpid();
    else
        blk->pid = 0;

    /* update stats */
    f_malloc_stats[MEMPOOL(flags)].malloc_calls++;
    f_malloc_stats[MEMPOOL(flags)].objects_allocated++;
    f_malloc_stats[MEMPOOL(flags)].mem_allocated += ((uint32_t)blk->size + sizeof(struct f_malloc_block));

    ret = (void *)(((uint8_t *)blk) + sizeof(struct f_malloc_block)); // pointer to newly allocated mem

out:
    /* Userspace calls release mlock in the syscall handler. */
    if (this_task_getpid() == 0) {
        mutex_unlock(mlock);
    }
    if (ret && (flags & MEM_TASK)) {
        memset(ret, 0x55, size);
    }
    return ret;
}


void kernel_task_init(void)
{
    /* task0 = kernel */
    irq_off();
    kernel->tb.sp = msp_read(); // SP needs to be current SP
    kernel->tb.pid = next_pid();
    kernel->tb.tid = 1;
    kernel->tb.tgroup = NULL;
    kernel->tb.ppid = scheduler_get_cur_pid();
    kernel->tb.nice = NICE_DEFAULT;
    kernel->tb.start = NULL;
    kernel->tb.arg = NULL;
    kernel->tb.timer_id = -1;
    kernel->tb.filedesc_table = NULL;
    kernel->tb.timeslice = TIMESLICE(kernel);
    kernel->tb.state = TASK_RUNNABLE;
    kernel->tb.cwd = fno_search("/");
    kernel->tb.state = TASK_RUNNABLE;
    kernel->tb.next = NULL;
    tasklist_add(&tasks_running, kernel);
    irq_on();

    /* Set kernel as current task */
    _cur_task = kernel;
}

    static inline void irq_off(void)
    {
        asm volatile ("cpsid i                \n");
    }

    static inline void irq_on(void)
    {
        asm volatile ("cpsie i                \n");
    }

static __inl void *msp_read(void)
{
    void *ret = NULL;
    asm volatile("mrs %0, msp" : "=r"(ret));
    return ret;
}

static int next_pid(void)
{
    static unsigned int next_available = 0;
    uint16_t ret = (uint16_t)((next_available)&0xFFFF);
    next_available++;
    if (next_available > 0xFFFF) {
        next_available = 2;
    }
    while (tasklist_get(&tasks_idling, next_available) ||
           tasklist_get(&tasks_running, next_available))
        next_available++;
    return ret;
}

int fpb_init(void)
{
    if (FPB_CTRL == 0x0) {
        return -1;
    }
    if (FPB_COMP[0] == 0x0) {
        return -1;
    }
    /* Enable Debug Monitor Exception */
    DBG_DEMCR = DBG_DEMCR_MON_EN;
    FPB_CTRL = FPB_CTRL_ENABLE | FPB_CTRL_KEY | (1 << FPB_NUM_CODE2_OFF) | (2 << FPB_NUM_LIT_MASK_OFF);
    nvic_enable_irq(DEBUG_MONITOR_IRQ);
}

void vfs_init(void)
{
    struct fnode *dev = NULL;
    /* Initialize "/" */
    FNO_ROOT.owner = NULL;
    FNO_ROOT.fname = "/";
    FNO_ROOT.parent = &FNO_ROOT;
    FNO_ROOT.children = NULL;
    FNO_ROOT.next = NULL ;
    FNO_ROOT.flags = FL_DIR | FL_RDWR;

    /* Init "/dev" dir */
    fno_mkdir(NULL, "dev", NULL);

    /* Init "/sys" dir */
    fno_mkdir(NULL, "sys", NULL);

    /* Init "/tmp" dir */
    fno_mkdir(NULL, "tmp", NULL);

    /* Init "/bin" dir */
    fno_mkdir(NULL, "bin", NULL);

    /* Init "/mnt" dir */
    fno_mkdir(NULL, "mnt", NULL);
}

void devnull_init(struct fnode *dev)
{
    strcpy(mod_devnull.name,"devnull");
    mod_devnull.family = FAMILY_FILE;
    mod_devnull.ops.open = devnull_open;
    mod_devnull.ops.read = devnull_read;
    mod_devnull.ops.poll = devnull_poll;
    mod_devnull.ops.write = devnull_write;

    devnull = fno_create_wronly(&mod_devnull, "null", dev);
    devnull->flags |= FL_TTY;
    devzero = fno_create_rdonly(&mod_devnull, "zero", dev);
    devzero->flags |= FL_TTY;
    register_module(&mod_devnull);
}

int register_module(struct module *m)
{
    m->next = MODS;
    MODS = m;
    return 0;
}

static void hw_init(void)
{
    gpio_init();
    exti_init();
    uart_init();
    ptmx_init();
    rng_init();
    sdram_init();
    machine_init();
    lowpower_init();
    SysTick_Config(CONFIG_SYS_CLOCK / 1000);
}

void mpu_init(void)
{
    if (!mpu_present())
        return;

    /* User area: prio 0, from start */
    mpu_setaddr(0, 0);              /* Userspace memory block   0x00000000 (1G) - Internal flash is an exception of this */
    mpu_setattr(0, MPUSIZE_1G | MPU_RASR_ENABLE | MPU_RASR_ATTR_SCB | MPU_RASR_ATTR_AP_PRW_URW);

    mpu_setaddr(1, EXTRAM_START);   /* External RAM bank        0x60000000 (512M) */
    mpu_setattr(1, MPUSIZE_512M   | MPU_RASR_ENABLE | MPU_RASR_ATTR_SCB | MPU_RASR_ATTR_AP_PRW_URW);

    /* Read-only sectors */
    mpu_setaddr(2, FLASH_START);    /* Internal Flash           0x00000000 - 0x0FFFFFFF (256M) */
    mpu_setattr(2, MPUSIZE_256M | MPU_RASR_ENABLE | MPU_RASR_ATTR_SCB | MPU_RASR_ATTR_AP_PRO_URO);

    /* System (No user access) */
    mpu_setaddr(3, RAM_START);      /* Kernel memory            0x20000000 (CONFIG_KRAM_SIZE KB) */
    mpu_setattr(3, mpu_size(CONFIG_KRAM_SIZE << 10) | MPU_RASR_ENABLE | MPU_RASR_ATTR_SCB | MPU_RASR_ATTR_AP_PRW_UNO);

    /* Priority 4 reserved for task stack exception in kernel memory */

    mpu_setaddr(5, DEV_START);      /* Peripherals              0x40000000 (512MB)*/
    mpu_setattr(5, MPUSIZE_1G | MPU_RASR_ENABLE | MPU_RASR_ATTR_S | MPU_RASR_ATTR_B | MPU_RASR_ATTR_AP_PRW_UNO);
    mpu_setaddr(6, EXTDEV_START);   /* External Peripherals     0xA0000000 (1GB)   */
    mpu_setattr(6, MPUSIZE_1G | MPU_RASR_ENABLE | MPU_RASR_ATTR_S | MPU_RASR_ATTR_B | MPU_RASR_ATTR_AP_PRW_UNO);
    mpu_setaddr(7, REG_START);      /* System Level             0xE0000000 (256MB) */
    mpu_setattr(7, MPUSIZE_256M | MPU_RASR_ENABLE | MPU_RASR_ATTR_S | MPU_RASR_ATTR_B | MPU_RASR_ATTR_AP_PRW_UNO);


#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    SCB_SHCSR |= SCB_SHCSR_MEMFAULTENA;
#endif
    mpu_enable();
}


void memfs_init(void)
{
    mod_memfs.family = FAMILY_FILE;
    strcpy(mod_memfs.name,"memfs");

    mod_memfs.mount = memfs_mount;

    mod_memfs.ops.read = memfs_read;
    mod_memfs.ops.poll = memfs_poll;
    mod_memfs.ops.write = memfs_write;
    mod_memfs.ops.seek = memfs_seek;
    mod_memfs.ops.creat = memfs_creat;
    mod_memfs.ops.unlink = memfs_unlink;
    mod_memfs.ops.close = memfs_close;
    mod_memfs.ops.truncate = memfs_truncate;
    register_module(&mod_memfs);
}

void xipfs_init(void)
{
    mod_xipfs.family = FAMILY_FILE;
    mod_xipfs.mount = xipfs_mount;
    strcpy(mod_xipfs.name,"xipfs");
    mod_xipfs.ops.read = xipfs_read;
    mod_xipfs.ops.poll = xipfs_poll;
    mod_xipfs.ops.write = xipfs_write;
    mod_xipfs.ops.seek = xipfs_seek;
    mod_xipfs.ops.creat = xipfs_creat;
    mod_xipfs.ops.unlink = xipfs_unlink;
    mod_xipfs.ops.close = xipfs_close;
    mod_xipfs.ops.exe = xipfs_exe;

    mod_xipfs.ops.block_read = xipfs_block_read;
    register_module(&mod_xipfs);
}

void sysfs_init(void)
{
    mod_sysfs.family = FAMILY_FILE;
    strcpy(mod_sysfs.name, "sysfs");

    mod_sysfs.mount = sysfs_mount;

    mod_sysfs.ops.read = sysfs_read;
    mod_sysfs.ops.poll = sysfs_poll;
    mod_sysfs.ops.write = sysfs_write;
    mod_sysfs.ops.close = sysfs_close;

    sysfs = fno_search("/sys");
    fno_mkdir(&mod_sysfs, "net", sysfs);
    fno_mkdir(&mod_sysfs, "power", sysfs);
    register_module(&mod_sysfs);
    sysfs_mutex = mutex_init();
}

int fatfs_init(void)
{
    mod_fatfs.family = FAMILY_FILE;
    strcpy(mod_fatfs.name,"fatfs");

    mod_fatfs.mount = fatfs_mount;
    mod_fatfs.ops.open = fatfs_open;
    mod_fatfs.ops.creat = fatfs_creat;
    mod_fatfs.ops.read = fatfs_read;
    mod_fatfs.ops.write = fatfs_write;
    mod_fatfs.ops.seek = fatfs_seek;
    mod_fatfs.ops.truncate = fatfs_truncate;
    mod_fatfs.ops.close = fatfs_close;
    mod_fatfs.ops.unlink = fatfs_unlink;

    register_module(&mod_fatfs);
    return 0;
}

void ltdc_init(void)
{
    lcd_pinmux();
    ltdc_clock();
    ltdc_config(); /* Configure LCD : Only one layer is used */
    ltdc_config_layer(&ltdc_info);
    ili9341_init();
    register_framebuffer(&ltdc_info);
    register_module(&mod_ltdc);
}

int fbcon_init(uint32_t cols, uint32_t rows)
{
    struct fnode *devfs = fno_search("/dev");
    struct dev_fbcon *fbcon = kalloc(sizeof(struct dev_fbcon));
    struct fnode *fno_fbcon;

    if (!fbcon)
        return -1;

    memset(fbcon, 0, sizeof(struct dev_fbcon));
    screen_cols = cols;
    screen_rows = rows;

    fbcon_l = screen_cols / FONT_WIDTH;
    fbcon_h = screen_rows / FONT_HEIGHT;

    fbcon->buffer = u_malloc(fbcon_l * fbcon_h);
    if (!fbcon->buffer) {
        kfree(fbcon);
        return -1;
    }
    fbcon->colormap = u_malloc(fbcon_l * fbcon_h);
    if (!fbcon->colormap) {
        kfree(fbcon->buffer);
        kfree(fbcon);
        return -1;
    }

    if (devfs == NULL) {
        kfree(fbcon->colormap);
        kfree(fbcon->buffer);
        kfree(fbcon);
        return -1;
    }

    memset(fbcon->buffer, 0, (fbcon_l * fbcon_h));
    memset(fbcon->colormap, COLOR_DEFAULT, (fbcon_l * fbcon_h));
    register_module(&mod_devfbcon);
    fno_fbcon = fno_create(&mod_devfbcon, "fbcon", devfs);
    fno_fbcon->priv = fbcon;
    fbcon->size_x = fbcon_l;
    fbcon->size_y = fbcon_h;
    fbcon->screen = framebuffer_get();
    fbcon->color = COLOR_DEFAULT;
    framebuffer_setcmap(xterm_cmap);
    /* Test */
    devfbcon_write(fno_fbcon, color, 2);
    devfbcon_write(fno_fbcon, frosted_banner, strlen(frosted_banner));
    devfbcon_write(fno_fbcon, white, 2);
    devfbcon_write(fno_fbcon, fbcon_banner, strlen(fbcon_banner));
    return 0;
}

int tty_console_init(void)
{
    TTY.mod_kbd = module_search(KBD_MOD);
    TTY.mod_fbcon = module_search(FBCON_MOD);
    if(!TTY.mod_kbd || !TTY.mod_fbcon)
        return -1;
    TTY.kbd = fno_search(KBD_PATH);
    TTY.fbcon = fno_search(FBCON_PATH);
    devfile_create();
    return 0;
}

int vfs_mount(char *source, char *target, char *module, uint32_t flags, void *args)
{
    struct module *m;
    if (!module || !target)
        return -ENOMEDIUM;
    m = module_search(module);
    if (!m || !m->mount)
        return -EOPNOTSUPP;
    if (m->mount(source, target, flags, args) == 0) {
        struct mountpoint *mp = kalloc(sizeof(struct mountpoint));
        if (mp) {
            mp->target = fno_search(target);
            mp->next = MTAB;
            MTAB = mp;
        }
        return 0;
    }
    return -ENOENT;
}

int klog_init(void)
{
    klog.fno = fno_create_rdonly(&mod_klog, "klog", fno_search("/dev"));
    if (klog.fno == NULL) {
        return -1;
    }
    klog.buf = cirbuf_create(CONFIG_KLOG_SIZE);
	klog.used = 0;
    klog.task = NULL;
    klog_lock = mutex_init();
    return 0;
}

void socket_un_init(void)
{
    mod_socket_un.family = FAMILY_UNIX;
    strcpy(mod_socket_un.name,"un");
    mod_socket_un.ops.poll = sock_poll;
    mod_socket_un.ops.close = sock_close;

    mod_socket_un.ops.socket     = sock_socket;
    mod_socket_un.ops.connect    = sock_connect;
    mod_socket_un.ops.accept     = sock_accept;
    mod_socket_un.ops.bind       = sock_bind;
    mod_socket_un.ops.listen     = sock_listen;
    mod_socket_un.ops.recvfrom   = sock_recvfrom;
    mod_socket_un.ops.sendto     = sock_sendto;
    mod_socket_un.ops.shutdown   = sock_shutdown;

    register_module(&mod_socket_un);
    register_addr_family(&mod_socket_un, FAMILY_UNIX);
}

void frosted_kernel(int xipfs_mounted)
{
    struct vfs_info *vfsi = NULL;

    if (xipfs_mounted == 0)
    {
        struct fnode *fno = fno_search(init_path);
        void * memptr;
        size_t mem_size;
        size_t stack_size;
        uint32_t got_loc;
        if (!fno) {
            /* PANIC: Unable to find /bin/init */
            while(1 < 2);
        }

        if (fno->owner && fno->owner->ops.exe) {
            vfsi = fno->owner->ops.exe(fno, (void *)init_args);
            task_create(vfsi, (void *)init_args, NICE_DEFAULT);
        }
    } else {
        IDLE();
    }


#ifdef CONFIG_PICOTCP
    pico_stack_init();
    socket_in_init();
    pico_lock_init();

    /* Network devices initialization */
    usb_ethernet_init(USB_DEV_FS);
    pico_loop_create();
    pico_eth_start();
#endif

    frosted_scheduler_on();

    while(1) {
        check_tasklets();
#ifdef CONFIG_PICOTCP
        if (pico_trylock_kernel() == 0) {
            pico_stack_tick();
            pico_unlock();
        }
#endif
        asm volatile ("wfe");
        //__WFI();
    }
}

struct vfs_info {
    int type;
    int pic;
    void (*init)(void *);
    void * allocated;
    uint32_t text_size;
    uint32_t data_size;
};

struct fnode {
    struct module *owner;
    char *fname;
    char *linkname;
    uint32_t flags;
    struct fnode *parent;
    struct fnode *children;
    void *priv;
    uint32_t dir_ptr;
    uint32_t size;
    int32_t usage_count;
    struct fnode *next;
};

int task_create(struct vfs_info *vfsi, void *arg, unsigned int nice)
{
    struct task *new;
    int i;
    struct filedesc_table *ft;

    new = task_space_alloc(sizeof(struct task));
    if (!new) {
        return -ENOMEM;
    }
    memset(&new->tb, 0, sizeof(struct task_block));
    new->tb.pid = next_pid();
    new->tb.tid = 1;
    new->tb.tgroup = NULL;

    new->tb.ppid = scheduler_get_cur_pid();
    new->tb.nice = nice;
    new->tb.filedesc_table = NULL;
    new->tb.flags = 0;
    new->tb.cwd = fno_search("/");
    new->tb.vfsi = vfsi;
    new->tb.tracer = NULL;

    ft = _cur_task->tb.filedesc_table;

    /* Inherit cwd, file descriptors from parent */
    if (new->tb.ppid > 1) { /* Start from parent #2 */
        new->tb.cwd = task_getcwd();
        for (i = 0; (ft) && (i < ft->n_files); i++) {
            task_filedesc_add_to_task(new, ft->fdesc[i].fno);
            new->tb.filedesc_table->fdesc[i].mask = ft->fdesc[i].mask;
        }
    }

    new->tb.next = NULL;
    tasklist_add(&tasks_running, new);

    number_of_tasks++;
    task_create_real(new, vfsi, arg, nice);
    new->tb.state = TASK_RUNNABLE;
    return new->tb.pid;
}

#define IDLE() while(1){do{}while(0);}

void frosted_scheduler_on(void)
{
    nvic_set_priority(NVIC_PENDSV_IRQ, 2);
    nvic_set_priority(NVIC_SV_CALL_IRQ, 1);
#ifdef CUSTOM_SYSTICK
    ostick_init(1, &sys_tick_handler);
    ostick_start();
#else
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0);
    nvic_enable_irq(NVIC_SYSTICK_IRQ);
    systick_counter_enable();
    systick_interrupt_enable();
#endif
    _sched_active = 1;
}

struct tasklet {
    void (*exe)(void *);
    void *arg;
};

