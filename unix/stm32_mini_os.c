
#define FLASH_START       (0x00000000)
#define RAM_START         (0x20000000)
#define DEV_START         (0x40000000)
#define EXTRAM_START      (0xC0000000)
#define EXTFLASH_START    (0x80000000)
#define EXTDEV_START      (0xA0000000)
#define REG_START         (0xE0000000)


    /* Read-only sectors */
    mpu_setaddr(2, FLASH_START);    /* Internal Flash           0x00000000 - 0x0FFFFFFF (256M) */

static void mpu_setaddr(int region, uint32_t addr)
{
    mpu_select(region);
    MPU_RBAR = addr;
}

static void mpu_select(uint32_t region)
{
    MPU_RNR = region;
}

_mutex_lock:
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
   LDREX   r1, [r0]
#elif defined(__ARCH_V6M__)
   CPSID   i
   LDR     r1, [r0]
#endif
   CMP     r1, #0             // Test if mutex holds the value 0
   BEQ     _mutex_lock_fail   // If it does, return 0
   SUBS    r1, #1             // If not, decrement temporary copy
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
   STREX   r2, r1, [r0]       // Attempt Store-Exclusive
   CMP     r2, #0             // Check if Store-Exclusive succeeded
   BNE     _mutex_lock        // If Store-Exclusive failed, retry from start
   DMB                        // Required before accessing protected resource
#elif defined(__ARM_ARCH_6M__)
   STR     r1, [r0]
   CPSIE   i
#endif
   MOVS    r0, #0             // Successfully locked.
   BX      lr


   extern int _mutex_lock(void *);



   struct semaphore {
    int value;
    uint32_t signature;
    int listeners;
    int last;
    struct task **listener;
};


struct __attribute__((packed)) task {
    struct task_block tb;
    uint32_t stack[SCHEDULER_STACK_SIZE / 4];
};


struct __attribute__((packed)) task_block {
    /* Watch out for alignment here
     * (try to pack togehter smaller fields)
     * */
    void (*start)(void *);
    void *arg;

    uint16_t flags;
    uint8_t state;
    int8_t nice;

    uint16_t timeslice;
    uint16_t pid;

    uint16_t ppid;
    uint16_t tid;
    uint16_t joiner_thread_tid;
    uint16_t _padding;
    struct thread_group *tgroup;
    struct task *tracer;
    int exitval;
    struct fnode *cwd;
    struct task_handler *sighdlr;
    sigset_t sigmask;
    sigset_t sigpend;
    struct filedesc_table *filedesc_table;
    void *sp;
    void *osp;
    void *cur_stack;
    struct task *next;
    struct vfs_info *vfsi;
    int timer_id;
    uint32_t *specifics;
    uint32_t n_specifics;
};

struct __attribute__((packed)) task {
    struct task_block tb;
    uint32_t stack[SCHEDULER_STACK_SIZE / 4];
};


int suspend_on_sem_wait(sem_t *s)
{
    int ret;
    if (!s)
        return -EINVAL;
    ret = _sem_wait(s);
    if (ret != 0) {
        _add_listener(s);
        return EAGAIN;
    }
    return 0;
}


int suspend_on_mutex_lock(mutex_t *s)
{
    int ret;
    if (!s)
        return -EINVAL;
    ret = _mutex_lock(s);
    if (ret != 0) {
        _add_listener(s);
        return EAGAIN;
    }
    return 0;
}


struct task *this_task(void)
{
    /* External modules like locks.c expect this to
     * return NULL when in kernel
     */
    if (in_kernel())
        return NULL;
    return _cur_task;
}


static __inl int in_kernel(void)
{
    return ((_cur_task->tb.pid == 0) && (_cur_task->tb.tid <= 1));
}


void task_set_timer_id(int id)
{
    _cur_task->tb.timer_id = id;
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


struct f_malloc_block {
    uint32_t magic;                 /* magic fingerprint */
    struct f_malloc_block * prev;   /* previous block */
    struct f_malloc_block * next;   /* next, or last block? */
    size_t size;                    /* malloc size excluding this block - next block is adjacent, if !last_block */
    uint32_t flags;
    int pid;
};


    while((size % 4) != 0) {
        size++;
    }


uint16_t this_task_getpid(void)
{
    return scheduler_get_cur_pid();
}


static uint16_t scheduler_get_cur_pid(void)
{
    if (!_cur_task)
        return 0;
    return _cur_task->tb.pid;
}


int mutex_lock(mutex_t *s)
{
    if (this_task() == NULL)
        return mutex_spinlock(s);
    if (!s)
        return -EINVAL;
    if(_mutex_lock(s) != 0) {
        _add_listener(s);
        task_suspend();
        return SYS_CALL_AGAIN;
    }
    _del_listener(s);
    return 0;
}


static struct f_malloc_block * f_find_best_fit(int flags, size_t size, struct f_malloc_block ** last)
{
    struct f_malloc_block *found = NULL, *blk = malloc_entry[MEMPOOL(flags)];

    /* See if we can find a free block that fits */
    while (blk) /* last entry will break the loop */
    {
        *last = blk; /* last travelled node */

        if (block_fits(blk, size, flags))
        {
            /* found a fit - is it better than a possible previously found block? */
            if ((!found) || (blk->size < found->size))
                found = blk;
        }
        if ((blk->next) && (!block_valid(blk->next)))
            blk->next = NULL;

        /* travel next block */
        blk = blk->next;
    }
    return found;
}


static struct f_malloc_block *malloc_entry[5] = {NULL, NULL, NULL, NULL, NULL};

static inline int MEMPOOL(int x)
{
#ifdef CONFIG_TCPIP_MEMPOOL
    if (x == MEM_TCPIP)
        return 3;
#endif
    if (x == MEM_EXTRA)
        return 4;
    if (x == MEM_TASK)
        return 2;
    if (x == MEM_USER)
        return 1;
    return 0;
}


struct f_malloc_block {
    uint32_t magic;                 /* magic fingerprint */
    struct f_malloc_block * prev;   /* previous block */
    struct f_malloc_block * next;   /* next, or last block? */
    size_t size;                    /* malloc size excluding this block - next block is adjacent, if !last_block */
    uint32_t flags;
    int pid;
};


static int block_fits(struct f_malloc_block *blk, size_t size, int flags)
{
    uint32_t baddr = (uint32_t)blk;
    uint32_t reqsize = size + sizeof(struct f_malloc_block);
    if (!blk)
        return 0;

    if (!block_valid(blk))
        return 0;

    if (in_use(blk))
        return 0;

    if (size > blk->size)
        return 0;

    return 1;
}

#define block_valid(b) ((b) && (b->magic == F_MALLOC_MAGIC))

#define F_MALLOC_MAGIC    (0xDECEA5ED)

#define in_use(x) (((x->flags) & F_IN_USE) == F_IN_USE)

#define F_IN_USE 0x20


void *memset(void *s, int c, size_t n)
{
	unsigned char *d = (unsigned char *)s;

	while (n--) {
		*d++ = (unsigned char)c;
	}

	return s;
}


char *strcat(char *dest, const char *src)
{
    int i = 0;
    int j = strlen(dest);

    for (i = 0; i < strlen(src); i++) {
        dest[j++] = src[i];
    }
    dest[j] = '\0';

    return dest;
}

int strcmp(const char *s1, const char *s2)
{
    int diff = 0;

    while (!diff && *s1) {
        diff = (int)*s1 - (int)*s2;
        s1++;
        s2++;
    }

	return diff;
}


int strcasecmp(const char *s1, const char *s2)
{
    int diff = 0;

    while (!diff && *s1) {
        diff = (int)*s1 - (int)*s2;

        if ((diff == 'A' - 'a') || (diff == 'a' - 'A'))
            diff = 0;

        s1++;
        s2++;
    }

	return diff;
}


size_t strlen(const char *s)
{
    int i = 0;

    while (s[i] != 0)
        i++;

    return i;
}


char *strncat(char *dest, const char *src, size_t n)
{
    int i = 0;
    int j = strlen(dest);

    for (i = 0; i < strlen(src); i++) {
        if (j >= (n - 1)) {
            break;
        }
        dest[j++] = src[i];
    }
    dest[j] = '\0';

    return dest;
}


int strncmp(const char *s1, const char *s2, size_t n)
{
    int diff = 0;

    while (n > 0) {
        diff = (unsigned char)*s1 - (unsigned char)*s2;
        if (diff || !*s1)
            break;
        s1++;
        s2++;
        n--;
    }

    return diff;
}

void *memcpy(void *dst, const void *src, size_t n)
{
    int i;
    const char *s = (const char *)src;
    char *d = (char *)dst;

    for (i = 0; i < n; i++) {
        d[i] = s[i];
    }

    return dst;
}


char *strncpy(char *dst, const char *src, size_t n)
{
    int i;

    for (i = 0; i < n; i++) {
        dst[i] = src[i];
        if (src[i] == '\0')
            break;
    }

    return dst;
}

char *strcpy(char *dst, const char *src)
{
    int i = 0;

    while(1 < 2) {
        dst[i] = src[i];
        if (src[i] == '\0')
            break;
        i++;
    }

    return dst;
}


int memcmp(const void *_s1, const void *_s2, size_t n)
{
    int diff = 0;
    const unsigned char *s1 = (const unsigned char *)_s1;
    const unsigned char *s2 = (const unsigned char *)_s2;

    while (!diff && n) {
        diff = (int)*s1 - (int)*s2;
        s1++;
        s2++;
        n--;
    }

	return diff;
}


static mutex_t * const mlock = (mutex_t *)(&_mlock);

int suspend_on_mutex_lock(mutex_t *s)
{
    int ret;
    if (!s)
        return -EINVAL;
    ret = _mutex_lock(s);
    if (ret != 0) {
        _add_listener(s);
        return EAGAIN;
    }
    return 0;
}


{
    struct nvic_stack_frame *nvic_frame;
    struct extra_stack_frame *extra_frame;
    uint8_t *sp;

    if (nice < NICE_RT)
        nice = NICE_RT;

    if (nice > NICE_MAX)
        nice = NICE_MAX;

    new->tb.start = vfsi->init;
    new->tb.arg = task_pass_args(arg);
    new->tb.timeslice = TIMESLICE(new);
    new->tb.state = TASK_RUNNABLE;
    new->tb.sighdlr = NULL;
    new->tb.sigpend = 0;
    new->tb.sigmask = 0;
    new->tb.tracer = NULL;
    new->tb.timer_id = -1;
    new->tb.specifics = NULL;
    new->tb.n_specifics = 0;

    if ((new->tb.flags &TASK_FLAG_VFORK) != 0) {
        struct task *pt = tasklist_get(&tasks_idling, new->tb.ppid);
        if (!pt)
            pt = tasklist_get(&tasks_running, new->tb.ppid);
        if (pt) {
            /* Restore parent's stack */
            memcpy((void *)pt->tb.cur_stack, (void *)&new->stack,
                   SCHEDULER_STACK_SIZE);
            task_resume_vfork(pt);
        }
        new->tb.flags &= (~TASK_FLAG_VFORK);
    }

    /* stack memory */
    sp = (((uint8_t *)(&new->stack)) + SCHEDULER_STACK_SIZE - NVIC_FRAME_SIZE);

    new->tb.cur_stack = &new->stack;

    /* Change relocated section ownership */
    fmalloc_chown((void *)vfsi->pic, new->tb.pid);

    /* Stack frame is at the end of the stack space,
       the NVIC_FRAME is required for context-switching */
    nvic_frame = (struct nvic_stack_frame *)sp;
    memset(nvic_frame, 0, NVIC_FRAME_SIZE);
    nvic_frame->r0 = (uint32_t) new->tb.arg;
    nvic_frame->pc = (uint32_t) new->tb.start;
    nvic_frame->lr = (uint32_t)task_end;
    nvic_frame->psr = 0x01000000u;
    /* The EXTRA_FRAME is needed in order to save/restore
       the task context when servicing PendSV exceptions */
    sp -= EXTRA_FRAME_SIZE;
    extra_frame = (struct extra_stack_frame *)sp;
    extra_frame->r9 = new->tb.vfsi->pic;
    new->tb.sp = (uint32_t *)sp;
}

