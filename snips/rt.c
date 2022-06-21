#ifndef __USE_FILE_OFFSET64
extern int open (const char *__file, int __oflag, ...) __nonnull ((1));
typedef long                            rt_base_t;      /**< Nbit CPU related date type */

rt_inline rt_base_t _heap_lock(void)
{
#if defined(RT_USING_HEAP_ISR)
    return rt_hw_interrupt_disable();
#elif defined(RT_USING_MUTEX)
    if (rt_thread_self())
        return rt_mutex_take(&_lock, RT_WAITING_FOREVER);
    else
        return RT_EOK;
#else
    rt_enter_critical();
    return RT_EOK;
#endif
}

;/*
; * rt_base_t rt_hw_interrupt_disable();
; */
    .globl rt_hw_interrupt_disable
rt_hw_interrupt_disable:
    MRS     R0, CPSR
    ORR     R1, R0, #NOINT
    MSR     CPSR_c, R1
    BX      LR
typedef long                            rt_base_t;      /**< Nbit CPU related date type */

typedef struct rt_thread *rt_thread_t;

//===
/**
 * Thread structure
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



struct rt_list_node
{
    struct rt_list_node *next;                          /**< point to next node. */
    struct rt_list_node *prev;                          /**< point to prev node. */
};

typedef rt_base_t                       rt_err_t;       /**< Type for error number */


typedef unsigned long rt_sigset_t;


/**
 * timer structure
 */
struct rt_timer
{
    struct rt_object parent;                            /**< inherit from rt_object */

    rt_list_t        row[RT_TIMER_SKIP_LIST_LEVEL];

    void (*timeout_func)(void *parameter);              /**< timeout function */
    void            *parameter;                         /**< timeout function's parameter */

    rt_tick_t        init_tick;                         /**< timer timeout tick */
    rt_tick_t        timeout_tick;                      /**< timeout tick */
};

typedef rt_uint32_t                     rt_tick_t;      /**< Type for tick count */

/**
 * Base structure of Kernel object
 */
struct rt_object
{
    char       name[RT_NAME_MAX];                       /**< name of kernel object */
    rt_uint8_t type;                                    /**< type of kernel object */
    rt_uint8_t flag;                                    /**< flag of kernel object */

#ifdef RT_USING_MODULE
    void      *module_id;                               /**< id of application module */
#endif
    rt_list_t  list;                                    /**< list node of kernel object */
};



/**
 * CPUs definitions
 *
 */
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

#endif

struct rt_thread *rt_current_thread = RT_NULL;



//====
/* calculate speed */
static void calculate_speed_print(rt_uint32_t speed)
{
    rt_uint32_t k,m;

    k = speed/1024UL;
    if( k )
    {
        m = k/1024UL;
        if( m )
        {
            rt_kprintf("%d.%dMbyte/s",m,k%1024UL*100/1024UL);
        }
        else
        {
            rt_kprintf("%d.%dKbyte/s",k,speed%1024UL*100/1024UL);
        }
    }
    else
    {
        rt_kprintf("%dbyte/s",speed);
    }
}

/**
 * block device geometry structure
 */
struct rt_device_blk_geometry
{
    rt_uint32_t sector_count;                           /**< count of sectors */
    rt_uint32_t bytes_per_sector;                       /**< number of bytes per sector */
    rt_uint32_t block_size;                             /**< number of bytes to erase one block */
};

/**@{*/

/* RT-Thread error code definitions */
#define RT_EOK                          0               /**< There is no error */
#define RT_ERROR                        1               /**< A generic error happens */
#define RT_ETIMEOUT                     2               /**< Timed out */
#define RT_EFULL                        3               /**< The resource is full */
#define RT_EEMPTY                       4               /**< The resource is empty */
#define RT_ENOMEM                       5               /**< No memory */
#define RT_ENOSYS                       6               /**< No system */
#define RT_EBUSY                        7               /**< Busy */
#define RT_EIO                          8               /**< IO error */
#define RT_EINTR                        9               /**< Interrupted system call */
#define RT_EINVAL                       10              /**< Invalid argument */

/**@}*/

/**
 * @brief This function will open a device.
 *
 * @param dev is the pointer of device driver structure.
 *
 * @param oflag is the flags for device open.
 *
 * @return the result, RT_EOK on successfully.
 */
rt_err_t rt_device_open(rt_device_t dev, rt_uint16_t oflag)
{
    rt_err_t result = RT_EOK;

    /* parameter check */
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(rt_object_get_type(&dev->parent) == RT_Object_Class_Device);

    /* if device is not initialized, initialize it. */
    if (!(dev->flag & RT_DEVICE_FLAG_ACTIVATED))
    {
        if (device_init != RT_NULL)
        {
            result = device_init(dev);
            if (result != RT_EOK)
            {
                RT_DEBUG_LOG(RT_DEBUG_DEVICE, ("To initialize device:%s failed. The error code is %d\n",
                           dev->parent.name, result));

                return result;
            }
        }

        dev->flag |= RT_DEVICE_FLAG_ACTIVATED;
    }

    /* device is a stand alone device and opened */
    if ((dev->flag & RT_DEVICE_FLAG_STANDALONE) &&
        (dev->open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        return -RT_EBUSY;
    }

    /* call device_open interface */
    if (device_open != RT_NULL)
    {
        result = device_open(dev, oflag);
    }
    else
    {
        /* set open flag */
        dev->open_flag = (oflag & RT_DEVICE_OFLAG_MASK);
    }

    /* set open flag */
    if (result == RT_EOK || result == -RT_ENOSYS)
    {
        dev->open_flag |= RT_DEVICE_OFLAG_OPEN;

        dev->ref_count++;
        /* don't let bad things happen silently. If you are bitten by this assert,
         * please set the ref_count to a bigger type. */
        RT_ASSERT(dev->ref_count != 0);
    }

    return result;
}

//===
#define RT_NULL                         (0)

#define device_control  (dev->ops->control)



/**
 * @brief Allocate a block of memory with a minimum of 'size' bytes.
 *
 * @param size is the minimum size of the requested block in bytes.
 *
 * @return the pointer to allocated memory or NULL if no free memory was found.
 */
RT_WEAK void *rt_malloc(rt_size_t size)
{
    rt_base_t level;
    void *ptr;

    /* Enter critical zone */
    level = _heap_lock();
    /* allocate memory block from system heap */
    ptr = _MEM_MALLOC(size);
    /* Exit critical zone */
    _heap_unlock(level);
    /* call 'rt_malloc' hook */
    RT_OBJECT_HOOK_CALL(rt_malloc_hook, (ptr, size));
    return ptr;
}

struct rt_small_mem_item
{
    rt_ubase_t              pool_ptr;         /**< small memory object addr */
#ifdef ARCH_CPU_64BIT
    rt_uint32_t             resv;
#endif /* ARCH_CPU_64BIT */
    rt_size_t               next;             /**< next free item */
    rt_size_t               prev;             /**< prev free item */
#ifdef RT_USING_MEMTRACE
#ifdef ARCH_CPU_64BIT
    rt_uint8_t              thread[8];       /**< thread name */
#else
    rt_uint8_t              thread[4];       /**< thread name */
#endif /* ARCH_CPU_64BIT */
#endif /* RT_USING_MEMTRACE */
};

struct rt_small_mem
{
    struct rt_memory            parent;                 /**< inherit from rt_memory */
    rt_uint8_t                 *heap_ptr;               /**< pointer to the heap */
    struct rt_small_mem_item   *heap_end;
    struct rt_small_mem_item   *lfree;
    rt_size_t                   mem_size_aligned;       /**< aligned memory size */
};

#define RT_ALIGN(size, align)           (((size) + (align) - 1) & ~((align) - 1))

#define RT_ALIGN_SIZE 4

static struct rt_memheap system_heap;

/**@}*/

/**
 * @ingroup BasicDef
 *
 * @def RT_ALIGN(size, align)
 * Return the most contiguous size aligned at specified width. RT_ALIGN(13, 4)
 * would return 16.
 */
#define RT_ALIGN(size, align)           (((size) + (align) - 1) & ~((align) - 1))


/**
 * @addtogroup MM
 */

/**@{*/

/**
 * @brief Allocate a block of memory with a minimum of 'size' bytes.
 *
 * @param m the small memory management object.
 *
 * @param size is the minimum size of the requested block in bytes.
 *
 * @return the pointer to allocated memory or NULL if no free memory was found.
 */
void *rt_smem_alloc(rt_smem_t m, rt_size_t size)
{
    rt_size_t ptr, ptr2;
    struct rt_small_mem_item *mem, *mem2;
    struct rt_small_mem *small_mem;

    if (size == 0)
        return RT_NULL;

    RT_ASSERT(m != RT_NULL);
    RT_ASSERT(rt_object_get_type(&m->parent) == RT_Object_Class_Memory);
    RT_ASSERT(rt_object_is_systemobject(&m->parent));

    if (size != RT_ALIGN(size, RT_ALIGN_SIZE))
        RT_DEBUG_LOG(RT_DEBUG_MEM, ("malloc size %d, but align to %d\n",
                                    size, RT_ALIGN(size, RT_ALIGN_SIZE)));
    else
        RT_DEBUG_LOG(RT_DEBUG_MEM, ("malloc size %d\n", size));

    small_mem = (struct rt_small_mem *)m;
    /* alignment size */
    size = RT_ALIGN(size, RT_ALIGN_SIZE);

    if (size > small_mem->mem_size_aligned)
    {
        RT_DEBUG_LOG(RT_DEBUG_MEM, ("no memory\n"));

        return RT_NULL;
    }

    /* every data block must be at least MIN_SIZE_ALIGNED long */
    if (size < MIN_SIZE_ALIGNED)
        size = MIN_SIZE_ALIGNED;

    for (ptr = (rt_uint8_t *)small_mem->lfree - small_mem->heap_ptr;
         ptr <= small_mem->mem_size_aligned - size;
         ptr = ((struct rt_small_mem_item *)&small_mem->heap_ptr[ptr])->next)
    {
        mem = (struct rt_small_mem_item *)&small_mem->heap_ptr[ptr];

        if ((!MEM_ISUSED(mem)) && (mem->next - (ptr + SIZEOF_STRUCT_MEM)) >= size)
        {
            /* mem is not used and at least perfect fit is possible:
             * mem->next - (ptr + SIZEOF_STRUCT_MEM) gives us the 'user data size' of mem */

            if (mem->next - (ptr + SIZEOF_STRUCT_MEM) >=
                (size + SIZEOF_STRUCT_MEM + MIN_SIZE_ALIGNED))
            {
                /* (in addition to the above, we test if another struct rt_small_mem_item (SIZEOF_STRUCT_MEM) containing
                 * at least MIN_SIZE_ALIGNED of data also fits in the 'user data space' of 'mem')
                 * -> split large block, create empty remainder,
                 * remainder must be large enough to contain MIN_SIZE_ALIGNED data: if
                 * mem->next - (ptr + (2*SIZEOF_STRUCT_MEM)) == size,
                 * struct rt_small_mem_item would fit in but no data between mem2 and mem2->next
                 * @todo we could leave out MIN_SIZE_ALIGNED. We would create an empty
                 *       region that couldn't hold data, but when mem->next gets freed,
                 *       the 2 regions would be combined, resulting in more free memory
                 */
                ptr2 = ptr + SIZEOF_STRUCT_MEM + size;

                /* create mem2 struct */
                mem2       = (struct rt_small_mem_item *)&small_mem->heap_ptr[ptr2];
                mem2->pool_ptr = MEM_FREED();
                mem2->next = mem->next;
                mem2->prev = ptr;
#ifdef RT_USING_MEMTRACE
                rt_smem_setname(mem2, "    ");
#endif /* RT_USING_MEMTRACE */

                /* and insert it between mem and mem->next */
                mem->next = ptr2;

                if (mem2->next != small_mem->mem_size_aligned + SIZEOF_STRUCT_MEM)
                {
                    ((struct rt_small_mem_item *)&small_mem->heap_ptr[mem2->next])->prev = ptr2;
                }
                small_mem->parent.used += (size + SIZEOF_STRUCT_MEM);
                if (small_mem->parent.max < small_mem->parent.used)
                    small_mem->parent.max = small_mem->parent.used;
            }
            else
            {
                /* (a mem2 struct does no fit into the user data space of mem and mem->next will always
                 * be used at this point: if not we have 2 unused structs in a row, plug_holes should have
                 * take care of this).
                 * -> near fit or excact fit: do not split, no mem2 creation
                 * also can't move mem->next directly behind mem, since mem->next
                 * will always be used at this point!
                 */
                small_mem->parent.used += mem->next - ((rt_uint8_t *)mem - small_mem->heap_ptr);
                if (small_mem->parent.max < small_mem->parent.used)
                    small_mem->parent.max = small_mem->parent.used;
            }
            /* set small memory object */
            mem->pool_ptr = MEM_USED();
#ifdef RT_USING_MEMTRACE
            if (rt_thread_self())
                rt_smem_setname(mem, rt_thread_self()->name);
            else
                rt_smem_setname(mem, "NONE");
#endif /* RT_USING_MEMTRACE */

            if (mem == small_mem->lfree)
            {
                /* Find next free block after mem and update lowest free pointer */
                while (MEM_ISUSED(small_mem->lfree) && small_mem->lfree != small_mem->heap_end)
                    small_mem->lfree = (struct rt_small_mem_item *)&small_mem->heap_ptr[small_mem->lfree->next];

                RT_ASSERT(((small_mem->lfree == small_mem->heap_end) || (!MEM_ISUSED(small_mem->lfree))));
            }
            RT_ASSERT((rt_ubase_t)mem + SIZEOF_STRUCT_MEM + size <= (rt_ubase_t)small_mem->heap_end);
            RT_ASSERT((rt_ubase_t)((rt_uint8_t *)mem + SIZEOF_STRUCT_MEM) % RT_ALIGN_SIZE == 0);
            RT_ASSERT((((rt_ubase_t)mem) & (RT_ALIGN_SIZE - 1)) == 0);

            RT_DEBUG_LOG(RT_DEBUG_MEM,
                         ("allocate memory at 0x%x, size: %d\n",
                          (rt_ubase_t)((rt_uint8_t *)mem + SIZEOF_STRUCT_MEM),
                          (rt_ubase_t)(mem->next - ((rt_uint8_t *)mem - small_mem->heap_ptr))));

            /* return the memory data except mem struct */
            return (rt_uint8_t *)mem + SIZEOF_STRUCT_MEM;
        }
    }

    return RT_NULL;
}

#ifdef RT_USING_HOOK
static void (*rt_malloc_hook)(void *ptr, rt_size_t size);


/**
 * @brief Allocate a block of memory with a minimum of 'size' bytes.
 *
 * @param size is the minimum size of the requested block in bytes.
 *
 * @return the pointer to allocated memory or NULL if no free memory was found.
 */
RT_WEAK void *rt_malloc(rt_size_t size)
{
    rt_base_t level;
    void *ptr;

    /* Enter critical zone */
    level = _heap_lock();
    /* allocate memory block from system heap */
    ptr = _MEM_MALLOC(size);
    /* Exit critical zone */
    _heap_unlock(level);
    /* call 'rt_malloc' hook */
    RT_OBJECT_HOOK_CALL(rt_malloc_hook, (ptr, size));
    return ptr;
}


#define RT_DEBUG_LOG(type, message)                                           \
do                                                                            \
{                                                                             \
    if (type)                                                                 \
        rt_kprintf message;                                                   \
}                                                                             \
while (0)


//===
/**
 * This function will print a formatted string on system console.
 *
 * @param fmt is the format parameters.
 *
 * @return The number of characters actually written to buffer.
 */
RT_WEAK int rt_kprintf(const char *fmt, ...)
{
    va_list args;
    rt_size_t length;
    static char rt_log_buf[RT_CONSOLEBUF_SIZE];

    va_start(args, fmt);
    /* the return value of vsnprintf is the number of bytes that would be
     * written to buffer had if the size of the buffer been sufficiently
     * large excluding the terminating null byte. If the output string
     * would be larger than the rt_log_buf, we have to adjust the output
     * length. */
    length = rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    if (length > RT_CONSOLEBUF_SIZE - 1)
        length = RT_CONSOLEBUF_SIZE - 1;
#ifdef RT_USING_DEVICE
    if (_console_device == RT_NULL)
    {
        rt_hw_console_output(rt_log_buf);
    }
    else
    {
        rt_device_write(_console_device, 0, rt_log_buf, length);
    }
#else
    rt_hw_console_output(rt_log_buf);
#endif /* RT_USING_DEVICE */
    va_end(args);

    return length;
}

#define SIZEOF_STRUCT_MEM    LWIP_MEM_ALIGN_SIZE(sizeof(struct mem))

#define LWIP_MEM_ALIGN_SIZE(size) (((size) + MEM_ALIGNMENT - 1) & ~(MEM_ALIGNMENT-1))

#define rt_tick rt_cpu_index(0)->tick

/**
 * @brief   This fucntion will return the cpu object corresponding to index.
 *
 * @return  Return a pointer to the cpu object corresponding to index.
 */
struct rt_cpu *rt_cpu_index(int index)
{
    return &_cpus[index];
}
calculate_speed_print( (geometry.bytes_per_sector*200UL*RT_TICK_PER_SECOND)/(tick_end-tick_start) );

calculate_speed_print( (geometry.bytes_per_sector * sector * 10 * RT_TICK_PER_SECOND)/(tick_end-tick_start) );

Proc_8 (Arr_1_Par_Ref, Arr_2_Par_Ref, Int_1_Par_Val, Int_2_Par_Val) /*********************************************************************/
    /* executed once      */
    /* Int_Par_Val_1 == 3 */
    /* Int_Par_Val_2 == 7 */
Arr_1_Dim       Arr_1_Par_Ref;
Arr_2_Dim       Arr_2_Par_Ref;
int             Int_1_Par_Val;
int             Int_2_Par_Val;

typedef long                            rt_base_t;      /**< Nbit CPU related date type */

//===
/**
 * @brief This function will start the timer
 *
 * @param timer the timer to be started
 *
 * @return the operation status, RT_EOK on OK, -RT_ERROR on error
 */
rt_err_t rt_timer_start(rt_timer_t timer)
{
    unsigned int row_lvl;
    rt_list_t *timer_list;
    register rt_base_t level;
    register rt_bool_t need_schedule;
    rt_list_t *row_head[RT_TIMER_SKIP_LIST_LEVEL];
    unsigned int tst_nr;
    static unsigned int random_nr;

    /* parameter check */
    RT_ASSERT(timer != RT_NULL);
    RT_ASSERT(rt_object_get_type(&timer->parent) == RT_Object_Class_Timer);

    need_schedule = RT_FALSE;

    /* stop timer firstly */
    level = rt_hw_interrupt_disable();
    /* remove timer from list */
    _timer_remove(timer);
    /* change status of timer */
    timer->parent.flag &= ~RT_TIMER_FLAG_ACTIVATED;

    RT_OBJECT_HOOK_CALL(rt_object_take_hook, (&(timer->parent)));

    timer->timeout_tick = rt_tick_get() + timer->init_tick;

#ifdef RT_USING_TIMER_SOFT
    if (timer->parent.flag & RT_TIMER_FLAG_SOFT_TIMER)
    {
        /* insert timer to soft timer list */
        timer_list = _soft_timer_list;
    }
    else
#endif /* RT_USING_TIMER_SOFT */
    {
        /* insert timer to system timer list */
        timer_list = _timer_list;
    }

    row_head[0]  = &timer_list[0];
    for (row_lvl = 0; row_lvl < RT_TIMER_SKIP_LIST_LEVEL; row_lvl++)
    {
        for (; row_head[row_lvl] != timer_list[row_lvl].prev;
             row_head[row_lvl]  = row_head[row_lvl]->next)
        {
            struct rt_timer *t;
            rt_list_t *p = row_head[row_lvl]->next;

            /* fix up the entry pointer */
            t = rt_list_entry(p, struct rt_timer, row[row_lvl]);

            /* If we have two timers that timeout at the same time, it's
             * preferred that the timer inserted early get called early.
             * So insert the new timer to the end the the some-timeout timer
             * list.
             */
            if ((t->timeout_tick - timer->timeout_tick) == 0)
            {
                continue;
            }
            else if ((t->timeout_tick - timer->timeout_tick) < RT_TICK_MAX / 2)
            {
                break;
            }
        }
        if (row_lvl != RT_TIMER_SKIP_LIST_LEVEL - 1)
            row_head[row_lvl + 1] = row_head[row_lvl] + 1;
    }

    /* Interestingly, this super simple timer insert counter works very very
     * well on distributing the list height uniformly. By means of "very very
     * well", I mean it beats the randomness of timer->timeout_tick very easily
     * (actually, the timeout_tick is not random and easy to be attacked). */
    random_nr++;
    tst_nr = random_nr;

    rt_list_insert_after(row_head[RT_TIMER_SKIP_LIST_LEVEL - 1],
                         &(timer->row[RT_TIMER_SKIP_LIST_LEVEL - 1]));
    for (row_lvl = 2; row_lvl <= RT_TIMER_SKIP_LIST_LEVEL; row_lvl++)
    {
        if (!(tst_nr & RT_TIMER_SKIP_LIST_MASK))
            rt_list_insert_after(row_head[RT_TIMER_SKIP_LIST_LEVEL - row_lvl],
                                 &(timer->row[RT_TIMER_SKIP_LIST_LEVEL - row_lvl]));
        else
            break;
        /* Shift over the bits we have tested. Works well with 1 bit and 2
         * bits. */
        tst_nr >>= (RT_TIMER_SKIP_LIST_MASK + 1) >> 1;
    }

    timer->parent.flag |= RT_TIMER_FLAG_ACTIVATED;

#ifdef RT_USING_TIMER_SOFT
    if (timer->parent.flag & RT_TIMER_FLAG_SOFT_TIMER)
    {
        /* check whether timer thread is ready */
        if ((_soft_timer_status == RT_SOFT_TIMER_IDLE) &&
           ((_timer_thread.stat & RT_THREAD_STAT_MASK) == RT_THREAD_SUSPEND))
        {
            /* resume timer thread to check soft timer */
            rt_thread_resume(&_timer_thread);
            need_schedule = RT_TRUE;
        }
    }
#endif /* RT_USING_TIMER_SOFT */

    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    if (need_schedule)
    {
        rt_schedule();
    }

    return RT_EOK;
}

/**
 * @brief This function will perform one scheduling. It will select one thread
 *        with the highest priority level in global ready queue or local ready queue,
 *        then switch to it.
 */
void rt_schedule(void)
{
    rt_base_t level;
    struct rt_thread *to_thread;
    struct rt_thread *current_thread;
    struct rt_cpu    *pcpu;
    int cpu_id;

    /* disable interrupt */
    level  = rt_hw_interrupt_disable();

    cpu_id = rt_hw_cpu_id();
    pcpu   = rt_cpu_index(cpu_id);
    current_thread = pcpu->current_thread;

    /* whether do switch in interrupt */
    if (pcpu->irq_nest)
    {
        pcpu->irq_switch_flag = 1;
        rt_hw_interrupt_enable(level);
        goto __exit;
    }

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

    if (current_thread->scheduler_lock_nest == 1) /* whether lock scheduler */
    {
        rt_ubase_t highest_ready_priority;

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

                /* switch to new thread */
                RT_DEBUG_LOG(RT_DEBUG_SCHEDULER,
                        ("[%d]switch to priority#%d "
                         "thread:%.*s(sp:0x%08x), "
                         "from thread:%.*s(sp: 0x%08x)\n",
                         pcpu->irq_nest, highest_ready_priority,
                         RT_NAME_MAX, to_thread->name, to_thread->sp,
                         RT_NAME_MAX, current_thread->name, current_thread->sp));

#ifdef RT_USING_OVERFLOW_CHECK
                _rt_scheduler_stack_check(to_thread);
#endif /* RT_USING_OVERFLOW_CHECK */

                RT_OBJECT_HOOK_CALL(rt_scheduler_switch_hook, (current_thread));

                rt_hw_context_switch((rt_ubase_t)&current_thread->sp,
                        (rt_ubase_t)&to_thread->sp, to_thread);
            }
        }
    }

    /* enable interrupt */
    rt_hw_interrupt_enable(level);

#ifdef RT_USING_SIGNALS
    /* check stat of thread for signal */
    level = rt_hw_interrupt_disable();
    if (current_thread->stat & RT_THREAD_STAT_SIGNAL_PENDING)
    {
        extern void rt_thread_handle_sig(rt_bool_t clean_state);

        current_thread->stat &= ~RT_THREAD_STAT_SIGNAL_PENDING;

        rt_hw_interrupt_enable(level);

        /* check signal status */
        rt_thread_handle_sig(RT_TRUE);
    }
    else
    {
        rt_hw_interrupt_enable(level);
    }
#endif /* RT_USING_SIGNALS */

__exit:
    return ;
}
#else

            __IO   uint8_t error                       : 1;    /* This bit will be set when three attempts have been made to perform a transaction

						*/
/* Time Value */
typedef struct rt_hwtimerval
{
    rt_int32_t sec;      /* second */
    rt_int32_t usec;     /* microsecond */
} rt_hwtimerval_t;

//====
void mem_test(uint32_t address, uint32_t size )
{
    uint32_t i;

    printf("memtest,address: 0x%08X size: 0x%08X\r\n", address, size);

    /**< 8bit test */
    {
        uint8_t * p_uint8_t = (uint8_t *)address;
        for(i=0; i<size/sizeof(uint8_t); i++)
        {
            *p_uint8_t++ = (uint8_t)i;
        }

        p_uint8_t = (uint8_t *)address;
        for(i=0; i<size/sizeof(uint8_t); i++)
        {
            if( *p_uint8_t != (uint8_t)i )
            {
                printf("8bit test fail @ 0x%08X\r\nsystem halt!!!!!",(uint32_t)p_uint8_t);
                while(1);
            }
            p_uint8_t++;
        }
        printf("8bit test pass!!\r\n");
    }

    /**< 16bit test */
    {
        uint16_t * p_uint16_t = (uint16_t *)address;
        for(i=0; i<size/sizeof(uint16_t); i++)
        {
            *p_uint16_t++ = (uint16_t)i;
        }

        p_uint16_t = (uint16_t *)address;
        for(i=0; i<size/sizeof(uint16_t); i++)
        {
            if( *p_uint16_t != (uint16_t)i )
            {
                printf("16bit test fail @ 0x%08X\r\nsystem halt!!!!!",(uint32_t)p_uint16_t);
                while(1);
            }
            p_uint16_t++;
        }
        printf("16bit test pass!!\r\n");
    }

    /**< 32bit test */
    {
        uint32_t * p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            *p_uint32_t++ = (uint32_t)i;
        }

        p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            if( *p_uint32_t != (uint32_t)i )
            {
                printf("32bit test fail @ 0x%08X\r\nsystem halt!!!!!",(uint32_t)p_uint32_t);
                while(1);
            }
            p_uint32_t++;
        }
        printf("32bit test pass!!\r\n");
    }

    /**< 32bit Loopback test */
    {
        uint32_t * p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            *p_uint32_t  = (uint32_t)p_uint32_t;
            p_uint32_t++;
        }

        p_uint32_t = (uint32_t *)address;
        for(i=0; i<size/sizeof(uint32_t); i++)
        {
            if( *p_uint32_t != (uint32_t)p_uint32_t )
            {
                printf("32bit Loopback test fail @ 0x%08X", (uint32_t)p_uint32_t);
                printf(" data:0x%08X \r\n", (uint32_t)*p_uint32_t);
                printf("system halt!!!!!",(uint32_t)p_uint32_t);
                while(1);
            }
            p_uint32_t++;
        }
        printf("32bit Loopback test pass!!\r\n");
    }
}

#define RT_TICK_PER_SECOND 100u
typedef signed long int __int64_t;


/**
 * @brief    This function will return the passed millisecond from boot.
 *
 * @note     if the value of RT_TICK_PER_SECOND is lower than 1000 or
 *           is not an integral multiple of 1000, this function will not
 *           provide the correct 1ms-based tick.
 *
 * @return   Return passed millisecond from boot.
 */
RT_WEAK rt_tick_t rt_tick_get_millisecond(void)
{
#if 1000 % RT_TICK_PER_SECOND == 0u
    return rt_tick_get() * (1000u / RT_TICK_PER_SECOND);
#else
    #warning "rt-thread cannot provide a correct 1ms-based tick any longer,\
    please redefine this function in another file by using a high-precision hard-timer."
    return 0;
#endif /* 1000 % RT_TICK_PER_SECOND == 0u */
}

/**@}*/

