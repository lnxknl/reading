/* Define basic errno for rhino modules */

typedef enum
{
    RHINO_SUCCESS = 0u,
    RHINO_SYS_FATAL_ERR,
    RHINO_SYS_SP_ERR,
    RHINO_RUNNING,
    RHINO_STOPPED,
    RHINO_INV_PARAM,
    RHINO_NULL_PTR,
    RHINO_INV_ALIGN,
    RHINO_KOBJ_TYPE_ERR,
    RHINO_KOBJ_DEL_ERR,
    RHINO_KOBJ_DOCKER_EXIST,
    RHINO_KOBJ_BLK,
    RHINO_KOBJ_SET_FULL,
    RHINO_NOTIFY_FUNC_EXIST,

    RHINO_MM_POOL_SIZE_ERR = 100u,
    RHINO_MM_ALLOC_SIZE_ERR,
    RHINO_MM_FREE_ADDR_ERR,
    RHINO_MM_CORRUPT_ERR,
    RHINO_DYN_MEM_PROC_ERR,
    RHINO_NO_MEM,
    RHINO_RINGBUF_FULL,
    RHINO_RINGBUF_EMPTY,

    RHINO_SCHED_DISABLE = 200u,
    RHINO_SCHED_ALREADY_ENABLED,
    RHINO_SCHED_LOCK_COUNT_OVF,
    RHINO_INV_SCHED_WAY,

    RHINO_TASK_INV_STACK_SIZE = 300u,
    RHINO_TASK_NOT_SUSPENDED,
    RHINO_TASK_DEL_NOT_ALLOWED,
    RHINO_TASK_SUSPEND_NOT_ALLOWED,
    RHINO_TASK_CANCELED,
    RHINO_SUSPENDED_COUNT_OVF,
    RHINO_BEYOND_MAX_PRI,
    RHINO_PRI_CHG_NOT_ALLOWED,
    RHINO_INV_TASK_STATE,
    RHINO_IDLE_TASK_EXIST,

    RHINO_NO_PEND_WAIT = 400u,
    RHINO_BLK_ABORT,
    RHINO_BLK_TIMEOUT,
    RHINO_BLK_DEL,
    RHINO_BLK_INV_STATE,
    RHINO_BLK_POOL_SIZE_ERR,

    RHINO_TIMER_STATE_INV = 500u,

    RHINO_NO_THIS_EVENT_OPT = 600u,

    RHINO_BUF_QUEUE_INV_SIZE = 700u,
    RHINO_BUF_QUEUE_SIZE_ZERO,
    RHINO_BUF_QUEUE_FULL,
    RHINO_BUF_QUEUE_MSG_SIZE_OVERFLOW,
    RHINO_QUEUE_FULL,
    RHINO_QUEUE_NOT_FULL,

    RHINO_SEM_OVF = 800u,
    RHINO_SEM_TASK_WAITING,

    RHINO_MUTEX_NOT_RELEASED_BY_OWNER = 900u,
    RHINO_MUTEX_OWNER_NESTED,
    RHINO_MUTEX_NESTED_OVF,

    RHINO_NOT_CALLED_BY_INTRPT = 1000u,
    RHINO_TRY_AGAIN,

    RHINO_WORKQUEUE_EXIST = 1100u,
    RHINO_WORKQUEUE_NOT_EXIST,
    RHINO_WORKQUEUE_WORK_EXIST,
    RHINO_WORKQUEUE_BUSY,
    RHINO_WORKQUEUE_WORK_RUNNING,

    RHINO_TASK_STACK_OVF = 1200u,
    RHINO_INTRPT_STACK_OVF,

    RHINO_STATE_ALIGN = INT_MAX /* keep enum 4 bytes at 32bit machine */
} kstat_t;



#define krhino_spin_lock_init(lock)                     do {                                            \
                                                            kspinlock_t *s = (kspinlock_t *)(lock);     \
                                                            s->owner       = KRHINO_SPINLOCK_FREE_VAL;  \
                                                        } while(0)



kstat_t krhino_sched_disable(void)
{
    CPSR_ALLOC();

    RHINO_CRITICAL_ENTER();

    INTRPT_NESTED_LEVEL_CHK();

    if (g_sched_lock[cpu_cur_get()] >= SCHED_MAX_LOCK_COUNT) {
        RHINO_CRITICAL_EXIT();
        return RHINO_SCHED_LOCK_COUNT_OVF;
    }

#if (RHINO_CONFIG_SYS_STATS > 0)
    sched_disable_measure_start();
#endif

    g_sched_lock[cpu_cur_get()]++;

    RHINO_CRITICAL_EXIT();

    return RHINO_SUCCESS;
}


kstat_t krhino_sched_enable(void)
{
    CPSR_ALLOC();

    RHINO_CRITICAL_ENTER();

    INTRPT_NESTED_LEVEL_CHK();

    if (g_sched_lock[cpu_cur_get()] == 0u) {
        RHINO_CRITICAL_EXIT();
        return RHINO_SCHED_ALREADY_ENABLED;
    }

    g_sched_lock[cpu_cur_get()]--;

    if (g_sched_lock[cpu_cur_get()] > 0u) {
        RHINO_CRITICAL_EXIT();
        return RHINO_SCHED_DISABLE;
    }

#if (RHINO_CONFIG_SYS_STATS > 0)
    sched_disable_measure_stop();
#endif

    RHINO_CRITICAL_EXIT_SCHED();

    return RHINO_SUCCESS;
}


#define krhino_spin_lock_irq_save(lock, flags)          do {                                            \
                                                            (void)lock;                                 \
                                                            flags          = cpu_intrpt_save();         \
                                                        } while (0)



#define krhino_spin_unlock_irq_restore(lock, flags)     do {                                            \
                                                            (void)lock;                                 \
                                                            cpu_intrpt_restore(flags);                  \
                                                        } while (0)


RHINO_INLINE void klist_init(klist_t *list_head)
{
    list_head->next = list_head;
    list_head->prev = list_head;
}


/**
 * As a member of other structures, 'klist_t' can form a doubly linked list.
 */

typedef struct klist_s {
    struct klist_s *next;
    struct klist_s *prev;
} klist_t;


void runqueue_init(runqueue_t *rq)
{
    uint8_t prio;

    rq->highest_pri = RHINO_CONFIG_PRI_MAX;

    for (prio = 0; prio < RHINO_CONFIG_PRI_MAX; prio++) {
        rq->cur_list_item[prio] = NULL;
    }
}


typedef struct {
    klist_t  *cur_list_item[RHINO_CONFIG_PRI_MAX];
    uint32_t  task_bit_map[NUM_WORDS];
    uint8_t   highest_pri;
} runqueue_t;


void tick_list_init(void)
{
   klist_init(&g_tick_head);
}


RHINO_INLINE void klist_init(klist_t *list_head)
{
    list_head->next = list_head;
    list_head->prev = list_head;
}


void kobj_list_init(void)
{
    klist_init(&(g_kobj_list.task_head));
    klist_init(&(g_kobj_list.mutex_head));

#if (RHINO_CONFIG_SEM > 0)
    klist_init(&(g_kobj_list.sem_head));
#endif

#if (RHINO_CONFIG_QUEUE > 0)
    klist_init(&(g_kobj_list.queue_head));
#endif

#if (RHINO_CONFIG_BUF_QUEUE > 0)
    klist_init(&(g_kobj_list.buf_queue_head));
#endif

#if (RHINO_CONFIG_EVENT_FLAG > 0)
    klist_init(&(g_kobj_list.event_head));
#endif
}

kobj_list_t g_kobj_list;


typedef struct {
    klist_t task_head;
    klist_t mutex_head;

#if (RHINO_CONFIG_SEM > 0)
    klist_t sem_head;
#endif

#if (RHINO_CONFIG_QUEUE > 0)
    klist_t queue_head;
#endif

#if (RHINO_CONFIG_EVENT_FLAG > 0)
    klist_t event_head;
#endif

#if (RHINO_CONFIG_BUF_QUEUE > 0)
    klist_t buf_queue_head;
#endif
} kobj_list_t;


typedef char     name_t;
typedef uint8_t  suspend_nested_t;
typedef uint32_t sem_count_t;
typedef uint32_t mutex_nested_t;
typedef uint64_t sys_time_t;
typedef uint64_t tick_t;
typedef int64_t  tick_i_t;
typedef uint64_t idle_count_t;
typedef uint64_t ctx_switch_t;


typedef struct {
    /**<
     *  Task SP, update when task switching.
     *  Access by assemble code, so do not change position.
     */
    void            *task_stack;
    /**< access by activation, so do not change position */
    const name_t    *task_name;
#if (RHINO_CONFIG_TASK_INFO > 0)
    /**< access by assemble code, so do not change position */
    void            *user_info[RHINO_CONFIG_TASK_INFO_NUM];
#endif

#if (RHINO_CONFIG_USER_SPACE > 0)
    void            *task_ustack;
    uint32_t         ustack_size;
    uint32_t         pid;
    uint8_t          mode;
    uint8_t          is_proc;
    cpu_stack_t     *task_ustack_base;
    klist_t          task_user;
    void            *task_group;
#endif
    /**<
     *  Task stack info
     *  start 'task_stack_base', len 'stack_size * sizeof(cpu_stack_t)'
     */
    cpu_stack_t     *task_stack_base;
    size_t           stack_size;
    /**<
     *  Put task into different linked lists according to the status:
     *  1. ready queue. The list hade is g_ready_queue->cur_list_item[prio]
     *  2. The pending queue. The list hade is blk_obj in the blocking object (sem / mutex / queue, etc.).
     *     The final blocking result is recorded in blk_state
     */
    klist_t          task_list;
    /**< Count of entering the K_SUSPENDED state */
    suspend_nested_t suspend_count;
    /**< Mutex owned by this task */
    struct mutex_s  *mutex_list;
#if (RHINO_CONFIG_KOBJ_LIST > 0)
    /**< Link all task for statistics */
    klist_t          task_stats_item;
#endif
    /**< Put task into the timeout list of the tick, list head is 'g_tick_head' */
    klist_t          tick_list;
    /**< When 'g_tick_count' reaches tick_match, task's PEND or SLEEP state expires. */
    tick_t           tick_match;
    /**< Countdown of the PEND state */
    tick_t           tick_remain;

    /**< Passing massage, for 'queue' and 'buf_queue' */
    void            *msg;
#if (RHINO_CONFIG_BUF_QUEUE > 0)
    /**< Record the msg length, for 'buf_queue'. */
    size_t           bq_msg_size;
#endif
    /**<  */
    /**< Task status */
    task_stat_t      task_state;
    /**< Reasons for the end of the blocking state */
    blk_state_t      blk_state;

    /* Task block on mutex, queue, semphore, event */
    blk_obj_t       *blk_obj;

    uint32_t         task_id;
#if (RHINO_CONFIG_MM_DEBUG > 0)
    uint32_t         task_alloc_size;
#endif

#if (RHINO_CONFIG_TASK_SEM > 0)
    /**< Task semaphore  */
    struct sem_s    *task_sem_obj;
#endif

#if (RHINO_CONFIG_SYS_STATS > 0)
    size_t           task_free_stack_size;
    ctx_switch_t     task_ctx_switch_times;
    uint64_t         task_time_total_run;
    uint64_t         task_time_total_run_prev;
    lr_timer_t       task_time_this_run;
    lr_timer_t       task_exec_time;
    lr_timer_t       task_time_start;
    hr_timer_t       task_intrpt_disable_time_max;
    hr_timer_t       task_sched_disable_time_max;
#endif

#if (RHINO_CONFIG_SCHED_RR > 0)
    /**< During this round of scheduling, tasks can execute 'time_slice' ticks */
    uint32_t         time_slice;
    /**< Once RR scheduling, tasks can execute a total of 'time_total' ticks */
    uint32_t         time_total;
#endif

#if (RHINO_CONFIG_EVENT_FLAG > 0)
    uint32_t         pend_flags;
    void            *pend_info;
    uint8_t          pend_option;
#endif
    /**< KSCHED_FIFO / KSCHED_RR / KSCHED_CFS */
    uint8_t          sched_policy;

#if (RHINO_CONFIG_SCHED_CFS > 0)
    cfs_node         node;
#endif
    /**< On which CPU the task runs */
    uint8_t          cpu_num;

#if (RHINO_CONFIG_CPU_NUM > 1)
    /**< Whether the task is binded to the cpu, 0 no, 1 yes */
    uint8_t          cpu_binded;
    /**< Whether the task is ready to execute, 0 no, 1 yes */
    uint8_t          cur_exc;
    klist_t          task_del_item;
#endif

#if (RHINO_CONFIG_TASK_DEL > 0)
    uint8_t          cancel;
#endif

    /**< current prio */
    uint8_t          prio;
    /**< base prio */
    uint8_t          b_prio;
    /**< buffer from internal malloc or caller input */
    uint8_t          mm_alloc_flag;

    void            *ptcb;  /* pthread control block */

#if (RHINO_CONFIG_NEWLIBC_REENT > 0)
    struct _reent *newlibc_reent; /* newlib libc reentrancy */
#endif
} ktask_t;

typedef struct {
    blk_obj_t    blk_obj;           /**< Manage blocked tasks */
    void        *buf;               /**< ringbuf address */
    k_ringbuf_t  ringbuf;           /**< ringbuf management */
    size_t       max_msg_size;      /**< limited message length */
    size_t       cur_num;           /**< msgs used */
    size_t       peak_num;          /**< maximum msgs used */
    size_t       min_free_buf_size; /**< minimum free size */
#if (RHINO_CONFIG_KOBJ_LIST > 0)
    klist_t      buf_queue_item;    /**< kobj list for statistics */
#endif
#if (RHINO_CONFIG_USER_SPACE > 0)
    uint32_t     key;
#endif
    uint8_t      mm_alloc_flag;     /**< buffer from internal malloc or caller input */
} kbuf_queue_t;

typedef struct {
    ktimer_t   *timer;
    uint8_t     cb_num;         /**< TIMER_CMD_START/STOP/... */
    tick_t      first;

    union {
        tick_t  round;
        void   *arg;
    } u;
} k_timer_queue_cb;


void krhino_stack_ovf_check(void)
{
    ktask_t     *cur;
    cpu_stack_t *stack_start;
    uint8_t      i;

    cur = g_active_task[cpu_cur_get()];
    stack_start = cur->task_stack_base;

    for (i = 0; i < RHINO_CONFIG_STK_CHK_WORDS; i++) {
        if (*stack_start++ != RHINO_TASK_STACK_OVF_MAGIC) {
            k_err_proc(RHINO_TASK_STACK_OVF);
        }
    }

    if ((cpu_stack_t *)(cur->task_stack) < stack_start) {
        k_err_proc(RHINO_TASK_STACK_OVF);
    }

#if (RHINO_CONFIG_USER_SPACE > 0)
    if (cur->pid == 0) {
        return;
    }

    stack_start = cur->task_ustack_base;

    for (i = 0; i < RHINO_CONFIG_STK_CHK_WORDS; i++) {
        if (*stack_start++ != RHINO_TASK_STACK_OVF_MAGIC) {
            k_err_proc(RHINO_TASK_STACK_OVF);
        }
    }

    if ((cpu_stack_t *)(cur->task_ustack) < stack_start) {
        k_err_proc(RHINO_TASK_STACK_OVF);
    }
#endif
}

void krhino_init_hook(void)
{
}

void k_mm_init(void)
{
    uint32_t e = 0;

    /* init memory region */
    (void)krhino_init_mm_head(&g_kmm_head, g_mm_region[0].start, g_mm_region[0].len);
    for (e = 1 ; e < g_region_num ; e++) {
        krhino_add_mm_region(g_kmm_head, g_mm_region[e].start, g_mm_region[e].len);
    }
}

kstat_t krhino_sem_create(ksem_t *sem, const name_t *name, sem_count_t count)
{
    return sem_create(sem, name, count, K_OBJ_STATIC_ALLOC);
}

static kstat_t sem_create(ksem_t *sem, const name_t *name, sem_count_t count,
                          uint8_t mm_alloc_flag)
{
#if (RHINO_CONFIG_KOBJ_LIST > 0)
    CPSR_ALLOC();
#endif

    NULL_PARA_CHK(sem);
    NULL_PARA_CHK(name);

    memset(sem, 0, sizeof(ksem_t));

    /* init the list */
    klist_init(&sem->blk_obj.blk_list);

    /* init resource */
    sem->count              = count;
    sem->peak_count         = count;
    sem->blk_obj.name       = name;
    sem->blk_obj.blk_policy = BLK_POLICY_PRI;
    sem->mm_alloc_flag      = mm_alloc_flag;
#if (RHINO_CONFIG_TASK_DEL > 0)
    sem->blk_obj.cancel     = 1u;
#endif

#if (RHINO_CONFIG_KOBJ_LIST > 0)
    RHINO_CRITICAL_ENTER();
    klist_insert(&(g_kobj_list.sem_head), &sem->sem_item);
    RHINO_CRITICAL_EXIT();
#endif

    sem->blk_obj.obj_type = RHINO_SEM_OBJ_TYPE;

    TRACE_SEM_CREATE(krhino_cur_task_get(), sem);

    return RHINO_SUCCESS;
}

__attribute__((weak)) void dyn_mem_proc_task_start(void)
{
    krhino_task_create(&g_dyn_task, "dyn_mem_proc_task", 0, RHINO_CONFIG_K_DYN_MEM_TASK_PRI,
                       0, g_dyn_task_stack, RHINO_CONFIG_K_DYN_TASK_STACK, dyn_mem_proc_task, 1);
}

kstat_t krhino_task_cpu_create(ktask_t *task, const name_t *name, void *arg,
                               uint8_t prio, tick_t ticks, cpu_stack_t *stack_buf,
                               size_t stack_size, task_entry_t entry, uint8_t cpu_num,
                               uint8_t autorun)
{
    return task_create(task, name, arg, prio, ticks, stack_buf, stack_size, entry,
                       autorun, K_OBJ_STATIC_ALLOC, cpu_num, 1, KSCHED_RR);
}

static kstat_t task_create(ktask_t *task, const name_t *name, void *arg,
                           uint8_t prio, tick_t ticks, cpu_stack_t *stack_buf,
                           size_t stack_size, task_entry_t entry, uint8_t autorun,
                           uint8_t mm_alloc_flag, uint8_t cpu_num, uint8_t cpu_binded,
                           uint8_t sched_policy)
{
    CPSR_ALLOC();
    cpu_stack_t *tmp;
    uint8_t      i = 0;

    (void)cpu_binded;
    (void)i;

    NULL_PARA_CHK(task);
    NULL_PARA_CHK(name);
    NULL_PARA_CHK(entry);
    NULL_PARA_CHK(stack_buf);

    if (stack_size == 0u) {
        return RHINO_TASK_INV_STACK_SIZE;
    }

    if (prio >= RHINO_CONFIG_PRI_MAX) {
        return RHINO_BEYOND_MAX_PRI;
    }

#if (RHINO_CONFIG_SCHED_CFS > 0)
    if (task_policy_check(prio, sched_policy) != RHINO_SUCCESS) {
        return task_policy_check(prio, sched_policy);
    }
#endif

    RHINO_CRITICAL_ENTER();

    INTRPT_NESTED_LEVEL_CHK();

    /* idle task is only allowed to create once */
    if (prio == RHINO_IDLE_PRI) {
        if (g_idle_task_spawned[cpu_num] > 0u) {
            RHINO_CRITICAL_EXIT();
            return RHINO_IDLE_TASK_EXIST;
        }

        g_idle_task_spawned[cpu_num] = 1u;
    }

    RHINO_CRITICAL_EXIT();

    memset(task, 0, sizeof(ktask_t));

#if (RHINO_CONFIG_SCHED_RR > 0)
    if (ticks > 0u) {
        task->time_total = ticks;
    } else {
        task->time_total = RHINO_CONFIG_TIME_SLICE_DEFAULT;
    }

    task->time_slice   = task->time_total;
#endif

    task->sched_policy = sched_policy;

    if (autorun > 0u) {
        task->task_state    = K_RDY;
    } else {
        task->task_state    = K_SUSPENDED;
        task->suspend_count = 1u;
    }

    /* init all the stack element to 0 */
    task->task_stack_base = stack_buf;
    tmp = stack_buf;

    memset(tmp, 0, stack_size * sizeof(cpu_stack_t));

    klist_init(&task->tick_list);
    task->task_name        = name;
    task->prio             = prio;
    task->b_prio           = prio;
    task->stack_size       = stack_size;
    task->mm_alloc_flag    = mm_alloc_flag;
    task->cpu_num          = cpu_num;
    task->task_id          = ++g_task_id;
#if (RHINO_CONFIG_MM_DEBUG > 0)
    task->task_alloc_size  = 0;
#endif
#if (RHINO_CONFIG_USER_SPACE > 0)
    task->mode             = 0;
    task->pid              = 0;
    task->task_ustack_base = 0;
    task->task_group       = 0;
#endif

#if (RHINO_CONFIG_CPU_NUM > 1)
    task->cpu_binded = cpu_binded;
#endif

#if (RHINO_CONFIG_TASK_STACK_OVF_CHECK > 0)
#if (RHINO_CONFIG_CPU_STACK_DOWN > 0)
    tmp = task->task_stack_base;
    for (i = 0; i < RHINO_CONFIG_STK_CHK_WORDS; i++) {
        *tmp++ = RHINO_TASK_STACK_OVF_MAGIC;
    }
#else
    tmp = (cpu_stack_t *)(task->task_stack_base) + task->stack_size - RHINO_CONFIG_STK_CHK_WORDS;
    for (i = 0; i < RHINO_CONFIG_STK_CHK_WORDS; i++) {
        *tmp++ = RHINO_TASK_STACK_OVF_MAGIC;
    }
#endif
#endif

    task->task_stack = cpu_task_stack_init(stack_buf, stack_size, arg, entry);

#if (RHINO_CONFIG_USER_HOOK > 0)
    krhino_task_create_hook(task);
#endif

    TRACE_TASK_CREATE(task);

    RHINO_CRITICAL_ENTER();

#if (RHINO_CONFIG_KOBJ_LIST > 0)
    klist_insert(&(g_kobj_list.task_head), &task->task_stats_item);
#endif

    if (autorun > 0u) {
        ready_list_add_tail(&g_ready_queue, task);
        /* if system is not start,not call core_sched */
        if (g_sys_stat == RHINO_RUNNING) {
            RHINO_CRITICAL_EXIT_SCHED();
            return RHINO_SUCCESS;
        }
    }

    RHINO_CRITICAL_EXIT();
    return RHINO_SUCCESS;
}

void workqueue_init(void)
{
    klist_init(&g_workqueue_list_head);

    krhino_workqueue_create(&g_workqueue_default, "DEFAULT-WORKQUEUE",
                            RHINO_CONFIG_WORKQUEUE_TASK_PRIO, g_workqueue_stack,
                            RHINO_CONFIG_WORKQUEUE_STACK_SIZE);
}

kstat_t krhino_workqueue_create(kworkqueue_t *workqueue, const name_t *name,
                                uint8_t pri, cpu_stack_t *stack_buf, size_t stack_size)
{
    CPSR_ALLOC();
    kstat_t ret;

    NULL_PARA_CHK(workqueue);
    NULL_PARA_CHK(name);
    NULL_PARA_CHK(stack_buf);

    if (pri >= RHINO_CONFIG_PRI_MAX) {
        return RHINO_BEYOND_MAX_PRI;
    }

    if (stack_size == 0u) {
        return RHINO_TASK_INV_STACK_SIZE;
    }

    ret = workqueue_is_exist(workqueue);
    if (ret == RHINO_WORKQUEUE_EXIST) {
        return RHINO_WORKQUEUE_EXIST;
    }

    klist_init(&(workqueue->workqueue_node));
    klist_init(&(workqueue->work_list));
    workqueue->work_current = NULL;
    workqueue->name      = name;

    ret = krhino_sem_create(&(workqueue->sem), "WORKQUEUE-SEM", 0);
    if (ret != RHINO_SUCCESS) {
        return ret;
    }

    RHINO_CRITICAL_ENTER();
    klist_insert(&g_workqueue_list_head, &(workqueue->workqueue_node));
    RHINO_CRITICAL_EXIT();

    ret = krhino_task_create(&(workqueue->worker), name, (void *)workqueue, pri,
                             0, stack_buf, stack_size, worker_task, 1);
    if (ret != RHINO_SUCCESS) {
        RHINO_CRITICAL_ENTER();
        klist_rm_init(&(workqueue->workqueue_node));
        RHINO_CRITICAL_EXIT();
        krhino_sem_del(&(workqueue->sem));
        return ret;
    }

    TRACE_WORKQUEUE_CREATE(krhino_cur_task_get(), workqueue);

    return RHINO_SUCCESS;
}

void ktimer_init(void)
{
    klist_init(&g_timer_head);

    krhino_fix_buf_queue_create(&g_timer_queue, "timer_queue", timer_queue_cb,
                                sizeof(k_timer_queue_cb), RHINO_CONFIG_TIMER_MSG_NUM);

    krhino_task_create(&g_timer_task, "timer_task", NULL,
                       RHINO_CONFIG_TIMER_TASK_PRI, 0u, g_timer_task_stack,
                       RHINO_CONFIG_TIMER_TASK_STACK_SIZE, timer_task, 1u);
}

void cpu_usage_stats_start(void)
{
    /* create a statistic task to calculate cpu usage */
    krhino_task_create(&g_cpu_usage_task, "cpu_stats", 0,
                       RHINO_CONFIG_CPU_USAGE_TASK_PRI,
                       0, g_cpu_task_stack, RHINO_CONFIG_CPU_USAGE_TASK_STACK, cpu_usage_task_entry,
                       1);
}

RHINO_INLINE void rhino_stack_check_init(void)
{
#if (RHINO_CONFIG_INTRPT_STACK_OVF_CHECK > 0)
#if (RHINO_CONFIG_CPU_STACK_DOWN > 0)
    g_intrpt_stack_bottom  = (cpu_stack_t *)RHINO_CONFIG_INTRPT_STACK_TOP;
    *g_intrpt_stack_bottom = RHINO_INTRPT_STACK_OVF_MAGIC;
#else
    g_intrpt_stack_top  = (cpu_stack_t *)RHINO_CONFIG_INTRPT_STACK_TOP;
    *g_intrpt_stack_top = RHINO_INTRPT_STACK_OVF_MAGIC;
#endif
#endif /* RHINO_CONFIG_INTRPT_STACK_OVF_CHECK */

#if (RHINO_CONFIG_STACK_OVF_CHECK_HW != 0)
    cpu_intrpt_stack_protect();
#endif
}

kstat_t krhino_task_dyn_create(ktask_t **task, const name_t *name, void *arg,
                               uint8_t pri, tick_t ticks, size_t stack,
                               task_entry_t entry, uint8_t autorun)
{
    return task_dyn_create(task, name, arg, pri, ticks, stack, entry, 0, 0, autorun, KSCHED_RR);
}

kstat_t krhino_start(void)
{
    ktask_t *preferred_task;

    if (g_sys_stat == RHINO_STOPPED) {
#if (RHINO_CONFIG_CPU_NUM > 1)
        for (uint8_t i = 0; i < RHINO_CONFIG_CPU_NUM; i++) {
            preferred_task            = preferred_cpu_ready_task_get(&g_ready_queue, i);
            preferred_task->cpu_num   = i;
            preferred_task->cur_exc   = 1;
            g_preferred_ready_task[i] = preferred_task;
            g_active_task[i]          = g_preferred_ready_task[i];
            g_active_task[i]->cur_exc = 1;
        }
#else
        preferred_task = preferred_cpu_ready_task_get(&g_ready_queue, 0);
        g_preferred_ready_task[0] = preferred_task;
        g_active_task[0] = preferred_task;
#endif

#if (RHINO_CONFIG_USER_HOOK > 0)
        krhino_start_hook();
#endif

        g_sys_stat = RHINO_RUNNING;
        cpu_first_task_start();

        /* should not be here */
        return RHINO_SYS_FATAL_ERR;
    }

    return RHINO_RUNNING;
}

ktask_t *preferred_cpu_ready_task_get(runqueue_t *rq, uint8_t cpu_num)
{
    klist_t *iter;
    ktask_t *task;
    uint32_t task_bit_map[NUM_WORDS];
    klist_t *node;
    uint8_t  flag;
    uint8_t  i;
    uint8_t  highest_pri = rq->highest_pri;

    node = rq->cur_list_item[highest_pri];
    iter = node;

    for (i = 0; i < NUM_WORDS; i++) {
        task_bit_map[i] = rq->task_bit_map[i];
    }

    while (1) {

        task = krhino_list_entry(iter, ktask_t, task_list);

        if (g_active_task[cpu_num] == task) {
            return task;
        }

        flag = ((task->cur_exc == 0) && (task->cpu_binded == 0))
               || ((task->cur_exc == 0) && (task->cpu_binded == 1) && (task->cpu_num == cpu_num));

        if (flag > 0) {
            return task;
        }

        if (iter->next == rq->cur_list_item[highest_pri]) {
            krhino_bitmap_clear(task_bit_map, highest_pri);
            highest_pri = krhino_bitmap_first(task_bit_map);
            iter = rq->cur_list_item[highest_pri];
        } else {
            iter = iter->next;
        }
    }
}

void krhino_start_hook(void)
{
}

cpu_first_task_start
    ;set PendSV prority to the lowest
    LDR     R0, =SHPR3_PRI_14
    LDR     R1, =PRI_LVL_PENDSV
    STRB    R1, [R0]

    ;set Systick prority to the lowest
    LDR     R0, =SHPR3_PRI_15
    LDR     R1, =PRI_LVL_SYSTICK
    STRB    R1, [R0]

    ;indicate PendSV_Handler branch to _pendsv_handler_nosave
    MOVS    R0, #0
    MSR     PSP, R0

    ;make PendSV exception pending
    LDR     R0, =SCB_ICSR
    LDR     R1, =ICSR_PENDSVSET
    STR     R1, [R0]

    ;goto PendSV_Handler
    CPSIE   I
    B       .

PendSV_Handler
    CPSID   I
    MRS     R0, PSP
    ;branch if cpu_first_task_start
    CMP     R0, #0
    BEQ     _first_task_restore

    ;hardware saved R0~R3,R12,LR,PC,xPSR

    ;save context
    SUBS    R0, R0, #0x24
    STM     R0, {R4-R11, LR}

    ;g_active_task->task_stack = context region
    LDR     R1, =g_active_task
    LDR     R1, [R1]
    STR     R0, [R1]

#if (RHINO_CONFIG_TASK_STACK_OVF_CHECK > 0)
    BL      krhino_stack_ovf_check
#endif
#if (RHINO_CONFIG_SYS_STATS > 0)
    BL      krhino_task_sched_stats_get
#endif

    kstat_t krhino_task_sleep(tick_t ticks)
{
    CPSR_ALLOC();
    uint8_t cur_cpu_num;
    kstat_t ret;

    if (ticks == 0u) {
        return RHINO_INV_PARAM;
    }

    RHINO_CRITICAL_ENTER();

    INTRPT_NESTED_LEVEL_CHK();

    cur_cpu_num = cpu_cur_get();

    /* system is locked so task can not be blocked just return immediately */
    if (g_sched_lock[cur_cpu_num] > 0u) {
        RHINO_CRITICAL_EXIT();
        return RHINO_SCHED_DISABLE;
    }

    g_active_task[cur_cpu_num]->task_state = K_SLEEP;
    tick_list_insert(g_active_task[cur_cpu_num], ticks);
    ready_list_rm(&g_ready_queue, g_active_task[cur_cpu_num]);

    TRACE_TASK_SLEEP(g_active_task[cur_cpu_num], ticks);

    RHINO_CRITICAL_EXIT_SCHED();

    RHINO_CPU_INTRPT_DISABLE();

    /* is task timeout normally after sleep */
    ret = pend_state_end_proc(g_active_task[cpu_cur_get()], NULL);

    RHINO_CPU_INTRPT_ENABLE();

    return ret;
}

#define CPSR_ALLOC()                cpu_cpsr_t cpsr

#define RHINO_CRITICAL_ENTER()      \
    do {                            \
        RHINO_CPU_INTRPT_DISABLE(); \
        RHINO_INTDIS_MEAS_START();  \
    } while (0)

#define INTRPT_NESTED_LEVEL_CHK()                        \
    do {                                                 \
        if (g_intrpt_nested_level[cpu_cur_get()] > 0u) { \
            RHINO_CRITICAL_EXIT();                       \
            return RHINO_NOT_CALLED_BY_INTRPT;           \
        }                                                \
    } while (0)

RHINO_INLINE uint8_t cpu_cur_get(void)
{
    return 0;
}

#define RHINO_CRITICAL_EXIT()       \
    do {                            \
        RHINO_INTDIS_MEAS_STOP();   \
        RHINO_CPU_INTRPT_ENABLE();  \
    } while (0)

void tick_list_insert(ktask_t *task, tick_t time)
{
    task->tick_match  = g_tick_count + time;
    task->tick_remain = time;
    tick_list_pri_insert(&g_tick_head, task);
}

RHINO_INLINE void tick_list_pri_insert(klist_t *head, ktask_t *task)
{
    tick_t   val;
    klist_t *q;
    klist_t *list_start;
    klist_t *list_end;
    ktask_t *task_iter_temp;

    list_start = head;
    list_end   = head;

    val = task->tick_remain;

    for (q = list_start->next; q != list_end; q = q->next) {
        task_iter_temp = krhino_list_entry(q, ktask_t, tick_list);
        if ((task_iter_temp->tick_match - g_tick_count) > val) {
            break;
        }
    }

    klist_insert(q, &task->tick_list);
}

void ready_list_rm(runqueue_t *rq, ktask_t *task)
{
    int32_t  i;
    uint8_t  pri = task->prio;

    TRACE_TASK_STOP_READY(task);

#if (RHINO_CONFIG_SCHED_CFS > 0)
    if (task->sched_policy == KSCHED_CFS) {
        if (g_active_task[cpu_cur_get()] != task) {
            cfs_node_del(&task->node);
        }
        return;
    }
#endif

    /* if the ready list is not only one, we do not need to update the highest prio */
    if ((rq->cur_list_item[pri]) != (rq->cur_list_item[pri]->next)) {
        if (rq->cur_list_item[pri] == &task->task_list) {
            rq->cur_list_item[pri] = rq->cur_list_item[pri]->next;
        }

        klist_rm(&task->task_list);
        return;
    }

    /* only one item,just set cur item ptr to NULL */
    rq->cur_list_item[pri] = NULL;

    krhino_bitmap_clear(rq->task_bit_map, pri);

    /* if task prio not equal to the highest prio, then we do not need to update the highest prio */
    /* this condition happens when a current high prio task to suspend a low priotity task */
    if (pri != rq->highest_pri) {
        return;
    }

    /* find the highest ready task */
    i = krhino_bitmap_first(rq->task_bit_map);

    /* update the next highest prio task */
    if (i >= 0) {
        rq->highest_pri = i;
    } else {
        k_err_proc(RHINO_SYS_FATAL_ERR);
    }
}

#define RHINO_CRITICAL_EXIT_SCHED() \
    do {                            \
        RHINO_INTDIS_MEAS_STOP();   \
        core_sched();               \
        cpu_intrpt_restore(cpsr);   \
    } while (0)

#define RHINO_CPU_INTRPT_DISABLE() do{ cpsr = cpu_intrpt_save(); }while(0)

.global cpu_intrpt_save
.type cpu_intrpt_save, %function
cpu_intrpt_save:
    csrr    a0, mstatus
    csrc    mstatus, 8
    ret

    kstat_t pend_state_end_proc(ktask_t *task, blk_obj_t *blk_obj)
{
    kstat_t status;

    (void)blk_obj;

    switch (task->blk_state) {
        case BLK_FINISH:
            status = RHINO_SUCCESS;
            break;
        case BLK_ABORT:
            status = RHINO_BLK_ABORT;
            break;
        case BLK_TIMEOUT:
            status = RHINO_BLK_TIMEOUT;
            break;
        case BLK_DEL:
            status = RHINO_BLK_DEL;
            break;
        default:
            k_err_proc(RHINO_BLK_INV_STATE);
            status = RHINO_BLK_INV_STATE;
            break;
    }

#if (RHINO_CONFIG_TASK_DEL > 0)
    if (blk_obj == NULL) {
        if (task->cancel == 1u) {
            status = RHINO_TASK_CANCELED;
        }
        return status;
    }

    if ((task->cancel == 1u) && (blk_obj->cancel == 1u)) {
        status = RHINO_TASK_CANCELED;
    }
#endif

    return status;
}

#define RHINO_CPU_INTRPT_ENABLE()  do{ cpu_intrpt_restore(cpsr); }while(0)

.global cpu_intrpt_restore
.type cpu_intrpt_restore, %function
cpu_intrpt_restore:
    csrw    mstatus, a0
    ret

static void debug_task_show(int (*print_func)(const char *fmt, ...), ktask_t *task)
{
    size_t        free_size;
    int           stat_idx;
    int           i;
    int           cpu_idx;
    char         *cpu_stat[] = { "UNK",      "RDY", "PEND",    "SUS",
                                 "PEND_SUS", "SLP", "SLP_SUS", "DEL"
                               };
    const name_t *task_name;

    char s_task_overview[] = "                              0x         0x      "
                             "   0x        (0x        )                        "
                             "           \r\n";

    if (krhino_task_stack_min_free(task, &free_size) != RHINO_SUCCESS) {
        free_size = 0;
    }
    free_size *= sizeof(cpu_stack_t);

    /* set name */
    task_name = task->task_name == NULL ? "anonym" : task->task_name;
    for (i = 0; i < 20; i++) {
        s_task_overview[i] = ' ';
    }
    for (i = 0; i < strlen(task_name); i++) {
        s_task_overview[i] = task_name[i];
    }

    /* set state */
    stat_idx = task->task_state >= sizeof(cpu_stat) / sizeof(char *)
               ? 0
               : task->task_state;
    for (i = 21; i < 29; i++) {
        s_task_overview[i] = ' ';
    }
    for (i = 21; i < 29; i++) {
        if (cpu_stat[stat_idx][i - 21] == '\0') {
            break;
        }
        s_task_overview[i] = cpu_stat[stat_idx][i - 21];
    }

    /* set stack priority */
    k_int2str(task->prio, &s_task_overview[32]);

    /* set stack info */
    k_int2str((int)task->task_stack_base, &s_task_overview[43]);
    k_int2str((int)task->stack_size * sizeof(cpu_stack_t),
              &s_task_overview[54]);
    k_int2str((int)free_size, &s_task_overview[65]);
    cpu_idx = 65 + 17;

    (void)cpu_idx;
#if (RHINO_CONFIG_CPU_NUM > 1)
    s_task_overview[cpu_idx] = (uint8_t)('0' + task->cpu_binded);
    s_task_overview[cpu_idx + 12] = (uint8_t)('0' + task->cpu_num);
    s_task_overview[cpu_idx + 23] = (uint8_t)('0' + task->cur_exc);
#endif

    /* print */
    if (print_func == NULL) {
        print_func = printf;
    }
    print_func(s_task_overview);

}

#define krhino_list_entry(node, type, member) ((type *)((uint8_t *)(node) - (size_t)(&((type *)0)->member)))


static const JSCFunctionListEntry js_log_funcs[] = {
    JS_CFUNC_DEF("debug", 0, native_debug_log_out ),
    JS_CFUNC_DEF("info", 0, native_info_log_out ),
    JS_CFUNC_DEF("warn", 0, native_warn_log_out ),
    JS_CFUNC_DEF("error", 0, native_error_log_out),
    JS_CFUNC_DEF("fatal", 0, native_fatal_log_out),
    JS_CFUNC_DEF("stdloglevel", 1, native_set_stdlog_level ),
    JS_CFUNC_DEF("cloudloglevel", 1, native_set_popcloud_log_level ),
    JS_CFUNC_DEF("fsloglevel", 1, native_set_popfs_log_level ),
    JS_CFUNC_DEF("setlogfilepath", 1, native_set_log_file_path ),
    JS_CFUNC_DEF("setlogfilesize", 1, native_set_log_file_size ),
};

