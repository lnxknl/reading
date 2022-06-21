
(void)execve(samplePath, NULL, NULL);

void exit(int status)
{
    PRINT_ERR("%s NOT SUPPORT\n", __FUNCTION__);
    errno = ENOSYS;
    while (1);
}

///用户态异常处理函数
STATIC VOID OsUserExcHandle(ExcContext *excBufAddr)
{
    UINT32 intSave;
    UINT32 currCpu = ArchCurrCpuid();
    LosTaskCB *runTask = OsCurrTaskGet();
    LosProcessCB *runProcess = OsCurrProcessGet();

    if (g_excFromUserMode[ArchCurrCpuid()] == FALSE) { //内核态直接退出,不处理了.
        return;
    }

#ifdef LOSCFG_KERNEL_SMP
    LOS_SpinLock(&g_excSerializerSpin);
    if (g_nextExcWaitCpu != INVALID_CPUID) {
        g_currHandleExcCpuid = g_nextExcWaitCpu;
        g_nextExcWaitCpu = INVALID_CPUID;
    } else {
        g_currHandleExcCpuid = INVALID_CPUID;
    }
    g_currHandleExcPID = OS_INVALID_VALUE;
    LOS_SpinUnlock(&g_excSerializerSpin);
#else
    g_currHandleExcCpuid = INVALID_CPUID;
#endif

#ifdef LOSCFG_KERNEL_SMP
#ifdef LOSCFG_FS_VFS
    OsWakeConsoleSendTask();
#endif
#endif

#ifdef LOSCFG_BLACKBOX
    BBoxNotifyError("USER_CRASH", MODULE_SYSTEM, "Crash in user", 0);
#endif
    SCHEDULER_LOCK(intSave);
#ifdef LOSCFG_SAVE_EXCINFO
    OsProcessExitCodeCoreDumpSet(runProcess);
#endif
    OsProcessExitCodeSignalSet(runProcess, SIGUSR2);

    /* An exception was raised by a task that is not the current main thread during the exit process of
     * the current process.
     */
    if (runProcess->processStatus & OS_PROCESS_FLAG_EXIT) {
        SCHEDULER_UNLOCK(intSave);
        /* Exception handling All operations should be kept prior to that operation */
        OsExcRestore();
        OsRunningTaskToExit(runTask, OS_PRO_EXIT_OK);
    } else {
        SCHEDULER_UNLOCK(intSave);

        /* Exception handling All operations should be kept prior to that operation */
        OsExcRestore();
        /* kill user exc process */
        LOS_Exit(OS_PRO_EXIT_OK); //进程退出
    }

    /* User mode exception handling failed , which normally does not exist */ //用户态的异常处理失败，通常情况下不会发生
    g_curNestCount[currCpu]++;
    g_intCount[currCpu]++;
    PrintExcInfo("User mode exception ends unscheduled!\n");
}

#define OS_STRING(x) #x
#define X_STRING(x) OS_STRING(x)

/* type definitions */
typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned int UINT32;
typedef signed char INT8;
typedef signed short INT16;
typedef signed int INT32;
typedef float FLOAT;
typedef double DOUBLE;
typedef char CHAR;
typedef unsigned long u_long;
typedef u_long ULong;
typedef long Long;

#ifdef __LP64__
typedef long unsigned int UINT64;
typedef long signed int INT64;
typedef unsigned long UINTPTR;
typedef signed long INTPTR;
#else
typedef unsigned long long UINT64;
typedef signed long long INT64;
typedef unsigned int UINTPTR;
typedef signed int INTPTR;
#endif

#ifdef __LP64__
typedef __uint128_t UINT128;
typedef INT64 ssize_t;
typedef UINT64 size_t;
#define LOSCFG_AARCH64
#else
typedef INT32 ssize_t;
typedef UINT32 size_t;
#endif

typedef UINTPTR AARCHPTR;
typedef size_t BOOL;

#define VOID void
#define STATIC static

#ifndef FALSE
#define FALSE 0U
#endif

#ifndef TRUE
#define TRUE 1U
#endif

#ifndef NULL
#define NULL ((VOID *)0)
#endif

#define OS_NULL_BYTE ((UINT8)0xFF)
#define OS_NULL_SHORT ((UINT16)0xFFFF)
#define OS_NULL_INT ((UINT32)0xFFFFFFFF)

#ifndef USER
#define USER
#endif

#ifndef LOS_OK
#define LOS_OK 0
#endif

#ifndef LOS_NOK
#define LOS_NOK 1
#endif

#ifndef LOS_EPERM
#define LOS_EPERM 1
#endif

#ifndef LOS_ESRCH
#define LOS_ESRCH 3
#endif

#ifndef LOS_ECHILD
#define LOS_ECHILD 10
#endif

#ifndef LOS_EINVAL
#define LOS_EINVAL 22
#endif

#ifndef LOS_EOPNOTSUPP
#define LOS_EOPNOTSUPP 95
#endif

#define OS_FAIL 1
#define OS_ERROR (UINT32)(-1)
#define OS_INVALID (UINT32)(-1)
#define OS_INVALID_VALUE ((UINT32)0xFFFFFFFF)

#define asm __asm
#ifdef typeof
#undef typeof
#endif
#define typeof __typeof__

#ifndef LOS_LABEL_DEFN
#define LOS_LABEL_DEFN(label) label
#endif

#ifndef LOSARC_ALIGNMENT
#define LOSARC_ALIGNMENT 8
#endif
/* And corresponding power of two alignment */
#ifndef LOSARC_P2ALIGNMENT
#ifdef LOSCFG_AARCH64
#define LOSARC_P2ALIGNMENT 3
#else
#define LOSARC_P2ALIGNMENT 2
#endif
#endif

#define PAGE_SIZE_SHIFT (12)

#ifndef PAGE_SIZE
#define PAGE_SIZE (1UL << PAGE_SIZE_SHIFT)
#endif
#define USER_PAGE_SIZE (1UL << 12)

#if ARM64_CPU_CORTEX_A53 || ARM64_CPU_CORTEX_A57 || ARM64_CPU_CORTEX_A72
#define CACHE_LINE 64
#else
#define CACHE_LINE 32
#endif

typedef int status_t;
typedef unsigned long vaddr_t;
typedef unsigned long paddr_t;
typedef unsigned int uint;
typedef unsigned long pte_t;

/* Give a type or object explicit minimum alignment */
#if !defined(LOSBLD_ATTRIB_ALIGN)
#define LOSBLD_ATTRIB_ALIGN(__align__) __attribute__((aligned(__align__)))
#endif

/* Assign a defined variable to a specific section */
#if !defined(LOSBLD_ATTRIB_SECTION)
#define LOSBLD_ATTRIB_SECTION(__sect__) __attribute__((section(__sect__)))
#endif

/*
 * Tell the compiler not to throw away a variable or function. Only known
 * available on 3.3.2 or above. Old version's didn't throw them away,
 * but using the unused attribute should stop warnings.
 */
#define LOSBLD_ATTRIB_USED __attribute__((used))


#endif /* _LOS_TYPEDEF_H */


///找到一个合适的栈
STATIC INLINE BOOL FindSuitableStack(UINTPTR regFP, UINTPTR *start, UINTPTR *end, vaddr_t *vaddr)
{
    UINT32 index, stackStart, stackEnd;
    BOOL found = FALSE;
    LosTaskCB *taskCB = NULL;
    const StackInfo *stack = NULL;
    vaddr_t kvaddr;

    if (g_excFromUserMode[ArchCurrCpuid()] == TRUE) {//当前CPU在用户态执行发生异常
        taskCB = OsCurrTaskGet();
        stackStart = taskCB->userMapBase;                     //用户态栈基地址,即用户态栈顶
        stackEnd = taskCB->userMapBase + taskCB->userMapSize; //用户态栈结束地址
        if (IsValidFP(regFP, stackStart, stackEnd, &kvaddr) == TRUE) {
            found = TRUE;
            goto FOUND;
        }
        return found;
    }

    /* Search in the task stacks | 找任务的内核态栈*/
    for (index = 0; index < g_taskMaxNum; index++) {
        taskCB = &g_taskCBArray[index];
        if (OsTaskIsUnused(taskCB)) {
            continue;
        }

        stackStart = taskCB->topOfStack;                   //内核态栈顶
        stackEnd = taskCB->topOfStack + taskCB->stackSize; //内核态栈底
        if (IsValidFP(regFP, stackStart, stackEnd, &kvaddr) == TRUE) {
            found = TRUE;
            goto FOUND;
        }
    }

    /* Search in the exc stacks */ //从异常栈中找
    for (index = 0; index < sizeof(g_excStack) / sizeof(StackInfo); index++) {
        stack = &g_excStack[index];
        stackStart = (UINTPTR)stack->stackTop;
        stackEnd = stackStart + LOSCFG_KERNEL_CORE_NUM * stack->stackSize;
        if (IsValidFP(regFP, stackStart, stackEnd, &kvaddr) == TRUE) {
            found = TRUE;
            goto FOUND;
        }
    }

FOUND:
    if (found == TRUE) {
        *start = stackStart;
        *end = stackEnd;
        *vaddr = kvaddr;
    }

    return found;
}

///打印调用栈信息
VOID OsCallStackInfo(VOID)
{
    UINT32 count = 0;
    LosTaskCB *runTask = OsCurrTaskGet();
    UINTPTR stackBottom = runTask->topOfStack + runTask->stackSize; //内核态的 栈底 = 栈顶 + 大小
    UINT32 *stackPointer = (UINT32 *)stackBottom;

    PrintExcInfo("runTask->stackPointer = 0x%x\n"
                 "runTask->topOfStack = 0x%x\n"
                 "text_start:0x%x,text_end:0x%x\n",
                 stackPointer, runTask->topOfStack, &__text_start, &__text_end);
    //打印OS_MAX_BACKTRACE多一条栈信息,注意stack中存放的是函数调用地址和指令的地址
    while ((stackPointer > (UINT32 *)runTask->topOfStack) && (count < OS_MAX_BACKTRACE)) {
        if ((*stackPointer > (UINTPTR)(&__text_start)) && //正常情况下 sp的内容都是文本段的内容
            (*stackPointer < (UINTPTR)(&__text_end)) &&
            IS_ALIGNED((*stackPointer), POINTER_SIZE)) { //sp的内容是否对齐, sp指向指令的地址
            if ((*(stackPointer - 1) > (UINT32)runTask->topOfStack) &&
                (*(stackPointer - 1) < stackBottom) && //@note_why 这里为什么要对 stackPointer - 1 进行判断
                IS_ALIGNED((*(stackPointer - 1)), POINTER_SIZE)) {
                count++;
                PrintExcInfo("traceback %u -- lr = 0x%x\n", count, *stackPointer);
            }
        }
        stackPointer--;
    }
    PRINTK("\n");
}

typedef struct {
    SortLinkAttribute taskSortLink;          /* task sort link */
    UINT64            responseTime;          /* Response time for current CPU tick interrupts */
    UINT32            responseID;            /* The response ID of the current CPU tick interrupt */
    UINT32            idleTaskID;            /* idle task id */
    UINT32            taskLockCnt;           /* task lock flag */
    UINT32            schedFlag;             /* pending scheduler flag */
} SchedRunQue;

extern SchedRunQue g_schedRunQue[LOSCFG_KERNEL_CORE_NUM];

VOID OsSchedUpdateExpireTime(VOID);

STATIC INLINE SchedRunQue *OsSchedRunQue(VOID)
{
    return &g_schedRunQue[ArchCurrCpuid()];
}

// //定义用于读取或写入异常信息的指针函数类型
typedef VOID (*log_read_write_fn)(UINT32 startAddr, UINT32 space, UINT32 rwFlag, CHAR *buf);

///获取异常信息读写函数
log_read_write_fn GetExcInfoRW(VOID)
{
    return g_excInfoRW;
}

STATIC EXC_PROC_FUNC g_excHook = (EXC_PROC_FUNC)OsExcHook; ///< 全局异常处理钩子

VOID OsExcHook(UINT32 excType, ExcContext *excBufAddr, UINT32 far, UINT32 fsr)
{                                             //参考文档 https://gitee.com/openharmony/docs/blob/master/kernel/%E7%94%A8%E6%88%B7%E6%80%81%E5%BC%82%E5%B8%B8%E4%BF%A1%E6%81%AF%E8%AF%B4%E6%98%8E.md
    OsExcType(excType, excBufAddr, far, fsr); //1.打印异常的类型
    OsExcSysInfo(excType, excBufAddr);        //2.打印异常的基本信息
    OsExcRegsInfo(excBufAddr);                //3.打印异常的寄存器信息

    BackTrace(excBufAddr->R11); //4.打印调用栈信息,

    (VOID) OsShellCmdTskInfoGet(OS_ALL_TASK_MASK, NULL, OS_PROCESS_INFO_ALL); //打印进程线程基本信息 相当于执行下 shell task -a 命令

#ifndef LOSCFG_DEBUG_VERSION //打开debug开关
    if (g_excFromUserMode[ArchCurrCpuid()] != TRUE) {
#endif
#ifdef LOSCFG_KERNEL_VM
        OsDumpProcessUsedMemNode(OS_EXC_VMM_NO_REGION);
#endif
        OsExcStackInfo(); //	打印任务栈的信息
#ifndef LOSCFG_DEBUG_VERSION
    }
#endif

    OsDumpContextMem(excBufAddr); // 打印上下文

    (VOID) OsShellCmdMemCheck(0, NULL); //检查内存,相当于执行 shell memcheck 命令

#ifdef LOSCFG_COREDUMP
    LOS_CoreDumpV2(excType, excBufAddr);
#endif

    OsUserExcHandle(excBufAddr); //用户态下异常的处理
}


UINT32 BackTraceGet(UINTPTR regFP, IpInfo *callChain, UINT32 maxDepth)
{
    UINTPTR tmpFP, backLR;
    UINTPTR stackStart, stackEnd;
    UINTPTR backFP = regFP;
    UINT32 count = 0;
    BOOL ret;
    VADDR_T kvaddr;

    if (FindSuitableStack(regFP, &stackStart, &stackEnd, &kvaddr) == FALSE) {
        if (callChain == NULL) {
            PrintExcInfo("traceback error fp = 0x%x\n", regFP);
        }
        return 0;
    }

    /*
     * Check whether it is the leaf function.
     * Generally, the frame pointer points to the address of link register, while in the leaf function,
     * there's no function call, and compiler will not store the link register, but the frame pointer
     * will still be stored and updated. In that case we needs to find the right position of frame pointer.
     */
    tmpFP = *(UINTPTR *)(UINTPTR)kvaddr;
    if (IsValidFP(tmpFP, stackStart, stackEnd, NULL) == TRUE) {
        backFP = tmpFP;
        if (callChain == NULL) {
            PrintExcInfo("traceback fp fixed, trace using   fp = 0x%x\n", backFP);
        }
    }

    while (IsValidFP(backFP, stackStart, stackEnd, &kvaddr) == TRUE) {
        tmpFP = backFP;
#ifdef LOSCFG_COMPILER_CLANG_LLVM
        backFP = *(UINTPTR *)(UINTPTR)kvaddr;
        if (IsValidFP(tmpFP + POINTER_SIZE, stackStart, stackEnd, &kvaddr) == FALSE) {
            if (callChain == NULL) {
                PrintExcInfo("traceback backLR check failed, backLP: 0x%x\n", tmpFP + POINTER_SIZE);
            }
            return 0;
        }
        backLR = *(UINTPTR *)(UINTPTR)kvaddr;
#else
        backLR = *(UINTPTR *)(UINTPTR)kvaddr;
        if (IsValidFP(tmpFP - POINTER_SIZE, stackStart, stackEnd, &kvaddr) == FALSE) {
            if (callChain == NULL) {
                PrintExcInfo("traceback backFP check failed, backFP: 0x%x\n", tmpFP - POINTER_SIZE);
            }
            return 0;
        }
        backFP = *(UINTPTR *)(UINTPTR)kvaddr;
#endif
        IpInfo info = {0};
        ret = OsGetUsrIpInfo((VADDR_T)backLR, &info);
        if (callChain == NULL) {
            PrintExcInfo("traceback %u -- lr = 0x%x    fp = 0x%x ", count, backLR, backFP);
            if (ret) {
#ifdef LOSCFG_KERNEL_VM
                PrintExcInfo("lr in %s --> 0x%x\n", info.f_path, info.ip);
#else
                PrintExcInfo("\n");
#endif
            } else {
                PrintExcInfo("\n");
            }
        } else {
            (VOID)memcpy_s(&callChain[count], sizeof(IpInfo), &info, sizeof(IpInfo));
        }
        count++;
        if ((count == maxDepth) || (backFP == tmpFP)) {
            break;
        }
    }
    return count;
}


///打印调用栈信息
VOID OsCallStackInfo(VOID)
{
    UINT32 count = 0;
    LosTaskCB *runTask = OsCurrTaskGet();
    UINTPTR stackBottom = runTask->topOfStack + runTask->stackSize; //内核态的 栈底 = 栈顶 + 大小
    UINT32 *stackPointer = (UINT32 *)stackBottom;

    PrintExcInfo("runTask->stackPointer = 0x%x\n"
                 "runTask->topOfStack = 0x%x\n"
                 "text_start:0x%x,text_end:0x%x\n",
                 stackPointer, runTask->topOfStack, &__text_start, &__text_end);
    //打印OS_MAX_BACKTRACE多一条栈信息,注意stack中存放的是函数调用地址和指令的地址
    while ((stackPointer > (UINT32 *)runTask->topOfStack) && (count < OS_MAX_BACKTRACE)) {
        if ((*stackPointer > (UINTPTR)(&__text_start)) && //正常情况下 sp的内容都是文本段的内容
            (*stackPointer < (UINTPTR)(&__text_end)) &&
            IS_ALIGNED((*stackPointer), POINTER_SIZE)) { //sp的内容是否对齐, sp指向指令的地址
            if ((*(stackPointer - 1) > (UINT32)runTask->topOfStack) &&
                (*(stackPointer - 1) < stackBottom) && //@note_why 这里为什么要对 stackPointer - 1 进行判断
                IS_ALIGNED((*(stackPointer - 1)), POINTER_SIZE)) {
                count++;
                PrintExcInfo("traceback %u -- lr = 0x%x\n", count, *stackPointer);
            }
        }
        stackPointer--;
    }
    PRINTK("\n");
}

/***********************************************
R11寄存器(frame pointer)
在程序执行过程中（通常是发生了某种意外情况而需要进行调试），通过SP和FP所限定的stack frame，
就可以得到母函数的SP和FP，从而得到母函数的stack frame（PC，LR，SP，FP会在函数调用的第一时间压栈），
以此追溯，即可得到所有函数的调用顺序。
***********************************************/
VOID OsTaskBackTrace(UINT32 taskID) //任务栈信息追溯
{
    LosTaskCB *taskCB = NULL;

    if (OS_TID_CHECK_INVALID(taskID)) {
        PRINT_ERR("\r\nTask ID is invalid!\n");
        return;
    }
    taskCB = OS_TCB_FROM_TID(taskID);
    if (OsTaskIsUnused(taskCB) || (taskCB->taskEntry == NULL)) {
        PRINT_ERR("\r\nThe task is not created!\n");
        return;
    }
    PRINTK("TaskName = %s\n", taskCB->taskName);
    PRINTK("TaskID = 0x%x\n", taskCB->taskID);
    BackTrace(((TaskContext *)(taskCB->stackPointer))->R11); /* R11 : FP */
}

