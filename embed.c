    uint32_t nsec = (tick % 100000) * 10000;


struct timespec
{
#ifdef __USE_TIME_BITS64
  __time64_t tv_sec;		/* Seconds.  */
#else
  __time_t tv_sec;		/* Seconds.  */
#endif
#if __WORDSIZE == 64 \
  || (defined __SYSCALL_WORDSIZE && __SYSCALL_WORDSIZE == 64) \
  || (__TIMESIZE == 32 && !defined __USE_TIME_BITS64)
  __syscall_slong_t tv_nsec;	/* Nanoseconds.  */
#else
# if __BYTE_ORDER == __BIG_ENDIAN
  int: 32;           /* Padding.  */
  long int tv_nsec;  /* Nanoseconds.  */
# else
  long int tv_nsec;  /* Nanoseconds.  */
  int: 32;           /* Padding.  */
# endif
#endif
};

#endif

//Padding aligns structure members to "natural" address boundaries - say, int members would have offsets, which are mod(4) == 0 on 32-bit platform. Padding is on by default. It inserts the following "gaps" into your first structure:

struct mystruct_A {
    char a;
    char gap_0[3]; /* inserted by compiler: for alignment of b */
    int b;
    char c;
    char gap_1[3]; /* -"-: for alignment of the whole struct in an array */
} x;
Packing, on the other hand prevents compiler from doing padding - this has to be explicitly requested - under GCC it's __attribute__((__packed__)), so the following:

struct __attribute__((__packed__)) mystruct_A {
    char a;
    int b;
    char c;
};
/*
would produce structure of size 6 on a 32-bit architecture.

A note though - unaligned memory access is slower on architectures that allow it (like x86 and amd64), and is explicitly prohibited on strict alignment architectures like SPARC.

*/




// size is 8, 4 + 1, then round to multiple of 4 (int's size),
struct stu_a {
    int i;
    char c;
};

// size is 16, 8 + 1, then round to multiple of 8 (long's size),
struct stu_b {
    long l;
    char c;
};

// size is 24, l need padding by 4 before it, then round to multiple of 8 (long's size),
struct stu_c {
    int i;
    long l;
    char c;
};

// size is 16, 8 + 4 + 1, then round to multiple of 8 (long's size),
struct stu_d {
    long l;
    int i;
    char c;
};

// size is 16, 8 + 4 + 1, then round to multiple of 8 (double's size),
struct stu_e {
    double d;
    int i;
    char c;
};

// size is 24, d need align to 8, then round to multiple of 8 (double's size),
struct stu_f {
    int i;
    double d;
    char c;
};

// size is 4,
struct stu_g {
    int i;
};

// size is 8,
struct stu_h {
    long l;
};
// test - padding within a single struct,
int test_struct_padding() {
    printf("%s: %ld\n", "stu_a", sizeof(struct stu_a));
    printf("%s: %ld\n", "stu_b", sizeof(struct stu_b));
    printf("%s: %ld\n", "stu_c", sizeof(struct stu_c));
    printf("%s: %ld\n", "stu_d", sizeof(struct stu_d));
    printf("%s: %ld\n", "stu_e", sizeof(struct stu_e));
    printf("%s: %ld\n", "stu_f", sizeof(struct stu_f));

    printf("%s: %ld\n", "stu_g", sizeof(struct stu_g));
    printf("%s: %ld\n", "stu_h", sizeof(struct stu_h));

    return 0;
}

// test - address of struct,
int test_struct_address() {
    printf("%s: %ld\n", "stu_g", sizeof(struct stu_g));
    printf("%s: %ld\n", "stu_h", sizeof(struct stu_h));
    printf("%s: %ld\n", "stu_f", sizeof(struct stu_f));

    struct stu_g g;
    struct stu_h h;
    struct stu_f f1;
    struct stu_f f2;
    int x = 1;
    long y = 1;

    printf("address of %s: %p\n", "g", &g);
    printf("address of %s: %p\n", "h", &h);
    printf("address of %s: %p\n", "f1", &f1);
    printf("address of %s: %p\n", "f2", &f2);
    printf("address of %s: %p\n", "x", &x);
    printf("address of %s: %p\n", "y", &y);

    // g is only 4 bytes itself, but distance to next struct is 16 bytes(on 64 bit system) or 8 bytes(on 32 bit system),
    printf("space between %s and %s: %ld\n", "g", "h", (long)(&h) - (long)(&g));

    // h is only 8 bytes itself, but distance to next struct is 16 bytes(on 64 bit system) or 8 bytes(on 32 bit system),
    printf("space between %s and %s: %ld\n", "h", "f1", (long)(&f1) - (long)(&h));

    // f1 is only 24 bytes itself, but distance to next struct is 32 bytes(on 64 bit system) or 24 bytes(on 32 bit system),
    printf("space between %s and %s: %ld\n", "f1", "f2", (long)(&f2) - (long)(&f1));

    // x is not a struct, and it reuse those empty space between struts, which exists due to padding, e.g between g & h,
    printf("space between %s and %s: %ld\n", "x", "f2", (long)(&x) - (long)(&f2));
    printf("space between %s and %s: %ld\n", "g", "x", (long)(&x) - (long)(&g));

    // y is not a struct, and it reuse those empty space between struts, which exists due to padding, e.g between h & f1,
    printf("space between %s and %s: %ld\n", "x", "y", (long)(&y) - (long)(&x));
    printf("space between %s and %s: %ld\n", "h", "y", (long)(&y) - (long)(&h));

    return 0;
}

int main(int argc, char * argv[]) {
    test_struct_padding();
    // test_struct_address();

    return 0;
}


//test
[tong@free ~/work/test]$ ./a2
stu_a size: 24
stu_b size: 16
[tong@free ~/work/test]$ cat a2.c

#include<stdio.h>
struct stu_a{
        int a;
        long b;
        char c;
}s1;

struct stu_b{
        long b;
        int a;
        char c;
}s2;
int main()
{
        printf("stu_a size: %ld\n",sizeof(s1));
        printf("stu_b size: %ld\n",sizeof(s2));

        return 0;
}


//bitfield

struct name { int a:16; }
It means a is defined as 16-bit memory space. The remaining bits (16 bits) from int can be used to defined another variable, say b, like this:

struct name { int a:16;  int b:16; }
So if int is 32-bit (4 bytes), then the memory of one int is divided into two variables a and b.

PS: I'm assuming sizeof(int) = 4 bytes, and 1 byte = 8 bits


//test
[tong@free ~/work/test]$ ./a3
stu size: 4
[tong@free ~/work/test]$ cat a3.c

#include<stdio.h>
struct s
    {
     int a:1;
     int b:2;
     int c:7;
    }s0;

int main()
{
        printf("stu size: %ld\n",sizeof(s0));
        return 0;
}


[tong@free ~/work/test]$ ./a3
stu size: 8
[tong@free ~/work/test]$ cat a3.c

#include<stdio.h>
struct s
    {
     int :20;
     int a:1;
     int b:2;
     int c:7;
     int :5;
    }s0;

int main()
{
        printf("stu size: %ld\n",sizeof(s0));
        return 0;
}



void Usart_Rx_Task(TSS_TaskMessageDef *p)
{
    Log_printf("Usart", "(%d)%s", RxBufferLen, RxBuffer);
    int8_t ret = TSS_CommandProcess(RxBuffer, RxBufferLen);
    if (ret == TSS_NONEAT)
    {
        Log_printf("AT", "At without this command!\r\n");
    }
    else if (ret == TSS_ATERROR)
    {
        Log_printf("AT", "At command error!\r\n");
    }
    memset(RxBuffer, 0, BUFFER_MAX);
    RxBufferLen = 0;
}


static uint16_t RxBufferLen = 0;

#define BUFFER_MAX 512


void *usart_thread_function(void *arg)
{
    while (1)
    {
        /* Read one byte from the receive data register */
        RxBuffer[RxBufferLen] = getchar();
        if (RxBufferLen < BUFFER_MAX)
            RxBufferLen++;
        //延时触发接收任务
        TSS_ThreadSetTrigger(USART_RX_TASK_ID, 5);
    }
    return NULL;
}

uint8_t USART_RX_TASK_ID = 0;


/**
  * @brief  触发一个异步任务的执行
  * @param  ThreadID    任务ID
  * @param  Delay       0，任务立刻执行；>0，延时一定时间执行
  * @retval 无
  * @note   需要立刻执行的任务会以最快的速度得到执行
            不是立刻执行的任务会被刷新执行时间
  */
void TSS_ThreadSetTrigger(uint8_t ThreadID, int32_t Delay)
{
    if (TSS_TASKLIST.TASK_List[ThreadID].Types == TASK_TRIGGER)
    {
        //如果是一个需要立刻执行的任务
        if (Delay <= 0)
        {
#if TSS_TRIGGERNOW == 1 //这种触发方式实时性较好，但是在中断中调用会影响实时性
            TSS_TASKLIST.TASK_List[ThreadID].Delay[DELAY_TICK] = TSS_TASKLIST.TASK_List[ThreadID].Delay[DELAY_CYCLE];
            TSS_TASKLIST.TASK_List[ThreadID].Status = TASK_Running;
            TSS_TASKLIST.TASK_List[ThreadID].Callback(NULL);
#else //这种方式实时性一般，但是依然能保证任务得到执行
            TSS_TASKLIST.TASK_List[ThreadID].Status = TASK_Ready;
            TSS_TASKLIST.TASK_List[ThreadID].Delay[DELAY_TICK] = 0;
#endif
        }
        else
        {
            TSS_TASKLIST.TASK_List[ThreadID].Delay[DELAY_TICK] = Delay;
        }
    }
}


TSS_TaskList TSS_TASKLIST = {
    .TASK_Count = 1,
};


    /**
	 * @brief 任务列表结构体
	 */
    typedef struct
    {
        uint8_t TASK_Count;            //任务计数
        volatile uint32_t TSS_RunTime; //系统运行时间
        TSS_TaskInfoDef TASK_List[TSS_TASKNUMMAX];
    } TSS_TaskList;


    /**
	 * @brief 任务信息结构体
	 */
    typedef struct
    {
        uint8_t ID;                 //任务序号
        volatile int32_t Delay[2];  //任务剩余延时时间
        volatile uint8_t Status;    //任务状态
        uint8_t Priority;           //优先级
        TSS_TaskTypes Types;        //任务类型
        TSS_TaskFunction Callback;  //任务函数指针
        TSS_TaskMessageDef Message; //任务消息
    } TSS_TaskInfoDef;


    /**
	 * @brief 任务类型枚举定义
	 */
    typedef enum
    {
        TASK_HZ = 0,  //按照设定的执行频率不断执行
        TASK_TRIGGER, //触发异步执行
    } TSS_TaskTypes;


    typedef void (*TSS_TaskFunction)(TSS_TaskMessageDef *);

        /**
	 * @brief 任务消息结构体
	 */
    typedef struct
    {
        int32_t past;     //任务延迟时间
        uint8_t effect;   //消息有效位，消息发出后为1，任务执行一次置0
        uint32_t message; //消息
        void *data;       //数据指针
    } TSS_TaskMessageDef;


static void TSS_ActionTask(TSS_TaskMessageDef *p)
{
    TSS_ActionPlayerDef *Action = (TSS_ActionPlayerDef *)p->data;

    if (Action->status != TSS_ACTION_BLOCK)
    {
        uint16_t node = Action->play_node;
        void *parameter = Action->table[node].parameter;
        switch (Action->status)
        {
        case TSS_ACTION_START:
            if (Action->table[node].start_fun != NULL)
            {
                Action->table[node].start_fun(parameter);
            }
            Action->status = TSS_ACTION_RUN;
            break;
        case TSS_ACTION_RUN:
            if (Action->table[node].condition_fun != NULL)
            {
                int8_t condition = Action->table[node].condition_fun(parameter);
                if (condition == TSS_ACTIONFUN_OK)
                {
                    if (TSS_GetRunTime() - Action->play_time > Action->table[node].cycle)
                    {
                        Action->play_time = TSS_GetRunTime();
                        Action->table[node].run_fun(parameter);
                    }
                }
                else if (condition == TSS_ACTIONFUN_END)
                {
                    Action->status = TSS_ACTION_END;
                }
            }
            else
            {
                Action->status = TSS_ACTION_END;
            }
            break;
        case TSS_ACTION_END:
            if (Action->table[node].end_fun != NULL)
            {
                Action->table[node].end_fun(parameter);
            }
     uint8_t TSS_ThreadCreate(TSS_TaskFunction Callback, uint8_t Priority, TSS_TaskTypes Types, int32_t Delay);
           Action->status = TSS_ACTION_START;

            //动作播放结束
            Action->play_node++;
            if (Action->play_node >= Action->play_len)
            {
                Action->times--;
                if (Action->times == 0)
                {
                    Action->status = TSS_ACTION_BLOCK;
                }
                else
                {
                    Action->play_node = 0;
                    Action->play_time = 0;
                    Action->status = TSS_ACTION_START;
                }
            }
            break;
        default:
            break;
        }
    }
}


    TSS_ACTIONTASK_ID = TSS_ThreadCreate(TSS_ActionTask, Priority, TASK_HZ, 10);

    static void TSS_ActionTask(TSS_TaskMessageDef *p)


uint8_t TSS_ThreadCreate(TSS_TaskFunction Callback, uint8_t Priority, TSS_TaskTypes Types, int32_t Delay);


void Hex2Ascii(uint8_t *Data, uint16_t Len, uint8_t *Str)
{
    // sprintf((char *)Str, "0x");
    Str[0] = 0;
    for (uint16_t i = 0; i < Len; i++)
    {
        sprintf((char *)&Str[strlen((char *)Str)], "%2X ", Data[i]);
    }
}


    //动作播放器状态
    typedef struct
    {
        uint8_t status;            //状态标志
        uint32_t play_time;        //上次运行函数执行时间
        uint16_t play_node;        //当前播放节点
        TSS_ActionTableDef *table; //需要播放的动作列表
        uint16_t play_len;         //动作组长度
        uint32_t times;            //重复循环次数
    } TSS_ActionPlayerDef;


int8_t TSS_ActionStart(TSS_ActionPlayerDef *Action, TSS_ActionTableDef *Table, uint16_t Len, uint32_t Times)
{
    if (Times == 0)
        Times = 1;
    if (Action->status == TSS_ACTION_BLOCK)
    {
        memset((void *)Action, 0, sizeof(TSS_ActionPlayerDef));
        Action->table = Table;
        Action->play_len = Len;
        Action->status = TSS_ACTION_START;
        Action->times = Times;
        return TSS_OK;
    }
    return TSS_ERROR;
}



void TSS_Start(void)
{
    uint8_t i;
    TSS_TaskInfoDef *ReadyTask = NULL;
    while (1)
    {
        ReadyTask = &TSS_TASKLIST.TASK_List[0];
        //检索一遍任务
        for (i = 1; i < TSS_TASKLIST.TASK_Count; i++)
        {
            //找到需要执行的任务
            if (TSS_TASKLIST.TASK_List[i].Status == TASK_Ready && TSS_TASKLIST.TASK_List[i].Priority >= ReadyTask->Priority)
            {
                //找出优先级最高的任务
                ReadyTask = &TSS_TASKLIST.TASK_List[i];
            }
        }
        //执行任务函数
        if (ReadyTask != &TSS_TASKLIST.TASK_List[0] && ReadyTask->Callback != NULL && ReadyTask->Status == TASK_Ready)
        {
            ReadyTask->Status = TASK_Running;
            ReadyTask->Message.past = ReadyTask->Delay[DELAY_CYCLE] - ReadyTask->Delay[DELAY_TICK];
            ReadyTask->Callback(&ReadyTask->Message);
            //任务结束处理
            ReadyTask->Message.effect = 0;
            ReadyTask->Status = TASK_Blocked;
        }
#if TSS_IDLETASK == 1
        //执行空闲任务
        else
        {
            TSS_IdleTask(NULL);
        }
#endif
    }
}



/**
  * @brief  TSS列表插入
  * @param  list   管理内存的首地址
  * @param  data   要插入数据的首地址
  * @param  len    要插入数据的内存大小
  * @param  num    要把数据插入列表中的第几个
  * @retval 返回成功或者失败
  * @note   无
  */
int8_t TSS_ListInsert(void *list, void *data, uint16_t len, uint16_t num)
{
    if (MEM_SPACE(list) < len + 2)
        return TSS_MEMFULL;
    if (num > MEM_NUM())
        return TSS_NODATA;
    memmove((void *)((uint8_t *)MEM_DATA(list, num) + len), MEM_DATA(list, num), MEM_OFFSET(list, num, MEM_NUM()));
    memcpy(MEM_DATA(list, num), data, len);
    memmove((void *)(&MEM_MOVE_8(list, MEM_SIZE() - (MEM_NUM() + 1) * 2)),
            (void *)(&MEM_MOVE_8(list, MEM_SIZE() - MEM_NUM() * 2)),
            (MEM_NUM() - num) * 2);
    MEM_NUM() += 1;
    MEM_LEN(num) = len;
    return TSS_OK;
}



//返回列表剩余空间
static int16_t MEM_SPACE(void *list)
{
    uint16_t offset = MEM_OFFSET(list, 0, MEM_NUM());
    return MEM_SIZE() - 4 - MEM_NUM() * 2 - offset;
}



//从指定地址偏移一定的长度，进行数据读写
#define MEM_MOVE_8(p, size) *((volatile uint8_t *)p + (size))
#define MEM_MOVE_16(p, size) *((volatile uint16_t *)p + (size) / 2)
#define MEM_MOVE_32(p, size) *((volatile uint32_t *)p + (size) / 4)
//偏移的重要地址
#define MEM_SIZE() MEM_MOVE_16(list, 0)                      //内存大小
#define MEM_NUM() MEM_MOVE_16(list, 2)                       //数据个数
#define MEM_LEN(n) MEM_MOVE_16(list, MEM_SIZE() - 2 - 2 * n) //第n个数据的宽度


//查找第n个数据的地址
static void *MEM_DATA(void *list, uint16_t n)
{
    if (n > MEM_NUM())
        return NULL;
    uint16_t offset = MEM_OFFSET(list, 0, n);
    return (void *)&MEM_MOVE_8(list, 4 + offset);
}
//返回列表剩余空间
static int16_t MEM_SPACE(void *list)
{
    uint16_t offset = MEM_OFFSET(list, 0, MEM_NUM());
    return MEM_SIZE() - 4 - MEM_NUM() * 2 - offset;
}

