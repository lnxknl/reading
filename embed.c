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
        //????????????????????????
        TSS_ThreadSetTrigger(USART_RX_TASK_ID, 5);
    }
    return NULL;
}

uint8_t USART_RX_TASK_ID = 0;


/**
  * @brief  ?????????????????????????????????
  * @param  ThreadID    ??????ID
  * @param  Delay       0????????????????????????>0???????????????????????????
  * @retval ???
  * @note   ????????????????????????????????????????????????????????????
            ???????????????????????????????????????????????????
  */
void TSS_ThreadSetTrigger(uint8_t ThreadID, int32_t Delay)
{
    if (TSS_TASKLIST.TASK_List[ThreadID].Types == TASK_TRIGGER)
    {
        //??????????????????????????????????????????
        if (Delay <= 0)
        {
#if TSS_TRIGGERNOW == 1 //??????????????????????????????????????????????????????????????????????????????
            TSS_TASKLIST.TASK_List[ThreadID].Delay[DELAY_TICK] = TSS_TASKLIST.TASK_List[ThreadID].Delay[DELAY_CYCLE];
            TSS_TASKLIST.TASK_List[ThreadID].Status = TASK_Running;
            TSS_TASKLIST.TASK_List[ThreadID].Callback(NULL);
#else //?????????????????????????????????????????????????????????????????????
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
	 * @brief ?????????????????????
	 */
    typedef struct
    {
        uint8_t TASK_Count;            //????????????
        volatile uint32_t TSS_RunTime; //??????????????????
        TSS_TaskInfoDef TASK_List[TSS_TASKNUMMAX];
    } TSS_TaskList;


    /**
	 * @brief ?????????????????????
	 */
    typedef struct
    {
        uint8_t ID;                 //????????????
        volatile int32_t Delay[2];  //????????????????????????
        volatile uint8_t Status;    //????????????
        uint8_t Priority;           //?????????
        TSS_TaskTypes Types;        //????????????
        TSS_TaskFunction Callback;  //??????????????????
        TSS_TaskMessageDef Message; //????????????
    } TSS_TaskInfoDef;


    /**
	 * @brief ????????????????????????
	 */
    typedef enum
    {
        TASK_HZ = 0,  //???????????????????????????????????????
        TASK_TRIGGER, //??????????????????
    } TSS_TaskTypes;


    typedef void (*TSS_TaskFunction)(TSS_TaskMessageDef *);

        /**
	 * @brief ?????????????????????
	 */
    typedef struct
    {
        int32_t past;     //??????????????????
        uint8_t effect;   //????????????????????????????????????1????????????????????????0
        uint32_t message; //??????
        void *data;       //????????????
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

            //??????????????????
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


    //?????????????????????
    typedef struct
    {
        uint8_t status;            //????????????
        uint32_t play_time;        //??????????????????????????????
        uint16_t play_node;        //??????????????????
        TSS_ActionTableDef *table; //???????????????????????????
        uint16_t play_len;         //???????????????
        uint32_t times;            //??????????????????
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
        //??????????????????
        for (i = 1; i < TSS_TASKLIST.TASK_Count; i++)
        {
            //???????????????????????????
            if (TSS_TASKLIST.TASK_List[i].Status == TASK_Ready && TSS_TASKLIST.TASK_List[i].Priority >= ReadyTask->Priority)
            {
                //??????????????????????????????
                ReadyTask = &TSS_TASKLIST.TASK_List[i];
            }
        }
        //??????????????????
        if (ReadyTask != &TSS_TASKLIST.TASK_List[0] && ReadyTask->Callback != NULL && ReadyTask->Status == TASK_Ready)
        {
            ReadyTask->Status = TASK_Running;
            ReadyTask->Message.past = ReadyTask->Delay[DELAY_CYCLE] - ReadyTask->Delay[DELAY_TICK];
            ReadyTask->Callback(&ReadyTask->Message);
            //??????????????????
            ReadyTask->Message.effect = 0;
            ReadyTask->Status = TASK_Blocked;
        }
#if TSS_IDLETASK == 1
        //??????????????????
        else
        {
            TSS_IdleTask(NULL);
        }
#endif
    }
}



/**
  * @brief  TSS????????????
  * @param  list   ????????????????????????
  * @param  data   ???????????????????????????
  * @param  len    ??????????????????????????????
  * @param  num    ???????????????????????????????????????
  * @retval ????????????????????????
  * @note   ???
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



//????????????????????????
static int16_t MEM_SPACE(void *list)
{
    uint16_t offset = MEM_OFFSET(list, 0, MEM_NUM());
    return MEM_SIZE() - 4 - MEM_NUM() * 2 - offset;
}



//?????????????????????????????????????????????????????????
#define MEM_MOVE_8(p, size) *((volatile uint8_t *)p + (size))
#define MEM_MOVE_16(p, size) *((volatile uint16_t *)p + (size) / 2)
#define MEM_MOVE_32(p, size) *((volatile uint32_t *)p + (size) / 4)
//?????????????????????
#define MEM_SIZE() MEM_MOVE_16(list, 0)                      //????????????
#define MEM_NUM() MEM_MOVE_16(list, 2)                       //????????????
#define MEM_LEN(n) MEM_MOVE_16(list, MEM_SIZE() - 2 - 2 * n) //???n??????????????????


//?????????n??????????????????
static void *MEM_DATA(void *list, uint16_t n)
{
    if (n > MEM_NUM())
        return NULL;
    uint16_t offset = MEM_OFFSET(list, 0, n);
    return (void *)&MEM_MOVE_8(list, 4 + offset);
}
//????????????????????????
static int16_t MEM_SPACE(void *list)
{
    uint16_t offset = MEM_OFFSET(list, 0, MEM_NUM());
    return MEM_SIZE() - 4 - MEM_NUM() * 2 - offset;
}

