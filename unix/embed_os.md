import os
import logging
import sys
import psutil
from shutil import copy

from .utils import execute_program
from .get_tools import get_make_tool, \
                       get_cmake_tool, \
                       get_cxx_tool, \
                       get_c_tool, \
                       get_valgrind_tool
from .settings import DEFAULT_CMAKE_GENERATORS


void print_memory_info()
{
#if defined MBED_HEAP_STATS_ENABLED &&  MBED_MEM_TRACING_ENABLED
    // Grab the heap statistics
    mbed_stats_heap_t heap_stats;
    mbed_stats_heap_get(&heap_stats);
    printf("Heap size: %lu / %lu bytes\r\n",
           heap_stats.current_size,
           heap_stats.reserved_size);
#endif
}

#define TEST_ASSERT_NOT_NULL_MESSAGE(pointer, message)                                             UNITY_TEST_ASSERT_NOT_NULL((pointer), __LINE__, (message))


#define UNITY_TEST_ASSERT_NOT_NULL(pointer, line, message)                                       UNITY_TEST_ASSERT(((pointer) != NULL),  (UNITY_LINE_TYPE)(line), (message))


#define UNITY_TEST_ASSERT(condition, line, message)                                              if (condition) {} else {UNITY_TEST_FAIL((UNITY_LINE_TYPE)(line), (message));}

#define UNITY_TEST_FAIL(line, message)   UnityFail(   (message), (UNITY_LINE_TYPE)(line))

void UnityFail(const char* msg, const UNITY_LINE_TYPE line)
{
    UNITY_SKIP_EXECUTION;

    UnityTestResultsBegin(Unity.TestFile, line);
    UnityPrintFail();
    if (msg != NULL)
    {
        UNITY_OUTPUT_CHAR(':');

#ifndef UNITY_EXCLUDE_DETAILS
        if (Unity.CurrentDetail1)
        {
            UnityPrint(UnityStrDetail1Name);
            UnityPrint(Unity.CurrentDetail1);
            if (Unity.CurrentDetail2)
            {
                UnityPrint(UnityStrDetail2Name);
                UnityPrint(Unity.CurrentDetail2);
            }
            UnityPrint(UnityStrSpacer);
        }
#endif
        if (msg[0] != ' ')
        {
            UNITY_OUTPUT_CHAR(' ');
        }
        UnityPrint(msg);
    }
    UNITY_FAIL_AND_BAIL;
}


#define UNITY_SKIP_EXECUTION  { if ((Unity.CurrentTestFailed != 0) || (Unity.CurrentTestIgnored != 0)) {return;} }


class TimerBase {

public:
    /** Start the timer
     */
    void start();

    /** Stop the timer
     */
    void stop();

    /** Reset the timer to 0.
     *
     * If it was already running, it will continue
     */
    void reset();

    /** Get the time passed in seconds
     *
     *  @returns    Time passed in seconds
     */
    MBED_DEPRECATED_SINCE("mbed-os-6.0.0", "Floating point operators should normally be avoided for code size. If really needed, you can use `duration<float>{elapsed_time()}.count()`")
    float read() const;

    /** Get the time passed in milliseconds
     *
     *  @returns    Time passed in milliseconds
     */
    MBED_DEPRECATED_SINCE("mbed-os-6.0.0", "Use the Chrono-based elapsed_time method.  If integer milliseconds are needed, you can use `duration_cast<milliseconds>(elapsed_time()).count()`")
    int read_ms() const;

    /** Get the time passed in microseconds
     *
     *  @returns    Time passed in microseconds
     */
    MBED_DEPRECATED_SINCE("mbed-os-6.0.0", "Use the Chrono-based elapsed_time method.  If integer microseconds are needed, you can use `elapsed_time().count()`")
    int read_us() const;

    /** An operator shorthand for read()
     */
    MBED_DEPRECATED_SINCE("mbed-os-6.0.0", "Floating point operators should normally be avoided for code size. If really needed, you can use `duration<float>{elapsed_time()}.count()`")
    operator float() const;

    /** Get in a high resolution type the time passed in microseconds.
     *  Returns a 64 bit integer.
     */
    MBED_DEPRECATED_SINCE("mbed-os-6.0.0", "Use the Chrono-based elapsed_time method.  If integer microseconds are needed, you can use `elapsed_time().count()`")
    us_timestamp_t read_high_resolution_us() const;

    /** Get in a high resolution type the time passed in microseconds.
     *  Returns a 64 bit integer chrono duration.
     */
    std::chrono::microseconds elapsed_time() const;

#if !defined(DOXYGEN_ONLY)
protected:
    TimerBase(const ticker_data_t *data);
    TimerBase(const ticker_data_t *data, bool lock_deepsleep);
    TimerBase(const TimerBase &t);
    TimerBase(TimerBase &&t);
    ~TimerBase();

    const TimerBase &operator=(const TimerBase &) = delete;

    std::chrono::microseconds slicetime() const;
    TickerDataClock::time_point _start{};   // the start time of the latest slice
    std::chrono::microseconds _time{};    // any accumulated time from previous slices
    TickerDataClock _ticker_data;
    bool _lock_deepsleep;    // flag that indicates if deep sleep should be disabled
    bool _running = false;   // whether the timer is running

private:
    // Copy storage while a lock is held
    TimerBase(const TimerBase &t, const CriticalSectionLock &) : TimerBase(t, false) {}
    // Copy storage only - used by delegating constructors
    TimerBase(const TimerBase &t, bool) : _start(t._start), _time(t._time), _ticker_data(t._ticker_data), _lock_deepsleep(t._lock_deepsleep), _running(t._running) {}
};


class Timer : public TimerBase {
public:
    Timer();
};


uint32_t sd_nvic_critical_region_enter(uint8_t * p_is_nested_critical_region)
{
    __disable_irq();

    *p_is_nested_critical_region = (m_in_critical_region != 0);
    m_in_critical_region++;

    return NRF_SUCCESS;
}


#define NRF_SUCCESS                           (NRF_ERROR_BASE_NUM + 0)  ///< Successful command



/// @cond Make doxygen skip this file

/** @defgroup NRF_ERRORS_BASE Error Codes Base number definitions
 * @{ */
#define NRF_ERROR_BASE_NUM      (0x0)       ///< Global error base
#define NRF_ERROR_SDM_BASE_NUM  (0x1000)    ///< SDM error base
#define NRF_ERROR_SOC_BASE_NUM  (0x2000)    ///< SoC error base
#define NRF_ERROR_STK_BASE_NUM  (0x3000)    ///< STK error base
/** @} */


void file_test_write(const char *file, size_t offset, const unsigned char *data, size_t data_length, size_t block_size)
{
    char filename[255] = { 0 };
    snprintf(filename, 255, "/sd/%s", file);

    FILE *output = fopen(filename, "w+");
    TEST_ASSERT_NOT_NULL_MESSAGE(output, "could not open file");

    int result = fseek(output, offset, SEEK_SET);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, result, "could not seek to location");

    Timer timer;
    timer.start();

    size_t index = 0;
    while (index < data_length) {
        size_t write_length = data_length - index;

        if (write_length > block_size) {
            write_length = block_size;
        }

        size_t written = fwrite(&data[index], sizeof(unsigned char), write_length, output);
        TEST_ASSERT_EQUAL_UINT_MESSAGE(write_length, written, "failed to write");

        index += write_length;
    }
    TEST_ASSERT_EQUAL_UINT_MESSAGE(index, data_length, "wrong length");

    result = fclose(output);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, result, "could not close file");

    timer.stop();
    tr_info("[FS] Wrote: \"%s\" %.2fKB (%.2fKB/s, %.2f secs)", file,
            float(data_length) / 1024, float(data_length) / timer.read() / 1024, timer.read());
}


    struct control_t : private base_control_t
    {
        control_t() : base_control_t(make_base_control_t(REPEAT_UNDECLR, TIMEOUT_UNDECLR)) {}

        control_t(repeat_t repeat, uint32_t timeout_ms) :
            base_control_t(make_base_control_t(repeat, timeout_ms)) {}

        control_t(repeat_t repeat) :
            base_control_t(make_base_control_t(repeat, TIMEOUT_UNDECLR)) {}

        control_t(uint32_t timeout_ms) :
            base_control_t(make_base_control_t(REPEAT_UNDECLR, timeout_ms)) {}

        control_t(const base_control_t& other) :
            base_control_t(other) {}

        friend control_t operator+(const control_t& lhs, const control_t& rhs) {
            control_t result(
                repeat_t(lhs.repeat | rhs.repeat),
                (rhs.timeout == TIMEOUT_NONE) ? rhs.timeout : lhs.timeout);

            if (result.timeout != TIMEOUT_NONE && result.timeout > rhs.timeout) {
                result.timeout = rhs.timeout;
            }

            if (result.repeat & REPEAT_NONE) {
                result.repeat = REPEAT_NONE;
            }
            else {
                if (result.repeat & REPEAT_SETUP_TEARDOWN) {
                    result.repeat = repeat_t(result.repeat & ~REPEAT_CASE_ONLY);
                }
                if (result.timeout == TIMEOUT_NONE && result.repeat & REPEAT_ON_TIMEOUT) {
                    result.repeat = repeat_t(result.repeat & ~REPEAT_ON_TIMEOUT);
                }
            }

            return result;
        }

        repeat_t
        inline get_repeat() const {
            return repeat;
        }
        uint32_t
        inline get_timeout() const {
            return timeout;
        }

    private:
        static base_control_t make_base_control_t(repeat_t repeat, uint32_t timeout) {
            base_control_t result = {
                repeat,
                timeout
            };
            return result;
        }

        friend class Harness;
    };



    class Thread {
public:

    Thread(osPriority priority = osPriorityNormal,
           uint32_t stack_size = OS_STACK_SIZE,
           unsigned char *stack_mem = nullptr, const char *name = nullptr)
    {
    }

    Thread(uint32_t tz_module, osPriority priority = osPriorityNormal,
           uint32_t stack_size = OS_STACK_SIZE,
           unsigned char *stack_mem = nullptr, const char *name = nullptr)
    {
    }

    osStatus start(mbed::Callback<void()> task);

    osStatus join()
    {
        return 0;
    };
    osStatus terminate();
    osStatus set_priority(osPriority priority)
    {
        return 0;
    };
    osPriority get_priority() const
    {
        return osPriorityNormal;
    };
    uint32_t flags_set(uint32_t flags)
    {
        return 0;
    };

    /** State of the Thread */
    enum State {
        Inactive,           /**< NOT USED */
        Ready,              /**< Ready to run */
        Running,            /**< Running */
        WaitingDelay,       /**< Waiting for a delay to occur */
        WaitingJoin,        /**< Waiting for thread to join. Only happens when using RTX directly. */
        WaitingThreadFlag,  /**< Waiting for a thread flag to be set */
        WaitingEventFlag,   /**< Waiting for a event flag to be set */
        WaitingMutex,       /**< Waiting for a mutex event to occur */
        WaitingSemaphore,   /**< Waiting for a semaphore event to occur */
        WaitingMemoryPool,  /**< Waiting for a memory pool */
        WaitingMessageGet,  /**< Waiting for message to arrive */
        WaitingMessagePut,  /**< Waiting for message to be send */
        WaitingInterval,    /**< NOT USED */
        WaitingOr,          /**< NOT USED */
        WaitingAnd,         /**< NOT USED */
        WaitingMailbox,     /**< NOT USED (Mail is implemented as MemoryPool and Queue) */

        /* Not in sync with RTX below here */
        Deleted,            /**< The task has been deleted or not started */
    };

    State get_state() const
    {
        return Ready;
    };
    uint32_t stack_size() const
    {
        return 0;
    };
    uint32_t free_stack() const
    {
        return 0;
    };
    uint32_t used_stack() const
    {
        return 0;
    };
    uint32_t max_stack() const
    {
        return 0;
    };
    const char *get_name() const
    {
        return "";
    };
    osThreadId_t get_id() const
    {
        return 0;
    };
    virtual ~Thread();
private:
    // Required to share definitions without
    // delegated constructors
    void constructor(osPriority priority = osPriorityNormal,
                     uint32_t stack_size = OS_STACK_SIZE,
                     unsigned char *stack_mem = nullptr,
                     const char *name = nullptr);
};


struct DigitalOut {
    virtual ~DigitalOut() = default;
    virtual void write(int value) = 0;
    virtual int read() = 0;
    virtual int is_connected() = 0;

    DigitalOut &operator= (int value)
    {
        // Underlying implementation is responsible for thread-safety
        write(value);
        return *this;
    }

    DigitalOut &operator= (DigitalOut &rhs)
    {
        // Underlying implementation is responsible for thread-safety
        write(rhs.read());
        return *this;
    }

    operator int()
    {
        // Underlying implementation is responsible for thread-safety
        return read();
    }

};


typedef struct {
    uint8_t octet[6]; /**< Unique 6-byte MAC address */
} cy_mac_addr_t;

typedef struct {
    uint8_t type;
    union {
        cy_ip_addr_v4_t addrv4;
        cy_ip_addr_v6_t addrv6;
    };
} cy_ip_addr_t;


/** Utility macro when neither NDEBUG or CY_NO_ASSERT is not declared to check a condition and, if
   false, trigger a breakpoint */
#if defined(NDEBUG) || defined(CY_NO_ASSERT)
    #define CY_ASSERT(x)    do {                \
                            } while(false)
#else
    #define CY_ASSERT(x)    do {                \
                                if(!(x))        \
                                {               \
                                    CY_HALT();  \
                                }               \
                            } while(false)
#endif // defined(NDEBUG)


static inline void CY_HALT(void)
{
    __asm("    bkpt    1");
}

// Clear all the memory statistics
static void test_clear_stats()
{
    memset(&stats, 0, sizeof(stats));
}

void mbed_mem_trace_set_callback(mbed_mem_trace_cb_t cb)

mbed_mem_trace_set_callback(test_trace_cb);
typedef void (*mbed_mem_trace_cb_t)(uint8_t op, void *res, void *caller, ...);

extern "C" void test_trace_cb(uint8_t op, void *res, void *caller, ...)

    int expected_key = 1;
    do {
        greentea_parse_kv(_key, _value, sizeof(_key), sizeof(_value));
        expected_key = strcmp(_key, "base_time");
    } while (expected_key);


    class Mutex {
public:
    Mutex();

    Mutex(const char *name);

    osStatus lock();

    bool trylock();

    bool trylock_for(uint32_t millisec);

    bool trylock_until(uint64_t millisec);

    osStatus unlock();

    osThreadId_t get_owner();

    ~Mutex();
};


void increment_on_signal()
{
    mutex.lock();

    cond.wait();
    change_counter++;

    mutex.unlock();
}

template<typename T>
void test_reschedule(void)
{
    TimeoutDriftTester<T> timeout(TEST_DELAY);

    timeout.reschedule_callback();
    ThisThread::sleep_for(TEST_DELAY * 5);
    TEST_ASSERT(timeout.get_callback_count() >= 3);
}

void send_thread(EventFlags *ef, uint32_t flags, milliseconds wait)
{
    for (uint32_t i = 0; i <= MAX_FLAG_POS; i++) {
        const uint32_t flag = flags & (1 << i);
        if (flag) {
            ef->set(flag);
            if (wait != 0ms) {
                ThisThread::sleep_for(wait);
            }
        }
    }
}


struct linked_list {
    linked_list *next;
    uint8_t data[MALLOC_TEST_SIZE];
};


extern uint32_t __mbed_sbrk_start_0;
extern uint32_t __mbed_krbs_start_0;
unsigned char *mbed_heap_start_0 = (unsigned char *) &__mbed_sbrk_start_0;;
uint32_t mbed_heap_size_0 = (uint32_t) &__mbed_krbs_start_0 - (uint32_t) &__mbed_sbrk_start_0;


#define TEST_VALUE      789
static struct Test {
    Test() : val(TEST_VALUE) {}
    ~Test() {}
    int val;
} t;


#define TEST_ASSERT_EQUAL(expected, actual)                                                        UNITY_TEST_ASSERT_EQUAL_INT((expected), (actual), __LINE__, NULL)



/*
 * Return true if the region is filled only with the specified value
 */
static bool valid_fill(uint8_t *data, uint32_t size, uint8_t fill)
{
    for (uint32_t i = 0; i < size; i++) {
        if (data[i] != fill) {
            return false;
        }
    }
    return true;
}


static void allocate_and_fill_heap(linked_list *&head)
{
    linked_list *current;

    current = (linked_list *) malloc(sizeof(linked_list));
    TEST_ASSERT_NOT_NULL(current);

    current->next = NULL;
    memset((void *) current->data, MALLOC_FILL, sizeof(current->data));

    // Allocate until malloc returns NULL
    head = current;
    while (true) {

        // Allocate
        linked_list *temp = (linked_list *) malloc(sizeof(linked_list));

        if (NULL == temp) {
            break;
        }
        bool result = rangeinrange((uint32_t) temp, sizeof(linked_list), (uint32_t)mbed_heap_start, mbed_heap_size);
#if defined(TOOLCHAIN_GCC_ARM) && defined(MBED_SPLIT_HEAP)
        if (false == result) {
            result = rangeinrange((uint32_t) temp, sizeof(linked_list), (uint32_t)mbed_heap_start_0, mbed_heap_size_0);
        }
#endif
        TEST_ASSERT_TRUE_MESSAGE(result, "Memory allocation out of range");

        // Init
        temp->next = NULL;
        memset((void *) temp->data, MALLOC_FILL, sizeof(current->data));

        // Add to list
        current->next = temp;
        current = temp;
    }
}


#define MALLOC_FILL                 0x55


[cling]$ int arr[3]
(int [3]) { 0, 0, 0 }
[cling]$ arr
(int [3]) { 0, 0, 0 }
[cling]$ printf("%ld",arr)
input_line_21:2:15: warning: format specifies type 'long' but the argument has type 'int *' [-Wformat]
 printf("%ld",arr)
         ~~~  ^~~
139979006353432(int) 15


static void allocate_and_fill_heap(linked_list *&head)


    while (true) {

        // Allocate
        linked_list *temp = (linked_list *) malloc(sizeof(linked_list));

        if (NULL == temp) {
            break;
        }
        bool result = rangeinrange((uint32_t) temp, sizeof(linked_list), (uint32_t)mbed_heap_start, mbed_heap_size);
#if defined(TOOLCHAIN_GCC_ARM) && defined(MBED_SPLIT_HEAP)
        if (false == result) {
            result = rangeinrange((uint32_t) temp, sizeof(linked_list), (uint32_t)mbed_heap_start_0, mbed_heap_size_0);
        }
#endif
        TEST_ASSERT_TRUE_MESSAGE(result, "Memory allocation out of range");

        // Init
        temp->next = NULL;
        memset((void *) temp->data, MALLOC_FILL, sizeof(current->data));

        // Add to list
        current->next = temp;
        current = temp;
    }



    #include<stdio.h>

int main()
{
        int a=0;
//      while(1) {
        for(;;){
        a+=1;
//              if(a == 3){
//                      continue;
//              }
        printf("aaaa\n");
                if(a == 5){
                        break;
                }
        }
        printf("hello world\n");

        return 0;
}


void task_using_malloc(void)
{
    void *data = NULL;

    while (thread_should_continue) {
        // Repeatedly allocate and free memory
        data = malloc(THREAD_MALLOC_SIZE);
        TEST_ASSERT_NOT_NULL(data);

        // test whole allocated memory
        memset(data, 0, THREAD_MALLOC_SIZE);

        free(data);
    }
}


extern uint32_t mbed_heap_size;
static const int test_timeout = 25;
volatile bool thread_should_continue = true;
#define NUM_THREADS         4
#define THREAD_MALLOC_SIZE  100

#if defined(__CORTEX_A9) || defined(__CORTEX_A5)
#define THREAD_STACK_SIZE   512
#elif defined(__CORTEX_M23) || defined(__CORTEX_M33)
#define THREAD_STACK_SIZE   512
#elif defined(TARGET_ARM_FM)
#define THREAD_STACK_SIZE   512
#elif defined(TARGET_CY8CKIT_062_WIFI_BT_PSA)
#define THREAD_STACK_SIZE   512
#else
#define THREAD_STACK_SIZE   256
#endif


[cling]$ *ap0  =3
(int) 3
[cling]$ *d0
input_line_14:2:2: warning: ISO C++ does not allow indirection on operand of type 'void *' [-Wvoid-ptr-dereference]
 *d0
 ^~~
[cling]$ (int *)d0
(int *) 0x5561f5cb4c40
[cling]$ *(int *)d0
(int) 3

[cling]$ #include<stdlib.h>
[cling]$ void* d1 =malloc(4);
[cling]$ d1
(void *) 0x558f22f43e70
[cling]$ *(int *)d1 = 8
(int) 8
[cling]$ delete d1;
input_line_8:2:2: warning: cannot delete expression with pointer-to-'void' type 'void *' [-Wdelete-incomplete]
 delete d1;
 ^      ~~
[cling]$ delete (int *)d1;
[cling]$ *(int *)d1
(int) 598831360
[cling]$ d1 = NULL;
[cling]$ *(int *)d1
input_line_12:2:3: warning: null passed to a callee that requires a non-null argument [-Wnonnull]
 *(int *)d1
  ^~~~~~~~~
