#pragma once

#include "exec_queue.h"
#include "FreeRTOS.h"
#include "task.h"

enum exec_type_t : uint8_t // 执行器种类枚举
{
    freq = 0,
    ob,
};

typedef struct taskarg_s // 任务内部传参结构体
{
    const uint32_t worker_id_{};
    const exec_type_t type_ {}; // 执行器类型 (周期型执行器还是突发型执行器)
    const uint32_t freq_{}; // 该等级执行器的执行频率
    const uint32_t average_ms_{}; // 经测算得到的任务平均延迟
    uint32_t delay_ticks_ = UINT32_MAX;
} taskarg_t;

typedef struct threadoption_s // 线程配置结构体
{
    const char* thread_name_{};
    osPriority_t thread_priority_{};
    taskarg_t arg_;
} threadoption_t;



class EQWorker // 执行器基类，每个执行器对象对应一个实际FreeRTOS任务
{
public:
    constexpr EQWorker() noexcept = default;
    void eqworkerTask(void* arg);
    // uint32_t delay_ticks_ = UINT32_MAX;

protected:
    uint8_t is_init_; // 执行器对象初始化标志

    /**
     * 一次性创建多个周期型任务执行器
     * @param toptions 任务参数 数组
     * @param tcbs 任务控制块 数组
     * @param tstacks 任务栈指针 数组
     * @param thandles 任务句柄 数组
     * @param taskfunc 任务主函数
     * @return 成功 / 失败
     */
    template <uint32_t level_num>
    static uint8_t create_task(
        const threadoption_s toptions[level_num],
        StaticTask_t tcbs[level_num],
        StackType_t tstacks[level_num][EQ_WORKER_STACK_SIZE],
        TaskHandle_t thandles[level_num], void (*taskfunc)(void* arg)
    );
    static constexpr uint32_t freq_to_ticks(uint32_t freq);
};


class FreqWorker : public EQWorker
{
public:
    static constexpr uint32_t freq_level_num = (uint32_t)FrequencyExecQueue::f_size; // 获取配置中的优先级数量
    static constexpr threadoption_s topt[freq_level_num] = { // 任务配置参数
        {.thread_name_ = "f_level0", .thread_priority_ = osPriorityHigh7,   .arg_ = {.worker_id_ = FrequencyExecQueue::f_level0, .type_ = freq, .freq_ = EQ_ERR, .average_ms_ = EQ_ERR}},
        {.thread_name_ = "f_level1", .thread_priority_ = osPriorityHigh,    .arg_ = {.worker_id_ = FrequencyExecQueue::f_level1, .type_ = freq, .freq_ = EQ_ERR, .average_ms_ = EQ_ERR}},
        {.thread_name_ = "f_level2", .thread_priority_ = osPriorityNormal7, .arg_ = {.worker_id_ = FrequencyExecQueue::f_level2, .type_ = freq, .freq_ = EQ_ERR, .average_ms_ = EQ_ERR}},
    };

    /**
     * 周期型任务执行队列，任务执行后再次插回fifo队列，按分频系数进行执行频率控制。
     * @param arg 任务配置结构体
     */
    static void eqworkerTask(void* arg);

    /**
     * 用于计算需要延迟多久以执行下一个任务
     * @param arg 任务参数结构体
     * @return
     */
static constexpr uint8_t calculate_delay_ticks(taskarg_t* arg);

private:
    // rtos任务相关静态内存分配
    static constinit FrequencyExecQueue eq[freq_level_num]; // 存储队列对象的数组
    static StackType_t WorkerStack[freq_level_num][EQ_WORKER_STACK_SIZE];
    static StaticTask_t WorkerTCB[freq_level_num];
    static TaskHandle_t eq_task_handle[freq_level_num];
    static constinit osThreadAttr_t thread_attr[freq_level_num];

    uint8_t init_eq(); // 初始化queue对象并创建RTOS任务
};


class PriWorker : public EQWorker
{
public:
    static constexpr uint32_t pri_level_num = (uint32_t)PriorityExecQueue::p_size; // 获取配置中的优先级数量
    static constexpr threadoption_s topt[pri_level_num] = {
        {.thread_name_ = "p_level0", .thread_priority_ = osPriorityHigh7,   .arg_ = {.worker_id_ = PriorityExecQueue::p_level0, .type_ = ob, .freq_ = EQ_ERR, .average_ms_ = 20}},
        {.thread_name_ = "p_level1", .thread_priority_ = osPriorityHigh,    .arg_ = {.worker_id_ = PriorityExecQueue::p_level1, .type_ = ob, .freq_ = EQ_ERR, .average_ms_ = 20}},
        {.thread_name_ = "p_level2", .thread_priority_ = osPriorityNormal7, .arg_ = {.worker_id_ = PriorityExecQueue::p_level2, .type_ = ob, .freq_ = EQ_ERR, .average_ms_ = 20}},
    };

    /**
     * 突发型任务执行器任务函数
     * 取出任务并执行，执行后不放回
     * @param arg 任务参数结构体
     */
    static void eqworkerTask(void* arg);

private:
    static constinit PriorityExecQueue eq[pri_level_num]; // 存储队列对象的数组
    static StackType_t WorkerStack[pri_level_num][EQ_WORKER_STACK_SIZE];
    static StaticTask_t WorkerTCB[pri_level_num];
    static TaskHandle_t eq_task_handle[pri_level_num];
    static constinit osThreadAttr_t thread_attr[pri_level_num];

    uint8_t init_eq(); // 初始化queue对象并创建RTOS任务

};
