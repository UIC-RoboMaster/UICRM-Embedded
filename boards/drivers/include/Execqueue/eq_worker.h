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
    const uint32_t worker_id_;
    const exec_type_t type_; // 执行器类型 (周期型执行器还是突发型执行器)
    const uint32_t freq_; // 该等级执行器的执行频率
    const uint32_t average_ms_; // 经测算得到的任务平均延迟
    uint32_t delay_ticks_ = UINT32_MAX;
} taskarg_t;

typedef struct threadoption_s // 线程配置结构体
{
    const char* thread_name_;
    osPriority_t thread_priority_;
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
};


class FreqWorker : public EQWorker
{
private:
    // static constexpr uint32_t task_average_time_ms = 20;
    static constexpr uint32_t freq_level_num = (uint32_t)FrequencyExecQueue::f_size;
    static constinit FrequencyExecQueue eq[freq_level_num]; // 存储队列对象的数组
    static constinit osThreadAttr_t thread_attr[freq_level_num];
    uint8_t init_eq();

    /**
     * 一次性创建多个周期型任务执行器
     * @param toptions 任务参数 数组
     * @param tcbs 任务控制块 数组
     * @param tstacks 任务栈指针 数组
     * @param thandles 任务句柄 数组
     * @return 成功 / 失败
     */
    static uint8_t create_task(
        const threadoption_s toptions[freq_level_num],
        StaticTask_t tcbs[freq_level_num],
        StackType_t tstacks[freq_level_num][EQ_WORKER_STACK_SIZE],
        TaskHandle_t thandles[freq_level_num]
        );
    static constexpr uint32_t freq_to_ticks(uint32_t freq);

public:
    static StaticTask_t WorkerTCB[freq_level_num];
    static StackType_t WorkerStack[freq_level_num][EQ_WORKER_STACK_SIZE];
    static TaskHandle_t eq_task_handle[freq_level_num];

    static constexpr threadoption_s topt[freq_level_num] = {
        {.thread_name_ = "f_level0", .thread_priority_ = osPriorityHigh7,   .arg_ = {.worker_id_ = FrequencyExecQueue::f_level0, .type_ = freq, .freq_ = EQ_ERR, .average_ms_ = EQ_ERR}},
        {.thread_name_ = "f_level1", .thread_priority_ = osPriorityHigh,    .arg_ = {.worker_id_ = FrequencyExecQueue::f_level1, .type_ = freq, .freq_ = EQ_ERR, .average_ms_ = EQ_ERR}},
        {.thread_name_ = "f_level2", .thread_priority_ = osPriorityNormal7, .arg_ = {.worker_id_ = FrequencyExecQueue::f_level2, .type_ = freq, .freq_ = EQ_ERR, .average_ms_ = EQ_ERR}},
    };

    static void eqworkerTask(void* arg);
    static constexpr uint8_t calculate_delay_ticks(taskarg_t* arg);
};


class PriWorker : public EQWorker
{
private:
    static constexpr uint32_t pri_level_num = (uint32_t)PriorityExecQueue::p_size;
    static constinit PriorityExecQueue eq[pri_level_num];
    static constinit osThreadAttr_t thread_attr[pri_level_num];

public:
    static constexpr threadoption_s topt[PriorityExecQueue::p_size] = {
        {.thread_name_ = "p_level0", .thread_priority_ = osPriorityHigh7,   .arg_ = {.worker_id_ = PriorityExecQueue::p_level0, .type_ = ob, .freq_ = EQ_ERR}},
        {.thread_name_ = "p_level1", .thread_priority_ = osPriorityHigh,    .arg_ = {.worker_id_ = PriorityExecQueue::p_level1, .type_ = ob, .freq_ = EQ_ERR}},
        {.thread_name_ = "p_level2", .thread_priority_ = osPriorityNormal7, .arg_ = {.worker_id_ = PriorityExecQueue::p_level2, .type_ = ob, .freq_ = EQ_ERR}},
    };
};
