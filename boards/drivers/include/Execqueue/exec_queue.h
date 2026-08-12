#pragma once
#include "cmsis_os2.h"


#define EQ_DEBUG
#define EQ_ERR UINT8_MAX
#define EQ_OK 0
#define EQ_WORKER_STACK_SIZE 256

// extern struct osThreadAttr_t;
// 使用牺牲一个元素的环形缓冲区作为fifo队列。
// constexpr static uint32_t worker_number = 3;
constexpr static uint32_t queue_size = 20;      // FIFO尺寸

class ExecQueue
{
public:
    constexpr ExecQueue() noexcept = default;
    uint32_t task_num_; // 当前处于执行队列中的任务数量

    /** RTOS任务配置相关 **/
    typedef struct task_param_s // 任务参数
    {
        void* params_;
    } task_param_t;

    typedef struct queue_task_s // 任务内容块
    {
        uint8_t (*task_)(task_param_s *p);
        uint32_t freq_division_ = EQ_ERR;
        task_param_t param_;
    } queue_task_t;


protected:
    uint8_t fifo_init();
    uint32_t id_, head_, tail_;
    queue_task_t *(*equeue_pool_)[queue_size];

    /**
     * 将某任务加入队列
     * @param p 添加的任务内容块指针
     * @return 成功：ifdef EQ_DEBUG: 任务队列数量，else: 0。失败：255
     */
    uint8_t push_task(queue_task_t* p);

    /**
     * 按序弹出任务
     * @return 成功：任务内容块指针 失败：nullptr
     */
    queue_task_t* pop_task();

    uint8_t is_full() const;
    uint8_t is_empty() const;

#ifdef EQ_DEBUG
    static uint32_t get_task_timecost(queue_task_t* qt);
    void get_task_num(uint32_t num) const;
#endif
};

class PriorityExecQueue : public ExecQueue {
public:
    constexpr PriorityExecQueue() noexcept = default;
    enum priority_level_t : uint32_t
    {
        p_level0 = 0,
        p_level1,
        p_level2,
        p_size, // 突发性工作队列：优先级数量
    };

    /**
     * add outbreak task 提交某任务到某个指定的突发性工作队列
     * @param p 任务内容块
     */
    uint8_t add_obtask(queue_task_t* p);
    uint8_t get_exec_obtask(queue_task_t* p);



    /**
     *
     * @param worker_id 执行器编号，与该执行器所处FreeRTOS优先级有关
     * @return 初始化行为是否成功
     */
    uint8_t init(const uint32_t worker_id);

private:
    static constinit inline queue_task_t *queue_pool[(uint32_t)p_size][queue_size] {}; // fifo指针队列
};

class FrequencyExecQueue : public ExecQueue
{
public:
    constexpr FrequencyExecQueue() noexcept = default;
    enum freq_level_t : uint32_t
    {
        f_level0 = 0,
        f_level1,
        f_level2,
        f_size, // 周期性工作队列：频率等级数量
    };
    static constexpr uint32_t freq_hz[f_size]  = {
        10,
        100,
        1000,
    };

    /**
     * add cycle task 提交某任务到某个指定的周期性工作队列
     * @param p 任务内容块
     */
    uint8_t add_cytask(queue_task_t* p);

    /**
     * 获取当前待执行的任务并自动循环
     * @param p 入参容器，如果有可用的任务，将返回该任务句柄, 如果没有，返回 nullptr
     * @return 是否成功
     */
    uint8_t get_exec_cytask(queue_task_t* p);

    /**
     * 配置执行器编号、分配内存池、初始化软件环形缓冲区 & FIFO队列
     * @param worker_id 执行器编号
     * @return 初始化是否成功
     */
    uint8_t init(const uint32_t worker_id);
    //
    // // 计算需要延迟的时间
    // uint32_t calculate_delay_ms(uint32_t num);


private: static constinit inline queue_task_t *queue_pool[(uint32_t)f_size][queue_size] {};
};




// 任务执行函数，用于批量创建任务或外部接口管理
// void p_eqworkerTask(void* arg);
// void f_eqworkerTask(void* arg);
// static constinit inline ExecQueue p_eq[(uint32_t)p_size] {};
// static constinit inline ExecQueue f_eq[(uint32_t)p_size] {};
// static constinit uint8_t is_p_eqinit = EQ_ERR;
// static constinit uint8_t is_f_eqinit = EQ_ERR;
// static void init_p_eq();
// static void init_f_eq();

