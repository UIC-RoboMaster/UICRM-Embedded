#pragma once
#include "cmsis_os2.h"

#define EQ_DEBUG
#define EQ_ERR UINT8_MAX
#define EQ_OK 0
// extern struct osThreadAttr_t;

// 使用牺牲一个元素的环形缓冲区作为fifo队列。



constexpr static uint32_t worker_number = 3;    // 优先级数量
constexpr static uint32_t queue_size = 20;      // FIFO尺寸

class ExecQueue
{
public:
    constexpr ExecQueue() noexcept = default;
    uint8_t init(const uint32_t worker_id);

    typedef struct task_param_s // 任务参数
    {
        void* params_;
    } task_param_t;

    typedef struct queue_task_s // 任务内容块
    {
        uint8_t (*task_)(task_param_t *p);
        task_param_t param_;
        uint32_t cycle_times_;
        uint32_t freq_;
    } queue_task_t;

    typedef uint32_t exec_freq_t;

    static constinit inline queue_task_t *equeue_pool[worker_number][queue_size] {}; // fifo指针队列


private:
    uint8_t worker_init();
    uint8_t fifo_init();
    uint8_t create_worker_task();

    uint8_t id_, head_, tail_;
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

    /**
     * add outbreak task 提交某任务到某个指定的突发性工作队列
     * @param p 任务内容块
     * @param pri 指定的工作队列优先级
     */
    uint8_t add_obtask(queue_task_t* p, uint32_t pri);

    /**
     * add cycle task 提交某任务到某个指定的周期性工作队列
     * @param p 任务内容块
     * @param ef 执行频率级别
     */
    uint8_t add_cytask(queue_task_t* p, exec_freq_t ef);



#ifdef EQ_DEBUG
    uint8_t get_task_num() const;
    uint32_t get_task_timecost() const;
#endif

};

void eqworkerTask(void* arg);
static constinit inline ExecQueue eq[worker_number] {};
static constinit uint8_t is_eqinit = EQ_ERR;
static void init_eq();

const osThreadAttr_t queueworker {
    .name = "",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
    .tz_module = 0,
    .reserved = 0
};

typedef struct task_param_s
{

} task_param_t;