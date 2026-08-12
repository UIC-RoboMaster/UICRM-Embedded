#include "../../include/Execqueue/exec_queue.h"

#include "portmacro.h"
#include "stm32f4xx_hal_def.h"
#include "task.h"

/*********** ExecQueue 父类函数 ************/
uint8_t ExecQueue::fifo_init()
{
    if constexpr (!queue_size) return EQ_ERR;
    this->tail_ = 0;
    this->head_ = 1;
    return EQ_OK;
}

uint8_t ExecQueue::push_task(queue_task_t *p)
{
    if (this->is_full()) return EQ_ERR;
    *equeue_pool_[this->head_] = p;
    this->head_ = (head_+1) % queue_size;
    return EQ_OK;
}

ExecQueue::queue_task_t* ExecQueue::pop_task()
{
    if (this->is_empty()) return nullptr;
    const int i = tail_;
    tail_ = (tail_+1) % queue_size;
    return *equeue_pool_[i];
}

uint8_t ExecQueue::is_full() const
{
    return (head_+1) % queue_size == tail_;
}

uint8_t ExecQueue::is_empty() const
{
    return (tail_+1) % queue_size == head_;
}



#ifdef EQ_DEBUG
uint32_t ExecQueue::get_task_timecost(queue_task_t* qt)
{
    // 进入临界区来避免干扰
    taskENTER_CRITICAL();
    const uint32_t start = DWT->CYCCNT;
    qt->task_(&qt->param_);
    const uint32_t res = DWT->CYCCNT-start;
    taskEXIT_CRITICAL();
    return res;
}
void ExecQueue::get_task_num(uint32_t num) const
{
    num = head_-tail_;
    num = num > 0 ? num : num+queue_size;
}
#endif


/*********** PriorityExecQueue 派生类函数 ************/
uint8_t PriorityExecQueue::init(const uint32_t worker_id)
{
    this->id_ = worker_id;
    this->equeue_pool_ = &queue_pool[id_];
    if (!this->fifo_init()) return EQ_ERR;
    return EQ_OK;
}

uint8_t PriorityExecQueue::add_obtask(queue_task_t* p)
{
    if (this->is_full()) return EQ_ERR;
    if (p->freq_division_ != EQ_ERR) return EQ_ERR; // 突发性任务没有分频系数参数
    this->push_task(p);
    this->task_num_++;
    return EQ_OK;
}

uint8_t PriorityExecQueue::get_exec_obtask(queue_task_t* p)
{
    UNUSED(p);
    p = nullptr;
    if (this->is_empty()) return EQ_ERR;
    p = this->pop_task();
    this->task_num_--;
    return EQ_OK;
}


/*********** FrequencyExecQueue 派生类函数 ************/
uint8_t FrequencyExecQueue::init(const uint32_t worker_id)
{
    this->id_ = worker_id;
    this->equeue_pool_ = &queue_pool[id_];
    if (!this->fifo_init()) return EQ_ERR;
    return EQ_OK;
}

uint8_t FrequencyExecQueue::add_cytask(queue_task_t* p)
{
    if (this->is_full()) return EQ_ERR;
    if (p->freq_division_ == EQ_ERR) return EQ_ERR; // 周期性任务必须有分频系数参数
    this->push_task(p);
    this->task_num_++;
    return EQ_OK;
}

uint8_t FrequencyExecQueue::get_exec_cytask(queue_task_t* p)
{
    p = nullptr;
    if (this->is_empty()) return EQ_ERR;
    p = this->pop_task();
    this->push_task(p);
    return EQ_OK;
}

uint32_t calculate_delay_ms(uint32_t num);
