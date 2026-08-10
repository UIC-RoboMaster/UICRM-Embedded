#include "../../include/Execqueue/exec_queue.h"

#include "stm32f4xx_hal_def.h"

uint8_t ExecQueue::init(const uint32_t worker_id)
{
    this->id_ = worker_id;
    this->equeue_pool_ = &equeue_pool[id_];
    if (!this->fifo_init()) return EQ_ERR;
    return EQ_OK;
}

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
uint8_t ExecQueue::get_task_num() const
{
    const int i = head_-tail_;
    return i>0 ? i : i+queue_size;
}
#endif

static void init_eq()
{
    for (uint8_t i = 0; i < worker_number; i++) eq->init(i);
}


void eqworkerTask(void* arg)
{
    // UNUSED(arg);
    if constexpr (!queue_size) return;
    if (!is_eqinit) return;
    init_eq();
    while (1)
    {

    }
}