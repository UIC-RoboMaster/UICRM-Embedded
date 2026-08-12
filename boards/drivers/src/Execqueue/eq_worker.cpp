#include "../../include/Execqueue/eq_worker.h"

uint8_t FreqWorker::init_eq()
{
    if (is_init_) return EQ_ERR; is_init_ = true;
    for (uint32_t i = 0; i<freq_level_num; i++) eq[i].init(i);
    this->create_task(
        &*topt,
        WorkerTCB,
        WorkerStack,
        eq_task_handle);
    return EQ_OK;
}

uint8_t FreqWorker::create_task(
    const threadoption_s toptions[freq_level_num],
    StaticTask_t tcbs[freq_level_num],
    StackType_t tstacks[freq_level_num][EQ_WORKER_STACK_SIZE],
    TaskHandle_t thandles[freq_level_num])
{
    if (!toptions || !tcbs || !thandles) return EQ_ERR;
    for (uint32_t i = 0; i<freq_level_num; i++)
    {
        thandles[i] = xTaskCreateStatic( // 创建任务
            eqworkerTask,
            toptions[i].thread_name_,
            EQ_WORKER_STACK_SIZE,
            (void*)&toptions[i].arg_,
            toptions[i].thread_priority_,
            tstacks[i],
            &tcbs[i]
        );
        if (thandles[i] == nullptr) return EQ_ERR; // 检查是否创建成功
    }
    return EQ_OK;
}

void FreqWorker::eqworkerTask(void* arg)
{
    // 获取任务参数并验证合法性
    taskarg_t* eqtask_arg = (taskarg_t*)arg;
    if (eqtask_arg->type_ != freq) return;

    // 拿到对象句柄
    const FrequencyExecQueue* e = &eq[eqtask_arg->worker_id_];

    // 初始化任务参数
    static uint32_t worker_num = e->task_num_;
    static ExecQueue::queue_task_t* q = nullptr;

    while (1) // 任务主循环
    {
        if (worker_num != e->task_num_) // 更新任务延时
        {
            worker_num = e->task_num_;
            calculate_delay_ticks(eqtask_arg);
        }

        // 取出任务，执行，放回
        eq[eqtask_arg->worker_id_].get_exec_cytask(q); // 获取任务
        q->task_(&q->param_); // 执行任务

        vTaskDelay(eqtask_arg->delay_ticks_); // 进行适当延时
    }
}

constexpr uint8_t FreqWorker::calculate_delay_ticks(taskarg_t* arg)
{
    if (!arg || arg->average_ms_ == EQ_ERR) return EQ_ERR;
    arg->delay_ticks_ = freq_to_ticks(arg->freq_ / eq[arg->worker_id_].task_num_) - arg->average_ms_; // 执行频率转换为tick数，除以任务总数，减去任务执行时间
    return EQ_OK;
}

constexpr uint32_t FreqWorker::freq_to_ticks(uint32_t freq = EQ_ERR)
{
    if (freq == EQ_ERR) return freq;
    return pdMS_TO_TICKS(1000u/freq);
}