#include "event_hub.h"

EventHub::EventHub() {
}

// 初始化消息缓存队列
uint8_t EventHub::init_eList() {
    if (this->eListInited_ == 1)  return 0;
    this->eListInited_ = 1;

    // 初始化事件容器，0号容器为头节点
    for (uint8_t i = 0; i < EVENTHUB_CONTAINER_MAX_EVENT_NUM; i++) {
        eList_[i].next_ = (i < EVENTHUB_CONTAINER_MAX_EVENT_NUM - 1) ? &eList_[i + 1] : nullptr;
        eList_[i].topic_ = empty;
    }
    empty_eList_head_ = &eList_[0];
    return 1;
}

uint8_t EventHub::init_eRecvList() {
    if (this->eRecvListInited_ == 1)  return 0;
    this->eRecvListInited_ = 1;
    for (uint8_t i = 0; i < EVENTHUB_RECEIVE_FIFO_MAX_NUM; i++) {
        eRecvList_[i].next_ = (i < EVENTHUB_RECEIVE_FIFO_MAX_NUM - 1) ? &eRecvList_[i + 1] : nullptr;
        eRecvList_[i].topic_ = empty;
    }
    empty_eRecvList_head_ = &eRecvList_[0];
    return 1;
}

