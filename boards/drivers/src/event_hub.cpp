#include "event_hub.h"

EventHub::EventHub() {
}

// 初始化消息缓存队列
uint8_t EventHub::init_eList() {
    if (this->eListInited_ == 1)  return 0;
    this->eListInited_ = 1;

    // 初始化事件容器，0号容器为头节点
    for (uint8_t i = 0; i < EVENTHUB_CONTAINER_MAX_EVENT_NUM; i++) {
        eCacheList_[i].next_ = (i < EVENTHUB_CONTAINER_MAX_EVENT_NUM - 1) ? &eCacheList_[i + 1] : nullptr;
        eCacheList_[i].topic_ = TpcID_t::EMPTYTOPIC;
    }
    empty_eCacheList_head_ = &eCacheList_[0];
    return 1;
}

uint8_t EventHub::init_eRecvList() {
    if (this->eRecvListInited_ == 1)  return 0;
    this->eRecvListInited_ = 1;
    for (uint8_t i = 0; i < EVENTHUB_RECEIVE_FIFO_MAX_NUM; i++) {
        eRecvList_[i].next_ = (i < EVENTHUB_RECEIVE_FIFO_MAX_NUM - 1) ? &eRecvList_[i + 1] : nullptr;
        eRecvList_[i].topic_ = TpcID_t::EMPTYTOPIC;
    }
    empty_eRecvList_head_ = &eRecvList_[0];
    return 1;
}

uint8_t EventHub::subscribe_topic(subcriber_t* s, TpcIDMask_t t) {
    if (!t) return -1;
    // 每次取出一位
    for (TpcIDMask_t i = 0; t ; i++) {
        if (i > sizeof(TpcIDMask_t)) return -2;
        if (t & 0xfe) {
            topic_subscribers_[i]->next_ = s;
            topic_subscribers_[i] = s;
        }
        t = t >> 1;
    }
    return 1;
}

uint8_t EventHub::publish_event(topic_t topic, priority_t pri, uint32_t* msg, uint32_t msg_len) {
    if (pri == urgent) {
        subcriber_t* s = topic_subscribers_[get_eIDNumber(topic)];
        // 直接调接收者函数
        while (!s) {
            s->rx_.urgent_rx_(msg, msg_len);
            s = s->next_;
        }
    }
    event_container_t* et {};
    get_empty_container(&et);
    if (!et) return -1;
    et->topic_ = topic;
    et->priority_ = pri;
    et->msgbody_ = {msg, msg_len};
    return 0;
}

// uint8_t EventHub::dump_and_sort(event_container_t* event) {
//
// }

void EventHub::get_empty_container(EventHub::event_container_t** et) const {
    if (this->eRecvListInited_ == 0)  return;

    if (empty_eCacheList_head_ == nullptr) return;
    *et = empty_eCacheList_head_;
    empty_eCacheList_head_ = empty_eCacheList_head_->next_;
}