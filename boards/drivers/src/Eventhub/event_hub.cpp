#include <cstring>
#include "../../include/Eventhub/event_hub.h"

EventHub::EventHub() = default;

/****** 消息传入缓存 ******/
uint8_t EventHub::init_erecvlist() {
    if (this->erecvlist_inited_ == 1)  return -1;
    this->erecvlist_inited_ = 1;
    for (uint8_t i = 0; i < EVENTHUB_RECEIVE_MAX_NUM; i++) {
        erecvlist_[i].next_ = (i < EVENTHUB_RECEIVE_MAX_NUM - 1) ? &erecvlist_[i + 1] : nullptr;
        erecvlist_[i].topic_ = TpcID_t::EMPTYTOPIC;
        erecvlist_[i].et_location_ = recvcache;
    }
    erecvlist_empty_head_ = &erecvlist_[0];
    return 0;
}

uint8_t EventHub::recvlist_get_empty_et(event_container_t** p_et) const {
    if (this->erecvlist_inited_ == 0)  return -1;

    if (erecvlist_empty_head_ == nullptr) return 1;
    *p_et = erecvlist_empty_head_;
    erecvlist_empty_head_ = erecvlist_empty_head_->next_;
    return 0;
}

uint8_t EventHub::recvlist_clear_et(event_container_t* et) {
    if (!et) return -1;
    et->topic_ = topic_t::EMPTYTOPIC;
    et->next_ =erecvlist_empty_head_;
    erecvlist_empty_head_ = et;
    et->et_location_ = recvcache;
    return 0;
}

/****** 消息主缓存 ******/
uint8_t EventHub::init_maincache() {
    if (this->emaincache_inited_ == 1)  return -1;
    this->emaincache_inited_ = 1;

    // 初始化事件容器，0号容器为头节点
    for (uint8_t i = 0; i < EVENTHUB_CONTAINER_MAX_EVENT_NUM; i++) {
        emaincache_[i].next_ = (i < EVENTHUB_CONTAINER_MAX_EVENT_NUM - 1) ? &emaincache_[i + 1] : nullptr;
        emaincache_[i].topic_ = TpcID_t::EMPTYTOPIC;
        erecvlist_[i].et_location_ = maincache;
    }
    emaincache_empty_head_ = &emaincache_[0];
    return 0;
}

uint8_t EventHub::maincache_clear_et(event_container_t* et) {
    if (!et) return -1;
    et->topic_ = topic_t::EMPTYTOPIC;
    et->next_ =emaincache_empty_head_;
    emaincache_empty_head_ = et;
    et->et_location_ = maincache;
    return 0;
}

uint8_t EventHub::emaincache_get_empty_et(EventHub::event_container_t** p_et) const {
    if (this->erecvlist_inited_ == 0)  return -1;

    if (emaincache_empty_head_ == nullptr) return 1;
    *p_et = emaincache_empty_head_;
    emaincache_empty_head_ = emaincache_empty_head_->next_;
    return 0;
}

/****** 紧急消息 ******/
uint8_t EventHub::init_urgentcache() {
    if (this->eurgentcache_inited_ == 1)  return -1;
    this->eurgentcache_inited_ = 1;
    for (uint8_t i = 0; i < EVENTHUB_URGENT_FIFO_MAX_NUM; i++) {
        eurgentcache_[i].next_ = (i < EVENTHUB_URGENT_FIFO_MAX_NUM - 1) ? &eurgentcache_[i + 1] : nullptr;
        eurgentcache_[i].topic_ = TpcID_t::EMPTYTOPIC;
        erecvlist_[i].et_location_ = urcache;
    }
    eurgentcache_empty_head_ = &eurgentcache_[0];
    eurgentcache_full_head_ = nullptr;
    return 0;
}

uint8_t EventHub::eurentcache_clearct(event_container_t* et) {
    if (!et) return -1;
    et->next_ = eurgentcache_empty_head_;
    eurgentcache_empty_head_ = et;
    et->et_location_ = urcache;
    return 0;
}

uint8_t EventHub::eurentcache_getemptyct(event_container_t** p_et) const {
    if (this->eurgentcache_inited_ == 0)  return -1;

    // 申请空容器
    if (eurgentcache_empty_head_ == nullptr) return 1;
    *p_et = eurgentcache_empty_head_;
    eurgentcache_empty_head_ = eurgentcache_empty_head_->next_;
    // 更新已填充链表节点
    if (!eurgentcache_full_head_) {
        eurgentcache_full_head_ = *p_et;
    } else {
        (*p_et)->next_ = eurgentcache_full_head_;
        eurgentcache_full_head_ = *p_et;
    }
    return 0;
}

uint8_t EventHub::eurentcache_getfullctnum() const {
    if (this->eurgentcache_inited_ == 0)  return -1;
    if (eurgentcache_full_head_ == nullptr) return 0;
    uint8_t num = 0;
    event_container_t* et = eurgentcache_full_head_;
    while (et) {
        num++;
        et = et->next_;
    }
    return num;
}

uint8_t EventHub::eurentcache_getemptyctnum() const {
    if (this->eurgentcache_inited_ == 0)  return -1;
    if (eurgentcache_empty_head_ == nullptr) return 0;
    uint8_t num = 0;
    event_container_t* et = eurgentcache_empty_head_;
    while (et) {
        num++;
        et = et->next_;
    }
    return num;
}

/****** 发布者函数 ******/
uint8_t EventHub::subscribe_topic(subcriber_t* s, TpcIDMask_t t) {
    if (!t) return -1;
    // 每次取出一位
    for (TpcIDMask_t i = 0; t ; i++) {
        if (i > sizeof(TpcIDMask_t)) return 1;
        if (t & 0xfe) {
            topic_subscribers_[i]->next_ = s;
            topic_subscribers_[i] = s;
        }
        t = t >> 1;
    }
    return 0;
}

uint8_t EventHub::event_release(msg_owner_t* own = nullptr, uint32_t pc = 0, topic_t topic = topic_t::EMPTYTOPIC, priority_t pri = p_empty, uint32_t* msg = nullptr, uint32_t msg_len = 0) const {
    if (!own || !pc || topic == topic_t::EMPTYTOPIC || pri ==p_empty || !msg || msg_len) return -1;

    event_container_t* et;
    if (pri == urgent) { if (!eurentcache_getemptyct(&et)) return -1; } // 如果是紧急事件，使用紧急缓冲区
    else {if (!recvlist_get_empty_et(&et)) return -1;}

    et->topic_ = topic;
    et->priority_ = pri;
    et->msgbody_.owner_ = own;
    et->msgbody_.publisher_cookie_ = pc;
    et->msgbody_.payload_ = msg;
    et->msgbody_.length_ = msg_len;

    // 非紧急消息就开始处理下一轮, 紧急消息直接发出去，
    if (pri != urgent) return 0;
    const subcriber_t* s = topic_subscribers_[get_eIDNumber(topic)];
    while (s) {
        increase_e_lifespan(et);
        s->rx_.rx_func_[(uint32_t)topic](et);
        s = s->next_;
    }
    return 1;
}

/****** 工具函数 ******/
uint8_t EventHub::dump_and_sort(event_container_t* et) const {
    uint8_t i, ret = 0;
    if (sizeof(i) != sizeof(priority_t)) return 0;

    // 从高优先级逐级移动所有事件
    for (i = p_err; i > p_empty; i++) {
        if (!erecvlist_pri_idx_[i]) continue; // 此优先级没有传入事件
        while (et != nullptr) { // 移动该优先级全部事件
            // 获取cache容器，拷入内容，并添加链表节点
            if (!chech_et_location(erecvlist_pri_idx_[i], recvcache)) return -1; // 容器必须来自接收缓冲区
            if (!emaincache_get_empty_et(&et)) return ret; // 如果没拿到空容器，
            memcpy(et, erecvlist_pri_idx_[i], sizeof(event_container_t));
            et->et_location_ = maincache;
            recvlist_clear_et(erecvlist_pri_idx_[i]); // 清空容器

            if (!emaincache_pri_idx_[i]) { emaincache_pri_idx_[i] = et; }
            else { emaincache_pri_idx_[i]->next_ = et; }
            erecvlist_pri_idx_[i] = erecvlist_pri_idx_[i]->next_; ret++; // 处理同优先级的下一事件
        }
    }
    if (!ret) return ret;

    for (i = urgent-1; et!=nullptr; i--) (et = emaincache_pri_idx_[i]); // 从最高优先级开始搜索事件，找到事件后将该容器弹出
    return ret;
}

EventHub::priority_t EventHub::get_e_priority(const event_container_t* et) {
    if (!et) return p_empty;
    return et->priority_;
}

topic_t EventHub::get_e_topic(const event_container_t* et) {
    if (!et) return topic_t::EMPTYTOPIC;
    return et->topic_;
}

uint8_t EventHub::increase_e_lifespan(event_container_t* event) {
    if (!event) return -1;
    event->life_span_++;
    return event->life_span_;
}

uint8_t EventHub::decrease_e_lifespan(event_container_t* event) {
    if (!event) return -1;
    event->life_span_--;
    return event->life_span_;
}

/****** 消息传递函数 ******/
uint8_t EventHub::publish_e_to_subscribers(event_container_t* et) {
    if (!et) return -1;
    if (!chech_et_location(et, maincache)) return -1; // 消息必须来自主缓冲区

#ifdef EH_DEBUG
    int i = 0;
#endif

    // 从消息获取主题，再去查订阅表确定有多少人订阅了这条消息
    topic_t tpc = et->topic_;
    const subcriber_t* s = topic_subscribers_[get_eIDNumber(tpc)];
    if (!s) return 0; // 没有人订阅该主题
    while (s != nullptr) { // 开始发布消息
        increase_e_lifespan(et);
        s->rx_.rx_func_[(uint32_t)tpc](et);
        s = s->next_;
#ifdef EH_DEBUG
        i++;
    }
    return i;
#endif
    }
    return 0;
}

uint8_t EventHub::chech_et_location(event_container_t* et, et_location_t loc) {
    return et->et_location_ == loc ? 0 : 1;
}
