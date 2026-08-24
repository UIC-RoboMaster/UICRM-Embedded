#include <cstring>
#include "../../include/Eventhub/event_hub.h"

EventHub::EventHub() = default;

uint8_t EventHub::init_emaincache() {
    if (this->emaincache_inited_ == 1)  return -1;
    this->emaincache_inited_ = 1;

    for (uint8_t i = 0; i < EVENTHUB_CONTAINER_MAX_EVENT_NUM; i++) {
        emaincache_[i].next_ = (i < EVENTHUB_CONTAINER_MAX_EVENT_NUM - 1) ? &emaincache_[i + 1] : nullptr;
        emaincache_[i].topic_ = TpcID_t::EMPTYTOPIC;
        emaincache_[i].et_location_ = maincache;
    }
    emaincache_empty_head_ = &emaincache_[0];
    
    for (uint8_t i = 0; i < sizeof(priority_t); i++) {
        emaincache_pri_idx_[i] = nullptr;
    }
    return 0;
}

uint8_t EventHub::emaincache_get_empty_et(EventHub::event_container_t** p_et) const {
    if (this->emaincache_inited_ == 0)  return -1;

    if (emaincache_empty_head_ == nullptr) return 1;
    *p_et = emaincache_empty_head_;
    emaincache_empty_head_ = emaincache_empty_head_->next_;
    return 0;
}

uint8_t EventHub::emaincache_insert_sorted(event_container_t* et) const {
    if (!et) return -1;
    if (et->priority_ == p_empty || et->priority_ >= urgent) return -1;

    priority_t pri = et->priority_;
    
    if (!emaincache_pri_idx_[pri]) {
        emaincache_pri_idx_[pri] = et;
        et->next_ = nullptr;
    } else {
        event_container_t* tail = emaincache_pri_idx_[pri];
        while (tail->next_) {
            tail = tail->next_;
        }
        tail->next_ = et;
        et->next_ = nullptr;
    }
    return 0;
}

uint8_t EventHub::emaincache_pop_highest(event_container_t** et) const {
    if (!et) return -1;
    
    for (int i = urgent - 1; i > p_empty; i--) {
        if (emaincache_pri_idx_[i]) {
            *et = emaincache_pri_idx_[i];
            emaincache_pri_idx_[i] = emaincache_pri_idx_[i]->next_;
            (*et)->next_ = nullptr;
            return 0;
        }
    }
    
    return 1;
}

uint8_t EventHub::init_eurgentcache() {
    if (this->eurgentcache_inited_ == 1)  return -1;
    this->eurgentcache_inited_ = 1;
    for (uint8_t i = 0; i < EVENTHUB_URGENT_FIFO_MAX_NUM; i++) {
        eurgentcache_[i].next_ = (i < EVENTHUB_URGENT_FIFO_MAX_NUM - 1) ? &eurgentcache_[i + 1] : nullptr;
        eurgentcache_[i].topic_ = TpcID_t::EMPTYTOPIC;
        eurgentcache_[i].et_location_ = urcache;
    }
    eurgentcache_empty_head_ = &eurgentcache_[0];
    eurgentcache_full_head_ = nullptr;
    return 0;
}

uint8_t EventHub::eurentcache_getemptyct(event_container_t** p_et) const {
    if (this->eurgentcache_inited_ == 0)  return -1;

    if (eurgentcache_empty_head_ == nullptr) return 1;
    *p_et = eurgentcache_empty_head_;
    eurgentcache_empty_head_ = eurgentcache_empty_head_->next_;
    (*p_et)->next_ = nullptr;
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

uint8_t EventHub::clear_et(event_container_t* et) {
    if (!et) return -1;
    et->topic_ = topic_t::EMPTYTOPIC;
    et->life_span_ = 0;
    et_location_t eloc = get_e_loc(et);

    event_container_t** empty_head = nullptr;

    switch (eloc) {
        case maincache: empty_head = &emaincache_empty_head_;   break;
        case urcache:   empty_head = &eurgentcache_empty_head_; break;
        default: return -1;
    }

    et->next_ = *empty_head;
    *empty_head = et;
    return 0;
}

EventHub::et_location_t EventHub::get_e_loc(const event_container_t* et) {
    return et->et_location_;
}

uint8_t EventHub::regisrter_subscriber(subcriber_t* s, TpcIDMask_t t) {
    if (!t) return -1;
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

uint8_t EventHub::event_release(
    event_container_t::msg_owner_t* own,
    uint32_t pc,
    topic_t topic,
    priority_t pri,
    uint32_t* msg,
    uint32_t msg_len
    ) const {
    if (!own || !pc || topic == topic_t::EMPTYTOPIC || pri == p_empty || !msg || !msg_len) return -1;

    event_container_t* et = nullptr;

    if (pri == urgent) {
        if (eurentcache_getemptyct(&et) != 0) return -1;
        
        et->topic_ = topic;
        et->priority_ = pri;
        et->msgbody_.owner_ = own;
        et->msgbody_.publisher_cookie_ = pc;
        et->msgbody_.payload_ = msg;
        et->msgbody_.length_ = msg_len;
        et->life_span_ = 0;

        const subcriber_t* s = topic_subscribers_[get_eIDNumber(topic)];
        while (s) {
            increase_e_lifespan(et);
            s->rx_.rx_func_[(uint32_t)topic](et);
            s = s->next_;
        }

        if (et->life_span_ > 0) {
            et->next_ = published_event_;
            published_event_ = et;
        } else {
            clear_et(et);
        }
        return 1;
    } else {
        if (emaincache_get_empty_et(&et) != 0) return -1;
        
        et->topic_ = topic;
        et->priority_ = pri;
        et->msgbody_.owner_ = own;
        et->msgbody_.publisher_cookie_ = pc;
        et->msgbody_.payload_ = msg;
        et->msgbody_.length_ = msg_len;
        et->life_span_ = 0;

        if (emaincache_insert_sorted(et) != 0) {
            clear_et(et);
            return -1;
        }
        return 0;
    }
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

uint8_t EventHub::publish_e_to_subscribers(event_container_t* et) {
    if (!et) return -1;
    if (chech_et_location(et, maincache) != 0) return -1;

#ifdef EH_DEBUG
    int i = 0;
#endif

    topic_t tpc = et->topic_;
    const subcriber_t* s = topic_subscribers_[get_eIDNumber(tpc)];
    if (!s) return 0;
    
    while (s != nullptr) {
        increase_e_lifespan(et);
        s->rx_.rx_func_[(uint32_t)tpc](et);
        s = s->next_;
#ifdef EH_DEBUG
        i++;
#endif
    }
    
#ifdef EH_DEBUG
    return i;
#else
    if (!published_event_) {
        published_event_ = et;
    } else {
        et->next_ = published_event_;
        published_event_ = et;
    }
    return 1;
#endif
}

uint8_t EventHub::chech_et_location(event_container_t* et, et_location_t loc) {
    return et->et_location_ == loc ? 0 : 1;
}

uint8_t EventHub::recover_expire_container() {
    if (!published_event_) return 0;
    
    uint8_t ret = 0;
    event_container_t* et = published_event_;
    event_container_t* prev = nullptr;
    
    while (et != nullptr) {
        if (et->life_span_ == 0) {
            et_location_t loc = get_e_loc(et);
            
            if (loc == maincache || loc == urcache) {
                et->msgbody_.owner_->recover_space_(&et->msgbody_);
                
                if (prev) {
                    prev->next_ = et->next_;
                } else {
                    published_event_ = et->next_;
                }
                
                event_container_t* next = et->next_;
                clear_et(et);
                et = next;
                ret++;
                continue;
            }
        }
        prev = et;
        et = et->next_;
    }
    return ret;
}
