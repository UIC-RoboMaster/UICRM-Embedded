#pragma once
#include <cstdint>
#include "../event_type.h"

constexpr static uint8_t EVENTHUB_CONTAINER_MAX_EVENT_NUM = 32;
constexpr static uint8_t EVENTHUB_RECEIVE_FIFO_MAX_NUM = 8;
constexpr static uint8_t EVENTHUB_RECEIVER_CALLBACK_NUM = 8;

class EventHub {
public:
    typedef uint8_t lifespan_t;

    typedef enum : uint8_t {
        pri_empty = 0,
        verylow = 1,
        low,
        medium,
        high,
        veryhigh,
        urgent, // 有此标记的事件会走紧急通道
        pri_err = 0xf
    } priority_t;


    typedef enum : uint8_t {
        event_empty = 0,
        can,
        uart,
        referee,

        event_err = 0xf,
    } event_type_t;

    /**
     * @brief 事件容器结构体定义
     * @param next_ 下一个事件容器指针挂点
     * @param priority_ 事件的优先级
    * @param life_span_ 事件的生命周期
     * @param topic_ 事件的类型
     * @param msgbody_ 事件的数据本体
     */
    typedef struct event_container_s {
        event_container_s* next_ = nullptr;
        priority_t priority_ = pri_empty;
        lifespan_t life_span_ = 0;
        event_type_t event_type_ = event_empty;
        topic_t topic_ = TpcID_t::EMPTYTOPIC;

        /**
         * @brief 消息数据结构体定义
         * @param payload_ 消息数据指针
         * @param length_ 消息数据长度
         */
        typedef struct msgpayload_s {
            uint32_t* payload_ = nullptr;
            uint32_t length_ = 0;
        } msgpayload_t;
        msgpayload_t msgbody_;
    } event_container_t;

    /**
     * @brief 订阅者结构体定义
     * @param next_ 下一个订阅者指针挂点
     * @param rx_ 接收函数
     * @param subcriber_id_ 订阅者ID，使用位运算的方式包含该任务订阅的所有事件类型
     */
    typedef struct subcriber_s {
        subcriber_s* next_ = nullptr;

        /**
         * 接收者可能会用多种接收函数来处理消息
         */
        typedef struct subcriber_rx_s {
            uint32_t* rx_func_[EVENTHUB_RECEIVER_CALLBACK_NUM]{};
            uint8_t (*info_rx_)(event_container_t* ec) = nullptr;
            uint8_t (*urgent_rx_)(uint32_t* msg, uint32_t msgsize) = nullptr;
        } subcriber_rx_t;
        subcriber_rx_t rx_ {};

    } subcriber_t;

    EventHub();

    /**
     *
     * @param subcriber 订阅者配置结构体
     * @param tgp_ 使用位运算的方式包含该任务订阅的所有事件类型,为便于运算，使用uint16_t进行封装
     * @return 成功订阅的事件数量
     */
    static uint8_t subscribe_topic(subcriber_t* subcriber, TpcIDMask_t tgp_);
    // 发布者使用，用于发布事件：向系统fifo队列存入事件 或 发布紧急事件，此时不走事件总线任务分发，而是直接调接收者函数
    uint8_t publish_event(topic_t topic, priority_t priority, uint32_t* msg, uint32_t msg_len);
    // 订阅者使用：减少事件生命周期
    uint8_t decrease_e_lifespan(event_container_t* event);

private:
    // 初始化事件容器表和等待读取队列



    // typedef struct empty_eNode_e {
    //     empty_eNode_e* next_;
    // } empty_eNode_t;
    //
    // typedef struct eNode_s {
    //     event_container_t* event_;
    //     eNode_s* next_;
    // } eNode_t;

    uint8_t init_eRecvList();
    uint8_t eRecvListInited_ = 0;
    constinit static event_container_t eRecvList_[EVENTHUB_RECEIVE_FIFO_MAX_NUM]; // 传入消息池
    constinit static event_container_t* empty_eRecvList_head_; // 空闲传入消息池链表头指针
    constinit static inline event_container_t* pri_idx_eRecvList[sizeof(priority_t)] {};

    uint8_t init_eList();
    uint8_t eListInited_ = 0;
    constinit static event_container_t eCacheList_[EVENTHUB_CONTAINER_MAX_EVENT_NUM]; // 内存池
    constinit static event_container_t* empty_eCacheList_head_; // 空闲内存池槽位链表头指针
    constinit static inline event_container_t* pri_idx_eCacheList[sizeof(priority_t)]; // 以优先级链表的方式保存事件，next指针将指向下一个同优先级事件，如果某个优先级没有事件，则指针为nullptr

    constinit static inline subcriber_t* topic_subscribers_[sizeof(TpcIDMask_t)] {}; // 订阅者链表 表头指针组
    constinit static inline event_container_t* published_event_ {}; // 已发布事件链表头

    // 从传入事件队列中取出全部事件并按优先级放入事件缓存
    uint8_t dump_and_sort(event_container_t* event);
    // 获取事件优先级
    priority_t get_e_priority(uint8_t priority);
    // 从事件缓存中发布事件给订阅者，并加入已发布事件链表
    uint8_t publish_e_to_subscribers(event_container_t* event);
    // 检查已发布事件链表，清空到期容器
    uint8_t clear_expire_container();

    // 从传入事件队列获取一个空容器并填充内容,返回void就是没有空容器
    void get_empty_container(EventHub::event_container_t** et = nullptr) const;
};
