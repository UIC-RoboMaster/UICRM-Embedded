#pragma once
#include <cstdint>
#include "../Eventhub/event_type.h"

constexpr static uint8_t EVENTHUB_CONTAINER_MAX_EVENT_NUM = 32;
constexpr static uint8_t EVENTHUB_RECEIVE_MAX_NUM = 8;
constexpr static uint8_t EVENTHUB_URGENT_FIFO_MAX_NUM = 8;

class EventHub {
public:
    typedef uint8_t lifespan_t;

    typedef enum : uint8_t {
        p_empty = 0,
        verylow = 1,
        low,
        medium,
        high,
        veryhigh,
        urgent, // 有此标记的事件会走紧急通道,此值尺寸必须小于 uint32_t
        p_err = 0xf
    } priority_t;

    typedef enum : uint8_t {
        e_empty = 0,
        can,
        uart,
        referee,
        size,
        e_err = 0xf,
    } event_type_t;

    typedef enum : uint8_t {
        l_empty = 0,
        urcache,
        recvcache,
        maincache,
        l_size,
        l_err = 0xf,
    } et_location_t;

    typedef struct msg_owner_s msg_owner_t;

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
        priority_t priority_ = p_empty;
        lifespan_t life_span_ = 0;
        et_location_t et_location_ = l_empty;
        event_type_t event_type_ = e_empty;
        topic_t topic_ = TpcID_t::EMPTYTOPIC;

        /**
         * @brief 消息数据结构体定义
         * @param owner_ 消息数据的实际管理者，此为接口，实际内容定义于发布者命名空间。
         * @param publisher_cookie_ 用于释放消息所需的额外参数
         * @param payload_ 消息数据指针
         * @param length_ 消息数据长度
         */
        typedef struct msgpayload_s {
            msg_owner_t* owner_ = nullptr;
            uintptr_t publisher_cookie_ = 0;
            uint32_t length_ = 0;
            uint32_t* payload_ = nullptr;
        } msgpayload_t;
        msgpayload_t msgbody_;
    } event_container_t;

    /**
     * @brief 订阅者结构体定义
     * @param next_ 下一个订阅者指针挂点
     * @param rx_func_ 接收函数组
     * @param sub_maskmap 订阅事件位表，使用位运算的方式包含该任务订阅的所有事件类型
     * @param subcriber_cookie 订阅者的额外参数，接收函数用
     */
    typedef struct subcriber_s {
        subcriber_s* next_ = nullptr;

        // typedef struct event_handler_fn_s {
        //     event_container_t* ct;
        // } event_handler_fn_t;
        /**
         * @param rx_func_ 接收函数组 以 event_handler_fn_t 包装体作为入参.
         * @param sub_maskmap 订阅事件位表
         * @param subcriber_cookie 订阅者的额外参数
         */

        typedef struct recvmsg_s {
            uint8_t (*rx_func_[(uint32_t)TpcID_t::Count])(event_container_t*) {};
            TpcIDSize_t sub_maskmap {};
            uintptr_t subcriber_cookie {};
        } recvmsg_t;
        recvmsg_t rx_ {};
    } subcriber_t;

    EventHub();

    /**
     * @param subcriber 订阅者配置结构体
     * @param tgp_ 使用位运算的方式包含该任务订阅的所有事件类型,为便于运算，使用uint16_t进行封装
     * @return 成功订阅的事件数量
     */
    static uint8_t subscribe_topic(subcriber_t* subcriber, TpcIDMask_t tgp_);
    //
    /**
     *  发布者使用，用于发布事件：向系统fifo队列存入事件 或 发布紧急事件，此时不走事件总线任务分发，而是直接调接收者函数
     * @param owner 消息buffer所有者，如果msg_len为-1则无视此参数
     * @param publish_cookie 为回收消息 buffer 所需额外参数，如果msg_len为-1则无视此参数
     * @param topic 此消息的主题，查订阅表后发给订阅此类消息的订阅者。
     * @param priority 如果值为 urgent，直接调接收函数，不等待。
     * @param msg 消息本体，如果消息较大需要传入消息指针
     * @param msg_len msg_len为1则认为msg变量保存的是实际内容而不是指针
     * @return 0：接收到常规消息 1：接收到紧急消息 -1：缓冲区容量不足
     */
    uint8_t event_release(msg_owner_t* owner, uint32_t publish_cookie, topic_t topic , priority_t priority, uint32_t* msg, uint32_t msg_len) const;
    // 订阅者使用：减少事件生命周期
    static uint8_t decrease_e_lifespan(event_container_t* event);
    static uint8_t increase_e_lifespan(event_container_t* event);
private:
    // 初始化事件容器表和等待读取队列
    uint8_t init_erecvlist();
    uint8_t erecvlist_inited_ = 0;
    constinit static event_container_t erecvlist_[EVENTHUB_RECEIVE_MAX_NUM]; // 传入消息池
    constinit static event_container_t* erecvlist_empty_head_; // 空闲传入消息池链表头指针
    constinit static inline event_container_t* erecvlist_pri_idx_[sizeof(priority_t)] {};
    static uint8_t recvlist_clear_et(event_container_t* et); // et: event container
    uint8_t recvlist_get_empty_et(EventHub::event_container_t** p_et = nullptr) const;

    uint8_t init_maincache();
    uint8_t emaincache_inited_ = 0;
    constinit static event_container_t emaincache_[EVENTHUB_CONTAINER_MAX_EVENT_NUM]; // 消息缓存池
    constinit static event_container_t* emaincache_empty_head_; // 空闲内存池槽位链表头指针
    constinit static inline event_container_t* emaincache_pri_idx_[sizeof(priority_t)] {}; // 以优先级链表的方式保存事件，next指针将指向下一个同优先级事件，如果某个优先级没有事件，则指针为nullptr
    static uint8_t maincache_clear_et(event_container_t* et);
    uint8_t emaincache_get_empty_et(EventHub::event_container_t** p_et = nullptr) const;

    // 为什么要做一个专门的缓冲区？因为紧急事件可能会出现密集突发，上一个容器还没有解除占用，下一个事件就已经来了
    // 信息传递机制需要尽量统一信息装箱\拆箱流程
    uint8_t init_urgentcache();
    uint8_t eurgentcache_inited_ = 0;
    // static uint8_t eUrgentF_push(event_container_t* et);
    constinit static event_container_t eurgentcache_[EVENTHUB_URGENT_FIFO_MAX_NUM];
    constinit static event_container_t* eurgentcache_empty_head_;
    constinit static event_container_t* eurgentcache_full_head_;
    /**
     * @brief 获取一个空闲的紧急事件容器 event_urgent_cache_get_empty_container
     * @param p_et 容器指针
     * @return 成功 / 失败
     */
    uint8_t eurentcache_getemptyct(event_container_t** p_et = nullptr) const;

    /**
     * @brief 清空一个紧急事件容器，将其放回空闲链表
     * @param et 需要被清空的容器句柄
     * @return 成功 / 失败
     */
    static uint8_t eurentcache_clearct(event_container_t* et);
    uint8_t eurentcache_getfullctnum() const; // 调试用，获取当前有多少个容器被填充
    uint8_t eurentcache_getemptyctnum() const; // 调试用，获取当前有多少个容器是空闲的

    constinit static inline subcriber_t* topic_subscribers_[sizeof(TpcIDMask_t)] {}; // 订阅者链表 表头指针组
    constinit static inline event_container_t* published_event_ {}; // 已发布事件链表头

    /**
     * 从传入事件队列中取出全部事件并按优先级放入事件缓存
     * @param et 将返回当前最高优先级的事件容器指针
     * @return 取出的事件数量
     */
    uint8_t dump_and_sort(event_container_t* et) const;
    // 获取事件优先级
    static priority_t get_e_priority(const event_container_t* et);
    // 获取事件主题
    static topic_t get_e_topic(const event_container_t* et);
    // 获取事件类型
    static topic_t get_e_type(const event_container_t* et);

    /**
     * 从事件缓存中发布事件给订阅者，并加入已发布事件链表
     * @param et 需要被发布的事件容器句柄
     * @return 成功通知了多少个订阅者，如果为 0 意味着没人订阅，为 -1 则出错
     *
     */
    uint8_t publish_e_to_subscribers(event_container_t* et);
    // 检查已发布事件链表，清空到期容器
    uint8_t clear_expire_container();

    // 检查消息容器位置
    static uint8_t chech_et_location(event_container_t* et, et_location_t loc) ;

    // 从传入事件队列获取一个空容器并填充内容,返回void就是没有空容器

};
