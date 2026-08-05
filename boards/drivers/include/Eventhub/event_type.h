#pragma once

// constexpr static uint16_t EMPTY_EVENT = 0;
// constexpr static uint16_t EVENT_0 = 0x00 | 0b1;
// constexpr static uint16_t EVENT_1 = EVENT_0 << 1;
// constexpr static uint16_t EVENT_2 = EVENT_0 << 2;
// constexpr static uint16_t EVENT_3 = EVENT_0 << 3;
// constexpr static uint16_t EVENT_4 = EVENT_0 << 4;
// constexpr static uint16_t EVENT_5 = EVENT_0 << 5;
// constexpr static uint16_t EVENT_6 = EVENT_0 << 6;
// constexpr static uint16_t EVENT_7 = EVENT_0 << 7;
// constexpr static uint16_t EVENT_8 = EVENT_0 << 8;
// constexpr static uint16_t EVENT_9 = EVENT_0 << 9;
// constexpr static uint16_t EVENT_10 = EVENT_0 << 10;
// constexpr static uint16_t EVENT_11 = EVENT_0 << 11;
// constexpr static uint16_t EVENT_12 = EVENT_0 << 12;
// constexpr static uint16_t EVENT_13 = EVENT_0 << 13;
// constexpr static uint16_t EVENT_14 = EVENT_0 << 14;
// constexpr static uint16_t EVENT_15 = EVENT_0 << 15;
// constexpr static uint16_t EMPTY_EVENT = 0xFFFF;

typedef uint8_t TpcIDSize_t;
enum class TpcID_t : TpcIDSize_t {
    Event0  = 0, // 掩码为0b1, 掩码为 0b0时意味着empty event
    Event1,
    Event2,
    Event3,
    Event4,
    Event5,
    Event6,
    Event7,
    Event8,
    Event9,
    Event10,
    Event11,
    Event12,
    Event13,
    Event14,
    Count,
    EMPTYTOPIC = 0xFF
};



// 将使用位运算检查事件类型，每个二进制位代表一种事件
typedef TpcID_t topic_t;
typedef uint16_t TpcIDMask_t;

// 从id查掩码
constexpr TpcIDMask_t get_eIDMask(TpcID_t id)
{
    return static_cast<TpcIDMask_t>(
        1u << static_cast<TpcIDSize_t>(id)
    );
}

// 从id查编号
constexpr TpcIDSize_t get_eIDNumber(TpcID_t id)
{
    return static_cast<TpcIDSize_t>(id);
}

// 从掩码查id
constexpr TpcID_t get_eID(TpcIDMask_t mask)
{
    if (mask == 0u || (mask & static_cast<TpcIDMask_t>(mask - 1u)) != 0u) {
        return TpcID_t::EMPTYTOPIC;
    }

    TpcIDSize_t number = 0u;
    while ((mask & 1u) == 0u) {
        mask = static_cast<TpcIDMask_t>(mask >> 1u);
        ++number;
    }

    return number < get_eIDNumber(TpcID_t::Count)
        ? static_cast<TpcID_t>(number)
        : TpcID_t::EMPTYTOPIC;
}

// 从编号查id
constexpr TpcID_t get_eID(TpcIDSize_t size)
{
    return static_cast<TpcID_t>(size);
}