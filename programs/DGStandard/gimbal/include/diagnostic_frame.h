#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace dgstandard {

// USB 异步发送时保留整帧数据；忙时丢弃新帧，不覆盖仍在发送的内存。
class DiagnosticFrame {
  public:
    void Begin(bool transport_idle) {
        writable_ = transport_idle;
        if (writable_)
            length_ = 0;
    }

    void Append(const char* data, std::size_t length) {
        if (!writable_)
            return;
        const std::size_t remaining = sizeof(data_) - length_;
        const std::size_t copied = length < remaining ? length : remaining;
        std::memcpy(data_ + length_, data, copied);
        length_ += copied;
    }

    uint8_t* Data() { return data_; }
    std::size_t Length() const { return writable_ ? length_ : 0; }

  private:
    uint8_t data_[2048] = {};
    std::size_t length_ = 0;
    bool writable_ = false;
};

}  // namespace dgstandard
