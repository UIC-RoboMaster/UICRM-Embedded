#include "../include/diagnostic_frame.h"

#include <cassert>
#include <cstdio>
#include <cstring>

int main() {
    dgstandard::DiagnosticFrame frame;
    char line[] = "DGSTD STABLE-V9 USB+UART6\r\n";
    frame.Begin(true);
    frame.Append(line, sizeof(line) - 1);
    const uint8_t* in_flight = frame.Data();
    const std::size_t length = frame.Length();
    // 模拟 diagnostic_line 返回、栈内格式化缓冲被后续一行重用。
    std::memset(line, 'X', sizeof(line));
    assert(std::memcmp(in_flight, "DGSTD STABLE-V9 USB+UART6\r\n", length) == 0);
    // USB BUSY 时跳过新帧，异步传输的数据保持原样。
    frame.Begin(false);
    frame.Append("corrupted", 9);
    assert(frame.Length() == 0);
    assert(std::memcmp(in_flight, "DGSTD STABLE-V9 USB+UART6\r\n", length) == 0);
    frame.Begin(true);
    frame.Append("mode=SPIN\r\n", 11);
    assert(frame.Length() == 11);
    assert(std::memcmp(frame.Data(), "mode=SPIN\r\n", 11) == 0);
    char oversized[3000];
    std::memset(oversized, 'A', sizeof(oversized));
    frame.Begin(true);
    frame.Append(oversized, sizeof(oversized));
    assert(frame.Length() == 2048);
    frame.Append("more", 4);
    assert(frame.Length() == 2048);
    for (std::size_t i = 0; i < frame.Length(); ++i)
        assert(frame.Data()[i] == 'A');
    std::puts("PASS: USB frame copies stack data, preserves in-flight bytes while busy, resumes and bounds overflow");
}
