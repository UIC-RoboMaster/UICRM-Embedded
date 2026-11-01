"""用传感器/AHRS 桩执行实际 IMU_typeC::Update，回归温度回落导致的姿态冻结。"""

import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def update_definition(source):
    start = source.index("void IMU_typeC::Update()")
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


FIXTURE = r'''
#include <cstdint>
#include <cstdio>
#include <cstdlib>

static void require(bool condition, const char* message) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

namespace bsp {
struct SensorStub {
    float rate = 0, temperature = 44;
    void gyro_read_over(uint8_t*, float* gyro) { gyro[0] = gyro[1] = 0; gyro[2] = rate; }
    void accel_read_over(uint8_t*, float* accel, float*) { accel[0] = accel[1] = 0; accel[2] = 9.8f; }
    void temperature_read_over(uint8_t*, float* temp) { *temp = temperature; }
};

class IMU_typeC {
public:
    void Update();
    bool calibrate_ = false, calidone_ = false, useMag_ = false;
    unsigned count_ = 0, zeroDriftTry = 3;
    float zeroDrift[3] = {}, zeroDriftTemp[3] = {};
    volatile uint32_t attitude_update_count_ = 0;
    uint8_t gyro_update_flag = 0, accel_update_flag = 0, accel_temp_update_flag = 0;
    uint8_t gyro_dma_rx_buf[12] = {}, accel_dma_rx_buf[12] = {}, accel_temp_dma_rx_buf[12] = {};
    float Temp = 0, TempPWM = 0;
    struct { float temp = 45; } heater_param_;
    struct { float gyro[3] = {}, accel[3] = {}, temp = 0, time = 0; } BMI088_real_data_;
    struct { float mag[3] = {}; } IST8310_real_data_;
    SensorStub BMI088_;
    float INS_accel[3] = {}, INS_gyro[3] = {}, INS_angle[3] = {}, INS_quat[4] = {};
    float accel_fliter_1[3] = {}, accel_fliter_2[3] = {}, accel_fliter_3[3] = {};
    const float fliter_num[3] = {1.929454f, -0.9317835f, 0.00232946f};
    unsigned ahrs_calls = 0, heater_calls = 0;
    float last_rate = 0;
    float TempControl(float) { ++heater_calls; return 0.5f; }
    void AHRS_update(float*, float, float* gyro, float*, float*) { ++ahrs_calls; last_rate = gyro[2]; }
    void GetAngle(float*, float* yaw, float* pitch, float* roll) {
        *yaw += last_rate * 0.001f; *pitch = *roll = 0;
    }
    void Sample(float temperature, float rate) {
        BMI088_.temperature = temperature;
        BMI088_.rate = rate;
        gyro_update_flag = 1 << IMU_NOTIFY_SHFITS;
        accel_update_flag = accel_temp_update_flag = 1 << IMU_UPDATE_SHFITS;
        Update();
    }
};

@UPDATE@
}  // namespace bsp

int main() {
    bsp::IMU_typeC imu;
    imu.Sample(44, 0);
    require(imu.ahrs_calls == 0, "must request calibration before attitude updates");
    imu.calibrate_ = true;
    for (int i = 0; i < 10; ++i)
        imu.Sample(42, 0);
    require(!imu.calidone_ && imu.count_ == 0, "cold start must still wait for warm-up");
    require(imu.heater_calls == 11, "heater must keep running during warm-up");
    for (unsigned i = 0; i < imu.zeroDriftTry; ++i)
        imu.Sample(44, 0);
    require(imu.calidone_, "warm calibration must complete");
    require(imu.ahrs_calls == 0, "calibration samples must not feed attitude integration");

    for (int i = 0; i < 1000; ++i)
        imu.Sample(42.5f, 0.74f);
    require(imu.ahrs_calls == 1000, "calibrated IMU must keep updating below 43 C");
    require(imu.INS_angle[0] > 0.73f && imu.INS_angle[0] < 0.75f, "yaw must track motion during temperature dip");
    require(imu.attitude_update_count_ == 1000, "sample counter must count completed attitude updates");
    for (int i = 0; i < 200; ++i)
        imu.Sample(i % 2 ? 43.2f : 42.8f, 0.2f);
    require(imu.ahrs_calls == 1200 && imu.attitude_update_count_ == 1200,
            "crossing warm threshold repeatedly must not pause or recalibrate");
    require(imu.count_ == imu.zeroDriftTry, "completed calibration must not restart sampling");
    std::puts("PASS: actual IMU Update waits for initial warm calibration, then keeps attitude/gyro/heater/sample count updating through temperature dips");
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="g++")
    parser.add_argument("--source", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[4]
    driver = root / "boards/drivers/DJI_Board_TypeC"
    source = (args.source or driver / "src/bsp_imu.cpp").read_text(encoding="utf-8")
    header = (driver / "include/bsp_imu.h").read_text(encoding="utf-8")
    # 只抽出实际 Update 函数及所用常量；不复制温度门槛或校准逻辑到测试。
    defines = "\n".join(re.findall(
        r"^#define (?:IMU_(?:NOTIFY|UPDATE)_SHFITS|BMI088_(?:GYRO|ACCEL)_RX_BUF_DATA_OFFSET|INS_(?:YAW|PITCH|ROLL)_ADDRESS_OFFSET)\s+.*$",
        header, re.MULTILINE))
    cpp = defines + "\n" + FIXTURE.replace("@UPDATE@", update_definition(source))
    with tempfile.TemporaryDirectory(prefix="dgstd-imu-temperature-") as folder:
        unit = Path(folder) / "imu_temperature.cpp"
        executable = Path(folder) / "imu_temperature.exe"
        unit.write_text(cpp, encoding="utf-8")
        subprocess.run([args.cxx, "-std=c++14", "-Wall", "-Wextra", "-Werror", str(unit), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
