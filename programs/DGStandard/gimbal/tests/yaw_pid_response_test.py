"""Run the actual ConstrainedPID and DGStandard Yaw parameters on a host compiler.

Checks the millisecond integral response, continuous small-angle correction and
output limits. This does not model the vehicle's inertia, friction or resonance.
"""

import argparse
from pathlib import Path
import re
import subprocess
import tempfile


HARNESS = r'''
#include "pid.h"
#include "yaw_stabilization.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
using std::abs;
constexpr float PI = dgstandard::kYawPi;
template <typename T> T clip(T v, T lo, T hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
namespace control {
@IMPLEMENTATION@
}

static void require(bool condition, const char* message) {
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
int main() {
    control::ConstrainedPID::PID_Init_t angle_parameters = @ANGLE@;
    control::ConstrainedPID::PID_Init_t speed_parameters = @SPEED@;
    auto previous_angle_parameters = angle_parameters;
    previous_angle_parameters.kp = 12;
    previous_angle_parameters.deadband = PI / 1800;
    auto previous_speed_parameters = speed_parameters;
    previous_speed_parameters.ki = 80;
    control::ConstrainedPID angle(angle_parameters), speed(speed_parameters);
    control::ConstrainedPID previous_angle(previous_angle_parameters);
    control::ConstrainedPID previous_speed(previous_speed_parameters);

    // A fixed small speed error for 100 ms used to build ten times the integral.
    for (int i = 0; i < 100; ++i) {
        speed.ComputeOutput(0.1f, 0);
        previous_speed.ComputeOutput(0.1f, 0);
    }
    const float integral = speed.State().iout;
    const float previous_integral = previous_speed.State().iout;
    require(integral > 0 && integral < previous_integral * 0.11f,
            "small speed errors must accumulate substantially less integral");

    // No jump from zero to a finite speed command at the old angle deadband.
    require(previous_angle.ComputeOutput(0.001f, 0) == 0,
            "baseline must reproduce the previous angle deadband");
    require(angle.ComputeOutput(0.001f, 0) > 0,
            "sub-deadband heading errors must still receive continuous correction");
    float maximum_jump = 0, previous_output = 0;
    for (int i = 0; i < 30; ++i)
        angle.ComputeOutput(-0.003f, 0);
    for (int i = -300; i <= 300; ++i) {
        const float output = angle.ComputeOutput(i * 0.00001f, 0);
        if (i > -300)
            maximum_jump = std::fmax(maximum_jump, std::fabs(output - previous_output));
        previous_output = output;
    }
    require(maximum_jump < 0.001f, "near-zero angle sweep must not contain a deadband step");

    // A saturated demand and subsequent reversal must stay inside both bounds.
    for (int i = 0; i < 6000; ++i) {
        const float command = angle.ComputeOutput(i < 3000 ? 3.0f : -3.0f, 0);
        const float output = speed.ComputeOutput(command, i < 3000 ? -2.0f : 2.0f);
        require(std::fabs(command) <= dgstandard::kYawMaxRate + 0.001f,
                "angle controller must retain its speed limit");
        require(std::fabs(output) <= dgstandard::kYawMaxOutput + 0.001f,
                "speed controller must retain its output limit");
        require(std::fabs(speed.State().iout) <= speed_parameters.max_iout + 0.001f,
                "integral must remain bounded through reversal");
    }
    require(speed.State().output < 0, "reversed command must eventually reverse output");
    std::printf("PASS: actual Yaw PID; 100 ms integral %.1f -> %.1f, continuous small-angle correction, saturation and reversal bounds\n",
                previous_integral, integral);
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="g++")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[4]
    algorithm = root / "boards/algorithm"
    gimbal = root / "programs/DGStandard/gimbal"
    source = (algorithm / "src/pid.cpp").read_text(encoding="utf-8")
    # Compile the real constrained PID methods; CMSIS's separate PIDController
    # class is unused and excluded, so only its header type needs a stub.
    implementation = source[source.index("    ConstrainedPID::ConstrainedPID()"):source.rfind("}")]
    task = (gimbal / "src/gimbal_task.cpp").read_text(encoding="utf-8")

    def parameters(name):
        match = re.search(r"\b" + name + r"\s*=\s*(\{.*?\});", task, re.S)
        if not match:
            raise RuntimeError(f"Missing actual Yaw initialization: {name}")
        return match.group(1)

    harness = HARNESS.replace("@IMPLEMENTATION@", implementation)
    harness = harness.replace("@ANGLE@", parameters("yaw_motor_theta_pid_init"))
    harness = harness.replace("@SPEED@", parameters("yaw_motor_omega_pid_init"))
    with tempfile.TemporaryDirectory(prefix="dgstd-yaw-pid-") as directory:
        temp = Path(directory)
        (temp / "main.h").write_text("#pragma once\n#include <cstdint>\n#define UNUSED(x) (void)(x)\n", encoding="utf-8")
        (temp / "arm_math.h").write_text("#pragma once\nstruct arm_pid_instance_f32 {};\n", encoding="utf-8")
        test_source = temp / "yaw_pid_test.cpp"
        test_source.write_text(harness, encoding="utf-8")
        executable = temp / "yaw_pid_test.exe"
        subprocess.run([args.cxx, "-std=c++14", "-Wall", "-Wextra", "-Werror",
                        "-I", str(temp), "-I", str(algorithm / "include"),
                        "-I", str(gimbal / "include"), str(test_source), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
