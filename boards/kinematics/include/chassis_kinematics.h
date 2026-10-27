#pragma once

namespace kinematics {
    // Legacy wheel-equivalent coordinates: X right, Y forward, turn clockwise.
    // These values are NOT chassis m/s or rad/s. Geometry/unit conversion,
    // input validation and actuator limits belong to the caller.
    struct planar_motion {
        float x = 0;
        float y = 0;
        float turn = 0;
    };

    // Signed motor speeds, including the legacy right-side motor reversal.
    struct four_wheel_speeds {
        float front_left = 0;
        float front_right = 0;
        float back_left = 0;
        float back_right = 0;
    };

    // Chassis command -> wheel speeds, before speed limiting.
    // Keep independent solvers so each layout's geometry can evolve separately.
    inline four_wheel_speeds MecanumInverse(float x, float y, float turn, float offset = 0) {
        return {
            y + x + turn * (1 - offset),
            -(y - x - turn * (1 - offset)),
            y - x + turn * (1 + offset),
            -(y + x - turn * (1 + offset)),
        };
    }

    // Legacy four-omni mixing currently has the same coefficients as mecanum.
    inline four_wheel_speeds OmniInverse(float x, float y, float turn, float offset = 0) {
        return {
            y + x + turn * (1 - offset),
            -(y - x - turn * (1 - offset)),
            y - x + turn * (1 + offset),
            -(y + x - turn * (1 + offset)),
        };
    }

    // Wheel speeds -> wheel-equivalent chassis motion. Uses the same offset
    // and motor sign convention as the corresponding inverse solver.
    inline planar_motion MecanumForward(const four_wheel_speeds& wheels, float offset = 0) {
        const float turn = (wheels.front_left + wheels.front_right + wheels.back_left + wheels.back_right) * 0.25f;
        return {
            (wheels.front_left + wheels.front_right - wheels.back_left - wheels.back_right) * 0.25f + offset * turn,
            (wheels.front_left + wheels.back_left - wheels.front_right - wheels.back_right) * 0.25f,
            turn,
        };
    }

    inline planar_motion OmniForward(const four_wheel_speeds& wheels, float offset = 0) {
        const float turn = (wheels.front_left + wheels.front_right + wheels.back_left + wheels.back_right) * 0.25f;
        return {
            (wheels.front_left + wheels.front_right - wheels.back_left - wheels.back_right) * 0.25f + offset * turn,
            (wheels.front_left + wheels.back_left - wheels.front_right - wheels.back_right) * 0.25f,
            turn,
        };
    }
}  // namespace kinematics
