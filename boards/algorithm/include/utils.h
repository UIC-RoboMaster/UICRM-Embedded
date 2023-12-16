/*###########################################################
 # Copyright (c) 2023. BNU-HKBU UIC RoboMaster              #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#pragma once
#include "main.h"

/**
 * @brief 将一个值限制在一个范围内
 *
 * @tparam T    值的类型
 * @param value 要限制的值
 * @param min   范围最小值
 * @param max   范围最大值
 *
 * @return 限制在范围内的值
 *
 * @note 如果min > max，行为未定义
 */
/**
 * @brief clip a value to fall into a given range
 *
 * @tparam T    type of the value
 * @param value value to the clipped
 * @param min   range min
 * @param max   range max
 *
 * @return clipped value that falls in the range [min, max]
 *
 * @note undefined behavior if min > max
 */
template <typename T>
T clip(T value, T min, T max) {
    return value < min ? min : (value > max ? max : value);
}

/**
 * @brief 环绕一个值使其落入给定范围
 *
 * @tparam T    值的类型
 * @param value 要环绕的值
 * @param min   范围最小值
 * @param max   范围最大值
 *
 * @return 落入范围[min, max]的环绕值
 *
 * @note 如果value距离min或max超过一个周期，行为未定义
 */
/**
 * @brief wrap around a value to fall into a given range
 *
 * @tparam T    type of the value
 * @param value value to be wrapped around
 * @param min   range min
 * @param max   range max
 *
 * @return wrapped around value that falls in the range [min, max]
 *
 * @note undefined behavior if value is more than one cycle away from min or
 * max
 */
template <typename T>
T wrap(T value, T min, T max) {
    const T range = max - min;
    return value < min ? value + range : (value > max ? value - range : value);
}

/**
 * @brief 环绕一个值使其落入给定范围，此函数会不断环绕直到落入范围
 *
 * @tparam T    值的类型
 * @param value 要环绕的值
 * @param min   范围最小值
 * @param max   范围最大值
 *
 * @return 落入范围[min, max]的环绕值
 *
 * @note 如果value距离min或max超过一个周期，行为未定义
 */
/**
 * @brief wrap around a value to fall into a given range, this function will
 * wrap around until it falls into the range
 *
 * @tparam T    type of the value
 * @param value value to be wrapped around
 * @param min   range min
 * @param max   range max
 *
 * @return wrapped around value that falls in the range [min, max]
 *
 * @note undefined behavior if value is more than one cycle away from min or
 * max
 */
template <typename T>
T wrapc(T value, T min, T max) {
    const T range = max - min;
    while (value < min) {
            value += range;
    }
    while (value > max) {
            value -= range;
    }
    return value;
}

/**
 * @brief 将一个值限制范围后，环绕使其落入给定的环绕范围
 *
 * @tparam T        值的类型
 * @param value     要环绕的值
 * @param min       限制范围最小值
 * @param max       限制范围最大值
 * @param range_min 环绕范围最小值
 * @param range_max 环绕范围最大值
 *
 * @return 落入环绕范围[range_min, range_max]的环绕值
 *
 * @note 如果min > max，行为未定义
 */
/**
 * @brief clip a value to fall into a given range; can wrap around domain
 *
 * @tparam T        type of the value
 * @param value     value to the clipped
 * @param min       clipping range min
 * @param max       clipping range max
 * @param range_min domain range min
 * @param range_max domain range max
 *
 * @return clipped value that falls in the range [min, max]
 *
 * @note undefined behavior if min > max
 */
template <typename T>
T wrapping_clip(T value, T min, T max, T range_min, T range_max) {
    value = wrap<T>(value, range_min, range_max);
    if (max >= min) {
        return value < min ? min : (value > max ? max : value);
    } else {
        // min > max; wrap around
        T middle = (min + max) / 2;
        if (value <= max && value >= range_min) {
            return value;
        } else if (value >= min && value <= range_max) {
            return value;
        } else if (value > max && value < middle) {
            return max;
        } else {
            return min;
        }
    }
}

/**
 * @brief 取最大值
 *
 * @tparam T     值的类型
 * @param value1 第一个值
 * @param value2 第二个值
 *
 * @return 两个值中的最大值
 */
/**
 * @brief max of two values
 *
 * @tparam T     type of the value
 * @param value1 first value
 * @param value2 second value
 *
 * @return the max of the values
 */
template <typename T>
T max(T value1, T value2) {
    return value1 < value2 ? value2 : value1;
}

/**
 * @brief 取最小值
 *
 * @tparam T     值的类型
 * @param value1 第一个值
 * @param value2 第二个值
 *
 * @return 两个值中的最小值
 */
/**
 * @brief min of two values
 *
 * @tparam T     type of the value
 * @param value1 first value
 * @param value2 second value
 *
 * @return the min of the values
 */
template <typename T>
T min(T value1, T value2) {
    return value1 < value2 ? value1 : value2;
}

/**
 * @brief 零点判断
 *
 * @tparam T    值的类型
 * @param value 要判断的值
 * @param zero  零点
 *
 * @return -1 小于零点，1 大于零点，0 等于零点
 */
/**
 * @brief get sign of a value
 *
 * @tparam T    type of the value
 * @param value value to be tested
 * @param zero  zero point of that value
 *
 * @return -1 for less than, 1 for greater than, and 0 for equal to
 */
template <typename T>
int sign(T value, T zero) {
    return value < zero ? -1 : (value > zero ? 1 : 0);
}

/**
 * @brief 边缘判断
 * @tparam T 受判断的数据类型
 */
template <typename T>
class EdgeDetector {
  public:
    /**
     * @brief 构造函数
     * @param initial 初始值
     */
    /**
     * @brief constructor
     * @param initial initial value
     */
    EdgeDetector<T>(T initial);
    /**
     * @brief 输入信号
     * @param signal 信号
     */
    /**
     * @brief input signal
     * @param signal signal
     */
    void input(T signal);
    /**
     * @brief 判断值是否变化（包括positive和negative）
     * @return true 变化，false 未变化
     */
    /**
     * @brief check if the value has changed (including positive and negative)
     * @return true if changed, false otherwise
     */
    bool edge();
    /**
     * @brief 判断值是否变大
     * @return true 变大，false 未变大
     */
    /**
     * @brief check if the value has increased
     * @return true if increased, false otherwise
     */
    bool posEdge();
    /**
     * @brief 判断值是否变小
     * @return true 变小，false 未变小
     */
    /**
     * @brief check if the value has decreased
     * @return true if decreased, false otherwise
     */
    bool negEdge();

  private:
    T prev_;
    bool posEdge_;
    bool negEdge_;
};

/**
 * @brief 布尔边缘判断
 * @details 用于判断布尔值的变化
 */
/**
 * @brief boolean edge detector
 * @details used to detect changes in boolean values
 */
class BoolEdgeDetector {
  public:
    /**
     * @brief 构造函数
     * @param initial 初始值
     */
    /**
     * @brief constructor
     * @param initial initial value
     */
    BoolEdgeDetector(bool initial);
    /**
     * @brief 输入信号
     * @param signal 信号
     */
    /**
     * @brief input signal
     * @param signal signal
     */
    void input(bool signal);
    /**
     * @brief 判断值是否变化（不论变高还是变低）
     * @return true 变化，false 未变化
     */
    /**
     * @brief check if the value has changed (including positive and negative)
     * @return true if changed, false otherwise
     */
    bool edge();
    /**
     * @brief 判断值是否变大
     * @return true 变大，false 未变大
     */
    /**
     * @brief check if the value has increased
     * @return true if increased, false otherwise
     */
    bool posEdge();
    /**
     * @brief 判断值是否变小
     * @return true 变小，false 未变小
     */
    /**
     * @brief check if the value has decreased
     * @return true if decreased, false otherwise
     */
    bool negEdge();
    /**
     * @brief 获取当前值
     * @return 当前值
     */
    /**
     * @brief get current value
     * @return current value
     */
    bool get();

  private:
    bool prev_;
    bool posEdge_;
    bool negEdge_;
};

/**
 * @brief 浮点数边缘判断
 * @details 用于判断浮点数的变化
 * @note 由于浮点数的精度问题，可能会出现误判
 */
/**
 * @brief float edge detector
 * @details used to detect changes in float values
 * @note due to the precision of float, there might be false positives
 */
class FloatEdgeDetector {
  public:
    /**
     * @brief 构造函数
     * @param initial   初始值
     * @param threshold 阈值
     */
    /**
     * @brief constructor
     * @param initial  initial value
     * @param threshold threshold
     */
    FloatEdgeDetector(float initial, float threshold = 0);
    /**
     * @brief 输入数据
     * @param signal 输入的数据
     */
    /**
     * @brief input signal
     * @param signal signal
     */
    void input(float signal);
    /**
     * @brief 判断值是否变化（不论变高还是变低）
     * @return true 变化，false 未变化
     */
    /**
     * @brief check if the value has changed (including positive and negative)
     * @return true if changed, false otherwise
     */
    bool edge();
    /**
     * @brief 判断值是否变大
     * @return true 变大，false 未变大
     */
    /**
     * @brief check if the value has increased
     * @return true if increased, false otherwise
     */
    bool posEdge();
    /**
     * @brief 判断值是否变小
     * @return true 变小，false 未变小
     */
    /**
     * @brief check if the value has decreased
     * @return true if decreased, false otherwise
     */
    bool negEdge();

  private:
    float prev_;
    float threshold_;
    bool posEdge_;
    bool negEdge_;
};

/**
 * @brief 斜坡信号源
 * @details 用于产生斜坡信号，避免信号突变
 */
/**
 * @brief ramp signal source
 * @details used to generate ramp signal to avoid sudden changes
 */
class RampSource {
  public:
    /**
     * @brief 构造函数
     * @param initial 初始值
     * @param min     最小值
     * @param max     最大值
     * @param step    步长
     */
    /**
     * @brief constructor
     * @param initial initial value
     * @param min    min value
     * @param max   max value
     * @param step step
     */
    RampSource(float initial, float min, float max, float step);
    /**
     * @brief 计算下一个周期
     * @param input 输入值
     * @return 下一个周期的值
     */
    /**
     * @brief calculate next value
     * @param input input value
     * @return next value
     */
    float Calc(float input);
    /**
     * @brief 获取当前值
     * @return 当前值
     */
    /**
     * @brief get current value
     * @return current value
     */
    float Get();
    /**
     * @brief 获取最大值
     * @return 最大值
     */
    /**
     * @brief get max value
     * @return max value
     */
    float GetMax();
    /**
     * @brief 获取最小值
     * @return 最小值
     */
    /**
     * @brief get min value
     * @return min value
     */
    float GetMin();
    /**
     * @brief 设置最大值
     * @param max 最大值
     */
    /**
     * @brief set max value
     * @param max max value
     */
    void SetMax(float max);
    /**
     * @brief 设置最小值
     * @param min 最小值
     */
    /**
     * @brief set min value
     * @param min min value
     */
    void SetMin(float min);
    /**
     * @brief 强制覆写当前值
     * @param current 当前值
     */
    /**
     * @brief set current value
     * @param current current value
     */
    void SetCurrent(float current);

  private:
    float input_;
    float output_;
    float min_;
    float max_;
    float step_;
};
