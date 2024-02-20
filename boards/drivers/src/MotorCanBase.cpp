/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
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

#include "MotorCanBase.h"
#include "arm_math.h"
#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "utils.h"

using namespace bsp;

namespace driver {

    /**
     * @brief standard can motor callback, used to update motor data
     *
     * @param data data that come from motor
     * @param args pointer to a MotorCANBase instance
     */
    static void can_motor_callback(const uint8_t data[], void* args) {
        MotorCANBase* motor = reinterpret_cast<MotorCANBase*>(args);
        motor->UpdateData(data);
    }

    bool MotorCANBase::is_init_ = false;
    uint16_t MotorCANBase::id_[10]={0};
    bsp::CAN* MotorCANBase::can_to_index_[10] = {nullptr};
    uint8_t MotorCANBase::group_cnt_=0;
    MotorCANBase* MotorCANBase::motors_[10][4]={{nullptr}};
    uint8_t MotorCANBase::motor_cnt_[10]={0};
    bsp::Thread* MotorCANBase::can_motor_thread_ = nullptr;
    uint32_t MotorCANBase::delay_time=1;

    MotorCANBase::MotorCANBase(bsp::CAN* can, uint16_t rx_id, uint16_t tx_id)
        : theta_(0), omega_(0), can_(can), rx_id_(rx_id) {
        // 大疆的电机，自动识别TX_ID
        if(tx_id == 0x00){
            constexpr uint16_t GROUP_SIZE = 4;
            constexpr uint16_t RX1_ID_START = 0x201;
            constexpr uint16_t RX2_ID_START = 0x205;
            constexpr uint16_t RX3_ID_START = 0x209;
            constexpr uint16_t TX1_ID = 0x200;
            constexpr uint16_t TX2_ID = 0x1ff;
            constexpr uint16_t TX3_ID = 0x2ff;

            RM_ASSERT_GE(rx_id, RX1_ID_START, "Invalid rx id");
            RM_ASSERT_LT(rx_id, RX3_ID_START + GROUP_SIZE, "Invalid rx id");
            if (rx_id >= RX3_ID_START)
                tx_id_ = TX3_ID;
            else if (rx_id >= RX2_ID_START)
                tx_id_ = TX2_ID;
            else
                tx_id_ = TX1_ID;
        }
        else{
                tx_id_ = tx_id;
        }

        // 如果是第一次初始化，需要创建一个后台线程以固定频率输出电机指令
        if(!is_init_){
            is_init_=true;
            bsp::thread_init_t thread_init = {
                .func = CanMotorThread,
                .args = nullptr,
                .attr = can_motor_thread_attr_
            };
            can_motor_thread_ = new bsp::Thread(thread_init);
            can_motor_thread_->Start();
            memset(id_,0xff,sizeof(id_));
            memset(motors_, 0, sizeof(motors_));
            group_cnt_=0;
            memset(motor_cnt_,0,sizeof(motor_cnt_));
        }
        // 如果已经初始化，需要检查是否有重复的ID，如果没有则加入到数组以使后台线程能够持续给电机输出数据
        for(uint8_t i=0;i<10;i++){
            if(tx_id_==id_[i] && can_to_index_[i]==can_){
                if(motor_cnt_[i]<4){
                    motors_[i][motor_cnt_[i]]=this;
                    motor_cnt_[i]++;
                    break;
                }
                else{
                    RM_ASSERT_TRUE(false,"Exceeding maximum of 4 motor commands per CAN message");
                }
            }else if(id_[i]==0xffff){
                id_[i]=tx_id_;
                can_to_index_[i]=can_;
                motors_[i][0]=this;
                group_cnt_++;
                motor_cnt_[i]++;
                break;
            }
        }

        // 默认PID参数
        omega_pid_ = control::ConstrainedPID();
        theta_pid_ = control::ConstrainedPID();


        align_angle_ = -1;  // Wait for Update to initialize
        motor_angle_ = 0;
        offset_angle_ = 0;
        servo_angle_ = 0;
        cumulated_angle_ = 0;
        inner_wrap_detector_ = new FloatEdgeDetector(0, PI);
        outer_wrap_detector_ = new FloatEdgeDetector(0, PI);

        target_=0;
    }

    void MotorCANBase::SetFrequency(uint32_t freq) {
        // 频率设置必须在电机初始化之前
        RM_ASSERT_FALSE(is_init_, "Frequency should be set before motor initialization");
        // 通过频率设置每秒输出时的延迟时间
        delay_time = 1000 / freq;
    }

    void MotorCANBase::TransmitOutput(MotorCANBase* motors[], uint8_t num_motors) {
        uint8_t data[8] = {0};

        RM_ASSERT_GT(num_motors, 0, "Meaningless empty can motor transmission");
        RM_ASSERT_LE(num_motors, 4, "Exceeding maximum of 4 motor commands per CAN message");
        //获取输出的数据到缓冲区
        for (uint8_t i = 0; i < num_motors; ++i) {
            RM_ASSERT_EQ(motors[i]->tx_id_, motors[0]->tx_id_, "tx id mismatch");
            RM_ASSERT_EQ(motors[i]->can_, motors[0]->can_, "can line mismatch");
            const uint8_t motor_idx = (motors[i]->rx_id_ - 1) % 4;
            const int16_t output = motors[i]->output_;
            data[2 * motor_idx] = output >> 8;
            data[2 * motor_idx + 1] = output & 0xff;
        }
        // 发送数据
        motors[0]->can_->Transmit(motors[0]->tx_id_, data, 8);
    }

    float MotorCANBase::GetTheta() const {
        return theta_;
    }

    float MotorCANBase::GetThetaDelta(float target) const {
        return wrap<float>(target - theta_, -PI, PI);
    }

    float MotorCANBase::GetOmega() const {
        return omega_;
    }

    float MotorCANBase::GetOmegaDelta(float target) const {
        return target - omega_;
    }

    int16_t MotorCANBase::GetCurr() const {
        return 0;
    }

    uint16_t MotorCANBase::GetTemp() const {
        return 0;
    }
    void MotorCANBase::CanMotorThread(void* args) {
        UNUSED(args);
        // 后台线程，用于持续输出电机指令
        while(1){
            // 遍历所有的电机组，对每个组的电机进行输出
            for(uint8_t i=0;i<group_cnt_;i++){
                // 计算每个组的电机的PID输出
                for(uint8_t j=0;j<motor_cnt_[i];j++){
                    motors_[i][j]->CalcOutput();
                }
                // 输出电机指令
                TransmitOutput(motors_[i],motor_cnt_[i]);
            }
            osDelay(delay_time);
        }

    }
    void MotorCANBase::SetTarget(float target) {
        // 设置目标值
        // 目标值的单位取决于电机的模式
        // 如果电机启动了角度环PID，则目标值为角度，单位为Rad
        // 如果电机没启动角度环PID的情况下启动了速度环PID，则目标值为角速度，单位为Rad/s
        target_ = target;
    }
    void MotorCANBase::CalcOutput() {
        float output = target_;
        if(mode_ & THETA){
            // 如果电机启动了角度环PID，则计算角度环PID输出
            if(mode_ & ABSOLUTE){
                // 如果电机启动了绝对控制模式，则直接使用角度环PID输出
                if(abs(output-theta_)>PI){
                    // 超过半圈，反方向走
                    output = output>theta_?output-2*PI:output+2*PI;
                }
                output = theta_pid_.ComputeOutput(output, GetOutputShaftTheta());
            }else{
                output = theta_pid_.ComputeOutput(output,GetOutputShaftTheta());
            }

        }
        if(mode_ & OMEGA){
            output = omega_pid_.ComputeOutput(output, GetOutputShaftOmega());
        }
        if(mode_ != NONE){
            SetOutput((int16_t)output);
        }

    }
    void MotorCANBase::ReInitPID(control::ConstrainedPID::PID_Init_t pid_init, uint8_t mode) {
        if(mode & OMEGA){
                omega_pid_.Reinit(pid_init);
        }
        else if(mode & THETA){
                theta_pid_.Reinit(pid_init);
        }
    }
    void MotorCANBase::SetMode(uint8_t mode) {
        mode_ = mode;
    }
    void MotorCANBase::UpdateData(const uint8_t* data) {
        UNUSED(data);


            // 角度速度环控制且非绝对控制模式时，需要使用统计总角度值
            // 获取初始化时的角度值
            if (align_angle_ < 0)
                align_angle_ = theta_;

            // 如果电机角度从接近 2PI 跳到接近 0，则回绕检测器将检测到下降沿，这意味着电机在穿过编码器边界时正向正方向转动。 反之亦然，电机角度从接近 0 跃升至接近 2PI

            motor_angle_ = theta_ - align_angle_;
            // 获得实际的电机屁股角度
            inner_wrap_detector_->input(motor_angle_);
            // 输入电机屁股的角度到边界检测器
            if (inner_wrap_detector_->negEdge())
                // 检测到下降沿，代表电机正向转动了一整圈
                offset_angle_ =
                    wrap<float>(offset_angle_ + 2 * PI / transmission_ratio_, 0, 2 * PI);
                // 更新实际的输出轴角度偏差
            else if (inner_wrap_detector_->posEdge())
                // 检测到上升沿，代表电机反向转动了一整圈
                offset_angle_ =
                    wrap<float>(offset_angle_ - 2 * PI / transmission_ratio_, 0, 2 * PI);
                // 更新实际的输出轴角度偏差

            servo_angle_ =
                wrap<float>(offset_angle_ + motor_angle_ / transmission_ratio_, 0, 2 * PI);
            // 更新实际输出轴角度
            outer_wrap_detector_->input(servo_angle_);
            // 输入实际输出轴角度到边界检测器

            // 绝对模式下不需要累积角度
            if(!(mode_ & ABSOLUTE)) {
                if (outer_wrap_detector_->negEdge())
                    cumulated_angle_ += 2 * PI;
                // 检测到下降沿，代表电机输出轴正向转动了一整圈
                else if (outer_wrap_detector_->posEdge())
                    cumulated_angle_ -= 2 * PI;
                // 检测到上升沿，代表电机输出轴反向转动了一整圈
            }

    }
    void MotorCANBase::SetTransmissionRatio(float ratio) {
        // 设置电机的传动比
        // 这里的传动比不是电机的实际传动比，而是电机与编码器的传动比
        transmission_ratio_ = ratio;
    }
    float MotorCANBase::GetOutputShaftTheta() const {
        return (servo_angle_+cumulated_angle_);
    }
    float MotorCANBase::GetOutputShaftOmega() const {
        return omega_/transmission_ratio_;
    }
    float MotorCANBase::GetTarget() const {
        return target_;
    }

    Motor3508::Motor3508(CAN* can, uint16_t rx_id) : MotorCANBase(can, rx_id) {
        can->RegisterRxCallback(rx_id, can_motor_callback, this);
    }

    void Motor3508::UpdateData(const uint8_t data[]) {
        const int16_t raw_theta = data[0] << 8 | data[1];
        const int16_t raw_omega = data[2] << 8 | data[3];
        raw_current_get_ = data[4] << 8 | data[5];
        raw_temperature_ = data[6];

        constexpr float THETA_SCALE = 2 * PI / 8192;  // digital -> rad
        constexpr float OMEGA_SCALE = 2 * PI / 60;    // rpm -> rad / sec
        theta_ = raw_theta * THETA_SCALE;
        omega_ = raw_omega * OMEGA_SCALE;

        connection_flag_ = true;
        MotorCANBase::UpdateData(data);
    }

    void Motor3508::PrintData() const {
        print("theta: % .4f ", GetTheta());
        print("output shaft theta: % .4f ", GetOutputShaftTheta());
        print("omega: % .4f ", GetOmega());
        print("output shaft omega: % .4f ", GetOutputShaftOmega());
        print("raw temperature: %3d ", raw_temperature_);
        print("raw current get: % d \r\n", raw_current_get_);
    }

    void Motor3508::SetOutput(int16_t val) {
        constexpr int16_t MAX_ABS_CURRENT = 12288;  // ~20A
        output_ = clip<int16_t>(val, -MAX_ABS_CURRENT, MAX_ABS_CURRENT);
    }

    int16_t Motor3508::GetCurr() const {
        return raw_current_get_;
    }

    uint16_t Motor3508::GetTemp() const {
        return raw_temperature_;
    }

    Motor6020::Motor6020(CAN* can, uint16_t rx_id,bool current_ctl) : MotorCANBase(can, rx_id) {
        can->RegisterRxCallback(rx_id, can_motor_callback, this);
        if(current_ctl){
            tx_id_--;
        }
    }

    void Motor6020::UpdateData(const uint8_t data[]) {
        const int16_t raw_theta = data[0] << 8 | data[1];
        const int16_t raw_omega = data[2] << 8 | data[3];
        raw_current_get_ = data[4] << 8 | data[5];
        raw_temperature_ = data[6];

        constexpr float THETA_SCALE = 2 * PI / 8192;  // digital -> rad
        constexpr float OMEGA_SCALE = 2 * PI / 60;    // rpm -> rad / sec
        theta_ = raw_theta * THETA_SCALE;
        omega_ = raw_omega * OMEGA_SCALE;

        connection_flag_ = true;
        MotorCANBase::UpdateData(data);
    }

    void Motor6020::PrintData() const {
        print("theta: % .4f ", GetTheta());
        print("omega: % .4f ", GetOmega());
        print("raw temperature: %3d ", raw_temperature_);
        print("raw current get: % d \r\n", raw_current_get_);
    }

    void Motor6020::SetOutput(int16_t val) {
        constexpr int16_t MAX_ABS_CURRENT = 30000;  // ~
        output_ = clip<int16_t>(val, -MAX_ABS_CURRENT, MAX_ABS_CURRENT);
    }

    int16_t Motor6020::GetCurr() const {
        return raw_current_get_;
    }

    uint16_t Motor6020::GetTemp() const {
        return raw_temperature_;
    }




    Motor2006::Motor2006(CAN* can, uint16_t rx_id) : MotorCANBase(can, rx_id) {
        can->RegisterRxCallback(rx_id, can_motor_callback, this);
    }

    void Motor2006::UpdateData(const uint8_t data[]) {
        const int16_t raw_theta = data[0] << 8 | data[1];
        const int16_t raw_omega = data[2] << 8 | data[3];
        raw_current_get_ = data[4] << 8 | data[5];

        constexpr float THETA_SCALE = 2 * PI / 8192;  // digital -> rad
        constexpr float OMEGA_SCALE = 2 * PI / 60;    // rpm -> rad / sec
        theta_ = raw_theta * THETA_SCALE;
        omega_ = raw_omega * OMEGA_SCALE;

        connection_flag_ = true;
        MotorCANBase::UpdateData(data);
    }

    void Motor2006::PrintData() const {
        print("theta: % .4f ", GetTheta());
        print("output shaft theta: % .4f ", GetOutputShaftTheta());
        print("omega: % .4f ", GetOmega());
        print("output shaft omega: % .4f ", GetOutputShaftOmega());
        print("raw current get: % d \r\n", raw_current_get_);
    }

    void Motor2006::SetOutput(int16_t val) {
        constexpr int16_t MAX_ABS_CURRENT = 10000;  // ~10A
        output_ = clip<int16_t>(val, -MAX_ABS_CURRENT, MAX_ABS_CURRENT);
    }

    int16_t Motor2006::GetCurr() const {
        return raw_current_get_;
    }



    /**
     * @brief default servomotor callback that overrides the standard can motor
     * callback
     *
     * @param data data that come from motor
     * @param args pointer to a ServoMotor instance
     */
    static void servomotor_callback(const uint8_t data[], void* args) {
        ServoMotor* servo = reinterpret_cast<ServoMotor*>(args);
        servo->UpdateData(data);
    }

    ServoMotor::ServoMotor(servo_t data, float align_angle, float proximity_in,
                           float proximity_out) {
        motor_ = data.motor;
        max_speed_ = data.transmission_ratio * data.max_speed;
        max_acceleration_ = data.transmission_ratio * data.max_acceleration;
        transmission_ratio_ = data.transmission_ratio;
        proximity_in_ = proximity_in;
        proximity_out_ = proximity_out;

        hold_ = true;
        target_angle_ = 0;
        align_angle_ = align_angle;  // Wait for Update to initialize
        motor_angle_ = 0;
        offset_angle_ = 0;
        servo_angle_ = 0;
        cumulated_angle_ = 0;
        inner_wrap_detector_ = new FloatEdgeDetector(0, PI);
        outer_wrap_detector_ = new FloatEdgeDetector(0, PI);
        hold_detector_ = new BoolEdgeDetector(false);

        omega_pid_.Reinit(data.omega_pid_param, data.max_iout, data.max_out);
        hold_pid_.Reinit(data.hold_pid_param, data.hold_max_iout, data.hold_max_out);

        // override origianal motor rx callback with servomotor callback
        data.motor->can_->RegisterRxCallback(data.motor->rx_id_, servomotor_callback, this);

        // Initially jam detection is not enabled, it is enabled only if user calls
        // RegisterJamCallback in the future.
        jam_callback_ = nullptr;
        detect_head_ = -1;
        detect_period_ = -1;
        detect_total_ = 0;
        detect_buf_ = nullptr;
    }

    servo_status_t ServoMotor::SetTarget(const float target, bool override) {
        if (!hold_ && !override)
            return INPUT_REJECT;
        servo_status_t dir = target < target_angle_ ? TURNING_ANTICLOCKWISE : TURNING_CLOCKWISE;
        target_angle_ = target;
        return dir;
    }

    void ServoMotor::SetMaxSpeed(const float max_speed) {
        if (max_speed > 0)
            max_speed_ = transmission_ratio_ * max_speed;
        else
            RM_EXPECT_TRUE(false, "Max speed should be positive");
    }

    void ServoMotor::SetMaxAcceleration(const float max_acceleration) {
        if (max_acceleration > 0)
            max_acceleration_ = transmission_ratio_ * max_acceleration;
        else
            RM_EXPECT_TRUE(false, "Max acceleration should be positive");
    }

    void ServoMotor::CalcOutput() {
        // if holding status toggle, reseting corresponding pid to avoid error
        // building up
        hold_detector_->input(hold_);
        if (hold_detector_->edge()) {
            omega_pid_.Reset();
            hold_pid_.Reset();
        }

        if (hold_detector_->negEdge())
            start_time_ = GetHighresTickMicroSec();

        // calculate desired output with pid
        int16_t command;
        float target_diff = (target_angle_ - servo_angle_ - cumulated_angle_) * transmission_ratio_;
        // v = sqrt(2 * a * d)
        uint64_t current_time = GetHighresTickMicroSec();
        if (!hold_) {
            float speed_max_start =
                (current_time - start_time_) / 10e6 * max_acceleration_ * transmission_ratio_;
            float speed_max_target = sqrt(2 * max_acceleration_ * abs(target_diff));
            float current_speed =
                speed_max_start > speed_max_target ? speed_max_target : speed_max_start;
            current_speed = clip<float>(current_speed, 0, max_speed_);
            command = omega_pid_.ComputeConstrainedOutput(
                motor_->GetOmegaDelta(sign<float>(target_diff, 0) * current_speed));
        } else {
            command = hold_pid_.ComputeConstrainedOutput(motor_->GetOmegaDelta(target_diff * 50));
        }
        motor_->SetOutput(command);

        // jam detection mechanism
        if (detect_buf_ != nullptr) {
            // update rolling sum and circular buffer
            detect_total_ += command - detect_buf_[detect_head_];
            detect_buf_[detect_head_] = command;
            detect_head_ = detect_head_ + 1 < detect_period_ ? detect_head_ + 1 : 0;

            // detect if motor is jammed
            jam_detector_->input(abs(detect_total_) >= jam_threshold_);
            if (jam_detector_->posEdge()) {
                servo_jam_t data;
                data.speed = max_speed_ / transmission_ratio_;
                jam_callback_(this, data);
            }
        }
    }

    bool ServoMotor::Holding() const {
        return hold_;
    }

    float ServoMotor::GetTarget() const {
        return target_angle_;
    }

    void ServoMotor::RegisterJamCallback(jam_callback_t callback, float effort_threshold,
                                         uint8_t detect_period) {
        constexpr int maximum_command = 32768;  // maximum command that a CAN motor can accept
        RM_ASSERT_TRUE(effort_threshold > 0 && effort_threshold <= 1,
                       "Effort threshold should between 0 and 1");
        // storing function pointer for future invocation
        jam_callback_ = callback;

        // create and initialize circular buffer
        detect_head_ = 0;
        detect_period_ = detect_period;
        detect_total_ = 0;
        if (detect_buf_ != nullptr)
            delete detect_buf_;
        detect_buf_ = new int16_t[detect_period];
        memset(detect_buf_, 0, detect_period);

        // calculate callback trigger threshold and triggering facility
        jam_threshold_ = maximum_command * effort_threshold * detect_period;
        jam_detector_ = new BoolEdgeDetector(false);
    }

    void ServoMotor::PrintData() const {
        print("Svo-align: % 10.6f ", align_angle_);
        print("Svo-theta: % 10.6f ", GetTheta());
        print("Svo-omega: % 10.6f ", GetOmega());
        print("Svo-target: % 10.6f ", target_angle_);
        if (hold_)
            print("Svo-status: holding ");
        else
            print("Svo-status: moving  ");
        motor_->PrintData();
    }

    float ServoMotor::GetTheta() const {
        return servo_angle_ + cumulated_angle_;
    }

    float ServoMotor::GetThetaDelta(const float target) const {
        return target - GetTheta();
    }

    float ServoMotor::GetOmega() const {
        return motor_->omega_ / transmission_ratio_;
    }

    float ServoMotor::GetOmegaDelta(const float target) const {
        return target - motor_->GetOmega() / transmission_ratio_;
    }

    void ServoMotor::UpdateData(const uint8_t data[]) {
        motor_->UpdateData(data);

        // TODO: change the align angle calibration method
        // This is a dumb method to get the align angle
        if (align_angle_ < 0)
            align_angle_ = motor_->theta_;

        // If motor angle is jumped from near 2PI to near 0, then wrap detecter will
        // sense a negative edge, which means that the motor is turning in positive
        // direction when crossing encoder boarder. Vice versa for motor angle jumped
        // from near 0 to near 2PI
        motor_angle_ = motor_->theta_ - align_angle_;
        inner_wrap_detector_->input(motor_angle_);
        if (inner_wrap_detector_->negEdge())
            offset_angle_ = wrap<float>(offset_angle_ + 2 * PI / transmission_ratio_, 0, 2 * PI);
        else if (inner_wrap_detector_->posEdge())
            offset_angle_ = wrap<float>(offset_angle_ - 2 * PI / transmission_ratio_, 0, 2 * PI);

        servo_angle_ = wrap<float>(offset_angle_ + motor_angle_ / transmission_ratio_, 0, 2 * PI);
        outer_wrap_detector_->input(servo_angle_);
        if (outer_wrap_detector_->negEdge())
            cumulated_angle_ += 2 * PI;
        else if (outer_wrap_detector_->posEdge())
            cumulated_angle_ -= 2 * PI;

        // determine if the motor should be in hold state
        float diff = abs(GetThetaDelta(target_angle_));
        if (!hold_ && diff < proximity_in_)
            hold_ = true;
        if (hold_ && diff > proximity_out_)
            hold_ = false;
    }

    SteeringMotor::SteeringMotor(steering_t data) {
        servo_t servo_data;
        servo_data.motor = data.motor;
        servo_data.max_speed = data.max_speed;
        servo_data.max_acceleration = data.max_acceleration;
        servo_data.transmission_ratio = data.transmission_ratio;
        servo_data.omega_pid_param = data.omega_pid_param;
        servo_data.max_iout = data.max_iout;
        servo_data.max_out = data.max_out;
        servo_ = new ServoMotor(servo_data, data.offset_angle);

        test_speed_ = data.test_speed;
        align_detect_func = data.align_detect_func;
        calibrate_offset = data.calibrate_offset;
        align_angle_ = 0;
        align_detector = new BoolEdgeDetector(false);
        align_complete_ = false;
    }

    float SteeringMotor::GetRawTheta() const {
        return servo_->GetTheta();
    }

    void SteeringMotor::PrintData() const {
        print("Str-align: %10.5f ", align_angle_);
        servo_->PrintData();
    }

    void SteeringMotor::TurnRelative(float angle) {
        servo_->SetTarget(servo_->GetTarget() + angle, true);
    }

    void SteeringMotor::TurnAbsolute(float angle) {
        servo_->SetTarget(angle);
    }

    bool SteeringMotor::AlignUpdate() {
        if (align_complete_) {
            servo_->SetTarget(align_angle_, true);
            servo_->CalcOutput();
            return true;
        } else if (align_detect_func()) {
            float current_theta = servo_->motor_->GetTheta();
            float offset = wrap<float>(servo_->align_angle_ - current_theta, -PI, PI);
            float current =
                (current_theta + offset - servo_->align_angle_) / servo_->transmission_ratio_ +
                servo_->offset_angle_ + servo_->cumulated_angle_;
            align_angle_ = current + calibrate_offset;
            align_complete_ = true;
            servo_->SetTarget(align_angle_, true);
            servo_->CalcOutput();
            return true;
        } else {
            servo_->motor_->SetOutput(servo_->omega_pid_.ComputeConstrainedOutput(
                servo_->motor_->GetOmegaDelta(test_speed_ * servo_->transmission_ratio_)));
        }
        return false;
    }

    void SteeringMotor::Update() {
        servo_->CalcOutput();
    }

    FlyWheelMotor::FlyWheelMotor(flywheel_t data) {
        motor_ = data.motor;
        max_speed_ = data.max_speed;
        target_speed_ = 0;
        is_inverted_ = data.is_inverted;
        omega_pid_ = control::PIDController(data.omega_pid_param);
    }
    void FlyWheelMotor::SetSpeed(float speed) {
        if (is_inverted_) {
            speed = -speed;
        }
        speed = clip<float>(speed, -max_speed_, max_speed_);
        target_speed_ = speed;
    }
    void FlyWheelMotor::CalcOutput() {
        motor_->SetOutput(
            omega_pid_.ComputeConstrainedOutput(motor_->GetOmegaDelta(target_speed_)));
    }
    float FlyWheelMotor::GetTarget() const {
        if (is_inverted_) {
            return -target_speed_;
        } else {
            return target_speed_;
        }
    }
    void FlyWheelMotor::PrintData() const {
        print("Fly-target: %2.5f ", target_speed_);
        motor_->PrintData();
    }
    void FlyWheelMotor::UpdateData(const uint8_t data[]) {
        motor_->UpdateData(data);
    }
    float FlyWheelMotor::GetTheta() const {
        return motor_->GetTheta();
    }
    float FlyWheelMotor::GetThetaDelta(const float target) const {
        return motor_->GetThetaDelta(target);
    }
    float FlyWheelMotor::GetOmega() const {
        return motor_->GetOmega();
    }
    float FlyWheelMotor::GetOmegaDelta(const float target) const {
        return motor_->GetOmegaDelta(target);
    }

}  // namespace driver
