#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include "PID.hpp"
#include "main.h"
#include "Encoder.hpp"
#include "motor.hpp"
#include "Kinematics.hpp"
#define ACCELERATION_LIMIT 5.0f // rad/s^2, giới hạn gia tốc để tránh sốc cơ học
#define FILTER_ALPHA 0.85f // Hệ số lọc cho bộ lọc thông thấp (Low-pass filter)
#define SAMPLING_TIME_S  0.02f // 20ms = 0.02s, tương ứng với tần số điều khiển 50Hz   
#define PI               3.14159265359f
#define PPR              330.0f // Pulse Per Revolution - Số xung/vòng của encoder
// Hằng số để chuyển từ xung/thời gian lấy mẫu sang rad/s
#define TICKS_TO_RAD_PER_S ((2.0f * PI) / (PPR * SAMPLING_TIME_S))

class RobotController {
private:
    // PID cho 2 bánh: Trái (Left) và Phải (Right)
	Encoder EncoderL;
	Encoder EncoderR;
	Motor motorL;
	Motor motorR;
    PID pid_l, pid_r;
    Kinematics kinematics{0.3f, 0.05f}; // L=0.3m, R=0.05m (Cần điều chỉnh theo thực tế)

    float velocity_target[2] = {0, 0};   // 0: Left, 1: Right
    float velocity_current[2] = {0, 0};
    float filtered_actual[2] = {0, 0};
    int16_t prev_enc[2] = {0, 0};

public:
    RobotController() : 
		EncoderL(&htim4, false), EncoderR(&htim1, true),
		motorL(&htim3, TIM_CHANNEL_2, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15),
		motorR(&htim3, TIM_CHANNEL_1, GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13),
        // Kp, Ki, Kd, Out_Limit (W_MAX), Int_Limit
        pid_l(0.5f, 1.5f, 0.01f, 30.0f, 10.0f),
        pid_r(0.5f, 1.5f, 0.01f, 30.0f, 10.0f) {}
        void move(float linear_v, float angular_w)
        {
            WheelVelocities target_speeds = kinematics.getWheelSpeeds(linear_v, angular_w);
            setTargetVelocities(target_speeds.left, target_speeds.right);
        }

    void setTargetVelocities(float v_left, float v_right);
    void update(void); // Gọi trong ngắt Timer 20ms
    
    // Hàm lấy vận tốc thực tế đã lọc (dùng cho Odometry sau này)
    float getFilteredVelocityL() { return filtered_actual[0]; }
    float getFilteredVelocityR() { return filtered_actual[1]; }
};

#endif