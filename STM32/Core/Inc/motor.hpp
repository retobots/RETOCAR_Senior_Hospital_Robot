#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP
#include "main.h"
#include <math.h>

class Motor {
private:
    // Các thông số hằng số (Thay cho #define)
    static constexpr uint16_t PWM_MAX = 65535; // Giá trị PWM tối đa cho 16-bit timer
    static constexpr float W_MAX = 30.0f; // Tốc độ tối đa của động cơ (rad/s), cần điều chỉnh theo thực tế

	TIM_HandleTypeDef* _htim;
    uint32_t _channel; // TIM_CHANNEL_1, TIM_CHANNEL_2, v.v.
    GPIO_TypeDef* _port1; 	
    uint16_t _pin1; //
    GPIO_TypeDef* _port2; 
    uint16_t _pin2; 

    // Hàm nội bộ để chuẩn hóa (Private helper)
    float normalize(float w); // Chuyển đổi từ rad/s sang giá trị -1.0 đến 1.0

public:
    // Constructor đóng vai trò là Motor_Init
    Motor(TIM_HandleTypeDef* htim, uint32_t channel, 
          GPIO_TypeDef* port1, uint16_t pin1, 
          GPIO_TypeDef* port2, uint16_t pin2);

    // Huy có thể chọn đặt tốc độ theo rad/s hoặc theo %
    void setVelocity(float w_rad_s); // Đặt tốc độ động cơ theo rad/s
    void stop(); // Dừng động cơ (Coast)
};

#endif