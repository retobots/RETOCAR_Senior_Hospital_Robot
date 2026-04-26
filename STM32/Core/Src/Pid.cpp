#include "Pid.hpp"
#include <math.h>

PID::PID(float kp, float ki, float kd, float out_limit, float int_limit)
    : _kp(kp), _ki(ki), _kd(kd), 
      _out_max(out_limit), _out_min(-out_limit),
      _integ_max(int_limit), _integ_min(-int_limit),
      _integral(0.0f), _prev_error(0.0f), _prev_measured(0.0f), _first_run(true) {}

float PID::compute(float setpoint, float measured, float dt) {
	if (dt <= 0.0f) return 0.0f;

    // 1. Reset integral nếu mục tiêu dừng (Tránh trôi robot)
    if (fabsf(setpoint) < 0.01f) {
        _integral = 0.0f;
    }

	// 2. Tính toán sai số
	float error = setpoint - measured;
	// 3. Thành phần P
	float p_out = _kp * error;

	// 3. --- LOGIC ANTI-WINDUP THÔNG MINH (Kế thừa từ mẫu) ---
    // Tính toán output tiềm năng trước khi thêm thành phần D
    float potential_output = p_out + (_ki * _integral);

    // Điều kiện tích lũy I:
    // - Output chưa bão hòa
    // - HOẶC sai số đang giúp kéo output ra khỏi vùng bão hòa
    if ((potential_output < _out_max && potential_output > _out_min) ||
        (error > 0 && potential_output < _out_min) ||
        (error < 0 && potential_output > _out_max)) 
    {
        _integral += error * dt;
        
        // Giới hạn cứng tích phân để an toàn lớp thứ 2
        if (_integral > _integ_max) _integral = _integ_max;
        if (_integral < _integ_min) _integral = _integ_min;
    }

    float i_out = _ki * _integral;

    // 4. Thành phần D (Derivative on Measurement)
    float d_out = 0.0f;
    if (!_first_run) {
        float derivative = (measured - _prev_measured) / dt;
        d_out = -_kd * derivative; // Dấu âm vì tính dựa trên đo lường (measured)
    }
    
    _prev_measured = measured;
    _first_run = false;

    // 5. Tổng hợp và giới hạn output cuối cùng
    float output = p_out + i_out + d_out;

    if (output > _out_max) output = _out_max;
    if (output < _out_min) output = _out_min;

    return output;
}


void PID::reset() {
    _integral = 0.0f;
    _prev_error = 0.0f;
    _prev_measured = 0.0f;
    _first_run = true;
}

void PID::setTunings(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}
