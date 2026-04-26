#include "Kinematics.hpp"

Kinematics::Kinematics(float wheel_track, float wheel_radius) 
    : L(wheel_track), R(wheel_radius) {}

WheelVelocities Kinematics::getWheelSpeeds(float linear_v, float angular_w) {
    WheelVelocities wheels;
    
    // Tính vận tốc dài từng bánh (m/s)
    float v_l = linear_v - (angular_w * L / 2.0f);
    float v_r = linear_v + (angular_w * L / 2.0f);
    
    // Đổi sang vận tốc góc motor (rad/s) để nạp vào PID
    wheels.left = v_l / R;
    wheels.right = v_r / R;
    
    return wheels;
}

void Kinematics::getRobotVelocities(float w_l_actual, float w_r_actual, float &v_linear, float &w_angular) {
    // Đổi từ rad/s thực tế của motor sang m/s của bánh xe
    float v_l = w_l_actual * R;
    float v_r = w_r_actual * R;
    
    // Tính ngược lại vận tốc tổng quát của robot
    v_linear = (v_r + v_l) / 2.0f;
    w_angular = (v_r - v_l) / L;
}