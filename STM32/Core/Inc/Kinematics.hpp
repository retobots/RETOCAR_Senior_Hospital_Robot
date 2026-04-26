#ifndef KINEMATICS_H
#define KINEMATICS_H

struct WheelVelocities {
    float left;
    float right;
};

class Kinematics {
public:
    // L: Khoảng cách 2 bánh (m), R: Bán kính bánh xe (m)
    Kinematics(float wheel_track, float wheel_radius);

    // Inverse Kinematics: Từ (v, w) robot -> (w_left, w_right) motor (rad/s)
    WheelVelocities getWheelSpeeds(float linear_v, float angular_w);

    // Forward Kinematics: Từ (w_left, w_right) motor -> (v, w) robot (m/s, rad/s)
    void getRobotVelocities(float w_l_actual, float w_r_actual, float &v_linear, float &w_angular);

private:
    float L;
    float R;
};

#endif