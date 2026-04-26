#ifndef PID_HPP
#define PID_HPP

class PID {
private:
    // Hằng số PID
    float _kp;
    float _ki;
    float _kd;

    // Biến lưu trạng thái
    float _integral;
    float _prev_error;
    float _prev_measured;

    // Giới hạn chống bão hòa và giới hạn đầu ra
    float _out_max;
    float _out_min;
    float _integ_max;
    float _integ_min;

    // Cờ kiểm tra lần chạy đầu tiên để tránh lỗi vi phân (D)
    bool _first_run;

public:
    /**
	 * @brief Constructor để khởi tạo các thông số PID và giới hạn
	 * @param kp: Hằng số tỷ lệ (Proportional gain)
	 * @param ki: Hằng số tích phân (Integral gain)
	 * @param kd: Hằng số vi phân (Derivative gain)
	 * @param out_limit: Giới hạn tuyệt đối của đầu ra (Cả dương và âm)
	 * @param int_limit: Giới hạn tuyệt đối của phần tích phân để tránh bão hòa
	 */
    PID(float kp, float ki, float kd, float out_limit, float int_limit);

    /**
     * @brief Hàm tính toán PID
     * @param setpoint: Vận tốc mong muốn (Target rad/s)
     * @param measured: Vận tốc thực tế (Actual rad/s)
     * @param dt: Thời gian lấy mẫu (Sampling time, ví dụ 0.02s)
     * @return Giá trị điều khiển (PWM hoặc hệ số chuẩn hóa)
     */
    float compute(float setpoint, float measured, float dt);

    /**
     * @brief Reset các giá trị tích phân và sai số (Dùng khi robot dừng hẳn)
     */
    void reset();

    // Các hàm để thay đổi thông số PID khi đang chạy (tùy chọn)
    void setTunings(float kp, float ki, float kd);
};

#endif