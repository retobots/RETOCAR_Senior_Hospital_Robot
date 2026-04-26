#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "main.h"

class Encoder {
private:
    TIM_HandleTypeDef* _htim;
    int16_t _prev_count;
	bool _is_reserved; // Dùng để đánh dấu nếu cần thiết cho việc đảo chiều hoặc các tính năng khác
    static constexpr int32_t ENCODER_MAX = 65535; // Giá trị max của bộ đếm 16-bit

public:
    // Constructor
    Encoder(TIM_HandleTypeDef* htim, bool is_reserved = false);

    // Bắt đầu Timer Encoder
    void begin();

    // Hàm cập nhật và lấy giá trị chênh lệch (Delta)
    int32_t update();
    
    // Hàm reset về 0
    void reset();
};

#endif