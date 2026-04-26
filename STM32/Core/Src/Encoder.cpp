/*
 * Encoder.cpp
 *
 *  Created on: Apr 24, 2026
 *      Author: huy
 */

#include "Encoder.hpp"

Encoder::Encoder (TIM_HandleTypeDef* htim, bool _is_reserved) : _htim(htim), _prev_count(0), 
_is_reserved(_is_reserved) {}

void Encoder::begin() {
	HAL_TIM_Encoder_Start(_htim, TIM_CHANNEL_ALL); // Bắt đầu cả 2 kênh
}

void Encoder::reset() {
	__HAL_TIM_SET_COUNTER(_htim, 0); // Reset bộ đếm về 0
	_prev_count = 0; // Cập nhật lại giá trị trước đó
}

int32_t Encoder::update() {
    // 1. Lấy giá trị counter hiện tại (thường là 16-bit)
    uint32_t current_count = __HAL_TIM_GET_COUNTER(_htim);
    const int32_t ENC_MAX = 65536; // Giá trị cực đại của Timer 16-bit

    // 2. Tính toán độ chênh lệch (Delta) ban đầu
    int32_t delta = (int32_t)current_count - (int32_t)_prev_count;

    // 3. XỬ LÝ TRÀN SỐ (Cơ chế so sánh ngưỡng)
    // Nếu delta dương quá lớn (> 1/2 chu kỳ), nghĩa là thực tế wheel đang quay lùi và bị tràn từ 0 về 65535
    if (delta > (ENC_MAX / 2)) {
        delta -= ENC_MAX;
    }
    // Nếu delta âm quá lớn (< -1/2 chu kỳ), nghĩa là thực tế wheel đang quay tiến và bị tràn từ 65535 về 0
    else if (delta < -(ENC_MAX / 2)) {
        delta += ENC_MAX;
    }

    // 4. Lưu lại giá trị cho lần sau
    _prev_count = current_count;

    // 5. Đảo chiều nếu phần cứng được gắn ngược
    return _is_reserved ? -delta : delta;
}

