#include "vehicom/vehicom.hpp"

namespace vehicom
{
    void VEHICOM::commonInfoSetup()
    {
        // インクリメントカウンタ
        udp_send_cnt_ = (udp_send_cnt_ + 1) % 255;
        this->out_com_.increCount = udp_send_cnt_;
    }

    void VEHICOM::timeInfoSetup()
    {
        // 現在時刻取得
        std::tm * tm = nullptr;
        std::timespec ts;
        if (std::timespec_get(&ts, TIME_UTC) == 0) {
            RCLCPP_ERROR_STREAM(get_logger(), "Failed to get time now");
            return;
        }
        time_t time = ts.tv_sec;
        tm = std::localtime(&time);

        // 時刻格納
        this->out_com_.tLeap = 0; // bool value
        this->out_com_.tHour = (uint8_t)((0x7F & tm->tm_hour) >> 1);
        this->out_com_.tMin = tm->tm_min;
        uint16_t make_v = static_cast<uint16_t>((tm->tm_sec + ts.tv_nsec / 1000000000.0) * 1000.0);
        make_v = htons(make_v);
        this->out_com_.tSec[0] = uint8_t(make_v & 0x00FF);
        this->out_com_.tSec[1] = uint8_t((make_v & 0xFF00) >> 8);
    }
}