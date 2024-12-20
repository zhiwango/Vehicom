#ifndef VEHICOM__VEHICOM_HPP_
#define VEHICOM__VEHICOM_HPP_

// Core
#include <math.h>
#include <chrono>
#include <arpa/inet.h>
#include <cstdint>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <udp_driver/udp_driver.hpp>

// Autoware
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

namespace vehicom
{
  using nav_msgs::msg::Odometry;
  using tier4_rtc_msgs::msg::CooperateStatusArray;
  using autoware_adapi_v1_msgs::msg::VelocityFactorArray;
  using autoware_vehicle_msgs::msg::VelocityReport;
  using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  using drivers::udp_driver::UdpDriver;

class VEHICOM : public rclcpp::Node
{
private:
  struct alignas(1) TD001
  {
    // DF_共通領域管理情報           8byte
    uint8_t comServStdID : 3;     // DE_共通サービス規格ID - 3bit
    uint8_t msgID : 2;            // DE_メッセージID - 2bit
    uint8_t Ver : 3;              // DE_バージョン情報 - 3bit
    uint8_t vID[4];               // DE_車両ID - 32bit
    uint8_t increCount;           // DE_インクリメントカウンタ - 8bit
    uint8_t comAppDataLen;  // DE_共通アプリデータ長 - 8bit
    uint8_t optFlg;               // DE_オプションフラグ - 8bit

    // DF_時刻情報               4byte
    uint8_t tLeap : 1;          // DE_うるう秒補正情報 - 1bit
    uint8_t tHour : 7;          // DE_時刻（時）- 7bit
    uint8_t tMin;               // DE_時刻（分）- 8bit
    uint8_t tSec[2];            // DE_時刻（秒）- 16bit

    // DF_位置情報               11byte
    uint8_t Latitude[4];        // DE_緯度 - 32bit
    uint8_t Longitude[4];       // DE_経度 - 32bit
    uint8_t Elevation[2];       // DE_高度 - 16bit
    uint8_t posConf : 4;        // DE_位置取得情報 - 4bit
    uint8_t eleConf : 4;        // DE_高度取得情報 - 4bit

    // DF_車両状態情報            9byte
    uint8_t speed[2];             // DE_車速
    uint8_t head[2];              // DE_車両方位角
    uint8_t accel[2];             // DE_前後加速度
    uint8_t speedConf  : 3;       // DE_車速取得情報
    uint8_t headConf   : 3;       // DE_車両方位角取得情報
    uint8_t accelConf  : 3;       // DE_前後加速度取得情報
    uint8_t transStat  : 3;       // DE_シフトポジション
    uint16_t steerAngle :12;       // DE_ステアリング角度

    // DF_車両属性情報            4byte
    uint8_t vRoleClass : 4;     // DE_車両用途種別
    uint8_t vSizeClass : 4;     // DE_車両サイズ種別
    uint16_t vWid       : 10;    // DE_車幅
    uint16_t vLen       : 12;    // DE_車長

    // DF_位置オプション情報
    // DF_GNSS状態オプション情報
    // DF_位置取得オプション情報
    // DF_車両状態オプション情報
    // DF_交差点情報
    // DF_拡張情報

    // DF_自由領域管理情報                     1byte
    uint8_t indivindivAppHeaderLen : 5;     // DE_自由アプリヘッダ長
    uint8_t numIndivAppData        : 3;     // DE_個別アプリデータ数

    // DF_個別アプリデータ管理情報セット          3byte * N
    struct indivAppDataMgmtInfo
    { // DF_個別アプリデータ管理情報             3Byte
      uint8_t indivServStdID;                // DE_個別サービス規格 - 8bit
      uint8_t indivAppDataAddress;           // DE_個別アプリデータ先頭アドレス - 8bit
      uint8_t indivAppDataLen;               // DE_個別アプリデータ長 - 8bit
      // 個別アプリデータ                       1byte
      uint8_t indivAppData;                  // 交差点接近0, 右折1, 右折2, 右折3
    };
    indivAppDataMgmtInfo indivInfos[10];     // N = 10
  };

  // Parameter
  double intersection_pose_x_;          // 交差点座標
  double intersection_pose_y_;
  double distance_to_intersection_;
  uint16_t vehicle_width_;              // 車両幅208cm
  uint16_t vehicle_length_;             // 車両長699cm

  // UDP Communication
  std::unique_ptr<IoContext> ctx_{std::make_unique<IoContext>(1)};
  std::unique_ptr<UdpDriver> udp_sender_{std::make_unique<UdpDriver>(*ctx_)};
  std::string ip_send_;
  uint16_t port_send_;
  uint8_t udp_send_cnt_ = 0;
  void udp_publish(uint8_t individual_data);
  void Initialize(TD001 & output);

  // Subscriber

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();

  // Callback

  // Function
  double calcEculideanDistance(double x1, double y1, double x2, double y2);
  void commonInfoSetup();
  void timeInfoSetup();

public:
  explicit VEHICOM(const rclcpp::NodeOptions & node_options);
  TD001 out_com_{};
};
} // namespace vehicom

#endif // VEHICOM__VEHICOM_HPP_