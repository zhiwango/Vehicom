#include "vehicom/vehicom.hpp"

namespace vehicom
{
VEHICOM::VEHICOM(const rclcpp::NodeOptions & node_options)
: Node("vehicom", node_options)
{
  using std::placeholders::_1;

  // Parameters
  intersection_pose_x_ = static_cast<int>(declare_parameter("intersection_pose_x", 0.0));
  intersection_pose_y_ = static_cast<int>(declare_parameter("intersection_pose_y", 0.0));
  crosswalk_pose_x_ = static_cast<int>(declare_parameter("crosswalk_pose_x", 0.0));
  crosswalk_pose_y_ = static_cast<int>(declare_parameter("crosswalk_pose_y", 0.0));
  vehicle_width_ = (uint16_t)(declare_parameter<double>("vehicle_width"));
  vehicle_length_ = (uint16_t)(declare_parameter<double>("vehicle_length"));
  vehicle_restart_velocity_ = static_cast<double>(declare_parameter("vehicle_restart_velocity_", 1.3));

  // State
  initState();

  // Subscrier
  current_pose_sub_ = this->create_subscription<Odometry>(
    "~/input/current_pose", 1, std::bind(&VEHICOM::onCurrentPose, this, _1));
  crosswalk_velocity_factor_sub_ = this->create_subscription<VelocityFactorArray>(
    "~/input/crosswalk_velocity_factor", 1, std::bind(&VEHICOM::onCrosswalkVelocityFactor, this, _1));
  vehicle_velocity_report_sub_ = this->create_subscription<VelocityReport>(
    "~/input/vehicle_velocity", 1, std::bind(&VEHICOM::onVehicleVelocityReport, this, _1));
  vehicle_turn_indicators_report_sub_ = this->create_subscription<TurnIndicatorsReport>(
    "~/input/vehicle_turn_indicators", 1, std::bind(&VEHICOM::onVehicleTurnIndicatorsReport, this, _1));

  // Publisher
  try {
    ip_send_ = declare_parameter<std::string>("ip_send");
    port_send_ = static_cast<uint16_t>(declare_parameter<int>("port_send"));
    udp_sender_->init_sender(ip_send_, port_send_);
    udp_sender_->sender()->open();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating UDP sender: %s:%i - %s", ip_send_.c_str(), port_send_,
      ex.what());
  }
  // Timer
  const auto update_period_ns = rclcpp::Rate(2.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), update_period_ns, std::bind(&VEHICOM::onTimer, this));
}

void VEHICOM::udp_publish(uint8_t individual_data)
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
  this->out_com_.tTime = (uint8_t)(0x7F & tm->tm_hour);
  this->out_com_.tMin = tm->tm_min;
  uint16_t make_v = static_cast<uint16_t>((tm->tm_sec + ts.tv_nsec / 1000000000.0) * 1000.0);
  make_v = htons(make_v);
  this->out_com_.tSec[1] = uint8_t((make_v & 0xFF00) >> 8);
  this->out_com_.tSec[0] = uint8_t((make_v & 0x00FF) >> 0);

  // インクリメントカウンタ
  udp_send_cnt_ = (udp_send_cnt_ + 1) % 255;
  this->out_com_.increCount = udp_send_cnt_;

  // 個別アプリデータ
  this->out_com_.indivAppData = individual_data;

  // udp出力データ作成
  std::vector<uint8_t> send_udp_data;
  uint8_t * data = reinterpret_cast<uint8_t *>(&this->out_com_);
  for (uint32_t i = 0; i < sizeof(TD001); i++) {
    send_udp_data.push_back(data[i]);
  }

  // udp送信
  udp_sender_->sender()->send(const_cast<std::vector<uint8_t> &>(send_udp_data));

  // 初期化処理
  Initialize(this->out_com_);
}

void VEHICOM::onCurrentPose(const Odometry::ConstSharedPtr msg)
{
  distance_to_intersection_ = calcEculideanDistance(msg->pose.pose.position.x, msg->pose.pose.position.y, intersection_pose_x_, intersection_pose_y_);
  distance_to_crosswalk_ = calcEculideanDistance(msg->pose.pose.position.x, msg->pose.pose.position.y, crosswalk_pose_x_, crosswalk_pose_y_);
  // RCLCPP_INFO(this->get_logger(), "The distance to target is: %f", distance_to_intersection_);
  // RCLCPP_INFO(this->get_logger(), "The distance to target is: %f", distance_to_crosswalk_);
}

void VEHICOM::onCrosswalkVelocityFactor(const VelocityFactorArray::ConstSharedPtr msg)
{
  if(msg->factors.empty() || std::isinf(msg->factors[0].distance) ||std::isnan(msg->factors[0].distance))
  {
    is_stop_before_crosswalk_ = false;
    return;
  }
  else
  {
    is_stop_before_crosswalk_ = true;
  }
}

void VEHICOM::onVehicleVelocityReport(const VelocityReport::ConstSharedPtr msg)
{
  // Treat the vehicle as stopped
  if(msg->longitudinal_velocity < 0.1)
  {
    is_vehicle_stopped_ = true;
  }
  else
  {
    is_vehicle_stopped_ = false;
  }

  // Treat the vehicle as moving
  if(msg->longitudinal_velocity > vehicle_restart_velocity_)
  {
    is_vehicle_moving_ = true;
  }
  else
  {
    is_vehicle_moving_ = false;
  }
}

void VEHICOM::onVehicleTurnIndicatorsReport(const TurnIndicatorsReport::ConstSharedPtr msg)
{
  if(msg->report == 3)
  {
    is_vehicle_trun_right_ = true;
  }
  else
  {
    is_vehicle_trun_right_ = false;
  }
}

void VEHICOM::onTimer()
{
  stateJudgement();
}

void VEHICOM::Initialize(TD001 & output)
{
  // DF_共通領域管理情報                                 8byte
  output.comServStdID = 0x01;                         // DE_共通サービス規格ID - 3bit
  output.msgID = 0x01;                                // DE_メッセージID - 2bit
  output.Ver = 0x01;                                  // DE_バージョン情報 - 3bit
  output.vID[0] = uint8_t((0x0 & 0xFF000000) >> 24);  // DE_車両ID - 32bit
  output.vID[1] = uint8_t((0x0 & 0x00FF0000) >> 16);
  output.vID[2] = uint8_t((0x0 & 0x0000FF00) >> 8);
  output.vID[3] = uint8_t((0x0 & 0x000000FF) >> 0);
  output.increCount = 0x00;                           // DE_インクリメントカウンタ - 8bit
  output.comAppDataLen = 0x1c;                        // DE_共通アプリデータ長 - 8 bit
  output.optFlg = 0x00;                               // DE_オプションフラグ - 8bit

  // DF_時刻情報 - 4byte
  output.tTime = 0x00;                                // DE_うるう秒補正情報 - 1bit
  output.tTime |= 0x7F;                               // DE_時刻（時） - 7bit
  output.tMin = 0xFF;                                 // DE_時刻（分）- 8bit
  output.tSec[0] = 0xFF;                              // DE_時刻（秒）- 16bit
  output.tSec[1] = 0xFF;

  // DF_位置情報                  11byte
  output.Latitude[0] = 0x11;    // DE_緯度 - 32bit
  output.Latitude[1] = 0x11;
  output.Latitude[2] = 0x11;
  output.Latitude[3] = 0x11;
  output.Longitude[0] = 0x22;   // DE_経度 - 32bit
  output.Longitude[1] = 0x22;
  output.Longitude[2] = 0x22;
  output.Longitude[3] = 0x22;
  output.Elevation[0] = 0x33;   // DE_高度 - 16bit
  output.Elevation[1] = 0x33;
  output.posConf = 0x00;        // DE_位置取得情報 - 4bit
  output.eleConf = 0x00;        // DE_高度取得情報 - 4bit

  // DF_車両状態情報              9byte
  output.speed[0] = 0xFF;       // DE_車速 - 16bit
  output.speed[1] = 0xFF;
  output.head[0] = 0xFF;        // DE_車両方位角 - 16bit
  output.head[1] = 0xFF;
  output.accel[0] = 0xFF;       // DE_前後加速度 - 16bit
  output.accel[1] = 0xFF;
  output.vehicleConf[0] = (0x00 << 5);   // DE_車速取得情報       : 3
  output.vehicleConf[0] |= (0x00 << 2);  // DE_車両方位角取得情報  : 3
  output.vehicleConf[0] |= (0x00 >> 1);  // DE_前後加速度取得情報  : 3
  output.vehicleConf[1] = 0x00;
  output.vehicleConf[1] |= (0x00 << 4);  // DE_シフトポジション    : 3
  output.vehicleConf[1] |= (0x00 << 0);  // DE_ステアリング角度    : 12
  output.vehicleConf[2] = 0x00;

  // DF_車両属性情報                      4byte
  output.vSizeClass = 0x0E;             // DE_車両サイズ種別 - 4bit
  output.vRoleClass = 0x0F;             // DE_車両用途種別 - 4bit
  // 0x34, 0x02, 0xbb
  output.vehicleSize[0] = uint8_t((vehicle_width_ & 0x03FC) >> 2);    // DE_車幅 - 10bit
  output.vehicleSize[1] = uint8_t((vehicle_width_ & 0x0003) << 6);
  output.vehicleSize[1] |= uint8_t((vehicle_length_ & 0x3F00) >> 8);  // DE_車長 - 14bit
  output.vehicleSize[2] = uint8_t(vehicle_length_ & 0x00FF);

   // DF_自由領域管理情報                      1byte
  output.indivindivAppHeaderLen = 0x04;     // DE_自由アプリヘッダ長 - 5bit
  output.numIndivAppData = 0x01;            // DE_個別アプリデータ数 - 3bit

  // DF_個別アプリデータ管理情報セット           3byte
  output.indivServStdID = 0x01;             // DE_個別サービス規格 - 8bit
  output.indivAppDataAddress = 0x00;        // DE_個別アプリデータ先頭アドレス - 8bit
  output.indivAppDataLen = 0x01;            // DE_個別アプリデータ長 - 8bit

  // 個別アプリデータ                          1byte
  // output.indivAppData = 0xFF;            // 交差点接近0, 右折1, 右折2, 右折3
}

void VEHICOM::initState()
{
  is_stop_before_crosswalk_ = false;
  is_vehicle_moving_ = false;
  is_vehicle_stopped_ = false;
  is_vehicle_trun_right_ = false;
}

double VEHICOM::calcEculideanDistance(double x1, double y1, double x2, double y2)
{
  double dist;
  dist = pow(x1 - x2, 2) + pow(y1 - y2, 2);
  return sqrt(dist);
}

void VEHICOM::stateJudgement()
{
  // Static Parameters
  static int runPhaseZeroTimes = 0;
  static int runPhaseOneTimes = 0;
  static int runPhaseThreeTimes = 0;
  static bool hasEnterIntersection = false;
  static bool hasStopCrosswalk = false;
  static bool hasOverCrosswalk = false;

  if(distance_to_intersection_ < 20.0 && is_vehicle_trun_right_) // 送信処理開始
  {
    // Phase 0
    if(distance_to_intersection_ < 10.0 && runPhaseZeroTimes == 0)
    {
      udp_publish(0x00);
      RCLCPP_INFO(this->get_logger(), "右折交差点に接近する。");
      runPhaseZeroTimes++;
    }

    if(distance_to_intersection_ < 2.0)
    {
      hasEnterIntersection = true;
    }

    // Phase 1
    if(hasEnterIntersection && runPhaseZeroTimes != 0 &&!hasStopCrosswalk && !hasOverCrosswalk) // 信号停止線を5m超えたら、処理開始
    {
      // 0x01を送信する前に,0x02を送信することを判断できた際に、0x01を一秒間送信継続
      if (is_stop_before_crosswalk_ && runPhaseOneTimes < 2 && !is_vehicle_stopped_)
      {
        udp_publish(0x01);
        RCLCPP_INFO(this->get_logger(), "右折交差点に侵入する。");
        runPhaseOneTimes++;
      }
      else if(is_stop_before_crosswalk_ && is_vehicle_stopped_) //Interstionにより停車した場合はデータ送信をし続ける
      {
        udp_publish(0x01);
        RCLCPP_INFO(this->get_logger(), "右折交差点に侵入する。");
        runPhaseOneTimes = 0;
      }
      // 横断歩道の前に停止判定しない場合
      else if (!is_stop_before_crosswalk_)
      {
        udp_publish(0x01);
        RCLCPP_INFO(this->get_logger(), "右折交差点に侵入する。");
        runPhaseOneTimes = 0;
      }

      if (distance_to_intersection_ > 21.0 && distance_to_crosswalk_ < 3.0)
      {
        hasOverCrosswalk = true;
      }
    }

    // Phase 2
    if(is_stop_before_crosswalk_ && is_vehicle_stopped_ && distance_to_crosswalk_ < 12.0) //歩行者がいる場合
    {
      udp_publish(0x02);
      hasStopCrosswalk = true;
      RCLCPP_INFO(this->get_logger(), "横断歩道の前で一時停止した。");
    }

    // Phase 3
    if(is_vehicle_moving_ && runPhaseThreeTimes < 1 && hasStopCrosswalk)
    {
      udp_publish(0x03);
      RCLCPP_INFO(this->get_logger(), "横断歩道から再発進する。");
      runPhaseThreeTimes ++;
    }
  }
  else // Reset static parameter
  {
    runPhaseZeroTimes = 0;
    runPhaseOneTimes = 0;
    runPhaseThreeTimes = 0;
    hasEnterIntersection = false;
    hasStopCrosswalk = false;
    hasOverCrosswalk = false;
  }
}

} // namespace vehicom

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vehicom::VEHICOM)