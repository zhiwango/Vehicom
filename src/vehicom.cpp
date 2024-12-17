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
  vehicle_width_ = (uint16_t)(declare_parameter<double>("vehicle_width"));
  vehicle_length_ = (uint16_t)(declare_parameter<double>("vehicle_length"));

  // State

  // Subscrier

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
  this->out_com_.tHour = (uint8_t)(0x7F & tm->tm_hour);
  this->out_com_.tMin = tm->tm_min;
  uint16_t make_v = static_cast<uint16_t>((tm->tm_sec + ts.tv_nsec / 1000000000.0) * 1000.0);
  make_v = htons(make_v);
  this->out_com_.tSec[1] = uint8_t((make_v & 0xFF00) >> 8);
  this->out_com_.tSec[0] = uint8_t((make_v & 0x00FF) >> 0);

  // インクリメントカウンタ
  udp_send_cnt_ = (udp_send_cnt_ + 1) % 255;
  this->out_com_.increCount = udp_send_cnt_;

  // 個別アプリデータ
  this->out_com_.indivInfos[0].indivAppData = individual_data;

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

void VEHICOM::onTimer()
{
  main();
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
  output.tLeap = 0x00;                                // DE_うるう秒補正情報 - 1bit
  output.tHour = 0x00;                               // DE_時刻（時） - 7bit
  output.tMin = 0x00;                                 // DE_時刻（分）- 8bit
  output.tSec[0] = 0x00;                              // DE_時刻（秒）- 16bit
  output.tSec[1] = 0x00;

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
  output.speedConf = 0x07;      // DE_車速取得情報       : 3
  output.headConf = 0x07;       // DE_車両方位角取得情報  : 3
  output.accelConf = 0x07;      // DE_前後加速度取得情報  : 3
  output.transStat = 0x07;      // DE_シフトポジション    : 3
  output.steerAngle = 0xFF;     // DE_ステアリング角度    : 12

  // DF_車両属性情報                      4byte
  output.vSizeClass = 0x0E;             // DE_車両サイズ種別 - 4bit
  output.vRoleClass = 0x0F;             // DE_車両用途種別 - 4bit
  output.vWid = vehicle_width_;         // DE_車幅 - 10bit
  output.vLen = vehicle_length_;        // DE_車幅 - 10bit

   // DF_自由領域管理情報                      1byte
  output.indivindivAppHeaderLen = 0x00;     // DE_自由アプリヘッダ長 - 5bit
  output.numIndivAppData = 0x00;            // DE_個別アプリデータ数 - 3bit

  // DF_個別アプリデータ管理情報セット           3byte
  output.indivInfos[0].indivServStdID = 0x00;             // DE_個別サービス規格 - 8bit
  output.indivInfos[0].indivAppDataAddress = 0x00;        // DE_個別アプリデータ先頭アドレス - 8bit
  output.indivInfos[0].indivAppDataLen = 0x00;            // DE_個別アプリデータ長 - 8bit

  // 個別アプリデータ                          1byte
  // output.indivAppData = 0xFF;            // 交差点接近0, 右折1, 右折2, 右折3
}


double VEHICOM::calcEculideanDistance(double x1, double y1, double x2, double y2)
{
  double dist;
  dist = pow(x1 - x2, 2) + pow(y1 - y2, 2);
  return sqrt(dist);
}

void VEHICOM::main()
{
  udp_publish(0x00);
}

} // namespace vehicom

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vehicom::VEHICOM)