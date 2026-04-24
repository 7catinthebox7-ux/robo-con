// 2026 Mamoru Uchiuda(Meitec)

#include <chrono>
#include <memory>
#include <string>

#include "hardware/feetech_servo/feetech_servo.h"

#include "gflags/gflags.h"
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "std_srvs/srv/set_bool.hpp"

DEFINE_string(servo_port, "/dev/ttyUSB0", "Serial port for FeetechServo");
DEFINE_int32(servo_id, 1, "Target servo ID");
DEFINE_int32(servo_velocity, 1000, "Velocity command value [-2047, 2047]");

namespace meitec::ros2 {

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode(std::string port, uint8_t id, int16_t velocity);
  ~ControllerNode() = default;

 private:
  void timer_callback();
  void handle_set_enable(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // メンバー変数
  std::unique_ptr<meitec::hardware::FeetechServo> servo_driver_;
  bool velocity_enabled_{false};
  uint8_t servo_id_{1};
  int16_t target_velocity_{0};

  // rclcpp
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
};

ControllerNode::ControllerNode(std::string port, uint8_t id, int16_t velocity)
    : Node("controller_node"), servo_id_(id), target_velocity_(velocity) {
  using namespace std::literals::chrono_literals;
  // FeetechServoの初期化
  servo_driver_ = std::make_unique<meitec::hardware::FeetechServo>(port);

  // 初期設定: ホイールモード + トルクON + 速度=0
  servo_driver_->SetWheelMode(servo_id_);
  servo_driver_->SetTorque(servo_id_, true);
  servo_driver_->WriteVelocity(servo_id_, 0);

  // サービス: 速度制御の開始/停止フラグ
  enable_service_ = this->create_service<std_srvs::srv::SetBool>(
      "~/set_velocity_enable", std::bind(&ControllerNode::handle_set_enable, this,
                                         std::placeholders::_1, std::placeholders::_2));

  // 定周期で速度コマンド送信 (100ms周期)
  timer_ = this->create_wall_timer(100ms, std::bind(&ControllerNode::timer_callback, this));

  SPDLOG_INFO("ControllerNode ready. Call ~/set_velocity_enable to start/stop.");
}

void ControllerNode::handle_set_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  velocity_enabled_ = request->data;

  if (velocity_enabled_) {
    SPDLOG_INFO("Velocity control STARTED (id={}, vel={})", servo_id_, target_velocity_);
    response->success = true;
    response->message = "velocity control started";
  } else {
    // 停止時は即座に速度0を書き込む（安全のため)
    const bool ok = servo_driver_->WriteVelocity(servo_id_, 0);
    SPDLOG_INFO("Velocity control STOPPED (write_zero={})", ok);
    response->success = ok;
    response->message = ok ? "velocity control stopped" : "failed to write zero velocity";
  }
}

void ControllerNode::timer_callback() {
  // フラグがfalseの間は何もしない（停止状態)
  if (!velocity_enabled_) {
    return;
  }

  // 速度制御中は定期的に目標速度を書き込み続ける
  if (!servo_driver_->WriteVelocity(servo_id_, target_velocity_)) {
    SPDLOG_WARN("Failed to write velocity to servo id={}", servo_id_);
  }
}

}  // namespace meitec::ros2

// main関数
int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  rclcpp::init(argc, argv);

  std::shared_ptr<meitec::ros2::ControllerNode> node;
  try {
    SPDLOG_INFO("Starting ControllerNode...");
    node = std::make_shared<meitec::ros2::ControllerNode>(FLAGS_servo_port, FLAGS_servo_id,
                                                          FLAGS_servo_velocity);
  } catch (const std::exception &e) {
    SPDLOG_ERROR("Failed to start ControllerNode: {}", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);

  // 終了時に安全のため速度0を書き込む
  SPDLOG_INFO("Shutting down ControllerNode...");
  rclcpp::shutdown();
  return 0;
}
