// 2026 Mamoru Uchiuda(Meitec)

#include <chrono>
#include <string>

#include "hardware/feetech_servo/feetech_servo.h"

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

namespace meitec::ros2 {

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode();
  ~ControllerNode() = default;

 private:
  // メンバー関数
  void timer_callback();

  // メンバー変数
  std::unique_ptr<meitec::hardware::FeetechServo> servo_driver_;

  // rclcpp
  rclcpp::TimerBase::SharedPtr timer_;
};

ControllerNode::ControllerNode() : Node("controller_node") {
  using namespace std::literals::chrono_literals;
  // タイマーコールバックの初期化
  timer_ = this->create_wall_timer(1000ms, std::bind(&ControllerNode::timer_callback, this));

  // FeetechServoの初期化
  servo_driver_ = std::make_unique<meitec::hardware::FeetechServo>();
}

void ControllerNode::timer_callback() {
  //
  SPDLOG_INFO("Hello, World from ControllerNode!");
}

}  // namespace meitec::ros2

// main関数
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<meitec::ros2::ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
