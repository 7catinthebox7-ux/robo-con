// 2026 Mamoru Uchiuda(Meitec)

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode();
  ~ControllerNode() = default;

 private:
  // メンバー関数
  void timer_callback();

  // メンバー変数

  // rclcpp
  rclcpp::TimerBase::SharedPtr timer_;
};

ControllerNode::ControllerNode() : Node("controller_node") {
  using namespace std::literals::chrono_literals;
  // タイマーコールバックの初期化
  timer_ = this->create_wall_timer(1000ms, std::bind(&ControllerNode::timer_callback, this));
}

void ControllerNode::timer_callback() { 
  // 
  SPDLOG_INFO("Hello, World from ControllerNode!"); 
}

// main関数
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
