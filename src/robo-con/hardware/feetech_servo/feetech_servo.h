// 2026 Mamoru Uchiuda(Meitec)
#pragma once

#include <termios.h>
#include <cstdint>
#include <string>
#include <vector>

#include "spdlog/spdlog.h"

namespace meitec::hardware {

struct ServoTarget {
  uint8_t id;
  uint16_t position;
  uint16_t velocity;
};

class FeetechServo {
 public:
  explicit FeetechServo(std::string port);
  ~FeetechServo();

  // ホイール(速度)モード切替
  bool SetWheelMode(uint8_t id);
  bool SetPositionMode(uint8_t id);

  // 速度指令 [-2047, 2047] (符号で方向)
  bool WriteVelocity(uint8_t id, int16_t velocity);

  // トルクON/OFF
  bool SetTorque(uint8_t id, bool enable);

 private:
  // パケット生成・送受信
  bool WriteRegisters(uint8_t id, uint8_t address, const std::vector<uint8_t> &data);
  bool ReadResponse(std::vector<uint8_t> &response, size_t expected_size);

  // メンバー変数
  std::string port_;
  int32_t fd_;

  // 定数 (STS/SCSレジスタアドレス)
  static constexpr uint32_t kDefaultBaudRate = B1000000;
  static constexpr uint8_t kHeader1 = 0xFF;
  static constexpr uint8_t kHeader2 = 0xFF;
  static constexpr uint8_t kInstWrite = 0x03;

  // レジスタアドレス (STSシリーズ)
  static constexpr uint8_t kAddrOperatingMode = 33;  // 0:位置, 1:ホイール
  static constexpr uint8_t kAddrTorqueEnable = 40;
  static constexpr uint8_t kAddrGoalVelocity = 46;  // ホイールモード時の速度
  static constexpr uint8_t kAddrGoalPosition = 42;
};

}  // namespace meitec::hardware
