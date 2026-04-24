// 2026 Mamoru Uchiuda(Meitec)

#include "feetech_servo.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <numeric>
#include <stdexcept>

namespace meitec::hardware {

FeetechServo::FeetechServo(std::string port) : port_(std::move(port)), fd_(-1) {
  if (port_.empty()) {
    SPDLOG_ERROR("Serial port cannot be empty");
    throw std::invalid_argument("Serial port cannot be empty");
  }

  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    SPDLOG_ERROR("Failed to open serial port: {}", port_);
    throw std::runtime_error("Failed to open serial port");
  }

  // termios設定: 1Mbps, 8N1, RAW
  struct termios tty {};
  if (tcgetattr(fd_, &tty) != 0) {
    SPDLOG_ERROR("tcgetattr failed");
    throw std::runtime_error("tcgetattr failed");
  }
  cfsetispeed(&tty, kDefaultBaudRate);
  cfsetospeed(&tty, kDefaultBaudRate);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;  // 100ms タイムアウト
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    SPDLOG_ERROR("tcsetattr failed");
    throw std::runtime_error("tcsetattr failed");
  }

  SPDLOG_INFO("FeetechServo initialized with port: {}", port_);
}

FeetechServo::~FeetechServo() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

// WRITEパケット送信: [FF FF ID LEN INST ADDR DATA... CHKSUM]
bool FeetechServo::WriteRegisters(uint8_t id, uint8_t address, const std::vector<uint8_t> &data) {
  std::vector<uint8_t> packet;
  packet.reserve(data.size() + 7);

  const uint8_t length = static_cast<uint8_t>(data.size() + 3);  // INST + ADDR + DATA + CHKSUM
  packet.push_back(kHeader1);
  packet.push_back(kHeader2);
  packet.push_back(id);
  packet.push_back(length);
  packet.push_back(kInstWrite);
  packet.push_back(address);
  packet.insert(packet.end(), data.begin(), data.end());

  // チェックサム: ~(ID + LEN + INST + ADDR + DATA...)
  uint32_t sum = id + length + kInstWrite + address;
  sum = std::accumulate(data.begin(), data.end(), sum);
  packet.push_back(static_cast<uint8_t>(~sum & 0xFF));

  // バッファクリアして送信
  tcflush(fd_, TCIOFLUSH);
  ssize_t written = write(fd_, packet.data(), packet.size());
  if (written != static_cast<ssize_t>(packet.size())) {
    SPDLOG_ERROR("Serial write failed: id={}, addr={}", id, address);
    return false;
  }
  // 応答は省略可 (ブロードキャスト動作確認用途ならReadResponse参照)
  return true;
}

bool FeetechServo::SetWheelMode(uint8_t id) {
  // ホイールモード = 1
  return WriteRegisters(id, kAddrOperatingMode, {0x01});
}

bool FeetechServo::SetPositionMode(uint8_t id) {
  return WriteRegisters(id, kAddrOperatingMode, {0x00});
}

bool FeetechServo::SetTorque(uint8_t id, bool enable) {
  return WriteRegisters(id, kAddrTorqueEnable, {static_cast<uint8_t>(enable ? 1 : 0)});
}

bool FeetechServo::WriteVelocity(uint8_t id, int16_t velocity) {
  // Feetechは最上位ビットで方向を表現 (bit15=1で逆転)
  uint16_t raw = 0;
  if (velocity < 0) {
    raw = static_cast<uint16_t>(-velocity) & 0x7FFF;
    raw |= 0x8000;
  } else {
    raw = static_cast<uint16_t>(velocity) & 0x7FFF;
  }
  const uint8_t low = raw & 0xFF;
  const uint8_t high = (raw >> 8) & 0xFF;
  return WriteRegisters(id, kAddrGoalVelocity, {low, high});
}

}  // namespace meitec::hardware
