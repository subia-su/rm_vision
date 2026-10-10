// Byte-stream framing for the existing 16-byte / 26-byte protocol.
#ifndef RM_SERIAL_DRIVER__PACKET_STREAM_HPP_
#define RM_SERIAL_DRIVER__PACKET_STREAM_HPP_

#include <array>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{
static_assert(sizeof(ReceivePacket) == 16, "Feedback wire layout changed");
static_assert(sizeof(SendPacket) == 26, "Target wire layout changed");

class ReceivePacketParser
{
public:
  bool push(uint8_t byte, ReceivePacket & packet)
  {
    if (used_ == 0 && byte != 0x5A) return false;
    bytes_[used_++] = byte;
    if (used_ != bytes_.size()) return false;

    if (crc16::Verify_CRC16_Check_Sum(bytes_.data(), bytes_.size())) {
      std::memcpy(&packet, bytes_.data(), bytes_.size());
      used_ = 0;
      ++frames_;
      return true;
    }

    ++crc_errors_;
    // Retain the next possible header after a corrupt/truncated packet.
    std::size_t next = 1;
    while (next < used_ && bytes_[next] != 0x5A) ++next;
    used_ -= next;
    std::memmove(bytes_.data(), bytes_.data() + next, used_);
    return false;
  }

  void reset() { used_ = 0; }
  uint32_t frames() const { return frames_; }
  uint32_t crc_errors() const { return crc_errors_; }

private:
  std::array<uint8_t, sizeof(ReceivePacket)> bytes_{};
  std::size_t used_ = 0;
  uint32_t frames_ = 0;
  uint32_t crc_errors_ = 0;
};

template<typename Write>
void sendPacketFully(const std::vector<uint8_t> & packet, Write write)
{
  std::size_t offset = 0;
  while (offset < packet.size()) {
    const std::vector<uint8_t> remaining(packet.begin() + offset, packet.end());
    const auto written = write(remaining);
    if (written == 0 || written > remaining.size()) {
      throw std::runtime_error("Serial write made no progress or returned an invalid size");
    }
    offset += written;
  }
}
}  // namespace rm_serial_driver
#endif
