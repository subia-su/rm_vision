// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "rm_serial_driver/crc.hpp"

namespace rm_serial_driver
{
// 16 位定点协议缩放因子：所有 float 字段统一 ×100 转 int16_t
// 范围：-327.68 ~ 327.67，精度 0.01
constexpr float SCALE = 100.0f;

inline int16_t to_fixed16(float v)
{
  const float scaled = v * SCALE;
  if (!std::isfinite(scaled)) {
    throw std::domain_error("Non-finite target value");
  }
  const float integral = std::trunc(scaled);
  if (integral < -32768.0f || integral > 32767.0f) {
    throw std::out_of_range("Target value exceeds int16 x100 range");
  }
  // Preserve the existing truncation rule for values inside the wire range.
  return static_cast<int16_t>(integral);
}

inline float from_fixed16(int16_t v)
{
  return static_cast<float>(v) / SCALE;
}

struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  int16_t roll;    // 实际值 = roll / 100
  int16_t pitch;   // 实际值 = pitch / 100
  int16_t yaw;     // 实际值 = yaw / 100
  int16_t aim_x;   // 实际值 = aim_x / 100
  int16_t aim_y;   // 实际值 = aim_y / 100
  int16_t aim_z;   // 实际值 = aim_z / 100
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  bool tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  int16_t x;      // 实际值 = x / 100
  int16_t y;      // 实际值 = y / 100
  int16_t z;      // 实际值 = z / 100
  int16_t yaw;    // 实际值 = yaw / 100
  int16_t vx;     // 实际值 = vx / 100
  int16_t vy;     // 实际值 = vy / 100
  int16_t vz;     // 实际值 = vz / 100
  int16_t v_yaw;  // 实际值 = v_yaw / 100
  int16_t r1;     // 实际值 = r1 / 100
  int16_t r2;     // 实际值 = r2 / 100
  int16_t dz;     // 实际值 = dz / 100
  uint16_t checksum = 0;
} __attribute__((packed));

constexpr std::size_t RECEIVE_PACKET_SIZE = 16;
constexpr std::size_t SEND_PACKET_SIZE = 26;

inline int16_t readInt16Le(const uint8_t * data)
{
  const uint16_t bits = static_cast<uint16_t>(data[0]) |
    (static_cast<uint16_t>(data[1]) << 8);
  return static_cast<int16_t>(bits < 0x8000u ? bits : static_cast<int32_t>(bits) - 0x10000);
}

inline void writeInt16Le(uint8_t * data, int16_t value)
{
  const uint16_t bits = static_cast<uint16_t>(value);
  data[0] = static_cast<uint8_t>(bits);
  data[1] = static_cast<uint8_t>(bits >> 8);
}

// Called only after a complete frame has passed CRC validation.
inline ReceivePacket decodeReceivePacket(const uint8_t * data)
{
  ReceivePacket packet{};
  packet.header = data[0];
  packet.detect_color = data[1] & 0x01u;
  packet.reset_tracker = (data[1] & 0x02u) != 0;
  packet.reserved = data[1] >> 2;
  packet.roll = readInt16Le(data + 2);
  packet.pitch = readInt16Le(data + 4);
  packet.yaw = readInt16Le(data + 6);
  packet.aim_x = readInt16Le(data + 8);
  packet.aim_y = readInt16Le(data + 10);
  packet.aim_z = readInt16Le(data + 12);
  packet.checksum = static_cast<uint16_t>(data[14]) |
    (static_cast<uint16_t>(data[15]) << 8);
  return packet;
}

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  if (data.size() != RECEIVE_PACKET_SIZE || data[0] != 0x5A ||
    !crc16::Verify_CRC16_Check_Sum(data.data(), data.size()))
  {
    throw std::invalid_argument("Invalid feedback frame");
  }
  return decodeReceivePacket(data.data());
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  // The wire layout must not depend on C++ bitfield allocation or host endian.
  std::vector<uint8_t> packet(SEND_PACKET_SIZE, 0);
  packet[0] = 0xA5;
  packet[1] = (data.tracking ? 0x01u : 0u) |
    ((data.id & 0x07u) << 1) | ((data.armors_num & 0x07u) << 4);
  const int16_t values[] = {
    data.x, data.y, data.z, data.yaw, data.vx, data.vy, data.vz,
    data.v_yaw, data.r1, data.r2, data.dz};
  for (std::size_t i = 0; i < 11; ++i) {
    writeInt16Le(packet.data() + 2 + 2 * i, values[i]);
  }
  crc16::Append_CRC16_Check_Sum(packet.data(), packet.size());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
