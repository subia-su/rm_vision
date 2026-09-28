// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
// 16 位定点协议缩放因子：所有 float 字段统一 ×100 转 int16_t
// 范围：-327.68 ~ 327.67，精度 0.01
constexpr float SCALE = 100.0f;

inline int16_t to_fixed16(float v)
{
  return static_cast<int16_t>(v * SCALE);
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

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
