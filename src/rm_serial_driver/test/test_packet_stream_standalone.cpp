// Copyright (c) 2026 xkddyl
// Licensed under the Apache-2.0 License.

#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

#include "rm_serial_driver/packet_stream.hpp"

using rm_serial_driver::fromVector;
using rm_serial_driver::ReceivePacket;
using rm_serial_driver::ReceivePacketParser;
using rm_serial_driver::sendPacketFully;
using rm_serial_driver::SendPacket;
using rm_serial_driver::to_fixed16;
using rm_serial_driver::toVector;

static std::vector<uint8_t> feedback()
{
  ReceivePacket p{};
  p.roll = 0x5A;  // A header byte inside a valid payload must not resynchronize it.
  p.pitch = -20; p.yaw = 100;
  std::vector<uint8_t> bytes(sizeof(p));
  std::memcpy(bytes.data(), &p, sizeof(p));
  crc16::Append_CRC16_Check_Sum(bytes.data(), bytes.size());
  return bytes;
}

int main()
{
  std::vector<uint8_t> crc_reference = {'1', '2', '3', '4', '5', '6', '7', '8', '9', 0, 0};
  crc16::Append_CRC16_Check_Sum(crc_reference.data(), crc_reference.size());
  assert(crc_reference[9] == 0x91 && crc_reference[10] == 0x6F);
  const std::vector<uint8_t> golden_feedback = {
    0x5A, 0x03, 0x0C, 0x00, 0xEC, 0xFF, 0x64, 0x00,
    0x7B, 0x00, 0x38, 0xFE, 0x15, 0x03, 0x07, 0x2B};
  const auto golden = fromVector(golden_feedback);
  assert(golden.detect_color == 1 && golden.reset_tracker && golden.reserved == 0);
  assert(golden.roll == 12 && golden.pitch == -20 && golden.yaw == 100);
  assert(golden.aim_x == 123 && golden.aim_y == -456 && golden.aim_z == 789);
  assert(to_fixed16(1.239f) == 123 && to_fixed16(-1.239f) == -123);
  assert(to_fixed16(-327.0f) == -32700 && to_fixed16(327.67f) == 32767);
  for (float invalid : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(), 327.68f, -327.69f})
  {
    bool rejected = false;
    try { (void)to_fixed16(invalid); }
    catch (const std::logic_error &) { rejected = true; }
    assert(rejected);
  }
  for (std::size_t length = 0; length <= 17; ++length) {
    if (length == golden_feedback.size()) continue;
    bool rejected = false;
    try { (void)fromVector(std::vector<uint8_t>(length, 0x5A)); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  auto corrupt = golden_feedback;
  corrupt.back() ^= 1;
  bool crc_rejected = false;
  try { (void)fromVector(corrupt); }
  catch (const std::invalid_argument &) { crc_rejected = true; }
  assert(crc_rejected);
  const auto frame = feedback();
  for (std::size_t split = 1; split < frame.size(); ++split) {
    ReceivePacketParser parser;
    ReceivePacket out{};
    unsigned decoded = 0;
    for (std::size_t i = 0; i < split; ++i) decoded += parser.push(frame[i], out);
    assert(decoded == 0);
    for (std::size_t i = split; i < frame.size(); ++i) decoded += parser.push(frame[i], out);
    assert(decoded == 1 && out.roll == 0x5A && out.pitch == -20 && out.yaw == 100);
  }
  for (std::size_t truncated = 1; truncated < frame.size(); ++truncated) {
    ReceivePacketParser parser;
    ReceivePacket out{};
    for (std::size_t i = 0; i < truncated; ++i) parser.push(frame[i], out);
    for (auto byte : frame) parser.push(byte, out);
    assert(parser.frames() == 1 && out.pitch == -20);
  }
  ReceivePacketParser parser;
  ReceivePacket out{};
  auto bad = frame; bad.back() ^= 1;
  for (auto byte : bad) parser.push(byte, out);
  for (unsigned n = 0; n < 128; ++n)
    for (auto byte : frame) parser.push(byte, out);
  assert(parser.frames() == 128 && parser.crc_errors() >= 1);
  parser.push(0x5A, out); parser.reset();
  for (auto byte : frame) parser.push(byte, out);
  assert(parser.frames() == 129);

  SendPacket target{};
  target.tracking = true; target.id = 4; target.armors_num = 4; target.reserved = 1;
  target.x = 500; target.y = -100; target.z = 50; target.yaw = 157;
  target.vx = 25; target.vy = -50; target.vz = 0; target.v_yaw = 120;
  target.r1 = 26; target.r2 = 30; target.dz = 5;
  const auto data = toVector(target);
  const std::vector<uint8_t> golden_target = {
    0xA5, 0x49, 0xF4, 0x01, 0x9C, 0xFF, 0x32, 0x00, 0x9D, 0x00,
    0x19, 0x00, 0xCE, 0xFF, 0x00, 0x00, 0x78, 0x00, 0x1A, 0x00,
    0x1E, 0x00, 0x05, 0x00, 0x14, 0xC5};
  assert(data == golden_target);
  for (unsigned id = 0; id < 8; ++id) {
    for (unsigned count : {2u, 3u, 4u}) {
      for (bool tracking : {false, true}) {
        target.id = id; target.armors_num = count; target.tracking = tracking;
        const auto encoded = toVector(target);
        assert(encoded[1] == (unsigned(tracking) | (id << 1) | (count << 4)));
        assert(crc16::Verify_CRC16_Check_Sum(encoded.data(), encoded.size()));
      }
    }
  }
  for (std::size_t max_write = 1; max_write <= data.size(); ++max_write) {
    std::vector<uint8_t> sent;
    sendPacketFully(data, [&](const std::vector<uint8_t> & remaining) {
      const auto count = std::min(max_write, remaining.size());
      sent.insert(sent.end(), remaining.begin(), remaining.begin() + count);
      return count;
    });
    assert(sent == data);
  }
  bool failed = false;
  try { sendPacketFully(data, [](const std::vector<uint8_t> &) { return std::size_t{0}; }); }
  catch (const std::runtime_error &) { failed = true; }
  assert(failed);
  failed = false;
  try {
    sendPacketFully(data, [](const std::vector<uint8_t> & remaining) {
      return remaining.size() + 1;
    });
  } catch (const std::runtime_error &) { failed = true; }
  assert(failed);
  std::cout << "PASS serial: split reads, CRC recovery, bursts, short writes, "
            << "golden wire frames, flags, numeric guards\n";
}
