#include <algorithm>
#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>

#include "rm_serial_driver/packet_stream.hpp"

using namespace rm_serial_driver;

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
  target.tracking = true; target.armors_num = 4; target.x = 500; target.y = -100;
  crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&target), sizeof(target));
  const auto data = toVector(target);
  assert(data.size() == 26 && data[0] == 0xA5 && data[1] == 0x41);
  assert(data[2] == 0xF4 && data[3] == 0x01 && data[4] == 0x9C && data[5] == 0xFF);
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
  std::cout << "PASS serial: split reads, truncated/bad CRC recovery, bursts, short writes, wire layout\n";
}
