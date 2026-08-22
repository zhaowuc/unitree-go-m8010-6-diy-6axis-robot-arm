#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "unitreeMotor/unitreeMotor.h"

namespace {

template <typename T>
void zero_object(T& value) {
  static_assert(std::is_trivially_copyable<T>::value,
                "SDK wrapper must be trivially copyable");
  std::memset(static_cast<void*>(&value), 0, sizeof(value));
}

std::uint16_t load_u16_le(const std::uint8_t* bytes) {
  return static_cast<std::uint16_t>(bytes[0]) |
         static_cast<std::uint16_t>(bytes[1] << 8U);
}

std::uint16_t crc16_kermit(const std::uint8_t* bytes, std::size_t size) {
  std::uint16_t crc = 0;
  while (size-- != 0U) {
    crc = static_cast<std::uint16_t>(crc ^ *bytes++);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U
                ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                : static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

struct Result {
  double literal;
  std::int16_t count;
  double decoded;
};

Result serialize_tau(double literal) {
  MotorCmd command;
  zero_object(command);
  command.motorType = MotorType::GO_M8010_6;
  command.id = 0;
  command.mode = static_cast<unsigned short>(
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC));
  command.tau = static_cast<float>(literal);
  command.dq = 0.0F;
  command.q = 0.0F;
  command.kp = 0.0F;
  command.kd = 0.0F;
  command.Res.u32 = 0;
  command.modify_data(&command);

  if (command.hex_len != 17) throw std::runtime_error("PACKET_LENGTH");
  const std::uint8_t* packet = command.get_motor_send_data();
  if (packet == nullptr || packet[0] != 0xfeU || packet[1] != 0xeeU ||
      packet[2] != 0x10U || (packet[2] & 0x80U) != 0U) {
    throw std::runtime_error("PACKET_STRUCTURE");
  }
  if (load_u16_le(packet + 15) != crc16_kermit(packet, 15)) {
    throw std::runtime_error("PACKET_CRC");
  }
  const auto count = static_cast<std::int16_t>(load_u16_le(packet + 3));
  return {literal, count, static_cast<double>(count) / 256.0};
}

}  // namespace

int main() {
  try {
    static_assert(sizeof(ControlData_t) == 17, "GO packet ABI mismatch");
    static_assert(sizeof(std::int16_t) == 2, "int16 ABI mismatch");
    if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC) != 1) {
      throw std::runtime_error("FOC_MODE_AUTHORITY");
    }

    constexpr std::array<double, 15> kCases = {
        -129.0, -128.0, -127.99609375, -1.0, -0.25, -0.05,
        -0.00390625, 0.0, 0.00390625, 0.05, 0.25, 1.0,
        127.99609375, 128.0, 129.0};
    std::cout << "literal_tau_nm,raw_int16_count,decoded_tau_nm\n";
    std::array<Result, kCases.size()> results{};
    for (std::size_t i = 0; i < kCases.size(); ++i) {
      results[i] = serialize_tau(kCases[i]);
      std::cout << std::setprecision(17) << results[i].literal << ','
                << results[i].count << ',' << results[i].decoded << '\n';
    }

    const Result zero = serialize_tau(0.0);
    const Result bounded_positive = serialize_tau(0.05);
    const Result bounded_negative = serialize_tau(-0.05);
    const Result positive = serialize_tau(0.25);
    const Result negative = serialize_tau(-0.25);
    if (zero.count != 0 || bounded_positive.count != 12 ||
        bounded_negative.count != -12 || positive.count != 64 ||
        negative.count != -64 ||
        bounded_positive.count != -bounded_negative.count ||
        positive.count != -negative.count) {
      throw std::runtime_error("ZERO_OR_SIGN_SYMMETRY");
    }
    std::cout << "SELF_TEST=PASS\n"
              << "SERIAL_PORT_CONSTRUCTED=NO\n"
              << "DEVICE_IO=NO\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "SELF_TEST=FAIL\nREASON=" << error.what() << '\n';
    return 2;
  }
}
