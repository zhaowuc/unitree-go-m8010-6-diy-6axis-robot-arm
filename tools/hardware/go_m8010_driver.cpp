#include "go_m8010_driver.hpp"

#include <algorithm>
#include <array>
#include <climits>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace go_m8010 {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kBaud = 4000000.0;
constexpr double kFinalMaxJointVelocityRadS = 12.0 * kPi / 180.0;
constexpr std::size_t kFrozenSdkPacketSize = 17;
constexpr std::size_t kFrozenSdkPayloadSize = 15;
constexpr std::uintptr_t kFrozenGoPacketOffsetInMotorCmd = 36;

using FrozenSdkPacket = std::array<std::uint8_t, kFrozenSdkPacketSize>;

struct FrozenPacketExpectation {
  std::uint8_t id = 0;
  std::uint8_t mode = 0;
  std::uint16_t tau_bits = 0;
  std::uint16_t dq_bits = 0;
  std::uint32_t q_bits = 0;
  std::uint16_t kp_count = 0;
  std::uint16_t kd_count = 0;
};

static_assert(CHAR_BIT == 8, "Frozen SDK requires 8-bit bytes");
static_assert(sizeof(float) == 4, "Frozen SDK requires 32-bit float");
static_assert(sizeof(unsigned short) == 2,
              "Frozen SDK requires 16-bit unsigned short");
static_assert(sizeof(MotorType) == 4, "Frozen SDK enum ABI mismatch");
static_assert(sizeof(ControlData_t) == 17, "Frozen GO packet ABI mismatch");
static_assert(alignof(ControlData_t) == 1, "Frozen GO packet packing mismatch");
static_assert(sizeof(MotorCmd) == 88, "Frozen MotorCmd ABI mismatch");
static_assert(alignof(MotorCmd) == 4, "Frozen MotorCmd alignment mismatch");
static_assert(std::is_trivially_copyable<MotorCmd>::value,
              "Frozen MotorCmd must remain trivially copyable");
static_assert(std::is_trivially_destructible<MotorCmd>::value,
              "Frozen MotorCmd must remain trivially destructible");

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Frozen Unitree SDK requires a little-endian target"
#endif

template <typename T>
void zero_object(T& value) {
  static_assert(std::is_trivially_copyable<T>::value,
                "SDK packet wrapper must remain trivially copyable");
  std::memset(static_cast<void*>(&value), 0, sizeof(value));
}

std::uint16_t load_u16_le(const std::uint8_t* bytes) {
  return static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(bytes[0]) |
      (static_cast<std::uint16_t>(bytes[1]) << 8U));
}

std::uint32_t load_u32_le(const std::uint8_t* bytes) {
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

void store_u16_le(std::uint8_t* bytes, std::uint16_t value) {
  bytes[0] = static_cast<std::uint8_t>(value & 0xffU);
  bytes[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

// Independent bit-at-a-time CRC-16/KERMIT implementation.  It intentionally
// does not call the SDK CRC routine or use the SDK table, so the self-test is
// not circular.
std::uint16_t independent_crc16_kermit(const std::uint8_t* bytes,
                                       std::size_t size) {
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

void refresh_packet_crc(FrozenSdkPacket& packet) {
  store_u16_le(packet.data() + kFrozenSdkPayloadSize,
               independent_crc16_kermit(packet.data(),
                                         kFrozenSdkPayloadSize));
}

void validate_packet_structure(const FrozenSdkPacket& packet, int id,
                               int mode) {
  if (id < 0 || id > 15 || mode < 0 || mode > 7) {
    throw std::runtime_error("SDK_PACKET_EXPECTATION_OUT_OF_RANGE");
  }
  if (packet[0] != 0xfeU || packet[1] != 0xeeU) {
    throw std::runtime_error("SDK_PACKET_HEADER_MISMATCH");
  }
  // The frozen SDK preserves this bit from its private packet cache.  CRC can
  // still be valid when the bit is one, so it is a separate hard invariant.
  if ((packet[2] & 0x80U) != 0U) {
    throw std::runtime_error("SDK_PACKET_RESERVED_BIT_NONZERO");
  }
  const std::uint16_t computed_crc =
      independent_crc16_kermit(packet.data(), kFrozenSdkPayloadSize);
  const std::uint16_t stored_crc =
      load_u16_le(packet.data() + kFrozenSdkPayloadSize);
  if (stored_crc != computed_crc) {
    throw std::runtime_error("SDK_PACKET_CRC_MISMATCH");
  }
  const auto expected_mode_byte = static_cast<std::uint8_t>(
      static_cast<unsigned>(id) |
      (static_cast<unsigned>(mode) << 4U));
  if (packet[2] != expected_mode_byte) {
    throw std::runtime_error("SDK_PACKET_MODE_OR_ID_MISMATCH");
  }
}

void validate_packet_fields(const FrozenSdkPacket& packet,
                            const FrozenPacketExpectation& expected) {
  validate_packet_structure(packet, expected.id, expected.mode);
  if (load_u16_le(packet.data() + 3) != expected.tau_bits ||
      load_u16_le(packet.data() + 5) != expected.dq_bits ||
      load_u32_le(packet.data() + 7) != expected.q_bits ||
      load_u16_le(packet.data() + 11) != expected.kp_count ||
      load_u16_le(packet.data() + 13) != expected.kd_count) {
    throw std::runtime_error("SDK_PACKET_FIELD_ENCODING_MISMATCH");
  }
}

FrozenSdkPacket serialize_command_with_frozen_sdk(MotorCmd& command) {
  // These are the real frozen shared-library entry points.  They only mutate
  // and expose MotorCmd's in-memory packet cache; no SerialPort is involved.
  command.modify_data(&command);
  if (command.hex_len != static_cast<int>(kFrozenSdkPacketSize)) {
    throw std::runtime_error("SDK_PACKET_LENGTH_NOT_17");
  }
  const auto* source = command.get_motor_send_data();
  if (source == nullptr) {
    throw std::runtime_error("SDK_PACKET_POINTER_NULL");
  }
  const auto* object_bytes =
      reinterpret_cast<const std::uint8_t*>(std::addressof(command));
  const std::uintptr_t source_address =
      reinterpret_cast<std::uintptr_t>(source);
  const std::uintptr_t object_address =
      reinterpret_cast<std::uintptr_t>(object_bytes);
  if (source_address < object_address ||
      source_address - object_address != kFrozenGoPacketOffsetInMotorCmd) {
    throw std::runtime_error("SDK_MOTORCMD_PRIVATE_LAYOUT_MISMATCH");
  }
  FrozenSdkPacket packet{};
  std::copy_n(source, packet.size(), packet.begin());
  return packet;
}

void initialize_command(MotorCmd& command, int id, int mode, double tau,
                        double dq, double q, double kp, double kw) {
  // The vendor MotorCmd default constructor is user-provided and empty.  A
  // braced construction therefore does not initialize its private packet
  // cache.  The frozen serializer preserves bit 7 of the cached mode byte, so
  // zero the complete trivially-copyable object before assigning API fields.
  zero_object(command);
  command.motorType = MotorType::GO_M8010_6;
  command.hex_len = 0;
  command.id = static_cast<unsigned short>(id);
  command.mode = static_cast<unsigned short>(mode);
  command.tau = static_cast<float>(tau);
  command.dq = static_cast<float>(dq);
  command.q = static_cast<float>(q);
  command.kp = static_cast<float>(kp);
  command.kd = static_cast<float>(kw);
  command.Res.u32 = 0;
}

double local_difference(double value, double reference) {
  const double delta = value - reference;
  if (!std::isfinite(delta) || std::abs(delta) >= kPi) {
    throw std::runtime_error("RAW_WRAP_OR_JUMP_AMBIGUOUS");
  }
  return delta;
}

MotorCmd make_command(int id, int mode, double tau, double dq, double q,
                      double kp, double kw) {
  MotorCmd command;
  initialize_command(command, id, mode, tau, dq, q, kp, kw);
  // Every production command is structurally serialized and audited before
  // any SerialPort can consume it.  sendRecv() may serialize it again, but the
  // complete-object zeroing makes that operation deterministic.
  const FrozenSdkPacket packet = serialize_command_with_frozen_sdk(command);
  validate_packet_structure(packet, id, mode);
  return command;
}

void initialize_data(MotorData& data) {
  // MotorData has the same empty-constructor pattern.  The receive buffer is
  // filled by the SDK, but deterministic initialization prevents stale bytes
  // from surviving short/failed receives.
  zero_object(data);
  data.motorType = MotorType::GO_M8010_6;
  data.hex_len = 0;
  data.motor_id = 0;
  data.mode = 0;
  data.temp = 0;
  data.merror = 0;
  data.tau = std::numeric_limits<float>::quiet_NaN();
  data.dq = std::numeric_limits<float>::quiet_NaN();
  data.q = std::numeric_limits<float>::quiet_NaN();
  data.correct = false;
}

class VelocityEstimator {
 public:
  void reset() { samples_.clear(); }
  void update(double time, double position, State& state) {
    if (!samples_.empty()) {
      state.dt_s = time - samples_.back().first;
      if (!(state.dt_s > 0.0)) {
        throw std::runtime_error("FEEDBACK_TIMESTAMP_INVALID");
      }
      if (state.dt_s > 0.1) {
        // A scheduler or fsync pause invalidates every finite-difference
        // window, but it must not poison the estimator forever: terminal
        // BRAKE confirmation still needs to consume subsequent feedback.
        // The FINAL runner treats loss of velocity_valid during active FOC
        // as a fail-closed safety event.
        samples_.clear();
        samples_.emplace_back(time, position);
        return;
      }
      state.qdot_joint_single =
          (position - samples_.back().second) / state.dt_s;
    }
    samples_.emplace_back(time, position);
    while (samples_.size() > 11) samples_.pop_front();
    if (samples_.size() >= 5) state.qdot_joint_fast = slope(5);
    if (samples_.size() >= 11) {
      state.qdot_joint_slow = slope(11);
      state.velocity_valid = true;
    }
  }

 private:
  double slope(std::size_t count) const {
    const std::size_t first = samples_.size() - count;
    double mt = 0.0, mp = 0.0;
    for (std::size_t i = first; i < samples_.size(); ++i) {
      mt += samples_[i].first;
      mp += samples_[i].second;
    }
    mt /= static_cast<double>(count);
    mp /= static_cast<double>(count);
    double num = 0.0, den = 0.0;
    for (std::size_t i = first; i < samples_.size(); ++i) {
      const double dt = samples_[i].first - mt;
      num += dt * (samples_[i].second - mp);
      den += dt * dt;
    }
    if (!(den > 0.0)) throw std::runtime_error("VELOCITY_REGRESSION_INVALID");
    return num / den;
  }
  std::deque<std::pair<double, double>> samples_;
};
}  // namespace

void runFrozenSdkPacketSelfTest() {
  const int brake_mode =
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
  const int foc_mode =
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
  if (brake_mode != 0 || foc_mode != 1 ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) -
               6.3299999237060547) > 1e-6) {
    throw std::runtime_error("SDK_QUERY_AUTHORITY_MISMATCH");
  }

  constexpr FrozenSdkPacket kBrakeGolden = {
      0xfe, 0xee, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x23};
  constexpr FrozenSdkPacket kFocGainGolden = {
      0xfe, 0xee, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x80, 0x02, 0x40, 0x00, 0x05, 0xa7};
  constexpr FrozenSdkPacket kFocScaledGolden = {
      0xfe, 0xee, 0x10, 0x00, 0x00, 0x51, 0x00, 0x5f, 0x14,
      0x00, 0x00, 0x80, 0x02, 0x40, 0x00, 0x8f, 0x5d};
  constexpr FrozenSdkPacket kFocNegativeScaledGolden = {
      0xfe, 0xee, 0x10, 0x00, 0x00, 0xaf, 0xff, 0xa1, 0xeb,
      0xff, 0xff, 0x80, 0x02, 0x40, 0x00, 0x6c, 0x18};

  const FrozenPacketExpectation brake_expected{
      0, static_cast<std::uint8_t>(brake_mode), 0, 0, 0, 0, 0};
  const FrozenPacketExpectation foc_gain_expected{
      0, static_cast<std::uint8_t>(foc_mode), 0, 0, 0, 640, 64};
  const FrozenPacketExpectation foc_scaled_expected{
      0, static_cast<std::uint8_t>(foc_mode), 0, 81, 5215, 640, 64};
  const FrozenPacketExpectation foc_negative_scaled_expected{
      0, static_cast<std::uint8_t>(foc_mode), 0, 0xffafU, 0xffffeba1U,
      640, 64};

  MotorCmd brake = make_command(0, brake_mode, 0.0, 0.0, 0.0, 0.0, 0.0);
  const FrozenSdkPacket brake_packet =
      serialize_command_with_frozen_sdk(brake);
  validate_packet_fields(brake_packet, brake_expected);
  if (brake_packet != kBrakeGolden) {
    throw std::runtime_error("SDK_BRAKE_GOLDEN_PACKET_MISMATCH");
  }

  MotorCmd foc_gain =
      make_command(0, foc_mode, 0.0, 0.0, 0.0, 0.50, 0.05);
  const FrozenSdkPacket foc_gain_packet =
      serialize_command_with_frozen_sdk(foc_gain);
  validate_packet_fields(foc_gain_packet, foc_gain_expected);
  if (foc_gain_packet != kFocGainGolden) {
    throw std::runtime_error("SDK_FOC_GAIN_GOLDEN_PACKET_MISMATCH");
  }

  // Values are deliberately far from quantization boundaries.  The frozen
  // SDK encodes dq as trunc(dq / 6.2832 * 256) and q as
  // trunc(q / 6.2832 * 32768), yielding 81 and 5215 respectively.
  MotorCmd foc_scaled =
      make_command(0, foc_mode, 0.0, 2.0, 1.0, 0.50, 0.05);
  const FrozenSdkPacket foc_scaled_packet =
      serialize_command_with_frozen_sdk(foc_scaled);
  validate_packet_fields(foc_scaled_packet, foc_scaled_expected);
  if (foc_scaled_packet != kFocScaledGolden) {
    throw std::runtime_error("SDK_FOC_SCALED_GOLDEN_PACKET_MISMATCH");
  }

  MotorCmd foc_negative_scaled =
      make_command(0, foc_mode, 0.0, -2.0, -1.0, 0.50, 0.05);
  const FrozenSdkPacket foc_negative_scaled_packet =
      serialize_command_with_frozen_sdk(foc_negative_scaled);
  validate_packet_fields(foc_negative_scaled_packet,
                         foc_negative_scaled_expected);
  if (foc_negative_scaled_packet != kFocNegativeScaledGolden) {
    throw std::runtime_error(
        "SDK_FOC_NEGATIVE_SCALED_GOLDEN_PACKET_MISMATCH");
  }

  // Prove that the production initializer erases every possible old value of
  // the SDK's private mode byte and all other cached packet bytes.
  constexpr std::array<std::uint8_t, 4> kDirtyPatterns = {
      0x00, 0x80, 0xa5, 0xff};
  for (const std::uint8_t pattern : kDirtyPatterns) {
    MotorCmd dirty_command;
    std::memset(static_cast<void*>(std::addressof(dirty_command)), pattern,
                sizeof(dirty_command));
    initialize_command(dirty_command, 0, foc_mode, 0.0, 0.0, 0.0, 0.50,
                       0.05);
    const FrozenSdkPacket dirty_packet =
        serialize_command_with_frozen_sdk(dirty_command);
    validate_packet_fields(dirty_packet, foc_gain_expected);
    if (dirty_packet != kFocGainGolden) {
      throw std::runtime_error("SDK_PACKET_DIRTY_PATTERN_NONDETERMINISM");
    }
  }

  const auto expect_rejection = [](const FrozenSdkPacket& packet,
                                   const FrozenPacketExpectation& expected,
                                   const char* expected_reason) {
    try {
      validate_packet_fields(packet, expected);
    } catch (const std::runtime_error& error) {
      if (std::string(error.what()) == expected_reason) return;
      throw std::runtime_error("SDK_PACKET_NEGATIVE_WRONG_REJECTION");
    }
    throw std::runtime_error("SDK_PACKET_NEGATIVE_NOT_REJECTED");
  };

  // A malicious/defective packet can have reserved bit 7 set and still carry
  // a valid CRC, so this must fail independently of CRC validation.
  FrozenSdkPacket bad_reserved = kFocGainGolden;
  bad_reserved[2] = static_cast<std::uint8_t>(bad_reserved[2] | 0x80U);
  refresh_packet_crc(bad_reserved);
  expect_rejection(bad_reserved, foc_gain_expected,
                   "SDK_PACKET_RESERVED_BIT_NONZERO");

  FrozenSdkPacket bad_crc = kFocGainGolden;
  bad_crc[7] = static_cast<std::uint8_t>(bad_crc[7] ^ 0x01U);
  expect_rejection(bad_crc, foc_gain_expected, "SDK_PACKET_CRC_MISMATCH");

  FrozenSdkPacket bad_mode = kFocGainGolden;
  bad_mode[2] = 0x00;
  refresh_packet_crc(bad_mode);
  expect_rejection(bad_mode, foc_gain_expected,
                   "SDK_PACKET_MODE_OR_ID_MISMATCH");

  FrozenSdkPacket bad_field = kFocGainGolden;
  bad_field[11] = static_cast<std::uint8_t>(bad_field[11] ^ 0x01U);
  refresh_packet_crc(bad_field);
  expect_rejection(bad_field, foc_gain_expected,
                   "SDK_PACKET_FIELD_ENCODING_MISMATCH");
}

struct GoM8010Driver::Impl {
  explicit Impl(DriverConfig value)
      : config(std::move(value)),
        brake_mode(queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE)),
        foc_mode(queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC)),
        command(make_command(config.motor_id, brake_mode, 0, 0, 0, 0, 0)) {
    if (config.motor_id != 0 || config.direction_sign != 1 ||
        std::abs(config.gear_ratio - 6.3299999237060547) > 1e-12 ||
        brake_mode != 0 || foc_mode != 1 ||
        std::abs(queryGearRatio(MotorType::GO_M8010_6) -
                 config.gear_ratio) > 1e-6) {
      throw std::runtime_error("DRIVER_AUTHORITY_MISMATCH");
    }
  }

  void set_command(double tau, double dq, double q, double kp, double kw,
                   int mode) {
    if (!std::isfinite(tau) || !std::isfinite(dq) || !std::isfinite(q) ||
        !std::isfinite(kp) || !std::isfinite(kw) || std::abs(tau) > 0.05 ||
        kp < 0.0 || kp > 0.50 || kw < 0.0 || kw > 0.05) {
      throw std::runtime_error("COMMAND_OUTSIDE_FAST_AUTHORITY");
    }
    command = make_command(config.motor_id, mode, tau, dq, q, kp, kw);
    audit.mode = mode;
    audit.tau_rotor_nm = tau;
    audit.dq_rotor_rad_s = dq;
    audit.q_rotor_rad = q;
    audit.kp = kp;
    audit.kw = kw;
    audit.encoded_tau_count = static_cast<int>(std::trunc(tau * 256.0));
    audit.encoded_kpos = static_cast<int>(std::trunc(kp * 1280.0));
    audit.encoded_kspd = static_cast<int>(std::trunc(kw * 1280.0));
  }

  DriverConfig config;
  int brake_mode;
  int foc_mode;
  std::unique_ptr<SerialPort> serial;
  MotorCmd command;
  CommandState audit;
  VelocityEstimator estimator;
  std::chrono::steady_clock::time_point start;
  std::uint64_t sequence = 0;
  bool healthy = false;
  bool reference_set = false;
  double reference_raw = 0.0;
  double previous_raw = 0.0;
  double unwrapped_raw = 0.0;
  double position_kp = 0.20;
  double position_kw = 0.01;
  double position_tff = 0.0;
};

GoM8010Driver::GoM8010Driver(DriverConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
GoM8010Driver::~GoM8010Driver() = default;

void GoM8010Driver::connect() {
  if (impl_->serial) throw std::runtime_error("DRIVER_ALREADY_CONNECTED");
  // Fail closed on a frozen-SDK ABI or serializer mismatch before opening the
  // target device.  The self-test itself is pure in-memory code.
  runFrozenSdkPacketSelfTest();
  impl_->serial = std::make_unique<SerialPort>(
      impl_->config.port, 16, static_cast<std::uint32_t>(kBaud), 20000,
      BlockYN::NO, bytesize_t::eightbits, parity_t::parity_none,
      stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none);
  impl_->start = std::chrono::steady_clock::now();
  impl_->healthy = true;
}

void GoM8010Driver::disconnect() noexcept {
  impl_->serial.reset();
  impl_->healthy = false;
}

State GoM8010Driver::readTelemetry() {
  if (!impl_->serial) throw std::runtime_error("DRIVER_NOT_CONNECTED");
  MotorData data{};
  initialize_data(data);
  bool sent = false;
  try { sent = impl_->serial->sendRecv(&impl_->command, &data); } catch (...) {}
  const auto now = std::chrono::steady_clock::now();
  State state;
  state.sequence = impl_->sequence++;
  state.monotonic_s = std::chrono::duration<double>(now - impl_->start).count();
  state.send_recv = sent;
  state.correct = data.correct;
  state.motor_id = data.motor_id;
  state.mode = data.mode;
  state.q_raw = data.q;
  state.dq_sdk = data.dq;
  state.tau_sdk = data.tau;
  state.temperature_c = data.temp;
  state.merror = data.merror;
  state.feedback_valid =
      sent && data.correct && data.motor_id == impl_->config.motor_id &&
      std::isfinite(state.q_raw) && std::isfinite(state.dq_sdk) &&
      std::isfinite(state.tau_sdk) && state.temperature_c >= 0;
  if (!state.feedback_valid) {
    impl_->healthy = false;
    return state;
  }
  if (impl_->reference_set) {
    impl_->unwrapped_raw += local_difference(state.q_raw, impl_->previous_raw);
    impl_->previous_raw = state.q_raw;
    state.q_raw_unwrapped = impl_->unwrapped_raw;
    state.q_joint = impl_->config.direction_sign *
                    (state.q_raw_unwrapped - impl_->reference_raw) /
                    impl_->config.gear_ratio;
    state.joint_envelope_ok =
        std::abs(state.q_joint) <= impl_->config.joint_envelope_rad;
    impl_->estimator.update(state.monotonic_s, state.q_joint, state);
    if (std::isfinite(state.qdot_joint_single)) {
      state.qdot_rotor_fd = state.qdot_joint_single * impl_->config.gear_ratio;
    }
  } else {
    state.q_raw_unwrapped = state.q_raw;
  }
  impl_->healthy = state.merror == 0 && state.temperature_c >= 0 &&
                   state.temperature_c < impl_->config.temperature_limit_c &&
                   state.joint_envelope_ok;
  return state;
}

State GoM8010Driver::readState() {
  State state = readTelemetry();
  if (!state.feedback_valid || state.merror != 0 || state.temperature_c < 0 ||
      state.temperature_c >= impl_->config.temperature_limit_c ||
      !state.joint_envelope_ok) {
    throw std::runtime_error("FEEDBACK_SAFETY_VALIDATION_FAILED");
  }
  return state;
}

void GoM8010Driver::brake() {
  impl_->set_command(0, 0, 0, 0, 0, impl_->brake_mode);
}
void GoM8010Driver::enterFoc() {
  impl_->set_command(0, 0, 0, 0, 0, impl_->foc_mode);
}
void GoM8010Driver::enterPositionMode(double kp, double kd) {
  setPositionControl(kp, kd, 0.0);
  enterFoc();
}
void GoM8010Driver::commandTorqueRotorNm(double torque_nm) {
  impl_->set_command(torque_nm, 0, 0, 0, 0, impl_->foc_mode);
}
void GoM8010Driver::commandRotorState(double tau, double dq, double q,
                                      double kp, double kw) {
  impl_->set_command(tau, dq, q, kp, kw, impl_->foc_mode);
}
void GoM8010Driver::setPositionControl(double kp, double kw, double tff) {
  if (!std::isfinite(kp) || !std::isfinite(kw) || !std::isfinite(tff) ||
      kp < 0 || kp > 0.50 || kw < 0 || kw > 0.05 || std::abs(tff) > 0.05) {
    throw std::runtime_error("POSITION_CONTROL_PARAMETER_INVALID");
  }
  impl_->position_kp = kp;
  impl_->position_kw = kw;
  impl_->position_tff = tff;
}
void GoM8010Driver::commandJointPositionRad(double joint) {
  commandJointState(joint, 0.0);
}
void GoM8010Driver::commandJointState(double joint, double velocity) {
  if (!impl_->reference_set || !std::isfinite(joint) ||
      !std::isfinite(velocity) ||
      std::abs(joint) > impl_->config.joint_envelope_rad ||
      std::abs(velocity) > kFinalMaxJointVelocityRadS + 1e-12) {
    throw std::runtime_error("JOINT_POSITION_COMMAND_INVALID");
  }
  const double rotor = impl_->reference_raw + impl_->config.direction_sign *
                       joint * impl_->config.gear_ratio;
  const double rotor_velocity = impl_->config.direction_sign * velocity *
                                impl_->config.gear_ratio;
  impl_->set_command(impl_->position_tff, rotor_velocity, rotor,
                     impl_->position_kp,
                     impl_->position_kw, impl_->foc_mode);
}
void GoM8010Driver::setSessionReference(double raw) {
  if (!std::isfinite(raw)) throw std::runtime_error("SESSION_REFERENCE_INVALID");
  impl_->reference_raw = raw;
  impl_->previous_raw = raw;
  impl_->unwrapped_raw = raw;
  impl_->reference_set = true;
  impl_->estimator.reset();
}
void GoM8010Driver::setSessionZero(double raw) { setSessionReference(raw); }
bool GoM8010Driver::emergencyBrake() noexcept {
  if (!impl_->serial) return false;
  int consecutive = 0;
  try {
    brake();
    for (int i = 0; i < 50; ++i) {
      try {
        const State state = readTelemetry();
        consecutive =
            state.feedback_valid && state.merror == 0 &&
                    state.mode == static_cast<unsigned>(impl_->brake_mode)
                ? consecutive + 1
                : 0;
      } catch (...) { consecutive = 0; }
      if (consecutive >= 5) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } catch (...) { return false; }
  return false;
}
bool GoM8010Driver::healthy() const noexcept { return impl_->healthy; }
bool GoM8010Driver::referenceSet() const noexcept { return impl_->reference_set; }
int GoM8010Driver::brakeMode() const noexcept { return impl_->brake_mode; }
int GoM8010Driver::focMode() const noexcept { return impl_->foc_mode; }
double GoM8010Driver::sessionReferenceRaw() const {
  if (!impl_->reference_set) throw std::runtime_error("SESSION_REFERENCE_UNSET");
  return impl_->reference_raw;
}
const CommandState& GoM8010Driver::commandState() const noexcept {
  return impl_->audit;
}
}  // namespace go_m8010
