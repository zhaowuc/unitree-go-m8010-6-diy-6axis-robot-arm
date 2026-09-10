#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include <sys/file.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <linux/fs.h>
#include <time.h>
#include <unistd.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace {
using Clock = std::chrono::steady_clock;

std::uint64_t monotonic_ns();
constexpr char kGate[] = "V15_30A_GUI_GO_CONTROL_AUTHORIZED=YES";
constexpr double kPi = 3.14159265358979323846;
constexpr double kGear = 6.329999923706055;
constexpr double kPeriod = 0.01;
// Full frozen 3D-model limits relative to the current vertical session anchor
// [0, 90, -14.40, 13.49, 47.94, 0] degrees.  Command endpoints are not
// statically inset; collision clearance is a separate whole-arm GUI guard.
constexpr std::array<double, 6> kModelCommandLower{{
    -180.0 * kPi / 180.0, -260.0 * kPi / 180.0,
    -155.6 * kPi / 180.0, -129.49 * kPi / 180.0,
    -118.54 * kPi / 180.0, -180.0 * kPi / 180.0}};
constexpr std::array<double, 6> kModelCommandUpper{{
    +180.0 * kPi / 180.0, +80.0 * kPi / 180.0,
    +184.4 * kPi / 180.0, +145.51 * kPi / 180.0,
    +103.26 * kPi / 180.0, +180.0 * kPi / 180.0}};
// Feedback hard-fault detection is intentionally distinct from command
// authorization.  Half a degree covers endpoint quantization and trajectory
// observer noise without extending any commanded target range.
constexpr double kFeedbackEnvelopeTolerance = 0.5 * kPi / 180.0;
constexpr double kJ2SyncWarningLimit = 0.25 * kPi / 180.0;
constexpr double kJ2SyncLimit = 0.5 * kPi / 180.0;
constexpr double kJ2SyncRecoveryLimit = kJ2SyncWarningLimit;
constexpr int kJ2SyncRecoveryConsecutiveFrames = 5;
constexpr double kJ2SessionStartupTolerance = 2.0 * kPi / 180.0;
constexpr double kJ2StartupPermitRawDelta =
    kGear * 0.25 * kPi / 180.0;
constexpr double kJ2StartupPermitRawSpan =
    kGear * 0.20 * kPi / 180.0;
constexpr std::uint64_t kJ2StartupPermitMaximumTtlNs = 30000000000ULL;
constexpr std::uint64_t kJ2CaptureToPermitMaximumAgeNs = 300000000000ULL;
constexpr std::size_t kJ2StartupPermitMinimumBrakeFrames = 50U;
// A newly opened J2 serial channel has once produced exactly three paired
// no-reply transactions before becoming healthy.  Qualify the channel before
// publishing any UDP state so neither the raw capture nor the production GUI
// consumes an uninitialized first sample.  This window is prefix-only: after
// the first healthy pair, any invalid pair is a hard startup failure.
constexpr int kJ2StartupPrimeMaximumInvalidPrefixPairs = 3;
constexpr int kJ2StartupPrimeRequiredHealthyPairs = 5;
constexpr std::uint64_t kJ2StartupPrimeMaximumElapsedNs = 500000000ULL;
constexpr double kPersistentPhaseMismatchLimit = 0.10;
// GO-M8010-6 manual values and all controller torque fields are rotor-side.
// The frozen V15.18A gravity audit peaks at 13.7584214318 N.m on J2, or
// 1.086763 N.m per rotor after the 6.33:1 reduction and equal motor share.
// Keep the normal-motion envelope above that load with bounded headroom, but
// well below the manual's 23.7 N.m geared-output / 6.33 rotor equivalent.
constexpr double kGoManualMaximumOutputTorqueNm = 23.7;
constexpr double kGoManualMaximumRotorEquivalentNm =
    kGoManualMaximumOutputTorqueNm / kGear;
constexpr std::array<double, 6> kFrozenMaximumGravityJointNm{{
    3.818740297241469e-16, 13.75842143182542, 6.4747735382240394,
    2.2171628590237833, 0.7261632329533525, 0.01010275736943872}};
constexpr std::array<double, 6> kHoldIntegralRotorHardNm{{
    0.35, 1.50, 1.60, 0.75, 0.50, 0.0}};
constexpr std::array<double, 6> kAuxPredictedRotorWorkNm{{
    2.50, 0.0, 3.00, 2.50, 2.00, 0.0}};
constexpr std::array<double, 6> kAuxPredictedRotorPdHardNm{{
    2.50, 0.0, 3.00, 2.50, 2.00, 0.0}};
// Existing controller software working clamps.  They may indicate software
// saturation to the watchdog, but are not continuous-torque ratings.
constexpr double kJ2RotorTorqueFeedbackHardNm = 1.80;
constexpr double kJ2RecoveryRotorTorqueFeedbackHardNm = 3.20;
constexpr double kJ2PredictedRotorPdHardNm = 1.75;
constexpr double kJ2RecoveryPredictedRotorPdHardNm = 2.80;
constexpr double kJ2PredictedRotorWorkNm = 1.75;
constexpr double kJ2RecoveryPredictedRotorWorkNm = 3.00;
constexpr double kJ2IntegralRotorHardNm = kHoldIntegralRotorHardNm[1];
constexpr double kJ2IntegralKiPerRotorRadS = 1.00;
constexpr double kJ2IntegralRateHardNmS = 0.50;
constexpr double kJ2IntegralEnterError = 10.0 * kPi / 180.0;
constexpr double kJ2IntegralEnterVelocity = 1.5 * kPi / 180.0;
constexpr double kJ2IntegralDeadband = 0.05 * kPi / 180.0;
constexpr double kJ2EndpointProfileTolerance = 0.10 * kPi / 180.0;
constexpr int kJ2IntegralDwellFrames = 20;
constexpr double kAuxIntegralKiPerRotorRadS = 0.50;
constexpr double kAuxIntegralRateHardNmS = 0.50;
constexpr double kAuxIntegralEnterError = 8.0 * kPi / 180.0;
constexpr double kAuxIntegralEnterVelocity = 1.5 * kPi / 180.0;
constexpr double kAuxIntegralDeadband = 0.05 * kPi / 180.0;
constexpr int kAuxIntegralDwellFrames = 20;
constexpr double kJ2DerivedVelocityFilterAlpha = 0.10;
constexpr double kJ2MaximumAcceleration = 15.0 * kPi / 180.0;
constexpr double kJ2BaseKp = 0.60;
constexpr double kJ2BaseKd = 0.10;
constexpr double kJ2MovingKpLimit = 3.00;
constexpr double kJ2MovingKdLimit = 0.30;
constexpr double kJ2KpRampSeconds = 0.50;
constexpr double kArrivalTolerance = 0.5 * kPi / 180.0;
constexpr double kArrivalDwellSeconds = 0.5;
constexpr int kArrivalDwellMinimumFrames = 50;
constexpr double kBrakeStationaritySpan = 0.20 * kPi / 180.0;
constexpr double kTargetTimeoutSeconds = 90.0;
constexpr double kLeaseSeconds = 0.5;
constexpr double kFixedHoldCaptureWindow = 2.0 * kPi / 180.0;
constexpr double kEmpiricalInitialHoldCaptureWindow =
    0.25 * kPi / 180.0;
constexpr double kFixedHoldRepeatTolerance = 1e-9;
constexpr double kFixedHoldFeedbackFreshSeconds = 0.10;
constexpr int kBrakeMode = 0;
constexpr int kFocMode = 1;
constexpr char kThermalConfigSha256[] =
    "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467";
constexpr int kThermalCooldownConsecutiveFrames = 500;
// These watchdog thresholds observe the controller's existing software
// governors/working clamps.  They are conservative project guards pending
// replay/plant validation, never a motor continuous-torque rating.
constexpr char kLoadLimitWatchdogAuthority[] =
    "SOFTWARE_GUARD_NOT_CONTINUOUS_RATING";
constexpr double kNoProgressMinimumPositionError = 2.0 * kPi / 180.0;
constexpr double kNoProgressMinimumImprovement = 0.25 * kPi / 180.0;
constexpr double kNoProgressWindowSeconds = 3.0;
constexpr int kNoProgressMinimumQualifyingFrames = 100;
constexpr std::size_t kCommandPacketBudget = 32U;
constexpr std::size_t kMaximumTrackedCommandSources = 32U;
constexpr std::uint64_t kMaximumCommandSourceAgeNs = 250000000ULL;
constexpr std::uint64_t kCommandSourceTakeoverLockNs = 500000000ULL;
constexpr std::uint64_t kMaximumCommandSequence =
    static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
constexpr std::uint64_t kMaximumQuinticIntervalCount = 1000000ULL;
constexpr std::uint64_t kMaximumQuinticSamplePeriodNs = 10000000ULL;
constexpr double kQuinticPeakVelocityScale = 1.875;
constexpr double kQuinticPeakAccelerationScale = 5.773502691896257645;
constexpr double kCommandRejectSummarySeconds = 5.0;
constexpr int kActiveDeadlineConsecutiveLimit = 3;
constexpr std::array<double, 6> kKpLimits{{
    1.5, 3.0, 2.0, 2.0, 1.5, 0.0}};
constexpr std::array<double, 6> kKdLimits{{
    0.15, 0.30, 0.15, 0.15, 0.12, 0.0}};
constexpr std::array<double, 6> kRecoveryFeedforwardLimits{{
    0.20, 1.75, 1.00, 0.40, 0.20, 0.0}};
constexpr char kGravityAuthoritySchema[] =
    "go-m8010-gravity-command-authority/1.1";
constexpr char kOfficialGravityAuthoritySchema[] =
    "go-m8010-gravity-command-authority/1.0";
constexpr char kEmpiricalAuthorityClass[] =
    "EMPIRICAL_VALIDATION_ENVELOPE";
constexpr char kEmpiricalRatingClassification[] =
    "NOT_OFFICIAL_CONTINUOUS_RATING";
constexpr char kProductionModelSha256[] =
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9";
constexpr char kGravityConfigSha256[] =
    "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d";
constexpr std::uint64_t kMaximumGravityAuthorityAgeNs = 250000000ULL;
constexpr std::uint64_t kEmpiricalZeroHoldTransitionMaximumAgeNs =
    2000000000ULL;
// Frozen-model command envelopes, not continuous motor torque ratings.
constexpr std::array<double, 6> kGravityFeedforwardLimits{{
    0.20, 1.75, 1.10, 0.40, 0.20, 0.0}};
constexpr double kGravityFeedforwardSlewNmPerSecond = 1.0;
std::atomic<bool> g_stop{false};

struct ThermalPolicy {
  double normal_below_c = 0.0;
  double warning_below_c = 0.0;
  double derating_start_c = 0.0;
  double thermal_stop_c = 0.0;
  double rearm_below_c = 0.0;
  double cooldown_seconds = 0.0;
  double slope_window_seconds = 0.0;
  bool configured = false;
};

ThermalPolicy g_thermal_policy;

double thermal_derating_factor_for_raw_temperature(int temperature_c) {
  if (!g_thermal_policy.configured)
    throw std::runtime_error("THERMAL_POLICY_NOT_CONFIGURED");
  if (temperature_c < 0 ||
      temperature_c >= g_thermal_policy.thermal_stop_c)
    return 0.0;
  if (temperature_c <= g_thermal_policy.derating_start_c) return 1.0;
  return std::clamp(
      (g_thermal_policy.thermal_stop_c - temperature_c) /
          (g_thermal_policy.thermal_stop_c -
           g_thermal_policy.derating_start_c),
      0.0, 1.0);
}

std::uint64_t monotonic_ns_at(Clock::time_point when) {
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
      when.time_since_epoch()).count();
  return count > 0 ? static_cast<std::uint64_t>(count) : 0U;
}

bool valid_source_instance_id(const std::string& value) {
  return value.size() == 32U &&
      std::all_of(value.begin(), value.end(), [](char character) {
        return (character >= '0' && character <= '9') ||
            (character >= 'a' && character <= 'f');
      });
}

bool within_model_command_envelope(int joint_index, double position) {
  if (joint_index < 0 || joint_index >= 6 || !std::isfinite(position))
    return false;
  const std::size_t index = static_cast<std::size_t>(joint_index);
  return position >= kModelCommandLower[index] - 1e-12 &&
      position <= kModelCommandUpper[index] + 1e-12;
}

bool within_mechanical_feedback_envelope(int joint_index, double position) {
  if (joint_index < 0 || joint_index >= 6 || !std::isfinite(position))
    return false;
  const std::size_t index = static_cast<std::size_t>(joint_index);
  return position >=
          kModelCommandLower[index] - kFeedbackEnvelopeTolerance - 1e-12 &&
      position <=
          kModelCommandUpper[index] + kFeedbackEnvelopeTolerance + 1e-12;
}

bool valid_sha256(const std::string& value) {
  return value.size() == 64U &&
      std::all_of(value.begin(), value.end(), [](char character) {
        return (character >= '0' && character <= '9') ||
            (character >= 'a' && character <= 'f');
      });
}

class Sha256 {
 public:
  Sha256()
      : state_{{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U}} {}

  void update(const std::uint8_t* data, std::size_t size) {
    if (data == nullptr && size != 0U)
      throw std::runtime_error("SHA256_INPUT_INVALID");
    if (size > (std::numeric_limits<std::uint64_t>::max() - bit_count_) / 8U)
      throw std::runtime_error("SHA256_INPUT_TOO_LARGE");
    bit_count_ += static_cast<std::uint64_t>(size) * 8U;
    while (size != 0U) {
      const std::size_t copied =
          std::min(size, block_.size() - block_size_);
      std::memcpy(block_.data() + block_size_, data, copied);
      block_size_ += copied;
      data += copied;
      size -= copied;
      if (block_size_ == block_.size()) {
        transform(block_.data());
        block_size_ = 0U;
      }
    }
  }

  std::array<std::uint8_t, 32> finish() {
    const std::uint64_t original_bit_count = bit_count_;
    block_[block_size_++] = 0x80U;
    if (block_size_ > 56U) {
      std::fill(block_.begin() + static_cast<std::ptrdiff_t>(block_size_),
                block_.end(), 0U);
      transform(block_.data());
      block_size_ = 0U;
    }
    std::fill(block_.begin() + static_cast<std::ptrdiff_t>(block_size_),
              block_.begin() + 56, 0U);
    for (std::size_t index = 0; index < 8U; ++index) {
      block_[63U - index] = static_cast<std::uint8_t>(
          original_bit_count >> (index * 8U));
    }
    transform(block_.data());
    block_size_ = 0U;

    std::array<std::uint8_t, 32> digest{};
    for (std::size_t word = 0; word < state_.size(); ++word) {
      digest[word * 4U] = static_cast<std::uint8_t>(state_[word] >> 24U);
      digest[word * 4U + 1U] =
          static_cast<std::uint8_t>(state_[word] >> 16U);
      digest[word * 4U + 2U] =
          static_cast<std::uint8_t>(state_[word] >> 8U);
      digest[word * 4U + 3U] = static_cast<std::uint8_t>(state_[word]);
    }
    return digest;
  }

 private:
  static std::uint32_t rotate_right(std::uint32_t value,
                                    unsigned int count) {
    return (value >> count) | (value << (32U - count));
  }

  void transform(const std::uint8_t* block) {
    static constexpr std::array<std::uint32_t, 64> kRoundConstants{{
        0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
        0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
        0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
        0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
        0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
        0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
        0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
        0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
        0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
        0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
        0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
        0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
        0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
        0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
        0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
        0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U}};
    std::array<std::uint32_t, 64> schedule{};
    for (std::size_t index = 0; index < 16U; ++index) {
      const std::size_t offset = index * 4U;
      schedule[index] =
          (static_cast<std::uint32_t>(block[offset]) << 24U) |
          (static_cast<std::uint32_t>(block[offset + 1U]) << 16U) |
          (static_cast<std::uint32_t>(block[offset + 2U]) << 8U) |
          static_cast<std::uint32_t>(block[offset + 3U]);
    }
    for (std::size_t index = 16U; index < schedule.size(); ++index) {
      const std::uint32_t s0 =
          rotate_right(schedule[index - 15U], 7U) ^
          rotate_right(schedule[index - 15U], 18U) ^
          (schedule[index - 15U] >> 3U);
      const std::uint32_t s1 =
          rotate_right(schedule[index - 2U], 17U) ^
          rotate_right(schedule[index - 2U], 19U) ^
          (schedule[index - 2U] >> 10U);
      schedule[index] = schedule[index - 16U] + s0 +
          schedule[index - 7U] + s1;
    }

    std::uint32_t a = state_[0];
    std::uint32_t b = state_[1];
    std::uint32_t c = state_[2];
    std::uint32_t d = state_[3];
    std::uint32_t e = state_[4];
    std::uint32_t f = state_[5];
    std::uint32_t g = state_[6];
    std::uint32_t h = state_[7];
    for (std::size_t index = 0; index < schedule.size(); ++index) {
      const std::uint32_t sum1 = rotate_right(e, 6U) ^
          rotate_right(e, 11U) ^ rotate_right(e, 25U);
      const std::uint32_t choice = (e & f) ^ ((~e) & g);
      const std::uint32_t temporary1 =
          h + sum1 + choice + kRoundConstants[index] + schedule[index];
      const std::uint32_t sum0 = rotate_right(a, 2U) ^
          rotate_right(a, 13U) ^ rotate_right(a, 22U);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temporary2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temporary1;
      d = c;
      c = b;
      b = a;
      a = temporary1 + temporary2;
    }
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_{};
  std::array<std::uint8_t, 64> block_{};
  std::uint64_t bit_count_ = 0U;
  std::size_t block_size_ = 0U;
};

std::string sha256_hex(const std::string& data) {
  Sha256 hash;
  hash.update(reinterpret_cast<const std::uint8_t*>(data.data()), data.size());
  const auto digest = hash.finish();
  static constexpr char kHex[] = "0123456789abcdef";
  std::string result(digest.size() * 2U, '0');
  for (std::size_t index = 0; index < digest.size(); ++index) {
    result[index * 2U] = kHex[digest[index] >> 4U];
    result[index * 2U + 1U] = kHex[digest[index] & 0x0fU];
  }
  return result;
}

void sha256_self_test() {
  if (sha256_hex("") !=
      "e3b0c44298fc1c149afbf4c8996fb924"
      "27ae41e4649b934ca495991b7852b855")
    throw std::runtime_error("SHA256_EMPTY_SELF_TEST_FAILED");
  if (sha256_hex("abc") !=
      "ba7816bf8f01cfea414140de5dae2223"
      "b00361a396177a9cb410ff61f20015ad")
    throw std::runtime_error("SHA256_ABC_SELF_TEST_FAILED");
  if (sha256_hex(std::string(56U, 'a')) !=
      "b35439a4ac6f0948b6d6f9e3c6af0f5"
      "f590ce20f1bde7090ef7970686ec6738a")
    throw std::runtime_error("SHA256_PADDING_SELF_TEST_FAILED");
  if (sha256_hex(std::string(1000U, 'a')) !=
      "41edece42d63e8d9bf515a9ba6932e1c"
      "20cbc9f5a5d134645adb5db1b9737ea3")
    throw std::runtime_error("SHA256_MULTIBLOCK_SELF_TEST_FAILED");
}

class UniqueFd {
 public:
  UniqueFd() = default;
  explicit UniqueFd(int descriptor) : descriptor_(descriptor) {}
  ~UniqueFd() { reset(); }
  UniqueFd(const UniqueFd&) = delete;
  UniqueFd& operator=(const UniqueFd&) = delete;
  UniqueFd(UniqueFd&& other) noexcept : descriptor_(other.release()) {}
  UniqueFd& operator=(UniqueFd&& other) noexcept {
    if (this != &other) reset(other.release());
    return *this;
  }
  int get() const noexcept { return descriptor_; }
  explicit operator bool() const noexcept { return descriptor_ >= 0; }
  int release() noexcept {
    const int descriptor = descriptor_;
    descriptor_ = -1;
    return descriptor;
  }
  void reset(int descriptor = -1) noexcept {
    if (descriptor_ >= 0) (void)::close(descriptor_);
    descriptor_ = descriptor;
  }
 private:
  int descriptor_ = -1;
};

struct SecureFileBytes {
  std::string data;
  std::string sha256;
  struct stat status{};
};

SecureFileBytes read_secure_regular_fd(
    int descriptor, std::size_t maximum_size, bool require_private_0600,
    const char* unsafe_error, const char* read_error) {
  struct stat before{};
  if (::fstat(descriptor, &before) != 0 || !S_ISREG(before.st_mode) ||
      before.st_uid != ::geteuid() || before.st_nlink != 1 ||
      before.st_size <= 0 ||
      static_cast<std::uint64_t>(before.st_size) > maximum_size ||
      (require_private_0600 && (before.st_mode & 07777) != 0600))
    throw std::runtime_error(unsafe_error);
  std::string data(static_cast<std::size_t>(before.st_size), '\0');
  std::size_t offset = 0U;
  while (offset < data.size()) {
    const ssize_t received =
        ::read(descriptor, data.data() + offset, data.size() - offset);
    if (received > 0) {
      offset += static_cast<std::size_t>(received);
    } else if (received < 0 && errno == EINTR) {
      continue;
    } else {
      throw std::runtime_error(read_error);
    }
  }
  std::uint8_t extra = 0U;
  ssize_t extra_read = -1;
  do {
    extra_read = ::read(descriptor, &extra, 1U);
  } while (extra_read < 0 && errno == EINTR);
  struct stat after{};
  if (extra_read != 0 || ::fstat(descriptor, &after) != 0 ||
      before.st_dev != after.st_dev || before.st_ino != after.st_ino ||
      before.st_size != after.st_size || before.st_mtim.tv_sec != after.st_mtim.tv_sec ||
      before.st_mtim.tv_nsec != after.st_mtim.tv_nsec ||
      before.st_ctim.tv_sec != after.st_ctim.tv_sec ||
      before.st_ctim.tv_nsec != after.st_ctim.tv_nsec)
    throw std::runtime_error(read_error);
  return {data, sha256_hex(data), before};
}

SecureFileBytes read_secure_owned_file(
    const std::string& path, std::size_t maximum_size,
    const char* open_error, const char* unsafe_error, const char* read_error) {
  UniqueFd descriptor(::open(
      path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK));
  if (!descriptor) throw std::runtime_error(open_error);
  return read_secure_regular_fd(
      descriptor.get(), maximum_size, true, unsafe_error, read_error);
}

SecureFileBytes read_secure_owned_policy_file(
    const std::string& path, std::size_t maximum_size,
    const char* open_error, const char* unsafe_error, const char* read_error) {
  UniqueFd descriptor(::open(
      path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK));
  if (!descriptor) throw std::runtime_error(open_error);
  // Repository policy files are intentionally readable (normally 0644), but
  // must still be a stable, single-link regular file owned by this worker's
  // effective user.  The exact byte identity is pinned below before any
  // serial device is inspected or opened.
  return read_secure_regular_fd(
      descriptor.get(), maximum_size, false, unsafe_error, read_error);
}

std::string trim_ascii(std::string value) {
  const auto not_space = [](unsigned char character) {
    return character != ' ' && character != '\t' &&
        character != '\r' && character != '\n';
  };
  const auto first = std::find_if(value.begin(), value.end(), not_space);
  const auto last = std::find_if(value.rbegin(), value.rend(), not_space).base();
  if (first >= last) return {};
  return std::string(first, last);
}

ThermalPolicy parse_thermal_policy_yaml(const std::string& data) {
  std::map<std::string, std::string> scalars;
  std::size_t offset = 0U;
  while (offset <= data.size()) {
    const std::size_t newline = data.find('\n', offset);
    std::string line = data.substr(
        offset, newline == std::string::npos
                    ? std::string::npos : newline - offset);
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    line = trim_ascii(std::move(line));
    if (!line.empty()) {
      const std::size_t colon = line.find(':');
      if (colon == std::string::npos)
        throw std::runtime_error("THERMAL_CONFIG_YAML_INVALID");
      const std::string key = trim_ascii(line.substr(0U, colon));
      const std::string value = trim_ascii(line.substr(colon + 1U));
      if (key.empty() || value.empty() || !scalars.emplace(key, value).second)
        throw std::runtime_error("THERMAL_CONFIG_YAML_INVALID");
    }
    if (newline == std::string::npos) break;
    offset = newline + 1U;
  }
  const auto required_string = [&](const char* key) -> const std::string& {
    const auto found = scalars.find(key);
    if (found == scalars.end())
      throw std::runtime_error("THERMAL_CONFIG_FIELD_MISSING");
    return found->second;
  };
  if (required_string("schema") != "go-m8010-thermal-limits/1.0")
    throw std::runtime_error("THERMAL_CONFIG_SCHEMA_MISMATCH");
  const auto required_number = [&](const char* key) {
    const std::string& encoded = required_string(key);
    std::size_t consumed = 0U;
    double value = 0.0;
    try {
      value = std::stod(encoded, &consumed);
    } catch (const std::exception&) {
      throw std::runtime_error("THERMAL_CONFIG_NUMBER_INVALID");
    }
    if (consumed != encoded.size() || !std::isfinite(value))
      throw std::runtime_error("THERMAL_CONFIG_NUMBER_INVALID");
    return value;
  };
  ThermalPolicy policy;
  policy.normal_below_c = required_number("normal_below_c");
  policy.warning_below_c = required_number("warning_below_c");
  policy.derating_start_c = required_number("derating_start_c");
  policy.thermal_stop_c = required_number("thermal_stop_c");
  policy.rearm_below_c = required_number("rearm_below_c");
  policy.cooldown_seconds = required_number("cooldown_seconds");
  policy.slope_window_seconds = required_number("slope_window_seconds");
  if (!(policy.normal_below_c < policy.warning_below_c &&
        policy.warning_below_c < policy.derating_start_c &&
        policy.derating_start_c < policy.thermal_stop_c &&
        policy.normal_below_c < policy.rearm_below_c &&
        policy.rearm_below_c < policy.thermal_stop_c &&
        policy.cooldown_seconds > 0.0 &&
        policy.slope_window_seconds > 0.0))
    throw std::runtime_error("THERMAL_CONFIG_THRESHOLD_ORDER_INVALID");
  policy.configured = true;
  return policy;
}

ThermalPolicy self_test_thermal_policy() {
  return parse_thermal_policy_yaml(
      "schema: go-m8010-thermal-limits/1.0\n"
      "normal_below_c: 45.0\n"
      "warning_below_c: 50.0\n"
      "derating_start_c: 55.0\n"
      "thermal_stop_c: 60.0\n"
      "rearm_below_c: 55.0\n"
      "cooldown_seconds: 30.0\n"
      "slope_window_seconds: 120.0\n");
}

nlohmann::json parse_json_bytes(const std::string& data,
                                const char* invalid_error) {
  try {
    return nlohmann::json::parse(data);
  } catch (const nlohmann::json::exception&) {
    throw std::runtime_error(invalid_error);
  }
}

bool valid_power_session_id(const std::string& value) {
  return !value.empty() && value.size() <= 256U &&
      std::all_of(value.begin(), value.end(), [](unsigned char character) {
        return character >= 0x20U && character != 0x7fU;
      });
}

bool valid_host_boot_id(const std::string& value) {
  if (value.size() != 36U ||
      value[8] != '-' || value[13] != '-' || value[18] != '-' ||
      value[23] != '-')
    return false;
  bool nonzero = false;
  for (std::size_t index = 0; index < value.size(); ++index) {
    if (index == 8U || index == 13U || index == 18U || index == 23U) continue;
    const char character = value[index];
    if (!((character >= '0' && character <= '9') ||
          (character >= 'a' && character <= 'f')))
      return false;
    nonzero = nonzero || character != '0';
  }
  return nonzero;
}

std::string current_host_boot_id() {
  std::ifstream stream("/proc/sys/kernel/random/boot_id");
  std::string value;
  if (!stream || !std::getline(stream, value) || !valid_host_boot_id(value))
    throw std::runtime_error("HOST_BOOT_ID_UNAVAILABLE");
  return value;
}

std::uint64_t current_boottime_ns() {
  timespec value{};
  if (::clock_gettime(CLOCK_BOOTTIME, &value) != 0 || value.tv_sec < 0 ||
      value.tv_nsec < 0 || value.tv_nsec >= 1000000000L)
    throw std::runtime_error("CLOCK_BOOTTIME_UNAVAILABLE");
  const auto seconds = static_cast<std::uint64_t>(value.tv_sec);
  const auto nanoseconds = static_cast<std::uint64_t>(value.tv_nsec);
  if (seconds >
      (std::numeric_limits<std::uint64_t>::max() - nanoseconds) /
          1000000000ULL)
    throw std::runtime_error("CLOCK_BOOTTIME_OVERFLOW");
  const std::uint64_t result = seconds * 1000000000ULL + nanoseconds;
  if (result == 0U) throw std::runtime_error("CLOCK_BOOTTIME_INVALID");
  return result;
}

void signal_handler(int) { g_stop.store(true); }

template <typename T> void zero_object(T& value) {
  static_assert(std::is_trivially_copyable<T>::value, "SDK wrapper ABI");
  std::memset(static_cast<void*>(&value), 0, sizeof(value));
}

std::uint16_t load_u16_le(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0]) |
         static_cast<std::uint16_t>(p[1] << 8U);
}

std::uint16_t crc16_kermit(const std::uint8_t* p, std::size_t n) {
  std::uint16_t crc = 0;
  while (n-- != 0U) {
    crc = static_cast<std::uint16_t>(crc ^ *p++);
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 1U) != 0U
                ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                : static_cast<std::uint16_t>(crc >> 1U);
  }
  return crc;
}

MotorCmd make_command(int id, int mode, double q, double dq, double kp,
                      double kd, double tau = 0.0) {
  MotorCmd command;
  zero_object(command);
  command.motorType = MotorType::GO_M8010_6;
  command.id = static_cast<unsigned short>(id);
  command.mode = static_cast<unsigned short>(mode);
  command.q = static_cast<float>(q);
  command.dq = static_cast<float>(dq);
  command.kp = static_cast<float>(kp);
  command.kd = static_cast<float>(kd);
  command.tau = static_cast<float>(tau);
  command.Res.u32 = 0;
  command.modify_data(&command);
  if (command.hex_len != 17) throw std::runtime_error("COMMAND_LENGTH_INVALID");
  const auto* raw = command.get_motor_send_data();
  if (raw == nullptr || raw[0] != 0xfeU || raw[1] != 0xeeU ||
      raw[2] != static_cast<std::uint8_t>(id | (mode << 4)) ||
      (raw[2] & 0x80U) != 0U ||
      load_u16_le(raw + 15) != crc16_kermit(raw, 15))
    throw std::runtime_error("COMMAND_PACKET_AUDIT_FAILED");
  return command;
}

struct Options {
  bool execute = false;
  bool brake_only = false;
  bool audit_j2_session_bundle = false;
  bool audit_go_aux_session_bundle = false;
  std::string confirm;
  std::string bus;
  int feedback_port = 15300;
  std::string zero_file;
  std::string recovery_hint_file;
  std::string j2_session_reference_file;
  std::string j2_session_launch_permit_file;
  std::string go_aux_session_reference_file;
  std::string go_aux_session_launch_permit_file;
  std::string expected_zero_sha256;
  std::string expected_j2_session_reference_sha256;
  std::string expected_j2_power_session_id;
  std::string expected_go_aux_session_reference_sha256;
  std::string expected_go_aux_power_session_id;
  std::string expected_worker_sha256;
  std::string thermal_config_file;
  std::string expected_thermal_config_sha256;
  std::string expected_gravity_authority_class;
  std::string expected_empirical_envelope_id;
  std::string expected_empirical_envelope_sha256;
  std::string expected_gravity_anchor_sha256;
  std::string expected_gravity_session_id;
  std::string expected_gravity_state_instance_id;
};

struct ExpectedGravityAuthorityBinding {
  std::string authority_class = "UNBOUND_SELF_TEST";
  std::string empirical_envelope_id;
  std::string empirical_envelope_sha256;
  std::string anchor_sha256;
  std::string session_id;
  std::string state_instance_id;
};

ExpectedGravityAuthorityBinding g_expected_gravity_authority_binding;

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&]() {
      if (++i >= argc) throw std::runtime_error("CLI_VALUE_MISSING");
      return std::string(argv[i]);
    };
    if (arg == "--execute") options.execute = true;
    else if (arg == "--brake-only") options.brake_only = true;
    else if (arg == "--audit-j2-session-bundle")
      options.audit_j2_session_bundle = true;
    else if (arg == "--audit-go-aux-session-bundle")
      options.audit_go_aux_session_bundle = true;
    else if (arg == "--confirm") options.confirm = next();
    else if (arg == "--bus") options.bus = next();
    else if (arg == "--feedback-port") options.feedback_port = std::stoi(next());
    else if (arg == "--zero-file") options.zero_file = next();
    else if (arg == "--recovery-hint-file") options.recovery_hint_file = next();
    else if (arg == "--j2-session-reference-file")
      options.j2_session_reference_file = next();
    else if (arg == "--j2-session-launch-permit-file")
      options.j2_session_launch_permit_file = next();
    else if (arg == "--go-aux-session-reference-file")
      options.go_aux_session_reference_file = next();
    else if (arg == "--go-aux-session-launch-permit-file")
      options.go_aux_session_launch_permit_file = next();
    else if (arg == "--expected-zero-sha256")
      options.expected_zero_sha256 = next();
    else if (arg == "--expected-j2-session-reference-sha256")
      options.expected_j2_session_reference_sha256 = next();
    else if (arg == "--expected-j2-power-session-id")
      options.expected_j2_power_session_id = next();
    else if (arg == "--expected-go-aux-session-reference-sha256")
      options.expected_go_aux_session_reference_sha256 = next();
    else if (arg == "--expected-go-aux-power-session-id")
      options.expected_go_aux_power_session_id = next();
    else if (arg == "--expected-worker-sha256")
      options.expected_worker_sha256 = next();
    else if (arg == "--thermal-config")
      options.thermal_config_file = next();
    else if (arg == "--expected-thermal-config-sha256")
      options.expected_thermal_config_sha256 = next();
    else if (arg == "--expected-gravity-authority-class")
      options.expected_gravity_authority_class = next();
    else if (arg == "--expected-empirical-envelope-id")
      options.expected_empirical_envelope_id = next();
    else if (arg == "--expected-empirical-envelope-sha256")
      options.expected_empirical_envelope_sha256 = next();
    else if (arg == "--expected-gravity-anchor-sha256")
      options.expected_gravity_anchor_sha256 = next();
    else if (arg == "--expected-gravity-session-id")
      options.expected_gravity_session_id = next();
    else if (arg == "--expected-gravity-state-instance-id")
      options.expected_gravity_state_instance_id = next();
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  const bool j2_bundle_fields_missing =
      options.zero_file.empty() || options.recovery_hint_file.empty() ||
      options.j2_session_reference_file.empty() ||
      options.j2_session_launch_permit_file.empty() ||
      options.expected_zero_sha256.empty() ||
      options.expected_j2_session_reference_sha256.empty() ||
      options.expected_j2_power_session_id.empty() ||
      options.expected_worker_sha256.empty();
  const bool go_aux_bundle_fields_missing =
      options.zero_file.empty() || options.recovery_hint_file.empty() ||
      options.go_aux_session_reference_file.empty() ||
      options.go_aux_session_launch_permit_file.empty() ||
      options.expected_zero_sha256.empty() ||
      options.expected_go_aux_session_reference_sha256.empty() ||
      options.expected_go_aux_power_session_id.empty() ||
      options.expected_worker_sha256.empty();
  if (options.audit_j2_session_bundle && options.audit_go_aux_session_bundle)
    throw std::runtime_error("SESSION_BUNDLE_AUDIT_SCOPE_AMBIGUOUS");
  if (options.audit_j2_session_bundle) {
    if (options.execute || options.brake_only || options.bus != "j2" ||
        j2_bundle_fields_missing)
      throw std::runtime_error("J2_SESSION_BUNDLE_AUDIT_GATE_INVALID");
    return options;
  }
  if (options.audit_go_aux_session_bundle) {
    if (options.execute || options.brake_only ||
        (options.bus != "j1" && options.bus != "j345") ||
        go_aux_bundle_fields_missing)
      throw std::runtime_error("GO_AUX_SESSION_BUNDLE_AUDIT_GATE_INVALID");
    return options;
  }
  if (!options.execute) return options;
  if (options.confirm != kGate) throw std::runtime_error("ACTIVE_CONTROL_GATE_MISSING");
  if (options.bus != "j1" && options.bus != "j2" && options.bus != "j345")
    throw std::runtime_error("BUS_NOT_ALLOWED");
  if (options.bus == "j2" && !options.brake_only &&
      j2_bundle_fields_missing)
    throw std::runtime_error("J2_SESSION_REFERENCE_GATE_MISSING");
  if ((options.bus == "j1" || options.bus == "j345") &&
      !options.brake_only && go_aux_bundle_fields_missing)
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_GATE_MISSING");
  if (options.thermal_config_file.empty() ||
      options.expected_thermal_config_sha256.empty())
    throw std::runtime_error("THERMAL_CONFIG_AUTHORITY_GATE_MISSING");
  if (!options.brake_only) {
    const bool empirical = options.expected_gravity_authority_class ==
        kEmpiricalAuthorityClass;
    const bool official = options.expected_gravity_authority_class ==
        "OFFICIAL_CONTINUOUS_RATING";
    const bool none = options.expected_gravity_authority_class == "NONE";
    if (!empirical && !official && !none)
      throw std::runtime_error(
          "GRAVITY_AUTHORITY_STARTUP_CLASS_MISSING");
    if (none) {
      if (!options.expected_empirical_envelope_id.empty() ||
          !options.expected_empirical_envelope_sha256.empty() ||
          !options.expected_gravity_anchor_sha256.empty() ||
          !options.expected_gravity_session_id.empty() ||
          !options.expected_gravity_state_instance_id.empty())
        throw std::runtime_error(
            "GRAVITY_AUTHORITY_NONE_BINDING_INVALID");
    } else {
      if (!valid_sha256(options.expected_gravity_anchor_sha256) ||
          options.expected_gravity_session_id.empty() ||
          !valid_source_instance_id(
              options.expected_gravity_state_instance_id))
        throw std::runtime_error(
            "GRAVITY_AUTHORITY_STARTUP_BINDING_INVALID");
      if (empirical) {
        const auto& envelope_id = options.expected_empirical_envelope_id;
        if (envelope_id.size() != 38U ||
            envelope_id.rfind("v15-31b-empirical-", 0U) != 0U ||
            !std::all_of(
                envelope_id.begin() + 18, envelope_id.end(),
                [](char character) {
                  return (character >= '0' && character <= '9') ||
                      (character >= 'a' && character <= 'f');
                }) ||
            !valid_sha256(
                options.expected_empirical_envelope_sha256))
          throw std::runtime_error(
              "GRAVITY_EMPIRICAL_STARTUP_BINDING_INVALID");
      } else if (!options.expected_empirical_envelope_id.empty() ||
                 !options.expected_empirical_envelope_sha256.empty()) {
        throw std::runtime_error(
            "GRAVITY_OFFICIAL_EMPIRICAL_BINDING_FORBIDDEN");
      }
    }
  }
  if (options.feedback_port < 1024 || options.feedback_port > 65535)
    throw std::runtime_error("FEEDBACK_PORT_OUT_OF_RANGE");
  return options;
}

struct BusDefinition {
  const char* port;
  std::vector<const char*> locks;
  int command_port;
};

BusDefinition bus_definition(const std::string& bus) {
  if (bus == "j1") return {
      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if03-port0",
      {"/tmp/v15_30a_gui_j1.lock", "/tmp/go_m8010_ftasqa6f_channel3.lock"}, 15310};
  if (bus == "j2") return {
      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0",
      {"/tmp/v15_30a_gui_j2.lock", "/tmp/v15_23d_ft_j2_channel1.lock"}, 15312};
  return {
      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0",
      {"/tmp/v15_30a_gui_j345.lock", "/tmp/v15_23c_j3_bus.lock",
       "/tmp/v15_22b_j4_bus.lock", "/tmp/v15_22a_j5_bus.lock"}, 15313};
}

int open_command_socket(const BusDefinition& definition) {
  const int socket_fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (socket_fd < 0) throw std::runtime_error("UDP_SOCKET_FAILED");
  const int flags = ::fcntl(socket_fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    (void)::close(socket_fd);
    throw std::runtime_error("UDP_NONBLOCK_FAILED");
  }
  sockaddr_in bind_address{};
  bind_address.sin_family = AF_INET;
  bind_address.sin_port = htons(
      static_cast<std::uint16_t>(definition.command_port));
  bind_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(socket_fd, reinterpret_cast<const sockaddr*>(&bind_address),
             sizeof(bind_address)) != 0) {
    (void)::close(socket_fd);
    throw std::runtime_error("UDP_COMMAND_BIND_FAILED");
  }
  return socket_fd;
}

class ProcessLock {
 public:
  explicit ProcessLock(const char* path) {
    fd_ = ::open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd_ < 0 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0)
      throw std::runtime_error("PHYSICAL_BUS_ALREADY_OWNED");
  }
  ~ProcessLock() { if (fd_ >= 0) { (void)::flock(fd_, LOCK_UN); (void)::close(fd_); } }
 private:
  int fd_ = -1;
};

class AngleUnwrapper {
 public:
  double update(double raw) {
    if (!initialized_) { initialized_ = true; previous_ = value_ = raw; return raw; }
    double delta = raw - previous_;
    while (delta > kPi) delta -= 2.0 * kPi;
    while (delta < -kPi) delta += 2.0 * kPi;
    value_ += delta;
    previous_ = raw;
    return value_;
  }
 private:
  bool initialized_ = false;
  double previous_ = 0.0;
  double value_ = 0.0;
};

struct MotorRuntime {
  MotorRuntime(std::string motor_name, int motor_id, int logical_joint_index,
               int motor_sign, double maximum_kp, double maximum_kd)
      : name(std::move(motor_name)), id(motor_id),
        joint_index(logical_joint_index), sign(motor_sign),
        kp_limit(maximum_kp), kd_limit(maximum_kd) {}

  std::string name;
  int id;
  int joint_index;
  int sign;
  double kp_limit;
  double kd_limit;
  AngleUnwrapper unwrap;
  std::deque<double> history;
  std::deque<double> j2_startup_brake_history;
  double unwrapped = 0.0;
  double reference = 0.0;
  double last_q = 0.0;
  double last_dq = 0.0;
  double last_tau = 0.0;
  // Last torque feed-forward value that was actually serialized on the GO
  // rotor-side wire command.  This is deliberately separate from measured
  // MotorData.tau and from the reducer-side joint estimate.
  double last_tau_cmd_rotor_nm = 0.0;
  std::uint64_t last_valid_feedback_monotonic_ns = 0U;
  int temperature = 0;
  int merror = -1;
  int returned_mode = -1;
  bool reference_ready = false;
  bool persistent_reference_configured = false;
  double persistent_reference = 0.0;
  bool recovery_hint_configured = false;
  double recovery_hint = 0.0;
  bool session_reference_configured = false;
  double session_reference = 0.0;
  double session_logical_position = 0.0;
  double session_capture_raw_position = 0.0;
  double session_startup_logical_position = 0.0;
  bool supported_near_vertical_recovery = false;
  bool valid = false;
  // Transport/mode health for the most recent transaction only.  `valid`
  // remains the five-frame hard-fault latch input so a single bad frame does
  // not unload FOC; this edge bit prevents stale feedback from being
  // republished as fresh or used to mint a new collision authorization.
  bool last_frame_valid = false;
  bool fault_latched = false;
  // Transport continuity is recoverable only through the in-process
  // BRAKE-only recommissioning state machine.  Keep it distinct from drive,
  // envelope, thermal and motion faults so a power-cycle cannot accidentally
  // clear an unrelated safety latch.
  bool transport_fault_latched = false;
  bool non_transport_fault_latched = false;
  // Temperature is an actuation interlock, not evidence that the serial link
  // or motor identity was lost.  Keep it separate from `valid` and from the
  // permanent non-thermal fault latch so an over-temperature worker can stay
  // alive in BRAKE, publish live cooling feedback, and require an explicit
  // operator re-arm epoch.
  bool thermal_fault_latched = false;
  bool thermal_warning_reported = false;
  int consecutive_invalid = 0;
  bool speed_ready = false;
  double previous_scaled_position = 0.0;
  Clock::time_point previous_feedback_at{};
  double integral_encoder_velocity = std::numeric_limits<double>::quiet_NaN();
  int fast_speed_count = 0;
  int slow_speed_count = 0;
  bool velocity_degraded = false;
};

double observed_integral_encoder_velocity(
    const MotorRuntime& motor, double scaled_position, Clock::time_point feedback_at) {
  const double dt = std::chrono::duration<double>(
      feedback_at - motor.previous_feedback_at).count();
  if (!motor.speed_ready || !motor.last_frame_valid ||
      !(dt > 1e-4 && dt < 0.5) || !std::isfinite(scaled_position))
    return std::numeric_limits<double>::quiet_NaN();
  const double velocity = (scaled_position - motor.previous_scaled_position) / dt;
  // Same slow encoder observer coefficient as J2, for integral learning only.
  // The vendor dq remains untouched for PD, wire feedback and the governor.
  return std::isfinite(motor.integral_encoder_velocity)
      ? motor.integral_encoder_velocity + kJ2DerivedVelocityFilterAlpha *
          (velocity - motor.integral_encoder_velocity)
      : velocity;
}

double reference_for_j2_session(double unwrapped, const MotorRuntime& motor);
double reference_for_session_hint(double unwrapped, const MotorRuntime& motor,
                                  double hint, double maximum_offset);

struct J2GovernedReference {
  bool feasible = false;
  double alpha = 0.0;
  double q = 0.0;
  double dq = 0.0;
  std::array<double, 2> predicted_work_nm{};
  std::array<double, 2> predicted_pd_nm{};
};

bool intersect_absolute_affine_constraint(
    double base, double slope, double limit, double& lower, double& upper) {
  if (!std::isfinite(base) || !std::isfinite(slope) ||
      !std::isfinite(limit) || limit < 0.0 || lower > upper)
    return false;
  if (std::abs(slope) <= 1e-15)
    return std::abs(base) <= limit + 1e-12;
  const double first = (-limit - base) / slope;
  const double second = (limit - base) / slope;
  lower = std::max(lower, std::min(first, second));
  upper = std::min(upper, std::max(first, second));
  return lower <= upper + 1e-12;
}

J2GovernedReference govern_j2_reference(
    const std::vector<MotorRuntime>& motors,
    double measured_q, double measured_dq,
    double desired_q, double desired_dq,
    double kp, double kd, double common_feedforward_nm,
    double predicted_work_limit_nm, double predicted_pd_limit_nm) {
  J2GovernedReference result;
  if (motors.size() != 2U || !std::isfinite(measured_q) ||
      !std::isfinite(measured_dq) || !std::isfinite(desired_q) ||
      !std::isfinite(desired_dq) || !std::isfinite(kp) || kp < 0.0 ||
      !std::isfinite(kd) || kd < 0.0 ||
      !std::isfinite(common_feedforward_nm))
    return result;

  double lower = 0.0;
  double upper = 1.0;
  std::array<double, 2> pd_base{};
  std::array<double, 2> pd_slope{};
  std::array<double, 2> work_base{};
  std::array<double, 2> work_slope{};
  for (std::size_t index = 0; index < motors.size(); ++index) {
    const auto& motor = motors[index];
    pd_base[index] = kp *
        (motor.reference + motor.sign * kGear * measured_q -
         motor.unwrapped);
    pd_slope[index] = kp * motor.sign * kGear *
        (desired_q - measured_q);
    const double damping_base = kd *
        (motor.sign * kGear * measured_dq - motor.last_dq);
    const double damping_slope = kd * motor.sign * kGear *
        (desired_dq - measured_dq);
    work_base[index] = pd_base[index] + damping_base +
        motor.sign * common_feedforward_nm;
    work_slope[index] = pd_slope[index] + damping_slope;
    if (!intersect_absolute_affine_constraint(
            pd_base[index], pd_slope[index], predicted_pd_limit_nm,
            lower, upper) ||
        !intersect_absolute_affine_constraint(
            work_base[index], work_slope[index], predicted_work_limit_nm,
            lower, upper))
      return result;
  }

  result.feasible = true;
  result.alpha = std::clamp(upper, 0.0, 1.0);
  result.q = measured_q + result.alpha * (desired_q - measured_q);
  result.dq = measured_dq +
      result.alpha * (desired_dq - measured_dq);
  for (std::size_t index = 0; index < motors.size(); ++index) {
    result.predicted_pd_nm[index] =
        pd_base[index] + pd_slope[index] * result.alpha;
    result.predicted_work_nm[index] =
        work_base[index] + work_slope[index] * result.alpha;
  }
  return result;
}

struct SingleMotorGovernedReference {
  bool feasible = false;
  double alpha = 0.0;
  double q = 0.0;
  double dq = 0.0;
  double predicted_work_nm = 0.0;
  double predicted_pd_nm = 0.0;
};

SingleMotorGovernedReference govern_single_motor_reference(
    const MotorRuntime& motor, double measured_q, double measured_dq,
    double desired_q, double desired_dq, double kp, double kd,
    double logical_feedforward_nm, double predicted_work_limit_nm,
    double predicted_pd_limit_nm) {
  SingleMotorGovernedReference result;
  if (!std::isfinite(measured_q) || !std::isfinite(measured_dq) ||
      !std::isfinite(desired_q) || !std::isfinite(desired_dq) ||
      !std::isfinite(kp) || kp < 0.0 || !std::isfinite(kd) || kd < 0.0 ||
      !std::isfinite(logical_feedforward_nm))
    return result;
  const double pd_base = kp *
      (motor.reference + motor.sign * kGear * measured_q - motor.unwrapped);
  const double pd_slope = kp * motor.sign * kGear *
      (desired_q - measured_q);
  const double damping_base = kd *
      (motor.sign * kGear * measured_dq - motor.last_dq);
  const double damping_slope = kd * motor.sign * kGear *
      (desired_dq - measured_dq);
  const double work_base = pd_base + damping_base +
      motor.sign * logical_feedforward_nm;
  const double work_slope = pd_slope + damping_slope;
  double lower = 0.0;
  double upper = 1.0;
  if (!intersect_absolute_affine_constraint(
          pd_base, pd_slope, predicted_pd_limit_nm, lower, upper) ||
      !intersect_absolute_affine_constraint(
          work_base, work_slope, predicted_work_limit_nm, lower, upper))
    return result;
  result.feasible = true;
  result.alpha = std::clamp(upper, 0.0, 1.0);
  result.q = measured_q + result.alpha * (desired_q - measured_q);
  result.dq = measured_dq + result.alpha * (desired_dq - measured_dq);
  result.predicted_pd_nm = pd_base + pd_slope * result.alpha;
  result.predicted_work_nm = work_base + work_slope * result.alpha;
  return result;
}

int invalid_feedback_limit_for_bus(const std::string& bus) {
  // A single malformed/late frame must not withdraw torque from the shared
  // J3/J4/J5 load-bearing domain.  Explicit drive merror/temperature faults
  // remain immediate below; framing/returned-mode failures must persist for
  // five consecutive control cycles before becoming a hard domain fault.
  (void)bus;
  return 5;
}

struct J2SyncFaultFilter {
  bool observation_valid = false;
  bool warning = false;
  bool fault = false;
  bool release_observed = false;
  bool recovery_ready = false;
  bool rearm_pending_next_cycle = false;
  int recovery_frames = 0;
  double trip_error_rad = 0.0;
  std::uint64_t trip_activation_epoch = 0;
  std::uint64_t minimum_rearm_epoch = 0;
};

struct ThermalInterlockState {
  bool fault_latched = false;
  bool release_observed = false;
  bool cooldown_ready = false;
  bool rearm_pending_next_cycle = false;
  int cooldown_frames = 0;
  Clock::time_point cooldown_started_at{};
  std::string trip_reason;
  std::uint64_t trip_activation_epoch = 0;
  std::uint64_t minimum_rearm_epoch = 0;
};

struct NoProgressWatchdogState {
  bool fault_latched = false;
  bool release_observed = false;
  bool rearm_pending_next_cycle = false;
  int qualifying_frames = 0;
  Clock::time_point window_started_at{};
  double window_baseline_error_rad = 0.0;
  double trip_position_error_rad = 0.0;
  std::string trip_reason;
  std::uint64_t trip_activation_epoch = 0;
  std::uint64_t minimum_rearm_epoch = 0;
};

std::uint64_t saturating_next_activation_epoch(std::uint64_t epoch) {
  return epoch == std::numeric_limits<std::uint64_t>::max()
      ? epoch : epoch + 1U;
}

void reset_thermal_cooldown_evidence(ThermalInterlockState& state) {
  state.cooldown_ready = false;
  state.cooldown_frames = 0;
  state.cooldown_started_at = Clock::time_point{};
}

bool latch_thermal_interlock(
    ThermalInterlockState& state, const char* trip_reason,
    std::uint64_t active_epoch, std::uint64_t current_minimum_epoch,
    std::uint64_t highest_rejected_active_epoch) {
  if (trip_reason == nullptr || *trip_reason == '\0') return false;
  const bool newly_latched = !state.fault_latched;
  state.fault_latched = true;
  state.release_observed = false;
  state.rearm_pending_next_cycle = false;
  reset_thermal_cooldown_evidence(state);
  if (newly_latched) state.trip_reason = trip_reason;
  state.trip_activation_epoch = std::max(
      state.trip_activation_epoch, active_epoch);
  state.minimum_rearm_epoch = std::max({
      state.minimum_rearm_epoch,
      current_minimum_epoch,
      saturating_next_activation_epoch(active_epoch),
      saturating_next_activation_epoch(highest_rejected_active_epoch)});
  return newly_latched;
}

bool observe_raw_temperature_thermal_trip(
    ThermalInterlockState& state, int raw_temperature_c,
    std::uint64_t active_epoch, std::uint64_t current_minimum_epoch,
    std::uint64_t highest_rejected_active_epoch) {
  if (raw_temperature_c < g_thermal_policy.thermal_stop_c) return false;
  return latch_thermal_interlock(
      state, "RAW_TEMPERATURE_LIMIT", active_epoch, current_minimum_epoch,
      highest_rejected_active_epoch);
}

bool observe_thermal_cooldown_frame(
    ThermalInterlockState& state, bool all_domain_motors_valid_brake_below_55,
    Clock::time_point observed_at) {
  if (!state.fault_latched) {
    reset_thermal_cooldown_evidence(state);
    return false;
  }
  if (!all_domain_motors_valid_brake_below_55) {
    reset_thermal_cooldown_evidence(state);
    state.release_observed = false;
    state.rearm_pending_next_cycle = false;
    return false;
  }
  if (state.cooldown_frames == 0)
    state.cooldown_started_at = observed_at;
  if (state.cooldown_frames < std::numeric_limits<int>::max())
    ++state.cooldown_frames;
  const double elapsed_seconds = std::chrono::duration<double>(
      observed_at - state.cooldown_started_at).count();
  const bool was_ready = state.cooldown_ready;
  state.cooldown_ready =
      state.cooldown_frames >= kThermalCooldownConsecutiveFrames &&
      elapsed_seconds >= g_thermal_policy.cooldown_seconds;
  return state.cooldown_ready && !was_ready;
}

bool observe_explicit_thermal_release(
    ThermalInterlockState& state, bool explicit_release_packet_received) {
  if (!state.fault_latched || !state.cooldown_ready ||
      !explicit_release_packet_received)
    return false;
  const bool newly_observed = !state.release_observed;
  state.release_observed = true;
  return newly_observed;
}

bool request_thermal_rearm_for_next_cycle(
    ThermalInterlockState& state, bool command_lease_fresh,
    bool active_owned_joint_requested, std::uint64_t activation_epoch,
    std::uint64_t highest_rejected_active_epoch) {
  const bool acceptable = state.fault_latched && state.cooldown_ready &&
      state.release_observed && command_lease_fresh &&
      active_owned_joint_requested &&
      activation_epoch > state.trip_activation_epoch &&
      activation_epoch >= state.minimum_rearm_epoch &&
      activation_epoch > highest_rejected_active_epoch;
  if (!acceptable) return false;
  state.rearm_pending_next_cycle = true;
  return true;
}

bool apply_pending_thermal_rearm_at_cycle_start(
    ThermalInterlockState& state) {
  if (!state.fault_latched || !state.rearm_pending_next_cycle) return false;
  state.fault_latched = false;
  state.release_observed = false;
  state.rearm_pending_next_cycle = false;
  state.trip_reason.clear();
  reset_thermal_cooldown_evidence(state);
  return true;
}

void reset_no_progress_observation(NoProgressWatchdogState& state) {
  state.qualifying_frames = 0;
  state.window_started_at = Clock::time_point{};
  state.window_baseline_error_rad = 0.0;
}

bool latch_position_safety_watchdog(
    NoProgressWatchdogState& state, double position_error_rad,
    const char* trip_reason, std::uint64_t active_epoch,
    std::uint64_t current_minimum_epoch,
    std::uint64_t highest_rejected_active_epoch) {
  if (state.fault_latched || !std::isfinite(position_error_rad) ||
      position_error_rad < 0.0 || trip_reason == nullptr ||
      *trip_reason == '\0')
    return false;
  state.fault_latched = true;
  state.release_observed = false;
  state.rearm_pending_next_cycle = false;
  state.trip_position_error_rad = position_error_rad;
  state.trip_reason = trip_reason;
  state.trip_activation_epoch = std::max(
      state.trip_activation_epoch, active_epoch);
  state.minimum_rearm_epoch = std::max({
      state.minimum_rearm_epoch,
      current_minimum_epoch,
      saturating_next_activation_epoch(active_epoch),
      saturating_next_activation_epoch(highest_rejected_active_epoch)});
  return true;
}

bool observe_no_progress_watchdog(
    NoProgressWatchdogState& state, bool position_foc_observation_valid,
    double position_error_rad, bool software_saturation_observed,
    Clock::time_point observed_at, std::uint64_t active_epoch,
    std::uint64_t current_minimum_epoch,
    std::uint64_t highest_rejected_active_epoch) {
  if (state.fault_latched) return false;
  const bool qualifying = position_foc_observation_valid &&
      software_saturation_observed && std::isfinite(position_error_rad) &&
      position_error_rad >= kNoProgressMinimumPositionError;
  if (!qualifying) {
    reset_no_progress_observation(state);
    return false;
  }
  if (state.qualifying_frames == 0 ||
      observed_at < state.window_started_at) {
    state.qualifying_frames = 1;
    state.window_started_at = observed_at;
    state.window_baseline_error_rad = position_error_rad;
    return false;
  }
  if (state.window_baseline_error_rad - position_error_rad >=
      kNoProgressMinimumImprovement) {
    state.qualifying_frames = 1;
    state.window_started_at = observed_at;
    state.window_baseline_error_rad = position_error_rad;
    return false;
  }
  if (state.qualifying_frames < std::numeric_limits<int>::max())
    ++state.qualifying_frames;
  const double elapsed_seconds = std::chrono::duration<double>(
      observed_at - state.window_started_at).count();
  if (state.qualifying_frames < kNoProgressMinimumQualifyingFrames ||
      elapsed_seconds < kNoProgressWindowSeconds)
    return false;

  return latch_position_safety_watchdog(
      state, position_error_rad, "LOAD_LIMIT_NO_PROGRESS", active_epoch,
      current_minimum_epoch, highest_rejected_active_epoch);
}

bool observe_explicit_no_progress_release(
    NoProgressWatchdogState& state,
    bool explicit_release_packet_received) {
  if (!state.fault_latched || !explicit_release_packet_received) return false;
  const bool newly_observed = !state.release_observed;
  state.release_observed = true;
  return newly_observed;
}

bool request_no_progress_rearm_for_next_cycle(
    NoProgressWatchdogState& state, bool command_lease_fresh,
    bool active_owned_joint_requested,
    bool all_domain_motors_valid_brake,
    std::uint64_t activation_epoch,
    std::uint64_t highest_rejected_active_epoch) {
  const bool acceptable = state.fault_latched && state.release_observed &&
      command_lease_fresh && active_owned_joint_requested &&
      all_domain_motors_valid_brake &&
      activation_epoch > state.trip_activation_epoch &&
      activation_epoch >= state.minimum_rearm_epoch &&
      activation_epoch > highest_rejected_active_epoch;
  if (!acceptable) return false;
  state.rearm_pending_next_cycle = true;
  return true;
}

bool apply_pending_no_progress_rearm_at_cycle_start(
    NoProgressWatchdogState& state) {
  if (!state.fault_latched || !state.rearm_pending_next_cycle) return false;
  state.fault_latched = false;
  state.release_observed = false;
  state.rearm_pending_next_cycle = false;
  state.trip_position_error_rad = 0.0;
  state.trip_reason.clear();
  reset_no_progress_observation(state);
  return true;
}

void no_progress_watchdog_self_test() {
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  const double blocked_error = 4.0 * kPi / 180.0;
  NoProgressWatchdogState state;
  if (observe_no_progress_watchdog(
          state, true, blocked_error, false, start, 4U, 0U, 4U) ||
      state.qualifying_frames != 0)
    throw std::runtime_error("NO_PROGRESS_SATURATION_GATE_SELF_TEST_FAILED");
  if (observe_no_progress_watchdog(
          state, true, blocked_error, true, start, 4U, 0U, 4U))
    throw std::runtime_error("NO_PROGRESS_FIRST_FRAME_SELF_TEST_FAILED");
  for (int frame = 1; frame < kNoProgressMinimumQualifyingFrames; ++frame) {
    if (observe_no_progress_watchdog(
            state, true, blocked_error, true,
            start + std::chrono::milliseconds(frame * 10),
            4U, 0U, 4U))
      throw std::runtime_error("NO_PROGRESS_SHORT_WINDOW_SELF_TEST_FAILED");
  }
  if (!observe_no_progress_watchdog(
          state, true, blocked_error, true,
          start + std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(kNoProgressWindowSeconds)),
          4U, 0U, 4U) ||
      !state.fault_latched || state.minimum_rearm_epoch != 5U)
    throw std::runtime_error("NO_PROGRESS_TRIP_SELF_TEST_FAILED");
  if (request_no_progress_rearm_for_next_cycle(
          state, true, true, true, 5U, 5U))
    throw std::runtime_error("NO_PROGRESS_RELEASE_REQUIRED_SELF_TEST_FAILED");
  if (!observe_explicit_no_progress_release(state, true) ||
      request_no_progress_rearm_for_next_cycle(
          state, true, true, true, 4U, 4U) ||
      request_no_progress_rearm_for_next_cycle(
          state, true, true, false, 5U, 4U) ||
      !request_no_progress_rearm_for_next_cycle(
          state, true, true, true, 5U, 4U) ||
      !state.fault_latched || !state.rearm_pending_next_cycle)
    throw std::runtime_error("NO_PROGRESS_HIGHER_EPOCH_SELF_TEST_FAILED");
  if (!apply_pending_no_progress_rearm_at_cycle_start(state) ||
      state.fault_latched || state.rearm_pending_next_cycle)
    throw std::runtime_error("NO_PROGRESS_NEXT_CYCLE_REARM_SELF_TEST_FAILED");

  NoProgressWatchdogState improving;
  (void)observe_no_progress_watchdog(
      improving, true, blocked_error, true, start, 8U, 0U, 0U);
  const double improved_error = blocked_error -
      kNoProgressMinimumImprovement - 0.01 * kPi / 180.0;
  (void)observe_no_progress_watchdog(
      improving, true, improved_error, true,
      start + std::chrono::seconds(1), 8U, 0U, 0U);
  if (improving.qualifying_frames != 1 ||
      improving.window_started_at != start + std::chrono::seconds(1))
    throw std::runtime_error("NO_PROGRESS_IMPROVEMENT_RESET_SELF_TEST_FAILED");
  (void)observe_no_progress_watchdog(
      improving, true, improved_error, false,
      start + std::chrono::seconds(2), 8U, 0U, 0U);
  if (improving.qualifying_frames != 0)
    throw std::runtime_error("NO_PROGRESS_GAP_RESET_SELF_TEST_FAILED");

  NoProgressWatchdogState arrival_timeout;
  if (!latch_position_safety_watchdog(
          arrival_timeout, blocked_error, "POSITION_ARRIVAL_TIMEOUT", 12U,
          9U, 12U) ||
      !arrival_timeout.fault_latched ||
      arrival_timeout.trip_reason != "POSITION_ARRIVAL_TIMEOUT" ||
      arrival_timeout.trip_position_error_rad != blocked_error ||
      arrival_timeout.trip_activation_epoch != 12U ||
      arrival_timeout.minimum_rearm_epoch != 13U ||
      latch_position_safety_watchdog(
          arrival_timeout, blocked_error, "POSITION_ARRIVAL_TIMEOUT", 13U,
          13U, 13U))
    throw std::runtime_error("POSITION_ARRIVAL_TIMEOUT_LATCH_SELF_TEST_FAILED");

  NoProgressWatchdogState exact_governor_abort;
  if (!latch_position_safety_watchdog(
          exact_governor_abort, blocked_error,
          "EXACT_TRAJECTORY_LOAD_GOVERNOR_ABORT", 14U, 0U, 14U) ||
      exact_governor_abort.trip_reason !=
          "EXACT_TRAJECTORY_LOAD_GOVERNOR_ABORT" ||
      exact_governor_abort.minimum_rearm_epoch != 15U)
    throw std::runtime_error(
        "EXACT_TRAJECTORY_LOAD_GOVERNOR_ABORT_SELF_TEST_FAILED");

  NoProgressWatchdogState exhausted;
  exhausted.fault_latched = true;
  exhausted.release_observed = true;
  exhausted.trip_activation_epoch =
      std::numeric_limits<std::uint64_t>::max();
  exhausted.minimum_rearm_epoch = exhausted.trip_activation_epoch;
  if (request_no_progress_rearm_for_next_cycle(
          exhausted, true, true, true,
          exhausted.trip_activation_epoch,
          exhausted.trip_activation_epoch))
    throw std::runtime_error("NO_PROGRESS_EPOCH_EXHAUSTION_SELF_TEST_FAILED");
}

void thermal_interlock_self_test() {
  const auto start = Clock::time_point{} + std::chrono::seconds(1);
  ThermalInterlockState state;
  if (observe_raw_temperature_thermal_trip(state, 59, 7U, 0U, 0U) ||
      state.fault_latched)
    throw std::runtime_error("THERMAL_59_TRIP_SELF_TEST_FAILED");
  if (!observe_raw_temperature_thermal_trip(state, 60, 7U, 0U, 7U) ||
      !state.fault_latched || state.trip_activation_epoch != 7U ||
      state.minimum_rearm_epoch != 8U)
    throw std::runtime_error("THERMAL_RAW_60_TRIP_SELF_TEST_FAILED");
  if (observe_raw_temperature_thermal_trip(state, 59, 7U, 0U, 7U) ||
      !state.fault_latched)
    throw std::runtime_error("THERMAL_59_AUTO_RECOVERY_SELF_TEST_FAILED");
  if (observe_explicit_thermal_release(state, true))
    throw std::runtime_error("THERMAL_EARLY_RELEASE_SELF_TEST_FAILED");
  for (int frame = 0; frame < kThermalCooldownConsecutiveFrames; ++frame) {
    (void)observe_thermal_cooldown_frame(
        state, true, start + std::chrono::milliseconds(frame * 10));
  }
  if (state.cooldown_ready ||
      state.cooldown_frames != kThermalCooldownConsecutiveFrames)
    throw std::runtime_error("THERMAL_FIVE_SECOND_COOLDOWN_SELF_TEST_FAILED");
  if (!observe_thermal_cooldown_frame(
          state, true,
          start + std::chrono::duration_cast<Clock::duration>(
              std::chrono::duration<double>(
                  g_thermal_policy.cooldown_seconds))) ||
      !state.cooldown_ready)
    throw std::runtime_error("THERMAL_30_SECOND_COOLDOWN_SELF_TEST_FAILED");
  if (request_thermal_rearm_for_next_cycle(state, true, true, 8U, 8U) ||
      state.rearm_pending_next_cycle)
    throw std::runtime_error("THERMAL_RELEASE_REQUIRED_SELF_TEST_FAILED");
  if (!observe_explicit_thermal_release(state, true) ||
      request_thermal_rearm_for_next_cycle(state, true, true, 7U, 7U) ||
      !request_thermal_rearm_for_next_cycle(state, true, true, 8U, 7U) ||
      !state.fault_latched || !state.rearm_pending_next_cycle)
    throw std::runtime_error("THERMAL_HIGHER_EPOCH_SELF_TEST_FAILED");
  if (!apply_pending_thermal_rearm_at_cycle_start(state) ||
      state.fault_latched || state.rearm_pending_next_cycle)
    throw std::runtime_error("THERMAL_NEXT_CYCLE_REARM_SELF_TEST_FAILED");

  if (!observe_raw_temperature_thermal_trip(state, 61, 9U, 0U, 9U))
    throw std::runtime_error("THERMAL_RETRIP_SELF_TEST_FAILED");
  (void)observe_thermal_cooldown_frame(state, true, start);
  (void)observe_thermal_cooldown_frame(
      state, false, start + std::chrono::milliseconds(10));
  if (state.cooldown_frames != 0 || state.cooldown_ready ||
      state.cooldown_started_at != Clock::time_point{})
    throw std::runtime_error("THERMAL_COOLDOWN_RESET_SELF_TEST_FAILED");

  ThermalInterlockState exhausted;
  const auto maximum_epoch = std::numeric_limits<std::uint64_t>::max();
  (void)observe_raw_temperature_thermal_trip(
      exhausted, 60, maximum_epoch, 0U, maximum_epoch);
  exhausted.cooldown_ready = true;
  exhausted.release_observed = true;
  if (exhausted.minimum_rearm_epoch != maximum_epoch ||
      request_thermal_rearm_for_next_cycle(
          exhausted, true, true, maximum_epoch, maximum_epoch))
    throw std::runtime_error("THERMAL_EPOCH_EXHAUSTION_SELF_TEST_FAILED");

  ThermalInterlockState exact_trajectory_abort;
  if (!latch_thermal_interlock(
          exact_trajectory_abort, "EXACT_TRAJECTORY_DERATING_ABORT",
          12U, 0U, 12U) || !exact_trajectory_abort.fault_latched ||
      exact_trajectory_abort.trip_reason !=
          "EXACT_TRAJECTORY_DERATING_ABORT" ||
      exact_trajectory_abort.minimum_rearm_epoch != 13U)
    throw std::runtime_error(
        "THERMAL_EXACT_TRAJECTORY_ABORT_SELF_TEST_FAILED");
}

void thermal_derating_self_test() {
  if (thermal_derating_factor_for_raw_temperature(54) != 1.0 ||
      thermal_derating_factor_for_raw_temperature(55) != 1.0 ||
      std::abs(thermal_derating_factor_for_raw_temperature(56) - 0.8) >
          1e-12 ||
      std::abs(thermal_derating_factor_for_raw_temperature(58) - 0.4) >
          1e-12 ||
      std::abs(thermal_derating_factor_for_raw_temperature(59) - 0.2) >
          1e-12 ||
      thermal_derating_factor_for_raw_temperature(60) != 0.0 ||
      thermal_derating_factor_for_raw_temperature(-1) != 0.0)
    throw std::runtime_error("THERMAL_DERATING_SELF_TEST_FAILED");
}

bool observe_position_arrival_dwell(
    bool endpoint_phase, bool healthy_within_tolerance,
    Clock::time_point observed_at, int& qualifying_frames,
    Clock::time_point& window_started_at, bool& arrived) {
  if (!endpoint_phase || !healthy_within_tolerance) {
    qualifying_frames = 0;
    window_started_at = Clock::time_point{};
    arrived = false;
    return false;
  }
  if (qualifying_frames == 0) window_started_at = observed_at;
  if (qualifying_frames < std::numeric_limits<int>::max())
    ++qualifying_frames;
  const double dwell_seconds = std::chrono::duration<double>(
      observed_at - window_started_at).count();
  arrived = qualifying_frames >= kArrivalDwellMinimumFrames &&
      dwell_seconds >= kArrivalDwellSeconds;
  return arrived;
}

void position_arrival_dwell_self_test() {
  int frames = 0;
  Clock::time_point started{};
  bool arrived = false;
  const auto origin = Clock::now();
  for (int index = 0; index < 50; ++index) {
    if (observe_position_arrival_dwell(
            true, true, origin + std::chrono::milliseconds(index * 10),
            frames, started, arrived))
      throw std::runtime_error("POSITION_ARRIVAL_EARLY_DWELL_SELF_TEST_FAILED");
  }
  if (!observe_position_arrival_dwell(
          true, true, origin + std::chrono::milliseconds(500),
          frames, started, arrived) || !arrived)
    throw std::runtime_error("POSITION_ARRIVAL_DWELL_SELF_TEST_FAILED");
  if (observe_position_arrival_dwell(
          true, false, origin + std::chrono::milliseconds(510),
          frames, started, arrived) || arrived || frames != 0)
    throw std::runtime_error("POSITION_ARRIVAL_DRIFT_RESET_SELF_TEST_FAILED");
  if (observe_position_arrival_dwell(
          false, true, origin + std::chrono::milliseconds(520),
          frames, started, arrived) || arrived || frames != 0)
    throw std::runtime_error("POSITION_ARRIVAL_ENDPOINT_GATE_SELF_TEST_FAILED");
}

bool all_domain_motors_thermal_cooldown_qualified(
    const std::vector<MotorRuntime>& motors) {
  return !motors.empty() &&
      std::all_of(motors.begin(), motors.end(), [](const MotorRuntime& motor) {
        return motor.last_frame_valid && motor.valid && !motor.fault_latched &&
            motor.merror == 0 && motor.returned_mode == kBrakeMode &&
            motor.temperature >= 0 &&
            motor.temperature < g_thermal_policy.rearm_below_c;
      });
}

bool all_domain_motors_valid_brake_for_rearm(
    const std::vector<MotorRuntime>& motors) {
  return !motors.empty() &&
      std::all_of(motors.begin(), motors.end(), [](const MotorRuntime& motor) {
        return motor.last_frame_valid && motor.valid && !motor.fault_latched &&
            motor.merror == 0 && motor.returned_mode == kBrakeMode &&
            motor.temperature >= 0 &&
            motor.temperature < g_thermal_policy.thermal_stop_c;
      });
}

void publish_thermal_latch_to_motors(
    std::vector<MotorRuntime>& motors, bool fault_latched) {
  for (auto& motor : motors)
    motor.thermal_fault_latched = fault_latched;
}

struct BoundedHoldIntegralState {
  double accumulator_nm = 0.0;
  int dwell_frames = 0;
};

void reset_bounded_hold_integral(BoundedHoldIntegralState& state) {
  state.accumulator_nm = 0.0;
  state.dwell_frames = 0;
}

double frozen_hold_integral_wire(
    const BoundedHoldIntegralState& state, double hard_limit_nm) {
  return std::clamp(std::round(state.accumulator_nm * 256.0) / 256.0,
                    -hard_limit_nm, hard_limit_nm);
}

double update_bounded_hold_integral(
    BoundedHoldIntegralState& state, bool active,
    bool learning_permitted, double error_rad, double velocity_rad_s,
    double hard_limit_nm, double ki_per_rotor_rad_s,
    double rate_limit_nm_s, double enter_error_rad,
    double enter_velocity_rad_s, double deadband_rad, int dwell_required) {
  if (!active) {
    reset_bounded_hold_integral(state);
    return 0.0;
  }
  if (!std::isfinite(state.accumulator_nm)) state.accumulator_nm = 0.0;
  const bool learning_gate = learning_permitted &&
      std::isfinite(error_rad) && std::isfinite(velocity_rad_s) &&
      std::abs(error_rad) <= enter_error_rad &&
      std::abs(velocity_rad_s) <= enter_velocity_rad_s;
  state.dwell_frames = learning_gate
      ? std::min(state.dwell_frames + 1, std::max(1, dwell_required)) : 0;
  // No automatic unwind is allowed.  A lease interruption, a temporary
  // observation gap, or an external displacement must preserve the learned
  // load-bearing bias.  Only an inactive (BRAKE/DRAG/hard-fault) domain resets
  // it; while active, a genuine opposite target error reverses it gradually.
  if (learning_gate && state.dwell_frames >= std::max(1, dwell_required) &&
      std::abs(error_rad) > deadband_rad) {
    const double requested_rate = std::clamp(
        ki_per_rotor_rad_s * kGear * error_rad,
        -rate_limit_nm_s, rate_limit_nm_s);
    const double next_accumulator = std::clamp(
        state.accumulator_nm + requested_rate * kPeriod,
        -hard_limit_nm, hard_limit_nm);
    state.accumulator_nm =
        state.accumulator_nm * next_accumulator < 0.0 &&
            state.accumulator_nm * requested_rate < 0.0
        ? 0.0 : next_accumulator;
  }
  return std::clamp(
      std::round(state.accumulator_nm * 256.0) / 256.0,
      -hard_limit_nm, hard_limit_nm);
}

double frozen_required_rotor_gravity_nm(std::size_t joint) {
  if (joint >= kFrozenMaximumGravityJointNm.size())
    return std::numeric_limits<double>::infinity();
  const double motor_share = joint == 1U ? 2.0 : 1.0;
  return kFrozenMaximumGravityJointNm[joint] / (motor_share * kGear);
}

void reset_j2_sync_recovery_evidence(J2SyncFaultFilter& state) {
  state.release_observed = false;
  state.recovery_ready = false;
  state.rearm_pending_next_cycle = false;
  state.recovery_frames = 0;
}

bool latch_j2_sync_interlock(
    J2SyncFaultFilter& state, double error_rad,
    std::uint64_t active_epoch, std::uint64_t current_minimum_epoch,
    std::uint64_t highest_rejected_active_epoch) {
  const bool newly_latched = !state.fault;
  state.fault = true;
  reset_j2_sync_recovery_evidence(state);
  if (newly_latched)
    state.trip_error_rad = std::isfinite(error_rad) ? error_rad : 0.0;
  state.trip_activation_epoch = std::max(
      state.trip_activation_epoch, active_epoch);
  state.minimum_rearm_epoch = std::max({
      state.minimum_rearm_epoch,
      current_minimum_epoch,
      saturating_next_activation_epoch(active_epoch),
      saturating_next_activation_epoch(highest_rejected_active_epoch)});
  return newly_latched;
}

bool observe_j2_sync_error(
    J2SyncFaultFilter& state, double error_rad,
    std::uint64_t active_epoch, std::uint64_t current_minimum_epoch,
    std::uint64_t highest_rejected_active_epoch) {
  state.observation_valid = std::isfinite(error_rad);
  if (!state.observation_valid) {
    state.warning = true;
    (void)latch_j2_sync_interlock(
        state, error_rad, active_epoch, current_minimum_epoch,
        highest_rejected_active_epoch);
    return true;
  }
  const double magnitude = std::abs(error_rad);
  state.warning = magnitude > kJ2SyncWarningLimit;
  if (magnitude > kJ2SyncLimit) {
    (void)latch_j2_sync_interlock(
        state, error_rad, active_epoch, current_minimum_epoch,
        highest_rejected_active_epoch);
    return true;
  }
  if (!state.fault) {
    reset_j2_sync_recovery_evidence(state);
    return false;
  }
  if (magnitude <= kJ2SyncRecoveryLimit) {
    if (state.recovery_frames < std::numeric_limits<int>::max())
      ++state.recovery_frames;
    state.recovery_ready =
        state.recovery_frames >= kJ2SyncRecoveryConsecutiveFrames;
  } else {
    reset_j2_sync_recovery_evidence(state);
  }
  return true;
}

bool observe_explicit_j2_sync_release(
    J2SyncFaultFilter& state, bool explicit_release_packet_received) {
  if (!state.fault || !state.recovery_ready ||
      !explicit_release_packet_received)
    return false;
  const bool newly_observed = !state.release_observed;
  state.release_observed = true;
  return newly_observed;
}

bool request_j2_sync_rearm_for_next_cycle(
    J2SyncFaultFilter& state, bool command_lease_fresh,
    bool active_owned_joint_requested,
    bool all_domain_motors_valid_brake,
    std::uint64_t activation_epoch,
    std::uint64_t highest_rejected_active_epoch) {
  const bool acceptable = state.fault && state.observation_valid &&
      !state.warning && state.recovery_ready && state.release_observed &&
      command_lease_fresh && active_owned_joint_requested &&
      all_domain_motors_valid_brake &&
      activation_epoch > state.trip_activation_epoch &&
      activation_epoch >= state.minimum_rearm_epoch &&
      activation_epoch > highest_rejected_active_epoch;
  if (!acceptable) return false;
  state.rearm_pending_next_cycle = true;
  return true;
}

bool apply_pending_j2_sync_rearm_at_cycle_start(
    J2SyncFaultFilter& state) {
  if (!state.fault || !state.rearm_pending_next_cycle) return false;
  state.fault = false;
  state.observation_valid = false;
  state.warning = false;
  state.trip_error_rad = 0.0;
  reset_j2_sync_recovery_evidence(state);
  return true;
}

bool observe_feedback_frame_validity(
    MotorRuntime& motor, bool frame_valid, int invalid_limit) {
  motor.last_frame_valid = frame_valid;
  if (frame_valid) {
    motor.valid = true;
    motor.consecutive_invalid = 0;
    return false;
  }
  motor.integral_encoder_velocity = std::numeric_limits<double>::quiet_NaN();
  motor.consecutive_invalid = std::min(
      motor.consecutive_invalid + 1, std::max(1, invalid_limit));
  if (motor.consecutive_invalid < std::max(1, invalid_limit)) return false;
  motor.valid = false;
  motor.transport_fault_latched = true;
  motor.fault_latched = true;
  return true;
}

bool observe_j2_sync_unavailable(J2SyncFaultFilter& state) {
  // Missing/asynchronous feedback proves neither warning clearance nor safe
  // recovery.  Retain a hard latch and invalidate all recovery/rearm evidence.
  state.observation_valid = false;
  state.warning = false;
  reset_j2_sync_recovery_evidence(state);
  return state.fault;
}

std::vector<MotorRuntime> make_motors(const std::string& bus) {
  if (bus == "j1") return {{"J1", 0, 0, +1, 1.50, 0.15}};
  if (bus == "j2") return {
      {"J2A", 0, 1, -1, 3.00, 0.30}, {"J2B", 1, 1, +1, 3.00, 0.30}};
  return {
      {"J3", 3, 2, +1, 2.00, 0.15},
      {"J4", 4, 3, -1, 2.00, 0.15},
      {"J5", 5, 4, +1, 1.50, 0.12}};
}

std::string load_persistent_zero(const std::string& path,
                                 std::vector<MotorRuntime>& motors) {
  if (path.empty()) return {};
  const SecureFileBytes source = read_secure_owned_file(
      path, 1048576U, "PERSISTENT_ZERO_OPEN_FAILED",
      "PERSISTENT_ZERO_FILE_UNSAFE", "PERSISTENT_ZERO_READ_FAILED");
  const nlohmann::json document =
      parse_json_bytes(source.data, "PERSISTENT_ZERO_JSON_INVALID");
  if (document.at("schema") != "go-m8010-persistent-software-zero/1.0" ||
      document.at("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1")
    throw std::runtime_error("PERSISTENT_ZERO_SCHEMA_MISMATCH");
  const auto& writes = document.at("writes");
  if (writes.at("motor_internal_zero_modified").get<bool>() ||
      writes.at("rid_written").get<bool>() ||
      writes.at("flash_or_eeprom_written").get<bool>())
    throw std::runtime_error("PERSISTENT_ZERO_FORBIDDEN_WRITE_RECORD");
  const auto& values = document.at("motors");
  for (auto& motor : motors) {
    const double reference = values.at(motor.name).at("raw_position_rad").get<double>();
    if (!std::isfinite(reference))
      throw std::runtime_error("PERSISTENT_ZERO_NONFINITE");
    motor.persistent_reference = reference;
    motor.persistent_reference_configured = true;
  }
  return source.sha256;
}

std::string load_recovery_hints(const std::string& path,
                                std::vector<MotorRuntime>& motors) {
  if (path.empty()) return {};
  const SecureFileBytes source = read_secure_owned_file(
      path, 1048576U, "RECOVERY_HINT_OPEN_FAILED",
      "RECOVERY_HINT_FILE_UNSAFE", "RECOVERY_HINT_READ_FAILED");
  const nlohmann::json document =
      parse_json_bytes(source.data, "RECOVERY_HINT_JSON_INVALID");
  if (document.at("schema") != "go-m8010-recovery-branch-hints/1.0" ||
      document.at("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1")
    throw std::runtime_error("RECOVERY_HINT_SCHEMA_MISMATCH");
  const auto& hints = document.at("motors");
  for (auto& motor : motors) {
    const double hint = hints.at(motor.name).at("logical_position_rad").get<double>();
    if (!within_mechanical_feedback_envelope(motor.joint_index, hint))
      throw std::runtime_error("RECOVERY_HINT_OUTSIDE_MECHANICAL_ENVELOPE");
    motor.recovery_hint = hint;
    motor.recovery_hint_configured = true;
  }
  return source.sha256;
}

bool validate_preserved_session_geometry(const nlohmann::json& document) {
  if (!document.contains("preserved_reference")) return false;
  const auto& proof = document.at("preserved_reference");
  if (proof.at("schema") != "go-m8010-preserved-session-reference/1.0")
    throw std::runtime_error("PRESERVED_REFERENCE_SCHEMA_MISMATCH");
  if (proof.contains("supported_near_vertical_recovery")) {
    const nlohmann::json expected = {
        {"schema", "go-m8010-supported-near-vertical-recovery/1.0"},
        {"purpose", "SUPPORTED_RETURN_TO_HASH_BOUND_INITIAL_POSE"},
        {"software_maximum_offset_deg", 5.0},
        {"user_declared_near_original_pose", true}, {"support_reliable", true},
        {"self_return_authorized", true},
        {"externally_measured_five_degree_accuracy_claimed", false},
        {"operator_evidence_id", document.at("operator_confirmation").at("evidence_id")}};
    if (proof.at("supported_near_vertical_recovery") != expected)
      throw std::runtime_error("SUPPORTED_NEAR_VERTICAL_RECOVERY_DECLARATION_MISMATCH");
  }
  const auto source_file = read_secure_owned_file(
      proof.at("source_path").get<std::string>(), 4194304U,
      "PRESERVED_REFERENCE_OPEN_FAILED", "PRESERVED_REFERENCE_FILE_UNSAFE",
      "PRESERVED_REFERENCE_READ_FAILED");
  if (source_file.sha256 != proof.at("source_sha256").get<std::string>())
    throw std::runtime_error("PRESERVED_REFERENCE_SHA256_MISMATCH");
  const auto source = parse_json_bytes(source_file.data, "PRESERVED_REFERENCE_JSON_INVALID");
  if (source.contains("preserved_reference") || source.at("anchor_version") != 1)
    throw std::runtime_error("PRESERVED_REFERENCE_REQUIRES_ORIGINAL_GEOMETRY");
  const auto& evidence = source.at("source_evidence");
  const auto evidence_file = read_secure_owned_file(
      evidence.at("path").get<std::string>(), 4194304U,
      "PRESERVED_RAW_EVIDENCE_OPEN_FAILED", "PRESERVED_RAW_EVIDENCE_FILE_UNSAFE",
      "PRESERVED_RAW_EVIDENCE_READ_FAILED");
  const auto& embedded = source.at("raw_capture");
  if (evidence_file.sha256 != evidence.at("sha256").get<std::string>() ||
      embedded.at("source_file_sha256") != evidence.at("sha256"))
    throw std::runtime_error("PRESERVED_RAW_EVIDENCE_SHA256_MISMATCH");
  const auto capture = parse_json_bytes(evidence_file.data, "PRESERVED_RAW_EVIDENCE_JSON_INVALID");
  if (capture.at("schema") != embedded.at("source_schema") ||
      capture.at("status") != "PASS" || capture.at("physical_power_state_during_capture") != "24V_ON")
    throw std::runtime_error("PRESERVED_RAW_EVIDENCE_STATUS_INVALID");
  for (const char* field : {"capture_id", "power_session_id", "host_boot_id", "recorded_boottime_ns"})
    if (capture.at(field) != embedded.at(field))
      throw std::runtime_error("PRESERVED_RAW_EVIDENCE_IDENTITY_MISMATCH");
  const auto utc_text = [](std::string value) {
    if (!value.empty() && value.back() == 'Z') value.replace(value.size() - 1, 1, "+00:00");
    return value;
  };
  if (utc_text(capture.at("recorded_at_utc").get<std::string>()) !=
      utc_text(embedded.at("recorded_at_utc").get<std::string>()))
    throw std::runtime_error("PRESERVED_RAW_EVIDENCE_TIMESTAMP_MISMATCH");
  const auto& confirmation = source.at("operator_confirmation");
  for (const char* field : {"vertical_initialization_pose", "support_reliable", "arm_not_moved", "not_at_mechanical_limit"})
    if (!confirmation.at(field).get<bool>())
      throw std::runtime_error("PRESERVED_REFERENCE_CONFIRMATION_MISSING");
  const bool source_j2 = source.at("motors").contains("J2A");
  const std::string expected_gate = std::string(source_j2 ? "J2_VERTICAL" : "WHOLE_ARM_VERTICAL") +
      "=YES;SUPPORT_RELIABLE=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES";
  if (confirmation.at("confirmation_gate") != expected_gate ||
      confirmation.at("power_session_id") != capture.at("power_session_id") ||
      confirmation.at("evidence_id").get<std::string>().empty() ||
      confirmation.at("confirmed_at_utc").get<std::string>().empty())
    throw std::runtime_error("PRESERVED_REFERENCE_CONFIRMATION_SCOPE_MISMATCH");
  const auto& safety = capture.at("safety");
  if (safety.at("execution_policy") != "BRAKE_ONLY" ||
      safety.at("all_controller_modes") != nlohmann::json::array({"brake"}) ||
      safety.at("command_rx_enabled").get<bool>())
    throw std::runtime_error("PRESERVED_RAW_EVIDENCE_NOT_BRAKE_ONLY");
  for (const char* field : {"foc_tx_attempt_count", "active_or_hold_commands_sent", "communication_failure_packets", "merror_nonzero_packets"})
    if (!safety.at(field).is_number_integer() || safety.at(field).get<int>() != 0)
      throw std::runtime_error("PRESERVED_RAW_EVIDENCE_FAULT_OR_ACTIVE_COMMAND");
  for (const char* field : {"motor_internal_zero_modified", "rid_written", "flash_or_eeprom_written"})
    if (safety.at(field).get<bool>())
      throw std::runtime_error("PRESERVED_RAW_EVIDENCE_FORBIDDEN_WRITE");
  if (capture.at("motors").size() != embedded.at("motors").size())
    throw std::runtime_error("PRESERVED_RAW_EVIDENCE_MOTOR_SET_MISMATCH");
  for (auto item = capture.at("motors").begin(); item != capture.at("motors").end(); ++item) {
    for (const char* field : {"sample_count", "unwrapped_raw_position_rad"})
      if (item.value().at(field) != embedded.at("motors").at(item.key()).at(field))
        throw std::runtime_error("PRESERVED_RAW_EVIDENCE_STATISTICS_MISMATCH");
    const auto& domain = source_j2 ? capture : capture.at("domains").at(item.key() == "J1" ? "j1" : "j345");
    const auto& stats = item.value().at("unwrapped_raw_position_rad");
    const double coverage = domain.at("source_coverage_s").get<double>();
    const int count = domain.at("packet_count").get<int>();
    const double minimum = stats.at("minimum").get<double>(), maximum = stats.at("maximum").get<double>();
    const double mean = stats.at("mean").get<double>(), span = stats.at("span").get<double>();
    const double deviation = stats.at("standard_deviation").get<double>();
    if (!domain.at("packet_count").is_number_integer() || count < (source_j2 ? 450 : 500) ||
        !std::isfinite(coverage) || coverage < 4.0 || item.value().at("sample_count") != count ||
        source.at("motors").at(item.key()).at("sample_count") != count ||
        !std::isfinite(minimum) || !std::isfinite(maximum) || !std::isfinite(mean) ||
        !std::isfinite(span) || !std::isfinite(deviation) || minimum > mean || mean > maximum ||
        span < 0.0 || span > kGear * kBrakeStationaritySpan ||
        deviation < 0.0 || deviation > span + 1e-12 || std::abs(maximum - minimum - span) > 1e-12 + span * 1e-9 ||
        source.at("motors").at(item.key()).at("sample_span_raw_rad") != span)
      throw std::runtime_error("PRESERVED_RAW_EVIDENCE_COVERAGE_OR_STATIONARITY_INVALID");
  }
  if (source_j2) {
    const auto& summary = capture.at("raw_safety_summary");
    for (const char* field : {"startup_brake_prime", "phase_tx_accounting"})
      if (embedded.at(field) != summary.at(field))
        throw std::runtime_error("PRESERVED_RAW_EVIDENCE_J2_PROOF_MISMATCH");
    for (const char* field : {"path", "sha256", "terminal_proof"})
      if (embedded.at("worker").at(field) != summary.at("worker").at(field))
        throw std::runtime_error("PRESERVED_RAW_EVIDENCE_WORKER_MISMATCH");
    for (const char* field : {"packet_count", "source_coverage_s"})
      if (embedded.at(field) != capture.at(field))
        throw std::runtime_error("PRESERVED_RAW_EVIDENCE_COVERAGE_MISMATCH");
  } else {
    if (embedded.at("worker") != capture.at("worker") ||
        embedded.at("domains").size() != capture.at("domains").size())
      throw std::runtime_error("PRESERVED_RAW_EVIDENCE_GO_AUX_PROOF_MISMATCH");
    for (auto domain = capture.at("domains").begin(); domain != capture.at("domains").end(); ++domain)
      for (const char* field : {"packet_count", "source_coverage_s", "motor_names"})
        if (domain.value().at(field) != embedded.at("domains").at(domain.key()).at(field))
          throw std::runtime_error("PRESERVED_RAW_EVIDENCE_GO_AUX_PROOF_MISMATCH");
  }
  for (const char* field : {"schema", "reference_name", "parent_persistent_zero_sha256"})
    if (source.at(field) != document.at(field))
      throw std::runtime_error("PRESERVED_REFERENCE_PARENT_MISMATCH");
  for (const char* field : {"recovery_branch_hints_sha256", "initial_pose_sha256"})
    if (source.at("preserved_inputs").at(field) != document.at("preserved_inputs").at(field))
      throw std::runtime_error("PRESERVED_REFERENCE_INPUT_MISMATCH");
  for (const char* field : {"is_software_zero", "authorizes_active_control", "authorizes_motor_internal_write"})
    if (source.at("control_authority").at(field).get<bool>())
      throw std::runtime_error("PRESERVED_REFERENCE_SCOPE_INVALID");
  for (const char* field : {"motor_internal_zero_modified", "rid_written", "flash_or_eeprom_written"})
    if (source.at("writes").at(field).get<bool>())
      throw std::runtime_error("PRESERVED_REFERENCE_WRITE_INVALID");
  if (source.at("motors").size() != document.at("motors").size())
    throw std::runtime_error("PRESERVED_REFERENCE_MOTOR_SET_MISMATCH");
  for (auto item = document.at("motors").begin(); item != document.at("motors").end(); ++item) {
    for (const char* field : {"session_reference_raw_rad", "logical_position_rad", "sign", "gear_ratio"})
      if (item.value().at(field) != source.at("motors").at(item.key()).at(field))
        throw std::runtime_error("PRESERVED_REFERENCE_GEOMETRY_CHANGED");
    if (std::abs(source.at("motors").at(item.key()).at("session_reference_raw_rad").get<double>() -
        source.at("raw_capture").at("motors").at(item.key()).at("unwrapped_raw_position_rad").at("mean").get<double>()) > 1e-12)
      throw std::runtime_error("PRESERVED_REFERENCE_ORIGINAL_CAPTURE_MISMATCH");
  }
  return true;
}

void apply_session_capture_reference(
    MotorRuntime& motor, const nlohmann::json& value,
    const nlohmann::json& statistics, bool preserved, bool supported_recovery) {
  motor.session_capture_raw_position = statistics.at("mean").get<double>();
  const double proof_limit = supported_recovery ? 5.0 * kPi / 180.0 : kJ2SessionStartupTolerance;
  const auto proof_reference = [&](double raw) {
    return reference_for_session_hint(raw, motor, motor.session_logical_position, proof_limit);
  };
  motor.session_startup_logical_position = motor.sign *
      (motor.session_capture_raw_position -
       proof_reference(motor.session_capture_raw_position)) / kGear;
  if (preserved) {
    (void)proof_reference(statistics.at("minimum").get<double>());
    (void)proof_reference(statistics.at("maximum").get<double>());
    const double claimed = value.at("startup_logical_position_rad").get<double>();
    if (!std::isfinite(claimed) ||
        std::abs(claimed - motor.session_startup_logical_position) > 1e-12 ||
        !within_mechanical_feedback_envelope(motor.joint_index, claimed))
      throw std::runtime_error("PRESERVED_REFERENCE_STARTUP_POSITION_MISMATCH");
  }
  motor.supported_near_vertical_recovery = supported_recovery;
}

void apply_j2_session_reference_document(
    const nlohmann::json& document,
    const std::string& expected_zero_sha256,
    const std::string& actual_recovery_hint_sha256,
    const std::string& expected_power_session_id,
    const std::string& expected_host_boot_id,
    std::vector<MotorRuntime>& motors) {
  const bool preserved_geometry = validate_preserved_session_geometry(document);
  if (!valid_sha256(expected_zero_sha256))
    throw std::runtime_error("J2_SESSION_PARENT_SHA256_INVALID");
  if (!valid_sha256(actual_recovery_hint_sha256))
    throw std::runtime_error("J2_SESSION_HINT_SHA256_INVALID");
  if (!valid_power_session_id(expected_power_session_id) ||
      !valid_host_boot_id(expected_host_boot_id))
    throw std::runtime_error("J2_SESSION_IDENTITY_INVALID");
  if (document.at("schema") !=
          "go-m8010-j2-power-session-reference/1.0" ||
      document.at("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1")
    throw std::runtime_error("J2_SESSION_REFERENCE_SCHEMA_MISMATCH");
  const std::string parent_sha =
      document.at("parent_persistent_zero_sha256").get<std::string>();
  if (!valid_sha256(parent_sha) || parent_sha != expected_zero_sha256)
    throw std::runtime_error("J2_SESSION_REFERENCE_PARENT_MISMATCH");
  const auto& preserved_inputs = document.at("preserved_inputs");
  if (preserved_inputs.at("recovery_branch_hints_sha256").get<std::string>() !=
          actual_recovery_hint_sha256 ||
      preserved_inputs.at("overwritten_or_deleted").get<bool>())
    throw std::runtime_error("J2_SESSION_REFERENCE_HINT_SHA256_MISMATCH");
  const auto& evidence = document.at("source_evidence");
  const std::string evidence_sha = evidence.at("sha256").get<std::string>();
  if (evidence.at("path").get<std::string>().empty() ||
      !valid_sha256(evidence_sha))
    throw std::runtime_error("J2_SESSION_REFERENCE_EVIDENCE_INVALID");
  const auto& confirmation = document.at("operator_confirmation");
  for (const char* field : {
           "vertical_initialization_pose", "support_reliable",
           "arm_not_moved", "not_at_mechanical_limit"}) {
    if (!confirmation.at(field).get<bool>())
      throw std::runtime_error("J2_SESSION_REFERENCE_CONFIRMATION_MISSING");
  }
  const auto& raw_capture = document.at("raw_capture");
  const std::string raw_power_session_id =
      raw_capture.at("power_session_id").get<std::string>();
  const std::string raw_host_boot_id =
      raw_capture.at("host_boot_id").get<std::string>();
  const std::uint64_t raw_recorded_boottime_ns =
      raw_capture.at("recorded_boottime_ns").get<std::uint64_t>();
  if (raw_capture.at("source_schema") !=
          "go-m8010-j2-brake-raw-capture-statistics/1.0" ||
      raw_capture.at("source_file_sha256").get<std::string>() != evidence_sha ||
      raw_capture.at("packet_count").get<int>() < 450 ||
      raw_capture.at("source_coverage_s").get<double>() < 4.0 ||
      raw_power_session_id !=
          confirmation.at("power_session_id").get<std::string>() ||
      raw_power_session_id != expected_power_session_id ||
      !valid_host_boot_id(raw_host_boot_id) ||
      raw_host_boot_id != expected_host_boot_id ||
      raw_recorded_boottime_ns == 0U)
    throw std::runtime_error("J2_SESSION_REFERENCE_RAW_CAPTURE_INVALID");
  const auto& writes = document.at("writes");
  if (writes.at("motor_internal_zero_modified").get<bool>() ||
      writes.at("rid_written").get<bool>() ||
      writes.at("flash_or_eeprom_written").get<bool>())
    throw std::runtime_error("J2_SESSION_REFERENCE_FORBIDDEN_WRITE_RECORD");
  const auto& control_authority = document.at("control_authority");
  if (control_authority.at("is_software_zero").get<bool>() ||
      control_authority.at("authorizes_active_control").get<bool>() ||
      control_authority.at("authorizes_motor_internal_write").get<bool>())
    throw std::runtime_error("J2_SESSION_REFERENCE_SCOPE_INVALID");
  const auto& values = document.at("motors");
  const auto& raw_motors = raw_capture.at("motors");
  if (!values.is_object() || values.size() != 2U ||
      !values.contains("J2A") || !values.contains("J2B") ||
      !raw_motors.is_object() || raw_motors.size() != 2U ||
      !raw_motors.contains("J2A") || !raw_motors.contains("J2B"))
    throw std::runtime_error("J2_SESSION_REFERENCE_MOTOR_SET_INVALID");
  for (auto& motor : motors) {
    if (std::string(motor.name) != "J2A" &&
        std::string(motor.name) != "J2B")
      continue;
    const auto& value = values.at(motor.name);
    const double reference =
        value.at("session_reference_raw_rad").get<double>();
    const double logical = value.at("logical_position_rad").get<double>();
    const double span = value.at("sample_span_raw_rad").get<double>();
    const int samples = value.at("sample_count").get<int>();
    const int sign = value.at("sign").get<int>();
    const double gear = value.at("gear_ratio").get<double>();
    const auto& raw_value = raw_motors.at(motor.name);
    const auto& raw_statistics =
        raw_value.at("unwrapped_raw_position_rad");
    if (!std::isfinite(reference) || !std::isfinite(logical) ||
        !std::isfinite(span) || span < 0.0 ||
        span / kGear > kBrakeStationaritySpan || samples < 450 ||
        sign != motor.sign || !std::isfinite(gear) ||
        std::abs(gear - kGear) > 1e-6 || std::abs(logical) > 1e-9 ||
        raw_value.at("sample_count").get<int>() != samples ||
        (!preserved_geometry &&
         std::abs(raw_statistics.at("mean").get<double>() - reference) > 1e-12) ||
        std::abs(raw_statistics.at("span").get<double>() - span) > 1e-12)
      throw std::runtime_error("J2_SESSION_REFERENCE_MOTOR_VALUE_INVALID");
    if (!motor.recovery_hint_configured ||
        std::abs(motor.recovery_hint - logical) > 1e-9)
      throw std::runtime_error("J2_SESSION_REFERENCE_HINT_MISMATCH");
    motor.session_reference = reference;
    motor.session_logical_position = logical;
    motor.session_reference_configured = true;
    apply_session_capture_reference(motor, value, raw_statistics, preserved_geometry,
        preserved_geometry && document.at("preserved_reference").contains("supported_near_vertical_recovery"));
  }
  if (motors.size() != 2U ||
      !std::all_of(motors.begin(), motors.end(), [](const MotorRuntime& motor) {
        return motor.session_reference_configured;
      }))
    throw std::runtime_error("J2_SESSION_REFERENCE_INCOMPLETE");
  if (preserved_geometry && std::abs(motors[0].session_startup_logical_position -
          motors[1].session_startup_logical_position) > kJ2SyncLimit)
    throw std::runtime_error("PRESERVED_REFERENCE_J2_SYNC_EXCEEDED");
}

std::string canonical_existing_path(const std::string& path,
                                    const char* error);

struct LoadedJ2SessionReference {
  std::string canonical_path;
  std::string sha256;
};

LoadedJ2SessionReference load_j2_session_reference(
    const std::string& path, const std::string& expected_zero_sha256,
    const std::string& actual_recovery_hint_sha256,
    const std::string& expected_session_reference_sha256,
    const std::string& expected_power_session_id,
    const std::string& expected_host_boot_id,
    std::vector<MotorRuntime>& motors) {
  if (path.empty()) return {};
  if (!valid_sha256(expected_session_reference_sha256))
    throw std::runtime_error("J2_SESSION_REFERENCE_EXPECTED_SHA256_INVALID");
  const std::string canonical_path = canonical_existing_path(
      path, "J2_SESSION_REFERENCE_CANONICAL_PATH_FAILED");
  const SecureFileBytes source = read_secure_owned_file(
      canonical_path, 2097152U, "J2_SESSION_REFERENCE_OPEN_FAILED",
      "J2_SESSION_REFERENCE_FILE_UNSAFE",
      "J2_SESSION_REFERENCE_READ_FAILED");
  if (source.sha256 != expected_session_reference_sha256)
    throw std::runtime_error("J2_SESSION_REFERENCE_SHA256_MISMATCH");
  const nlohmann::json document =
      parse_json_bytes(source.data, "J2_SESSION_REFERENCE_JSON_INVALID");
  apply_j2_session_reference_document(
      document, expected_zero_sha256, actual_recovery_hint_sha256,
      expected_power_session_id, expected_host_boot_id, motors);
  return {canonical_path, source.sha256};
}

void apply_go_aux_session_reference_document(
    const nlohmann::json& document,
    const std::string& expected_zero_sha256,
    const std::string& actual_recovery_hint_sha256,
    const std::string& expected_power_session_id,
    const std::string& expected_host_boot_id,
    std::vector<MotorRuntime>& motors) {
  const bool preserved_geometry = validate_preserved_session_geometry(document);
  if (!valid_sha256(expected_zero_sha256) ||
      !valid_sha256(actual_recovery_hint_sha256) ||
      !valid_power_session_id(expected_power_session_id) ||
      !valid_host_boot_id(expected_host_boot_id))
    throw std::runtime_error("GO_AUX_SESSION_IDENTITY_INVALID");
  if (document.at("schema") !=
          "go-m8010-go-aux-power-session-reference/1.0" ||
      document.at("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1")
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_SCHEMA_MISMATCH");
  if (document.at("parent_persistent_zero_sha256").get<std::string>() !=
      expected_zero_sha256)
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_PARENT_MISMATCH");
  const auto& preserved = document.at("preserved_inputs");
  if (preserved.at("recovery_branch_hints_sha256").get<std::string>() !=
          actual_recovery_hint_sha256 ||
      !valid_sha256(preserved.at("initial_pose_sha256").get<std::string>()) ||
      preserved.at("overwritten_or_deleted").get<bool>() ||
      preserved.at("recovery_hints_used_for_go_aux").get<bool>())
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_PRESERVED_INPUT_MISMATCH");
  const auto& evidence = document.at("source_evidence");
  const std::string evidence_sha = evidence.at("sha256").get<std::string>();
  if (evidence.at("path").get<std::string>().empty() ||
      !valid_sha256(evidence_sha))
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_EVIDENCE_INVALID");
  const auto& confirmation = document.at("operator_confirmation");
  for (const char* field : {
           "vertical_initialization_pose", "support_reliable",
           "arm_not_moved", "not_at_mechanical_limit"}) {
    if (!confirmation.at(field).get<bool>())
      throw std::runtime_error("GO_AUX_SESSION_REFERENCE_CONFIRMATION_MISSING");
  }
  const auto& writes = document.at("writes");
  if (writes.at("motor_internal_zero_modified").get<bool>() ||
      writes.at("rid_written").get<bool>() ||
      writes.at("flash_or_eeprom_written").get<bool>())
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_FORBIDDEN_WRITE_RECORD");
  const auto& authority = document.at("control_authority");
  if (authority.at("is_software_zero").get<bool>() ||
      authority.at("authorizes_active_control").get<bool>() ||
      authority.at("authorizes_motor_internal_write").get<bool>())
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_SCOPE_INVALID");
  const auto& raw_capture = document.at("raw_capture");
  if (raw_capture.at("source_schema") !=
          "go-m8010-go-aux-brake-raw-capture-statistics/1.0" ||
      raw_capture.at("source_file_sha256").get<std::string>() != evidence_sha ||
      raw_capture.at("power_session_id").get<std::string>() !=
          expected_power_session_id ||
      raw_capture.at("host_boot_id").get<std::string>() !=
          expected_host_boot_id ||
      raw_capture.at("recorded_boottime_ns").get<std::uint64_t>() == 0U)
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_RAW_CAPTURE_INVALID");
  const auto& values = document.at("motors");
  const auto& raw_motors = raw_capture.at("motors");
  const std::set<std::string> expected_names{"J1", "J3", "J4", "J5"};
  std::set<std::string> value_names;
  std::set<std::string> raw_names;
  for (auto item = values.begin(); item != values.end(); ++item)
    value_names.insert(item.key());
  for (auto item = raw_motors.begin(); item != raw_motors.end(); ++item)
    raw_names.insert(item.key());
  if (!values.is_object() || !raw_motors.is_object() ||
      value_names != expected_names || raw_names != expected_names)
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_MOTOR_SET_INVALID");
  for (auto& motor : motors) {
    if (expected_names.count(motor.name) == 0U)
      throw std::runtime_error("GO_AUX_SESSION_REFERENCE_BUS_MOTOR_INVALID");
    const auto& value = values.at(motor.name);
    const double reference =
        value.at("session_reference_raw_rad").get<double>();
    const double logical = value.at("logical_position_rad").get<double>();
    const double span = value.at("sample_span_raw_rad").get<double>();
    const int samples = value.at("sample_count").get<int>();
    const int sign = value.at("sign").get<int>();
    const double gear = value.at("gear_ratio").get<double>();
    const auto& raw_value = raw_motors.at(motor.name);
    const auto& statistics = raw_value.at("unwrapped_raw_position_rad");
    if (!std::isfinite(reference) || !std::isfinite(logical) ||
        !within_mechanical_feedback_envelope(motor.joint_index, logical) ||
        !std::isfinite(span) || span < 0.0 ||
        span / kGear > kBrakeStationaritySpan || samples < 500 ||
        sign != motor.sign || !std::isfinite(gear) ||
        std::abs(gear - kGear) > 1e-6 ||
        raw_value.at("sample_count").get<int>() != samples ||
        (!preserved_geometry &&
         std::abs(statistics.at("mean").get<double>() - reference) > 1e-12) ||
        std::abs(statistics.at("span").get<double>() - span) > 1e-12)
      throw std::runtime_error("GO_AUX_SESSION_REFERENCE_MOTOR_VALUE_INVALID");
    motor.session_reference = reference;
    motor.session_logical_position = logical;
    motor.session_reference_configured = true;
    apply_session_capture_reference(motor, value, statistics, preserved_geometry,
        preserved_geometry && document.at("preserved_reference").contains("supported_near_vertical_recovery"));
  }
  if (!std::all_of(motors.begin(), motors.end(), [](const MotorRuntime& motor) {
        return motor.session_reference_configured;
      }))
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_INCOMPLETE");
}

LoadedJ2SessionReference load_go_aux_session_reference(
    const std::string& path, const std::string& expected_zero_sha256,
    const std::string& actual_recovery_hint_sha256,
    const std::string& expected_session_reference_sha256,
    const std::string& expected_power_session_id,
    const std::string& expected_host_boot_id,
    std::vector<MotorRuntime>& motors) {
  if (!valid_sha256(expected_session_reference_sha256))
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_EXPECTED_SHA256_INVALID");
  const std::string canonical_path = canonical_existing_path(
      path, "GO_AUX_SESSION_REFERENCE_CANONICAL_PATH_FAILED");
  const SecureFileBytes source = read_secure_owned_file(
      canonical_path, 4194304U, "GO_AUX_SESSION_REFERENCE_OPEN_FAILED",
      "GO_AUX_SESSION_REFERENCE_FILE_UNSAFE",
      "GO_AUX_SESSION_REFERENCE_READ_FAILED");
  if (source.sha256 != expected_session_reference_sha256)
    throw std::runtime_error("GO_AUX_SESSION_REFERENCE_SHA256_MISMATCH");
  const nlohmann::json document =
      parse_json_bytes(source.data, "GO_AUX_SESSION_REFERENCE_JSON_INVALID");
  apply_go_aux_session_reference_document(
      document, expected_zero_sha256, actual_recovery_hint_sha256,
      expected_power_session_id, expected_host_boot_id, motors);
  return {canonical_path, source.sha256};
}

struct J2LaunchPermit {
  std::string permit_id;
  std::string power_session_id;
  std::string pending_path;
  std::string inflight_path;
  std::string spent_path;
  std::uint64_t issued_boottime_ns = 0U;
  std::uint64_t expires_boottime_ns = 0U;
  std::uint64_t capture_boottime_ns = 0U;
  std::size_t minimum_brake_frames = 0U;
  double maximum_raw_phase_delta_rad = 0.0;
  double maximum_raw_span_rad = 0.0;
  std::string filename;
  dev_t file_device = 0;
  ino_t file_inode = 0;
  UniqueFd permit_fd;
  UniqueFd pending_directory_fd;
  UniqueFd inflight_directory_fd;
  UniqueFd spent_directory_fd;
  std::string state = "NONE";
};
static_assert(!std::is_copy_constructible<J2LaunchPermit>::value,
              "J2 permit must retain unique fd ownership");
static_assert(std::is_nothrow_move_constructible<J2LaunchPermit>::value,
              "J2 permit move construction must not throw");
static_assert(std::is_nothrow_move_assignable<J2LaunchPermit>::value,
              "J2 permit move assignment must not throw");

std::string canonical_existing_path(const std::string& path,
                                    const char* error) {
  try {
    return std::filesystem::canonical(std::filesystem::path(path)).string();
  } catch (const std::filesystem::filesystem_error&) {
    throw std::runtime_error(error);
  }
}

UniqueFd open_secure_owned_directory(const std::string& path) {
  UniqueFd descriptor(::open(
      path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_DIRECTORY));
  struct stat status{};
  if (!descriptor || ::fstat(descriptor.get(), &status) != 0 ||
      !S_ISDIR(status.st_mode) || status.st_uid != ::geteuid() ||
      (status.st_mode & 07777) != 0700)
    throw std::runtime_error("J2_LAUNCH_PERMIT_DIRECTORY_UNSAFE");
  return descriptor;
}

struct OpenedPermitDocument {
  UniqueFd descriptor;
  struct stat status{};
  nlohmann::json document;
};

OpenedPermitDocument read_secure_owned_permit_at(
    int directory_descriptor, const std::string& filename) {
  UniqueFd descriptor(::openat(
      directory_descriptor, filename.c_str(),
      O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK));
  if (!descriptor) throw std::runtime_error("J2_LAUNCH_PERMIT_OPEN_FAILED");
  SecureFileBytes source = read_secure_regular_fd(
      descriptor.get(), 131072U, true, "J2_LAUNCH_PERMIT_FILE_UNSAFE",
      "J2_LAUNCH_PERMIT_READ_FAILED");
  nlohmann::json document =
      parse_json_bytes(source.data, "J2_LAUNCH_PERMIT_JSON_INVALID");
  return {std::move(descriptor), source.status, std::move(document)};
}

SecureFileBytes read_running_executable() {
  // /proc/self/exe is intentionally a procfs magic link to the exact inode
  // mapped into this process, so O_NOFOLLOW cannot be used on this one path.
  UniqueFd descriptor(::open("/proc/self/exe", O_RDONLY | O_CLOEXEC));
  if (!descriptor) throw std::runtime_error("J2_WORKER_OPEN_FAILED");
  return read_secure_regular_fd(
      descriptor.get(), 67108864U, false, "J2_WORKER_FILE_UNSAFE",
      "J2_WORKER_READ_FAILED");
}

std::string expected_permit_directory(const char* state) {
  return "/run/user/" + std::to_string(static_cast<unsigned long>(::geteuid())) +
      "/go-m8010/anchors/" + state;
}

void validate_permit_state_path(const std::string& path, const char* state,
                                const std::string& filename) {
  const std::filesystem::path normalized =
      std::filesystem::path(path).lexically_normal();
  if (!normalized.is_absolute() || normalized.filename().string() != filename ||
      normalized.parent_path().string() != expected_permit_directory(state))
    throw std::runtime_error("J2_LAUNCH_PERMIT_PATH_INVALID");
}

J2LaunchPermit apply_j2_launch_permit_document(
    const nlohmann::json& document,
    const Options& options,
    const std::string& canonical_reference_path,
    const std::string& canonical_permit_path,
    const std::string& canonical_worker_path,
    const std::string& actual_zero_sha256,
    const std::string& actual_reference_sha256,
    const std::string& actual_worker_sha256,
    const std::string& host_boot_id,
    std::uint64_t now_boottime_ns) {
  if (!valid_sha256(options.expected_zero_sha256) ||
      !valid_sha256(options.expected_j2_session_reference_sha256) ||
      !valid_sha256(options.expected_worker_sha256) ||
      !valid_sha256(actual_zero_sha256) ||
      !valid_sha256(actual_reference_sha256) ||
      !valid_sha256(actual_worker_sha256) ||
      options.expected_zero_sha256 != actual_zero_sha256 ||
      options.expected_j2_session_reference_sha256 != actual_reference_sha256 ||
      options.expected_worker_sha256 != actual_worker_sha256 ||
      !valid_power_session_id(options.expected_j2_power_session_id) ||
      !valid_host_boot_id(host_boot_id) || now_boottime_ns == 0U)
    throw std::runtime_error("J2_LAUNCH_PERMIT_EXPECTATION_INVALID");
  if (document.at("schema") !=
          "go-m8010-j2-power-session-launch-permit/1.0" ||
      document.at("permit_version").get<int>() != 1 ||
      document.at("scope") != "j2" ||
      !document.at("single_use").get<bool>())
    throw std::runtime_error("J2_LAUNCH_PERMIT_SCHEMA_MISMATCH");

  J2LaunchPermit permit;
  permit.permit_id = document.at("permit_id").get<std::string>();
  permit.power_session_id =
      document.at("power_session_id").get<std::string>();
  if (!valid_sha256(permit.permit_id) ||
      permit.power_session_id != options.expected_j2_power_session_id ||
      document.at("host_boot_id").get<std::string>() != host_boot_id ||
      document.at("parent_persistent_zero_sha256").get<std::string>() !=
          actual_zero_sha256)
    throw std::runtime_error("J2_LAUNCH_PERMIT_BINDING_MISMATCH");

  const auto& reference = document.at("session_reference");
  if (reference.at("path").get<std::string>() != canonical_reference_path ||
      reference.at("sha256").get<std::string>() != actual_reference_sha256)
    throw std::runtime_error("J2_LAUNCH_PERMIT_REFERENCE_MISMATCH");
  const auto& worker = document.at("worker");
  if (worker.at("path").get<std::string>() != canonical_worker_path ||
      worker.at("sha256").get<std::string>() != actual_worker_sha256)
    throw std::runtime_error("J2_LAUNCH_PERMIT_WORKER_MISMATCH");

  const auto& source_capture = document.at("source_capture");
  permit.capture_boottime_ns =
      source_capture.at("recorded_boottime_ns").get<std::uint64_t>();
  if (source_capture.at("host_boot_id").get<std::string>() != host_boot_id ||
      permit.capture_boottime_ns == 0U)
    throw std::runtime_error("J2_LAUNCH_PERMIT_CAPTURE_MISMATCH");
  permit.issued_boottime_ns =
      document.at("issued_boottime_ns").get<std::uint64_t>();
  permit.expires_boottime_ns =
      document.at("expires_boottime_ns").get<std::uint64_t>();
  if (permit.issued_boottime_ns == 0U ||
      permit.expires_boottime_ns <= permit.issued_boottime_ns ||
      permit.expires_boottime_ns - permit.issued_boottime_ns >
          kJ2StartupPermitMaximumTtlNs ||
      permit.capture_boottime_ns > permit.issued_boottime_ns ||
      permit.issued_boottime_ns - permit.capture_boottime_ns >
          kJ2CaptureToPermitMaximumAgeNs ||
      now_boottime_ns < permit.issued_boottime_ns ||
      now_boottime_ns >= permit.expires_boottime_ns)
    throw std::runtime_error("J2_LAUNCH_PERMIT_EXPIRED_OR_FUTURE");

  permit.pending_path = document.at("pending_path").get<std::string>();
  permit.inflight_path = document.at("inflight_path").get<std::string>();
  permit.spent_path = document.at("spent_path").get<std::string>();
  if (permit.pending_path != canonical_permit_path)
    throw std::runtime_error("J2_LAUNCH_PERMIT_CANONICAL_PATH_MISMATCH");
  permit.filename =
      std::filesystem::path(permit.pending_path).filename().string();
  if (permit.filename != permit.permit_id + ".json")
    throw std::runtime_error("J2_LAUNCH_PERMIT_FILENAME_MISMATCH");
  validate_permit_state_path(permit.pending_path, "pending", permit.filename);
  validate_permit_state_path(permit.inflight_path, "inflight", permit.filename);
  validate_permit_state_path(permit.spent_path, "spent", permit.filename);

  const auto& serial = document.at("serial");
  const std::vector<int> motor_ids =
      serial.at("motor_ids").get<std::vector<int>>();
  const auto& signs = serial.at("signs");
  if (serial.at("stable_by_id") !=
          "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0" ||
      serial.at("bus") != "j2" || motor_ids != std::vector<int>({0, 1}) ||
      std::abs(serial.at("gear_ratio").get<double>() - kGear) > 1e-6 ||
      !signs.is_object() || signs.size() != 2U ||
      signs.at("J2A").get<int>() != -1 ||
      signs.at("J2B").get<int>() != 1)
    throw std::runtime_error("J2_LAUNCH_PERMIT_SERIAL_SCOPE_INVALID");

  const auto& recheck = document.at("startup_recheck");
  permit.minimum_brake_frames =
      recheck.at("minimum_brake_frames").get<std::size_t>();
  permit.maximum_raw_phase_delta_rad =
      recheck.at("max_raw_phase_delta_rad").get<double>();
  permit.maximum_raw_span_rad =
      recheck.at("max_raw_span_rad").get<double>();
  if (permit.minimum_brake_frames < kJ2StartupPermitMinimumBrakeFrames ||
      permit.minimum_brake_frames > 150U ||
      !std::isfinite(permit.maximum_raw_phase_delta_rad) ||
      permit.maximum_raw_phase_delta_rad <= 0.0 ||
      permit.maximum_raw_phase_delta_rad > kJ2StartupPermitRawDelta + 1e-12 ||
      !std::isfinite(permit.maximum_raw_span_rad) ||
      permit.maximum_raw_span_rad <= 0.0 ||
      permit.maximum_raw_span_rad > kJ2StartupPermitRawSpan + 1e-12)
    throw std::runtime_error("J2_LAUNCH_PERMIT_RECHECK_INVALID");
  permit.state = "PENDING";
  return permit;
}

J2LaunchPermit apply_go_aux_launch_permit_document(
    const nlohmann::json& document,
    const Options& options,
    const std::string& canonical_reference_path,
    const std::string& canonical_permit_path,
    const std::string& canonical_worker_path,
    const std::string& actual_zero_sha256,
    const std::string& actual_reference_sha256,
    const std::string& actual_worker_sha256,
    const std::string& host_boot_id,
    std::uint64_t now_boottime_ns) {
  if (!valid_sha256(options.expected_zero_sha256) ||
      !valid_sha256(options.expected_go_aux_session_reference_sha256) ||
      !valid_sha256(options.expected_worker_sha256) ||
      options.expected_zero_sha256 != actual_zero_sha256 ||
      options.expected_go_aux_session_reference_sha256 !=
          actual_reference_sha256 ||
      options.expected_worker_sha256 != actual_worker_sha256 ||
      !valid_power_session_id(options.expected_go_aux_power_session_id) ||
      !valid_host_boot_id(host_boot_id) || now_boottime_ns == 0U)
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_EXPECTATION_INVALID");
  if (document.at("schema") !=
          "go-m8010-go-aux-power-session-launch-permit/1.0" ||
      document.at("permit_version").get<int>() != 1 ||
      document.at("scope").get<std::string>() != options.bus ||
      !document.at("single_use").get<bool>())
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_SCHEMA_MISMATCH");
  J2LaunchPermit permit;
  permit.permit_id = document.at("permit_id").get<std::string>();
  permit.power_session_id = document.at("power_session_id").get<std::string>();
  if (!valid_sha256(permit.permit_id) ||
      permit.power_session_id != options.expected_go_aux_power_session_id ||
      document.at("host_boot_id").get<std::string>() != host_boot_id ||
      document.at("parent_persistent_zero_sha256").get<std::string>() !=
          actual_zero_sha256)
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_BINDING_MISMATCH");
  const auto& reference = document.at("session_reference");
  if (reference.at("path").get<std::string>() != canonical_reference_path ||
      reference.at("sha256").get<std::string>() != actual_reference_sha256)
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_REFERENCE_MISMATCH");
  const auto& worker = document.at("worker");
  if (worker.at("path").get<std::string>() != canonical_worker_path ||
      worker.at("sha256").get<std::string>() != actual_worker_sha256)
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_WORKER_MISMATCH");
  const auto& source_capture = document.at("source_capture");
  permit.capture_boottime_ns =
      source_capture.at("recorded_boottime_ns").get<std::uint64_t>();
  if (source_capture.at("host_boot_id").get<std::string>() != host_boot_id ||
      permit.capture_boottime_ns == 0U)
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_CAPTURE_MISMATCH");
  permit.issued_boottime_ns =
      document.at("issued_boottime_ns").get<std::uint64_t>();
  permit.expires_boottime_ns =
      document.at("expires_boottime_ns").get<std::uint64_t>();
  if (permit.issued_boottime_ns == 0U ||
      permit.expires_boottime_ns <= permit.issued_boottime_ns ||
      permit.expires_boottime_ns - permit.issued_boottime_ns >
          kJ2StartupPermitMaximumTtlNs ||
      permit.capture_boottime_ns > permit.issued_boottime_ns ||
      permit.issued_boottime_ns - permit.capture_boottime_ns >
          kJ2CaptureToPermitMaximumAgeNs ||
      now_boottime_ns < permit.issued_boottime_ns ||
      now_boottime_ns >= permit.expires_boottime_ns)
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_EXPIRED_OR_FUTURE");
  permit.pending_path = document.at("pending_path").get<std::string>();
  permit.inflight_path = document.at("inflight_path").get<std::string>();
  permit.spent_path = document.at("spent_path").get<std::string>();
  if (permit.pending_path != canonical_permit_path)
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_CANONICAL_PATH_MISMATCH");
  permit.filename =
      std::filesystem::path(permit.pending_path).filename().string();
  if (permit.filename != permit.permit_id + ".json")
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_FILENAME_MISMATCH");
  validate_permit_state_path(permit.pending_path, "pending", permit.filename);
  validate_permit_state_path(permit.inflight_path, "inflight", permit.filename);
  validate_permit_state_path(permit.spent_path, "spent", permit.filename);
  const auto& serial = document.at("serial");
  const BusDefinition definition = bus_definition(options.bus);
  const std::vector<int> expected_ids =
      options.bus == "j1" ? std::vector<int>{0} : std::vector<int>{3, 4, 5};
  const auto& signs = serial.at("signs");
  const bool signs_valid = options.bus == "j1"
      ? signs.is_object() && signs.size() == 1U && signs.at("J1").get<int>() == 1
      : signs.is_object() && signs.size() == 3U &&
          signs.at("J3").get<int>() == 1 && signs.at("J4").get<int>() == -1 &&
          signs.at("J5").get<int>() == 1;
  if (serial.at("stable_by_id").get<std::string>() != definition.port ||
      serial.at("bus").get<std::string>() != options.bus ||
      serial.at("motor_ids").get<std::vector<int>>() != expected_ids ||
      std::abs(serial.at("gear_ratio").get<double>() - kGear) > 1e-6 ||
      !signs_valid)
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_SERIAL_SCOPE_INVALID");
  const auto& recheck = document.at("startup_recheck");
  permit.minimum_brake_frames =
      recheck.at("minimum_brake_frames").get<std::size_t>();
  permit.maximum_raw_phase_delta_rad =
      recheck.at("max_raw_phase_delta_rad").get<double>();
  permit.maximum_raw_span_rad =
      recheck.at("max_raw_span_rad").get<double>();
  if (permit.minimum_brake_frames < kJ2StartupPermitMinimumBrakeFrames ||
      permit.minimum_brake_frames > 150U ||
      !std::isfinite(permit.maximum_raw_phase_delta_rad) ||
      permit.maximum_raw_phase_delta_rad <= 0.0 ||
      permit.maximum_raw_phase_delta_rad > kJ2StartupPermitRawDelta + 1e-12 ||
      !std::isfinite(permit.maximum_raw_span_rad) ||
      permit.maximum_raw_span_rad <= 0.0 ||
      permit.maximum_raw_span_rad > kJ2StartupPermitRawSpan + 1e-12)
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_RECHECK_INVALID");
  permit.state = "PENDING";
  return permit;
}

void require_j2_launch_permit_fresh(
    const J2LaunchPermit& permit, std::uint64_t now_boottime_ns) {
  if (permit.issued_boottime_ns == 0U ||
      permit.expires_boottime_ns <= permit.issued_boottime_ns ||
      permit.expires_boottime_ns - permit.issued_boottime_ns >
          kJ2StartupPermitMaximumTtlNs ||
      permit.capture_boottime_ns == 0U ||
      permit.capture_boottime_ns > permit.issued_boottime_ns ||
      permit.issued_boottime_ns - permit.capture_boottime_ns >
          kJ2CaptureToPermitMaximumAgeNs ||
      now_boottime_ns < permit.issued_boottime_ns ||
      now_boottime_ns >= permit.expires_boottime_ns ||
      permit.capture_boottime_ns > now_boottime_ns ||
      now_boottime_ns - permit.capture_boottime_ns >
          kJ2CaptureToPermitMaximumAgeNs)
    throw std::runtime_error("J2_LAUNCH_PERMIT_EXPIRED_OR_FUTURE");
}

bool secure_permit_identity_matches(const struct stat& status,
                                    const J2LaunchPermit& permit) {
  return S_ISREG(status.st_mode) && status.st_uid == ::geteuid() &&
      (status.st_mode & 07777) == 0600 && status.st_nlink == 1 &&
      status.st_dev == permit.file_device && status.st_ino == permit.file_inode;
}

void require_secure_owned_directory_fd(int descriptor) {
  struct stat status{};
  if (descriptor < 0 || ::fstat(descriptor, &status) != 0 ||
      !S_ISDIR(status.st_mode) || status.st_uid != ::geteuid() ||
      (status.st_mode & 07777) != 0700)
    throw std::runtime_error("J2_LAUNCH_PERMIT_DIRECTORY_UNSAFE");
}

void require_permit_at_directory(
    const J2LaunchPermit& permit, int directory_descriptor,
    const char* error) {
  struct stat held{};
  struct stat located{};
  if (!permit.permit_fd || ::fstat(permit.permit_fd.get(), &held) != 0 ||
      !secure_permit_identity_matches(held, permit) ||
      ::fstatat(directory_descriptor, permit.filename.c_str(), &located,
                AT_SYMLINK_NOFOLLOW) != 0 ||
      !secure_permit_identity_matches(located, permit) ||
      held.st_dev != located.st_dev || held.st_ino != located.st_ino)
    throw std::runtime_error(error);
}

J2LaunchPermit load_j2_launch_permit(
    const Options& options,
    const LoadedJ2SessionReference& session_reference,
    const std::string& actual_zero_sha256,
    const std::string& host_boot_id,
    std::uint64_t now_boottime_ns) {
  const std::filesystem::path normalized_permit_path =
      std::filesystem::path(options.j2_session_launch_permit_file)
          .lexically_normal();
  const std::string canonical_permit_path = normalized_permit_path.string();
  const std::string filename = normalized_permit_path.filename().string();
  if (!normalized_permit_path.is_absolute() ||
      canonical_permit_path != options.j2_session_launch_permit_file)
    throw std::runtime_error("J2_LAUNCH_PERMIT_CANONICAL_PATH_FAILED");
  validate_permit_state_path(canonical_permit_path, "pending", filename);

  UniqueFd pending_directory =
      open_secure_owned_directory(expected_permit_directory("pending"));
  UniqueFd inflight_directory =
      open_secure_owned_directory(expected_permit_directory("inflight"));
  UniqueFd spent_directory =
      open_secure_owned_directory(expected_permit_directory("spent"));
  OpenedPermitDocument opened =
      read_secure_owned_permit_at(pending_directory.get(), filename);
  const std::string canonical_worker_path = canonical_existing_path(
      "/proc/self/exe", "J2_WORKER_CANONICAL_PATH_FAILED");
  const SecureFileBytes worker = read_running_executable();
  J2LaunchPermit permit = apply_j2_launch_permit_document(
      opened.document, options, session_reference.canonical_path,
      canonical_permit_path, canonical_worker_path, actual_zero_sha256,
      session_reference.sha256, worker.sha256, host_boot_id,
      now_boottime_ns);
  permit.file_device = opened.status.st_dev;
  permit.file_inode = opened.status.st_ino;
  permit.permit_fd = std::move(opened.descriptor);
  permit.pending_directory_fd = std::move(pending_directory);
  permit.inflight_directory_fd = std::move(inflight_directory);
  permit.spent_directory_fd = std::move(spent_directory);
  require_permit_at_directory(
      permit, permit.pending_directory_fd.get(),
      "J2_LAUNCH_PERMIT_LOAD_IDENTITY_MISMATCH");
  require_j2_launch_permit_fresh(permit, now_boottime_ns);
  return permit;
}

J2LaunchPermit load_go_aux_launch_permit(
    const Options& options,
    const LoadedJ2SessionReference& session_reference,
    const std::string& actual_zero_sha256,
    const std::string& host_boot_id,
    std::uint64_t now_boottime_ns) {
  const std::filesystem::path normalized_permit_path =
      std::filesystem::path(options.go_aux_session_launch_permit_file)
          .lexically_normal();
  const std::string canonical_permit_path = normalized_permit_path.string();
  const std::string filename = normalized_permit_path.filename().string();
  if (!normalized_permit_path.is_absolute() ||
      canonical_permit_path != options.go_aux_session_launch_permit_file)
    throw std::runtime_error("GO_AUX_LAUNCH_PERMIT_CANONICAL_PATH_FAILED");
  validate_permit_state_path(canonical_permit_path, "pending", filename);
  UniqueFd pending_directory =
      open_secure_owned_directory(expected_permit_directory("pending"));
  UniqueFd inflight_directory =
      open_secure_owned_directory(expected_permit_directory("inflight"));
  UniqueFd spent_directory =
      open_secure_owned_directory(expected_permit_directory("spent"));
  OpenedPermitDocument opened =
      read_secure_owned_permit_at(pending_directory.get(), filename);
  const std::string canonical_worker_path = canonical_existing_path(
      "/proc/self/exe", "GO_AUX_WORKER_CANONICAL_PATH_FAILED");
  const SecureFileBytes worker = read_running_executable();
  J2LaunchPermit permit = apply_go_aux_launch_permit_document(
      opened.document, options, session_reference.canonical_path,
      canonical_permit_path, canonical_worker_path, actual_zero_sha256,
      session_reference.sha256, worker.sha256, host_boot_id,
      now_boottime_ns);
  permit.file_device = opened.status.st_dev;
  permit.file_inode = opened.status.st_ino;
  permit.permit_fd = std::move(opened.descriptor);
  permit.pending_directory_fd = std::move(pending_directory);
  permit.inflight_directory_fd = std::move(inflight_directory);
  permit.spent_directory_fd = std::move(spent_directory);
  require_permit_at_directory(
      permit, permit.pending_directory_fd.get(),
      "GO_AUX_LAUNCH_PERMIT_LOAD_IDENTITY_MISMATCH");
  require_j2_launch_permit_fresh(permit, now_boottime_ns);
  return permit;
}

void rename_permit_no_replace(
    J2LaunchPermit& permit, int source_directory_descriptor,
    int destination_directory_descriptor, const char* error) {
  require_secure_owned_directory_fd(source_directory_descriptor);
  require_secure_owned_directory_fd(destination_directory_descriptor);
  require_permit_at_directory(permit, source_directory_descriptor, error);
  // This check is intentionally adjacent to renameat2: a process paused during
  // the 50-frame recheck cannot activate with an expired or over-age capture.
  require_j2_launch_permit_fresh(permit, current_boottime_ns());
  const long result = ::syscall(
      SYS_renameat2, source_directory_descriptor, permit.filename.c_str(),
      destination_directory_descriptor, permit.filename.c_str(),
      RENAME_NOREPLACE);
  if (result != 0) throw std::runtime_error(error);

  struct stat destination{};
  if (::fstatat(destination_directory_descriptor, permit.filename.c_str(),
                &destination, AT_SYMLINK_NOFOLLOW) != 0 ||
      !secure_permit_identity_matches(destination, permit))
    throw std::runtime_error(error);
  struct stat source{};
  errno = 0;
  if (::fstatat(source_directory_descriptor, permit.filename.c_str(), &source,
                AT_SYMLINK_NOFOLLOW) == 0 || errno != ENOENT)
    throw std::runtime_error(error);
  // Close the pause-at-expiry window on both transitions. If this post-check
  // fails after a successful rename, the token remains consumed/spent and the
  // caller fails closed without opening the command socket.
  require_j2_launch_permit_fresh(permit, current_boottime_ns());
}

void consume_j2_launch_permit(J2LaunchPermit& permit) {
  if (permit.state != "PENDING")
    throw std::runtime_error("J2_LAUNCH_PERMIT_NOT_PENDING");
  rename_permit_no_replace(
      permit, permit.pending_directory_fd.get(),
      permit.inflight_directory_fd.get(),
      "J2_LAUNCH_PERMIT_CONSUME_FAILED");
  permit.state = "INFLIGHT";
}

void spend_j2_launch_permit(J2LaunchPermit& permit) {
  if (permit.state != "INFLIGHT")
    throw std::runtime_error("J2_LAUNCH_PERMIT_NOT_INFLIGHT");
  rename_permit_no_replace(
      permit, permit.inflight_directory_fd.get(),
      permit.spent_directory_fd.get(),
      "J2_LAUNCH_PERMIT_SPEND_FAILED");
  permit.state = "SPENT";
}

double reference_for_recovery_branch(double unwrapped,
                                     const MotorRuntime& motor) {
  const double desired_reference =
      unwrapped - motor.sign * kGear * motor.recovery_hint;
  const double reference = motor.persistent_reference +
      std::round((desired_reference - motor.persistent_reference) /
                 (2.0 * kPi)) * (2.0 * kPi);
  // A prior-power persistent value is usable only when its single-turn phase
  // agrees with the phase implied by the trusted logical hint.  std::remainder
  // is always within [-pi, pi], so comparing it with pi is not a safety check.
  if (std::abs(std::remainder(
          desired_reference - motor.persistent_reference, 2.0 * kPi)) >
      kPersistentPhaseMismatchLimit)
    throw std::runtime_error("RECOVERY_HINT_BRANCH_INCONSISTENT");
  return reference;
}

double reference_for_session_hint(double unwrapped, const MotorRuntime& motor,
                                  double hint, double maximum_offset) {
  if (!motor.session_reference_configured)
    throw std::runtime_error("J2_SESSION_REFERENCE_NOT_CONFIGURED");
  const double desired_reference = unwrapped -
      motor.sign * kGear * hint;
  const double reference = motor.session_reference +
      std::round((desired_reference - motor.session_reference) /
                 (2.0 * kPi)) * (2.0 * kPi);
  const double recovered =
      motor.sign * (unwrapped - reference) / kGear;
  if (!std::isfinite(recovered) ||
      std::abs(recovered - hint) > maximum_offset)
    throw std::runtime_error("J2_SESSION_REFERENCE_STARTUP_MISMATCH");
  return reference;
}

double reference_for_j2_session(double unwrapped, const MotorRuntime& motor) {
  return reference_for_session_hint(unwrapped, motor,
      motor.supported_near_vertical_recovery ? motor.session_startup_logical_position
                                           : motor.session_logical_position,
      kJ2SessionStartupTolerance);
}

struct Feedback {
  bool send_recv_returned = false;
  bool send_recv_threw = false;
  bool transport_ok = false;
  bool identity_ok = false;
  // `continuity_valid` deliberately excludes drive-temperature and merror.
  // Those are live actuation interlocks; counting them as missing identity
  // incorrectly turns a thermally healthy serial session into
  // POWER_CONTINUITY_LOST and kills the feedback publisher.
  bool continuity_valid = false;
  bool actuation_safe = false;
  MotorData data;
};

struct InvalidFeedbackSnapshot {
  std::uint64_t cycle = 0;
  int sent_mode = -1;
  int returned_mode = -1;
  bool transport_ok = false;
  bool identity_ok = false;
  bool sdk_correct = false;
  int returned_id = -1;
  int merror = -1;
  int temperature = std::numeric_limits<int>::min();
  double q = std::numeric_limits<double>::quiet_NaN();
  double dq = std::numeric_limits<double>::quiet_NaN();
  double tau = std::numeric_limits<double>::quiet_NaN();
  int consecutive_before = 0;
};

struct InvalidFeedbackLogState {
  bool active = false;
  std::uint64_t total_count = 0;
  std::uint64_t episode_count = 0;
  std::uint64_t transition_count = 0;
  std::uint64_t pending_count = 0;
  std::uint64_t pending_transition_count = 0;
  Clock::time_point last_report_at{};
  bool pending_snapshot = false;
  InvalidFeedbackSnapshot first;
  InvalidFeedbackSnapshot last;
};

InvalidFeedbackSnapshot invalid_feedback_snapshot(
    std::uint64_t cycle, int sent_mode, int consecutive_before,
    const Feedback& feedback) {
  return {
      cycle, sent_mode, static_cast<int>(feedback.data.mode),
      feedback.transport_ok, feedback.identity_ok, feedback.data.correct,
      static_cast<int>(feedback.data.motor_id), feedback.data.merror,
      feedback.data.temp, feedback.data.q, feedback.data.dq, feedback.data.tau,
      consecutive_before};
}

void append_invalid_feedback_snapshot(
    const char* label, const InvalidFeedbackSnapshot& snapshot) {
  std::cerr << ' ' << label << "_cycle=" << snapshot.cycle
            << ' ' << label << "_sent_mode=" << snapshot.sent_mode
            << ' ' << label << "_returned_mode=" << snapshot.returned_mode
            << ' ' << label << "_transport_ok=" << snapshot.transport_ok
            << ' ' << label << "_identity_ok=" << snapshot.identity_ok
            << ' ' << label << "_sdk_correct=" << snapshot.sdk_correct
            << ' ' << label << "_returned_id=" << snapshot.returned_id
            << ' ' << label << "_merror=" << snapshot.merror
            << ' ' << label << "_temp=" << snapshot.temperature
            << ' ' << label << "_q=" << snapshot.q
            << ' ' << label << "_dq=" << snapshot.dq
            << ' ' << label << "_tau=" << snapshot.tau
            << ' ' << label << "_consecutive_before="
            << snapshot.consecutive_before;
}

void log_invalid_feedback_episode(
    const char* event, const std::string& motor_name,
    const InvalidFeedbackLogState& state) {
  std::cerr << event
            << " motor=" << motor_name
            << " active=" << state.active
            << " episode_count=" << state.episode_count
            << " interval_invalid_count=" << state.pending_count
            << " interval_transition_count="
            << state.pending_transition_count
            << " total_invalid_count=" << state.total_count
            << " total_transition_count=" << state.transition_count;
  if (state.pending_snapshot) {
    append_invalid_feedback_snapshot("first", state.first);
    append_invalid_feedback_snapshot("last", state.last);
  }
  std::cerr << std::endl;
}

bool diagnostic_log_interval_elapsed(
    Clock::time_point now, Clock::time_point last_report_at) {
  return last_report_at == Clock::time_point{} ||
      std::chrono::duration<double>(now - last_report_at).count() >=
          kCommandRejectSummarySeconds;
}

void flush_invalid_feedback_summary(
    const char* event, const std::string& motor_name,
    InvalidFeedbackLogState& state, Clock::time_point now, bool final) {
  if (state.pending_count == 0U && state.pending_transition_count == 0U)
    return;
  if (!final && !diagnostic_log_interval_elapsed(now, state.last_report_at))
    return;
  log_invalid_feedback_episode(event, motor_name, state);
  state.pending_count = 0U;
  state.pending_transition_count = 0U;
  state.pending_snapshot = false;
  state.last_report_at = now;
}

struct DomainBrakeLogState {
  std::uint64_t total_count = 0;
  std::uint64_t pending_count = 0;
  Clock::time_point last_report_at{};
  std::string last_reason;
};

void record_domain_brake_event(
    const std::string& bus, const std::string& reason,
    DomainBrakeLogState& state, Clock::time_point now) {
  ++state.total_count;
  state.last_reason = reason;
  if (state.total_count == 1U) {
    state.last_report_at = now;
    std::cerr << "OWNED_DOMAIN_BRAKE_BEGIN"
              << " bus=" << bus
              << " reason=" << reason
              << " interval_count=1 total_count=1"
              << std::endl;
    return;
  }
  ++state.pending_count;
  if (!diagnostic_log_interval_elapsed(now, state.last_report_at)) return;
  std::cerr << "OWNED_DOMAIN_BRAKE_SUMMARY"
            << " bus=" << bus
            << " last_reason=" << state.last_reason
            << " interval_count=" << state.pending_count
            << " total_count=" << state.total_count
            << std::endl;
  state.pending_count = 0U;
  state.last_report_at = now;
}

void flush_domain_brake_summary(
    const std::string& bus, DomainBrakeLogState& state,
    Clock::time_point now) {
  if (state.pending_count == 0U) return;
  std::cerr << "OWNED_DOMAIN_BRAKE_FINAL"
            << " bus=" << bus
            << " last_reason=" << state.last_reason
            << " interval_count=" << state.pending_count
            << " total_count=" << state.total_count
            << std::endl;
  state.pending_count = 0U;
  state.last_report_at = now;
}

struct ActiveCommandBlockedSnapshot {
  std::uint64_t cycle = 0;
  bool lease_fresh = false;
  bool lease_safe_hold = false;
  bool domain_fault = false;
  bool j2_sync_fault = false;
  bool j2_pair_ready = false;
  std::uint64_t activation_epoch = 0;
  std::uint64_t minimum_activation_epoch = 0;
  std::array<bool, 6> mask{};
  std::array<bool, 6> reference_ready{};
  std::array<bool, 6> valid{};
  std::array<bool, 6> fault_latched{};
};

struct ActiveCommandBlockedLogState {
  bool active = false;
  std::uint64_t total_count = 0;
  std::uint64_t episode_count = 0;
  std::uint64_t transition_count = 0;
  std::uint64_t pending_count = 0;
  std::uint64_t pending_transition_count = 0;
  Clock::time_point last_report_at{};
  bool pending_snapshot = false;
  ActiveCommandBlockedSnapshot first;
  ActiveCommandBlockedSnapshot last;
};

void append_active_command_blocked_snapshot(
    const char* label, const ActiveCommandBlockedSnapshot& snapshot,
    const std::vector<MotorRuntime>& motors) {
  std::cerr << ' ' << label << "_cycle=" << snapshot.cycle
            << ' ' << label << "_lease_fresh=" << snapshot.lease_fresh
            << ' ' << label << "_lease_safe_hold=" << snapshot.lease_safe_hold
            << ' ' << label << "_domain_fault=" << snapshot.domain_fault
            << ' ' << label << "_j2_sync_fault=" << snapshot.j2_sync_fault
            << ' ' << label << "_j2_pair_ready=" << snapshot.j2_pair_ready
            << ' ' << label << "_activation_epoch="
            << snapshot.activation_epoch
            << ' ' << label << "_minimum_activation_epoch="
            << snapshot.minimum_activation_epoch;
  for (const auto& motor : motors) {
    const auto joint = static_cast<std::size_t>(motor.joint_index);
    std::cerr << ' ' << label << "_motor=" << motor.name
              << ",mask=" << snapshot.mask[joint]
              << ",reference=" << snapshot.reference_ready[joint]
              << ",valid=" << snapshot.valid[joint]
              << ",latched=" << snapshot.fault_latched[joint];
  }
}

void log_active_command_blocked_episode(
    const char* event, const std::string& bus,
    const ActiveCommandBlockedLogState& state,
    const std::vector<MotorRuntime>& motors) {
  std::cerr << event
            << " bus=" << bus
            << " active=" << state.active
            << " episode_count=" << state.episode_count
            << " interval_blocked_count=" << state.pending_count
            << " interval_transition_count="
            << state.pending_transition_count
            << " total_blocked_count=" << state.total_count
            << " total_transition_count=" << state.transition_count;
  if (state.pending_snapshot) {
    append_active_command_blocked_snapshot("first", state.first, motors);
    append_active_command_blocked_snapshot("last", state.last, motors);
  }
  std::cerr << std::endl;
}

void flush_active_command_blocked_summary(
    const char* event, const std::string& bus,
    ActiveCommandBlockedLogState& state,
    const std::vector<MotorRuntime>& motors,
    Clock::time_point now, bool final) {
  if (state.pending_count == 0U &&
      state.pending_transition_count == 0U)
    return;
  if (!final && !diagnostic_log_interval_elapsed(now, state.last_report_at))
    return;
  log_active_command_blocked_episode(event, bus, state, motors);
  state.pending_count = 0U;
  state.pending_transition_count = 0U;
  state.pending_snapshot = false;
  state.last_report_at = now;
}

struct TxAudit {
  explicit TxAudit(bool brake_only_value) : brake_only(brake_only_value) {}
  bool brake_only = false;
  std::uint64_t tx_attempt_total = 0;
  std::uint64_t brake_tx_attempt_count = 0;
  std::uint64_t foc_tx_attempt_count = 0;
  std::uint64_t other_mode_tx_attempt_count = 0;
  std::uint64_t brake_only_guard_block_count = 0;
  std::uint64_t serial_send_call_count = 0;
  std::uint64_t foc_serial_send_call_count = 0;
};

int audit_tx_before_serial_send(
    MotorCmd& command, int expected_mode, TxAudit& audit) {
  ++audit.tx_attempt_total;
  const auto* raw = command.get_motor_send_data();
  const int encoded_mode = raw == nullptr
      ? -1 : static_cast<int>((raw[2] >> 4U) & 0x07U);
  if (encoded_mode == kBrakeMode) ++audit.brake_tx_attempt_count;
  else if (encoded_mode == kFocMode) ++audit.foc_tx_attempt_count;
  else ++audit.other_mode_tx_attempt_count;
  if (raw == nullptr || static_cast<int>(command.mode) != encoded_mode ||
      expected_mode != encoded_mode) {
    if (audit.brake_only) ++audit.brake_only_guard_block_count;
    throw std::runtime_error("TX_WIRE_MODE_AUDIT_FAILED");
  }
  if (audit.brake_only && encoded_mode != kBrakeMode) {
    ++audit.brake_only_guard_block_count;
    throw std::runtime_error("BRAKE_ONLY_NON_BRAKE_BLOCKED");
  }
  return encoded_mode;
}

Feedback transact(SerialPort& serial, MotorCmd& command, int expected_id,
                  int expected_mode, TxAudit& audit) {
  const int encoded_mode =
      audit_tx_before_serial_send(command, expected_mode, audit);
  ++audit.serial_send_call_count;
  if (encoded_mode == kFocMode) ++audit.foc_serial_send_call_count;
  Feedback result;
  zero_object(result.data);
  result.data.motorType = MotorType::GO_M8010_6;
  result.data.motor_id = 0xffU;
  result.data.mode = 0xffU;
  result.data.temp = std::numeric_limits<int>::min();
  result.data.merror = -1;
  result.data.q = result.data.dq = result.data.tau =
      std::numeric_limits<float>::quiet_NaN();
  result.data.correct = false;
  try {
    result.send_recv_returned = serial.sendRecv(&command, &result.data);
  } catch (...) {
    result.send_recv_threw = true;
  }
  result.transport_ok = result.send_recv_returned && result.data.correct;
  result.identity_ok = result.transport_ok &&
      static_cast<int>(result.data.motor_id) == expected_id;
  result.continuity_valid = result.identity_ok &&
      static_cast<int>(result.data.mode) == expected_mode &&
      std::isfinite(result.data.q) &&
      std::isfinite(result.data.dq) && std::isfinite(result.data.tau);
  result.actuation_safe = result.continuity_valid &&
      result.data.merror == 0 && result.data.temp >= 0 &&
      result.data.temp < g_thermal_policy.thermal_stop_c;
  return result;
}

enum class J2StartupPrimePairState {
  kCommonColdNoReply,
  kHealthy,
  kInvalid,
};

struct J2StartupBrakePrimeEvidence {
  bool applicable = false;
  bool passed = false;
  int invalid_prefix_pairs = 0;
  int healthy_qualification_pairs = 0;
  int attempted_pairs = 0;
  std::uint64_t tx_attempt_count = 0U;
  std::uint64_t elapsed_ns = 0U;
};

bool is_cold_no_reply_feedback(const Feedback& feedback) {
  return !feedback.send_recv_returned && !feedback.send_recv_threw &&
      !feedback.transport_ok && !feedback.identity_ok &&
      !feedback.continuity_valid && !feedback.actuation_safe &&
      !feedback.data.correct &&
      static_cast<int>(feedback.data.motor_id) == 0xff &&
      static_cast<int>(feedback.data.mode) == 0xff &&
      feedback.data.merror == -1 &&
      feedback.data.temp == std::numeric_limits<int>::min() &&
      !std::isfinite(feedback.data.q) &&
      !std::isfinite(feedback.data.dq) &&
      !std::isfinite(feedback.data.tau);
}

J2StartupPrimePairState classify_j2_startup_prime_pair(
    const std::array<Feedback, 2>& feedback) {
  if (std::all_of(
          feedback.begin(), feedback.end(),
          [](const Feedback& sample) { return sample.actuation_safe; }))
    return J2StartupPrimePairState::kHealthy;
  if (std::all_of(
          feedback.begin(), feedback.end(), is_cold_no_reply_feedback))
    return J2StartupPrimePairState::kCommonColdNoReply;
  return J2StartupPrimePairState::kInvalid;
}

void observe_j2_startup_prime_pair_state(
    J2StartupBrakePrimeEvidence& evidence, bool& healthy_prefix_closed,
    J2StartupPrimePairState state) {
  if (state == J2StartupPrimePairState::kCommonColdNoReply) {
    if (healthy_prefix_closed)
      throw std::runtime_error(
          "J2_STARTUP_BRAKE_PRIME_INVALID_AFTER_HEALTHY");
    if (evidence.invalid_prefix_pairs >=
        kJ2StartupPrimeMaximumInvalidPrefixPairs)
      throw std::runtime_error(
          "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_TOO_LONG");
    ++evidence.invalid_prefix_pairs;
    return;
  }
  if (state != J2StartupPrimePairState::kHealthy)
    throw std::runtime_error(
        healthy_prefix_closed
            ? "J2_STARTUP_BRAKE_PRIME_INVALID_AFTER_HEALTHY"
            : "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_SIGNATURE");
  healthy_prefix_closed = true;
  ++evidence.healthy_qualification_pairs;
}

bool j2_startup_prime_policy_rejects(
    J2StartupBrakePrimeEvidence evidence, bool healthy_prefix_closed,
    J2StartupPrimePairState state) {
  try {
    observe_j2_startup_prime_pair_state(
        evidence, healthy_prefix_closed, state);
  } catch (const std::runtime_error&) {
    return true;
  }
  return false;
}

void j2_startup_prime_policy_self_test() {
  Feedback cold;
  zero_object(cold.data);
  cold.data.motor_id = 0xffU;
  cold.data.mode = 0xffU;
  cold.data.temp = std::numeric_limits<int>::min();
  cold.data.merror = -1;
  cold.data.q = cold.data.dq = cold.data.tau =
      std::numeric_limits<float>::quiet_NaN();
  cold.data.correct = false;
  Feedback healthy = cold;
  healthy.send_recv_returned = true;
  healthy.transport_ok = true;
  healthy.identity_ok = true;
  healthy.continuity_valid = true;
  healthy.actuation_safe = true;
  healthy.data.motor_id = 0U;
  healthy.data.mode = static_cast<decltype(healthy.data.mode)>(kBrakeMode);
  healthy.data.temp = 30;
  healthy.data.merror = 0;
  healthy.data.q = healthy.data.dq = healthy.data.tau = 0.0F;
  healthy.data.correct = true;
  Feedback threw = cold;
  threw.send_recv_threw = true;

  if (classify_j2_startup_prime_pair({cold, cold}) !=
          J2StartupPrimePairState::kCommonColdNoReply ||
      classify_j2_startup_prime_pair({healthy, healthy}) !=
          J2StartupPrimePairState::kHealthy ||
      classify_j2_startup_prime_pair({cold, healthy}) !=
          J2StartupPrimePairState::kInvalid ||
      classify_j2_startup_prime_pair({threw, threw}) !=
          J2StartupPrimePairState::kInvalid)
    throw std::runtime_error(
        "J2_STARTUP_BRAKE_PRIME_CLASSIFIER_SELF_TEST_FAILED");

  J2StartupBrakePrimeEvidence evidence;
  bool healthy_prefix_closed = false;
  for (int count = 0; count < kJ2StartupPrimeMaximumInvalidPrefixPairs;
       ++count)
    observe_j2_startup_prime_pair_state(
        evidence, healthy_prefix_closed,
        J2StartupPrimePairState::kCommonColdNoReply);
  if (evidence.invalid_prefix_pairs != 3 || healthy_prefix_closed ||
      !j2_startup_prime_policy_rejects(
          evidence, healthy_prefix_closed,
          J2StartupPrimePairState::kCommonColdNoReply))
    throw std::runtime_error(
        "J2_STARTUP_BRAKE_PRIME_PREFIX_SELF_TEST_FAILED");
  for (int count = 0; count < kJ2StartupPrimeRequiredHealthyPairs; ++count)
    observe_j2_startup_prime_pair_state(
        evidence, healthy_prefix_closed, J2StartupPrimePairState::kHealthy);
  if (!healthy_prefix_closed || evidence.healthy_qualification_pairs != 5 ||
      !j2_startup_prime_policy_rejects(
          evidence, healthy_prefix_closed,
          J2StartupPrimePairState::kCommonColdNoReply) ||
      !j2_startup_prime_policy_rejects(
          J2StartupBrakePrimeEvidence{}, false,
          J2StartupPrimePairState::kInvalid))
    throw std::runtime_error(
        "J2_STARTUP_BRAKE_PRIME_CLOSED_WINDOW_SELF_TEST_FAILED");
}

std::uint64_t elapsed_ns_since(Clock::time_point started_at) {
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now() - started_at).count();
  return elapsed > 0 ? static_cast<std::uint64_t>(elapsed) : 0U;
}

J2StartupBrakePrimeEvidence prime_j2_serial_before_feedback(
    SerialPort& serial, std::vector<MotorRuntime>& motors, TxAudit& audit) {
  if (motors.size() != 2U || motors[0].name != "J2A" ||
      motors[1].name != "J2B")
    throw std::runtime_error("J2_STARTUP_BRAKE_PRIME_MOTOR_SET_INVALID");

  J2StartupBrakePrimeEvidence evidence;
  evidence.applicable = true;
  const std::uint64_t tx_attempts_before = audit.tx_attempt_total;
  const auto started_at = Clock::now();
  bool healthy_prefix_closed = false;
  std::array<std::deque<double>, 2> healthy_position_windows;
  while (evidence.healthy_qualification_pairs <
         kJ2StartupPrimeRequiredHealthyPairs) {
    if (g_stop.load())
      throw std::runtime_error("J2_STARTUP_BRAKE_PRIME_INTERRUPTED");
    if (elapsed_ns_since(started_at) > kJ2StartupPrimeMaximumElapsedNs)
      throw std::runtime_error("J2_STARTUP_BRAKE_PRIME_TIMEOUT");

    std::array<Feedback, 2> feedback;
    for (std::size_t index = 0; index < feedback.size(); ++index) {
      motors[index].last_tau_cmd_rotor_nm = 0.0;
      MotorCmd brake = make_command(
          motors[index].id, kBrakeMode, 0.0, 0.0, 0.0, 0.0);
      feedback[index] = transact(
          serial, brake, motors[index].id, kBrakeMode, audit);
    }
    ++evidence.attempted_pairs;

    const J2StartupPrimePairState state =
        classify_j2_startup_prime_pair(feedback);
    observe_j2_startup_prime_pair_state(
        evidence, healthy_prefix_closed, state);
    if (state == J2StartupPrimePairState::kCommonColdNoReply) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    for (std::size_t index = 0; index < feedback.size(); ++index) {
      auto& motor = motors[index];
      const auto& sample = feedback[index].data;
      motor.last_q = sample.q;
      motor.last_dq = sample.dq;
      motor.last_tau = sample.tau;
      motor.last_valid_feedback_monotonic_ns = monotonic_ns();
      motor.temperature = sample.temp;
      motor.merror = sample.merror;
      motor.returned_mode = sample.mode;
      motor.unwrapped = motor.unwrap.update(motor.last_q);
      motor.last_frame_valid = true;
      motor.valid = true;
      motor.consecutive_invalid = 0;
      healthy_position_windows[index].push_back(motor.unwrapped);
    }
    if (evidence.healthy_qualification_pairs <
        kJ2StartupPrimeRequiredHealthyPairs)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  evidence.elapsed_ns = elapsed_ns_since(started_at);
  if (evidence.elapsed_ns == 0U ||
      evidence.elapsed_ns > kJ2StartupPrimeMaximumElapsedNs)
    throw std::runtime_error("J2_STARTUP_BRAKE_PRIME_TIMEOUT");
  for (const auto& positions : healthy_position_windows) {
    if (positions.size() !=
        static_cast<std::size_t>(kJ2StartupPrimeRequiredHealthyPairs))
      throw std::runtime_error("J2_STARTUP_BRAKE_PRIME_HISTORY_INCOMPLETE");
    const auto minmax = std::minmax_element(
        positions.begin(), positions.end());
    if (*minmax.second - *minmax.first >
        kGear * kBrakeStationaritySpan + 1e-12)
      throw std::runtime_error("J2_STARTUP_BRAKE_PRIME_NOT_STATIONARY");
  }
  evidence.tx_attempt_count = audit.tx_attempt_total - tx_attempts_before;
  if (evidence.tx_attempt_count !=
      2U * static_cast<std::uint64_t>(evidence.attempted_pairs))
    throw std::runtime_error("J2_STARTUP_BRAKE_PRIME_TX_ACCOUNTING_FAILED");
  evidence.passed = true;
  return evidence;
}

double median(const std::deque<double>& values) {
  std::vector<double> sorted(values.begin(), values.end());
  std::sort(sorted.begin(), sorted.end());
  const std::size_t middle = sorted.size() / 2U;
  return sorted.size() % 2U ? sorted[middle]
                            : 0.5 * (sorted[middle - 1U] + sorted[middle]);
}

bool moving_velocity_guard_tripped(
    const std::string& bus, MotorRuntime& motor, double speed_degrees) {
  bool tripped = false;
  // This guard is called only while a normal POSITION profile is advancing.
  // Fixed HOLD/non-moving and endpoint-phase axes bypass it in
  // should_apply_velocity_guard(), so an external push cannot withdraw
  // holding torque, including when gravity temporarily delays physical
  // arrival. Keep the stricter runaway thresholds for the commanded profile.
  if (bus == "j1") {
    motor.fast_speed_count = speed_degrees > 35.0
        ? motor.fast_speed_count + 1 : 0;
    motor.slow_speed_count = speed_degrees > 30.0
        ? motor.slow_speed_count + 1 : 0;
    if (speed_degrees >= 40.0 || motor.fast_speed_count >= 2 ||
        motor.slow_speed_count >= 10 || std::abs(motor.last_tau) >= 5.0)
      tripped = true;
  } else if (bus == "j2") {
    if (speed_degrees > 20.0) tripped = true;
  } else if (motor.name == "J3") {
    if (speed_degrees > 25.0) tripped = true;
  } else {
    motor.slow_speed_count = speed_degrees > 30.0
        ? motor.slow_speed_count + 1 : 0;
    if (speed_degrees >= 40.0 || motor.slow_speed_count >= 3)
      tripped = true;
  }
  return tripped;
}

bool apply_velocity_guards(const std::string& bus, MotorRuntime& motor,
                           double logical_velocity) {
  const double speed_degrees = std::abs(logical_velocity) * 180.0 / kPi;
  const bool tripped = moving_velocity_guard_tripped(
      bus, motor, speed_degrees);
  if (tripped && !motor.velocity_degraded) {
    std::cerr << "EXTERNAL_MOTION_OBSERVED"
              << " bus=" << bus
              << " motor=" << motor.name
              << " logical_speed_deg_s=" << speed_degrees
              << " fast_count=" << motor.fast_speed_count
              << " slow_count=" << motor.slow_speed_count
              << std::endl;
    motor.velocity_degraded = true;
  } else if (!tripped && motor.velocity_degraded) {
    std::cerr << "EXTERNAL_MOTION_SETTLED"
              << " bus=" << bus
              << " motor=" << motor.name
              << std::endl;
    motor.velocity_degraded = false;
  }
  return tripped;
}

struct QuinticTrajectoryDescriptor {
  bool present = false;
  std::string plan_token_id;
  std::string trajectory_sha256;
  std::array<double, 6> start_rad{};
  std::array<double, 6> target_rad{};
  std::uint64_t duration_ns = 0;
  std::uint64_t interval_count = 0;
  std::uint64_t execute_at_monotonic_ns = 0;
  std::int64_t segment_index = 0;
  std::int64_t segment_count = 0;
};

bool same_quintic_trajectory_descriptor(
    const QuinticTrajectoryDescriptor& first,
    const QuinticTrajectoryDescriptor& second) {
  return first.present == second.present &&
      first.plan_token_id == second.plan_token_id &&
      first.trajectory_sha256 == second.trajectory_sha256 &&
      first.start_rad == second.start_rad &&
      first.target_rad == second.target_rad &&
      first.duration_ns == second.duration_ns &&
      first.interval_count == second.interval_count &&
      first.execute_at_monotonic_ns == second.execute_at_monotonic_ns &&
      first.segment_index == second.segment_index &&
      first.segment_count == second.segment_count;
}

struct GravityCommandAuthority {
  bool present = false;
  bool official_continuous_authority = false;
  std::string source_instance_id;
  std::uint64_t sequence = 0;
  std::uint64_t source_monotonic_ns = 0;
  std::string session_id;
  std::string state_instance_id;
  std::string authority_class;
  std::string rating_classification;
  std::string empirical_envelope_id;
  std::string empirical_envelope_sha256;
  std::string empirical_envelope_expires_at_utc;
  std::uint64_t empirical_envelope_deadline_monotonic_ns = 0;
  std::string anchor_sha256;
  std::uint64_t empirical_stage_index = 0;
  bool empirical_position_validation_authorized = false;
  double empirical_maximum_position_segment_seconds = 0.0;
  double empirical_maximum_abs_position_segment_deg = 0.0;
  bool empirical_assisted_teach_authorized = false;
  bool hand_guidance_profile = false;
  double empirical_maximum_teach_excursion_deg = 0.0;
  double empirical_maximum_teach_seconds = 0.0;
  double empirical_maximum_teach_velocity_deg_s = 0.0;
  double gravity_scale = 0.0;
  double gravity_scale_target = 0.0;
  std::array<double, 6> feedforward_nm{};
};

bool empirical_expiry_is_future(const std::string& value) {
  if (value.size() < 20U || value.back() != 'Z' ||
      value.at(4) != '-' || value.at(7) != '-' || value.at(10) != 'T' ||
      value.at(13) != ':' || value.at(16) != ':')
    return false;
  std::tm parsed{};
  std::istringstream stream(value.substr(0U, 19U));
  stream >> std::get_time(&parsed, "%Y-%m-%dT%H:%M:%S");
  if (stream.fail()) return false;
  if (value.size() > 20U) {
    if (value.at(19) != '.') return false;
    for (std::size_t index = 20U; index + 1U < value.size(); ++index)
      if (value.at(index) < '0' || value.at(index) > '9') return false;
  }
  const std::time_t expires = timegm(&parsed);
  return expires > 0 && std::time(nullptr) < expires;
}

struct HandGuidanceReference {
  bool present = false;
  bool freeze_reference = false;
  double maximum_excursion_deg = 10.0;
  std::array<double, 6> origin_rad{};
  std::array<double, 6> velocity_rad_s{};
};

struct GuidanceReferenceRejected : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct GuiCommand {
  std::string schema = "go-m8010-gui-command/1.0";
  std::string mode = "brake";
  std::string source_instance_id;
  std::uint64_t source_monotonic_ns = 0;
  std::uint64_t source_sequence = 0;
  std::array<double, 6> targets{};
  std::array<bool, 6> active_joint_mask{};
  std::array<bool, 6> moving_joint_mask{};
  std::uint64_t activation_epoch = 0;
  std::array<double, 6> kp{{0.5, 1.0, 0.6, 0.5, 0.5, 0.0}};
  std::array<double, 6> kd{{0.05, 0.10, 0.05, 0.05, 0.05, 0.0}};
  std::array<double, 6> feedforward_nm{};
  GravityCommandAuthority gravity_authority;
  HandGuidanceReference hand_guidance;
  bool recovery = false;
  double vmax = 5.0 * kPi / 180.0;
  double amax = 20.0 * kPi / 180.0;
  QuinticTrajectoryDescriptor quintic;
  Clock::time_point received_at{};
  bool received = false;
};

std::uint64_t gravity_authority_maximum_age_ns(
    const GuiCommand& command, const GravityCommandAuthority& authority) {
  const bool zero_hold_transition =
      !authority.official_continuous_authority &&
      authority.authority_class == kEmpiricalAuthorityClass &&
      command.mode == "hold" &&
      std::none_of(
          command.moving_joint_mask.begin(), command.moving_joint_mask.end(),
          [](bool moving) { return moving; }) &&
      authority.empirical_stage_index == 0U &&
      std::abs(authority.gravity_scale) <= 1e-12 &&
      std::abs(authority.gravity_scale_target) <= 1e-12 &&
      std::all_of(
          authority.feedforward_nm.begin(), authority.feedforward_nm.end(),
          [](double value) { return std::abs(value) <= 1e-12; });
  return zero_hold_transition
      ? kEmpiricalZeroHoldTransitionMaximumAgeNs
      : kMaximumGravityAuthorityAgeNs;
}

bool command_uses_empirical_gravity_authority(const GuiCommand& command) {
  return command.gravity_authority.present &&
      !command.gravity_authority.official_continuous_authority;
}

bool hand_guidance_return_only(const GuiCommand& command) {
  const auto& authority = command.gravity_authority;
  return authority.present && authority.hand_guidance_profile &&
      authority.empirical_stage_index == 4U && authority.gravity_scale == 1.0 &&
      authority.gravity_scale_target == 1.0 && authority.empirical_position_validation_authorized &&
      !authority.empirical_assisted_teach_authorized;
}

bool empirical_gravity_authority_is_current(const GuiCommand& command) {
  return !command_uses_empirical_gravity_authority(command) ||
      (empirical_expiry_is_future(
           command.gravity_authority.empirical_envelope_expires_at_utc) &&
       monotonic_ns_at(Clock::now()) <
           command.gravity_authority
               .empirical_envelope_deadline_monotonic_ns);
}

std::uint64_t strict_positive_uint64(
    const nlohmann::json& value, const char* error) {
  if (!value.is_number_unsigned()) throw std::runtime_error(error);
  const std::uint64_t parsed = value.get<std::uint64_t>();
  if (parsed == 0U) throw std::runtime_error(error);
  return parsed;
}

std::int64_t strict_nonnegative_int64(
    const nlohmann::json& value, const char* error) {
  if (value.is_number_unsigned()) {
    const std::uint64_t parsed = value.get<std::uint64_t>();
    if (parsed > static_cast<std::uint64_t>(
                     std::numeric_limits<std::int64_t>::max()))
      throw std::runtime_error(error);
    return static_cast<std::int64_t>(parsed);
  }
  if (!value.is_number_integer()) throw std::runtime_error(error);
  const std::int64_t parsed = value.get<std::int64_t>();
  if (parsed < 0) throw std::runtime_error(error);
  return parsed;
}

std::array<double, 6> strict_finite_six_vector(
    const nlohmann::json& value, const char* size_error,
    const char* type_error) {
  if (!value.is_array() || value.size() != 6U)
    throw std::runtime_error(size_error);
  std::array<double, 6> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    if (!value.at(index).is_number()) throw std::runtime_error(type_error);
    result[index] = value.at(index).get<double>();
    if (!std::isfinite(result[index])) throw std::runtime_error(type_error);
  }
  return result;
}

bool approved_gravity_scale_target(double value) {
  constexpr std::array<double, 5> kLevels{{0.0, 0.25, 0.50, 0.75, 1.0}};
  return std::isfinite(value) &&
      std::any_of(kLevels.begin(), kLevels.end(), [&](double level) {
        return std::abs(value - level) <= 1e-12;
      });
}

GravityCommandAuthority parse_gravity_command_authority(
    const nlohmann::json& value, const GuiCommand& candidate,
    Clock::time_point received_at) {
  GravityCommandAuthority result;
  const bool nonzero_feedforward = std::any_of(
      candidate.feedforward_nm.begin(), candidate.feedforward_nm.end(),
      [](double item) { return std::abs(item) > 1e-12; });
  if (!value.contains("gravity_authority")) {
    if (nonzero_feedforward)
      throw std::runtime_error("COMMAND_GRAVITY_AUTHORITY_MISSING");
    return result;
  }
  if (candidate.mode != "hold" && candidate.mode != "position" && candidate.mode != "teach")
    throw std::runtime_error("COMMAND_GRAVITY_AUTHORITY_MODE_INVALID");
  const auto& authority = value.at("gravity_authority");
  constexpr std::array<const char*, 22> kFields{{
      "schema", "source_instance_id", "sequence", "source_monotonic_ns",
      "model_sha256", "gravity_config_sha256", "session_id",
      "state_instance_id", "gravity_scale", "gravity_scale_target",
      "feedforward_nm", "authority_class", "rating_classification",
      "empirical_envelope_id", "empirical_envelope_sha256",
      "empirical_envelope_expires_at_utc",
      "empirical_envelope_deadline_monotonic_ns", "anchor_sha256",
      "empirical_stage_index", "empirical_position_validation_authorized",
      "empirical_maximum_position_segment_seconds",
      "empirical_maximum_abs_position_segment_deg"}};
  constexpr std::array<const char*, 5> kTeachFields{{
      "empirical_assisted_teach_authorized", "empirical_maximum_teach_excursion_deg",
      "empirical_maximum_teach_seconds", "empirical_maximum_teach_velocity_deg_s",
      "empirical_allowed_teach_joints"}};
  if (!authority.is_object() || !authority.contains("schema") ||
      !authority.at("schema").is_string())
    throw std::runtime_error("COMMAND_GRAVITY_AUTHORITY_FIELDS_INVALID");
  const std::string authority_schema =
      authority.at("schema").get<std::string>();
  const bool empirical = authority_schema == kGravityAuthoritySchema;
  const bool official = authority_schema == kOfficialGravityAuthoritySchema;
  const bool extended_teach = empirical && authority.size() == kFields.size() + kTeachFields.size();
  if ((!empirical && !official) ||
      (!extended_teach && authority.size() != (empirical ? kFields.size() : 11U)))
    throw std::runtime_error("COMMAND_GRAVITY_AUTHORITY_FIELDS_INVALID");
  constexpr std::array<const char*, 11> kCommonFields{{
      "schema", "source_instance_id", "sequence", "source_monotonic_ns",
      "model_sha256", "gravity_config_sha256", "session_id",
      "state_instance_id", "gravity_scale", "gravity_scale_target",
      "feedforward_nm"}};
  for (const char* field : kCommonFields)
    if (!authority.contains(field))
      throw std::runtime_error("COMMAND_GRAVITY_AUTHORITY_FIELDS_INVALID");
  if (empirical)
    for (const char* field : kFields)
      if (!authority.contains(field))
        throw std::runtime_error("COMMAND_GRAVITY_AUTHORITY_FIELDS_INVALID");
  if (extended_teach) {
    for (const char* field : kTeachFields)
      if (!authority.contains(field))
        throw std::runtime_error("COMMAND_GRAVITY_TEACH_FIELDS_INVALID");
    result.hand_guidance_profile = authority.at("empirical_allowed_teach_joints") ==
        nlohmann::json::array({"J1", "J2", "J3", "J4", "J5", "J6"});
    if (!authority.at("empirical_assisted_teach_authorized").is_boolean() ||
        (!result.hand_guidance_profile && authority.at("empirical_allowed_teach_joints") !=
            nlohmann::json::array({"J1", "J2", "J3", "J4", "J5"})))
      throw std::runtime_error("COMMAND_GRAVITY_TEACH_SCOPE_INVALID");
    result.empirical_assisted_teach_authorized =
        authority.at("empirical_assisted_teach_authorized").get<bool>();
    for (const auto& field : std::array<std::pair<const char*, double*>, 3>{{
        {"empirical_maximum_teach_excursion_deg", &result.empirical_maximum_teach_excursion_deg},
        {"empirical_maximum_teach_seconds", &result.empirical_maximum_teach_seconds},
        {"empirical_maximum_teach_velocity_deg_s", &result.empirical_maximum_teach_velocity_deg_s}}}) {
      if (!authority.at(field.first).is_number())
        throw std::runtime_error("COMMAND_GRAVITY_TEACH_BOUND_INVALID");
      *field.second = authority.at(field.first).get<double>();
      const std::string name(field.first);
      const double hard = result.hand_guidance_profile
          ? name == "empirical_maximum_teach_seconds" ? 600.0
            : name == "empirical_maximum_teach_excursion_deg" ? 20.0 : 30.0
          : name == "empirical_maximum_teach_seconds" ? 30.0 : 5.0;
      if (!std::isfinite(*field.second) || *field.second <= 0.0 || *field.second > hard ||
          (result.hand_guidance_profile && *field.second != hard &&
           !(name == "empirical_maximum_teach_excursion_deg" && *field.second == 10.0)))
        throw std::runtime_error("COMMAND_GRAVITY_TEACH_BOUND_INVALID");
    }
  }
  if (
      !authority.at("model_sha256").is_string() ||
      authority.at("model_sha256").get<std::string>() !=
          kProductionModelSha256 ||
      !authority.at("gravity_config_sha256").is_string() ||
      authority.at("gravity_config_sha256").get<std::string>() !=
          kGravityConfigSha256)
    throw std::runtime_error("COMMAND_GRAVITY_AUTHORITY_HASH_INVALID");
  result.official_continuous_authority = official;
  const std::uint64_t received_ns = monotonic_ns_at(received_at);
  if (empirical) {
  if (!authority.at("authority_class").is_string() ||
      authority.at("authority_class").get<std::string>() !=
          kEmpiricalAuthorityClass ||
      !authority.at("rating_classification").is_string() ||
      authority.at("rating_classification").get<std::string>() !=
          kEmpiricalRatingClassification)
    throw std::runtime_error("COMMAND_GRAVITY_EMPIRICAL_CLASS_INVALID");
  result.authority_class =
      authority.at("authority_class").get<std::string>();
  result.rating_classification =
      authority.at("rating_classification").get<std::string>();
  if (!authority.at("empirical_envelope_id").is_string() ||
      !authority.at("empirical_envelope_sha256").is_string() ||
      !authority.at("empirical_envelope_expires_at_utc").is_string() ||
      !authority.at("anchor_sha256").is_string())
    throw std::runtime_error("COMMAND_GRAVITY_EMPIRICAL_IDENTITY_INVALID");
  result.empirical_envelope_id =
      authority.at("empirical_envelope_id").get<std::string>();
  result.empirical_envelope_sha256 =
      authority.at("empirical_envelope_sha256").get<std::string>();
  result.empirical_envelope_expires_at_utc =
      authority.at("empirical_envelope_expires_at_utc").get<std::string>();
  result.empirical_envelope_deadline_monotonic_ns = strict_positive_uint64(
      authority.at("empirical_envelope_deadline_monotonic_ns"),
      "COMMAND_GRAVITY_EMPIRICAL_DEADLINE_INVALID");
  result.anchor_sha256 = authority.at("anchor_sha256").get<std::string>();
  if (result.empirical_envelope_id.size() != 38U ||
      result.empirical_envelope_id.rfind("v15-31b-empirical-", 0U) != 0U ||
      !std::all_of(
          result.empirical_envelope_id.begin() + 18,
          result.empirical_envelope_id.end(), [](char character) {
            return (character >= '0' && character <= '9') ||
                (character >= 'a' && character <= 'f');
          }) ||
      !valid_sha256(result.empirical_envelope_sha256) ||
      !valid_sha256(result.anchor_sha256) ||
      !empirical_expiry_is_future(
          result.empirical_envelope_expires_at_utc))
    throw std::runtime_error("COMMAND_GRAVITY_EMPIRICAL_IDENTITY_INVALID");
  constexpr std::uint64_t kMaximumEmpiricalLifetimeNs =
      4200ULL * 1000ULL * 1000ULL * 1000ULL;
  if (result.empirical_envelope_deadline_monotonic_ns <= received_ns ||
      result.empirical_envelope_deadline_monotonic_ns - received_ns >
          kMaximumEmpiricalLifetimeNs)
    throw std::runtime_error(
        "COMMAND_GRAVITY_EMPIRICAL_DEADLINE_INVALID");
  result.empirical_stage_index = strict_nonnegative_int64(
      authority.at("empirical_stage_index"),
      "COMMAND_GRAVITY_EMPIRICAL_STAGE_INVALID");
  if (result.empirical_stage_index >= 5U)
    throw std::runtime_error("COMMAND_GRAVITY_EMPIRICAL_STAGE_INVALID");
  if (!authority.at("empirical_position_validation_authorized").is_boolean() ||
      !authority.at("empirical_maximum_position_segment_seconds").is_number() ||
      !authority.at("empirical_maximum_abs_position_segment_deg").is_number())
    throw std::runtime_error("COMMAND_GRAVITY_EMPIRICAL_POSITION_INVALID");
  result.empirical_position_validation_authorized =
      authority.at("empirical_position_validation_authorized").get<bool>();
  result.empirical_maximum_position_segment_seconds =
      authority.at("empirical_maximum_position_segment_seconds").get<double>();
  result.empirical_maximum_abs_position_segment_deg =
      authority.at("empirical_maximum_abs_position_segment_deg").get<double>();
  if (!std::isfinite(result.empirical_maximum_position_segment_seconds) ||
      !std::isfinite(result.empirical_maximum_abs_position_segment_deg) ||
      std::abs(result.empirical_maximum_position_segment_seconds - 15.0) >
          1e-12 ||
      std::abs(result.empirical_maximum_abs_position_segment_deg - 5.0) >
          1e-12 ||
      (result.empirical_position_validation_authorized &&
       result.empirical_stage_index != 4U))
    throw std::runtime_error("COMMAND_GRAVITY_EMPIRICAL_POSITION_INVALID");
  }
  if (!authority.at("source_instance_id").is_string())
    throw std::runtime_error("COMMAND_GRAVITY_SOURCE_INVALID");
  result.source_instance_id =
      authority.at("source_instance_id").get<std::string>();
  if (!valid_source_instance_id(result.source_instance_id))
    throw std::runtime_error("COMMAND_GRAVITY_SOURCE_INVALID");
  result.sequence = strict_positive_uint64(
      authority.at("sequence"), "COMMAND_GRAVITY_SEQUENCE_INVALID");
  if (result.sequence > kMaximumCommandSequence)
    throw std::runtime_error("COMMAND_GRAVITY_SEQUENCE_INVALID");
  result.source_monotonic_ns = strict_positive_uint64(
      authority.at("source_monotonic_ns"),
      "COMMAND_GRAVITY_TIMESTAMP_INVALID");
  if (!authority.at("session_id").is_string() ||
      !authority.at("state_instance_id").is_string())
    throw std::runtime_error("COMMAND_GRAVITY_SESSION_INVALID");
  result.session_id = authority.at("session_id").get<std::string>();
  result.state_instance_id =
      authority.at("state_instance_id").get<std::string>();
  if (result.session_id.empty() || result.session_id.size() > 512U ||
      result.state_instance_id.empty() ||
      result.state_instance_id.size() > 512U)
    throw std::runtime_error("COMMAND_GRAVITY_SESSION_INVALID");
  if (!authority.at("gravity_scale").is_number() ||
      !authority.at("gravity_scale_target").is_number())
    throw std::runtime_error("COMMAND_GRAVITY_SCALE_INVALID");
  result.gravity_scale = authority.at("gravity_scale").get<double>();
  result.gravity_scale_target =
      authority.at("gravity_scale_target").get<double>();
  if (!std::isfinite(result.gravity_scale) ||
      result.gravity_scale < 0.0 || result.gravity_scale > 1.0 ||
      !approved_gravity_scale_target(result.gravity_scale_target))
    throw std::runtime_error("COMMAND_GRAVITY_SCALE_INVALID");
  constexpr std::array<double, 5> kEmpiricalLevels{{
      0.0, 0.25, 0.50, 0.75, 1.0}};
  if (empirical && std::abs(result.gravity_scale_target -
               kEmpiricalLevels[result.empirical_stage_index]) > 1e-12)
    throw std::runtime_error("COMMAND_GRAVITY_EMPIRICAL_STAGE_INVALID");
  if (candidate.mode == "teach" && (!extended_teach ||
      (!result.empirical_assisted_teach_authorized &&
       !(result.hand_guidance_profile && candidate.hand_guidance.present)) ||
      !result.empirical_position_validation_authorized ||
      result.empirical_stage_index != 4U ||
      std::abs(result.gravity_scale - 1.0) > 1e-6 ||
      std::abs(result.gravity_scale_target - 1.0) > 1e-12))
    throw std::runtime_error("COMMAND_GRAVITY_TEACH_AUTHORITY_MISSING");
  result.feedforward_nm = strict_finite_six_vector(
      authority.at("feedforward_nm"),
      "COMMAND_GRAVITY_FEEDFORWARD_SIZE_INVALID",
      "COMMAND_GRAVITY_FEEDFORWARD_VALUE_INVALID");
  for (std::size_t joint = 0; joint < result.feedforward_nm.size(); ++joint) {
    if (result.feedforward_nm[joint] != candidate.feedforward_nm[joint] ||
        std::abs(result.feedforward_nm[joint]) >
            kGravityFeedforwardLimits[joint] + 1e-12)
      throw std::runtime_error("COMMAND_GRAVITY_FEEDFORWARD_LIMIT");
  }
  if (std::abs(result.feedforward_nm[5]) > 1e-12)
    throw std::runtime_error("COMMAND_GRAVITY_J6_MUST_BE_ZERO");
  if (result.source_monotonic_ns > received_ns ||
      received_ns - result.source_monotonic_ns >
          gravity_authority_maximum_age_ns(candidate, result))
    throw std::runtime_error("COMMAND_GRAVITY_TIMESTAMP_STALE");
  const auto& expected = g_expected_gravity_authority_binding;
  const bool startup_binding_enforced =
      expected.authority_class != "UNBOUND_SELF_TEST";
  if (startup_binding_enforced) {
    if (expected.authority_class == "NONE")
      throw std::runtime_error(
          "COMMAND_GRAVITY_STARTUP_BINDING_NOT_AUTHORIZED");
    if ((official && expected.authority_class !=
                         "OFFICIAL_CONTINUOUS_RATING") ||
        (empirical && expected.authority_class !=
                          kEmpiricalAuthorityClass))
      throw std::runtime_error(
          "COMMAND_GRAVITY_STARTUP_CLASS_MISMATCH");
    if (result.session_id != expected.session_id ||
        result.state_instance_id != expected.state_instance_id)
      throw std::runtime_error(
          "COMMAND_GRAVITY_STARTUP_SESSION_MISMATCH");
    if (empirical &&
        (result.empirical_envelope_id != expected.empirical_envelope_id ||
         result.empirical_envelope_sha256 !=
             expected.empirical_envelope_sha256 ||
         result.anchor_sha256 != expected.anchor_sha256))
      throw std::runtime_error(
          "COMMAND_GRAVITY_STARTUP_EMPIRICAL_BINDING_MISMATCH");
  }
  result.present = true;
  return result;
}

QuinticTrajectoryDescriptor parse_quintic_trajectory_descriptor(
    const nlohmann::json& command, const GuiCommand& candidate) {
  const auto& plan_token_id = command.at("plan_token_id");
  if (!plan_token_id.is_string() ||
      !valid_sha256(plan_token_id.get<std::string>()))
    throw std::runtime_error("COMMAND_PLAN_TOKEN_ID_INVALID");
  const auto& trajectory = command.at("trajectory");
  constexpr std::array<const char*, 10> kRequiredFields{{
      "schema", "trajectory_sha256", "profile", "start_rad",
      "target_rad", "duration_ns", "interval_count",
      "execute_at_monotonic_ns", "segment_index", "segment_count"}};
  if (!trajectory.is_object() || trajectory.size() != kRequiredFields.size())
    throw std::runtime_error("COMMAND_QUINTIC_FIELDS_INVALID");
  for (const char* field : kRequiredFields)
    if (!trajectory.contains(field))
      throw std::runtime_error("COMMAND_QUINTIC_FIELDS_INVALID");
  if (!trajectory.at("schema").is_string() ||
      trajectory.at("schema").get<std::string>() !=
          "go-m8010-quintic-command/1.0")
    throw std::runtime_error("COMMAND_QUINTIC_SCHEMA_MISMATCH");
  if (!trajectory.at("profile").is_string() ||
      trajectory.at("profile").get<std::string>() !=
          "quintic-rest-to-rest-v1")
    throw std::runtime_error("COMMAND_QUINTIC_PROFILE_MISMATCH");

  QuinticTrajectoryDescriptor result;
  result.present = true;
  result.plan_token_id = plan_token_id.get<std::string>();
  const auto& trajectory_sha256 = trajectory.at("trajectory_sha256");
  if (!trajectory_sha256.is_string() ||
      !valid_sha256(trajectory_sha256.get<std::string>()))
    throw std::runtime_error("COMMAND_TRAJECTORY_SHA256_INVALID");
  result.trajectory_sha256 = trajectory_sha256.get<std::string>();
  result.start_rad = strict_finite_six_vector(
      trajectory.at("start_rad"), "COMMAND_QUINTIC_START_SIZE_INVALID",
      "COMMAND_QUINTIC_START_VALUE_INVALID");
  result.target_rad = strict_finite_six_vector(
      trajectory.at("target_rad"), "COMMAND_QUINTIC_TARGET_SIZE_INVALID",
      "COMMAND_QUINTIC_TARGET_VALUE_INVALID");
  result.duration_ns = strict_positive_uint64(
      trajectory.at("duration_ns"), "COMMAND_QUINTIC_DURATION_INVALID");
  result.interval_count = strict_positive_uint64(
      trajectory.at("interval_count"),
      "COMMAND_QUINTIC_INTERVAL_COUNT_INVALID");
  if (result.interval_count > kMaximumQuinticIntervalCount)
    throw std::runtime_error("COMMAND_QUINTIC_INTERVAL_COUNT_INVALID");
  // The descriptor is the shared execution clock.  Require an exact integer
  // nanosecond grid so every worker advances through the same sample indices;
  // a quotient rounded down from a non-integral grid would otherwise leave a
  // potentially large, endpoint-only final step.
  if (result.duration_ns % result.interval_count != 0U)
    throw std::runtime_error("COMMAND_QUINTIC_SAMPLE_GRID_INVALID");
  const std::uint64_t sample_period_ns =
      result.duration_ns / result.interval_count;
  if (sample_period_ns == 0U ||
      sample_period_ns > kMaximumQuinticSamplePeriodNs)
    throw std::runtime_error("COMMAND_QUINTIC_SAMPLE_GRID_INVALID");
  result.execute_at_monotonic_ns = strict_positive_uint64(
      trajectory.at("execute_at_monotonic_ns"),
      "COMMAND_QUINTIC_EXECUTE_AT_INVALID");
  result.segment_index = strict_nonnegative_int64(
      trajectory.at("segment_index"),
      "COMMAND_QUINTIC_SEGMENT_INDEX_INVALID");
  result.segment_count = strict_nonnegative_int64(
      trajectory.at("segment_count"),
      "COMMAND_QUINTIC_SEGMENT_COUNT_INVALID");
  if (result.segment_count <= 0 ||
      result.segment_index >= result.segment_count)
    throw std::runtime_error("COMMAND_QUINTIC_SEGMENT_RANGE_INVALID");
  if (result.target_rad != candidate.targets)
    throw std::runtime_error("COMMAND_QUINTIC_TARGETS_MISMATCH");

  std::size_t moving_joint = candidate.moving_joint_mask.size();
  for (std::size_t joint = 0; joint < candidate.moving_joint_mask.size();
       ++joint) {
    if (candidate.moving_joint_mask[joint]) {
      if (moving_joint != candidate.moving_joint_mask.size())
        throw std::runtime_error("COMMAND_QUINTIC_MOVING_JOINT_COUNT_INVALID");
      moving_joint = joint;
    } else if (result.start_rad[joint] != result.target_rad[joint]) {
      throw std::runtime_error("COMMAND_QUINTIC_NONMOVING_AXIS_CHANGED");
    }
  }
  if (moving_joint == candidate.moving_joint_mask.size())
    throw std::runtime_error("COMMAND_QUINTIC_MOVING_JOINT_COUNT_INVALID");
  const double duration_seconds =
      static_cast<double>(result.duration_ns) * 1e-9;
  const double displacement = std::abs(
      result.target_rad[moving_joint] - result.start_rad[moving_joint]);
  if (displacement == 0.0)
    throw std::runtime_error("COMMAND_QUINTIC_MOVING_DISPLACEMENT_ZERO");
  // A non-zero move needs at least one interior grid point.  This makes the
  // exact endpoint pin a final bounded grid step rather than the trajectory's
  // sole discontinuous start-to-target update.
  if (result.interval_count < 2U)
    throw std::runtime_error("COMMAND_QUINTIC_ENDPOINT_STEP_INVALID");
  const double peak_velocity =
      kQuinticPeakVelocityScale * displacement / duration_seconds;
  const double peak_acceleration = kQuinticPeakAccelerationScale *
      displacement / (duration_seconds * duration_seconds);
  const double acceleration_limit = moving_joint == 1U
      ? std::min(candidate.amax, kJ2MaximumAcceleration) : candidate.amax;
  if (!std::isfinite(peak_velocity) ||
      peak_velocity > candidate.vmax + 1e-12)
    throw std::runtime_error("COMMAND_QUINTIC_PEAK_VELOCITY_LIMIT");
  if (!std::isfinite(peak_acceleration) ||
      peak_acceleration > acceleration_limit + 1e-12)
    throw std::runtime_error("COMMAND_QUINTIC_PEAK_ACCELERATION_LIMIT");
  // Quintic smoothstep is symmetric: 1-S(1-u) == S(u).  Evaluate the
  // endpoint remainder at u=1/N directly to avoid catastrophic cancellation
  // when N is near its one-million interval bound.
  const double endpoint_fraction = 1.0 /
      static_cast<double>(result.interval_count);
  const double endpoint_fraction_squared =
      endpoint_fraction * endpoint_fraction;
  const double endpoint_fraction_cubed =
      endpoint_fraction_squared * endpoint_fraction;
  const double endpoint_blend = endpoint_fraction_cubed *
      (10.0 + endpoint_fraction *
          (-15.0 + 6.0 * endpoint_fraction));
  const double endpoint_step = displacement * endpoint_blend;
  const double maximum_grid_step = candidate.vmax *
      static_cast<double>(sample_period_ns) * 1e-9;
  if (!std::isfinite(endpoint_step) || endpoint_step <= 0.0 ||
      endpoint_step > maximum_grid_step + 1e-12)
    throw std::runtime_error("COMMAND_QUINTIC_ENDPOINT_STEP_INVALID");
  return result;
}

void parse_command(
    const std::string& text, GuiCommand& command,
    Clock::time_point received_at) {
  const auto value = nlohmann::json::parse(text);
  const std::string schema = value.at("schema").get<std::string>();
  if (schema != "go-m8010-gui-command/1.0" &&
      schema != "go-m8010-gui-command/1.1" &&
      schema != "go-m8010-gui-command/1.2" &&
      schema != "go-m8010-gui-command/1.3" &&
      schema != "go-m8010-gui-command/1.4" &&
      schema != "go-m8010-gui-command/1.5")
    throw std::runtime_error("COMMAND_SCHEMA_MISMATCH");
  const std::string mode = value.at("mode").get<std::string>();
  if (mode != "brake" && mode != "drag" && mode != "hold" && mode != "position" && mode != "teach")
    throw std::runtime_error("COMMAND_MODE_INVALID");
  if (schema != "go-m8010-gui-command/1.2" &&
      schema != "go-m8010-gui-command/1.3" &&
      schema != "go-m8010-gui-command/1.4" &&
      schema != "go-m8010-gui-command/1.5" && mode != "brake")
    throw std::runtime_error("COMMAND_LEGACY_ACTIVE_REJECTED");
  if (schema == "go-m8010-gui-command/1.3" &&
      mode != "position" && mode != "brake")
    throw std::runtime_error("COMMAND_V13_MODE_INVALID");
  if ((mode == "teach" && schema != "go-m8010-gui-command/1.4" && schema != "go-m8010-gui-command/1.5") ||
      (schema == "go-m8010-gui-command/1.4" && mode != "teach" && mode != "brake"))
    throw std::runtime_error("COMMAND_TEACH_SCHEMA_MODE_INVALID");
  if (schema == "go-m8010-gui-command/1.5" && mode != "teach" && mode != "hold" && mode != "brake")
    throw std::runtime_error("COMMAND_GUIDANCE_MODE_INVALID");
  GuiCommand candidate = command;
  candidate.schema = schema;
  candidate.mode = mode;
  candidate.quintic = QuinticTrajectoryDescriptor{};
  candidate.gravity_authority = GravityCommandAuthority{};
  candidate.hand_guidance = HandGuidanceReference{};
  if (schema == "go-m8010-gui-command/1.5" && mode != "brake") {
    const auto& reference = value.at("hand_guidance");
    const std::array<const char*, 7> fields{{"schema", "origin_rad", "velocity_rad_s",
        "maximum_velocity_deg_s", "maximum_excursion_deg", "maximum_reference_error_deg", "freeze_reference"}};
    if (!reference.is_object() || reference.size() != fields.size())
      throw std::runtime_error("COMMAND_GUIDANCE_FIELDS_INVALID");
    for (const auto* field : fields)
      if (!reference.contains(field)) throw std::runtime_error("COMMAND_GUIDANCE_FIELDS_INVALID");
    if (reference.at("schema") != "go-m8010-hand-guidance-reference/1.0")
      throw std::runtime_error("COMMAND_GUIDANCE_SCHEMA_INVALID");
    if (!reference.at("freeze_reference").is_boolean())
      throw std::runtime_error("COMMAND_GUIDANCE_FREEZE_INVALID");
    candidate.hand_guidance.freeze_reference = reference.at("freeze_reference").get<bool>();
    for (const auto& bound : std::array<std::pair<const char*, double>, 2>{{
        {"maximum_velocity_deg_s", 30.0}, {"maximum_reference_error_deg", 2.0}}})
      if (!reference.at(bound.first).is_number() || reference.at(bound.first).get<double>() != bound.second)
        throw std::runtime_error("COMMAND_GUIDANCE_PROFILE_INVALID");
    if (!reference.at("maximum_excursion_deg").is_number() ||
        (reference.at("maximum_excursion_deg") != 10.0 && reference.at("maximum_excursion_deg") != 20.0))
      throw std::runtime_error("COMMAND_GUIDANCE_PROFILE_INVALID");
    candidate.hand_guidance.maximum_excursion_deg = reference.at("maximum_excursion_deg").get<double>();
    candidate.hand_guidance.origin_rad = strict_finite_six_vector(reference.at("origin_rad"), "COMMAND_GUIDANCE_ORIGIN_INVALID", "COMMAND_GUIDANCE_ORIGIN_INVALID");
    candidate.hand_guidance.velocity_rad_s = strict_finite_six_vector(reference.at("velocity_rad_s"), "COMMAND_GUIDANCE_VELOCITY_INVALID", "COMMAND_GUIDANCE_VELOCITY_INVALID");
    candidate.hand_guidance.present = true;
  } else if (mode != "brake" && value.contains("hand_guidance")) {
    throw std::runtime_error("COMMAND_GUIDANCE_LEGACY_SCHEMA_REJECTED");
  }
  const auto targets = value.at("targets_rad").get<std::vector<double>>();
  const auto kp = value.at("kp").get<std::vector<double>>();
  const auto kd = value.at("kd").get<std::vector<double>>();
  const auto feedforward = value.value(
      "feedforward_nm", std::vector<double>(6U, 0.0));
  const bool recovery = value.value("recovery", false);
  const nlohmann::json active_joint_mask =
      schema == "go-m8010-gui-command/1.0"
          ? nlohmann::json::array({false, false, false, false, false, false})
          : value.at("active_joint_mask");
  if (schema == "go-m8010-gui-command/1.3" && mode == "position" &&
      !value.contains("moving_joint_mask"))
    throw std::runtime_error("COMMAND_QUINTIC_MOVING_MASK_MISSING");
  const nlohmann::json moving_joint_mask = value.contains("moving_joint_mask")
      ? value.at("moving_joint_mask")
      : mode == "position" ? active_joint_mask
                            : nlohmann::json::array(
                                  {false, false, false, false, false, false});
  if (targets.size() != 6U || kp.size() != 6U || kd.size() != 6U ||
      feedforward.size() != 6U)
    throw std::runtime_error("COMMAND_VECTOR_SIZE_INVALID");
  if (!active_joint_mask.is_array() || active_joint_mask.size() != 6U)
    throw std::runtime_error("COMMAND_ACTIVE_MASK_SIZE_INVALID");
  if (!moving_joint_mask.is_array() || moving_joint_mask.size() != 6U)
    throw std::runtime_error("COMMAND_MOVING_MASK_SIZE_INVALID");
  for (std::size_t i = 0; i < 6U; ++i) {
    if (!active_joint_mask.at(i).is_boolean())
      throw std::runtime_error("COMMAND_ACTIVE_MASK_TYPE_INVALID");
    if (!moving_joint_mask.at(i).is_boolean())
      throw std::runtime_error("COMMAND_MOVING_MASK_TYPE_INVALID");
    if (!std::isfinite(targets[i]))
      throw std::runtime_error("COMMAND_TARGET_NONFINITE");
    if (!std::isfinite(kp[i]) || !std::isfinite(kd[i]))
      throw std::runtime_error("COMMAND_GAIN_NONFINITE");
    if (!std::isfinite(feedforward[i]))
      throw std::runtime_error("COMMAND_FEEDFORWARD_NONFINITE");
    candidate.targets[i] = targets[i];
    candidate.active_joint_mask[i] = active_joint_mask.at(i).get<bool>();
    candidate.moving_joint_mask[i] = moving_joint_mask.at(i).get<bool>();
    candidate.kp[i] = kp[i];
    candidate.kd[i] = kd[i];
    candidate.feedforward_nm[i] = feedforward[i];
  }
  for (std::size_t i = 0; i < 6U; ++i) {
    if (candidate.moving_joint_mask[i] &&
        !candidate.active_joint_mask[i])
      throw std::runtime_error("COMMAND_MOVING_MASK_NOT_ACTIVE_SUBSET");
    if (candidate.moving_joint_mask[i] && mode != "position" && mode != "teach")
      throw std::runtime_error("COMMAND_MOVING_MASK_MODE_INVALID");
  }
  if (schema == "go-m8010-gui-command/1.5" && mode == "brake") {
    candidate.feedforward_nm.fill(0.0);
  } else {
    candidate.gravity_authority = parse_gravity_command_authority(value, candidate, received_at);
  }
  if (mode == "teach" && !candidate.hand_guidance.present &&
      (std::count(candidate.moving_joint_mask.begin(), candidate.moving_joint_mask.end(), true) != 1 ||
       candidate.moving_joint_mask[5] || !candidate.gravity_authority.present || candidate.gravity_authority.hand_guidance_profile))
    throw std::runtime_error("COMMAND_GRAVITY_TEACH_SCOPE_INVALID");
  if (candidate.hand_guidance.present &&
      (!candidate.gravity_authority.present || !candidate.gravity_authority.hand_guidance_profile ||
       candidate.hand_guidance.maximum_excursion_deg != candidate.gravity_authority.empirical_maximum_teach_excursion_deg ||
       !std::all_of(candidate.active_joint_mask.begin(), candidate.active_joint_mask.end(), [](bool active) { return active; }) ||
       (mode == "teach" && (!candidate.gravity_authority.empirical_position_validation_authorized || candidate.gravity_authority.empirical_stage_index != 4U ||
        candidate.gravity_authority.gravity_scale != 1.0 || candidate.gravity_authority.gravity_scale_target != 1.0 ||
        std::none_of(candidate.moving_joint_mask.begin(), candidate.moving_joint_mask.end(), [](bool moving) { return moving; })))))
    throw std::runtime_error("COMMAND_GRAVITY_GUIDANCE_SCOPE_INVALID");
  if (mode == "teach" && hand_guidance_return_only(candidate) &&
      (!candidate.hand_guidance.freeze_reference || std::any_of(
          candidate.hand_guidance.velocity_rad_s.begin(), candidate.hand_guidance.velocity_rad_s.end(),
          [](double value) { return value != 0.0; })))
    throw GuidanceReferenceRejected("COMMAND_GUIDANCE_RETURN_ONLY_REQUIRES_FREEZE");
  if (schema == "go-m8010-gui-command/1.3" && mode == "position" &&
      !candidate.gravity_authority.present)
    throw std::runtime_error("COMMAND_GRAVITY_AUTHORITY_MISSING");
  if (recovery && (schema != "go-m8010-gui-command/1.2" ||
                   (mode != "hold" && mode != "position")))
    throw std::runtime_error("COMMAND_RECOVERY_MODE_INVALID");
  // The production GUI command socket has no per-command signed recovery
  // permit or authenticated source.  Startup/session permits authorize only
  // BRAKE initialization; an arbitrary localhost sender must not widen target,
  // feedforward, velocity, or J2 torque envelopes with one boolean field.
  if (recovery)
    throw std::runtime_error("COMMAND_RECOVERY_UNAUTHORIZED");
  candidate.recovery = false;
  const double requested_vmax = value.at("maximum_velocity_rad_s").get<double>();
  const double requested_amax = value.at("maximum_acceleration_rad_s2").get<double>();
  if (!std::isfinite(requested_vmax) || requested_vmax <= 0.0 ||
      !std::isfinite(requested_amax) || requested_amax <= 0.0)
    throw std::runtime_error("COMMAND_PROFILE_INVALID");
  candidate.vmax = std::min(requested_vmax, (candidate.hand_guidance.present ? 30.0 : 5.0) * kPi / 180.0);
  candidate.amax = std::min(requested_amax, 20.0 * kPi / 180.0);
  if (schema == "go-m8010-gui-command/1.3" && mode == "position")
    candidate.quintic = parse_quintic_trajectory_descriptor(value, candidate);
  if (mode == "brake" &&
      std::any_of(candidate.active_joint_mask.begin(), candidate.active_joint_mask.end(),
                  [](bool active) { return active; }))
    throw std::runtime_error("COMMAND_BRAKE_MASK_NOT_EMPTY");
  if (schema == "go-m8010-gui-command/1.0") {
    candidate.activation_epoch = 0;
  } else {
    const auto& activation_epoch = value.at("activation_epoch");
    if (!activation_epoch.is_number_unsigned())
      throw std::runtime_error("COMMAND_ACTIVATION_EPOCH_INVALID");
    candidate.activation_epoch = activation_epoch.get<std::uint64_t>();
    if (candidate.activation_epoch >
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
      throw std::runtime_error("COMMAND_ACTIVATION_EPOCH_INVALID");
  }
  if ((mode == "hold" || mode == "position" || mode == "teach") &&
      std::any_of(candidate.active_joint_mask.begin(), candidate.active_joint_mask.end(),
                  [](bool active) { return active; }) &&
      candidate.activation_epoch == 0)
    throw std::runtime_error("COMMAND_ACTIVE_EPOCH_ZERO");
  if (mode == "brake") {
    // Emergency BRAKE remains compatible with v1.0/v1.1 and deliberately
    // ignores even malformed source metadata. Never inherit the preceding
    // active command's identity into the normalized brake command.
    candidate.source_instance_id.clear();
    candidate.source_monotonic_ns = 0;
    candidate.source_sequence = 0;
  } else {
    const auto& source_instance_id = value.at("source_instance_id");
    if (!source_instance_id.is_string())
      throw std::runtime_error("COMMAND_SOURCE_INSTANCE_INVALID");
    candidate.source_instance_id = source_instance_id.get<std::string>();
    if (!valid_source_instance_id(candidate.source_instance_id))
      throw std::runtime_error("COMMAND_SOURCE_INSTANCE_INVALID");

    const auto& source_monotonic_ns = value.at("source_monotonic_ns");
    if (!source_monotonic_ns.is_number_unsigned())
      throw std::runtime_error("COMMAND_SOURCE_TIMESTAMP_INVALID");
    candidate.source_monotonic_ns =
        source_monotonic_ns.get<std::uint64_t>();
    if (candidate.source_monotonic_ns == 0U)
      throw std::runtime_error("COMMAND_SOURCE_TIMESTAMP_INVALID");
    const std::uint64_t received_ns = monotonic_ns_at(received_at);
    if (candidate.source_monotonic_ns > received_ns)
      throw std::runtime_error("COMMAND_SOURCE_TIMESTAMP_FUTURE");
    if (received_ns - candidate.source_monotonic_ns >
        kMaximumCommandSourceAgeNs)
      throw std::runtime_error("COMMAND_SOURCE_TIMESTAMP_STALE");

    const auto& sequence = value.at("sequence");
    if (!sequence.is_number_unsigned())
      throw std::runtime_error("COMMAND_SOURCE_SEQUENCE_INVALID");
    candidate.source_sequence = sequence.get<std::uint64_t>();
    if (candidate.source_sequence == 0U ||
        candidate.source_sequence > kMaximumCommandSequence)
      throw std::runtime_error("COMMAND_SOURCE_SEQUENCE_INVALID");
  }
  candidate.received_at = received_at;
  candidate.received = true;
  command = std::move(candidate);
}

void parse_command(const std::string& text, GuiCommand& command) {
  // Capture receipt only after the caller has finished producing text.  A
  // default third argument would have unspecified ordering versus expressions
  // such as fresh_source_self_test_payload(...), and could appear a few
  // microseconds earlier than the embedded source timestamp.
  parse_command(text, command, Clock::now());
}

std::uint64_t minimum_epoch_after_lease(
    std::uint64_t current_minimum, bool lease_fresh,
    bool active_requested, std::uint64_t command_epoch) {
  if (lease_fresh || !active_requested) return current_minimum;
  return std::max(current_minimum, command_epoch + 1U);
}

// Staged software stopping bounds, not a measured braking-distance guarantee
// or a motor continuous rating. They do not widen the ordinary 5 deg/s gate.
constexpr std::uint64_t kTeachStoppingHoldNs = 1000000000ULL;
constexpr double kTeachStoppingErrorRad = 2.0 * kPi / 180.0;
constexpr double kTeachRestrictedRearmErrorRad = 0.25 * kPi / 180.0;
constexpr double kTeachRestrictedRearmVelocity = 0.25 * kPi / 180.0;
constexpr std::uint64_t kTeachRestrictedStableNs = 500000000ULL;

struct AssistedTeachExitHold {
  bool present = false;
  bool acknowledged = false;
  bool completed = false;
  bool restricted = false;
  std::uint64_t stable_since_monotonic_ns = 0;
  int stable_frames = 0;
  std::size_t joint_index = 0;
  std::uint64_t press_activation_epoch = 0;
  std::uint64_t started_monotonic_ns = 0;
  std::uint64_t deadline_monotonic_ns = 0;
  std::string reason;
  std::array<double, 6> targets_rad{};
  double initial_velocity_rad_s = 0.0;
};

struct CommandSafetyState {
  GuiCommand last_accepted_command;
  GuiCommand teach_command;
  std::uint64_t teach_started_monotonic_ns = 0;
  bool teach_active = false;
  AssistedTeachExitHold teach_exit_hold;
  bool guidance_bound = false;
  bool guidance_origin_bound = false;
  bool guidance_active = false;
  bool guidance_paused = false;
  std::string guidance_paused_reason;
  std::uint64_t guidance_started_ns = 0;
  GuiCommand guidance_press;
  GuiCommand guidance_reference;
  std::uint64_t minimum_activation_epoch = 0;
  std::uint64_t last_seen_activation_epoch = 0;
  // Incoming active commands at this epoch or below remain rejected.  This is
  // intentionally separate from minimum_activation_epoch: rejecting a new
  // candidate must not revoke the older command that is already controlling.
  std::uint64_t highest_rejected_active_epoch = 0;
  std::array<bool, 6> fixed_target_valid{};
  std::array<std::uint64_t, 6> fixed_target_epoch{};
  std::array<double, 6> fixed_target{};
  std::array<bool, 6> position_target_valid{};
  std::array<std::uint64_t, 6> position_target_epoch{};
  std::array<double, 6> position_target{};
  bool position_authority_schema_valid = false;
  std::uint64_t position_authority_schema_epoch = 0;
  std::string position_authority_schema;
  QuinticTrajectoryDescriptor quintic_descriptor;
  bool gravity_policy_valid = false;
  std::uint64_t gravity_policy_epoch = 0;
  std::string gravity_source_instance_id;
  std::string gravity_session_id;
  std::string gravity_state_instance_id;
  double gravity_scale_target = 0.0;
  std::string gravity_empirical_envelope_id;
  std::string gravity_empirical_envelope_sha256;
  std::string gravity_anchor_sha256;
  std::uint64_t gravity_empirical_stage_index = 0;
  bool gravity_empirical_position_validation_authorized = false;
  bool gravity_empirical_active = false;
  std::string spent_gravity_empirical_envelope_sha256;
};

struct CommandRejectAggregate {
  std::uint64_t total_count = 0;
  std::uint64_t pending_count = 0;
  Clock::time_point last_report_at{};
};

struct AcceptedCommandSource {
  std::uint64_t last_sequence = 0;
  std::uint64_t last_source_monotonic_ns = 0;
  std::uint64_t last_used = 0;
};

struct CommandReceiveState {
  std::map<std::string, CommandRejectAggregate> rejected_by_reason;
  std::map<std::string, AcceptedCommandSource> accepted_sources;
  std::string active_source_instance_id;
  Clock::time_point active_source_received_at{};
  std::uint64_t source_use_counter = 0;
  std::uint64_t budget_total_count = 0;
  std::uint64_t budget_pending_count = 0;
  Clock::time_point budget_last_report_at{};
};

struct CommandReceiveResult {
  bool domain_release_received = false;
  std::size_t packets_processed = 0;
  bool packet_budget_reached = false;
};

void validate_command_source_replay(
    const GuiCommand& command, const CommandReceiveState& state) {
  if (command.mode == "brake") return;
  if (!state.active_source_instance_id.empty() &&
      command.source_instance_id != state.active_source_instance_id &&
      state.active_source_received_at != Clock::time_point{}) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        command.received_at - state.active_source_received_at).count();
    if (elapsed <= static_cast<std::int64_t>(kCommandSourceTakeoverLockNs))
      throw std::runtime_error("COMMAND_SOURCE_INSTANCE_BUSY");
  }
  const auto found = state.accepted_sources.find(command.source_instance_id);
  if (found == state.accepted_sources.end()) return;
  if (command.source_sequence <= found->second.last_sequence)
    throw std::runtime_error("COMMAND_SOURCE_SEQUENCE_REPLAY");
  if (command.source_monotonic_ns <=
      found->second.last_source_monotonic_ns)
    throw std::runtime_error("COMMAND_SOURCE_TIMESTAMP_REPLAY");
}

void observe_command_source(
    const GuiCommand& command, CommandReceiveState& state) {
  if (command.mode == "brake") return;
  auto found = state.accepted_sources.find(command.source_instance_id);
  if (found == state.accepted_sources.end() &&
      state.accepted_sources.size() >= kMaximumTrackedCommandSources) {
    const auto oldest = std::min_element(
        state.accepted_sources.begin(), state.accepted_sources.end(),
        [](const auto& left, const auto& right) {
          return left.second.last_used < right.second.last_used;
        });
    if (oldest != state.accepted_sources.end())
      state.accepted_sources.erase(oldest);
  }
  ++state.source_use_counter;
  auto& accepted = state.accepted_sources[command.source_instance_id];
  accepted.last_sequence = command.source_sequence;
  accepted.last_source_monotonic_ns = command.source_monotonic_ns;
  accepted.last_used = state.source_use_counter;
  state.active_source_instance_id = command.source_instance_id;
  state.active_source_received_at = command.received_at;
}

bool command_log_interval_elapsed(
    Clock::time_point now, Clock::time_point last_report_at) {
  return last_report_at == Clock::time_point{} ||
      std::chrono::duration<double>(now - last_report_at).count() >=
          kCommandRejectSummarySeconds;
}

void flush_command_receive_summaries(
    CommandReceiveState& state, Clock::time_point now, bool final) {
  for (auto& item : state.rejected_by_reason) {
    auto& aggregate = item.second;
    if (aggregate.pending_count == 0U ||
        (!final && !command_log_interval_elapsed(
                       now, aggregate.last_report_at)))
      continue;
    std::cerr << (final ? "GUI_COMMAND_REJECTED_FINAL"
                        : "GUI_COMMAND_REJECTED_SUMMARY")
              << " reason=" << item.first
              << " interval_count=" << aggregate.pending_count
              << " total_count=" << aggregate.total_count
              << std::endl;
    aggregate.pending_count = 0U;
    aggregate.last_report_at = now;
  }
  if (state.budget_pending_count > 0U &&
      (final || command_log_interval_elapsed(
                    now, state.budget_last_report_at))) {
    std::cerr << (final ? "GUI_COMMAND_RX_BUDGET_FINAL"
                        : "GUI_COMMAND_RX_BUDGET_SUMMARY")
              << " interval_count=" << state.budget_pending_count
              << " total_count=" << state.budget_total_count
              << " packet_budget=" << kCommandPacketBudget
              << std::endl;
    state.budget_pending_count = 0U;
    state.budget_last_report_at = now;
  }
}

void record_command_rejection(
    const std::string& reason, CommandReceiveState& state,
    Clock::time_point now) {
  auto& aggregate = state.rejected_by_reason[reason];
  ++aggregate.total_count;
  if (aggregate.total_count == 1U) {
    aggregate.last_report_at = now;
    std::cerr << "GUI_COMMAND_REJECTED_BEGIN"
              << " reason=" << reason
              << " interval_count=1 total_count=1"
              << std::endl;
  } else {
    ++aggregate.pending_count;
  }
}

void record_command_budget_reached(
    CommandReceiveState& state, Clock::time_point now) {
  ++state.budget_total_count;
  if (state.budget_total_count == 1U) {
    state.budget_last_report_at = now;
    std::cerr << "GUI_COMMAND_RX_BUDGET_BEGIN"
              << " interval_count=1 total_count=1"
              << " packet_budget=" << kCommandPacketBudget
              << std::endl;
  } else {
    ++state.budget_pending_count;
  }
}

std::string command_rejection_reason(const std::exception& error) {
  if (dynamic_cast<const nlohmann::json::exception*>(&error) != nullptr)
    return "COMMAND_JSON_INVALID";
  return error.what();
}

bool is_position_holding_mode(const std::string& mode) {
  return mode == "hold" || mode == "position" || mode == "teach";
}

bool joint_is_assisted_teach(
    const GuiCommand& command, const std::string& mode, std::size_t joint) {
  return mode == "teach" && joint < 5U && command.active_joint_mask[joint] &&
      command.moving_joint_mask[joint];
}

bool joint_uses_fixed_hold_target(
    const GuiCommand& command, const std::string& effective_mode,
    std::size_t joint) {
  return effective_mode == "hold" ||
      ((effective_mode == "position" || effective_mode == "teach") &&
       command.active_joint_mask[joint] &&
       !command.moving_joint_mask[joint]);
}

bool position_profile_at_authorized_endpoint(
    const GuiCommand& command, std::size_t joint,
    double profile_q, double profile_dq) {
  return joint < command.targets.size() && command.mode == "position" &&
      command.active_joint_mask[joint] && command.moving_joint_mask[joint] &&
      std::isfinite(profile_q) && std::isfinite(profile_dq) &&
      std::abs(profile_q - command.targets[joint]) <= 1e-9 &&
      std::abs(profile_dq) <= 1e-9;
}

bool use_j2_moving_limits(
    const GuiCommand& command, const std::string& effective_mode,
    bool position_endpoint_phase = false) {
  return effective_mode == "position" && !command.recovery &&
      command.active_joint_mask[1] && command.moving_joint_mask[1] &&
      !position_endpoint_phase;
}

bool use_j2_hold_protection_limits(
    const GuiCommand& command, const std::string& effective_mode,
    bool position_endpoint_phase = false) {
  return command.recovery ||
      joint_is_assisted_teach(command, effective_mode, 1U) ||
      joint_uses_fixed_hold_target(command, effective_mode, 1U) ||
      (position_endpoint_phase && effective_mode == "position" &&
       command.active_joint_mask[1] && command.moving_joint_mask[1]);
}

std::pair<double, double> j2_gain_targets(
    const GuiCommand& command, const std::string& effective_mode,
    const MotorRuntime& motor, bool position_endpoint_phase = false) {
  const bool moving = use_j2_moving_limits(
      command, effective_mode, position_endpoint_phase);
  return {
      std::min(
          std::min(command.kp[1], motor.kp_limit),
          moving ? kJ2MovingKpLimit : motor.kp_limit),
      std::min(
          std::min(command.kd[1], motor.kd_limit),
          moving ? kJ2MovingKdLimit : motor.kd_limit)};
}

bool should_apply_velocity_guard(
    const GuiCommand& command, const std::string& effective_mode,
    std::size_t joint, bool position_endpoint_phase = false) {
  return effective_mode == "position" &&
      command.moving_joint_mask[joint] && !command.recovery &&
      !position_endpoint_phase;
}

int next_active_deadline_miss_count(
    int current, bool active, bool missed) {
  if (!active || !missed) return 0;
  return std::min(current + 1, kActiveDeadlineConsecutiveLimit);
}

bool command_selects_owned_joint(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors) {
  return std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
    return command.active_joint_mask[
        static_cast<std::size_t>(motor.joint_index)];
  });
}

bool command_releases_owned_domain(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors) {
  return command.mode == "brake" || command.mode == "drag" ||
      !command_selects_owned_joint(command, motors);
}

bool healthy_logical_position_for_joint(
    const std::vector<MotorRuntime>& motors, int joint_index,
    double& logical_position);

void spend_empirical_gravity_authority(CommandSafetyState& safety) {
  if (!safety.gravity_empirical_active ||
      safety.gravity_empirical_envelope_sha256.empty())
    return;
  safety.spent_gravity_empirical_envelope_sha256 =
      safety.gravity_empirical_envelope_sha256;
  safety.gravity_empirical_active = false;
}

void validate_first_quintic_start_against_feedback(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors);

void validate_and_observe_gravity_policy(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors,
    CommandSafetyState& safety) {
  const bool active_owned = is_position_holding_mode(command.mode) &&
      command_selects_owned_joint(command, motors);
  if (!active_owned) return;
  if (!command.gravity_authority.present && !command.recovery)
    throw std::runtime_error("COMMAND_EMPIRICAL_AUTHORITY_MISSING");
  if (!command.gravity_authority.present) return;
  const auto& authority = command.gravity_authority;
  if (!authority.official_continuous_authority &&
      !safety.spent_gravity_empirical_envelope_sha256.empty() &&
      authority.empirical_envelope_sha256 ==
          safety.spent_gravity_empirical_envelope_sha256)
    throw std::runtime_error("COMMAND_EMPIRICAL_ENVELOPE_SPENT");
  if (command.mode == "position" &&
      !authority.official_continuous_authority &&
      !authority.empirical_position_validation_authorized)
    throw std::runtime_error(
        "COMMAND_EMPIRICAL_POSITION_AUTHORITY_MISSING");
  if (safety.gravity_policy_valid &&
      safety.gravity_policy_epoch == command.activation_epoch) {
    const bool identity_changed =
        safety.gravity_source_instance_id != authority.source_instance_id ||
        safety.gravity_session_id != authority.session_id ||
        safety.gravity_state_instance_id != authority.state_instance_id ||
        safety.gravity_empirical_envelope_id !=
            authority.empirical_envelope_id ||
        safety.gravity_empirical_envelope_sha256 !=
            authority.empirical_envelope_sha256 ||
        safety.gravity_anchor_sha256 != authority.anchor_sha256;
    const bool previously_empirical =
        !safety.gravity_empirical_envelope_id.empty();
    if (identity_changed ||
        previously_empirical == authority.official_continuous_authority)
      throw std::runtime_error(
          "COMMAND_GRAVITY_POLICY_CHANGED_SAME_EPOCH");
    if (authority.official_continuous_authority) {
      if (safety.gravity_empirical_stage_index !=
              authority.empirical_stage_index ||
          safety.gravity_empirical_position_validation_authorized !=
              authority.empirical_position_validation_authorized ||
          std::abs(safety.gravity_scale_target -
                   authority.gravity_scale_target) > 1e-12)
        throw std::runtime_error(
            "COMMAND_GRAVITY_POLICY_CHANGED_SAME_EPOCH");
      return;
    }
    const bool same_stage =
        safety.gravity_empirical_stage_index ==
            authority.empirical_stage_index &&
        std::abs(safety.gravity_scale_target -
                 authority.gravity_scale_target) <= 1e-12;
    const bool adjacent_stage =
        authority.empirical_stage_index ==
            safety.gravity_empirical_stage_index + 1U;
    const bool position_authority_unchanged =
        safety.gravity_empirical_position_validation_authorized ==
            authority.empirical_position_validation_authorized;
    const bool final_stage_position_unlock =
        same_stage && authority.empirical_stage_index == 4U &&
        !safety.gravity_empirical_position_validation_authorized &&
        authority.empirical_position_validation_authorized;
    if ((!same_stage && !adjacent_stage) ||
        (adjacent_stage &&
         authority.empirical_position_validation_authorized) ||
        (!position_authority_unchanged &&
         !final_stage_position_unlock))
      throw std::runtime_error(
          "COMMAND_GRAVITY_POLICY_CHANGED_SAME_EPOCH");
    // The activation epoch still binds one unchanged current-position HOLD.
    // A separately confirmed adjacent empirical rung is a monotonic sub-state
    // whose continuously changing feedforward must reach the hardware on
    // ordinary heartbeats; it must not require a worker/J6 re-enable.
    safety.gravity_scale_target = authority.gravity_scale_target;
    safety.gravity_empirical_stage_index =
        authority.empirical_stage_index;
    safety.gravity_empirical_position_validation_authorized =
        authority.empirical_position_validation_authorized;
    return;
  }
  if (!authority.official_continuous_authority &&
      safety.gravity_policy_valid) {
    if (safety.gravity_empirical_envelope_id !=
            authority.empirical_envelope_id ||
        safety.gravity_anchor_sha256 != authority.anchor_sha256 ||
        authority.empirical_stage_index <
            safety.gravity_empirical_stage_index ||
        authority.empirical_stage_index >
            safety.gravity_empirical_stage_index + 1U)
      throw std::runtime_error(
          "COMMAND_EMPIRICAL_STAGE_SEQUENCE_INVALID");
  } else if (!authority.official_continuous_authority &&
             authority.empirical_stage_index != 0U) {
    throw std::runtime_error(
        "COMMAND_EMPIRICAL_FIRST_STAGE_NOT_ZERO");
  }
  safety.gravity_policy_valid = true;
  safety.gravity_policy_epoch = command.activation_epoch;
  safety.gravity_source_instance_id = authority.source_instance_id;
  safety.gravity_session_id = authority.session_id;
  safety.gravity_state_instance_id = authority.state_instance_id;
  safety.gravity_scale_target = authority.gravity_scale_target;
  safety.gravity_empirical_envelope_id = authority.empirical_envelope_id;
  safety.gravity_empirical_envelope_sha256 =
      authority.empirical_envelope_sha256;
  safety.gravity_anchor_sha256 = authority.anchor_sha256;
  safety.gravity_empirical_stage_index = authority.empirical_stage_index;
  safety.gravity_empirical_position_validation_authorized =
      authority.empirical_position_validation_authorized;
  safety.gravity_empirical_active =
      !authority.official_continuous_authority;
}

void validate_and_observe_position_authority(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors,
    CommandSafetyState& safety) {
  if (command.mode != "position" ||
      !command_selects_owned_joint(command, motors))
    return;
  CommandSafetyState candidate = safety;
  if (candidate.position_authority_schema_valid &&
      candidate.position_authority_schema_epoch == command.activation_epoch) {
    if (candidate.position_authority_schema != command.schema)
      throw std::runtime_error(
          "COMMAND_POSITION_SCHEMA_CHANGED_SAME_EPOCH");
    if (command.schema == "go-m8010-gui-command/1.3" &&
        !same_quintic_trajectory_descriptor(
            candidate.quintic_descriptor, command.quintic))
      throw std::runtime_error(
          "COMMAND_QUINTIC_DESCRIPTOR_CHANGED_SAME_EPOCH");
    return;
  }
  if (command.schema == "go-m8010-gui-command/1.3") {
    if (!command.quintic.present)
      throw std::runtime_error("COMMAND_QUINTIC_DESCRIPTOR_MISSING");
    const std::uint64_t received_ns = monotonic_ns_at(command.received_at);
    if (received_ns == 0U ||
        received_ns >= command.quintic.execute_at_monotonic_ns)
      throw std::runtime_error("COMMAND_QUINTIC_FIRST_PACKET_TOO_LATE");
    validate_first_quintic_start_against_feedback(command, motors);
    if (!command.gravity_authority.present)
      throw std::runtime_error(
          "COMMAND_EMPIRICAL_POSITION_SEGMENT_INVALID");
    if (!command.gravity_authority.official_continuous_authority) {
      if (!command.gravity_authority
               .empirical_position_validation_authorized ||
          static_cast<double>(command.quintic.duration_ns) * 1e-9 >
              command.gravity_authority
                  .empirical_maximum_position_segment_seconds + 1e-12)
        throw std::runtime_error(
            "COMMAND_EMPIRICAL_POSITION_SEGMENT_INVALID");
      const double maximum_displacement =
        command.gravity_authority.empirical_maximum_abs_position_segment_deg *
        kPi / 180.0;
      for (std::size_t joint = 0; joint < command.moving_joint_mask.size();
           ++joint) {
        if (command.moving_joint_mask[joint] &&
            std::abs(command.quintic.target_rad[joint] -
                     command.quintic.start_rad[joint]) >
                maximum_displacement + 1e-12)
          throw std::runtime_error(
              "COMMAND_EMPIRICAL_POSITION_DISPLACEMENT_INVALID");
      }
    }
  }
  candidate.position_authority_schema_valid = true;
  candidate.position_authority_schema_epoch = command.activation_epoch;
  candidate.position_authority_schema = command.schema;
  candidate.quintic_descriptor = command.quintic;
  safety = std::move(candidate);
}

void validate_guidance_return_only(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors,
    const CommandSafetyState& safety) {
  if (!hand_guidance_return_only(command) || command_releases_owned_domain(command, motors)) return;
  const auto& previous = safety.last_accepted_command;
  const auto& a = command.gravity_authority;
  const auto& b = previous.gravity_authority;
  if (!previous.received || !b.present || command.source_instance_id != previous.source_instance_id ||
      a.source_instance_id != b.source_instance_id || a.session_id != b.session_id ||
      a.state_instance_id != b.state_instance_id || a.empirical_envelope_id != b.empirical_envelope_id ||
      a.empirical_envelope_sha256 != b.empirical_envelope_sha256 || a.anchor_sha256 != b.anchor_sha256 ||
      a.empirical_maximum_teach_excursion_deg != b.empirical_maximum_teach_excursion_deg ||
      a.empirical_envelope_expires_at_utc != b.empirical_envelope_expires_at_utc ||
      a.empirical_envelope_deadline_monotonic_ns > b.empirical_envelope_deadline_monotonic_ns)
    throw std::runtime_error("COMMAND_GRAVITY_RETURN_ONLY_IDENTITY_CHANGED");
  if (command.active_joint_mask != previous.active_joint_mask || command.kp != previous.kp || command.kd != previous.kd)
    throw GuidanceReferenceRejected("COMMAND_GUIDANCE_RETURN_ONLY_PARAMETERS_CHANGED");
  if (command.mode == "hold") {
    for (const auto& motor : motors) {
      const auto joint = static_cast<std::size_t>(motor.joint_index);
      if (command.targets[joint] != previous.targets[joint])
        throw GuidanceReferenceRejected("COMMAND_GUIDANCE_RETURN_ONLY_HOLD_TARGET_CHANGED");
    }
    return;
  }
  if (command.mode == "teach" && safety.guidance_active && command.hand_guidance.present &&
      command.hand_guidance.freeze_reference && command.activation_epoch == safety.guidance_press.activation_epoch &&
      std::all_of(command.hand_guidance.velocity_rad_s.begin(), command.hand_guidance.velocity_rad_s.end(),
                  [](double value) { return value == 0.0; })) return;
  const auto& press = safety.guidance_press;
  if (command.mode != "position" || safety.guidance_active || !safety.guidance_origin_bound || !press.hand_guidance.present ||
      press.gravity_authority.empirical_envelope_sha256 != a.empirical_envelope_sha256 ||
      command.schema != "go-m8010-gui-command/1.3" || !command.quintic.present ||
      std::count(command.moving_joint_mask.begin(), command.moving_joint_mask.end(), true) != 1)
    throw GuidanceReferenceRejected("COMMAND_GUIDANCE_RETURN_ONLY_OPERATION_REJECTED");
  const auto joint = static_cast<std::size_t>(std::distance(command.moving_joint_mask.begin(),
      std::find(command.moving_joint_mask.begin(), command.moving_joint_mask.end(), true)));
  const double start = command.quintic.start_rad[joint], target = command.quintic.target_rad[joint];
  const double origin = press.hand_guidance.origin_rad[joint];
  if (!std::isfinite(origin) || target < std::min(start, origin) - 1e-12 || target > std::max(start, origin) + 1e-12)
    throw GuidanceReferenceRejected("COMMAND_GUIDANCE_RETURN_ONLY_MUST_APPROACH_ORIGIN");
}

void validate_command_for_owned_domain(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors) {
  if (command.mode == "brake" || command.mode == "drag") return;
  std::set<int> validated_joints;
  for (const auto& motor : motors) {
    const int joint_index = motor.joint_index;
    const auto joint = static_cast<std::size_t>(joint_index);
    if (!command.active_joint_mask[joint] ||
        !validated_joints.insert(joint_index).second)
      continue;
    // POSITION, its non-moving support axes, HOLD and signed recovery all use
    // the same model-derived session envelope.  A rejected candidate never
    // replaces the previously accepted FOC command in receive_latest().
    if (!command.hand_guidance.present && !within_model_command_envelope(
            joint_index, command.targets[joint]))
      throw std::runtime_error("COMMAND_TARGET_MODEL_ENVELOPE");
    if (command.quintic.present &&
        !within_model_command_envelope(
            joint_index, command.quintic.start_rad[joint]))
      throw std::runtime_error("COMMAND_QUINTIC_START_MODEL_ENVELOPE");
    if (command.kp[joint] < 0.0 ||
        command.kp[joint] > kKpLimits[joint] + 1e-12 ||
        command.kd[joint] < 0.0 ||
        command.kd[joint] > kKdLimits[joint] + 1e-12)
      throw std::runtime_error("COMMAND_GAIN_ENVELOPE");
    const double feedforward_limit = command.gravity_authority.present
        ? kGravityFeedforwardLimits[joint]
        : command.recovery ? kRecoveryFeedforwardLimits[joint] : 0.0;
    if (std::abs(command.feedforward_nm[joint]) >
        feedforward_limit + 1e-12)
      throw std::runtime_error("COMMAND_FEEDFORWARD_ENVELOPE");
  }
}

bool all_motor_feedback_healthy(const std::vector<MotorRuntime>& motors) {
  return std::all_of(motors.begin(), motors.end(), [](const MotorRuntime& motor) {
    return motor.reference_ready && motor.valid && !motor.fault_latched &&
        motor.merror == 0 && motor.temperature >= 0 &&
        motor.temperature < g_thermal_policy.thermal_stop_c;
  });
}

bool motor_feedback_requires_domain_brake(const MotorRuntime& motor) {
  return !motor.valid || motor.fault_latched || motor.merror != 0 ||
      motor.temperature < 0 ||
      motor.temperature >= g_thermal_policy.thermal_stop_c;
}

bool owned_domain_feedback_requires_brake(
    const std::vector<MotorRuntime>& motors) {
  return std::any_of(
      motors.begin(), motors.end(), motor_feedback_requires_domain_brake);
}

bool selected_owned_motors_confirmed_foc(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors) {
  bool selected = false;
  for (const auto& motor : motors) {
    const std::size_t joint = static_cast<std::size_t>(motor.joint_index);
    if (!command.active_joint_mask[joint]) continue;
    selected = true;
    if (!motor.reference_ready || !motor.valid || !motor.last_frame_valid ||
        motor.fault_latched ||
        motor.returned_mode != kFocMode)
      return false;
  }
  return selected;
}

bool same_external_hold_authority(
    const GuiCommand& first, const GuiCommand& second) {
  return first.source_instance_id == second.source_instance_id &&
      first.activation_epoch == second.activation_epoch &&
      first.schema == second.schema &&
      first.mode == second.mode && first.targets == second.targets &&
      first.active_joint_mask == second.active_joint_mask &&
      first.moving_joint_mask == second.moving_joint_mask &&
      first.kp == second.kp && first.kd == second.kd &&
      first.feedforward_nm == second.feedforward_nm &&
      first.recovery == second.recovery &&
      same_quintic_trajectory_descriptor(first.quintic, second.quintic);
}

bool capture_lease_safe_hold_command(
    const std::string& bus, const GuiCommand& source,
    const std::vector<MotorRuntime>& motors, bool domain_fault,
    bool j2_sync_fault, GuiCommand& captured,
    const std::array<bool, 6>* position_arrived_once = nullptr) {
  (void)position_arrived_once;
  if (!source.received || !is_position_holding_mode(source.mode) ||
      !command_selects_owned_joint(source, motors) || domain_fault ||
      j2_sync_fault || !all_motor_feedback_healthy(motors))
    return false;

  GuiCommand candidate = source;
  candidate.mode = "hold";
  candidate.targets.fill(0.0);
  candidate.active_joint_mask.fill(false);
  candidate.moving_joint_mask.fill(false);
  if (bus == "j2") {
    if (motors.size() != 2U || !source.active_joint_mask[1]) return false;
    // j2_sync_fault is the worker's hard-latched coupled-axis decision.  This
    // helper must consume that decision rather than independently reconstruct
    // synchronization from non-atomic per-motor state at a lease boundary.
    // Lease loss is not permission to adopt a gravity/external-force offset.
    // Preserve the exact authorized endpoint even before arrival so the
    // fixed-position safe hold continues restoring toward it.
    candidate.targets[1] = source.targets[1];
    if (!within_mechanical_feedback_envelope(1, candidate.targets[1]))
      return false;
    candidate.active_joint_mask[1] = true;
  } else {
    for (const auto& motor : motors) {
      const std::size_t joint = static_cast<std::size_t>(motor.joint_index);
      if (!source.active_joint_mask[joint]) continue;
      const double logical = motor.sign *
          (motor.unwrapped - motor.reference) / kGear;
      if (!within_mechanical_feedback_envelope(motor.joint_index, logical))
        return false;
      candidate.targets[joint] = source.targets[joint];
      if (!within_mechanical_feedback_envelope(
              motor.joint_index, candidate.targets[joint]))
        return false;
      candidate.active_joint_mask[joint] = true;
    }
  }
  captured = std::move(candidate);
  return true;
}

bool external_command_explicitly_releases_safe_hold(
    const GuiCommand& command, bool lease_fresh,
    const std::vector<MotorRuntime>& motors) {
  if (!lease_fresh) return false;
  if (command.mode == "brake" || command.mode == "drag") return true;
  return is_position_holding_mode(command.mode) &&
      !command_selects_owned_joint(command, motors);
}

bool external_command_can_resume_from_safe_hold(
    const GuiCommand& command, bool lease_fresh,
    const std::vector<MotorRuntime>& motors,
    std::uint64_t source_epoch, std::uint64_t minimum_epoch,
    std::uint64_t highest_rejected_active_epoch = 0) {
  return lease_fresh && is_position_holding_mode(command.mode) &&
      command_selects_owned_joint(command, motors) &&
      command.activation_epoch > source_epoch &&
      command.activation_epoch >= minimum_epoch &&
      command.activation_epoch > highest_rejected_active_epoch;
}

bool command_epoch_is_acceptable(
    const GuiCommand& command, const CommandSafetyState& safety,
    const std::vector<MotorRuntime>& motors) {
  return command_releases_owned_domain(command, motors) ||
      (command.activation_epoch >= safety.last_seen_activation_epoch &&
       command.activation_epoch >= safety.minimum_activation_epoch &&
       command.activation_epoch > safety.highest_rejected_active_epoch);
}

void observe_rejected_active_epoch(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors,
    CommandSafetyState& safety) {
  if (!is_position_holding_mode(command.mode) ||
      !command_selects_owned_joint(command, motors))
    return;
  safety.highest_rejected_active_epoch = std::max(
      safety.highest_rejected_active_epoch, command.activation_epoch);
  safety.last_seen_activation_epoch = std::max(
      safety.last_seen_activation_epoch, command.activation_epoch);
}

bool healthy_logical_position_for_joint(
    const std::vector<MotorRuntime>& motors, int joint_index,
    double& logical_position) {
  std::vector<double> positions;
  const auto now = Clock::now();
  for (const auto& motor : motors) {
    if (motor.joint_index != joint_index) continue;
    if (!motor.reference_ready || !motor.valid || motor.fault_latched ||
        motor.merror != 0 || motor.temperature < 0 ||
        motor.temperature >= g_thermal_policy.thermal_stop_c ||
        motor.previous_feedback_at == Clock::time_point{} ||
        std::chrono::duration<double>(
            now - motor.previous_feedback_at).count() >
            kFixedHoldFeedbackFreshSeconds)
      return false;
    const double position = motor.sign *
        (motor.unwrapped - motor.reference) / kGear;
    if (!within_mechanical_feedback_envelope(joint_index, position))
      return false;
    positions.push_back(position);
  }
  if (positions.empty()) return false;
  const auto minmax = std::minmax_element(positions.begin(), positions.end());
  if (positions.size() > 1U &&
      *minmax.second - *minmax.first > kJ2SyncLimit)
    return false;
  logical_position = std::accumulate(
      positions.begin(), positions.end(), 0.0) /
      static_cast<double>(positions.size());
  return std::isfinite(logical_position);
}

void validate_and_observe_hand_guidance(
    GuiCommand& command, const std::vector<MotorRuntime>& motors, CommandSafetyState& safety) {
  if (command_releases_owned_domain(command, motors)) {
    safety.guidance_bound = safety.guidance_active = safety.guidance_paused = false;
    safety.guidance_origin_bound = false;
    safety.guidance_paused_reason.clear();
    return;
  }
  if (safety.guidance_origin_bound &&
      command.gravity_authority.empirical_envelope_sha256 == safety.guidance_press.gravity_authority.empirical_envelope_sha256 &&
      command.gravity_authority.empirical_maximum_teach_excursion_deg != safety.guidance_press.gravity_authority.empirical_maximum_teach_excursion_deg)
    throw GuidanceReferenceRejected("COMMAND_GUIDANCE_SESSION_EXCURSION_CHANGED");
  if (!command.hand_guidance.present) {
    if (safety.guidance_active)
      throw std::runtime_error("COMMAND_GUIDANCE_EXIT_REQUIRES_V15_HOLD");
    safety.guidance_bound = safety.guidance_paused = false;
    safety.guidance_paused_reason.clear();
    return;
  }
  const bool entering = command.mode == "teach" && !safety.guidance_active;
  const bool initial_hold = command.mode == "hold" && !safety.guidance_bound;
  if (entering || initial_hold) {
    const auto& previous = safety.last_accepted_command;
    if (previous.mode != "hold" || command.activation_epoch <= previous.activation_epoch ||
        command.source_instance_id != previous.source_instance_id ||
        command.active_joint_mask != previous.active_joint_mask || command.kp != previous.kp || command.kd != previous.kd ||
        (safety.teach_exit_hold.present && !safety.teach_exit_hold.completed) ||
        !all_motor_feedback_healthy(motors))
      throw std::runtime_error("COMMAND_GUIDANCE_ENTRY_REQUIRES_FRESH_HOLD");
    for (const auto& motor : motors) {
      if (!motor.last_frame_valid || motor.returned_mode != kFocMode ||
          std::chrono::duration<double>(command.received_at - motor.previous_feedback_at).count() < 0.0 ||
          std::chrono::duration<double>(command.received_at - motor.previous_feedback_at).count() > kFixedHoldFeedbackFreshSeconds)
        throw std::runtime_error("COMMAND_GUIDANCE_ENTRY_REQUIRES_FRESH_HOLD");
    }
    if ((entering && command.hand_guidance.freeze_reference) || std::any_of(
            command.hand_guidance.velocity_rad_s.begin(), command.hand_guidance.velocity_rad_s.end(),
            [](double value) { return value != 0.0; }))
      throw GuidanceReferenceRejected("COMMAND_GUIDANCE_REF_ENTRY_REQUIRES_ZERO_VELOCITY");
    // The origin bounds the whole session, not the current press. Keep it
    // unchanged on re-entry, including after an ordinary fixed HOLD.
    const bool session_origin_bound = safety.guidance_origin_bound && safety.guidance_press.hand_guidance.present &&
        safety.guidance_press.gravity_authority.empirical_envelope_sha256 ==
            command.gravity_authority.empirical_envelope_sha256;
    if (session_origin_bound &&
        safety.guidance_press.hand_guidance.origin_rad != command.hand_guidance.origin_rad)
      throw GuidanceReferenceRejected("COMMAND_GUIDANCE_REF_SESSION_ORIGIN_CHANGED");
    for (const auto& motor : motors) {
      const auto joint = static_cast<std::size_t>(motor.joint_index);
      if (command.targets[joint] != previous.targets[joint])
        throw GuidanceReferenceRejected("COMMAND_GUIDANCE_REF_ENTRY_HOLD_TARGET_CHANGED");
      if (!session_origin_bound && command.hand_guidance.origin_rad[joint] != previous.targets[joint])
        throw GuidanceReferenceRejected("COMMAND_GUIDANCE_REF_INITIAL_ORIGIN_NOT_HELD");
    }
    safety.guidance_press = command;
    safety.guidance_reference = previous;
    safety.guidance_bound = true;
    // A RETURN_ONLY cancellation may establish an exact-HOLD context for a
    // domain which never admitted the press; it grants no new motion origin.
    safety.guidance_origin_bound = session_origin_bound || !hand_guidance_return_only(command);
    safety.guidance_active = entering;
    safety.guidance_paused = false;
    safety.guidance_paused_reason.clear();
    safety.guidance_started_ns = entering ? monotonic_ns_at(command.received_at) : 0U;
  } else {
    if (!safety.guidance_bound ||
        command.source_instance_id != safety.guidance_press.source_instance_id ||
        command.hand_guidance.origin_rad != safety.guidance_press.hand_guidance.origin_rad ||
        command.active_joint_mask != safety.guidance_press.active_joint_mask ||
        command.kp != safety.guidance_press.kp || command.kd != safety.guidance_press.kd)
      throw std::runtime_error("COMMAND_GUIDANCE_PRESS_CONTEXT_CHANGED");
    if (command.mode == "teach" &&
        (command.activation_epoch != safety.guidance_press.activation_epoch ||
         command.moving_joint_mask != safety.guidance_press.moving_joint_mask))
      throw std::runtime_error("COMMAND_GUIDANCE_PRESS_CONTEXT_CHANGED");
    if (command.mode == "hold" && (command.activation_epoch < safety.guidance_press.activation_epoch ||
        (command.activation_epoch == safety.guidance_press.activation_epoch && safety.guidance_press.mode != "hold")))
      throw std::runtime_error("COMMAND_GUIDANCE_HOLD_REQUIRES_HIGHER_EPOCH");
  }
  const auto& previous = safety.guidance_reference;
  const auto received_ns = monotonic_ns_at(command.received_at);
  if (command.mode == "teach" && command.hand_guidance.freeze_reference) {
    if (std::any_of(command.hand_guidance.velocity_rad_s.begin(), command.hand_guidance.velocity_rad_s.end(),
                    [](double value) { return value != 0.0; }))
      throw GuidanceReferenceRejected("COMMAND_GUIDANCE_FREEZE_REQUIRES_ZERO_VELOCITY");
    if (!safety.guidance_paused) {
      safety.guidance_paused = true;
      safety.guidance_paused_reason = "GUI_RELEASE";
    }
  }
  if (safety.guidance_active && !safety.guidance_paused && received_ns >= safety.guidance_started_ns &&
      received_ns - safety.guidance_started_ns >= 600000000000ULL) {
    safety.guidance_paused = true;
    safety.guidance_paused_reason = "DEADMAN_TIMEOUT";
  }
  if (command.mode == "teach" && safety.guidance_paused) {
    // Fresh source/authority/profile were still checked. Freeze and queued
    // heartbeats cannot move the last accepted goal or restart its clock.
    command.targets = previous.targets;
    command.hand_guidance.velocity_rad_s.fill(0.0);
  }
  const double source_dt = command.source_monotonic_ns > previous.source_monotonic_ns
      ? static_cast<double>(command.source_monotonic_ns - previous.source_monotonic_ns) * 1e-9 : 0.0;
  const bool frozen_owned_refresh = std::all_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
    const auto joint = static_cast<std::size_t>(motor.joint_index);
    return command.targets[joint] == previous.targets[joint] && command.hand_guidance.velocity_rad_s[joint] == 0.0;
  });
  std::set<int> checked;
  for (const auto& motor : motors) {
    const auto joint = static_cast<std::size_t>(motor.joint_index);
    if (!checked.insert(motor.joint_index).second) continue;
    double actual = 0.0;
    if (!motor.last_frame_valid || !healthy_logical_position_for_joint(motors, motor.joint_index, actual))
      throw std::runtime_error("COMMAND_GUIDANCE_FEEDBACK_UNHEALTHY");
    if (entering && std::abs(command.targets[joint] - actual) > 2.0 * kPi / 180.0 + 1e-12)
      throw GuidanceReferenceRejected("COMMAND_GUIDANCE_REF_ENTRY_TARGET_NOT_CURRENT");
    if (!within_model_command_envelope(motor.joint_index, command.targets[joint]) ||
        !within_model_command_envelope(motor.joint_index, command.hand_guidance.origin_rad[joint]))
      throw GuidanceReferenceRejected("COMMAND_GUIDANCE_REF_MODEL_LIMIT");
    if (command.mode == "hold") {
      if (command.targets[joint] != previous.targets[joint] || command.hand_guidance.velocity_rad_s[joint] != 0.0)
        throw GuidanceReferenceRejected("COMMAND_GUIDANCE_REF_HOLD_ACK_MISMATCH");
      safety.fixed_target_valid[joint] = true;
      safety.fixed_target_epoch[joint] = command.activation_epoch;
      safety.fixed_target[joint] = command.targets[joint];
      continue;
    }
    if (!command.moving_joint_mask[joint] &&
        (command.targets[joint] != safety.last_accepted_command.targets[joint] ||
         command.hand_guidance.velocity_rad_s[joint] != 0.0))
      throw std::runtime_error("COMMAND_GUIDANCE_UNSELECTED_HOLD_CHANGED");
    if (std::abs(command.targets[joint] - command.hand_guidance.origin_rad[joint]) > command.hand_guidance.maximum_excursion_deg * kPi / 180.0 + 1e-12 ||
        std::abs(command.hand_guidance.velocity_rad_s[joint]) > 30.0 * kPi / 180.0 + 1e-12)
      throw GuidanceReferenceRejected("COMMAND_GUIDANCE_REF_PROFILE_LIMIT");
    if (!frozen_owned_refresh && !safety.guidance_paused &&
        (std::abs(command.targets[joint] - actual) > 2.0 * kPi / 180.0 + 1e-12 ||
         source_dt <= 0.0 ||
         std::abs(command.targets[joint] - previous.targets[joint]) > 30.0 * kPi / 180.0 * source_dt + 1e-9))
      throw GuidanceReferenceRejected("COMMAND_GUIDANCE_REF_ACTUAL_OR_STEP_LIMIT");
  }
  safety.guidance_reference = command;
  if (command.mode == "hold") {
    safety.guidance_active = safety.guidance_paused = false;
    safety.guidance_paused_reason.clear();
  }
}

std::string hand_guidance_runtime_blocker(
    GuiCommand& command, CommandSafetyState& safety, const std::vector<MotorRuntime>& motors,
    double j2_velocity, std::uint64_t now_ns) {
  if (!safety.guidance_bound)
    return "ASSISTED_TEACH_PRESS_STATE_INVALID";
  for (const auto& motor : motors) {
    const auto joint = static_cast<std::size_t>(motor.joint_index);
    double actual = 0.0;
    if (!motor.last_frame_valid || !healthy_logical_position_for_joint(motors, motor.joint_index, actual))
      return "ASSISTED_TEACH_FEEDBACK_UNHEALTHY";
    if (!within_mechanical_feedback_envelope(motor.joint_index, actual))
      return "ASSISTED_TEACH_MODEL_BOUND";
    const double velocity = joint == 1U ? j2_velocity : motor.integral_encoder_velocity;
    if (!std::isfinite(velocity)) return "ASSISTED_TEACH_ENCODER_VELOCITY_UNAVAILABLE";
    if (std::abs(velocity) > 30.0 * kPi / 180.0) return "HAND_GUIDANCE_ACTUAL_VELOCITY_LIMIT";
  }
  if (safety.guidance_active && !safety.guidance_paused && now_ns >= safety.guidance_started_ns &&
      now_ns - safety.guidance_started_ns >= 600000000000ULL) {
    safety.guidance_paused = true;
    safety.guidance_paused_reason = "DEADMAN_TIMEOUT";
  }
  if (safety.guidance_paused) {
    command.mode = "hold";
    command.targets = safety.guidance_reference.targets;
    command.moving_joint_mask.fill(false);
    command.hand_guidance.velocity_rad_s.fill(0.0);
  }
  return "";
}

void validate_and_observe_assisted_teach(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors,
    CommandSafetyState& safety) {
  auto& exit_hold = safety.teach_exit_hold;
  if (exit_hold.present && command_releases_owned_domain(command, motors)) {
    exit_hold = AssistedTeachExitHold{};
  } else if (exit_hold.present && command.mode != "teach") {
    if (!exit_hold.completed) {
      if (command.mode != "hold" ||
          command.activation_epoch <= exit_hold.press_activation_epoch ||
          command.targets != exit_hold.targets_rad ||
          command.active_joint_mask != safety.teach_command.active_joint_mask ||
          command.kp != safety.teach_command.kp || command.kd != safety.teach_command.kd)
        throw std::runtime_error("COMMAND_TEACH_EXIT_HOLD_ACK_MISMATCH");
      exit_hold.acknowledged = true;
      safety.teach_active = false;
      // This is an acknowledgement of a native captured target, not another
      // moving-angle capture. Bind the normal HOLD record to that exact goal.
      for (const auto& motor : motors) {
        const auto joint = static_cast<std::size_t>(motor.joint_index);
        safety.fixed_target_valid[joint] = true;
        safety.fixed_target_epoch[joint] = command.activation_epoch;
        safety.fixed_target[joint] = exit_hold.targets_rad[joint];
      }
      return;
    }
    if (command.targets != exit_hold.targets_rad || command.mode != "hold")
      exit_hold = AssistedTeachExitHold{};
  }
  if (safety.teach_active && command.mode != "teach") {
    if (!command_releases_owned_domain(command, motors)) {
      if (command.mode != "hold" ||
          command.activation_epoch <= safety.teach_command.activation_epoch ||
          command.active_joint_mask != safety.teach_command.active_joint_mask)
        throw std::runtime_error("COMMAND_TEACH_RELEASE_REQUIRES_NEW_HOLD");
      for (std::size_t joint = 0; joint < 6U; ++joint)
        if (!safety.teach_command.moving_joint_mask[joint] &&
            command.targets[joint] != safety.teach_command.targets[joint])
          throw std::runtime_error("COMMAND_TEACH_RELEASE_CHANGED_OTHER_HOLD");
      for (const auto& motor : motors) {
        const auto joint = static_cast<std::size_t>(motor.joint_index);
        if (!safety.teach_command.moving_joint_mask[joint]) continue;
        double actual = 0.0;
        if (!healthy_logical_position_for_joint(motors, motor.joint_index, actual) ||
            std::abs(command.targets[joint] - actual) > kEmpiricalInitialHoldCaptureWindow)
          throw std::runtime_error("COMMAND_TEACH_RELEASE_CAPTURE_FEEDBACK_MISMATCH");
      }
    }
    safety.teach_active = false;
    return;
  }
  if (command.mode != "teach") return;
  if (exit_hold.present && exit_hold.acknowledged && !exit_hold.completed)
    throw std::runtime_error("COMMAND_TEACH_EXIT_HOLD_STILL_STOPPING");
  if (!std::all_of(command.active_joint_mask.begin(), command.active_joint_mask.end(), [](bool active) { return active; }) ||
      std::count(command.moving_joint_mask.begin(), command.moving_joint_mask.end(), true) != 1 ||
      command.moving_joint_mask[5] ||
      !std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
        return command.moving_joint_mask[static_cast<std::size_t>(motor.joint_index)];
      }))
    throw std::runtime_error("COMMAND_TEACH_OWNED_MASK_INVALID");
  const auto& authority = command.gravity_authority;
  if (!authority.present || authority.official_continuous_authority ||
      !authority.empirical_assisted_teach_authorized ||
      !authority.empirical_position_validation_authorized ||
      authority.empirical_stage_index != 4U ||
      std::abs(authority.gravity_scale - 1.0) > 1e-6 ||
      std::abs(authority.gravity_scale_target - 1.0) > 1e-12)
    throw std::runtime_error("COMMAND_GRAVITY_TEACH_AUTHORITY_MISSING");
  if (safety.teach_active) {
    const auto& frozen = safety.teach_command;
    const auto& old_authority = frozen.gravity_authority;
    if (command.activation_epoch != frozen.activation_epoch ||
        command.targets != frozen.targets ||
        command.active_joint_mask != frozen.active_joint_mask ||
        command.moving_joint_mask != frozen.moving_joint_mask ||
        command.kp != frozen.kp || command.kd != frozen.kd ||
        authority.empirical_maximum_teach_seconds != old_authority.empirical_maximum_teach_seconds ||
        authority.empirical_maximum_teach_excursion_deg != old_authority.empirical_maximum_teach_excursion_deg ||
        authority.empirical_maximum_teach_velocity_deg_s != old_authority.empirical_maximum_teach_velocity_deg_s)
      throw std::runtime_error("COMMAND_TEACH_PRESS_MUTATED");
    return;
  }
  const auto& previous = safety.last_accepted_command;
  const double previous_age = std::chrono::duration<double>(
      command.received_at - previous.received_at).count();
  if (!previous.received || previous.mode != "hold" ||
      command.activation_epoch <= previous.activation_epoch ||
      previous_age < 0.0 || previous_age > kLeaseSeconds ||
      previous.active_joint_mask != command.active_joint_mask ||
      previous.kp != command.kp || previous.kd != command.kd ||
      !previous.gravity_authority.empirical_position_validation_authorized ||
      previous.gravity_authority.empirical_envelope_sha256 != authority.empirical_envelope_sha256 ||
      previous.gravity_authority.source_instance_id != authority.source_instance_id ||
      previous.gravity_authority.session_id != authority.session_id ||
      previous.gravity_authority.state_instance_id != authority.state_instance_id ||
      !selected_owned_motors_confirmed_foc(command, motors))
    throw std::runtime_error("COMMAND_TEACH_ESTABLISHED_HOLD_REQUIRED");
  for (std::size_t joint = 0; joint < 6U; ++joint) {
    if (!command.moving_joint_mask[joint]) {
      if (command.targets[joint] != previous.targets[joint])
        throw std::runtime_error("COMMAND_TEACH_CHANGED_OTHER_HOLD");
      continue;
    }
    double actual = 0.0;
    if (!healthy_logical_position_for_joint(motors, static_cast<int>(joint), actual) ||
        std::abs(command.targets[joint] - actual) > kEmpiricalInitialHoldCaptureWindow)
      throw std::runtime_error("COMMAND_TEACH_CAPTURE_FEEDBACK_MISMATCH");
    for (const auto& motor : motors)
      if (motor.joint_index == static_cast<int>(joint) &&
          (!motor.speed_ready || !std::isfinite(motor.integral_encoder_velocity)))
        throw std::runtime_error("COMMAND_TEACH_ENCODER_VELOCITY_UNAVAILABLE");
  }
  safety.teach_command = command;
  safety.teach_started_monotonic_ns = monotonic_ns_at(command.received_at);
  safety.teach_active = true;
  safety.teach_exit_hold = AssistedTeachExitHold{};
}

std::string assisted_teach_runtime_blocker(
    GuiCommand& command, CommandSafetyState& safety,
    const std::vector<MotorRuntime>& motors, double j2_encoder_velocity,
    std::uint64_t now_ns) {
  auto& exit_hold = safety.teach_exit_hold;
  if (command.mode != "teach" && !exit_hold.present) return "";
  if (exit_hold.completed && command.mode != "teach") return "";
  if (command.mode == "teach" && (!safety.teach_active || command.activation_epoch != safety.teach_command.activation_epoch ||
      safety.teach_started_monotonic_ns == 0U || now_ns < safety.teach_started_monotonic_ns)
      )
    return "ASSISTED_TEACH_PRESS_STATE_INVALID";
  const auto& authority = safety.teach_command.gravity_authority;
  for (const auto& motor : motors) {
    const auto joint = static_cast<std::size_t>(motor.joint_index);
    if (!safety.teach_command.moving_joint_mask[joint]) continue;
    double actual = 0.0;
    if (!motor.last_frame_valid || !healthy_logical_position_for_joint(motors, motor.joint_index, actual))
      return "ASSISTED_TEACH_FEEDBACK_UNHEALTHY";
    if (!within_model_command_envelope(motor.joint_index, actual))
      return "ASSISTED_TEACH_MODEL_BOUND";
    const double velocity = joint == 1U ? j2_encoder_velocity : motor.integral_encoder_velocity;
    if (!std::isfinite(velocity))
      return "ASSISTED_TEACH_ENCODER_VELOCITY_UNAVAILABLE";
    if (!exit_hold.present) {
      std::string reason;
      if (static_cast<double>(now_ns - safety.teach_started_monotonic_ns) * 1e-9 >= authority.empirical_maximum_teach_seconds)
        reason = "TIME_LIMIT";
      else if (std::abs(actual - safety.teach_command.targets[joint]) > authority.empirical_maximum_teach_excursion_deg * kPi / 180.0)
        reason = "EXCURSION_LIMIT";
      else if (std::abs(velocity) > authority.empirical_maximum_teach_velocity_deg_s * kPi / 180.0)
        reason = "VELOCITY_LIMIT";
      if (!reason.empty()) {
        exit_hold.present = true;
        exit_hold.joint_index = joint;
        exit_hold.press_activation_epoch = safety.teach_command.activation_epoch;
        exit_hold.started_monotonic_ns = now_ns;
        exit_hold.deadline_monotonic_ns = now_ns + kTeachStoppingHoldNs;
        exit_hold.reason = reason;
        exit_hold.targets_rad = safety.teach_command.targets;
        exit_hold.targets_rad[joint] = actual;
        exit_hold.initial_velocity_rad_s = velocity;
        std::cerr << "ASSISTED_TEACH_EXIT_HOLD reason=" << reason
                  << " joint=J" << joint + 1U
                  << " press_epoch=" << exit_hold.press_activation_epoch
                  << " started_monotonic_ns=" << now_ns
                  << " deadline_monotonic_ns=" << exit_hold.deadline_monotonic_ns
                  << " captured_target_rad=" << actual
                  << " initial_encoder_velocity_rad_s=" << velocity << std::endl;
      }
    }
    if (exit_hold.present && !exit_hold.completed) {
      if (now_ns < exit_hold.started_monotonic_ns)
        return "ASSISTED_TEACH_PRESS_STATE_INVALID";
      const double error = std::abs(actual - exit_hold.targets_rad[joint]);
      const bool within_normal_velocity = std::abs(velocity) <=
          authority.empirical_maximum_teach_velocity_deg_s * kPi / 180.0;
      const auto hard_stop = [&](const char* reason) {
        std::ostringstream detail;
        detail << std::setprecision(17) << "ASSISTED_TEACH_HARD_STOP reason=" << reason
               << " joint=J" << joint + 1U << " now_monotonic_ns=" << now_ns
               << " started_monotonic_ns=" << exit_hold.started_monotonic_ns
               << " deadline_monotonic_ns=" << exit_hold.deadline_monotonic_ns
               << " feedback_source_monotonic_ns=" << motor.last_valid_feedback_monotonic_ns
               << " actual_rad=" << actual << " captured_target_rad=" << exit_hold.targets_rad[joint]
               << " encoder_velocity_rad_s=" << velocity
               << " maximum_velocity_rad_s=" << authority.empirical_maximum_teach_velocity_deg_s * kPi / 180.0
               << " press_epoch=" << exit_hold.press_activation_epoch
               << " control_epoch=" << command.activation_epoch;
        std::cerr << detail.str() << std::endl;
        return std::string(reason);
      };
      if (error > kTeachStoppingErrorRad) {
        if (!within_normal_velocity)
          return hard_stop("ASSISTED_TEACH_STOP_ERROR_LIMIT");
        if (!exit_hold.restricted)
          std::cerr << "ASSISTED_TEACH_RESTRICTED_HOLD joint=J" << joint + 1U
                    << " captured_target_rad=" << exit_hold.targets_rad[joint]
                    << " actual_rad=" << actual
                    << " encoder_velocity_rad_s=" << velocity << std::endl;
        exit_hold.restricted = true;
      }
      if (exit_hold.restricted && !within_normal_velocity)
        return hard_stop("ASSISTED_TEACH_RESTRICTED_VELOCITY_LIMIT");
      if (now_ns >= exit_hold.deadline_monotonic_ns) {
        if (!within_normal_velocity)
          return hard_stop("ASSISTED_TEACH_STOP_VELOCITY_TIMEOUT");
        if (!exit_hold.restricted) exit_hold.completed = exit_hold.acknowledged;
      }
      if (exit_hold.restricted) {
        const bool settled = error <= kTeachRestrictedRearmErrorRad &&
            std::abs(velocity) <= kTeachRestrictedRearmVelocity;
        if (!settled) {
          exit_hold.stable_since_monotonic_ns = 0;
          exit_hold.stable_frames = 0;
        } else {
          if (exit_hold.stable_since_monotonic_ns == 0U)
            exit_hold.stable_since_monotonic_ns = now_ns;
          exit_hold.stable_frames = std::min(exit_hold.stable_frames + 1, 50);
          if (exit_hold.acknowledged && now_ns >= exit_hold.deadline_monotonic_ns &&
              exit_hold.stable_frames >= 50 && now_ns - exit_hold.stable_since_monotonic_ns >= kTeachRestrictedStableNs) {
            exit_hold.restricted = false;
            exit_hold.completed = true;
          }
        }
      }
    }
  }
  if (exit_hold.present && command.mode == "teach") {
    command.mode = "hold";
    command.targets = exit_hold.targets_rad;
    command.moving_joint_mask.fill(false);
  }
  return "";
}

bool uses_assisted_teach_damping(
    const GuiCommand& command, const std::string& mode, std::size_t joint,
    const AssistedTeachExitHold& exit_hold, std::uint64_t now_ns) {
  return (command.hand_guidance.present && (mode == "teach" || mode == "hold") && command.active_joint_mask[joint]) ||
      joint_is_assisted_teach(command, mode, joint) ||
      (mode == "hold" && exit_hold.present && !exit_hold.completed &&
       exit_hold.joint_index == joint && command.targets == exit_hold.targets_rad &&
       now_ns >= exit_hold.started_monotonic_ns && now_ns < exit_hold.deadline_monotonic_ns);
}

bool freezes_restricted_hold_integral(
    const GuiCommand& command, const std::string& mode, std::size_t joint,
    const AssistedTeachExitHold& exit_hold) {
  return mode == "hold" && exit_hold.present && exit_hold.restricted &&
      exit_hold.joint_index == joint && command.targets == exit_hold.targets_rad;
}

bool freezes_assisted_teach_integral(
    const GuiCommand& command, const std::string& mode, std::size_t joint,
    const CommandSafetyState& safety, std::uint64_t now_ns) {
  // Encoder damping remains useful after the exact HOLD ACK. Integral learning
  // is frozen only until that ACK has ended the guidance press.
  return (command.hand_guidance.present
      ? safety.guidance_active && command.active_joint_mask[joint]
      : uses_assisted_teach_damping(command, mode, joint, safety.teach_exit_hold, now_ns)) ||
      freezes_restricted_hold_integral(command, mode, joint, safety.teach_exit_hold);
}

void apply_assisted_teach_reference(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors,
    std::array<double, 6>& q, std::array<double, 6>& dq) {
  for (const auto& motor : motors) {
    const auto joint = static_cast<std::size_t>(motor.joint_index);
    if (!joint_is_assisted_teach(command, command.mode, joint)) continue;
    double actual = 0.0;
    if (!healthy_logical_position_for_joint(motors, motor.joint_index, actual))
      throw std::runtime_error("ASSISTED_TEACH_REFERENCE_FEEDBACK_UNHEALTHY");
    q[joint] = actual;
    dq[joint] = 0.0;
  }
}

void apply_hand_guidance_reference(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors,
    std::array<double, 6>& q, std::array<double, 6>& dq) {
  for (const auto& motor : motors) {
    const auto joint = static_cast<std::size_t>(motor.joint_index);
    q[joint] = command.targets[joint];
    dq[joint] = command.mode == "teach" ? command.hand_guidance.velocity_rad_s[joint] : 0.0;
  }
}

void validate_first_quintic_start_against_feedback(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors) {
  std::set<int> validated_joints;
  for (const auto& motor : motors) {
    const int joint_index = motor.joint_index;
    const auto joint = static_cast<std::size_t>(joint_index);
    if (!command.active_joint_mask[joint] ||
        !command.moving_joint_mask[joint] ||
        !validated_joints.insert(joint_index).second)
      continue;
    double actual_position_rad = 0.0;
    if (!healthy_logical_position_for_joint(
            motors, joint_index, actual_position_rad))
      throw std::runtime_error(
          "COMMAND_QUINTIC_START_FEEDBACK_UNHEALTHY");
    // Reuse the existing fixed-HOLD capture window and its 100 ms feedback
    // freshness predicate.  This is a defence-in-depth start binding, not a
    // new motor/load rating and not a replacement for collision authorization.
    if (std::abs(
            command.quintic.start_rad[joint] - actual_position_rad) >
        kFixedHoldCaptureWindow + 1e-12)
      throw std::runtime_error("COMMAND_QUINTIC_START_FEEDBACK_MISMATCH");
  }
}

bool maximum_moving_owned_position_error(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors,
    const std::array<bool, 6>& saturated_joint_mask,
    double& maximum_error_rad) {
  bool observed = false;
  maximum_error_rad = 0.0;
  std::set<int> observed_joints;
  for (const auto& motor : motors) {
    if (!observed_joints.insert(motor.joint_index).second) continue;
    const auto joint = static_cast<std::size_t>(motor.joint_index);
    if (!command.active_joint_mask[joint] ||
        !command.moving_joint_mask[joint] ||
        !saturated_joint_mask[joint])
      continue;
    double actual_position_rad = 0.0;
    if (!healthy_logical_position_for_joint(
            motors, motor.joint_index, actual_position_rad))
      return false;
    maximum_error_rad = std::max(
        maximum_error_rad,
        std::abs(command.targets[joint] - actual_position_rad));
    observed = true;
  }
  return observed && std::isfinite(maximum_error_rad);
}

void validate_and_observe_fixed_hold_targets(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors,
    CommandSafetyState& safety,
    const std::array<bool, 6>* position_arrived_once = nullptr,
    const std::array<bool, 6>* position_endpoint_reached = nullptr,
    const std::array<bool, 6>* position_tracking = nullptr,
    const std::array<std::uint64_t, 6>* position_tracking_epoch = nullptr,
    const std::array<double, 6>* position_tracking_target = nullptr) {
  CommandSafetyState candidate = safety;
  std::set<int> owned_joints;
  for (const auto& motor : motors) owned_joints.insert(motor.joint_index);
  if (command_releases_owned_domain(command, motors)) {
    for (const int joint_index : owned_joints) {
      const auto joint = static_cast<std::size_t>(joint_index);
      candidate.fixed_target_valid[static_cast<std::size_t>(joint_index)] =
          false;
      candidate.position_target_valid[joint] = false;
    }
    safety = std::move(candidate);
    return;
  }

  for (const int joint_index : owned_joints) {
    const std::size_t joint = static_cast<std::size_t>(joint_index);
    const bool joint_selected = command.active_joint_mask[joint];
    const bool moving_position_requested = joint_selected &&
        command.mode == "position" && command.moving_joint_mask[joint];
    const bool fixed_target_requested =
        joint_selected &&
        joint_uses_fixed_hold_target(command, command.mode, joint);
    // Preserve a fixed-target record across moving/deselected packets in the
    // same epoch. Otherwise a sender could toggle the moving bit, erase the
    // record, and recapture a displaced angle without advancing the epoch.
    if (!joint_selected) continue;
    if (moving_position_requested) {
      if (candidate.fixed_target_valid[joint] &&
          candidate.fixed_target_epoch[joint] == command.activation_epoch &&
          std::abs(command.targets[joint] - candidate.fixed_target[joint]) >
              kFixedHoldRepeatTolerance)
        throw std::runtime_error(
            "COMMAND_FIXED_HOLD_TARGET_CHANGED_SAME_EPOCH");
      if (candidate.position_target_valid[joint] &&
          candidate.position_target_epoch[joint] == command.activation_epoch &&
          std::abs(command.targets[joint] - candidate.position_target[joint]) >
              kFixedHoldRepeatTolerance)
        throw std::runtime_error(
            "COMMAND_POSITION_TARGET_CHANGED_SAME_EPOCH");
      candidate.position_target_valid[joint] = true;
      candidate.position_target_epoch[joint] = command.activation_epoch;
      candidate.position_target[joint] = command.targets[joint];
      continue;
    }
    if (!fixed_target_requested) continue;
    if (candidate.fixed_target_valid[joint] &&
        candidate.fixed_target_epoch[joint] == command.activation_epoch) {
      if (std::abs(command.targets[joint] - candidate.fixed_target[joint]) >
          kFixedHoldRepeatTolerance)
        throw std::runtime_error("COMMAND_FIXED_HOLD_TARGET_CHANGED_SAME_EPOCH");
      continue;
    }

    // Arrival/endpoint flags belong to the runtime profile that the main loop
    // has actually observed.  A UDP receive batch may contain POSITION E+1
    // immediately followed by HOLD E+1 before that loop has reset the flags
    // left by POSITION E.  Bind the proof to both epoch and target so stale
    // completion state can never authorize the new HOLD.
    const bool verified_position_arrival = position_arrived_once != nullptr &&
        position_endpoint_reached != nullptr &&
        position_tracking != nullptr &&
        position_tracking_epoch != nullptr &&
        position_tracking_target != nullptr &&
        (*position_tracking)[joint] &&
        (*position_tracking_epoch)[joint] == command.activation_epoch &&
        std::abs((*position_tracking_target)[joint] - command.targets[joint]) <=
            kFixedHoldRepeatTolerance &&
        (*position_arrived_once)[joint] &&
        (*position_endpoint_reached)[joint];
    if (verified_position_arrival &&
        candidate.position_target_valid[joint] &&
        candidate.position_target_epoch[joint] == command.activation_epoch &&
        std::abs(command.targets[joint] - candidate.position_target[joint]) <=
            kFixedHoldRepeatTolerance) {
      // GUI arrival changes POSITION to fixed HOLD without changing target or
      // epoch. Keep that exact endpoint authoritative even if an external
      // displacement is already outside the ordinary capture window.
      candidate.fixed_target_valid[joint] = true;
      candidate.fixed_target_epoch[joint] = command.activation_epoch;
      candidate.fixed_target[joint] = command.targets[joint];
      continue;
    }

    double actual = 0.0;
    if (!healthy_logical_position_for_joint(motors, joint_index, actual))
      throw std::runtime_error("COMMAND_FIXED_HOLD_CAPTURE_FEEDBACK_UNHEALTHY");
    const double capture_window =
        command_uses_empirical_gravity_authority(command) &&
            !command.gravity_authority
                 .empirical_position_validation_authorized
        ? kEmpiricalInitialHoldCaptureWindow
        : kFixedHoldCaptureWindow;
    if (std::abs(command.targets[joint] - actual) >
        capture_window + 1e-12)
      throw std::runtime_error("COMMAND_FIXED_HOLD_CAPTURE_WINDOW");
    candidate.fixed_target_valid[joint] = true;
    candidate.fixed_target_epoch[joint] = command.activation_epoch;
    candidate.fixed_target[joint] = command.targets[joint];
  }
  safety = std::move(candidate);
}

void observe_valid_command(
    const GuiCommand& command, const std::vector<MotorRuntime>& motors,
    CommandSafetyState& safety) {
  safety.last_accepted_command = command;
  safety.last_seen_activation_epoch = std::max(
      safety.last_seen_activation_epoch, command.activation_epoch);
  if (command_releases_owned_domain(command, motors))
    safety.minimum_activation_epoch = std::max(
        safety.minimum_activation_epoch,
        safety.last_seen_activation_epoch + 1U);
}

void observe_interarrival_lease(
    const GuiCommand& current, Clock::time_point received_at,
    const std::vector<MotorRuntime>& motors, CommandSafetyState& safety) {
  const bool active_requested = current.received &&
      is_position_holding_mode(current.mode) &&
      command_selects_owned_joint(current, motors);
  const bool lease_fresh = current.received &&
      std::chrono::duration<double>(received_at - current.received_at).count() <=
          kLeaseSeconds;
  safety.minimum_activation_epoch = minimum_epoch_after_lease(
      safety.minimum_activation_epoch, lease_fresh, active_requested,
      current.activation_epoch);
}

struct QuinticSampleClock {
  std::uint64_t sample_index = 0;
  const char* state = "PREPARED";
  bool endpoint = false;
};

QuinticSampleClock quintic_sample_clock(
    const QuinticTrajectoryDescriptor& trajectory,
    std::uint64_t now_monotonic_ns);
void apply_quintic_reference(
    int joint, const GuiCommand& command, const QuinticSampleClock& sample,
    std::array<double, 6>& q_command,
    std::array<double, 6>& dq_command);

std::string fresh_source_self_test_payload(const std::string& text) {
  auto value = nlohmann::json::parse(text);
  value["source_instance_id"] =
      "0123456789abcdef0123456789abcdef";
  value["source_monotonic_ns"] = monotonic_ns_at(Clock::now());
  return value.dump();
}

int enforce_brake_only_wire_mode(bool brake_only, int requested_mode) {
  if (brake_only && requested_mode != kBrakeMode)
    throw std::runtime_error("BRAKE_ONLY_NON_BRAKE_BLOCKED");
  return requested_mode;
}

void hand_guidance_self_test(const nlohmann::json& legacy_packet);

void assisted_teach_self_test() {
  const auto now = Clock::now();
  const auto ns = monotonic_ns_at(now);
  nlohmann::json packet = {
      {"schema", "go-m8010-gui-command/1.4"}, {"mode", "teach"},
      {"source_instance_id", std::string(32U, '1')}, {"source_monotonic_ns", ns},
      {"sequence", 1U}, {"activation_epoch", 2U},
      {"targets_rad", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {"active_joint_mask", {true, true, true, true, true, true}},
      {"moving_joint_mask", {false, false, false, true, false, false}},
      {"kp", {0.5, 1.0, 0.6, 0.5, 0.5, 0.0}},
      {"kd", {0.05, 0.10, 0.05, 0.05, 0.05, 0.0}},
      {"feedforward_nm", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {"maximum_velocity_rad_s", 0.08}, {"maximum_acceleration_rad_s2", 0.3},
      {"gravity_authority", {
          {"schema", kGravityAuthoritySchema}, {"source_instance_id", std::string(32U, '2')},
          {"sequence", 1U}, {"source_monotonic_ns", ns},
          {"model_sha256", kProductionModelSha256}, {"gravity_config_sha256", kGravityConfigSha256},
          {"session_id", "teach-self-test"}, {"state_instance_id", "teach-self-test-instance"},
          {"gravity_scale", 1.0}, {"gravity_scale_target", 1.0},
          {"feedforward_nm", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
          {"authority_class", kEmpiricalAuthorityClass}, {"rating_classification", kEmpiricalRatingClassification},
          {"empirical_envelope_id", "v15-31b-empirical-01234567890123456789"},
          {"empirical_envelope_sha256", std::string(64U, 'a')},
          {"empirical_envelope_expires_at_utc", "2099-01-01T00:00:00Z"},
          {"empirical_envelope_deadline_monotonic_ns", ns + 60000000000ULL},
          {"anchor_sha256", std::string(64U, 'b')}, {"empirical_stage_index", 4U},
          {"empirical_position_validation_authorized", true},
          {"empirical_maximum_position_segment_seconds", 15.0},
          {"empirical_maximum_abs_position_segment_deg", 5.0},
          {"empirical_assisted_teach_authorized", true},
          {"empirical_maximum_teach_excursion_deg", 5.0},
          {"empirical_maximum_teach_seconds", 30.0},
          {"empirical_maximum_teach_velocity_deg_s", 5.0},
          {"empirical_allowed_teach_joints", {"J1", "J2", "J3", "J4", "J5"}}}}};
  GuiCommand teach;
  parse_command(packet.dump(), teach, now);
  auto require_rejected = [](const auto& action) {
    bool rejected = false;
    try { action(); } catch (const std::runtime_error&) { rejected = true; }
    if (!rejected) throw std::runtime_error("ASSISTED_TEACH_NEGATIVE_SELF_TEST_FAILED");
  };
  for (const auto& item : std::array<std::pair<const char*, double>, 3>{{
      {"empirical_maximum_teach_seconds", 30.01},
      {"empirical_maximum_teach_excursion_deg", 5.01},
      {"empirical_maximum_teach_velocity_deg_s", 5.01}}}) {
    auto bad = packet; bad["gravity_authority"][item.first] = item.second;
    require_rejected([&] { GuiCommand parsed; parse_command(bad.dump(), parsed, now); });
  }
  for (int case_index = 0; case_index < 5; ++case_index) {
    auto bad = packet;
    if (case_index == 0) bad["gravity_authority"].erase("empirical_assisted_teach_authorized");
    if (case_index == 1) bad["gravity_authority"]["empirical_assisted_teach_authorized"] = false;
    if (case_index == 2) bad["gravity_authority"]["empirical_position_validation_authorized"] = false;
    if (case_index == 3) bad["moving_joint_mask"][2] = true;
    if (case_index == 4) bad["schema"] = "go-m8010-gui-command/1.2";
    require_rejected([&] { GuiCommand parsed; parse_command(bad.dump(), parsed, now); });
  }
  auto motors = make_motors("j345");
  for (auto& motor : motors) {
    motor.reference_ready = motor.valid = motor.last_frame_valid = motor.speed_ready = true;
    motor.previous_feedback_at = now; motor.merror = 0; motor.temperature = 30;
    motor.returned_mode = kFocMode; motor.integral_encoder_velocity = 0.0;
  }
  CommandSafetyState safety;
  require_rejected([&] { auto candidate = safety; validate_and_observe_assisted_teach(teach, motors, candidate); });
  GuiCommand hold = teach;
  hold.schema = "go-m8010-gui-command/1.2"; hold.mode = "hold";
  hold.activation_epoch = 1U; hold.moving_joint_mask.fill(false);
  hold.gravity_authority.empirical_assisted_teach_authorized = false;
  validate_and_observe_fixed_hold_targets(hold, motors, safety);
  observe_valid_command(hold, motors, safety);
  auto bad = teach; bad.targets[2] = 0.001;
  require_rejected([&] { auto candidate = safety; validate_and_observe_assisted_teach(bad, motors, candidate); });
  bad = teach; bad.targets[3] = kEmpiricalInitialHoldCaptureWindow + 0.001;
  require_rejected([&] { auto candidate = safety; validate_and_observe_assisted_teach(bad, motors, candidate); });
  motors[1].returned_mode = kBrakeMode;
  require_rejected([&] { auto candidate = safety; validate_and_observe_assisted_teach(teach, motors, candidate); });
  motors[1].returned_mode = kFocMode;
  validate_and_observe_assisted_teach(teach, motors, safety);
  validate_and_observe_fixed_hold_targets(teach, motors, safety);
  observe_valid_command(teach, motors, safety);
  const auto first_press_ns = safety.teach_started_monotonic_ns;
  GuiCommand heartbeat = teach; heartbeat.received_at += std::chrono::milliseconds(500);
  validate_and_observe_assisted_teach(heartbeat, motors, safety);
  if (safety.teach_started_monotonic_ns != first_press_ns)
    throw std::runtime_error("ASSISTED_TEACH_HEARTBEAT_RESTARTED_TIMER");
  for (int case_index = 0; case_index < 4; ++case_index) {
    bad = heartbeat;
    if (case_index == 0) ++bad.activation_epoch;
    if (case_index == 1) bad.targets[3] = 0.001;
    if (case_index == 2) { bad.moving_joint_mask[3] = false; bad.moving_joint_mask[2] = true; }
    if (case_index == 3) bad.kd[3] = 0.01;
    require_rejected([&] { auto candidate = safety; validate_and_observe_assisted_teach(bad, motors, candidate); });
  }
  if (!assisted_teach_runtime_blocker(teach, safety, motors, 0.0, first_press_ns + 1000U).empty())
    throw std::runtime_error("ASSISTED_TEACH_RUNNING_SELF_TEST_FAILED");
  motors[1].unwrapped = motors[1].sign * kGear * 0.01;
  motors[1].integral_encoder_velocity = std::numeric_limits<double>::quiet_NaN();
  if (assisted_teach_runtime_blocker(teach, safety, motors, 0.0, ns + 1000U) != "ASSISTED_TEACH_ENCODER_VELOCITY_UNAVAILABLE")
    throw std::runtime_error("ASSISTED_TEACH_UNKNOWN_SPEED_SELF_TEST_FAILED");
  motors[1].integral_encoder_velocity = 0.0;
  auto q = teach.targets; std::array<double, 6> dq{};
  apply_assisted_teach_reference(teach, motors, q, dq);
  if (std::abs(q[3] - 0.01) > 1e-12 || q[2] != teach.targets[2] || q[4] != teach.targets[4] || teach.targets[3] != 0.0)
    throw std::runtime_error("ASSISTED_TEACH_FOLLOW_SELF_TEST_FAILED");
  BoundedHoldIntegralState integral{0.12, 20};
  for (int frame = 0; frame < 100; ++frame) (void)frozen_hold_integral_wire(integral, 0.35);
  if (integral.accumulator_nm != 0.12 || integral.dwell_frames != 20)
    throw std::runtime_error("ASSISTED_TEACH_INTEGRAL_FREEZE_SELF_TEST_FAILED");
  GuiCommand released = hold; released.activation_epoch = 3U; released.targets[3] = q[3];
  bad = released; bad.targets[2] = 0.001;
  require_rejected([&] { auto candidate = safety; validate_and_observe_assisted_teach(bad, motors, candidate); });
  validate_and_observe_assisted_teach(released, motors, safety);
  validate_and_observe_fixed_hold_targets(released, motors, safety);
  if (safety.teach_active || safety.fixed_target[3] != q[3] || safety.fixed_target[2] != hold.targets[2])
    throw std::runtime_error("ASSISTED_TEACH_RELEASE_SELF_TEST_FAILED");
  auto pair = make_motors("j2");
  for (std::size_t i = 0; i < pair.size(); ++i) {
    auto& motor = pair[i]; motor.reference_ready = motor.valid = motor.last_frame_valid = true;
    motor.merror = 0; motor.temperature = 30; motor.previous_feedback_at = Clock::now();
    motor.unwrapped = motor.sign * kGear * (0.01 + 0.002 * static_cast<double>(i));
  }
  GuiCommand pair_teach = teach; pair_teach.active_joint_mask.fill(false); pair_teach.moving_joint_mask.fill(false);
  pair_teach.active_joint_mask[1] = pair_teach.moving_joint_mask[1] = true;
  apply_assisted_teach_reference(pair_teach, pair, q, dq);
  if (std::abs(q[1] - 0.011) > 1e-12 || pair[0].reference != 0.0 || pair[1].reference != 0.0 ||
      !use_j2_hold_protection_limits(pair_teach, "teach"))
    throw std::runtime_error("ASSISTED_TEACH_J2_COMMON_REFERENCE_SELF_TEST_FAILED");

  // Replay the observed manual event (8.6 deg/s at 1.08 deg displacement).
  // Finite manual limits capture one loaded HOLD; they never create a fault
  // or discard the other five immutable goals or the short-lived authority.
  for (const std::string reason : {"TIME_LIMIT", "EXCURSION_LIMIT", "VELOCITY_LIMIT"}) {
    auto event_motors = make_motors("j1");
    auto& motor = event_motors[0];
    motor.reference_ready = motor.valid = motor.last_frame_valid = motor.speed_ready = true;
    motor.previous_feedback_at = Clock::now(); motor.merror = 0; motor.temperature = 30;
    motor.returned_mode = kFocMode;
    const double position = (reason == "EXCURSION_LIMIT" ? -5.01 : -1.08) * kPi / 180.0;
    motor.unwrapped = motor.sign * kGear * position;
    motor.integral_encoder_velocity = reason == "VELOCITY_LIMIT" ? -8.6 * kPi / 180.0 : 0.0;
    GuiCommand event_command = teach;
    event_command.moving_joint_mask.fill(false); event_command.moving_joint_mask[0] = true;
    CommandSafetyState event_safety;
    event_safety.teach_active = true; event_safety.teach_command = event_command;
    event_safety.teach_started_monotonic_ns = ns;
    event_safety.gravity_empirical_active = true;
    event_safety.gravity_empirical_envelope_sha256 = std::string(64U, 'a');
    const auto event_ns = ns + (reason == "TIME_LIMIT" ? 30000000000ULL : 10000000ULL);
    GuiCommand controlled = event_command;
    if (!assisted_teach_runtime_blocker(controlled, event_safety, event_motors, 0.0, event_ns).empty() ||
        controlled.mode != "hold" || controlled.targets[0] != position ||
        !is_position_holding_mode(controlled.mode) || !selected_owned_motors_confirmed_foc(controlled, event_motors) ||
        !event_safety.gravity_empirical_active || !event_safety.spent_gravity_empirical_envelope_sha256.empty() ||
        event_safety.teach_exit_hold.reason != reason)
      throw std::runtime_error("ASSISTED_TEACH_SOFT_EXIT_SELF_TEST_FAILED");
    for (std::size_t joint = 1; joint < 6U; ++joint)
      if (controlled.targets[joint] != event_command.targets[joint])
        throw std::runtime_error("ASSISTED_TEACH_SOFT_EXIT_MOVED_OTHER_GOAL");
    const auto captured = event_safety.teach_exit_hold;
    if (reason == "VELOCITY_LIMIT") {
      // Last five native-stamped HOLD observations before the real 20260908
      // 051622 session trip. Velocity is reconstructed from the transmitted
      // damping and frozen -0.015625 Nm integral, not directly sampled here.
      // The FIRST >2-degree frame's filtered velocity was NOT recorded: this
      // replay cannot establish that the physical failure is necessarily fixed.
      struct TracePoint { std::uint64_t dt_ns; double error_rad; double velocity; };
      const std::array<TracePoint, 5> known_trace{{
          {481107661ULL, 0.030837202519146553, 0.041976390538618465},
          {511114435ULL, 0.032715304408928322, 0.047434039704066781},
          {541067330ULL, 0.032685003026748372, 0.037788614675795494},
          {581049295ULL, 0.03362406338785498, 0.029443282822927654},
          {621145569ULL, 0.034411654502924743, 0.027170242717397427}}};
      auto trace_state = event_safety;
      GuiCommand trace_command = controlled;
      for (const auto& point : known_trace) {
        motor.unwrapped = motor.sign * kGear * (position + point.error_rad);
        motor.integral_encoder_velocity = point.velocity;
        if (!assisted_teach_runtime_blocker(trace_command, trace_state, event_motors, 0.0,
                captured.started_monotonic_ns + point.dt_ns).empty() || trace_command.targets != captured.targets_rad)
          throw std::runtime_error("ASSISTED_TEACH_REAL_PRETRIP_TRACE_CHANGED_HOLD");
      }
      motor.unwrapped = motor.sign * kGear * position;
      motor.integral_encoder_velocity = -8.6 * kPi / 180.0;
    }
    motor.unwrapped += motor.sign * kGear * 0.3 * kPi / 180.0;
    GuiCommand old_heartbeat = event_command;
    validate_and_observe_assisted_teach(old_heartbeat, event_motors, event_safety);
    if (!assisted_teach_runtime_blocker(old_heartbeat, event_safety, event_motors, 0.0, event_ns + 100000000ULL).empty() ||
        old_heartbeat.targets != captured.targets_rad || old_heartbeat.mode != "hold" ||
        event_safety.teach_exit_hold.started_monotonic_ns != captured.started_monotonic_ns ||
        event_safety.teach_exit_hold.deadline_monotonic_ns != captured.deadline_monotonic_ns)
      throw std::runtime_error("ASSISTED_TEACH_SOFT_EXIT_HEARTBEAT_RECAPTURED");
    GuiCommand ack = controlled; ++ack.activation_epoch;
    GuiCommand bad_ack = ack; bad_ack.targets[0] += 0.001;
    require_rejected([&] { auto state = event_safety; validate_and_observe_assisted_teach(bad_ack, event_motors, state); });
    validate_and_observe_assisted_teach(ack, event_motors, event_safety);
    validate_and_observe_fixed_hold_targets(ack, event_motors, event_safety);
    if (!event_safety.teach_exit_hold.acknowledged ||
        event_safety.teach_exit_hold.started_monotonic_ns != captured.started_monotonic_ns ||
        event_safety.fixed_target[0] != position || !uses_assisted_teach_damping(ack, "hold", 0U, captured, event_ns + 100000000ULL))
      throw std::runtime_error("ASSISTED_TEACH_SOFT_EXIT_ACK_RECAPTURED");
    GuiCommand restart = event_command; restart.activation_epoch += 2;
    require_rejected([&] { auto state = event_safety; validate_and_observe_assisted_teach(restart, event_motors, state); });
    auto expired_state = event_safety;
    motor.integral_encoder_velocity = -8.6 * kPi / 180.0;
    if (assisted_teach_runtime_blocker(ack, expired_state, event_motors, 0.0, captured.deadline_monotonic_ns) != "ASSISTED_TEACH_STOP_VELOCITY_TIMEOUT")
      throw std::runtime_error("ASSISTED_TEACH_STOP_DEADLINE_NOT_ENFORCED");
    motor.integral_encoder_velocity = std::numeric_limits<double>::quiet_NaN();
    if (assisted_teach_runtime_blocker(ack, expired_state, event_motors, 0.0, event_ns + 1U) != "ASSISTED_TEACH_ENCODER_VELOCITY_UNAVAILABLE")
      throw std::runtime_error("ASSISTED_TEACH_STOP_ACCEPTED_UNKNOWN_SPEED");
    motor.integral_encoder_velocity = 0.0;
    motor.previous_feedback_at = Clock::now() - std::chrono::milliseconds(101);
    if (assisted_teach_runtime_blocker(ack, expired_state, event_motors, 0.0, event_ns + 1U) != "ASSISTED_TEACH_FEEDBACK_UNHEALTHY")
      throw std::runtime_error("ASSISTED_TEACH_STOP_ACCEPTED_STALE_FEEDBACK");
    motor.previous_feedback_at = Clock::now();
    motor.unwrapped = motor.sign * kGear * (position + 2.01 * kPi / 180.0);
    motor.integral_encoder_velocity = 5.01 * kPi / 180.0;
    if (assisted_teach_runtime_blocker(ack, expired_state, event_motors, 0.0, event_ns + 1U) != "ASSISTED_TEACH_STOP_ERROR_LIMIT")
      throw std::runtime_error("ASSISTED_TEACH_STOP_ERROR_NOT_ENFORCED");
    // Both possible first-crossing speed branches remain explicit: low-speed
    // disturbance keeps fixed HOLD; exceeding the original 5 deg/s stays hard.
    auto restricted_state = event_safety;
    motor.integral_encoder_velocity = 1.6 * kPi / 180.0;
    if (!assisted_teach_runtime_blocker(ack, restricted_state, event_motors, 0.0, event_ns + 1U).empty() ||
        !restricted_state.teach_exit_hold.restricted || restricted_state.teach_exit_hold.completed ||
        ack.targets != captured.targets_rad || !restricted_state.gravity_empirical_active ||
        !freezes_restricted_hold_integral(ack, "hold", 0U, restricted_state.teach_exit_hold))
      throw std::runtime_error("ASSISTED_TEACH_LOW_SPEED_DISTURBANCE_DROPPED_HOLD");
    require_rejected([&] { auto state = restricted_state; validate_and_observe_assisted_teach(restart, event_motors, state); });
    GuiCommand position_attempt = ack; position_attempt.mode = "position";
    require_rejected([&] { auto state = restricted_state; validate_and_observe_assisted_teach(position_attempt, event_motors, state); });
    motor.unwrapped = motor.sign * kGear * (position + 1.0 * kPi / 180.0);
    motor.integral_encoder_velocity = 5.01 * kPi / 180.0;
    if (assisted_teach_runtime_blocker(ack, restricted_state, event_motors, 0.0, event_ns + 2U) != "ASSISTED_TEACH_RESTRICTED_VELOCITY_LIMIT")
      throw std::runtime_error("ASSISTED_TEACH_RESTRICTION_RELAXED_SPEED");
    motor.unwrapped = motor.sign * kGear * (position + 0.10 * kPi / 180.0);
    motor.integral_encoder_velocity = 0.10 * kPi / 180.0;
    const auto stable_start = captured.deadline_monotonic_ns + 100000000ULL;
    for (int frame = 0; frame <= 50; ++frame) {
      if (!assisted_teach_runtime_blocker(ack, restricted_state, event_motors, 0.0,
              stable_start + static_cast<std::uint64_t>(frame) * 10000000ULL).empty())
        throw std::runtime_error("ASSISTED_TEACH_RESTRICTED_STABLE_HOLD_FAILED");
      if (frame < 50 && (!restricted_state.teach_exit_hold.restricted || restricted_state.teach_exit_hold.completed))
        throw std::runtime_error("ASSISTED_TEACH_RESTRICTED_REARM_TOO_EARLY");
    }
    if (restricted_state.teach_exit_hold.restricted || !restricted_state.teach_exit_hold.completed ||
        restricted_state.teach_exit_hold.targets_rad != captured.targets_rad ||
        restricted_state.teach_exit_hold.deadline_monotonic_ns != captured.deadline_monotonic_ns)
      throw std::runtime_error("ASSISTED_TEACH_RESTRICTED_REARM_MUTATED_CAPTURE");
    motor.unwrapped = motor.sign * kGear * position;
    motor.integral_encoder_velocity = 0.0;
    if (!assisted_teach_runtime_blocker(ack, event_safety, event_motors, 0.0, captured.deadline_monotonic_ns).empty() ||
        !event_safety.teach_exit_hold.completed || uses_assisted_teach_damping(ack, "hold", 0U, captured, captured.deadline_monotonic_ns))
      throw std::runtime_error("ASSISTED_TEACH_STOP_GRACE_NOT_FINITE");
    // A delayed ACK is still valid once healthy, using the same frozen target.
    auto late_state = expired_state;
    late_state.teach_exit_hold.acknowledged = false;
    late_state.teach_active = true;
    ack.received_at = Clock::now();
    validate_and_observe_assisted_teach(ack, event_motors, late_state);
    if (!assisted_teach_runtime_blocker(ack, late_state, event_motors, 0.0, captured.deadline_monotonic_ns + 1U).empty() ||
        late_state.teach_exit_hold.deadline_monotonic_ns != captured.deadline_monotonic_ns)
      throw std::runtime_error("ASSISTED_TEACH_LATE_ACK_RESET_DEADLINE");
  }
  auto stale_authority = packet;
  stale_authority["gravity_authority"]["source_monotonic_ns"] = ns - kMaximumGravityAuthorityAgeNs - 1U;
  require_rejected([&] { GuiCommand parsed; parse_command(stale_authority.dump(), parsed, now); });
  hand_guidance_self_test(packet);
}

void command_mask_self_test() {
  assisted_teach_self_test();
  GuiCommand zero_hold_transition;
  zero_hold_transition.mode = "hold";
  GravityCommandAuthority zero_empirical_authority;
  zero_empirical_authority.authority_class = kEmpiricalAuthorityClass;
  if (gravity_authority_maximum_age_ns(
          zero_hold_transition, zero_empirical_authority) !=
      kEmpiricalZeroHoldTransitionMaximumAgeNs)
    throw std::runtime_error(
        "COMMAND_ZERO_HOLD_TRANSITION_AGE_SELF_TEST_FAILED");
  zero_hold_transition.moving_joint_mask[0] = true;
  if (gravity_authority_maximum_age_ns(
          zero_hold_transition, zero_empirical_authority) !=
      kMaximumGravityAuthorityAgeNs)
    throw std::runtime_error(
        "COMMAND_MOVING_AUTHORITY_AGE_SELF_TEST_FAILED");
  zero_hold_transition.moving_joint_mask[0] = false;
  zero_empirical_authority.gravity_scale_target = 0.25;
  if (gravity_authority_maximum_age_ns(
          zero_hold_transition, zero_empirical_authority) !=
      kMaximumGravityAuthorityAgeNs)
    throw std::runtime_error(
        "COMMAND_NONZERO_AUTHORITY_AGE_SELF_TEST_FAILED");

  if (enforce_brake_only_wire_mode(true, kBrakeMode) != kBrakeMode)
    throw std::runtime_error("BRAKE_ONLY_BRAKE_SELF_TEST_FAILED");
  bool non_brake_rejected = false;
  try {
    (void)enforce_brake_only_wire_mode(true, kFocMode);
  } catch (const std::runtime_error& error) {
    non_brake_rejected =
        std::string(error.what()) == "BRAKE_ONLY_NON_BRAKE_BLOCKED";
  }
  if (!non_brake_rejected)
    throw std::runtime_error("BRAKE_ONLY_FOC_REJECTION_SELF_TEST_FAILED");
  TxAudit brake_audit(true);
  MotorCmd brake_packet = make_command(0, kBrakeMode, 0.0, 0.0, 0.0, 0.0);
  if (audit_tx_before_serial_send(
          brake_packet, kBrakeMode, brake_audit) != kBrakeMode ||
      brake_audit.tx_attempt_total != 1U ||
      brake_audit.brake_tx_attempt_count != 1U ||
      brake_audit.serial_send_call_count != 0U)
    throw std::runtime_error("BRAKE_ONLY_TX_GUARD_BRAKE_SELF_TEST_FAILED");
  TxAudit foc_audit(true);
  MotorCmd foc_packet = make_command(0, kFocMode, 0.0, 0.0, 0.0, 0.0);
  bool foc_guard_rejected = false;
  try {
    (void)audit_tx_before_serial_send(foc_packet, kFocMode, foc_audit);
  } catch (const std::runtime_error& error) {
    foc_guard_rejected =
        std::string(error.what()) == "BRAKE_ONLY_NON_BRAKE_BLOCKED";
  }
  if (!foc_guard_rejected || foc_audit.tx_attempt_total != 1U ||
      foc_audit.foc_tx_attempt_count != 1U ||
      foc_audit.brake_only_guard_block_count != 1U ||
      foc_audit.serial_send_call_count != 0U)
    throw std::runtime_error("BRAKE_ONLY_TX_GUARD_FOC_SELF_TEST_FAILED");
  int deadline_misses = next_active_deadline_miss_count(0, true, true);
  if (deadline_misses != 1 ||
      deadline_misses >= kActiveDeadlineConsecutiveLimit)
    throw std::runtime_error("DEADLINE_SINGLE_MISS_SELF_TEST_FAILED");
  deadline_misses = next_active_deadline_miss_count(
      deadline_misses, true, false);
  if (deadline_misses != 0)
    throw std::runtime_error("DEADLINE_RECOVERY_SELF_TEST_FAILED");
  for (int frame = 0; frame < kActiveDeadlineConsecutiveLimit; ++frame)
    deadline_misses = next_active_deadline_miss_count(
        deadline_misses, true, true);
  if (deadline_misses != kActiveDeadlineConsecutiveLimit)
    throw std::runtime_error("DEADLINE_CONSECUTIVE_SELF_TEST_FAILED");
  for (std::size_t index = 0; index < 6U; ++index) {
    const int joint_index = static_cast<int>(index);
    if (!within_model_command_envelope(
            joint_index, kModelCommandLower[index]) ||
        !within_model_command_envelope(
            joint_index, kModelCommandUpper[index]) ||
        within_model_command_envelope(
            joint_index,
            kModelCommandLower[index] - 0.01 * kPi / 180.0) ||
        within_model_command_envelope(
            joint_index,
            kModelCommandUpper[index] + 0.01 * kPi / 180.0))
      throw std::runtime_error(
          "MODEL_COMMAND_FULL_ENDPOINTS_SELF_TEST_FAILED");
    if (!within_mechanical_feedback_envelope(
            joint_index,
            kModelCommandLower[index] - 0.49 * kPi / 180.0) ||
        !within_mechanical_feedback_envelope(
            joint_index,
            kModelCommandUpper[index] + 0.49 * kPi / 180.0) ||
        within_mechanical_feedback_envelope(
            joint_index,
            kModelCommandLower[index] - 0.51 * kPi / 180.0) ||
        within_mechanical_feedback_envelope(
            joint_index,
            kModelCommandUpper[index] + 0.51 * kPi / 180.0))
      throw std::runtime_error(
          "FEEDBACK_ENDPOINT_TOLERANCE_SELF_TEST_FAILED");
  }
  MotorRuntime hinted{"J3", 3, 2, +1, 0.60, 0.05};
  hinted.persistent_reference = -12.203996658325195;
  hinted.persistent_reference_configured = true;
  hinted.recovery_hint = 2.7094333607633976;
  hinted.recovery_hint_configured = true;
  const double current_raw = hinted.persistent_reference + 2.0 * (2.0 * kPi) +
      hinted.sign * kGear * hinted.recovery_hint;
  const double hinted_reference = reference_for_recovery_branch(current_raw, hinted);
  const double recovered = hinted.sign * (current_raw - hinted_reference) / kGear;
  if (std::abs(recovered - hinted.recovery_hint) > 0.02)
    throw std::runtime_error("RECOVERY_HINT_BRANCH_SELF_TEST_FAILED");
  MotorRuntime real_j2_a{"J2A", 0, 1, -1, 3.00, 0.30};
  MotorRuntime real_j2_b{"J2B", 1, 1, +1, 3.00, 0.30};
  real_j2_a.persistent_reference = -4.596924286444928;
  real_j2_b.persistent_reference = 12.4254104489523;
  real_j2_a.persistent_reference_configured = true;
  real_j2_b.persistent_reference_configured = true;
  real_j2_a.recovery_hint_configured = true;
  real_j2_b.recovery_hint_configured = true;
  const double real_raw_a = 5.04623;
  const double real_raw_b = 2.76539;
  const double legacy_reference_a = real_j2_a.persistent_reference +
      std::round((real_raw_a - real_j2_a.persistent_reference) /
                 (2.0 * kPi)) * (2.0 * kPi);
  const double legacy_reference_b = real_j2_b.persistent_reference +
      std::round((real_raw_b - real_j2_b.persistent_reference) /
                 (2.0 * kPi)) * (2.0 * kPi);
  const double legacy_q_a =
      -1.0 * (real_raw_a - legacy_reference_a) / kGear;
  const double legacy_q_b =
      +1.0 * (real_raw_b - legacy_reference_b) / kGear;
  if (std::abs(0.5 * (legacy_q_a + legacy_q_b) -
               26.38306165 * kPi / 180.0) > 1e-8)
    throw std::runtime_error("J2_REAL_FALSE_BRANCH_REPRODUCTION_FAILED");
  bool stale_persistent_rejected = false;
  try {
    (void)reference_for_recovery_branch(real_raw_a, real_j2_a);
  } catch (const std::runtime_error&) {
    stale_persistent_rejected = true;
  }
  if (!stale_persistent_rejected)
    throw std::runtime_error("J2_STALE_PERSISTENT_PHASE_NOT_REJECTED");
  const std::string session_parent_sha(64U, 'a');
  const std::string session_evidence_sha(64U, 'b');
  const std::string session_hint_sha(64U, 'f');
  const std::string session_boot_id =
      "01234567-89ab-cdef-0123-456789abcdef";
  const nlohmann::json session_document = {
      {"schema", "go-m8010-j2-power-session-reference/1.0"},
      {"reference_name", "PERSISTENT_SOFTWARE_ZERO_V1"},
      {"parent_persistent_zero_sha256", session_parent_sha},
      {"source_evidence", {
          {"path", "/evidence/raw.json"},
          {"sha256", session_evidence_sha}}},
      {"preserved_inputs", {
          {"recovery_branch_hints_sha256", session_hint_sha},
          {"overwritten_or_deleted", false}}},
      {"operator_confirmation", {
          {"vertical_initialization_pose", true},
          {"support_reliable", true},
          {"arm_not_moved", true},
          {"not_at_mechanical_limit", true},
          {"power_session_id", "self-test-power-session"}}},
      {"writes", {
          {"motor_internal_zero_modified", false},
          {"rid_written", false},
          {"flash_or_eeprom_written", false}}},
      {"control_authority", {
          {"is_software_zero", false},
          {"authorizes_active_control", false},
          {"authorizes_motor_internal_write", false}}},
      {"motors", {
          {"J2A", {
              {"session_reference_raw_rad", real_raw_a},
              {"logical_position_rad", 0.0},
              {"sample_count", 500},
              {"sample_span_raw_rad", 0.001},
              {"sign", -1}, {"gear_ratio", kGear}}},
          {"J2B", {
              {"session_reference_raw_rad", real_raw_b},
              {"logical_position_rad", 0.0},
              {"sample_count", 500},
              {"sample_span_raw_rad", 0.001},
              {"sign", +1}, {"gear_ratio", kGear}}}}},
      {"raw_capture", {
          {"source_schema",
           "go-m8010-j2-brake-raw-capture-statistics/1.0"},
          {"source_file_sha256", session_evidence_sha},
          {"packet_count", 500},
           {"source_coverage_s", 5.0},
           {"power_session_id", "self-test-power-session"},
           {"host_boot_id", session_boot_id},
           {"recorded_boottime_ns", 123456789ULL},
          {"motors", {
              {"J2A", {
                  {"sample_count", 500},
                  {"unwrapped_raw_position_rad", {
                      {"mean", real_raw_a}, {"span", 0.001}}}}},
              {"J2B", {
                  {"sample_count", 500},
                  {"unwrapped_raw_position_rad", {
                      {"mean", real_raw_b}, {"span", 0.001}}}}}}}}}};
  std::vector<MotorRuntime> session_motors{real_j2_a, real_j2_b};
  apply_j2_session_reference_document(
      session_document, session_parent_sha, session_hint_sha,
      "self-test-power-session", session_boot_id, session_motors);
  if (!session_motors[0].session_reference_configured ||
      !session_motors[1].session_reference_configured)
    throw std::runtime_error("J2_SESSION_DOCUMENT_SELF_TEST_FAILED");
  const double session_reference_a =
      reference_for_j2_session(real_raw_a, session_motors[0]);
  const double session_reference_b =
      reference_for_j2_session(real_raw_b, session_motors[1]);
  const double session_q_a =
      -1.0 * (real_raw_a - session_reference_a) / kGear;
  const double session_q_b =
      +1.0 * (real_raw_b - session_reference_b) / kGear;
  if (std::abs(0.5 * (session_q_a + session_q_b)) > 1e-12 ||
      std::abs(session_q_a - session_q_b) > 1e-12)
    throw std::runtime_error("J2_SESSION_REFERENCE_REAL_SELF_TEST_FAILED");
  bool far_session_pose_rejected = false;
  try {
    (void)reference_for_j2_session(
        real_raw_a - kGear * 3.0 * kPi / 180.0, session_motors[0]);
  } catch (const std::runtime_error&) {
    far_session_pose_rejected = true;
  }
  if (!far_session_pose_rejected)
    throw std::runtime_error("J2_SESSION_STARTUP_GATE_SELF_TEST_FAILED");
  Options permit_options;
  permit_options.expected_zero_sha256 = session_parent_sha;
  permit_options.expected_j2_session_reference_sha256 =
      std::string(64U, 'c');
  permit_options.expected_worker_sha256 = std::string(64U, 'd');
  permit_options.expected_j2_power_session_id = "self-test-power-session";
  const std::string permit_id(64U, 'e');
  const std::string permit_filename = permit_id + ".json";
  const std::string pending_path =
      expected_permit_directory("pending") + "/" + permit_filename;
  const std::string inflight_path =
      expected_permit_directory("inflight") + "/" + permit_filename;
  const std::string spent_path =
      expected_permit_directory("spent") + "/" + permit_filename;
  const std::uint64_t permit_issued_ns = 2000000000ULL;
  const nlohmann::json permit_document = {
      {"schema", "go-m8010-j2-power-session-launch-permit/1.0"},
      {"permit_version", 1},
      {"permit_id", permit_id},
      {"scope", "j2"},
      {"single_use", true},
      {"power_session_id", "self-test-power-session"},
      {"host_boot_id", session_boot_id},
      {"issued_boottime_ns", permit_issued_ns},
      {"expires_boottime_ns", permit_issued_ns + 30000000000ULL},
      {"parent_persistent_zero_sha256", session_parent_sha},
      {"session_reference", {
          {"path", "/anchor/self-test.json"},
          {"sha256", permit_options.expected_j2_session_reference_sha256}}},
      {"worker", {
          {"path", "/worker/self-test"},
          {"sha256", permit_options.expected_worker_sha256}}},
      {"source_capture", {
          {"host_boot_id", session_boot_id},
          {"recorded_boottime_ns", 1000000000ULL}}},
      {"pending_path", pending_path},
      {"inflight_path", inflight_path},
      {"spent_path", spent_path},
      {"serial", {
          {"stable_by_id",
           "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0"},
          {"bus", "j2"},
          {"motor_ids", {0, 1}},
          {"gear_ratio", kGear},
          {"signs", {{"J2A", -1}, {"J2B", 1}}}}},
      {"startup_recheck", {
          {"minimum_brake_frames", 50},
          {"max_raw_phase_delta_rad", kJ2StartupPermitRawDelta},
          {"max_raw_span_rad", kJ2StartupPermitRawSpan}}}};
  const J2LaunchPermit parsed_permit = apply_j2_launch_permit_document(
      permit_document, permit_options, "/anchor/self-test.json",
      pending_path, "/worker/self-test", session_parent_sha,
      permit_options.expected_j2_session_reference_sha256,
      permit_options.expected_worker_sha256, session_boot_id,
      permit_issued_ns + 1U);
  if (parsed_permit.state != "PENDING" ||
      parsed_permit.minimum_brake_frames != 50U)
    throw std::runtime_error("J2_LAUNCH_PERMIT_SELF_TEST_FAILED");
  require_j2_launch_permit_fresh(parsed_permit, permit_issued_ns + 1U);
  bool transition_expiry_rejected = false;
  try {
    require_j2_launch_permit_fresh(
        parsed_permit, permit_issued_ns + 30000000000ULL);
  } catch (const std::runtime_error&) {
    transition_expiry_rejected = true;
  }
  if (!transition_expiry_rejected)
    throw std::runtime_error("J2_TRANSITION_EXPIRY_SELF_TEST_FAILED");
  bool expired_permit_rejected = false;
  try {
    (void)apply_j2_launch_permit_document(
        permit_document, permit_options, "/anchor/self-test.json",
        pending_path, "/worker/self-test", session_parent_sha,
        permit_options.expected_j2_session_reference_sha256,
        permit_options.expected_worker_sha256, session_boot_id,
        permit_issued_ns + 30000000000ULL);
  } catch (const std::runtime_error&) {
    expired_permit_rejected = true;
  }
  if (!expired_permit_rejected)
    throw std::runtime_error("J2_EXPIRED_LAUNCH_PERMIT_SELF_TEST_FAILED");
  GuiCommand command;
  const std::string command_payload = fresh_source_self_test_payload(
      R"({"schema":"go-m8010-gui-command/1.2","sequence":1,
      "mode":"position","targets_rad":[0,0,0,0,0,0],
      "active_joint_mask":[false,false,false,true,false,false],
      "activation_epoch":7,"maximum_velocity_rad_s":0.08,
      "maximum_acceleration_rad_s2":0.3,
      "kp":[0.5,1.0,0.6,0.5,0.5,0.0],
       "kd":[0.05,0.10,0.05,0.05,0.05,0.0]})");
  parse_command(command_payload, command);
  const std::uint64_t minimum = minimum_epoch_after_lease(
      0, false, command.active_joint_mask[3], command.activation_epoch);
  if (!command.active_joint_mask[3] || command.active_joint_mask[2] ||
      command.active_joint_mask[4] || !command.moving_joint_mask[3] ||
      command.activation_epoch != 7U ||
      minimum != 8U || command.activation_epoch >= minimum)
    throw std::runtime_error("COMMAND_MASK_SELF_TEST_FAILED");

  const auto parse_is_rejected = [](const nlohmann::json& value) {
    try {
      GuiCommand parsed;
      parse_command(value.dump(), parsed);
    } catch (const std::exception&) {
      return true;
    }
    return false;
  };
  auto source_test = nlohmann::json::parse(command_payload);
  source_test["schema"] = "go-m8010-gui-command/1.1";
  if (!parse_is_rejected(source_test))
    throw std::runtime_error("COMMAND_LEGACY_ACTIVE_SELF_TEST_FAILED");
  source_test = nlohmann::json::parse(command_payload);
  source_test["mode"] = "drag";
  source_test["active_joint_mask"] =
      nlohmann::json::array({true, true, true, true, true, true});
  source_test["moving_joint_mask"] =
      nlohmann::json::array({false, false, false, false, false, false});
  source_test["activation_epoch"] = 0U;
  source_test["source_monotonic_ns"] = monotonic_ns_at(Clock::now());
  source_test["sequence"] = 2U;
  GuiCommand drag_command;
  parse_command(source_test.dump(), drag_command);
  if (drag_command.mode != "drag" || drag_command.activation_epoch != 0U ||
      !std::all_of(
          drag_command.active_joint_mask.begin(),
          drag_command.active_joint_mask.end(), [](bool active) {
            return active;
          }))
    throw std::runtime_error("COMMAND_DRAG_ZERO_EPOCH_SELF_TEST_FAILED");
  source_test = nlohmann::json::parse(command_payload);
  source_test["source_monotonic_ns"] = 1U;
  if (!parse_is_rejected(source_test))
    throw std::runtime_error("COMMAND_STALE_SOURCE_SELF_TEST_FAILED");
  source_test = nlohmann::json::parse(command_payload);
  source_test["source_monotonic_ns"] =
      monotonic_ns_at(Clock::now()) + 1000000000ULL;
  if (!parse_is_rejected(source_test))
    throw std::runtime_error("COMMAND_FUTURE_SOURCE_SELF_TEST_FAILED");
  source_test = nlohmann::json::parse(command_payload);
  source_test["source_monotonic_ns"] = monotonic_ns_at(Clock::now());
  source_test["source_instance_id"] =
      "0123456789ABCDEF0123456789ABCDEF";
  if (!parse_is_rejected(source_test))
    throw std::runtime_error("COMMAND_SOURCE_ID_SELF_TEST_FAILED");
  source_test = nlohmann::json::parse(command_payload);
  source_test["source_monotonic_ns"] = monotonic_ns_at(Clock::now());
  source_test["sequence"] = 0U;
  if (!parse_is_rejected(source_test))
    throw std::runtime_error("COMMAND_SOURCE_SEQUENCE_SELF_TEST_FAILED");
  for (const char* legacy_schema : {
           "go-m8010-gui-command/1.0",
           "go-m8010-gui-command/1.1",
           "go-m8010-gui-command/1.2",
           "go-m8010-gui-command/1.3"}) {
    auto emergency_brake = nlohmann::json::parse(command_payload);
    emergency_brake["schema"] = legacy_schema;
    emergency_brake["mode"] = "brake";
    emergency_brake["active_joint_mask"] =
        nlohmann::json::array({false, false, false, false, false, false});
    emergency_brake["moving_joint_mask"] =
        nlohmann::json::array({false, false, false, false, false, false});
    emergency_brake["source_instance_id"] = 7;
    emergency_brake["source_monotonic_ns"] = "malformed";
    emergency_brake["sequence"] = -1;
    GuiCommand parsed_brake = command;
    parse_command(emergency_brake.dump(), parsed_brake);
    if (parsed_brake.mode != "brake" ||
        !parsed_brake.source_instance_id.empty() ||
        parsed_brake.source_monotonic_ns != 0U ||
        parsed_brake.source_sequence != 0U || parsed_brake.quintic.present)
      throw std::runtime_error(
          "COMMAND_LEGACY_BRAKE_SOURCE_COMPAT_SELF_TEST_FAILED");
  }

  CommandReceiveState replay_state;
  validate_command_source_replay(command, replay_state);
  observe_command_source(command, replay_state);
  bool sequence_replay_rejected = false;
  try {
    validate_command_source_replay(command, replay_state);
  } catch (const std::runtime_error&) {
    sequence_replay_rejected = true;
  }
  GuiCommand timestamp_replay = command;
  ++timestamp_replay.source_sequence;
  bool timestamp_replay_rejected = false;
  try {
    validate_command_source_replay(timestamp_replay, replay_state);
  } catch (const std::runtime_error&) {
    timestamp_replay_rejected = true;
  }
  if (!sequence_replay_rejected || !timestamp_replay_rejected)
    throw std::runtime_error("COMMAND_SOURCE_REPLAY_SELF_TEST_FAILED");
  ++timestamp_replay.source_monotonic_ns;
  validate_command_source_replay(timestamp_replay, replay_state);
  observe_command_source(timestamp_replay, replay_state);
  constexpr char kHexDigits[] = "0123456789abcdef";
  GuiCommand competing_source = command;
  competing_source.source_instance_id.assign(32U, '0');
  competing_source.received_at = replay_state.active_source_received_at +
      std::chrono::milliseconds(500);
  bool live_source_lock_rejected = false;
  try {
    validate_command_source_replay(competing_source, replay_state);
  } catch (const std::runtime_error&) {
    live_source_lock_rejected = true;
  }
  if (!live_source_lock_rejected)
    throw std::runtime_error("COMMAND_LIVE_SOURCE_LOCK_SELF_TEST_FAILED");
  competing_source.received_at += std::chrono::nanoseconds(1);
  validate_command_source_replay(competing_source, replay_state);
  observe_command_source(competing_source, replay_state);
  for (std::size_t index = 1;
       index <= kMaximumTrackedCommandSources; ++index) {
    GuiCommand another_source = command;
    another_source.source_instance_id.assign(32U, '0');
    another_source.source_instance_id[30] =
        kHexDigits[(index >> 4U) & 0x0fU];
    another_source.source_instance_id[31] = kHexDigits[index & 0x0fU];
    another_source.received_at = replay_state.active_source_received_at +
        std::chrono::milliseconds(501);
    validate_command_source_replay(another_source, replay_state);
    observe_command_source(another_source, replay_state);
  }
  if (replay_state.accepted_sources.size() !=
      kMaximumTrackedCommandSources)
    throw std::runtime_error("COMMAND_SOURCE_BOUND_SELF_TEST_FAILED");
  auto j345_motors = make_motors("j345");

  const Clock::time_point quintic_received_at = Clock::now();
  const std::uint64_t quintic_received_ns =
      monotonic_ns_at(quintic_received_at);
  nlohmann::json quintic_document = {
      {"schema", "go-m8010-gui-command/1.3"},
      {"sequence", 11U},
      {"mode", "position"},
      {"targets_rad", {0.0, 0.0, 0.0, 0.01, 0.0, 0.0}},
      {"active_joint_mask", {false, false, false, true, false, false}},
      {"moving_joint_mask", {false, false, false, true, false, false}},
      {"activation_epoch", 9U},
      {"maximum_velocity_rad_s", 0.08},
      {"maximum_acceleration_rad_s2", 0.30},
      {"kp", {0.5, 1.0, 0.6, 0.5, 0.5, 0.0}},
      {"kd", {0.05, 0.10, 0.05, 0.05, 0.05, 0.0}},
      {"feedforward_nm", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {"source_instance_id", "0123456789abcdef0123456789abcdef"},
      {"source_monotonic_ns", quintic_received_ns},
      {"gravity_authority", {
          {"schema", kOfficialGravityAuthoritySchema},
          {"source_instance_id", "fedcba9876543210fedcba9876543210"},
          {"sequence", 1U},
          {"source_monotonic_ns", quintic_received_ns},
          {"model_sha256", kProductionModelSha256},
          {"gravity_config_sha256", kGravityConfigSha256},
          {"session_id", "self-test-session"},
          {"state_instance_id", "self-test-state"},
          {"gravity_scale", 0.0},
          {"gravity_scale_target", 0.0},
          {"feedforward_nm", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}}},
      {"plan_token_id", std::string(64U, 'a')},
      {"trajectory", {
          {"schema", "go-m8010-quintic-command/1.0"},
          {"trajectory_sha256", std::string(64U, 'b')},
          {"profile", "quintic-rest-to-rest-v1"},
          {"start_rad", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
          {"target_rad", {0.0, 0.0, 0.0, 0.01, 0.0, 0.0}},
          {"duration_ns", 2000000000ULL},
          {"interval_count", 200U},
          {"execute_at_monotonic_ns", quintic_received_ns + 1000000000ULL},
          {"segment_index", 0},
          {"segment_count", 1}}}};
  GuiCommand quintic_command;
  parse_command(
      quintic_document.dump(), quintic_command, quintic_received_at);
  if (quintic_command.schema != "go-m8010-gui-command/1.3" ||
      !quintic_command.quintic.present || command.quintic.present ||
      quintic_command.quintic.plan_token_id != std::string(64U, 'a') ||
      quintic_command.quintic.trajectory_sha256 != std::string(64U, 'b'))
    throw std::runtime_error("COMMAND_QUINTIC_PARSE_SELF_TEST_FAILED");
  GuiCommand invalid_quintic_start = quintic_command;
  invalid_quintic_start.quintic.start_rad[3] =
      kModelCommandLower[3] - 0.01;
  bool invalid_quintic_start_rejected = false;
  try {
    validate_command_for_owned_domain(
        invalid_quintic_start, j345_motors);
  } catch (const std::runtime_error&) {
    invalid_quintic_start_rejected = true;
  }
  if (!invalid_quintic_start_rejected)
    throw std::runtime_error(
        "COMMAND_QUINTIC_START_ENVELOPE_SELF_TEST_FAILED");

  const auto quintic_parse_is_rejected = [&](nlohmann::json candidate) {
    try {
      GuiCommand parsed;
      parse_command(candidate.dump(), parsed, quintic_received_at);
    } catch (const std::exception&) {
      return true;
    }
    return false;
  };
  auto malformed_quintic = quintic_document;
  malformed_quintic["trajectory"]["unexpected"] = true;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error("COMMAND_QUINTIC_STRICT_FIELDS_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic.erase("moving_joint_mask");
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error(
        "COMMAND_QUINTIC_MOVING_MASK_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic["trajectory"]["target_rad"][3] = 0.02;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error("COMMAND_QUINTIC_TARGET_MATCH_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic["active_joint_mask"][2] = true;
  malformed_quintic["moving_joint_mask"][2] = true;
  malformed_quintic["targets_rad"][2] = 0.005;
  malformed_quintic["trajectory"]["target_rad"][2] = 0.005;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error("COMMAND_QUINTIC_ONE_MOVING_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic["trajectory"]["start_rad"][2] = 0.001;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error("COMMAND_QUINTIC_FIXED_AXIS_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic["trajectory"]["interval_count"] = 1000001U;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error("COMMAND_QUINTIC_INTERVAL_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic["trajectory"]["duration_ns"] = 2000000001ULL;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error(
        "COMMAND_QUINTIC_INTEGER_GRID_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic["trajectory"]["interval_count"] = 100U;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error(
        "COMMAND_QUINTIC_TEN_MS_GRID_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic["targets_rad"][3] = 0.0000001;
  malformed_quintic["trajectory"]["target_rad"][3] = 0.0000001;
  malformed_quintic["trajectory"]["duration_ns"] = 10000000ULL;
  malformed_quintic["trajectory"]["interval_count"] = 1U;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error(
        "COMMAND_QUINTIC_ENDPOINT_STEP_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic["targets_rad"][3] = 0.0;
  malformed_quintic["trajectory"]["target_rad"][3] = 0.0;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error(
        "COMMAND_QUINTIC_ZERO_DISPLACEMENT_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic["trajectory"]["duration_ns"] = 1000000U;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error("COMMAND_QUINTIC_PEAK_LIMIT_SELF_TEST_FAILED");
  malformed_quintic = quintic_document;
  malformed_quintic["trajectory"]["segment_count"] = 0;
  if (!quintic_parse_is_rejected(malformed_quintic))
    throw std::runtime_error("COMMAND_QUINTIC_SEGMENT_SELF_TEST_FAILED");

  auto quintic_feedback_motors = j345_motors;
  for (auto& motor : quintic_feedback_motors) {
    motor.reference = 0.0;
    motor.unwrapped = 0.0;
    motor.reference_ready = true;
    motor.valid = true;
    motor.last_frame_valid = true;
    motor.fault_latched = false;
    motor.merror = 0;
    motor.temperature = 25;
    motor.returned_mode = kFocMode;
    motor.previous_feedback_at = Clock::now();
  }
  auto mismatched_quintic_feedback = quintic_feedback_motors;
  for (auto& motor : mismatched_quintic_feedback) {
    if (motor.joint_index == 3)
      motor.unwrapped = motor.sign * kGear *
          (kFixedHoldCaptureWindow + 0.001);
  }
  bool mismatched_quintic_start_rejected = false;
  try {
    CommandSafetyState mismatch_safety;
    validate_and_observe_position_authority(
        quintic_command, mismatched_quintic_feedback, mismatch_safety);
  } catch (const std::runtime_error& error) {
    mismatched_quintic_start_rejected =
        std::string(error.what()) ==
            "COMMAND_QUINTIC_START_FEEDBACK_MISMATCH";
  }
  if (!mismatched_quintic_start_rejected)
    throw std::runtime_error(
        "COMMAND_QUINTIC_START_MISMATCH_SELF_TEST_FAILED");
  auto unhealthy_quintic_feedback = quintic_feedback_motors;
  for (auto& motor : unhealthy_quintic_feedback) {
    if (motor.joint_index == 3)
      motor.previous_feedback_at = Clock::time_point{};
  }
  bool unhealthy_quintic_start_rejected = false;
  try {
    CommandSafetyState unhealthy_safety;
    validate_and_observe_position_authority(
        quintic_command, unhealthy_quintic_feedback, unhealthy_safety);
  } catch (const std::runtime_error& error) {
    unhealthy_quintic_start_rejected =
        std::string(error.what()) ==
            "COMMAND_QUINTIC_START_FEEDBACK_UNHEALTHY";
  }
  if (!unhealthy_quintic_start_rejected)
    throw std::runtime_error(
        "COMMAND_QUINTIC_START_HEALTH_SELF_TEST_FAILED");

  CommandSafetyState quintic_safety;
  validate_and_observe_position_authority(
      quintic_command, quintic_feedback_motors, quintic_safety);
  GuiCommand late_heartbeat = quintic_command;
  late_heartbeat.received_at = quintic_received_at +
      std::chrono::milliseconds(1500);
  validate_and_observe_position_authority(
      late_heartbeat, quintic_feedback_motors, quintic_safety);
  GuiCommand changed_descriptor = late_heartbeat;
  ++changed_descriptor.quintic.duration_ns;
  bool changed_descriptor_rejected = false;
  try {
    validate_and_observe_position_authority(
        changed_descriptor, quintic_feedback_motors, quintic_safety);
  } catch (const std::runtime_error&) {
    changed_descriptor_rejected = true;
  }
  if (!changed_descriptor_rejected)
    throw std::runtime_error(
        "COMMAND_QUINTIC_IMMUTABLE_SELF_TEST_FAILED");
  GuiCommand late_first_packet = quintic_command;
  late_first_packet.received_at = quintic_received_at +
      std::chrono::seconds(1);
  bool late_first_packet_rejected = false;
  try {
    CommandSafetyState empty_safety;
    validate_and_observe_position_authority(
        late_first_packet, quintic_feedback_motors, empty_safety);
  } catch (const std::runtime_error&) {
    late_first_packet_rejected = true;
  }
  if (!late_first_packet_rejected)
    throw std::runtime_error(
        "COMMAND_QUINTIC_LATE_FIRST_PACKET_SELF_TEST_FAILED");
  GuiCommand quintic_higher_epoch = quintic_command;
  quintic_higher_epoch.activation_epoch = 10U;
  quintic_higher_epoch.received_at = quintic_received_at +
      std::chrono::seconds(2);
  quintic_higher_epoch.quintic.execute_at_monotonic_ns =
      quintic_received_ns + 3000000000ULL;
  validate_and_observe_position_authority(
      quintic_higher_epoch, quintic_feedback_motors, quintic_safety);

  QuinticTrajectoryDescriptor sample_trajectory = quintic_command.quintic;
  sample_trajectory.execute_at_monotonic_ns = 1000000000ULL;
  sample_trajectory.duration_ns = 1000000000ULL;
  sample_trajectory.interval_count = 10U;
  GuiCommand sample_command = quintic_command;
  sample_command.quintic = sample_trajectory;
  std::array<double, 6> sample_q{};
  std::array<double, 6> sample_dq{};
  const QuinticSampleClock scheduled_sample =
      quintic_sample_clock(sample_trajectory, 999999999ULL);
  apply_quintic_reference(
      3, sample_command, scheduled_sample, sample_q, sample_dq);
  if (scheduled_sample.sample_index != 0U ||
      std::string(scheduled_sample.state) != "PREPARED" ||
      sample_q[3] != sample_trajectory.start_rad[3] || sample_dq[3] != 0.0)
    throw std::runtime_error(
        "COMMAND_QUINTIC_PREPARED_SAMPLE_SELF_TEST_FAILED");
  const QuinticSampleClock middle_sample =
      quintic_sample_clock(sample_trajectory, 1550000000ULL);
  apply_quintic_reference(
      3, sample_command, middle_sample, sample_q, sample_dq);
  if (middle_sample.sample_index != 5U ||
      std::string(middle_sample.state) != "RUNNING" ||
      std::abs(sample_q[3] - 0.005) > 1e-12 || sample_dq[3] <= 0.0)
    throw std::runtime_error(
        "COMMAND_QUINTIC_INTEGER_SAMPLE_SELF_TEST_FAILED");
  const QuinticSampleClock endpoint_sample =
      quintic_sample_clock(sample_trajectory, 2000000000ULL);
  apply_quintic_reference(
      3, sample_command, endpoint_sample, sample_q, sample_dq);
  if (endpoint_sample.sample_index != sample_trajectory.interval_count ||
      std::string(endpoint_sample.state) != "COMPLETE" ||
      sample_q[3] != sample_trajectory.target_rad[3] || sample_dq[3] != 0.0)
    throw std::runtime_error(
        "COMMAND_QUINTIC_EXACT_ENDPOINT_SELF_TEST_FAILED");

  validate_command_for_owned_domain(command, j345_motors);
  GuiCommand mixed_position;
  parse_command(fresh_source_self_test_payload(
      R"({"schema":"go-m8010-gui-command/1.2","sequence":3,
      "mode":"position","targets_rad":[0,0,0.05,2.0,0,0],
      "active_joint_mask":[false,false,true,true,false,false],
      "moving_joint_mask":[false,false,true,false,false,false],
      "activation_epoch":8,"maximum_velocity_rad_s":0.08,
      "maximum_acceleration_rad_s2":0.3,
      "kp":[0.5,1.0,0.6,0.5,0.5,0.0],
      "kd":[0.05,0.10,0.05,0.05,0.05,0.0]})"), mixed_position);
  validate_command_for_owned_domain(mixed_position, j345_motors);
  if (!mixed_position.moving_joint_mask[2] ||
      mixed_position.moving_joint_mask[3] ||
      !joint_uses_fixed_hold_target(mixed_position, "position", 3U) ||
      should_apply_velocity_guard(mixed_position, "position", 3U))
    throw std::runtime_error("COMMAND_MOVING_MASK_SELF_TEST_FAILED");
  bool moving_subset_rejected = false;
  try {
    GuiCommand invalid_subset;
    parse_command(fresh_source_self_test_payload(
        R"({"schema":"go-m8010-gui-command/1.2","sequence":4,
        "mode":"position","targets_rad":[0,0,0,0,0,0],
        "active_joint_mask":[false,false,false,false,false,false],
        "moving_joint_mask":[false,false,true,false,false,false],
        "activation_epoch":8,"maximum_velocity_rad_s":0.08,
        "maximum_acceleration_rad_s2":0.3,
        "kp":[0.5,1.0,0.6,0.5,0.5,0.0],
        "kd":[0.05,0.10,0.05,0.05,0.05,0.0]})"), invalid_subset);
  } catch (const std::runtime_error&) {
    moving_subset_rejected = true;
  }
  if (!moving_subset_rejected)
    throw std::runtime_error("COMMAND_MOVING_SUBSET_SELF_TEST_FAILED");
  GuiCommand unrelated_target = mixed_position;
  unrelated_target.targets[2] = 100.0;
  validate_command_for_owned_domain(unrelated_target, make_motors("j1"));
  GuiCommand full_range_moving_target = mixed_position;
  full_range_moving_target.targets[2] = kModelCommandUpper[2];
  validate_command_for_owned_domain(full_range_moving_target, j345_motors);
  GuiCommand out_of_model_target = full_range_moving_target;
  out_of_model_target.targets[2] =
      kModelCommandUpper[2] + 0.01 * kPi / 180.0;
  bool out_of_model_rejected = false;
  try {
    validate_command_for_owned_domain(
        out_of_model_target, j345_motors);
  } catch (const std::runtime_error&) {
    out_of_model_rejected = true;
  }
  // A defensive target rejection must not turn the already accepted POSITION
  // command into BRAKE or replace its endpoint.
  if (!out_of_model_rejected ||
      full_range_moving_target.mode != "position" ||
      command_releases_owned_domain(full_range_moving_target, j345_motors) ||
      std::abs(full_range_moving_target.targets[2] -
               kModelCommandUpper[2]) > 1e-12)
    throw std::runtime_error(
        "REJECTED_TARGET_PRESERVES_POSITION_AUTHORITY_SELF_TEST_FAILED");
  auto mixed_motors = j345_motors;
  for (auto& motor : mixed_motors) {
    const double logical = motor.joint_index == 2 ? 0.10 : 0.20;
    motor.reference = 0.0;
    motor.unwrapped = motor.sign * kGear * logical;
    motor.reference_ready = true;
    motor.valid = true;
    motor.last_frame_valid = true;
    motor.fault_latched = false;
    motor.merror = 0;
    motor.temperature = 25;
    motor.returned_mode = kFocMode;
    motor.previous_feedback_at = Clock::now();
  }
  MotorRuntime j2_velocity_guard{"J2A", 0, 1, -1, 3.00, 0.30};
  MotorRuntime j3_velocity_guard{"J3", 3, 2, +1, 2.00, 0.15};
  MotorRuntime j1_velocity_guard{"J1", 0, 0, +1, 1.50, 0.15};
  if (moving_velocity_guard_tripped("j2", j2_velocity_guard, 20.0) ||
      !moving_velocity_guard_tripped("j2", j2_velocity_guard, 20.01) ||
      moving_velocity_guard_tripped("j345", j3_velocity_guard, 25.0) ||
      !moving_velocity_guard_tripped("j345", j3_velocity_guard, 25.01) ||
      moving_velocity_guard_tripped("j1", j1_velocity_guard, 36.0) ||
      !moving_velocity_guard_tripped("j1", j1_velocity_guard, 36.0))
    throw std::runtime_error("MOVING_VELOCITY_GUARD_SELF_TEST_FAILED");

  for (std::size_t joint = 0; joint < 5U; ++joint) {
    const double required = frozen_required_rotor_gravity_nm(joint);
    const double capacity = joint == 1U
        ? kJ2PredictedRotorWorkNm : kHoldIntegralRotorHardNm[joint];
    if (!std::isfinite(required) ||
        capacity < 1.50 * required - 1e-12 ||
        kHoldIntegralRotorHardNm[joint] < 1.25 * required - 1e-12 ||
        kHoldIntegralRotorHardNm[joint] >=
            kGoManualMaximumRotorEquivalentNm ||
        capacity >= kGoManualMaximumRotorEquivalentNm)
      throw std::runtime_error("HOLD_TORQUE_CAPACITY_SELF_TEST_FAILED");
    const double bounded_aux_total = kHoldIntegralRotorHardNm[joint] +
        kKpLimits[joint] * kGear * kAuxIntegralEnterError +
        kKdLimits[joint] * kGear * kAuxIntegralEnterVelocity;
    if (joint != 1U &&
        bounded_aux_total >= 0.95 * kGoManualMaximumRotorEquivalentNm)
      throw std::runtime_error("HOLD_TOTAL_TORQUE_BUDGET_SELF_TEST_FAILED");
  }
  if (kJ2MovingKpLimit != kKpLimits[1] ||
      kJ2MovingKdLimit != kKdLimits[1] ||
      kJ2PredictedRotorWorkNm <= kJ2IntegralRotorHardNm ||
      kJ2RecoveryPredictedRotorWorkNm >=
          kGoManualMaximumRotorEquivalentNm)
    throw std::runtime_error("J2_LOADED_MOTION_ENVELOPE_SELF_TEST_FAILED");

  MotorRuntime aux_governor_motor{"J3", 3, 2, +1, 2.00, 0.15};
  aux_governor_motor.reference = 0.0;
  aux_governor_motor.unwrapped = 0.0;
  aux_governor_motor.last_dq = 0.0;
  const double aux_authorized_target = 20.0 * kPi / 180.0;
  const SingleMotorGovernedReference aux_governed =
      govern_single_motor_reference(
          aux_governor_motor, 0.0, 0.0,
          aux_authorized_target, 0.0, 2.0, 0.15, 1.60,
          kAuxPredictedRotorWorkNm[2],
          kAuxPredictedRotorPdHardNm[2]);
  if (!aux_governed.feasible || aux_governed.alpha <= 0.0 ||
      aux_governed.alpha >= 1.0 || aux_governed.q <= 0.0 ||
      aux_governed.q >= aux_authorized_target ||
      std::abs(aux_governed.predicted_work_nm) >
          kAuxPredictedRotorWorkNm[2] + 1e-12 ||
      std::abs(aux_authorized_target - 20.0 * kPi / 180.0) > 1e-12)
    throw std::runtime_error("AUX_WIRE_GOVERNOR_SELF_TEST_FAILED");

  // Replay a stationary encoder with the observed noisy vendor dq. Learning
  // must use encoder motion, while real motion and observation gaps block it.
  MotorRuntime encoder_integral_motor{"J1", 0, 0, +1, 1.50, 0.15};
  encoder_integral_motor.last_dq = -0.085302 * kGear;
  encoder_integral_motor.speed_ready = true;
  encoder_integral_motor.last_frame_valid = true;
  encoder_integral_motor.previous_feedback_at = Clock::now();
  const auto encoder_step = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(kPeriod));
  auto learn_encoder = [&](BoundedHoldIntegralState& state, double velocity) {
    return update_bounded_hold_integral(
        state, true, true, -0.279 * kPi / 180.0, velocity,
        kHoldIntegralRotorHardNm[0], kAuxIntegralKiPerRotorRadS,
        kAuxIntegralRateHardNmS, kAuxIntegralEnterError,
        kAuxIntegralEnterVelocity, kAuxIntegralDeadband,
        kAuxIntegralDwellFrames);
  };
  BoundedHoldIntegralState encoder_integral, vendor_integral;
  for (int frame = 0; frame < 200; ++frame) {
    const auto at = encoder_integral_motor.previous_feedback_at + encoder_step;
    encoder_integral_motor.integral_encoder_velocity =
        observed_integral_encoder_velocity(encoder_integral_motor, 0.0, at);
    encoder_integral_motor.previous_feedback_at = at;
    learn_encoder(encoder_integral, encoder_integral_motor.integral_encoder_velocity);
    learn_encoder(vendor_integral, encoder_integral_motor.last_dq / kGear);
  }
  if (!(encoder_integral.accumulator_nm < -0.01) || vendor_integral.accumulator_nm != 0.0)
    throw std::runtime_error("ENCODER_INTEGRAL_STATIONARY_SELF_TEST_FAILED");
  const double learned_encoder_bias = encoder_integral.accumulator_nm;
  encoder_integral_motor.integral_encoder_velocity = std::numeric_limits<double>::quiet_NaN();
  for (int frame = 0; frame < 100; ++frame) {
    const auto at = encoder_integral_motor.previous_feedback_at + encoder_step;
    const double position = encoder_integral_motor.previous_scaled_position +
        3.0 * kPi / 180.0 * kPeriod;
    encoder_integral_motor.integral_encoder_velocity =
        observed_integral_encoder_velocity(encoder_integral_motor, position, at);
    encoder_integral_motor.previous_scaled_position = position;
    encoder_integral_motor.previous_feedback_at = at;
    learn_encoder(encoder_integral, encoder_integral_motor.integral_encoder_velocity);
  }
  const double gap_velocity = observed_integral_encoder_velocity(
      encoder_integral_motor, encoder_integral_motor.previous_scaled_position,
      encoder_integral_motor.previous_feedback_at + std::chrono::seconds(1));
  learn_encoder(encoder_integral, gap_velocity);
  observe_feedback_frame_validity(encoder_integral_motor, false, 5);
  if (encoder_integral.accumulator_nm != learned_encoder_bias ||
      encoder_integral.dwell_frames != 0 || std::isfinite(gap_velocity) ||
      std::isfinite(encoder_integral_motor.integral_encoder_velocity))
    throw std::runtime_error("ENCODER_INTEGRAL_MOTION_GAP_SELF_TEST_FAILED");

  BoundedHoldIntegralState integral_self_test;
  double integral_wire = 0.0;
  for (int frame = 0; frame < 80; ++frame) {
    integral_wire = update_bounded_hold_integral(
        integral_self_test, true, true, 0.05, 0.0, 0.50,
        kAuxIntegralKiPerRotorRadS, kAuxIntegralRateHardNmS,
        kAuxIntegralEnterError, kAuxIntegralEnterVelocity,
        kAuxIntegralDeadband, kAuxIntegralDwellFrames);
  }
  const double learned_integral = integral_self_test.accumulator_nm;
  const double preserved_wire = update_bounded_hold_integral(
      integral_self_test, true, false, -0.10, 0.0, 0.50,
      kAuxIntegralKiPerRotorRadS, kAuxIntegralRateHardNmS,
      kAuxIntegralEnterError, kAuxIntegralEnterVelocity,
      kAuxIntegralDeadband, kAuxIntegralDwellFrames);
  if (learned_integral <= 0.0 || integral_wire <= 0.0 ||
      std::abs(integral_self_test.accumulator_nm - learned_integral) > 1e-12 ||
      std::abs(preserved_wire - integral_wire) > 1e-12)
    throw std::runtime_error("HOLD_INTEGRAL_PRESERVE_SELF_TEST_FAILED");
  BoundedHoldIntegralState cross_zero_integral;
  cross_zero_integral.accumulator_nm = 0.001;
  cross_zero_integral.dwell_frames = kAuxIntegralDwellFrames;
  if (update_bounded_hold_integral(
          cross_zero_integral, true, true, -0.10, 0.0, 0.50,
          kAuxIntegralKiPerRotorRadS, kAuxIntegralRateHardNmS,
          kAuxIntegralEnterError, kAuxIntegralEnterVelocity,
          kAuxIntegralDeadband, kAuxIntegralDwellFrames) != 0.0 ||
      cross_zero_integral.accumulator_nm != 0.0)
    throw std::runtime_error("HOLD_INTEGRAL_CROSS_ZERO_SELF_TEST_FAILED");
  for (int frame = 0; frame < 1000; ++frame) {
    integral_wire = update_bounded_hold_integral(
        integral_self_test, true, true, 0.10, 0.0, 0.50,
        kAuxIntegralKiPerRotorRadS, kAuxIntegralRateHardNmS,
        kAuxIntegralEnterError, kAuxIntegralEnterVelocity,
        kAuxIntegralDeadband, kAuxIntegralDwellFrames);
  }
  if (integral_self_test.accumulator_nm > 0.50 + 1e-12 ||
      integral_wire > 0.50 + 1e-12 ||
      update_bounded_hold_integral(
          integral_self_test, false, false, 0.0, 0.0, 0.50,
          kAuxIntegralKiPerRotorRadS, kAuxIntegralRateHardNmS,
          kAuxIntegralEnterError, kAuxIntegralEnterVelocity,
          kAuxIntegralDeadband, kAuxIntegralDwellFrames) != 0.0 ||
      integral_self_test.accumulator_nm != 0.0)
    throw std::runtime_error("HOLD_INTEGRAL_BOUND_RESET_SELF_TEST_FAILED");

  J2SyncFaultFilter warning_sync;
  if (observe_j2_sync_error(
          warning_sync, kJ2SyncWarningLimit, 4U, 0U, 0U) ||
      warning_sync.warning || !warning_sync.observation_valid ||
      observe_j2_sync_error(
          warning_sync, kJ2SyncWarningLimit + 1e-9, 4U, 0U, 0U) ||
      !warning_sync.warning || warning_sync.fault ||
      observe_j2_sync_error(
          warning_sync, kJ2SyncLimit, 4U, 0U, 0U) ||
      !warning_sync.warning || warning_sync.fault)
    throw std::runtime_error("J2_SYNC_WARNING_BOUNDARY_SELF_TEST_FAILED");

  J2SyncFaultFilter hard_sync;
  if (!observe_j2_sync_error(
          hard_sync, kJ2SyncLimit + 1e-9, 7U, 3U, 6U) ||
      !hard_sync.fault || !hard_sync.warning ||
      hard_sync.trip_activation_epoch != 7U ||
      hard_sync.minimum_rearm_epoch != 8U)
    throw std::runtime_error("J2_SYNC_SINGLE_PAIR_HARD_SELF_TEST_FAILED");
  for (int frame = 0; frame < kJ2SyncRecoveryConsecutiveFrames - 1; ++frame) {
    if (!observe_j2_sync_error(
            hard_sync, 0.2 * kPi / 180.0, 7U, 8U, 7U) ||
        hard_sync.recovery_ready || !hard_sync.fault)
      throw std::runtime_error("J2_SYNC_RECOVERY_EVIDENCE_SELF_TEST_FAILED");
  }
  if (!observe_j2_sync_error(
          hard_sync, 0.2 * kPi / 180.0, 7U, 8U, 7U) ||
      !hard_sync.recovery_ready || hard_sync.warning ||
      observe_explicit_j2_sync_release(hard_sync, false) ||
      request_j2_sync_rearm_for_next_cycle(
          hard_sync, true, true, true, 8U, 7U) ||
      !observe_explicit_j2_sync_release(hard_sync, true) ||
      request_j2_sync_rearm_for_next_cycle(
          hard_sync, true, true, true, 7U, 7U) ||
      request_j2_sync_rearm_for_next_cycle(
          hard_sync, true, true, true, 8U, 8U) ||
      !request_j2_sync_rearm_for_next_cycle(
          hard_sync, true, true, true, 9U, 8U) ||
      !hard_sync.fault || !hard_sync.rearm_pending_next_cycle)
    throw std::runtime_error("J2_SYNC_EXPLICIT_REARM_SELF_TEST_FAILED");
  if (!apply_pending_j2_sync_rearm_at_cycle_start(hard_sync) ||
      hard_sync.fault || hard_sync.rearm_pending_next_cycle ||
      hard_sync.release_observed || hard_sync.recovery_ready)
    throw std::runtime_error("J2_SYNC_NEXT_CYCLE_REARM_SELF_TEST_FAILED");

  J2SyncFaultFilter unavailable_sync;
  if (!observe_j2_sync_error(
          unavailable_sync, kJ2SyncLimit + 1e-9, 5U, 0U, 0U) ||
      !observe_j2_sync_error(
          unavailable_sync, 0.2 * kPi / 180.0, 5U, 6U, 5U) ||
      !observe_j2_sync_unavailable(unavailable_sync) ||
      unavailable_sync.observation_valid ||
      unavailable_sync.recovery_frames != 0 ||
      unavailable_sync.recovery_ready || !unavailable_sync.fault)
    throw std::runtime_error(
        "J2_SYNC_UNAVAILABLE_RESETS_REARM_SELF_TEST_FAILED");
  J2SyncFaultFilter nonfinite_sync;
  if (!observe_j2_sync_error(
          nonfinite_sync, std::numeric_limits<double>::quiet_NaN(),
          9U, 0U, 0U) ||
      !nonfinite_sync.fault || nonfinite_sync.observation_valid)
    throw std::runtime_error("J2_SYNC_NONFINITE_HARD_SELF_TEST_FAILED");

  GuiCommand fixed_hold = mixed_position;
  fixed_hold.mode = "hold";
  fixed_hold.recovery = false;
  fixed_hold.feedforward_nm.fill(0.0);
  fixed_hold.active_joint_mask.fill(false);
  fixed_hold.moving_joint_mask.fill(false);
  fixed_hold.active_joint_mask[2] = true;
  fixed_hold.targets[2] = 0.10 + 1.0 * kPi / 180.0;
  fixed_hold.activation_epoch = 20U;
  validate_command_for_owned_domain(fixed_hold, mixed_motors);
  GuiCommand out_of_model_hold = fixed_hold;
  out_of_model_hold.targets[2] =
      kModelCommandLower[2] - 0.01 * kPi / 180.0;
  bool out_of_model_hold_rejected = false;
  try {
    validate_command_for_owned_domain(out_of_model_hold, mixed_motors);
  } catch (const std::runtime_error&) {
    out_of_model_hold_rejected = true;
  }
  if (!out_of_model_hold_rejected || fixed_hold.mode != "hold" ||
      command_releases_owned_domain(fixed_hold, mixed_motors) ||
      std::abs(fixed_hold.targets[2] -
               (0.10 + 1.0 * kPi / 180.0)) > 1e-12)
    throw std::runtime_error(
        "REJECTED_TARGET_PRESERVES_HOLD_AUTHORITY_SELF_TEST_FAILED");
  CommandSafetyState fixed_hold_safety;
  validate_and_observe_fixed_hold_targets(
      fixed_hold, mixed_motors, fixed_hold_safety);
  if (!fixed_hold_safety.fixed_target_valid[2] ||
      fixed_hold_safety.fixed_target_epoch[2] != 20U ||
      std::abs(fixed_hold_safety.fixed_target[2] - fixed_hold.targets[2]) >
          1e-12)
    throw std::runtime_error("FIXED_HOLD_INITIAL_CAPTURE_SELF_TEST_FAILED");
  auto stale_hold_motors = mixed_motors;
  for (auto& motor : stale_hold_motors)
    motor.previous_feedback_at =
        Clock::now() - std::chrono::milliseconds(101);
  CommandSafetyState stale_hold_safety;
  bool stale_hold_rejected = false;
  try {
    validate_and_observe_fixed_hold_targets(
        fixed_hold, stale_hold_motors, stale_hold_safety);
  } catch (const std::runtime_error&) {
    stale_hold_rejected = true;
  }
  if (!stale_hold_rejected)
    throw std::runtime_error("FIXED_HOLD_STALE_FEEDBACK_SELF_TEST_FAILED");
  for (auto& motor : mixed_motors) {
    if (motor.joint_index == 2)
      motor.unwrapped += motor.sign * kGear * 5.0 * kPi / 180.0;
  }
  validate_and_observe_fixed_hold_targets(
      fixed_hold, mixed_motors, fixed_hold_safety);
  GuiCommand changed_same_epoch = fixed_hold;
  changed_same_epoch.targets[2] += 0.001;
  bool changed_same_epoch_rejected = false;
  try {
    validate_and_observe_fixed_hold_targets(
        changed_same_epoch, mixed_motors, fixed_hold_safety);
  } catch (const std::runtime_error&) {
    changed_same_epoch_rejected = true;
  }
  if (!changed_same_epoch_rejected ||
      std::abs(fixed_hold_safety.fixed_target[2] - fixed_hold.targets[2]) >
          1e-12)
    throw std::runtime_error("FIXED_HOLD_IMMUTABLE_SELF_TEST_FAILED");
  GuiCommand moving_same_epoch = fixed_hold;
  moving_same_epoch.mode = "position";
  moving_same_epoch.moving_joint_mask[2] = true;
  validate_and_observe_fixed_hold_targets(
      moving_same_epoch, mixed_motors, fixed_hold_safety);
  bool toggle_bypass_rejected = false;
  try {
    validate_and_observe_fixed_hold_targets(
        changed_same_epoch, mixed_motors, fixed_hold_safety);
  } catch (const std::runtime_error&) {
    toggle_bypass_rejected = true;
  }
  if (!toggle_bypass_rejected)
    throw std::runtime_error("FIXED_HOLD_TOGGLE_BYPASS_SELF_TEST_FAILED");
  GuiCommand next_epoch_capture = fixed_hold;
  next_epoch_capture.activation_epoch = 21U;
  next_epoch_capture.targets[2] = 0.10 + 6.0 * kPi / 180.0;
  validate_and_observe_fixed_hold_targets(
      next_epoch_capture, mixed_motors, fixed_hold_safety);
  GuiCommand distant_next_epoch = next_epoch_capture;
  distant_next_epoch.activation_epoch = 22U;
  distant_next_epoch.targets[2] += 3.0 * kPi / 180.0;
  bool distant_capture_rejected = false;
  try {
    validate_and_observe_fixed_hold_targets(
        distant_next_epoch, mixed_motors, fixed_hold_safety);
  } catch (const std::runtime_error&) {
    distant_capture_rejected = true;
  }
  if (!distant_capture_rejected ||
      fixed_hold_safety.fixed_target_epoch[2] != 21U)
    throw std::runtime_error("FIXED_HOLD_CAPTURE_WINDOW_SELF_TEST_FAILED");

  // A rejected active epoch is an incoming-command fence only. It must not
  // mutate the already accepted target, but neither feedback motion nor a
  // switch to moving POSITION may make that epoch authoritative later.
  observe_rejected_active_epoch(
      distant_next_epoch, mixed_motors, fixed_hold_safety);
  GuiCommand same_rejected_epoch = next_epoch_capture;
  same_rejected_epoch.activation_epoch = 22U;
  if (fixed_hold_safety.highest_rejected_active_epoch != 22U ||
      command_epoch_is_acceptable(
          same_rejected_epoch, fixed_hold_safety, mixed_motors))
    throw std::runtime_error("REJECTED_ACTIVE_EPOCH_HOLD_SELF_TEST_FAILED");
  GuiCommand rejected_epoch_position = same_rejected_epoch;
  rejected_epoch_position.mode = "position";
  rejected_epoch_position.moving_joint_mask[2] = true;
  if (command_epoch_is_acceptable(
          rejected_epoch_position, fixed_hold_safety, mixed_motors))
    throw std::runtime_error(
        "REJECTED_ACTIVE_EPOCH_POSITION_SELF_TEST_FAILED");
  if (external_command_can_resume_from_safe_hold(
          same_rejected_epoch, true, mixed_motors, 21U, 0U,
          fixed_hold_safety.highest_rejected_active_epoch))
    throw std::runtime_error("REJECTED_ACTIVE_EPOCH_LEASE_SELF_TEST_FAILED");
  GuiCommand higher_after_rejection = same_rejected_epoch;
  higher_after_rejection.activation_epoch = 23U;
  if (!command_epoch_is_acceptable(
          higher_after_rejection, fixed_hold_safety, mixed_motors) ||
      !external_command_can_resume_from_safe_hold(
          higher_after_rejection, true, mixed_motors, 21U, 0U,
          fixed_hold_safety.highest_rejected_active_epoch))
    throw std::runtime_error("REJECTED_ACTIVE_HIGHER_EPOCH_SELF_TEST_FAILED");
  for (const std::string release_mode : {"brake", "drag"}) {
    GuiCommand release = same_rejected_epoch;
    release.mode = release_mode;
    release.active_joint_mask.fill(false);
    release.moving_joint_mask.fill(false);
    if (!command_epoch_is_acceptable(release, fixed_hold_safety, mixed_motors))
      throw std::runtime_error("REJECTED_ACTIVE_RELEASE_SELF_TEST_FAILED");
  }
  GuiCommand rejected_epoch_deselect = same_rejected_epoch;
  rejected_epoch_deselect.active_joint_mask.fill(false);
  rejected_epoch_deselect.moving_joint_mask.fill(false);
  if (!command_epoch_is_acceptable(
          rejected_epoch_deselect, fixed_hold_safety, mixed_motors))
    throw std::runtime_error("REJECTED_ACTIVE_DESELECT_SELF_TEST_FAILED");

  // Record the moving endpoint so the GUI's automatic POSITION->HOLD
  // transition keeps the exact target even after an external displacement.
  auto transition_motors = mixed_motors;
  double transition_actual = 0.0;
  if (!healthy_logical_position_for_joint(
          transition_motors, 2, transition_actual))
    throw std::runtime_error("POSITION_TARGET_RECORD_FEEDBACK_SELF_TEST_FAILED");
  GuiCommand moving_authority = fixed_hold;
  moving_authority.mode = "position";
  moving_authority.moving_joint_mask[2] = true;
  moving_authority.activation_epoch = 30U;
  moving_authority.targets[2] = transition_actual;
  CommandSafetyState position_target_safety;
  validate_and_observe_fixed_hold_targets(
      moving_authority, transition_motors, position_target_safety);
  if (!position_target_safety.position_target_valid[2] ||
      position_target_safety.position_target_epoch[2] != 30U)
    throw std::runtime_error("POSITION_TARGET_RECORD_SELF_TEST_FAILED");
  GuiCommand changed_moving_target = moving_authority;
  changed_moving_target.targets[2] += 0.001;
  bool changed_moving_rejected = false;
  try {
    validate_and_observe_fixed_hold_targets(
        changed_moving_target, transition_motors, position_target_safety);
  } catch (const std::runtime_error&) {
    changed_moving_rejected = true;
  }
  if (!changed_moving_rejected)
    throw std::runtime_error("POSITION_TARGET_IMMUTABLE_SELF_TEST_FAILED");
  for (auto& motor : transition_motors) {
    if (motor.joint_index == 2)
      motor.unwrapped += motor.sign * kGear * 4.0 * kPi / 180.0;
  }
  CommandSafetyState automatic_hold_safety = position_target_safety;
  GuiCommand automatic_hold = moving_authority;
  automatic_hold.mode = "hold";
  automatic_hold.moving_joint_mask[2] = false;
  bool premature_hold_rejected = false;
  try {
    validate_and_observe_fixed_hold_targets(
        automatic_hold, transition_motors, automatic_hold_safety);
  } catch (const std::runtime_error&) {
    premature_hold_rejected = true;
  }
  if (!premature_hold_rejected)
    throw std::runtime_error(
        "POSITION_TO_HOLD_PREMATURE_SELF_TEST_FAILED");
  automatic_hold_safety = position_target_safety;
  std::array<bool, 6> verified_arrival{};
  std::array<bool, 6> verified_endpoint{};
  std::array<bool, 6> verified_tracking{};
  std::array<std::uint64_t, 6> verified_tracking_epoch{};
  std::array<double, 6> verified_tracking_target{};
  verified_arrival[2] = true;
  verified_endpoint[2] = true;
  verified_tracking[2] = true;
  verified_tracking_epoch[2] = moving_authority.activation_epoch;
  verified_tracking_target[2] = moving_authority.targets[2];
  validate_and_observe_fixed_hold_targets(
      automatic_hold, transition_motors, automatic_hold_safety,
      &verified_arrival, &verified_endpoint, &verified_tracking,
      &verified_tracking_epoch, &verified_tracking_target);
  if (!automatic_hold_safety.fixed_target_valid[2] ||
      std::abs(automatic_hold_safety.fixed_target[2] -
               moving_authority.targets[2]) > 1e-12)
    throw std::runtime_error(
        "POSITION_TO_HOLD_TARGET_REUSE_SELF_TEST_FAILED");

  // Completion proof from E30 must not authorize E31 in the same socket
  // receive batch, even when both epochs use the same numeric endpoint.
  CommandSafetyState batched_transition_safety = position_target_safety;
  GuiCommand batched_position = moving_authority;
  batched_position.activation_epoch = 31U;
  validate_and_observe_fixed_hold_targets(
      batched_position, transition_motors, batched_transition_safety);
  GuiCommand batched_hold = batched_position;
  batched_hold.mode = "hold";
  batched_hold.moving_joint_mask[2] = false;
  bool stale_epoch_proof_rejected = false;
  try {
    validate_and_observe_fixed_hold_targets(
        batched_hold, transition_motors, batched_transition_safety,
        &verified_arrival, &verified_endpoint, &verified_tracking,
        &verified_tracking_epoch, &verified_tracking_target);
  } catch (const std::runtime_error&) {
    stale_epoch_proof_rejected = true;
  }
  if (!stale_epoch_proof_rejected)
    throw std::runtime_error(
        "POSITION_TO_HOLD_STALE_EPOCH_PROOF_SELF_TEST_FAILED");
  verified_tracking_epoch[2] = batched_position.activation_epoch;
  verified_tracking_target[2] += 0.001;
  bool wrong_target_proof_rejected = false;
  try {
    validate_and_observe_fixed_hold_targets(
        batched_hold, transition_motors, batched_transition_safety,
        &verified_arrival, &verified_endpoint, &verified_tracking,
        &verified_tracking_epoch, &verified_tracking_target);
  } catch (const std::runtime_error&) {
    wrong_target_proof_rejected = true;
  }
  if (!wrong_target_proof_rejected)
    throw std::runtime_error(
        "POSITION_TO_HOLD_WRONG_TARGET_PROOF_SELF_TEST_FAILED");
  verified_tracking_target[2] = batched_position.targets[2];
  validate_and_observe_fixed_hold_targets(
      batched_hold, transition_motors, batched_transition_safety,
      &verified_arrival, &verified_endpoint, &verified_tracking,
      &verified_tracking_epoch, &verified_tracking_target);
  if (!batched_transition_safety.fixed_target_valid[2] ||
      batched_transition_safety.fixed_target_epoch[2] != 31U)
    throw std::runtime_error(
        "POSITION_TO_HOLD_MATCHED_RUNTIME_PROOF_SELF_TEST_FAILED");
  double displaced_actual = 0.0;
  if (!healthy_logical_position_for_joint(
          transition_motors, 2, displaced_actual))
    throw std::runtime_error("MANUAL_STOP_FEEDBACK_SELF_TEST_FAILED");
  CommandSafetyState manual_stop_safety = position_target_safety;
  GuiCommand manual_stop = automatic_hold;
  manual_stop.targets[2] = displaced_actual;
  validate_and_observe_fixed_hold_targets(
      manual_stop, transition_motors, manual_stop_safety);
  if (!manual_stop_safety.fixed_target_valid[2] ||
      std::abs(manual_stop_safety.fixed_target[2] - displaced_actual) > 1e-12)
    throw std::runtime_error("MANUAL_STOP_CAPTURE_SELF_TEST_FAILED");

  const auto diagnostic_test_time = Clock::now();
  if (diagnostic_log_interval_elapsed(
          diagnostic_test_time, diagnostic_test_time) ||
      !diagnostic_log_interval_elapsed(
          diagnostic_test_time + std::chrono::milliseconds(5001),
          diagnostic_test_time))
    throw std::runtime_error("DIAGNOSTIC_RATE_LIMIT_SELF_TEST_FAILED");
  for (auto& motor : mixed_motors) {
    const double logical = motor.joint_index == 2 ? 0.10 : 0.20;
    motor.unwrapped = motor.reference + motor.sign * kGear * logical;
  }

  GuiCommand mixed_lease_capture;
  if (!capture_lease_safe_hold_command(
          "j345", mixed_position, mixed_motors, false, false,
          mixed_lease_capture) ||
      std::abs(mixed_lease_capture.targets[2] - 0.05) > 1e-12 ||
      std::abs(mixed_lease_capture.targets[3] - 2.0) > 1e-12 ||
      std::any_of(
          mixed_lease_capture.moving_joint_mask.begin(),
          mixed_lease_capture.moving_joint_mask.end(),
          [](bool moving) { return moving; }))
    throw std::runtime_error("COMMAND_MIXED_LEASE_CAPTURE_SELF_TEST_FAILED");
  GuiCommand recovery_command;
  bool recovery_command_rejected = false;
  try {
    parse_command(fresh_source_self_test_payload(
        R"({"schema":"go-m8010-gui-command/1.2","sequence":2,
        "mode":"position","targets_rad":[-0.02,-1.53,-0.87,-0.94,0.0,-0.03],
        "active_joint_mask":[true,true,true,true,true,true],
        "activation_epoch":8,"maximum_velocity_rad_s":0.01,
        "maximum_acceleration_rad_s2":0.03,
        "kp":[0.5,1.0,0.6,0.5,0.5,0.0],
        "kd":[0.05,0.10,0.05,0.05,0.05,0.0],
        "feedforward_nm":[0.0,0.0,0.0,0.0,0.0,0.0],
        "recovery":true})"), recovery_command);
  } catch (const std::runtime_error& error) {
    recovery_command_rejected =
        std::string(error.what()) == "COMMAND_RECOVERY_UNAUTHORIZED";
  }
  if (!recovery_command_rejected)
    throw std::runtime_error("RECOVERY_COMMAND_AUTHORIZATION_SELF_TEST_FAILED");
  GuiCommand lease_source;
  parse_command(fresh_source_self_test_payload(
      R"({"schema":"go-m8010-gui-command/1.2","sequence":3,
      "mode":"position","targets_rad":[0.0,0.04,0.0,0.0,0.0,0.0],
      "active_joint_mask":[false,true,false,false,false,false],
      "moving_joint_mask":[false,true,false,false,false,false],
      "activation_epoch":8,"maximum_velocity_rad_s":0.01,
      "maximum_acceleration_rad_s2":0.03,
      "kp":[0.5,1.0,0.6,0.5,0.5,0.0],
      "kd":[0.05,0.10,0.05,0.05,0.05,0.0]})"), lease_source);
  if (!lease_source.received || lease_source.recovery)
    throw std::runtime_error("NORMAL_COMMAND_RECOVERY_STATE_SELF_TEST_FAILED");
  auto lease_motors = make_motors("j2");
  for (auto& motor : lease_motors) {
    motor.reference = motor.id == 0 ? -3.0 : 4.0;
    motor.unwrapped = motor.reference + motor.sign * kGear * 0.25;
    motor.reference_ready = true;
    motor.valid = true;
    motor.last_frame_valid = true;
    motor.fault_latched = false;
    motor.merror = 0;
    motor.temperature = 25;
    motor.returned_mode = kFocMode;
  }
  if (owned_domain_feedback_requires_brake(lease_motors))
    throw std::runtime_error("DOMAIN_FEEDBACK_HEALTH_SELF_TEST_FAILED");
  for (int frame = 1; frame < 5; ++frame) {
    if (observe_feedback_frame_validity(lease_motors[0], false, 5) ||
        !lease_motors[0].valid || lease_motors[0].last_frame_valid ||
        owned_domain_feedback_requires_brake(lease_motors))
      throw std::runtime_error(
          "DOMAIN_TRANSIENT_FEEDBACK_HOLD_SELF_TEST_FAILED");
  }
  if (!observe_feedback_frame_validity(lease_motors[0], false, 5) ||
      lease_motors[0].valid ||
      !owned_domain_feedback_requires_brake(lease_motors))
    throw std::runtime_error(
        "DOMAIN_SUSTAINED_FEEDBACK_BRAKE_SELF_TEST_FAILED");
  lease_motors[0].fault_latched = false;
  if (observe_feedback_frame_validity(lease_motors[0], true, 5) ||
      !lease_motors[0].valid || !lease_motors[0].last_frame_valid ||
      lease_motors[0].consecutive_invalid != 0)
    throw std::runtime_error("DOMAIN_FEEDBACK_RECOVERY_SELF_TEST_FAILED");
  lease_source.active_joint_mask.fill(false);
  lease_source.moving_joint_mask.fill(false);
  lease_source.active_joint_mask[1] = true;
  lease_source.moving_joint_mask[1] = true;
  lease_source.activation_epoch = 8U;
  GuiCommand lease_capture;
  if (!selected_owned_motors_confirmed_foc(lease_source, lease_motors) ||
      !capture_lease_safe_hold_command(
          "j2", lease_source, lease_motors, false, false, lease_capture) ||
      lease_capture.mode != "hold" ||
      std::abs(lease_capture.targets[1] - 0.04) > 1e-12 ||
      std::abs(lease_capture.targets[1] - 0.25) < 1e-6 ||
      !lease_capture.active_joint_mask[1])
    throw std::runtime_error("LEASE_SAFE_HOLD_CAPTURE_SELF_TEST_FAILED");
  GuiCommand hold_source = lease_source;
  hold_source.mode = "hold";
  hold_source.recovery = false;
  hold_source.feedforward_nm.fill(0.0);
  hold_source.moving_joint_mask.fill(false);
  hold_source.targets[1] = 0.04;
  validate_command_for_owned_domain(hold_source, lease_motors);
  GuiCommand wide_hold_target = hold_source;
  wide_hold_target.targets[1] = -1.0;
  validate_command_for_owned_domain(wide_hold_target, lease_motors);
  GuiCommand hold_capture;
  if (!capture_lease_safe_hold_command(
          "j2", hold_source, lease_motors, false, false, hold_capture) ||
      std::abs(hold_capture.targets[1] - 0.04) > 1e-12)
    throw std::runtime_error("LEASE_SAFE_HOLD_TARGET_SELF_TEST_FAILED");
  const double external_displacement = 2.0 * kPi / 180.0;
  for (auto& motor : lease_motors)
    motor.unwrapped = motor.reference + motor.sign * kGear *
        (hold_source.targets[1] + external_displacement);
  if (!capture_lease_safe_hold_command(
          "j2", hold_source, lease_motors, false, false, hold_capture) ||
      std::abs(hold_capture.targets[1] - 0.04) > 1e-12 ||
      external_displacement <= 1.5 * kPi / 180.0 ||
      std::min(hold_source.kp[1], lease_motors[0].kp_limit) * kGear *
              external_displacement >= kJ2RecoveryPredictedRotorWorkNm ||
      !use_j2_hold_protection_limits(hold_source, "hold") ||
      should_apply_velocity_guard(hold_source, "hold", 1U))
    throw std::runtime_error("LEASE_SAFE_HOLD_EXTERNAL_FORCE_SELF_TEST_FAILED");
  GuiCommand normal_position = lease_source;
  normal_position.recovery = false;
  normal_position.feedforward_nm.fill(0.0);
  if (!use_j2_moving_limits(normal_position, "position") ||
      use_j2_hold_protection_limits(normal_position, "position") ||
      use_j2_moving_limits(normal_position, "position", true) ||
      !use_j2_hold_protection_limits(normal_position, "position", true) ||
      !should_apply_velocity_guard(normal_position, "position", 1U) ||
      use_j2_moving_limits(hold_source, "hold") ||
      !use_j2_hold_protection_limits(hold_source, "hold") ||
      should_apply_velocity_guard(
          normal_position, "position", 1U, true))
    throw std::runtime_error("J2_HOLD_PROTECTION_SELF_TEST_FAILED");
  GuiCommand maximum_gain_position = normal_position;
  maximum_gain_position.kp[1] = 3.0;
  maximum_gain_position.kd[1] = 0.30;
  const auto moving_gain_targets = j2_gain_targets(
      maximum_gain_position, "position", lease_motors[0]);
  const auto arrived_gain_targets = j2_gain_targets(
      maximum_gain_position, "position", lease_motors[0], true);
  GuiCommand maximum_gain_hold = maximum_gain_position;
  maximum_gain_hold.mode = "hold";
  maximum_gain_hold.moving_joint_mask[1] = false;
  const auto hold_gain_targets = j2_gain_targets(
      maximum_gain_hold, "hold", lease_motors[0]);
  if (std::abs(moving_gain_targets.first - kJ2MovingKpLimit) > 1e-12 ||
      std::abs(moving_gain_targets.second - kJ2MovingKdLimit) > 1e-12 ||
      std::abs(arrived_gain_targets.first - 3.0) > 1e-12 ||
      std::abs(arrived_gain_targets.second - 0.30) > 1e-12 ||
      std::abs(hold_gain_targets.first - 3.0) > 1e-12 ||
      std::abs(hold_gain_targets.second - 0.30) > 1e-12)
    throw std::runtime_error("J2_PHASE_GAIN_SELF_TEST_FAILED");

  double affine_lower = 0.0;
  double affine_upper = 1.0;
  if (!intersect_absolute_affine_constraint(
          +0.55, -0.20, 0.50, affine_lower, affine_upper) ||
      !intersect_absolute_affine_constraint(
          -0.53, +0.20, 0.50, affine_lower, affine_upper) ||
      std::abs(affine_lower - 0.25) > 1e-12 ||
      std::abs(affine_upper - 1.0) > 1e-12)
    throw std::runtime_error("J2_AFFINE_INTERVAL_SELF_TEST_FAILED");

  auto replay_motors = make_motors("j2");
  const double replay_measured_q = 5.575 * kPi / 180.0;
  replay_motors[0].reference = -3.0;
  replay_motors[1].reference = 4.0;
  for (auto& motor : replay_motors)
    motor.unwrapped = motor.reference +
        motor.sign * kGear * replay_measured_q;
  replay_motors[0].last_dq = -1.47263;
  replay_motors[1].last_dq = +1.37445;
  const double replay_measured_dq = 0.5 *
      (-replay_motors[0].last_dq / kGear +
       replay_motors[1].last_dq / kGear);
  const J2GovernedReference replay_governed = govern_j2_reference(
      replay_motors, replay_measured_q, replay_measured_dq,
      5.0 * kPi / 180.0, 5.0 * kPi / 180.0,
      kJ2MovingKpLimit, kJ2MovingKdLimit, 0.0,
      kJ2PredictedRotorWorkNm, kJ2PredictedRotorPdHardNm);
  if (!replay_governed.feasible || replay_governed.alpha < 1.0 - 1e-12 ||
      std::any_of(
          replay_governed.predicted_work_nm.begin(),
          replay_governed.predicted_work_nm.end(),
          [](double value) {
            return std::abs(value) > kJ2PredictedRotorWorkNm + 1e-12;
          }) ||
      std::any_of(
          replay_governed.predicted_pd_nm.begin(),
          replay_governed.predicted_pd_nm.end(),
          [](double value) {
            return std::abs(value) > kJ2PredictedRotorPdHardNm + 1e-12;
          }))
    throw std::runtime_error("J2_EVENT_REPLAY_GOVERNOR_SELF_TEST_FAILED");

  auto displaced_motors = make_motors("j2");
  const double displaced_target = 0.04;
  const double displaced_measured =
      displaced_target + 20.0 * kPi / 180.0;
  displaced_motors[0].reference = -3.0;
  displaced_motors[1].reference = 4.0;
  for (auto& motor : displaced_motors) {
    motor.unwrapped = motor.reference +
        motor.sign * kGear * displaced_measured;
    motor.last_dq = 0.0;
  }
  const J2GovernedReference displaced_governed = govern_j2_reference(
      displaced_motors, displaced_measured, 0.0,
      displaced_target, 0.0, 3.0, 0.30, 0.0,
      kJ2RecoveryPredictedRotorWorkNm,
      kJ2RecoveryPredictedRotorPdHardNm);
  if (!displaced_governed.feasible || displaced_governed.alpha <= 0.0 ||
      displaced_governed.alpha >= 1.0 ||
      displaced_governed.q >= displaced_measured ||
      displaced_governed.q <= displaced_target ||
      std::any_of(
          displaced_governed.predicted_work_nm.begin(),
          displaced_governed.predicted_work_nm.end(),
          [](double value) {
            return std::abs(value) >
                kJ2RecoveryPredictedRotorWorkNm + 1e-12;
          }))
    throw std::runtime_error(
        "J2_EXTERNAL_DISPLACEMENT_GOVERNOR_SELF_TEST_FAILED");
  const J2GovernedReference moving_limited_at_profile_endpoint =
      govern_j2_reference(
          displaced_motors, displaced_measured, 0.0,
          displaced_target, 0.0,
          kJ2MovingKpLimit, kJ2MovingKdLimit, 0.0,
          kJ2PredictedRotorWorkNm, kJ2PredictedRotorPdHardNm);
  normal_position.targets[1] = displaced_target;
  if (!position_profile_at_authorized_endpoint(
          normal_position, 1U, displaced_target, 0.0) ||
      use_j2_moving_limits(normal_position, "position", true) ||
      !use_j2_hold_protection_limits(normal_position, "position", true) ||
      !moving_limited_at_profile_endpoint.feasible ||
      moving_limited_at_profile_endpoint.alpha <= 0.0 ||
      moving_limited_at_profile_endpoint.alpha >= 1.0 ||
      displaced_governed.alpha <=
          moving_limited_at_profile_endpoint.alpha + 1e-12 ||
      std::abs(normal_position.targets[1] - displaced_target) > 1e-12)
    throw std::runtime_error(
        "J2_PROFILE_ENDPOINT_HOLD_PROTECTION_SELF_TEST_FAILED");
  normal_position.targets[1] = 0.04;
  std::array<bool, 6> completed_position{};
  completed_position[1] = true;
  GuiCommand completed_position_capture;
  if (!capture_lease_safe_hold_command(
          "j2", normal_position, lease_motors, false, false,
          completed_position_capture, &completed_position) ||
      std::abs(completed_position_capture.targets[1] - 0.04) > 1e-12)
    throw std::runtime_error(
        "LEASE_SAFE_HOLD_COMPLETED_POSITION_TARGET_SELF_TEST_FAILED");
  for (auto& motor : lease_motors)
    motor.unwrapped = motor.reference + motor.sign * kGear * 0.25;
  GuiCommand same_epoch = lease_source;
  GuiCommand higher_epoch = lease_source;
  higher_epoch.activation_epoch = 9U;
  if (external_command_can_resume_from_safe_hold(
          same_epoch, true, lease_motors, 8U, 9U) ||
      !external_command_can_resume_from_safe_hold(
          higher_epoch, true, lease_motors, 8U, 9U))
    throw std::runtime_error("LEASE_SAFE_HOLD_EPOCH_SELF_TEST_FAILED");
  GuiCommand explicit_brake = lease_source;
  explicit_brake.mode = "brake";
  explicit_brake.active_joint_mask.fill(false);
  if (!external_command_explicitly_releases_safe_hold(
          explicit_brake, true, lease_motors))
    throw std::runtime_error("LEASE_SAFE_HOLD_BRAKE_SELF_TEST_FAILED");
  GuiCommand explicit_drag = lease_source;
  explicit_drag.mode = "drag";
  if (!external_command_explicitly_releases_safe_hold(
          explicit_drag, true, lease_motors))
    throw std::runtime_error("LEASE_SAFE_HOLD_DRAG_SELF_TEST_FAILED");
  lease_motors[0].returned_mode = kBrakeMode;
  if (selected_owned_motors_confirmed_foc(lease_source, lease_motors))
    throw std::runtime_error("LEASE_SAFE_HOLD_FOC_SELF_TEST_FAILED");
  GuiCommand same_authority_heartbeat = lease_source;
  ++same_authority_heartbeat.source_sequence;
  ++same_authority_heartbeat.source_monotonic_ns;
  GuiCommand changed_authority = same_authority_heartbeat;
  changed_authority.targets[1] += 0.01;
  GuiCommand transient_mode_capture;
  if (!same_external_hold_authority(
          lease_source, same_authority_heartbeat) ||
      same_external_hold_authority(lease_source, changed_authority) ||
      !capture_lease_safe_hold_command(
          "j2", lease_source, lease_motors, false, false,
          transient_mode_capture) ||
      std::abs(transient_mode_capture.targets[1] -
               lease_source.targets[1]) > 1e-12)
    throw std::runtime_error(
        "LEASE_SAFE_HOLD_STICKY_TRANSIENT_SELF_TEST_FAILED");
  lease_motors[0].returned_mode = kFocMode;
  if (capture_lease_safe_hold_command(
          "j2", lease_source, lease_motors, true, false, lease_capture))
    throw std::runtime_error("LEASE_SAFE_HOLD_DOMAIN_SELF_TEST_FAILED");
  lease_motors[0].fault_latched = true;
  if (capture_lease_safe_hold_command(
          "j2", lease_source, lease_motors, false, false, lease_capture))
    throw std::runtime_error("LEASE_SAFE_HOLD_FAULT_SELF_TEST_FAILED");
  lease_motors[0].fault_latched = false;
  lease_motors[0].unwrapped -=
      kGear * (kJ2SyncWarningLimit + 1e-6);
  if (!capture_lease_safe_hold_command(
          "j2", lease_source, lease_motors, false, false, lease_capture) ||
      std::abs(lease_capture.targets[1] - lease_source.targets[1]) > 1e-12)
    throw std::runtime_error(
        "LEASE_SAFE_HOLD_WARNING_SYNC_SELF_TEST_FAILED");
  if (capture_lease_safe_hold_command(
          "j2", lease_source, lease_motors, false, true, lease_capture))
    throw std::runtime_error("LEASE_SAFE_HOLD_SYNC_SELF_TEST_FAILED");
  CommandSafetyState safety;
  observe_valid_command(command, j345_motors, safety);
  GuiCommand replay = command;
  command.activation_epoch = 8U;
  observe_valid_command(command, j345_motors, safety);
  if (command_epoch_is_acceptable(replay, safety, j345_motors))
    throw std::runtime_error("COMMAND_REPLAY_SELF_TEST_FAILED");
  observe_interarrival_lease(
      command, command.received_at + std::chrono::milliseconds(501),
      j345_motors, safety);
  command.mode = "brake";
  command.active_joint_mask.fill(false);
  command.moving_joint_mask.fill(false);
  observe_valid_command(command, j345_motors, safety);
  if (safety.minimum_activation_epoch != 9U)
    throw std::runtime_error("COMMAND_BRAKE_LATCH_SELF_TEST_FAILED");
  GuiCommand same_epoch_active = mixed_position;
  same_epoch_active.activation_epoch = 8U;
  if (command_epoch_is_acceptable(
          same_epoch_active, safety, j345_motors))
    throw std::runtime_error("COMMAND_RELEASE_BACKLOG_SELF_TEST_FAILED");
  GuiCommand higher_epoch_active = same_epoch_active;
  higher_epoch_active.activation_epoch = 9U;
  if (!command_epoch_is_acceptable(
          higher_epoch_active, safety, j345_motors))
    throw std::runtime_error("COMMAND_HIGHER_EPOCH_SELF_TEST_FAILED");
  GuiCommand domain_deselect = higher_epoch_active;
  domain_deselect.active_joint_mask.fill(false);
  domain_deselect.moving_joint_mask.fill(false);
  observe_valid_command(domain_deselect, j345_motors, safety);
  if (safety.minimum_activation_epoch != 10U ||
      command_epoch_is_acceptable(
          higher_epoch_active, safety, j345_motors))
    throw std::runtime_error("COMMAND_DOMAIN_DESELECT_FENCE_SELF_TEST_FAILED");
  CommandSafetyState drag_safety;
  GuiCommand drag_active = mixed_position;
  drag_active.activation_epoch = 5U;
  observe_valid_command(drag_active, j345_motors, drag_safety);
  GuiCommand drag_release = drag_active;
  drag_release.mode = "drag";
  drag_release.moving_joint_mask.fill(false);
  observe_valid_command(drag_release, j345_motors, drag_safety);
  if (drag_safety.minimum_activation_epoch != 6U ||
      command_epoch_is_acceptable(
          drag_active, drag_safety, j345_motors))
    throw std::runtime_error("COMMAND_DRAG_FENCE_SELF_TEST_FAILED");
  CommandSafetyState empirical_release_safety;
  empirical_release_safety.gravity_empirical_active = true;
  empirical_release_safety.gravity_empirical_envelope_sha256 =
      std::string(64U, 'a');
  spend_empirical_gravity_authority(empirical_release_safety);
  GuiCommand raced_empirical = mixed_position;
  raced_empirical.activation_epoch = 100U;
  raced_empirical.gravity_authority.present = true;
  raced_empirical.gravity_authority.official_continuous_authority = false;
  raced_empirical.gravity_authority.empirical_envelope_sha256 =
      std::string(64U, 'a');
  bool empirical_race_rejected = false;
  try {
    validate_and_observe_gravity_policy(
        raced_empirical, j345_motors, empirical_release_safety);
  } catch (const std::runtime_error& error) {
    empirical_race_rejected =
        std::string(error.what()) == "COMMAND_EMPIRICAL_ENVELOPE_SPENT";
  }
  if (!empirical_race_rejected ||
      empirical_release_safety.gravity_empirical_active)
    throw std::runtime_error(
        "COMMAND_EMPIRICAL_RELEASE_SPEND_SELF_TEST_FAILED");
  command.activation_epoch = static_cast<std::uint64_t>(
      std::numeric_limits<std::int64_t>::max());
  observe_valid_command(command, j345_motors, safety);
  if (safety.minimum_activation_epoch != command.activation_epoch + 1U)
    throw std::runtime_error("COMMAND_HARD_STOP_FENCE_SELF_TEST_FAILED");
}

CommandReceiveResult receive_latest(
    int socket_fd, GuiCommand& command, CommandSafetyState& safety,
    const std::vector<MotorRuntime>& motors,
    const std::array<bool, 6>& position_arrived_once,
    const std::array<bool, 6>& position_endpoint_reached,
    const std::array<bool, 6>& position_tracking,
    const std::array<std::uint64_t, 6>& position_tracking_epoch,
    const std::array<double, 6>& position_tracking_target,
    CommandReceiveState& receive_state) {
  std::array<char, 8192> buffer{};
  CommandReceiveResult result;
  flush_command_receive_summaries(receive_state, Clock::now(), false);
  for (std::size_t packet_index = 0;
       packet_index < kCommandPacketBudget; ++packet_index) {
    const ssize_t received = ::recv(socket_fd, buffer.data(), buffer.size(), 0);
    if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      return result;
    if (received <= 0) return result;
    ++result.packets_processed;
    try {
      observe_interarrival_lease(command, Clock::now(), motors, safety);
      GuiCommand candidate;
      parse_command(
          std::string(buffer.data(), static_cast<std::size_t>(received)), candidate);
      // Replay/takeover validation is read-only. Check it before an invalid
      // active attempt can raise the rejected-epoch fence, but commit source
      // state only after every domain and target validation succeeds.
      validate_command_source_replay(candidate, receive_state);
      if (!receive_state.active_source_instance_id.empty() &&
          candidate.source_instance_id !=
              receive_state.active_source_instance_id &&
          is_position_holding_mode(candidate.mode) &&
          command_selects_owned_joint(candidate, motors) &&
          candidate.activation_epoch <= safety.last_seen_activation_epoch)
        throw std::runtime_error(
            "COMMAND_SOURCE_TAKEOVER_EPOCH_REPLAY");
      if (!command_epoch_is_acceptable(candidate, safety, motors))
        throw std::runtime_error("COMMAND_ACTIVATION_EPOCH_REPLAY");
      CommandSafetyState candidate_safety = safety;
      try {
        validate_command_for_owned_domain(candidate, motors);
        validate_and_observe_gravity_policy(
            candidate, motors, candidate_safety);
        validate_guidance_return_only(candidate, motors, candidate_safety);
        validate_and_observe_hand_guidance(candidate, motors, candidate_safety);
        if (!candidate.hand_guidance.present)
          validate_and_observe_assisted_teach(candidate, motors, candidate_safety);
        validate_and_observe_position_authority(
            candidate, motors, candidate_safety);
        validate_and_observe_fixed_hold_targets(
            candidate, motors, candidate_safety,
            &position_arrived_once, &position_endpoint_reached,
            &position_tracking, &position_tracking_epoch,
            &position_tracking_target);
      } catch (const GuidanceReferenceRejected&) {
        // An otherwise authorized 1.5 reference can be corrected next frame;
        // no goal, accepted-source state, lease or epoch is committed here.
        throw;
      } catch (...) {
        // Do not commit the candidate or its target, but permanently fence its
        // active epoch. The already accepted older command remains untouched.
        observe_rejected_active_epoch(candidate, motors, safety);
        throw;
      }
      const bool domain_release =
          command_releases_owned_domain(candidate, motors);
      if (domain_release)
        spend_empirical_gravity_authority(candidate_safety);
      observe_valid_command(candidate, motors, candidate_safety);
      observe_command_source(candidate, receive_state);
      safety = std::move(candidate_safety);
      result.domain_release_received =
          result.domain_release_received || domain_release;
      command = std::move(candidate);
    }
    catch (const std::exception& error) {
      const std::string reason = command_rejection_reason(error);
      if (reason.rfind("COMMAND_GRAVITY_", 0U) == 0U ||
          reason.rfind("COMMAND_EMPIRICAL_", 0U) == 0U) {
        // An omitted or launch-binding-mismatched authority revokes the
        // cached active command immediately.  Do not keep an older FOC/HOLD
        // alive for the remainder of its lease after the identity violation.
        spend_empirical_gravity_authority(safety);
        command = GuiCommand{};
        result.domain_release_received = true;
      }
      record_command_rejection(reason, receive_state, Clock::now());
    }
  }
  result.packet_budget_reached = true;
  record_command_budget_reached(receive_state, Clock::now());
  return result;
}

void update_profile(int joint, const GuiCommand& command,
                    std::array<double, 6>& q_command,
                    std::array<double, 6>& dq_command) {
  const double amax = joint == 1
      ? std::min(command.amax, kJ2MaximumAcceleration) : command.amax;
  const double error = command.targets[static_cast<std::size_t>(joint)] -
                       q_command[static_cast<std::size_t>(joint)];
  const double stopping_speed = std::sqrt(2.0 * amax * std::abs(error));
  const double wanted = std::copysign(std::min(command.vmax, stopping_speed), error);
  const double previous_speed = std::abs(dq_command[static_cast<std::size_t>(joint)]);
  const double delta_v = std::clamp(
      wanted - dq_command[static_cast<std::size_t>(joint)],
      -amax * kPeriod, amax * kPeriod);
  dq_command[static_cast<std::size_t>(joint)] += delta_v;
  const double current_speed = std::abs(dq_command[static_cast<std::size_t>(joint)]);
  const double snap_distance = current_speed * kPeriod +
                               0.5 * amax * kPeriod * kPeriod;
  if (previous_speed <= amax * kPeriod &&
      current_speed <= amax * kPeriod && std::abs(error) <= snap_distance) {
    q_command[static_cast<std::size_t>(joint)] = command.targets[static_cast<std::size_t>(joint)];
    dq_command[static_cast<std::size_t>(joint)] = 0.0;
  } else {
    q_command[static_cast<std::size_t>(joint)] +=
        dq_command[static_cast<std::size_t>(joint)] * kPeriod;
  }
}

QuinticSampleClock quintic_sample_clock(
    const QuinticTrajectoryDescriptor& trajectory,
    std::uint64_t now_monotonic_ns) {
  if (!trajectory.present || trajectory.duration_ns == 0U ||
      trajectory.interval_count == 0U)
    throw std::runtime_error("COMMAND_QUINTIC_RUNTIME_DESCRIPTOR_INVALID");
  if (now_monotonic_ns < trajectory.execute_at_monotonic_ns)
    return {};
  const std::uint64_t elapsed_ns =
      now_monotonic_ns - trajectory.execute_at_monotonic_ns;
  if (elapsed_ns >= trajectory.duration_ns)
    return {trajectory.interval_count, "COMPLETE", true};
  // This is the protocol's integer clock, not a floating-time
  // reconstruction.  The 128-bit numerator prevents overflow while preserving
  // exactly floor((now-execute_at)*N/duration_ns).
  const unsigned __int128 numerator =
      static_cast<unsigned __int128>(elapsed_ns) *
      static_cast<unsigned __int128>(trajectory.interval_count);
  const auto sample_index = static_cast<std::uint64_t>(
      numerator / static_cast<unsigned __int128>(trajectory.duration_ns));
  return {sample_index, "RUNNING", false};
}

void apply_quintic_reference(
    int joint, const GuiCommand& command, const QuinticSampleClock& sample,
    std::array<double, 6>& q_command,
    std::array<double, 6>& dq_command) {
  if (!command.quintic.present)
    throw std::runtime_error("COMMAND_QUINTIC_RUNTIME_DESCRIPTOR_MISSING");
  const auto index = static_cast<std::size_t>(joint);
  if (sample.endpoint ||
      sample.sample_index >= command.quintic.interval_count) {
    // Never leave the endpoint to polynomial rounding.
    q_command[index] = command.quintic.target_rad[index];
    dq_command[index] = 0.0;
    return;
  }
  const double normalized = static_cast<double>(sample.sample_index) /
      static_cast<double>(command.quintic.interval_count);
  const double normalized_squared = normalized * normalized;
  const double normalized_cubed = normalized_squared * normalized;
  const double blend = normalized_cubed *
      (10.0 + normalized * (-15.0 + 6.0 * normalized));
  const double blend_derivative =
      30.0 * normalized_squared *
      (1.0 - normalized) * (1.0 - normalized);
  const double displacement = command.quintic.target_rad[index] -
      command.quintic.start_rad[index];
  const double duration_seconds =
      static_cast<double>(command.quintic.duration_ns) * 1e-9;
  q_command[index] = command.quintic.start_rad[index] +
      displacement * blend;
  dq_command[index] = displacement * blend_derivative / duration_seconds;
}

std::uint64_t monotonic_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count());
}

std::string thermal_state_for_raw_temperature(
    int raw_temperature_c, const ThermalInterlockState& thermal_interlock) {
  if (raw_temperature_c < 0) return "OFFLINE";
  if (thermal_interlock.fault_latched) {
    if (thermal_interlock.cooldown_ready &&
        raw_temperature_c < g_thermal_policy.rearm_below_c)
      return "WAIT_OPERATOR_CONFIRM";
    if (raw_temperature_c < g_thermal_policy.rearm_below_c)
      return "COOLDOWN";
    return "THERMAL_STOP";
  }
  if (raw_temperature_c >= g_thermal_policy.thermal_stop_c)
    return "THERMAL_STOP";
  if (raw_temperature_c >= g_thermal_policy.derating_start_c)
    return "DERATING";
  if (raw_temperature_c >= g_thermal_policy.normal_below_c)
    return "WARNING";
  return "NORMAL";
}

std::string feedback_payload(const std::vector<MotorRuntime>& motors,
                             std::uint64_t stamp, const std::string& mode,
                             const J2SyncFaultFilter& j2_sync_interlock,
                             bool domain_fault,
                             bool lease_safe_hold,
                             const GuiCommand& control_command,
                             const std::array<double, 6>&
                                 applied_gravity_feedforward_nm,
                             const std::string& trajectory_state,
                             std::uint64_t trajectory_sample_index,
                             const ThermalInterlockState& thermal_interlock,
                             const NoProgressWatchdogState& no_progress_watchdog,
                             bool no_progress_observation_valid,
                             double no_progress_position_error_rad,
                             bool software_saturation_observed,
                             double thermal_derating_factor,
                             const AssistedTeachExitHold& teach_exit_hold,
                             const GuiCommand* accepted_guidance = nullptr,
                             bool guidance_paused = false,
                             const std::string& guidance_paused_reason = "DEADMAN_TIMEOUT") {
  nlohmann::json samples = nlohmann::json::array();
  nlohmann::json controller_mode_by_motor = nlohmann::json::object();
  nlohmann::json thermal_state_by_motor = nlohmann::json::object();
  nlohmann::json tau_j2_logical_total_nm = nullptr;
  int maximum_raw_temperature_c = std::numeric_limits<int>::min();
  for (const auto& motor : motors) {
    maximum_raw_temperature_c = std::max(
        maximum_raw_temperature_c, motor.temperature);
    samples.push_back({
        {"motor", motor.name}, {"position_rad", motor.last_q},
        {"velocity_rad_s", motor.last_dq},
        {"tau_cmd_rotor_nm", motor.last_tau_cmd_rotor_nm},
        {"tau_feedback_rotor_nm", motor.last_tau},
        {"tau_joint_estimated_nm", motor.sign * motor.last_tau * kGear},
        {"last_valid_feedback_monotonic_ns",
         motor.last_valid_feedback_monotonic_ns == 0U
             ? nlohmann::json(nullptr)
             : nlohmann::json(motor.last_valid_feedback_monotonic_ns)},
        {"gravity_feedforward_rotor_nm",
         motor.sign * applied_gravity_feedforward_nm[
             static_cast<std::size_t>(motor.joint_index)]},
        {"temperature_c", motor.temperature},
        {"merror", motor.merror},
        {"communication_ok",
         motor.last_frame_valid && motor.valid && !motor.fault_latched},
        {"thermal_fault_latched", motor.thermal_fault_latched},
        {"load_limit_no_progress", no_progress_watchdog.fault_latched},
        {"trajectory_plan_token_id", control_command.quintic.present
             ? control_command.quintic.plan_token_id : ""},
        {"trajectory_sha256", control_command.quintic.present
             ? control_command.quintic.trajectory_sha256 : ""},
        {"trajectory_state", trajectory_state},
        {"trajectory_sample_index", trajectory_sample_index},
        {"trajectory_interval_count", control_command.quintic.present
             ? control_command.quintic.interval_count : 0U},
        {"unwrapped_raw_position_rad", motor.unwrapped},
        {"software_zero_reference_raw_rad", motor.reference},
        {"recovery_hint_configured", motor.recovery_hint_configured},
        {"recovery_hint_logical_position_rad", motor.recovery_hint},
        {"j2_session_reference_configured",
         motor.session_reference_configured},
        {"j2_session_reference_raw_rad", motor.session_reference},
        {"j2_session_logical_position_rad",
         motor.session_logical_position},
        {"power_session_reference_configured",
         motor.session_reference_configured},
        {"power_session_reference_raw_rad", motor.session_reference},
        {"power_session_logical_position_rad",
         motor.session_logical_position}});
    if (accepted_guidance != nullptr && accepted_guidance->hand_guidance.present) {
      samples.back()["accepted_guidance_target_rad"] = accepted_guidance->targets[static_cast<std::size_t>(motor.joint_index)];
      samples.back()["accepted_guidance_activation_epoch"] = accepted_guidance->activation_epoch;
      if (guidance_paused) samples.back()["guidance_paused_reason"] = guidance_paused_reason;
    }
    controller_mode_by_motor[motor.name] =
        !motor.last_frame_valid || !motor.valid || motor.fault_latched
            ? "unknown"
            : motor.returned_mode == kBrakeMode
                  ? "brake"
                  : mode != "brake" && motor.returned_mode == kFocMode
                        ? (mode == "teach" && !joint_is_assisted_teach(
                              control_command, mode, static_cast<std::size_t>(motor.joint_index))
                               ? "hold" : mode) : "unknown";
    thermal_state_by_motor[motor.name] =
        thermal_state_for_raw_temperature(
            motor.temperature, thermal_interlock);
  }
  if (motors.size() == 2U && motors[0].name == "J2A" &&
      motors[1].name == "J2B") {
    tau_j2_logical_total_nm =
        motors[0].sign * motors[0].last_tau * kGear +
        motors[1].sign * motors[1].last_tau * kGear;
  }
  const std::string thermal_state = thermal_state_for_raw_temperature(
      maximum_raw_temperature_c, thermal_interlock);
  nlohmann::json payload = {
      {"schema", "go-m8010-motor-feedback/1.0"},
      {"source_monotonic_ns", stamp}, {"samples", samples},
      {"tau_j2_logical_total_nm", tau_j2_logical_total_nm},
      {"controller_mode", mode},
      {"controller_mode_by_motor", controller_mode_by_motor},
      {"j2_sync_fault", j2_sync_interlock.fault},
      {"j2_sync_warning", j2_sync_interlock.warning},
      {"j2_sync_observation_valid", j2_sync_interlock.observation_valid},
      {"j2_sync_recovery_ready", j2_sync_interlock.recovery_ready},
      {"j2_sync_release_observed", j2_sync_interlock.release_observed},
      {"j2_sync_rearm_pending_next_cycle",
       j2_sync_interlock.rearm_pending_next_cycle},
      {"j2_sync_recovery_valid_pair_frames",
       j2_sync_interlock.recovery_frames},
      {"j2_sync_trip_error_rad", j2_sync_interlock.trip_error_rad},
      {"j2_sync_trip_activation_epoch",
       j2_sync_interlock.trip_activation_epoch},
      {"j2_sync_minimum_rearm_epoch",
       j2_sync_interlock.minimum_rearm_epoch},
      {"j2_sync_warning_threshold_deg", 0.25},
      {"j2_sync_hard_threshold_deg", 0.5},
      {"domain_fault", domain_fault},
      {"lease_safe_hold", lease_safe_hold},
      {"gravity_authority_present",
       control_command.gravity_authority.present},
      {"gravity_policy_identity_status",
       control_command.gravity_authority.present ? "ECHOED" : "NO_AUTHORITY"},
      {"gravity_source_instance_id",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.source_instance_id
           : std::string()},
      {"gravity_session_id",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.session_id : std::string()},
      {"gravity_state_instance_id",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.state_instance_id
           : std::string()},
      {"gravity_model_sha256",
       control_command.gravity_authority.present
           ? std::string(kProductionModelSha256) : std::string()},
      {"gravity_config_sha256",
       control_command.gravity_authority.present
           ? std::string(kGravityConfigSha256) : std::string()},
      {"gravity_continuous_rotor_limits_authoritative",
       control_command.gravity_authority.present &&
           control_command.gravity_authority.official_continuous_authority},
      {"gravity_authority_class",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.authority_class
           : std::string()},
      {"gravity_rating_classification",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.rating_classification
           : std::string()},
      {"gravity_empirical_envelope_id",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.empirical_envelope_id
           : std::string()},
      {"gravity_empirical_envelope_sha256",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.empirical_envelope_sha256
           : std::string()},
      {"gravity_empirical_envelope_expires_at_utc",
       control_command.gravity_authority.present
           ? control_command.gravity_authority
                 .empirical_envelope_expires_at_utc
           : std::string()},
      {"gravity_anchor_sha256",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.anchor_sha256
           : std::string()},
      {"gravity_empirical_stage_index",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.empirical_stage_index : 0U},
      {"gravity_empirical_position_validation_authorized",
       control_command.gravity_authority.present &&
           control_command.gravity_authority
               .empirical_position_validation_authorized},
      {"gravity_scale",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.gravity_scale : 0.0},
      {"gravity_scale_target",
       control_command.gravity_authority.present
           ? control_command.gravity_authority.gravity_scale_target : 0.0},
      {"feedforward_nm", applied_gravity_feedforward_nm},
      {"trajectory_plan_token_id", control_command.quintic.present
           ? control_command.quintic.plan_token_id : ""},
      {"trajectory_sha256", control_command.quintic.present
           ? control_command.quintic.trajectory_sha256 : ""},
      {"trajectory_state", trajectory_state},
      {"trajectory_sample_index", trajectory_sample_index},
      {"trajectory_interval_count", control_command.quintic.present
           ? control_command.quintic.interval_count : 0U},
      {"thermal_fault", thermal_interlock.fault_latched},
      {"thermal_fault_latched", thermal_interlock.fault_latched},
      {"thermal_cooldown_ready", thermal_interlock.cooldown_ready},
      {"thermal_release_observed", thermal_interlock.release_observed},
      {"thermal_rearm_pending",
       thermal_interlock.rearm_pending_next_cycle},
      {"thermal_rearm_pending_next_cycle",
       thermal_interlock.rearm_pending_next_cycle},
      {"thermal_cooldown_valid_brake_frames",
       thermal_interlock.cooldown_frames},
      {"thermal_trip_activation_epoch",
       thermal_interlock.trip_activation_epoch},
      {"thermal_minimum_rearm_epoch",
       thermal_interlock.minimum_rearm_epoch},
      {"thermal_state", thermal_state},
      {"thermal_state_by_motor", thermal_state_by_motor},
      {"thermal_trip_reason", thermal_interlock.trip_reason},
      {"thermal_config_sha256", kThermalConfigSha256},
      {"thermal_derating_factor", thermal_derating_factor},
      {"thermal_raw_temperature_c", maximum_raw_temperature_c},
      {"no_progress_fault", no_progress_watchdog.fault_latched},
      {"load_limit_fault", no_progress_watchdog.fault_latched},
      {"load_limit_no_progress", no_progress_watchdog.fault_latched},
      {"no_progress_release_observed",
       no_progress_watchdog.release_observed},
      {"no_progress_rearm_pending_next_cycle",
       no_progress_watchdog.rearm_pending_next_cycle},
      {"no_progress_watchdog_qualifying_frames",
       no_progress_watchdog.qualifying_frames},
      {"no_progress_observation_valid", no_progress_observation_valid},
      {"no_progress_position_error_rad",
       no_progress_observation_valid ? no_progress_position_error_rad : 0.0},
      {"no_progress_trip_position_error_rad",
       no_progress_watchdog.trip_position_error_rad},
      {"position_safety_trip_reason", no_progress_watchdog.trip_reason},
      {"software_saturation_observed", software_saturation_observed},
      {"load_limit_watchdog_authority", kLoadLimitWatchdogAuthority},
      {"no_progress_trip_activation_epoch",
       no_progress_watchdog.trip_activation_epoch},
      {"no_progress_minimum_rearm_epoch",
       no_progress_watchdog.minimum_rearm_epoch}};
  if (teach_exit_hold.present && !teach_exit_hold.completed && mode == "hold" &&
      control_command.targets == teach_exit_hold.targets_rad) {
    payload["controller_activation_epoch"] = control_command.activation_epoch;
    payload["assisted_teach_exit_hold"] = {
        {"schema", "go-m8010-teach-exit-hold/1.0"},
        {"joint_index", teach_exit_hold.joint_index},
        {"press_activation_epoch", teach_exit_hold.press_activation_epoch},
        {"started_monotonic_ns", teach_exit_hold.started_monotonic_ns},
        {"deadline_monotonic_ns", teach_exit_hold.deadline_monotonic_ns},
        {"reason", teach_exit_hold.reason},
        {"targets_rad", teach_exit_hold.targets_rad},
        {"initial_velocity_rad_s", teach_exit_hold.initial_velocity_rad_s}};
    if (teach_exit_hold.restricted)
      payload["assisted_teach_exit_hold"]["restricted"] = true;
  }
  return payload.dump();
}

void hand_guidance_self_test(const nlohmann::json& legacy_packet) {
  for (const double excursion : {10.0, 20.0}) for (const std::string bus : {"j1", "j2", "j345"}) {
    const auto now = Clock::now();
    const auto ns = monotonic_ns_at(now);
    auto packet = legacy_packet;
    packet["schema"] = "go-m8010-gui-command/1.5";
    packet["source_monotonic_ns"] = ns - 90000000ULL;
    packet["moving_joint_mask"] = {true, true, true, true, true, true};
    packet["maximum_velocity_rad_s"] = 30.0 * kPi / 180.0;
    packet["hand_guidance"] = {{"schema", "go-m8010-hand-guidance-reference/1.0"},
        {"origin_rad", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
        {"velocity_rad_s", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
        {"maximum_velocity_deg_s", 30.0}, {"maximum_excursion_deg", excursion}, {"maximum_reference_error_deg", 2.0},
        {"freeze_reference", false}};
    auto& authority = packet["gravity_authority"];
    authority["source_monotonic_ns"] = ns;
    authority["empirical_envelope_deadline_monotonic_ns"] = ns + 1000000000000ULL;
    authority["empirical_allowed_teach_joints"] = {"J1", "J2", "J3", "J4", "J5", "J6"};
    authority["empirical_maximum_teach_excursion_deg"] = excursion;
    authority["empirical_maximum_teach_seconds"] = 600.0;
    authority["empirical_maximum_teach_velocity_deg_s"] = 30.0;
    auto motors = make_motors(bus);
    for (auto& motor : motors) {
      motor.reference_ready = motor.valid = motor.last_frame_valid = motor.speed_ready = true;
      motor.returned_mode = kFocMode; motor.merror = 0; motor.temperature = 30;
      motor.previous_feedback_at = now - std::chrono::milliseconds(1);
      motor.integral_encoder_velocity = 0.0;
    }
    auto hold_packet = packet;
    hold_packet["schema"] = "go-m8010-gui-command/1.2"; hold_packet["mode"] = "hold";
    hold_packet["activation_epoch"] = 1U; hold_packet["source_monotonic_ns"] = ns - 100000000ULL;
    hold_packet["moving_joint_mask"] = {false, false, false, false, false, false};
    hold_packet.erase("hand_guidance");
    hold_packet["gravity_authority"]["empirical_stage_index"] = 0U;
    hold_packet["gravity_authority"]["empirical_position_validation_authorized"] = false;
    hold_packet["gravity_authority"]["gravity_scale"] = 0.0;
    hold_packet["gravity_authority"]["gravity_scale_target"] = 0.0;
    GuiCommand command;
    parse_command(hold_packet.dump(), command, now);
    CommandSafetyState safety;
    CommandReceiveState receive_state;
    validate_command_for_owned_domain(command, motors);
    validate_and_observe_gravity_policy(command, motors, safety);
    for (std::uint64_t stage = 1U; stage < 5U; ++stage) {
      command.gravity_authority.empirical_stage_index = stage;
      command.gravity_authority.gravity_scale = static_cast<double>(stage) * 0.25;
      command.gravity_authority.gravity_scale_target = static_cast<double>(stage) * 0.25;
      validate_and_observe_gravity_policy(command, motors, safety);
    }
    command.gravity_authority.empirical_position_validation_authorized = true;
    validate_and_observe_gravity_policy(command, motors, safety);
    validate_and_observe_fixed_hold_targets(command, motors, safety);
    observe_valid_command(command, motors, safety);
    observe_command_source(command, receive_state);
    struct Pair { int fd[2]{-1, -1}; ~Pair() { for (int item : fd) if (item >= 0) ::close(item); } } sockets;
    if (::socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets.fd) != 0 ||
        ::fcntl(sockets.fd[0], F_SETFL, O_NONBLOCK) != 0)
      throw std::runtime_error("GUIDANCE_TEST_SOCKET_FAILED");
    const std::array<bool, 6> no_flags{};
    const std::array<std::uint64_t, 6> no_epochs{};
    const std::array<double, 6> zero{};
    auto send = [&](const nlohmann::json& value) {
      const auto data = value.dump();
      if (::send(sockets.fd[1], data.data(), data.size(), 0) != static_cast<ssize_t>(data.size()))
        throw std::runtime_error("GUIDANCE_TEST_SEND_FAILED");
      return receive_latest(sockets.fd[0], command, safety, motors, no_flags, no_flags, no_flags,
                            no_epochs, zero, receive_state);
    };
    const auto initial_command = command;
    const auto initial_safety = safety;
    const auto initial_receive = receive_state;
    for (bool return_only : {false, true}) for (bool admitted : {false, true}) {
      command = initial_command;
      command.received_at = Clock::now() - std::chrono::milliseconds(450);
      command.source_monotonic_ns = monotonic_ns_at(command.received_at);
      safety = initial_safety;
      safety.last_accepted_command = command;
      receive_state = initial_receive;
      receive_state.accepted_sources[command.source_instance_id].last_source_monotonic_ns = command.source_monotonic_ns;
      const auto old_hold = command;
      for (auto& motor : motors) {
        motor.unwrapped = motor.reference + motor.sign * kGear * (admitted ? 0.0 : 2.1 * kPi / 180.0);
        motor.previous_feedback_at = Clock::now();
      }
      auto attempted = packet;
      attempted["sequence"] = 2U;
      send(attempted);
      if (safety.guidance_active != admitted || safety.highest_rejected_active_epoch != 0U)
        throw std::runtime_error("GUIDANCE_MIXED_ADMISSION_SETUP_FAILED");
      auto abort = packet;
      abort["mode"] = "hold";
      abort["activation_epoch"] = 3U;
      abort["sequence"] = 3U;
      abort["source_monotonic_ns"] = monotonic_ns_at(Clock::now());
      abort["moving_joint_mask"] = {false, false, false, false, false, false};
      if (return_only) abort["gravity_authority"]["empirical_assisted_teach_authorized"] = false;
      if (!admitted) {
        auto changed = abort;
        changed["targets_rad"][motors.front().joint_index] = 0.001;
        send(changed);
        changed = abort;
        changed["hand_guidance"]["origin_rad"][motors.front().joint_index] = 0.01;
        send(changed);
        if (command.activation_epoch != old_hold.activation_epoch || command.received_at != old_hold.received_at ||
            safety.highest_rejected_active_epoch != 0U)
          throw std::runtime_error("GUIDANCE_INITIAL_ABORT_CHANGED_OWNED_HOLD");
        auto unhealthy = motors;
        unhealthy.front().merror = 1;
        GuiCommand abort_command;
        parse_command(abort.dump(), abort_command);
        auto unhealthy_safety = safety;
        bool rejected_unhealthy = false;
        try { validate_and_observe_hand_guidance(abort_command, unhealthy, unhealthy_safety); }
        catch (const GuidanceReferenceRejected&) {}
        catch (const std::runtime_error&) { rejected_unhealthy = true; }
        if (!rejected_unhealthy)
          throw std::runtime_error("GUIDANCE_INITIAL_ABORT_ACCEPTED_UNHEALTHY_FEEDBACK");
      }
      send(abort);
      if (command.mode != "hold" || command.activation_epoch != 3U || safety.guidance_active ||
          !safety.guidance_bound || command.targets != old_hold.targets ||
          safety.guidance_origin_bound != (admitted || !return_only) ||
          (!admitted && safety.guidance_started_ns != 0U))
        throw std::runtime_error("GUIDANCE_MIXED_ADMISSION_ABORT_HOLD_FAILED");
      abort["sequence"] = 4U;
      abort["source_monotonic_ns"] = monotonic_ns_at(Clock::now());
      send(abort);
      if (command.source_sequence != 4U || command.activation_epoch != 3U ||
          safety.guidance_origin_bound != (admitted || !return_only))
        throw std::runtime_error("GUIDANCE_MIXED_ADMISSION_ABORT_HOLD_RENEWAL_FAILED");
      const auto after_old_lease = command.received_at + std::chrono::milliseconds(100);
      auto fresh_safety = safety, expired_safety = initial_safety;
      observe_interarrival_lease(command, after_old_lease, motors, fresh_safety);
      observe_interarrival_lease(old_hold, after_old_lease, motors, expired_safety);
      if (fresh_safety.minimum_activation_epoch != safety.minimum_activation_epoch ||
          expired_safety.minimum_activation_epoch <= old_hold.activation_epoch)
        throw std::runtime_error("GUIDANCE_ABORT_DID_NOT_RENEW_VALID_HOLD_LEASE");
    }
    command = initial_command;
    safety = initial_safety;
    receive_state = initial_receive;
    for (auto& motor : motors) {
      motor.unwrapped = motor.reference;
      motor.previous_feedback_at = Clock::now();
    }
    packet["sequence"] = 2U;
    send(packet);
    if (!command.hand_guidance.present || !safety.guidance_active || command.activation_epoch != 2U)
      throw std::runtime_error("GUIDANCE_ENTRY_SELF_TEST_FAILED");
    // Both endpoints are admissible while the independent two-degree
    // tracking guard and the signed session excursion remain enforced.
    for (double direction : {-1.0, 1.0}) for (int trial = 0; trial < 3; ++trial) {
      auto candidate = command;
      auto checked_safety = safety;
      auto feedback = motors;
      const double target = direction * (excursion + (trial == 1 ? 0.01 : 0.0)) * kPi / 180.0;
      candidate.source_monotonic_ns += 20000000ULL;
      for (auto& motor : feedback) {
        const auto joint = static_cast<std::size_t>(motor.joint_index);
        candidate.targets[joint] = target;
        checked_safety.guidance_reference.targets[joint] = target - direction * 0.1 * kPi / 180.0;
        motor.unwrapped = motor.reference + motor.sign * kGear *
            (target - (trial == 2 ? direction * 2.01 * kPi / 180.0 : 0.0));
      }
      bool rejected = false;
      try { validate_and_observe_hand_guidance(candidate, feedback, checked_safety); }
      catch (const GuidanceReferenceRejected&) { rejected = true; }
      if (rejected != (trial != 0)) throw std::runtime_error("GUIDANCE_EXCURSION_VERSUS_TRACKING_BOUND_FAILED");
    }
    for (const auto& motor : motors)
      if (!freezes_assisted_teach_integral(command, "teach", motor.joint_index, safety, ns))
        throw std::runtime_error("GUIDANCE_ACTIVE_INTEGRAL_NOT_FROZEN");
    packet["sequence"] = 3U; packet["source_monotonic_ns"] = ns - 70000000ULL;
    packet["targets_rad"] = std::vector<double>(6, 0.001);
    packet["hand_guidance"]["velocity_rad_s"] = std::vector<double>(6, 0.02);
    send(packet);
    if (command.source_sequence != 3U) throw std::runtime_error("GUIDANCE_UPDATE_SELF_TEST_FAILED");
    for (int bad_case = 0; bad_case < 4; ++bad_case) {
      auto bad = packet;
      bad["sequence"] = 100U + bad_case;
      bad["source_monotonic_ns"] = ns - 60000000ULL + static_cast<std::uint64_t>(bad_case);
      const auto joint = static_cast<std::size_t>(motors.front().joint_index);
      if (bad_case == 0) bad["targets_rad"][joint] = 11.0 * kPi / 180.0;
      if (bad_case == 1) bad["targets_rad"][joint] = 3.0 * kPi / 180.0;
      if (bad_case == 2) bad["hand_guidance"]["velocity_rad_s"][joint] = 30.01 * kPi / 180.0;
      if (bad_case == 3) { bad["source_monotonic_ns"] = ns - 70000000ULL + 1000U; bad["targets_rad"][joint] = 0.002; }
      const auto receipt = command.received_at;
      send(bad);
      if (command.source_sequence != 3U || command.received_at != receipt ||
          command.targets != safety.guidance_reference.targets || safety.highest_rejected_active_epoch != 0U ||
          receive_state.accepted_sources[command.source_instance_id].last_sequence != 3U)
        throw std::runtime_error("GUIDANCE_SOFT_REJECT_CHANGED_GOAL_LEASE_OR_EPOCH");
    }
    packet["sequence"] = 4U; packet["source_monotonic_ns"] = ns - 20000000ULL;
    packet["targets_rad"] = std::vector<double>(6, 0.0015);
    send(packet);
    if (command.source_sequence != 4U) throw std::runtime_error("GUIDANCE_RETRY_SELF_TEST_FAILED");
    auto q = zero, dq = zero;
    apply_hand_guidance_reference(command, motors, q, dq);
    for (const auto& motor : motors) {
      const auto joint = static_cast<std::size_t>(motor.joint_index);
      if (q[joint] != 0.0015 || dq[joint] != 0.02)
        throw std::runtime_error("GUIDANCE_REFERENCE_WAS_RECAPTURED");
    }
    auto displaced = motors;
    for (auto& motor : displaced)
      motor.unwrapped = motor.reference + motor.sign * kGear * (0.0015 + 2.5 * kPi / 180.0);
    auto keep_goal = command;
    keep_goal.hand_guidance.velocity_rad_s.fill(0.0);
    auto keep_safety = safety;
    validate_and_observe_hand_guidance(keep_goal, displaced, keep_safety);
    bool moving_refresh_rejected = false;
    try {
      auto moving_refresh = command;
      auto moving_safety = safety;
      validate_and_observe_hand_guidance(moving_refresh, displaced, moving_safety);
    } catch (const GuidanceReferenceRejected&) { moving_refresh_rejected = true; }
    if (keep_goal.targets != command.targets || !moving_refresh_rejected)
      throw std::runtime_error("GUIDANCE_FROZEN_ONLY_LEAD_EXCEPTION_FAILED");
    auto deadline_safety = safety;
    auto deadline_candidate = command;
    deadline_candidate.received_at += std::chrono::seconds(600);
    deadline_candidate.targets.fill(0.02);
    validate_and_observe_hand_guidance(deadline_candidate, motors, deadline_safety);
    if (!deadline_safety.guidance_paused || deadline_safety.guidance_paused_reason != "DEADMAN_TIMEOUT" ||
        deadline_candidate.targets != command.targets)
      throw std::runtime_error("GUIDANCE_RECEIVE_DEADLINE_CHANGED_GOAL");
    auto runtime_command = command;
    auto runtime_safety = safety;
    motors.front().integral_encoder_velocity = 30.01 * kPi / 180.0;
    if (hand_guidance_runtime_blocker(runtime_command, runtime_safety, motors,
            bus == "j2" ? motors.front().integral_encoder_velocity : 0.0, ns) != "HAND_GUIDANCE_ACTUAL_VELOCITY_LIMIT")
      throw std::runtime_error("GUIDANCE_ACTUAL_SPEED_SELF_TEST_FAILED");
    motors.front().integral_encoder_velocity = 0.0;
    if (!hand_guidance_runtime_blocker(runtime_command, runtime_safety, motors, 0.0,
            safety.guidance_started_ns + 600000000000ULL).empty() || runtime_command.mode != "hold" ||
        runtime_command.targets != command.targets || !runtime_safety.guidance_paused || safety.teach_exit_hold.present)
      throw std::runtime_error("GUIDANCE_TIMEOUT_MUST_RETAIN_HOLD");
    // Release uses no feedback-derived target: freeze the native accepted goal,
    // then ignore an already queued moving reference until exact owned ACK.
    const auto press_started = safety.guidance_started_ns;
    packet["sequence"] = 5U; packet["source_monotonic_ns"] = ns - 15000000ULL;
    packet["targets_rad"] = std::vector<double>(6, -0.08);
    packet["hand_guidance"]["freeze_reference"] = true;
    packet["hand_guidance"]["velocity_rad_s"] = std::vector<double>(6, 0.0);
    send(packet);
    if (command.source_sequence != 5U || !safety.guidance_paused ||
        safety.guidance_paused_reason != "GUI_RELEASE" || safety.guidance_started_ns != press_started)
      throw std::runtime_error("GUIDANCE_RELEASE_FREEZE_SELF_TEST_FAILED");
    for (const auto& motor : motors)
      if (command.targets[static_cast<std::size_t>(motor.joint_index)] != 0.0015)
        throw std::runtime_error("GUIDANCE_RELEASE_USED_STALE_TARGET");
    packet["sequence"] = 6U; packet["source_monotonic_ns"] = ns - 10000000ULL;
    packet["hand_guidance"]["freeze_reference"] = false;
    packet["hand_guidance"]["velocity_rad_s"] = std::vector<double>(6, 0.02);
    packet["targets_rad"] = std::vector<double>(6, 0.08);
    send(packet);
    for (const auto& motor : motors)
      if (command.source_sequence != 6U || command.targets[static_cast<std::size_t>(motor.joint_index)] != 0.0015 ||
          command.hand_guidance.velocity_rad_s[static_cast<std::size_t>(motor.joint_index)] != 0.0)
        throw std::runtime_error("GUIDANCE_PAUSED_HEARTBEAT_CHANGED_TARGET");
    runtime_command = command;
    if (!hand_guidance_runtime_blocker(runtime_command, safety, motors, 0.0, ns).empty() ||
        runtime_command.mode != "hold" || safety.guidance_started_ns != press_started)
      throw std::runtime_error("GUIDANCE_RELEASE_DID_NOT_RETAIN_HOLD");
    for (const auto& motor : motors)
      if (!freezes_assisted_teach_integral(runtime_command, "hold", motor.joint_index, safety, ns))
        throw std::runtime_error("GUIDANCE_UNACKED_FREEZE_INTEGRAL_NOT_FROZEN");
    const auto raw = nlohmann::json::parse(feedback_payload(motors, ns, "hold", J2SyncFaultFilter{}, false,
        false, runtime_command, zero, "INACTIVE", 0, ThermalInterlockState{}, NoProgressWatchdogState{},
        false, 0.0, false, 1.0, AssistedTeachExitHold{}, &safety.last_accepted_command, true, safety.guidance_paused_reason));
    for (const auto& sample : raw.at("samples"))
      if (sample.at("accepted_guidance_target_rad").get<double>() != 0.0015 ||
          sample.at("accepted_guidance_activation_epoch").get<std::uint64_t>() != 2U ||
          sample.at("guidance_paused_reason") != "GUI_RELEASE")
        throw std::runtime_error("GUIDANCE_RAW_ACK_SELF_TEST_FAILED");
    packet["mode"] = "hold"; packet["activation_epoch"] = 3U; packet["sequence"] = 7U;
    packet["source_monotonic_ns"] = ns - 1000000ULL;
    packet["moving_joint_mask"] = {false, false, false, false, false, false};
    packet["hand_guidance"]["velocity_rad_s"] = std::vector<double>(6, 0.0);
    packet["targets_rad"] = std::vector<double>(6, 0.003);  // Other domains may ACK different last samples.
    for (const auto& motor : motors) packet["targets_rad"][motor.joint_index] = 0.0015;
    send(packet);
    if (command.mode != "hold" || command.activation_epoch != 3U || safety.guidance_active || safety.guidance_paused)
      throw std::runtime_error("GUIDANCE_OWNED_HOLD_ACK_SELF_TEST_FAILED");
    for (const auto& motor : motors) {
      const auto joint = static_cast<std::size_t>(motor.joint_index);
      if (freezes_assisted_teach_integral(command, "hold", joint, safety, ns) ||
          !uses_assisted_teach_damping(command, "hold", joint, safety.teach_exit_hold, ns))
        throw std::runtime_error("GUIDANCE_ACK_MUST_RESTORE_INTEGRAL_KEEP_ENCODER_DAMPING");
      BoundedHoldIntegralState integral;
      for (int frame = 0; frame < 100; ++frame)
        (void)update_bounded_hold_integral(integral, true,
            !freezes_assisted_teach_integral(command, "hold", joint, safety, ns),
            1.1 * kPi / 180.0, 0.0, kHoldIntegralRotorHardNm[joint],
            joint == 1U ? kJ2IntegralKiPerRotorRadS : kAuxIntegralKiPerRotorRadS,
            joint == 1U ? kJ2IntegralRateHardNmS : kAuxIntegralRateHardNmS,
            joint == 1U ? kJ2IntegralEnterError : kAuxIntegralEnterError,
            joint == 1U ? kJ2IntegralEnterVelocity : kAuxIntegralEnterVelocity,
            joint == 1U ? kJ2IntegralDeadband : kAuxIntegralDeadband,
            joint == 1U ? kJ2IntegralDwellFrames : kAuxIntegralDwellFrames);
      if (!(integral.accumulator_nm > 0.0 && integral.accumulator_nm <= kHoldIntegralRotorHardNm[joint]))
        throw std::runtime_error("GUIDANCE_ACK_BOUNDED_INTEGRAL_DID_NOT_LEARN");
    }
    // An already-held post-drag pose may be >2 degrees from the session origin.
    // A rejected new press must leave that HOLD renewable at its accepted epoch.
    auto stamp_packet = [](nlohmann::json& value) {
      const auto stamp = monotonic_ns_at(Clock::now());
      value["source_monotonic_ns"] = stamp;
      value["gravity_authority"]["source_monotonic_ns"] = stamp;
    };
    auto moving = packet;
    moving["mode"] = "teach";
    moving["activation_epoch"] = 4U;
    moving["sequence"] = 8U;
    moving["moving_joint_mask"] = {true, true, true, true, true, true};
    stamp_packet(moving);
    send(moving);
    if (command.mode != "teach" || command.activation_epoch != 4U)
      throw std::runtime_error("GUIDANCE_REENTRY_MOVEMENT_START_FAILED");
    const auto moving_start = command.targets;
    for (int step = 1; step <= 6; ++step) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      moving["sequence"] = 8U + step;
      for (std::size_t joint = 0; joint < 6U; ++joint)
        moving["targets_rad"][joint] = moving_start[joint] +
            (3.0 * kPi / 180.0 - moving_start[joint]) * static_cast<double>(step) / 6.0;
      for (auto& motor : motors) {
        const auto joint = static_cast<std::size_t>(motor.joint_index);
        motor.unwrapped = motor.reference + motor.sign * kGear * moving["targets_rad"][joint].get<double>();
        motor.previous_feedback_at = Clock::now();
      }
      stamp_packet(moving);
      send(moving);
      if (command.source_sequence != static_cast<std::uint64_t>(8 + step))
        throw std::runtime_error("GUIDANCE_REENTRY_BOUNDED_MOVEMENT_FAILED");
    }
    moving["sequence"] = 15U;
    moving["hand_guidance"]["freeze_reference"] = true;
    stamp_packet(moving);
    send(moving);
    auto held = moving;
    held["mode"] = "hold";
    held["moving_joint_mask"] = {false, false, false, false, false, false};
    held["activation_epoch"] = 5U;
    held["sequence"] = 16U;
    stamp_packet(held);
    send(held);
    if (command.mode != "hold" || command.activation_epoch != 5U || safety.guidance_active)
      throw std::runtime_error("GUIDANCE_REENTRY_HELD_POSE_SETUP_FAILED");
    auto reentry = held;
    reentry["schema"] = "go-m8010-gui-command/1.5";
    reentry["mode"] = "teach";
    reentry["activation_epoch"] = 6U;
    reentry["moving_joint_mask"] = {true, true, true, true, true, true};
    reentry["hand_guidance"]["freeze_reference"] = false;
    const auto held_receipt = command.received_at;
    for (int bad_case = 0; bad_case < 2; ++bad_case) {
      auto bad = reentry;
      bad["sequence"] = 90U + bad_case;
      stamp_packet(bad);
      if (bad_case == 0) bad["targets_rad"][motors.front().joint_index] = 0.06;
      else bad["hand_guidance"]["origin_rad"][motors.front().joint_index] = 0.01;
      send(bad);
      if (command.mode != "hold" || command.activation_epoch != 5U ||
          command.received_at != held_receipt || safety.highest_rejected_active_epoch != 0U ||
          receive_state.accepted_sources[command.source_instance_id].last_sequence != 16U)
        throw std::runtime_error("GUIDANCE_REJECTED_ENTRY_POISONED_HELD_EPOCH");
    }
    held["sequence"] = 17U;
    stamp_packet(held);
    send(held);
    if (command.source_sequence != 17U || command.mode != "hold" || command.received_at <= held_receipt)
      throw std::runtime_error("GUIDANCE_REJECTED_ENTRY_BLOCKED_HOLD_RENEWAL");
    reentry["sequence"] = 18U;
    stamp_packet(reentry);
    send(reentry);
    if (command.mode != "teach" || command.activation_epoch != 6U || !safety.guidance_active ||
        command.hand_guidance.origin_rad != zero)
      throw std::runtime_error("GUIDANCE_REENTRY_FROM_OFFSET_HOLD_FAILED");
    auto restricted = reentry;
    restricted["gravity_authority"]["empirical_assisted_teach_authorized"] = false;
    restricted["gravity_authority"]["empirical_envelope_deadline_monotonic_ns"] =
        command.gravity_authority.empirical_envelope_deadline_monotonic_ns - 1U;
    restricted["sequence"] = 19U;
    stamp_packet(restricted);
    const auto prior_receipt = command.received_at;
    send(restricted);  // A now-ineligible moving heartbeat grants no lease.
    if (command.source_sequence != 18U || command.received_at != prior_receipt ||
        safety.highest_rejected_active_epoch != 0U)
      throw std::runtime_error("GUIDANCE_RETURN_ONLY_MOVING_HEARTBEAT_CHANGED_STATE");
    restricted["sequence"] = 20U;
    restricted["hand_guidance"]["freeze_reference"] = true;
    stamp_packet(restricted);
    send(restricted);
    if (!safety.guidance_paused || !hand_guidance_return_only(command))
      throw std::runtime_error("GUIDANCE_RETURN_ONLY_FREEZE_REJECTED");
    restricted["mode"] = "hold";
    restricted["activation_epoch"] = 7U;
    restricted["sequence"] = 21U;
    restricted["moving_joint_mask"] = {false, false, false, false, false, false};
    stamp_packet(restricted);
    send(restricted);
    if (command.mode != "hold" || safety.guidance_active || command.activation_epoch != 7U)
      throw std::runtime_error("GUIDANCE_RETURN_ONLY_HOLD_ACK_REJECTED");
    auto extended_deadline = command;
    ++extended_deadline.gravity_authority.empirical_envelope_deadline_monotonic_ns;
    bool deadline_extension_rejected = false;
    try { validate_guidance_return_only(extended_deadline, motors, safety); }
    catch (const std::runtime_error&) { deadline_extension_rejected = true; }
    if (!deadline_extension_rejected)
      throw std::runtime_error("GUIDANCE_RETURN_ONLY_DEADLINE_EXTENSION_ACCEPTED");
    auto returning = restricted;
    returning.erase("hand_guidance");
    returning["schema"] = "go-m8010-gui-command/1.3";
    returning["mode"] = "position";
    returning["activation_epoch"] = 8U;
    returning["sequence"] = 22U;
    const auto moving_joint = static_cast<std::size_t>(motors.front().joint_index);
    returning["moving_joint_mask"][moving_joint] = true;
    returning["targets_rad"][moving_joint] = 0.0;
    returning["plan_token_id"] = std::string(64U, 'c');
    returning["trajectory"] = {{"schema", "go-m8010-quintic-command/1.0"},
        {"trajectory_sha256", std::string(64U, 'd')}, {"profile", "quintic-rest-to-rest-v1"},
        {"start_rad", restricted["targets_rad"]}, {"target_rad", returning["targets_rad"]},
        {"duration_ns", 2000000000ULL}, {"interval_count", 200U},
        {"execute_at_monotonic_ns", monotonic_ns_at(Clock::now()) + 1000000000ULL},
        {"segment_index", 0}, {"segment_count", 1}};
    stamp_packet(returning);
    GuiCommand parsed_return;
    parse_command(returning.dump(), parsed_return);
    bool unbound_rejected = false;
    try { validate_guidance_return_only(parsed_return, motors, initial_safety); }
    catch (const GuidanceReferenceRejected&) { unbound_rejected = true; }
    if (!unbound_rejected) throw std::runtime_error("GUIDANCE_RETURN_ONLY_UNBOUND_MOTION_ACCEPTED");
    for (double rejected_target_deg : {4.0, -1.0}) {
      auto wrong_direction = returning;
      wrong_direction["targets_rad"][moving_joint] = rejected_target_deg * kPi / 180.0;
      wrong_direction["trajectory"]["target_rad"] = wrong_direction["targets_rad"];
      stamp_packet(wrong_direction);
      const auto receipt = command.received_at;
      send(wrong_direction);
      if (command.mode != "hold" || command.activation_epoch != 7U || command.received_at != receipt ||
          safety.highest_rejected_active_epoch != 0U)
        throw std::runtime_error("GUIDANCE_RETURN_ONLY_WRONG_DIRECTION_CHANGED_HOLD");
    }
    stamp_packet(returning);
    send(returning);
    if (command.mode != "position" || command.activation_epoch != 8U ||
        command.targets[moving_joint] != 0.0 || safety.guidance_press.hand_guidance.origin_rad != zero)
      throw std::runtime_error("GUIDANCE_RETURN_ONLY_SIGNED_RETURN_REJECTED");
    packet = reentry;
    packet["mode"] = "brake"; packet["active_joint_mask"] = {false, false, false, false, false, false};
    packet["moving_joint_mask"] = {false, false, false, false, false, false};
    packet["hand_guidance"] = "malformed-but-brake-must-exit";
    send(packet);
    if (command.mode != "brake" || safety.guidance_bound)
      throw std::runtime_error("GUIDANCE_BRAKE_EXIT_SELF_TEST_FAILED");
  }
}

bool send_terminal_brake(SerialPort& serial, std::vector<MotorRuntime>& motors,
                         const std::string& bus, TxAudit& audit) noexcept {
  try {
    std::vector<std::deque<double>> position_windows(motors.size());
    for (int frame = 0; frame < 20; ++frame) {
      bool frame_safe = true;
      std::vector<double> frame_positions(motors.size(), 0.0);
      for (std::size_t index = 0; index < motors.size(); ++index) {
        auto& motor = motors[index];
        motor.last_tau_cmd_rotor_nm = 0.0;
        MotorCmd brake = make_command(motor.id, kBrakeMode, 0.0, 0.0, 0.0, 0.0);
        const Feedback feedback = transact(
            serial, brake, motor.id, kBrakeMode, audit);
        bool sample_safe = feedback.continuity_valid;
        if (feedback.continuity_valid) {
          motor.last_q = feedback.data.q;
          motor.last_dq = feedback.data.dq;
          motor.last_tau = feedback.data.tau;
          motor.last_valid_feedback_monotonic_ns = monotonic_ns();
          motor.temperature = feedback.data.temp;
          motor.merror = feedback.data.merror;
          motor.returned_mode = feedback.data.mode;
          motor.unwrapped = motor.unwrap.update(motor.last_q);
          frame_positions[index] = motor.unwrapped;
          const auto feedback_at = Clock::now();
          bool velocity_valid = false;
          bool velocity_tripped = false;
          const double scaled_position = motor.sign * motor.unwrapped / kGear;
          if (motor.speed_ready) {
            const double dt = std::chrono::duration<double>(
                feedback_at - motor.previous_feedback_at).count();
            if (dt > 1e-4 && dt < 0.5) {
              velocity_valid = true;
              velocity_tripped = apply_velocity_guards(
                  bus, motor,
                  (scaled_position - motor.previous_scaled_position) / dt);
            }
          }
          motor.previous_scaled_position = scaled_position;
          motor.previous_feedback_at = feedback_at;
          motor.speed_ready = true;
          const bool torque_safe =
              bus != "j1" || std::abs(motor.last_tau) < 5.0;
          sample_safe = velocity_valid && !velocity_tripped && torque_safe;
        }
        frame_safe = frame_safe && sample_safe;
      }
      if (frame_safe) {
        for (std::size_t index = 0; index < motors.size(); ++index) {
          auto& window = position_windows[index];
          window.push_back(frame_positions[index]);
          while (window.size() > 5U) window.pop_front();
        }
      } else {
        for (auto& window : position_windows) window.clear();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return std::all_of(position_windows.begin(), position_windows.end(),
                       [](const std::deque<double>& window) {
      if (window.size() != 5U) return false;
      const auto minmax = std::minmax_element(window.begin(), window.end());
      return *minmax.second - *minmax.first <=
             kGear * kBrakeStationaritySpan + 1e-12;
    });
  } catch (...) {
    return false;
  }
}

class TerminalBrakeGuard {
 public:
  TerminalBrakeGuard(std::unique_ptr<SerialPort>& serial,
                     std::vector<MotorRuntime>& motors,
                     const std::string& bus, TxAudit& audit)
      : serial_(serial), motors_(motors), bus_(bus), audit_(audit) {}
  ~TerminalBrakeGuard() {
    if (!armed_) return;
    const bool confirmed = serial_ &&
        send_terminal_brake(*serial_, motors_, bus_, audit_);
    const char* report = confirmed
        ? "TERMINAL_PATH=EXCEPTION\nFINAL_MODE=BRAKE\nFINAL_BRAKE=PASS\n"
        : "TERMINAL_PATH=EXCEPTION\nFINAL_MODE=UNCONFIRMED\nFINAL_BRAKE=FAIL\n";
    std::size_t remaining = std::strlen(report);
    while (remaining > 0U) {
      const ssize_t written = ::write(STDERR_FILENO, report, remaining);
      if (written > 0) {
        report += static_cast<std::size_t>(written);
        remaining -= static_cast<std::size_t>(written);
      } else if (written < 0 && errno == EINTR) {
        continue;
      } else {
        break;
      }
    }
  }
  bool brake_and_disarm() noexcept {
    const bool confirmed =
        !armed_ || (serial_ &&
            send_terminal_brake(*serial_, motors_, bus_, audit_));
    armed_ = false;
    return confirmed;
  }
 private:
  std::unique_ptr<SerialPort>& serial_;
  std::vector<MotorRuntime>& motors_;
  const std::string& bus_;
  TxAudit& audit_;
  bool armed_ = true;
};

bool recover_go_transport_in_brake(
    const BusDefinition& definition, std::unique_ptr<SerialPort>& serial,
    std::vector<MotorRuntime>& motors, const std::string& bus,
    double maximum_phase_delta_rad, TxAudit& audit,
    std::uint64_t& recovery_attempt_count,
    std::uint64_t& recovery_success_count) {
  ++recovery_attempt_count;
  const std::uint64_t episode = recovery_attempt_count;
  std::cerr << "COMMUNICATION_RECOVERY_BEGIN"
            << " bus=" << bus
            << " episode=" << episode
            << " policy=BRAKE_ONLY_RECOMMISSION"
            << std::endl;

  if (serial) (void)send_terminal_brake(*serial, motors, bus, audit);
  serial.reset();
  std::vector<std::deque<double>> stable_windows(motors.size());
  std::chrono::milliseconds reopen_delay(100);
  std::uint64_t reopen_attempt = 0U;
  int invalid_transport_cycles = 0;

  while (!g_stop.load()) {
    if (!serial) {
      ++reopen_attempt;
      try {
        serial = std::make_unique<SerialPort>(
            definition.port, 16, 4000000, 20000, BlockYN::NO,
            bytesize_t::eightbits, parity_t::parity_none,
            stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none);
        std::cerr << "COMMUNICATION_TRANSPORT_REOPENED"
                  << " bus=" << bus
                  << " episode=" << episode
                  << " reopen_attempt=" << reopen_attempt
                  << std::endl;
      } catch (...) {
        std::cerr << "COMMUNICATION_TRANSPORT_WAIT"
                  << " bus=" << bus
                  << " episode=" << episode
                  << " reopen_attempt=" << reopen_attempt
                  << " retry_ms=" << reopen_delay.count()
                  << std::endl;
        std::this_thread::sleep_for(reopen_delay);
        reopen_delay = std::min(
            reopen_delay * 2, std::chrono::milliseconds(2000));
        continue;
      }
    }

    bool cycle_safe = true;
    bool transport_valid = true;
    for (std::size_t index = 0; index < motors.size(); ++index) {
      auto& motor = motors[index];
      motor.last_tau_cmd_rotor_nm = 0.0;
      MotorCmd brake = make_command(
          motor.id, kBrakeMode, 0.0, 0.0, 0.0, 0.0);
      const Feedback feedback = transact(
          *serial, brake, motor.id, kBrakeMode, audit);
      if (!feedback.continuity_valid) {
        cycle_safe = false;
        transport_valid = false;
        stable_windows[index].clear();
        continue;
      }
      motor.last_q = feedback.data.q;
      motor.last_dq = feedback.data.dq;
      motor.last_tau = feedback.data.tau;
      motor.temperature = feedback.data.temp;
      motor.merror = feedback.data.merror;
      motor.returned_mode = feedback.data.mode;
      motor.last_valid_feedback_monotonic_ns = monotonic_ns();
      motor.unwrapped = motor.unwrap.update(motor.last_q);
      const double logical =
          motor.sign * (motor.unwrapped - motor.reference) / kGear;
      const double phase_reference = motor.session_reference_configured
          ? motor.session_capture_raw_position : motor.reference;
      const double phase_delta = std::abs(std::remainder(
          motor.unwrapped - phase_reference, 2.0 * kPi));
      const bool sample_safe = feedback.data.merror == 0 &&
          feedback.data.temp >= 0 &&
          feedback.data.temp < g_thermal_policy.thermal_stop_c &&
          // Raw BRAKE-only acquisition has no active-session pose authority.
          // Preserve its diagnostic reference; never select a new zero here.
          (audit.brake_only ||
           (std::isfinite(logical) &&
            within_mechanical_feedback_envelope(motor.joint_index, logical) &&
            phase_delta <= maximum_phase_delta_rad + 1e-12));
      if (!sample_safe) {
        cycle_safe = false;
        stable_windows[index].clear();
        continue;
      }
      auto& window = stable_windows[index];
      window.push_back(motor.unwrapped);
      while (window.size() > 5U) window.pop_front();
    }

    if (cycle_safe && !audit.brake_only && bus == "j2" && motors.size() == 2U) {
      const double q_a =
          -1.0 * (motors[0].unwrapped - motors[0].reference) / kGear;
      const double q_b =
          +1.0 * (motors[1].unwrapped - motors[1].reference) / kGear;
      if (!std::isfinite(q_a) || !std::isfinite(q_b) ||
          std::abs(q_a - q_b) > kJ2SyncLimit + 1e-12) {
        cycle_safe = false;
        for (auto& window : stable_windows) window.clear();
      }
    }
    bool stable = cycle_safe && std::all_of(
        stable_windows.begin(), stable_windows.end(),
        [](const std::deque<double>& window) {
          if (window.size() != 5U) return false;
          const auto minmax = std::minmax_element(
              window.begin(), window.end());
          return *minmax.second - *minmax.first <=
              kGear * kBrakeStationaritySpan + 1e-12;
        });
    if (stable) {
      for (auto& motor : motors) {
        motor.valid = true;
        motor.last_frame_valid = true;
        motor.consecutive_invalid = 0;
        motor.transport_fault_latched = false;
        motor.fault_latched = motor.non_transport_fault_latched;
        motor.speed_ready = false;
        motor.fast_speed_count = 0;
        motor.slow_speed_count = 0;
      }
      ++recovery_success_count;
      std::cerr << "COMMUNICATION_RECOVERY_READY"
                << " bus=" << bus
                << " episode=" << episode
                << " stable_brake_frames=5"
                << " old_command_discarded=YES"
                << " repreview_required=YES"
                << std::endl;
      return true;
    }
    if (transport_valid) {
      invalid_transport_cycles = 0;
      reopen_delay = std::chrono::milliseconds(100);
    } else if (++invalid_transport_cycles >= invalid_feedback_limit_for_bus(bus)) {
      // An adapter can disappear again while recovery is waiting for frames.
      // Resolve the stable by-id path again instead of retaining its old fd.
      serial.reset();
      invalid_transport_cycles = 0;
      for (auto& window : stable_windows) window.clear();
      std::this_thread::sleep_for(reopen_delay);
      reopen_delay = std::min(
          reopen_delay * 2, std::chrono::milliseconds(2000));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

int run(const Options& options) {
  g_thermal_policy = self_test_thermal_policy();
  sha256_self_test();
  j2_startup_prime_policy_self_test();
  thermal_interlock_self_test();
  thermal_derating_self_test();
  position_arrival_dwell_self_test();
  no_progress_watchdog_self_test();
  if (options.audit_j2_session_bundle) {
    command_mask_self_test();
    const std::string host_boot_id = current_host_boot_id();
    auto motors = make_motors("j2");
    const std::string actual_zero_sha256 =
        load_persistent_zero(options.zero_file, motors);
    if (actual_zero_sha256 != options.expected_zero_sha256)
      throw std::runtime_error("PERSISTENT_ZERO_SHA256_MISMATCH");
    const std::string actual_recovery_hint_sha256 =
        load_recovery_hints(options.recovery_hint_file, motors);
    const LoadedJ2SessionReference session_reference =
        load_j2_session_reference(
            options.j2_session_reference_file, actual_zero_sha256,
            actual_recovery_hint_sha256,
            options.expected_j2_session_reference_sha256,
            options.expected_j2_power_session_id, host_boot_id, motors);
    const J2LaunchPermit permit = load_j2_launch_permit(
        options, session_reference, actual_zero_sha256, host_boot_id,
        current_boottime_ns());
    std::cout << "DRY_RUN=YES\nSERIAL_OPENED=NO\n"
              << "J2_SESSION_BUNDLE_AUDIT=PASS\n"
              << "J2_LAUNCH_PERMIT_STATE=" << permit.state << "\n"
              << "J2_LAUNCH_PERMIT_ID=" << permit.permit_id << "\n";
    return 0;
  }
  if (options.audit_go_aux_session_bundle) {
    command_mask_self_test();
    const std::string host_boot_id = current_host_boot_id();
    auto motors = make_motors(options.bus);
    const std::string actual_zero_sha256 =
        load_persistent_zero(options.zero_file, motors);
    if (actual_zero_sha256 != options.expected_zero_sha256)
      throw std::runtime_error("PERSISTENT_ZERO_SHA256_MISMATCH");
    const std::string actual_recovery_hint_sha256 =
        load_recovery_hints(options.recovery_hint_file, motors);
    const LoadedJ2SessionReference session_reference =
        load_go_aux_session_reference(
            options.go_aux_session_reference_file, actual_zero_sha256,
            actual_recovery_hint_sha256,
            options.expected_go_aux_session_reference_sha256,
            options.expected_go_aux_power_session_id, host_boot_id, motors);
    const J2LaunchPermit permit = load_go_aux_launch_permit(
        options, session_reference, actual_zero_sha256, host_boot_id,
        current_boottime_ns());
    std::cout << "DRY_RUN=YES\nSERIAL_OPENED=NO\n"
              << "GO_AUX_SESSION_BUNDLE_AUDIT=PASS\n"
              << "GO_AUX_LAUNCH_PERMIT_STATE=" << permit.state << "\n"
              << "GO_AUX_LAUNCH_PERMIT_ID=" << permit.permit_id << "\n";
    return 0;
  }
  if (!options.execute) {
    command_mask_self_test();
    std::cout << "DRY_RUN=YES\nSERIAL_OPENED=NO\nDEFAULT_MODE=BRAKE\n"
                 "BRAKE_ONLY_CAPABILITY=YES\n"
                 "ASSISTED_TEACH_CAPABILITY=SINGLE_GO_JOINT_AFTER_CONFIRMED_HOLD\n"
                 "ASSISTED_TEACH_PRESS_LIMITS=30_SECONDS_5_DEGREES_5_DEG_S\nASSISTED_TEACH_SOFT_LIMIT_ACTION=CAPTURE_SELECTED_HOLD\nASSISTED_TEACH_STOPPING_HOLD_LIMITS=1_SECOND_2_DEGREE_ERROR\n"
                 "ASSISTED_TEACH_EMPIRICAL_LEASE_EXPIRY=BRAKE\n"
                 "CONTROL_LOOP_HZ=100\n"
                 "COMMAND_TARGET_POLICY=MODEL_SESSION_ENVELOPE_FULL_ENDPOINTS\n"
                 "COMMAND_TARGET_LIMITS_DEG="
                 "J1[-180,180],J2[-260,80],J3[-155.6,184.4],"
                 "J4[-129.49,145.51],J5[-118.54,103.26],J6[-180,180]\n"
                 "FEEDBACK_ENVELOPE_TOLERANCE_DEG=0.5\n"
                 "POSITION_ARRIVAL_TIMEOUT_SECONDS=90\n"
                 "J2_POSITION_MOVING_KP_MAX=3.0\n"
                 "J2_POSITION_MOVING_KD_MAX=0.30\n"
                 "J2_FIXED_HOLD_KP_MAX=3.0\nJ2_FIXED_HOLD_KD_MAX=0.30\n"
                 "J2_GAIN_RAMP=STATEFUL_NO_TRANSITION_DROP\n"
                 "J2_ACCELERATION_MAX_DEG_S2=15\n"
                 "J2_TFF=BOUNDED_COMMON_HOLD_INTEGRAL\nJ2_TFF_HARD_NM=1.50\n"
                 "J2_PREDICTED_WORK_NM=1.75\nJ2_HOLD_PREDICTED_WORK_NM=3.00\n"
                 "GO_AUX_TFF=BOUNDED_HOLD_INTEGRAL\n"
                 "GO_AUX_TFF_HARD_NM=J1:0.35,J3:1.60,J4:0.75,J5:0.50\n"
                 "GO_AUX_PREDICTED_WORK_NM=J1:2.50,J3:3.00,J4:2.50,J5:2.00\n"
                 "GO_AUX_GOVERNOR=IMMUTABLE_PLANNER_AFFINE_WIRE_REFERENCE\n"
                 "J2_GOVERNOR=MEASURED_STATE_AFFINE_INTERVAL_NO_SOFT_BRAKE\n"
                 "J2_TRANSIENT_INVALID_FEEDBACK_FRAMES=4\n"
                 "J2_STARTUP_SERIAL_PRIME=BEFORE_UDP_PREFIX_MAX3_THEN_5_HEALTHY_BRAKE\n"
                 "J2_SYNC_WARNING_DEG=0.25\n"
                 "J2_SYNC_HARD_DEG=0.5\n"
                 "J2_SYNC_HARD_POLICY=SINGLE_VALID_PAIR_BOTH_BRAKE_LATCHED\n"
                 "J2_SYNC_REARM_POLICY=FIVE_VALID_PAIRS_EXPLICIT_RELEASE_HIGHER_EPOCH_NEXT_CYCLE\n"
                  "J2_STARTUP_REFERENCE_POLICY=PARENT_BOUND_POWER_SESSION\n"
                  "J2_SESSION_STARTUP_TOLERANCE_DEG=2\n"
                  "J2_MISSING_SESSION_REFERENCE_POLICY=BRAKE_ONLY\n"
                  "J2_LAUNCH_PERMIT_POLICY=BOOT_BOUND_30S_SINGLE_USE\n"
                  "J2_STARTUP_RECHECK=BRAKE_50_FRAMES_BEFORE_COMMAND_BIND\n"
                  "J2_POWER_CONTINUITY_LOSS_POLICY=TERMINAL_BRAKE_AND_RECAPTURE\n"
                 "LEASE_EXPIRY_POLICY=HEALTH_GATED_FIXED_POSITION_SAFE_HOLD\n"
                  "LEASE_SAFE_HOLD_RESUME=HIGHER_ACTIVATION_EPOCH_REQUIRED\n"
                  "LEASE_SAFE_HOLD_STATUS_FIELD=lease_safe_hold\n"
                  "THERMAL_STOP_RAW_C="
              << g_thermal_policy.thermal_stop_c << "\n"
              << "THERMAL_COOLDOWN_RAW_BELOW_C="
              << g_thermal_policy.rearm_below_c << "\n"
              << "THERMAL_COOLDOWN_MIN_SECONDS="
              << g_thermal_policy.cooldown_seconds << "\n"
              <<
                  "THERMAL_COOLDOWN_MIN_VALID_BRAKE_FRAMES=500\n"
                  "THERMAL_REARM_POLICY=EXPLICIT_RELEASE_HIGHER_EPOCH_NEXT_CYCLE\n"
                  "THERMAL_COMMUNICATION_POLICY=ORTHOGONAL_KEEP_POLLING\n"
                  "LOAD_LIMIT_WATCHDOG_AUTHORITY="
                  "SOFTWARE_GUARD_NOT_CONTINUOUS_RATING\n"
                  "LOAD_LIMIT_NO_PROGRESS_ERROR_DEG=2\n"
                  "LOAD_LIMIT_NO_PROGRESS_IMPROVEMENT_DEG=0.25\n"
                  "LOAD_LIMIT_NO_PROGRESS_WINDOW_SECONDS=3\n"
                  "LOAD_LIMIT_NO_PROGRESS_MIN_VALID_FRAMES=100\n"
                  "LOAD_LIMIT_REARM_POLICY=EXPLICIT_RELEASE_HIGHER_EPOCH_NEXT_CYCLE\n"
                  "HOLD_TARGET_POLICY=FIXED_GUI_TARGET_MECHANICAL_ENVELOPE\n"
                 "FIXED_HOLD_CAPTURE_WINDOW_DEG=2\n"
                 "FIXED_HOLD_FEEDBACK_FRESH_MS=100\n"
                 "FIXED_HOLD_SAME_EPOCH_TARGET=IMMUTABLE\n"
                  "POSITION_MOVING_MASK=V12_OPTIONAL_V13_REQUIRED_ACTIVE_SUBSET\n"
                  "V13_QUINTIC_PROFILE=quintic-rest-to-rest-v1\n"
                  "V13_QUINTIC_CLOCK=INTEGER_SAMPLE_INDEX_NO_REPLAN\n"
                  "V13_QUINTIC_GRID=EXACT_INTEGER_NS_1_TO_10_MS\n"
                  "V13_QUINTIC_FIRST_PACKET=STRICTLY_BEFORE_EXECUTE_AT\n"
                  "V13_QUINTIC_FIRST_START_MATCH="
                  "FRESH_FEEDBACK_FIXED_HOLD_CAPTURE_WINDOW\n"
                  "MOVING_POSITION_VELOCITY_POLICY=TRANSITION_LOG_ONLY_NO_AUTHORITY_WITHDRAWAL\n"
                  "POSITION_ARRIVAL_TIMEOUT_POLICY="
                  "LATCHED_DOMAIN_BRAKE_EXPLICIT_RELEASE_HIGHER_EPOCH_NEXT_CYCLE\n"
                  "POSITION_ARRIVAL_DWELL=ENDPOINT_500MS_AND_50_VALID_FRAMES\n"
                  "THERMAL_CONFIG_SHA256="
                  "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467\n"
                  "THERMAL_DERATING_POLICY=LEGACY_PROFILE_LINEAR_VMAX_AMAX_KP_KD\n"
                  "V13_THERMAL_DERATING_POLICY="
                  "ABORT_LATCHED_BRAKE_REPREVIEW_REQUIRED\n"
                  "THERMAL_STOP_POLICY=RAW_GE_CONFIG_STOP_C_LATCHED_DOMAIN_BRAKE\n"
                 "COMMAND_PACKET_BUDGET_PER_CYCLE=32\n"
                 "COMMAND_REJECT_SUMMARY_SECONDS=5\n"
                 "ACTIVE_DEADLINE_CONSECUTIVE_LIMIT=3\n"
                 "ACTIVE_DEADLINE_POLICY=TRANSITION_LOG_ONLY_NO_AUTHORITY_WITHDRAWAL\n"
                 "EXPLICIT_DRAG_MODE=BRAKE\n"
                 "MOTOR_INTERNAL_ZERO_WRITE=NO\n";
    return 0;
  }
  g_expected_gravity_authority_binding.authority_class =
      options.brake_only ? "NONE" :
      options.expected_gravity_authority_class;
  g_expected_gravity_authority_binding.empirical_envelope_id =
      options.expected_empirical_envelope_id;
  g_expected_gravity_authority_binding.empirical_envelope_sha256 =
      options.expected_empirical_envelope_sha256;
  g_expected_gravity_authority_binding.anchor_sha256 =
      options.expected_gravity_anchor_sha256;
  g_expected_gravity_authority_binding.session_id =
      options.expected_gravity_session_id;
  g_expected_gravity_authority_binding.state_instance_id =
      options.expected_gravity_state_instance_id;
  if (::prctl(PR_SET_PDEATHSIG, SIGTERM) != 0 || ::getppid() == 1)
    throw std::runtime_error("PARENT_DEATH_GUARD_FAILED");
  if (options.expected_thermal_config_sha256 != kThermalConfigSha256)
    throw std::runtime_error("THERMAL_CONFIG_EXPECTED_SHA256_MISMATCH");
  const SecureFileBytes thermal_config = read_secure_owned_policy_file(
      options.thermal_config_file, 64U * 1024U,
      "THERMAL_CONFIG_OPEN_FAILED", "THERMAL_CONFIG_FILE_UNSAFE",
      "THERMAL_CONFIG_READ_FAILED");
  if (thermal_config.sha256 != kThermalConfigSha256)
    throw std::runtime_error("THERMAL_CONFIG_SHA256_MISMATCH");
  g_thermal_policy = parse_thermal_policy_yaml(thermal_config.data);
  const BusDefinition definition = bus_definition(options.bus);
  if (::access(definition.port, R_OK | W_OK) != 0)
    throw std::runtime_error("STABLE_PORT_NOT_ACCESSIBLE");
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode ||
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC) != kFocMode ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) - kGear) > 1e-6)
    throw std::runtime_error("SDK_AUTHORITY_MISMATCH");
  const bool j2_active_session = options.bus == "j2" && !options.brake_only;
  const bool go_aux_active_session =
      (options.bus == "j1" || options.bus == "j345") && !options.brake_only;
  const bool active_power_session =
      j2_active_session || go_aux_active_session;
  const std::string host_boot_id =
      active_power_session ? current_host_boot_id() : std::string();
  auto motors = make_motors(options.bus);
  std::string actual_zero_sha256;
  std::string actual_recovery_hint_sha256;
  // BRAKE-only acquisition establishes raw, current-power phase evidence.  It
  // must not load or branch-select from a previous session's persistent zero
  // or recovery hints; doing so could reject exactly the new phase that the
  // capture is intended to evidence.
  if (!options.brake_only) {
    actual_zero_sha256 = load_persistent_zero(options.zero_file, motors);
    actual_recovery_hint_sha256 =
        load_recovery_hints(options.recovery_hint_file, motors);
  }
  J2LaunchPermit j2_launch_permit;
  if (j2_active_session) {
    if (actual_zero_sha256 != options.expected_zero_sha256)
      throw std::runtime_error("PERSISTENT_ZERO_SHA256_MISMATCH");
    const LoadedJ2SessionReference session_reference =
        load_j2_session_reference(
            options.j2_session_reference_file, actual_zero_sha256,
            actual_recovery_hint_sha256,
            options.expected_j2_session_reference_sha256,
            options.expected_j2_power_session_id, host_boot_id, motors);
    j2_launch_permit = load_j2_launch_permit(
        options, session_reference, actual_zero_sha256, host_boot_id,
        current_boottime_ns());
  }
  if (go_aux_active_session) {
    if (actual_zero_sha256 != options.expected_zero_sha256)
      throw std::runtime_error("PERSISTENT_ZERO_SHA256_MISMATCH");
    const LoadedJ2SessionReference session_reference =
        load_go_aux_session_reference(
            options.go_aux_session_reference_file, actual_zero_sha256,
            actual_recovery_hint_sha256,
            options.expected_go_aux_session_reference_sha256,
            options.expected_go_aux_power_session_id, host_boot_id, motors);
    j2_launch_permit = load_go_aux_launch_permit(
        options, session_reference, actual_zero_sha256, host_boot_id,
        current_boottime_ns());
  }
  std::vector<std::unique_ptr<ProcessLock>> locks;
  for (const char* path : definition.locks)
    locks.push_back(std::make_unique<ProcessLock>(path));
  if (active_power_session) consume_j2_launch_permit(j2_launch_permit);
  auto serial = std::make_unique<SerialPort>(
      definition.port, 16, 4000000, 20000, BlockYN::NO,
      bytesize_t::eightbits, parity_t::parity_none,
      stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none);
  TxAudit tx_audit(options.brake_only);
  TerminalBrakeGuard brake_guard(serial, motors, options.bus, tx_audit);
  J2StartupBrakePrimeEvidence j2_startup_brake_prime;
  if (options.bus == "j2")
    j2_startup_brake_prime =
        prime_j2_serial_before_feedback(*serial, motors, tx_audit);

  int command_socket = -1;
  const int feedback_socket = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (feedback_socket < 0)
    throw std::runtime_error("UDP_SOCKET_FAILED");
  if (!options.brake_only && !active_power_session)
    command_socket = open_command_socket(definition);
  sockaddr_in feedback_address{};
  feedback_address.sin_family = AF_INET;
  feedback_address.sin_port = htons(static_cast<std::uint16_t>(options.feedback_port));
  feedback_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  GuiCommand command;
  GuiCommand lease_safe_hold_command;
  std::array<double, 6> q_command{};
  std::array<double, 6> dq_command{};
  std::string previous_mode = "brake";
  std::array<bool, 6> previous_active_joint_mask{};
  std::array<bool, 6> previous_moving_joint_mask{};
  CommandSafetyState command_safety;
  CommandReceiveState command_receive_state;
  ThermalInterlockState thermal_interlock;
  NoProgressWatchdogState no_progress_watchdog;
  bool lease_safe_hold_active = false;
  bool prior_external_hold_confirmed = false;
  GuiCommand confirmed_external_hold_command;
  std::uint64_t lease_safe_hold_source_epoch = 0;
  bool j2_sync_fault = false;
  J2SyncFaultFilter j2_sync_filter;
  bool j2_startup_verified = !active_power_session;
  bool j2_power_continuity_lost = false;
  bool domain_fault = false;
  bool domain_fault_reported = false;
  std::string domain_fault_reason;
  std::array<bool, 6> position_tracking{};
  std::array<bool, 6> position_endpoint_reached{};
  std::array<bool, 6> position_arrived_once{};
  std::array<bool, 6> position_arrival_overdue{};
  std::array<int, 6> position_arrival_qualifying_frames{};
  std::array<Clock::time_point, 6> position_arrival_window_started_at{};
  std::array<std::uint64_t, 6> position_tracking_epoch{};
  std::array<double, 6> last_position_targets{};
  std::array<Clock::time_point, 6> position_started_at{};
  position_started_at.fill(Clock::now());
  auto next = Clock::now();
  std::uint64_t cycles = 0;
  std::uint64_t communication_recovery_attempt_count = 0;
  std::uint64_t communication_recovery_success_count = 0;
  bool previous_j2_pair_ready = false;
  std::array<BoundedHoldIntegralState, 6> hold_integral_states{};
  std::array<double, 6> hold_integral_wire_nm{};
  std::array<double, 6> applied_gravity_feedforward_nm{};
  double previous_thermal_derating_factor = 1.0;
  double j2_integral_wire_nm = 0.0;
  double j2_previous_common_position = 0.0;
  bool j2_previous_common_ready = false;
  double j2_derived_velocity_filtered = 0.0;
  double j2_gain_kp = 0.0;
  double j2_gain_kd = 0.0;
  bool j2_gain_ready = false;
  bool j2_governor_limited = false;
  std::array<bool, 6> aux_governor_limited{};
  bool j2_governor_postcheck_reported = false;
  bool j2_torque_feedback_saturated_reported = false;
  int active_deadline_miss_count = 0;
  bool active_deadline_degraded = false;
  std::vector<InvalidFeedbackLogState> invalid_feedback_logs(motors.size());
  DomainBrakeLogState domain_brake_log;
  ActiveCommandBlockedLogState active_command_blocked_log;
  while (!g_stop.load()) {
    const auto loop_started = Clock::now();
    const bool j2_sync_rearmed_this_cycle =
        options.bus == "j2" &&
        apply_pending_j2_sync_rearm_at_cycle_start(j2_sync_filter);
    j2_sync_fault = j2_sync_filter.fault;
    const bool thermal_rearmed_this_cycle =
        apply_pending_thermal_rearm_at_cycle_start(thermal_interlock);
    const bool no_progress_rearmed_this_cycle =
        apply_pending_no_progress_rearm_at_cycle_start(no_progress_watchdog);
    publish_thermal_latch_to_motors(
        motors, thermal_interlock.fault_latched);
    if (thermal_rearmed_this_cycle) {
      std::cerr << "THERMAL_REARM_APPLIED"
                << " bus=" << options.bus
                << " activation_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << thermal_interlock.minimum_rearm_epoch
                << std::endl;
    }
    if (no_progress_rearmed_this_cycle) {
      std::cerr << "LOAD_LIMIT_NO_PROGRESS_REARM_APPLIED"
                << " bus=" << options.bus
                << " activation_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << no_progress_watchdog.minimum_rearm_epoch
                << std::endl;
    }
    if (j2_sync_rearmed_this_cycle) {
      std::cerr << "J2_SYNC_REARM_APPLIED"
                << " bus=" << options.bus
                << " activation_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << j2_sync_filter.minimum_rearm_epoch
                << std::endl;
    }
    const bool thermal_fault_latched_at_cycle_start =
        thermal_interlock.fault_latched;
    const bool no_progress_fault_latched_at_cycle_start =
        no_progress_watchdog.fault_latched;
    const bool j2_sync_fault_latched_at_cycle_start =
        j2_sync_filter.fault;
    auto latch_domain_fault = [&](const char* reason) {
      if (!domain_fault) domain_fault_reason = reason;
      domain_fault = true;
    };
    if (std::any_of(
            motors.begin(), motors.end(), [](const MotorRuntime& motor) {
              return motor.fault_latched;
            }))
      latch_domain_fault("OWNED_MOTOR_FAULT_LATCHED");
    const bool previous_owned_joint_active = previous_mode != "brake" &&
        std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
          return previous_active_joint_mask[
              static_cast<std::size_t>(motor.joint_index)];
        });
    // Startup and BRAKE polling may be delayed by process scheduling without
    // creating a control hazard.  Keep the hard timing latch for an actually
    // active joint, where a missed deadline does affect commanded motion.
    // The recovery supervisor and four controller processes share a non-RT
    // Linux host. Allow five extra 10 ms control periods of scheduler jitter in
    // recovery, while retaining the original 2 ms deadline for normal GUI use.
    const bool previous_recovery = lease_safe_hold_active
        ? lease_safe_hold_command.recovery : command.recovery;
    const auto active_deadline_slack = previous_recovery
        ? std::chrono::milliseconds(50) : std::chrono::milliseconds(2);
    const bool active_deadline_missed = cycles > 0U &&
        previous_owned_joint_active &&
        loop_started > next + active_deadline_slack;
    active_deadline_miss_count = next_active_deadline_miss_count(
        active_deadline_miss_count, previous_owned_joint_active,
        active_deadline_missed);
    if (active_deadline_missed) {
      // Rebase the schedule so one host stall cannot be counted again by a
      // burst of catch-up iterations. Only distinct consecutive late frames
      // can reach the active-control fault threshold.
      next = loop_started;
      if (active_deadline_miss_count >= kActiveDeadlineConsecutiveLimit &&
          !active_deadline_degraded) {
        std::cerr << "ACTIVE_LOOP_DEGRADED"
                  << " bus=" << options.bus
                  << " consecutive_misses=" << active_deadline_miss_count
                  << std::endl;
        active_deadline_degraded = true;
      }
    } else if (active_deadline_degraded &&
               active_deadline_miss_count == 0) {
      std::cerr << "ACTIVE_LOOP_RECOVERED"
                << " bus=" << options.bus << std::endl;
      active_deadline_degraded = false;
    }
    const GuiCommand command_before_receive = command;
    const bool lease_expired_before_receive = command.received &&
        !g_stop.load() &&
        std::chrono::duration<double>(Clock::now() - command.received_at).count() >
            kLeaseSeconds;
    const bool empirical_expired_before_receive = command.received &&
        command_uses_empirical_gravity_authority(command) &&
        !empirical_gravity_authority_is_current(command);
    if (command_safety.gravity_empirical_active &&
        (lease_expired_before_receive || empirical_expired_before_receive ||
         domain_fault || j2_sync_fault ||
         thermal_interlock.fault_latched ||
         no_progress_watchdog.fault_latched || g_stop.load()))
      spend_empirical_gravity_authority(command_safety);
    CommandReceiveResult receive_result;
    if (command_socket >= 0) {
      receive_result = receive_latest(
          command_socket, command, command_safety, motors,
          position_arrived_once, position_endpoint_reached,
          position_tracking, position_tracking_epoch,
          last_position_targets,
          command_receive_state);
    }
    const bool explicit_release_packet_received =
        receive_result.domain_release_received;
    const bool command_lease_fresh = command.received && !g_stop.load() &&
        std::chrono::duration<double>(Clock::now() - command.received_at).count() <=
            kLeaseSeconds;
    const bool command_requests_active_owned_joint = command.received &&
        is_position_holding_mode(command.mode) &&
        std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
          return command.active_joint_mask[
              static_cast<std::size_t>(motor.joint_index)];
        });
    command_safety.minimum_activation_epoch = minimum_epoch_after_lease(
        command_safety.minimum_activation_epoch, command_lease_fresh,
        command_requests_active_owned_joint, command.activation_epoch);
    if (thermal_interlock.fault_latched) {
      thermal_interlock.minimum_rearm_epoch = std::max(
          {thermal_interlock.minimum_rearm_epoch,
           command_safety.minimum_activation_epoch,
           saturating_next_activation_epoch(
               command_safety.highest_rejected_active_epoch)});
      command_safety.minimum_activation_epoch = std::max(
          command_safety.minimum_activation_epoch,
          thermal_interlock.minimum_rearm_epoch);
    }
    if (no_progress_watchdog.fault_latched) {
      no_progress_watchdog.minimum_rearm_epoch = std::max(
          {no_progress_watchdog.minimum_rearm_epoch,
           command_safety.minimum_activation_epoch,
           saturating_next_activation_epoch(
               command_safety.highest_rejected_active_epoch)});
      command_safety.minimum_activation_epoch = std::max(
          command_safety.minimum_activation_epoch,
          no_progress_watchdog.minimum_rearm_epoch);
    }
    if (j2_sync_filter.fault) {
      j2_sync_filter.minimum_rearm_epoch = std::max(
          {j2_sync_filter.minimum_rearm_epoch,
           command_safety.minimum_activation_epoch,
           saturating_next_activation_epoch(
               command_safety.highest_rejected_active_epoch)});
      command_safety.minimum_activation_epoch = std::max(
          command_safety.minimum_activation_epoch,
          j2_sync_filter.minimum_rearm_epoch);
    }
    if (thermal_fault_latched_at_cycle_start &&
        observe_explicit_thermal_release(
            thermal_interlock, explicit_release_packet_received)) {
      std::cerr << "THERMAL_RELEASE_OBSERVED"
                << " bus=" << options.bus
                << " command_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << thermal_interlock.minimum_rearm_epoch
                << std::endl;
    }
    if (no_progress_fault_latched_at_cycle_start &&
        observe_explicit_no_progress_release(
            no_progress_watchdog, explicit_release_packet_received)) {
      std::cerr << "LOAD_LIMIT_NO_PROGRESS_RELEASE_OBSERVED"
                << " bus=" << options.bus
                << " command_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << no_progress_watchdog.minimum_rearm_epoch
                << std::endl;
    }
    if (j2_sync_fault_latched_at_cycle_start &&
        observe_explicit_j2_sync_release(
            j2_sync_filter, explicit_release_packet_received)) {
      std::cerr << "J2_SYNC_RELEASE_OBSERVED"
                << " bus=" << options.bus
                << " command_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << j2_sync_filter.minimum_rearm_epoch
                << std::endl;
    }
    const bool lease_expiry_detected =
        lease_expired_before_receive || !command_lease_fresh;
    if (command_safety.gravity_empirical_active &&
        (lease_expiry_detected || explicit_release_packet_received ||
         domain_fault || j2_sync_fault ||
         thermal_interlock.fault_latched ||
         no_progress_watchdog.fault_latched))
      spend_empirical_gravity_authority(command_safety);
    const GuiCommand& lease_expiry_source = lease_expired_before_receive
        ? command_before_receive : command;
    const bool fresh_command_explicitly_releases =
        explicit_release_packet_received ||
        external_command_explicitly_releases_safe_hold(
            command, command_lease_fresh, motors);
    const bool fresh_command_supersedes_expired_epoch =
        external_command_can_resume_from_safe_hold(
            command, command_lease_fresh, motors,
            lease_expiry_source.activation_epoch,
            command_safety.minimum_activation_epoch,
            command_safety.highest_rejected_active_epoch);
    if (prior_external_hold_confirmed && command_lease_fresh &&
        !same_external_hold_authority(
            confirmed_external_hold_command, command))
      prior_external_hold_confirmed = false;
    if (fresh_command_explicitly_releases || domain_fault || j2_sync_fault ||
        thermal_interlock.fault_latched ||
        no_progress_watchdog.fault_latched)
      prior_external_hold_confirmed = false;
    bool lease_safe_hold_entered_this_cycle = false;
    if (lease_safe_hold_active && fresh_command_explicitly_releases) {
      std::cerr << "LEASE_SAFE_HOLD_EXIT"
                << " bus=" << options.bus
                << " reason=EXPLICIT_"
                << (explicit_release_packet_received ? "RELEASE_PACKET" :
                    command.mode == "drag" ? "DRAG" :
                    command.mode == "brake" ? "BRAKE" : "DOMAIN_DESELECT")
                << " source_epoch=" << lease_safe_hold_source_epoch
                << " command_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << command_safety.minimum_activation_epoch
                << std::endl;
      lease_safe_hold_active = false;
    } else if (lease_safe_hold_active &&
               external_command_can_resume_from_safe_hold(
                   command, command_lease_fresh, motors,
                   lease_safe_hold_source_epoch,
                   command_safety.minimum_activation_epoch,
                   command_safety.highest_rejected_active_epoch)) {
      std::cerr << "LEASE_SAFE_HOLD_EXIT"
                << " bus=" << options.bus
                << " reason=HIGHER_ACTIVATION_EPOCH"
                << " source_epoch=" << lease_safe_hold_source_epoch
                << " command_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << command_safety.minimum_activation_epoch
                << std::endl;
      lease_safe_hold_active = false;
    }
    if (!lease_safe_hold_active && lease_expiry_detected &&
        prior_external_hold_confirmed && !g_stop.load() &&
        !fresh_command_explicitly_releases &&
        !fresh_command_supersedes_expired_epoch &&
        // An empirical proof is deliberately short-lived.  A dead Router
        // must reach BRAKE at the 500 ms command lease boundary instead of
        // converting the last proof/target into an unbounded local HOLD.
        !command_uses_empirical_gravity_authority(
            confirmed_external_hold_command) &&
        !thermal_interlock.fault_latched &&
        !no_progress_watchdog.fault_latched) {
      GuiCommand captured;
      if (capture_lease_safe_hold_command(
              options.bus, confirmed_external_hold_command, motors, domain_fault,
              j2_sync_fault, captured, &position_arrived_once)) {
        captured.feedforward_nm = applied_gravity_feedforward_nm;
        if (captured.gravity_authority.present)
          captured.gravity_authority.feedforward_nm =
              applied_gravity_feedforward_nm;
        lease_safe_hold_command = std::move(captured);
        lease_safe_hold_source_epoch =
            confirmed_external_hold_command.activation_epoch;
        lease_safe_hold_active = true;
        lease_safe_hold_entered_this_cycle = true;
        prior_external_hold_confirmed = false;
        for (const auto& motor : motors) {
          const auto joint = static_cast<std::size_t>(motor.joint_index);
          if (!lease_safe_hold_command.active_joint_mask[joint]) continue;
          q_command[joint] = lease_safe_hold_command.targets[joint];
          dq_command[joint] = 0.0;
        }
        std::cerr << "LEASE_SAFE_HOLD_ENTER"
                  << " bus=" << options.bus
                  << " source_mode=" << lease_expiry_source.mode
                  << " source_epoch=" << lease_safe_hold_source_epoch
                  << " minimum_epoch="
                  << command_safety.minimum_activation_epoch;
        for (const auto& motor : motors) {
          const auto joint = static_cast<std::size_t>(motor.joint_index);
          if (!lease_safe_hold_command.active_joint_mask[joint]) continue;
          std::cerr << " motor=" << motor.name
                    << ",logical_target_rad="
                    << lease_safe_hold_command.targets[joint]
                    << ",kp=" << lease_safe_hold_command.kp[joint]
                    << ",kd=" << lease_safe_hold_command.kd[joint]
                    << ",feedforward_nm="
                    << lease_safe_hold_command.feedforward_nm[joint];
        }
        std::cerr << std::endl;
      }
    }
    GuiCommand control_command = lease_safe_hold_active
        ? lease_safe_hold_command : command;
    const bool empirical_authority_expired =
        !empirical_gravity_authority_is_current(control_command);
    double thermal_derating_factor = 1.0;
    for (const auto& motor : motors) {
      thermal_derating_factor = std::min(
          thermal_derating_factor,
          thermal_derating_factor_for_raw_temperature(motor.temperature));
    }
    GuiCommand thermally_derated_command = control_command;
    thermally_derated_command.vmax *= thermal_derating_factor;
    thermally_derated_command.amax *= thermal_derating_factor;
    for (const auto& motor : motors) {
      const auto joint = static_cast<std::size_t>(motor.joint_index);
      thermally_derated_command.kp[joint] *= thermal_derating_factor;
      thermally_derated_command.kd[joint] *= thermal_derating_factor;
    }
    if (std::abs(
            thermal_derating_factor - previous_thermal_derating_factor) >
        1e-12) {
      std::cerr << "THERMAL_DERATING_UPDATE"
                << " bus=" << options.bus
                << " factor=" << thermal_derating_factor
                << std::endl;
      previous_thermal_derating_factor = thermal_derating_factor;
    }
    const bool control_authority_available =
        !g_stop.load() && (lease_safe_hold_active || command_lease_fresh);
    std::string effective_mode = control_command.mode;
    if (options.brake_only) effective_mode = "brake";
    // Re-evaluate wall-clock expiry every control cycle.  Validation only at
    // UDP receive time leaves an active command live if the Router or gravity
    // node dies immediately before the envelope expires.
    if (empirical_authority_expired) effective_mode = "brake";
    if (!control_authority_available)
      effective_mode = "brake";
    if (explicit_release_packet_received)
      effective_mode = "brake";
    if (domain_fault) effective_mode = "brake";
    if (options.bus == "j2" && j2_sync_fault)
      effective_mode = "brake";
    if (thermal_interlock.fault_latched) effective_mode = "brake";
    if (no_progress_watchdog.fault_latched) effective_mode = "brake";
    if (active_power_session && !j2_startup_verified)
      effective_mode = "brake";
    // Legacy drag remains a deliberate torque release. The separately
    // authorized teach mode follows one joint with bounded gravity support.
    if (effective_mode == "drag")
      effective_mode = "brake";
    if ((control_command.hand_guidance.present && is_position_holding_mode(effective_mode)) || effective_mode == "teach" ||
        (effective_mode == "hold" && command_safety.teach_exit_hold.present)) {
      const std::string teach_blocker = control_command.hand_guidance.present
          ? hand_guidance_runtime_blocker(control_command, command_safety, motors, j2_derived_velocity_filtered, monotonic_ns())
          : assisted_teach_runtime_blocker(control_command, command_safety, motors, j2_derived_velocity_filtered, monotonic_ns());
      if (!teach_blocker.empty()) {
        latch_position_safety_watchdog(
            no_progress_watchdog, 0.0, teach_blocker.c_str(),
            control_command.activation_epoch,
            command_safety.minimum_activation_epoch,
            command_safety.highest_rejected_active_epoch);
        command_safety.minimum_activation_epoch = std::max(
            command_safety.minimum_activation_epoch, no_progress_watchdog.minimum_rearm_epoch);
        effective_mode = "brake";
        prior_external_hold_confirmed = false;
        std::cerr << teach_blocker << " bus=" << options.bus << std::endl;
      } else {
        effective_mode = control_command.mode;
        thermally_derated_command.mode = control_command.mode;
        thermally_derated_command.targets = control_command.targets;
        thermally_derated_command.moving_joint_mask = control_command.moving_joint_mask;
      }
    }
    const auto teach_cycle_ns = monotonic_ns();
    const bool teach_stopping_hold = effective_mode == "hold" &&
        command_safety.teach_exit_hold.present && !command_safety.teach_exit_hold.completed;
    std::array<double, 6> teach_damping_nm{};
    if (effective_mode == "teach" || teach_stopping_hold ||
        (control_command.hand_guidance.present && effective_mode == "hold")) {
      std::set<int> damped_joints;
      for (const auto& motor : motors) {
        const auto joint = static_cast<std::size_t>(motor.joint_index);
        if (!uses_assisted_teach_damping(control_command, effective_mode, joint,
                command_safety.teach_exit_hold, teach_cycle_ns) ||
            !damped_joints.insert(motor.joint_index).second) continue;
        const double velocity = joint == 1U ? j2_derived_velocity_filtered : motor.integral_encoder_velocity;
        const double reference_velocity = control_command.hand_guidance.present && effective_mode == "teach"
            ? control_command.hand_guidance.velocity_rad_s[joint] : 0.0;
        teach_damping_nm[joint] = std::min(thermally_derated_command.kd[joint], motor.kd_limit) * kGear * (reference_velocity - velocity);
        // Vendor velocity has proven stationary noise. Apply damping from the
        // encoder observer as explicit torque and disable only selected KD.
        thermally_derated_command.kd[joint] = 0.0;
      }
    }
    // Once an empirical lifecycle has reached a hardware BRAKE edge it is
    // single-use spent.  Latch before a later UDP datagram can try the same
    // envelope at a higher activation epoch.
    if (effective_mode == "brake")
      spend_empirical_gravity_authority(command_safety);
    if (!lease_safe_hold_active && effective_mode != "brake" &&
        control_command.activation_epoch <
            command_safety.minimum_activation_epoch)
      effective_mode = "brake";
    if (effective_mode == "drag" || is_position_holding_mode(effective_mode)) {
      const bool owned_joint_selected =
          std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
            return control_command.active_joint_mask[
                static_cast<std::size_t>(motor.joint_index)];
          });
      if (!owned_joint_selected) effective_mode = "brake";
    }
    const bool exact_trajectory_thermal_derating_region =
        effective_mode == "position" && control_command.quintic.present &&
        std::any_of(
            motors.begin(), motors.end(), [](const MotorRuntime& motor) {
              return motor.temperature >=
                  g_thermal_policy.derating_start_c;
            });
    if (exact_trajectory_thermal_derating_region) {
      const bool newly_latched = latch_thermal_interlock(
          thermal_interlock, "EXACT_TRAJECTORY_DERATING_ABORT",
          control_command.activation_epoch,
          command_safety.minimum_activation_epoch,
          command_safety.highest_rejected_active_epoch);
      command_safety.minimum_activation_epoch = std::max(
          command_safety.minimum_activation_epoch,
          thermal_interlock.minimum_rearm_epoch);
      publish_thermal_latch_to_motors(motors, true);
      effective_mode = "brake";
      prior_external_hold_confirmed = false;
      if (newly_latched) {
        std::cerr << "V13_THERMAL_DERATING_TRAJECTORY_ABORT"
                  << " bus=" << options.bus
                  << " activation_epoch="
                  << control_command.activation_epoch
                  << " minimum_epoch="
                  << thermal_interlock.minimum_rearm_epoch
                  << " policy=REPREVIEW_REQUIRED"
                  << std::endl;
      }
    }
    if (effective_mode == "position") {
      std::set<int> owned_joints;
      for (const auto& motor : motors) owned_joints.insert(motor.joint_index);
      for (const int joint : owned_joints) {
        const std::size_t index = static_cast<std::size_t>(joint);
        if (!control_command.active_joint_mask[index] ||
            !control_command.moving_joint_mask[index]) {
          position_tracking[index] = false;
          position_tracking_epoch[index] = 0U;
          position_endpoint_reached[index] = false;
          position_arrived_once[index] = false;
          position_arrival_overdue[index] = false;
          position_arrival_qualifying_frames[index] = 0;
          position_arrival_window_started_at[index] = Clock::time_point{};
          continue;
        }
        if (!position_tracking[index] ||
            position_tracking_epoch[index] !=
                control_command.activation_epoch ||
            std::abs(control_command.targets[index] -
                     last_position_targets[index]) > 1e-9) {
          position_started_at[index] = Clock::now();
          position_endpoint_reached[index] = false;
          position_arrived_once[index] = false;
          position_arrival_overdue[index] = false;
          position_arrival_qualifying_frames[index] = 0;
          position_arrival_window_started_at[index] = Clock::time_point{};
        }
        last_position_targets[index] = control_command.targets[index];
        position_tracking_epoch[index] = control_command.activation_epoch;
        position_tracking[index] = true;
      }
    } else {
      position_tracking.fill(false);
      position_tracking_epoch.fill(0U);
      position_endpoint_reached.fill(false);
      position_arrived_once.fill(false);
      position_arrival_overdue.fill(false);
      position_arrival_qualifying_frames.fill(0);
      position_arrival_window_started_at.fill(Clock::time_point{});
    }
    bool j2_pair_ready = options.bus != "j2" ||
        (motors.size() == 2U &&
         std::all_of(motors.begin(), motors.end(), [](const MotorRuntime& motor) {
           return motor.reference_ready && motor.valid && !motor.fault_latched;
         }));
    const bool position_control_requested =
        is_position_holding_mode(effective_mode);
    QuinticSampleClock trajectory_sample;
    bool trajectory_sample_valid = false;
    if (effective_mode == "position" && control_command.quintic.present) {
      trajectory_sample = quintic_sample_clock(
          control_command.quintic, monotonic_ns());
      trajectory_sample_valid = true;
    }
    const bool active_transition = position_control_requested &&
        !lease_safe_hold_entered_this_cycle &&
        (effective_mode != previous_mode ||
         std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
            const std::size_t index = static_cast<std::size_t>(motor.joint_index);
            return control_command.active_joint_mask[index] &&
                (!previous_active_joint_mask[index] ||
                 control_command.moving_joint_mask[index] !=
                     previous_moving_joint_mask[index]);
           }));
    if (!control_command.quintic.present &&
        effective_mode == "position" &&
        control_command.moving_joint_mask[1] && options.bus == "j2" &&
        j2_pair_ready &&
        (active_transition || !previous_j2_pair_ready)) {
      const double q_a = -1.0 * (motors[0].unwrapped - motors[0].reference) / kGear;
      const double q_b = +1.0 * (motors[1].unwrapped - motors[1].reference) / kGear;
      q_command[1] = 0.5 * (q_a + q_b);
      dq_command[1] = 0.0;
    } else if (!control_command.quintic.present &&
               effective_mode == "position" && options.bus != "j2") {
      for (const auto& motor : motors) {
        const std::size_t index = static_cast<std::size_t>(motor.joint_index);
        const bool motor_transition =
            control_command.active_joint_mask[index] &&
            control_command.moving_joint_mask[index] &&
            (effective_mode != previous_mode ||
             !previous_active_joint_mask[index] ||
             !previous_moving_joint_mask[index]);
        if (!motor.reference_ready || !motor_transition) continue;
        q_command[index] =
            motor.sign * (motor.unwrapped - motor.reference) / kGear;
        dq_command[index] = 0.0;
      }
    }
    if (is_position_holding_mode(effective_mode)) {
      std::set<int> fixed_hold_joints;
      for (const auto& motor : motors) {
        const auto joint = static_cast<std::size_t>(motor.joint_index);
        if (!control_command.active_joint_mask[joint] ||
            !joint_uses_fixed_hold_target(
                control_command, effective_mode, joint) ||
            !fixed_hold_joints.insert(motor.joint_index).second)
          continue;
        q_command[joint] = control_command.targets[joint];
        dq_command[joint] = 0.0;
      }
    }
    if (effective_mode == "teach" && !control_command.hand_guidance.present)
      apply_assisted_teach_reference(control_command, motors, q_command, dq_command);
    std::set<int> planned;
    if (effective_mode == "position" && j2_pair_ready) {
      for (const auto& motor : motors) {
        if (motor.reference_ready && !motor.fault_latched &&
            control_command.active_joint_mask[
                static_cast<std::size_t>(motor.joint_index)] &&
            control_command.moving_joint_mask[
                static_cast<std::size_t>(motor.joint_index)] &&
            planned.insert(motor.joint_index).second) {
          if (control_command.quintic.present) {
            apply_quintic_reference(
                motor.joint_index, control_command, trajectory_sample,
                q_command, dq_command);
          } else {
            update_profile(
                motor.joint_index, thermally_derated_command,
                q_command, dq_command);
          }
        }
      }
    } else {
      dq_command.fill(0.0);
    }
    if (effective_mode == "teach" && control_command.hand_guidance.present) {
      apply_hand_guidance_reference(control_command, motors, q_command, dq_command);
    }
    // Keep the trajectory generator state independent from the governed wire
    // reference.  Under a heavy load the J2 governor may legitimately limit
    // one cycle's PD command; feeding that limited value back into q_command
    // would move the planner endpoint itself and could prevent hold protection
    // from ever engaging.  The authorized target/profile remain immutable.
    std::array<bool, 6> profile_endpoint_phase{};
    for (const int joint : planned) {
      const auto index = static_cast<std::size_t>(joint);
      // Exact v1.3 q/dq samples are an immutable preview contract.  They are
      // never time-scaled independently at runtime: entering the thermal
      // derating region has already latched BRAKE above and requires a fresh
      // preview/token.  Legacy profiles retain conservative live derating.
      if (!control_command.quintic.present)
        dq_command[index] *= thermal_derating_factor;
      profile_endpoint_phase[index] = position_profile_at_authorized_endpoint(
          control_command, index, q_command[index], dq_command[index]);
    }
    std::array<double, 6> aux_wire_q_command = q_command;
    std::array<double, 6> aux_wire_dq_command = dq_command;
    double j2_wire_q_command = q_command[1];
    double j2_wire_dq_command = dq_command[1];

    if (!is_position_holding_mode(effective_mode)) {
      applied_gravity_feedforward_nm.fill(0.0);
    } else {
      const double maximum_step =
          kGravityFeedforwardSlewNmPerSecond * kPeriod;
      for (const auto& motor : motors) {
        const auto joint = static_cast<std::size_t>(motor.joint_index);
        const double target =
            control_command.gravity_authority.present &&
                control_command.active_joint_mask[joint]
            ? control_command.feedforward_nm[joint] : 0.0;
        applied_gravity_feedforward_nm[joint] += std::clamp(
            target - applied_gravity_feedforward_nm[joint],
            -maximum_step, maximum_step);
      }
    }

    hold_integral_wire_nm.fill(0.0);
    bool teach_governor_infeasible = false;
    j2_integral_wire_nm = 0.0;
    if (thermal_derating_factor < 1.0 - 1e-12) {
      for (auto& state : hold_integral_states)
        reset_bounded_hold_integral(state);
    }
    double j2_effective_kp = j2_gain_kp;
    double j2_effective_kd = j2_gain_kd;
    const bool j2_hold_protection = options.bus == "j2" &&
        use_j2_hold_protection_limits(
            control_command, effective_mode,
            profile_endpoint_phase[1] || position_endpoint_reached[1] ||
                position_arrived_once[1]);
    if (options.bus == "j2") {
      const bool j2_active = position_control_requested && j2_pair_ready &&
          control_authority_available &&
          control_command.active_joint_mask[1] && !domain_fault &&
          !j2_sync_fault;
      if (j2_active) {
        const auto target_gains = j2_gain_targets(
            thermally_derated_command, effective_mode, motors[0],
            profile_endpoint_phase[1] || position_endpoint_reached[1] ||
                position_arrived_once[1]);
        const double target_kp = target_gains.first;
        const double target_kd = target_gains.second;
        if (!j2_gain_ready) {
          j2_gain_kp = std::min(target_kp, kJ2BaseKp);
          j2_gain_kd = std::min(target_kd, kJ2BaseKd);
          j2_gain_ready = true;
        } else {
          const double kp_step =
              (kKpLimits[1] - kJ2BaseKp) * kPeriod / kJ2KpRampSeconds;
          const double kd_step =
              (kKdLimits[1] - kJ2BaseKd) * kPeriod / kJ2KpRampSeconds;
          j2_gain_kp = j2_gain_kp > target_kp
              ? target_kp : std::min(target_kp, j2_gain_kp + kp_step);
          j2_gain_kd = j2_gain_kd > target_kd
              ? target_kd : std::min(target_kd, j2_gain_kd + kd_step);
        }
        j2_effective_kp = j2_gain_kp;
        j2_effective_kd = j2_gain_kd;
        const double q_a = -1.0 * (motors[0].unwrapped - motors[0].reference) / kGear;
        const double q_b = +1.0 * (motors[1].unwrapped - motors[1].reference) / kGear;
        const double q_measured = 0.5 * (q_a + q_b);
        const double dq_a = -motors[0].last_dq / kGear;
        const double dq_b = +motors[1].last_dq / kGear;
        const double dq_measured = 0.5 * (dq_a + dq_b);
        const double error = q_command[1] - q_measured;
        const bool j2_integral_learning_phase =
            joint_uses_fixed_hold_target(
                control_command, effective_mode, 1U) ||
            profile_endpoint_phase[1] || position_endpoint_reached[1] ||
            position_arrived_once[1];
        if (control_command.recovery)
          reset_bounded_hold_integral(hold_integral_states[1]);
        j2_integral_wire_nm = freezes_assisted_teach_integral(
                control_command, effective_mode, 1U, command_safety, teach_cycle_ns)
            ? frozen_hold_integral_wire(hold_integral_states[1], kJ2IntegralRotorHardNm)
            : update_bounded_hold_integral(
            hold_integral_states[1], true,
            !lease_safe_hold_active && !control_command.recovery &&
                thermal_derating_factor >= 1.0 - 1e-12 &&
                j2_integral_learning_phase &&
                std::all_of(
                    motors.begin(), motors.end(),
                    [](const MotorRuntime& motor) {
                      return motor.consecutive_invalid == 0;
                    }),
            error, j2_derived_velocity_filtered,
            kJ2IntegralRotorHardNm, kJ2IntegralKiPerRotorRadS,
            kJ2IntegralRateHardNmS, kJ2IntegralEnterError,
            kJ2IntegralEnterVelocity, kJ2IntegralDeadband,
            kJ2IntegralDwellFrames);
        hold_integral_wire_nm[1] = j2_integral_wire_nm;

        const double predicted_work_limit = j2_hold_protection
            ? kJ2RecoveryPredictedRotorWorkNm : kJ2PredictedRotorWorkNm;
        const double predicted_pd_limit = j2_hold_protection
            ? kJ2RecoveryPredictedRotorPdHardNm : kJ2PredictedRotorPdHardNm;
        J2GovernedReference governed = govern_j2_reference(
            motors, q_measured, dq_measured, q_command[1], dq_command[1],
            j2_effective_kp, j2_effective_kd,
            applied_gravity_feedforward_nm[1] + j2_integral_wire_nm + teach_damping_nm[1],
            predicted_work_limit, predicted_pd_limit);
        bool internal_fallback = false;
        if (!governed.feasible) {
          // Prediction saturation is recoverable and must never withdraw the
          // gravity-bearing FOC authority. Preserve the learned load bias,
          // remove damping, and rebase only the wire reference this cycle.
          j2_effective_kd = 0.0;
          j2_gain_kd = 0.0;
          governed = govern_j2_reference(
              motors, q_measured, dq_measured, q_command[1], dq_command[1],
              j2_effective_kp, j2_effective_kd,
              applied_gravity_feedforward_nm[1] + j2_integral_wire_nm + teach_damping_nm[1],
              predicted_work_limit, predicted_pd_limit);
          internal_fallback = true;
        }
        if ((effective_mode == "teach" || teach_stopping_hold) && !governed.feasible)
          teach_governor_infeasible = true;
        if (governed.feasible) {
          const bool exact_recipe = (effective_mode == "position" && control_command.quintic.present) ||
              control_command.hand_guidance.present;
          // alpha==1 authorizes the original signed planner sample. Avoid
          // rebuilding it as measured + (target - measured), which can differ
          // from the hashed target by one floating-point rounding step.
          j2_wire_q_command = exact_recipe && governed.alpha == 1.0
              ? q_command[1] : governed.q;
          j2_wire_dq_command = exact_recipe && governed.alpha == 1.0
              ? dq_command[1] : governed.dq;
        } else {
          // Valid, synchronized feedback should make the measured common
          // state feasible. Keep FOC and the measured state if an internal
          // arithmetic inconsistency is ever observed; real drive/thermal/
          // communication/torque faults remain fail-closed below.
          j2_effective_kp = std::min(j2_effective_kp, kJ2BaseKp);
          j2_effective_kd = 0.0;
          j2_gain_kp = j2_effective_kp;
          j2_gain_kd = 0.0;
          j2_wire_q_command = q_measured;
          j2_wire_dq_command = dq_measured;
          internal_fallback = true;
        }
        // An exact v1.3 recipe is immutable. Even a sub-nanoradian governor
        // interpolation changes the signed q/dq sample covered by the
        // preview hash, so there is deliberately no epsilon here.
        const bool limited = internal_fallback ||
            !governed.feasible || governed.alpha != 1.0;
        if (limited && !j2_governor_limited) {
          std::cerr << "J2_GOVERNOR_LIMITED"
                    << " alpha=" << governed.alpha
                    << " fallback=" << internal_fallback
                    << " target_rad=" << control_command.targets[1]
                    << " measured_rad=" << q_measured
                    << std::endl;
        } else if (!limited && j2_governor_limited) {
          std::cerr << "J2_GOVERNOR_RECOVERED"
                    << " target_rad=" << control_command.targets[1]
                    << " measured_rad=" << q_measured
                    << std::endl;
        }
        j2_governor_limited = limited;
      } else {
        j2_integral_wire_nm = update_bounded_hold_integral(
            hold_integral_states[1], false, false, 0.0, 0.0,
            kJ2IntegralRotorHardNm, kJ2IntegralKiPerRotorRadS,
            kJ2IntegralRateHardNmS, kJ2IntegralEnterError,
            kJ2IntegralEnterVelocity, kJ2IntegralDeadband,
            kJ2IntegralDwellFrames);
        j2_gain_kp = 0.0;
        j2_gain_kd = 0.0;
        j2_gain_ready = false;
        j2_governor_limited = false;
      }
    }

    std::array<bool, 6> software_saturation_joint_mask{};
    if (effective_mode == "position") {
      if (options.bus == "j2") {
        software_saturation_joint_mask[1] =
            control_command.active_joint_mask[1] &&
            control_command.moving_joint_mask[1] && j2_governor_limited;
      } else {
        for (const auto& motor : motors) {
          const auto joint = static_cast<std::size_t>(motor.joint_index);
          software_saturation_joint_mask[joint] =
              control_command.active_joint_mask[joint] &&
              control_command.moving_joint_mask[joint] &&
              aux_governor_limited[joint];
        }
      }
    }
    bool software_saturation_observed_this_cycle = std::any_of(
        software_saturation_joint_mask.begin(),
        software_saturation_joint_mask.end(),
        [](bool saturated) { return saturated; });
    bool no_progress_observation_valid = false;
    double no_progress_position_error_rad = 0.0;
    bool any_foc_sent = false;
    bool lease_safe_hold_abort_pending = false;
    std::string lease_safe_hold_abort_reason;
    bool motor_domain_brake_this_cycle =
        thermal_interlock.fault_latched ||
        no_progress_watchdog.fault_latched ||
        owned_domain_feedback_requires_brake(motors);
    std::string motor_domain_brake_reason = thermal_interlock.fault_latched
        ? "THERMAL_INTERLOCK_LATCHED"
        : no_progress_watchdog.fault_latched
              ? no_progress_watchdog.trip_reason
        : motor_domain_brake_this_cycle
              ? "PREVIOUS_MOTOR_FEEDBACK_UNHEALTHY" : "";
    if (options.bus != "j2") {
      for (const auto& motor : motors) {
        const std::size_t joint =
            static_cast<std::size_t>(motor.joint_index);
        const bool integral_active = !options.brake_only &&
            position_control_requested && control_authority_available &&
            control_command.active_joint_mask[joint] &&
            motor.reference_ready && motor.valid && !motor.fault_latched &&
            !motor_domain_brake_this_cycle && !domain_fault && !g_stop.load();
        const double measured_q = motor.reference_ready
            ? motor.sign * (motor.unwrapped - motor.reference) / kGear : 0.0;
        const double measured_dq = motor.sign * motor.last_dq / kGear;
        const bool integral_learning_phase =
            joint_uses_fixed_hold_target(
                control_command, effective_mode, joint) ||
            profile_endpoint_phase[joint] ||
            position_endpoint_reached[joint] ||
            position_arrived_once[joint];
        if (control_command.recovery)
          reset_bounded_hold_integral(hold_integral_states[joint]);
        hold_integral_wire_nm[joint] = integral_active && freezes_assisted_teach_integral(
                control_command, effective_mode, joint, command_safety, teach_cycle_ns)
            ? frozen_hold_integral_wire(hold_integral_states[joint], kHoldIntegralRotorHardNm[joint])
            : update_bounded_hold_integral(
            hold_integral_states[joint], integral_active,
            integral_active && !lease_safe_hold_active &&
                !control_command.recovery &&
                thermal_derating_factor >= 1.0 - 1e-12 &&
                integral_learning_phase &&
                motor.consecutive_invalid == 0,
            q_command[joint] - measured_q, motor.integral_encoder_velocity,
            kHoldIntegralRotorHardNm[joint],
            kAuxIntegralKiPerRotorRadS, kAuxIntegralRateHardNmS,
            kAuxIntegralEnterError, kAuxIntegralEnterVelocity,
            kAuxIntegralDeadband, kAuxIntegralDwellFrames);
        if (integral_active) {
          const double effective_kp = std::min(
              thermally_derated_command.kp[joint], motor.kp_limit);
          const double effective_kd = std::min(
              thermally_derated_command.kd[joint], motor.kd_limit);
          const SingleMotorGovernedReference governed =
              govern_single_motor_reference(
                  motor, measured_q, measured_dq,
                  q_command[joint], dq_command[joint],
                  effective_kp, effective_kd,
                  applied_gravity_feedforward_nm[joint] +
                      hold_integral_wire_nm[joint] + teach_damping_nm[joint],
                  kAuxPredictedRotorWorkNm[joint],
                  kAuxPredictedRotorPdHardNm[joint]);
          if ((effective_mode == "teach" || teach_stopping_hold) && !governed.feasible)
            teach_governor_infeasible = true;
          if (governed.feasible) {
            const bool exact_recipe = (effective_mode == "position" && control_command.quintic.present) ||
                control_command.hand_guidance.present;
            aux_wire_q_command[joint] =
                exact_recipe && governed.alpha == 1.0
                ? q_command[joint] : governed.q;
            aux_wire_dq_command[joint] =
                exact_recipe && governed.alpha == 1.0
                ? dq_command[joint] : governed.dq;
          } else {
            // Arithmetic fallback is still FOC: preserve the immutable raw
            // planner endpoint and learned bias, but make only this wire
            // reference equal to fresh measured state for one cycle.
            aux_wire_q_command[joint] = measured_q;
            aux_wire_dq_command[joint] = measured_dq;
          }
          // Preserve the exact previewed q/dq sample at the policy boundary:
          // every non-unity interpolation aborts and requires a new preview.
          const bool limited =
              !governed.feasible || governed.alpha != 1.0;
          if (limited && !aux_governor_limited[joint]) {
            std::cerr << "AUX_GOVERNOR_LIMITED"
                      << " joint=J" << joint + 1U
                      << " alpha=" << governed.alpha
                      << " target_rad=" << control_command.targets[joint]
                      << " measured_rad=" << measured_q << std::endl;
          } else if (!limited && aux_governor_limited[joint]) {
            std::cerr << "AUX_GOVERNOR_RECOVERED"
                      << " joint=J" << joint + 1U
                      << " target_rad=" << control_command.targets[joint]
                      << " measured_rad=" << measured_q << std::endl;
          }
          aux_governor_limited[joint] = limited;
        } else {
          aux_governor_limited[joint] = false;
        }
      }
    }
    if (options.bus != "j2") {
      for (const auto& motor : motors) {
        const auto joint = static_cast<std::size_t>(motor.joint_index);
        software_saturation_joint_mask[joint] =
            control_command.active_joint_mask[joint] &&
            control_command.moving_joint_mask[joint] &&
            aux_governor_limited[joint];
      }
      software_saturation_observed_this_cycle = std::any_of(
          software_saturation_joint_mask.begin(),
          software_saturation_joint_mask.end(),
          [](bool saturated) { return saturated; });
    }
    if ((effective_mode == "teach" || teach_stopping_hold) && teach_governor_infeasible) {
      latch_position_safety_watchdog(
          no_progress_watchdog, 0.0, "ASSISTED_TEACH_LOAD_GOVERNOR_ABORT",
          control_command.activation_epoch, command_safety.minimum_activation_epoch,
          command_safety.highest_rejected_active_epoch);
      command_safety.minimum_activation_epoch = std::max(
          command_safety.minimum_activation_epoch, no_progress_watchdog.minimum_rearm_epoch);
      motor_domain_brake_this_cycle = true;
      motor_domain_brake_reason = no_progress_watchdog.trip_reason;
      effective_mode = "brake";
      prior_external_hold_confirmed = false;
      spend_empirical_gravity_authority(command_safety);
    }
    bool exact_load_governor_abort_latched = false;
    if (effective_mode == "position" && control_command.quintic.present &&
        software_saturation_observed_this_cycle) {
      double exact_governor_position_error_rad = 0.0;
      (void)maximum_moving_owned_position_error(
          control_command, motors, software_saturation_joint_mask,
          exact_governor_position_error_rad);
      reset_no_progress_observation(no_progress_watchdog);
      no_progress_position_error_rad = exact_governor_position_error_rad;
      exact_load_governor_abort_latched = latch_position_safety_watchdog(
          no_progress_watchdog, exact_governor_position_error_rad,
          "EXACT_TRAJECTORY_LOAD_GOVERNOR_ABORT",
          control_command.activation_epoch,
          command_safety.minimum_activation_epoch,
          command_safety.highest_rejected_active_epoch);
      if (exact_load_governor_abort_latched) {
        command_safety.minimum_activation_epoch = std::max(
            command_safety.minimum_activation_epoch,
            no_progress_watchdog.minimum_rearm_epoch);
        motor_domain_brake_this_cycle = true;
        motor_domain_brake_reason = no_progress_watchdog.trip_reason;
        effective_mode = "brake";
        prior_external_hold_confirmed = false;
        std::cerr << "V13_LOAD_GOVERNOR_TRAJECTORY_ABORT"
                  << " bus=" << options.bus
                  << " activation_epoch="
                  << control_command.activation_epoch
                  << " position_error_rad="
                  << exact_governor_position_error_rad
                  << " minimum_epoch="
                  << no_progress_watchdog.minimum_rearm_epoch
                  << " policy=REPREVIEW_REQUIRED"
                  << std::endl;
      }
    }
    std::size_t motor_runtime_index = 0;
    for (auto& motor : motors) {
      int send_mode = kBrakeMode;
      double q = 0.0, dq = 0.0, kp = 0.0, kd = 0.0;
      const bool active_allowed = !options.brake_only &&
          (!active_power_session || j2_startup_verified) &&
          motor.reference_ready && motor.valid &&
          !motor.fault_latched && !thermal_interlock.fault_latched &&
          !no_progress_watchdog.fault_latched &&
          !motor_domain_brake_this_cycle &&
          !(options.bus == "j2" && j2_sync_fault) && !domain_fault && j2_pair_ready &&
          !g_stop.load() && !lease_safe_hold_abort_pending &&
          control_authority_available &&
          control_command.active_joint_mask[
              static_cast<std::size_t>(motor.joint_index)];
      if (is_position_holding_mode(effective_mode) &&
          active_allowed) {
        send_mode = kFocMode;
        const std::size_t joint = static_cast<std::size_t>(motor.joint_index);
        const double wire_q = options.bus == "j2"
            ? j2_wire_q_command : aux_wire_q_command[joint];
        const double wire_dq = options.bus == "j2"
            ? j2_wire_dq_command : aux_wire_dq_command[joint];
        q = motor.reference + motor.sign * kGear * wire_q;
        dq = motor.sign * kGear * wire_dq;
        kp = options.bus == "j2" ? j2_effective_kp
                                  : std::min(
                                        thermally_derated_command.kp[joint],
                                        motor.kp_limit);
        kd = options.bus == "j2" ? j2_effective_kd
                                  : std::min(
                                        thermally_derated_command.kd[joint],
                                        motor.kd_limit);
      }
      const std::size_t joint = static_cast<std::size_t>(motor.joint_index);
      const double tau = send_mode == kFocMode
          ? motor.sign * (applied_gravity_feedforward_nm[joint] +
              hold_integral_wire_nm[joint] + teach_damping_nm[joint])
          : 0.0;
      send_mode = enforce_brake_only_wire_mode(options.brake_only, send_mode);
      motor.last_tau_cmd_rotor_nm = send_mode == kFocMode ? tau : 0.0;
      MotorCmd packet = make_command(motor.id, send_mode, q, dq, kp, kd, tau);
      any_foc_sent = any_foc_sent || send_mode == kFocMode;
      const Feedback feedback = transact(
          *serial, packet, motor.id, send_mode, tx_audit);
      if (options.bus == "j2") {
        auto& log = invalid_feedback_logs[motor_runtime_index];
        const auto diagnostic_now = Clock::now();
        if (!feedback.continuity_valid) {
          const InvalidFeedbackSnapshot snapshot = invalid_feedback_snapshot(
              cycles, send_mode, motor.consecutive_invalid, feedback);
          if (!log.active) {
            log.active = true;
            log.episode_count = 0;
            ++log.transition_count;
            ++log.pending_transition_count;
          }
          ++log.episode_count;
          ++log.total_count;
          ++log.pending_count;
          if (!log.pending_snapshot) {
            log.first = snapshot;
            log.pending_snapshot = true;
          }
          log.last = snapshot;
          if (log.total_count == 1U) {
            log_invalid_feedback_episode(
                "J2_INVALID_FEEDBACK_BEGIN", motor.name, log);
            log.pending_count = 0U;
            log.pending_transition_count = 0U;
            log.pending_snapshot = false;
            log.last_report_at = diagnostic_now;
          } else {
            flush_invalid_feedback_summary(
                "J2_INVALID_FEEDBACK_SUMMARY", motor.name, log,
                diagnostic_now, false);
          }
        } else if (log.active) {
          log.active = false;
          log.episode_count = 0;
          ++log.transition_count;
          ++log.pending_transition_count;
        }
        flush_invalid_feedback_summary(
            "J2_INVALID_FEEDBACK_SUMMARY", motor.name, log,
            diagnostic_now, false);
      }
      const bool sustained_invalid = observe_feedback_frame_validity(
          motor, feedback.continuity_valid,
          invalid_feedback_limit_for_bus(options.bus));
      if (sustained_invalid)
        j2_power_continuity_lost = true;
      if (options.bus == "j2" && sustained_invalid) j2_pair_ready = false;
      if (feedback.identity_ok) {
        motor.temperature = feedback.data.temp;
        motor.merror = feedback.data.merror;
        motor.returned_mode = feedback.data.mode;
        // A returned-mode or numeric mismatch is represented by
        // continuity_valid and becomes a hard communication fault only after
        // the consecutive-invalid threshold. Drive errors remain an immediate
        // non-thermal latch; temperature has its own live BRAKE interlock.
        if (feedback.data.merror != 0 || feedback.data.temp < 0) {
          motor.non_transport_fault_latched = true;
          motor.fault_latched = true;
        }
        const bool newly_latched = observe_raw_temperature_thermal_trip(
            thermal_interlock, feedback.data.temp,
            control_command.activation_epoch,
            command_safety.minimum_activation_epoch,
            command_safety.highest_rejected_active_epoch);
        if (feedback.data.temp >= g_thermal_policy.thermal_stop_c) {
          command_safety.minimum_activation_epoch = std::max(
              command_safety.minimum_activation_epoch,
              thermal_interlock.minimum_rearm_epoch);
          publish_thermal_latch_to_motors(motors, true);
          motor_domain_brake_this_cycle = true;
          motor_domain_brake_reason = "MOTOR_RAW_TEMPERATURE_THERMAL_LATCH";
          effective_mode = "brake";
          prior_external_hold_confirmed = false;
          position_tracking.fill(false);
          position_tracking_epoch.fill(0U);
          position_endpoint_reached.fill(false);
          position_arrived_once.fill(false);
          position_arrival_overdue.fill(false);
          position_arrival_qualifying_frames.fill(0);
          position_arrival_window_started_at.fill(Clock::time_point{});
          dq_command.fill(0.0);
          for (auto& state : hold_integral_states)
            reset_bounded_hold_integral(state);
          hold_integral_wire_nm.fill(0.0);
          j2_integral_wire_nm = 0.0;
          j2_gain_kp = 0.0;
          j2_gain_kd = 0.0;
          j2_gain_ready = false;
          j2_governor_limited = false;
          aux_governor_limited.fill(false);
          if (options.bus == "j2") j2_pair_ready = false;
          if (lease_safe_hold_active) {
            if (!lease_safe_hold_abort_pending)
              lease_safe_hold_abort_reason = "THERMAL_INTERLOCK";
            lease_safe_hold_abort_pending = true;
          }
          if (newly_latched) {
            std::cerr << "THERMAL_TRIP"
                      << " bus=" << options.bus
                      << " motor=" << motor.name
                      << " raw_temperature_c=" << feedback.data.temp
                      << " trip_epoch="
                      << thermal_interlock.trip_activation_epoch
                      << " minimum_epoch="
                      << thermal_interlock.minimum_rearm_epoch
                      << std::endl;
          }
        }
      }
      if (feedback.continuity_valid) {
        motor.last_q = feedback.data.q;
        motor.last_dq = feedback.data.dq;
        motor.last_tau = feedback.data.tau;
        motor.last_valid_feedback_monotonic_ns = monotonic_ns();
        motor.unwrapped = motor.unwrap.update(motor.last_q);
        if (active_power_session && !j2_startup_verified) {
          if (send_mode != kBrakeMode)
            throw std::runtime_error(j2_active_session
                ? "J2_STARTUP_RECHECK_NON_BRAKE_MODE"
                : "GO_AUX_STARTUP_RECHECK_NON_BRAKE_MODE");
          motor.j2_startup_brake_history.push_back(motor.unwrapped);
          while (motor.j2_startup_brake_history.size() >
                 j2_launch_permit.minimum_brake_frames)
            motor.j2_startup_brake_history.pop_front();
        }
        const auto feedback_at = Clock::now();
        const double scaled_position = motor.sign * motor.unwrapped / kGear;
        motor.integral_encoder_velocity = observed_integral_encoder_velocity(
            motor, scaled_position, feedback_at);
        // Fixed HOLD and endpoint-phase POSITION paths interpret motion as
        // position error to recover, not as a reason to withdraw torque.
        // Velocity guards apply only while the commanded profile is advancing.
        // Communication, returned-mode, drive, temperature,
        // mechanical-envelope and J2 sync checks remain.
        if (send_mode == kFocMode && motor.speed_ready &&
            should_apply_velocity_guard(
                control_command, effective_mode, joint,
                profile_endpoint_phase[joint] ||
                    position_endpoint_reached[joint] ||
                    position_arrived_once[joint])) {
          const double dt = std::chrono::duration<double>(
              feedback_at - motor.previous_feedback_at).count();
          if (dt > 1e-4 && dt < 0.5)
            apply_velocity_guards(
                options.bus, motor,
                (scaled_position - motor.previous_scaled_position) / dt);
        } else {
          motor.fast_speed_count = 0;
          motor.slow_speed_count = 0;
          if (motor.velocity_degraded) {
            std::cerr << "EXTERNAL_MOTION_SETTLED"
                      << " bus=" << options.bus
                      << " motor=" << motor.name
                      << " reason=velocity_observer_inactive"
                      << std::endl;
            motor.velocity_degraded = false;
          }
        }
        motor.previous_scaled_position = scaled_position;
        motor.previous_feedback_at = feedback_at;
        motor.speed_ready = true;
        if (!motor.reference_ready && motor.session_reference_configured) {
          motor.reference = reference_for_j2_session(motor.unwrapped, motor);
          motor.reference_ready = true;
          std::cerr << (j2_active_session
                            ? "J2_SESSION_REFERENCE motor="
                            : "GO_AUX_SESSION_REFERENCE motor=")
                    << motor.name
                    << " session_hint_logical_rad="
                    << motor.session_logical_position
                    << " unwrapped_raw_rad=" << motor.unwrapped
                    << " session_reference_raw_rad="
                    << motor.session_reference
                    << " selected_reference_raw_rad=" << motor.reference
                    << " recovered_logical_rad="
                    << motor.sign * (motor.unwrapped - motor.reference) / kGear
                    << std::endl;
        } else if (!motor.reference_ready &&
                   motor.persistent_reference_configured &&
                   !options.brake_only) {
          motor.reference = motor.recovery_hint_configured
              ? reference_for_recovery_branch(motor.unwrapped, motor)
              : motor.persistent_reference +
                  std::round((motor.unwrapped - motor.persistent_reference) /
                             (2.0 * kPi)) * (2.0 * kPi);
          motor.reference_ready = true;
          std::cerr << "RECOVERY_REFERENCE motor=" << motor.name
                    << " hint_configured=" << motor.recovery_hint_configured
                    << " hint_logical_rad=" << motor.recovery_hint
                    << " unwrapped_raw_rad=" << motor.unwrapped
                    << " persistent_raw_rad=" << motor.persistent_reference
                    << " selected_reference_raw_rad=" << motor.reference
                    << " recovered_logical_rad="
                    << motor.sign * (motor.unwrapped - motor.reference) / kGear
                    << std::endl;
        } else if (!motor.reference_ready && send_mode == kBrakeMode) {
          motor.history.push_back(motor.unwrapped);
          while (motor.history.size() > 50U) motor.history.pop_front();
          if (motor.history.size() == 50U) {
            const auto minmax = std::minmax_element(motor.history.begin(), motor.history.end());
            std::deque<double> tail(motor.history.end() - 10, motor.history.end());
            const double reference = median(motor.history);
            if ((*minmax.second - *minmax.first) / kGear <= 0.20 * kPi / 180.0 &&
                std::abs(median(tail) - reference) / kGear <= 0.10 * kPi / 180.0) {
              motor.reference = reference;
              motor.reference_ready = true;
            }
          }
        }
        if (motor.reference_ready) {
          const double logical = motor.sign * (motor.unwrapped - motor.reference) / kGear;
          if (!within_mechanical_feedback_envelope(motor.joint_index, logical)) {
            motor.non_transport_fault_latched = true;
            motor.fault_latched = true;
          }
        }
      }
      if (motor_feedback_requires_domain_brake(motor)) {
        const std::string fault_reason = !motor.valid
            ? "MOTOR_COMMUNICATION_OR_RETURNED_MODE"
            : motor.merror != 0 ? "MOTOR_DRIVE_ERROR"
            : (motor.temperature < 0 ||
               motor.temperature >= g_thermal_policy.thermal_stop_c)
                  ? "MOTOR_TEMPERATURE" : "MOTOR_FAULT_LATCH";
        if (!motor_domain_brake_this_cycle)
          motor_domain_brake_reason = fault_reason;
        motor_domain_brake_this_cycle = true;
        if (lease_safe_hold_active) {
          if (!lease_safe_hold_abort_pending)
            lease_safe_hold_abort_reason = fault_reason;
          lease_safe_hold_abort_pending = true;
        }
      }
      if (options.bus == "j2" && motor.fault_latched) j2_pair_ready = false;
      ++motor_runtime_index;
    }

    const bool thermal_cooldown_frame_qualified =
        all_domain_motors_thermal_cooldown_qualified(motors);
    if (observe_thermal_cooldown_frame(
            thermal_interlock, thermal_cooldown_frame_qualified,
            Clock::now())) {
      std::cerr << "THERMAL_COOLDOWN_READY"
                << " bus=" << options.bus
                << " valid_brake_frames="
                << thermal_interlock.cooldown_frames
                << " minimum_seconds="
                << g_thermal_policy.cooldown_seconds
                << std::endl;
    }

    if (j2_power_continuity_lost) {
      if (command_socket >= 0) {
        ::close(command_socket);
        command_socket = -1;
      }
      const std::uint64_t lost_epoch = command.activation_epoch;
      command_safety.highest_rejected_active_epoch = std::max(
          command_safety.highest_rejected_active_epoch, lost_epoch);
      command_safety.minimum_activation_epoch = std::max(
          command_safety.minimum_activation_epoch,
          saturating_next_activation_epoch(lost_epoch));
      command = GuiCommand{};
      command_receive_state = CommandReceiveState{};
      previous_mode = "brake";
      previous_active_joint_mask.fill(false);
      previous_moving_joint_mask.fill(false);
      q_command.fill(0.0);
      dq_command.fill(0.0);
      prior_external_hold_confirmed = false;
      lease_safe_hold_active = false;
      lease_safe_hold_abort_pending = false;
      position_tracking.fill(false);
      position_tracking_epoch.fill(0U);
      position_endpoint_reached.fill(false);
      position_arrived_once.fill(false);
      position_arrival_overdue.fill(false);
      const double recovery_phase_limit = j2_launch_permit.maximum_raw_phase_delta_rad;
      if (!recover_go_transport_in_brake(
              definition, serial, motors, options.bus,
              recovery_phase_limit, tx_audit,
              communication_recovery_attempt_count,
              communication_recovery_success_count)) {
        break;
      }
      j2_power_continuity_lost = false;
      j2_pair_ready = options.bus != "j2" || motors.size() == 2U;
      if (!options.brake_only) {
        command_socket = open_command_socket(definition);
        std::cerr << "COMMUNICATION_COMMAND_RECEIVER_REOPENED"
                  << " bus=" << options.bus
                  << " command_port=" << definition.command_port
                  << " minimum_activation_epoch="
                  << command_safety.minimum_activation_epoch
                  << std::endl;
      }
      continue;
    }
    if (active_power_session && !j2_startup_verified) {
      const bool histories_ready = std::all_of(
          motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
            return motor.valid && motor.reference_ready &&
                motor.j2_startup_brake_history.size() ==
                    j2_launch_permit.minimum_brake_frames;
          });
      if (histories_ready) {
        bool recheck_ok = true;
        for (const auto& motor : motors) {
          const auto minmax = std::minmax_element(
              motor.j2_startup_brake_history.begin(),
              motor.j2_startup_brake_history.end());
          const double phase_center = median(motor.j2_startup_brake_history);
          const double phase_delta = std::abs(std::remainder(
              phase_center - motor.session_capture_raw_position, 2.0 * kPi));
          const double span = *minmax.second - *minmax.first;
          const double logical =
              motor.sign * (motor.unwrapped - motor.reference) / kGear;
          recheck_ok = recheck_ok && std::isfinite(phase_center) &&
              std::isfinite(phase_delta) && std::isfinite(span) &&
              std::isfinite(logical) &&
              phase_delta <=
                  j2_launch_permit.maximum_raw_phase_delta_rad + 1e-12 &&
              span <= j2_launch_permit.maximum_raw_span_rad + 1e-12 &&
              std::abs(logical - motor.session_startup_logical_position) <=
                  0.25 * kPi / 180.0 + 1e-12;
        }
        if (j2_active_session) {
          const double q_a =
              -1.0 * (motors[0].unwrapped - motors[0].reference) / kGear;
          const double q_b =
              +1.0 * (motors[1].unwrapped - motors[1].reference) / kGear;
          recheck_ok = recheck_ok &&
              std::abs(q_a - q_b) <= kJ2SyncLimit + 1e-12;
        }
        if (!recheck_ok)
          throw std::runtime_error(j2_active_session
              ? "J2_STARTUP_BRAKE_RECHECK_FAILED"
              : "GO_AUX_STARTUP_BRAKE_RECHECK_FAILED");
        spend_j2_launch_permit(j2_launch_permit);
        command_socket = open_command_socket(definition);
        j2_startup_verified = true;
        std::cerr << (j2_active_session
                          ? "J2_STARTUP_BRAKE_RECHECK_PASS"
                          : "GO_AUX_STARTUP_BRAKE_RECHECK_PASS")
                  << " frames=" << j2_launch_permit.minimum_brake_frames
                  << " permit_id=" << j2_launch_permit.permit_id
                  << " command_port=" << definition.command_port
                  << std::endl;
      }
    }

    if (std::any_of(
            motors.begin(), motors.end(), [](const MotorRuntime& motor) {
              return motor.fault_latched;
            })) {
      latch_domain_fault("OWNED_MOTOR_INVALID_OR_LATCHED");
    }
    auto transact_and_commit_brake = [&](MotorRuntime& motor) {
      motor.last_tau_cmd_rotor_nm = 0.0;
      MotorCmd brake = make_command(
          motor.id, kBrakeMode, 0.0, 0.0, 0.0, 0.0);
      const Feedback brake_feedback = transact(
          *serial, brake, motor.id, kBrakeMode, tx_audit);
      const bool sustained_invalid = observe_feedback_frame_validity(
          motor, brake_feedback.continuity_valid,
          invalid_feedback_limit_for_bus(options.bus));
      if (sustained_invalid)
        j2_power_continuity_lost = true;
      if (brake_feedback.identity_ok) {
        motor.temperature = brake_feedback.data.temp;
        motor.merror = brake_feedback.data.merror;
        motor.returned_mode = brake_feedback.data.mode;
        if (brake_feedback.data.merror != 0 ||
            brake_feedback.data.temp < 0) {
          motor.non_transport_fault_latched = true;
          motor.fault_latched = true;
        }
        if (observe_raw_temperature_thermal_trip(
                thermal_interlock, brake_feedback.data.temp,
                control_command.activation_epoch,
                command_safety.minimum_activation_epoch,
                command_safety.highest_rejected_active_epoch)) {
          command_safety.minimum_activation_epoch = std::max(
              command_safety.minimum_activation_epoch,
              thermal_interlock.minimum_rearm_epoch);
          std::cerr << "THERMAL_TRIP_ON_BRAKE_FEEDBACK"
                    << " bus=" << options.bus
                    << " motor=" << motor.name
                    << " raw_temperature_c="
                    << brake_feedback.data.temp
                    << std::endl;
        }
      }
      if (brake_feedback.continuity_valid) {
        motor.last_q = brake_feedback.data.q;
        motor.last_dq = brake_feedback.data.dq;
        motor.last_tau = brake_feedback.data.tau;
        motor.last_valid_feedback_monotonic_ns = monotonic_ns();
        motor.unwrapped = motor.unwrap.update(motor.last_q);
      }
      publish_thermal_latch_to_motors(
          motors, thermal_interlock.fault_latched);
    };
    if (motor_domain_brake_this_cycle) {
      if (any_foc_sent)
        record_domain_brake_event(
            options.bus, motor_domain_brake_reason,
            domain_brake_log, Clock::now());
      effective_mode = "brake";
      if (options.bus == "j2") j2_pair_ready = false;
      for (auto& motor : motors) transact_and_commit_brake(motor);
    }
    if (options.bus == "j2" && motors.size() == 2U &&
        motors[0].reference_ready && motors[1].reference_ready &&
        motors[0].valid && motors[1].valid &&
        motors[0].last_frame_valid && motors[1].last_frame_valid) {
      const double q_a = -1.0 * (motors[0].unwrapped - motors[0].reference) / kGear;
      const double q_b = +1.0 * (motors[1].unwrapped - motors[1].reference) / kGear;
      // This is one valid J2A/J2B paired observation from the current control
      // cycle.  The contract is strict: warning is live above 0.25 degree and
      // any one pair above 0.5 degree latches BOTH motors in BRAKE immediately.
      // A hard latch never clears merely because later samples look aligned.
      const double sync_error = q_a - q_b;
      const bool previous_sync_fault = j2_sync_fault;
      const bool previous_sync_warning = j2_sync_filter.warning;
      const bool previous_recovery_ready = j2_sync_filter.recovery_ready;
      j2_sync_fault = observe_j2_sync_error(
          j2_sync_filter, sync_error,
          control_command.activation_epoch,
          command_safety.minimum_activation_epoch,
          command_safety.highest_rejected_active_epoch);
      if (j2_sync_filter.warning && !previous_sync_warning) {
        std::cerr << "J2_SYNC_WARNING"
                  << " error_deg=" << sync_error * 180.0 / kPi
                  << " warning_threshold_deg=0.25"
                  << std::endl;
      } else if (!j2_sync_filter.warning && previous_sync_warning) {
        std::cerr << "J2_SYNC_WARNING_CLEARED"
                  << " error_deg=" << sync_error * 180.0 / kPi
                  << " hard_latch_retained=" << j2_sync_fault
                  << std::endl;
      }
      if (j2_sync_fault && !previous_sync_fault) {
        command_safety.minimum_activation_epoch = std::max(
            command_safety.minimum_activation_epoch,
            j2_sync_filter.minimum_rearm_epoch);
        std::cerr << "J2_SYNC_HARD_LATCHED"
                  << " error_deg=" << sync_error * 180.0 / kPi
                  << " hard_threshold_deg=0.5"
                  << " action=BOTH_BRAKE"
                  << " activation_epoch="
                  << j2_sync_filter.trip_activation_epoch
                  << " minimum_epoch="
                  << j2_sync_filter.minimum_rearm_epoch
                  << std::endl;
      }
      if (j2_sync_filter.recovery_ready && !previous_recovery_ready) {
        std::cerr << "J2_SYNC_RECOVERY_READY"
                  << " error_deg=" << sync_error * 180.0 / kPi
                  << " valid_pair_frames="
                  << j2_sync_filter.recovery_frames
                  << " latch_retained=YES"
                  << " next=EXPLICIT_RELEASE_HIGHER_EPOCH"
                  << std::endl;
      }
      if (j2_sync_fault) {
        effective_mode = "brake";
        j2_pair_ready = false;
        for (auto& motor : motors) transact_and_commit_brake(motor);
      }
      const double q_common = 0.5 * (q_a + q_b);
      if (j2_previous_common_ready) {
        const double derived_velocity =
            (q_common - j2_previous_common_position) / kPeriod;
        j2_derived_velocity_filtered += kJ2DerivedVelocityFilterAlpha *
            (derived_velocity - j2_derived_velocity_filtered);
      } else {
        j2_derived_velocity_filtered = 0.0;
        j2_previous_common_ready = true;
      }
      j2_previous_common_position = q_common;
      const double feedback_torque_limit = j2_hold_protection
          ? kJ2RecoveryRotorTorqueFeedbackHardNm
          : kJ2RotorTorqueFeedbackHardNm;
      const bool torque_feedback_saturated = any_foc_sent &&
          (std::abs(motors[0].last_tau) >= feedback_torque_limit ||
           std::abs(motors[1].last_tau) >= feedback_torque_limit);
      // This is a configured software working-clamp observation, explicitly
      // not continuous-torque authority.  A single saturated sample is not a
      // drive failure; sustained saturation plus large position error and no
      // progress is consumed by the domain watchdog below.
      if (effective_mode == "position" &&
          control_command.active_joint_mask[1] &&
          control_command.moving_joint_mask[1]) {
        software_saturation_joint_mask[1] =
            software_saturation_joint_mask[1] || torque_feedback_saturated;
        software_saturation_observed_this_cycle =
            software_saturation_observed_this_cycle ||
            torque_feedback_saturated;
      }
      if (torque_feedback_saturated &&
          !j2_torque_feedback_saturated_reported) {
        std::cerr << "J2_TORQUE_FEEDBACK_SOFTWARE_SATURATION_OBSERVED"
                  << " tau_a_rotor_nm=" << motors[0].last_tau
                  << " tau_b_rotor_nm=" << motors[1].last_tau
                  << " software_limit_rotor_nm=" << feedback_torque_limit
                  << " authority=" << kLoadLimitWatchdogAuthority
                  << std::endl;
      } else if (!torque_feedback_saturated &&
                 j2_torque_feedback_saturated_reported) {
        std::cerr << "J2_TORQUE_FEEDBACK_SOFTWARE_SATURATION_RECOVERED"
                  << std::endl;
      }
      j2_torque_feedback_saturated_reported = torque_feedback_saturated;
      if (any_foc_sent) {
        const double expected_a_pd = j2_effective_kp *
            (motors[0].reference - kGear * j2_wire_q_command -
             motors[0].unwrapped);
        const double expected_b_pd = j2_effective_kp *
            (motors[1].reference + kGear * j2_wire_q_command -
             motors[1].unwrapped);
        const double predicted_pd_limit = j2_hold_protection
            ? kJ2RecoveryPredictedRotorPdHardNm : kJ2PredictedRotorPdHardNm;
        const bool postcheck_exceeded =
            std::abs(expected_a_pd) > predicted_pd_limit + 1e-9 ||
            std::abs(expected_b_pd) > predicted_pd_limit + 1e-9;
        if (postcheck_exceeded && !j2_governor_postcheck_reported) {
          std::cerr << "J2_GOVERNOR_POSTCHECK_LIMITED"
                    << " expected_a_pd_nm=" << expected_a_pd
                    << " expected_b_pd_nm=" << expected_b_pd
                    << " limit_nm=" << predicted_pd_limit
                    << std::endl;
        } else if (!postcheck_exceeded &&
                   j2_governor_postcheck_reported) {
          std::cerr << "J2_GOVERNOR_POSTCHECK_RECOVERED" << std::endl;
        }
        j2_governor_postcheck_reported = postcheck_exceeded;
      }
    } else if (options.bus == "j2" && motors.size() == 2U) {
      const bool previous_sync_warning = j2_sync_filter.warning;
      j2_sync_fault = observe_j2_sync_unavailable(j2_sync_filter);
      if (previous_sync_warning) {
        std::cerr << "J2_SYNC_WARNING_OBSERVATION_UNAVAILABLE"
                  << " hard_latch_retained=" << j2_sync_fault
                  << std::endl;
      }
    }
    if (effective_mode == "position") {
      std::set<int> owned_joints;
      for (const auto& motor : motors) owned_joints.insert(motor.joint_index);
      for (const int joint : owned_joints) {
        const std::size_t index = static_cast<std::size_t>(joint);
        const auto arrival_now = Clock::now();
        if (!position_tracking[index] ||
            !control_command.active_joint_mask[index]) {
          position_endpoint_reached[index] = false;
          position_arrived_once[index] = false;
          position_arrival_overdue[index] = false;
          position_arrival_qualifying_frames[index] = 0;
          position_arrival_window_started_at[index] = Clock::time_point{};
          continue;
        }
        const bool endpoint = profile_endpoint_phase[index];
        if (endpoint && !position_endpoint_reached[index]) {
          position_started_at[index] = arrival_now;
          position_arrived_once[index] = false;
          position_arrival_qualifying_frames[index] = 0;
          position_arrival_window_started_at[index] = Clock::time_point{};
        }
        position_endpoint_reached[index] = endpoint;
        double actual = 0.0;
        const bool within_arrival_window =
            healthy_logical_position_for_joint(motors, joint, actual) &&
            std::abs(actual - control_command.targets[index]) <=
                kArrivalTolerance;
        const bool was_arrived = position_arrived_once[index];
        const bool stable_arrival = observe_position_arrival_dwell(
            endpoint, within_arrival_window, arrival_now,
            position_arrival_qualifying_frames[index],
            position_arrival_window_started_at[index],
            position_arrived_once[index]);
        if (was_arrived && !stable_arrival)
          std::cerr << "POSITION_ARRIVAL_STABILITY_LOST"
                    << " bus=" << options.bus
                    << " joint=" << joint
                    << std::endl;
        if (stable_arrival && !was_arrived) {
          if (position_arrival_overdue[index])
            std::cerr << "POSITION_ARRIVAL_RECOVERED"
                      << " bus=" << options.bus
                      << " joint=" << joint
                      << std::endl;
          position_arrived_once[index] = true;
          position_arrival_overdue[index] = false;
        }
      }
    }
    bool position_arrival_timeout_detected = false;
    double position_arrival_timeout_error_rad = 0.0;
    if (effective_mode == "position" && options.bus == "j2" &&
        position_tracking[1] &&
        !position_arrived_once[1] &&
        position_endpoint_reached[1] &&
        std::chrono::duration<double>(Clock::now() - position_started_at[1]).count() >=
            kTargetTimeoutSeconds && motors.size() == 2U &&
          motors[0].reference_ready && motors[1].reference_ready) {
        const double q_a = -1.0 * (motors[0].unwrapped - motors[0].reference) / kGear;
        const double q_b = +1.0 * (motors[1].unwrapped - motors[1].reference) / kGear;
        const double arrival_error = std::abs(
            0.5 * (q_a + q_b) - control_command.targets[1]);
        if (!position_arrival_overdue[1]) {
          std::cerr << "POSITION_ARRIVAL_OVERDUE"
                    << " bus=j2 joint=1"
                    << " position_error_rad=" << arrival_error
                    << std::endl;
          position_arrival_overdue[1] = true;
          position_arrival_timeout_detected = true;
          position_arrival_timeout_error_rad = arrival_error;
        }
    } else if (effective_mode == "position" && options.bus != "j2") {
      for (auto& motor : motors) {
        const std::size_t index = static_cast<std::size_t>(motor.joint_index);
        if (!position_tracking[index] || position_arrived_once[index] ||
            !position_endpoint_reached[index] ||
            std::chrono::duration<double>(Clock::now() - position_started_at[index]).count() <
                kTargetTimeoutSeconds ||
            !motor.reference_ready || motor.fault_latched)
          continue;
        const double logical = motor.sign * (motor.unwrapped - motor.reference) / kGear;
        const double arrival_error = std::abs(
            logical - control_command.targets[index]);
        if (!position_arrival_overdue[index]) {
          std::cerr << "POSITION_ARRIVAL_OVERDUE"
                    << " bus=" << options.bus
                    << " joint=" << motor.joint_index
                    << " motor=" << motor.name
                    << " position_error_rad=" << arrival_error
                    << std::endl;
          position_arrival_overdue[index] = true;
          position_arrival_timeout_detected = true;
          position_arrival_timeout_error_rad = std::max(
              position_arrival_timeout_error_rad, arrival_error);
        }
      }
    }
    bool position_arrival_timeout_latched = false;
    if (position_arrival_timeout_detected) {
      reset_no_progress_observation(no_progress_watchdog);
      no_progress_position_error_rad = position_arrival_timeout_error_rad;
      position_arrival_timeout_latched = latch_position_safety_watchdog(
          no_progress_watchdog, position_arrival_timeout_error_rad,
          "POSITION_ARRIVAL_TIMEOUT", control_command.activation_epoch,
          command_safety.minimum_activation_epoch,
          command_safety.highest_rejected_active_epoch);
    }
    no_progress_observation_valid = effective_mode == "position" &&
        any_foc_sent && !domain_fault && !j2_sync_fault &&
        !thermal_interlock.fault_latched &&
        !no_progress_watchdog.fault_latched &&
        (!control_command.quintic.present ||
         (trajectory_sample_valid &&
          std::string(trajectory_sample.state) != "PREPARED")) &&
        selected_owned_motors_confirmed_foc(control_command, motors) &&
        maximum_moving_owned_position_error(
            control_command, motors, software_saturation_joint_mask,
            no_progress_position_error_rad);
    const bool no_progress_tripped_this_cycle =
        exact_load_governor_abort_latched ||
        position_arrival_timeout_latched || observe_no_progress_watchdog(
            no_progress_watchdog, no_progress_observation_valid,
            no_progress_position_error_rad,
            software_saturation_observed_this_cycle, Clock::now(),
            control_command.activation_epoch,
            command_safety.minimum_activation_epoch,
            command_safety.highest_rejected_active_epoch);
    if (no_progress_tripped_this_cycle) {
      command_safety.minimum_activation_epoch = std::max(
          command_safety.minimum_activation_epoch,
          no_progress_watchdog.minimum_rearm_epoch);
      motor_domain_brake_this_cycle = true;
      motor_domain_brake_reason = no_progress_watchdog.trip_reason;
      record_domain_brake_event(
          options.bus, motor_domain_brake_reason,
          domain_brake_log, Clock::now());
      effective_mode = "brake";
      prior_external_hold_confirmed = false;
      position_tracking.fill(false);
      position_tracking_epoch.fill(0U);
      position_endpoint_reached.fill(false);
      position_arrived_once.fill(false);
      position_arrival_overdue.fill(false);
      position_arrival_qualifying_frames.fill(0);
      position_arrival_window_started_at.fill(Clock::time_point{});
      dq_command.fill(0.0);
      for (auto& state : hold_integral_states)
        reset_bounded_hold_integral(state);
      hold_integral_wire_nm.fill(0.0);
      j2_integral_wire_nm = 0.0;
      j2_gain_kp = 0.0;
      j2_gain_kd = 0.0;
      j2_gain_ready = false;
      j2_governor_limited = false;
      aux_governor_limited.fill(false);
      if (options.bus == "j2") j2_pair_ready = false;
      if (lease_safe_hold_active) {
        if (!lease_safe_hold_abort_pending)
          lease_safe_hold_abort_reason = no_progress_watchdog.trip_reason;
        lease_safe_hold_abort_pending = true;
      }
      std::cerr << "POSITION_SAFETY_LATCH"
                << " bus=" << options.bus
                << " reason=" << no_progress_watchdog.trip_reason
                << " position_error_rad="
                << no_progress_position_error_rad
                << " baseline_error_rad="
                << no_progress_watchdog.window_baseline_error_rad
                << " qualifying_frames="
                << no_progress_watchdog.qualifying_frames
                << " window_seconds=" << kNoProgressWindowSeconds
                << " activation_epoch="
                << no_progress_watchdog.trip_activation_epoch
                << " minimum_epoch="
                << no_progress_watchdog.minimum_rearm_epoch
                << " authority=" << kLoadLimitWatchdogAuthority
                << std::endl;
      for (auto& motor : motors) transact_and_commit_brake(motor);
    }

    bool lease_safe_hold_aborted_this_cycle = false;
    if (lease_safe_hold_active &&
        (lease_safe_hold_abort_pending || domain_fault ||
         thermal_interlock.fault_latched ||
         no_progress_watchdog.fault_latched ||
         (options.bus == "j2" && j2_sync_fault) || g_stop.load() ||
         !all_motor_feedback_healthy(motors))) {
      if (lease_safe_hold_abort_reason.empty()) {
        lease_safe_hold_abort_reason = domain_fault
            ? "DOMAIN_FAULT"
            : thermal_interlock.fault_latched
                  ? "THERMAL_INTERLOCK"
            : no_progress_watchdog.fault_latched
                  ? no_progress_watchdog.trip_reason
            : (options.bus == "j2" && j2_sync_fault)
                  ? "J2_SYNC_FAULT"
                  : g_stop.load() ? "STOP_REQUESTED" : "FEEDBACK_UNHEALTHY";
      }
      std::cerr << "LEASE_SAFE_HOLD_ABORT"
                << " bus=" << options.bus
                << " reason=" << lease_safe_hold_abort_reason
                << " source_epoch=" << lease_safe_hold_source_epoch
                << " domain_fault=" << domain_fault
                << " domain_fault_reason=" << domain_fault_reason
                << " j2_sync_fault=" << j2_sync_fault;
      for (const auto& motor : motors) {
        std::cerr << " motor=" << motor.name
                  << ",reference=" << motor.reference_ready
                  << ",valid=" << motor.valid
                  << ",latched=" << motor.fault_latched
                  << ",merror=" << motor.merror
                  << ",temp=" << motor.temperature;
      }
      std::cerr << std::endl;
      lease_safe_hold_active = false;
      lease_safe_hold_aborted_this_cycle = true;
      effective_mode = "brake";
      for (auto& motor : motors) transact_and_commit_brake(motor);
    }

    const bool active_command_blocked =
        is_position_holding_mode(effective_mode) && !any_foc_sent;
    if (active_command_blocked) {
      ActiveCommandBlockedSnapshot snapshot;
      snapshot.cycle = cycles;
      snapshot.lease_fresh = command_lease_fresh;
      snapshot.lease_safe_hold = lease_safe_hold_active;
      snapshot.domain_fault = domain_fault;
      snapshot.j2_sync_fault = j2_sync_fault;
      snapshot.j2_pair_ready = j2_pair_ready;
      snapshot.activation_epoch = control_command.activation_epoch;
      snapshot.minimum_activation_epoch =
          command_safety.minimum_activation_epoch;
      snapshot.mask = control_command.active_joint_mask;
      for (const auto& motor : motors) {
        const auto joint = static_cast<std::size_t>(motor.joint_index);
        snapshot.reference_ready[joint] = motor.reference_ready;
        snapshot.valid[joint] = motor.valid;
        snapshot.fault_latched[joint] = motor.fault_latched;
      }
      if (!active_command_blocked_log.active) {
        active_command_blocked_log.active = true;
        active_command_blocked_log.episode_count = 0;
        ++active_command_blocked_log.transition_count;
        ++active_command_blocked_log.pending_transition_count;
      }
      ++active_command_blocked_log.episode_count;
      ++active_command_blocked_log.total_count;
      ++active_command_blocked_log.pending_count;
      if (!active_command_blocked_log.pending_snapshot) {
        active_command_blocked_log.first = snapshot;
        active_command_blocked_log.pending_snapshot = true;
      }
      active_command_blocked_log.last = snapshot;
      if (active_command_blocked_log.total_count == 1U) {
        log_active_command_blocked_episode(
            "ACTIVE_COMMAND_BLOCKED_BEGIN", options.bus,
            active_command_blocked_log, motors);
        active_command_blocked_log.pending_count = 0U;
        active_command_blocked_log.pending_transition_count = 0U;
        active_command_blocked_log.pending_snapshot = false;
        active_command_blocked_log.last_report_at = Clock::now();
      }
    } else if (active_command_blocked_log.active) {
      active_command_blocked_log.active = false;
      active_command_blocked_log.episode_count = 0;
      ++active_command_blocked_log.transition_count;
      ++active_command_blocked_log.pending_transition_count;
    }
    flush_active_command_blocked_summary(
        "ACTIVE_COMMAND_BLOCKED_SUMMARY", options.bus,
        active_command_blocked_log, motors, Clock::now(), false);
    if (domain_fault || (options.bus == "j2" && j2_sync_fault)) {
      if (domain_fault && !domain_fault_reported) {
        std::cerr << "DOMAIN_FAULT bus=" << options.bus
                  << " reason=" << domain_fault_reason;
        for (const auto& motor : motors) {
          std::cerr << " motor=" << motor.name
                    << ",valid=" << motor.valid
                    << ",latched=" << motor.fault_latched
                    << ",tau_nm=" << motor.last_tau
                    << ",dq_raw=" << motor.last_dq;
        }
        std::cerr << std::endl;
        domain_fault_reported = true;
      }
      for (auto& motor : motors) transact_and_commit_brake(motor);
    }

    if (options.bus == "j2" && request_j2_sync_rearm_for_next_cycle(
            j2_sync_filter, command_lease_fresh,
            command_requests_active_owned_joint && !domain_fault &&
                !thermal_interlock.fault_latched &&
                !no_progress_watchdog.fault_latched && !g_stop.load(),
            all_domain_motors_valid_brake_for_rearm(motors),
            command.activation_epoch,
            command_safety.highest_rejected_active_epoch)) {
      // The release/higher-epoch decision cycle remains BOTH BRAKE.  Only the
      // next loop's apply_pending_j2_sync_rearm_at_cycle_start() may clear it.
      std::cerr << "J2_SYNC_REARM_PENDING_NEXT_CYCLE"
                << " bus=" << options.bus
                << " activation_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << j2_sync_filter.minimum_rearm_epoch
                << std::endl;
    }
    if (request_thermal_rearm_for_next_cycle(
            thermal_interlock, command_lease_fresh,
            command_requests_active_owned_joint && !domain_fault &&
                !(options.bus == "j2" && j2_sync_fault) && !g_stop.load(),
            command.activation_epoch,
            command_safety.highest_rejected_active_epoch)) {
      // The latch deliberately remains set for this complete decision cycle.
      // apply_pending_thermal_rearm_at_cycle_start() is the only clearing edge.
      std::cerr << "THERMAL_REARM_PENDING_NEXT_CYCLE"
                << " bus=" << options.bus
                << " activation_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << thermal_interlock.minimum_rearm_epoch
                << std::endl;
    }
    if (request_no_progress_rearm_for_next_cycle(
            no_progress_watchdog, command_lease_fresh,
            command_requests_active_owned_joint && !domain_fault &&
                !(options.bus == "j2" && j2_sync_fault) &&
                !thermal_interlock.fault_latched && !g_stop.load(),
            all_domain_motors_valid_brake_for_rearm(motors),
            command.activation_epoch,
            command_safety.highest_rejected_active_epoch)) {
      // The release/higher-epoch decision cycle stays in domain BRAKE.  Only
      // the next loop's apply_pending_no_progress_rearm_at_cycle_start clears.
      std::cerr << "LOAD_LIMIT_NO_PROGRESS_REARM_PENDING_NEXT_CYCLE"
                << " bus=" << options.bus
                << " activation_epoch=" << command.activation_epoch
                << " minimum_epoch="
                << no_progress_watchdog.minimum_rearm_epoch
                << std::endl;
    }

    const std::uint64_t stamp = monotonic_ns();
    const bool active_requested =
        (effective_mode == "drag" || is_position_holding_mode(effective_mode)) &&
        std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
          return control_command.active_joint_mask[
              static_cast<std::size_t>(motor.joint_index)];
        });
    const std::string reported_mode =
        (lease_safe_hold_aborted_this_cycle || domain_fault || j2_sync_fault ||
         (active_requested && !any_foc_sent))
            ? "brake" : effective_mode;
    std::string reported_trajectory_state = "INACTIVE";
    std::uint64_t reported_trajectory_sample_index = 0U;
    if (control_command.quintic.present) {
      const QuinticSampleClock feedback_sample = trajectory_sample_valid
          ? trajectory_sample
          : quintic_sample_clock(control_command.quintic, stamp);
      if (reported_mode == "position" && trajectory_sample_valid) {
        reported_trajectory_sample_index = feedback_sample.sample_index;
        reported_trajectory_state = feedback_sample.state;
      }
    }
    const std::string payload = feedback_payload(
        motors, stamp, reported_mode, j2_sync_filter, domain_fault,
        lease_safe_hold_active && reported_mode == "hold", control_command,
        applied_gravity_feedforward_nm,
        reported_trajectory_state, reported_trajectory_sample_index,
        thermal_interlock,
        no_progress_watchdog, no_progress_observation_valid,
        no_progress_position_error_rad,
        software_saturation_observed_this_cycle,
        thermal_derating_factor, command_safety.teach_exit_hold,
        &command_safety.last_accepted_command, command_safety.guidance_paused, command_safety.guidance_paused_reason);
    (void)::sendto(feedback_socket, payload.data(), payload.size(), 0,
                   reinterpret_cast<const sockaddr*>(&feedback_address),
                   sizeof(feedback_address));
    previous_mode = effective_mode;
    previous_active_joint_mask = active_requested
        ? control_command.active_joint_mask : std::array<bool, 6>{};
    previous_moving_joint_mask = active_requested
        ? control_command.moving_joint_mask : std::array<bool, 6>{};
    previous_j2_pair_ready = j2_pair_ready;
    const bool external_hold_confirmed_this_cycle =
        !lease_safe_hold_active && !lease_safe_hold_aborted_this_cycle &&
        command_lease_fresh && is_position_holding_mode(effective_mode) &&
        any_foc_sent && !domain_fault && !j2_sync_fault &&
        !thermal_interlock.fault_latched &&
        !no_progress_watchdog.fault_latched &&
        all_motor_feedback_healthy(motors) &&
        selected_owned_motors_confirmed_foc(command, motors);
    if (external_hold_confirmed_this_cycle) {
      confirmed_external_hold_command = command;
      prior_external_hold_confirmed = true;
    } else if (lease_safe_hold_active || lease_safe_hold_aborted_this_cycle ||
               domain_fault || j2_sync_fault ||
               thermal_interlock.fault_latched ||
               no_progress_watchdog.fault_latched ||
               fresh_command_explicitly_releases) {
      prior_external_hold_confirmed = false;
    }
    ++cycles;
    if (options.bus == "j345" &&
        std::any_of(motors.begin(), motors.end(), [](const MotorRuntime& motor) {
          return motor.fault_latched;
        }))
      next = Clock::now();
    next += std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(kPeriod));
    std::this_thread::sleep_until(next);
  }
  const std::uint64_t final_brake_tx_attempts_before =
      tx_audit.tx_attempt_total;
  const bool final_brake_confirmed = brake_guard.brake_and_disarm();
  const std::uint64_t final_brake_tx_attempt_count =
      tx_audit.tx_attempt_total - final_brake_tx_attempts_before;
  const bool tx_counts_consistent =
      tx_audit.tx_attempt_total ==
          tx_audit.brake_tx_attempt_count +
              tx_audit.foc_tx_attempt_count +
              tx_audit.other_mode_tx_attempt_count &&
      tx_audit.serial_send_call_count <= tx_audit.tx_attempt_total &&
      tx_audit.foc_serial_send_call_count <=
          tx_audit.foc_tx_attempt_count;
  const bool phase_tx_accounting_pass =
      final_brake_tx_attempt_count == 20U * motors.size() &&
      (options.bus != "j2" ||
       (j2_startup_brake_prime.applicable &&
        j2_startup_brake_prime.passed &&
        j2_startup_brake_prime.tx_attempt_count ==
            2U * static_cast<std::uint64_t>(
                j2_startup_brake_prime.attempted_pairs)));
  const bool brake_only_audit_pass = phase_tx_accounting_pass &&
      (!options.brake_only ||
      (command_socket < 0 && tx_counts_consistent &&
       tx_audit.tx_attempt_total == tx_audit.brake_tx_attempt_count &&
       tx_audit.foc_tx_attempt_count == 0U &&
       tx_audit.other_mode_tx_attempt_count == 0U &&
       tx_audit.brake_only_guard_block_count == 0U &&
       tx_audit.foc_serial_send_call_count == 0U &&
       tx_audit.serial_send_call_count == tx_audit.brake_tx_attempt_count));
  flush_command_receive_summaries(
      command_receive_state, Clock::now(), true);
  for (std::size_t index = 0; index < invalid_feedback_logs.size(); ++index) {
    flush_invalid_feedback_summary(
        "J2_INVALID_FEEDBACK_FINAL", motors[index].name,
        invalid_feedback_logs[index], Clock::now(), true);
  }
  flush_domain_brake_summary(options.bus, domain_brake_log, Clock::now());
  flush_active_command_blocked_summary(
      "ACTIVE_COMMAND_BLOCKED_FINAL", options.bus,
      active_command_blocked_log, motors, Clock::now(), true);
  if (command_socket >= 0) ::close(command_socket);
  ::close(feedback_socket);
  std::cout << "TERMINAL_PATH=NORMAL\n"
            << "GUI_GO_CONTROLLER_BUS=" << options.bus << "\n"
            << "EXECUTION_POLICY="
            << (options.brake_only ? "BRAKE_ONLY" : "NORMAL") << "\n"
            << "COMMAND_RX_ENABLED="
            << (options.brake_only ? "NO" : "YES") << "\n"
             << "J2_SESSION_REFERENCE_CONFIGURED="
            << (options.bus == "j2" &&
                        std::all_of(
                            motors.begin(), motors.end(),
                            [](const MotorRuntime& motor) {
                              return motor.session_reference_configured;
                            })
                    ? "YES" : "NO") << "\n"
             << "J2_STARTUP_SESSION_VERIFIED="
              << (j2_active_session && j2_startup_verified ? "YES" : "NO")
              << "\n"
             << "J2_LAUNCH_PERMIT_STATE="
              << (j2_active_session ? j2_launch_permit.state : "NONE") << "\n"
             << "J2_STARTUP_BRAKE_PRIME_STATE="
             << (!j2_startup_brake_prime.applicable
                     ? "NOT_APPLICABLE"
                     : (j2_startup_brake_prime.passed ? "PASS" : "FAIL"))
             << "\n"
             << "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS="
             << kJ2StartupPrimeMaximumInvalidPrefixPairs << "\n"
             << "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS="
             << j2_startup_brake_prime.invalid_prefix_pairs << "\n"
             << "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS="
             << kJ2StartupPrimeRequiredHealthyPairs << "\n"
             << "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS="
             << j2_startup_brake_prime.healthy_qualification_pairs << "\n"
             << "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS="
             << j2_startup_brake_prime.attempted_pairs << "\n"
             << "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT="
             << j2_startup_brake_prime.tx_attempt_count << "\n"
             << "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS="
             << j2_startup_brake_prime.elapsed_ns << "\n"
             << "GO_AUX_SESSION_REFERENCE_CONFIGURED="
             << (go_aux_active_session &&
                         std::all_of(
                             motors.begin(), motors.end(),
                             [](const MotorRuntime& motor) {
                               return motor.session_reference_configured;
                             })
                     ? "YES" : "NO") << "\n"
             << "GO_AUX_STARTUP_SESSION_VERIFIED="
             << (go_aux_active_session && j2_startup_verified ? "YES" : "NO")
             << "\n"
             << "GO_AUX_LAUNCH_PERMIT_STATE="
             << (go_aux_active_session ? j2_launch_permit.state : "NONE") << "\n"
             << "COMMUNICATION_RECOVERY_ATTEMPT_COUNT="
             << communication_recovery_attempt_count << "\n"
             << "COMMUNICATION_RECOVERY_SUCCESS_COUNT="
             << communication_recovery_success_count << "\n"
             << "COMPLETED_CYCLES=" << cycles << "\n"
            << "TX_ATTEMPT_TOTAL=" << tx_audit.tx_attempt_total << "\n"
            << "BRAKE_TX_ATTEMPT_COUNT="
            << tx_audit.brake_tx_attempt_count << "\n"
            << "FOC_TX_ATTEMPT_COUNT=" << tx_audit.foc_tx_attempt_count << "\n"
            << "OTHER_MODE_TX_ATTEMPT_COUNT="
            << tx_audit.other_mode_tx_attempt_count << "\n"
            << "BRAKE_ONLY_GUARD_BLOCK_COUNT="
            << tx_audit.brake_only_guard_block_count << "\n"
            << "SERIAL_SEND_CALL_COUNT="
            << tx_audit.serial_send_call_count << "\n"
            << "FOC_SERIAL_SEND_CALL_COUNT="
            << tx_audit.foc_serial_send_call_count << "\n"
            << "FINAL_BRAKE_TX_ATTEMPT_COUNT="
            << final_brake_tx_attempt_count << "\n"
            << "BRAKE_ONLY_AUDIT="
            << (options.brake_only
                    ? (brake_only_audit_pass ? "PASS" : "FAIL")
                    : "NOT_APPLICABLE") << "\n"
            << "FINAL_MODE=" << (final_brake_confirmed ? "BRAKE" : "UNCONFIRMED") << "\n"
            << "FINAL_BRAKE=" << (final_brake_confirmed ? "PASS" : "FAIL") << "\n"
            << "MOTOR_INTERNAL_ZERO_WRITE=NO\n";
  return final_brake_confirmed && brake_only_audit_pass ? 0 : 2;
}
}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  try { return run(parse_options(argc, argv)); }
  catch (const std::exception& error) {
    std::cerr << "V15_30A_GUI_GO_CONTROLLER=BLOCKED\nREASON=" << error.what() << '\n';
    return 2;
  }
}
