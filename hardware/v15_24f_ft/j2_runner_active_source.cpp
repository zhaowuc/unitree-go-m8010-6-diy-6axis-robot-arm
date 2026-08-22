#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace {

using Clock = std::chrono::steady_clock;
constexpr char kPort[] =
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0";
constexpr char kLockPath[] = "/tmp/v15_23d_ft_j2_channel1.lock";
constexpr int kIdA = 0;
constexpr int kIdB = 1;
constexpr int kBrakeMode = 0;
constexpr int kFocMode = 1;
constexpr int kSignA = -1;
constexpr int kSignB = +1;
constexpr double kPi = 3.14159265358979323846;
constexpr double kGear = 6.3299999237060547;
constexpr double kBaseKp = 0.60;
constexpr double kKd = 0.05;
constexpr double kTauFf = 0.0;
constexpr double kHz = 100.0;
constexpr double kPeriod = 0.01;
constexpr double kVmax = 10.0 * kPi / 180.0;
constexpr double kAccel = 30.0 * kPi / 180.0;
constexpr double kCommandEnvelope = 5.5 * kPi / 180.0;
constexpr double kFeedbackEnvelope = 7.0 * kPi / 180.0;
constexpr double kEndpointSyncAccept = 0.3 * kPi / 180.0;
constexpr double kMotionSyncAccept = 0.7 * kPi / 180.0;
constexpr double kSyncHardAbort = 1.0 * kPi / 180.0;
constexpr double kUnexpectedVelocityAbort = 25.0 * kPi / 180.0;
constexpr double kCaptureSpanLimit = 0.2 * kPi / 180.0;
constexpr double kCaptureTailMedianLimit = 0.1 * kPi / 180.0;
constexpr int kTempLimit = 60;
constexpr char kRepoRoot[] = "/home/car/go-m8010-robot-arm-v15-20a";
constexpr char kSourceHead[] =
    "5129a975d5c456aa04fed2e7ff47456a56785489";
constexpr char kEligibilityPath[] = "/tmp/v15_24f_next_phase_gate.json";
constexpr char kCsvHeader[] =
    "tick,timestamp_s,phase,level,target_kp,active_kp,"
    "kp_cmd_count,kp_cmd_decoded,kd_cmd_count,kd_cmd_decoded,"
    "tff_nm,a_tff_count,b_tff_count,"
    "a_mode,b_mode,a_q_cmd,b_q_cmd,a_dq_cmd,b_dq_cmd,"
    "a_raw,b_raw,qA_logical_rad,qB_logical_rad,qJ2_logical_rad,"
    "q_ref_rad,e_common_rad,e_sync_rad,a_dq_feedback,b_dq_feedback,"
    "a_dq_logical,b_dq_logical,a_tau,b_tau,tau_J2_feedback,"
    "expected_A_PD_tau,expected_B_PD_tau,"
    "a_temp,b_temp,a_merror,b_merror,a_received_id,b_received_id,"
    "a_send_recv,b_send_recv,a_correct,b_correct,a_crc_ok,b_crc_ok,"
    "a_valid,b_valid,termination_reason,cycle_period_ms,cycle_jitter_ms";

struct LevelConfig {
  double target_kp = 0.0;
  double ramp_duration = 0.0;
  const char* label = "";
};

volatile std::sig_atomic_t g_stop = 0;
void signal_handler(int) { g_stop = 1; }

double smoothstep5(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

bool endpoint_accepted(double error, double sync, double max_motion_sync,
                       double position_tolerance) {
  return error <= position_tolerance && sync <= kEndpointSyncAccept &&
      max_motion_sync <= kMotionSyncAccept;
}

std::string sha256_file(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("SHA256_FILE_OPEN_FAILED:" + path);
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
      EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA256_INIT_FAILED");
  std::array<char, 65536> buffer{};
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = stream.gcount();
    if (count > 0 &&
        EVP_DigestUpdate(context.get(), buffer.data(),
                         static_cast<std::size_t>(count)) != 1)
      throw std::runtime_error("SHA256_UPDATE_FAILED");
  }
  if (!stream.eof()) throw std::runtime_error("SHA256_FILE_READ_FAILED:" + path);
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1 ||
      digest_size != 32U)
    throw std::runtime_error("SHA256_FINAL_FAILED");
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < digest_size; ++i)
    result << std::setw(2) << static_cast<unsigned int>(digest[i]);
  return result.str();
}

template <typename T>
void zero_object(T& value) {
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
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U
          ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
          : static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

MotorCmd make_command(int id, int mode, double q, double dq,
                      double kp, double kd, double tau) {
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
  const std::uint8_t* raw = command.get_motor_send_data();
  if (raw == nullptr || raw[0] != 0xfeU || raw[1] != 0xeeU ||
      raw[2] != static_cast<std::uint8_t>(id | (mode << 4)) ||
      (raw[2] & 0x80U) != 0U ||
      load_u16_le(raw + 15) != crc16_kermit(raw, 15)) {
    throw std::runtime_error("COMMAND_PACKET_AUDIT_FAILED");
  }
  return command;
}

MotorCmd brake_command(int id) {
  return make_command(id, kBrakeMode, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void initialize_feedback(MotorData& data) {
  zero_object(data);
  data.motorType = MotorType::GO_M8010_6;
  data.motor_id = 0xffU;
  data.mode = 0xffU;
  data.temp = std::numeric_limits<int>::min();
  data.merror = -1;
  data.q = std::numeric_limits<float>::quiet_NaN();
  data.dq = std::numeric_limits<float>::quiet_NaN();
  data.tau = std::numeric_limits<float>::quiet_NaN();
  data.correct = false;
}

struct Feedback {
  bool send_recv = false;
  bool correct = false;
  bool crc_ok = false;
  int id = -1;
  int mode = -1;
  int temp = -1;
  int merror = -1;
  double q = std::numeric_limits<double>::quiet_NaN();
  double dq = std::numeric_limits<double>::quiet_NaN();
  double tau = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
};

class Unwrapper {
 public:
  double update(double raw) {
    if (!initialized_) {
      initialized_ = true;
      previous_ = raw;
      value_ = raw;
      return value_;
    }
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

double median(std::vector<double> values) {
  if (values.empty()) throw std::runtime_error("MEDIAN_EMPTY");
  std::sort(values.begin(), values.end());
  const std::size_t m = values.size() / 2U;
  return values.size() % 2U == 0U
      ? (values[m - 1U] + values[m]) / 2.0 : values[m];
}

bool capture_is_static(const std::vector<double>& raw_values) {
  if (raw_values.size() != 50U) return false;
  const auto bounds = std::minmax_element(raw_values.begin(), raw_values.end());
  const double output_span = (*bounds.second - *bounds.first) / kGear;
  const double full_median = median(raw_values);
  const std::vector<double> tail(raw_values.end() - 10, raw_values.end());
  const double tail_offset = std::abs(median(tail) - full_median) / kGear;
  return output_span <= kCaptureSpanLimit &&
      tail_offset <= kCaptureTailMedianLimit;
}

struct ProfilePoint { double q = 0.0; double dq = 0.0; };

class Trapezoid {
 public:
  explicit Trapezoid(double displacement)
      : sign_(displacement < 0.0 ? -1.0 : 1.0),
        distance_(std::abs(displacement)) {
    const double threshold = kVmax * kVmax / kAccel;
    if (distance_ <= threshold) {
      t_acc_ = std::sqrt(distance_ / kAccel);
      v_peak_ = kAccel * t_acc_;
      t_cruise_ = 0.0;
    } else {
      t_acc_ = kVmax / kAccel;
      v_peak_ = kVmax;
      t_cruise_ = (distance_ - kAccel * t_acc_ * t_acc_) / kVmax;
    }
    duration_ = 2.0 * t_acc_ + t_cruise_;
  }
  double duration() const { return duration_; }
  ProfilePoint sample(double t) const {
    t = std::clamp(t, 0.0, duration_);
    const double d_acc = 0.5 * kAccel * t_acc_ * t_acc_;
    double q = 0.0;
    double v = 0.0;
    if (t < t_acc_) {
      q = 0.5 * kAccel * t * t;
      v = kAccel * t;
    } else if (t < t_acc_ + t_cruise_) {
      const double u = t - t_acc_;
      q = d_acc + v_peak_ * u;
      v = v_peak_;
    } else if (t < duration_) {
      const double u = duration_ - t;
      q = distance_ - 0.5 * kAccel * u * u;
      v = kAccel * u;
    } else {
      q = distance_;
      v = 0.0;
    }
    return {sign_ * q, sign_ * v};
  }
 private:
  double sign_ = 1.0;
  double distance_ = 0.0;
  double t_acc_ = 0.0;
  double t_cruise_ = 0.0;
  double v_peak_ = 0.0;
  double duration_ = 0.0;
};

struct Cli {
  std::string phase;
  std::string output;
  std::string eligibility_file;
  std::string authorization_gate;
  std::string safety_gate;
  std::string power_on_gate;
  bool self_test = false;
};

Cli parse_cli(int argc, char** argv) {
  Cli cli;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() {
      if (++i >= argc) throw std::runtime_error("CLI_VALUE_MISSING");
      return std::string(argv[i]);
    };
    if (arg == "--self-test") cli.self_test = true;
    else if (arg == "--phase") cli.phase = value();
    else if (arg == "--output") cli.output = value();
    else if (arg == "--eligibility-file") cli.eligibility_file = value();
    else if (arg == "--authorization-gate") cli.authorization_gate = value();
    else if (arg == "--safety-gate") cli.safety_gate = value();
    else if (arg == "--power-on-gate") cli.power_on_gate = value();
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  if (cli.self_test) return cli;
  const bool allowed_phase = cli.phase == "j2-kp100-run1" ||
      cli.phase == "j2-kp100-repeat" ||
      cli.phase == "j2-kp140-run1" ||
      cli.phase == "j2-kp140-repeat";
  if (!allowed_phase)
    throw std::runtime_error("PHASE_NOT_ALLOWED");
  const std::string expected_output =
      cli.phase == "j2-kp100-run1"
          ? "hardware/v15_24f_ft/j2_kp100_run1.csv"
      : cli.phase == "j2-kp100-repeat"
          ? "hardware/v15_24f_ft/j2_kp100_repeat.csv"
      : cli.phase == "j2-kp140-run1"
          ? "hardware/v15_24f_ft/j2_kp140_run1.csv"
          : "hardware/v15_24f_ft/j2_kp140_repeat.csv";
  if (cli.output != expected_output)
    throw std::runtime_error("OUTPUT_PATH_NOT_ALLOWED");
  if (std::filesystem::exists(std::string(kRepoRoot) + "/" + cli.output))
    throw std::runtime_error("EVIDENCE_OUTPUT_ALREADY_EXISTS_REFUSE_OVERWRITE");
  if (cli.phase == "j2-kp100-run1") {
    if (!cli.eligibility_file.empty())
      throw std::runtime_error("LEVEL_A_RUN_MUST_NOT_USE_PHASE_GATE");
  } else if (cli.eligibility_file != kEligibilityPath) {
    throw std::runtime_error("NEXT_PHASE_ELIGIBILITY_GATE_MISSING");
  }
  const std::string expected_auth =
      cli.phase == "j2-kp100-run1"
          ? "J2_KP100_RUN1_AUTHORIZED=YES"
      : cli.phase == "j2-kp100-repeat"
          ? "J2_KP100_EXACT_REPEAT_AUTHORIZED=YES"
      : cli.phase == "j2-kp140-run1"
          ? "J2_KP140_RUN1_AUTHORIZED=YES"
          : "J2_KP140_EXACT_REPEAT_AUTHORIZED=YES";
  if (cli.authorization_gate != expected_auth ||
      cli.safety_gate != "J2_PLUS_MINUS_5_CLEAR=YES" ||
      cli.power_on_gate != "V15_24F_24V_POWER_ON_CONFIRMED=YES")
    throw std::runtime_error("DUAL_OPERATOR_GATE_MISSING");
  return cli;
}

std::ofstream open_evidence_csv(const std::string& path) {
  std::ofstream csv(path, std::ios::out | std::ios::trunc);
  if (!csv) throw std::runtime_error("CSV_OPEN_FAILED");
  csv << kCsvHeader << '\n';
  csv.flush();
  if (!csv) throw std::runtime_error("CSV_HEADER_WRITE_FAILED");
  return csv;
}

LevelConfig level_config(const Cli& cli) {
  if (cli.phase == "j2-kp100-run1" ||
      cli.phase == "j2-kp100-repeat")
    return {1.00, 0.50, "J2_KP100"};
  return {1.40, 0.75, "J2_KP140"};
}

void verify_phase_eligibility(const Cli& cli) {
  if (cli.phase == "j2-kp100-run1") return;
  std::ifstream stream(cli.eligibility_file);
  if (!stream) throw std::runtime_error("NEXT_PHASE_GATE_OPEN_FAILED");
  nlohmann::json gate;
  stream >> gate;
  const std::string prerequisite =
      cli.phase == "j2-kp100-repeat"
          ? "hardware/v15_24f_ft/j2_kp100_run1.csv"
      : cli.phase == "j2-kp140-run1"
          ? "hardware/v15_24f_ft/j2_kp100_run1.csv"
          : "hardware/v15_24f_ft/j2_kp140_run1.csv";
  const std::string reason =
      cli.phase == "j2-kp100-repeat"
          ? "J2_KP100_RUN1_PASS"
      : cli.phase == "j2-kp140-run1"
          ? "J2_KP100_RUN1_SAFE_TRACKING_FAIL"
          : "J2_KP140_RUN1_PASS";
  if (gate.at("schema").get<std::string>() !=
          "V15_24F_NEXT_PHASE_GATE_V1" ||
      !gate.at("eligible").get<bool>() ||
      gate.at("allowed_phase").get<std::string>() != cli.phase ||
      gate.at("reason").get<std::string>() != reason ||
      gate.at("source_head").get<std::string>() != kSourceHead ||
      gate.at("operator_observation").get<std::string>() != "SAFE" ||
      gate.at("final_brake").get<std::string>() != "PASS" ||
      gate.at("center_gate_fix").get<std::string>() != "YES" ||
      gate.at("timing_100hz_result").get<std::string>() != "PASS" ||
      gate.at("hold_result").get<std::string>() != "PASS" ||
      gate.at("prerequisite_csv_path").get<std::string>() != prerequisite ||
      gate.at("prerequisite_csv_sha256").get<std::string>() !=
          sha256_file(std::string(kRepoRoot) + "/" + prerequisite))
    throw std::runtime_error("NEXT_PHASE_GATE_AUTHORITY_MISMATCH");
  const bool repeat_phase = cli.phase == "j2-kp100-repeat" ||
      cli.phase == "j2-kp140-repeat";
  if ((repeat_phase && gate.at("prerequisite_result").get<std::string>() !=
                           "PASS") ||
      (cli.phase == "j2-kp140-run1" &&
       (gate.at("prerequisite_result").get<std::string>() != "FAIL" ||
        !gate.at("safe_tracking_fail_eligible").get<bool>())))
    throw std::runtime_error("NEXT_PHASE_GATE_RESULT_MISMATCH");
}

LevelConfig load_preflight(const Cli& cli) {
  verify_phase_eligibility(cli);
  return level_config(cli);
}

class ProcessLock {
 public:
  ProcessLock() {
    fd_ = ::open(kLockPath, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd_ < 0 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0)
      throw std::runtime_error("J2_CHANNEL1_LOCK_FAILED");
  }
  ~ProcessLock() {
    if (fd_ >= 0) {
      (void)::flock(fd_, LOCK_UN);
      (void)::close(fd_);
    }
  }
 private:
  int fd_ = -1;
};

struct PairState {
  Feedback a;
  Feedback b;
  double raw_a = std::numeric_limits<double>::quiet_NaN();
  double raw_b = std::numeric_limits<double>::quiet_NaN();
  double q_a = std::numeric_limits<double>::quiet_NaN();
  double q_b = std::numeric_limits<double>::quiet_NaN();
  double q_j2 = std::numeric_limits<double>::quiet_NaN();
  double e_sync = std::numeric_limits<double>::quiet_NaN();
  double q_ref = std::numeric_limits<double>::quiet_NaN();
  double e_common = std::numeric_limits<double>::quiet_NaN();
  double kp = 0.0;
  std::uint16_t kp_count = 0;
  double kp_decoded = 0.0;
  std::uint16_t kd_count = 0;
  double kd_decoded = 0.0;
  std::int16_t a_tff_count = 0;
  std::int16_t b_tff_count = 0;
  double dq_a_logical = std::numeric_limits<double>::quiet_NaN();
  double dq_b_logical = std::numeric_limits<double>::quiet_NaN();
  double tau_j2_feedback = std::numeric_limits<double>::quiet_NaN();
  double expected_a_pd_tau = std::numeric_limits<double>::quiet_NaN();
  double expected_b_pd_tau = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
};

class Runner {
 public:
  explicit Runner(const Cli& cli)
      : cli_(cli), config_(load_preflight(cli)), lock_(),
        csv_(open_evidence_csv(std::string(kRepoRoot) + "/" + cli.output)),
        serial_(kPort, 16, 4000000, 20000, BlockYN::NO,
                bytesize_t::eightbits, parity_t::parity_none,
                stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none) {
    origin_ = Clock::now();
  }
  ~Runner() { if (!terminal_brake_done_) safe_brake(); }

  int run() {
    try {
      return run_dual_motion();
    } catch (const std::exception& error) {
      termination_reason_ = error.what();
      const bool brake = terminal_brake();
      std::cerr << "FINAL_DUAL_5_FRAME_BRAKE="
                << (brake ? "PASS" : "FAIL") << '\n';
      throw;
    } catch (...) {
      termination_reason_ = "NON_STD_EXCEPTION";
      const bool brake = terminal_brake();
      std::cerr << "FINAL_DUAL_5_FRAME_BRAKE="
                << (brake ? "PASS" : "FAIL") << '\n';
      throw;
    }
  }

 private:
  Feedback transact(MotorCmd& command, int expected_id, int expected_mode) {
    MotorData data;
    initialize_feedback(data);
    Feedback f;
    try { f.send_recv = serial_.sendRecv(&command, &data); }
    catch (...) { f.send_recv = false; }
    f.correct = data.correct;
    f.id = static_cast<int>(data.motor_id);
    f.mode = static_cast<int>(data.mode);
    f.temp = data.temp;
    f.merror = data.merror;
    f.q = data.q;
    f.dq = data.dq;
    f.tau = data.tau;
    const std::uint8_t* raw = data.get_motor_recv_data();
    if (raw != nullptr) {
      f.crc_ok = raw[0] == 0xfdU && raw[1] == 0xeeU &&
          (raw[2] & 0x80U) == 0U &&
          load_u16_le(raw + 14) == crc16_kermit(raw, 14);
    }
    f.valid = f.send_recv && f.correct && f.crc_ok &&
        f.id == expected_id && f.mode == expected_mode && f.merror == 0 &&
        f.temp >= 0 && f.temp < kTempLimit && std::isfinite(f.q) &&
        std::isfinite(f.dq) && std::isfinite(f.tau);
    return f;
  }

  PairState transact_pair(int mode, double q_target, double dq_target,
                          double kp, const std::string& phase,
                          Clock::time_point scheduled,
                          bool ignore_stop = false) {
    if (g_stop != 0 && !ignore_stop)
      throw std::runtime_error("OPERATOR_ABORT");
    if (mode == kFocMode && !reference_set_)
      throw std::runtime_error("REFERENCE_NOT_SET");
    if (std::abs(q_target) > kCommandEnvelope + 1e-12)
      throw std::runtime_error("COMMAND_ENVELOPE");
    if (mode == kFocMode &&
        (!std::isfinite(kp) || kp < kBaseKp - 1e-12 ||
         kp > config_.target_kp + 1e-12 || kp > 1.40 + 1e-12))
      throw std::runtime_error("KP_ENVELOPE");
    std::this_thread::sleep_until(scheduled);
    const Clock::time_point begin = Clock::now();
    if (g_stop != 0 && !ignore_stop)
      throw std::runtime_error("OPERATOR_ABORT");
    if (mode == kFocMode && begin > scheduled + std::chrono::milliseconds(2))
      throw std::runtime_error("CONTROL_DEADLINE_MISS_GT_2MS");

    const double a_q_cmd = mode == kFocMode
        ? ref_a_ + kSignA * kGear * q_target : 0.0;
    const double b_q_cmd = mode == kFocMode
        ? ref_b_ + kSignB * kGear * q_target : 0.0;
    const double a_dq_cmd = mode == kFocMode
        ? kSignA * kGear * dq_target : 0.0;
    const double b_dq_cmd = mode == kFocMode
        ? kSignB * kGear * dq_target : 0.0;
    MotorCmd command_a = mode == kFocMode
        ? make_command(kIdA, mode, a_q_cmd, a_dq_cmd, kp, kKd, kTauFf)
        : brake_command(kIdA);
    MotorCmd command_b = mode == kFocMode
        ? make_command(kIdB, mode, b_q_cmd, b_dq_cmd, kp, kKd, kTauFf)
        : brake_command(kIdB);
    PairState state;
    std::string deferred_abort;
    state.kp = mode == kFocMode ? kp : 0.0;
    if (mode == kFocMode) {
      const std::uint8_t* packet_a = command_a.get_motor_send_data();
      const std::uint8_t* packet_b = command_b.get_motor_send_data();
      state.kp_count = load_u16_le(packet_a + 11);
      state.kd_count = load_u16_le(packet_a + 13);
      state.kp_decoded = static_cast<double>(state.kp_count) / 1280.0;
      state.kd_decoded = static_cast<double>(state.kd_count) / 1280.0;
      state.a_tff_count = static_cast<std::int16_t>(load_u16_le(packet_a + 3));
      state.b_tff_count = static_cast<std::int16_t>(load_u16_le(packet_b + 3));
      if (state.a_tff_count != 0 || state.b_tff_count != 0)
        throw std::runtime_error("TFF_ZERO_CONTRACT_VIOLATION");
      if (state.kp_count != load_u16_le(packet_b + 11) ||
          state.kd_count != load_u16_le(packet_b + 13))
        throw std::runtime_error("DUAL_GAIN_ENCODING_MISMATCH");
    }
    state.q_ref = mode == kFocMode ? q_target
                                   : std::numeric_limits<double>::quiet_NaN();
    state.a = transact(command_a, kIdA, mode);
    state.b = transact(command_b, kIdB, mode);
    if (state.a.valid && state.b.valid) {
      state.raw_a = unwrap_a_.update(state.a.q);
      state.raw_b = unwrap_b_.update(state.b.q);
      if (reference_set_) {
        state.q_a = kSignA * (state.raw_a - ref_a_) / kGear;
        state.q_b = kSignB * (state.raw_b - ref_b_) / kGear;
        state.q_j2 = 0.5 * (state.q_a + state.q_b);
        state.e_common = q_target - state.q_j2;
        state.e_sync = state.q_a - state.q_b;
        state.dq_a_logical = kSignA * state.a.dq / kGear;
        state.dq_b_logical = kSignB * state.b.dq / kGear;
        state.tau_j2_feedback = kGear *
            (kSignA * state.a.tau + kSignB * state.b.tau);
        state.expected_a_pd_tau = kp * (a_q_cmd - state.raw_a);
        state.expected_b_pd_tau = kp * (b_q_cmd - state.raw_b);
        if (std::abs(state.q_a) > kFeedbackEnvelope ||
            std::abs(state.q_b) > kFeedbackEnvelope ||
            std::abs(state.q_j2) > kFeedbackEnvelope)
          deferred_abort = "FEEDBACK_ENVELOPE";
        if (mode == kFocMode &&
            (std::abs(state.dq_a_logical) > kUnexpectedVelocityAbort ||
             std::abs(state.dq_b_logical) > kUnexpectedVelocityAbort) &&
            deferred_abort.empty())
          deferred_abort = "UNEXPECTED_LOGICAL_VELOCITY_GT_25DEG_S";
        if (mode == kFocMode)
          max_logical_tau_feedback_ = std::max(
              max_logical_tau_feedback_, std::abs(state.tau_j2_feedback));
      }
      state.valid = true;
    }
    log_row(state, phase, a_q_cmd, b_q_cmd, a_dq_cmd, b_dq_cmd, begin);
    ++tick_;
    if (!deferred_abort.empty()) throw std::runtime_error(deferred_abort);
    if (mode == kFocMode &&
        (state.a.merror != 0 || state.b.merror != 0))
      throw std::runtime_error("MERROR_NONZERO_ACTIVE");
    if (mode == kFocMode && (!state.a.valid || !state.b.valid))
      throw std::runtime_error("ACTIVE_FEEDBACK_LOSS_OR_INVALID");
    if (state.valid && reference_set_ &&
        std::abs(state.e_sync) > kSyncHardAbort)
      throw std::runtime_error("SYNC_HARD_ABORT_GT_1DEG");
    return state;
  }

  std::pair<std::vector<double>, std::vector<double>> capture_references() {
    std::vector<double> a_values;
    std::vector<double> b_values;
    a_values.reserve(50U);
    b_values.reserve(50U);
    Clock::time_point next = Clock::now();
    for (int frame = 0; frame < 50; ++frame) {
      const PairState state = transact_pair(kBrakeMode, 0.0, 0.0,
                                            0.0, "SESSION_BRAKE_CAPTURE", next);
      if (state.valid) {
        a_values.push_back(state.raw_a);
        b_values.push_back(state.raw_b);
      }
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
      if (next < Clock::now()) next = Clock::now();
    }
    if (a_values.size() != 50U || b_values.size() != 50U)
      throw std::runtime_error("DUAL_SESSION_NOT_50_VALID_EACH");
    if (!capture_is_static(a_values) || !capture_is_static(b_values))
      throw std::runtime_error("DUAL_SESSION_BRAKE_CAPTURE_NOT_STATIC");
    return {a_values, b_values};
  }

  void establish_references(const std::vector<double>& a_values,
                            const std::vector<double>& b_values) {
    ref_a_ = median(a_values);
    ref_b_ = median(b_values);
    reference_set_ = true;
  }

  std::vector<PairState> run_target_hold(const std::string& phase,
                                         double target, double duration,
                                         double kp,
                                         bool collect_motion_sync) {
    const int count = static_cast<int>(std::ceil(duration * kHz));
    std::vector<PairState> values;
    values.reserve(static_cast<std::size_t>(count));
    Clock::time_point next = active_started_ ? next_active_ : Clock::now();
    active_started_ = true;
    for (int frame = 0; frame < count; ++frame) {
      PairState state = transact_pair(kFocMode, target, 0.0, kp, phase, next);
      if (state.valid) {
        values.push_back(state);
        if (collect_motion_sync)
          max_motion_sync_ = std::max(max_motion_sync_, std::abs(state.e_sync));
        if (collect_motion_sync)
          max_common_error_ = std::max(max_common_error_, std::abs(state.e_common));
      }
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
    }
    next_active_ = next;
    return values;
  }

  void run_profile(const std::string& phase, double start, double displacement,
                   double kp) {
    Trapezoid profile(displacement);
    const int count = static_cast<int>(std::ceil(profile.duration() * kHz)) + 1;
    Clock::time_point next = active_started_ ? next_active_ : Clock::now();
    active_started_ = true;
    for (int frame = 0; frame < count; ++frame) {
      const double t = std::min(frame * kPeriod, profile.duration());
      const ProfilePoint point = profile.sample(t);
      const PairState state = transact_pair(kFocMode, start + point.q, point.dq,
                                            kp, phase, next);
      if (state.valid)
        max_motion_sync_ = std::max(max_motion_sync_, std::abs(state.e_sync));
      if (state.valid)
        max_common_error_ = std::max(max_common_error_, std::abs(state.e_common));
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
    }
    next_active_ = next;
  }

  bool terminal_brake() {
    bool pass = true;
    Clock::time_point next = Clock::now();
    for (int frame = 0; frame < 5; ++frame) {
      try {
        const PairState state = transact_pair(kBrakeMode, 0.0, 0.0,
                                              0.0, "FINAL_DUAL_BRAKE", next,
                                              true);
        pass = pass && state.a.valid && state.b.valid;
      } catch (...) {
        pass = false;
      }
      next += std::chrono::milliseconds(10);
    }
    csv_.flush();
    if (!csv_) pass = false;
    final_brake_pass_ = pass;
    if (pass) terminal_brake_done_ = true;
    else safe_brake();
    return pass;
  }

  void safe_brake() noexcept {
    try {
      for (int frame = 0; frame < 20; ++frame) {
        MotorCmd a = brake_command(kIdA);
        MotorCmd b = brake_command(kIdB);
        (void)transact(a, kIdA, kBrakeMode);
        (void)transact(b, kIdB, kBrakeMode);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      terminal_brake_done_ = true;
    } catch (...) {}
  }

  void log_row(const PairState& state, const std::string& phase,
               double a_q_cmd, double b_q_cmd,
               double a_dq_cmd, double b_dq_cmd,
               Clock::time_point begin) {
    const double now_s = std::chrono::duration<double>(begin - origin_).count();
    double period_ms = std::numeric_limits<double>::quiet_NaN();
    double jitter_ms = std::numeric_limits<double>::quiet_NaN();
    if (have_previous_) {
      period_ms = (now_s - previous_s_) * 1000.0;
      jitter_ms = std::abs(period_ms - 10.0);
    }
    previous_s_ = now_s;
    have_previous_ = true;
    csv_ << tick_ << ',' << std::setprecision(17) << now_s << ',' << phase << ','
         << config_.label << ',' << config_.target_kp << ',' << state.kp << ','
         << state.kp_count << ',' << state.kp_decoded << ','
         << state.kd_count << ',' << state.kd_decoded << ','
         << kTauFf << ',' << state.a_tff_count << ',' << state.b_tff_count << ','
         << state.a.mode << ',' << state.b.mode << ','
         << a_q_cmd << ',' << b_q_cmd << ',' << a_dq_cmd << ',' << b_dq_cmd << ','
         << state.raw_a << ',' << state.raw_b << ',' << state.q_a << ',' << state.q_b
         << ',' << state.q_j2 << ',' << state.q_ref << ',' << state.e_common
         << ',' << state.e_sync << ','
         << state.a.dq << ',' << state.b.dq << ','
         << state.dq_a_logical << ',' << state.dq_b_logical << ','
         << state.a.tau << ',' << state.b.tau << ',' << state.tau_j2_feedback << ','
         << state.expected_a_pd_tau << ',' << state.expected_b_pd_tau
         << ',' << state.a.temp << ',' << state.b.temp << ','
         << state.a.merror << ',' << state.b.merror << ','
         << state.a.id << ',' << state.b.id << ','
         << static_cast<int>(state.a.send_recv) << ','
         << static_cast<int>(state.b.send_recv) << ','
         << static_cast<int>(state.a.correct) << ','
         << static_cast<int>(state.b.correct) << ','
         << static_cast<int>(state.a.crc_ok) << ','
         << static_cast<int>(state.b.crc_ok) << ','
         << static_cast<int>(state.a.valid) << ',' << static_cast<int>(state.b.valid)
         << ',' << termination_reason_ << ',' << period_ms << ',' << jitter_ms << '\n';
    csv_.flush();
    if (!csv_) throw std::runtime_error("CSV_WRITE_OR_FLUSH_FAILED");
  }

  static std::vector<double> tail_field(const std::vector<PairState>& values,
                                        bool sync, std::size_t count) {
    if (values.size() < count) throw std::runtime_error("ENDPOINT_FEEDBACK_INSUFFICIENT");
    std::vector<double> result;
    result.reserve(count);
    for (auto it = values.end() - static_cast<std::ptrdiff_t>(count);
         it != values.end(); ++it)
      result.push_back(sync ? it->e_sync : it->q_j2);
    return result;
  }

  int run_dual_hold() {
    const auto capture = capture_references();
    establish_references(capture.first, capture.second);
    const auto hold = run_target_hold("DUAL_CURRENT_HOLD", 0.0, 1.5,
                                      config_.target_kp, false);
    if (hold.size() != 150U) throw std::runtime_error("DUAL_HOLD_FEEDBACK_INSUFFICIENT");
    double max_sync = 0.0;
    double max_motion = 0.0;
    for (const PairState& state : hold) {
      max_sync = std::max(max_sync, std::abs(state.e_sync));
      max_motion = std::max(max_motion, std::abs(state.q_j2));
    }
    const double final_sync = std::abs(median(tail_field(hold, true, 50U)));
    const double final_position = std::abs(median(tail_field(hold, false, 50U)));
    const bool numeric_pass = final_sync <= 0.3 * kPi / 180.0 &&
                              max_motion <= 1.0 * kPi / 180.0;
    const bool brake = terminal_brake();
    std::cout << std::setprecision(17)
              << "DUAL_SESSION_A0_RAW_RAD=" << ref_a_ << '\n'
              << "DUAL_SESSION_B0_RAW_RAD=" << ref_b_ << '\n'
              << "DUAL_KP=0.50\nDUAL_KD=0.05\nDUAL_TFF=0\n"
              << "DUAL_HOLD_FINAL_ESYNC_DEG=" << final_sync * 180.0 / kPi << '\n'
              << "DUAL_HOLD_MAX_ESYNC_DEG=" << max_sync * 180.0 / kPi << '\n'
              << "DUAL_HOLD_MAX_LOGICAL_MOTION_DEG=" << max_motion * 180.0 / kPi << '\n'
              << "DUAL_HOLD_FINAL_LOGICAL_POSITION_DEG="
              << final_position * 180.0 / kPi << '\n'
              << "DUAL_HOLD_NUMERIC_RESULT=" << (numeric_pass ? "PASS" : "FAIL") << '\n'
              << "FINAL_DUAL_5_FRAME_BRAKE=" << (brake ? "PASS" : "FAIL") << '\n';
    return numeric_pass && brake ? 0 : 2;
  }

  std::vector<PairState> run_kp_ramp() {
    const int intervals = static_cast<int>(
        std::lround(config_.ramp_duration * kHz));
    std::vector<PairState> values;
    values.reserve(static_cast<std::size_t>(intervals + 1));
    Clock::time_point next = active_started_ ? next_active_ : Clock::now();
    active_started_ = true;
    for (int frame = 0; frame <= intervals; ++frame) {
      const double fraction = smoothstep5(
          static_cast<double>(frame) / static_cast<double>(intervals));
      const double kp = kBaseKp + (config_.target_kp - kBaseKp) * fraction;
      const PairState state = transact_pair(
          kFocMode, 0.0, 0.0, kp, "KP_RAMP_HOLD", next);
      if (state.valid) values.push_back(state);
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
    }
    next_active_ = next;
    return values;
  }

  struct EndpointSummary {
    double actual = 0.0;
    double error = 0.0;
    double sync = 0.0;
  };

  EndpointSummary accept_endpoint(const std::string& label,
                                  const std::vector<PairState>& values,
                                  double target, double position_tolerance,
                                  std::size_t tail_count) {
    EndpointSummary summary;
    summary.actual = median(tail_field(values, false, tail_count));
    summary.error = std::abs(summary.actual - target);
    summary.sync = std::abs(median(tail_field(values, true, tail_count)));
    const bool pass = endpoint_accepted(
        summary.error, summary.sync, max_motion_sync_, position_tolerance);
    std::cout << std::setprecision(17)
              << label << "_ACTUAL_DEG=" << summary.actual * 180.0 / kPi << '\n'
              << label << "_ERROR_DEG=" << summary.error * 180.0 / kPi << '\n'
              << label << "_ESYNC_DEG=" << summary.sync * 180.0 / kPi << '\n'
              << label << "_GATE=" << (pass ? "PASS" : "FAIL") << '\n';
    if (!pass)
      throw std::runtime_error(label + "_ACCEPTANCE_FAILED_STOP_ROUTE");
    return summary;
  }

  int run_dual_motion() {
    const auto capture = capture_references();
    establish_references(capture.first, capture.second);

    const auto ramp = run_kp_ramp();
    const auto hold = run_target_hold("CURRENT_HOLD", 0.0, 1.0,
                                      config_.target_kp, false);
    if (ramp.size() != static_cast<std::size_t>(
            std::lround(config_.ramp_duration * kHz) + 1) ||
        hold.size() != 100U)
      throw std::runtime_error("HOLD_FEEDBACK_INSUFFICIENT");
    const double hold_final_sync =
        std::abs(median(tail_field(hold, true, 50U)));
    double hold_max_sync = 0.0;
    double hold_max_motion = 0.0;
    for (const auto* sequence : {&ramp, &hold}) {
      for (const PairState& state : *sequence) {
        hold_max_sync = std::max(hold_max_sync, std::abs(state.e_sync));
        hold_max_motion = std::max(hold_max_motion, std::abs(state.q_j2));
      }
    }
    const bool hold_pass = hold_final_sync <= kEndpointSyncAccept &&
        hold_max_sync <= kEndpointSyncAccept &&
        hold_max_motion <= 1.0 * kPi / 180.0;
    std::cout << std::setprecision(17)
              << "RUN=" << cli_.phase << '\n'
              << "DUAL_SESSION_A0_RAW_RAD=" << ref_a_ << '\n'
              << "DUAL_SESSION_B0_RAW_RAD=" << ref_b_ << '\n'
              << "SESSION_CENTER_SOURCE=50_VALID_BRAKE_FRAME_MEDIAN\n"
              << "SESSION_CENTER_REDEFINED=NO\n"
              << "KP_START=" << kBaseKp << '\n'
              << "KP_TARGET=" << config_.target_kp << '\n'
              << "KP_RAMP_DURATION_S=" << config_.ramp_duration << '\n'
              << "KD=" << kKd << '\n'
              << "TFF=0\n"
              << "HOLD_FINAL_ESYNC_DEG=" << hold_final_sync * 180.0 / kPi << '\n'
              << "HOLD_MAX_ESYNC_DEG=" << hold_max_sync * 180.0 / kPi << '\n'
              << "HOLD_MAX_LOGICAL_MOTION_DEG="
              << hold_max_motion * 180.0 / kPi << '\n'
              << "HOLD_NUMERIC_RESULT=" << (hold_pass ? "PASS" : "FAIL") << '\n';
    if (!hold_pass)
      throw std::runtime_error("HOLD_ACCEPTANCE_FAILED_STOP_ROUTE");

    max_motion_sync_ = 0.0;
    max_common_error_ = 0.0;
    run_profile("PLUS_5_PROFILE", 0.0, 5.0 * kPi / 180.0,
                config_.target_kp);
    const auto plus_values = run_target_hold(
        "PLUS_5_ENDPOINT", 5.0 * kPi / 180.0, 0.4,
        config_.target_kp, true);
    const EndpointSummary plus = accept_endpoint(
        "PLUS_5", plus_values, 5.0 * kPi / 180.0,
        1.0 * kPi / 180.0, 30U);

    run_profile("FIRST_CENTER_PROFILE", 5.0 * kPi / 180.0,
                -5.0 * kPi / 180.0, config_.target_kp);
    const auto first_values = run_target_hold(
        "FIRST_CENTER_ENDPOINT", 0.0, 0.4, config_.target_kp, true);
    const EndpointSummary first = accept_endpoint(
        "FIRST_CENTER", first_values, 0.0,
        0.75 * kPi / 180.0, 30U);

    run_profile("MINUS_5_PROFILE", 0.0, -5.0 * kPi / 180.0,
                config_.target_kp);
    const auto minus_values = run_target_hold(
        "MINUS_5_ENDPOINT", -5.0 * kPi / 180.0, 0.4,
        config_.target_kp, true);
    const EndpointSummary minus = accept_endpoint(
        "MINUS_5", minus_values, -5.0 * kPi / 180.0,
        1.0 * kPi / 180.0, 30U);

    run_profile("FINAL_CENTER_PROFILE", -5.0 * kPi / 180.0,
                5.0 * kPi / 180.0, config_.target_kp);
    const auto final_values = run_target_hold(
        "FINAL_CENTER_ENDPOINT", 0.0, 0.5, config_.target_kp, true);
    const EndpointSummary final = accept_endpoint(
        "FINAL_CENTER", final_values, 0.0,
        0.75 * kPi / 180.0, 40U);

    termination_reason_ = "COMPLETED_ROUTE";
    const bool brake = terminal_brake();
    std::cout << std::setprecision(17)
              << "V15_24E_CENTER_GATE_ISSUE_FIXED=YES\n"
              << "DUAL_ROUTE=0_TO_PLUS5_TO_0_TO_MINUS5_TO_0\n"
              << "PLUS_5_LOGICAL_ACTUAL_DEG=" << plus.actual * 180.0 / kPi << '\n'
              << "PLUS_5_LOGICAL_ERROR_DEG=" << plus.error * 180.0 / kPi << '\n'
              << "FIRST_CENTER_ACTUAL_DEG=" << first.actual * 180.0 / kPi << '\n'
              << "FIRST_CENTER_ERROR_DEG=" << first.error * 180.0 / kPi << '\n'
              << "MINUS_5_LOGICAL_ACTUAL_DEG=" << minus.actual * 180.0 / kPi << '\n'
              << "MINUS_5_LOGICAL_ERROR_DEG=" << minus.error * 180.0 / kPi << '\n'
              << "FINAL_CENTER_ACTUAL_DEG=" << final.actual * 180.0 / kPi << '\n'
              << "FINAL_CENTER_ERROR_DEG=" << final.error * 180.0 / kPi << '\n'
              << "MAX_MOTION_ESYNC_DEG=" << max_motion_sync_ * 180.0 / kPi << '\n'
              << "MAX_COMMON_MODE_ERROR_DEG=" << max_common_error_ * 180.0 / kPi << '\n'
              << "MAX_LOGICAL_PAIRED_TAU_FEEDBACK_NM="
              << max_logical_tau_feedback_ << '\n'
              << "DUAL_MOTION_NUMERIC_RESULT=PASS\n"
              << "FINAL_DUAL_5_FRAME_BRAKE=" << (brake ? "PASS" : "FAIL") << '\n';
    std::cout << "V15_24F_FT_J2_DRIVE_AUTHORITY_RESULT="
              << (brake ? "PASS" : "FAIL") << '\n';
    return brake ? 0 : 2;
  }

  const Cli& cli_;
  const LevelConfig config_;
  ProcessLock lock_;
  std::ofstream csv_;
  SerialPort serial_;
  Clock::time_point origin_;
  Unwrapper unwrap_a_;
  Unwrapper unwrap_b_;
  double ref_a_ = 0.0;
  double ref_b_ = 0.0;
  bool reference_set_ = false;
  bool terminal_brake_done_ = false;
  bool final_brake_pass_ = false;
  bool have_previous_ = false;
  double previous_s_ = 0.0;
  std::uint64_t tick_ = 0;
  bool active_started_ = false;
  Clock::time_point next_active_{};
  double max_motion_sync_ = 0.0;
  double max_common_error_ = 0.0;
  double max_logical_tau_feedback_ = 0.0;
  std::string termination_reason_ = "RUNNING";
};

void self_test() {
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode ||
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC) != kFocMode ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) - kGear) > 1e-6)
    throw std::runtime_error("SDK_AUTHORITY_MISMATCH");
  MotorCmd brake_a = brake_command(kIdA);
  MotorCmd brake_b = brake_command(kIdB);
  MotorCmd foc100_a = make_command(kIdA, kFocMode, 1.0, -0.2,
                                   1.00, kKd, kTauFf);
  MotorCmd foc100_b = make_command(kIdB, kFocMode, 2.0, 0.2,
                                   1.00, kKd, kTauFf);
  MotorCmd foc140_a = make_command(kIdA, kFocMode, 1.0, -0.2,
                                   1.40, kKd, kTauFf);
  const std::uint8_t* p100a = foc100_a.get_motor_send_data();
  const std::uint8_t* p100b = foc100_b.get_motor_send_data();
  const std::uint8_t* p140a = foc140_a.get_motor_send_data();
  if (p100a != nullptr && p100b != nullptr && p140a != nullptr)
    std::cout << "SELFTEST_KP100_COUNT=" << load_u16_le(p100a + 11) << '\n'
              << "SELFTEST_KP140_COUNT=" << load_u16_le(p140a + 11) << '\n'
              << "SELFTEST_KD_COUNT=" << load_u16_le(p100a + 13) << '\n';
  if (p100a == nullptr || p100b == nullptr || p140a == nullptr ||
      p100a[2] != static_cast<std::uint8_t>(kIdA | (kFocMode << 4)) ||
      p100b[2] != static_cast<std::uint8_t>(kIdB | (kFocMode << 4)) ||
      load_u16_le(p100a + 3) != 0U || load_u16_le(p100b + 3) != 0U ||
      load_u16_le(p140a + 3) != 0U ||
      load_u16_le(p100a + 11) != 1280U ||
      load_u16_le(p100b + 11) != 1280U ||
      load_u16_le(p140a + 11) != 1791U ||
      load_u16_le(p100a + 13) != 64U ||
      load_u16_le(p100b + 13) != 64U ||
      load_u16_le(p140a + 13) != 64U)
    throw std::runtime_error("DUAL_PACKET_SELF_TEST_FAILED");
  (void)brake_a;
  (void)brake_b;
  Trapezoid plus(5.0 * kPi / 180.0);
  Trapezoid minus(-5.0 * kPi / 180.0);
  if (std::abs(plus.sample(plus.duration()).q - 5.0 * kPi / 180.0) > 1e-12 ||
      std::abs(minus.sample(minus.duration()).q + 5.0 * kPi / 180.0) > 1e-12)
    throw std::runtime_error("PROFILE_SELF_TEST_FAILED");
  for (const auto& ramp_case :
       std::array<std::pair<int, double>, 2>{{{50, 1.00}, {75, 1.40}}}) {
    double previous_kp = kBaseKp;
    for (int frame = 0; frame <= ramp_case.first; ++frame) {
      const double fraction = smoothstep5(
          static_cast<double>(frame) / static_cast<double>(ramp_case.first));
      const double kp = kBaseKp + (ramp_case.second - kBaseKp) * fraction;
      if (kp + 1e-12 < previous_kp || kp < kBaseKp - 1e-12 ||
          kp > ramp_case.second + 1e-12)
        throw std::runtime_error("KP_RAMP_MONOTONIC_SELF_TEST_FAILED");
      previous_kp = kp;
    }
    if (std::abs(previous_kp - ramp_case.second) > 1e-12)
      throw std::runtime_error("KP_RAMP_ENDPOINT_SELF_TEST_FAILED");
  }
  const double q = 5.0 * kPi / 180.0;
  if (!(kSignA * kGear * q < 0.0 && kSignB * kGear * q > 0.0))
    throw std::runtime_error("DUAL_SIGN_SELF_TEST_FAILED");
  if (endpoint_accepted(1.01 * kPi / 180.0, 0.0, 0.0,
                        1.0 * kPi / 180.0) ||
      endpoint_accepted(0.0, 0.31 * kPi / 180.0, 0.0,
                        1.0 * kPi / 180.0) ||
      endpoint_accepted(0.0, 0.0, 0.71 * kPi / 180.0,
                        1.0 * kPi / 180.0) ||
      !endpoint_accepted(1.0 * kPi / 180.0, 0.3 * kPi / 180.0,
                         0.7 * kPi / 180.0, 1.0 * kPi / 180.0))
    throw std::runtime_error("FAIL_CLOSED_ENDPOINT_GATE_SELF_TEST_FAILED");
  std::vector<double> static_capture(50U, 1.0);
  for (std::size_t i = 0; i < static_capture.size(); ++i)
    static_capture[i] += (static_cast<double>(i) - 24.5) *
        (0.19 * kPi / 180.0 * kGear / 49.0);
  std::vector<double> drifting_capture(50U, 1.0);
  for (std::size_t i = 0; i < drifting_capture.size(); ++i)
    drifting_capture[i] += static_cast<double>(i) *
        (0.21 * kPi / 180.0 * kGear / 49.0);
  if (!capture_is_static(static_capture) || capture_is_static(drifting_capture))
    throw std::runtime_error("STATIC_CAPTURE_GATE_SELF_TEST_FAILED");
  const auto raw_from_output_deg = [](double degrees) {
    return degrees * kPi / 180.0 * kGear;
  };
  std::vector<double> span_boundary(50U, 0.0);
  span_boundary[0] = raw_from_output_deg(-0.1);
  span_boundary[1] = raw_from_output_deg(0.1);
  std::vector<double> span_fail = span_boundary;
  span_fail[0] = raw_from_output_deg(-0.1005);
  span_fail[1] = raw_from_output_deg(0.1005);
  std::vector<double> tail_boundary(50U, 0.0);
  std::fill(tail_boundary.end() - 10, tail_boundary.end(),
            raw_from_output_deg(0.1));
  std::vector<double> tail_fail(50U, 0.0);
  std::fill(tail_fail.end() - 10, tail_fail.end(),
            raw_from_output_deg(0.101));
  if (!capture_is_static(span_boundary) || capture_is_static(span_fail) ||
      !capture_is_static(tail_boundary) || capture_is_static(tail_fail))
    throw std::runtime_error("STATIC_CAPTURE_BOUNDARY_SELF_TEST_FAILED");
  std::cout << "V15_24F_FT_J2_DRIVE_AUTHORITY_SELF_TEST=PASS\n"
            << "SERIAL_PORT_CONSTRUCTED=NO\n"
            << "J2A_ID=0\nJ2B_ID=1\n"
            << "SIGN_A=-1\nSIGN_B=+1\nSIGN_RELATION=OPPOSITE\n"
            << "SEPARATE_RAW_CURRENT_TARGETS=YES\n"
            << "ONLY_KP_CHANGED=YES\nBASE_KP=0.60\n"
            << "LEVEL_A_KP=1.00\nLEVEL_A_RAMP_S=0.50\n"
            << "LEVEL_B_KP=1.40\nLEVEL_B_RAMP_S=0.75\n"
            << "KP_RAMP_EXACT_51_AND_76_FRAME_SELF_TEST=PASS\n"
            << "DUAL_KD=0.05\nDUAL_TFF=0\n"
            << "POST_RAMP_HOLD_DURATION_S=1.0\n"
            << "MOTION_ROUTE=0_TO_PLUS5_TO_0_TO_MINUS5_TO_0\n"
            << "SESSION_CENTER_CAPTURE_FRAMES=50\n"
            << "SESSION_CENTER_STATIC_SPAN_MAX_DEG=0.2\n"
            << "SESSION_CENTER_LAST10_MEDIAN_OFFSET_MAX_DEG=0.1\n"
            << "STATIC_CAPTURE_GATE_SELF_TEST=PASS\n"
            << "TERMINATION_REASON_IN_FINAL_BRAKE_ROWS=YES\n"
            << "SESSION_CENTER_REDEFINED=NO\n"
            << "FAIL_CLOSED_ENDPOINT_GATE_SELF_TEST=PASS\n"
            << "V15_24E_CENTER_GATE_ISSUE_FIXED=YES\n"
            << "SYNC_HOLD_ACCEPT_DEG=0.3\nSYNC_MOTION_ACCEPT_DEG=0.7\n"
            << "SYNC_HARD_ABORT_DEG=1.0\n"
            << "UNEXPECTED_LOGICAL_VELOCITY_ABORT_DEG_S=25\n"
            << "CALIBRATE_PATH=NO\nID_WRITE_PATH=NO\nZERO_WRITE_PATH=NO\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::signal(SIGHUP, signal_handler);
  std::signal(SIGQUIT, signal_handler);
  try {
    const Cli cli = parse_cli(argc, argv);
    if (cli.self_test) {
      self_test();
      return 0;
    }
    Runner runner(cli);
    return runner.run();
  } catch (const std::exception& error) {
    std::cerr << "V15_24F_FT_J2_DRIVE_AUTHORITY_RESULT=FAIL\nREASON="
              << error.what() << '\n';
    return 2;
  }
}
