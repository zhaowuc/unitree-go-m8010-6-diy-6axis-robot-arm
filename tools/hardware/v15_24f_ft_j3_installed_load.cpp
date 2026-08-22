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
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
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
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0";
constexpr char kLockPath[] = "/tmp/v15_23c_j3_bus.lock";
constexpr char kConfigurationGate[] =
    "J3_FINAL_INSTALLED_LOAD_CONFIGURATION_CONFIRMED=YES";
constexpr char kClearanceGate[] = "J3_PLUS_MINUS_5_CLEAR=YES";
constexpr char kBrakeGate[] = "J3_EMERGENCY_BRAKE_READY=YES";
constexpr char kPinchGate[] = "J3_OPERATOR_CLEAR=YES";
constexpr char kPowerGate[] = "V15_24F_24V_POWER_ON_CONFIRMED=YES";
constexpr int kJ3 = 3;
constexpr int kJ4 = 4;
constexpr int kJ5 = 5;
constexpr int kBrakeMode = 0;
constexpr int kFocMode = 1;
constexpr double kPi = 3.14159265358979323846;
constexpr double kGear = 6.3299999237060547;
constexpr double kKpRunA = 0.60;
constexpr double kKpRunB = 0.70;
constexpr double kKd = 0.05;
constexpr double kTauFf = 0.0;
constexpr double kHz = 100.0;
constexpr double kPeriod = 0.01;
constexpr double kVmax = 10.0 * kPi / 180.0;
constexpr double kAccel = 30.0 * kPi / 180.0;
constexpr double kCommandEnvelope = 6.0 * kPi / 180.0;
constexpr double kFeedbackEnvelope = 8.0 * kPi / 180.0;
constexpr double kUnexpectedVelocityAbort = 25.0 * kPi / 180.0;
constexpr double kBrakeStationaritySpanDeg = 0.2;
constexpr double kBrakeStationarityTailMedianDeltaDeg = 0.1;
constexpr int kTempLimit = 60;
constexpr char kRepoRoot[] = "/home/car/go-m8010-robot-arm-v15-20a";
constexpr char kSourceHead[] =
    "528abf487160476ece450c0f6c635e19476ac8d7";
constexpr char kEligibilityPath[] = "/tmp/v15_30a_j3_next_phase_gate.json";
constexpr char kCsvHeader[] =
    "tick,timestamp_s,phase,run_label,kp_cmd_literal,kp_cmd_count,"
    "kp_cmd_decoded,kd_cmd_literal,kd_cmd_count,kd_cmd_decoded,"
    "tff_cmd_literal,tff_cmd_count,tff_cmd_decoded,j3_mode,"
    "q_cmd_motor_rad,dq_cmd_motor_rad_s,q_feedback_motor_rad,"
    "q_relative_ros_rad,q_ref_rad,position_error_rad,expected_pd_tau_nm,"
    "dq_feedback_motor_rad_s,dq_logical_rad_s,fast_velocity_deg_s,"
    "slow_velocity_deg_s,tau_feedback_nm,"
    "logical_tau_feedback_nm,temperature_c,merror,received_id,"
    "send_recv,correct,crc_ok,valid,abort_reason,"
    "j4_topology,j4_mode,j4_temperature_c,j4_merror,j4_received_id,"
    "j4_send_recv,j4_correct,j4_crc_ok,j4_valid,"
    "j5_topology,j5_mode,j5_temperature_c,j5_merror,j5_received_id,"
    "j5_send_recv,j5_correct,j5_crc_ok,j5_valid,"
    "final_brake_row_pass,cycle_period_ms,cycle_jitter_ms";

std::ofstream open_evidence_csv(const std::string& path) {
  std::ofstream csv(path, std::ios::out | std::ios::trunc);
  if (!csv) throw std::runtime_error("CSV_OPEN_FAILED");
  csv << kCsvHeader << '\n';
  csv.flush();
  if (!csv) throw std::runtime_error("CSV_HEADER_WRITE_OR_FLUSH_FAILED");
  return csv;
}

volatile std::sig_atomic_t g_stop = 0;
void signal_handler(int) { g_stop = 1; }

bool position_accepted(double error_deg, double tolerance_deg) {
  return error_deg <= tolerance_deg;
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

MotorCmd make_command(int id, int mode, double q, double dq, double kp,
                      double kd, double tau) {
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

std::string invalid_feedback_reason(const Feedback& f, int expected_id,
                                    int expected_mode) {
  if (!f.send_recv) return "SEND_RECV_FAILED";
  if (!f.correct) return "SDK_CORRECT_FALSE";
  if (!f.crc_ok) return "CRC_INVALID";
  if (f.id != expected_id) return "RECEIVED_ID_MISMATCH";
  if (f.mode != expected_mode) return "MODE_MISMATCH";
  if (f.merror != 0) return "MERROR_NONZERO";
  if (f.temp < 0 || f.temp >= kTempLimit)
    return "TEMPERATURE_INVALID_OR_LIMIT";
  if (!std::isfinite(f.q)) return "Q_FEEDBACK_NONFINITE";
  if (!std::isfinite(f.dq)) return "DQ_FEEDBACK_NONFINITE";
  if (!std::isfinite(f.tau)) return "TAU_FEEDBACK_NONFINITE";
  return "FEEDBACK_INVALID_UNCLASSIFIED";
}

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

class VelocityEstimator {
 public:
  void reset() { samples_.clear(); }
  void update(double t, double q) {
    samples_.emplace_back(t, q);
    while (samples_.size() > 11U) samples_.pop_front();
    fast_ = samples_.size() >= 5U ? slope(5U) : 0.0;
    if (samples_.size() >= 11U) {
      slow_ = slope(11U);
      valid_ = true;
    } else {
      slow_ = 0.0;
      valid_ = false;
    }
  }
  double fast_deg_s() const { return fast_ * 180.0 / kPi; }
  double slow_deg_s() const { return slow_ * 180.0 / kPi; }
  bool valid() const { return valid_; }
 private:
  double slope(std::size_t count) const {
    const std::size_t first = samples_.size() - count;
    double mt = 0.0, mq = 0.0;
    for (std::size_t i = first; i < samples_.size(); ++i) {
      mt += samples_[i].first;
      mq += samples_[i].second;
    }
    mt /= static_cast<double>(count);
    mq /= static_cast<double>(count);
    double num = 0.0, den = 0.0;
    for (std::size_t i = first; i < samples_.size(); ++i) {
      const double dt = samples_[i].first - mt;
      num += dt * (samples_[i].second - mq);
      den += dt * dt;
    }
    if (!(den > 0.0)) throw std::runtime_error("VELOCITY_ESTIMATOR_INVALID");
    return num / den;
  }
  std::deque<std::pair<double, double>> samples_;
  double fast_ = 0.0;
  double slow_ = 0.0;
  bool valid_ = false;
};

struct Cli {
  std::string phase;
  std::string output;
  std::string configuration;
  std::string clearance;
  std::string brake_gate;
  std::string pinch_gate;
  std::string power_on_gate;
  std::string authorization_gate;
  std::string eligibility_file;
  std::string j4_topology;
  std::string j5_topology;
  int ros_sign = 0;
  double kp = 0.0;
  bool self_test = false;
};

bool phase_gate_safety_contract_accepted(const std::string& phase,
                                         const nlohmann::json& gate) {
  try {
    if (gate.at("center_gate_fix").get<std::string>() != "YES" ||
        gate.at("timing_100hz_result").get<std::string>() != "PASS" ||
        gate.at("hold_result").get<std::string>() != "PASS")
      return false;
    if (phase == "j3-kp070-run1")
      return gate.at("safe_tracking_fail_eligible").get<bool>();
    return gate.at("prerequisite_result").get<std::string>() == "PASS";
  } catch (...) {
    return false;
  }
}

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
    else if (arg == "--final-installed-load-configuration")
      cli.configuration = value();
    else if (arg == "--clearance") cli.clearance = value();
    else if (arg == "--brake-ready") cli.brake_gate = value();
    else if (arg == "--operator-clear") cli.pinch_gate = value();
    else if (arg == "--power-on") cli.power_on_gate = value();
    else if (arg == "--authorization-gate") cli.authorization_gate = value();
    else if (arg == "--eligibility-file") cli.eligibility_file = value();
    else if (arg == "--j4-topology") cli.j4_topology = value();
    else if (arg == "--j5-topology") cli.j5_topology = value();
    else if (arg == "--raw-to-ros-sign") cli.ros_sign = std::stoi(value());
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  if (cli.self_test) return cli;
  const bool allowed_phase = cli.phase == "j3-kp060-run1" ||
      cli.phase == "j3-kp060-repeat" ||
      cli.phase == "j3-kp070-run1" ||
      cli.phase == "j3-kp070-repeat";
  if (!allowed_phase) {
    throw std::runtime_error("PHASE_NOT_ALLOWED");
  }
  cli.kp = cli.phase == "j3-kp060-run1" ||
      cli.phase == "j3-kp060-repeat" ? kKpRunA : kKpRunB;
  const std::string expected_output =
      cli.phase == "j3-kp060-run1"
          ? "hardware/v15_30a_ft/j3_installed_run.csv"
      : cli.phase == "j3-kp060-repeat"
          ? "hardware/v15_30a_ft/j3_installed_repeat.csv"
      : cli.phase == "j3-kp070-run1"
          ? "hardware/v15_30a_ft/j3_kp070_run.csv"
          : "hardware/v15_30a_ft/j3_kp070_repeat.csv";
  if (cli.output != expected_output) {
    throw std::runtime_error("OUTPUT_PATH_NOT_ALLOWED");
  }
  if (std::filesystem::exists(std::string(kRepoRoot) + "/" + cli.output))
    throw std::runtime_error("EVIDENCE_OUTPUT_ALREADY_EXISTS_REFUSE_OVERWRITE");
  if (cli.eligibility_file != kEligibilityPath)
    throw std::runtime_error("NEXT_PHASE_ELIGIBILITY_GATE_MISSING");
  if ((cli.j4_topology != "connected" && cli.j4_topology != "disconnected") ||
      (cli.j5_topology != "connected" && cli.j5_topology != "disconnected")) {
    throw std::runtime_error("AUX_TOPOLOGY_REQUIRED");
  }
  const bool active_gate = cli.configuration == kConfigurationGate &&
      cli.clearance == kClearanceGate && cli.brake_gate == kBrakeGate &&
      cli.pinch_gate == kPinchGate && cli.power_on_gate == kPowerGate;
  if (!active_gate) {
    throw std::runtime_error("ACTIVE_MOTION_OPERATOR_GATE_MISSING");
  }
  if (cli.ros_sign != 1) {
    throw std::runtime_error("J3_FROZEN_SIGN_MUST_BE_PLUS_ONE");
  }
  const std::string expected_auth =
      cli.phase == "j3-kp060-run1"
          ? "J3_KP060_RUN1_AUTHORIZED=YES"
      : cli.phase == "j3-kp060-repeat"
          ? "J3_KP060_EXACT_REPEAT_AUTHORIZED=YES"
      : cli.phase == "j3-kp070-run1"
          ? "J3_KP070_RUN1_AUTHORIZED=YES"
          : "J3_KP070_EXACT_REPEAT_AUTHORIZED=YES";
  if (cli.authorization_gate != expected_auth) {
    throw std::runtime_error("J3_RUN_AUTHORIZATION_MISSING");
  }
  std::ifstream gate_stream(cli.eligibility_file);
  if (!gate_stream) throw std::runtime_error("NEXT_PHASE_GATE_OPEN_FAILED");
  nlohmann::json gate;
  gate_stream >> gate;
  std::string prerequisite;
  std::string reason;
  if (cli.phase == "j3-kp060-run1") {
    prerequisite = "";
    reason = "J3_INDEPENDENT_OPERATOR_AUTHORIZATION";
  } else if (cli.phase == "j3-kp060-repeat") {
    prerequisite = "hardware/v15_30a_ft/j3_installed_run.csv";
    reason = "J3_KP060_RUN1_PASS";
  } else if (cli.phase == "j3-kp070-run1") {
    prerequisite = "hardware/v15_30a_ft/j3_installed_run.csv";
    reason = "J3_KP060_RUN1_SAFE_TRACKING_FAIL";
  } else {
    prerequisite = "hardware/v15_30a_ft/j3_kp070_run.csv";
    reason = "J3_KP070_RUN1_PASS";
  }
  if (gate.at("schema").get<std::string>() !=
          "V15_30A_J3_NEXT_PHASE_GATE_V1" ||
      !gate.at("eligible").get<bool>() ||
      gate.at("allowed_phase").get<std::string>() != cli.phase ||
      gate.at("reason").get<std::string>() != reason ||
      gate.at("source_head").get<std::string>() != kSourceHead ||
      gate.at("prerequisite_csv_path").get<std::string>() != prerequisite ||
      gate.at("operator_observation").get<std::string>() != "SAFE" ||
      gate.at("final_brake").get<std::string>() != "PASS" ||
      !phase_gate_safety_contract_accepted(cli.phase, gate))
    throw std::runtime_error("NEXT_PHASE_GATE_AUTHORITY_MISMATCH");
  if (cli.phase == "j3-kp060-run1") {
    if (gate.at("j2_policy").get<std::string>() != "STATE_ONLY_NO_MOTION" ||
        gate.at("j2_safe_static_state").get<std::string>() != "CONFIRMED" ||
        gate.at("j3_clearance_deg").get<double>() < 5.0 ||
        gate.at("observer_emergency_power_cutoff").get<std::string>() !=
            "READY" ||
        gate.at("final_installed_configuration").get<std::string>() !=
            "CONFIRMED")
      throw std::runtime_error("V15_30A_INDEPENDENT_J3_GATE_MISMATCH");
  } else if (gate.at("prerequisite_csv_sha256").get<std::string>() !=
             sha256_file(std::string(kRepoRoot) + "/" + prerequisite)) {
    throw std::runtime_error("NEXT_PHASE_PREREQUISITE_HASH_MISMATCH");
  }
  return cli;
}

class ProcessLock {
 public:
  ProcessLock() {
    fd_ = ::open(kLockPath, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd_ < 0 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0)
      throw std::runtime_error("J345_BUS_LOCK_FAILED");
  }
  ~ProcessLock() {
    if (fd_ >= 0) { (void)::flock(fd_, LOCK_UN); (void)::close(fd_); }
  }
 private:
  int fd_ = -1;
};

struct ProfilePoint { double q = 0.0; double dq = 0.0; };

class Trapezoid {
 public:
  explicit Trapezoid(double displacement) : sign_(displacement < 0.0 ? -1.0 : 1.0),
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
    double q = 0.0, v = 0.0;
    const double d_acc = 0.5 * kAccel * t_acc_ * t_acc_;
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
  double sign_ = 1.0, distance_ = 0.0, t_acc_ = 0.0;
  double t_cruise_ = 0.0, v_peak_ = 0.0, duration_ = 0.0;
};

double median(std::vector<double> values) {
  if (values.empty()) throw std::runtime_error("MEDIAN_EMPTY");
  std::sort(values.begin(), values.end());
  const std::size_t m = values.size() / 2U;
  return values.size() % 2U ? values[m] : 0.5 * (values[m - 1U] + values[m]);
}

struct BrakeStationarity {
  double span_deg = std::numeric_limits<double>::infinity();
  double tail_median_delta_deg = std::numeric_limits<double>::infinity();

  bool accepted() const {
    return span_deg <= kBrakeStationaritySpanDeg + 1e-12 &&
           tail_median_delta_deg <=
               kBrakeStationarityTailMedianDeltaDeg + 1e-12;
  }
};

BrakeStationarity evaluate_brake_stationarity(
    const std::vector<double>& values) {
  if (values.size() != 50U)
    throw std::runtime_error("BRAKE_STATIONARITY_REQUIRES_EXACTLY_50_FRAMES");
  const auto minmax = std::minmax_element(values.begin(), values.end());
  const double full_median = median(values);
  const double tail_median = median(std::vector<double>(values.end() - 10,
                                                        values.end()));
  return {
      (*minmax.second - *minmax.first) / kGear * 180.0 / kPi,
      std::abs(tail_median - full_median) / kGear * 180.0 / kPi,
  };
}

double mean(const std::vector<double>& values) {
  return std::accumulate(values.begin(), values.end(), 0.0) /
         static_cast<double>(values.size());
}

double stddev(const std::vector<double>& values) {
  const double m = mean(values);
  double sum = 0.0;
  for (double value : values) { const double d = value - m; sum += d * d; }
  return std::sqrt(sum / static_cast<double>(values.size()));
}

struct Metrics {
  std::vector<double> active_tx_times;
  std::vector<double> q_output_deg;
  std::vector<double> fast_abs;
  std::vector<double> slow_abs;
  std::vector<double> sdk_dq_abs;
  std::vector<double> tau;
  int temp_min = std::numeric_limits<int>::max();
  int temp_max = std::numeric_limits<int>::min();
  int merror_nonzero = 0;
  int invalid_feedback = 0;
  int warning_speed_frames = 0;
  int sustained_over30 = 0;
};

class Runner {
 public:
  Runner(const Cli& cli)
      : cli_(cli), lock_(),
        csv_(open_evidence_csv(std::string(kRepoRoot) + "/" + cli.output)),
        serial_(kPort, 16, 4000000, 20000, BlockYN::NO,
          bytesize_t::eightbits, parity_t::parity_none,
          stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none) {
    origin_ = Clock::now();
  }
  ~Runner() { if (!terminal_brake_done_) safe_brake(); }

  int run() {
    try {
      return run_bidirectional();
    } catch (const std::exception& exc) {
      termination_reason_ = exc.what();
      const bool brake = terminal_brake();
      std::cerr << "FINAL_J3_5_FRAME_BRAKE="
                << (brake ? "PASS" : "FAIL") << '\n';
      throw;
    } catch (...) {
      termination_reason_ = "NON_STD_EXCEPTION";
      const bool brake = terminal_brake();
      std::cerr << "FINAL_J3_5_FRAME_BRAKE="
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
    f.valid = f.send_recv && f.correct && f.crc_ok && f.id == expected_id &&
              f.mode == expected_mode && f.merror == 0 && f.temp >= 0 &&
              f.temp < kTempLimit && std::isfinite(f.q) && std::isfinite(f.dq) &&
              std::isfinite(f.tau);
    return f;
  }

  Feedback poll_brake(int id) {
    MotorCmd command = brake_command(id);
    return transact(command, id, kBrakeMode);
  }

  void check_aux(const Feedback& f, int id) {
    if (!f.valid)
      throw std::runtime_error("AUX_ID" + std::to_string(id) + "_" +
                               invalid_feedback_reason(f, id, kBrakeMode));
  }

  void poll_aux() {
    if (cli_.j4_topology == "connected") {
      j4_ = poll_brake(kJ4); check_aux(j4_, kJ4);
    }
    if (cli_.j5_topology == "connected") {
      j5_ = poll_brake(kJ5); check_aux(j5_, kJ5);
    }
  }

  std::vector<double> capture_brake(int count, const std::string& phase) {
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(count));
    Clock::time_point next = Clock::now();
    for (int i = 0; i < count; ++i) {
      if (g_stop != 0) throw std::runtime_error("OPERATOR_ABORT");
      std::this_thread::sleep_until(next);
      const Clock::time_point begin = Clock::now();
      if (begin > next + std::chrono::milliseconds(2))
        throw std::runtime_error("BRAKE_CAPTURE_DEADLINE_MISS_GT_2MS");
      Feedback f = poll_brake(kJ3);
      if (!f.valid) {
        const std::string reason = "J3_" +
            invalid_feedback_reason(f, kJ3, kBrakeMode);
        ++metrics_.invalid_feedback;
        if (reason == "J3_MERROR_NONZERO") ++metrics_.merror_nonzero;
        log_row(phase, f, 0.0, 0.0,
                std::numeric_limits<double>::quiet_NaN(), begin, false,
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(), -1, reason);
        ++tick_;
        throw std::runtime_error(reason);
      } else {
        values.push_back(j3_unwrap_.update(f.q));
      }
      if (i % 5 == 0) {
        try {
          poll_aux();
        } catch (const std::exception& exc) {
          log_row(phase, f, 0.0, 0.0,
                  std::numeric_limits<double>::quiet_NaN(), begin, false,
                  std::numeric_limits<double>::quiet_NaN(),
                  std::numeric_limits<double>::quiet_NaN(), -1, exc.what());
          ++tick_;
          throw;
        }
      }
      log_row(phase, f, 0.0, 0.0,
              std::numeric_limits<double>::quiet_NaN(), begin, false,
              std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::quiet_NaN(), -1, "");
      ++tick_;
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
    }
    if (static_cast<int>(values.size()) != count)
      throw std::runtime_error("BRAKE_CAPTURE_NOT_FULL_VALID");
    return values;
  }

  void establish_reference(const std::vector<double>& values) {
    q_reference_ = median(values);
    reference_set_ = true;
    velocity_.reset();
  }

  Feedback active_tick(const std::string& phase, double q_output,
                       double dq_output, Clock::time_point scheduled) {
    if (g_stop != 0) throw std::runtime_error("OPERATOR_ABORT");
    if (!reference_set_) throw std::runtime_error("REFERENCE_NOT_SET");
    if (std::abs(q_output) > kCommandEnvelope + 1e-12)
      throw std::runtime_error("COMMAND_ENVELOPE");
    std::this_thread::sleep_until(scheduled);
    const Clock::time_point begin = Clock::now();
    if (g_stop != 0) throw std::runtime_error("OPERATOR_ABORT");
    if (begin > scheduled + std::chrono::milliseconds(2))
      throw std::runtime_error("CONTROL_DEADLINE_MISS_GT_2MS");
    const double q_cmd = q_reference_ + kGear * q_output;
    const double dq_cmd = kGear * dq_output;
    MotorCmd command = make_command(kJ3, kFocMode, q_cmd, dq_cmd, cli_.kp, kKd, kTauFf);
    const std::uint8_t* packet = command.get_motor_send_data();
    last_kp_count_ = load_u16_le(packet + 11);
    last_kd_count_ = load_u16_le(packet + 13);
    last_tff_count_ = static_cast<std::int16_t>(load_u16_le(packet + 3));
    if (last_tff_count_ != 0)
      throw std::runtime_error("TFF_ZERO_CONTRACT_VIOLATION");
    Feedback f = transact(command, kJ3, kFocMode);
    metrics_.active_tx_times.push_back(
        std::chrono::duration<double>(begin - origin_).count());
    double fast = std::numeric_limits<double>::quiet_NaN();
    double slow = std::numeric_limits<double>::quiet_NaN();
    if (!f.valid) {
      const std::string reason = "J3_" +
          invalid_feedback_reason(f, kJ3, kFocMode);
      ++metrics_.invalid_feedback;
      if (reason == "J3_MERROR_NONZERO") ++metrics_.merror_nonzero;
      log_row(phase, f, q_cmd, dq_cmd, q_output, begin, true, fast, slow,
              -1, reason);
      ++tick_;
      throw std::runtime_error(reason);
    } else {
      const double q_unwrapped = j3_unwrap_.update(f.q);
      const double q_rel_output = (q_unwrapped - q_reference_) / kGear;
      if (std::abs(q_rel_output) > kFeedbackEnvelope) {
        const std::string reason = "FEEDBACK_ENVELOPE";
        log_row(phase, f, q_cmd, dq_cmd, q_output, begin, true, fast, slow,
                -1, reason);
        ++tick_;
        throw std::runtime_error(reason);
      }
      const double t = std::chrono::duration<double>(begin - origin_).count();
      velocity_.update(t, q_rel_output);
      fast = std::abs(velocity_.fast_deg_s());
      slow = std::abs(velocity_.slow_deg_s());
      const double sdk_logical = std::abs(f.dq / kGear) * 180.0 / kPi;
      metrics_.q_output_deg.push_back(q_rel_output * 180.0 / kPi);
      metrics_.fast_abs.push_back(fast);
      metrics_.slow_abs.push_back(slow);
      metrics_.sdk_dq_abs.push_back(std::abs(f.dq));
      metrics_.tau.push_back(f.tau);
      metrics_.temp_min = std::min(metrics_.temp_min, f.temp);
      metrics_.temp_max = std::max(metrics_.temp_max, f.temp);
      if (fast > 25.0 || slow > 25.0 || sdk_logical > 25.0) {
        const std::string reason =
            "UNEXPECTED_LOGICAL_VELOCITY_GT_25DEG_S";
        log_row(phase, f, q_cmd, dq_cmd, q_output, begin, true, fast, slow,
                -1, reason);
        ++tick_;
        throw std::runtime_error(reason);
      }
      if (fast > 20.0 || slow > 20.0) ++metrics_.warning_speed_frames;
    }
    if (tick_ % 5U == 0U) {
      try {
        poll_aux();
      } catch (const std::exception& exc) {
        log_row(phase, f, q_cmd, dq_cmd, q_output, begin, true, fast, slow,
                -1, exc.what());
        ++tick_;
        throw;
      }
    }
    log_row(phase, f, q_cmd, dq_cmd, q_output, begin, true, fast, slow, -1,
            "");
    ++tick_;
    return f;
  }

  std::vector<double> run_hold_segment(const std::string& phase, double target,
                                       double duration) {
    const int count = static_cast<int>(std::ceil(duration * kHz));
    std::vector<double> values;
    Clock::time_point next = active_schedule_started_ ? next_active_ : Clock::now();
    active_schedule_started_ = true;
    for (int i = 0; i < count; ++i) {
      Feedback f = active_tick(phase, target, 0.0, next);
      if (f.valid) values.push_back((j3_unwrap_.update(f.q) - q_reference_) / kGear);
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
    }
    next_active_ = next;
    return values;
  }

  void run_profile(const std::string& phase, double start, double displacement) {
    Trapezoid profile(displacement);
    const int count = static_cast<int>(std::ceil(profile.duration() * kHz)) + 1;
    Clock::time_point next = active_schedule_started_ ? next_active_ : Clock::now();
    active_schedule_started_ = true;
    for (int i = 0; i < count; ++i) {
      const double t = std::min(i * kPeriod, profile.duration());
      const ProfilePoint point = profile.sample(t);
      (void)active_tick(phase, start + point.q, point.dq, next);
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
    }
    next_active_ = next;
  }

  bool terminal_brake() {
    bool pass = true;
    Clock::time_point next = Clock::now();
    for (int i = 0; i < 5; ++i) {
      std::this_thread::sleep_until(next);
      Feedback f3;
      bool row_pass = false;
      try {
        f3 = poll_brake(kJ3);
        row_pass = f3.valid;
        if (cli_.j4_topology == "connected") {
          j4_ = poll_brake(kJ4); row_pass = row_pass && j4_.valid;
        }
        if (cli_.j5_topology == "connected") {
          j5_ = poll_brake(kJ5); row_pass = row_pass && j5_.valid;
        }
      } catch (...) {
        row_pass = false;
      }
      pass = pass && row_pass;
      try {
        std::string brake_reason = termination_reason_;
        if (!row_pass) {
          if (!brake_reason.empty()) brake_reason += "|";
          brake_reason += "FINAL_BRAKE_ROW_INVALID";
        }
        log_row("FINAL_BRAKE", f3, 0.0, 0.0,
                std::numeric_limits<double>::quiet_NaN(), Clock::now(), false,
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(), row_pass ? 1 : 0,
                brake_reason);
      } catch (...) {
        safe_brake();
        throw;
      }
      ++tick_;
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
    }
    final_brake_pass_ = pass;
    try {
      csv_.flush();
      if (!csv_) throw std::runtime_error("CSV_FINAL_BRAKE_FLUSH_FAILED");
    } catch (...) {
      safe_brake();
      throw;
    }
    if (pass) terminal_brake_done_ = true;
    else safe_brake();
    return pass;
  }

  void safe_brake() noexcept {
    try {
      Clock::time_point next = Clock::now();
      for (int i = 0; i < 20; ++i) {
        std::this_thread::sleep_until(next);
        MotorCmd j3 = brake_command(kJ3);
        (void)transact(j3, kJ3, kBrakeMode);
        if (cli_.j4_topology == "connected") {
          MotorCmd j4 = brake_command(kJ4);
          (void)transact(j4, kJ4, kBrakeMode);
        }
        if (cli_.j5_topology == "connected") {
          MotorCmd j5 = brake_command(kJ5);
          (void)transact(j5, kJ5, kBrakeMode);
        }
        next += std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(kPeriod));
      }
      terminal_brake_done_ = true;
    } catch (...) {}
  }

  void log_row(const std::string& phase, const Feedback& f, double q_cmd,
               double dq_cmd, double q_ref, Clock::time_point begin,
               bool active, double fast_velocity_deg_s,
               double slow_velocity_deg_s, int final_brake_row_pass,
               const std::string& abort_reason) {
    double q_unwrapped = std::numeric_limits<double>::quiet_NaN();
    double q_rel_motor = std::numeric_limits<double>::quiet_NaN();
    double q_rel_output = std::numeric_limits<double>::quiet_NaN();
    if (f.valid) {
      q_unwrapped = j3_unwrap_.update(f.q);
      if (reference_set_) {
        q_rel_motor = q_unwrapped - q_reference_;
        q_rel_output = q_rel_motor / kGear;
      }
    }
    double period_ms = std::numeric_limits<double>::quiet_NaN();
    double jitter_ms = std::numeric_limits<double>::quiet_NaN();
    const double now_s = std::chrono::duration<double>(begin - origin_).count();
    if (active && have_previous_active_) {
      period_ms = (now_s - previous_active_s_) * 1000.0;
      jitter_ms = std::abs(period_ms - 10.0);
    }
    if (active) { previous_active_s_ = now_s; have_previous_active_ = true; }
    const double ros = (cli_.ros_sign == 1 || cli_.ros_sign == -1)
                           ? q_rel_output * cli_.ros_sign
                           : std::numeric_limits<double>::quiet_NaN();
    const std::uint16_t kp_count = active ? last_kp_count_ : 0U;
    const std::uint16_t kd_count = active ? last_kd_count_ : 0U;
    const std::int16_t tff_count = active ? last_tff_count_ : 0;
    const double position_error = active && std::isfinite(ros)
        ? q_ref - ros : std::numeric_limits<double>::quiet_NaN();
    const double expected_pd = active && std::isfinite(q_unwrapped)
        ? cli_.kp * (q_cmd - q_unwrapped)
        : std::numeric_limits<double>::quiet_NaN();
    const double dq_logical = f.valid
        ? cli_.ros_sign * f.dq / kGear
        : std::numeric_limits<double>::quiet_NaN();
    const double logical_tau = f.valid
        ? cli_.ros_sign * kGear * f.tau
        : std::numeric_limits<double>::quiet_NaN();
    std::string csv_abort_reason = abort_reason;
    for (char& c : csv_abort_reason) {
      if (c == ',' || c == '\n' || c == '\r') c = ';';
    }
    csv_ << tick_ << ',' << std::setprecision(17) << now_s << ',' << phase << ','
         << cli_.phase << ',' << (active ? cli_.kp : 0.0) << ','
         << kp_count << ',' << static_cast<double>(kp_count) / 1280.0 << ','
         << (active ? kKd : 0.0) << ',' << kd_count << ','
         << static_cast<double>(kd_count) / 1280.0 << ','
         << (active ? kTauFf : 0.0) << ',' << tff_count << ','
         << static_cast<double>(tff_count) / 256.0 << ','
         << f.mode << ',' << q_cmd << ',' << dq_cmd << ',' << q_unwrapped << ','
         << ros << ',' << q_ref << ',' << position_error << ',' << expected_pd << ','
         << f.dq << ',' << dq_logical << ',' << fast_velocity_deg_s << ','
         << slow_velocity_deg_s << ',' << f.tau << ',' << logical_tau << ','
         << f.temp << ',' << f.merror << ',' << f.id << ','
         << static_cast<int>(f.send_recv) << ',' << static_cast<int>(f.correct) << ','
         << static_cast<int>(f.crc_ok) << ',' << static_cast<int>(f.valid) << ','
         << csv_abort_reason << ','
         << cli_.j4_topology << ','
         << j4_.mode << ',' << j4_.temp << ',' << j4_.merror << ',' << j4_.id
         << ',' << static_cast<int>(j4_.send_recv)
         << ',' << static_cast<int>(j4_.correct)
         << ',' << static_cast<int>(j4_.crc_ok)
         << ',' << static_cast<int>(j4_.valid) << ',' << cli_.j5_topology << ','
         << j5_.mode << ',' << j5_.temp << ',' << j5_.merror << ',' << j5_.id
         << ',' << static_cast<int>(j5_.send_recv)
         << ',' << static_cast<int>(j5_.correct)
         << ',' << static_cast<int>(j5_.crc_ok)
         << ',' << static_cast<int>(j5_.valid) << ','
         << final_brake_row_pass << ','
         << period_ms << ',' << jitter_ms << '\n';
    csv_.flush();
    if (!csv_) throw std::runtime_error("CSV_WRITE_OR_FLUSH_FAILED");
  }

  void print_common() const {
    double rate = 0.0, jitter = 0.0;
    if (metrics_.active_tx_times.size() >= 2U) {
      const double span = metrics_.active_tx_times.back() - metrics_.active_tx_times.front();
      rate = static_cast<double>(metrics_.active_tx_times.size() - 1U) / span;
      for (std::size_t i = 1; i < metrics_.active_tx_times.size(); ++i) {
        jitter = std::max(jitter, std::abs((metrics_.active_tx_times[i] -
            metrics_.active_tx_times[i - 1U]) * 1000.0 - 10.0));
      }
    }
    auto max_or_zero = [](const std::vector<double>& v) {
      return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());
    };
    const auto minmax_tau = metrics_.tau.empty()
        ? std::pair<double,double>{0.0,0.0}
        : std::pair<double,double>{*std::min_element(metrics_.tau.begin(), metrics_.tau.end()),
                                   *std::max_element(metrics_.tau.begin(), metrics_.tau.end())};
    std::cout << std::setprecision(17)
              << "ACTUAL_COMMAND_RATE_HZ=" << rate << '\n'
              << "MAX_TX_JITTER_MS=" << jitter << '\n'
              << "MAX_QDOT_FAST_DEG_S=" << max_or_zero(metrics_.fast_abs) << '\n'
              << "MAX_QDOT_SLOW_DEG_S=" << max_or_zero(metrics_.slow_abs) << '\n'
              << "MAX_SDK_DQ_RAD_S=" << max_or_zero(metrics_.sdk_dq_abs) << '\n'
              << "TAU_MIN=" << minmax_tau.first << '\n'
              << "TAU_MAX=" << minmax_tau.second << '\n'
              << "TEMP_MIN_C=" << metrics_.temp_min << '\n'
              << "TEMP_MAX_C=" << metrics_.temp_max << '\n'
              << "MERROR_NONZERO_FRAMES=" << metrics_.merror_nonzero << '\n'
              << "INVALID_FEEDBACK_FRAMES=" << metrics_.invalid_feedback << '\n'
              << "FINAL_J3_BRAKE_SENT=YES\n"
              << "FINAL_5_FRAME_BRAKE=" << (final_brake_pass_ ? "PASS" : "FAIL") << '\n'
              << "J4_TOPOLOGY=" << (cli_.j4_topology == "connected" ? "CONNECTED_BRAKE" : "PHYSICALLY_DISCONNECTED") << '\n'
              << "J5_TOPOLOGY=" << (cli_.j5_topology == "connected" ? "CONNECTED_BRAKE" : "PHYSICALLY_DISCONNECTED") << '\n';
  }

  int run_baseline() {
    const auto values = capture_brake(200, "POST_DISASSEMBLY_BRAKE_BASELINE");
    std::cout << std::setprecision(17)
              << "BASELINE_VALID=200\nBASELINE_TOTAL=200\n"
              << "BASELINE_RAW_MEDIAN_RAD=" << median(values) << '\n'
              << "BASELINE_RAW_MEAN_RAD=" << mean(values) << '\n'
              << "BASELINE_RAW_STD_RAD=" << stddev(values) << '\n'
              << "BASELINE_RAW_MIN_RAD=" << *std::min_element(values.begin(), values.end()) << '\n'
              << "BASELINE_RAW_MAX_RAD=" << *std::max_element(values.begin(), values.end()) << '\n';
    const bool brake = terminal_brake();
    std::cout << "FINAL_J3_BRAKE_SENT=YES\nFINAL_5_FRAME_BRAKE="
              << (brake ? "PASS" : "FAIL") << "\nJ4_TOPOLOGY="
              << (cli_.j4_topology == "connected" ? "CONNECTED_BRAKE" : "PHYSICALLY_DISCONNECTED")
              << "\nJ5_TOPOLOGY="
              << (cli_.j5_topology == "connected" ? "CONNECTED_BRAKE" : "PHYSICALLY_DISCONNECTED") << '\n';
    return brake ? 0 : 2;
  }

  int run_session() {
    const auto values = capture_brake(100, "J3_D3_KP060_BRAKE_BASELINE");
    std::cout << std::setprecision(17)
              << "SESSION_VALID=100\nSESSION_TOTAL=100\n"
              << "SESSION_RAW_MEDIAN_RAD=" << median(values) << '\n'
              << "SESSION_RAW_MEAN_RAD=" << mean(values) << '\n'
              << "SESSION_RAW_STD_RAD=" << stddev(values) << '\n'
              << "SESSION_RAW_MIN_RAD=" << *std::min_element(values.begin(), values.end()) << '\n'
              << "SESSION_RAW_MAX_RAD=" << *std::max_element(values.begin(), values.end()) << '\n';
    const bool brake = terminal_brake();
    std::cout << "FINAL_J3_BRAKE_SENT=YES\nFINAL_5_FRAME_BRAKE="
              << (brake ? "PASS" : "FAIL") << "\nJ4_TOPOLOGY=PHYSICALLY_DISCONNECTED"
              << "\nJ5_TOPOLOGY=PHYSICALLY_DISCONNECTED\n";
    return brake ? 0 : 2;
  }

  int run_hold() {
    const auto capture = capture_brake(50, "HOLD_START_CAPTURE");
    establish_reference(capture);
    const auto hold = run_hold_segment("HOLD_CURRENT", 0.0, 2.0);
    if (hold.size() < 190U) throw std::runtime_error("HOLD_FEEDBACK_INSUFFICIENT");
    double max_drift = 0.0;
    for (double q : hold) max_drift = std::max(max_drift, std::abs(q * 180.0 / kPi));
    std::vector<double> final_values(hold.end() - 50, hold.end());
    const double final_error = std::abs(median(final_values) * 180.0 / kPi);
    const bool numeric_pass = max_drift <= 1.0 && final_error <= 0.5;
    const bool brake = terminal_brake();
    std::cout << std::setprecision(17)
              << "Q_HOLD_START_RAW_RAD=" << q_reference_ << '\n'
              << "HOLD_MAX_OUTPUT_DRIFT_DEG=" << max_drift << '\n'
              << "HOLD_FINAL_MEDIAN_ERROR_DEG=" << final_error << '\n'
              << "HOLD_NUMERIC_RESULT=" << (numeric_pass ? "PASS" : "FAIL") << '\n';
    print_common();
    return numeric_pass && brake ? 0 : 2;
  }

  int run_sign() {
    const auto capture = capture_brake(50, "SIGN_START_CAPTURE");
    establish_reference(capture);
    const auto prehold = run_hold_segment("SIGN_PRE_HOLD", 0.0, 0.3);
    if (prehold.size() < 25U) throw std::runtime_error("SIGN_PRE_HOLD_FAILED");
    run_profile("SIGN_PLUS_3_PROFILE", 0.0, 3.0 * kPi / 180.0);
    const auto endpoint = run_hold_segment("SIGN_PLUS_3_HOLD", 3.0 * kPi / 180.0, 0.5);
    const double actual = median(endpoint) * 180.0 / kPi;
    const double endpoint_error = std::abs(actual - 3.0);
    run_profile("SIGN_RETURN_PROFILE", 3.0 * kPi / 180.0, -3.0 * kPi / 180.0);
    const auto returned = run_hold_segment("SIGN_RETURN_HOLD", 0.0, 0.5);
    const double return_error = std::abs(median(returned) * 180.0 / kPi);
    const bool pass = actual > 0.0 && endpoint_error <= 1.0 && return_error <= 0.75;
    const bool brake = terminal_brake();
    std::cout << std::setprecision(17)
              << "Q_SIGN_SESSION_RAW_RAD=" << q_reference_ << '\n'
              << "SIGN_PLUS_3_ACTUAL_DELTA_DEG=" << actual << '\n'
              << "SIGN_PLUS_3_ENDPOINT_ERROR_DEG=" << endpoint_error << '\n'
              << "SIGN_PLUS_3_RETURN_ERROR_DEG=" << return_error << '\n'
              << "SIGN_PROBE_RESULT=" << (pass ? "PASS" : "FAIL") << '\n';
    print_common();
    return pass && brake ? 0 : 2;
  }

  int run_session2() {
    const auto values = capture_brake(100, "SESSION_2_CAPTURE");
    const double med = median(values);
    std::cout << std::setprecision(17)
              << "SESSION_2_VALID=100\nSESSION_2_TOTAL=100\n"
              << "SESSION_2_RAW_MEDIAN_RAD=" << med << '\n'
              << "SESSION_2_RAW_MEAN_RAD=" << mean(values) << '\n'
              << "SESSION_2_RAW_STD_RAD=" << stddev(values) << '\n'
              << "SESSION_2_RAW_MIN_RAD=" << *std::min_element(values.begin(), values.end()) << '\n'
              << "SESSION_2_RAW_MAX_RAD=" << *std::max_element(values.begin(), values.end()) << '\n';
    const bool brake = terminal_brake();
    std::cout << "FINAL_J3_BRAKE_SENT=YES\nFINAL_5_FRAME_BRAKE="
              << (brake ? "PASS" : "FAIL") << "\nJ4_TOPOLOGY=PHYSICALLY_DISCONNECTED"
              << "\nJ5_TOPOLOGY=PHYSICALLY_DISCONNECTED\n";
    return brake ? 0 : 2;
  }

  double accept_j3_endpoint(const std::string& label,
                            const std::vector<double>& values,
                            double target_deg, double tolerance_deg,
                            std::size_t tail_count) {
    if (values.size() < tail_count)
      throw std::runtime_error(label + "_FEEDBACK_INSUFFICIENT");
    const double actual = statistics_median_tail(values, tail_count) * 180.0 / kPi;
    const double error = std::abs(actual - target_deg);
    std::cout << std::setprecision(17)
              << label << "_ACTUAL_DEG=" << actual << '\n'
              << label << "_ERROR_DEG=" << error << '\n'
              << label << "_GATE="
              << (position_accepted(error, tolerance_deg) ? "PASS" : "FAIL")
              << '\n';
    if (!position_accepted(error, tolerance_deg))
      throw std::runtime_error(label + "_ACCEPTANCE_FAILED_STOP_ROUTE");
    return actual;
  }

  static double statistics_median_tail(const std::vector<double>& values,
                                       std::size_t count) {
    return median(std::vector<double>(
        values.end() - static_cast<std::ptrdiff_t>(count), values.end()));
  }

  int run_bidirectional() {
    const auto capture = capture_brake(50, "SESSION_BRAKE_CAPTURE");
    const BrakeStationarity stationarity =
        evaluate_brake_stationarity(capture);
    std::cout << std::setprecision(17)
              << "SESSION_BRAKE_SPAN_DEG=" << stationarity.span_deg << '\n'
              << "SESSION_BRAKE_LAST10_MEDIAN_DELTA_DEG="
              << stationarity.tail_median_delta_deg << '\n'
              << "SESSION_BRAKE_STATIONARITY_GATE="
              << (stationarity.accepted() ? "PASS" : "FAIL") << '\n';
    if (!stationarity.accepted())
      throw std::runtime_error("SESSION_BRAKE_STATIONARITY_GATE_FAILED");
    establish_reference(capture);
    const auto hold = run_hold_segment("CURRENT_HOLD", 0.0, 1.5);
    if (hold.size() != 150U)
      throw std::runtime_error("HOLD_FEEDBACK_INSUFFICIENT");
    double hold_max = 0.0;
    for (double q : hold)
      hold_max = std::max(hold_max, std::abs(q * 180.0 / kPi));
    const double hold_error = std::abs(
        statistics_median_tail(hold, 50U) * 180.0 / kPi);
    const bool hold_pass = hold_max <= 1.0 && hold_error <= 0.5;
    std::cout << std::setprecision(17)
              << "RUN=" << cli_.phase << '\n'
              << "MECHANICAL_STATE=BIG_ARM_AND_FOREARM_INSTALLED\n"
              << "J3_KP=" << cli_.kp << "\nJ3_KD=0.05\nJ3_TFF=0\n"
              << "Q_SESSION_REFERENCE_RAW_RAD=" << q_reference_ << '\n'
              << "SESSION_CENTER_SOURCE=50_VALID_BRAKE_FRAME_MEDIAN\n"
              << "SESSION_CENTER_REDEFINED=NO\n"
              << "INSTALLED_HOLD_RESULT=" << (hold_pass ? "PASS" : "FAIL") << '\n'
              << "INSTALLED_HOLD_MAX_DRIFT_DEG=" << hold_max << '\n'
              << "INSTALLED_HOLD_FINAL_ERROR_DEG=" << hold_error << '\n';
    if (!hold_pass)
      throw std::runtime_error("HOLD_ACCEPTANCE_FAILED_STOP_ROUTE");

    run_profile("PLUS_5_PROFILE", 0.0, 5.0 * kPi / 180.0);
    const auto plus_values = run_hold_segment(
        "PLUS_5_ENDPOINT", 5.0 * kPi / 180.0, 0.4);
    const double plus_actual = accept_j3_endpoint(
        "PLUS_5", plus_values, 5.0, 1.0, 30U);

    run_profile("FIRST_CENTER_PROFILE", 5.0 * kPi / 180.0,
                -5.0 * kPi / 180.0);
    const auto first_values = run_hold_segment("FIRST_CENTER_ENDPOINT", 0.0, 0.4);
    const double first_actual = accept_j3_endpoint(
        "FIRST_CENTER", first_values, 0.0, 0.75, 30U);

    run_profile("MINUS_5_PROFILE", 0.0, -5.0 * kPi / 180.0);
    const auto minus_values = run_hold_segment(
        "MINUS_5_ENDPOINT", -5.0 * kPi / 180.0, 0.4);
    const double minus_actual = accept_j3_endpoint(
        "MINUS_5", minus_values, -5.0, 1.0, 30U);

    run_profile("FINAL_CENTER_PROFILE", -5.0 * kPi / 180.0,
                5.0 * kPi / 180.0);
    const auto final_values = run_hold_segment("FINAL_CENTER_ENDPOINT", 0.0, 0.5);
    const double final_actual = accept_j3_endpoint(
        "FINAL_CENTER", final_values, 0.0, 0.75, 40U);

    termination_reason_ = "COMPLETED_ROUTE";
    const bool brake = terminal_brake();
    const char* const overall_result = brake ? "PASS" : "FAIL_FINAL_BRAKE";
    std::cout << std::setprecision(17)
              << "J3_ROUTE=0_TO_PLUS5_TO_0_TO_MINUS5_TO_0\n"
              << "PLUS_5_ACTUAL_DEG=" << plus_actual << '\n'
              << "PLUS_5_ERROR_DEG=" << std::abs(plus_actual - 5.0) << '\n'
              << "FIRST_CENTER_ACTUAL_DEG=" << first_actual << '\n'
              << "FIRST_CENTER_ERROR_DEG=" << std::abs(first_actual) << '\n'
              << "MINUS_5_ACTUAL_DEG=" << minus_actual << '\n'
              << "MINUS_5_ERROR_DEG=" << std::abs(minus_actual + 5.0) << '\n'
              << "FINAL_CENTER_ACTUAL_DEG=" << final_actual << '\n'
              << "FINAL_CENTER_ERROR_DEG=" << std::abs(final_actual) << '\n'
              << "POSITION_TRACKING_RESULT=" << overall_result << '\n'
              << "COMMUNICATION_RESULT=" << overall_result << '\n'
              << "BIDIRECTIONAL_RESULT=" << overall_result << '\n';
    print_common();
    return brake ? 0 : 2;
  }

  const Cli& cli_;
  ProcessLock lock_;
  std::ofstream csv_;
  SerialPort serial_;
  Clock::time_point origin_;
  Unwrapper j3_unwrap_;
  VelocityEstimator velocity_;
  Feedback j4_, j5_;
  Metrics metrics_;
  double q_reference_ = 0.0;
  bool reference_set_ = false;
  bool terminal_brake_done_ = false;
  bool final_brake_pass_ = false;
  std::string termination_reason_ = "RUNNING";
  std::uint16_t last_kp_count_ = 0;
  std::uint16_t last_kd_count_ = 0;
  std::int16_t last_tff_count_ = 0;
  std::uint64_t tick_ = 0;
  bool have_previous_active_ = false;
  double previous_active_s_ = 0.0;
  bool active_schedule_started_ = false;
  Clock::time_point next_active_{};
};

void self_test() {
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode ||
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC) != kFocMode ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) - kGear) > 1e-6) {
    throw std::runtime_error("SDK_AUTHORITY_MISMATCH");
  }
  MotorCmd brake = brake_command(kJ3);
  MotorCmd foc = make_command(kJ3, kFocMode, 1.0, 0.2, kKpRunA, kKd, kTauFf);
  MotorCmd foc70 = make_command(kJ3, kFocMode, 1.0, 0.2, kKpRunB, kKd, kTauFf);
  const std::uint8_t* foc_packet = foc.get_motor_send_data();
  const std::uint8_t* foc70_packet = foc70.get_motor_send_data();
  if (foc_packet == nullptr || foc70_packet == nullptr ||
      load_u16_le(foc_packet + 3) != 0U ||
      load_u16_le(foc70_packet + 3) != 0U ||
      load_u16_le(foc_packet + 11) != 768U ||
      load_u16_le(foc70_packet + 11) != 895U ||
      load_u16_le(foc_packet + 13) != 64U) {
    throw std::runtime_error("KP_0_60_SDK_ENCODING_INVALID");
  }
  (void)brake; (void)foc; (void)foc70;
  Trapezoid plus(5.0 * kPi / 180.0);
  Trapezoid five(-5.0 * kPi / 180.0);
  const auto p3 = plus.sample(plus.duration());
  const auto p5 = five.sample(five.duration());
  if (std::abs(p3.q - 5.0 * kPi / 180.0) > 1e-12 || p3.dq != 0.0 ||
      std::abs(p5.q + 5.0 * kPi / 180.0) > 1e-12 || p5.dq != 0.0) {
    throw std::runtime_error("TRAPEZOID_SELF_TEST_FAILED");
  }
  if (position_accepted(1.01, 1.0) ||
      position_accepted(0.76, 0.75) ||
      !position_accepted(1.0, 1.0) ||
      !position_accepted(0.75, 0.75))
    throw std::runtime_error("FAIL_CLOSED_ENDPOINT_GATE_SELF_TEST_FAILED");
  const auto motor_radians_from_output_degrees = [](double degrees) {
    return degrees * kPi / 180.0 * kGear;
  };
  std::vector<double> stable(50U, 0.0);
  if (!evaluate_brake_stationarity(stable).accepted())
    throw std::runtime_error("BRAKE_STATIONARITY_STABLE_SELF_TEST_FAILED");
  std::vector<double> span_boundary = stable;
  span_boundary[0] = motor_radians_from_output_degrees(-0.1);
  span_boundary[1] = motor_radians_from_output_degrees(0.1);
  if (!evaluate_brake_stationarity(span_boundary).accepted())
    throw std::runtime_error("BRAKE_STATIONARITY_SPAN_BOUNDARY_SELF_TEST_FAILED");
  std::vector<double> span_fail = stable;
  span_fail[0] = motor_radians_from_output_degrees(-0.1005);
  span_fail[1] = motor_radians_from_output_degrees(0.1005);
  if (evaluate_brake_stationarity(span_fail).accepted())
    throw std::runtime_error("BRAKE_STATIONARITY_SPAN_FAIL_SELF_TEST_FAILED");
  std::vector<double> tail_boundary = stable;
  std::fill(tail_boundary.end() - 10, tail_boundary.end(),
            motor_radians_from_output_degrees(0.1));
  if (!evaluate_brake_stationarity(tail_boundary).accepted())
    throw std::runtime_error("BRAKE_STATIONARITY_TAIL_BOUNDARY_SELF_TEST_FAILED");
  std::vector<double> tail_fail = stable;
  std::fill(tail_fail.end() - 10, tail_fail.end(),
            motor_radians_from_output_degrees(0.101));
  if (evaluate_brake_stationarity(tail_fail).accepted())
    throw std::runtime_error("BRAKE_STATIONARITY_TAIL_FAIL_SELF_TEST_FAILED");
  bool wrong_count_rejected = false;
  try {
    (void)evaluate_brake_stationarity(std::vector<double>(49U, 0.0));
  } catch (const std::runtime_error&) {
    wrong_count_rejected = true;
  }
  if (!wrong_count_rejected)
    throw std::runtime_error("BRAKE_STATIONARITY_COUNT_SELF_TEST_FAILED");
  nlohmann::json pass_gate = {
      {"center_gate_fix", "YES"},
      {"timing_100hz_result", "PASS"},
      {"hold_result", "PASS"},
      {"prerequisite_result", "PASS"},
  };
  if (!phase_gate_safety_contract_accepted("j3-kp060-run1", pass_gate) ||
      !phase_gate_safety_contract_accepted("j3-kp060-repeat", pass_gate) ||
      !phase_gate_safety_contract_accepted("j3-kp070-repeat", pass_gate))
    throw std::runtime_error("PREREQUISITE_PASS_GATE_SELF_TEST_FAILED");
  pass_gate["prerequisite_result"] = "FAIL";
  if (phase_gate_safety_contract_accepted("j3-kp060-repeat", pass_gate))
    throw std::runtime_error("PREREQUISITE_FAIL_GATE_SELF_TEST_FAILED");
  nlohmann::json safe_fail_gate = {
      {"center_gate_fix", "YES"},
      {"timing_100hz_result", "PASS"},
      {"hold_result", "PASS"},
      {"safe_tracking_fail_eligible", true},
  };
  if (!phase_gate_safety_contract_accepted("j3-kp070-run1",
                                            safe_fail_gate))
    throw std::runtime_error("SAFE_TRACKING_FAIL_GATE_SELF_TEST_FAILED");
  safe_fail_gate["safe_tracking_fail_eligible"] = false;
  if (phase_gate_safety_contract_accepted("j3-kp070-run1",
                                           safe_fail_gate))
    throw std::runtime_error("UNSAFE_TRACKING_FAIL_GATE_SELF_TEST_FAILED");
  nlohmann::json common_gate_fail = {
      {"center_gate_fix", "NO"},
      {"timing_100hz_result", "PASS"},
      {"hold_result", "PASS"},
      {"prerequisite_result", "PASS"},
  };
  if (phase_gate_safety_contract_accepted("j3-kp060-run1",
                                           common_gate_fail))
    throw std::runtime_error("COMMON_SAFETY_GATE_SELF_TEST_FAILED");
  Feedback no_feedback;
  if (invalid_feedback_reason(no_feedback, kJ3, kFocMode) !=
      "SEND_RECV_FAILED")
    throw std::runtime_error("INVALID_FEEDBACK_REASON_SELF_TEST_FAILED");
  Feedback motor_error;
  motor_error.send_recv = true;
  motor_error.correct = true;
  motor_error.crc_ok = true;
  motor_error.id = kJ3;
  motor_error.mode = kFocMode;
  motor_error.temp = 25;
  motor_error.merror = 1;
  motor_error.q = 0.0;
  motor_error.dq = 0.0;
  motor_error.tau = 0.0;
  if (invalid_feedback_reason(motor_error, kJ3, kFocMode) !=
      "MERROR_NONZERO")
    throw std::runtime_error("MERROR_REASON_SELF_TEST_FAILED");
  const std::string csv_header(kCsvHeader);
  if (static_cast<std::size_t>(std::count(csv_header.begin(), csv_header.end(),
                                          ',')) != 55U)
    throw std::runtime_error("CSV_FIELD_COUNT_SELF_TEST_FAILED");
  for (const char* field : {
           "fast_velocity_deg_s", "slow_velocity_deg_s", "abort_reason",
           "j4_received_id", "j4_send_recv", "j4_crc_ok", "j4_valid",
           "j4_temperature_c", "j5_received_id", "j5_send_recv",
           "j5_crc_ok", "j5_valid", "j5_temperature_c",
           "final_brake_row_pass"}) {
    if (csv_header.find(field) == std::string::npos)
      throw std::runtime_error("AUX_BRAKE_CSV_SCHEMA_SELF_TEST_FAILED");
  }
  std::cout << "V15_24F_FT_J3_INSTALLED_LOAD_SELF_TEST=PASS\n"
            << "KP_0_60_SDK_ENCODING_VALID=YES\nKP060_ENCODED_COUNT=768\n"
            << "KP_0_70_SDK_ENCODING_VALID=YES\nKP070_ENCODED_COUNT=895\n"
            << "MECHANICAL_STATE=BIG_ARM_AND_FOREARM_INSTALLED\n"
            << "VISIBLE_MOTION_ONLY_PLUS_MINUS_5DEG=YES\n"
            << "PLUS_2_OR_3_CLI_PATH=NO\n"
            << "SERIAL_PORT_CONSTRUCTED=NO\n"
            << "J3_ONLY_FOC_PATH=YES\nJ4_J5_FOC_PATH=NO\n"
            << "SESSION_CENTER_CAPTURE_FRAMES=50\n"
            << "SESSION_CENTER_REDEFINED=NO\n"
            << "BRAKE_STATIONARITY_SPAN_LIMIT_DEG=0.2\n"
            << "BRAKE_STATIONARITY_TAIL_MEDIAN_DELTA_LIMIT_DEG=0.1\n"
            << "BRAKE_STATIONARITY_GATE_SELF_TEST=PASS\n"
            << "NEXT_PHASE_RESULT_GATE_SELF_TEST=PASS\n"
             << "INVALID_FEEDBACK_ABORT_REASON_SELF_TEST=PASS\n"
             << "TERMINATION_REASON_IN_FINAL_BRAKE_ROWS=YES\n"
            << "AUX_BRAKE_CSV_SCHEMA_SELF_TEST=PASS\n"
            << "CSV_FIELD_COUNT=56\n"
            << "ABSOLUTE_100HZ_SCHEDULING=YES\n"
            << "FAIL_CLOSED_ENDPOINT_GATE_SELF_TEST=PASS\n"
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
    if (cli.self_test) { self_test(); return 0; }
    Runner runner(cli);
    return runner.run();
  } catch (const std::exception& exc) {
    std::cerr << "V15_24F_FT_J3_INSTALLED_LOAD_RESULT=FAIL\nREASON="
              << exc.what() << '\n';
    return 2;
  }
}
