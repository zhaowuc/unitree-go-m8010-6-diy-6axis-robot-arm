#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace {

using Clock = std::chrono::steady_clock;
constexpr char kPort[] =
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0";
constexpr char kLockPath[] = "/tmp/v15_23d_ft_j2_channel1.lock";
constexpr char kOutputP0[] = "/tmp/v15_24e_j2_manual_pose_p0.json";
constexpr char kOutputP1[] = "/tmp/v15_24e_j2_manual_pose_p1.json";
constexpr char kOutputCenter[] = "/tmp/v15_24e_j2_manual_pose_center.json";
constexpr int kIdA = 0;
constexpr int kIdB = 1;
constexpr int kBrakeMode = 0;
constexpr int kCapturePairs = 50;
constexpr int kTerminalBrakePairs = 5;
constexpr int kTemperatureLimit = 60;
constexpr double kPeriodSeconds = 0.01;

volatile std::sig_atomic_t g_stop = 0;
void signal_handler(int) { g_stop = 1; }

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

MotorCmd brake_command(int id) {
  MotorCmd command;
  zero_object(command);
  command.motorType = MotorType::GO_M8010_6;
  command.id = static_cast<unsigned short>(id);
  command.mode = static_cast<unsigned short>(kBrakeMode);
  command.q = 0.0F;
  command.dq = 0.0F;
  command.kp = 0.0F;
  command.kd = 0.0F;
  command.tau = 0.0F;
  command.Res.u32 = 0;
  command.modify_data(&command);
  if (command.hex_len != 17) throw std::runtime_error("BRAKE_COMMAND_LENGTH_INVALID");
  const std::uint8_t* raw = command.get_motor_send_data();
  if (raw == nullptr || raw[0] != 0xfeU || raw[1] != 0xeeU ||
      raw[2] != static_cast<std::uint8_t>(id) ||
      load_u16_le(raw + 3) != 0U || load_u16_le(raw + 5) != 0U ||
      load_u16_le(raw + 7) != 0U || load_u16_le(raw + 11) != 0U ||
      load_u16_le(raw + 13) != 0U ||
      load_u16_le(raw + 15) != crc16_kermit(raw, 15))
    throw std::runtime_error("NONZERO_OR_INVALID_BRAKE_PACKET");
  return command;
}

void initialize_feedback(MotorData& data) {
  zero_object(data);
  data.motorType = MotorType::GO_M8010_6;
}

struct Feedback {
  bool send_recv = false;
  bool correct = false;
  bool crc_ok = false;
  bool valid = false;
  int id = -1;
  int mode = -1;
  int temp = -1;
  int merror = -1;
  double q = 0.0;
  double dq = 0.0;
  double tau = 0.0;
};

class Unwrapper {
 public:
  double update(double wrapped) {
    if (!initialized_) {
      initialized_ = true;
      previous_ = wrapped;
      unwrapped_ = wrapped;
      return unwrapped_;
    }
    double delta = wrapped - previous_;
    constexpr double kTwoPi = 6.28318530717958647692;
    while (delta > 3.14159265358979323846) delta -= kTwoPi;
    while (delta < -3.14159265358979323846) delta += kTwoPi;
    unwrapped_ += delta;
    previous_ = wrapped;
    return unwrapped_;
  }

 private:
  bool initialized_ = false;
  double previous_ = 0.0;
  double unwrapped_ = 0.0;
};

double median(std::vector<double> values) {
  if (values.empty()) throw std::runtime_error("MEDIAN_EMPTY");
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  return n % 2U != 0U ? values[n / 2U]
                       : 0.5 * (values[n / 2U - 1U] + values[n / 2U]);
}

struct Cli {
  std::string pose;
  std::string output;
  std::string authorization_gate;
  std::string safety_gate;
  std::string power_gate;
  bool self_test = false;
};

Cli parse_cli(int argc, char** argv) {
  Cli cli;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto value = [&]() {
      if (++index >= argc) throw std::runtime_error("CLI_VALUE_MISSING");
      return std::string(argv[index]);
    };
    if (argument == "--self-test") cli.self_test = true;
    else if (argument == "--pose") cli.pose = value();
    else if (argument == "--output") cli.output = value();
    else if (argument == "--authorization-gate") cli.authorization_gate = value();
    else if (argument == "--safety-gate") cli.safety_gate = value();
    else if (argument == "--power-gate") cli.power_gate = value();
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  if (cli.self_test) return cli;
  if (cli.pose != "P0" && cli.pose != "P1" && cli.pose != "CENTER")
    throw std::runtime_error("POSE_NOT_ALLOWED");
  const std::string expected_output = cli.pose == "P0" ? kOutputP0
      : cli.pose == "P1" ? kOutputP1 : kOutputCenter;
  const std::string expected_authorization = cli.pose == "P0"
      ? "J2_MANUAL_DIRECTION_P0_BRAKE_CAPTURE_AUTHORIZED=YES"
      : cli.pose == "P1"
          ? "J2_MANUAL_DIRECTION_P1_BRAKE_CAPTURE_AUTHORIZED=YES"
          : "J2_SAFE_CENTER_BRAKE_CAPTURE_AUTHORIZED=YES";
  if (cli.output != expected_output ||
      cli.authorization_gate != expected_authorization ||
      cli.safety_gate != "J2_MANUAL_DIRECTION_ARM_STATIC_SUPPORTED=YES" ||
      cli.power_gate != "J2_MANUAL_DIRECTION_24V_ON_CONFIRMED=YES")
    throw std::runtime_error("BRAKE_CAPTURE_OPERATOR_GATE_MISSING");
  if (std::filesystem::exists(cli.output))
    throw std::runtime_error("BRAKE_CAPTURE_OUTPUT_EXISTS_REFUSE_OVERWRITE");
  return cli;
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

class CaptureRunner {
 public:
  explicit CaptureRunner(const Cli& cli)
      : cli_(cli), lock_(),
        serial_(kPort, 16, 4000000, 20000, BlockYN::NO,
                bytesize_t::eightbits, parity_t::parity_none,
                stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none) {}

  ~CaptureRunner() {
    if (!terminal_brake_complete_) safe_brake();
  }

  int run() {
    try {
      std::vector<double> a_values;
      std::vector<double> b_values;
      a_values.reserve(kCapturePairs);
      b_values.reserve(kCapturePairs);
      Clock::time_point next = Clock::now();
      for (int pair = 0; pair < kCapturePairs; ++pair) {
        if (g_stop != 0) throw std::runtime_error("OPERATOR_ABORT");
        std::this_thread::sleep_until(next);
        const Feedback a = transact(kIdA);
        const Feedback b = transact(kIdB);
        if (!a.valid || !b.valid)
          throw std::runtime_error("BRAKE_CAPTURE_FEEDBACK_INVALID");
        a_values.push_back(unwrap_a_.update(a.q));
        b_values.push_back(unwrap_b_.update(b.q));
        maximum_temperature_ = std::max({maximum_temperature_, a.temp, b.temp});
        next += std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(kPeriodSeconds));
        if (next < Clock::now()) next = Clock::now();
      }
      const bool brake = terminal_brake();
      if (!brake) throw std::runtime_error("FINAL_DUAL_5_FRAME_BRAKE_FAILED");
      write_result(a_values, b_values);
      std::cout << std::setprecision(17)
                << "V15_24E_J2_MANUAL_DIRECTION_BRAKE_CAPTURE=PASS\n"
                << "POSE=" << cli_.pose << '\n'
                << "J2A_ID=0\nJ2B_ID=1\n"
                << "CAPTURE_MODE=BRAKE_ONLY\n"
                << "FOC_COMMANDS_SENT=0\n"
                << "NONZERO_Q_DQ_KP_KD_TFF_COMMANDS_SENT=0\n"
                << "VALID_PAIRS=50\n"
                << "J2A_RAW_MEDIAN_RAD=" << median(a_values) << '\n'
                << "J2B_RAW_MEDIAN_RAD=" << median(b_values) << '\n'
                << "FINAL_DUAL_5_FRAME_BRAKE=PASS\n";
      return 0;
    } catch (...) {
      const bool brake = terminal_brake();
      std::cerr << "FINAL_DUAL_5_FRAME_BRAKE="
                << (brake ? "PASS" : "FAIL") << '\n';
      throw;
    }
  }

 private:
  Feedback transact(int expected_id) {
    MotorCmd command = brake_command(expected_id);
    MotorData data;
    initialize_feedback(data);
    Feedback feedback;
    try {
      feedback.send_recv = serial_.sendRecv(&command, &data);
    } catch (...) {
      feedback.send_recv = false;
    }
    feedback.correct = data.correct;
    feedback.id = static_cast<int>(data.motor_id);
    feedback.mode = static_cast<int>(data.mode);
    feedback.temp = data.temp;
    feedback.merror = data.merror;
    feedback.q = data.q;
    feedback.dq = data.dq;
    feedback.tau = data.tau;
    const std::uint8_t* raw = data.get_motor_recv_data();
    if (raw != nullptr) {
      feedback.crc_ok = raw[0] == 0xfdU && raw[1] == 0xeeU &&
          (raw[2] & 0x80U) == 0U &&
          load_u16_le(raw + 14) == crc16_kermit(raw, 14);
    }
    feedback.valid = feedback.send_recv && feedback.correct && feedback.crc_ok &&
        feedback.id == expected_id && feedback.mode == kBrakeMode &&
        feedback.merror == 0 && feedback.temp >= 0 &&
        feedback.temp < kTemperatureLimit && std::isfinite(feedback.q) &&
        std::isfinite(feedback.dq) && std::isfinite(feedback.tau);
    return feedback;
  }

  bool terminal_brake() {
    if (terminal_brake_complete_) return final_brake_pass_;
    bool pass = true;
    for (int pair = 0; pair < kTerminalBrakePairs; ++pair) {
      const Feedback a = transact(kIdA);
      const Feedback b = transact(kIdB);
      pass = pass && a.valid && b.valid;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    final_brake_pass_ = pass;
    if (pass) terminal_brake_complete_ = true;
    else safe_brake();
    return pass;
  }

  void safe_brake() noexcept {
    try {
      for (int pair = 0; pair < 20; ++pair) {
        (void)transact(kIdA);
        (void)transact(kIdB);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    } catch (...) {}
    terminal_brake_complete_ = true;
  }

  void write_result(const std::vector<double>& a_values,
                    const std::vector<double>& b_values) const {
    const auto minmax_a = std::minmax_element(a_values.begin(), a_values.end());
    const auto minmax_b = std::minmax_element(b_values.begin(), b_values.end());
    nlohmann::json result = {
        {"schema", "V15_24E_J2_MANUAL_DIRECTION_BRAKE_CAPTURE_V1"},
        {"pose", cli_.pose},
        {"capture_mode", "BRAKE_ONLY"},
        {"valid_pairs", kCapturePairs},
        {"foc_commands_sent", 0},
        {"nonzero_q_dq_kp_kd_tff_commands_sent", 0},
        {"j2a", {{"id", kIdA}, {"raw_median_rad", median(a_values)},
                  {"raw_min_rad", *minmax_a.first},
                  {"raw_max_rad", *minmax_a.second}}},
        {"j2b", {{"id", kIdB}, {"raw_median_rad", median(b_values)},
                  {"raw_min_rad", *minmax_b.first},
                  {"raw_max_rad", *minmax_b.second}}},
        {"maximum_temperature_c", maximum_temperature_},
        {"final_dual_5_frame_brake", final_brake_pass_ ? "PASS" : "FAIL"},
        {"permanent_zero_modified", false},
    };
    const std::string temporary = cli_.output + ".tmp";
    std::filesystem::remove(temporary);
    {
      std::ofstream stream(temporary, std::ios::out | std::ios::trunc);
      if (!stream) throw std::runtime_error("BRAKE_CAPTURE_JSON_OPEN_FAILED");
      stream << std::setw(2) << result << '\n';
      if (!stream) throw std::runtime_error("BRAKE_CAPTURE_JSON_WRITE_FAILED");
    }
    std::filesystem::rename(temporary, cli_.output);
  }

  const Cli& cli_;
  ProcessLock lock_;
  SerialPort serial_;
  Unwrapper unwrap_a_;
  Unwrapper unwrap_b_;
  int maximum_temperature_ = 0;
  bool terminal_brake_complete_ = false;
  bool final_brake_pass_ = false;
};

void self_test() {
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode)
    throw std::runtime_error("BRAKE_MODE_AUTHORITY_MISMATCH");
  MotorCmd a = brake_command(kIdA);
  MotorCmd b = brake_command(kIdB);
  if (a.get_motor_send_data()[2] != 0U || b.get_motor_send_data()[2] != 1U)
    throw std::runtime_error("DUAL_BRAKE_PACKET_ID_MISMATCH");
  std::cout << "V15_24E_J2_BRAKE_POSE_CAPTURE_SELF_TEST=PASS\n"
            << "SERIAL_PORT_CONSTRUCTED=NO\n"
            << "J2A_ID=0\nJ2B_ID=1\n"
            << "CAPTURE_PAIRS=50\n"
            << "COMMAND_MODE=BRAKE_ONLY\n"
            << "FOC_COMMAND_PATH=NO\n"
            << "NONZERO_Q_DQ_KP_KD_TFF_PATH=NO\n"
            << "CALIBRATE_ID_OR_ZERO_WRITE_PATH=NO\n";
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
    CaptureRunner runner(cli);
    return runner.run();
  } catch (const std::exception& error) {
    std::cerr << "V15_24E_J2_MANUAL_DIRECTION_BRAKE_CAPTURE=FAIL\n"
              << "REASON=" << error.what() << '\n';
    return 2;
  }
}
