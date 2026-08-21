#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

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
constexpr double kKp = 0.60;
constexpr double kKd = 0.05;
constexpr double kTauFf = 0.0;
constexpr double kHz = 100.0;
constexpr double kPeriod = 0.01;
constexpr double kVmax = 10.0 * kPi / 180.0;
constexpr double kAccel = 30.0 * kPi / 180.0;
constexpr double kCommandEnvelope = 5.5 * kPi / 180.0;
constexpr double kFeedbackEnvelope = 7.0 * kPi / 180.0;
constexpr double kSyncHardAbort = 1.0 * kPi / 180.0;
constexpr int kTempLimit = 60;

std::atomic<bool> g_stop{false};
void signal_handler(int) { g_stop.store(true); }

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
    else if (arg == "--authorization-gate") cli.authorization_gate = value();
    else if (arg == "--safety-gate") cli.safety_gate = value();
    else if (arg == "--power-on-gate") cli.power_on_gate = value();
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  if (cli.self_test) return cli;
  if (cli.phase != "run1" && cli.phase != "run2")
    throw std::runtime_error("PHASE_NOT_ALLOWED");
  const std::string expected_output = cli.phase == "run1"
      ? "hardware/v15_23e_r1_ft/j2_dual_kp060_run1.csv"
      : "hardware/v15_23e_r1_ft/j2_dual_kp060_run2.csv";
  if (cli.output != expected_output)
    throw std::runtime_error("OUTPUT_PATH_NOT_ALLOWED");
  const std::string expected_auth = cli.phase == "run1"
      ? "J2_DUAL_KP060_RUN1_AUTHORIZED=YES"
      : "J2_DUAL_KP060_RUN2_EXACT_REPEAT_AUTHORIZED=YES";
  if (cli.authorization_gate != expected_auth ||
      cli.safety_gate !=
          "J2_KP060_DUAL_CONFIGURATION_UNCHANGED_AND_SAFETY_READY=YES" ||
      cli.power_on_gate != "J2_KP060_24V_POWER_ON_CONFIRMED=YES")
    throw std::runtime_error("DUAL_OPERATOR_GATE_MISSING");
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
  bool valid = false;
};

class Runner {
 public:
  explicit Runner(const Cli& cli)
      : cli_(cli), lock_(),
        serial_(kPort, 16, 4000000, 20000, BlockYN::NO,
                bytesize_t::eightbits, parity_t::parity_none,
                stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none),
        csv_(cli.output, std::ios::out | std::ios::trunc) {
    if (!csv_) throw std::runtime_error("CSV_OPEN_FAILED");
    csv_ << "tick,timestamp_s,phase,a_mode,b_mode,a_q_cmd,b_q_cmd,"
            "a_dq_cmd,b_dq_cmd,a_raw,b_raw,qA_logical_rad,qB_logical_rad,"
            "qJ2_logical_rad,q_ref_rad,e_common_rad,e_sync_rad,a_dq_feedback,b_dq_feedback,"
            "a_tau,b_tau,a_temp,b_temp,a_merror,b_merror,a_valid,b_valid,"
            "cycle_period_ms,cycle_jitter_ms\n";
    origin_ = Clock::now();
  }
  ~Runner() { if (!terminal_brake_done_) safe_brake(); }

  int run() {
    try {
      return run_dual_motion();
    } catch (...) {
      (void)terminal_brake();
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
                          const std::string& phase, Clock::time_point scheduled) {
    if (g_stop.load()) throw std::runtime_error("OPERATOR_ABORT");
    if (mode == kFocMode && !reference_set_)
      throw std::runtime_error("REFERENCE_NOT_SET");
    if (std::abs(q_target) > kCommandEnvelope + 1e-12)
      throw std::runtime_error("COMMAND_ENVELOPE");
    std::this_thread::sleep_until(scheduled);
    const Clock::time_point begin = Clock::now();

    const double a_q_cmd = mode == kFocMode
        ? ref_a_ + kSignA * kGear * q_target : 0.0;
    const double b_q_cmd = mode == kFocMode
        ? ref_b_ + kSignB * kGear * q_target : 0.0;
    const double a_dq_cmd = mode == kFocMode
        ? kSignA * kGear * dq_target : 0.0;
    const double b_dq_cmd = mode == kFocMode
        ? kSignB * kGear * dq_target : 0.0;
    MotorCmd command_a = mode == kFocMode
        ? make_command(kIdA, mode, a_q_cmd, a_dq_cmd, kKp, kKd, kTauFf)
        : brake_command(kIdA);
    MotorCmd command_b = mode == kFocMode
        ? make_command(kIdB, mode, b_q_cmd, b_dq_cmd, kKp, kKd, kTauFf)
        : brake_command(kIdB);

    PairState state;
    state.q_ref = mode == kFocMode ? q_target
                                   : std::numeric_limits<double>::quiet_NaN();
    state.a = transact(command_a, kIdA, mode);
    state.b = transact(command_b, kIdB, mode);
    if (state.a.merror > 0 || state.b.merror > 0)
      throw std::runtime_error("MERROR_NONZERO");
    update_invalid(state.a.valid, consecutive_invalid_a_, "J2A_FIVE_CONSECUTIVE_INVALID");
    update_invalid(state.b.valid, consecutive_invalid_b_, "J2B_FIVE_CONSECUTIVE_INVALID");
    if (state.a.valid && state.b.valid) {
      state.raw_a = unwrap_a_.update(state.a.q);
      state.raw_b = unwrap_b_.update(state.b.q);
      if (reference_set_) {
        state.q_a = kSignA * (state.raw_a - ref_a_) / kGear;
        state.q_b = kSignB * (state.raw_b - ref_b_) / kGear;
        state.q_j2 = 0.5 * (state.q_a + state.q_b);
        state.e_common = q_target - state.q_j2;
        state.e_sync = state.q_a - state.q_b;
        if (std::abs(state.q_a) > kFeedbackEnvelope ||
            std::abs(state.q_b) > kFeedbackEnvelope)
          throw std::runtime_error("FEEDBACK_ENVELOPE");
      }
      state.valid = true;
    }
    log_row(state, phase, a_q_cmd, b_q_cmd, a_dq_cmd, b_dq_cmd, begin);
    ++tick_;
    if (state.valid && reference_set_ &&
        std::abs(state.e_sync) > kSyncHardAbort)
      throw std::runtime_error("SYNC_HARD_ABORT_GT_1DEG");
    return state;
  }

  static void update_invalid(bool valid, int& consecutive, const char* reason) {
    if (valid) consecutive = 0;
    else if (++consecutive >= 5) throw std::runtime_error(reason);
  }

  std::pair<std::vector<double>, std::vector<double>> capture_references() {
    std::vector<double> a_values;
    std::vector<double> b_values;
    a_values.reserve(50U);
    b_values.reserve(50U);
    Clock::time_point next = Clock::now();
    for (int frame = 0; frame < 50; ++frame) {
      const PairState state = transact_pair(kBrakeMode, 0.0, 0.0,
                                            "SESSION_BRAKE_CAPTURE", next);
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
                                         bool collect_motion_sync) {
    const int count = static_cast<int>(std::ceil(duration * kHz));
    std::vector<PairState> values;
    values.reserve(static_cast<std::size_t>(count));
    Clock::time_point next = active_started_ ? next_active_ : Clock::now();
    active_started_ = true;
    for (int frame = 0; frame < count; ++frame) {
      PairState state = transact_pair(kFocMode, target, 0.0, phase, next);
      if (state.valid) {
        values.push_back(state);
        if (collect_motion_sync)
          max_motion_sync_ = std::max(max_motion_sync_, std::abs(state.e_sync));
        if (collect_motion_sync)
          max_common_error_ = std::max(max_common_error_, std::abs(state.e_common));
      }
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
      if (next < Clock::now()) next = Clock::now();
    }
    next_active_ = next;
    return values;
  }

  void run_profile(const std::string& phase, double start, double displacement) {
    Trapezoid profile(displacement);
    const int count = static_cast<int>(std::ceil(profile.duration() * kHz)) + 1;
    Clock::time_point next = active_started_ ? next_active_ : Clock::now();
    active_started_ = true;
    for (int frame = 0; frame < count; ++frame) {
      const double t = std::min(frame * kPeriod, profile.duration());
      const ProfilePoint point = profile.sample(t);
      const PairState state = transact_pair(kFocMode, start + point.q, point.dq,
                                            phase, next);
      if (state.valid)
        max_motion_sync_ = std::max(max_motion_sync_, std::abs(state.e_sync));
      if (state.valid)
        max_common_error_ = std::max(max_common_error_, std::abs(state.e_common));
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
      if (next < Clock::now()) next = Clock::now();
    }
    next_active_ = next;
  }

  bool terminal_brake() {
    bool pass = true;
    Clock::time_point next = Clock::now();
    for (int frame = 0; frame < 5; ++frame) {
      try {
        const PairState state = transact_pair(kBrakeMode, 0.0, 0.0,
                                              "FINAL_DUAL_BRAKE", next);
        pass = pass && state.a.valid && state.b.valid;
      } catch (...) {
        pass = false;
      }
      next += std::chrono::milliseconds(10);
    }
    terminal_brake_done_ = true;
    final_brake_pass_ = pass;
    csv_.flush();
    return pass;
  }

  void safe_brake() noexcept {
    try {
      for (int frame = 0; frame < 5; ++frame) {
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
         << state.a.mode << ',' << state.b.mode << ','
         << a_q_cmd << ',' << b_q_cmd << ',' << a_dq_cmd << ',' << b_dq_cmd << ','
         << state.raw_a << ',' << state.raw_b << ',' << state.q_a << ',' << state.q_b
         << ',' << state.q_j2 << ',' << state.q_ref << ',' << state.e_common
         << ',' << state.e_sync << ','
         << state.a.dq << ',' << state.b.dq << ',' << state.a.tau << ',' << state.b.tau
         << ',' << state.a.temp << ',' << state.b.temp << ','
         << state.a.merror << ',' << state.b.merror << ','
         << static_cast<int>(state.a.valid) << ',' << static_cast<int>(state.b.valid)
         << ',' << period_ms << ',' << jitter_ms << '\n';
    csv_.flush();
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
    const auto hold = run_target_hold("DUAL_CURRENT_HOLD", 0.0, 1.5, false);
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

  int run_dual_motion() {
    const auto capture = capture_references();
    establish_references(capture.first, capture.second);
    const auto prehold = run_target_hold("DUAL_CURRENT_HOLD", 0.0, 1.5, false);
    const double prehold_sync = std::abs(median(tail_field(prehold, true, 50U)));
    double prehold_max_sync = 0.0;
    double prehold_max_motion = 0.0;
    for (const PairState& state : prehold) {
      prehold_max_sync = std::max(prehold_max_sync, std::abs(state.e_sync));
      prehold_max_motion = std::max(prehold_max_motion, std::abs(state.q_j2));
    }
    if (prehold_sync > 0.3 * kPi / 180.0 ||
        prehold_max_motion > 1.0 * kPi / 180.0) {
      const bool brake = terminal_brake();
      std::cout << std::setprecision(17)
                << "RUN=" << cli_.phase << '\n'
                << "DUAL_SESSION_A0_RAW_RAD=" << ref_a_ << '\n'
                << "DUAL_SESSION_B0_RAW_RAD=" << ref_b_ << '\n'
                << "DUAL_HOLD_FINAL_ESYNC_DEG=" << prehold_sync * 180.0 / kPi << '\n'
                << "DUAL_HOLD_MAX_ESYNC_DEG=" << prehold_max_sync * 180.0 / kPi << '\n'
                << "DUAL_HOLD_NUMERIC_RESULT=FAIL\n"
                << "FINAL_DUAL_5_FRAME_BRAKE=" << (brake ? "PASS" : "FAIL") << '\n';
      return 2;
    }

    max_motion_sync_ = 0.0;
    max_common_error_ = 0.0;
    run_profile("PLUS_5_PROFILE", 0.0, 5.0 * kPi / 180.0);
    const auto plus = run_target_hold("PLUS_5_ENDPOINT", 5.0 * kPi / 180.0,
                                      0.4, true);
    const double plus_actual = median(tail_field(plus, false, 30U));
    const double plus_sync = std::abs(median(tail_field(plus, true, 30U)));

    run_profile("FIRST_CENTER_PROFILE", 5.0 * kPi / 180.0,
                -5.0 * kPi / 180.0);
    const auto first = run_target_hold("FIRST_CENTER_ENDPOINT", 0.0, 0.4, true);
    const double first_error = std::abs(median(tail_field(first, false, 30U)));

    run_profile("MINUS_5_PROFILE", 0.0, -5.0 * kPi / 180.0);
    const auto minus = run_target_hold("MINUS_5_ENDPOINT", -5.0 * kPi / 180.0,
                                       0.4, true);
    const double minus_actual = median(tail_field(minus, false, 30U));
    const double minus_sync = std::abs(median(tail_field(minus, true, 30U)));

    run_profile("FINAL_CENTER_PROFILE", -5.0 * kPi / 180.0,
                5.0 * kPi / 180.0);
    const auto final = run_target_hold("FINAL_CENTER_ENDPOINT", 0.0, 0.5, true);
    const double final_error = std::abs(median(tail_field(final, false, 40U)));

    const double plus_error = std::abs(plus_actual - 5.0 * kPi / 180.0);
    const double minus_error = std::abs(minus_actual + 5.0 * kPi / 180.0);
    const bool pass = plus_error <= 1.0 * kPi / 180.0 &&
        first_error <= 0.75 * kPi / 180.0 &&
        minus_error <= 1.0 * kPi / 180.0 &&
        final_error <= 0.75 * kPi / 180.0 &&
        plus_sync <= 0.3 * kPi / 180.0 &&
        minus_sync <= 0.3 * kPi / 180.0 &&
        max_motion_sync_ <= 0.7 * kPi / 180.0;
    const bool brake = terminal_brake();
    std::cout << std::setprecision(17)
              << "RUN=" << cli_.phase << '\n'
              << "ONLY_KP_CHANGED=YES\nOLD_KP=0.50\nNEW_KP=0.60\n"
              << "DUAL_SESSION_A0_RAW_RAD=" << ref_a_ << '\n'
              << "DUAL_SESSION_B0_RAW_RAD=" << ref_b_ << '\n'
              << "DUAL_HOLD_NUMERIC_RESULT=PASS\n"
              << "DUAL_HOLD_FINAL_ESYNC_DEG=" << prehold_sync * 180.0 / kPi << '\n'
              << "DUAL_HOLD_MAX_ESYNC_DEG=" << prehold_max_sync * 180.0 / kPi << '\n'
              << "DUAL_ROUTE=0_TO_PLUS5_TO_0_TO_MINUS5_TO_0\n"
              << "PLUS_5_LOGICAL_ACTUAL_DEG=" << plus_actual * 180.0 / kPi << '\n'
              << "PLUS_5_LOGICAL_ERROR_DEG=" << plus_error * 180.0 / kPi << '\n'
              << "PLUS_5_ESYNC_DEG=" << plus_sync * 180.0 / kPi << '\n'
              << "FIRST_CENTER_ERROR_DEG=" << first_error * 180.0 / kPi << '\n'
              << "MINUS_5_LOGICAL_ACTUAL_DEG=" << minus_actual * 180.0 / kPi << '\n'
              << "MINUS_5_LOGICAL_ERROR_DEG=" << minus_error * 180.0 / kPi << '\n'
              << "MINUS_5_ESYNC_DEG=" << minus_sync * 180.0 / kPi << '\n'
              << "FINAL_CENTER_ERROR_DEG=" << final_error * 180.0 / kPi << '\n'
              << "MAX_MOTION_ESYNC_DEG=" << max_motion_sync_ * 180.0 / kPi << '\n'
              << "MAX_COMMON_MODE_ERROR_DEG=" << max_common_error_ * 180.0 / kPi << '\n'
              << "DUAL_MOTION_NUMERIC_RESULT=" << (pass ? "PASS" : "FAIL") << '\n'
              << "FINAL_DUAL_5_FRAME_BRAKE=" << (brake ? "PASS" : "FAIL") << '\n';
    return pass && brake ? 0 : 2;
  }

  const Cli& cli_;
  ProcessLock lock_;
  SerialPort serial_;
  std::ofstream csv_;
  Clock::time_point origin_;
  Unwrapper unwrap_a_;
  Unwrapper unwrap_b_;
  double ref_a_ = 0.0;
  double ref_b_ = 0.0;
  bool reference_set_ = false;
  int consecutive_invalid_a_ = 0;
  int consecutive_invalid_b_ = 0;
  bool terminal_brake_done_ = false;
  bool final_brake_pass_ = false;
  bool have_previous_ = false;
  double previous_s_ = 0.0;
  std::uint64_t tick_ = 0;
  bool active_started_ = false;
  Clock::time_point next_active_{};
  double max_motion_sync_ = 0.0;
  double max_common_error_ = 0.0;
};

void self_test() {
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode ||
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC) != kFocMode ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) - kGear) > 1e-6)
    throw std::runtime_error("SDK_AUTHORITY_MISMATCH");
  MotorCmd brake_a = brake_command(kIdA);
  MotorCmd brake_b = brake_command(kIdB);
  MotorCmd foc_a = make_command(kIdA, kFocMode, 1.0, -0.2,
                                kKp, kKd, kTauFf);
  MotorCmd foc_b = make_command(kIdB, kFocMode, 2.0, 0.2,
                                kKp, kKd, kTauFf);
  const std::uint8_t* pa = foc_a.get_motor_send_data();
  const std::uint8_t* pb = foc_b.get_motor_send_data();
  if (pa == nullptr || pb == nullptr ||
      pa[2] != static_cast<std::uint8_t>(kIdA | (kFocMode << 4)) ||
      pb[2] != static_cast<std::uint8_t>(kIdB | (kFocMode << 4)) ||
      load_u16_le(pa + 11) != 768U || load_u16_le(pb + 11) != 768U ||
      load_u16_le(pa + 13) != 64U || load_u16_le(pb + 13) != 64U)
    throw std::runtime_error("DUAL_PACKET_SELF_TEST_FAILED");
  (void)brake_a;
  (void)brake_b;
  Trapezoid plus(5.0 * kPi / 180.0);
  Trapezoid minus(-5.0 * kPi / 180.0);
  if (std::abs(plus.sample(plus.duration()).q - 5.0 * kPi / 180.0) > 1e-12 ||
      std::abs(minus.sample(minus.duration()).q + 5.0 * kPi / 180.0) > 1e-12)
    throw std::runtime_error("PROFILE_SELF_TEST_FAILED");
  const double q = 5.0 * kPi / 180.0;
  if (!(kSignA * kGear * q < 0.0 && kSignB * kGear * q > 0.0))
    throw std::runtime_error("DUAL_SIGN_SELF_TEST_FAILED");
  std::cout << "V15_23E_R1_FT_J2_DUAL_KP060_SELF_TEST=PASS\n"
            << "SERIAL_PORT_CONSTRUCTED=NO\n"
            << "J2A_ID=0\nJ2B_ID=1\n"
            << "SIGN_A=-1\nSIGN_B=+1\nSIGN_RELATION=OPPOSITE\n"
            << "SEPARATE_RAW_CURRENT_TARGETS=YES\n"
            << "ONLY_KP_CHANGED=YES\nOLD_KP=0.50\n"
            << "DUAL_KP=0.60\nDUAL_KD=0.05\nDUAL_TFF=0\n"
            << "HOLD_DURATION_S=1.5\n"
            << "MOTION_ROUTE=0_TO_PLUS5_TO_0_TO_MINUS5_TO_0\n"
            << "SYNC_HOLD_ACCEPT_DEG=0.3\nSYNC_MOTION_ACCEPT_DEG=0.7\n"
            << "SYNC_HARD_ABORT_DEG=1.0\n"
            << "CALIBRATE_PATH=NO\nID_WRITE_PATH=NO\nZERO_WRITE_PATH=NO\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  try {
    const Cli cli = parse_cli(argc, argv);
    if (cli.self_test) {
      self_test();
      return 0;
    }
    Runner runner(cli);
    return runner.run();
  } catch (const std::exception& error) {
    std::cerr << "V15_23E_R1_FT_J2_DUAL_KP060_RESULT=FAIL\nREASON="
              << error.what() << '\n';
    return 2;
  }
}
