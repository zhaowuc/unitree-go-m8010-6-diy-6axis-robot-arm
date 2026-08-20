#include <algorithm>
#include <array>
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
#include <numeric>
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
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0";
constexpr char kLockPath[] = "/tmp/v15_22a_j5_bus.lock";
constexpr char kClearanceGate[] = "J5_PHYSICAL_CLEARANCE_PLUS_MINUS_10DEG=YES";
constexpr char kBrakeGate[] = "J5_EMERGENCY_BRAKE_READY=YES";
constexpr int kJ3 = 3;
constexpr int kJ4 = 4;
constexpr int kJ5 = 5;
constexpr int kBrakeMode = 0;
constexpr int kFocMode = 1;
constexpr double kPi = 3.14159265358979323846;
constexpr double kGear = 6.3299999237060547;
constexpr double kKp = 0.50;
constexpr double kKd = 0.05;
constexpr double kTauFf = 0.0;
constexpr double kHz = 100.0;
constexpr double kPeriod = 0.01;
constexpr double kVmax = 10.0 * kPi / 180.0;
constexpr double kAccel = 30.0 * kPi / 180.0;
constexpr double kCommandEnvelope = 6.0 * kPi / 180.0;
constexpr double kFeedbackEnvelope = 8.0 * kPi / 180.0;
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
  std::string clearance;
  std::string brake_gate;
  int ros_sign = 0;
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
    else if (arg == "--clearance") cli.clearance = value();
    else if (arg == "--brake-ready") cli.brake_gate = value();
    else if (arg == "--raw-to-ros-sign") cli.ros_sign = std::stoi(value());
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  if (cli.self_test) return cli;
  if (cli.phase != "hold" && cli.phase != "sign" &&
      cli.phase != "session2" && cli.phase != "bidirectional") {
    throw std::runtime_error("PHASE_NOT_ALLOWED");
  }
  const std::string prefix = "hardware/v15_22a/";
  if (cli.output.compare(0, prefix.size(), prefix) != 0 ||
      cli.output.size() <= prefix.size() + 4U ||
      cli.output.substr(cli.output.size() - 4U) != ".csv" ||
      cli.output.find("..") != std::string::npos) {
    throw std::runtime_error("OUTPUT_PATH_NOT_ALLOWED");
  }
  if (cli.phase != "session2" &&
      (cli.clearance != kClearanceGate || cli.brake_gate != kBrakeGate)) {
    throw std::runtime_error("ACTIVE_MOTION_OPERATOR_GATE_MISSING");
  }
  if (cli.phase == "bidirectional" && cli.ros_sign != 1 && cli.ros_sign != -1) {
    throw std::runtime_error("BIDIRECTIONAL_REQUIRES_FROZEN_SIGN");
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
      : cli_(cli), lock_(), serial_(kPort, 16, 4000000, 20000, BlockYN::NO,
          bytesize_t::eightbits, parity_t::parity_none,
          stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none),
        csv_(cli.output, std::ios::out | std::ios::trunc) {
    if (!csv_) throw std::runtime_error("CSV_OPEN_FAILED");
    csv_ << "tick,timestamp_s,phase,j5_mode,q_cmd_motor_rad,dq_cmd_motor_rad_s,"
            "q_feedback_motor_rad,q_relative_motor_rad,q_relative_output_protocol_rad,"
            "q_relative_ros_rad,qdot_fast_deg_s,qdot_slow_deg_s,dq_sdk,tau_feedback,"
            "temperature_c,merror,data_correct,j3_mode,j3_merror,j3_correct,"
            "j4_mode,j4_merror,j4_correct,tx_period_ms,tx_jitter_ms\n";
    origin_ = Clock::now();
  }
  ~Runner() { if (!terminal_brake_done_) safe_brake(); }

  int run() {
    if (cli_.phase == "hold") return run_hold();
    if (cli_.phase == "sign") return run_sign();
    if (cli_.phase == "session2") return run_session2();
    return run_bidirectional();
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
    int& consecutive = id == kJ3 ? j3_invalid_ : j4_invalid_;
    if (!f.valid) {
      ++consecutive;
      if (f.merror > 0) throw std::runtime_error("AUX_MERROR_NONZERO");
      if (f.send_recv && f.correct && f.mode != kBrakeMode)
        throw std::runtime_error("AUX_LEFT_BRAKE");
      if (f.temp >= kTempLimit) throw std::runtime_error("AUX_TEMPERATURE_LIMIT");
      if (consecutive >= 5) throw std::runtime_error("AUX_CONSECUTIVE_INVALID");
    } else consecutive = 0;
  }

  void poll_aux() {
    j3_ = poll_brake(kJ3); check_aux(j3_, kJ3);
    j4_ = poll_brake(kJ4); check_aux(j4_, kJ4);
  }

  std::vector<double> capture_brake(int count, const std::string& phase) {
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(count));
    Clock::time_point next = Clock::now();
    for (int i = 0; i < count; ++i) {
      if (g_stop.load()) throw std::runtime_error("OPERATOR_ABORT");
      std::this_thread::sleep_until(next);
      const Clock::time_point begin = Clock::now();
      Feedback f = poll_brake(kJ5);
      if (!f.valid) {
        ++j5_invalid_;
        ++metrics_.invalid_feedback;
        if (j5_invalid_ >= 5) throw std::runtime_error("J5_CONSECUTIVE_INVALID");
      } else {
        j5_invalid_ = 0;
        values.push_back(j5_unwrap_.update(f.q));
      }
      if (i % 5 == 0) poll_aux();
      log_row(phase, f, 0.0, 0.0, begin, false);
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
      if (next < Clock::now()) next = Clock::now();
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
    if (!reference_set_) throw std::runtime_error("REFERENCE_NOT_SET");
    if (std::abs(q_output) > kCommandEnvelope + 1e-12)
      throw std::runtime_error("COMMAND_ENVELOPE");
    std::this_thread::sleep_until(scheduled);
    const Clock::time_point begin = Clock::now();
    const double q_cmd = q_reference_ + kGear * q_output;
    const double dq_cmd = kGear * dq_output;
    MotorCmd command = make_command(kJ5, kFocMode, q_cmd, dq_cmd, kKp, kKd, kTauFf);
    Feedback f = transact(command, kJ5, kFocMode);
    metrics_.active_tx_times.push_back(
        std::chrono::duration<double>(begin - origin_).count());
    if (!f.valid) {
      ++j5_invalid_;
      ++metrics_.invalid_feedback;
      if (f.merror > 0) ++metrics_.merror_nonzero;
      if (j5_invalid_ >= 5) throw std::runtime_error("J5_CONSECUTIVE_INVALID");
    } else {
      j5_invalid_ = 0;
      const double q_unwrapped = j5_unwrap_.update(f.q);
      const double q_rel_output = (q_unwrapped - q_reference_) / kGear;
      if (std::abs(q_rel_output) > kFeedbackEnvelope)
        throw std::runtime_error("FEEDBACK_ENVELOPE");
      const double t = std::chrono::duration<double>(begin - origin_).count();
      velocity_.update(t, q_rel_output);
      const double fast = std::abs(velocity_.fast_deg_s());
      const double slow = std::abs(velocity_.slow_deg_s());
      metrics_.q_output_deg.push_back(q_rel_output * 180.0 / kPi);
      metrics_.fast_abs.push_back(fast);
      metrics_.slow_abs.push_back(slow);
      metrics_.tau.push_back(f.tau);
      metrics_.temp_min = std::min(metrics_.temp_min, f.temp);
      metrics_.temp_max = std::max(metrics_.temp_max, f.temp);
      if (fast > 40.0 || slow > 40.0) throw std::runtime_error("SPEED_EMERGENCY_40");
      if (slow > 30.0) {
        ++metrics_.sustained_over30;
        if (metrics_.sustained_over30 >= 3) throw std::runtime_error("SPEED_BRAKE_30_SUSTAINED");
      } else metrics_.sustained_over30 = 0;
      if (fast > 20.0 || slow > 20.0) ++metrics_.warning_speed_frames;
    }
    if (tick_ % 5U == 0U) poll_aux();
    log_row(phase, f, q_cmd, dq_cmd, begin, true);
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
      if (f.valid) values.push_back((j5_unwrap_.update(f.q) - q_reference_) / kGear);
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
    Clock::time_point next = active_schedule_started_ ? next_active_ : Clock::now();
    active_schedule_started_ = true;
    for (int i = 0; i < count; ++i) {
      const double t = std::min(i * kPeriod, profile.duration());
      const ProfilePoint point = profile.sample(t);
      (void)active_tick(phase, start + point.q, point.dq, next);
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
      if (next < Clock::now()) next = Clock::now();
    }
    next_active_ = next;
  }

  bool terminal_brake() {
    bool pass = true;
    for (int i = 0; i < 5; ++i) {
      Feedback f5 = poll_brake(kJ5);
      Feedback f3 = poll_brake(kJ3);
      Feedback f4 = poll_brake(kJ4);
      pass = pass && f5.valid && f3.valid && f4.valid;
      j3_ = f3; j4_ = f4;
      log_row("FINAL_BRAKE", f5, 0.0, 0.0, Clock::now(), false);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    terminal_brake_done_ = true;
    final_brake_pass_ = pass;
    csv_.flush();
    return pass;
  }

  void safe_brake() noexcept {
    try { (void)terminal_brake(); } catch (...) {}
  }

  void log_row(const std::string& phase, const Feedback& f, double q_cmd,
               double dq_cmd, Clock::time_point begin, bool active) {
    double q_unwrapped = std::numeric_limits<double>::quiet_NaN();
    double q_rel_motor = std::numeric_limits<double>::quiet_NaN();
    double q_rel_output = std::numeric_limits<double>::quiet_NaN();
    if (f.valid) {
      q_unwrapped = j5_unwrap_.update(f.q);
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
    csv_ << tick_ << ',' << std::setprecision(17) << now_s << ',' << phase << ','
         << f.mode << ',' << q_cmd << ',' << dq_cmd << ',' << q_unwrapped << ','
         << q_rel_motor << ',' << q_rel_output << ',' << ros << ','
         << velocity_.fast_deg_s() << ',' << velocity_.slow_deg_s() << ','
         << f.dq << ',' << f.tau << ',' << f.temp << ',' << f.merror << ','
         << static_cast<int>(f.correct) << ',' << j3_.mode << ',' << j3_.merror
         << ',' << static_cast<int>(j3_.correct) << ',' << j4_.mode << ','
         << j4_.merror << ',' << static_cast<int>(j4_.correct) << ','
         << period_ms << ',' << jitter_ms << '\n';
    csv_.flush();
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
              << "TAU_MIN=" << minmax_tau.first << '\n'
              << "TAU_MAX=" << minmax_tau.second << '\n'
              << "TEMP_MIN_C=" << metrics_.temp_min << '\n'
              << "TEMP_MAX_C=" << metrics_.temp_max << '\n'
              << "MERROR_NONZERO_FRAMES=" << metrics_.merror_nonzero << '\n'
              << "INVALID_FEEDBACK_FRAMES=" << metrics_.invalid_feedback << '\n'
              << "FINAL_J5_BRAKE_SENT=YES\n"
              << "FINAL_5_FRAME_BRAKE=" << (final_brake_pass_ ? "PASS" : "FAIL") << '\n'
              << "J3_J4_LEFT_BRAKE=" << (final_brake_pass_ ? "YES" : "NO") << '\n';
  }

  int run_hold() {
    const auto capture = capture_brake(50, "HOLD_START_CAPTURE");
    establish_reference(capture);
    const auto hold = run_hold_segment("HOLD_CURRENT", 0.0, 1.0);
    if (hold.size() < 90U) throw std::runtime_error("HOLD_FEEDBACK_INSUFFICIENT");
    double max_drift = 0.0;
    for (double q : hold) max_drift = std::max(max_drift, std::abs(q * 180.0 / kPi));
    std::vector<double> final_values(hold.end() - 30, hold.end());
    const double final_error = std::abs(median(final_values) * 180.0 / kPi);
    const bool numeric_pass = max_drift <= 1.5 && final_error <= 1.0;
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
    const bool pass = actual > 0.0 && endpoint_error <= 1.5 && return_error <= 1.0;
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
    std::cout << "FINAL_J5_BRAKE_SENT=YES\nFINAL_5_FRAME_BRAKE="
              << (brake ? "PASS" : "FAIL") << "\nJ3_J4_LEFT_BRAKE="
              << (brake ? "YES" : "NO") << '\n';
    return brake ? 0 : 2;
  }

  int run_bidirectional() {
    const auto capture = capture_brake(50, "BIDIRECTIONAL_START_CAPTURE");
    establish_reference(capture);
    const auto hold = run_hold_segment("SECOND_HOLD", 0.0, 1.0);
    double hold_max = 0.0;
    for (double q : hold) hold_max = std::max(hold_max, std::abs(q * 180.0 / kPi));
    std::vector<double> hold_final(hold.end() - 30, hold.end());
    const double hold_error = std::abs(median(hold_final) * 180.0 / kPi);
    const bool second_hold = hold_max <= 1.5 && hold_error <= 1.0;
    if (!second_hold) throw std::runtime_error("SECOND_HOLD_ACCEPTANCE_FAILED");

    run_profile("PLUS_5_PROFILE", 0.0, 5.0 * kPi / 180.0);
    const auto plus = run_hold_segment("PLUS_5_HOLD", 5.0 * kPi / 180.0, 0.4);
    const double plus_delta = median(plus) * 180.0 / kPi;
    const double plus_error = std::abs(plus_delta - 5.0);
    run_profile("FIRST_CENTER_PROFILE", 5.0 * kPi / 180.0, -5.0 * kPi / 180.0);
    const auto center = run_hold_segment("FIRST_CENTER_HOLD", 0.0, 0.4);
    const double center_error = std::abs(median(center) * 180.0 / kPi);
    run_profile("MINUS_5_PROFILE", 0.0, -5.0 * kPi / 180.0);
    const auto minus = run_hold_segment("MINUS_5_HOLD", -5.0 * kPi / 180.0, 0.4);
    const double minus_delta = median(minus) * 180.0 / kPi;
    const double minus_error = std::abs(minus_delta + 5.0);
    run_profile("FINAL_CENTER_PROFILE", -5.0 * kPi / 180.0, 5.0 * kPi / 180.0);
    const auto final = run_hold_segment("FINAL_CENTER_HOLD", 0.0, 0.5);
    const double final_error = std::abs(median(final) * 180.0 / kPi);
    const bool pass = plus_error <= 1.5 && center_error <= 1.0 &&
                      minus_error <= 1.5 && final_error <= 1.0;
    const bool brake = terminal_brake();
    std::cout << std::setprecision(17)
              << "Q_BIDIRECTIONAL_SESSION_RAW_RAD=" << q_reference_ << '\n'
              << "SECOND_HOLD_RESULT=PASS\n"
              << "SECOND_HOLD_MAX_DRIFT_DEG=" << hold_max << '\n'
              << "SECOND_HOLD_FINAL_ERROR_DEG=" << hold_error << '\n'
              << "PLUS_5_DELTA_DEG=" << plus_delta << '\n'
              << "PLUS_5_ERROR_DEG=" << plus_error << '\n'
              << "CENTER_RETURN_ERROR_DEG=" << center_error << '\n'
              << "MINUS_5_DELTA_DEG=" << minus_delta << '\n'
              << "MINUS_5_ERROR_DEG=" << minus_error << '\n'
              << "FINAL_RETURN_ERROR_DEG=" << final_error << '\n'
              << "BIDIRECTIONAL_RESULT=" << (pass ? "PASS" : "FAIL") << '\n';
    print_common();
    return pass && brake ? 0 : 2;
  }

  const Cli& cli_;
  ProcessLock lock_;
  SerialPort serial_;
  std::ofstream csv_;
  Clock::time_point origin_;
  Unwrapper j5_unwrap_;
  VelocityEstimator velocity_;
  Feedback j3_, j4_;
  Metrics metrics_;
  double q_reference_ = 0.0;
  bool reference_set_ = false;
  bool terminal_brake_done_ = false;
  bool final_brake_pass_ = false;
  int j3_invalid_ = 0, j4_invalid_ = 0, j5_invalid_ = 0;
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
  MotorCmd brake = brake_command(kJ5);
  MotorCmd foc = make_command(kJ5, kFocMode, 1.0, 0.2, kKp, kKd, kTauFf);
  (void)brake; (void)foc;
  Trapezoid three(3.0 * kPi / 180.0);
  Trapezoid five(-5.0 * kPi / 180.0);
  const auto p3 = three.sample(three.duration());
  const auto p5 = five.sample(five.duration());
  if (std::abs(p3.q - 3.0 * kPi / 180.0) > 1e-12 || p3.dq != 0.0 ||
      std::abs(p5.q + 5.0 * kPi / 180.0) > 1e-12 || p5.dq != 0.0) {
    throw std::runtime_error("TRAPEZOID_SELF_TEST_FAILED");
  }
  std::cout << "V15_22A_J5_LOCAL_MOTION_SELF_TEST=PASS\n"
            << "SERIAL_PORT_CONSTRUCTED=NO\n"
            << "J5_ONLY_FOC_PATH=YES\nJ3_J4_FOC_PATH=NO\n"
            << "CALIBRATE_PATH=NO\nID_WRITE_PATH=NO\nZERO_WRITE_PATH=NO\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  try {
    const Cli cli = parse_cli(argc, argv);
    if (cli.self_test) { self_test(); return 0; }
    Runner runner(cli);
    return runner.run();
  } catch (const std::exception& exc) {
    std::cerr << "V15_22A_RESULT=FAIL\nREASON=" << exc.what() << '\n';
    return 2;
  }
}
